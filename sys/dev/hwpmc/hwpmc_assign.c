/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Advanced Micro Devices, Inc.
 *
 * Most-constrained-first greedy assigner for hwpmc grouping/multiplex.
 *
 * Algorithm:
 *   1. Each event has a pmc_sched_constraint published by its class
 *      back-end through pcd_get_sched_constraint.  pc_allowed_rows is
 *      a bitmask in the backend's row-index namespace; pc_weight is
 *      popcount(pc_allowed_rows).
 *   2. pmu_assign_group walks events in increasing pc_weight order.
 *      Lower weight == more constrained == placed first.  Ties are
 *      broken by insertion order (TAILQ position) for determinism.
 *   3. For each event we scan its allowed rows, pick the first that
 *      is not already used by this group and that the existing
 *      pcd_allocate_pmc accepts.
 *   4. Failure to place ANY event triggers pmu_unassign_group, which
 *      releases everything that was placed earlier (all-or-none).
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/pmc.h>
#include <sys/proc.h>

#include "hwpmc_pmu.h"

/*
 * Look up the class descriptor for a PMC class.  Returns NULL if the
 * class is not present on this machine.  Used by the constraint /
 * can-assign wrappers below so the assigner never needs to name a
 * specific back-end.
 */
static struct pmc_classdep *
pmu_class_to_classdep(enum pmc_class class)
{
	struct pmc_mdep *mdep;
	struct pmc_classdep *pcd;
	u_int i;

	mdep = hwpmc_get_mdep();
	if (mdep == NULL)
		return (NULL);
	for (i = 0; i < mdep->pmd_nclass; i++) {
		pcd = &mdep->pmd_classdep[i];
		if (pcd->pcd_class == class)
			return (pcd);
	}
	return (NULL);
}

/*
 * Per-row feasibility check via the class back-end.  A class that does
 * not publish pcd_can_assign_pmc imposes no extra per-row constraint,
 * so an absent (NULL) provider means "allowed" (0); the group is still
 * gated by pcd_get_sched_constraint's allowed-row mask.
 */
static int
pmu_can_assign_pmc(struct pmc_classdep *pcd, int adjri, struct pmc *pm,
    const struct pmc_op_pmcallocate *a)
{
	if (pcd == NULL || pcd->pcd_can_assign_pmc == NULL)
		return (0);
	return (pcd->pcd_can_assign_pmc(adjri, pm, a));
}

/*
 * Fetch an event's scheduling constraint from its class back-end.  The
 * provider is optional: a class that does not implement grouping leaves
 * pcd_get_sched_constraint NULL, and we report the feature unsupported
 * (EOPNOTSUPP) so pmu_validate_group rejects the group cleanly.  This
 * keeps the assigner platform-agnostic -- no per-class special cases.
 */
int
pmu_event_get_constraint(pmu_event_t *pe,
    pmc_sched_constraint_t *cons)
{
	const struct pmc_op_pmcallocate *a;
	struct pmc_classdep *pcd;

	if (pe == NULL || cons == NULL)
		return (EINVAL);
	a = &pe->pe_alloc;
	pcd = pmu_class_to_classdep(a->pm_class);
	if (pcd == NULL || pcd->pcd_get_sched_constraint == NULL)
		return (EOPNOTSUPP);
	return (pcd->pcd_get_sched_constraint(pe->pe_pmc, a, cons));
}

/*
 * True if the given PMC class participates in event grouping, i.e. its
 * back-end publishes a scheduling-constraint provider.  Lets callers
 * (e.g. the deferred-group allocation path) reject unsupported classes
 * early without naming any specific architecture.
 */
bool
pmu_class_supports_grouping(enum pmc_class class)
{
	struct pmc_classdep *pcd;

	pcd = pmu_class_to_classdep(class);
	return (pcd != NULL && pcd->pcd_get_sched_constraint != NULL);
}

/*
 * Try to bind one event to a row.  used_mask tracks rows already
 * consumed by earlier events in this group.  evictable_rows is a
 * global-ri mask of rows whose current occupant rotation could evict;
 * those rows are treated as allocatable even when the framework's
 * occupancy checks reject them.  A NULL p probes against an empty
 * machine: the framework occupancy checks are skipped entirely.
 */
static int
pmu_assign_one(pmu_event_t *pe, struct proc *p, int cpu,
    uint32_t *used_mask, bool dry_run, uint64_t evictable_rows)
{
	struct pmc *pm;
	struct pmc_classdep *pcd;
	enum pmc_mode mode;
	uint32_t allowed, free_mask;
	int adjri, idcpu, n;
	bool sys;

	/*
	 * pe_cons.pc_allowed_rows / *used_mask / pc_fixed_row all live in
	 * the per-class adjri namespace (e.g., bits 0..5 for AMD CORE).
	 * Global ri = pcd->pcd_ri + adjri.  We iterate adjri space and
	 * convert to global ri only where the framework requires it.
	 */
	pcd = pmu_class_to_classdep(pe->pe_alloc.pm_class);
	if (pcd == NULL)
		return (EBUSY);
	pm = pe->pe_pmc;
	mode = pe->pe_alloc.pm_mode;
	allowed = pe->pe_cons.pc_allowed_rows;
	/*
	 * Virtual rows are CPU-agnostic and encode PMC_CPU_ANY; system
	 * rows are bound to the caller-supplied CPU and must encode it so
	 * PMC_TO_CPU() resolves correctly on the start/stop/read paths.
	 */
	sys = PMC_IS_SYSTEM_MODE(mode);
	idcpu = sys ? cpu : PMC_CPU_ANY;

	if ((pe->pe_cons.pc_flags & PMC_SC_F_FIXED) != 0) {
		adjri = pe->pe_cons.pc_fixed_row;
		if (adjri < 0 || adjri >= 32)
			return (EBUSY);
		if (((1u << adjri) & allowed) == 0 ||
		    (*used_mask & (1u << adjri)) != 0)
			return (EBUSY);
	} else {
		adjri = -1;
	}

	for (;;) {
		if (adjri < 0) {
			free_mask = allowed & ~*used_mask;
			if (free_mask == 0)
				return (EBUSY);
			adjri = ffs(free_mask) - 1;
		}
		if (adjri >= pcd->pcd_num)
			goto skip;
		n = pcd->pcd_ri + adjri;
		if (p != NULL && (!hwpmc_can_allocate_row(n, mode) ||
		    !hwpmc_can_allocate_rowindex(p, n, idcpu)) &&
		    (n >= 64 || (evictable_rows & (1ULL << n)) == 0))
			goto skip;
		/* System rows publish occupancy via phw_pmc (spec §5.4). */
		if (sys && p != NULL && !hwpmc_row_is_unallocated(cpu, n) &&
		    (n >= 64 || (evictable_rows & (1ULL << n)) == 0))
			goto skip;
		if (pmu_can_assign_pmc(pcd, adjri, pm, &pe->pe_alloc) != 0)
			goto skip;
		if (!dry_run &&
		    pcd->pcd_allocate_pmc(cpu, adjri, pm, &pe->pe_alloc) != 0)
			goto skip;

		*used_mask |= 1u << adjri;
		if (dry_run)
			return (0);
		pm->pm_id = PMC_ID_MAKE_ID(idcpu, mode,
		    pe->pe_alloc.pm_class, n);
		/*
		 * Mark the row's disposition to match its world: system
		 * rows are STANDALONE (so a process-mode PMC can never grab
		 * the same row on this CPU), virtual rows are THREAD.
		 */
		if (sys)
			hwpmc_mark_row_standalone(n);
		else
			hwpmc_mark_row_thread(n);
		return (0);

skip:
		if ((pe->pe_cons.pc_flags & PMC_SC_F_FIXED) != 0)
			return (EBUSY);
		allowed &= ~(1u << adjri);
		adjri = -1;
	}
}

/*
 * Atomically release every HW row currently held by pg.  All-or-none
 * scheduling means there is no "partial" path: either pg is fully
 * scheduled in (every sibling on a row) or fully out, so this single
 * function suffices.
 */
void
pmu_unassign_group(pmu_group_t *pg, int cpu)
{
	pmu_event_t *pe;
	struct pmc *pm;
	struct pmc_classdep *pcd;
	enum pmc_mode mode;
	int adjri, n;
	bool sys;

	sys = (pg != NULL && pg->pg_system);

	/*
	 * AMD's pcd_release_pmc is logically a no-op but its KASSERT
	 * still requires cpu >= 0.  PMC_CPU_ANY (-1) is a perfectly
	 * legitimate "any CPU" value for virtual-mode PMCs, so coerce
	 * it to CPU 0 here rather than fan it out across the per-class
	 * back-ends.  System groups always pass their real bound CPU.
	 */
	if (cpu < 0)
		cpu = 0;

	TAILQ_FOREACH(pe, &pg->pg_events, pe_sibling) {
		pm = pe->pe_pmc;
		if (PMC_ROW_IS_UNASSIGNED(pm))
			continue;
		n = PMC_TO_ROWINDEX(pm);
		mode = PMC_TO_MODE(pm);
		pcd = hwpmc_ri_to_classdep(n, &adjri);
		if (pcd != NULL)
			(void)pcd->pcd_release_pmc(cpu, adjri, pm);
		pm->pm_id = PMC_ID_MAKE_ID(PMC_CPU_ANY, mode,
		    pe->pe_alloc.pm_class, PMC_ROW_UNASSIGNED);
		/* Undo the disposition we took in pmu_assign_one(). */
		if (sys)
			hwpmc_unmark_row_standalone(n);
		else
			hwpmc_unmark_row_thread(n);
	}
	pg->pg_assigned = false;
}

/*
 * Sort events by pe_cons.pc_weight ascending into 'order[]'.  Stable
 * insertion-sort; ties keep insertion order.
 */
static void
pmu_sort_by_weight(pmu_group_t *pg, pmu_event_t **order, u_int n)
{
	pmu_event_t *pe, *cur;
	u_int i, j;

	i = 0;
	TAILQ_FOREACH(pe, &pg->pg_events, pe_sibling) {
		if (i >= n)
			break;
		order[i++] = pe;
	}
	for (i = 1; i < n; i++) {
		cur = order[i];
		j = i;
		while (j > 0 &&
		    order[j - 1]->pe_cons.pc_weight > cur->pe_cons.pc_weight) {
			order[j] = order[j - 1];
			j--;
		}
		order[j] = cur;
	}
}

static int
pmu_group_probe(pmu_group_t *pg, struct proc *p, int cpu,
    uint64_t evictable_rows)
{
	pmu_event_t *order[PMC_GROUP_MAX_MEMBERS];
	uint32_t used_mask;
	u_int i;
	int error;

	if (pg == NULL || pg->pg_nevents == 0)
		return (EINVAL);
	KASSERT(pg->pg_nevents <= PMC_GROUP_MAX_MEMBERS,
	    ("[pmu] probe: gid=%u nevents=%u", pg->pg_id, pg->pg_nevents));
	pmu_sort_by_weight(pg, order, pg->pg_nevents);
	used_mask = 0;
	error = 0;
	for (i = 0; i < pg->pg_nevents; i++) {
		error = pmu_assign_one(order[i], p, cpu, &used_mask, true,
		    evictable_rows);
		if (error != 0)
			break;
	}
	return (error == 0 ? 0 : ENOSPC);
}

int
pmu_group_can_fit(pmu_group_t *pg)
{
	return (pmu_group_probe(pg, NULL, 0, 0));
}

int
pmu_group_can_place(pmu_group_t *pg, struct proc *p, int cpu)
{
	if (p == NULL)
		return (EINVAL);
	return (pmu_group_probe(pg, p, cpu, 0));
}

/*
 * Third probe view (spec §7.2): could pg be placed if every row in
 * evictable_rows -- the rows currently held by the domain's placed MUX
 * groups -- were freed?  ENOSPC here means the group is genuinely
 * unsatisfiable: pinned groups and ungrouped PMCs hold the rows it
 * needs permanently, so rotation must skip it rather than evict for it.
 */
int
pmu_group_satisfiable(pmu_group_t *pg, struct proc *p, int cpu,
    uint64_t evictable_rows)
{
	if (p == NULL)
		return (EINVAL);
	return (pmu_group_probe(pg, p, cpu, evictable_rows));
}

int
pmu_assign_group(pmu_group_t *pg, struct proc *p, int cpu)
{
	pmu_event_t *order[PMC_GROUP_MAX_MEMBERS];
	uint32_t used_mask;
	u_int i;
	int error;

	if (pg == NULL || p == NULL)
		return (EINVAL);
	if (pg->pg_assigned)
		return (0);

	/*
	 * pmu_assign_group is called both from pmu_group_commit (where
	 * pg_committed is still false because we are mid-commit) and
	 * from pmu_pp_schedule_in (where pg_committed is true).  Don't
	 * gate on pg_committed; pmu_validate_group below covers all the
	 * structural invariants we actually care about.
	 */
	error = pmu_validate_group(pg);
	if (error != 0)
		return (error);

	KASSERT(pg->pg_nevents <= PMC_GROUP_MAX_MEMBERS,
	    ("[pmu] assign: gid=%u nevents=%u", pg->pg_id, pg->pg_nevents));

	/* See comment in pmu_unassign_group: AMD insists on cpu >= 0. */
	if (cpu < 0)
		cpu = 0;

	pmu_sort_by_weight(pg, order, pg->pg_nevents);

	used_mask = 0;
	error = 0;
	for (i = 0; i < pg->pg_nevents; i++) {
		error = pmu_assign_one(order[i], p, cpu, &used_mask, false, 0);
		if (error != 0)
			break;
	}

	if (error != 0) {
		pmu_unassign_group(pg, cpu);
		return (error);
	}

	pg->pg_assigned = true;
	return (0);
}

int
pmu_validate_group(pmu_group_t *pg)
{
	pmu_event_t *pe;
	pmc_sched_constraint_t cons;
	enum pmc_mode lmode;
	bool sys;
	u_int nleaders;
	int lcpu, rc;

	if (pg == NULL || pg->pg_leader == NULL)
		return (EINVAL);
	if (pg->pg_nevents == 0)
		return (EINVAL);
	nleaders = 0;

	/*
	 * Every sibling must share the leader's world.  A group is
	 * either fully virtual (per-process) or fully system-wide; mixed
	 * groups make no sense because they would rotate on different
	 * schedulers.  For system-wide groups we additionally require a
	 * single bound CPU -- all-or-none placement is per-CPU, so
	 * siblings on different CPUs could never be co-scheduled.
	 *
	 * A system-wide group must also be COUNTING (PMC_MODE_SC).  Every
	 * other system-wide mode -- PMC_MODE_SS today -- is rejected: those
	 * PMCs are published in per-CPU hardware and fanned out to a row on
	 * every CPU, and their owners carry po_sscount accounting, none of
	 * which this layer's single-bound-CPU, all-or-none placement models.
	 * Reject them cleanly so userland falls back instead of silently
	 * mis-sampling.
	 */
	lmode = pg->pg_leader->pe_alloc.pm_mode;
	sys = PMC_IS_SYSTEM_MODE(lmode);
	lcpu = pg->pg_leader->pe_alloc.pm_cpu;
	if (sys && lmode != PMC_MODE_SC) {
		PMCDBG2(PMC, OPS, 1,
		    "validate: gid=%u system sampling groups unsupported "
		    "(mode=%d)", pg->pg_id, (int)lmode);
		return (EOPNOTSUPP);
	}

	TAILQ_FOREACH(pe, &pg->pg_events, pe_sibling) {
		enum pmc_mode m = pe->pe_alloc.pm_mode;

		if (pe->pe_is_leader) {
			nleaders++;
			if (pg->pg_leader != pe)
				return (EINVAL);
		} else if ((pe->pe_alloc.pm_flags & PMC_F_GROUP_MUX) != 0)
			return (EINVAL);
		if (pe->pe_pmc == NULL)
			return (EINVAL);
		if (!PMC_ROW_IS_UNASSIGNED(pe->pe_pmc)) {
			PMCDBG2(PMC, OPS, 1,
			    "validate: gid=%u pm_id=0x%jx already assigned",
			    pg->pg_id, (uintmax_t)pe->pe_pmc->pm_id);
			return (EINVAL);
		}
		if ((pe->pe_alloc.pm_flags & PMC_F_GROUP_DEFER) == 0)
			return (EINVAL);
		if (PMC_IS_SYSTEM_MODE(m) != sys) {
			PMCDBG1(PMC, OPS, 1,
			    "validate: gid=%u mixed system/virtual siblings",
			    pg->pg_id);
			return (EINVAL);
		}
		if (sys) {
			if (m != PMC_MODE_SC)
				return (EOPNOTSUPP);
			if (pe->pe_alloc.pm_cpu != lcpu) {
				PMCDBG3(PMC, OPS, 1,
				    "validate: gid=%u system siblings span "
				    "cpus (%d != %d)", pg->pg_id,
				    pe->pe_alloc.pm_cpu, lcpu);
				return (EINVAL);
			}
		} else if (!PMC_IS_VIRTUAL_MODE(m)) {
			return (EINVAL);
		}
		rc = pmu_event_get_constraint(pe, &cons);
		if (rc != 0) {
			PMCDBG3(PMC, OPS, 1,
			    "validate: constraint err=%d class=%d sub=%d",
			    rc, (int)pe->pe_alloc.pm_class,
			    (int)pe->pe_alloc.pm_md.pm_amd.pm_amd_sub_class);
			return (EOPNOTSUPP);
		}
		if (cons.pc_allowed_rows == 0)
			return (EOPNOTSUPP);
	}
	if (nleaders != 1)
		return (EINVAL);
	return (0);
}
