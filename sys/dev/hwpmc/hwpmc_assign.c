/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Advanced Micro Devices, Inc.
 *
 * Most-constrained-first greedy assigner for hwpmc grouping/multiplex.
 *
 * Algorithm:
 *   1. Each event has a pmc_sched_constraint published by its class
 *      backend (today: amd_get_sched_constraint).  pc_allowed_rows is
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
#include "hwpmc_amd.h"

/*
 * Map class+sub-class to a constraint provider.  Today only AMD is
 * wired up; other backends fall through and the event is rejected by
 * pmu_validate_group below.
 */
int
pmu_event_get_constraint(struct pmu_event *pe,
    struct pmc_sched_constraint *cons)
{
	const struct pmc_op_pmcallocate *a;

	if (pe == NULL || cons == NULL)
		return (EINVAL);
	a = &pe->pe_alloc;
	switch (a->pm_class) {
	case PMC_CLASS_K8:
		return (amd_get_sched_constraint(pe->pe_pmc, a, cons));
	default:
		break;
	}
	return (EOPNOTSUPP);
}

u_int
pmu_count_core_hw_slots(struct pmu_group *pg, struct proc *p)
{
	struct pmu_event *pe;
	struct pmc_mdep *mdep;
	struct pmc_classdep *pcd;
	enum pmc_mode mode;
	u_int n, slots;
	int adjri;

	mdep = hwpmc_get_mdep();
	if (mdep == NULL || pg == NULL || pg->pg_leader == NULL)
		return (0);

	pe = pg->pg_leader;
	mode = pe->pe_alloc.pm_mode;
	slots = 0;
	for (n = 0; n < mdep->pmd_npmc; n++) {
		/*
		 * Restrict to rows belonging to the same PMC class as the
		 * leader, then translate the global row index `n' into the
		 * class-relative `adjri' before calling class-specific
		 * helpers like amd_can_assign_pmc().  Class helpers index
		 * into per-class descriptor tables (e.g., amd_pmcdesc[])
		 * sized to amd_npmcs, so feeding them the global `n' would
		 * trip their KASSERT bounds check or read the wrong row.
		 */
		pcd = hwpmc_ri_to_classdep(n, &adjri);
		if (pcd == NULL || pcd->pcd_caps == 0 ||
		    pcd->pcd_class != pe->pe_alloc.pm_class)
			continue;
		if (!hwpmc_can_allocate_row(n, mode) ||
		    !hwpmc_can_allocate_rowindex(p, n, PMC_CPU_ANY))
			continue;
		if (pe->pe_alloc.pm_class == PMC_CLASS_K8 &&
		    amd_can_assign_pmc(adjri, pe->pe_pmc, &pe->pe_alloc) != 0)
			continue;
		slots++;
	}
	PMCDBG3(MDP, ALL, 2, "count_slots: gid=%u class=%d slots=%u",
	    pg->pg_id, pe->pe_alloc.pm_class, slots);
	return (slots);
}

/*
 * Total number of HW rows that belong to the leader's PMC class on
 * this kernel, regardless of whether they are currently in use.
 * pmu_group_commit uses this to reject groups that could never fit
 * even on a freshly-booted system (nevents > class_total -> ENOSPC).
 * Per-process / per-row contention is the job of pmu_count_core_hw_slots.
 */
u_int
pmu_count_class_total(struct pmu_group *pg)
{
	struct pmu_event *pe;
	struct pmc_mdep *mdep;
	struct pmc_classdep *pcd;
	u_int n, slots;
	int adjri;

	mdep = hwpmc_get_mdep();
	if (mdep == NULL || pg == NULL || pg->pg_leader == NULL)
		return (0);

	pe = pg->pg_leader;
	slots = 0;
	for (n = 0; n < mdep->pmd_npmc; n++) {
		pcd = hwpmc_ri_to_classdep(n, &adjri);
		if (pcd == NULL || pcd->pcd_caps == 0)
			continue;
		if (pcd->pcd_class != pe->pe_alloc.pm_class)
			continue;
		slots++;
	}
	return (slots);
}

/*
 * Try to bind one event to a row.  used_mask tracks rows already
 * consumed by earlier events in this group.
 */
static int
pmu_assign_one(struct pmu_event *pe, struct proc *p, int cpu,
    uint32_t *used_mask)
{
	struct pmc *pm;
	struct pmc_classdep *pcd;
	struct pmc_mdep *mdep;
	enum pmc_mode mode;
	uint32_t allowed;
	int adjri, n;

	/*
	 * pe_cons.pc_allowed_rows / *used_mask / pc_fixed_row all live in
	 * the per-class adjri namespace (e.g., bits 0..5 for AMD CORE).
	 * Global ri = pcd->pcd_ri + adjri.  We iterate adjri space and
	 * convert to global ri only where the framework requires it.
	 */
	mdep = hwpmc_get_mdep();
	pm = pe->pe_pmc;
	mode = pe->pe_alloc.pm_mode;
	allowed = pe->pe_cons.pc_allowed_rows;

	if ((pe->pe_cons.pc_flags & PMC_SC_F_FIXED) != 0) {
		adjri = pe->pe_cons.pc_fixed_row;
		if (((1u << adjri) & allowed) == 0 ||
		    (*used_mask & (1u << adjri)) != 0)
			return (EBUSY);
	} else {
		adjri = -1;
	}

	for (;;) {
		uint32_t free_mask;
		int dummy_adjri;

		if (adjri < 0) {
			free_mask = allowed & ~*used_mask;
			if (free_mask == 0)
				return (EBUSY);
			adjri = ffs(free_mask) - 1;
		}
		/* Locate the global ri that owns this adjri for our class. */
		pcd = NULL;
		for (n = 0; n < (int)mdep->pmd_npmc; n++) {
			pcd = hwpmc_ri_to_classdep(n, &dummy_adjri);
			if (pcd != NULL &&
			    pcd->pcd_class == pe->pe_alloc.pm_class &&
			    dummy_adjri == adjri)
				break;
			pcd = NULL;
		}
		if (pcd == NULL || n >= (int)mdep->pmd_npmc)
			goto skip;
		if (!hwpmc_can_allocate_row(n, mode) ||
		    !hwpmc_can_allocate_rowindex(p, n, PMC_CPU_ANY))
			goto skip;
		if (pe->pe_alloc.pm_class == PMC_CLASS_K8 &&
		    amd_can_assign_pmc(adjri, pm, &pe->pe_alloc) != 0)
			goto skip;
		if (pcd->pcd_allocate_pmc(cpu, adjri, pm, &pe->pe_alloc) != 0)
			goto skip;

		*used_mask |= 1u << adjri;
		pm->pm_id = PMC_ID_MAKE_ID(PMC_CPU_ANY, mode,
		    pe->pe_alloc.pm_class, n);
		hwpmc_mark_row_thread(n);
		pe->pe_state = PMU_EVENT_STATE_ACTIVE;
		pe->pe_assigned_row = n;
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
pmu_unassign_group(struct pmu_group *pg, int cpu)
{
	struct pmu_event *pe;
	struct pmc *pm;
	struct pmc_classdep *pcd;
	enum pmc_mode mode;
	int adjri, n;

	/*
	 * AMD's pcd_release_pmc is logically a no-op but its KASSERT
	 * still requires cpu >= 0.  PMC_CPU_ANY (-1) is a perfectly
	 * legitimate "any CPU" value for virtual-mode PMCs, so coerce
	 * it to CPU 0 here rather than fan it out across the per-class
	 * back-ends.
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
		hwpmc_mark_row_free(n);
		pe->pe_state = PMU_EVENT_STATE_INACTIVE;
		pe->pe_assigned_row = -1;
	}
	pg->pg_used_rows_mask = 0;
	pg->pg_assigned = false;
}

/*
 * Sort events by pe_cons.pc_weight ascending into 'order[]'.  Stable
 * insertion-sort; ties keep insertion order.
 */
static void
pmu_sort_by_weight(struct pmu_group *pg, struct pmu_event **order, u_int n)
{
	struct pmu_event *pe, *cur;
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

int
pmu_assign_group(struct pmu_group *pg, struct proc *p, int cpu)
{
	struct pmu_event **order;
	struct pmc_mdep *mdep;
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

	mdep = hwpmc_get_mdep();
	if (mdep == NULL || pg->pg_nevents == 0)
		return (EINVAL);

	/* See comment in pmu_unassign_group: AMD insists on cpu >= 0. */
	if (cpu < 0)
		cpu = 0;

	order = malloc(sizeof(*order) * pg->pg_nevents, M_PMC,
	    M_WAITOK | M_ZERO);
	pmu_sort_by_weight(pg, order, pg->pg_nevents);

	used_mask = 0;
	error = 0;
	for (i = 0; i < pg->pg_nevents; i++) {
		error = pmu_assign_one(order[i], p, cpu, &used_mask);
		if (error != 0)
			break;
	}
	free(order, M_PMC);

	if (error != 0) {
		pmu_unassign_group(pg, cpu);
		return (error);
	}

	pg->pg_used_rows_mask = used_mask;
	pg->pg_assigned = true;
	return (0);
}

int
pmu_validate_group(struct pmu_group *pg)
{
	struct pmu_event *pe;
	struct pmc_sched_constraint cons;
	int rc;

	if (pg == NULL || pg->pg_leader == NULL)
		return (EINVAL);
	if (pg->pg_nevents < 2) {
		PMCDBG2(PMC, OPS, 1, "validate: gid=%u nevents=%u < 2",
		    pg->pg_id, pg->pg_nevents);
		return (EINVAL);
	}

	TAILQ_FOREACH(pe, &pg->pg_events, pe_sibling) {
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
		if (!PMC_IS_VIRTUAL_MODE(pe->pe_alloc.pm_mode))
			return (EINVAL);
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
	return (0);
}
