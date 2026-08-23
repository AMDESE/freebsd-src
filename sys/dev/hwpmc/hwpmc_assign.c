/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Advanced Micro Devices, Inc.
 *
 * Greedy row assigner for PMU event groups.
 *
 * Algorithm:
 * 1. Read event constraints from the class backend.
 * 2. Sort events by weight in ascending order.
 * 3. Assign each event to the first available allowed row.
 * 4. On assignment failure, release all assigned rows.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/pmc.h>
#include <sys/proc.h>

#include "hwpmc_pmu.h"

/*
 * Find the class descriptor for a PMC class.
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
 * Check row assignment feasibility with the class backend.
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
 * Get scheduling constraints for an event from the class backend.
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
 * Return true if the PMC class supports event grouping.
 */
bool
pmu_class_supports_grouping(enum pmc_class class)
{
	struct pmc_classdep *pcd;

	pcd = pmu_class_to_classdep(class);
	return (pcd != NULL && pcd->pcd_get_sched_constraint != NULL);
}

/*
 * Assign one event to a hardware row.
 */
static int
pmu_assign_one(pmu_group_t *pg, pmu_event_t *pe, struct proc *p, int cpu,
    uint32_t *used_mask, bool dry_run, uint64_t evictable_rows)
{
	struct pmu_group_target *pgt;
	struct pmc *pm;
	struct pmc_classdep *pcd;
	enum pmc_mode mode;
	uint32_t allowed, free_mask;
	int adjri, idcpu, n;
	bool sys;

	/*
	 * Constraints use the per-class row index (adjri).
	 * Global row index is pcd->pcd_ri + adjri.
	 */
	pcd = pmu_class_to_classdep(pe->pe_alloc.pm_class);
	if (pcd == NULL)
		return (EBUSY);
	pm = pe->pe_pmc;
	mode = pe->pe_alloc.pm_mode;
	allowed = pe->pe_cons.pc_allowed_rows;
	/* System mode binds to a specific CPU; virtual mode uses any CPU. */
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
		if (p != NULL) {
			if (!hwpmc_can_allocate_row(n, mode) &&
			    (n >= 64 ||
			    (evictable_rows & (1ULL << n)) == 0))
				goto skip;
			if (LIST_EMPTY(&pg->pg_targets)) {
				if (!hwpmc_can_allocate_rowindex(p, n, idcpu) &&
				    (n >= 64 ||
				    (evictable_rows & (1ULL << n)) == 0))
					goto skip;
			} else {
				LIST_FOREACH(pgt, &pg->pg_targets,
				    pgt_group_next) {
					if (hwpmc_can_allocate_rowindex(
					    pgt->pgt_pp->pp_proc, n, idcpu))
						continue;
					if (n < 64 &&
					    (evictable_rows & (1ULL << n)) != 0)
						continue;
					goto skip;
				}
			}
		}
		/* Check system row occupancy. */
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
		/* Mark row as STANDALONE for system mode or THREAD for virtual mode. */
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
 * Release all hardware rows assigned to a group.
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

	/* Set CPU index to 0 for virtual mode. */
	if (cpu < 0)
		cpu = 0;

	TAILQ_FOREACH(pe, &pg->pg_events, pe_sibling) {
		pm = pe->pe_pmc;
		if (PMC_ROW_IS_UNASSIGNED(pm))
			continue;
		n = PMC_TO_ROWINDEX(pm);
		mode = PMC_TO_MODE(pm);
		pcd = hwpmc_ri_to_classdep(n, &adjri);
		if (pcd != NULL) {
			/*
			 * Clear configuration before releasing the row.
			 * Unconfigure all CPUs for virtual mode.
			 */
			if (sys)
				(void)pcd->pcd_config_pmc(cpu, adjri, NULL);
			else
				hwpmc_unconfigure_row_all_cpus(pm, n);
			(void)pcd->pcd_release_pmc(cpu, adjri, pm);
		}
		pm->pm_id = PMC_ID_MAKE_ID(PMC_CPU_ANY, mode,
		    pe->pe_alloc.pm_class, PMC_ROW_UNASSIGNED);
		/* Clear row disposition. */
		if (sys)
			hwpmc_unmark_row_standalone(n);
		else
			hwpmc_unmark_row_thread(n);
	}
	pg->pg_assigned = false;
}

/*
 * Sort events by constraint weight in ascending order.
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
		error = pmu_assign_one(pg, order[i], p, cpu, &used_mask, true,
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
 * Check if a group can fit when evictable rows become free.
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

	/* Validate group configuration before assignment. */
	error = pmu_validate_group(pg);
	if (error != 0)
		return (error);

	KASSERT(pg->pg_nevents <= PMC_GROUP_MAX_MEMBERS,
	    ("[pmu] assign: gid=%u nevents=%u", pg->pg_id, pg->pg_nevents));

	/* Set CPU index to 0 for virtual mode. */
	if (cpu < 0)
		cpu = 0;

	pmu_sort_by_weight(pg, order, pg->pg_nevents);

	used_mask = 0;
	error = 0;
	for (i = 0; i < pg->pg_nevents; i++) {
		error = pmu_assign_one(pg, order[i], p, cpu, &used_mask, false,
		    0);
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
	bool sys;
	u_int nleaders;
	int lcpu, rc;

	if (pg == NULL || pg->pg_leader == NULL)
		return (EINVAL);
	if (pg->pg_nevents == 0)
		return (EINVAL);
	nleaders = 0;

	/*
	 * Validate group mode.  Siblings must be on the leader's side of the
	 * virtual/system split; counting and sampling may mix within a side.
	 * System groups bind to one CPU.
	 */
	sys = PMC_IS_SYSTEM_MODE(pg->pg_leader->pe_alloc.pm_mode);
	lcpu = pg->pg_leader->pe_alloc.pm_cpu;

	/*
	 * PMC_F_DESCENDANTS is leader-only and governs the whole group.  A
	 * system group is bound to a CPU and has no process target, so it
	 * cannot follow a fork.
	 */
	if (sys && (pg->pg_leader->pe_alloc.pm_flags & PMC_F_DESCENDANTS) !=
	    0) {
		PMCDBG1(PMC, OPS, 1,
		    "validate: gid=%u PMC_F_DESCENDANTS on a system group",
		    pg->pg_id);
		return (EINVAL);
	}

	TAILQ_FOREACH(pe, &pg->pg_events, pe_sibling) {
		enum pmc_mode m = pe->pe_alloc.pm_mode;

		if (pe->pe_is_leader) {
			nleaders++;
			if (pg->pg_leader != pe)
				return (EINVAL);
		} else if ((pe->pe_alloc.pm_flags &
		    (PMC_F_GROUP_MUX | PMC_F_DESCENDANTS)) != 0) {
			/* Both flags are leader-only. */
			return (EINVAL);
		}
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
			PMCDBG2(PMC, OPS, 1,
			    "validate: constraint err=%d class=%d",
			    rc, (int)pe->pe_alloc.pm_class);
			return (EOPNOTSUPP);
		}
		if (cons.pc_allowed_rows == 0)
			return (EOPNOTSUPP);
	}
	if (nleaders != 1)
		return (EINVAL);
	return (0);
}
