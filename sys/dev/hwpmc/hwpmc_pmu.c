/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Advanced Micro Devices, Inc.
 *
 * hwpmc PMU layer: event groups, all-or-none commit, time accounting and
 * round-robin multiplex rotation when nevents > available HW slots.  The
 * heavy lifting (per-class hardware row binding, KASSERT-checked AMD
 * sub-class match, etc.) lives in hwpmc_assign.c and the per-class
 * back-ends (hwpmc_amd.c, hwpmc_intel.c).
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/kthread.h>
#include <sys/malloc.h>
#include <sys/pmc.h>
#include <sys/proc.h>
#include <sys/sysctl.h>
#include <sys/sx.h>
#include <sys/unistd.h>

#include "hwpmc_pmu.h"

static MALLOC_DEFINE(M_PMU, "pmu", "hwpmc PMU grouping");

static uint32_t pmu_next_group_id = 1;

SYSCTL_DECL(_kern_hwpmc);

/*
 * Multiplex rotation period.  Default 50ms balances counter accuracy
 * (longer windows -> more stable counts per sibling) against fairness
 * (shorter windows -> every sibling gets time on HW within a few
 * scheduler quanta).  Tunable so tests with very long or very short
 * runtimes can adjust.
 */
static int pmu_mux_period_ms = 50;
SYSCTL_INT(_kern_hwpmc, OID_AUTO, mux_period_ms, CTLFLAG_RWTUN,
    &pmu_mux_period_ms, 0,
    "PMU multiplex rotation period in milliseconds");

static void pmu_pp_rotate_thread(void *arg);
static int pmu_pp_schedule_in(struct pmc_process *pp, struct pmu_group *pg);
static void pmu_pp_schedule_out(struct pmc_process *pp, struct pmu_group *pg);
static void pmu_pp_kick_rotate(struct pmc_process *pp);
static int pmu_group_attach_siblings(struct pmu_group *pg,
    struct pmc_process *pp);

struct pmu_event *
pmu_event_from_pmc(struct pmc *pm)
{
	return (pm != NULL ? pm->pm_pmu : NULL);
}

void
pmu_event_destroy(struct pmu_event *pe)
{
	if (pe == NULL)
		return;
	free(pe, M_PMU);
}

int
pmu_group_create(struct pmc_owner *po, uint32_t *pg_id)
{
	struct pmu_group *pg;

	if (po == NULL || pg_id == NULL)
		return (EINVAL);

	pg = malloc(sizeof(*pg), M_PMU, M_WAITOK | M_ZERO);
	pg->pg_id = pmu_next_group_id++;
	pg->pg_owner = po;
	TAILQ_INIT(&pg->pg_events);
	LIST_INSERT_HEAD(&po->po_groups, pg, pg_owner_next);
	*pg_id = pg->pg_id;
	return (0);
}

struct pmu_group *
pmu_group_lookup(struct pmc_owner *po, uint32_t pg_id)
{
	struct pmu_group *pg;

	LIST_FOREACH(pg, &po->po_groups, pg_owner_next) {
		if (pg->pg_id == pg_id)
			return (pg);
	}
	return (NULL);
}

int
pmu_group_add(struct pmu_group *pg, struct pmc *pm, bool leader)
{
	struct pmu_event *pe;

	if (pg == NULL || pm == NULL)
		return (EINVAL);
	if (pg->pg_committed) {
		PMCDBG1(PMC, OPS, 1, "group_add: gid=%u already committed",
		    pg->pg_id);
		return (EBUSY);
	}

	pe = pmu_event_from_pmc(pm);
	if (pe == NULL) {
		PMCDBG2(PMC, OPS, 1,
		    "group_add: pm_pmu==NULL pm=%p flags=0x%x",
		    pm, pm->pm_flags);
		return (EINVAL);
	}
	if (!PMC_ROW_IS_UNASSIGNED(pe->pe_pmc)) {
		PMCDBG2(PMC, OPS, 1,
		    "group_add: already assigned pm=%p id=0x%jx",
		    pm, (uintmax_t)pm->pm_id);
		return (EBUSY);
	}
	if (pe->pe_group == pg)
		return (0);
	if (pe->pe_group != NULL) {
		PMCDBG3(PMC, OPS, 1,
		    "group_add: pe %p in gid=%u, requested gid=%u",
		    pe, pe->pe_group->pg_id, pg->pg_id);
		return (EBUSY);
	}

	pe->pe_group = pg;
	pe->pe_is_leader = leader;
	TAILQ_INSERT_TAIL(&pg->pg_events, pe, pe_sibling);
	pg->pg_nevents++;

	if (leader)
		pg->pg_leader = pe;
	else if (pg->pg_leader == NULL)
		pg->pg_leader = pe;

	return (0);
}

int
pmu_group_commit(struct pmu_group *pg)
{
	struct pmu_event *pe;
	struct pmc_owner *po;
	struct proc *p;
	u_int hw_slots, class_total;
	int error;

	if (pg == NULL)
		return (EINVAL);
	if (pg->pg_committed)
		return (0);

	error = pmu_validate_group(pg);
	if (error != 0) {
		PMCDBG3(PMC, OPS, 1,
		    "group_commit: validate gid=%u err=%d nevents=%u",
		    pg->pg_id, error, pg->pg_nevents);
		return (error);
	}

	/*
	 * Pre-compute each event's constraint up front so the assigner
	 * can sort by weight without recomputing.
	 */
	TAILQ_FOREACH(pe, &pg->pg_events, pe_sibling) {
		error = pmu_event_get_constraint(pe, &pe->pe_cons);
		if (error != 0) {
			PMCDBG3(PMC, OPS, 1,
			    "group_commit: constraint err=%d class=%d sub=%d",
			    error, (int)pe->pe_alloc.pm_class,
			    (int)pe->pe_alloc.pm_md.pm_amd.pm_amd_sub_class);
			return (error);
		}
	}

	pg->pg_defer_ok = (pg->pg_leader->pe_alloc.pm_flags &
	    PMC_F_GROUP_MUX) != 0;

	po = pg->pg_owner;
	p = po != NULL ? po->po_owner : NULL;
	if (p != NULL) {
		class_total = pmu_count_class_total(pg);
		if (class_total == 0)
			return (EOPNOTSUPP);
		/*
		 * Reject groups that can never fit on the hardware no
		 * matter what.  Within-group placement is strictly
		 * all-or-none, so a single group with more events than
		 * the entire class has rows is unschedulable forever.
		 */
		if (pg->pg_nevents > class_total)
			return (ENOSPC);

		hw_slots = pmu_count_core_hw_slots(pg, p);

		/*
		 * Two scheduling regimes, both atomic:
		 *
		 * 1. The group fits in the rows currently free for this
		 *    target proc -- bind every sibling to a HW row right
		 *    now.  pmc_start(leader) finds an already-scheduled
		 *    group and just flips the per-PMC running flag.
		 *
		 * 2. The group does NOT fit at the moment (other groups
		 *    already attached to this proc are holding the rows
		 *    we would need).  If the caller passed
		 *    PMC_F_GROUP_MUX on the leader we accept the commit
		 *    in the DEFERRED state; the per-pp rotation kthread
		 *    will atomically schedule this group in -- ALL of
		 *    its siblings, never a subset -- whenever an evicted
		 *    peer frees enough rows for it.  Without the flag we
		 *    preserve the historical strict semantics and reject
		 *    with ENOSPC so the caller can pick a smaller group.
		 */
		PMCDBG5(PMC, OPS, 1,
		    "group_commit: gid=%u nevents=%u class_total=%u "
		    "hw_slots=%u defer_ok=%d", pg->pg_id, pg->pg_nevents,
		    class_total, hw_slots, (int)pg->pg_defer_ok);

		if (pg->pg_nevents <= hw_slots) {
			error = pmu_assign_group(pg, p, PMC_CPU_ANY);
			if (error != 0) {
				PMCDBG2(PMC, OPS, 1,
				    "group_commit: assign gid=%u err=%d",
				    pg->pg_id, error);
				return (error);
			}
			PMCDBG1(PMC, OPS, 1,
			    "group_commit: gid=%u BOUND", pg->pg_id);
		} else if (!pg->pg_defer_ok) {
			PMCDBG3(PMC, OPS, 1,
			    "group_commit: gid=%u ENOSPC %u > %u slots",
			    pg->pg_id, pg->pg_nevents, hw_slots);
			return (ENOSPC);
		} else {
			PMCDBG3(PMC, OPS, 1,
			    "group_commit: gid=%u DEFERRED %u > %u slots",
			    pg->pg_id, pg->pg_nevents, hw_slots);
		}
	}

	pg->pg_committed = true;
	return (0);
}

void
pmu_group_release(struct pmu_group *pg)
{
	struct pmu_event *pe;

	if (pg == NULL)
		return;

	/*
	 * The per-pp rotation kthread is torn down by pmu_group_on_release
	 * when the last group leaves a pp, so we have no per-group
	 * kthread to drain here.  pmu_group_release is reachable only
	 * after every sibling has been removed from the TAILQ, but be
	 * defensive against direct callers that bypass that path.
	 */
	while ((pe = TAILQ_FIRST(&pg->pg_events)) != NULL) {
		TAILQ_REMOVE(&pg->pg_events, pe, pe_sibling);
		pe->pe_group = NULL;
		pe->pe_is_leader = false;
	}

	if (pg->pg_owner != NULL)
		LIST_REMOVE(pg, pg_owner_next);
	/*
	 * Membership of pg on a pp's pp_pmu_groups list is tracked by
	 * pg_pp, not pg_attach_proc.  pg_attach_proc is set by
	 * pmc_attach_one_process even for groups that are never started
	 * (so never inserted onto pp_pmu_groups), so gating the
	 * LIST_REMOVE on pg_attach_proc would unlink a pg that was
	 * never linked and panic LIST_REMOVE with "prev->next != elm".
	 * Conversely, when pmu_group_on_release already pulled pg off
	 * pp_pmu_groups it cleared pg_pp, so this branch correctly
	 * skips the second LIST_REMOVE.
	 */
	if (pg->pg_pp != NULL) {
		if (pg->pg_pp->pp_pmu_rot_cursor == pg)
			pg->pg_pp->pp_pmu_rot_cursor = LIST_NEXT(pg,
			    pg_proc_next);
		LIST_REMOVE(pg, pg_proc_next);
		pg->pg_pp = NULL;
	}
	free(pg, M_PMU);
}

int
pmu_group_on_allocate(struct pmc *pm, const struct pmc_op_pmcallocate *pa)
{
	struct pmu_event *pe;

	pe = malloc(sizeof(*pe), M_PMU, M_WAITOK | M_ZERO);
	pe->pe_pmc = pm;
	pe->pe_alloc = *pa;
	pe->pe_state = PMU_EVENT_STATE_INACTIVE;
	pe->pe_assigned_row = -1;
	pm->pm_pmu = pe;
	return (0);
}

/*
 * Resolve the TARGET proc the group is being scheduled against.  All
 * pp_pmu_groups bookkeeping, pp_pmcs[] writes and rotation must hang
 * off the target's pmc_process -- that is the pp the scheduler walks
 * at csw_in/out time for the proc whose threads are actually running
 * the workload being measured.
 *
 * Lookup order:
 *   1. pg->pg_attach_proc, recorded by pmc_attach_one_process for the
 *      first sibling that successfully attaches (committed-bound or
 *      deferred-skip path; see hwpmc_mod.c).  This is the canonical
 *      and only correct source.
 *   2. Fallback: the pmc_target list of any committed sibling.  In
 *      practice all siblings of one group target the same proc, so
 *      the first non-empty pm_targets gives us the answer.  This is
 *      defensive only; once (1) is set on attach we never get here.
 *
 * pg->pg_owner / pg->pg_owner->po_owner is INTENTIONALLY NOT used
 * here: the owner (e.g. pmcstat parent) is typically NOT the target
 * (e.g. the child workload), and using owner-pp produced the
 * "Row index mismatch pmc 255 != ri N" csw_in panic where rotation
 * unbound rows on owner-pp while target-pp still held the stale
 * pp_pmcs[ri] -> pm pointer.
 */
static struct proc *
pmu_group_target_proc(struct pmu_group *pg)
{
	struct pmu_event *pe;
	struct pmc_target *ptgt;

	if (pg == NULL)
		return (NULL);
	if (pg->pg_attach_proc != NULL)
		return (pg->pg_attach_proc);
	TAILQ_FOREACH(pe, &pg->pg_events, pe_sibling) {
		if (pe->pe_pmc == NULL)
			continue;
		ptgt = LIST_FIRST(&pe->pe_pmc->pm_targets);
		if (ptgt != NULL && ptgt->pt_process != NULL)
			return (ptgt->pt_process->pp_proc);
	}
	return (NULL);
}

int
pmu_group_on_start(struct pmc *pm)
{
	struct pmu_event *pe;
	struct pmu_group *pg;
	struct pmc_process *pp;
	struct proc *p;
	int error;

	pe = pmu_event_from_pmc(pm);
	if (pe == NULL || pe->pe_group == NULL)
		return (0);

	pg = pe->pe_group;
	if (!pg->pg_committed)
		return (EDOOFUS);

	p = pmu_group_target_proc(pg);
	if (p == NULL) {
		printf("hwpmc: on_start gid=%u pm=%p NO TARGET PROC -> EINVAL\n",
		    pg->pg_id, pm);
		return (EINVAL);
	}

	pp = pmc_find_process_descriptor_pmu(p, PMC_FLAG_ALLOCATE);
	if (pp == NULL)
		return (ENOMEM);

	if (pg->pg_attach_proc == NULL)
		pg->pg_attach_proc = p;

	if (pg->pg_pp == NULL) {
		LIST_INSERT_HEAD(&pp->pp_pmu_groups, pg, pg_proc_next);
		pg->pg_pp = pp;
	}

	pg->pg_running = true;

	if (pg->pg_assigned) {
		printf("hwpmc: on_start gid=%u pm=%p ASSIGNED, attaching siblings on pp=%p (target pid=%d)\n",
		    pg->pg_id, pm, pp,
		    pp->pp_proc != NULL ? pp->pp_proc->p_pid : -1);
		error = pmu_group_attach_siblings(pg, pp);
		if (error != 0) {
			printf("hwpmc: on_start gid=%u attach_siblings err=%d\n",
			    pg->pg_id, error);
			return (error);
		}
	} else {
		int sin_err = pmu_pp_schedule_in(pp, pg);
		printf("hwpmc: on_start gid=%u pm=%p DEFERRED, schedule_in returned %d (pp=%p target pid=%d)\n",
		    pg->pg_id, pm, sin_err, pp,
		    pp->pp_proc != NULL ? pp->pp_proc->p_pid : -1);
	}

	TAILQ_FOREACH(pe, &pg->pg_events, pe_sibling) {
		pe->pe_pmc->pm_state = PMC_STATE_RUNNING;
		printf("hwpmc: on_start gid=%u pm=%p ri=%d pm_state->RUNNING pp_pmcs[ri]=%p\n",
		    pg->pg_id, pe->pe_pmc, PMC_TO_ROWINDEX(pe->pe_pmc),
		    !PMC_ROW_IS_UNASSIGNED(pe->pe_pmc) ?
		    pp->pp_pmcs[PMC_TO_ROWINDEX(pe->pe_pmc)].pp_pmc : NULL);
	}

	pmu_pp_kick_rotate(pp);
	return (0);
}

/*
 * Mark the group stopped.  We deliberately do NOT release HW rows
 * here:
 *
 *   1. The user-facing PMC_OP_PMCSTOP path runs with pmc_sx
 *      DOWNGRADED to a shared lock (see PMC_DOWNGRADE_SX in
 *      pmc_syscall_handler).  pmc_rotation_detach asserts
 *      sx_assert(&pmc_sx, SX_XLOCKED), so calling schedule-out
 *      from here would panic.
 *
 *   2. Stop semantics in the framework only freeze the HW counter,
 *      they do not unbind the row from the target proc -- so
 *      preserving rows across stop matches the existing per-PMC
 *      behaviour and lets a subsequent pmc_start re-run the group
 *      with the same row assignments.
 *
 * The rotation kthread skips stopped groups (it gates on
 * pg_running == true), so a stopped group does not get evicted as
 * a victim.  Final HW row release happens in pmu_group_on_release
 * which always runs under sx_xlock.
 */
void
pmu_group_on_stop(struct pmc *pm)
{
	struct pmu_event *pe;
	struct pmu_group *pg;

	pe = pmu_event_from_pmc(pm);
	if (pe == NULL || pe->pe_group == NULL)
		return;
	pg = pe->pe_group;
	if (!pg->pg_running)
		return;

	pg->pg_running = false;

	TAILQ_FOREACH(pe, &pg->pg_events, pe_sibling)
		pe->pe_pmc->pm_state = PMC_STATE_STOPPED;
}

/*
 * Time accounting and CSW/rotation hooks.  When a group is multiplexed the
 * read path scales the raw HW count by time_running/time_enabled; when it
 * is not multiplexed these helpers degenerate to no-ops because the active
 * flag never toggles.  pmu_rotate_groups is fired by a per-CPU callout
 * driven by kern.hwpmc.mux_period_ns and walks pg_proc_next to swap the
 * active sub-group.
 */
void pmu_event_account_in(struct pmu_event *pe __unused,
    uint64_t now __unused) { }
void pmu_event_account_out(struct pmu_event *pe __unused,
    uint64_t now __unused) { }
void pmu_rotate_groups(int cpu __unused) { }

void
pmu_group_csw_in(struct thread *td __unused, struct pmc_process *pp __unused)
{
}

void
pmu_group_csw_out(struct thread *td __unused, int cpu __unused)
{
}

int
pmu_group_read_value(struct pmc *pm __unused, pmc_value_t *value __unused)
{
	return (0);
}

void
pmu_group_on_release(struct pmc *pm)
{
	struct pmu_event *pe;
	struct pmu_group *pg;
	struct pmc_process *pp;
	struct proc *p;
	bool was_first_release;

	pe = pmu_event_from_pmc(pm);
	if (pe == NULL)
		return;
	pg = pe->pe_group;
	if (pg == NULL)
		goto destroy;

	/*
	 * The pp we hung off pp_pmu_groups is recorded as pg->pg_pp.
	 * It is the TARGET pp, NOT the owner.  Use that directly so we
	 * tear down rotation on the same pp the scheduler walks; falling
	 * back to looking up by attach_proc still works after pg_pp is
	 * cleared by an earlier sibling.
	 */
	pp = pg->pg_pp;
	p = pg->pg_attach_proc;
	if (pp == NULL && p != NULL)
		pp = pmc_find_process_descriptor_pmu(p, 0);

	/*
	 * Tear down the per-pp rotation kthread BEFORE we mutate any
	 * group state.  Once we hit this path we are about to remove pe
	 * from pg->pg_events; allowing the kthread to schedule_in or
	 * schedule_out concurrently could race with that removal.  We
	 * only do this on the FIRST release of any sibling (pg_pp set)
	 * and only when this is the last group on the pp -- otherwise
	 * other groups still want rotation.
	 */
	was_first_release = (pg->pg_pp != NULL);
	if (was_first_release && pp != NULL &&
	    LIST_FIRST(&pp->pp_pmu_groups) == pg &&
	    LIST_NEXT(pg, pg_proc_next) == NULL &&
	    pp->pp_pmu_rot_running) {
		pp->pp_pmu_rot_running = false;
		wakeup(pp);
		while (pp->pp_pmu_rot_td != NULL)
			(void)hwpmc_pmu_sx_sleep(&pp->pp_pmu_rot_td, 1,
			    "muxrel");
	}

	/*
	 * Critical: remove THIS pe from pg_events before any whole-
	 * group operation runs, because the framework's
	 * pmc_release_pmc_descriptor already called pcd_release_pmc /
	 * PMC_UNMARK_ROW_THREAD for this pm.  If pmu_pp_schedule_out
	 * (below) walked the still-present pe it would invoke
	 * pcd_release_pmc on this row a second time and corrupt the
	 * per-class accounting.
	 */
	TAILQ_REMOVE(&pg->pg_events, pe, pe_sibling);
	pe->pe_group = NULL;
	if (pg->pg_leader == pe)
		pg->pg_leader = TAILQ_FIRST(&pg->pg_events);
	if (pg->pg_nevents > 0)
		pg->pg_nevents--;

	/*
	 * Schedule the remaining siblings out atomically: stop them,
	 * drain in-flight csw, detach from pp, release their HW rows.
	 * Skipped on subsequent sibling releases (pg_assigned was
	 * cleared by the first one) and on the last release (no more
	 * siblings to schedule out).
	 */
	if (pg->pg_assigned && pp != NULL && pg->pg_nevents > 0)
		pmu_pp_schedule_out(pp, pg);
	else if (pg->pg_assigned)
		pg->pg_assigned = false;

	/*
	 * Unhook pg from pp->pp_pmu_groups now, while pp is still
	 * alive.  Callers (pmc_release_pmc_descriptor) are required
	 * to invoke us BEFORE their pm_targets unlink loop, because
	 * that loop is what eventually drives pp_refcnt to zero and
	 * frees pp; doing this LIST_REMOVE afterwards would touch
	 * freed memory.  Subsequent sibling releases see pg_pp ==
	 * NULL and skip this branch (and pmu_group_release below
	 * also short-circuits on pg_pp == NULL).
	 */
	if (was_first_release) {
		printf("hwpmc: about to LIST_REMOVE pg=%p pg_pp=%p "
		    "pg_attach_proc=%p pp_pmu_groups.first=%p "
		    "le_prev=%p *le_prev=%p le_next=%p\n",
		    pg, pp, pg->pg_attach_proc,
		    pp != NULL ? LIST_FIRST(&pp->pp_pmu_groups) : NULL,
		    pg->pg_proc_next.le_prev,
		    pg->pg_proc_next.le_prev != NULL ?
		    *pg->pg_proc_next.le_prev : NULL,
		    LIST_NEXT(pg, pg_proc_next));
		if (pp != NULL && pp->pp_pmu_rot_cursor == pg)
			pp->pp_pmu_rot_cursor = LIST_NEXT(pg, pg_proc_next);
		LIST_REMOVE(pg, pg_proc_next);
		pg->pg_pp = NULL;
		pg->pg_attach_proc = NULL;
	}

	if (pg->pg_nevents == 0) {
		pg->pg_used_rows_mask = 0;
		pmu_group_release(pg);
	}

destroy:
	pm->pm_pmu = NULL;
	pmu_event_destroy(pe);
}

/*
 * Hook every still-active sibling of pg onto pp via the rotation
 * attach helper.  Called both from pmu_group_on_start (initial
 * placement when commit already bound the rows) and from
 * pmu_pp_schedule_in (rotation brings the group in mid-flight).
 * Non-ACTIVE siblings should never exist when we get here -- group
 * scheduling is all-or-none, so once pmu_assign_group succeeds every
 * pe is ACTIVE; we still skip them defensively to keep an inconsistent
 * state from panicking.
 */
static int
pmu_group_attach_siblings(struct pmu_group *pg, struct pmc_process *pp)
{
	struct pmu_event *pe;
	struct pmc *pm;

	TAILQ_FOREACH(pe, &pg->pg_events, pe_sibling) {
		if (pe->pe_state != PMU_EVENT_STATE_ACTIVE)
			continue;
		pm = pe->pe_pmc;
		if (pp->pp_pmcs[PMC_TO_ROWINDEX(pm)].pp_pmc == pm)
			continue;	/* idempotent: already attached */
		pmc_rotation_attach(pm, pp);
	}
	return (0);
}

/*
 * Atomically schedule pg in: assign HW rows for ALL siblings or do
 * nothing.  Called either from pmu_group_on_start (lazy first
 * placement) or from the per-pp rotation kthread (when an evicted
 * peer freed enough rows for pg).  Returns 0 if pg is now scheduled,
 * ENOSPC if it still does not fit (pg stays deferred), other errno
 * if something hard-failed.
 */
static int
pmu_pp_schedule_in(struct pmc_process *pp, struct pmu_group *pg)
{
	struct pmu_event *pe;
	struct proc *p;
	u_int hw_slots;
	int error;

	if (pg == NULL || pg->pg_assigned)
		return (0);
	/*
	 * Use the recorded TARGET proc only.  Falling back to the
	 * owner here is what created the cross-pp inconsistency that
	 * panicked csw_in with "pmc 255 != ri N": owner-pp is NOT the
	 * pp the scheduler walks, so attach/detach there silently
	 * corrupts state.
	 */
	p = pmu_group_target_proc(pg);
	if (p == NULL)
		return (EINVAL);
	KASSERT(pp != NULL && pp->pp_proc == p,
	    ("[pmu] schedule_in: pp/p mismatch pp=%p pp->proc=%p p=%p",
	    pp, pp != NULL ? pp->pp_proc : NULL, p));

	hw_slots = pmu_count_core_hw_slots(pg, p);
	if (pg->pg_nevents > hw_slots)
		return (ENOSPC);

	error = pmu_assign_group(pg, p, PMC_CPU_ANY);
	if (error != 0) {
		PMCDBG2(PMC, OPS, 1,
		    "pp_schedule_in: assign gid=%u err=%d",
		    pg->pg_id, error);
		return (error);
	}

	/*
	 * Restore the per-pmc RUNNING state BEFORE we publish the row
	 * via attach_siblings.  pmu_pp_schedule_out stamped pm_state =
	 * STOPPED on every sibling so that pmc_process_csw_in (which
	 * checks pm_state == RUNNING before loading the HW counter,
	 * see hwpmc_mod.c csw_in path) would not race the drain phase
	 * of eviction and let new csw_in traffic increment pm_runcount
	 * while we were detaching.
	 *
	 * Doing the restore here -- after assign (rows are valid, pe
	 * is ACTIVE) but before attach (pp_pmcs[ri] is still NULL) --
	 * means by the time pmc_rotation_attach publishes the row via
	 * atomic_store_rel, pm_state is already RUNNING and any csw_in
	 * that observes the new row will load the counter immediately.
	 *
	 * Without this, a rotation-evicted-then-rebound group stayed at
	 * STOPPED forever and csw_in silently skipped the rebinding,
	 * which is the bug behind "Group 0 delta=0 between snapshots
	 * after a single eviction window" in pmc_mux_works_test.
	 */
	TAILQ_FOREACH(pe, &pg->pg_events, pe_sibling) {
		if (pe->pe_state == PMU_EVENT_STATE_ACTIVE)
			pe->pe_pmc->pm_state = PMC_STATE_RUNNING;
	}

	(void)pmu_group_attach_siblings(pg, pp);
	return (0);
}

/*
 * Atomically schedule pg out: detach every sibling from pp and
 * release every HW row in one pass.  Counts already accumulated in
 * pm->pm_gv.pm_savedvalue are preserved (we never zero them).
 * Caller holds pmc_sx exclusive.
 */
static void
pmu_pp_schedule_out(struct pmc_process *pp, struct pmu_group *pg)
{
	struct pmu_event *pe;

	if (pg == NULL || !pg->pg_assigned)
		return;

	/* Phase 1: stop every sibling so csw_in/csw_out skip them. */
	TAILQ_FOREACH(pe, &pg->pg_events, pe_sibling) {
		if (pe->pe_state != PMU_EVENT_STATE_ACTIVE)
			continue;
		pe->pe_pmc->pm_state = PMC_STATE_STOPPED;
	}

	/* Phase 2: drain any in-flight csw_out. */
	TAILQ_FOREACH(pe, &pg->pg_events, pe_sibling) {
		if (pe->pe_state != PMU_EVENT_STATE_ACTIVE)
			continue;
		pmc_rotation_drain(pe->pe_pmc);
	}

	/* Phase 3: detach every active sibling from the target. */
	TAILQ_FOREACH(pe, &pg->pg_events, pe_sibling) {
		struct pmc *pm = pe->pe_pmc;

		if (pe->pe_state != PMU_EVENT_STATE_ACTIVE)
			continue;
		if (pp->pp_pmcs[PMC_TO_ROWINDEX(pm)].pp_pmc == pm)
			pmc_rotation_detach(pm, pp);
	}

	/* Phase 4: release every HW row in one pass. */
	pmu_unassign_group(pg, 0);
}

/*
 * Purge every pmu_group hanging off pp->pp_pmu_groups and tear down the
 * per-pp rotation kthread.  Called from paths that are about to free pp
 * out from under the PMU layer (pmc_process_exit when the target proc
 * goes away, pmc_destroy_process_descriptor as a defensive belt-and-
 * suspenders), so that the eventual pmu_group_on_release for each
 * sibling sees pg_pp == NULL and skips its LIST_REMOVE/schedule_out
 * branch instead of dereferencing freed memory.
 *
 * The pmu_group structs themselves are NOT freed here -- they are owned
 * by the pmc_owner / pmu_event lifetime and will be reclaimed by
 * pmu_group_on_release / pmu_group_release in the normal way.  We only
 * sever the pp linkage.
 *
 * Caller must hold pmc_sx exclusive.
 */
void
pmu_pp_release_all(struct pmc_process *pp)
{
	struct pmu_group *pg, *next;

	hwpmc_pmu_sx_assert_xlocked();

	if (pp == NULL)
		return;

	/*
	 * Tear down the rotation kthread first.  It holds pp as its
	 * arg, so once pp is freed the next muxrot tick would
	 * dereference freed memory.  The sx_sleep below transiently
	 * drops pmc_sx; the kthread acquires it, observes
	 * pp_pmu_rot_running == false, clears pp_pmu_rot_td, and
	 * exits.
	 */
	if (pp->pp_pmu_rot_running) {
		pp->pp_pmu_rot_running = false;
		wakeup(pp);
		while (pp->pp_pmu_rot_td != NULL)
			(void)hwpmc_pmu_sx_sleep(&pp->pp_pmu_rot_td, 1,
			    "muxpurge");
	}

	/*
	 * Schedule out every group that is currently bound to pp, then
	 * unhook every pg from pp->pp_pmu_groups regardless of state.
	 * pg_pp gets cleared so a later pmu_group_on_release for any
	 * surviving sibling sees was_first_release == false and skips
	 * the now-stale LIST_REMOVE.
	 */
	pp->pp_pmu_rot_cursor = NULL;
	LIST_FOREACH_SAFE(pg, &pp->pp_pmu_groups, pg_proc_next, next) {
		if (pg->pg_assigned)
			pmu_pp_schedule_out(pp, pg);
		LIST_REMOVE(pg, pg_proc_next);
		pg->pg_pp = NULL;
		pg->pg_attach_proc = NULL;
		pg->pg_running = false;
	}
	LIST_INIT(&pp->pp_pmu_groups);
}

/*
 * Spawn the per-pp rotation kthread the first time multiple groups
 * are attached to one pp, OR if a deferred group is sitting on the
 * pp waiting to be brought in.  Idempotent: returns immediately once
 * the kthread is up.
 */
static void
pmu_pp_kick_rotate(struct pmc_process *pp)
{
	struct pmu_group *pg;
	int n_running, n_deferred;
	int error;

	if (pp == NULL || pp->pp_pmu_rot_running)
		return;

	n_running = n_deferred = 0;
	LIST_FOREACH(pg, &pp->pp_pmu_groups, pg_proc_next) {
		if (!pg->pg_running)
			continue;
		n_running++;
		if (!pg->pg_assigned)
			n_deferred++;
	}
	/*
	 * Only worth running rotation if at least one running group
	 * is currently deferred (i.e., something is waiting for HW)
	 * AND there is at least one peer that could be evicted.
	 */
	if (n_deferred == 0 || n_running < 2)
		return;

	pp->pp_pmu_rot_running = true;
	error = kthread_add(pmu_pp_rotate_thread, pp, NULL,
	    &pp->pp_pmu_rot_td, 0, 0, "pmu_rot_pp_%d",
	    pp->pp_proc != NULL ? pp->pp_proc->p_pid : -1);
	if (error != 0) {
		pp->pp_pmu_rot_running = false;
		pp->pp_pmu_rot_td = NULL;
		PMCDBG1(PMC, OPS, 1,
		    "pp_kick_rotate: kthread_add err=%d", error);
	}
}

/*
 * Run one rotation window for pp.
 *
 * Algorithm (simple, fair round-robin -- no FIFO list mutation):
 *
 *   Phase 1: schedule out EVERY currently scheduled+running group
 *            atomically.  After this every running group is deferred
 *            and the HW counter pool is empty.
 *
 *   Phase 2: walk pp_pmu_groups starting at pp_pmu_rot_cursor (or
 *            head if cursor is stale) and try to schedule each
 *            running group in.  As soon as one returns ENOSPC, that
 *            group becomes the cursor for the NEXT tick and we stop
 *            (don't try to fit smaller groups behind it -- that
 *            would re-introduce the starvation we are trying to
 *            avoid: a small high-rate group could slot in over and
 *            over while a larger group at the cursor never gets HW).
 *
 *            If we wrap around back to the cursor without ever
 *            hitting ENOSPC, every group fits simultaneously
 *            (oversubscription went away mid-test) and we just
 *            advance the cursor by one slot for the next tick so
 *            that future ticks rotate the starting position.
 *
 * Example: 3 running groups G1=3, G2=2, G3=3 events on a 6-counter
 * core pool.  Cursor starts at G1.
 *
 *   tick 0: cursor=G1.  Schedule G1 (3/6).  Schedule G2 (5/6).  Try
 *           G3: ENOSPC.  Next cursor=G3.  Live: {G1, G2}.
 *   tick 1: cursor=G3.  Schedule G3 (3/6).  Schedule G1 (6/6).  Try
 *           G2: ENOSPC.  Next cursor=G2.  Live: {G3, G1}.
 *   tick 2: cursor=G2.  Schedule G2 (2/6).  Schedule G3 (5/6).  Try
 *           G1: ENOSPC.  Next cursor=G1.  Live: {G2, G3}.
 *   tick 3: same as tick 0.
 *
 * Each group runs in 2 of every 3 ticks; HW utilisation is 2*total/3.
 *
 * Caller holds pmc_sx exclusive.
 */
static void
pmu_pp_rotate_one(struct pmc_process *pp)
{
	struct pmu_group *pg, *cursor;
	bool found_cursor;
	u_int ngroups, seen;
	int sin_err;

	if (pp == NULL || LIST_EMPTY(&pp->pp_pmu_groups))
		return;

	/*
	 * Pre-check: if no running group is currently deferred there
	 * is nothing to rotate.  Skip the evict-and-rebind cycle
	 * entirely so that the kthread, once awake, costs us nothing
	 * when oversubscription has subsided (e.g., the user stopped
	 * one of two groups).
	 */
	{
		bool need_rotate = false;
		LIST_FOREACH(pg, &pp->pp_pmu_groups, pg_proc_next) {
			if (pg->pg_running && !pg->pg_assigned) {
				need_rotate = true;
				break;
			}
		}
		if (!need_rotate)
			return;
	}

	/* Phase 1: evict every currently scheduled group. */
	LIST_FOREACH(pg, &pp->pp_pmu_groups, pg_proc_next) {
		if (pg->pg_assigned) {
			printf("hwpmc: rotate_one pp=%p evicting gid=%u "
			    "nevents=%u\n", pp, pg->pg_id, pg->pg_nevents);
			pmu_pp_schedule_out(pp, pg);
		}
	}

	/*
	 * Phase 2 setup: validate the cursor is still in pp_pmu_groups.
	 * A release path could have removed the cursor group between
	 * ticks; in that case (or first tick), reset to the head.
	 */
	ngroups = 0;
	cursor = pp->pp_pmu_rot_cursor;
	found_cursor = false;
	LIST_FOREACH(pg, &pp->pp_pmu_groups, pg_proc_next) {
		if (pg == cursor)
			found_cursor = true;
		ngroups++;
	}
	if (!found_cursor)
		cursor = LIST_FIRST(&pp->pp_pmu_groups);

	/*
	 * Phase 2: round-robin schedule_in starting at cursor, wrap
	 * once.  First ENOSPC pins the next tick's cursor and stops
	 * the walk.
	 */
	pg = cursor;
	seen = 0;
	while (seen < ngroups) {
		if (pg == NULL)
			pg = LIST_FIRST(&pp->pp_pmu_groups);

		if (pg->pg_running && !pg->pg_assigned) {
			sin_err = pmu_pp_schedule_in(pp, pg);
			printf("hwpmc: rotate_one pp=%p schedule_in gid=%u "
			    "nevents=%u -> %d\n",
			    pp, pg->pg_id, pg->pg_nevents, sin_err);
			if (sin_err == ENOSPC) {
				pp->pp_pmu_rot_cursor = pg;
				return;
			}
			/* sin_err == 0 (success) or hard fail; advance. */
		}

		seen++;
		pg = LIST_NEXT(pg, pg_proc_next);
	}

	/*
	 * Everything fit.  Advance cursor by one for the next tick so
	 * we rotate the start position even when there is no
	 * oversubscription this window.
	 */
	pp->pp_pmu_rot_cursor = LIST_NEXT(cursor, pg_proc_next);
	if (pp->pp_pmu_rot_cursor == NULL)
		pp->pp_pmu_rot_cursor = LIST_FIRST(&pp->pp_pmu_groups);
}

/*
 * Per-pp rotation kthread.  Wakes every kern.hwpmc.mux_period_ms,
 * runs one pmu_pp_rotate_one tick, and goes back to sleep.  Exits
 * when the last group is released from the pp (pmu_group_on_release
 * clears pp_pmu_rot_running and wakes us).
 */
static void
pmu_pp_rotate_thread(void *arg)
{
	struct pmc_process *pp = arg;
	int period_ticks;

	hwpmc_pmu_sx_xlock();
	while (pp->pp_pmu_rot_running) {
		period_ticks = (pmu_mux_period_ms * hz) / 1000;
		if (period_ticks < 1)
			period_ticks = 1;
		(void)hwpmc_pmu_sx_sleep(pp, period_ticks, "muxrot");
		if (!pp->pp_pmu_rot_running)
			break;
		pmu_pp_rotate_one(pp);
	}
	pp->pp_pmu_rot_td = NULL;
	wakeup(&pp->pp_pmu_rot_td);
	hwpmc_pmu_sx_xunlock();
	kthread_exit();
}

