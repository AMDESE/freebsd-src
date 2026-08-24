/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Advanced Micro Devices, Inc.
 *
 * PMU event grouping, time accounting, and multiplex rotation.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/kthread.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/mutex.h>
#include <sys/pcpu.h>
#include <sys/pmc.h>
#include <sys/pmckern.h>
#include <sys/pmclog.h>
#include <sys/prng.h>
#include <sys/proc.h>
#include <sys/sysctl.h>
#include <sys/sx.h>
#include <sys/taskqueue.h>
#include <sys/unistd.h>

#include "hwpmc_pmu.h"

static MALLOC_DEFINE(M_PMU, "pmu", "hwpmc PMU grouping");

static uint32_t pmu_next_group_id = 1;

extern struct mtx_pool *pmc_mtxpool;

LIST_HEAD(pmu_group_cpu_list, pmu_group_cpu_state);
static struct pmu_group_cpu_list pmu_group_cpu_active[MAXCPU];
static LIST_HEAD(, pmu_thread_residual) pmu_residual_freelist;
static struct mtx pmu_residual_freelist_mtx;
static struct task pmu_residual_free_task;

SYSCTL_DECL(_kern_hwpmc);

/*
 * Multiplex rotation period in milliseconds.
 */
static int pmu_mux_period_ms = 50;
SYSCTL_INT(_kern_hwpmc, OID_AUTO, mux_period_ms, CTLFLAG_RWTUN,
    &pmu_mux_period_ms, 0,
    "PMU multiplex rotation period floor in milliseconds");

/* Rotation window in ticks with random jitter. */
static int
pmu_rot_period_ticks(void)
{
	int period_ticks;

	period_ticks = (pmu_mux_period_ms * hz) / 1000;
	if (period_ticks < 1)
		period_ticks = 1;
	return (period_ticks + prng32_bounded(period_ticks / 4 + 1));
}

static void pmu_pp_rotate_thread(void *arg);
static int pmu_pp_schedule_in(struct pmc_process *pp, pmu_group_t *pg);
static void pmu_pp_schedule_out(struct pmc_process *pp, pmu_group_t *pg,
    bool drain_samples);
static void pmu_pp_kick_rotate(struct pmc_process *pp);
static void pmu_pp_unlink_group(pmu_group_t *pg, struct pmc_process *pp);
static void pmu_group_attach_siblings(pmu_group_t *pg,
    struct pmc_process *pp);
static void pmu_event_remove_target_residuals(pmu_event_t *pe,
    struct pmc_process *pp);
static int pmu_event_set_thread_residual_impl(struct pmc *pm,
    struct pmc_process *pp, lwpid_t tid, pmc_value_t value, uint8_t state);
static struct pmu_thread_residual *pmu_thread_residual_find_locked(
    struct pmc_process *pp, pmu_event_t *pe, lwpid_t tid);
static bool pmu_thread_exists_locked(struct pmc_process *pp, lwpid_t tid);
static void pmu_thread_residual_defer_free(struct pmu_thread_residual *ptr);
static void pmu_thread_residual_free_task_fn(void *arg, int pending);

/*
 * State for system-mode (PMC_MODE_SC) multiplexing.
 */
struct pmu_syscpu {
	struct pmu_group_list	sc_groups;	/* groups bound to this CPU */
	pmu_group_t		*sc_cursor;	/* rotation cursor */
	struct thread		*sc_td;		/* rotation thread */
	u_int			sc_quiesce;
	bool			sc_running;	/* rotation thread is active */
	bool			sc_needed;	/* deferred group is waiting */
};
static struct pmu_syscpu pmu_syscpu[MAXCPU];

static void pmu_syscpu_rotate_thread(void *arg);
static int pmu_sys_schedule_in(int cpu, pmu_group_t *pg);
static void pmu_sys_schedule_out(int cpu, pmu_group_t *pg);
static void pmu_sys_stop_rows(int cpu, pmu_group_t *pg, u_int nmembers);
static void pmu_syscpu_kick_rotate(int cpu);
static void pmu_syscpu_rotate_one(int cpu);

void
pmu_group_accounting_initialize(void)
{
	u_int cpu;

	for (cpu = 0; cpu < MAXCPU; cpu++)
		LIST_INIT(&pmu_group_cpu_active[cpu]);
	LIST_INIT(&pmu_residual_freelist);
	mtx_init(&pmu_residual_freelist_mtx, "pmu-residuals", "pmc-leaf",
	    MTX_SPIN);
	TASK_INIT(&pmu_residual_free_task, 0, pmu_thread_residual_free_task_fn,
	    NULL);
}

void
pmu_group_accounting_finalize(void)
{
	u_int cpu;

	for (cpu = 0; cpu < MAXCPU; cpu++)
		KASSERT(LIST_EMPTY(&pmu_group_cpu_active[cpu]),
		    ("[pmu] active accounting markers on CPU %u", cpu));
	taskqueue_drain(taskqueue_fast, &pmu_residual_free_task);
	pmu_thread_residual_free_task_fn(NULL, 0);
	KASSERT(LIST_EMPTY(&pmu_residual_freelist),
	    ("[pmu] deferred residual frees remain"));
	mtx_destroy(&pmu_residual_freelist_mtx);
}

static void
pmu_group_time_update_locked(pmu_group_t *pg, uint64_t now)
{
	uint64_t delta;

	if (pg->pg_timestamp_ticks == 0) {
		pg->pg_timestamp_ticks = now;
		return;
	}
	if (now <= pg->pg_timestamp_ticks)
		return;

	delta = now - pg->pg_timestamp_ticks;
	if (pg->pg_running) {
		if (pg->pg_system) {
			if (pg->pg_cpu >= 0 && pmc_cpu_is_active(pg->pg_cpu)) {
				pg->pg_time_enabled_ticks += delta;
				pg->pg_enabled_wall_ticks += delta;
				if (pg->pg_account_placement_admit)
					pg->pg_time_running_ticks += delta;
			}
		} else {
			pg->pg_time_enabled_ticks +=
			    (uint64_t)pg->pg_oncpu_threads * delta;
			pg->pg_time_running_ticks +=
			    (uint64_t)pg->pg_running_threads * delta;
			if (pg->pg_oncpu_threads != 0)
				pg->pg_enabled_wall_ticks += delta;
		}
	}
	pg->pg_timestamp_ticks = now;
}

static void
pmu_group_running_start_locked(pmu_group_t *pg, uint64_t now)
{

	pmu_group_time_update_locked(pg, now);
	if (!pg->pg_running) {
		pg->pg_running = true;
		pg->pg_wall_start_ticks = now;
	}
}

static void
pmu_group_running_stop_locked(pmu_group_t *pg, uint64_t now)
{

	pmu_group_time_update_locked(pg, now);
	if (!pg->pg_running)
		return;
	if (now > pg->pg_wall_start_ticks)
		pg->pg_wall_ticks += now - pg->pg_wall_start_ticks;
	pg->pg_wall_start_ticks = 0;
	pg->pg_running = false;
}

static uint64_t
pmu_group_ticks_to_ns(uint64_t ticks, uint64_t tickrate)
{
	uint64_t frac, quot, rem, secs;

	if (tickrate == 0)
		return (0);

	/*
	 * Scale ticks to nanoseconds without 128-bit division:
	 * (ticks / rate) * 1e9 + (ticks % rate) * 1e9 / rate.
	 */
	quot = ticks / tickrate;
	rem = ticks % tickrate;

	if (quot > UINT64_MAX / 1000000000)
		return (UINT64_MAX);
	secs = quot * 1000000000;

	/* Prevent overflow for high tick rates. */
	if (rem <= UINT64_MAX / 1000000000)
		frac = rem * 1000000000 / tickrate;
	else
		frac = rem / (tickrate / 1000000000);

	if (secs > UINT64_MAX - frac)
		return (UINT64_MAX);
	return (secs + frac);
}

void
pmu_group_time_snapshot_locked(pmu_group_t *pg,
    struct pmu_group_time_snapshot *snapshot, uint64_t now)
{
	uint64_t enabled, enabled_wall, running, tickrate, wall;

	KASSERT(pg != NULL && snapshot != NULL,
	    ("[pmu] invalid group time snapshot"));
	pmu_group_time_update_locked(pg, now);
	enabled = pg->pg_time_enabled_ticks;
	running = MIN(pg->pg_time_running_ticks, enabled);
	enabled_wall = pg->pg_enabled_wall_ticks;
	wall = pg->pg_wall_ticks;
	if (pg->pg_running && now > pg->pg_wall_start_ticks)
		wall += now - pg->pg_wall_start_ticks;
	tickrate = pg->pg_tickrate;
	snapshot->pgts_system = pg->pg_system;

	snapshot->pgts_enabled = pmu_group_ticks_to_ns(enabled, tickrate);
	snapshot->pgts_running = pmu_group_ticks_to_ns(running, tickrate);
	snapshot->pgts_enabled_wall = pmu_group_ticks_to_ns(enabled_wall,
	    tickrate);
	snapshot->pgts_wall = pmu_group_ticks_to_ns(wall, tickrate);
	if (snapshot->pgts_running > snapshot->pgts_enabled)
		snapshot->pgts_running = snapshot->pgts_enabled;
}

static void
pmu_group_accounting_block_locked(pmu_group_t *pg, uint64_t now)
{

	pmu_group_time_update_locked(pg, now);
	pg->pg_account_blocked = true;
	pg->pg_account_placement_admit = false;
}

static void
pmu_group_accounting_block(pmu_group_t *pg)
{

	mtx_pool_lock_spin(pmc_mtxpool, pg);
	pmu_group_accounting_block_locked(pg, cpu_ticks());
	mtx_pool_unlock_spin(pmc_mtxpool, pg);
}

static void
pmu_group_accounting_drain(pmu_group_t *pg, bool release)
{
	struct pmc_binding pb;
	u_int cpu;
#ifdef INVARIANTS
	int maxloop;

	maxloop = 100 * MAX(1, pg->pg_ncpu);
#endif

	sx_assert(&pmc_sx, SX_XLOCKED);
	KASSERT(pg->pg_account_blocked,
	    ("[pmu] draining unblocked group %u", pg->pg_id));

	if (pg->pg_ncpu != 0)
		pmc_save_cpu_binding(&pb);
	for (;;) {
		mtx_pool_lock_spin(pmc_mtxpool, pg);
		for (cpu = 0; cpu < pg->pg_ncpu; cpu++) {
			if (pg->pg_cpu_state[cpu].pgcs_counted)
				break;
		}
		if (cpu == pg->pg_ncpu) {
			KASSERT(pg->pg_oncpu_threads == 0,
			    ("[pmu] group %u has count without marker",
			    pg->pg_id));
			mtx_pool_unlock_spin(pmc_mtxpool, pg);
			break;
		}
		mtx_pool_unlock_spin(pmc_mtxpool, pg);
#ifdef INVARIANTS
		KASSERT(maxloop-- > 0,
		    ("[pmu] group %u accounting drain stuck", pg->pg_id));
#endif
		KASSERT(pmc_cpu_is_active(cpu),
		    ("[pmu] group %u marker on inactive CPU %u", pg->pg_id,
		    cpu));
		pmc_select_cpu(cpu);
		hwpmc_pmu_force_context_switch();
	}
	if (pg->pg_ncpu != 0)
		pmc_restore_cpu_binding(&pb);

	mtx_pool_lock_spin(pmc_mtxpool, pg);
	pmu_group_running_stop_locked(pg, cpu_ticks());
	if (!release)
		pg->pg_account_blocked = false;
	KASSERT(pg->pg_running_threads == 0,
	    ("[pmu] group %u has running accounting threads", pg->pg_id));
	for (cpu = 0; cpu < pg->pg_ncpu; cpu++) {
		KASSERT(!pg->pg_cpu_state[cpu].pgcs_counted &&
		    !pg->pg_cpu_state[cpu].pgcs_placed &&
		    !pg->pg_cpu_state[cpu].pgcs_transitioning &&
		    pg->pg_cpu_state[cpu].pgcs_td == NULL,
		    ("[pmu] group %u has active CPU %u marker", pg->pg_id,
		    cpu));
	}
	mtx_pool_unlock_spin(pmc_mtxpool, pg);
}

static void
pmu_group_kick_placement(pmu_group_t *pg)
{
	struct pmc_binding pb;
	bool kick;
	u_int cpu;

	KASSERT(!pg->pg_system && pg->pg_ncpu != 0,
	    ("[pmu] kick_placement: bad group %u", pg->pg_id));
	pmc_save_cpu_binding(&pb);
	for (cpu = 0; cpu < pg->pg_ncpu; cpu++) {
		mtx_pool_lock_spin(pmc_mtxpool, pg);
		kick = pg->pg_cpu_state[cpu].pgcs_counted;
		mtx_pool_unlock_spin(pmc_mtxpool, pg);
		if (!kick || !pmc_cpu_is_active(cpu))
			continue;
		pmc_select_cpu(cpu);
		hwpmc_pmu_force_context_switch();
	}
	pmc_restore_cpu_binding(&pb);
}

static void
pmu_group_accounting_prepare_release(pmu_group_t *pg)
{
	struct pmc_process *pp;

	pp = pg->pg_pp;
	if (pp != NULL)
		mtx_lock_spin(&pp->pp_pmu_lock);
	pmu_group_accounting_block(pg);
	if (pp != NULL)
		mtx_unlock_spin(&pp->pp_pmu_lock);
	pmu_group_accounting_drain(pg, true);
}

static void
pmu_pp_stop_rotate(struct pmc_process *pp, const char *wmesg)
{

	mtx_lock_spin(&pp->pp_pmu_lock);
	pp->pp_pmu_rot_quiesce++;
	pp->pp_pmu_rot_running = false;
	pp->pp_pmu_rot_needed = false;
	mtx_unlock_spin(&pp->pp_pmu_lock);
	wakeup(&pp->pp_pmu_rot_needed);
	while (pp->pp_pmu_rot_td != NULL)
		(void)sx_sleep(&pp->pp_pmu_rot_td, &pmc_sx, 0, wmesg, 1);
	mtx_lock_spin(&pp->pp_pmu_lock);
	KASSERT(pp->pp_pmu_rot_quiesce > 0,
	    ("[pmu] rotation quiesce underflow"));
	pp->pp_pmu_rot_quiesce--;
	mtx_unlock_spin(&pp->pp_pmu_lock);
}

/* Add group to process list if not present. */
static void
pmu_pp_link_group(pmu_group_t *pg, struct pmc_process *pp)
{

	if (pg->pg_pp != NULL)
		return;
	mtx_lock_spin(&pp->pp_pmu_lock);
	LIST_INSERT_HEAD(&pp->pp_pmu_groups, pg, pg_proc_next);
	pg->pg_pp = pp;
	mtx_unlock_spin(&pp->pp_pmu_lock);
}

/*
 * Does this group follow its targets through fork?  The flag is
 * leader-only and governs the whole group, and commit has already
 * rejected it on a system group.
 */
bool
pmu_group_follows_fork(pmu_group_t *pg)
{

	return (pg->pg_committed && !pg->pg_releasing && !pg->pg_system &&
	    pg->pg_leader != NULL &&
	    (pg->pg_leader->pe_alloc.pm_flags & PMC_F_DESCENDANTS) != 0);
}

/* Find the authoritative edge for pp, if this group already follows it. */
static struct pmu_group_target *
pmu_group_find_target(pmu_group_t *pg, struct pmc_process *pp)
{
	struct pmu_group_target *pgt;

	LIST_FOREACH(pgt, &pg->pg_targets, pgt_group_next) {
		if (pgt->pgt_pp == pp)
			return (pgt);
	}
	return (NULL);
}

/*
 * This function selects a surviving target for a scheduling-anchor handoff.
 * Prefer the explicit edge if it exists.  If not, use any inherited edge.
 * You must hold pmc_sx exclusive to call this function.
 */
static struct pmu_group_target *
pmu_group_replacement_target(pmu_group_t *pg, struct pmc_process *old_pp)
{
	struct pmu_group_target *pgt, *replacement;

	sx_assert(&pmc_sx, SX_XLOCKED);
	replacement = NULL;
	LIST_FOREACH(pgt, &pg->pg_targets, pgt_group_next) {
		if (pgt->pgt_pp == old_pp)
			continue;
		if (pgt->pgt_explicit)
			return (pgt);
		if (replacement == NULL)
			replacement = pgt;
	}
	return (replacement);
}

static bool
pmu_group_has_target(pmu_group_t *pg, struct pmc_process *pp)
{

	return (pmu_group_find_target(pg, pp) != NULL);
}

/*
 * This function allocates a target edge, but does not publish it.
 * Publication is a separate step.  This lets a multi-process attach finish
 * every fallible check before any edge becomes visible.
 */
struct pmu_group_target *
pmu_group_target_alloc(pmu_group_t *pg, struct pmc_process *pp, bool explicit,
    int malloc_flags)
{
	struct pmu_group_target *pgt;

	sx_assert(&pmc_sx, SX_XLOCKED);
	KASSERT(pg != NULL && pp != NULL,
	    ("[pmu] invalid target allocation pg=%p pp=%p", pg, pp));
	if (pmc_test_should_fail(PMC_TEST_FAIL_GROUP_TARGET_ALLOC))
		return (NULL);
	pgt = malloc(sizeof(*pgt), M_PMU, malloc_flags | M_ZERO);
	if (pgt == NULL)
		return (NULL);
	pmc_test_counter_add(PMC_TEST_LIVE_GROUP_TARGETS, 1);
	pgt->pgt_pp = pp;
	pgt->pgt_group = pg;
	pgt->pgt_explicit = explicit;
	return (pgt);
}

void
pmu_group_target_publish(struct pmu_group_target *pgt)
{
	pmu_group_t *pg;
	struct pmc_process *pp;

	sx_assert(&pmc_sx, SX_XLOCKED);
	KASSERT(pgt != NULL && pgt->pgt_group != NULL && pgt->pgt_pp != NULL,
	    ("[pmu] invalid target publication pgt=%p", pgt));
	pg = pgt->pgt_group;
	pp = pgt->pgt_pp;
	KASSERT(!pmu_group_has_target(pg, pp),
	    ("[pmu] duplicate group %u target pp=%p", pg->pg_id, pp));

	LIST_INSERT_HEAD(&pg->pg_targets, pgt, pgt_group_next);
	mtx_lock_spin(&pp->pp_pmu_lock);
	LIST_INSERT_HEAD(&pp->pp_pmu_targets, pgt, pgt_process_next);
	mtx_unlock_spin(&pp->pp_pmu_lock);
	if (pgt->pgt_explicit) {
		KASSERT(pg->pg_explicit_target == NULL && pg->pg_pp == NULL,
		    ("[pmu] group %u already has an explicit target", pg->pg_id));
		pg->pg_explicit_target = pgt;
		pmu_pp_link_group(pg, pp);
	}
}

void
pmu_group_target_discard(struct pmu_group_target *pgt)
{

	if (pgt == NULL)
		return;
	pmc_test_counter_add(PMC_TEST_LIVE_GROUP_TARGETS, -1);
	free(pgt, M_PMU);
}

void
pmu_group_target_remove(struct pmu_group_target *pgt)
{
	pmu_event_t *pe;
	pmu_group_t *pg;
	struct pmc_process *pp;

	sx_assert(&pmc_sx, SX_XLOCKED);
	KASSERT(pgt != NULL && pgt->pgt_group != NULL && pgt->pgt_pp != NULL,
	    ("[pmu] invalid target removal pgt=%p", pgt));
	pg = pgt->pgt_group;
	pp = pgt->pgt_pp;
	TAILQ_FOREACH(pe, &pg->pg_events, pe_sibling)
		pmu_event_remove_target_residuals(pe, pp);
	if (pg->pg_pp == pp)
		pmu_pp_unlink_group(pg, pp);
	if (pg->pg_explicit_target == pgt)
		pg->pg_explicit_target = NULL;
	mtx_lock_spin(&pp->pp_pmu_lock);
	LIST_REMOVE(pgt, pgt_process_next);
	mtx_unlock_spin(&pp->pp_pmu_lock);
	LIST_REMOVE(pgt, pgt_group_next);
	pmu_group_target_discard(pgt);
}

/*
 * This function records a target reached through fork.
 * The allocation must not sleep.  A fork must not fail.
 * If the allocation fails, the child is not monitored for this group.
 * The caller then records the miss.
 */
static bool
pmu_group_add_inherited(pmu_group_t *pg, struct pmc_process *pp)
{
	pmu_event_t *pe;
	struct pmc_target *links[PMC_GROUP_MAX_MEMBERS];
	struct pmu_group_target *pgt;
	struct pmc *pm;
	u_int i, nlinks;
	int ri;

	if (pmu_group_has_target(pg, pp))
		return (true);
	KASSERT(pg->pg_nevents <= PMC_GROUP_MAX_MEMBERS,
	    ("[pmu] group %u has too many members", pg->pg_id));
	memset(links, 0, sizeof(links));
	nlinks = 0;
	pgt = pmu_group_target_alloc(pg, pp, false, M_NOWAIT);
	if (pgt == NULL)
		return (false);

	if (pg->pg_assigned) {
		TAILQ_FOREACH(pe, &pg->pg_events, pe_sibling) {
			pm = pe->pe_pmc;
			KASSERT(!PMC_ROW_IS_UNASSIGNED(pm),
			    ("[pmu] assigned group %u has unassigned member",
			    pg->pg_id));
			ri = PMC_TO_ROWINDEX(pm);
			if (pp->pp_pmcs[ri].pp_pmc != NULL)
				goto fail;
			links[nlinks] =
			    pmc_target_object_alloc(M_NOWAIT, true);
			if (links[nlinks] == NULL)
				goto fail;
			nlinks++;
		}

		i = 0;
		TAILQ_FOREACH(pe, &pg->pg_events, pe_sibling) {
			pmc_link_target_process_preallocated(pe->pe_pmc, pp,
			    links[i]);
			links[i++] = NULL;
		}
	}

	/*
	 * Linearization point: every fallible allocation and every current row
	 * link is complete before the authoritative target becomes visible.
	 */
	pmu_group_target_publish(pgt);
	return (true);

fail:
	for (i = 0; i < nlinks; i++)
		pmc_target_object_free(links[i]);
	pmu_group_target_discard(pgt);
	return (false);
}

/*
 * Attach every group the parent is a target of to the child (spec §3.9).
 * A group is taken all at once or not at all, so that §2.4 holds in the
 * child; a group that cannot be recorded is skipped whole and counted, and
 * the caller reports how many were missed.
 *
 * Only groups whose leader carries PMC_F_DESCENDANTS follow a fork, and
 * only virtual ones: a system group is bound to a CPU and has no process
 * target, which commit already rejects.
 */
u_int
pmu_group_inherit(struct pmc_process *ppold, struct pmc_process *ppnew,
    struct pmc **missed_leaders, u_int missed_capacity, u_int *nmissed)
{
	pmu_group_t *pg;
	struct pmu_group_target *pgt;
	u_int ninherited, nmiss;

	sx_assert(&pmc_sx, SX_XLOCKED);
	ninherited = 0;
	nmiss = 0;

	/* Direct parents and inherited parents share the same target list. */
	LIST_FOREACH(pgt, &ppold->pp_pmu_targets, pgt_process_next) {
		pg = pgt->pgt_group;
		if (!pmu_group_follows_fork(pg))
			continue;
		if (pmu_group_add_inherited(pg, ppnew)) {
			ninherited++;
			continue;
		}
		/* Record the missed group by its leader handle, for per-group logging. */
		if (missed_leaders != NULL && nmiss < missed_capacity &&
		    pg->pg_leader != NULL)
			missed_leaders[nmiss] = pg->pg_leader->pe_pmc;
		nmiss++;
	}

	if (nmissed != NULL)
		*nmissed = nmiss;
	return (ninherited);
}

/* Remove every group-target edge held by pp.  This includes explicit and inherited edges. */
void
pmu_group_disinherit(struct pmc_process *pp)
{
	struct pmu_group_target *pgt;

	sx_assert(&pmc_sx, SX_XLOCKED);
	while ((pgt = LIST_FIRST(&pp->pp_pmu_targets)) != NULL)
		pmu_group_target_remove(pgt);
}

/* Remove group from process list and update cursor. */
static void
pmu_pp_unlink_group(pmu_group_t *pg, struct pmc_process *pp)
{

	mtx_lock_spin(&pp->pp_pmu_lock);
	if (pp->pp_pmu_rot_cursor == pg)
		pp->pp_pmu_rot_cursor = LIST_NEXT(pg, pg_proc_next);
	LIST_REMOVE(pg, pg_proc_next);
	pg->pg_pp = NULL;
	mtx_unlock_spin(&pp->pp_pmu_lock);
}

/* Assign deferred groups and restart rotation. */
static void
pmu_pp_backfill(struct pmc_process *pp)
{
	pmu_group_t *pg;

	if (pp->pp_pmu_unhashed)
		return;
	LIST_FOREACH(pg, &pp->pp_pmu_groups, pg_proc_next) {
		if (pg->pg_running && !pg->pg_assigned)
			(void)pmu_pp_schedule_in(pp, pg);
	}
	pmu_pp_kick_rotate(pp);
}

pmu_event_t *
pmu_event_from_pmc(struct pmc *pm)
{
	return (pm != NULL ? pm->pm_pmu : NULL);
}

/* Return group for PMC or NULL. */
pmu_group_t *
pmu_group_from_pmc(struct pmc *pm)
{
	pmu_event_t *pe;

	pe = pmu_event_from_pmc(pm);
	return (pe != NULL ? pe->pe_group : NULL);
}

void
pmu_event_destroy(pmu_event_t *pe)
{
	if (pe == NULL)
		return;
	counter_u64_free(pe->pe_samples);
#ifdef HWPMC_DEBUG
	counter_u64_free(pe->pe_samples_emitted);
	counter_u64_free(pe->pe_samples_dropped);
#endif
	free(pe, M_PMU);
}

void
pmu_group_create(struct pmc_owner *po, uint32_t *pg_id)
{
	pmu_group_t *pg;

	KASSERT(po != NULL && pg_id != NULL,
	    ("[pmu] pmu_group_create: null po=%p pg_id=%p", po, pg_id));

	pg = malloc(sizeof(*pg), M_PMU, M_WAITOK | M_ZERO);
	mtx_init(&pg->pg_snapshot_lock, "pmu group snapshot", NULL, MTX_SPIN);
	pg->pg_id = pmu_next_group_id++;
	pg->pg_owner = po;
	TAILQ_INIT(&pg->pg_events);
	LIST_INIT(&pg->pg_targets);
	LIST_INSERT_HEAD(&po->po_groups, pg, pg_owner_next);
	*pg_id = pg->pg_id;
}

pmu_group_t *
pmu_group_lookup(struct pmc_owner *po, uint32_t pg_id)
{
	pmu_group_t *pg;

	LIST_FOREACH(pg, &po->po_groups, pg_owner_next) {
		if (pg->pg_id == pg_id && !pg->pg_releasing)
			return (pg);
	}
	return (NULL);
}

static void
pmu_group_accounting_init(pmu_group_t *pg)
{
	u_int cpu;

	if (pg->pg_tickrate != 0)
		return;
	pg->pg_tickrate = cpu_tickrate();
	KASSERT(pg->pg_tickrate != 0, ("[pmu] zero CPU tick rate"));
	if (PMC_IS_SYSTEM_MODE(pg->pg_leader->pe_alloc.pm_mode))
		return;
	pg->pg_ncpu = pmc_cpu_max();
	pg->pg_cpu_state = malloc(sizeof(*pg->pg_cpu_state) * pg->pg_ncpu,
	    M_PMU, M_WAITOK | M_ZERO);
	for (cpu = 0; cpu < pg->pg_ncpu; cpu++)
		pg->pg_cpu_state[cpu].pgcs_group = pg;
}

int
pmu_group_add(pmu_group_t *pg, struct pmc *pm, bool leader)
{
	pmu_event_t *pe;

	KASSERT(pg != NULL && pm != NULL,
	    ("[pmu] pmu_group_add: null pg=%p pm=%p", pg, pm));

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
		return (leader ? EINVAL : 0);
	if (pe->pe_group != NULL) {
		PMCDBG3(PMC, OPS, 1,
		    "group_add: pe %p in gid=%u, requested gid=%u",
		    pe, pe->pe_group->pg_id, pg->pg_id);
		return (EBUSY);
	}
	if (pg->pg_nevents >= PMC_GROUP_MAX_MEMBERS)
		return (E2BIG);
	if (leader && pg->pg_leader != NULL)
		return (EINVAL);

	pe->pe_group = pg;
	pe->pe_is_leader = leader;
	TAILQ_INSERT_TAIL(&pg->pg_events, pe, pe_sibling);
	pg->pg_nevents++;

	if (leader)
		pg->pg_leader = pe;

	return (0);
}

int
pmu_group_commit(pmu_group_t *pg)
{
	pmu_event_t *pe;
	struct pmc_mdep *mdep;
	struct pmc_owner *po;
	struct proc *p;
	int error;

	if (pg == NULL)
		return (EINVAL);
	if (pg->pg_committed)
		return (EBUSY);
	mdep = hwpmc_get_mdep();
	if (mdep == NULL)
		return (EOPNOTSUPP);
	if (pg->pg_nevents > (u_int)mdep->pmd_npmc)
		return (ENOSPC);

	error = pmu_validate_group(pg);
	if (error != 0) {
		PMCDBG3(PMC, OPS, 1,
		    "group_commit: validate gid=%u err=%d nevents=%u",
		    pg->pg_id, error, pg->pg_nevents);
		return (error);
	}

	/* Cache event constraints for sorting. */
	TAILQ_FOREACH(pe, &pg->pg_events, pe_sibling) {
		error = pmu_event_get_constraint(pe, &pe->pe_cons);
		if (error != 0) {
			PMCDBG2(PMC, OPS, 1,
			    "group_commit: constraint err=%d class=%d",
			    error, (int)pe->pe_alloc.pm_class);
			return (error);
		}
	}

	pg->pg_defer_ok = (pg->pg_leader->pe_alloc.pm_flags &
	    PMC_F_GROUP_MUX) != 0;

	error = pmu_group_can_fit(pg);
	if (error != 0)
		return (error);
	pmu_group_accounting_init(pg);

	po = pg->pg_owner;
	p = po != NULL ? po->po_owner : NULL;
	if (p == NULL)
		return (EINVAL);

	/*
	 * System groups bind to one CPU.
	 * Hardware assignment occurs at group start.
	 */
	if (PMC_IS_SYSTEM_MODE(pg->pg_leader->pe_alloc.pm_mode)) {
		pg->pg_system = true;
		pg->pg_cpu = pg->pg_leader->pe_alloc.pm_cpu;

		/* Commit succeeds if the group can fit on the PMU. */
		error = pmu_group_can_place(pg, p, pg->pg_cpu);
		PMCDBG5(PMC, OPS, 1,
		    "group_commit: SYS gid=%u cpu=%d nevents=%u "
		    "place=%d defer_ok=%d", pg->pg_id, pg->pg_cpu,
		    pg->pg_nevents, error, (int)pg->pg_defer_ok);
		if (error != 0 && error != ENOSPC)
			return (error);

		pg->pg_committed = true;
		return (0);
	}

	/*
	 * Assign hardware rows immediately if available.
	 * If rows are busy and PMC_F_GROUP_MUX is set, defer assignment.
	 */
	error = pmu_group_can_place(pg, p, PMC_CPU_ANY);
	if (error == 0)
		error = pmu_assign_group(pg, p, PMC_CPU_ANY);
	if (error == 0) {
		PMCDBG1(PMC, OPS, 1,
		    "group_commit: gid=%u BOUND", pg->pg_id);
	} else if (error != EBUSY && error != ENOSPC) {
		return (error);
	} else if (!pg->pg_defer_ok) {
		return (ENOSPC);
	} else {
		PMCDBG1(PMC, OPS, 1,
		    "group_commit: gid=%u DEFERRED", pg->pg_id);
	}

	pg->pg_committed = true;
	return (0);
}

u_int
pmu_group_prepare_release(pmu_group_t *pg, struct pmc **members,
    u_int capacity, struct pmc_process **released_pp)
{
	pmu_event_t *pe, *tmp;
	struct pmu_group_target *pgt;
	struct pmc_process *held_pp, *pp;
	bool pp_unhashed;
	u_int n;

	sx_assert(&pmc_sx, SX_XLOCKED);
	KASSERT(pg != NULL && members != NULL,
	    ("[pmu] invalid group release preparation"));
	KASSERT(capacity >= pg->pg_nevents,
	    ("[pmu] group release capacity %u < %u", capacity,
	    pg->pg_nevents));
	KASSERT(!pg->pg_releasing,
	    ("[pmu] group %u release already active", pg->pg_id));

	pg->pg_releasing = true;
	if (released_pp != NULL)
		*released_pp = NULL;

	if (pg->pg_system) {
		if (pg->pg_leader != NULL)
			pmu_sys_group_pre_release(pg->pg_leader->pe_pmc);
		if (pg->pg_committed && !pg->pg_account_blocked)
			pmu_group_accounting_prepare_release(pg);
	} else {
		if (pg->pg_committed && !pg->pg_account_blocked)
			pmu_group_accounting_prepare_release(pg);
		held_pp = pg->pg_pp;
		if (held_pp != NULL) {
			mtx_lock_spin(&held_pp->pp_pmu_lock);
			held_pp->pp_pmu_refs++;
			pmc_test_counter_add(PMC_TEST_LIVE_ROTATION_REFS, 1);
			mtx_unlock_spin(&held_pp->pp_pmu_lock);
			pmu_pp_stop_rotate(held_pp, "muxrel");
		}

		pp = pg->pg_pp;
		if (pg->pg_assigned) {
			if (pp != NULL)
				pmu_pp_schedule_out(pp, pg, false);
			else
				pmu_unassign_group(pg, 0);
		}
		if (pg->pg_pp != NULL) {
			pp = pg->pg_pp;
			pmu_pp_unlink_group(pg, pp);
			pmu_pp_backfill(pp);
		}
		if (held_pp != NULL) {
			mtx_lock_spin(&held_pp->pp_pmu_lock);
			KASSERT(held_pp->pp_pmu_refs > 0,
			    ("[pmu] process reference underflow"));
			held_pp->pp_pmu_refs--;
			pmc_test_counter_add(PMC_TEST_LIVE_ROTATION_REFS, -1);
			pp_unhashed = held_pp->pp_pmu_unhashed;
			mtx_unlock_spin(&held_pp->pp_pmu_lock);
			wakeup(&held_pp->pp_pmu_refs);
			if (released_pp != NULL && !pp_unhashed && pp == held_pp)
				*released_pp = held_pp;
		}
	}

	KASSERT(!pg->pg_assigned && pg->pg_pp == NULL &&
	    !pg->pg_sys_listed,
	    ("[pmu] group %u still scheduled during release", pg->pg_id));
	n = 0;
	TAILQ_FOREACH_SAFE(pe, &pg->pg_events, pe_sibling, tmp) {
		KASSERT(n < capacity,
		    ("[pmu] group %u release member overflow", pg->pg_id));
		KASSERT(PMC_ROW_IS_UNASSIGNED(pe->pe_pmc),
		    ("[pmu] group %u member still assigned", pg->pg_id));
		LIST_FOREACH(pgt, &pg->pg_targets, pgt_group_next)
			pmu_event_remove_target_residuals(pe, pgt->pgt_pp);
		members[n++] = pe->pe_pmc;
		TAILQ_REMOVE(&pg->pg_events, pe, pe_sibling);
		pe->pe_group = NULL;
		pe->pe_is_leader = false;
	}
	pg->pg_leader = NULL;
	pg->pg_nevents = 0;
	if (pg->pg_owner != NULL) {
		LIST_REMOVE(pg, pg_owner_next);
		pg->pg_owner = NULL;
	}
	return (n);
}

void
pmu_group_release(pmu_group_t *pg)
{
	u_int cpu;

	if (pg == NULL)
		return;

	sx_assert(&pmc_sx, SX_XLOCKED);
	KASSERT(LIST_EMPTY(&pg->pg_targets),
	    ("[pmu] freeing group %u with live targets", pg->pg_id));
	KASSERT(pg->pg_explicit_target == NULL,
	    ("[pmu] freeing group %u with explicit target", pg->pg_id));
	KASSERT(TAILQ_EMPTY(&pg->pg_events) && pg->pg_nevents == 0 &&
	    pg->pg_leader == NULL,
	    ("[pmu] freeing non-empty group %u", pg->pg_id));
	KASSERT(!pg->pg_assigned && pg->pg_pp == NULL &&
	    !pg->pg_sys_listed && !pg->pg_sscounted,
	    ("[pmu] freeing scheduled group %u", pg->pg_id));
	if (pg->pg_owner != NULL) {
		LIST_REMOVE(pg, pg_owner_next);
		pg->pg_owner = NULL;
	}
	KASSERT(pg->pg_oncpu_threads == 0 && pg->pg_running_threads == 0,
	    ("[pmu] freeing group with active accounting"));
	KASSERT(!pg->pg_snapshot_pending && !pg->pg_snapshot_active,
	    ("[pmu] freeing group with active snapshot"));
	for (cpu = 0; cpu < pg->pg_ncpu; cpu++) {
		KASSERT(!pg->pg_cpu_state[cpu].pgcs_counted &&
		    !pg->pg_cpu_state[cpu].pgcs_placed &&
		    !pg->pg_cpu_state[cpu].pgcs_transitioning &&
		    pg->pg_cpu_state[cpu].pgcs_td == NULL,
		    ("[pmu] freeing group with active CPU marker"));
	}
	free(pg->pg_cpu_state, M_PMU);
	mtx_destroy(&pg->pg_snapshot_lock);
	free(pg, M_PMU);
}

int
pmu_group_on_allocate(struct pmc *pm, const struct pmc_op_pmcallocate *pa)
{
	pmu_event_t *pe;

	pe = malloc(sizeof(*pe), M_PMU, M_WAITOK | M_ZERO);
	pe->pe_pmc = pm;
	pe->pe_samples = counter_u64_alloc(M_WAITOK);
#ifdef HWPMC_DEBUG
	pe->pe_samples_emitted = counter_u64_alloc(M_WAITOK);
	pe->pe_samples_dropped = counter_u64_alloc(M_WAITOK);
#endif
	pe->pe_alloc = *pa;
	pm->pm_pmu = pe;
	return (0);
}

int
pmu_group_on_attach(struct pmc *pm, struct proc *p)
{
	pmu_event_t *pe;
	pmu_group_t *pg;
	struct pmu_group_target *pgt;
	struct pmc_process *pp;

	sx_assert(&pmc_sx, SX_XLOCKED);
	pe = pmu_event_from_pmc(pm);
	if (pe == NULL || pe->pe_group == NULL)
		return (0);
	pg = pe->pe_group;
	if (pg->pg_explicit_target != NULL &&
	    pg->pg_explicit_target->pgt_pp->pp_proc != p)
		return (EBUSY);
	pp = pmc_find_process_descriptor_pmu(p, PMC_FLAG_ALLOCATE);
	if (pp == NULL)
		return (ENOMEM);
	if (pg->pg_pp != NULL && pg->pg_pp != pp)
		return (EBUSY);
	if (pmu_group_has_target(pg, pp))
		return (0);
	pgt = pmu_group_target_alloc(pg, pp, true, M_WAITOK);
	if (pgt == NULL)
		return (ENOMEM);
	pmu_group_target_publish(pgt);
	return (0);
}

/*
 * Find the target process for a group.
 */
static struct proc *
pmu_group_target_proc(pmu_group_t *pg)
{
	if (pg == NULL || pg->pg_pp == NULL)
		return (NULL);
	return (pg->pg_pp->pp_proc);
}

int
pmu_group_on_start(struct pmc *pm)
{
	pmu_event_t *pe;
	pmu_group_t *pg;
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
		PMCDBG2(PMC, OPS, 1,
		    "on_start: gid=%u pm=%p no target proc -> EINVAL",
		    pg->pg_id, pm);
		return (EINVAL);
	}

	pp = pg->pg_pp;
	if (pp != NULL && pp->pp_pmu_unhashed)
		return (ESRCH);
	if (pp == NULL)
		pp = pmc_find_process_descriptor_pmu(p, PMC_FLAG_ALLOCATE);
	if (pp == NULL)
		return (ENOMEM);

	pmu_pp_link_group(pg, pp);

	mtx_pool_lock_spin(pmc_mtxpool, pg);
	if (pg->pg_account_blocked) {
		mtx_pool_unlock_spin(pmc_mtxpool, pg);
		return (EBUSY);
	}
	mtx_pool_unlock_spin(pmc_mtxpool, pg);

	if (pg->pg_assigned) {
		PMCDBG3(PMC, OPS, 2,
		    "on_start: gid=%u pm=%p ASSIGNED, attaching siblings "
		    "on pp=%p", pg->pg_id, pm, pp);
		pmu_group_attach_siblings(pg, pp);
	} else {
		error = pmu_pp_schedule_in(pp, pg);
		PMCDBG4(PMC, OPS, 2,
		    "on_start: gid=%u pm=%p DEFERRED schedule_in=%d pp=%p",
		    pg->pg_id, pm, error, pp);
		if (error != 0 && error != ENOSPC)
			return (error);
	}

	TAILQ_FOREACH(pe, &pg->pg_events, pe_sibling) {
		pe->pe_pmc->pm_state = PMC_STATE_RUNNING;
		PMCDBG3(PMC, OPS, 3,
		    "on_start: gid=%u pm=%p ri=%d -> RUNNING",
		    pg->pg_id, pe->pe_pmc, PMC_TO_ROWINDEX(pe->pe_pmc));
	}

	mtx_pool_lock_spin(pmc_mtxpool, pg);
	pmu_group_running_start_locked(pg, cpu_ticks());
	pg->pg_account_placement_admit = pg->pg_assigned;
	mtx_pool_unlock_spin(pmc_mtxpool, pg);

	(void)pmc_test_group_hold_resident(pg);
	pmu_pp_kick_rotate(pp);
	return (0);
}

/*
 * Mark the group stopped. Assigned rows stay reserved until release.
 */
void
pmu_group_on_stop(struct pmc *pm)
{
	pmu_event_t *pe;
	pmu_group_t *pg;
	struct pmc_process *pp;

	pe = pmu_event_from_pmc(pm);
	if (pe == NULL || pe->pe_group == NULL)
		return;
	/* Assert lock exclusivity for grouped stops. */
	sx_assert(&pmc_sx, SX_XLOCKED);
	pg = pe->pe_group;
	pp = pg->pg_pp;
	if (pp != NULL)
		mtx_lock_spin(&pp->pp_pmu_lock);
	mtx_pool_lock_spin(pmc_mtxpool, pg);
	if (!pg->pg_running) {
		mtx_pool_unlock_spin(pmc_mtxpool, pg);
		if (pp != NULL)
			mtx_unlock_spin(&pp->pp_pmu_lock);
		return;
	}
	pmu_group_accounting_block_locked(pg, cpu_ticks());
	mtx_pool_unlock_spin(pmc_mtxpool, pg);
	if (pp != NULL)
		mtx_unlock_spin(&pp->pp_pmu_lock);

	TAILQ_FOREACH(pe, &pg->pg_events, pe_sibling)
		pe->pe_pmc->pm_state = PMC_STATE_STOPPED;
	pmu_group_accounting_drain(pg, false);
}

/* Mark one group on-CPU for the switching-in thread. */
static void
pmu_group_csw_in_one(pmu_group_t *pg, struct thread *td, int cpu,
    uint64_t now)
{
	struct pmu_group_cpu_state *pgcs;

	if (pg->pg_system || cpu < 0 || (u_int)cpu >= pg->pg_ncpu)
		return;
	pgcs = &pg->pg_cpu_state[cpu];
	/* Do not wait for snapshot during context switch. */
	mtx_lock_spin(&pg->pg_snapshot_lock);
	mtx_pool_lock_spin(pmc_mtxpool, pg);
	if (!pg->pg_committed || !pg->pg_running || pg->pg_account_blocked) {
		mtx_pool_unlock_spin(pmc_mtxpool, pg);
		mtx_unlock_spin(&pg->pg_snapshot_lock);
		return;
	}
	KASSERT(!pgcs->pgcs_counted,
	    ("[pmu] duplicate CPU %d marker for group %u", cpu, pg->pg_id));
	pmu_group_time_update_locked(pg, now);
	pgcs->pgcs_td = td;
	pgcs->pgcs_counted = true;
	pgcs->pgcs_placed = false;
	pgcs->pgcs_transitioning = true;
	pg->pg_oncpu_threads++;
	LIST_INSERT_HEAD(&pmu_group_cpu_active[cpu], pgcs, pgcs_next);
	mtx_pool_unlock_spin(pmc_mtxpool, pg);
	mtx_unlock_spin(&pg->pg_snapshot_lock);
}

void
pmu_group_csw_in(struct thread *td, struct pmc_process *pp)
{
	struct pmu_group_target *pgt;
	uint64_t now;
	int cpu;

	/* Check lists without lock. */
	if (LIST_EMPTY(&pp->pp_pmu_targets))
		return;
	cpu = PCPU_GET(cpuid);
	now = cpu_ticks();
	mtx_lock_spin(&pp->pp_pmu_lock);
	LIST_FOREACH(pgt, &pp->pp_pmu_targets, pgt_process_next)
		pmu_group_csw_in_one(pgt->pgt_group, td, cpu, now);
	mtx_unlock_spin(&pp->pp_pmu_lock);
}

void
pmu_group_csw_in_complete(struct thread *td, int cpu)
{
	struct pmu_group_cpu_state *pgcs;
	pmu_group_t *pg;

	KASSERT(cpu >= 0 && cpu < MAXCPU,
	    ("[pmu] invalid csw-in completion CPU %d", cpu));
	LIST_FOREACH(pgcs, &pmu_group_cpu_active[cpu], pgcs_next) {
		if (pgcs->pgcs_td != td)
			continue;
		pg = pgcs->pgcs_group;
		mtx_lock_spin(&pg->pg_snapshot_lock);
		mtx_pool_lock_spin(pmc_mtxpool, pg);
		if (pgcs->pgcs_counted && pgcs->pgcs_td == td &&
		    pgcs->pgcs_transitioning)
			pgcs->pgcs_transitioning = false;
		mtx_pool_unlock_spin(pmc_mtxpool, pg);
		mtx_unlock_spin(&pg->pg_snapshot_lock);
	}
}

bool
pmu_group_csw_can_start(struct pmc *pm, struct thread *td, int cpu)
{
	struct pmu_group_cpu_state *pgcs;
	pmu_event_t *pe;
	pmu_group_t *pg;
	bool can_start;

	pe = pmu_event_from_pmc(pm);
	if (pe == NULL || pe->pe_group == NULL)
		return (true);
	pg = pe->pe_group;
	if (pg->pg_system || cpu < 0 || (u_int)cpu >= pg->pg_ncpu)
		return (false);

	pgcs = &pg->pg_cpu_state[cpu];
	mtx_pool_lock_spin(pmc_mtxpool, pg);
	can_start = pg->pg_committed && pg->pg_running &&
	    !pg->pg_account_blocked && pg->pg_account_placement_admit &&
	    pgcs->pgcs_counted && pgcs->pgcs_td == td;
	if (can_start && !pgcs->pgcs_placed) {
		pmu_group_time_update_locked(pg, cpu_ticks());
		pgcs->pgcs_placed = true;
		pg->pg_running_threads++;
		KASSERT(pg->pg_running_threads <= pg->pg_oncpu_threads,
		    ("[pmu] group %u running threads exceed on-CPU threads",
		    pg->pg_id));
	}
	mtx_pool_unlock_spin(pmc_mtxpool, pg);
	return (can_start);
}

void
pmu_group_csw_out(struct thread *td, int cpu)
{
	struct pmu_group_cpu_state *pgcs;
	pmu_group_t *pg;

	KASSERT(cpu >= 0 && cpu < MAXCPU,
	    ("[pmu] invalid csw-out CPU %d", cpu));
	LIST_FOREACH(pgcs, &pmu_group_cpu_active[cpu], pgcs_next) {
		if (pgcs->pgcs_td != td)
			continue;
		pg = pgcs->pgcs_group;
		/* Do not wait for snapshot during context switch. */
		mtx_lock_spin(&pg->pg_snapshot_lock);
		mtx_pool_lock_spin(pmc_mtxpool, pg);
		if (pgcs->pgcs_counted && pgcs->pgcs_td == td &&
		    !pgcs->pgcs_transitioning)
			pgcs->pgcs_transitioning = true;
		mtx_pool_unlock_spin(pmc_mtxpool, pg);
		mtx_unlock_spin(&pg->pg_snapshot_lock);
	}
}

void
pmu_group_csw_out_complete(struct thread *td, int cpu)
{
	struct pmu_group_cpu_state *pgcs, *next;
	pmu_group_t *pg;
	uint64_t now;

	KASSERT(cpu >= 0 && cpu < MAXCPU,
	    ("[pmu] invalid csw-out completion CPU %d", cpu));
	now = cpu_ticks();
	LIST_FOREACH_SAFE(pgcs, &pmu_group_cpu_active[cpu], pgcs_next,
	    next) {
		if (pgcs->pgcs_td != td)
			continue;
		pg = pgcs->pgcs_group;
		mtx_lock_spin(&pg->pg_snapshot_lock);
		mtx_pool_lock_spin(pmc_mtxpool, pg);
		if (!pgcs->pgcs_counted || pgcs->pgcs_td != td) {
			mtx_pool_unlock_spin(pmc_mtxpool, pg);
			mtx_unlock_spin(&pg->pg_snapshot_lock);
			continue;
		}
		pmu_group_time_update_locked(pg, now);
		KASSERT(pg->pg_oncpu_threads > 0,
		    ("[pmu] group %u on-CPU thread underflow", pg->pg_id));
		pg->pg_oncpu_threads--;
		if (pgcs->pgcs_placed) {
			KASSERT(pg->pg_running_threads > 0,
			    ("[pmu] group %u running thread underflow",
			    pg->pg_id));
			pg->pg_running_threads--;
		}
		LIST_REMOVE(pgcs, pgcs_next);
		pgcs->pgcs_td = NULL;
		pgcs->pgcs_counted = false;
		pgcs->pgcs_placed = false;
		pgcs->pgcs_transitioning = false;
		mtx_pool_unlock_spin(pmc_mtxpool, pg);
		mtx_unlock_spin(&pg->pg_snapshot_lock);
	}
}

void
pmu_group_on_release(struct pmc *pm)
{
	pmu_event_t *pe;
	pmu_group_t *pg;

	pe = pmu_event_from_pmc(pm);
	if (pe == NULL)
		return;
	pg = pe->pe_group;
	if (pg != NULL) {
		KASSERT(!pg->pg_committed,
		    ("[pmu] committed group %u released by member", pg->pg_id));
		TAILQ_REMOVE(&pg->pg_events, pe, pe_sibling);
		if (pg->pg_leader == pe)
			pg->pg_leader = NULL;
		KASSERT(pg->pg_nevents > 0,
		    ("[pmu] empty group %u member release", pg->pg_id));
		pg->pg_nevents--;
		pe->pe_group = NULL;
		pe->pe_is_leader = false;
	}
	pm->pm_pmu = NULL;
	pmu_event_destroy(pe);
	if (pg != NULL && pg->pg_nevents == 0)
		pmu_group_release(pg);
}

/*
 * This function finds saved sampling progress.
 * pp_tdslock covers both the thread list and the residual list.
 */
static struct pmu_thread_residual *
pmu_thread_residual_find_locked(struct pmc_process *pp, pmu_event_t *pe,
    lwpid_t tid)
{
	struct pmu_thread_residual *ptr;

	mtx_assert(pp->pp_tdslock, MA_OWNED);
	LIST_FOREACH(ptr, &pp->pp_pmu_residuals, ptr_next) {
		if (ptr->ptr_event == pe && ptr->ptr_tid == tid)
			return (ptr);
	}
	return (NULL);
}

static bool
pmu_thread_exists_locked(struct pmc_process *pp, lwpid_t tid)
{
	struct pmc_thread *pt;

	mtx_assert(pp->pp_tdslock, MA_OWNED);
	LIST_FOREACH(pt, &pp->pp_tds, pt_next) {
		if (pt->pt_td->td_tid == tid)
			return (true);
	}
	return (false);
}

/*
 * This function publishes or replaces one saved value.
 * pp_tdslock is a spin lock.  Allocate memory before you take this lock.
 * On thread exit, remove the thread descriptor first.  This stops a
 * concurrent save from creating stale state again.
 */
static int
pmu_event_set_thread_residual_impl(struct pmc *pm, struct pmc_process *pp,
    lwpid_t tid, pmc_value_t value, uint8_t state)
{
	struct pmu_thread_residual *discard, *ptr;
	pmu_event_t *pe;
	bool keep;

	sx_assert(&pmc_sx, SX_XLOCKED);
	pe = pmu_event_from_pmc(pm);
	if (pe == NULL || value > pm->pm_sc.pm_reloadcount)
		return (EINVAL);

	/*
	 * Keep an entry only for a partial count, or for a pending overflow.
	 * The UNINITIALIZED state removes any existing entry.
	 * Check the value and the state together.  This stops a zero value
	 * from looking like an overflow, and stops the opposite case too.
	 */
	switch (state) {
	case PMU_RESIDUAL_UNINITIALIZED:
		if (value != 0)
			return (EINVAL);
		keep = false;
		break;
	case PMU_RESIDUAL_COUNT_VALID:
		if (value == 0)
			return (EINVAL);
		keep = true;
		break;
	case PMU_RESIDUAL_OVERFLOW_PENDING:
		if (value != 0)
			return (EINVAL);
		keep = true;
		break;
	default:
		return (EINVAL);
	}

	ptr = keep ? malloc(sizeof(*ptr), M_PMU, M_WAITOK | M_ZERO) : NULL;
	discard = NULL;
	mtx_lock_spin(pp->pp_tdslock);
	if (!pmu_thread_exists_locked(pp, tid)) {
		mtx_unlock_spin(pp->pp_tdslock);
		free(ptr, M_PMU);
		return (ESRCH);
	}

	discard = pmu_thread_residual_find_locked(pp, pe, tid);
	if (!keep) {
		if (discard != NULL) {
			LIST_REMOVE(discard, ptr_next);
			pmc_test_counter_add(PMC_TEST_LIVE_RESIDUAL_ENTRIES, -1);
		}
	} else if (discard != NULL) {
		discard->ptr_value = value;
		discard->ptr_state = state;
		discard = ptr;
		ptr = NULL;
	} else {
		ptr->ptr_event = pe;
		ptr->ptr_tid = tid;
		ptr->ptr_value = value;
		ptr->ptr_state = state;
		LIST_INSERT_HEAD(&pp->pp_pmu_residuals, ptr, ptr_next);
		pmc_test_counter_add(PMC_TEST_LIVE_RESIDUAL_ENTRIES, 1);
		ptr = NULL;
	}
	mtx_unlock_spin(pp->pp_tdslock);

	if (discard != NULL)
		free(discard, M_PMU);
	return (0);
}

/*
 * This function saves the progress of each target thread.
 * First, take a snapshot of the row-indexed values under pp_tdslock.
 * Then release the lock, and allocate the stable entries.
 */
void
pmu_event_save_residual(struct pmc *pm, struct pmc_process *pp, int ri)
{
	struct pmu_thread_residual_snapshot {
		lwpid_t tid;
		pmc_value_t value;
	} *snapshot;
	pmu_event_t *pe;
	struct pmc_thread *pt;
	u_int capacity, i, nthreads;
	bool retry;
	int error __diagused;

	sx_assert(&pmc_sx, SX_XLOCKED);
	pe = pmu_event_from_pmc(pm);
	if (pe == NULL)
		return;

	for (;;) {
		capacity = 0;
		mtx_lock_spin(pp->pp_tdslock);
		LIST_FOREACH(pt, &pp->pp_tds, pt_next)
			capacity++;
		mtx_unlock_spin(pp->pp_tdslock);
		if (capacity == 0)
			return;

		snapshot = mallocarray(capacity, sizeof(*snapshot), M_PMU,
		    M_WAITOK);
		nthreads = 0;
		retry = false;
		mtx_lock_spin(pp->pp_tdslock);
		LIST_FOREACH(pt, &pp->pp_tds, pt_next) {
			if (nthreads == capacity) {
				retry = true;
				break;
			}
			snapshot[nthreads].tid = pt->pt_td->td_tid;
			snapshot[nthreads].value = pt->pt_pmcs[ri].pt_pmcval;
			nthreads++;
		}
		mtx_unlock_spin(pp->pp_tdslock);
		if (!retry)
			break;
		free(snapshot, M_PMU);
	}

	for (i = 0; i < nthreads; i++) {
		if (snapshot[i].value == 0 ||
		    snapshot[i].value > pm->pm_sc.pm_reloadcount)
			continue;
		error = pmu_event_set_thread_residual_impl(pm, pp,
		    snapshot[i].tid, snapshot[i].value,
		    PMU_RESIDUAL_COUNT_VALID);
		KASSERT(error == 0 || error == ESRCH,
		    ("[pmu] residual save failed for tid %d: %d",
		    snapshot[i].tid, error));
	}
	free(snapshot, M_PMU);
}

/*
 * Save a system-mode sampling member's progress, read straight off the
 * hardware at eviction.  A value outside (0, reloadcount] means the
 * counter overflowed or was never seeded, so the next placement should
 * start from a full period.
 */
void
pmu_event_set_residual(struct pmc *pm, pmc_value_t residual)
{
	pmu_event_t *pe;

	pe = pmu_event_from_pmc(pm);
	if (pe == NULL)
		return;
	pe->pe_residual = (residual > 0 &&
	    residual <= pm->pm_sc.pm_reloadcount) ? residual : 0;
}

/*
 * Seed the per-thread counts a sampling member resumes from.  A context
 * switch reads pt_pmcval and only falls back to the per-process value when
 * the thread has no descriptor, so restoring the per-process value alone
 * would be ignored for every real thread.
 */
void
pmu_event_restore_thread_residual(struct pmc *pm, struct pmc_process *pp,
    int ri)
{
	struct pmu_thread_residual *ptr;
	pmu_event_t *pe;
	struct pmc_thread *pt;

	pe = pmu_event_from_pmc(pm);
	if (pe == NULL)
		return;
	mtx_lock_spin(pp->pp_tdslock);
	LIST_FOREACH(pt, &pp->pp_tds, pt_next) {
		ptr = pmu_thread_residual_find_locked(pp, pe,
		    pt->pt_td->td_tid);
		/*
		 * A partial count resumes from the saved point.
		 * The overflow-pending state and the no-saved-progress state
		 * both reload a full period.
		 */
		pt->pt_pmcs[ri].pt_pmcval =
		    (ptr != NULL && ptr->ptr_state == PMU_RESIDUAL_COUNT_VALID) ?
		    ptr->ptr_value : pm->pm_sc.pm_reloadcount;
	}
	mtx_unlock_spin(pp->pp_tdslock);
}

/*
 * The count a sampling member resumes from at placement: its saved
 * progress, or a full period if it has none yet.
 */
pmc_value_t
pmu_event_restore_residual(struct pmc *pm)
{
	pmu_event_t *pe;

	pe = pmu_event_from_pmc(pm);
	if (pe == NULL || pe->pe_residual == 0 ||
	    pe->pe_residual > pm->pm_sc.pm_reloadcount)
		return (pm->pm_sc.pm_reloadcount);
	return (pe->pe_residual);
}

#ifdef HWPMC_DEBUG
int
pmu_event_get_thread_residual(struct pmc *pm, struct pmc_process *pp,
    lwpid_t tid, pmc_value_t *value, uint8_t *state)
{
	struct pmu_thread_residual *ptr;
	pmu_event_t *pe;
	int error;

	sx_assert(&pmc_sx, SX_XLOCKED);
	pe = pmu_event_from_pmc(pm);
	if (pe == NULL || value == NULL || state == NULL)
		return (EINVAL);

	error = 0;
	mtx_lock_spin(pp->pp_tdslock);
	if (!pmu_thread_exists_locked(pp, tid))
		error = ESRCH;
	else {
		ptr = pmu_thread_residual_find_locked(pp, pe, tid);
		if (ptr != NULL) {
			*value = ptr->ptr_value;
			*state = ptr->ptr_state;
		} else {
			*value = 0;
			*state = PMU_RESIDUAL_UNINITIALIZED;
		}
	}
	mtx_unlock_spin(pp->pp_tdslock);
	return (error);
}

int
pmu_event_set_thread_residual(struct pmc *pm, struct pmc_process *pp,
    lwpid_t tid, pmc_value_t value, uint8_t state)
{

	return (pmu_event_set_thread_residual_impl(pm, pp, tid, value, state));
}
#endif

static void
pmu_thread_residual_free_task_fn(void *arg __unused, int pending __unused)
{
	struct pmu_thread_residual *ptr;
	LIST_HEAD(, pmu_thread_residual) tmplist;

	LIST_INIT(&tmplist);
	mtx_lock_spin(&pmu_residual_freelist_mtx);
	while ((ptr = LIST_FIRST(&pmu_residual_freelist)) != NULL) {
		LIST_REMOVE(ptr, ptr_next);
		LIST_INSERT_HEAD(&tmplist, ptr, ptr_next);
	}
	mtx_unlock_spin(&pmu_residual_freelist_mtx);

	while ((ptr = LIST_FIRST(&tmplist)) != NULL) {
		LIST_REMOVE(ptr, ptr_next);
		free(ptr, M_PMU);
	}
}

static void
pmu_thread_residual_defer_free(struct pmu_thread_residual *ptr)
{

	mtx_lock_spin(&pmu_residual_freelist_mtx);
	LIST_INSERT_HEAD(&pmu_residual_freelist, ptr, ptr_next);
	taskqueue_enqueue(taskqueue_fast, &pmu_residual_free_task);
	mtx_unlock_spin(&pmu_residual_freelist_mtx);
}

void
pmu_thread_residual_remove(struct pmc_process *pp, lwpid_t tid)
{
	struct pmu_thread_residual *ptr;

	for (;;) {
		mtx_lock_spin(pp->pp_tdslock);
		LIST_FOREACH(ptr, &pp->pp_pmu_residuals, ptr_next) {
			if (ptr->ptr_tid == tid)
				break;
		}
		if (ptr != NULL) {
			LIST_REMOVE(ptr, ptr_next);
			pmc_test_counter_add(PMC_TEST_LIVE_RESIDUAL_ENTRIES, -1);
		}
		mtx_unlock_spin(pp->pp_tdslock);
		if (ptr == NULL)
			break;
		pmu_thread_residual_defer_free(ptr);
	}
}

static void
pmu_event_remove_target_residuals(pmu_event_t *pe, struct pmc_process *pp)
{
	struct pmu_thread_residual *ptr;

	for (;;) {
		mtx_lock_spin(pp->pp_tdslock);
		LIST_FOREACH(ptr, &pp->pp_pmu_residuals, ptr_next) {
			if (ptr->ptr_event == pe)
				break;
		}
		if (ptr != NULL) {
			LIST_REMOVE(ptr, ptr_next);
			pmc_test_counter_add(PMC_TEST_LIVE_RESIDUAL_ENTRIES, -1);
		}
		mtx_unlock_spin(pp->pp_tdslock);
		if (ptr == NULL)
			break;
		free(ptr, M_PMU);
	}
}

/*
 * Attach all sibling events of a group to the process descriptor.
 */
static void
pmu_group_attach_siblings(pmu_group_t *pg, struct pmc_process *anchor_pp)
{
	pmu_event_t *pe;
	struct pmc *pm;
	struct pmc_process *pp;
	struct pmu_group_target *pgt;

	KASSERT(anchor_pp == pg->pg_pp,
	    ("[pmu] group %u anchor mismatch", pg->pg_id));
	LIST_FOREACH(pgt, &pg->pg_targets, pgt_group_next) {
		pp = pgt->pgt_pp;
		TAILQ_FOREACH(pe, &pg->pg_events, pe_sibling) {
			pm = pe->pe_pmc;
			KASSERT(!PMC_ROW_IS_UNASSIGNED(pm),
			    ("[pmu] attach_siblings: gid=%u pm_id=0x%jx "
			    "unassigned", pg->pg_id, (uintmax_t)pm->pm_id));
			if (pp->pp_pmcs[PMC_TO_ROWINDEX(pm)].pp_pmc == pm)
				continue;	/* Already attached at this placement. */
			KASSERT(pp->pp_pmcs[PMC_TO_ROWINDEX(pm)].pp_pmc == NULL,
			    ("[pmu] group %u target pp=%p row=%d busy",
			    pg->pg_id, pp, PMC_TO_ROWINDEX(pm)));
			pmc_rotation_attach(pm, pp);
		}
	}
}

/*
 * Schedule a group in by assigning hardware rows for all siblings.
 */
static int
pmu_pp_schedule_in(struct pmc_process *pp, pmu_group_t *pg)
{
	pmu_event_t *pe;
	struct proc *p;
	int error;

	if (pg == NULL || pg->pg_assigned)
		return (0);
	if (pmc_test_group_hold_evicted(pg))
		return (ENOSPC);
	/* Target process only. */
	p = pmu_group_target_proc(pg);
	if (p == NULL)
		return (EINVAL);
	KASSERT(pp != NULL && pp->pp_proc == p,
	    ("[pmu] schedule_in: pp/p mismatch pp=%p pp->proc=%p p=%p",
	    pp, pp != NULL ? pp->pp_proc : NULL, p));

	error = pmu_group_can_place(pg, p, PMC_CPU_ANY);
	if (error != 0)
		return (error);

	error = pmu_assign_group(pg, p, PMC_CPU_ANY);
	if (error == EBUSY)
		error = ENOSPC;
	if (error != 0) {
		PMCDBG2(PMC, OPS, 1,
		    "pp_schedule_in: assign gid=%u err=%d",
		    pg->pg_id, error);
		return (error);
	}

	/* Set running state before publishing rows. */
	TAILQ_FOREACH(pe, &pg->pg_events, pe_sibling)
		pe->pe_pmc->pm_state = PMC_STATE_RUNNING;

	pmu_group_attach_siblings(pg, pp);
	mtx_pool_lock_spin(pmc_mtxpool, pg);
	pg->pg_account_placement_admit = true;
	mtx_pool_unlock_spin(pmc_mtxpool, pg);
	pmu_group_kick_placement(pg);
	if (pg->pg_running)
		(void)pmc_test_group_hold_resident(pg);
	return (0);
}

/*
 * Schedule a group out and release assigned hardware rows.
 */
static void
pmu_pp_schedule_out(struct pmc_process *pp, pmu_group_t *pg,
    bool drain_samples)
{
	pmu_event_t *pe;
	struct pmc_process *target_pp;
	struct pmu_group_target *pgt;

	if (pg == NULL || !pg->pg_assigned)
		return;

	mtx_pool_lock_spin(pmc_mtxpool, pg);
	pmu_group_time_update_locked(pg, cpu_ticks());
	pg->pg_account_placement_admit = false;
	mtx_pool_unlock_spin(pmc_mtxpool, pg);

	/* Phase 1: stop sibling events. */
	TAILQ_FOREACH(pe, &pg->pg_events, pe_sibling) {
		if (PMC_IS_SAMPLING_MODE(PMC_TO_MODE(pe->pe_pmc))) {
			pmc_rotation_drain_set(pe->pe_pmc, drain_samples);
			wmb();
		}
		pe->pe_pmc->pm_state = PMC_STATE_STOPPED;
	}

	/* Phase 2: flush log and drain context switches. */
	if (pg->pg_owner != NULL &&
	    (pg->pg_owner->po_flags & PMC_PO_OWNS_LOGFILE) != 0)
		(void)pmclog_flush(pg->pg_owner, 1);
	TAILQ_FOREACH(pe, &pg->pg_events, pe_sibling)
		pmc_rotation_drain(pe->pe_pmc);

	/* Phase 3: remove every transient row link from every stable target. */
	KASSERT(pp == pg->pg_pp,
	    ("[pmu] group %u schedule-out anchor mismatch", pg->pg_id));
	LIST_FOREACH(pgt, &pg->pg_targets, pgt_group_next) {
		target_pp = pgt->pgt_pp;
		TAILQ_FOREACH(pe, &pg->pg_events, pe_sibling) {
			struct pmc *pm = pe->pe_pmc;

			KASSERT(!PMC_ROW_IS_UNASSIGNED(pm),
			    ("[pmu] schedule_out: gid=%u pm_id=0x%jx "
			    "unassigned", pg->pg_id, (uintmax_t)pm->pm_id));
			if (target_pp->pp_pmcs[PMC_TO_ROWINDEX(pm)].pp_pmc == pm)
				pmc_rotation_detach(pm, target_pp);
		}
	}

	/* Phase 4: release hardware rows. */
	pmu_unassign_group(pg, 0);
	(void)pmc_test_group_hold_evicted(pg);
}

/*
 * Remove all groups from the process and stop the rotation thread.
 */
static void
pmu_pp_release_all(struct pmc_process *pp)
{
	pmu_event_t *pe;
	pmu_group_t *pg;
	struct pmc_process *replacement_pp;
	struct pmu_group_target *departing, *replacement;
	bool was_running;

	sx_assert(&pmc_sx, SX_XLOCKED);

	if (pp == NULL)
		return;

	mtx_lock_spin(&pp->pp_pmu_lock);
	LIST_FOREACH(pg, &pp->pp_pmu_groups, pg_proc_next)
		pmu_group_accounting_block(pg);
	mtx_unlock_spin(&pp->pp_pmu_lock);

	/* Stop the rotation thread before freeing the process descriptor. */
	pmu_pp_stop_rotate(pp, "muxpurge");
	while (pp->pp_pmu_refs != 0)
		(void)sx_sleep(&pp->pp_pmu_refs, &pmc_sx, 0, "muxrefs", 1);

	/* Schedule out assigned groups and unhook all groups. */
	for (;;) {
		mtx_lock_spin(&pp->pp_pmu_lock);
		pg = LIST_FIRST(&pp->pp_pmu_groups);
		mtx_unlock_spin(&pp->pp_pmu_lock);
		if (pg == NULL)
			break;

		departing = pmu_group_find_target(pg, pp);
		KASSERT(departing != NULL,
		    ("[pmu] group %u anchor has no target edge", pg->pg_id));
		replacement = pmu_group_replacement_target(pg, pp);
		replacement_pp = replacement != NULL ? replacement->pgt_pp : NULL;
		mtx_pool_lock_spin(pmc_mtxpool, pg);
		was_running = pg->pg_running;
		mtx_pool_unlock_spin(pmc_mtxpool, pg);
		pmu_group_accounting_drain(pg, true);
		if (pg->pg_assigned)
			pmu_pp_schedule_out(pp, pg, false);
		pmu_pp_unlink_group(pg, pp);
		pmu_group_target_remove(departing);
		if (replacement_pp == NULL)
			continue;

		/*
		 * pmc_sx serializes target-list mutations and process teardown.
		 * Change the old and new pp_pmu_groups lists separately so their
		 * spin locks are never held together.
		 */
		pmu_pp_link_group(pg, replacement_pp);
		if (was_running) {
			TAILQ_FOREACH(pe, &pg->pg_events, pe_sibling)
				pe->pe_pmc->pm_state = PMC_STATE_RUNNING;
		}
		mtx_pool_lock_spin(pmc_mtxpool, pg);
		if (was_running)
			pmu_group_running_start_locked(pg, cpu_ticks());
		pg->pg_account_placement_admit = pg->pg_assigned;
		pg->pg_account_blocked = false;
		mtx_pool_unlock_spin(pmc_mtxpool, pg);
		pmu_pp_backfill(replacement_pp);
	}
}

/* Initialize PMU fields in process descriptor. */
void
pmu_pp_init(struct pmc_process *pp)
{

	mtx_init(&pp->pp_pmu_lock, "pmc-pmu-groups", "pmc-pmu", MTX_SPIN);
	LIST_INIT(&pp->pp_pmu_groups);
	LIST_INIT(&pp->pp_pmu_targets);
	LIST_INIT(&pp->pp_pmu_residuals);
}

/* Destroy PMU state in process descriptor. */
void
pmu_pp_destroy(struct pmc_process *pp)
{

	pmu_pp_release_all(pp);
	/* Remove every authoritative edge after anchored groups are stopped. */
	pmu_group_disinherit(pp);
	KASSERT(LIST_EMPTY(&pp->pp_pmu_residuals),
	    ("[pmu] pp %p destroyed with saved residuals", pp));
	KASSERT(pp->pp_pmu_refs == 0 && pp->pp_pmu_rot_quiesce == 0 &&
	    pp->pp_pmu_rot_td == NULL,
	    ("[pmu] pp %p destroyed while referenced", pp));
	mtx_destroy(&pp->pp_pmu_lock);
}

void
pmu_group_detach_target(pmu_group_t *pg, struct pmc_process *pp)
{
	pmu_event_t *pe;
	struct pmu_group_target *pgt;

	sx_assert(&pmc_sx, SX_XLOCKED);
	KASSERT(pg != NULL && pp != NULL && pg->pg_pp == pp,
	    ("[pmu] invalid group target detach"));
	KASSERT(!pg->pg_releasing,
	    ("[pmu] group %u detach already active", pg->pg_id));
	pgt = pmu_group_find_target(pg, pp);
	KASSERT(pgt != NULL && pgt->pgt_explicit,
	    ("[pmu] group %u explicit target edge missing", pg->pg_id));

	pg->pg_releasing = true;
	mtx_lock_spin(&pp->pp_pmu_lock);
	pmu_group_accounting_block(pg);
	mtx_unlock_spin(&pp->pp_pmu_lock);
	TAILQ_FOREACH(pe, &pg->pg_events, pe_sibling)
		pe->pe_pmc->pm_state = PMC_STATE_STOPPED;
	pmu_group_accounting_drain(pg, false);
	pmu_pp_stop_rotate(pp, "muxdetach");
	if (pg->pg_assigned)
		pmu_pp_schedule_out(pp, pg, false);

	pmu_group_target_remove(pgt);
	TAILQ_FOREACH(pe, &pg->pg_events, pe_sibling) {
		pe->pe_pmc->pm_flags &= ~(PMC_F_ATTACH_DONE |
		    PMC_F_ATTACHED_TO_OWNER | PMC_F_NEEDS_LOGFILE);
	}
	pg->pg_releasing = false;
	wakeup(&pg->pg_releasing);

	pmu_pp_backfill(pp);
}

/*
 * This function detaches one inherited target from a group.  This target
 * is not the anchor.  The group stays alive for its other targets.
 * For example, use this function when a credential-changing exec revokes
 * one descendant's authorization.  Only this process's edge and row links
 * are removed.
 *
 * The function briefly schedules the group out at its real anchor.  This
 * drops every target's row links.  The function then removes this edge.
 * Then the function puts the group back at the surviving anchor.
 */
void
pmu_group_detach_inherited_target(pmu_group_t *pg, struct pmc_process *pp)
{
	pmu_event_t *pe;
	struct pmc_process *anchor_pp;
	struct pmu_group_target *pgt;
	bool was_running, was_assigned;

	sx_assert(&pmc_sx, SX_XLOCKED);
	KASSERT(pg != NULL && pp != NULL && pg->pg_pp != pp,
	    ("[pmu] inherited detach on anchor pp=%p", pp));
	KASSERT(!pg->pg_releasing,
	    ("[pmu] group %u detach already active", pg->pg_id));
	pgt = pmu_group_find_target(pg, pp);
	KASSERT(pgt != NULL && !pgt->pgt_explicit,
	    ("[pmu] group %u inherited target edge missing", pg->pg_id));

	anchor_pp = pg->pg_pp;
	pg->pg_releasing = true;
	mtx_pool_lock_spin(pmc_mtxpool, pg);
	was_running = pg->pg_running;
	was_assigned = pg->pg_assigned;
	mtx_pool_unlock_spin(pmc_mtxpool, pg);

	/* First, quiesce the group at the anchor.  Then drop every target's row links. */
	if (anchor_pp != NULL) {
		pmu_group_accounting_block(pg);
		TAILQ_FOREACH(pe, &pg->pg_events, pe_sibling)
			pe->pe_pmc->pm_state = PMC_STATE_STOPPED;
		pmu_group_accounting_drain(pg, false);
		pmu_pp_stop_rotate(anchor_pp, "muxreauth");
		if (was_assigned)
			pmu_pp_schedule_out(anchor_pp, pg, false);
	}

	pmu_group_target_remove(pgt);

	/*
	 * Restore the group for the remaining targets.
	 * This edge was never the anchor, so pg_pp does not change.
	 */
	if (was_running) {
		TAILQ_FOREACH(pe, &pg->pg_events, pe_sibling)
			pe->pe_pmc->pm_state = PMC_STATE_RUNNING;
	}
	mtx_pool_lock_spin(pmc_mtxpool, pg);
	if (was_running)
		pmu_group_running_start_locked(pg, cpu_ticks());
	pg->pg_account_placement_admit = pg->pg_assigned;
	pg->pg_account_blocked = false;
	mtx_pool_unlock_spin(pmc_mtxpool, pg);
	pg->pg_releasing = false;
	wakeup(&pg->pg_releasing);
	if (anchor_pp != NULL)
		pmu_pp_backfill(anchor_pp);
}

/* Return true if a running MUX group is unassigned. */
static bool
pmu_list_has_deferred(struct pmu_group_list *gl)
{
	pmu_group_t *pg;

	LIST_FOREACH(pg, gl, pg_proc_next) {
		if (pg->pg_running && !pg->pg_assigned && pg->pg_defer_ok)
			return (true);
	}
	return (false);
}

/*
 * Start or wake the process rotation thread.
 */
static void
pmu_pp_kick_rotate(struct pmc_process *pp)
{
	int error;

	if (pp == NULL || pp->pp_pmu_rot_quiesce != 0 || pp->pp_pmu_unhashed)
		return;
	if (!pmu_list_has_deferred(&pp->pp_pmu_groups))
		return;

	pp->pp_pmu_rot_needed = true;
	if (pp->pp_pmu_rot_running || pp->pp_pmu_rot_td != NULL) {
		wakeup(&pp->pp_pmu_rot_needed);
		return;
	}

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

void
pmu_pp_kick_after_exec(struct pmc_process *pp)
{

	sx_assert(&pmc_sx, SX_XLOCKED);
	pmu_pp_kick_rotate(pp);
}

/*
 * Return bitmask of rows held by assigned MUX groups.
 */
static uint64_t
pmu_list_evictable_rows(struct pmu_group_list *gl)
{
	pmu_event_t *pe;
	pmu_group_t *pg;
	uint64_t evictable;
	u_int n;

	evictable = 0;
	LIST_FOREACH(pg, gl, pg_proc_next) {
		if (!pg->pg_assigned || !pg->pg_defer_ok)
			continue;
		TAILQ_FOREACH(pe, &pg->pg_events, pe_sibling) {
			n = PMC_TO_ROWINDEX(pe->pe_pmc);
			if (n < 64)
				evictable |= 1ULL << n;
		}
	}
	return (evictable);
}

/*
 * Find the next assigned MUX group to evict.
 */
static pmu_group_t *
pmu_list_next_victim(struct pmu_group_list *gl, pmu_group_t **vpg,
    u_int *vseen, u_int ngroups)
{
	pmu_group_t *pg;

	while (*vseen < ngroups) {
		pg = *vpg;
		if (pg == NULL)
			pg = LIST_FIRST(gl);
		(*vseen)++;
		*vpg = LIST_NEXT(pg, pg_proc_next);
		if (pg->pg_assigned && pg->pg_defer_ok &&
		    !pmc_test_group_hold_resident(pg))
			return (pg);
	}
	return (NULL);
}

/*
 * Run one multiplex rotation tick for a process.
 * Evicts assigned MUX groups and assigns deferred MUX groups.
 */
static void
pmu_pp_rotate_one(struct pmc_process *pp)
{
	pmu_group_t *cursor, *pg, *victim, *vpg;
	struct proc *p;
	uint64_t evictable;
	u_int ngroups, seen, vseen;
	int sin_err;
	bool found_cursor, placed_any, satisfiable;

	if (pp == NULL)
		return;
	if (LIST_EMPTY(&pp->pp_pmu_groups)) {
		pp->pp_pmu_rot_needed = false;
		return;
	}

	/* Reset cursor to head if invalid. */
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

	/* Do not rotate if no deferred group can run. */
	evictable = pmu_list_evictable_rows(&pp->pp_pmu_groups);
	satisfiable = false;
	LIST_FOREACH(pg, &pp->pp_pmu_groups, pg_proc_next) {
		if (!pg->pg_running || pg->pg_assigned || !pg->pg_defer_ok)
			continue;
		p = pmu_group_target_proc(pg);
		if (p != NULL && pmu_group_satisfiable(pg, p, PMC_CPU_ANY,
		    evictable) == 0) {
			satisfiable = true;
			break;
		}
	}
	if (!satisfiable)
		goto out;

	/* Evict group at cursor and advance cursor. */
	vpg = cursor;
	vseen = 0;
	victim = pmu_list_next_victim(&pp->pp_pmu_groups, &vpg, &vseen,
	    ngroups);
	if (victim != NULL) {
		PMCDBG3(PMC, OPS, 4, "rotate: pp=%p evict gid=%u nevents=%u",
		    pp, victim->pg_id, victim->pg_nevents);
		pmu_pp_schedule_out(pp, victim, true);
		cursor = LIST_NEXT(victim, pg_proc_next);
		if (cursor == NULL)
			cursor = LIST_FIRST(&pp->pp_pmu_groups);
	}

	/* Assign deferred MUX groups in order. */
	placed_any = false;
	pg = cursor;
	seen = 0;
	while (seen < ngroups) {
		if (pg == NULL)
			pg = LIST_FIRST(&pp->pp_pmu_groups);
		seen++;
		if (!pg->pg_running || pg->pg_assigned || !pg->pg_defer_ok)
			goto next;
		p = pmu_group_target_proc(pg);
		if (p == NULL || pmu_group_satisfiable(pg, p, PMC_CPU_ANY,
		    evictable) != 0)
			goto next;
		sin_err = pmu_pp_schedule_in(pp, pg);
		while (sin_err == ENOSPC && !placed_any) {
			victim = pmu_list_next_victim(&pp->pp_pmu_groups,
			    &vpg, &vseen, ngroups);
			if (victim == NULL)
				break;
			PMCDBG2(PMC, OPS, 4,
			    "rotate: pp=%p escalate evict gid=%u",
			    pp, victim->pg_id);
			pmu_pp_schedule_out(pp, victim, true);
			sin_err = pmu_pp_schedule_in(pp, pg);
		}
		PMCDBG4(PMC, OPS, 4,
		    "rotate: pp=%p schedule_in gid=%u nevents=%u -> %d",
		    pp, pg->pg_id, pg->pg_nevents, sin_err);
		if (sin_err == 0) {
			placed_any = true;
		} else if (sin_err == ENOSPC) {
			pp->pp_pmu_rot_cursor = pg;
			goto out;
		}
		/* Skip group on hard error. */
next:
		pg = LIST_NEXT(pg, pg_proc_next);
	}
	pp->pp_pmu_rot_cursor = cursor;
out:
	/* Stop thread if all groups are assigned. */
	pp->pp_pmu_rot_needed = pmu_list_has_deferred(&pp->pp_pmu_groups);
}

/*
 * Rotation thread for process-mode groups.
 */
static void
pmu_pp_rotate_thread(void *arg)
{
	struct pmc_process *pp = arg;

	sx_xlock(&pmc_sx);
	while (pp->pp_pmu_rot_running) {
		(void)sx_sleep(&pp->pp_pmu_rot_needed, &pmc_sx, 0, "muxrot",
		    pmu_rot_period_ticks());
		if (!pp->pp_pmu_rot_running)
			break;
		pmu_pp_rotate_one(pp);
		if (!pp->pp_pmu_rot_needed)
			break;
	}
	pp->pp_pmu_rot_running = false;
	pp->pp_pmu_rot_td = NULL;
	wakeup(&pp->pp_pmu_rot_td);
	sx_xunlock(&pmc_sx);
	kthread_exit();
}

/*
 * System-mode (PMC_MODE_SC) grouping and per-CPU multiplexing.
 */

/* Update system-mode active count for the CPU. */
static void
pmu_sys_residency_mark(pmu_group_t *pg, bool admit)
{

	mtx_pool_lock_spin(pmc_mtxpool, pg);
	pmu_group_time_update_locked(pg, cpu_ticks());
	pg->pg_account_placement_admit = admit;
	mtx_pool_unlock_spin(pmc_mtxpool, pg);
}

/*
 * Schedule a system group in and start hardware counters.
 */
static int
pmu_sys_schedule_in(int cpu, pmu_group_t *pg)
{
	pmu_event_t *pe;
	struct proc *owner;
	int error;
	u_int nstarted;

	if (pg == NULL || pg->pg_assigned)
		return (0);
	if (pmc_test_group_hold_evicted(pg))
		return (ENOSPC);

	/* Check owner process allocation permission. */
	owner = pg->pg_owner != NULL ? pg->pg_owner->po_owner : NULL;
	if (owner == NULL)
		return (EINVAL);

	error = pmu_group_can_place(pg, owner, cpu);
	if (error != 0)
		return (error);

	error = pmu_assign_group(pg, owner, cpu);
	if (error == EBUSY)
		error = ENOSPC;
	if (error != 0) {
		PMCDBG3(PMC, OPS, 1,
		    "sys_schedule_in: assign gid=%u cpu=%d err=%d",
		    pg->pg_id, cpu, error);
		return (error);
	}

	/*
	 * Every member starts or none does (§2.4).  A class whose configure
	 * can fail would otherwise leave a row running unpublished, which is
	 * the occupancy hole of §5.4 in per-class form.
	 */
	nstarted = 0;
	TAILQ_FOREACH(pe, &pg->pg_events, pe_sibling) {
		pe->pe_pmc->pm_state = PMC_STATE_RUNNING;
		if (pmc_test_should_fail(PMC_TEST_FAIL_SYSTEM_START))
			error = EIO;
		else
			error = hwpmc_pmu_sys_start_row(cpu, pe->pe_pmc);
		if (error != 0) {
			PMCDBG3(PMC, OPS, 1,
			    "sys_schedule_in: start gid=%u cpu=%d err=%d",
			    pg->pg_id, cpu, error);
			pmu_sys_stop_rows(cpu, pg, nstarted);
			pmu_unassign_group(pg, cpu);
			return (error);
		}
		nstarted++;
		pmc_test_system_start_pause(pg, nstarted);
	}
	pmu_sys_residency_mark(pg, true);
	if (pg->pg_sys_listed)
		(void)pmc_test_group_hold_resident(pg);
	PMCDBG3(PMC, OPS, 4, "sys_schedule_in: gid=%u cpu=%d nevents=%u IN",
	    pg->pg_id, cpu, pg->pg_nevents);
	return (0);
}

/* Does any member of this group sample? */
static bool
pmu_group_has_sampling(pmu_group_t *pg)
{
	pmu_event_t *pe;

	TAILQ_FOREACH(pe, &pg->pg_events, pe_sibling) {
		if (PMC_IS_SAMPLING_MODE(PMC_TO_MODE(pe->pe_pmc)))
			return (true);
	}
	return (false);
}

/*
 * Process every sample still queued for this group's members.  A queued
 * sample holds a pointer to its PMC, so none may be left behind when a row
 * is reused or a member released (spec §6.7).
 */
static void
pmu_sys_group_prepare_drain(pmu_group_t *pg, u_int nmembers)
{
	pmu_event_t *pe;
	u_int member;
	bool marked;

	marked = false;
	member = 0;
	TAILQ_FOREACH(pe, &pg->pg_events, pe_sibling) {
		if (member == nmembers)
			break;
		if (PMC_IS_SAMPLING_MODE(PMC_TO_MODE(pe->pe_pmc))) {
			pmc_rotation_drain_set(pe->pe_pmc, true);
			marked = true;
		}
		member++;
	}
	KASSERT(member == nmembers,
	    ("[pmu] group %u drain member mismatch %u/%u", pg->pg_id,
	    member, nmembers));
	if (marked)
		wmb();
}

static void
pmu_sys_group_drain_samples(int cpu, pmu_group_t *pg, u_int nmembers)
{
	pmu_event_t *pe;
	u_int member;

	if (pg->pg_owner != NULL &&
	    (pg->pg_owner->po_flags & PMC_PO_OWNS_LOGFILE) != 0)
		(void)pmclog_flush(pg->pg_owner, 1);
	member = 0;
	TAILQ_FOREACH(pe, &pg->pg_events, pe_sibling) {
		if (member == nmembers)
			break;
		if (PMC_IS_SAMPLING_MODE(PMC_TO_MODE(pe->pe_pmc)))
			pmc_rotation_drain_cpu(pe->pe_pmc, cpu);
		member++;
	}
	KASSERT(member == nmembers,
	    ("[pmu] group %u drained member mismatch %u/%u", pg->pg_id,
	    member, nmembers));
}

/*
 * This function stops and drains the first nmembers rows.
 * Do this before you unpublish or reuse any row.
 * Normal eviction and partial-start rollback both use this function.
 */
static void
pmu_sys_stop_rows(int cpu, pmu_group_t *pg, u_int nmembers)
{
	pmu_event_t *pe;
	u_int member;

	pmu_sys_group_prepare_drain(pg, nmembers);
	member = 0;
	TAILQ_FOREACH(pe, &pg->pg_events, pe_sibling) {
		if (member == nmembers)
			break;
		hwpmc_pmu_sys_stop_row(cpu, pe->pe_pmc);
		member++;
	}
	KASSERT(member == nmembers,
	    ("[pmu] group %u stopped member mismatch %u/%u", pg->pg_id,
	    member, nmembers));

	if (nmembers != 0)
		pmc_test_sample_schedule_out(pg);
	pmu_sys_group_drain_samples(cpu, pg, nmembers);
}

/*
 * Schedule a system group out and stop hardware counters.
 */
static void
pmu_sys_schedule_out(int cpu, pmu_group_t *pg)
{
	if (pg == NULL || !pg->pg_assigned)
		return;

	pmu_sys_stop_rows(cpu, pg, pg->pg_nevents);
	pmu_sys_residency_mark(pg, false);
	pmu_unassign_group(pg, cpu);
	(void)pmc_test_group_hold_evicted(pg);
	PMCDBG2(PMC, OPS, 4, "sys_schedule_out: gid=%u cpu=%d OUT",
	    pg->pg_id, cpu);
}

/* Drop the group from the owner's system-sample accounting. */
static void
pmu_sys_group_sscount_drop(pmu_group_t *pg)
{

	if (!pg->pg_sscounted)
		return;
	pg->pg_sscounted = false;
	if (pg->pg_owner != NULL)
		hwpmc_sscount_sub(pg->pg_owner);
}

/*
 * Start a system group on its bound CPU.
 */
int
pmu_sys_group_on_start(struct pmc *pm)
{
	pmu_event_t *pe;
	pmu_group_t *pg;
	enum pmc_state prior_state;
	bool added_sscount, was_running;
	int cpu;

	pe = pmu_event_from_pmc(pm);
	if (pe == NULL || pe->pe_group == NULL)
		return (0);
	pg = pe->pe_group;
	if (!pg->pg_committed)
		return (EDOOFUS);
	if (!pg->pg_system)
		return (EINVAL);

	cpu = pg->pg_cpu;
	if (cpu < 0 || cpu >= MAXCPU)
		return (EINVAL);
	if (!pmc_cpu_is_active(cpu))
		return (ENXIO);

	prior_state = pm->pm_state;
	added_sscount = false;
	mtx_pool_lock_spin(pmc_mtxpool, pg);
	if (pg->pg_account_blocked) {
		mtx_pool_unlock_spin(pmc_mtxpool, pg);
		return (EBUSY);
	}
	was_running = pg->pg_running;
	pmu_group_running_start_locked(pg, cpu_ticks());
	mtx_pool_unlock_spin(pmc_mtxpool, pg);
	if (!was_running) {
		int sin_err;

		/*
		 * Publish system-sampling ownership before the hardware can
		 * accept a sample.  pmc_start() already sent the kernel mappings.
		 */
		if (!pg->pg_sscounted && pmu_group_has_sampling(pg) &&
		    pg->pg_owner != NULL) {
			hwpmc_sscount_add(pg->pg_owner);
			pg->pg_sscounted = true;
			added_sscount = true;
		}
		sin_err = pmu_sys_schedule_in(cpu, pg);
		PMCDBG3(PMC, OPS, 2,
		    "sys_on_start: gid=%u cpu=%d schedule_in=%d",
		    pg->pg_id, cpu, sin_err);
		/* Fail if non-multiplex group cannot fit. */
		if (sin_err != 0 && (sin_err != ENOSPC || !pg->pg_defer_ok)) {
			mtx_pool_lock_spin(pmc_mtxpool, pg);
			pmu_group_running_stop_locked(pg, cpu_ticks());
			mtx_pool_unlock_spin(pmc_mtxpool, pg);
			if (added_sscount)
				pmu_sys_group_sscount_drop(pg);
			TAILQ_FOREACH(pe, &pg->pg_events, pe_sibling)
				pe->pe_pmc->pm_state = prior_state;
			return (sin_err);
		}
	}

	/*
	 * List the group for rotation only once it is running: a group left
	 * listed after a failed start would be walked by the rotation thread
	 * while it owns nothing.
	 */
	if (!pg->pg_sys_listed) {
		LIST_INSERT_HEAD(&pmu_syscpu[cpu].sc_groups, pg, pg_proc_next);
		pg->pg_sys_listed = true;
	}

	/* Publish the logical start for every member of the group. */
	TAILQ_FOREACH(pe, &pg->pg_events, pe_sibling)
		pe->pe_pmc->pm_state = PMC_STATE_RUNNING;

	(void)pmc_test_group_hold_resident(pg);
	pmu_syscpu_kick_rotate(cpu);
	return (0);
}

/*
 * Stop a system group and release its hardware rows.
 */
void
pmu_sys_group_on_stop(struct pmc *pm)
{
	pmu_event_t *pe;
	pmu_group_t *pg;

	pe = pmu_event_from_pmc(pm);
	if (pe == NULL || pe->pe_group == NULL)
		return;
	pg = pe->pe_group;
	if (!pg->pg_system || !pg->pg_running)
		return;

	if (pg->pg_assigned)
		pmu_sys_schedule_out(pg->pg_cpu, pg);
	mtx_pool_lock_spin(pmc_mtxpool, pg);
	pmu_group_running_stop_locked(pg, cpu_ticks());
	mtx_pool_unlock_spin(pmc_mtxpool, pg);
	pmu_sys_group_sscount_drop(pg);

	TAILQ_FOREACH(pe, &pg->pg_events, pe_sibling)
		pe->pe_pmc->pm_state = PMC_STATE_STOPPED;
}

static void
pmu_syscpu_stop_rotate(struct pmu_syscpu *sc, const char *wmesg)
{

	mtx_pool_lock_spin(pmc_mtxpool, sc);
	sc->sc_quiesce++;
	sc->sc_running = false;
	sc->sc_needed = false;
	mtx_pool_unlock_spin(pmc_mtxpool, sc);
	wakeup(&sc->sc_needed);
	while (sc->sc_td != NULL)
		(void)sx_sleep(&sc->sc_td, &pmc_sx, 0, wmesg, 1);
	mtx_pool_lock_spin(pmc_mtxpool, sc);
	KASSERT(sc->sc_quiesce > 0,
	    ("[pmu] system rotation quiesce underflow"));
	sc->sc_quiesce--;
	mtx_pool_unlock_spin(pmc_mtxpool, sc);
}

/*
 * Prepare a system group for descriptor release.
 */
void
pmu_sys_group_pre_release(struct pmc *pm)
{
	pmu_event_t *pe;
	pmu_group_t *other, *pg;
	struct pmu_syscpu *sc;
	int cpu;

	pe = pmu_event_from_pmc(pm);
	if (pe == NULL || pe->pe_group == NULL)
		return;
	pg = pe->pe_group;
	if (!pg->pg_system)
		return;

	cpu = pg->pg_cpu;
	if (cpu < 0 || cpu >= MAXCPU)
		return;
	sc = &pmu_syscpu[cpu];

	/* Stop rotation before modifying group list. */
	if (pg->pg_sys_listed)
		pmu_syscpu_stop_rotate(sc, "muxsrel");

	if (pg->pg_assigned)
		pmu_sys_schedule_out(cpu, pg);
	else {
		pmu_sys_group_prepare_drain(pg, pg->pg_nevents);
		pmu_sys_group_drain_samples(cpu, pg, pg->pg_nevents);
	}
	mtx_pool_lock_spin(pmc_mtxpool, pg);
	pmu_group_running_stop_locked(pg, cpu_ticks());
	mtx_pool_unlock_spin(pmc_mtxpool, pg);

	pmu_sys_group_sscount_drop(pg);

	if (pg->pg_sys_listed) {
		if (sc->sc_cursor == pg)
			sc->sc_cursor = LIST_NEXT(pg, pg_proc_next);
		LIST_REMOVE(pg, pg_proc_next);
		pg->pg_sys_listed = false;
	}
	LIST_FOREACH(other, &sc->sc_groups, pg_proc_next) {
		if (other->pg_running && !other->pg_assigned)
			(void)pmu_sys_schedule_in(cpu, other);
	}
	pmu_syscpu_kick_rotate(cpu);
}

/*
 * Start or wake the CPU rotation thread.
 */
static void
pmu_syscpu_kick_rotate(int cpu)
{
	struct pmu_syscpu *sc;
	int error;

	sc = &pmu_syscpu[cpu];
	if (sc->sc_quiesce != 0)
		return;
	if (!pmu_list_has_deferred(&sc->sc_groups))
		return;

	sc->sc_needed = true;
	if (sc->sc_running || sc->sc_td != NULL) {
		wakeup(&sc->sc_needed);
		return;
	}

	sc->sc_running = true;
	error = kthread_add(pmu_syscpu_rotate_thread,
	    (void *)(intptr_t)cpu, NULL, &sc->sc_td, 0, 0,
	    "pmu_rot_cpu_%d", cpu);
	if (error != 0) {
		sc->sc_running = false;
		sc->sc_td = NULL;
		PMCDBG2(PMC, OPS, 1,
		    "syscpu_kick_rotate: cpu=%d kthread_add err=%d",
		    cpu, error);
	}
}

/*
 * Run one rotation tick on the bound CPU.
 */
static void
pmu_syscpu_rotate_one(int cpu)
{
	struct pmu_syscpu *sc;
	pmu_group_t *cursor, *pg, *victim, *vpg;
	struct proc *owner;
	uint64_t evictable;
	u_int ngroups, seen, vseen;
	int sin_err;
	bool found_cursor, placed_any, satisfiable;

	sc = &pmu_syscpu[cpu];
	if (LIST_EMPTY(&sc->sc_groups)) {
		sc->sc_needed = false;
		return;
	}

	ngroups = 0;
	cursor = sc->sc_cursor;
	found_cursor = false;
	LIST_FOREACH(pg, &sc->sc_groups, pg_proc_next) {
		if (pg == cursor)
			found_cursor = true;
		ngroups++;
	}
	if (!found_cursor)
		cursor = LIST_FIRST(&sc->sc_groups);

	evictable = pmu_list_evictable_rows(&sc->sc_groups);
	satisfiable = false;
	LIST_FOREACH(pg, &sc->sc_groups, pg_proc_next) {
		if (!pg->pg_running || pg->pg_assigned || !pg->pg_defer_ok)
			continue;
		owner = pg->pg_owner != NULL ? pg->pg_owner->po_owner : NULL;
		if (owner != NULL && pmu_group_satisfiable(pg, owner, cpu,
		    evictable) == 0) {
			satisfiable = true;
			break;
		}
	}
	if (!satisfiable)
		goto out;

	/* Evict group at cursor and advance cursor. */
	vpg = cursor;
	vseen = 0;
	victim = pmu_list_next_victim(&sc->sc_groups, &vpg, &vseen, ngroups);
	if (victim != NULL) {
		PMCDBG3(PMC, OPS, 4, "sysrotate: cpu=%d evict gid=%u "
		    "nevents=%u", cpu, victim->pg_id, victim->pg_nevents);
		pmu_sys_schedule_out(cpu, victim);
		cursor = LIST_NEXT(victim, pg_proc_next);
		if (cursor == NULL)
			cursor = LIST_FIRST(&sc->sc_groups);
	}

	/* Assign deferred MUX groups in order. */
	placed_any = false;
	pg = cursor;
	seen = 0;
	while (seen < ngroups) {
		if (pg == NULL)
			pg = LIST_FIRST(&sc->sc_groups);
		seen++;
		if (!pg->pg_running || pg->pg_assigned || !pg->pg_defer_ok)
			goto next;
		owner = pg->pg_owner != NULL ? pg->pg_owner->po_owner : NULL;
		if (owner == NULL || pmu_group_satisfiable(pg, owner, cpu,
		    evictable) != 0)
			goto next;
		sin_err = pmu_sys_schedule_in(cpu, pg);
		while (sin_err == ENOSPC && !placed_any) {
			victim = pmu_list_next_victim(&sc->sc_groups, &vpg,
			    &vseen, ngroups);
			if (victim == NULL)
				break;
			PMCDBG2(PMC, OPS, 4,
			    "sysrotate: cpu=%d escalate evict gid=%u",
			    cpu, victim->pg_id);
			pmu_sys_schedule_out(cpu, victim);
			sin_err = pmu_sys_schedule_in(cpu, pg);
		}
		if (sin_err == 0) {
			placed_any = true;
		} else if (sin_err == ENOSPC) {
			sc->sc_cursor = pg;
			goto out;
		}
		/* Skip group on hard error. */
next:
		pg = LIST_NEXT(pg, pg_proc_next);
	}
	sc->sc_cursor = cursor;
out:
	/* Stop thread if all groups are assigned. */
	sc->sc_needed = pmu_list_has_deferred(&sc->sc_groups);
}

/*
 * Rotation thread for system-mode groups on a CPU.
 */
static void
pmu_syscpu_rotate_thread(void *arg)
{
	int cpu = (int)(intptr_t)arg;
	struct pmu_syscpu *sc = &pmu_syscpu[cpu];

	sx_xlock(&pmc_sx);
	while (sc->sc_running) {
		(void)sx_sleep(&sc->sc_needed, &pmc_sx, 0, "muxsys",
		    pmu_rot_period_ticks());
		if (!sc->sc_running)
			break;
		pmu_syscpu_rotate_one(cpu);
		if (!sc->sc_needed)
			break;
	}
	sc->sc_running = false;
	sc->sc_td = NULL;
	wakeup(&sc->sc_td);
	sx_xunlock(&pmc_sx);
	kthread_exit();
}
