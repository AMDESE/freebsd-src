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
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/mutex.h>
#include <sys/pcpu.h>
#include <sys/pmc.h>
#include <sys/pmckern.h>
#include <sys/proc.h>
#include <sys/sysctl.h>
#include <sys/sx.h>
#include <sys/unistd.h>

#include "hwpmc_pmu.h"

static MALLOC_DEFINE(M_PMU, "pmu", "hwpmc PMU grouping");

static uint32_t pmu_next_group_id = 1;

extern struct mtx_pool *pmc_mtxpool;

LIST_HEAD(pmu_group_cpu_list, pmu_group_cpu_state);
static struct pmu_group_cpu_list pmu_group_cpu_active[MAXCPU];
static uint32_t pmu_group_time_id;
static int pmu_group_time_sysctl(SYSCTL_HANDLER_ARGS);

SYSCTL_DECL(_kern_hwpmc);
SYSCTL_UINT(_kern_hwpmc, OID_AUTO, group_time_id, CTLFLAG_RWTUN,
    &pmu_group_time_id, 0, "Selected PMU group accounting id");
SYSCTL_PROC(_kern_hwpmc, OID_AUTO, group_time,
    CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE, 0, 0,
    pmu_group_time_sysctl, "A", "Selected PMU group accounting in ns");

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
static int pmu_pp_schedule_in(struct pmc_process *pp, pmu_group_t *pg);
static void pmu_pp_schedule_out(struct pmc_process *pp, pmu_group_t *pg,
    bool drain_samples);
static void pmu_pp_kick_rotate(struct pmc_process *pp);
static int pmu_group_attach_siblings(pmu_group_t *pg,
    struct pmc_process *pp);

/*
 * System-wide (PMC_MODE_SC) multiplex state.  Process-mode groups hang
 * off a pmc_process and rotate in a per-pp kthread; system-wide groups
 * are bound to a single CPU and rotate in a per-CPU kthread keyed by
 * this registry.  pg_proc_next (the same LIST_ENTRY the per-pp path
 * uses) links a system group onto sc_groups -- the two worlds are
 * mutually exclusive for any given group (pg_system selects which).
 */
struct pmu_syscpu {
	LIST_HEAD(, pmu_group)	sc_groups;	/* groups bound to this CPU */
	pmu_group_t		*sc_cursor;	/* round-robin start point */
	struct thread		*sc_td;		/* rotation kthread */
	u_int			sc_quiesce;
	bool			sc_running;	/* kthread should keep going */
};
static struct pmu_syscpu pmu_syscpu[MAXCPU];

static void pmu_syscpu_rotate_thread(void *arg);
static int pmu_sys_schedule_in(int cpu, pmu_group_t *pg);
static void pmu_sys_schedule_out(int cpu, pmu_group_t *pg);
static void pmu_syscpu_kick_rotate(int cpu);
static void pmu_syscpu_rotate_one(int cpu);

void
pmu_group_accounting_initialize(void)
{
	u_int cpu;

	for (cpu = 0; cpu < MAXCPU; cpu++)
		LIST_INIT(&pmu_group_cpu_active[cpu]);
}

void
pmu_group_accounting_finalize(void)
{
	u_int cpu;

	for (cpu = 0; cpu < MAXCPU; cpu++)
		KASSERT(LIST_EMPTY(&pmu_group_cpu_active[cpu]),
		    ("[pmu] active accounting markers on CPU %u", cpu));
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
	__uint128_t ns;

	if (tickrate == 0)
		return (0);
	ns = (__uint128_t)ticks * 1000000000;
	ns /= tickrate;
	if (ns > UINT64_MAX)
		return (UINT64_MAX);
	return ((uint64_t)ns);
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

void
pmu_group_time_snapshot(pmu_group_t *pg,
    struct pmu_group_time_snapshot *snapshot)
{
	uint64_t now;

	now = cpu_ticks();
	mtx_pool_lock_spin(pmc_mtxpool, pg);
	pmu_group_time_snapshot_locked(pg, snapshot, now);
	mtx_pool_unlock_spin(pmc_mtxpool, pg);
}

static int
pmu_group_time_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct pmu_group_time_snapshot snapshot;
	struct pmc_owner *po;
	pmu_group_t *pg;
	char buf[256];
	uint32_t group_id;

	group_id = pmu_group_time_id;
	hwpmc_pmu_sx_xlock();
	po = pmc_find_owner_descriptor_pmu(curproc);
	pg = po != NULL ? pmu_group_lookup(po, group_id) : NULL;
	if (pg != NULL)
		pmu_group_time_snapshot(pg, &snapshot);
	hwpmc_pmu_sx_xunlock();
	if (pg == NULL)
		return (ENOENT);

	snprintf(buf, sizeof(buf),
	    "gid=%u unit=%s enabled=%ju running=%ju enabled_wall=%ju "
	    "wall=%ju", group_id, snapshot.pgts_system ? "wall-ns" :
	    "thread-ns", (uintmax_t)snapshot.pgts_enabled,
	    (uintmax_t)snapshot.pgts_running,
	    (uintmax_t)snapshot.pgts_enabled_wall,
	    (uintmax_t)snapshot.pgts_wall);
	return (sysctl_handle_string(oidp, buf, sizeof(buf), req));
}

static void
pmu_group_accounting_block(pmu_group_t *pg)
{
	uint64_t now;

	now = cpu_ticks();
	mtx_pool_lock_spin(pmc_mtxpool, pg);
	pmu_group_time_update_locked(pg, now);
	pg->pg_account_blocked = true;
	pg->pg_account_placement_admit = false;
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

	hwpmc_pmu_sx_assert_xlocked();
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
		if (!pmc_cpu_is_active(cpu))
			continue;
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

	if (pg->pg_system || pg->pg_ncpu == 0)
		return;
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
	mtx_unlock_spin(&pp->pp_pmu_lock);
	wakeup(pp);
	while (pp->pp_pmu_rot_td != NULL)
		(void)hwpmc_pmu_sx_sleep(&pp->pp_pmu_rot_td, 1, wmesg);
	mtx_lock_spin(&pp->pp_pmu_lock);
	KASSERT(pp->pp_pmu_rot_quiesce > 0,
	    ("[pmu] rotation quiesce underflow"));
	pp->pp_pmu_rot_quiesce--;
	mtx_unlock_spin(&pp->pp_pmu_lock);
}

pmu_event_t *
pmu_event_from_pmc(struct pmc *pm)
{
	return (pm != NULL ? pm->pm_pmu : NULL);
}

void
pmu_event_destroy(pmu_event_t *pe)
{
	if (pe == NULL)
		return;
	free(pe, M_PMU);
}

int
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
	LIST_INSERT_HEAD(&po->po_groups, pg, pg_owner_next);
	*pg_id = pg->pg_id;
	return (0);
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

	error = pmu_group_can_fit(pg);
	if (error != 0)
		return (error);
	pmu_group_accounting_init(pg);

	po = pg->pg_owner;
	p = po != NULL ? po->po_owner : NULL;
	if (p == NULL)
		return (EINVAL);

	/*
	 * System-wide group.  There is no target process: the group is
	 * pinned to a single CPU (validated identical across siblings).
	 * Unlike the process path below -- which binds a fitting group to
	 * HW rows right here at commit -- system groups defer ALL HW
	 * placement to pmu_sys_group_on_start() and the per-CPU rotation
	 * kthread, because programming a system counter requires the
	 * pmc_select_cpu() bind dance that only runs in thread context.
	 * Commit therefore only proves the group can ever fit.
	 */
	if (PMC_IS_SYSTEM_MODE(pg->pg_leader->pe_alloc.pm_mode)) {
		pg->pg_system = true;
		pg->pg_cpu = pg->pg_leader->pe_alloc.pm_cpu;

		error = pmu_group_can_place(pg, p, pg->pg_cpu);
		PMCDBG5(PMC, OPS, 1,
		    "group_commit: SYS gid=%u cpu=%d nevents=%u "
		    "place=%d defer_ok=%d", pg->pg_id, pg->pg_cpu,
		    pg->pg_nevents, error, (int)pg->pg_defer_ok);
		if (error != 0 && error != ENOSPC)
			return (error);
		if (error == ENOSPC && !pg->pg_defer_ok)
			return (ENOSPC);

		pg->pg_committed = true;
		return (0);
	}

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
	pmu_group_t *other;
	struct pmc_process *held_pp, *pp;
	bool pp_unhashed;
	u_int n;

	hwpmc_pmu_sx_assert_xlocked();
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
			mtx_lock_spin(&pp->pp_pmu_lock);
			if (pp->pp_pmu_rot_cursor == pg)
				pp->pp_pmu_rot_cursor = LIST_NEXT(pg,
				    pg_proc_next);
			LIST_REMOVE(pg, pg_proc_next);
			pg->pg_pp = NULL;
			pg->pg_attach_proc = NULL;
			mtx_unlock_spin(&pp->pp_pmu_lock);
			if (!pp->pp_pmu_unhashed) {
				LIST_FOREACH(other, &pp->pp_pmu_groups, pg_proc_next) {
					if (other->pg_running && !other->pg_assigned)
						(void)pmu_pp_schedule_in(pp, other);
				}
				pmu_pp_kick_rotate(pp);
			}
		}
		if (held_pp != NULL) {
			mtx_lock_spin(&held_pp->pp_pmu_lock);
			KASSERT(held_pp->pp_pmu_refs > 0,
			    ("[pmu] process reference underflow"));
			held_pp->pp_pmu_refs--;
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

	hwpmc_pmu_sx_assert_xlocked();
	KASSERT(TAILQ_EMPTY(&pg->pg_events) && pg->pg_nevents == 0 &&
	    pg->pg_leader == NULL,
	    ("[pmu] freeing non-empty group %u", pg->pg_id));
	KASSERT(!pg->pg_assigned && pg->pg_pp == NULL &&
	    !pg->pg_sys_listed,
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
	pe->pe_alloc = *pa;
	pe->pe_state = PMU_EVENT_STATE_INACTIVE;
	pe->pe_assigned_row = -1;
	pm->pm_pmu = pe;
	return (0);
}

int
pmu_group_on_attach(struct pmc *pm, struct proc *p)
{
	pmu_event_t *pe;
	pmu_group_t *pg;
	struct pmc_process *pp;

	hwpmc_pmu_sx_assert_xlocked();
	pe = pmu_event_from_pmc(pm);
	if (pe == NULL || pe->pe_group == NULL)
		return (0);
	pg = pe->pe_group;
	if (pg->pg_attach_proc != NULL && pg->pg_attach_proc != p)
		return (EBUSY);
	pp = pmc_find_process_descriptor_pmu(p, PMC_FLAG_ALLOCATE);
	if (pp == NULL)
		return (ENOMEM);
	if (pg->pg_pp != NULL && pg->pg_pp != pp)
		return (EBUSY);
	pg->pg_attach_proc = p;
	if (pg->pg_pp == NULL) {
		mtx_lock_spin(&pp->pp_pmu_lock);
		LIST_INSERT_HEAD(&pp->pp_pmu_groups, pg, pg_proc_next);
		pg->pg_pp = pp;
		mtx_unlock_spin(&pp->pp_pmu_lock);
	}
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
pmu_group_target_proc(pmu_group_t *pg)
{
	pmu_event_t *pe;
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

	if (pg->pg_attach_proc == NULL)
		pg->pg_attach_proc = p;

	if (pg->pg_pp == NULL) {
		mtx_lock_spin(&pp->pp_pmu_lock);
		LIST_INSERT_HEAD(&pp->pp_pmu_groups, pg, pg_proc_next);
		pg->pg_pp = pp;
		mtx_unlock_spin(&pp->pp_pmu_lock);
	}

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
		error = pmu_group_attach_siblings(pg, pp);
		if (error != 0) {
			PMCDBG2(PMC, OPS, 1,
			    "on_start: gid=%u attach_siblings err=%d",
			    pg->pg_id, error);
			return (error);
		}
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

	pmu_pp_kick_rotate(pp);
	return (0);
}

/*
 * Mark the group stopped.  We deliberately do NOT release HW rows
 * here:
 *
 *   1. Group PMCSTOP retains pmc_sx exclusively while active CPU
 *      markers are drained, but keeps the assigned rows reserved.
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
	pmu_event_t *pe;
	pmu_group_t *pg;
	struct pmc_process *pp;

	hwpmc_pmu_sx_assert_xlocked();
	pe = pmu_event_from_pmc(pm);
	if (pe == NULL || pe->pe_group == NULL)
		return;
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
	mtx_pool_unlock_spin(pmc_mtxpool, pg);
	pmu_group_accounting_block(pg);
	if (pp != NULL)
		mtx_unlock_spin(&pp->pp_pmu_lock);

	TAILQ_FOREACH(pe, &pg->pg_events, pe_sibling)
		pe->pe_pmc->pm_state = PMC_STATE_STOPPED;
	pmu_group_accounting_drain(pg, false);
}

/* Tick accounting and context-switch hooks. */
void pmu_event_account_in(pmu_event_t *pe __unused,
    uint64_t now __unused) { }
void pmu_event_account_out(pmu_event_t *pe __unused,
    uint64_t now __unused) { }
void pmu_rotate_groups(int cpu __unused) { }

static void
pmu_group_transition_lock(pmu_group_t *pg)
{

	for (;;) {
		mtx_lock_spin(&pg->pg_snapshot_lock);
		if (!pg->pg_snapshot_pending && !pg->pg_snapshot_active)
			return;
		mtx_unlock_spin(&pg->pg_snapshot_lock);
		cpu_spinwait();
	}
}

void
pmu_group_csw_in(struct thread *td, struct pmc_process *pp)
{
	struct pmu_group_cpu_state *pgcs;
	pmu_group_t *pg;
	uint64_t now;
	int cpu;

	cpu = PCPU_GET(cpuid);
	now = cpu_ticks();
	mtx_lock_spin(&pp->pp_pmu_lock);
	LIST_FOREACH(pg, &pp->pp_pmu_groups, pg_proc_next) {
		if (pg->pg_system || cpu < 0 || (u_int)cpu >= pg->pg_ncpu)
			continue;
		pgcs = &pg->pg_cpu_state[cpu];
		pmu_group_transition_lock(pg);
		mtx_pool_lock_spin(pmc_mtxpool, pg);
		if (!pg->pg_committed || !pg->pg_running ||
		    pg->pg_account_blocked) {
			mtx_pool_unlock_spin(pmc_mtxpool, pg);
			mtx_unlock_spin(&pg->pg_snapshot_lock);
			continue;
		}
		KASSERT(!pgcs->pgcs_counted,
		    ("[pmu] duplicate CPU %d marker for group %u", cpu,
		    pg->pg_id));
		if (pgcs->pgcs_counted) {
			mtx_pool_unlock_spin(pmc_mtxpool, pg);
			mtx_unlock_spin(&pg->pg_snapshot_lock);
			continue;
		}
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
	mtx_unlock_spin(&pp->pp_pmu_lock);
}

void
pmu_group_csw_in_complete(struct thread *td, int cpu)
{
	struct pmu_group_cpu_state *pgcs;
	pmu_group_t *pg;

	KASSERT(cpu >= 0 && cpu < MAXCPU,
	    ("[pmu] invalid csw-in completion CPU %d", cpu));
	if (cpu < 0 || cpu >= MAXCPU)
		return;

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
	if (cpu < 0 || cpu >= MAXCPU)
		return;

	LIST_FOREACH(pgcs, &pmu_group_cpu_active[cpu], pgcs_next) {
		if (pgcs->pgcs_td != td)
			continue;
		pg = pgcs->pgcs_group;
		pmu_group_transition_lock(pg);
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
	if (cpu < 0 || cpu >= MAXCPU)
		return;

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

int
pmu_group_read_value(struct pmc *pm, pmc_value_t *value)
{
	pmu_event_t *pe;
	pmu_group_t *pg;

	if (value == NULL)
		return (EINVAL);
	pe = pmu_event_from_pmc(pm);
	if (pe == NULL || pe->pe_group == NULL)
		return (ENOENT);
	pg = pe->pe_group;

	mtx_pool_lock_spin(pmc_mtxpool, pm);
	if (PMC_ROW_IS_UNASSIGNED(pm))
		*value = pm->pm_gv.pm_savedvalue;
	else if (pg->pg_system)
		*value += pm->pm_gv.pm_savedvalue;
	mtx_pool_unlock_spin(pmc_mtxpool, pm);
	return (0);
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
pmu_group_attach_siblings(pmu_group_t *pg, struct pmc_process *pp)
{
	pmu_event_t *pe;
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
pmu_pp_schedule_in(struct pmc_process *pp, pmu_group_t *pg)
{
	pmu_event_t *pe;
	struct proc *p;
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
	mtx_pool_lock_spin(pmc_mtxpool, pg);
	pg->pg_account_placement_admit = true;
	mtx_pool_unlock_spin(pmc_mtxpool, pg);
	pmu_group_kick_placement(pg);
	return (0);
}

/*
 * Atomically schedule pg out: detach every sibling from pp and
 * release every HW row in one pass.  Counts already accumulated in
 * pm->pm_gv.pm_savedvalue are preserved (we never zero them).
 * Caller holds pmc_sx exclusive.
 */
static void
pmu_pp_schedule_out(struct pmc_process *pp, pmu_group_t *pg,
    bool drain_samples)
{
	pmu_event_t *pe;

	if (pg == NULL || !pg->pg_assigned)
		return;

	mtx_pool_lock_spin(pmc_mtxpool, pg);
	pmu_group_time_update_locked(pg, cpu_ticks());
	pg->pg_account_placement_admit = false;
	mtx_pool_unlock_spin(pmc_mtxpool, pg);

	/* Phase 1: stop every sibling so csw_in/csw_out skip them. */
	TAILQ_FOREACH(pe, &pg->pg_events, pe_sibling) {
		if (pe->pe_state != PMU_EVENT_STATE_ACTIVE)
			continue;
		if (PMC_IS_SAMPLING_MODE(PMC_TO_MODE(pe->pe_pmc))) {
			atomic_store_rel_int(&pe->pe_pmc->pm_rotation_drain,
			    drain_samples);
			wmb();
		}
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
	pmu_group_t *pg;

	hwpmc_pmu_sx_assert_xlocked();

	if (pp == NULL)
		return;

	mtx_lock_spin(&pp->pp_pmu_lock);
	LIST_FOREACH(pg, &pp->pp_pmu_groups, pg_proc_next)
		pmu_group_accounting_block(pg);
	mtx_unlock_spin(&pp->pp_pmu_lock);

	/*
	 * Tear down the rotation kthread first.  It holds pp as its
	 * arg, so once pp is freed the next muxrot tick would
	 * dereference freed memory.  The sx_sleep below transiently
	 * drops pmc_sx; the kthread acquires it, observes
	 * pp_pmu_rot_running == false, clears pp_pmu_rot_td, and
	 * exits.
	 */
	pmu_pp_stop_rotate(pp, "muxpurge");
	while (pp->pp_pmu_refs != 0)
		(void)hwpmc_pmu_sx_sleep(&pp->pp_pmu_refs, 1,
		    "muxrefs");

	/*
	 * Schedule out every group that is currently bound to pp, then
	 * unhook every pg from pp->pp_pmu_groups regardless of state.
	 * pg_pp gets cleared so a later pmu_group_on_release for any
	 * surviving sibling skips the now-stale LIST_REMOVE.
	 */
	pp->pp_pmu_rot_cursor = NULL;
	for (;;) {
		mtx_lock_spin(&pp->pp_pmu_lock);
		pg = LIST_FIRST(&pp->pp_pmu_groups);
		mtx_unlock_spin(&pp->pp_pmu_lock);
		if (pg == NULL)
			break;
		pmu_group_accounting_drain(pg, true);
		if (pg->pg_assigned)
			pmu_pp_schedule_out(pp, pg, false);
		mtx_lock_spin(&pp->pp_pmu_lock);
		LIST_REMOVE(pg, pg_proc_next);
		pg->pg_pp = NULL;
		pg->pg_attach_proc = NULL;
		mtx_unlock_spin(&pp->pp_pmu_lock);
	}
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
	pmu_group_t *pg;
	int n_running, n_deferred;
	int error;

	if (pp == NULL || pp->pp_pmu_rot_running ||
	    pp->pp_pmu_rot_td != NULL || pp->pp_pmu_rot_quiesce != 0 ||
	    pp->pp_pmu_unhashed)
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
	pmu_group_t *pg, *cursor;
	u_int ngroups, seen;
	int sin_err;
	bool found_cursor, need_rotate;

	if (pp == NULL || LIST_EMPTY(&pp->pp_pmu_groups))
		return;

	/*
	 * Pre-check: if no running group is currently deferred there
	 * is nothing to rotate.  Skip the evict-and-rebind cycle
	 * entirely so that the kthread, once awake, costs us nothing
	 * when oversubscription has subsided (e.g., the user stopped
	 * one of two groups).
	 */
	need_rotate = false;
	LIST_FOREACH(pg, &pp->pp_pmu_groups, pg_proc_next) {
		if (pg->pg_running && !pg->pg_assigned) {
			need_rotate = true;
			break;
		}
	}
	if (!need_rotate)
		return;

	/* Phase 1: evict every currently scheduled group. */
	LIST_FOREACH(pg, &pp->pp_pmu_groups, pg_proc_next) {
		if (pg->pg_assigned) {
			PMCDBG3(PMC, OPS, 4,
			    "rotate: pp=%p evict gid=%u nevents=%u",
			    pp, pg->pg_id, pg->pg_nevents);
			pmu_pp_schedule_out(pp, pg, true);
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
			PMCDBG4(PMC, OPS, 4,
			    "rotate: pp=%p schedule_in gid=%u "
			    "nevents=%u -> %d",
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

/*
 * ============================================================
 * System-wide (PMC_MODE_SC) grouping + per-CPU multiplex.
 * ============================================================
 *
 * The functions above schedule groups onto a target process and
 * multiplex them with a per-pp kthread.  The mirror image below
 * schedules system-wide groups onto a single CPU and multiplexes them
 * with a per-CPU kthread.  The structural logic is identical (atomic
 * all-or-none schedule in/out, fair round-robin rotation with a moving
 * cursor); the only real difference is HOW the hardware is touched:
 *
 *   - process mode programs the counter lazily at csw_in time via
 *     pp_pmcs[] / pmc_rotation_attach;
 *
 *   - system mode programs the counter eagerly here, because a
 *     system-wide PMC is never context switched -- it just runs on its
 *     bound CPU until stopped.  hwpmc_pmu_sys_start_row/stop_row do the
 *     pmc_select_cpu() bind dance and carry the cumulative count in
 *     pm->pm_gv.pm_savedvalue so values are continuous across windows.
 */

/* Enter and leave SC residency at the final sibling's hardware edge. */
void
pmu_group_sys_row_started(struct pmc *pm)
{
	pmu_event_t *pe;
	pmu_group_t *pg;

	pe = pmu_event_from_pmc(pm);
	if (pe == NULL || pe->pe_group == NULL ||
	    TAILQ_NEXT(pe, pe_sibling) != NULL)
		return;
	pg = pe->pe_group;
	mtx_pool_lock_spin(pmc_mtxpool, pg);
	pmu_group_time_update_locked(pg, cpu_ticks());
	pg->pg_account_placement_admit = true;
	mtx_pool_unlock_spin(pmc_mtxpool, pg);
}

void
pmu_group_sys_row_stopped(struct pmc *pm)
{
	pmu_event_t *pe;
	pmu_group_t *pg;

	pe = pmu_event_from_pmc(pm);
	if (pe == NULL || pe->pe_group == NULL ||
	    TAILQ_NEXT(pe, pe_sibling) != NULL)
		return;
	pg = pe->pe_group;
	mtx_pool_lock_spin(pmc_mtxpool, pg);
	pmu_group_time_update_locked(pg, cpu_ticks());
	pg->pg_account_placement_admit = false;
	mtx_pool_unlock_spin(pmc_mtxpool, pg);
}

/*
 * Atomically schedule a system group in on its CPU: reserve a HW row
 * for every sibling (all-or-none) and program+start each one.  Returns
 * 0 on success, ENOSPC if the group does not currently fit (it stays
 * deferred for the rotation kthread), or another errno on hard failure.
 * Caller holds pmc_sx exclusive.
 */
static int
pmu_sys_schedule_in(int cpu, pmu_group_t *pg)
{
	pmu_event_t *pe;
	struct proc *owner;
	int error;

	if (pg == NULL || pg->pg_assigned)
		return (0);

	/*
	 * The owner proc (e.g. pmcstat) is used ONLY for the
	 * pmc_can_allocate_rowindex() bookkeeping check inside the
	 * assigner -- system rows are bound to pg_cpu, not to a process.
	 */
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

	TAILQ_FOREACH(pe, &pg->pg_events, pe_sibling) {
		if (pe->pe_state != PMU_EVENT_STATE_ACTIVE)
			continue;
		pe->pe_pmc->pm_state = PMC_STATE_RUNNING;
		hwpmc_pmu_sys_start_row(cpu, pe->pe_pmc);
	}
	PMCDBG3(PMC, OPS, 4, "sys_schedule_in: gid=%u cpu=%d nevents=%u IN",
	    pg->pg_id, cpu, pg->pg_nevents);
	return (0);
}

/*
 * Atomically schedule a system group out: stop+read every sibling's HW
 * (folding the window count into pm_gv.pm_savedvalue) and release every
 * row in one pass.  Caller holds pmc_sx exclusive.
 */
static void
pmu_sys_schedule_out(int cpu, pmu_group_t *pg)
{
	pmu_event_t *pe;

	if (pg == NULL || !pg->pg_assigned)
		return;

	TAILQ_FOREACH(pe, &pg->pg_events, pe_sibling) {
		if (pe->pe_state != PMU_EVENT_STATE_ACTIVE)
			continue;
		hwpmc_pmu_sys_stop_row(cpu, pe->pe_pmc);
		pe->pe_pmc->pm_state = PMC_STATE_STOPPED;
	}
	pmu_unassign_group(pg, cpu);
	PMCDBG2(PMC, OPS, 4, "sys_schedule_out: gid=%u cpu=%d OUT",
	    pg->pg_id, cpu);
}

/*
 * pmc_start() entry point for a system-wide group sibling.  Registers
 * the group on its CPU's rotation list, drives the initial HW placement
 * (first sibling only -- the rest are no-ops because the group is
 * already running), and starts the per-CPU rotation kthread if the CPU
 * is oversubscribed.  Idempotent across siblings.
 */
int
pmu_sys_group_on_start(struct pmc *pm)
{
	pmu_event_t *pe;
	pmu_group_t *pg;
	bool was_running;
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

	if (!pg->pg_sys_listed) {
		LIST_INSERT_HEAD(&pmu_syscpu[cpu].sc_groups, pg, pg_proc_next);
		pg->pg_sys_listed = true;
	}

	mtx_pool_lock_spin(pmc_mtxpool, pg);
	if (pg->pg_account_blocked) {
		mtx_pool_unlock_spin(pmc_mtxpool, pg);
		return (EBUSY);
	}
	was_running = pg->pg_running;
	pmu_group_running_start_locked(pg, cpu_ticks());
	mtx_pool_unlock_spin(pmc_mtxpool, pg);
	if (!was_running) {
		int sin_err __unused;

		sin_err = pmu_sys_schedule_in(cpu, pg);
		PMCDBG3(PMC, OPS, 2,
		    "sys_on_start: gid=%u cpu=%d schedule_in=%d",
		    pg->pg_id, cpu, sin_err);
	}

	TAILQ_FOREACH(pe, &pg->pg_events, pe_sibling)
		pe->pe_pmc->pm_state = PMC_STATE_RUNNING;

	pmu_syscpu_kick_rotate(cpu);
	return (0);
}

/*
 * pmc_stop() entry point for a system-wide group sibling.  Takes the
 * whole group off hardware (freezing its cumulative count in
 * pm_gv.pm_savedvalue) and frees its rows so a peer can use them.  The
 * group stays on the CPU's rotation list; the rotation kthread skips it
 * because pg_running is now false.  A subsequent pmc_start re-binds it.
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

	TAILQ_FOREACH(pe, &pg->pg_events, pe_sibling)
		pe->pe_pmc->pm_state = PMC_STATE_STOPPED;
}

static void
pmu_syscpu_stop_rotate(struct pmu_syscpu *sc, const char *wmesg)
{

	sc->sc_quiesce++;
	sc->sc_running = false;
	wakeup(sc);
	while (sc->sc_td != NULL)
		(void)hwpmc_pmu_sx_sleep(&sc->sc_td, 1, wmesg);
	KASSERT(sc->sc_quiesce > 0,
	    ("[pmu] system rotation quiesce underflow"));
	sc->sc_quiesce--;
}

/*
 * Called at the very top of pmc_release_pmc_descriptor(), BEFORE the
 * framework touches the row, for any system-wide group sibling.  Stops
 * per-CPU rotation, schedules the whole group out so every sibling becomes
 * UNASSIGNED, and unlinks the group from the CPU's rotation list.  After
 * this returns the framework sees PMC_ROW_IS_UNASSIGNED() for the
 * releasing pm and takes its deferred-release path, so it never
 * double-releases a HW row we already freed.  Idempotent across the
 * group's siblings.  Caller holds pmc_sx exclusive.
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

	/* Stop rotation before changing the CPU's group set. */
	if (pg->pg_sys_listed)
		pmu_syscpu_stop_rotate(sc, "muxsrel");

	if (pg->pg_assigned)
		pmu_sys_schedule_out(cpu, pg);
	mtx_pool_lock_spin(pmc_mtxpool, pg);
	pmu_group_running_stop_locked(pg, cpu_ticks());
	mtx_pool_unlock_spin(pmc_mtxpool, pg);

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
 * Spawn the per-CPU rotation kthread the first time a CPU is
 * oversubscribed (>= 2 running groups, at least one of them deferred).
 * Idempotent: returns immediately once the kthread is up.
 */
static void
pmu_syscpu_kick_rotate(int cpu)
{
	struct pmu_syscpu *sc;
	pmu_group_t *pg;
	int n_running, n_deferred, error;

	sc = &pmu_syscpu[cpu];
	if (sc->sc_running || sc->sc_td != NULL || sc->sc_quiesce != 0)
		return;

	n_running = n_deferred = 0;
	LIST_FOREACH(pg, &sc->sc_groups, pg_proc_next) {
		if (!pg->pg_running)
			continue;
		n_running++;
		if (!pg->pg_assigned)
			n_deferred++;
	}
	if (n_deferred == 0 || n_running < 2)
		return;

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
 * Run one rotation window for a CPU.  Identical fair round-robin
 * algorithm as pmu_pp_rotate_one(), keyed on the per-CPU group list:
 * evict everything, then schedule groups back in starting at the cursor
 * until one no longer fits (that group pins the next tick's cursor).
 * Caller holds pmc_sx exclusive.
 */
static void
pmu_syscpu_rotate_one(int cpu)
{
	struct pmu_syscpu *sc;
	pmu_group_t *pg, *cursor;
	bool found_cursor, need_rotate;
	u_int ngroups, seen;
	int sin_err;

	sc = &pmu_syscpu[cpu];
	if (LIST_EMPTY(&sc->sc_groups))
		return;

	need_rotate = false;
	LIST_FOREACH(pg, &sc->sc_groups, pg_proc_next) {
		if (pg->pg_running && !pg->pg_assigned) {
			need_rotate = true;
			break;
		}
	}
	if (!need_rotate)
		return;

	/* Phase 1: evict every currently scheduled group. */
	LIST_FOREACH(pg, &sc->sc_groups, pg_proc_next) {
		if (pg->pg_assigned)
			pmu_sys_schedule_out(cpu, pg);
	}

	/* Phase 2 setup: validate the cursor is still on the list. */
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

	/* Phase 2: round-robin schedule_in from cursor, wrap once. */
	pg = cursor;
	seen = 0;
	while (seen < ngroups) {
		if (pg == NULL)
			pg = LIST_FIRST(&sc->sc_groups);
		if (pg->pg_running && !pg->pg_assigned) {
			sin_err = pmu_sys_schedule_in(cpu, pg);
			if (sin_err == ENOSPC) {
				sc->sc_cursor = pg;
				return;
			}
		}
		seen++;
		pg = LIST_NEXT(pg, pg_proc_next);
	}

	sc->sc_cursor = LIST_NEXT(cursor, pg_proc_next);
	if (sc->sc_cursor == NULL)
		sc->sc_cursor = LIST_FIRST(&sc->sc_groups);
}

/*
 * Per-CPU rotation kthread.  Wakes every kern.hwpmc.mux_period_ms, runs
 * one rotation tick, and goes back to sleep.  Exits when the last group
 * leaves the CPU (pmu_sys_group_pre_release clears sc_running and wakes
 * us).
 */
static void
pmu_syscpu_rotate_thread(void *arg)
{
	int cpu = (int)(intptr_t)arg;
	struct pmu_syscpu *sc = &pmu_syscpu[cpu];
	int period_ticks;

	hwpmc_pmu_sx_xlock();
	while (sc->sc_running) {
		period_ticks = (pmu_mux_period_ms * hz) / 1000;
		if (period_ticks < 1)
			period_ticks = 1;
		(void)hwpmc_pmu_sx_sleep(sc, period_ticks, "muxsys");
		if (!sc->sc_running)
			break;
		pmu_syscpu_rotate_one(cpu);
	}
	sc->sc_td = NULL;
	wakeup(&sc->sc_td);
	hwpmc_pmu_sx_xunlock();
	kthread_exit();
}
