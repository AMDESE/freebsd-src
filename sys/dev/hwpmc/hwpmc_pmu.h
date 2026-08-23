/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Advanced Micro Devices, Inc.
 *
 * PMU event grouping and multiplexing interface.
 * Defers hardware row allocation until group start.
 */

#ifndef _DEV_HWPMC_PMU_H_
#define	_DEV_HWPMC_PMU_H_

#include <sys/mutex.h>
#include <sys/pmc.h>
#include <sys/queue.h>

#define	PMC_ROW_IS_UNASSIGNED(P)	(PMC_TO_ROWINDEX(P) == PMC_ROW_UNASSIGNED)

#ifdef _KERNEL

/*
 * Flags for process and thread descriptor lookups.
 */
enum pmc_flags {
	PMC_FLAG_NONE	  = 0x00, /* do nothing */
	PMC_FLAG_REMOVE   = 0x01, /* atomically remove entry from hash */
	PMC_FLAG_ALLOCATE = 0x02, /* add entry to hash if not found */
	PMC_FLAG_NOWAIT   = 0x04, /* do not wait for mallocs */
};

#ifdef HWPMC_DEBUG
/*
 * Test-only live-object and reference counters.  These are internal to the
 * hwpmc module and are exposed only by an HWPMC_DEBUG kernel.
 */
enum pmc_test_counter {
	PMC_TEST_LIVE_PMC_TARGETS,
	PMC_TEST_LIVE_GROUP_TARGETS,
	PMC_TEST_LIVE_TARGET_PROCESSES,
	PMC_TEST_LIVE_RESIDUAL_ENTRIES,
	PMC_TEST_LIVE_ROTATION_REFS,
	PMC_TEST_LIVE_RUN_REFS,
	PMC_TEST_COUNTER_COUNT
};

enum pmc_test_failpoint {
	PMC_TEST_FAIL_GROUP_TARGET_ALLOC,
	PMC_TEST_FAIL_TARGET_LINK_ALLOC,
	PMC_TEST_FAIL_ATTACH_AUTHORIZATION,
	PMC_TEST_FAIL_SYSTEM_START,
	PMC_TEST_FAILPOINT_COUNT
};

void	pmc_test_counter_add(enum pmc_test_counter counter, int64_t delta);
bool	pmc_test_should_fail(enum pmc_test_failpoint failpoint);
bool	pmc_test_group_hold_resident(pmu_group_t *pg);
bool	pmc_test_group_hold_evicted(pmu_group_t *pg);
void	pmc_test_descendants_attach_pause(pmu_group_t *pg, u_int ntargets);
void	pmc_test_descendants_exit_observed(struct proc *p);
void	pmc_test_system_start_pause(pmu_group_t *pg, u_int nstarted);
void	pmc_test_sample_accepted(struct pmc *pm, uint32_t pid, uint32_t tid);
bool	pmc_test_sample_worker_paused(struct pmc *pm);
void	pmc_test_sample_schedule_out(pmu_group_t *pg);
bool	pmc_test_callchain_log_should_fail(struct pmc *pm);
void	pmc_test_sample_dropped(struct pmc *pm);
void	pmc_test_sample_done(struct pmc *pm, bool emitted);
#else
#define	pmc_test_counter_add(counter, delta)	do { } while (0)
#define	pmc_test_should_fail(failpoint)		(false)
#define	pmc_test_group_hold_resident(pg)	(false)
#define	pmc_test_group_hold_evicted(pg)		(false)
#define	pmc_test_descendants_attach_pause(pg, n)	do { } while (0)
#define	pmc_test_descendants_exit_observed(p)	do { } while (0)
#define	pmc_test_system_start_pause(pg, n)	do { } while (0)
#define	pmc_test_sample_accepted(pm, pid, tid)	do { } while (0)
#define	pmc_test_sample_worker_paused(pm)	(false)
#define	pmc_test_sample_schedule_out(pg)		do { } while (0)
#define	pmc_test_callchain_log_should_fail(pm)	(false)
#define	pmc_test_sample_dropped(pm)		do { } while (0)
#define	pmc_test_sample_done(pm, emitted)	do { } while (0)
#endif

/*
 * Scheduling constraint from the class backend.
 * pc_allowed_rows is a bitmask of valid class row indices.
 */
#define	PMC_SC_F_FIXED		0x0001	/* must use pc_fixed_row */
#define	PMC_SC_F_SHARED		0x0004	/* tolerates sharing siblings */

struct pmc_sched_constraint {
	uint32_t	pc_allowed_rows;	/* bitmask of legal rows */
	uint8_t		pc_weight;		/* popcount(pc_allowed_rows) */
	uint8_t		pc_fixed_row;		/* used iff PMC_SC_F_FIXED */
	uint16_t	pc_flags;		/* PMC_SC_F_* */
};
typedef struct pmc_sched_constraint pmc_sched_constraint_t;

/* Event wrapper for group allocation state. */
struct pmu_event {
	TAILQ_ENTRY(pmu_event)		pe_sibling;
	pmu_group_t			*pe_group;
	struct pmc			*pe_pmc;
	counter_u64_t			pe_samples;
#ifdef HWPMC_DEBUG
	counter_u64_t			pe_samples_emitted;
	counter_u64_t			pe_samples_dropped;
	volatile u_int			pe_test_sample_pid;
	volatile u_int			pe_test_sample_tid;
#endif
	bool				pe_is_leader;
	struct pmc_op_pmcallocate	pe_alloc;
	pmc_sched_constraint_t		pe_cons;
	/*
	 * System-mode progress toward the next sample, kept across eviction.
	 * Virtual per-thread state is stored in pp_pmu_residuals instead.
	 * Zero means "start from the reload count".
	 */
	pmc_value_t			pe_residual;
};

/*
 * Explicit saved-residual state.  A raw count of zero is ambiguous: a
 * down-counting sampling PMC reads zero both when it has overflowed and when
 * it has no saved progress.  The state is stored separately so the two are
 * never inferred from the value alone.
 */
enum pmu_residual_state {
	PMU_RESIDUAL_UNINITIALIZED = 0,	/* no saved progress; full reload */
	PMU_RESIDUAL_COUNT_VALID = 1,	/* ptr_value is a partial count */
	PMU_RESIDUAL_OVERFLOW_PENDING = 2, /* overflow seen at schedule-out */
};

/*
 * Saved virtual-thread sampling progress.  Hardware rows are transient, so
 * the stable key is the event plus target TID within one process descriptor.
 * The containing process's pp_tdslock protects this list and its entries.
 */
struct pmu_thread_residual {
	LIST_ENTRY(pmu_thread_residual)	ptr_next;
	pmu_event_t			*ptr_event;
	lwpid_t				ptr_tid;
	pmc_value_t			ptr_value;
	uint8_t				ptr_state;	/* pmu_residual_state */
};

/*
 * One authoritative group-target edge.  Explicit and inherited targets use
 * the same representation.  Hardware-row links are transient and are not
 * stored here.
 *
 * Target-list mutations require pmc_sx exclusive.  When a process-tree
 * snapshot is also needed, lock order is:
 *
 *     pmc_sx -> proctree_lock -> pp_pmu_lock
 */
struct pmu_group_target {
	LIST_ENTRY(pmu_group_target)	pgt_group_next;	/* pg_targets */
	LIST_ENTRY(pmu_group_target)	pgt_process_next; /* pp_pmu_targets */
	pmu_group_t			*pgt_group;
	struct pmc_process		*pgt_pp;
	bool				pgt_explicit;
};

struct pmu_group_cpu_state {
	LIST_ENTRY(pmu_group_cpu_state) pgcs_next;
	pmu_group_t			*pgcs_group;
	struct thread			*pgcs_td;
	bool				pgcs_counted;
	bool				pgcs_placed;
	bool				pgcs_transitioning;
};

struct pmu_group_time_snapshot {
	uint64_t	pgts_enabled;
	uint64_t	pgts_running;
	uint64_t	pgts_enabled_wall;
	uint64_t	pgts_wall;
	bool		pgts_system;
};

/*
 * PMU event group for atomic scheduling.
 * All sibling events are assigned together or deferred together.
 */
struct pmu_group {
	LIST_ENTRY(pmu_group)		pg_owner_next;
	LIST_ENTRY(pmu_group)		pg_proc_next;
	uint32_t			pg_id;
	struct pmc_owner		*pg_owner;
	pmu_event_t			*pg_leader;
	TAILQ_HEAD(, pmu_event)		pg_events;
	u_int				pg_nevents;
	bool				pg_committed;
	bool				pg_assigned;	/* SCHEDULED if true */
	bool				pg_running;	/* between start/stop */
	bool				pg_defer_ok;	/* PMC_F_GROUP_MUX hint */
	struct pmu_group_target		*pg_explicit_target;
	struct pmc_process		*pg_pp;	/* current scheduling anchor */
	/*
	 * Authoritative target set.  pg_explicit_target, when non-NULL, points
	 * to the user-requested edge in this list.  pg_pp is only the current
	 * scheduling/rotation anchor and may identify an inherited target.
	 */
	LIST_HEAD(, pmu_group_target)	pg_targets;
	/* Virtual enabled and running times use thread ticks; wall uses TSC. */
	uint64_t			pg_time_enabled_ticks;
	uint64_t			pg_time_running_ticks;
	uint64_t			pg_enabled_wall_ticks;
	uint64_t			pg_wall_start_ticks;
	uint64_t			pg_wall_ticks;
	uint64_t			pg_timestamp_ticks;
	uint64_t			pg_tickrate;
	struct mtx			pg_snapshot_lock;
	u_int				pg_oncpu_threads;
	u_int				pg_running_threads;
	struct pmu_group_cpu_state	*pg_cpu_state;
	u_int				pg_ncpu;
	bool				pg_account_blocked;
	bool				pg_account_placement_admit;
	bool				pg_snapshot_pending;
	bool				pg_snapshot_active;
	bool				pg_releasing;
	/*
	 * System-mode (PMC_MODE_SC) state.
	 * System groups bind to one CPU and rotate on that CPU.
	 */
	bool				pg_system;	/* system-wide group */
	bool				pg_sys_listed;	/* on pmu_syscpu list */
	bool				pg_sscounted;	/* in po_sscount */
	int				pg_cpu;		/* bound CPU (system) */
};

/*
 * Core hooks from hwpmc_mod.c.
 */
struct pmc_mdep *hwpmc_get_mdep(void);
struct pmc_classdep *hwpmc_ri_to_classdep(int ri, int *adjri);
bool hwpmc_can_allocate_row(int ri, enum pmc_mode mode);
bool hwpmc_can_allocate_rowindex(struct proc *p, unsigned int ri, int cpu);
void hwpmc_mark_row_thread(int ri);
void hwpmc_unmark_row_thread(int ri);
void hwpmc_mark_row_standalone(int ri);
void hwpmc_unmark_row_standalone(int ri);
bool hwpmc_row_is_unallocated(int cpu, int ri);
void hwpmc_unconfigure_row_all_cpus(struct pmc *pm, int ri);

/*
 * Start and stop system-mode PMC hardware on the target CPU.
 * Values accumulate across multiplex windows.
 */
int hwpmc_pmu_sys_start_row(int cpu, struct pmc *pm);
void hwpmc_sscount_add(struct pmc_owner *po);
void hwpmc_sscount_sub(struct pmc_owner *po);
void hwpmc_pmu_sys_stop_row(int cpu, struct pmc *pm);

/*
 * Hardware row assigner (hwpmc_assign.c).
 */
int pmu_assign_group(pmu_group_t *pg, struct proc *p, int cpu);
void pmu_unassign_group(pmu_group_t *pg, int cpu);
int pmu_group_can_fit(pmu_group_t *pg);
int pmu_group_can_place(pmu_group_t *pg, struct proc *p, int cpu);
int pmu_group_satisfiable(pmu_group_t *pg, struct proc *p, int cpu,
    uint64_t evictable_rows);
int pmu_validate_group(pmu_group_t *pg);
int pmu_event_get_constraint(pmu_event_t *pe,
    pmc_sched_constraint_t *cons);
bool pmu_class_supports_grouping(enum pmc_class class);

/*
 * Group lifecycle (hwpmc_pmu.c).
 */
void pmu_group_create(struct pmc_owner *po, uint32_t *pg_id);
int pmu_group_add(pmu_group_t *pg, struct pmc *pm, bool leader);
int pmu_group_commit(pmu_group_t *pg);
pmu_group_t *pmu_group_lookup(struct pmc_owner *po, uint32_t pg_id);
u_int pmu_group_prepare_release(pmu_group_t *pg, struct pmc **members,
    u_int capacity, struct pmc_process **released_pp);
void pmu_group_release(pmu_group_t *pg);
void pmu_group_accounting_initialize(void);
void pmu_group_accounting_finalize(void);
void pmu_group_time_snapshot_locked(pmu_group_t *pg,
    struct pmu_group_time_snapshot *snapshot, uint64_t now);
void pmu_event_destroy(pmu_event_t *pe);
pmu_event_t *pmu_event_from_pmc(struct pmc *pm);
pmu_group_t *pmu_group_from_pmc(struct pmc *pm);
u_int pmu_group_inherit(struct pmc_process *ppold, struct pmc_process *ppnew,
    struct pmc **missed_leaders, u_int missed_capacity, u_int *nmissed);
bool pmu_group_follows_fork(pmu_group_t *pg);
void pmu_group_disinherit(struct pmc_process *pp);
struct pmu_group_target *pmu_group_target_alloc(pmu_group_t *pg,
    struct pmc_process *pp, bool explicit, int malloc_flags);
void pmu_group_target_publish(struct pmu_group_target *pgt);
void pmu_group_target_discard(struct pmu_group_target *pgt);
void pmu_group_target_remove(struct pmu_group_target *pgt);
void pmu_event_save_residual(struct pmc *pm, struct pmc_process *pp, int ri);
void pmu_event_set_residual(struct pmc *pm, pmc_value_t residual);
void pmu_event_restore_thread_residual(struct pmc *pm,
    struct pmc_process *pp, int ri);
pmc_value_t pmu_event_restore_residual(struct pmc *pm);
void pmu_thread_residual_remove(struct pmc_process *pp, lwpid_t tid);
#ifdef HWPMC_DEBUG
int pmu_event_get_thread_residual(struct pmc *pm, struct pmc_process *pp,
    lwpid_t tid, pmc_value_t *value, uint8_t *state);
int pmu_event_set_thread_residual(struct pmc *pm, struct pmc_process *pp,
    lwpid_t tid, pmc_value_t value, uint8_t state);
#endif

int pmu_group_on_allocate(struct pmc *pm, const struct pmc_op_pmcallocate *pa);
int pmu_group_on_attach(struct pmc *pm, struct proc *p);
int pmu_group_on_start(struct pmc *pm);
void pmu_group_on_stop(struct pmc *pm);
void pmu_group_on_release(struct pmc *pm);
void pmu_group_csw_in(struct thread *td, struct pmc_process *pp);
void pmu_group_csw_in_complete(struct thread *td, int cpu);
bool pmu_group_csw_can_start(struct pmc *pm, struct thread *td, int cpu);
void pmu_group_csw_out(struct thread *td, int cpu);
void pmu_group_csw_out_complete(struct thread *td, int cpu);

/*
 * System-mode (PMC_MODE_SC) group lifecycle functions.
 */
int pmu_sys_group_on_start(struct pmc *pm);
void pmu_sys_group_on_stop(struct pmc *pm);
void pmu_sys_group_pre_release(struct pmc *pm);

/*
 * Process descriptor PMU lifecycle functions.
 * Caller must hold pmc_sx exclusive before pmu_pp_destroy().
 */
void pmu_pp_init(struct pmc_process *pp);
void pmu_pp_destroy(struct pmc_process *pp);
void pmu_group_detach_target(pmu_group_t *pg, struct pmc_process *pp);
void pmu_group_detach_inherited_target(pmu_group_t *pg, struct pmc_process *pp);
void pmu_pp_kick_after_exec(struct pmc_process *pp);

/*
 * Find a process descriptor for the PMU layer.
 */
struct pmc_process *pmc_find_process_descriptor_pmu(struct proc *p,
    uint32_t mode);

void hwpmc_pmu_force_context_switch(void);

/*
 * Rotation helper functions (hwpmc_mod.c).
 */
void pmc_rotation_drain_set(struct pmc *pm, bool drain);
void pmc_rotation_drain(struct pmc *pm);
void pmc_rotation_drain_cpu(struct pmc *pm, int cpu);
void pmc_rotation_detach(struct pmc *pm, struct pmc_process *pp);
void pmc_rotation_attach(struct pmc *pm, struct pmc_process *pp);
struct pmc_target *pmc_target_object_alloc(int malloc_flags,
    bool inject_failure);
void pmc_target_object_free(struct pmc_target *pt);
void pmc_link_target_process_preallocated(struct pmc *pm,
    struct pmc_process *pp, struct pmc_target *pt);

#endif /* _KERNEL */

#endif /* _DEV_HWPMC_PMU_H_ */
