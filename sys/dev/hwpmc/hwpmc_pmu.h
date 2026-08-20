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
	bool				pe_is_leader;
	struct pmc_op_pmcallocate	pe_alloc;
	pmc_sched_constraint_t		pe_cons;
	/*
	 * Progress toward the next sample, kept across eviction.  Per-thread
	 * state lives in pt_pmcs[], which is indexed by hardware row, and a
	 * member does not keep its row across a rotation; this is keyed to
	 * the member instead.  Zero means "start from the reload count".
	 */
	pmc_value_t			pe_residual;
};

/*
 * A process a group follows through fork, beyond its explicit target.
 * The record is on two lists: the group's, to find every target, and the
 * process's, to find every group a descendant follows.
 */
struct pmu_group_target {
	LIST_ENTRY(pmu_group_target)	pgt_next;	/* pg_inherited */
	LIST_ENTRY(pmu_group_target)	pgt_pp_next;	/* pp_pmu_inherited */
	pmu_group_t			*pgt_group;
	struct pmc_process		*pgt_pp;
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
	struct proc			*pg_attach_proc;
	struct pmc_process		*pg_pp;	/* target process descriptor */
	/*
	 * Targets inherited through fork (spec §3.9).  The explicit target
	 * above anchors rotation, placement and release; these accumulate
	 * into the same member totals and are detached with the group.
	 */
	LIST_HEAD(, pmu_group_target)	pg_inherited;
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
void hwpmc_pmu_sys_start_row(int cpu, struct pmc *pm);
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
    u_int *nmissed);
void pmu_group_disinherit(struct pmc_process *pp);
void pmu_event_save_residual(struct pmc *pm, struct pmc_process *pp, int ri);
void pmu_event_set_residual(struct pmc *pm, pmc_value_t residual);
void pmu_event_restore_thread_residual(struct pmc *pm,
    struct pmc_process *pp, int ri);
pmc_value_t pmu_event_restore_residual(struct pmc *pm);

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
void pmc_rotation_drain(struct pmc *pm);
void pmc_rotation_detach(struct pmc *pm, struct pmc_process *pp);
void pmc_rotation_attach(struct pmc *pm, struct pmc_process *pp);

#endif /* _KERNEL */

#endif /* _DEV_HWPMC_PMU_H_ */
