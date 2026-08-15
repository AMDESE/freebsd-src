/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Advanced Micro Devices, Inc.
 *
 * hwpmc PMU grouping/multiplex layer.  This is a lightweight virtual
 * layer that lets multiple events be programmed atomically (group)
 * and, when there are not enough hardware rows, time-shared
 * (multiplex).  It does NOT replace the existing pcd_allocate_pmc /
 * pcd_release_pmc / pcd_start_pmc / pcd_stop_pmc paths; it only defers
 * those calls until the group is started, and drives them via a
 * sorted, constraint-aware assigner.
 */

#ifndef _DEV_HWPMC_PMU_H_
#define	_DEV_HWPMC_PMU_H_

#include <sys/mutex.h>
#include <sys/pmc.h>
#include <sys/queue.h>

#define	PMC_ROW_IS_UNASSIGNED(P)	(PMC_TO_ROWINDEX(P) == PMC_ROW_UNASSIGNED)

#ifdef _KERNEL

/*
 * Mode flags shared by the hwpmc process/thread descriptor lookup
 * helpers.  Originally local to hwpmc_mod.c; promoted to this header
 * so the PMU grouping layer (hwpmc_pmu.c) can use the same names.
 */
enum pmc_flags {
	PMC_FLAG_NONE	  = 0x00, /* do nothing */
	PMC_FLAG_REMOVE   = 0x01, /* atomically remove entry from hash */
	PMC_FLAG_ALLOCATE = 0x02, /* add entry to hash if not found */
	PMC_FLAG_NOWAIT   = 0x04, /* do not wait for mallocs */
};

/*
 * Scheduling constraint emitted by the per-class backend.  pc_allowed_rows
 * is a bitmask over the backend-defined row index namespace ("row 0..N-1
 * for this class on this CPU"), not a raw MSR/RDPMC number.
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

/* A pmu_event wraps one PMC with its immutable allocation state. */
struct pmu_event {
	TAILQ_ENTRY(pmu_event)		pe_sibling;
	pmu_group_t			*pe_group;
	struct pmc			*pe_pmc;
	counter_u64_t			pe_samples;
	bool				pe_is_leader;
	struct pmc_op_pmcallocate	pe_alloc;
	pmc_sched_constraint_t		pe_cons;
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
 * pmu_group is the unit of all-or-none scheduling.  Either every
 * sibling is bound to a HW row and attached to the target process
 * (pg_assigned == true == "scheduled in") or none of them is (pg_assigned
 * == false == "deferred").  When the union of all groups attached to
 * one pp exceeds the HW counter budget the per-pp rotation kthread
 * evicts the front-most scheduled group atomically and tries to bring
 * in any deferred group that now fits.  Within a group placement is
 * never partial -- every sibling is on a HW row whenever pg_assigned.
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
	struct pmc_process		*pg_pp;	/* TARGET pp we hung off
						 * pp_pmu_groups; cleared on
						 * release. */
	/* Virtual enabled/running are thread-ticks; wall fields stay wall-ticks. */
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
	 * System-wide (PMC_MODE_SC) group state.  Process-mode groups
	 * hang off a pmc_process (pg_pp) and rotate in a per-pp kthread;
	 * system-mode groups are bound to a single CPU and rotate in a
	 * per-CPU kthread instead (see the pmu_syscpu[] registry in
	 * hwpmc_pmu.c).  pg_system selects which world a group lives in.
	 */
	bool				pg_system;	/* system-wide group */
	bool				pg_sys_listed;	/* on pmu_syscpu list */
	int				pg_cpu;		/* bound CPU (system) */
};

/*
 * Hooks exported by hwpmc_mod.c so that hwpmc_pmu.c and hwpmc_assign.c
 * stay file-local for the rest of the hwpmc module.
 */
struct pmc_mdep *hwpmc_get_mdep(void);
struct pmc_classdep *hwpmc_ri_to_classdep(int ri, int *adjri);
bool hwpmc_can_allocate_row(int ri, enum pmc_mode mode);
bool hwpmc_can_allocate_rowindex(struct proc *p, unsigned int ri, int cpu);
void hwpmc_mark_row_thread(int ri);
void hwpmc_unmark_row_thread(int ri);
void hwpmc_mark_row_standalone(int ri);
void hwpmc_unmark_row_standalone(int ri);

/*
 * Program / unprogram one system-wide PMC row's hardware on its bound
 * CPU.  Both helpers do the pmc_select_cpu() bind dance internally.
 * start_row zeroes the hardware window and stop_row folds that window
 * into the 64-bit pm_gv.pm_savedvalue software total.  This keeps counts
 * continuous across multiplex windows without narrowing the cumulative
 * value to the hardware width.
 */
void hwpmc_pmu_sys_start_row(int cpu, struct pmc *pm);
void hwpmc_pmu_sys_stop_row(int cpu, struct pmc *pm);

/*
 * The PMU grouping/multiplex scheduler is architecture-independent:
 * hwpmc_pmu.c and hwpmc_assign.c are compiled on every platform.  The
 * only machine-specific piece is the per-event scheduling constraint,
 * which each PMC class supplies through the optional
 * pcd_get_sched_constraint / pcd_can_assign_pmc hooks of its
 * pmc_classdep (implemented for AMD in hwpmc_amd.c).  A class that does
 * not implement them leaves those pointers NULL, and the wrappers in
 * hwpmc_assign.c report the feature unsupported (EOPNOTSUPP) so a group
 * simply fails to commit on such a class -- no per-architecture #ifdefs
 * and no per-class checks anywhere in the grouping code.
 */

/*
 * Assigner (hwpmc_assign.c).  Scheduling is strictly all-or-none at
 * the group level: pmu_assign_group either places every sibling on a
 * HW row or rolls back; pmu_unassign_group releases every row in one
 * pass.  No partial-placement helpers are exposed -- the inter-group
 * rotation in hwpmc_pmu.c achieves multiplexing by atomically
 * scheduling out one group and scheduling in another, never by
 * splitting a single group across windows.
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
int pmu_group_read_value(struct pmc *pm, pmc_value_t *value);

/*
 * System-wide (PMC_MODE_SC) group lifecycle.  The process-mode hooks
 * above key everything off a pmc_process; system-wide groups have no
 * target process and are instead bound to a single CPU and rotated by a
 * per-CPU kthread.  pmc_start / pmc_stop in hwpmc_mod.c route system
 * group siblings here instead of through the normal per-CPU start/stop.
 * pmu_sys_group_pre_release MUST be called at the very top of
 * pmc_release_pmc_descriptor (before the row is touched): it tears down
 * rotation and atomically schedules the whole group out so every sibling
 * becomes UNASSIGNED, after which the framework's deferred-release path
 * handles each sibling without double-releasing a HW row.
 */
int pmu_sys_group_on_start(struct pmc *pm);
void pmu_sys_group_on_stop(struct pmc *pm);
void pmu_sys_group_pre_release(struct pmc *pm);

/*
 * PMU-layer half of the pmc_process lifecycle.  pmu_pp_init sets up the
 * spin lock and group list of a fresh pp; pmu_pp_destroy tears down the
 * per-pp rotation kthread and unhooks every pmu_group still hanging off
 * pp->pp_pmu_groups before the caller frees pp.  pmu_pp_destroy needs
 * pmc_sx held exclusive.
 */
void pmu_pp_init(struct pmc_process *pp);
void pmu_pp_destroy(struct pmc_process *pp);
void pmu_group_detach_target(pmu_group_t *pg, struct pmc_process *pp);
void pmu_pp_kick_after_exec(struct pmc_process *pp);

/*
 * Hook exported into hwpmc_mod.c so the PMU layer can find a
 * pmc_process descriptor without reaching into mod.c internals.
 */
struct pmc_process *pmc_find_process_descriptor_pmu(struct proc *p,
    uint32_t mode);

void hwpmc_pmu_force_context_switch(void);

/*
 * Rotation helpers (hwpmc_mod.c).  These manipulate pp_pmcs[ri]
 * and the pmc_target list directly so the multiplex rotation kthread
 * can swap which sibling owns each HW row without going through
 * pmc_attach_one_process / pmc_release_pmc_descriptor (which would
 * also stop-the-world / signal SIGIO).  Cumulative counts are kept
 * in pm->pm_gv.pm_savedvalue exactly as the framework's csw_out path
 * already maintains them, so detach/attach do not move counters.
 * pmc_rotation_drain spins until pm->pm_runcount drops to zero so
 * the swap is safe.
 */
void pmc_rotation_drain(struct pmc *pm);
void pmc_rotation_detach(struct pmc *pm, struct pmc_process *pp);
void pmc_rotation_attach(struct pmc *pm, struct pmc_process *pp);

#endif /* _KERNEL */

#endif /* _DEV_HWPMC_PMU_H_ */
