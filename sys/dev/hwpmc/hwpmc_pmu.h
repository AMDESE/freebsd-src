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
 * Per-event lifecycle state.  Events transition INACTIVE -> ACTIVE on
 * a successful pmu_group_on_start, and back to INACTIVE on csw_out
 * (when multiplexed) or on group release.
 */
enum pmu_event_state {
	PMU_EVENT_STATE_INACTIVE = 0,
	PMU_EVENT_STATE_ACTIVE,
	PMU_EVENT_STATE_ERROR,
};

/*
 * Scheduling constraint emitted by the per-class backend.  pc_allowed_rows
 * is a bitmask over the backend-defined row index namespace ("row 0..N-1
 * for this class on this CPU"), not a raw MSR/RDPMC number.
 */
#define	PMC_SC_F_FIXED		0x0001	/* must use pc_fixed_row */
#define	PMC_SC_F_EXCLUSIVE	0x0002	/* needs the whole PMU exclusively */
#define	PMC_SC_F_SHARED		0x0004	/* tolerates sharing siblings */

struct pmc_sched_constraint {
	uint32_t	pc_allowed_rows;	/* bitmask of legal rows */
	uint8_t		pc_weight;		/* popcount(pc_allowed_rows) */
	uint8_t		pc_fixed_row;		/* used iff PMC_SC_F_FIXED */
	uint16_t	pc_flags;		/* PMC_SC_F_* */
};

/*
 * pmu_event wraps a single struct pmc with grouping/multiplex state.
 * Time fields use sbintime_t units and are only updated when the parent
 * group is multiplexing (pg_mux); for plain grouped allocations they
 * stay at zero and the read path returns the raw hardware count.
 */
struct pmu_event {
	TAILQ_ENTRY(pmu_event)		pe_sibling;
	struct pmu_group		*pe_group;
	struct pmc			*pe_pmc;
	enum pmu_event_state		pe_state;
	bool				pe_is_leader;
	struct pmc_op_pmcallocate	pe_alloc;
	struct pmc_sched_constraint	pe_cons;
	uint64_t			pe_time_enabled;
	uint64_t			pe_time_running;
	uint64_t			pe_count_at_in;
	int				pe_assigned_row;
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
	struct pmu_event		*pg_leader;
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
	uint32_t			pg_used_rows_mask;
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
void hwpmc_mark_row_free(int ri);

/*
 * Per-class constraint provider.  Implemented for AMD in hwpmc_amd.c;
 * Intel/ARM keep returning EOPNOTSUPP until they are wired in.
 */
int amd_can_assign_pmc(int ri, struct pmc *pm,
    const struct pmc_op_pmcallocate *a);
int amd_get_sched_constraint(struct pmc *pm,
    const struct pmc_op_pmcallocate *a, struct pmc_sched_constraint *cons);

/*
 * Assigner (hwpmc_assign.c).  Scheduling is strictly all-or-none at
 * the group level: pmu_assign_group either places every sibling on a
 * HW row or rolls back; pmu_unassign_group releases every row in one
 * pass.  No partial-placement helpers are exposed -- the inter-group
 * rotation in hwpmc_pmu.c achieves multiplexing by atomically
 * scheduling out one group and scheduling in another, never by
 * splitting a single group across windows.
 */
int pmu_assign_group(struct pmu_group *pg, struct proc *p, int cpu);
void pmu_unassign_group(struct pmu_group *pg, int cpu);
u_int pmu_count_core_hw_slots(struct pmu_group *pg, struct proc *p);
u_int pmu_count_class_total(struct pmu_group *pg);
int pmu_validate_group(struct pmu_group *pg);
int pmu_event_get_constraint(struct pmu_event *pe,
    struct pmc_sched_constraint *cons);

/*
 * Group lifecycle (hwpmc_pmu.c).
 */
int pmu_group_create(struct pmc_owner *po, uint32_t *pg_id);
int pmu_group_add(struct pmu_group *pg, struct pmc *pm, bool leader);
int pmu_group_commit(struct pmu_group *pg);
struct pmu_group *pmu_group_lookup(struct pmc_owner *po, uint32_t pg_id);
void pmu_group_release(struct pmu_group *pg);
void pmu_event_destroy(struct pmu_event *pe);
struct pmu_event *pmu_event_from_pmc(struct pmc *pm);

int pmu_group_on_allocate(struct pmc *pm, const struct pmc_op_pmcallocate *pa);
int pmu_group_on_start(struct pmc *pm);
void pmu_group_on_stop(struct pmc *pm);
void pmu_group_on_release(struct pmc *pm);
void pmu_group_csw_in(struct thread *td, struct pmc_process *pp);
void pmu_group_csw_out(struct thread *td, int cpu);
int pmu_group_read_value(struct pmc *pm, pmc_value_t *value);

/*
 * Cleanup hook for callers that are about to free a pmc_process out from
 * under the PMU layer (target-process exit, descriptor destroy).  Tears
 * down the per-pp rotation kthread (which holds pp as its arg) and
 * unhooks every pmu_group still hanging off pp->pp_pmu_groups so the
 * eventual pmu_group_on_release won't dereference freed memory.  Caller
 * must hold pmc_sx exclusive.
 */
void pmu_pp_release_all(struct pmc_process *pp);

/*
 * Multiplex time-accounting / rotation helpers.  Active when pg_mux is
 * set; otherwise these are no-ops on the hot path.
 */
void pmu_event_account_in(struct pmu_event *pe, uint64_t now);
void pmu_event_account_out(struct pmu_event *pe, uint64_t now);
void pmu_rotate_groups(int cpu);

/*
 * Hooks exported into hwpmc_mod.c so the PMU layer can find/unlink a
 * pmc_process descriptor without reaching into mod.c internals.
 */
struct pmc_process *pmc_find_process_descriptor_pmu(struct proc *p,
    uint32_t mode);
void pmc_unlink_target_process_pmu(struct pmc *pm, struct pmc_process *pp);

void hwpmc_pmu_sx_xlock(void);
void hwpmc_pmu_sx_xunlock(void);
int hwpmc_pmu_sx_sleep(void *chan, int timo, const char *wmesg);
void hwpmc_pmu_sx_assert_xlocked(void);
pmc_value_t hwpmc_pmc_read_delta(int cpu, int ri, struct pmc *pm);
void hwpmc_pmu_accumulate_remove(int cpu, int ri, struct pmc *pm,
    struct pmc_process *pp);

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

/* Set by hwpmc_mod.c at pmc_initialize() time. */
extern int (*hwpmc_pmu_attach_p)(struct proc *p, struct pmc *pm);

#endif /* _KERNEL */

#endif /* _DEV_HWPMC_PMU_H_ */
