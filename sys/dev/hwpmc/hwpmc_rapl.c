/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Advanced Micro Devices, Inc.
 *
 * AMD/Intel RAPL energy counters exposed as an hwpmc(4) PMC class.
 *
 * Structurally a clone of the TSC class (hwpmc_tsc.c): read-only
 * (PMC_CAP_READ), system-scope only (PMC_MODE_SC), 64-bit virtual counter.
 * The RAPL-specific logic (energy-unit decode, overflow-safe tick->uJ
 * conversion, 32-bit wrap recovery, the 64-bit accumulator, the sticky
 * "lapsed" flag and the overflow-guard callout) is ported from the standalone
 * amdrapl(4) driver. See rapl-hwpmc-integration.md for the design rationale.
 */

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/callout.h>
#include <sys/malloc.h>
#include <sys/mutex.h>
#include <sys/pmc.h>
#include <sys/pmckern.h>
#include <sys/priv.h>
#include <sys/proc.h>
#include <sys/smp.h>
#include <sys/systm.h>

#include <machine/cpu.h>
#include <machine/cpufunc.h>
#include <machine/cputypes.h>
#include <machine/specialreg.h>

#include <x86/x86_var.h>

#include <dev/hwpmc/hwpmc_rapl.h>

/*
 * RAPL support.
 */

#define	RAPL_CAPS	PMC_CAP_READ

/*
 * Worst-case package watts for sizing the guard timer; high keeps the
 * wrap-safe interval conservative.
 */
#define	RAPL_GUARD_WATT		1000

/* Guard interval clamp band (ms). */
#define	RAPL_GUARD_MIN_MS	10
#define	RAPL_GUARD_MAX_MS	60000

/*
 * One row of the per-vendor event table: an hwpmc row (class-relative index)
 * mapped to the RAPL event it measures and the MSR that backs it.
 */
struct rapl_event {
	enum pmc_event	re_ev;		/* PMC_EV_RAPL_*			*/
	uint32_t	re_msr;		/* energy-status MSR for this event	*/
};

/*
 * Per-(cpu,row) accumulator state. The RAPL energy-status MSRs are 32-bit and
 * wrap; we fold each delta into a 64-bit accumulator so the value hwpmc exports
 * is monotonic. "lapsed" is sticky: it latches if a sampling gap ever exceeded
 * one worst-case wrap period, meaning the accumulator may have missed a wrap.
 */
struct rapl_value {
	uint64_t	rv_prev;	/* last raw 32-bit MSR value		*/
	uint64_t	rv_accum;	/* 64-bit lifetime tick accumulator	*/
	sbintime_t	rv_prev_time;
	bool		rv_primed;
	bool		rv_lapsed;
};

/*
 * Per-CPU data structure for RAPL. One pmc_hw per row (rc_hw[]) plus the
 * accumulator state for each row (rc_value[]).
 */
struct rapl_cpu {
	struct pmc_hw	rc_hw[RAPL_MAX_NPMCS];
	struct rapl_value rc_value[RAPL_MAX_NPMCS];
};

static struct rapl_cpu **rapl_pcpu;

/* Vendor-selected event table, populated in pmc_rapl_initialize(). */
static struct rapl_event rapl_events[RAPL_MAX_NPMCS];
static struct pmc_descr rapl_pmcdesc[RAPL_MAX_NPMCS];
static int rapl_npmcs;

/* Hardware energy unit as a power-of-two shift: 1 count == 1/2^unit J. */
static uint32_t rapl_energy_unit;

/*
 * Base global row index of the RAPL class, cached at initialize() time so the
 * pcpu_init/fini helpers can wire pc_hwpmcs[] without needing the
 * PMC_MDEP_CLASS_INDEX_RAPL constant (which the MD-wiring task adds later).
 */
static int rapl_ri;

/*
 * Overflow-guard callout. A single per-class periodic timer: while armed it
 * samples every (cpu,row) energy counter via smp_rendezvous so a 32-bit wrap
 * can never pass unseen between consumer reads. It is armed on the first
 * rapl_start_pmc (running count 0->1) and disarmed on the last rapl_stop_pmc
 * (1->0), so monitor IPIs fire only while something is actually monitoring.
 *
 * The guard reintroduces smp_rendezvous, but ONLY for the guard sweep -- the
 * read path (rapl_read_pmc) never rendezvous; it relies on hwpmc binding the
 * reader to the target CPU (see rapl_read_pmc). This is unlike the standalone
 * driver, which rendezvoused per-domain lead-sets on every read.
 */
static struct callout	rapl_guard_callout;
static sbintime_t	rapl_guard_sbt;
static sbintime_t	rapl_lapse_sbt;
static int		rapl_running;		/* # of RUNNING RAPL PMCs */
static bool		rapl_dying;		/* finalize in progress */

/*
 * Serializes rapl_running/rapl_dying and the arm/disarm decision. MTX_SPIN is
 * mandatory: rapl_start_pmc and rapl_stop_pmc are invoked by hwpmc from inside
 * critical_enter()/critical_exit(), where only spin mutexes are legal.
 */
static struct mtx	rapl_runcount_mtx;

/*
 * Serializes rapl_update_delta() between a bound rapl_read_pmc() and the guard
 * callout's rendezvous handler. A single spin mutex (not per-cpu) is sufficient
 * and simplest: the critical section update is tiny and energy reads are
 * infrequent, so contention is negligible. A spin mutex (not a sleepable one)
 * is required because update runs from the rendezvous IPI handler and from
 * inside hwpmc's critical_enter()/critical_exit() read path.
 */
static struct mtx	rapl_value_mtx;

/*
 * Convert a raw energy counter (ticks) to microjoules without overflowing
 * uint64_t. Ported verbatim from amdrapl(4).
 */
static uint64_t
rapl_raw_to_uj(uint64_t raw)
{
	uint64_t unit, whole, frac;

	unit = 1ULL << rapl_energy_unit;
	whole = raw / unit;
	frac = raw % unit;
	return (whole * 1000000ULL + (frac * 1000000ULL) / unit);
}

/*
 * Fold one fresh 32-bit MSR reading into a row's 64-bit accumulator, recovering
 * a single wrap. Caller must hold rapl_value_mtx. Ported from amdrapl(4)'s
 * amd_rapl_update_delta(), minus the diff/diff_time power bookkeeping (hwpmc
 * exports energy only) and the seqc (replaced by rapl_value_mtx).
 */
static void
rapl_update_delta(struct rapl_value *val, uint64_t cur)
{
	sbintime_t now = sbinuptime();
	uint64_t diff;

	cur &= UINT32_MAX;
	if (!val->rv_primed) {
		val->rv_prev = cur;
		val->rv_prev_time = now;
		val->rv_primed = true;
		return;
	}
	/*
	 * Skip sub-ms re-samples. Safe for the accumulator: rv_prev and
	 * rv_prev_time are left untouched, so the next sample folds the full
	 * elapsed interval in one delta and no energy is lost.
	 */
	if (now - val->rv_prev_time < SBT_1MS)
		return;
	/* Wrap correction recovers only one 32-bit wrap per sample. */
	if (cur >= val->rv_prev)
		diff = cur - val->rv_prev;
	else
		diff = (UINT32_MAX - val->rv_prev) + cur + 1;

	/*
	 * A gap longer than one worst-case wrap period may have hidden a wrap
	 * the fold above cannot recover; latch the sticky lapsed flag.
	 */
	if (now - val->rv_prev_time > rapl_lapse_sbt)
		val->rv_lapsed = true;
	val->rv_accum += diff;
	val->rv_prev = cur;
	val->rv_prev_time = now;
}

/*
 * Sample one row's MSR on the current CPU and fold it into that (cpu,row)
 * accumulator. Must run on the CPU whose counter is wanted: the read path is
 * already bound there, and the guard reaches every CPU via rendezvous.
 *
 * rapl_value_mtx serializes the fold against a concurrent read/guard sample of
 * the same (cpu,row). The teardown path drains the guard callout before freeing
 * any rc_value (see rapl_pcpu_fini), so this never runs against freed state.
 */
static void
rapl_sample_row(int cpu, int ri)
{
	struct rapl_value *val;
	uint64_t cur;

	if (rdmsr_safe(rapl_events[ri].re_msr, &cur) != 0)
		return;
	val = &rapl_pcpu[cpu]->rc_value[ri];
	mtx_lock_spin(&rapl_value_mtx);
	rapl_update_delta(val, cur);
	mtx_unlock_spin(&rapl_value_mtx);
}

/*
 * Guard rendezvous handler: sample every row on the CPU we are running on.
 * Invoked on all active CPUs by smp_rendezvous_cpus().
 */
static void
rapl_guard_handler(void *arg __unused)
{
	int cpu = curcpu;
	int ri;

	for (ri = 0; ri < rapl_npmcs; ri++)
		rapl_sample_row(cpu, ri);
}

static void	rapl_guard_tick(void *arg);

/*
 * (Re)arm the guard callout. Non-sleeping, so callout_reset_sbt is safe from
 * the critical-section start path. Caller must hold rapl_runcount_mtx, or call
 * before any concurrency exists (initialize()).
 */
static void
rapl_guard_schedule(void)
{
	callout_reset_sbt(&rapl_guard_callout, rapl_guard_sbt,
	    rapl_guard_sbt / 10, rapl_guard_tick, NULL, 0);
}

/*
 * Periodic overflow guard. Sweeps every active CPU's RAPL MSRs so no 32-bit
 * wrap is lost while a counter is allocated but not being read often. Runs in
 * the softclock swi thread with no lock held on entry (the callout has no
 * associated mutex).
 */
static void
rapl_guard_tick(void *arg __unused)
{
	smp_rendezvous(smp_no_rendezvous_barrier, rapl_guard_handler,
	    smp_no_rendezvous_barrier, NULL);
	/* Keep firing while counters run and we are not tearing down. */
	mtx_lock_spin(&rapl_runcount_mtx);
	if (!rapl_dying && rapl_running > 0)
		rapl_guard_schedule();
	mtx_unlock_spin(&rapl_runcount_mtx);
}

static int
rapl_allocate_pmc(int cpu __diagused, int ri, struct pmc *pm __unused,
    const struct pmc_op_pmcallocate *a)
{

	KASSERT(cpu >= 0 && cpu < pmc_cpu_max(),
	    ("[rapl,%d] illegal CPU value %d", __LINE__, cpu));
	KASSERT(ri >= 0 && ri < rapl_npmcs,
	    ("[rapl,%d] illegal row index %d", __LINE__, ri));

	if (a->pm_class != PMC_CLASS_RAPL)
		return (EINVAL);

	if (a->pm_mode != PMC_MODE_SC)
		return (EINVAL);

	/*
	 * RAPL energy counters are a power side channel: at high sampling rates
	 * they can leak data-dependent activity (the PLATYPUS attack, CVE-2020-
	 * 8694 / CVE-2020-12912), which is why Linux made its equivalent
	 * interfaces root-only. Require privilege unconditionally here.
	 *
	 * The hwpmc core already gates system-scope allocation on
	 * PRIV_PMC_SYSTEM, but that check is bypassed when an administrator sets
	 * security.bsd.unprivileged_syspmcs=1 (a reasonable escape hatch for
	 * ordinary event counters). Energy telemetry must not honor that bypass,
	 * so we re-check here regardless of the tunable. priv_check runs in the
	 * requesting thread's context (pcd_allocate_pmc is called from the
	 * PMC_OP_PMCALLOCATE syscall path).
	 */
	if (priv_check(curthread, PRIV_PMC_SYSTEM) != 0)
		return (EPERM);

	/* Reject events this vendor does not expose (e.g. DRAM on AMD). */
	if (a->pm_ev != rapl_events[ri].re_ev)
		return (EINVAL);

	return (0);
}

static int
rapl_config_pmc(int cpu, int ri, struct pmc *pm)
{
	struct pmc_hw *phw;

	PMCDBG3(MDP,CFG,1, "cpu=%d ri=%d pm=%p", cpu, ri, pm);

	KASSERT(cpu >= 0 && cpu < pmc_cpu_max(),
	    ("[rapl,%d] illegal CPU value %d", __LINE__, cpu));
	KASSERT(ri >= 0 && ri < rapl_npmcs,
	    ("[rapl,%d] illegal row-index %d", __LINE__, ri));

	phw = &rapl_pcpu[cpu]->rc_hw[ri];

	KASSERT(pm == NULL || phw->phw_pmc == NULL,
	    ("[rapl,%d] pm=%p phw->pm=%p hwpmc not unconfigured", __LINE__,
	    pm, phw->phw_pmc));

	phw->phw_pmc = pm;

	return (0);
}

static int
rapl_describe(int cpu, int ri, struct pmc_info *pi, struct pmc **ppmc)
{
	const struct pmc_descr *pd;
	struct pmc_hw *phw;

	KASSERT(cpu >= 0 && cpu < pmc_cpu_max(),
	    ("[rapl,%d] illegal CPU %d", __LINE__, cpu));
	KASSERT(ri >= 0 && ri < rapl_npmcs,
	    ("[rapl,%d] illegal row-index %d", __LINE__, ri));

	phw = &rapl_pcpu[cpu]->rc_hw[ri];
	pd  = &rapl_pmcdesc[ri];

	strlcpy(pi->pm_name, pd->pd_name, sizeof(pi->pm_name));
	pi->pm_class = pd->pd_class;

	if (phw->phw_state & PMC_PHW_FLAG_IS_ENABLED) {
		pi->pm_enabled = TRUE;
		*ppmc          = phw->phw_pmc;
	} else {
		pi->pm_enabled = FALSE;
		*ppmc          = NULL;
	}

	return (0);
}

static int
rapl_get_config(int cpu, int ri, struct pmc **ppm)
{

	KASSERT(cpu >= 0 && cpu < pmc_cpu_max(),
	    ("[rapl,%d] illegal CPU %d", __LINE__, cpu));
	KASSERT(ri >= 0 && ri < rapl_npmcs,
	    ("[rapl,%d] illegal row-index %d", __LINE__, ri));

	*ppm = rapl_pcpu[cpu]->rc_hw[ri].phw_pmc;

	return (0);
}

static int
rapl_get_msr(int ri, uint32_t *msr)
{

	KASSERT(ri >= 0 && ri < rapl_npmcs,
	    ("[rapl,%d] ri %d out of range", __LINE__, ri));

	*msr = rapl_events[ri].re_msr;

	return (0);
}

static int
rapl_pcpu_init(struct pmc_mdep *md __unused, int cpu)
{
	struct pmc_cpu *pc;
	struct rapl_cpu *rapl_pc;
	int ri, n;

	KASSERT(cpu >= 0 && cpu < pmc_cpu_max(),
	    ("[rapl,%d] illegal cpu %d", __LINE__, cpu));
	KASSERT(rapl_pcpu, ("[rapl,%d] null pcpu", __LINE__));
	KASSERT(rapl_pcpu[cpu] == NULL, ("[rapl,%d] non-null per-cpu",
	    __LINE__));

	rapl_pc = malloc(sizeof(struct rapl_cpu), M_PMC, M_WAITOK | M_ZERO);

	for (n = 0; n < rapl_npmcs; n++)
		rapl_pc->rc_hw[n].phw_state = PMC_PHW_FLAG_IS_ENABLED |
		    PMC_PHW_CPU_TO_STATE(cpu) | PMC_PHW_INDEX_TO_STATE(n) |
		    PMC_PHW_FLAG_IS_SHAREABLE;

	rapl_pcpu[cpu] = rapl_pc;

	KASSERT(pmc_pcpu, ("[rapl,%d] null generic pcpu", __LINE__));

	pc = pmc_pcpu[cpu];

	KASSERT(pc, ("[rapl,%d] null generic per-cpu", __LINE__));

	for (n = 0; n < rapl_npmcs; n++) {
		ri = rapl_ri + n;
		pc->pc_hwpmcs[ri] = &rapl_pc->rc_hw[n];
	}

	return (0);
}

static int
rapl_pcpu_fini(struct pmc_mdep *md __unused, int cpu)
{
	struct pmc_cpu *pc;
	int ri, n;

	KASSERT(cpu >= 0 && cpu < pmc_cpu_max(),
	    ("[rapl,%d] illegal cpu %d", __LINE__, cpu));
	KASSERT(rapl_pcpu[cpu] != NULL, ("[rapl,%d] null pcpu", __LINE__));

	/*
	 * Drain the guard callout before freeing any per-cpu state. hwpmc runs
	 * this per-cpu teardown before pmc_rapl_finalize, and rapl_stop_pmc only
	 * callout_stop()s (it cannot sleep in the critical section), so a guard
	 * handler dispatched just before the last stop may still be sampling
	 * rc_value on another CPU. All PMCs are already released here
	 * (rapl_running == 0), so the callout cannot rearm; the drain waits for
	 * any in-flight handler to finish, after which the plain free is safe.
	 * This runs in a sleepable context (sched_bind, not critical_enter), so
	 * callout_drain and free are both legal. The drain is a no-op on the
	 * already-stopped callout for every cpu after the first.
	 */
	callout_drain(&rapl_guard_callout);
	free(rapl_pcpu[cpu], M_PMC);
	rapl_pcpu[cpu] = NULL;

	pc = pmc_pcpu[cpu];
	for (n = 0; n < rapl_npmcs; n++) {
		ri = rapl_ri + n;
		pc->pc_hwpmcs[ri] = NULL;
	}

	return (0);
}

static int
rapl_read_pmc(int cpu, int ri, struct pmc *pm, pmc_value_t *v)
{
	enum pmc_mode mode __diagused;

	KASSERT(cpu >= 0 && cpu < pmc_cpu_max(),
	    ("[rapl,%d] illegal CPU value %d", __LINE__, cpu));
	KASSERT(ri >= 0 && ri < rapl_npmcs,
	    ("[rapl,%d] illegal ri %d", __LINE__, ri));

	mode = PMC_TO_MODE(pm);

	KASSERT(mode == PMC_MODE_SC,
	    ("[rapl,%d] illegal pmc mode %d", __LINE__, mode));

	PMCDBG1(MDP,REA,1, "rapl-read id=%d", ri);

	/*
	 * hwpmc has already bound this thread to PMC_TO_CPU(pm) == cpu and
	 * entered a critical section (hwpmc_mod.c system-mode read path), so
	 * rdmsr here reads the energy MSR of the domain the consumer asked for.
	 * Per-CPU aliasing (siblings sharing a socket/core MSR) is intended;
	 * userland binds one PMC per domain and does not sum siblings.
	 */
	rapl_sample_row(cpu, ri);
	*v = rapl_raw_to_uj(rapl_pcpu[cpu]->rc_value[ri].rv_accum);

	return (0);
}

static int
rapl_release_pmc(int cpu __diagused, int ri __diagused, struct pmc *pmc __unused)
{
	struct pmc_hw *phw __diagused;

	KASSERT(cpu >= 0 && cpu < pmc_cpu_max(),
	    ("[rapl,%d] illegal CPU value %d", __LINE__, cpu));
	KASSERT(ri >= 0 && ri < rapl_npmcs,
	    ("[rapl,%d] illegal row-index %d", __LINE__, ri));

	phw = &rapl_pcpu[cpu]->rc_hw[ri];

	KASSERT(phw->phw_pmc == NULL,
	    ("[rapl,%d] PHW pmc %p non-NULL", __LINE__, phw->phw_pmc));

	/*
	 * Nothing to do.
	 */
	return (0);
}

static int
rapl_start_pmc(int cpu __diagused, int ri __diagused, struct pmc *pm __unused)
{

	KASSERT(cpu >= 0 && cpu < pmc_cpu_max(),
	    ("[rapl,%d] illegal CPU value %d", __LINE__, cpu));
	KASSERT(ri >= 0 && ri < rapl_npmcs,
	    ("[rapl,%d] illegal row-index %d", __LINE__, ri));

	/*
	 * RAPL counters are free-running; "starting" only arms the overflow
	 * guard on the 0->1 transition so it runs exactly while >=1 RAPL PMC
	 * is running. hwpmc serializes start/stop under pmc_sx; rapl_runcount_mtx
	 * additionally orders us against the firing guard callout. This runs
	 * inside hwpmc's critical section, so the mutex must be MTX_SPIN and we
	 * must not sleep -- callout_reset_sbt (via rapl_guard_schedule) does not.
	 */
	mtx_lock_spin(&rapl_runcount_mtx);
	if (!rapl_dying && rapl_running++ == 0)
		rapl_guard_schedule();
	mtx_unlock_spin(&rapl_runcount_mtx);

	return (0);
}

static int
rapl_stop_pmc(int cpu __diagused, int ri __diagused, struct pmc *pm __unused)
{

	KASSERT(cpu >= 0 && cpu < pmc_cpu_max(),
	    ("[rapl,%d] illegal CPU value %d", __LINE__, cpu));
	KASSERT(ri >= 0 && ri < rapl_npmcs,
	    ("[rapl,%d] illegal row-index %d", __LINE__, ri));

	/*
	 * Disarm the guard on the last 1->0 transition. This runs inside
	 * hwpmc's critical section, so we MUST NOT sleep: callout_stop (never
	 * callout_drain, which sleeps) is used. A guard handler already
	 * dispatched on another CPU may still be finishing when we return; that
	 * is benign -- it samples through rapl_sample_row, which loads the
	 * per-cpu state under rapl_value_mtx and tolerates a NULL slot, and it
	 * will not reschedule because rapl_guard_tick re-checks rapl_running == 0
	 * under rapl_runcount_mtx before rearming.
	 */
	mtx_lock_spin(&rapl_runcount_mtx);
	if (rapl_running > 0 && --rapl_running == 0)
		callout_stop(&rapl_guard_callout);
	mtx_unlock_spin(&rapl_runcount_mtx);

	return (0);
}

static int
rapl_write_pmc(int cpu __diagused, int ri __diagused, struct pmc *pm __unused,
    pmc_value_t v __unused)
{

	KASSERT(cpu >= 0 && cpu < pmc_cpu_max(),
	    ("[rapl,%d] illegal CPU value %d", __LINE__, cpu));
	KASSERT(ri >= 0 && ri < rapl_npmcs,
	    ("[rapl,%d] illegal row-index %d", __LINE__, ri));

	/* Energy counters are not writable; refuse silently like TSC. */
	return (0);
}

/* RAPL MSR presence probe (fault-safe, on the boot CPU). */
static bool
rapl_msr_present(uint32_t msr)
{
	uint64_t v;

	return (x86_msr_op(msr, MSR_OP_RENDEZVOUS_ONE | MSR_OP_READ |
	    MSR_OP_SAFE | MSR_OP_CPUID(PCPU_GET(cpuid)), 0, &v) == 0);
}

/*
 * Append an event row to the table if its MSR responds on this hardware.
 */
static void
rapl_add_event(enum pmc_event ev, uint32_t msr, const char *name)
{

	if (!rapl_msr_present(msr))
		return;
	rapl_events[rapl_npmcs].re_ev = ev;
	rapl_events[rapl_npmcs].re_msr = msr;
	rapl_pmcdesc[rapl_npmcs].pd_class = PMC_CLASS_RAPL;
	rapl_pmcdesc[rapl_npmcs].pd_caps = RAPL_CAPS;
	rapl_pmcdesc[rapl_npmcs].pd_width = 64;
	strlcpy(rapl_pmcdesc[rapl_npmcs].pd_name, name,
	    sizeof(rapl_pmcdesc[rapl_npmcs].pd_name));
	rapl_npmcs++;
}

/*
 * Guard interval from the hardware energy unit. The counter wraps after
 * 2^32-1 ticks; at RAPL_GUARD_WATT that is max_energy_uj/(P*1e6) seconds, so we
 * sample at half that to avoid an unseen double-wrap.
 */
static sbintime_t
rapl_compute_guard_sbt(void)
{
	uint64_t max_energy_uj, guard_ms;

	max_energy_uj = rapl_raw_to_uj(UINT32_MAX);
	guard_ms = max_energy_uj / (2000ULL * RAPL_GUARD_WATT);
	if (guard_ms < RAPL_GUARD_MIN_MS)
		guard_ms = RAPL_GUARD_MIN_MS;
	else if (guard_ms > RAPL_GUARD_MAX_MS)
		guard_ms = RAPL_GUARD_MAX_MS;
	return (guard_ms * SBT_1MS);
}

int
pmc_rapl_initialize(struct pmc_mdep *md, int maxcpu)
{
	struct pmc_classdep *pcd;
	uint32_t unit_msr, pkg_msr, cores_msr, dram_msr;
	uint64_t unit_val;

	KASSERT(md != NULL, ("[rapl,%d] md is NULL", __LINE__));
	KASSERT(md->pmd_nclass >= 1, ("[rapl,%d] dubious md->nclass %d",
	    __LINE__, md->pmd_nclass));

	/* Select the per-vendor MSR set (design doc section 5). */
	switch (cpu_vendor_id) {
	case CPU_VENDOR_AMD:
	case CPU_VENDOR_HYGON:
		unit_msr = MSR_AMD_RAPL_POWER_UNIT;
		pkg_msr = MSR_AMD_PKG_ENERGY_STATUS;
		cores_msr = MSR_AMD_CORE_ENERGY_STATUS;
		dram_msr = 0;			/* AMD has no DRAM domain */
		break;
	case CPU_VENDOR_INTEL:
		unit_msr = MSR_RAPL_POWER_UNIT;
		pkg_msr = MSR_PKG_ENERGY_STATUS;
		cores_msr = MSR_PP0_ENERGY_STATUS;
		dram_msr = MSR_DRAM_ENERGY_STATUS;
		break;
	default:
		return (ENXIO);
	}

	/* Decode the energy unit once; bail if the unit MSR is absent. */
	if (x86_msr_op(unit_msr, MSR_OP_RENDEZVOUS_ONE | MSR_OP_READ |
	    MSR_OP_SAFE | MSR_OP_CPUID(PCPU_GET(cpuid)), 0, &unit_val) != 0)
		return (ENXIO);
	rapl_energy_unit = (unit_val >> 8) & 0x1f;

	/* Build the event table from the MSRs that actually respond. */
	rapl_npmcs = 0;
	rapl_add_event(PMC_EV_RAPL_ENERGY_PKG, pkg_msr, "RAPL_ENERGY_PKG");
	rapl_add_event(PMC_EV_RAPL_ENERGY_CORES, cores_msr,
	    "RAPL_ENERGY_CORES");
	if (dram_msr != 0)
		rapl_add_event(PMC_EV_RAPL_ENERGY_DRAM, dram_msr,
		    "RAPL_ENERGY_DRAM");

	/* No RAPL energy MSR responded: let the caller skip the class. */
	if (rapl_npmcs == 0)
		return (ENXIO);

	rapl_guard_sbt = rapl_compute_guard_sbt();
	/* Lapse threshold = one wrap period = twice the guard interval. */
	rapl_lapse_sbt = 2 * rapl_guard_sbt;
	rapl_running = 0;
	rapl_dying = false;

	mtx_init(&rapl_value_mtx, "rapl-value", NULL, MTX_SPIN);
	mtx_init(&rapl_runcount_mtx, "rapl-runcount", NULL, MTX_SPIN);
	/* MPSAFE callout with no associated mutex; the handler locks itself. */
	callout_init(&rapl_guard_callout, 1);

	rapl_pcpu = malloc(sizeof(struct rapl_cpu *) * maxcpu, M_PMC,
	    M_ZERO | M_WAITOK);

	pcd = &md->pmd_classdep[PMC_MDEP_CLASS_INDEX_RAPL];

	pcd->pcd_caps	= RAPL_CAPS;
	pcd->pcd_class	= PMC_CLASS_RAPL;
	pcd->pcd_num	= rapl_npmcs;
	pcd->pcd_ri	= md->pmd_npmc;
	pcd->pcd_width	= 64;

	pcd->pcd_allocate_pmc = rapl_allocate_pmc;
	pcd->pcd_config_pmc   = rapl_config_pmc;
	pcd->pcd_describe     = rapl_describe;
	pcd->pcd_get_config   = rapl_get_config;
	pcd->pcd_get_msr      = rapl_get_msr;
	pcd->pcd_pcpu_init    = rapl_pcpu_init;
	pcd->pcd_pcpu_fini    = rapl_pcpu_fini;
	pcd->pcd_read_pmc     = rapl_read_pmc;
	pcd->pcd_release_pmc  = rapl_release_pmc;
	pcd->pcd_start_pmc    = rapl_start_pmc;
	pcd->pcd_stop_pmc     = rapl_stop_pmc;
	pcd->pcd_write_pmc    = rapl_write_pmc;

	rapl_ri = md->pmd_npmc;
	md->pmd_npmc += rapl_npmcs;

	return (0);
}

void
pmc_rapl_finalize(struct pmc_mdep *md __diagused)
{
	PMCDBG0(MDP, INI, 1, "rapl-finalize");

	/*
	 * Stop the guard before tearing down per-cpu state. finalize is not
	 * called from a critical section, so callout_drain (which may sleep) is
	 * legal here and guarantees no handler is mid-flight afterwards.
	 */
	mtx_lock_spin(&rapl_runcount_mtx);
	rapl_dying = true;
	mtx_unlock_spin(&rapl_runcount_mtx);
	callout_drain(&rapl_guard_callout);

	for (int i = 0; i < pmc_cpu_max(); i++)
		KASSERT(rapl_pcpu[i] == NULL, ("[rapl,%d] non-null pcpu cpu %d",
		    __LINE__, i));

	mtx_destroy(&rapl_value_mtx);
	mtx_destroy(&rapl_runcount_mtx);

	free(rapl_pcpu, M_PMC);
	rapl_pcpu = NULL;
}
