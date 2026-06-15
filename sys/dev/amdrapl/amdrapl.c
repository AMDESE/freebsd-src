/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Advanced Micro Devices, Inc.
 *
 * AMD RAPL energy/power telemetry driver.
 */

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/callout.h>
#include <sys/cpu.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/priv.h>
#include <sys/sbuf.h>
#include <sys/seqc.h>
#include <sys/smp.h>
#include <sys/sysctl.h>

#include <machine/cputypes.h>
#include <machine/specialreg.h>

#include <x86/x86_smp.h>
#include <x86/x86_var.h>

#define	AMD_RAPL_DRIVER_NAME	"amd_rapl"

/* Worst-case package watts for sizing the guard timer; high keeps the wrap-safe interval conservative. */
#define	AMD_RAPL_GUARD_WATT		1000

/* Guard interval clamp band (ms). */
#define	AMD_RAPL_GUARD_MIN_MS		10
#define	AMD_RAPL_GUARD_MAX_MS		60000

/* Idle-disarm threshold in guard intervals; self-disarm after this many quiet periods. Tunable; floor 2. */
#define	AMD_RAPL_IDLE_GUARD_MULT	10

/* Ceiling for idle_guard_mult: keeps mult * guard_sbt below INT64_MAX. */
#define	AMD_RAPL_IDLE_GUARD_MULT_MAX	1000000

static MALLOC_DEFINE(M_AMDRAPL, "amdrapl", "AMD RAPL driver state");

struct amd_rapl_value {
	seqc_t seqc;		/* serializes the multi-field snapshot read */
	uint64_t prev;
	sbintime_t prev_time;
	uint64_t diff;
	sbintime_t diff_time;
	uint64_t accum;
	bool primed;
	bool lapsed;		/* a sample gap exceeded one wrap period */
};

struct amd_rapl_softc {
	struct callout sampling_timer;
	struct mtx mtx;
	struct sysctl_ctx_list clist;
	uint32_t energy_unit;
	uint64_t max_energy_uj;
	sbintime_t guard_sbt;
	sbintime_t lapse_sbt;
	volatile uint64_t last_read;
	u_int idle_guard_mult;
	bool force_guard;
	bool has_core;
	bool has_package;
	bool dying;
	device_t dev;
	int npackages;
	int *cpu_pkg_slot;
	cpuset_t package_leads;
	int ncores;
	int *cpu_core_slot;
	cpuset_t core_leads;
	struct amd_rapl_value *core_value;
	struct amd_rapl_value *package_value;
};

/*
 * Raw energy counter to microjoules. Split off whole Joules first; the naive
 * raw * 1000000 / 2^unit overflows uint64_t after weeks of uptime.
 */
static uint64_t
amd_rapl_raw_to_uj(struct amd_rapl_softc *sc, uint64_t raw)
{
	uint64_t unit, whole, frac;

	unit = 1UL << sc->energy_unit;
	whole = raw / unit;
	frac = raw % unit;
	return (whole * 1000000UL + (frac * 1000000UL) / unit);
}

static uint64_t
amd_rapl_count_ujoules(struct amd_rapl_softc *sc, struct amd_rapl_value *val)
{
	uint64_t accum;
	seqc_t s;

	do {
		s = seqc_read(&val->seqc);
		accum = val->accum;
	} while (seqc_consistent(&val->seqc, s) == false);
	return (amd_rapl_raw_to_uj(sc, accum));
}

/*
 * Power in milliwatts: uJ * 1000 / us, over the measured interval (diff_time)
 * so a changing cadence can't corrupt it. Microsecond resolution; flooring to
 * ms overstated power up to 2x for sub-10ms spacing.
 */
static uint64_t
amd_rapl_count_watt(struct amd_rapl_softc *sc, struct amd_rapl_value *val)
{
	uint64_t dt_us, energy_uj, diff;
	sbintime_t dt;
	seqc_t s;

	/* Snapshot diff and diff_time together; a concurrent writer must not
	 * pair this numerator with a different denominator. */
	do {
		s = seqc_read(&val->seqc);
		diff = val->diff;
		dt = val->diff_time;
	} while (seqc_consistent(&val->seqc, s) == false);

	/* Split seconds and fraction so multi-hour gaps can't overflow. */
	dt_us = (uint64_t)(dt >> 32) * 1000000 +
	    (((dt & UINT32_MAX) * 1000000) >> 32);
	if (dt_us == 0)
		return (0);
	energy_uj = amd_rapl_raw_to_uj(sc, diff);
	return (energy_uj * 1000 / dt_us);
}

static void
amd_rapl_update_delta(struct amd_rapl_softc *sc, struct amd_rapl_value *val,
    uint64_t cur)
{
	sbintime_t now = sbinuptime();

	cur &= UINT32_MAX;	/* hardware energy counter is 32-bit */
	seqc_write_begin(&val->seqc);
	if (!val->primed) {
		val->prev = cur;
		val->prev_time = now;
		val->diff = 0;
		val->diff_time = 0;
		val->primed = true;
		seqc_write_end(&val->seqc);
		return;
	}
	/*
	 * Coalesce sub-ms re-samples so a read burst can't shrink the power
	 * window toward zero; prev stays put and the next sample spans the full
	 * interval, losing no energy.
	 */
	if (now - val->prev_time < SBT_1MS) {
		seqc_write_end(&val->seqc);
		return;
	}
	if (cur >= val->prev)
		val->diff = cur - val->prev;
	else
		val->diff = (UINT32_MAX - val->prev) + cur + 1;
	val->diff_time = now - val->prev_time;
	/*
	 * The wrap correction recovers exactly one 32-bit wrap. A gap beyond one
	 * worst-case wrap period (only once the guard self-disarms) may hide
	 * extra wraps and under-count accum; latch a sticky flag (*_energy_lapsed).
	 */
	if (val->diff_time > sc->lapse_sbt)
		val->lapsed = true;
	val->accum += val->diff;
	val->prev = cur;
	val->prev_time = now;
	seqc_write_end(&val->seqc);
}

static void
amd_rapl_read_core_energy(void *arg)
{
	struct amd_rapl_softc *sc = arg;
	uint64_t cur;

	if (rdmsr_safe(MSR_AMD_CORE_ENERGY_STATUS, &cur) != 0)
		return;
	amd_rapl_update_delta(sc, &sc->core_value[sc->cpu_core_slot[curcpu]],
	    cur);
}

/*
 * Sample every per-core counter. The per-core MSR is shared by a core's SMT
 * siblings, so rendezvous only the core leads (one CPU per physical core), not
 * all_cpus -- counts the core domain once, not per sibling. The rendezvous IPI
 * mutex serializes invocations. Must NOT hold sc->mtx (no rendezvous under it).
 */
static void
amd_rapl_sample_cores(struct amd_rapl_softc *sc)
{
	smp_rendezvous_cpus(sc->core_leads, smp_no_rendezvous_barrier,
	    amd_rapl_read_core_energy, smp_no_rendezvous_barrier, sc);
}

/*
 * Read this CPU's socket-scoped package MSR and fold it into its slot. Runs as
 * a rendezvous action on each package lead so the read and accumulation happen
 * with preemption disabled, never reordered against a concurrent sampler.
 */
static void
amd_rapl_read_package_energy(void *arg)
{
	struct amd_rapl_softc *sc = arg;
	uint64_t cur;

	if (rdmsr_safe(MSR_AMD_PKG_ENERGY_STATUS, &cur) != 0)
		return;
	amd_rapl_update_delta(sc, &sc->package_value[sc->cpu_pkg_slot[curcpu]],
	    cur);
}

/*
 * Sample every package counter. Socket-scoped MSR, so rendezvous only the
 * package leads (one per socket). Like amd_rapl_sample_cores(), must NOT hold
 * sc->mtx.
 */
static void
amd_rapl_sample_package(struct amd_rapl_softc *sc)
{
	smp_rendezvous_cpus(sc->package_leads, smp_no_rendezvous_barrier,
	    amd_rapl_read_package_energy, smp_no_rendezvous_barrier, sc);
}

/*
 * Periodic overflow-guard sampler. Read-path sampling keeps the counters fresh;
 * this only bounds 32-bit wrap during read silence. CALLOUT_RETURNUNLOCKED:
 * drops sc->mtx before the rendezvous.
 */
static void
amd_rapl_sample(void *arg)
{
	struct amd_rapl_softc *sc = arg;
	sbintime_t now, last;
	u_int mult;

	mtx_unlock(&sc->mtx);
	if (sc->has_core)
		amd_rapl_sample_cores(sc);
	if (sc->has_package)
		amd_rapl_sample_package(sc);
	/*
	 * Demand-gate: keep firing only while a consumer reads. force_guard pins
	 * it always-on; else self-disarm after idle_guard_mult quiet periods, with
	 * the read paths re-arming. This final tick sampled, so accum is fresh.
	 */
	now = sbinuptime();
	last = (sbintime_t)atomic_load_acq_64(&sc->last_read);
	/* Clamp live (not just at attach) so a runtime tunable write takes effect. */
	mult = sc->idle_guard_mult;
	if (mult < 2)
		mult = 2;
	else if (mult > AMD_RAPL_IDLE_GUARD_MULT_MAX)
		mult = AMD_RAPL_IDLE_GUARD_MULT_MAX;
	/*
	 * Reschedule under sc->mtx. The dying check lets callout_drain win the
	 * teardown race: once dying is set we never re-arm.
	 */
	mtx_lock(&sc->mtx);
	if (!sc->dying &&
	    (sc->force_guard || (now - last) < (sbintime_t)mult * sc->guard_sbt))
		callout_reset_sbt(&sc->sampling_timer, sc->guard_sbt,
		    sc->guard_sbt / 10, amd_rapl_sample, sc, 0);
	mtx_unlock(&sc->mtx);
}

/*
 * Re-arm the guard if not already pending. A currently-executing handler is not
 * "pending", so this re-arms it too, closing the lost-wakeup window where the
 * handler stops just as a reader arrives. Requires sc->mtx.
 */
static void
amd_rapl_arm_guard(struct amd_rapl_softc *sc)
{
	mtx_lock(&sc->mtx);
	if (!sc->dying && !callout_pending(&sc->sampling_timer))
		callout_reset_sbt(&sc->sampling_timer, sc->guard_sbt,
		    sc->guard_sbt / 10, amd_rapl_sample, sc, 0);
	mtx_unlock(&sc->mtx);
}

/*
 * Per-core read entry: sample, stamp last_read, keep the guard armed.
 */
static void
amd_rapl_note_read_cores(struct amd_rapl_softc *sc)
{
	/* Suspended: leave the baseline for resume to re-prime. */
	if (sc->dying)
		return;
	amd_rapl_sample_cores(sc);
	atomic_store_rel_64(&sc->last_read, (uint64_t)sbinuptime());
	amd_rapl_arm_guard(sc);
}

/*
 * Per-package read entry: sample, stamp last_read, keep the guard armed.
 */
static void
amd_rapl_note_read_package(struct amd_rapl_softc *sc)
{
	/* See amd_rapl_note_read_cores(). */
	if (sc->dying)
		return;
	amd_rapl_sample_package(sc);
	atomic_store_rel_64(&sc->last_read, (uint64_t)sbinuptime());
	amd_rapl_arm_guard(sc);
}

/*
 * force_guard write hook. Arm here: only an armed callout reads the flag, so
 * enabling it after a self-disarm would otherwise wait for the next read.
 */
static int
sysctl_amd_rapl_force_guard(SYSCTL_HANDLER_ARGS)
{
	struct amd_rapl_softc *sc = arg1;
	int error;

	error = sysctl_handle_bool(oidp, &sc->force_guard, 0, req);
	if (error != 0 || req->newptr == NULL)
		return (error);
	if (sc->force_guard)
		amd_rapl_arm_guard(sc);
	return (0);
}

static int
sysctl_amd_rapl_display_package(SYSCTL_HANDLER_ARGS)
{
	struct sbuf sbs, *sb;
	struct amd_rapl_softc *sc = arg1;
	int err, i;

	/* Energy telemetry is a power side channel (PLATYPUS, CVE-2020-8694):
	 * privileged readers only. */
	err = priv_check(req->td, PRIV_DRIVER);
	if (err != 0)
		return (err);
	/* Size pass: worst-case length, no sampling rendezvous. */
	if (req->oldptr == NULL)
		return (SYSCTL_OUT(req, 0, 21 * sc->npackages + 1));
	amd_rapl_note_read_package(sc);
	sb = sbuf_new_for_sysctl(&sbs, NULL, 0, req);
	sbuf_printf(sb, "%ju",
	    (uintmax_t)amd_rapl_count_watt(sc, &sc->package_value[0]));
	for (i = 1; i < sc->npackages; i++)
		sbuf_printf(sb, ",%ju",
		    (uintmax_t)amd_rapl_count_watt(sc, &sc->package_value[i]));
	err = sbuf_finish(sb);
	sbuf_delete(sb);
	return (err);
}

static int
sysctl_amd_rapl_display_cores(SYSCTL_HANDLER_ARGS)
{
	struct sbuf sbs, *sb;
	struct amd_rapl_softc *sc = arg1;
	int err, i;

	err = priv_check(req->td, PRIV_DRIVER);
	if (err != 0)
		return (err);
	/* Size pass: worst-case length, no sampling rendezvous. */
	if (req->oldptr == NULL)
		return (SYSCTL_OUT(req, 0, 21 * sc->ncores + 1));
	amd_rapl_note_read_cores(sc);
	sb = sbuf_new_for_sysctl(&sbs, NULL, 0, req);
	sbuf_printf(sb, "%ju",
	    (uintmax_t)amd_rapl_count_watt(sc, &sc->core_value[0]));
	for (i = 1; i < sc->ncores; i++)
		sbuf_printf(sb, ",%ju",
		    (uintmax_t)amd_rapl_count_watt(sc, &sc->core_value[i]));
	err = sbuf_finish(sb);
	sbuf_delete(sb);
	return (err);
}

static int
sysctl_amd_rapl_display_package_uj(SYSCTL_HANDLER_ARGS)
{
	struct sbuf sbs, *sb;
	struct amd_rapl_softc *sc = arg1;
	int err, i;

	err = priv_check(req->td, PRIV_DRIVER);
	if (err != 0)
		return (err);
	/* Size pass: worst-case length, no sampling rendezvous. */
	if (req->oldptr == NULL)
		return (SYSCTL_OUT(req, 0, 21 * sc->npackages + 1));
	amd_rapl_note_read_package(sc);
	sb = sbuf_new_for_sysctl(&sbs, NULL, 0, req);
	sbuf_printf(sb, "%ju",
	    (uintmax_t)amd_rapl_count_ujoules(sc, &sc->package_value[0]));
	for (i = 1; i < sc->npackages; i++)
		sbuf_printf(sb, ",%ju",
		    (uintmax_t)amd_rapl_count_ujoules(sc, &sc->package_value[i]));
	err = sbuf_finish(sb);
	sbuf_delete(sb);
	return (err);
}

static int
sysctl_amd_rapl_display_cores_uj(SYSCTL_HANDLER_ARGS)
{
	struct sbuf sbs, *sb;
	struct amd_rapl_softc *sc = arg1;
	int err, i;

	err = priv_check(req->td, PRIV_DRIVER);
	if (err != 0)
		return (err);
	/* Size pass: worst-case length, no sampling rendezvous. */
	if (req->oldptr == NULL)
		return (SYSCTL_OUT(req, 0, 21 * sc->ncores + 1));
	amd_rapl_note_read_cores(sc);
	sb = sbuf_new_for_sysctl(&sbs, NULL, 0, req);
	sbuf_printf(sb, "%ju",
	    (uintmax_t)amd_rapl_count_ujoules(sc, &sc->core_value[0]));
	for (i = 1; i < sc->ncores; i++)
		sbuf_printf(sb, ",%ju",
		    (uintmax_t)amd_rapl_count_ujoules(sc, &sc->core_value[i]));
	err = sbuf_finish(sb);
	sbuf_delete(sb);
	return (err);
}

/*
 * Per-domain sticky lapse flags as a 0/1 list parallel to *_energy_uj. Status
 * read only: does not sample or re-arm, so a bare poll costs no IPIs.
 */
static int
sysctl_amd_rapl_display_package_lapsed(SYSCTL_HANDLER_ARGS)
{
	struct sbuf sbs, *sb;
	struct amd_rapl_softc *sc = arg1;
	int err, i;

	sb = sbuf_new_for_sysctl(&sbs, NULL, 0, req);
	sbuf_printf(sb, "%d", sc->package_value[0].lapsed ? 1 : 0);
	for (i = 1; i < sc->npackages; i++)
		sbuf_printf(sb, ",%d", sc->package_value[i].lapsed ? 1 : 0);
	err = sbuf_finish(sb);
	sbuf_delete(sb);
	return (err);
}

static int
sysctl_amd_rapl_display_cores_lapsed(SYSCTL_HANDLER_ARGS)
{
	struct sbuf sbs, *sb;
	struct amd_rapl_softc *sc = arg1;
	int err, i;

	sb = sbuf_new_for_sysctl(&sbs, NULL, 0, req);
	sbuf_printf(sb, "%d", sc->core_value[0].lapsed ? 1 : 0);
	for (i = 1; i < sc->ncores; i++)
		sbuf_printf(sb, ",%d", sc->core_value[i].lapsed ? 1 : 0);
	err = sbuf_finish(sb);
	sbuf_delete(sb);
	return (err);
}

static void
amd_rapl_identify(driver_t *driver, device_t parent)
{
	device_t child;

	/*
	 * One global instance: identify runs per cpuN parent, so gate on cpu0
	 * here instead of leaving a dead child under every other CPU.
	 */
	if (device_get_unit(parent) != 0)
		return;
	if (cpu_vendor_id != CPU_VENDOR_AMD &&
	    cpu_vendor_id != CPU_VENDOR_HYGON)
		return;
	if (!(amd_pminfo & AMDPM_RAPL))
		return;
	/* Don't attach twice. */
	if (device_find_child(parent, AMD_RAPL_DRIVER_NAME, DEVICE_UNIT_ANY) !=
	    NULL)
		return;
	child = device_add_child(parent, AMD_RAPL_DRIVER_NAME, DEVICE_UNIT_ANY);
	if (child == NULL)
		device_printf(parent,
		    "add " AMD_RAPL_DRIVER_NAME " child failed\n");
}

static int
amd_rapl_probe(device_t dev)
{
	if (cpu_vendor_id != CPU_VENDOR_AMD &&
	    cpu_vendor_id != CPU_VENDOR_HYGON)
		return (ENXIO);
	if (!(amd_pminfo & AMDPM_RAPL))
		return (ENXIO);
	if (resource_disabled(AMD_RAPL_DRIVER_NAME, 0))
		return (ENXIO);
	/* Only attach the first unit. */
	if (device_get_unit(dev) != 0)
		return (ENXIO);
	device_set_desc(dev, "AMD RAPL");
	return (0);

}

/*
 * Build the per-CPU -> dense package-slot map and elect one lead CPU per socket
 * for the socket-scoped package MSR. Distinct package ids (cpu_get_pkg_id) get a
 * dense slot in first-seen order. Tracks real socket topology, not NUMA domains
 * (which over-count under AMD NPS modes).
 */
static void
amd_rapl_build_package_map(struct amd_rapl_softc *sc)
{
	int *pkg_id_of_slot;
	int cpu, i, pkg, slot;

	sc->cpu_pkg_slot = malloc(sizeof(int) * (mp_maxid + 1), M_AMDRAPL,
	    M_WAITOK | M_ZERO);
	pkg_id_of_slot = malloc(sizeof(int) * mp_ncpus, M_TEMP,
	    M_WAITOK | M_ZERO);
	CPU_ZERO(&sc->package_leads);
	sc->npackages = 0;
	CPU_FOREACH(cpu) {
		pkg = cpu_get_pkg_id(cpu);
		slot = -1;
		for (i = 0; i < sc->npackages; i++) {
			if (pkg_id_of_slot[i] == pkg) {
				slot = i;
				break;
			}
		}
		if (slot == -1) {
			slot = sc->npackages++;
			pkg_id_of_slot[slot] = pkg;
			CPU_SET(cpu, &sc->package_leads);
		}
		sc->cpu_pkg_slot[cpu] = slot;
	}
	free(pkg_id_of_slot, M_TEMP);
}

/*
 * Build the per-CPU -> dense core-slot map and elect one lead CPU per physical
 * core for the SMT-shared per-core MSR. Like the package map: distinct core ids
 * (cpu_get_core_id) get a dense slot in first-seen order. One slot per physical
 * core, not per logical CPU (which would double-count on SMT parts).
 */
static void
amd_rapl_build_core_map(struct amd_rapl_softc *sc)
{
	int *core_id_of_slot;
	int cpu, i, core, slot;

	sc->cpu_core_slot = malloc(sizeof(int) * (mp_maxid + 1), M_AMDRAPL,
	    M_WAITOK | M_ZERO);
	core_id_of_slot = malloc(sizeof(int) * mp_ncpus, M_TEMP,
	    M_WAITOK | M_ZERO);
	CPU_ZERO(&sc->core_leads);
	sc->ncores = 0;
	CPU_FOREACH(cpu) {
		core = cpu_get_core_id(cpu);
		slot = -1;
		for (i = 0; i < sc->ncores; i++) {
			if (core_id_of_slot[i] == core) {
				slot = i;
				break;
			}
		}
		if (slot == -1) {
			slot = sc->ncores++;
			core_id_of_slot[slot] = core;
			CPU_SET(cpu, &sc->core_leads);
		}
		sc->cpu_core_slot[cpu] = slot;
	}
	free(core_id_of_slot, M_TEMP);
}

/*
 * Fault-safe RAPL MSR presence probe. MSR_OP_SAFE catches a #GP (a hypervisor
 * advertising the RAPL bit but not emulating the MSR) as absent, not a panic.
 */
static bool
amd_rapl_msr_present(u_int msr, u_int cpuid)
{
	uint64_t v;

	return (x86_msr_op(msr, MSR_OP_RENDEZVOUS_ONE | MSR_OP_READ |
	    MSR_OP_SAFE | MSR_OP_CPUID(cpuid), 0, &v) == 0);
}

/*
 * Guard interval from the hardware energy unit. The counter wraps after
 * max_energy_uj; at AMD_RAPL_GUARD_WATT that is max_energy_uj/(P*1e6) seconds,
 * so sample at half that to preclude an unseen double-wrap. Clamped to a band.
 */
static sbintime_t
amd_rapl_guard_sbt(struct amd_rapl_softc *sc)
{
	uint64_t guard_ms;

	guard_ms = sc->max_energy_uj / (2000ULL * AMD_RAPL_GUARD_WATT);
	if (guard_ms < AMD_RAPL_GUARD_MIN_MS)
		guard_ms = AMD_RAPL_GUARD_MIN_MS;
	else if (guard_ms > AMD_RAPL_GUARD_MAX_MS)
		guard_ms = AMD_RAPL_GUARD_MAX_MS;
	return (guard_ms * SBT_1MS);
}

static int
amd_rapl_attach(device_t dev)
{
	struct amd_rapl_softc *sc = device_get_softc(dev);
	u_int probe_cpu = cpu_get_pcpu(dev)->pc_cpuid;
	uint64_t value;
	int error;

	sc->dev = dev;
	/*
	 * Read the power-unit MSR fault-safely: without MSR_OP_SAFE the rdmsr #GP
	 * inside the rendezvous IPI has no pcb_onfault and panics the host. Nothing
	 * allocated yet, so a fault is a clean ENXIO abort.
	 */
	error = x86_msr_op(MSR_AMD_RAPL_POWER_UNIT,
	    MSR_OP_RENDEZVOUS_ONE | MSR_OP_READ | MSR_OP_SAFE |
		MSR_OP_CPUID(probe_cpu),
	    0, &value);
	if (error != 0) {
		device_printf(dev, "power-unit MSR (0x%x) read faulted: %d\n",
		    MSR_AMD_RAPL_POWER_UNIT, error);
		return (ENXIO);
	}
	sc->energy_unit = (value >> 8) & 0x1f;
	sc->max_energy_uj = amd_rapl_raw_to_uj(sc, UINT32_MAX);
	sc->guard_sbt = amd_rapl_guard_sbt(sc);
	/*
	 * Lapse threshold = one wrap period = twice the guard interval. Derive it
	 * from the clamped guard_sbt, not the raw unit, so it never drops below the
	 * sampling cadence and latches lapsed on every normal sample.
	 */
	sc->lapse_sbt = 2 * sc->guard_sbt;
	sc->idle_guard_mult = AMD_RAPL_IDLE_GUARD_MULT;
	sc->force_guard = false;
	sc->dying = false;
	/*
	 * Export only the domains that respond: a VM may emulate one and not the
	 * other. Abort before any allocation if neither responds.
	 */
	sc->has_package = amd_rapl_msr_present(MSR_AMD_PKG_ENERGY_STATUS,
	    probe_cpu);
	sc->has_core = amd_rapl_msr_present(MSR_AMD_CORE_ENERGY_STATUS,
	    probe_cpu);
	if (!sc->has_package && !sc->has_core) {
		device_printf(dev,
		    "no RAPL energy MSR responded; not attaching\n");
		return (ENXIO);
	}
	atomic_store_rel_64(&sc->last_read, (uint64_t)sbinuptime());
	amd_rapl_build_package_map(sc);
	amd_rapl_build_core_map(sc);
	sc->core_value = malloc(sizeof(struct amd_rapl_value) * sc->ncores,
	    M_AMDRAPL, M_WAITOK | M_ZERO);
	sc->package_value = malloc(sizeof(struct amd_rapl_value) * sc->npackages,
	    M_AMDRAPL, M_WAITOK | M_ZERO);
	mtx_init(&sc->mtx, AMD_RAPL_DRIVER_NAME, NULL, MTX_DEF);
	callout_init_mtx(&sc->sampling_timer, &sc->mtx, CALLOUT_RETURNUNLOCKED);
	/*
	 * Private sysctl context, not the device auto ctx (newbus frees that only
	 * after detach returns). detach frees it first to drain in-flight handlers
	 * before we destroy the mutex / free the arrays they dereference.
	 */
	sysctl_ctx_init(&sc->clist);
	if (sc->has_package) {
		SYSCTL_ADD_PROC(&sc->clist,
		    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
		    "package_mwatt",
		    CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE,
		    sc, 0, sysctl_amd_rapl_display_package, "A",
		    "Average package power in milliwatts over the most recent "
		    "sampling interval (the previous read or overflow-guard "
		    "tick), comma-separated per domain");
		SYSCTL_ADD_PROC(&sc->clist,
		    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
		    "package_energy_uj",
		    CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE,
		    sc, 0, sysctl_amd_rapl_display_package_uj, "A",
		    "Cumulative package energy in microjoules, comma-separated per "
		    "domain (cumulative across active-monitoring windows unless "
		    "force_guard=1)");
		SYSCTL_ADD_PROC(&sc->clist,
		    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
		    "package_energy_lapsed",
		    CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE,
		    sc, 0, sysctl_amd_rapl_display_package_lapsed, "A",
		    "Per-domain 0/1: 1 means a sample gap exceeded one worst-case "
		    "wrap period and package_energy_uj may have lost a counter "
		    "wrap (see force_guard)");
	}
	if (sc->has_core) {
		SYSCTL_ADD_PROC(&sc->clist,
		    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
		    "cores_mwatt",
		    CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE,
		    sc, 0, sysctl_amd_rapl_display_cores, "A",
		    "Average power in milliwatts over the most recent sampling "
		    "interval (the previous read or overflow-guard tick), "
		    "comma-separated per physical core");
		SYSCTL_ADD_PROC(&sc->clist,
		    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
		    "cores_energy_uj",
		    CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE,
		    sc, 0, sysctl_amd_rapl_display_cores_uj, "A",
		    "Cumulative per-core energy in microjoules, comma-separated per "
		    "physical core (cumulative across active-monitoring windows "
		    "unless force_guard=1)");
		SYSCTL_ADD_PROC(&sc->clist,
		    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
		    "cores_energy_lapsed",
		    CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE,
		    sc, 0, sysctl_amd_rapl_display_cores_lapsed, "A",
		    "Per-core 0/1: 1 means a sample gap exceeded one worst-case "
		    "wrap period and cores_energy_uj may have lost a counter "
		    "wrap (see force_guard)");
	}
	SYSCTL_ADD_UINT(&sc->clist,
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
	    "energy_unit", CTLFLAG_RD, &sc->energy_unit, 0,
	    "RAPL energy unit as a power-of-two shift (1 count = 1/2^unit J)");
	SYSCTL_ADD_U64(&sc->clist,
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
	    "max_energy_uj", CTLFLAG_RD, &sc->max_energy_uj, 0,
	    "Energy in microjoules of the maximum hardware counter value (2^32-1)");
	SYSCTL_ADD_PROC(&sc->clist,
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
	    "force_guard", CTLTYPE_U8 | CTLFLAG_RWTUN | CTLFLAG_MPSAFE,
	    sc, 0, sysctl_amd_rapl_force_guard, "CU",
	    "Pin the overflow-guard sampler always-on. When 0 (default) the guard "
	    "self-disarms while no consumer is reading and *_energy_uj counters are "
	    "cumulative across active-monitoring windows rather than since attach");
	SYSCTL_ADD_UINT(&sc->clist,
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
	    "idle_guard_mult", CTLFLAG_RWTUN, &sc->idle_guard_mult, 0,
	    "Idle disarm threshold as a multiple of the guard interval (>=2)");
	if (sc->idle_guard_mult < 2)
		sc->idle_guard_mult = 2;
	else if (sc->idle_guard_mult > AMD_RAPL_IDLE_GUARD_MULT_MAX)
		sc->idle_guard_mult = AMD_RAPL_IDLE_GUARD_MULT_MAX;
	/*
	 * OIDs are already live, so a reader may arm the guard concurrently; arm
	 * under sc->mtx so the two callout_reset paths serialize.
	 */
	mtx_lock(&sc->mtx);
	callout_reset_sbt(&sc->sampling_timer, sc->guard_sbt, sc->guard_sbt / 10,
	    amd_rapl_sample, sc, 0);
	mtx_unlock(&sc->mtx);
	return (0);
}

static int
amd_rapl_detach(device_t dev)
{
	struct amd_rapl_softc *sc = device_get_softc(dev);

	/*
	 * Order matters:
	 * 1. sysctl_ctx_free() unregisters the OIDs and drains in-flight handlers,
	 *    so no reader can touch sc after it returns. (The device auto ctx is
	 *    freed only after detach returns -- too late for the arrays below.)
	 * 2. sc->dying stops amd_rapl_sample() rescheduling so callout_drain()
	 *    converges; it also catches a guard re-armed by a step-1 handler.
	 */
	sysctl_ctx_free(&sc->clist);
	mtx_lock(&sc->mtx);
	sc->dying = true;
	mtx_unlock(&sc->mtx);
	callout_drain(&sc->sampling_timer);
	mtx_destroy(&sc->mtx);
	free(sc->cpu_pkg_slot, M_AMDRAPL);
	free(sc->cpu_core_slot, M_AMDRAPL);
	free(sc->core_value, M_AMDRAPL);
	free(sc->package_value, M_AMDRAPL);
	return (0);
}

static int
amd_rapl_suspend(device_t dev)
{
	struct amd_rapl_softc *sc = device_get_softc(dev);

	mtx_lock(&sc->mtx);
	sc->dying = true;
	mtx_unlock(&sc->mtx);
	callout_drain(&sc->sampling_timer);
	return (0);
}

static int
amd_rapl_resume(device_t dev)
{
	struct amd_rapl_softc *sc = device_get_softc(dev);
	int i;

	/*
	 * Re-baseline so the first post-resume sample is a zero delta: the counters
	 * reset across suspend, and without this the wrap correction would fold a
	 * spurious ~2^32 delta into accum. accum is preserved, so energy continues.
	 */
	for (i = 0; i < sc->ncores; i++)
		sc->core_value[i].primed = false;
	for (i = 0; i < sc->npackages; i++)
		sc->package_value[i].primed = false;
	atomic_store_rel_64(&sc->last_read, (uint64_t)sbinuptime());
	mtx_lock(&sc->mtx);
	sc->dying = false;
	callout_reset_sbt(&sc->sampling_timer, sc->guard_sbt,
	    sc->guard_sbt / 10, amd_rapl_sample, sc, 0);
	mtx_unlock(&sc->mtx);
	return (0);
}

static device_method_t amd_rapl_methods[] = {
	DEVMETHOD(device_identify, amd_rapl_identify),
	DEVMETHOD(device_probe, amd_rapl_probe),
	DEVMETHOD(device_attach, amd_rapl_attach),
	DEVMETHOD(device_detach, amd_rapl_detach),
	DEVMETHOD(device_suspend, amd_rapl_suspend),
	DEVMETHOD(device_resume, amd_rapl_resume),
	DEVMETHOD_END
};

static driver_t amd_rapl_driver = {
	AMD_RAPL_DRIVER_NAME,
	amd_rapl_methods,
	sizeof(struct amd_rapl_softc),
};

DRIVER_MODULE(amd_rapl, cpu, amd_rapl_driver, 0, 0);
