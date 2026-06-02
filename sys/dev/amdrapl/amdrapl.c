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
#include <sys/sbuf.h>
#include <sys/seqc.h>
#include <sys/smp.h>
#include <sys/sysctl.h>

#include <machine/cputypes.h>
#include <machine/specialreg.h>

#include <x86/x86_smp.h>
#include <x86/x86_var.h>

#define	AMD_RAPL_DRIVER_NAME	"amd_rapl"

#define	MSR_RAPL_PWRUNIT		0xC0010299
#define	MSR_RAPL_CORE_ENERGY_STATUS	0xC001029A
#define	MSR_RAPL_PACKAGE_ENERGY_STATUS	0xC001029B

/*
 * Worst-case package power (watts) used to size the overflow-guard timer. The
 * 32-bit hardware energy counter must not wrap more than once between guard
 * samples; a deliberately high value keeps the derived interval conservative
 * (mirrors Linux's 200W-per-domain assumption in its RAPL overflow timer, but
 * higher to cover dense server sockets).
 */
#define	AMD_RAPL_GUARD_WATT		1000

/* Clamp the derived guard interval to a sane band (milliseconds). */
#define	AMD_RAPL_GUARD_MIN_MS		10
#define	AMD_RAPL_GUARD_MAX_MS		60000

/*
 * Idle disarm threshold expressed as a multiple of the guard interval. The
 * demand-gated guard self-disarms once no consumer has read a sysctl for this
 * many guard periods. It must stay larger than any plausible slow-poll interval
 * so an actively-monitored-but-slow consumer does not let the guard lapse
 * mid-window (which would lose counter wraps). Overridable via the
 * idle_guard_mult tunable; floor of 2.
 */
#define	AMD_RAPL_IDLE_GUARD_MULT	4

static MALLOC_DEFINE(M_AMDRAPL, "amdrapl", "AMD RAPL driver state");

struct amd_rapl_value {
	seqc_t seqc;		/* serializes the multi-field snapshot read below */
	uint64_t prev;
	sbintime_t prev_time;
	uint64_t diff;
	sbintime_t diff_time;
	uint64_t accum;
	bool primed;
};

struct amd_rapl_softc {
	struct callout sampling_timer;
	struct mtx mtx;
	struct sysctl_ctx_list clist;
	uint32_t energy_unit;
	uint64_t max_energy_uj;
	sbintime_t guard_sbt;
	sbintime_t idle_sbt;
	volatile uint64_t last_read;
	u_int idle_guard_mult;
	bool force_guard;
	bool dying;
	device_t dev;
	int npackages;
	int *cpu_pkg_slot;
	cpuset_t package_leads;
	struct amd_rapl_value *core_value;
	struct amd_rapl_value *package_value;
};

/*
 * Convert a raw energy-counter value to microjoules.
 *
 * The straightforward raw * 1000000 / 2^unit overflows a uint64_t after a few
 * weeks of accumulated uptime, so split off the whole-Joule part first: the
 * result is bit-identical to the direct computation but only overflows when
 * the microjoule value itself would (millennia away).
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
 * Power = energy / time. We report milliwatts: microjoules divided by
 * milliseconds is exactly mW (1 uJ/ms = 1 mW), so no extra scaling factor is
 * needed and the result stays well within a uint64_t.
 *
 * diff_time is the wall-clock gap between the two samples that produced diff,
 * captured in amd_rapl_update_delta(). Deriving the denominator from the
 * measured interval -- rather than assuming a fixed sample rate -- means the
 * sampling cadence can change (see the unit-derived guard timer) without
 * silently corrupting the power figure.
 */
static uint64_t
amd_rapl_count_watt(struct amd_rapl_softc *sc, struct amd_rapl_value *val)
{
	uint64_t dt_ms, energy_uj, diff;
	sbintime_t dt;
	seqc_t s;

	/* diff and diff_time must come from the same sample; snapshot both
	 * under a seqc retry loop so a concurrent writer (the guard callout or
	 * another reader's on-read sample) cannot pair this read's numerator
	 * with that write's denominator (Phase 3.3). */
	do {
		s = seqc_read(&val->seqc);
		diff = val->diff;
		dt = val->diff_time;
	} while (seqc_consistent(&val->seqc, s) == false);

	dt_ms = dt / SBT_1MS;
	if (dt_ms == 0)
		return (0);
	energy_uj = amd_rapl_raw_to_uj(sc, diff);
	return (energy_uj / dt_ms);
}

static void
amd_rapl_update_delta(struct amd_rapl_value *val, uint64_t cur)
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
	if (cur >= val->prev)
		val->diff = cur - val->prev;
	else
		val->diff = (UINT32_MAX - val->prev) + cur + 1;
	val->diff_time = now - val->prev_time;
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

	if (rdmsr_safe(MSR_RAPL_CORE_ENERGY_STATUS, &cur) != 0)
		return;
	amd_rapl_update_delta(&sc->core_value[curcpu], cur);
}

/*
 * Refresh every per-core energy counter. Reading a per-core MSR is inherently
 * an each-CPU operation, so this broadcasts to all_cpus.
 *
 * Callable from both the periodic callout and a sysctl read path. Concurrent
 * invocations are serialized by smp_rendezvous_cpus()'s internal IPI mutex, so
 * the per-slot accumulation in amd_rapl_update_delta() never interleaves. Must
 * NOT be called with sc->mtx held: smp_rendezvous_cpus() cannot run under a
 * mutex.
 */
static void
amd_rapl_sample_cores(struct amd_rapl_softc *sc)
{
	smp_rendezvous_cpus(all_cpus, smp_no_rendezvous_barrier,
	    amd_rapl_read_core_energy, smp_no_rendezvous_barrier, sc);
}

/*
 * Read this CPU's socket-scoped package energy MSR and fold it into the package
 * slot for its physical package. Runs as an smp_rendezvous() action on each
 * package lead, so the MSR read and the per-slot accumulation happen together on
 * the lead CPU with preemption disabled -- they can never be reordered against a
 * concurrent sampler, which is what corrupted the counter when the read was done
 * outside the lock. curcpu is always a package lead here, so
 * cpu_pkg_slot[curcpu] selects that lead's slot.
 */
static void
amd_rapl_read_package_energy(void *arg)
{
	struct amd_rapl_softc *sc = arg;
	uint64_t cur;

	if (rdmsr_safe(MSR_RAPL_PACKAGE_ENERGY_STATUS, &cur) != 0)
		return;
	amd_rapl_update_delta(&sc->package_value[sc->cpu_pkg_slot[curcpu]], cur);
}

/*
 * Refresh every package energy counter. The package MSR is socket-scoped, so we
 * rendezvous only the package leads (one CPU per socket) -- not all_cpus -- which
 * touches exactly one CPU per socket and never the others (R-SMP-7, R-NF-2).
 * Concurrent invocations are serialized by smp_rendezvous_cpus()'s internal IPI
 * mutex, so the per-slot accumulation never interleaves and no sc->mtx is needed.
 * Like amd_rapl_sample_cores(), it must NOT be called with sc->mtx held: a
 * rendezvous cannot run under a mutex.
 */
static void
amd_rapl_sample_package(struct amd_rapl_softc *sc)
{
	smp_rendezvous_cpus(sc->package_leads, smp_no_rendezvous_barrier,
	    amd_rapl_read_package_energy, smp_no_rendezvous_barrier, sc);
}

/*
 * Periodic overflow-guard sampler. With on-demand sampling in the read path the
 * cumulative counters no longer depend on this callout for freshness; it exists
 * to bound the 32-bit hardware counter's wrap during read silence (mirrors the
 * Linux RAPL overflow timer). Runs without sc->mtx held -- the callout is
 * CALLOUT_RETURNUNLOCKED and drops the lock before rendezvousing.
 */
static void
amd_rapl_sample(void *arg)
{
	struct amd_rapl_softc *sc = arg;
	sbintime_t now, last;

	mtx_unlock(&sc->mtx);
	amd_rapl_sample_cores(sc);
	amd_rapl_sample_package(sc);
	/*
	 * Demand-gate (Phase 3.1): keep firing only while a consumer is actively
	 * reading. force_guard pins the guard always-on (recovers true-cumulative
	 * *_energy_uj). Otherwise self-disarm once reads have been quiet for
	 * idle_sbt; the sysctl read paths re-arm via amd_rapl_arm_guard(). This
	 * final tick still sampled, so accum/prev are maximally fresh at disarm.
	 */
	now = sbinuptime();
	last = (sbintime_t)atomic_load_acq_64(&sc->last_read);
	/*
	 * Re-take sc->mtx for the reschedule decision. Checking sc->dying here
	 * (set under sc->mtx by detach/suspend) is what lets callout_drain win
	 * the teardown race: once dying is set we never re-arm, so the drain sees
	 * a callout that has truly stopped instead of one that keeps rescheduling
	 * itself. callout_reset_sbt also wants the associated mtx held.
	 */
	mtx_lock(&sc->mtx);
	if (!sc->dying &&
	    (sc->force_guard || (now - last) < sc->idle_sbt))
		callout_reset_sbt(&sc->sampling_timer, sc->guard_sbt,
		    sc->guard_sbt / 10, amd_rapl_sample, sc, 0);
	mtx_unlock(&sc->mtx);
}

/*
 * Re-arm the demand-gated overflow guard if it is not already pending. Called
 * from the sysctl read paths after stamping last_read, so that any active
 * consumer keeps the guard alive even if it had self-disarmed during a quiet
 * spell. A guard handler currently executing is not "pending", so this still
 * re-arms it -- closing the lost-wakeup window where the handler is about to
 * decide to stop while a reader arrives. callout_reset/pending require sc->mtx.
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
 * Per-core read entry point: sample on demand, record the read time, then keep
 * the guard armed. sample_cores() must run outside sc->mtx (it rendezvouses);
 * last_read is a plain atomic store, so only the arm step takes the mutex.
 */
static void
amd_rapl_note_read_cores(struct amd_rapl_softc *sc)
{
	amd_rapl_sample_cores(sc);
	atomic_store_rel_64(&sc->last_read, (uint64_t)sbinuptime());
	amd_rapl_arm_guard(sc);
}

/*
 * Per-package read entry point: rendezvous the package leads to sample on
 * demand, record the read time, then keep the overflow guard armed.
 */
static void
amd_rapl_note_read_package(struct amd_rapl_softc *sc)
{
	amd_rapl_sample_package(sc);
	atomic_store_rel_64(&sc->last_read, (uint64_t)sbinuptime());
	amd_rapl_arm_guard(sc);
}

static int
sysctl_amd_rapl_display_package(SYSCTL_HANDLER_ARGS)
{
	struct sbuf sbs, *sb;
	struct amd_rapl_softc *sc = arg1;
	int err, i;

	/* Sample on read so power reflects the interval since the last read,
	 * not the (now seconds-long) overflow-guard cadence (G3 / Phase 1.4).
	 * The watt math divides by the measured interval, so an arbitrary read
	 * spacing is fine (Phase 2.3). */
	amd_rapl_note_read_package(sc);
	sb = sbuf_new_for_sysctl(&sbs, NULL, 0, req);
	sbuf_printf(sb, "%lu", amd_rapl_count_watt(sc, &sc->package_value[0]));
	for (i = 1; i < sc->npackages; i++)
		sbuf_printf(sb, ",%lu",
		    amd_rapl_count_watt(sc, &sc->package_value[i]));
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

	/* Sample on read so power reflects the interval since the last read,
	 * not the (now seconds-long) overflow-guard cadence (G3 / Phase 1.4).
	 * The watt math divides by the measured interval, so an arbitrary read
	 * spacing is fine (Phase 2.3). */
	amd_rapl_note_read_cores(sc);
	sb = sbuf_new_for_sysctl(&sbs, NULL, 0, req);
	sbuf_printf(sb, "%lu", amd_rapl_count_watt(sc, &sc->core_value[0]));
	for (i = 1; i < mp_ncpus; ++i)
		sbuf_printf(sb, ",%lu",
		    amd_rapl_count_watt(sc, &sc->core_value[i]));
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

	/* Sample at read time so the counter is fresh, not up to one timer
	 * period stale (G3 / Phase 1.4). */
	amd_rapl_note_read_package(sc);
	sb = sbuf_new_for_sysctl(&sbs, NULL, 0, req);
	sbuf_printf(sb, "%lu",
	    amd_rapl_count_ujoules(sc, &sc->package_value[0]));
	for (i = 1; i < sc->npackages; i++)
		sbuf_printf(sb, ",%lu",
		    amd_rapl_count_ujoules(sc, &sc->package_value[i]));
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

	/* Sample at read time so the counter is fresh, not up to one timer
	 * period stale (G3 / Phase 1.4). */
	amd_rapl_note_read_cores(sc);
	sb = sbuf_new_for_sysctl(&sbs, NULL, 0, req);
	sbuf_printf(sb, "%lu",
	    amd_rapl_count_ujoules(sc, &sc->core_value[0]));
	for (i = 1; i < mp_ncpus; ++i)
		sbuf_printf(sb, ",%lu",
		    amd_rapl_count_ujoules(sc, &sc->core_value[i]));
	err = sbuf_finish(sb);
	sbuf_delete(sb);
	return (err);
}

static void
amd_rapl_identify(driver_t *driver, device_t parent)
{
	device_t child;

	/* Make sure we're not being doubly invoked. */
	if (device_find_child(parent, AMD_RAPL_DRIVER_NAME, DEVICE_UNIT_ANY) !=
	    NULL)
		return;
	child = device_add_child(parent, AMD_RAPL_DRIVER_NAME, DEVICE_UNIT_ANY);
	if (child == NULL)
		device_printf(parent,
		    "add " AMD_RAPL_DRIVER_NAME "child failed\n");
}

static int
amd_rapl_probe(device_t dev)
{
	if (cpu_vendor_id != CPU_VENDOR_AMD)
		return (ENXIO);
	if (!(amd_pminfo & AMDPM_RAPL))
		return (ENXIO);
	if (resource_disabled(AMD_RAPL_DRIVER_NAME, 0))
		return (ENXIO);
	/*
	 * Only create a device for all cpu core.
	 */
	if (device_get_unit(dev) != 0)
		return (ENXIO);
	device_set_desc(dev, "AMD RAPL");
	return (0);

}

/*
 * Build the per-CPU -> dense-package-slot map and the set of lead CPUs (one per
 * physical package) used to sample the socket-scoped package energy MSR.
 *
 * The AMD package energy MSR is per-socket: reading it on any core of a socket
 * returns the whole socket's energy. We therefore enumerate distinct package
 * ids via cpu_get_pkg_id(), assign each a dense slot in first-seen order, elect
 * the first CPU seen in each package as its lead, and index package_value[] by
 * slot. This tracks real socket topology instead of approximating packages with
 * NUMA domains, which over-counts under AMD NPS (Nodes-Per-Socket) modes.
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
 * Derive the overflow-guard sampling interval from the hardware energy unit.
 *
 * The 32-bit counter wraps after max_energy_uj microjoules of consumption. At a
 * worst-case AMD_RAPL_GUARD_WATT load that is max_energy_uj / (P * 1e6) seconds;
 * we sample at half that so the counter can never double-wrap unseen between
 * guard ticks. Finer-grained energy units (larger shift) wrap sooner and so get
 * a proportionally shorter interval. The result is clamped to a sane band: too
 * short wastes IPIs, too long risks a missed wrap if the worst-case estimate is
 * exceeded.
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
	uint64_t value;

	sc->dev = dev;
	x86_msr_op(MSR_RAPL_PWRUNIT,
	    MSR_OP_RENDEZVOUS_ONE | MSR_OP_READ |
		MSR_OP_CPUID(cpu_get_pcpu(dev)->pc_cpuid),
	    0, &value);
	sc->energy_unit = (value >> 8) & 0x1f;
	sc->max_energy_uj = amd_rapl_raw_to_uj(sc, UINT32_MAX);
	sc->guard_sbt = amd_rapl_guard_sbt(sc);
	sc->idle_guard_mult = AMD_RAPL_IDLE_GUARD_MULT;
	sc->force_guard = false;
	sc->dying = false;
	sc->last_read = (uint64_t)sbinuptime();
	amd_rapl_build_package_map(sc);
	sc->core_value = malloc(sizeof(struct amd_rapl_value) * (mp_maxid + 1),
	    M_AMDRAPL, M_WAITOK | M_ZERO);
	sc->package_value = malloc(sizeof(struct amd_rapl_value) * sc->npackages,
	    M_AMDRAPL, M_WAITOK | M_ZERO);
	mtx_init(&sc->mtx, AMD_RAPL_DRIVER_NAME, NULL, MTX_DEF | MTX_RECURSE);
	callout_init_mtx(&sc->sampling_timer, &sc->mtx, CALLOUT_RETURNUNLOCKED);
	/*
	 * Register the sysctls on a private context (not the device's auto ctx,
	 * which newbus tears down only after detach returns). detach calls
	 * sysctl_ctx_free(&sc->clist) first, which unregisters the OIDs and drains
	 * any in-flight handler before we destroy the mutex / free the value
	 * arrays the handlers dereference.
	 */
	sysctl_ctx_init(&sc->clist);
	SYSCTL_ADD_PROC(&sc->clist,
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
	    "package_mwatt", CTLTYPE_STRING | CTLFLAG_RDTUN | CTLFLAG_MPSAFE,
	    sc, 0, sysctl_amd_rapl_display_package, "A", "");
	SYSCTL_ADD_PROC(&sc->clist,
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
	    "cores_mwatt", CTLTYPE_STRING | CTLFLAG_RDTUN | CTLFLAG_MPSAFE, sc,
	    0, sysctl_amd_rapl_display_cores, "A", "");
	SYSCTL_ADD_PROC(&sc->clist,
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
	    "package_energy_uj", CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE,
	    sc, 0, sysctl_amd_rapl_display_package_uj, "A",
	    "Cumulative package energy in microjoules, comma-separated per domain "
	    "(cumulative across active-monitoring windows unless force_guard=1)");
	SYSCTL_ADD_PROC(&sc->clist,
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
	    "cores_energy_uj", CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE,
	    sc, 0, sysctl_amd_rapl_display_cores_uj, "A",
	    "Cumulative per-core energy in microjoules, comma-separated per core "
	    "(cumulative across active-monitoring windows unless force_guard=1)");
	SYSCTL_ADD_UINT(&sc->clist,
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
	    "energy_unit", CTLFLAG_RD, &sc->energy_unit, 0,
	    "RAPL energy unit as a power-of-two shift (1 count = 1/2^unit J)");
	SYSCTL_ADD_U64(&sc->clist,
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
	    "max_energy_uj", CTLFLAG_RD, &sc->max_energy_uj, 0,
	    "Energy in microjoules of the maximum hardware counter value (2^32-1)");
	SYSCTL_ADD_BOOL(&sc->clist,
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
	    "force_guard", CTLFLAG_RWTUN, &sc->force_guard, 0,
	    "Pin the overflow-guard sampler always-on. When 0 (default) the guard "
	    "self-disarms while no consumer is reading and *_energy_uj counters are "
	    "cumulative across active-monitoring windows rather than since attach");
	SYSCTL_ADD_UINT(&sc->clist,
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
	    "idle_guard_mult", CTLFLAG_RWTUN, &sc->idle_guard_mult, 0,
	    "Idle disarm threshold as a multiple of the guard interval (>=2)");
	if (sc->idle_guard_mult < 2)
		sc->idle_guard_mult = 2;
	sc->idle_sbt = sc->idle_guard_mult * sc->guard_sbt;
	/*
	 * The OIDs above are already live, so a reader could be arming the guard
	 * concurrently via amd_rapl_arm_guard(). Honor the callout_init_mtx()
	 * contract and arm under sc->mtx so the two callout_reset paths serialize.
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
	 * Tear-down ordering matters (Phase 3.1):
	 *
	 * 1. sysctl_ctx_free() unregisters our OIDs -- so no new sysctl handler
	 *    can be dispatched -- and then sleeps until every in-flight handler
	 *    has returned (kern_sysctl.c drains oid_running under SYSCTL_WLOCK).
	 *    After it returns no reader can touch sc, including the value arrays
	 *    and sc->mtx. The device's own sysctl ctx would not help here: newbus
	 *    frees it only after detach returns, i.e. after we have already freed
	 *    these arrays, which is the use-after-free / destroyed-mutex #GP we
	 *    hit otherwise.
	 * 2. sc->dying (set under sc->mtx) stops amd_rapl_sample() from
	 *    rescheduling itself, so callout_drain() converges instead of chasing
	 *    a self-rearming callout. A handler that ran during step 1 may have
	 *    re-armed the guard; that fired-or-pending callout is caught here.
	 */
	sysctl_ctx_free(&sc->clist);
	mtx_lock(&sc->mtx);
	sc->dying = true;
	mtx_unlock(&sc->mtx);
	callout_drain(&sc->sampling_timer);
	mtx_destroy(&sc->mtx);
	free(sc->cpu_pkg_slot, M_AMDRAPL);
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

	sc->last_read = (uint64_t)sbinuptime();
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
};

static driver_t amd_rapl_driver = {
	AMD_RAPL_DRIVER_NAME,
	amd_rapl_methods,
	sizeof(struct amd_rapl_softc),
};

DRIVER_MODULE(amd_rapl, cpu, amd_rapl_driver, 0, 0);
