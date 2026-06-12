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

/*
 * Worst-case package watts, for sizing the overflow-guard timer. The 32-bit
 * energy counter must not wrap twice between guard samples; a high value keeps
 * the derived interval conservative. Linux assumes 200W/domain; higher here to
 * cover dense server sockets.
 */
#define	AMD_RAPL_GUARD_WATT		1000

/* Clamp the derived guard interval to a sane band (ms). */
#define	AMD_RAPL_GUARD_MIN_MS		10
#define	AMD_RAPL_GUARD_MAX_MS		60000

/*
 * Idle-disarm threshold, in guard intervals. The demand-gated guard self-disarms
 * after no sysctl read for this many guard periods. Must exceed any plausible
 * slow-poll interval, else a slow consumer lets the guard lapse mid-window and
 * latches lapsed. 10 periods cover a 5-minute poll cadence at the typical
 * ~33 s guard interval. Tunable via idle_guard_mult; floor 2.
 */
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
 * Convert a raw energy counter to microjoules.
 *
 * Direct raw * 1000000 / 2^unit overflows a uint64_t after weeks of uptime, so
 * split off the whole-Joule part first: bit-identical result, but overflows only
 * when the microjoule value itself would (millennia away).
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
 * Power = energy / time, reported in milliwatts: uJ * 1000 / us. Microsecond
 * resolution keeps short windows accurate -- flooring to whole milliseconds
 * overstated power up to 2x for sub-10ms sample spacings.
 *
 * diff_time is the wall-clock gap between the two samples that produced diff
 * (see amd_rapl_update_delta). Deriving the denominator from the measured
 * interval, not a fixed sample rate, lets the cadence change without corrupting
 * the power figure.
 */
static uint64_t
amd_rapl_count_watt(struct amd_rapl_softc *sc, struct amd_rapl_value *val)
{
	uint64_t dt_us, energy_uj, diff;
	sbintime_t dt;
	seqc_t s;

	/* diff and diff_time must come from one sample. Snapshot both under a
	 * seqc retry loop so a concurrent writer (the guard callout or another
	 * reader's sample) can't pair this numerator with that denominator. */
	do {
		s = seqc_read(&val->seqc);
		diff = val->diff;
		dt = val->diff_time;
	} while (seqc_consistent(&val->seqc, s) == false);

	/* Split seconds and fraction so multi-hour gaps cannot overflow. */
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
	 * Coalesce sub-millisecond re-samples. sysctl(8) calls a string handler
	 * twice per read -- once with oldptr==NULL to size, once to fill -- so the
	 * read path samples this slot twice, microseconds apart. Folding the second
	 * hit in collapses diff_time below the 1ms amd_rapl_count_watt() divides by,
	 * zeroing power. Drop it without touching prev/prev_time: the next real
	 * sample spans the full interval and accum loses no energy, and both passes
	 * read back an identical diff and format the same string (a length mismatch
	 * between passes fails the data pass with ENOMEM).
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
	 * The wrap correction above recovers exactly one 32-bit wrap. If the sample
	 * gap exceeds one worst-case wrap period the counter may have wrapped more
	 * than once unseen (only after the demand-gated guard self-disarms during
	 * read silence), so accum -- and the *_energy_uj derived from it -- may
	 * under-count. Latch a sticky flag, reported via *_energy_lapsed.
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
 * Refresh every per-core energy counter. The AMD per-core MSR is shared by a
 * physical core's SMT siblings (either thread returns the same whole-core
 * energy), so rendezvous only the core leads -- one CPU per physical core --
 * not all_cpus. Reports the core domain once per physical core (matches Linux
 * PERF_PMU_SCOPE_CORE) instead of double-counting per SMT sibling. curcpu is
 * always a core lead, so cpu_core_slot[curcpu] picks its slot.
 *
 * Callable from the callout and the sysctl read path. The IPI mutex inside
 * smp_rendezvous_cpus() serializes invocations, so the per-slot accumulation in
 * amd_rapl_update_delta() never interleaves. Must NOT hold sc->mtx: a rendezvous
 * can't run under a mutex.
 */
static void
amd_rapl_sample_cores(struct amd_rapl_softc *sc)
{
	smp_rendezvous_cpus(sc->core_leads, smp_no_rendezvous_barrier,
	    amd_rapl_read_core_energy, smp_no_rendezvous_barrier, sc);
}

/*
 * Read this CPU's socket-scoped package energy MSR and fold it into its package
 * slot. Runs as an smp_rendezvous() action on each package lead, so the MSR read
 * and the accumulation happen together with preemption disabled -- never
 * reordered against a concurrent sampler, which corrupted the counter when the
 * read ran outside the lock. curcpu is always a package lead, so
 * cpu_pkg_slot[curcpu] picks its slot.
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
 * Refresh every package energy counter. The package MSR is socket-scoped, so
 * rendezvous only the package leads (one CPU per socket), not all_cpus. The IPI
 * mutex inside smp_rendezvous_cpus() serializes invocations, so the per-slot
 * accumulation never interleaves and no sc->mtx is needed. Like
 * amd_rapl_sample_cores(), must NOT hold sc->mtx: a rendezvous can't run under a
 * mutex.
 */
static void
amd_rapl_sample_package(struct amd_rapl_softc *sc)
{
	smp_rendezvous_cpus(sc->package_leads, smp_no_rendezvous_barrier,
	    amd_rapl_read_package_energy, smp_no_rendezvous_barrier, sc);
}

/*
 * Periodic overflow-guard sampler. On-demand sampling in the read path already
 * keeps the cumulative counters fresh; this only bounds the 32-bit counter's
 * wrap during read silence (like the Linux RAPL overflow timer). Runs without
 * sc->mtx: the callout is CALLOUT_RETURNUNLOCKED and drops the lock before the
 * rendezvous.
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
	 * Demand-gate: keep firing only while a consumer is reading. force_guard
	 * pins the guard always-on (true-cumulative *_energy_uj). Else self-disarm
	 * once reads have been quiet for idle_guard_mult guard periods (computed live
	 * below); the read paths re-arm via amd_rapl_arm_guard(). This final tick
	 * still sampled, so accum/prev are fresh at disarm.
	 */
	now = sbinuptime();
	last = (sbintime_t)atomic_load_acq_64(&sc->last_read);
	/*
	 * Derive the idle-disarm threshold live from idle_guard_mult (clamped to
	 * its floor here, not just at attach) so a runtime write to the tunable
	 * takes effect on the next decision without a reload.
	 */
	mult = sc->idle_guard_mult;
	if (mult < 2)
		mult = 2;
	else if (mult > AMD_RAPL_IDLE_GUARD_MULT_MAX)
		mult = AMD_RAPL_IDLE_GUARD_MULT_MAX;
	/*
	 * Re-take sc->mtx for the reschedule decision. Checking sc->dying here (set
	 * under sc->mtx by detach/suspend) lets callout_drain win the teardown race:
	 * once dying is set we never re-arm, so the drain sees a truly-stopped
	 * callout, not one that keeps rescheduling. callout_reset_sbt wants the mtx
	 * held too.
	 */
	mtx_lock(&sc->mtx);
	if (!sc->dying &&
	    (sc->force_guard || (now - last) < (sbintime_t)mult * sc->guard_sbt))
		callout_reset_sbt(&sc->sampling_timer, sc->guard_sbt,
		    sc->guard_sbt / 10, amd_rapl_sample, sc, 0);
	mtx_unlock(&sc->mtx);
}

/*
 * Re-arm the demand-gated guard if not already pending. Called from the read
 * paths after stamping last_read, so an active consumer keeps the guard alive
 * even if it self-disarmed during a quiet spell. A handler currently executing
 * is not "pending", so this re-arms it too -- closing the lost-wakeup window
 * where the handler decides to stop just as a reader arrives.
 * callout_reset/pending require sc->mtx.
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
 * Per-core read entry: sample on demand, record the read time, keep the guard
 * armed. sample_cores() must run outside sc->mtx (it rendezvouses); last_read is
 * a plain atomic store, so only the arm step takes the mutex.
 */
static void
amd_rapl_note_read_cores(struct amd_rapl_softc *sc)
{
	amd_rapl_sample_cores(sc);
	atomic_store_rel_64(&sc->last_read, (uint64_t)sbinuptime());
	amd_rapl_arm_guard(sc);
}

/*
 * Per-package read entry: rendezvous the package leads to sample on demand,
 * record the read time, keep the guard armed.
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

	/* Sample on read so power reflects the interval since the last read, not
	 * the (seconds-long) guard cadence. The watt math divides by the measured
	 * interval, so arbitrary read spacing is fine. The sysctl size/data double
	 * call samples twice in quick succession; amd_rapl_update_delta() drops the
	 * sub-millisecond second hit so the interval is not collapsed. */
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

	/* Sample on read so power reflects the interval since the last read, not
	 * the (seconds-long) guard cadence. The watt math divides by the measured
	 * interval, so arbitrary read spacing is fine. One field per physical core.
	 * The sysctl size/data double call samples twice in quick succession;
	 * amd_rapl_update_delta() drops the sub-millisecond second hit so the
	 * interval is not collapsed. */
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

	/* Sample at read time so the counter is fresh, not up to one timer period
	 * stale. */
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

	/* Sample at read time so the counter is fresh, not up to one timer period
	 * stale. One field per physical core. */
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
 * Report each domain's sticky idle-lapse flag as a comma-separated 0/1 list
 * parallel to *_energy_uj. A 1 means a sample gap once exceeded one worst-case
 * wrap period, so that domain's *_energy_uj may have lost a 2^32 wrap.
 * Status read only: does not sample or re-arm the guard (the flag advances only
 * on a sample, which the *_energy_uj paths drive), so a bare poll costs no IPIs.
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
	 * One global instance: identify runs for every cpuN parent, so gate on
	 * cpu0 (and on RAPL hardware) here rather than rejecting in probe,
	 * which would leave a dead child under every other CPU.
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
 * Build the per-CPU -> dense-package-slot map and the lead CPUs (one per
 * physical package) used to sample the socket-scoped package MSR.
 *
 * The package MSR is per-socket: reading it on any core returns the whole
 * socket's energy. Enumerate distinct package ids via cpu_get_pkg_id(), give
 * each a dense slot in first-seen order, elect the first CPU seen as its lead,
 * index package_value[] by slot. Tracks real socket topology instead of
 * approximating packages with NUMA domains, which over-counts under AMD NPS
 * (Nodes-Per-Socket) modes.
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
 * Build the per-CPU -> dense-physical-core-slot map and the lead CPUs (one CPU
 * per physical core) used to sample the per-core MSR.
 *
 * The per-core MSR is shared by a core's SMT siblings, so -- like the package
 * map above -- enumerate distinct core ids via cpu_get_core_id(), give each a
 * dense slot in first-seen order, elect the first CPU seen as its lead, index
 * core_value[] by slot. Reports the core domain once per physical core, not once
 * per logical CPU (which would double-count the shared counter on SMT parts).
 * With SMT off every CPU is its own core and the map is one slot per CPU.
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
 * Fault-safe presence probe for a RAPL MSR. Reads it on the given CPU with
 * MSR_OP_SAFE so a #GP (e.g. a hypervisor that advertises the RAPL bit but does
 * not emulate the MSR) is caught and reported as absent instead of panicking the
 * host. Mirrors Linux's rdmsrq_safe-based perf_msr_probe.
 */
static bool
amd_rapl_msr_present(u_int msr, u_int cpuid)
{
	uint64_t v;

	return (x86_msr_op(msr, MSR_OP_RENDEZVOUS_ONE | MSR_OP_READ |
	    MSR_OP_SAFE | MSR_OP_CPUID(cpuid), 0, &v) == 0);
}

/*
 * Derive the overflow-guard interval from the hardware energy unit.
 *
 * The 32-bit counter wraps after max_energy_uj microjoules. At a worst-case
 * AMD_RAPL_GUARD_WATT load that is max_energy_uj / (P * 1e6) seconds; sample at
 * half that so the counter can't double-wrap unseen between ticks. Finer energy
 * units (larger shift) wrap sooner, so get a shorter interval. Clamped to a sane
 * band: too short wastes IPIs, too long risks a missed wrap.
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
	 * Read the power-unit MSR fault-safely. A hypervisor can advertise the RAPL
	 * bit (CPUID 0x80000007 EDX[14]) yet not emulate this MSR; without
	 * MSR_OP_SAFE the rdmsr #GP inside the rendezvous IPI has no pcb_onfault and
	 * panics the host. Nothing is allocated yet, so a fault is a clean ENXIO
	 * abort (only the _SAFE path even sets the return value).
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
	 * Lapse threshold: one worst-case wrap period at AMD_RAPL_GUARD_WATT, exactly
	 * twice the guard interval (the guard samples at half a wrap period). Derive
	 * it from the already-clamped guard_sbt, not the raw unit, so it can never
	 * drop below the sampling cadence -- else a very fine energy unit, whose guard
	 * interval was clamped up to AMD_RAPL_GUARD_MIN_MS, would latch val->lapsed on
	 * every normal sample. update_delta() recovers one wrap, so a gap beyond this
	 * risks an unrecovered wrap and latches val->lapsed.
	 */
	sc->lapse_sbt = 2 * sc->guard_sbt;
	sc->idle_guard_mult = AMD_RAPL_IDLE_GUARD_MULT;
	sc->force_guard = false;
	sc->dying = false;
	/*
	 * Probe each energy MSR fault-safely, export only the domains that respond:
	 * a VM may emulate one domain and not the other. If neither responds, abort
	 * before any allocation for a clean ENXIO.
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
	 * Register the sysctls on a private context, not the device's auto ctx
	 * (newbus frees that only after detach returns). detach calls
	 * sysctl_ctx_free(&sc->clist) first to unregister the OIDs and drain any
	 * in-flight handler before we destroy the mutex / free the value arrays the
	 * handlers dereference.
	 */
	sysctl_ctx_init(&sc->clist);
	if (sc->has_package) {
		SYSCTL_ADD_PROC(&sc->clist,
		    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
		    "package_mwatt",
		    CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE,
		    sc, 0, sysctl_amd_rapl_display_package, "A", "");
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
		    sc, 0, sysctl_amd_rapl_display_cores, "A", "");
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
	else if (sc->idle_guard_mult > AMD_RAPL_IDLE_GUARD_MULT_MAX)
		sc->idle_guard_mult = AMD_RAPL_IDLE_GUARD_MULT_MAX;
	/*
	 * The OIDs above are already live, so a reader could be arming the guard
	 * concurrently via amd_rapl_arm_guard(). Per the callout_init_mtx() contract,
	 * arm under sc->mtx so the two callout_reset paths serialize.
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
	 * Tear-down ordering matters:
	 *
	 * 1. sysctl_ctx_free() unregisters our OIDs -- no new handler dispatches --
	 *    then sleeps until every in-flight handler returns (kern_sysctl.c drains
	 *    oid_running under SYSCTL_WLOCK). After it returns no reader can touch
	 *    sc, the value arrays, or sc->mtx. The device's own sysctl ctx won't do:
	 *    newbus frees it only after detach returns, i.e. after we have freed
	 *    these arrays -- the use-after-free / destroyed-mutex #GP we hit
	 *    otherwise.
	 * 2. sc->dying (set under sc->mtx) stops amd_rapl_sample() from rescheduling,
	 *    so callout_drain() converges instead of chasing a self-rearming callout.
	 *    A handler from step 1 may have re-armed the guard; that pending callout
	 *    is caught here.
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
	 * Re-baseline so the first post-resume sample contributes a zero delta. The
	 * package powers down across suspend and the energy counters reset; without
	 * this, the wrap correction in amd_rapl_update_delta() would fold a spurious
	 * ~2^32 delta spanning the suspended interval into accum. accum is preserved,
	 * so cumulative energy continues rather than resetting. Iterate dense
	 * core/package slots, not logical CPUs -- core_value is sized by physical
	 * core.
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
