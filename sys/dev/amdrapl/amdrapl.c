
#include <sys/param.h>
#include <sys/bus.h>
#include <sys/callout.h>
#include <sys/cpu.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/sbuf.h>
#include <sys/smp.h>
#include <sys/sysctl.h>

#include <vm/vm.h>

#include <machine/cputypes.h>
#include <machine/specialreg.h>

#include <x86/x86_var.h>

#define	AMD_RAPL_DRIVER_NAME	"amd_rapl"

#define	MSR_RAPL_PWRUNIT		0xC0010299
#define	MSR_RAPL_CORE_ENERGY_STATUS	0xC001029A
#define	MSR_RAPL_PACKAGE_ENERGY_STATUS	0xC001029B
#define	AMD_RAPL_SAMPLE_UNIT		10

struct amd_rapl_value {
	uint64_t prev;
	volatile uint64_t diff;
	volatile uint64_t accum;
	bool primed;
};

struct amd_rapl_softc {
	struct callout sampling_timer;
	struct mtx mtx;
	uint32_t energy_unit;
	device_t dev;
	uint32_t cpus_per_domain;
	struct amd_rapl_value *core_value;
	struct amd_rapl_value *package_value;
};

static uint64_t
amd_rapl_count_watt(struct amd_rapl_softc *sc, struct amd_rapl_value *val)
{
	return ((val->diff) * 100 * 1000 / (1UL << sc->energy_unit));
}

/*
 * Convert the monotonic raw-count accumulator to microjoules.
 *
 * The straightforward accum * 1000000 / 2^unit overflows a uint64_t after a
 * few weeks of uptime, so split off the whole-Joule part first: the result is
 * bit-identical to the direct computation but only overflows when the µJ value
 * itself would (millennia away).
 */
static uint64_t
amd_rapl_count_ujoules(struct amd_rapl_softc *sc, struct amd_rapl_value *val)
{
	uint64_t accum, unit, whole, frac;

	accum = val->accum;
	unit = 1UL << sc->energy_unit;
	whole = accum / unit;
	frac = accum % unit;
	return (whole * 1000000UL + (frac * 1000000UL) / unit);
}

static void
amd_rapl_update_delta(struct amd_rapl_value *val, uint64_t cur)
{
	if (!val->primed) {
		val->prev = cur;
		val->diff = 0;
		val->primed = true;
		return;
	}
	if (cur >= val->prev)
		val->diff = cur - val->prev;
	else
		val->diff = (UINT32_MAX - val->prev) + cur + 1;
	val->accum += val->diff;
	val->prev = cur;
}

static void
amd_rapl_read_core_energy(void *arg)
{
	struct amd_rapl_softc *sc = arg;
	uint64_t cur;

	rdmsr_safe(MSR_RAPL_CORE_ENERGY_STATUS, &cur);
	amd_rapl_update_delta(&sc->core_value[curcpu], cur);
}

static void
amd_rapl_read_package_energy(void *arg)
{
	struct amd_rapl_softc *sc = arg;
	uint64_t cur;

	rdmsr_safe(MSR_RAPL_PACKAGE_ENERGY_STATUS, &cur);
	amd_rapl_update_delta(&sc->package_value[curcpu / sc->cpus_per_domain],
	    cur);
}

static void
amd_rapl_sample(void *arg)
{
	struct amd_rapl_softc *sc = arg;
	int i;
	cpuset_t package_cpus;

	mtx_unlock(&sc->mtx);
	smp_rendezvous_cpus(all_cpus, smp_no_rendezvous_barrier,
	    amd_rapl_read_core_energy, smp_no_rendezvous_barrier, sc);
	CPU_ZERO(&package_cpus);
	for (i = 0; i < vm_ndomains; i++)
		CPU_SET(i * sc->cpus_per_domain, &package_cpus);
	smp_rendezvous_cpus(package_cpus, smp_no_rendezvous_barrier,
	    amd_rapl_read_package_energy, smp_no_rendezvous_barrier, sc);
	callout_schedule_sbt(&sc->sampling_timer,
	    SBT_1MS * AMD_RAPL_SAMPLE_UNIT, SBT_1MS, 0);
}
static int
sysctl_amd_rapl_display_package(SYSCTL_HANDLER_ARGS)
{
	struct sbuf sbs, *sb;
	struct amd_rapl_softc *sc = arg1;
	int err, i;

	sb = sbuf_new_for_sysctl(&sbs, NULL, 0, req);
	sbuf_printf(sb, "%lu", amd_rapl_count_watt(sc, &sc->package_value[0]));
	for (i = 1; i < vm_ndomains; i++)
		sbuf_printf(sb, "%lu",
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

	sb = sbuf_new_for_sysctl(&sbs, NULL, 0, req);
	sbuf_printf(sb, "%lu",
	    amd_rapl_count_ujoules(sc, &sc->package_value[0]));
	for (i = 1; i < vm_ndomains; i++)
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
	sc->cpus_per_domain = mp_ncpus / vm_ndomains;
	sc->core_value = malloc(sizeof(struct amd_rapl_value) * mp_ncpus,
	    M_TEMP, M_WAITOK | M_ZERO);
	sc->package_value = malloc(sizeof(struct amd_rapl_value) * vm_ndomains,
	    M_TEMP, M_WAITOK | M_ZERO);
	mtx_init(&sc->mtx, AMD_RAPL_DRIVER_NAME, NULL, MTX_DEF | MTX_RECURSE);
	callout_init_mtx(&sc->sampling_timer, &sc->mtx, CALLOUT_RETURNUNLOCKED);
	SYSCTL_ADD_PROC(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
	    "package_mwatt", CTLTYPE_STRING | CTLFLAG_RDTUN | CTLFLAG_MPSAFE,
	    sc, 0, sysctl_amd_rapl_display_package, "A", "");
	SYSCTL_ADD_PROC(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
	    "cores_mwatt", CTLTYPE_STRING | CTLFLAG_RDTUN | CTLFLAG_MPSAFE, sc,
	    0, sysctl_amd_rapl_display_cores, "A", "");
	SYSCTL_ADD_PROC(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
	    "package_energy_uj", CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE,
	    sc, 0, sysctl_amd_rapl_display_package_uj, "A",
	    "Cumulative package energy in microjoules, comma-separated per domain");
	SYSCTL_ADD_PROC(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
	    "cores_energy_uj", CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE,
	    sc, 0, sysctl_amd_rapl_display_cores_uj, "A",
	    "Cumulative per-core energy in microjoules, comma-separated per core");
	callout_reset_sbt(&sc->sampling_timer, SBT_1MS * AMD_RAPL_SAMPLE_UNIT,
	    SBT_1MS, amd_rapl_sample, sc, 0);
	return (0);
}

static int
amd_rapl_detach(device_t dev)
{
	struct amd_rapl_softc *sc = device_get_softc(dev);

	if (callout_active(&sc->sampling_timer))
		callout_drain(&sc->sampling_timer);
	mtx_destroy(&sc->mtx);
	free(sc->core_value, M_TEMP);
	free(sc->package_value, M_TEMP);
	return (0);
}

static int
amd_rapl_suspend(device_t dev)
{
	struct amd_rapl_softc *sc = device_get_softc(dev);

	if (callout_active(&sc->sampling_timer))
		callout_drain(&sc->sampling_timer);
	return (0);
}

static int
amd_rapl_resume(device_t dev)
{
	struct amd_rapl_softc *sc = device_get_softc(dev);

	if (callout_deactivate(&sc->sampling_timer)) {
		callout_reset_sbt(&sc->sampling_timer,
		    SBT_1MS * AMD_RAPL_SAMPLE_UNIT, SBT_1MS, amd_rapl_sample,
		    sc, 0);
	}
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
