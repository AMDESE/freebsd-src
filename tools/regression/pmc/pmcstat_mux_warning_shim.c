/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Deterministic libpmc/sysctl boundary for pmcstat multiplex-warning tests.
 */

#include <sys/types.h>
#include <sys/sysctl.h>

#include <errno.h>
#include <pmc.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

static pmc_id_t next_pmcid = 1;

static int
copy_sysctl_value(void *oldp, size_t *oldlenp, const void *value,
    size_t value_size)
{

	if (oldp == NULL || oldlenp == NULL || *oldlenp < value_size) {
		errno = ENOMEM;
		return (-1);
	}
	memcpy(oldp, value, value_size);
	*oldlenp = value_size;
	return (0);
}

static int
read_time(const char *name, uint64_t *value)
{
	const char *text;
	char *end;
	unsigned long long parsed;

	text = getenv(name);
	if (text == NULL) {
		errno = EINVAL;
		return (-1);
	}
	errno = 0;
	parsed = strtoull(text, &end, 0);
	if (errno != 0 || *text == '\0' || *end != '\0') {
		errno = EINVAL;
		return (-1);
	}
	*value = parsed;
	return (0);
}

int
sysctlbyname(const char *name, void *oldp, size_t *oldlenp,
    const void *newp, size_t newlen)
{
	uint64_t freq;
	int value;

	if (newp != NULL || newlen != 0) {
		errno = EPERM;
		return (-1);
	}
	if (strcmp(name, "vm.ndomains") == 0) {
		value = 1;
		return (copy_sysctl_value(oldp, oldlenp, &value,
		    sizeof(value)));
	}
	if (strcmp(name, "machdep.tsc_freq") == 0) {
		freq = 1000;
		return (copy_sysctl_value(oldp, oldlenp, &freq, sizeof(freq)));
	}
	if (strcmp(name, "kern.hwpmc.mux_period_ms") == 0) {
		value = 1;
		return (copy_sysctl_value(oldp, oldlenp, &value,
		    sizeof(value)));
	}
	errno = ENOENT;
	return (-1);
}

int
pmc_init(void)
{

	return (0);
}

int
pmc_npmc(int cpu)
{

	(void)cpu;
	return (6);
}

int
pmc_allocate(const char *spec, enum pmc_mode mode, uint32_t flags, int cpu,
    pmc_id_t *pmcid, uint64_t count)
{

	(void)spec;
	(void)mode;
	(void)flags;
	(void)cpu;
	(void)count;
	*pmcid = next_pmcid++;
	return (0);
}

int
pmc_allocate_group(const char *spec, enum pmc_mode mode, uint32_t flags,
    int cpu, pmc_id_t *pmcid, uint64_t count)
{

	return (pmc_allocate(spec, mode, flags, cpu, pmcid, count));
}

int
pmc_capabilities(pmc_id_t pmcid, uint32_t *caps)
{

	(void)pmcid;
	*caps = 0;
	return (0);
}

int
pmc_group_create(uint32_t *groupid)
{

	*groupid = 1;
	return (0);
}

int
pmc_group_add(uint32_t groupid, pmc_id_t pmcid, int leader)
{

	(void)groupid;
	(void)pmcid;
	(void)leader;
	return (0);
}

int
pmc_group_commit(uint32_t groupid)
{

	(void)groupid;
	return (0);
}

int
pmc_group_read(pmc_id_t leader, uint32_t *nmembers,
    struct pmc_group_member *members, struct pmc_group_times *times)
{
	uint64_t enabled, running;
	uint32_t capacity;

	(void)leader;
	if (nmembers == NULL || read_time("PMCSTAT_TEST_ENABLED", &enabled) != 0 ||
	    read_time("PMCSTAT_TEST_RUNNING", &running) != 0)
		return (-1);
	capacity = *nmembers;
	*nmembers = 2;
	if (capacity != 0 && capacity < 2) {
		errno = E2BIG;
		return (-1);
	}
	if (members != NULL) {
		memset(members, 0, 2 * sizeof(*members));
		members[0].pm_pmcid = 1;
		members[1].pm_pmcid = 2;
	}
	if (times != NULL) {
		memset(times, 0, sizeof(*times));
		times->pgt_enabled = enabled;
		times->pgt_running = running;
		times->pgt_flags = PMC_GROUP_F_TIME_WALL_NS;
	}
	return (0);
}

int
pmc_configure_logfile(int fd)
{

	(void)fd;
	return (0);
}

int
pmc_close_logfile(void)
{

	return (0);
}

int
pmc_flush_logfile(void)
{

	return (0);
}

int
pmc_get_driver_stats(struct pmc_driverstats *stats)
{

	memset(stats, 0, sizeof(*stats));
	return (0);
}

int
pmc_set(pmc_id_t pmcid, pmc_value_t value)
{

	(void)pmcid;
	(void)value;
	return (0);
}

int
pmc_start(pmc_id_t pmcid)
{

	(void)pmcid;
	return (0);
}

int
pmc_stop(pmc_id_t pmcid)
{

	(void)pmcid;
	return (0);
}

int
pmc_release(pmc_id_t pmcid)
{

	(void)pmcid;
	return (0);
}

int
pmc_width(pmc_id_t pmcid, uint32_t *width)
{

	(void)pmcid;
	*width = 48;
	return (0);
}

int
pmc_read(pmc_id_t pmcid, pmc_value_t *value)
{

	(void)pmcid;
	*value = 0;
	return (0);
}

int
pmc_attach(pmc_id_t pmcid, pid_t pid)
{

	(void)pmcid;
	(void)pid;
	return (0);
}

int
pmc_pmu_enabled(void)
{

	return (1);
}

void
pmc_pmu_print_counters(const char *filter)
{

	(void)filter;
}

void
pmc_pmu_print_counter_desc(const char *event)
{

	(void)event;
}

uint64_t
pmc_pmu_sample_rate_get(const char *event)
{

	(void)event;
	return (100);
}

const char *
pmc_name_of_cputype(enum pmc_cputype cputype)
{

	(void)cputype;
	return ("test");
}
