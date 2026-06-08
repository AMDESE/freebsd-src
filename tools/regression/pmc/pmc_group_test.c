/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Regression test for hwpmc grouping (phase 1).
 *
 * Build:  cc -o pmc_group_test pmc_group_test.c -lpmc
 * Run:    sudo ./pmc_group_test     (requires hwpmc loaded, AMD CPU)
 */

#include <sys/types.h>
#include <sys/sysctl.h>

#include <err.h>
#include <errno.h>
#include <pmc.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static int
is_amd(void)
{
	char buf[64];
	size_t s = sizeof(buf);

	if (sysctlbyname("kern.hwpmc.cpuid", buf, &s, NULL, 0) != 0)
		return (0);
	return (strstr(buf, "AuthenticAMD") != NULL ||
	    strstr(buf, "HygonGenuine") != NULL);
}

/*
 * Probe how many process-mode core PMCs we can simultaneously allocate
 * with the canonical "instructions" event.  Returns the count without
 * leaving any allocations behind.  Used to size the rest of the test
 * dynamically per-CPU instead of relying on pmc_npmc(0), which sums
 * SOFT/TSC/K8/IBS classes and is therefore meaningless for sizing a
 * single core-class group.
 */
static int
probe_core_pmcs(void)
{
	pmc_id_t ids[64];
	int n = 0;

	while (n < (int)(sizeof(ids) / sizeof(ids[0]))) {
		if (pmc_allocate("instructions", PMC_MODE_TC, 0,
		    PMC_CPU_ANY, &ids[n], 0) < 0)
			break;
		n++;
	}
	for (int i = 0; i < n; i++)
		(void)pmc_release(ids[i]);
	return (n);
}

static int
test_basic_group(void)
{
	uint32_t gid;
	pmc_id_t pmc0, pmc1, pmc2;
	pmc_value_t v0, v1, v2;
	const char *ev0 = "instructions";
	const char *ev1 = "unhalted-cycles";
	const char *ev2 = "branches";
	volatile uint64_t spin;

	if (pmc_group_create(&gid) < 0) {
		warn("pmc_group_create");
		return (1);
	}
	if (pmc_allocate_group(ev0, PMC_MODE_TC, 0, PMC_CPU_ANY,
	    &pmc0, 0) < 0) {
		warn("allocate %s", ev0);
		return (1);
	}
	if (pmc_allocate_group(ev1, PMC_MODE_TC, 0, PMC_CPU_ANY,
	    &pmc1, 0) < 0) {
		warn("allocate %s", ev1);
		return (1);
	}
	if (pmc_allocate_group(ev2, PMC_MODE_TC, 0, PMC_CPU_ANY,
	    &pmc2, 0) < 0) {
		warn("allocate %s", ev2);
		return (1);
	}
	if (pmc_group_add(gid, pmc0, 1) < 0) {
		warn("group_add leader");
		return (1);
	}
	if (pmc_group_add(gid, pmc1, 0) < 0) {
		warn("group_add sibling 1");
		return (1);
	}
	if (pmc_group_add(gid, pmc2, 0) < 0) {
		warn("group_add sibling 2");
		return (1);
	}
	if (pmc_group_commit(gid) < 0) {
		warn("group_commit");
		return (1);
	}

	/*
	 * Process-mode PMCs require an explicit attach before start.
	 * pmu_group_target_proc() now insists on an attached target proc
	 * (the owner is intentionally NOT used as a fallback because that
	 * confused csw_in/out under the multiplex rework), so calling
	 * pmc_start without first pmc_attach'ing to ourselves now returns
	 * EINVAL.  Attach every sibling so the whole group can bind.
	 */
	if (pmc_attach(pmc0, getpid()) < 0) {
		warn("pmc_attach pmc0");
		return (1);
	}
	if (pmc_attach(pmc1, getpid()) < 0) {
		warn("pmc_attach pmc1");
		return (1);
	}
	if (pmc_attach(pmc2, getpid()) < 0) {
		warn("pmc_attach pmc2");
		return (1);
	}

	if (pmc_start(pmc0) < 0) {
		warn("pmc_start leader");
		return (1);
	}
	if (pmc_start(pmc1) < 0) {
		warn("pmc_start sibling 1");
		return (1);
	}
	if (pmc_start(pmc2) < 0) {
		warn("pmc_start sibling 2");
		return (1);
	}

	for (spin = 0; spin < 100000000ULL; spin++)
		;

	if (pmc_read(pmc0, &v0) < 0 || pmc_read(pmc1, &v1) < 0 ||
	    pmc_read(pmc2, &v2) < 0) {
		warn("pmc_read");
		return (1);
	}
	(void)pmc_stop(pmc0);
	(void)pmc_stop(pmc1);
	(void)pmc_stop(pmc2);
	printf("3-event group: pmc0=%ju pmc1=%ju pmc2=%ju\n",
	    (uintmax_t)v0, (uintmax_t)v1, (uintmax_t)v2);
	if (v0 == 0 || v1 == 0 || v2 == 0) {
		fprintf(stderr, "FAIL: at least one sibling never counted\n");
		(void)pmc_release(pmc0);
		(void)pmc_release(pmc1);
		(void)pmc_release(pmc2);
		return (1);
	}
	(void)pmc_release(pmc0);
	(void)pmc_release(pmc1);
	(void)pmc_release(pmc2);
	return (0);
}

/*
 * The atomic-group scheduler must reject a single group whose event
 * count exceeds the core HW counter pool.  Within-group placement is
 * all-or-none, so a group that cannot fit at commit MUST fail rather
 * than silently get split across rotation windows.
 *
 * Sizing: query the actual per-class core count via probe_core_pmcs()
 * (Zen5 = 6, Zen3/4 = 6, Zen6 = up to 12, EPYC = vendor-dependent),
 * then attempt a group of (core + 2) events.  Hard-coding 16 missed
 * Zen6 and any future generation that exceeds it.
 */
static int
test_oversubscription_rejected(void)
{
	uint32_t gid;
	pmc_id_t *ids;
	int core, target, i, allocated, err;

	core = probe_core_pmcs();
	if (core <= 0) {
		printf("SKIP: probe_core_pmcs returned %d\n", core);
		return (0);
	}
	target = core + 2;
	ids = calloc(target, sizeof(*ids));
	if (ids == NULL)
		return (1);

	if (pmc_group_create(&gid) < 0) {
		free(ids);
		return (1);
	}
	allocated = 0;
	for (i = 0; i < target; i++) {
		if (pmc_allocate_group("instructions", PMC_MODE_TC, 0,
		    PMC_CPU_ANY, &ids[i], 0) < 0)
			break;
		if (pmc_group_add(gid, ids[i], i == 0) < 0)
			break;
		allocated++;
	}
	if (allocated <= core) {
		fprintf(stderr,
		    "SKIP: only allocated %d events for a %d-counter pool; "
		    "cannot oversubscribe\n", allocated, core);
		for (i = 0; i < allocated; i++)
			(void)pmc_release(ids[i]);
		free(ids);
		return (0);
	}
	err = pmc_group_commit(gid);
	for (i = 0; i < allocated; i++)
		(void)pmc_release(ids[i]);
	free(ids);
	if (err == 0) {
		fprintf(stderr,
		    "FAIL: oversubscribed commit succeeded (allocated=%d "
		    "core_pool=%d)\n", allocated, core);
		return (1);
	}
	if (errno != ENOSPC && errno != EOPNOTSUPP) {
		fprintf(stderr,
		    "FAIL: expected ENOSPC/EOPNOTSUPP got errno=%d\n", errno);
		return (1);
	}
	printf("oversubscription rejected (allocated=%d core_pool=%d "
	    "errno=%d)\n", allocated, core, errno);
	return (0);
}

int
main(void)
{
	if (pmc_init() < 0)
		err(1, "pmc_init");
	if (!is_amd()) {
		printf("SKIP: non-AMD CPU\n");
		return (77);
	}
	if (test_basic_group() != 0)
		return (1);
	if (test_oversubscription_rejected() != 0)
		return (1);
	printf("pmc_group_test: OK\n");
	return (0);
}
