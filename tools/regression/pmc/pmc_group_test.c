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

/*
 * Must match PMC_HANDLE_DEFERRED_SLOTS in <sys/pmc.h>, which is only
 * visible under _KERNEL and so cannot be included here.  A group may hold
 * PMC_GROUP_MAX_MEMBERS (32) members, so the pool has to supply that many
 * concurrent deferred handles per (owner, CPU).
 */
#define	TEST_DEFERRED_HANDLE_SLOTS	32

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

static int
expect_errno(const char *operation, int error, int expected)
{
	if (error < 0 && errno == expected)
		return (0);
	fprintf(stderr, "FAIL: %s: expected errno %d, got error=%d errno=%d\n",
	    operation, expected, error, errno);
	return (1);
}

static int
test_allocation_gates(void)
{
	pmc_id_t pmcid;

	pmcid = PMC_ID_INVALID;
	if (expect_errno("grouped descendants",
	    pmc_allocate_group("instructions", PMC_MODE_TC,
	    PMC_F_DESCENDANTS, PMC_CPU_ANY, &pmcid, 0), EOPNOTSUPP) != 0)
		return (1);
	if (expect_errno("grouped counting initial value",
	    pmc_allocate_group("instructions", PMC_MODE_TC, 0,
	    PMC_CPU_ANY, &pmcid, 1), EINVAL) != 0)
		return (1);
	if (pmc_allocate_group("instructions", PMC_MODE_TS, 0,
	    PMC_CPU_ANY, &pmcid, 1000) < 0) {
		warn("grouped sampling reload count");
		return (1);
	}
	return (pmc_release(pmcid) < 0);
}

static int
test_deferred_handle_namespace(void)
{
	pmc_id_t fresh, ids[TEST_DEFERRED_HANDLE_SLOTS], stale, system;
	pmc_value_t value;
	int i, j, n, rc;

	for (i = 0; i < 160; i++) {
		if (pmc_allocate_group("instructions", PMC_MODE_TC, 0,
		    PMC_CPU_ANY, &fresh, 0) < 0 || pmc_release(fresh) < 0) {
			warn("deferred handle recycle %d", i);
			return (1);
		}
	}

	if (pmc_allocate_group("instructions", PMC_MODE_TC, 0,
	    PMC_CPU_ANY, &stale, 0) < 0 || pmc_release(stale) < 0)
		return (1);
	if (pmc_allocate_group("instructions", PMC_MODE_TC, 0,
	    PMC_CPU_ANY, &fresh, 0) < 0)
		return (1);
	if (fresh == stale || expect_errno("stale deferred handle",
	    pmc_read(stale, &value), EINVAL) != 0) {
		(void)pmc_release(fresh);
		return (1);
	}
	if (pmc_release(fresh) < 0)
		return (1);

	n = 0;
	while (n < TEST_DEFERRED_HANDLE_SLOTS) {
		if (pmc_allocate_group("instructions", PMC_MODE_TC, 0,
		    PMC_CPU_ANY, &ids[n], 0) < 0)
			break;
		for (j = 0; j < n; j++) {
			if (ids[j] == ids[n]) {
				fprintf(stderr,
				    "FAIL: duplicate deferred handle %#x\n", ids[n]);
				n++;
				goto fail;
			}
		}
		n++;
	}
	if (n != TEST_DEFERRED_HANDLE_SLOTS) {
		fprintf(stderr, "FAIL: allocated %d of %d deferred handles\n",
		    n, TEST_DEFERRED_HANDLE_SLOTS);
		goto fail;
	}
	errno = 0;
	if (pmc_allocate_group("instructions", PMC_MODE_TC, 0,
	    PMC_CPU_ANY, &fresh, 0) == 0) {
		fprintf(stderr, "FAIL: deferred handle exhaustion succeeded\n");
		(void)pmc_release(fresh);
		goto fail;
	}
	if (errno != EMFILE) {
		fprintf(stderr, "FAIL: deferred handle exhaustion errno=%d\n",
		    errno);
		goto fail;
	}
	if (pmc_allocate_group("instructions", PMC_MODE_SC, 0, 0,
	    &system, 0) < 0) {
		warn("per-CPU deferred handle namespace");
		goto fail;
	}
	if (pmc_release(system) < 0)
		goto fail;
	rc = 0;
	for (i = 0; i < n; i++) {
		if (pmc_release(ids[i]) < 0) {
			warn("release deferred handle %d", i);
			rc = 1;
		}
	}
	return (rc);

fail:
	for (i = 0; i < n; i++)
		(void)pmc_release(ids[i]);
	return (1);
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
	if (expect_errno("attach before commit",
	    pmc_attach(pmc0, getpid()), EINVAL) != 0)
		return (1);
	if (pmc_group_commit(gid) < 0) {
		warn("group_commit");
		return (1);
	}
	if (expect_errno("non-leader attach",
	    pmc_attach(pmc1, getpid()), ENOTTY) != 0)
		return (1);
	if (expect_errno("committed setcount", pmc_set(pmc1, 0), EBUSY) != 0)
		return (1);
	if (pmc_attach(pmc0, getpid()) < 0) {
		warn("pmc_attach leader");
		return (1);
	}
	if (expect_errno("second attach",
	    pmc_attach(pmc0, getpid()), EBUSY) != 0)
		return (1);
	if (expect_errno("non-leader start", pmc_start(pmc1), ENOTTY) != 0)
		return (1);
	if (pmc_start(pmc0) < 0) {
		warn("pmc_start leader");
		return (1);
	}
	if (expect_errno("non-leader stop", pmc_stop(pmc1), ENOTTY) != 0)
		return (1);

	for (spin = 0; spin < 100000000ULL; spin++)
		;

	if (pmc_read(pmc0, &v0) < 0 || pmc_read(pmc1, &v1) < 0 ||
	    pmc_read(pmc2, &v2) < 0) {
		warn("pmc_read");
		return (1);
	}
	(void)pmc_stop(pmc0);
	printf("3-event group: pmc0=%ju pmc1=%ju pmc2=%ju\n",
	    (uintmax_t)v0, (uintmax_t)v1, (uintmax_t)v2);
	if (v0 == 0 || v1 == 0 || v2 == 0) {
		fprintf(stderr, "FAIL: at least one sibling never counted\n");
		(void)pmc_release(pmc0);
		return (1);
	}
	(void)pmc_release(pmc0);
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
	int core, target, i, allocated, commit_errno, err;

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
	commit_errno = errno;
	if (err == 0)
		(void)pmc_release(ids[0]);
	else {
		for (i = 0; i < allocated; i++)
			(void)pmc_release(ids[i]);
	}
	free(ids);
	if (err == 0) {
		fprintf(stderr,
		    "FAIL: oversubscribed commit succeeded (allocated=%d "
		    "core_pool=%d)\n", allocated, core);
		return (1);
	}
	if (commit_errno != ENOSPC) {
		fprintf(stderr, "FAIL: expected ENOSPC got errno=%d\n",
		    commit_errno);
		return (1);
	}
	printf("oversubscription rejected (allocated=%d core_pool=%d "
	    "errno=%d)\n", allocated, core, commit_errno);
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
	if (test_allocation_gates() != 0)
		return (1);
	if (test_deferred_handle_namespace() != 0)
		return (1);
	if (test_basic_group() != 0)
		return (1);
	if (test_oversubscription_rejected() != 0)
		return (1);
	printf("pmc_group_test: OK\n");
	return (0);
}
