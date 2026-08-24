/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Test rejection of single groups larger than hardware capacity.
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

#define	MAX_EVENTS	24

static const char *events[MAX_EVENTS] = {
	"instructions",
	"unhalted-cycles",
	"branches",
	"branch-misses",
	"cache-references",
	"cache-misses",
	"l1d-loads",
	"l1d-load-misses",
	"l2-cache-references",
	"l2-cache-misses",
	"dispatch-stalls",
	"fp-retired",
	"branches-retired",
	"de-no-dispatch-per-slot",
	"l2-pf-hit-l2",
	"l2-pf-hit-l3",
	"ls_alloc_mab_count",
	"ls_not_halted_cyc",
	"ls_dispatch.all",
        "ls_mab_alloc.ls",
        "ls_mab_alloc.hwpf",
        "ls_mab_alloc.all",
	"ls_int_taken",
	"ls_stlf",
};

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

/* Return count of available core hardware counters. */
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

int
main(void)
{
	pmc_id_t ids[MAX_EVENTS];
	uint32_t gid = 0;
	int i, core, target, allocated, commit_errno, rc;

	if (pmc_init() < 0)
		err(1, "pmc_init");
	if (!is_amd()) {
		printf("SKIP: non-AMD CPU\n");
		return (77);
	}
	core = probe_core_pmcs();
	if (core < 2) {
		printf("SKIP: probe found only %d core PMCs\n", core);
		return (77);
	}

	/* Create group with (core + 2) events. */
	target = core + 2;
	if (target > MAX_EVENTS) {
		printf("SKIP: %d-counter CPU exceeds test pool (%d)\n",
		    core, MAX_EVENTS);
		return (77);
	}

	if (pmc_group_create(&gid) < 0)
		err(1, "pmc_group_create");

	allocated = 0;
	for (i = 0; i < MAX_EVENTS && allocated < target; i++) {
		uint32_t flags = 0;

		if (allocated == 0)
			flags |= PMC_F_GROUP_MUX;
		if (pmc_allocate_group(events[i], PMC_MODE_TC, flags,
		    PMC_CPU_ANY, &ids[allocated], 0) < 0) {
			/* Skip unsupported event. */
			continue;
		}
		if (pmc_group_add(gid, ids[allocated], allocated == 0) < 0) {
			(void)pmc_release(ids[allocated]);
			continue;
		}
		allocated++;
	}

	if (allocated <= core) {
		fprintf(stderr,
		    "SKIP: only %d events allocated, can't oversubscribe "
		    "a %d-counter core pool\n", allocated, core);
		for (i = 0; i < allocated; i++)
			(void)pmc_release(ids[i]);
		return (77);
	}

	rc = pmc_group_commit(gid);
	commit_errno = errno;
	if (rc == 0)
		(void)pmc_release(ids[0]);
	else {
		for (i = 0; i < allocated; i++)
			(void)pmc_release(ids[i]);
	}

	/* Single group exceeding hardware capacity must fail commit with ENOSPC. */
	if (rc == 0) {
		fprintf(stderr,
		    "FAIL: pmc_group_commit succeeded for %d events on "
		    "a %d-counter core pool; a single group must not be "
		    "split (within-group atomicity)\n", allocated, core);
		return (1);
	}
	if (commit_errno != ENOSPC) {
		fprintf(stderr,
		    "FAIL: expected ENOSPC from oversubscribed commit, "
		    "got errno=%d (%s)\n", commit_errno,
		    strerror(commit_errno));
		return (1);
	}

	printf("oversubscribed single group rejected (events=%d "
	    "core_pool=%d errno=%d)\n", allocated, core, commit_errno);
	printf("pmc_mux_test: OK\n");
	return (0);
}
