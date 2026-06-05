/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Regression test: a SINGLE group with more events than the class has
 * HW counters must be rejected at commit time.  Within-group placement
 * is strictly all-or-none -- there is no longer an in-group multiplex
 * mode that splits one group across rotation windows.  Inter-group
 * rotation (multiple groups whose union exceeds HW) is exercised by
 * pmc_mux_works_test.c instead.
 *
 * Build:  cc -o pmc_mux_test pmc_mux_test.c -lpmc
 * Run:    sudo ./pmc_mux_test     (requires hwpmc loaded, AMD CPU)
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

/*
 * Probe the actual per-class core PMC capacity by allocating
 * "instructions" repeatedly.  pmc_npmc(0) sums all classes
 * (SOFT/TSC/K8/IBS) and is therefore meaningless for sizing a
 * single core-class group: on Zen5 it returns 47 even though only
 * 6 core counters are available.
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

int
main(void)
{
	pmc_id_t ids[MAX_EVENTS];
	uint32_t gid = 0;
	int i, core, target, allocated, rc;

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

	/*
	 * Build a single group with (core + 2) events and demand that
	 * commit reject it.  Capped at MAX_EVENTS so we don't try to
	 * exceed our event_pool[].
	 */
	target = core + 2;
	if (target > MAX_EVENTS) {
		printf("SKIP: %d-counter CPU exceeds test pool (%d)\n",
		    core, MAX_EVENTS);
		return (77);
	}

	if (pmc_group_create(&gid) < 0)
		err(1, "pmc_group_create");

	allocated = 0;
	for (i = 0; i < target; i++) {
		uint32_t flags = 0;

		if (i == 0)
			flags |= PMC_F_GROUP_MUX;
		if (pmc_allocate_group(events[i], PMC_MODE_TC, flags,
		    PMC_CPU_ANY, &ids[i], 0) < 0) {
			/* Some events may not be supported on this CPU. */
			continue;
		}
		if (pmc_group_add(gid, ids[i], allocated == 0) < 0) {
			(void)pmc_release(ids[i]);
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
	for (i = 0; i < allocated; i++)
		(void)pmc_release(ids[i]);

	/*
	 * Under the strictly-atomic group scheduler a single group
	 * whose nevents exceeds the class total must be rejected here:
	 * within-group placement is all-or-none and would otherwise
	 * sacrifice that invariant the moment we tried to schedule it.
	 */
	if (rc == 0) {
		fprintf(stderr,
		    "FAIL: pmc_group_commit succeeded for %d events on "
		    "a %d-counter core pool; a single group must not be "
		    "split (within-group atomicity)\n", allocated, core);
		return (1);
	}
	if (errno != ENOSPC && errno != EOPNOTSUPP) {
		fprintf(stderr,
		    "FAIL: expected ENOSPC/EOPNOTSUPP from oversubscribed "
		    "commit, got errno=%d (%s)\n", errno, strerror(errno));
		return (1);
	}

	printf("oversubscribed single group rejected (events=%d "
	    "core_pool=%d errno=%d)\n", allocated, core, errno);
	printf("pmc_mux_test: OK\n");
	return (0);
}
