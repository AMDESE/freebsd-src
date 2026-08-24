/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Test PMU event multiplexing across rotating groups.
 */

#include <sys/types.h>
#include <sys/sysctl.h>
#include <sys/time.h>

#include <err.h>
#include <errno.h>
#include <pmc.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define	MAX_GROUPS	2
#define	MAX_PER_GROUP	8
#define	MAX_EVENTS	(MAX_GROUPS * MAX_PER_GROUP)

static const char *event_pool[] = {
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
	"ls_alloc_mab_count",
	"ls_not_halted_cyc",
	"ls_dispatch.all",
        "ls_mab_alloc.ls",
        "ls_mab_alloc.hwpf",
        "ls_mab_alloc.all",
	"ls_int_taken",
	"ls_stlf",
};
#define	POOL_SIZE	(int)(sizeof(event_pool) / sizeof(event_pool[0]))

#define	MUX_PERIOD_MS	10
#define	WARMUP_NS	(200ULL * 1000 * 1000)
#define	WINDOW_NS	(1200ULL * 1000 * 1000)

struct pmu_grp {
	uint32_t	gid;
	int		nevents;
	int		committed;
	pmc_id_t	ids[MAX_PER_GROUP];
	const char	*names[MAX_PER_GROUP];
	pmc_value_t	v1[MAX_PER_GROUP];
	pmc_value_t	v2[MAX_PER_GROUP];
	pmc_value_t	vfin[MAX_PER_GROUP];
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

static void
busy_for(uint64_t ns)
{
	struct timespec t0, now;
	volatile uint64_t spin = 0;

	clock_gettime(CLOCK_MONOTONIC, &t0);
	for (;;) {
		for (int k = 0; k < 200000; k++)
			spin++;
		clock_gettime(CLOCK_MONOTONIC, &now);
		uint64_t elapsed = (uint64_t)(now.tv_sec - t0.tv_sec) *
		    1000000000ULL + (uint64_t)(now.tv_nsec - t0.tv_nsec);
		if (elapsed >= ns)
			return;
	}
}

static int
set_mux_period(int new_ms, int *saved)
{
	size_t s = sizeof(*saved);

	*saved = -1;
	if (sysctlbyname("kern.hwpmc.mux_period_ms", saved, &s,
	    &new_ms, sizeof(new_ms)) != 0) {
		*saved = -1;
		return (-1);
	}
	return (0);
}

static void
restore_mux_period(int saved)
{
	if (saved < 0)
		return;
	(void)sysctlbyname("kern.hwpmc.mux_period_ms", NULL, NULL,
	    &saved, sizeof(saved));
}

static int
build_group(struct pmu_grp *g, int n_target, int pool_start, int pool_end)
{
	int i;

	if (pmc_group_create(&g->gid) < 0) {
		warn("pmc_group_create");
		return (-1);
	}
	g->nevents = 0;
	for (i = pool_start; i < pool_end && g->nevents < n_target; i++) {
		uint32_t flags = 0;
		pmc_id_t id;

		if (g->nevents == 0)
			flags |= PMC_F_GROUP_MUX;
		if (pmc_allocate_group(event_pool[i], PMC_MODE_TC, flags,
		    PMC_CPU_ANY, &id, 0) < 0) {
			/* Skip unsupported event. */
			continue;
		}
		if (pmc_group_add(g->gid, id, g->nevents == 0) < 0) {
			(void)pmc_release(id);
			continue;
		}
		g->ids[g->nevents] = id;
		g->names[g->nevents] = event_pool[i];
		g->nevents++;
	}
	return (g->nevents);
}

static void
release_group(struct pmu_grp *g)
{
	int i;

	if (g->committed)
		(void)pmc_release(g->ids[0]);
	else {
		for (i = 0; i < g->nevents; i++)
			(void)pmc_release(g->ids[i]);
	}
	g->committed = 0;
	g->nevents = 0;
}

int
main(void)
{
	struct pmu_grp grps[MAX_GROUPS];
	int core, saved_period;
	int per_group, total, i, j, rc;

	memset(grps, 0, sizeof(grps));

	if (pmc_init() < 0)
		err(1, "pmc_init");
	if (!is_amd()) {
		printf("SKIP: non-AMD CPU\n");
		return (77);
	}
	core = probe_core_pmcs();
	if (core < 2) {
		printf("SKIP: only %d core PMCs available\n", core);
		return (77);
	}

	/* Calculate group size to oversubscribe hardware counters. */
	per_group = (core * 2) / 3;
	if (per_group < 2)
		per_group = 2;
	if (per_group > MAX_PER_GROUP)
		per_group = MAX_PER_GROUP;
	total = per_group * MAX_GROUPS;
	if (total <= core) {
		printf("SKIP: %d-counter CPU; cannot oversubscribe with "
		    "%d groups of %d\n", core, MAX_GROUPS, per_group);
		return (77);
	}

	/* Create groups using distinct events from pool. */
	int pool_used = 0;
	for (i = 0; i < MAX_GROUPS; i++) {
		int got = build_group(&grps[i], per_group, pool_used,
		    POOL_SIZE);
		if (got < per_group) {
			fprintf(stderr,
			    "SKIP: only %d events available for group %d "
			    "(needed %d)\n", got, i, per_group);
			for (j = 0; j <= i; j++)
				release_group(&grps[j]);
			return (77);
		}
		/* Advance pool index for the next group. */
		pool_used += per_group;
		if (pool_used >= POOL_SIZE) {
			fprintf(stderr, "SKIP: ran out of distinct events\n");
			for (j = 0; j <= i; j++)
				release_group(&grps[j]);
			return (77);
		}
	}

	printf("inter-group multiplex test: %d groups x %d events on "
	    "%d HW core counters\n", MAX_GROUPS, per_group, core);

	for (i = 0; i < MAX_GROUPS; i++) {
		if (pmc_group_commit(grps[i].gid) < 0) {
			warn("pmc_group_commit g%d", i);
			for (j = 0; j < MAX_GROUPS; j++)
				release_group(&grps[j]);
			return (1);
		}
		grps[i].committed = 1;
	}

	for (i = 0; i < MAX_GROUPS; i++) {
		if (pmc_attach(grps[i].ids[0], getpid()) < 0) {
			warn("pmc_attach group %d leader", i);
			for (int k = 0; k < MAX_GROUPS; k++)
				release_group(&grps[k]);
			return (1);
		}
	}

	if (set_mux_period(MUX_PERIOD_MS, &saved_period) == 0)
		printf("set kern.hwpmc.mux_period_ms = %d (was %d)\n",
		    MUX_PERIOD_MS, saved_period);
	else
		printf("note: could not write kern.hwpmc.mux_period_ms; "
		    "using kernel default\n");

	for (i = 0; i < MAX_GROUPS; i++) {
		if (pmc_start(grps[i].ids[0]) < 0) {
			warn("pmc_start group %d leader", i);
			restore_mux_period(saved_period);
			for (j = 0; j < MAX_GROUPS; j++)
				release_group(&grps[j]);
			return (1);
		}
	}

	busy_for(WARMUP_NS);

	for (i = 0; i < MAX_GROUPS; i++)
		for (j = 0; j < grps[i].nevents; j++) {
			grps[i].v1[j] = 0;
			(void)pmc_read(grps[i].ids[j], &grps[i].v1[j]);
		}

	busy_for(WINDOW_NS);

	for (i = 0; i < MAX_GROUPS; i++)
		for (j = 0; j < grps[i].nevents; j++) {
			grps[i].v2[j] = 0;
			(void)pmc_read(grps[i].ids[j], &grps[i].v2[j]);
		}

	for (i = 0; i < MAX_GROUPS; i++)
		(void)pmc_stop(grps[i].ids[0]);

	for (i = 0; i < MAX_GROUPS; i++)
		for (j = 0; j < grps[i].nevents; j++) {
			grps[i].vfin[j] = 0;
			(void)pmc_read(grps[i].ids[j], &grps[i].vfin[j]);
		}

	/*
	 * Validation:
	 * (a) All events in all groups must advance.
	 * (b) All siblings within a group must advance together.
	 */
	rc = 0;
	pmc_value_t group_cycles[MAX_GROUPS] = { 0 };
	for (i = 0; i < MAX_GROUPS; i++) {
		printf("\nGroup %u (gid=%u):\n", i, grps[i].gid);
		printf("  %-26s %16s %16s %16s %16s\n",
		    "event", "snapshot1", "snapshot2", "final", "delta");
		int progressed = 0, nonzero = 0;
		for (j = 0; j < grps[i].nevents; j++) {
			pmc_value_t d = grps[i].v2[j] > grps[i].v1[j] ?
			    grps[i].v2[j] - grps[i].v1[j] : 0;
			printf("  %-26s %16ju %16ju %16ju %16ju\n",
			    grps[i].names[j],
			    (uintmax_t)grps[i].v1[j],
			    (uintmax_t)grps[i].v2[j],
			    (uintmax_t)grps[i].vfin[j],
			    (uintmax_t)d);
			if (d > 0)
				progressed++;
			if (grps[i].vfin[j] > 0)
				nonzero++;
			/* Track highest event count in group. */
			if (d > group_cycles[i])
				group_cycles[i] = d;
		}

		/* Verify all sibling events advanced. */
		if (nonzero != grps[i].nevents) {
			fprintf(stderr,
			    "FAIL: group %d: %d/%d siblings finished with "
			    "zero count -- within-group atomicity broken\n",
			    i, grps[i].nevents - nonzero, grps[i].nevents);
			rc = 1;
		}
		if (progressed != grps[i].nevents) {
			fprintf(stderr,
			    "FAIL: group %d: %d/%d siblings did not advance "
			    "between snapshots -- inter-group rotation is "
			    "not reaching them\n", i,
			    grps[i].nevents - progressed, grps[i].nevents);
			rc = 1;
		}
	}

	/* Verify equal execution across groups. */
	if (MAX_GROUPS == 2 && group_cycles[0] > 0 && group_cycles[1] > 0) {
		pmc_value_t hi = group_cycles[0] > group_cycles[1] ?
		    group_cycles[0] : group_cycles[1];
		pmc_value_t lo = group_cycles[0] < group_cycles[1] ?
		    group_cycles[0] : group_cycles[1];
		printf("\nfairness: max-rate per group: g0=%ju g1=%ju "
		    "(ratio hi/lo = %ju)\n",
		    (uintmax_t)group_cycles[0],
		    (uintmax_t)group_cycles[1],
		    (uintmax_t)(hi / lo));
		if (hi / lo > 64) {
			fprintf(stderr,
			    "FAIL: inter-group rotation is unfair: "
			    "one group got %jux more HW time than the "
			    "other\n", (uintmax_t)(hi / lo));
			rc = 1;
		}
	}

	for (i = 0; i < MAX_GROUPS; i++)
		release_group(&grps[i]);
	restore_mux_period(saved_period);

	if (rc == 0)
		printf("\npmc_mux_works_test: OK (%d groups x %d events on "
		    "%d HW core counters, %d-ms rotation)\n",
		    MAX_GROUPS, per_group, core, MUX_PERIOD_MS);
	return (rc);
}
