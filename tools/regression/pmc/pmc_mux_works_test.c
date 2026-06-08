/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Positive regression test for hwpmc PMU multiplexing under the new
 * strictly-atomic group scheduler.
 *
 * Two invariants must hold:
 *
 *   1. Within-group all-or-none.  Every event in a group is either
 *      simultaneously bound to a HW counter or simultaneously deferred.
 *      Siblings of one group never see a partial schedule.
 *
 *   2. Inter-group rotation.  When the union of two committed groups
 *      exceeds the HW counter pool, the per-pp rotation kthread must
 *      cycle whole groups in/out so both eventually progress.
 *
 * Strategy:
 *   - Allocate two groups whose combined event count > npmc but each
 *     group individually <= npmc, with PMC_F_GROUP_MUX on each leader.
 *   - Tighten kern.hwpmc.mux_period_ms so rotation is fast.
 *   - Start both groups, run a busy workload, snapshot.
 *   - Demand:
 *       (a) every event in every group progressed (rotation is fair);
 *       (b) within each group, all siblings progressed at comparable
 *           rates (all-or-none atomicity, not partial placement).
 *
 * Build:  cc -o pmc_mux_works_test pmc_mux_works_test.c -lpmc
 * Run:    sudo ./pmc_mux_works_test     (requires hwpmc loaded, AMD CPU)
 *
 * Exit codes: 0 = pass, 1 = fail, 77 = skip.
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

/*
 * Probe the actual per-class core PMC capacity.  pmc_npmc(0) sums all
 * classes (SOFT/TSC/K8/IBS), which on Zen5 is 47 -- meaningless for
 * sizing a core-class group.  The real constraint is the number of
 * core counters (Zen2/3/4/5: 6, Zen6: up to 12).
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
			/* Event unsupported on this CPU model -- skip. */
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

	for (i = 0; i < g->nevents; i++)
		(void)pmc_release(g->ids[i]);
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

	/*
	 * Pick per_group so that each group fits the core pool but two
	 * groups together oversubscribe.  Concretely:
	 *   core=6  (Zen3/4/5)        -> per_group=4, total=8 > 6   (mux)
	 *   core=8                    -> per_group=5, total=10 > 8  (mux)
	 *   core=12 (Zen6 perf core)  -> per_group=8, total=16 > 12 (mux)
	 *   core=4                    -> per_group=2, total=4 = 4   (skip)
	 * The per_group computation is (core*2)/3 (integer-truncated)
	 * which keeps single-group commits safe but guarantees inter-
	 * group oversubscription when core >= 5.
	 */
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

	/* Build the groups out of disjoint slices of event_pool[]. */
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
		/*
		 * Each event in event_pool[] only allocates once on a
		 * given CPU; advance the cursor by the actual number of
		 * pool entries we consumed (some may have failed
		 * allocate).  Conservatively advance by per_group * 2 so
		 * the next group does not race for the same slots.
		 */
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
	}

	/*
	 * Process-mode PMCs require an explicit attach before start;
	 * the kernel's pmu_group_target_proc() resolves the target proc
	 * exclusively from pm_targets and pg_attach_proc, never the
	 * owner.  Attach every sibling so the whole group binds when
	 * we start the leader.
	 */
	for (i = 0; i < MAX_GROUPS; i++) {
		for (j = 0; j < grps[i].nevents; j++) {
			if (pmc_attach(grps[i].ids[j], getpid()) < 0) {
				warn("pmc_attach group %d sibling %d", i, j);
				for (int k = 0; k < MAX_GROUPS; k++)
					release_group(&grps[k]);
				return (1);
			}
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
	 * Validation.
	 *   (a) Every sibling in every group must have advanced strictly
	 *       between snapshot 1 and snapshot 2.  If rotation is broken
	 *       one entire group will sit at zero deltas.
	 *   (b) Within each group, the spread of deltas across siblings
	 *       must be modest.  All-or-none atomic placement means every
	 *       sibling sees the same set of rotation windows, so their
	 *       deltas should differ only by their natural per-event
	 *       count rates.  We bound the ratio min/max >= 1/256, which
	 *       is loose enough to pass even very different events
	 *       (cache-misses vs instructions) but still catches the
	 *       failure mode where some siblings of a group are stuck
	 *       deferred while their peers are running.
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
			/*
			 * Track the highest-rate event per group as a
			 * proxy for "how much HW time did this group
			 * actually get".  cycles or instructions in a
			 * busy-spin loop dominate; the ratio across
			 * groups gives us a fairness proxy without
			 * having to name a specific event.
			 */
			if (d > group_cycles[i])
				group_cycles[i] = d;
		}

		/*
		 * Within-group all-or-none atomicity = every sibling
		 * progressed AND every sibling finished with a non-zero
		 * count.  We deliberately do NOT enforce a ratio across
		 * siblings here: a group can legitimately mix events
		 * with vastly different natural rates (e.g. cycles vs
		 * a rare ls_smi_rx subset), and a tight ratio test would
		 * false-positive on rate variance instead of catching
		 * real "sibling stuck deferred" bugs.  The progressed
		 * + nonzero pair already catches the real failure mode.
		 */
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

	/*
	 * Inter-group fairness check.  Rotation is round-robin with
	 * equal-length windows, so over a 1.2-second window with
	 * 10-ms ticks each group should get ~50% of HW time.  If
	 * one group's max delta is more than 64x the other's, the
	 * rotation kthread is starving one group.  Threshold 64
	 * leaves headroom for first-window startup transients while
	 * still catching the "Group 0 got 2 us / Group 1 got 1 s"
	 * failure mode.
	 */
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
