/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Advanced Micro Devices, Inc.
 */

/*
 * Sampling residuals must survive eviction.
 *
 * A multiplexed group is on hardware for only part of each rotation.  If
 * progress toward the next sample were discarded at eviction, a member
 * whose period is longer than one residency window would restart its climb
 * every window and never reach an overflow -- zero samples, indefinitely,
 * however long the run.  With the residual preserved, progress accumulates
 * across windows and delivery converges on residency times the natural
 * rate.
 *
 * The test picks a period deliberately larger than what one window can
 * retire, oversubscribes the PMU so rotation is forced, and requires
 * samples to arrive anyway.  A control group with the same period and no
 * multiplexing shows the period itself is reachable.
 */

#include <sys/types.h>
#include <sys/sysctl.h>

#include <errno.h>
#include <pmc.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <atf-c.h>

#define	TEST_EVENT	"instructions"

/*
 * The period must exceed what one residency window can retire, or progress
 * never has to carry across a window and the test proves nothing.  A 50 ms
 * window retires a few hundred million events on this class of hardware,
 * so 1e9 spans roughly two windows.  The workload is sized to deliver
 * enough samples for the counts to mean something.
 */
#define	LONG_PERIOD	(1000ULL * 1000 * 1000)
#define	SPIN_ITERS	(4000ULL * 1000 * 1000)

struct group {
	uint32_t	g_id;
	pmc_id_t	g_ids[PMC_GROUP_MAX_MEMBERS];
	u_int		g_n;
	bool		g_started;
};

static bool
is_amd(void)
{
	char vendor[64];
	size_t len;

	len = sizeof(vendor);
	if (sysctlbyname("kern.hwpmc.cpuid", vendor, &len, NULL, 0) != 0)
		return (false);
	return (strstr(vendor, "AuthenticAMD") != NULL ||
	    strstr(vendor, "HygonGenuine") != NULL);
}

static void
require_hwpmc(void)
{

	if (pmc_init() != 0)
		atf_tc_skip("hwpmc(4) is not available: %s", strerror(errno));
	if (!is_amd())
		atf_tc_skip("PMC grouping is supported only on AMD CPUs");
}

static void
group_init(struct group *g)
{
	u_int i;

	memset(g, 0, sizeof(*g));
	for (i = 0; i < nitems(g->g_ids); i++)
		g->g_ids[i] = PMC_ID_INVALID;
}

static void
group_teardown(struct group *g)
{
	u_int i;

	if (g->g_started)
		(void)pmc_stop(g->g_ids[0]);
	for (i = 0; i < g->g_n; i++) {
		if (g->g_ids[i] != PMC_ID_INVALID)
			(void)pmc_release(g->g_ids[i]);
	}
	group_init(g);
}

/*
 * Build and commit an n-member sampling group whose leader is member 0.
 * With mux set, the group may be time-multiplexed against others.
 */
static int
group_build(struct group *g, u_int n, bool mux)
{
	uint32_t flags;
	u_int i;

	group_init(g);
	g->g_n = n;
	for (i = 0; i < n; i++) {
		flags = (i == 0 && mux) ? PMC_F_GROUP_MUX : 0;
		if (pmc_allocate_group(TEST_EVENT, PMC_MODE_TS, flags,
		    PMC_CPU_ANY, &g->g_ids[i], LONG_PERIOD) != 0)
			return (-1);
	}
	if (pmc_group_create(&g->g_id) != 0)
		return (-1);
	for (i = 0; i < n; i++) {
		if (pmc_group_add(g->g_id, g->g_ids[i], i == 0) != 0)
			return (-1);
	}
	return (pmc_group_commit(g->g_id));
}

/* Largest group that commits, which is the usable row capacity. */
static u_int
probe_capacity(void)
{
	struct group probe;
	u_int n;

	for (n = PMC_GROUP_MAX_MEMBERS; n > 0; n--) {
		if (group_build(&probe, n, false) == 0) {
			group_teardown(&probe);
			return (n);
		}
		group_teardown(&probe);
	}
	return (0);
}

static void __attribute__((noinline))
spin(void)
{
	volatile uint64_t sink;
	uint64_t i;

	sink = 0;
	for (i = 0; i < SPIN_ITERS; i++)
		sink += i;
}

/*
 * Delivered sample count of the leader, which pm_value reports, and the
 * group's residency.  The buffer must hold every member: a short one fails
 * E2BIG and reads nothing.
 */
static pmc_value_t
leader_samples(struct group *g, double *residency)
{
	struct pmc_group_member m[PMC_GROUP_MAX_MEMBERS];
	struct pmc_group_times t;
	uint32_t n;

	n = g->g_n;
	memset(m, 0, sizeof(m));
	memset(&t, 0, sizeof(t));
	ATF_REQUIRE_MSG(pmc_group_read(g->g_ids[0], &n, m, &t) == 0,
	    "group read failed: %s", strerror(errno));
	ATF_REQUIRE_EQ(n, g->g_n);
	if (residency != NULL) {
		*residency = t.pgt_enabled != 0 ?
		    (double)t.pgt_running / t.pgt_enabled : 0.0;
	}
	return (m[0].pm_value);
}

static void
run_group(struct group *g)
{

	ATF_REQUIRE_MSG(pmc_attach(g->g_ids[0], getpid()) == 0,
	    "attach failed: %s", strerror(errno));
	ATF_REQUIRE_MSG(pmc_start(g->g_ids[0]) == 0, "start failed: %s",
	    strerror(errno));
	g->g_started = true;
}

/*
 * Control: a period this long is reachable when the group is never evicted.
 * If this fails the multiplexed case below proves nothing.
 */
ATF_TC_WITHOUT_HEAD(long_period_unmultiplexed);
ATF_TC_BODY(long_period_unmultiplexed, tc)
{
	struct group g;
	FILE *log;
	pmc_value_t samples;

	require_hwpmc();
	log = tmpfile();
	ATF_REQUIRE(log != NULL);
	ATF_REQUIRE(pmc_configure_logfile(fileno(log)) == 0);

	ATF_REQUIRE_MSG(group_build(&g, 1, false) == 0, "commit failed: %s",
	    strerror(errno));
	run_group(&g);
	spin();
	samples = leader_samples(&g, NULL);
	group_teardown(&g);
	(void)pmc_configure_logfile(-1);
	(void)fclose(log);

	printf("unmultiplexed: %ju samples\n", (uintmax_t)samples);
	ATF_CHECK_MSG(samples > 0,
	    "no samples without multiplexing: the period is unreachable in "
	    "this workload, so the multiplexed case cannot be judged");
}

/*
 * The real check.  Two oversubscribed groups force rotation, so neither is
 * resident continuously.  Without residual preservation the leader climbs
 * from zero every window and never overflows.
 */
ATF_TC_WITHOUT_HEAD(long_period_under_mux);
ATF_TC_BODY(long_period_under_mux, tc)
{
	struct group a, b;
	FILE *log;
	pmc_value_t samples;
	double residency;
	u_int cap;

	require_hwpmc();
	cap = probe_capacity();
	if (cap < 2)
		atf_tc_skip("need at least two group rows, have %u", cap);

	log = tmpfile();
	ATF_REQUIRE(log != NULL);
	ATF_REQUIRE(pmc_configure_logfile(fileno(log)) == 0);

	group_init(&b);
	ATF_REQUIRE_MSG(group_build(&a, cap, true) == 0,
	    "first group commit failed: %s", strerror(errno));
	if (group_build(&b, cap, true) != 0) {
		group_teardown(&b);
		group_teardown(&a);
		(void)pmc_configure_logfile(-1);
		(void)fclose(log);
		atf_tc_skip("could not oversubscribe the PMU: %s",
		    strerror(errno));
	}
	run_group(&a);
	run_group(&b);
	spin();
	samples = leader_samples(&a, &residency);
	group_teardown(&b);
	group_teardown(&a);
	(void)pmc_configure_logfile(-1);
	(void)fclose(log);

	printf("multiplexed: %ju samples at %.1f%% residency\n",
	    (uintmax_t)samples, residency * 100.0);
	ATF_CHECK_MSG(residency < 0.95,
	    "the group was resident %.1f%% of the time, so it was never "
	    "really multiplexed and the check below proves nothing",
	    residency * 100.0);
	ATF_CHECK_MSG(samples > 0,
	    "a multiplexed member whose period exceeds one residency window "
	    "delivered no samples: the residual was discarded at eviction");
}

ATF_TP_ADD_TCS(tp)
{

	ATF_TP_ADD_TC(tp, long_period_unmultiplexed);
	ATF_TP_ADD_TC(tp, long_period_under_mux);

	return (atf_no_error());
}
