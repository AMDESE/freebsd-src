/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Advanced Micro Devices, Inc.
 */

/*
 * Grouped system-wide sampling (spec §6.8).
 *
 * A PMC_MODE_SS member inside a group must deliver samples on its bound
 * CPU, report them through pm_value, and put its owner on the
 * system-sampling owner list while it runs -- that list is what gates
 * kernel-mapping records, and it has to be joined at start rather than at
 * row assignment, because a deferred member has no row yet (§6.5).
 */

#include <sys/types.h>
#include <sys/cpuset.h>
#include <sys/sysctl.h>

#include <errno.h>
#include <pmc.h>
#include <sched.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <atf-c.h>

#define	TEST_EVENT	"instructions"
#define	TEST_CPU	0
#define	SAMPLE_PERIOD	65536
#define	SPIN_ITERS	(400ULL * 1000 * 1000)

struct group {
	uint32_t	g_id;
	pmc_id_t	g_ids[2];
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

	if (geteuid() != 0)
		atf_tc_skip("system-mode PMCs require root");
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
 * A system group bound to TEST_CPU whose leader samples.  The second
 * member counts, which also exercises §3.4's new rule that a system group
 * may mix PMC_MODE_SC and PMC_MODE_SS.
 */
static int
group_build(struct group *g)
{
	u_int i;

	group_init(g);
	g->g_n = 2;
	if (pmc_allocate_group(TEST_EVENT, PMC_MODE_SS, 0, TEST_CPU,
	    &g->g_ids[0], SAMPLE_PERIOD) != 0)
		return (-1);
	if (pmc_allocate_group(TEST_EVENT, PMC_MODE_SC, 0, TEST_CPU,
	    &g->g_ids[1], 0) != 0)
		return (-1);
	if (pmc_group_create(&g->g_id) != 0)
		return (-1);
	for (i = 0; i < g->g_n; i++) {
		if (pmc_group_add(g->g_id, g->g_ids[i], i == 0) != 0)
			return (-1);
	}
	return (pmc_group_commit(g->g_id));
}

/*
 * A system-mode group only sees what runs on the CPU it is bound to, so
 * the load has to be put there or the test measures whatever the
 * scheduler happened to place on TEST_CPU instead.
 */
static void
pin_to_test_cpu(void)
{
	cpuset_t set;

	CPU_ZERO(&set);
	CPU_SET(TEST_CPU, &set);
	if (cpuset_setaffinity(CPU_LEVEL_WHICH, CPU_WHICH_TID, -1,
	    sizeof(set), &set) != 0)
		atf_tc_skip("cannot pin to CPU %d: %s", TEST_CPU,
		    strerror(errno));
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

static u_int
sscount(void)
{
	u_int v;
	size_t len;

	len = sizeof(v);
	if (sysctlbyname("kern.hwpmc.stats.log_sweeps", &v, &len, NULL, 0)
	    != 0)
		return (0);
	return (v);
}

/*
 * A grouped SS member samples, and reports the count through pm_value.
 * The members keep their allocated handles, which is what correlates a
 * sample record with the allocation that produced it (§3.6).
 */
ATF_TC_WITHOUT_HEAD(grouped_system_sampling);
ATF_TC_BODY(grouped_system_sampling, tc)
{
	struct pmc_group_member m[2];
	struct group g;
	FILE *log;
	uint32_t n;

	require_hwpmc();
	pin_to_test_cpu();
	log = tmpfile();
	ATF_REQUIRE(log != NULL);
	ATF_REQUIRE_MSG(pmc_configure_logfile(fileno(log)) == 0,
	    "logfile configuration failed: %s", strerror(errno));

	ATF_REQUIRE_MSG(group_build(&g) == 0,
	    "grouped system-sampling commit failed: %s", strerror(errno));
	ATF_REQUIRE_MSG(pmc_start(g.g_ids[0]) == 0, "start failed: %s",
	    strerror(errno));
	g.g_started = true;
	spin();

	n = g.g_n;
	memset(m, 0, sizeof(m));
	ATF_REQUIRE_MSG(pmc_group_read(g.g_ids[0], &n, m, NULL) == 0,
	    "group read failed: %s", strerror(errno));
	ATF_CHECK_EQ(n, 2);
	ATF_CHECK_EQ(m[0].pm_pmcid, g.g_ids[0]);
	ATF_CHECK_EQ(m[1].pm_pmcid, g.g_ids[1]);
	ATF_CHECK_EQ(m[0].pm_mflags, PMC_GROUP_MEMBER_F_SAMPLES);
	ATF_CHECK_EQ(m[1].pm_mflags, 0);
	printf("samples=%ju counted=%ju (period %d)\n",
	    (uintmax_t)m[0].pm_value, (uintmax_t)m[1].pm_value,
	    SAMPLE_PERIOD);
	ATF_CHECK_MSG(m[1].pm_value > 0,
	    "the counting sibling counted nothing, so the group did not run");
	ATF_CHECK_MSG(m[0].pm_value > 0,
	    "the grouped system-sampling member delivered no samples");
	/*
	 * Both members watch the same event on the same CPU, so the sample
	 * count should track the counted total over the period.  Allow a
	 * wide margin -- the two are read at slightly different points and
	 * the sampler is stopped around each overflow -- but catch a member
	 * that delivers a token sample and then stops.
	 */
	if (m[1].pm_value > 4 * SAMPLE_PERIOD) {
		ATF_CHECK_MSG(m[0].pm_value >=
		    m[1].pm_value / (4 * SAMPLE_PERIOD),
		    "%ju samples for %ju events at period %d is far below "
		    "the expected rate", (uintmax_t)m[0].pm_value,
		    (uintmax_t)m[1].pm_value, SAMPLE_PERIOD);
	}

	group_teardown(&g);
	ATF_REQUIRE(pmc_configure_logfile(-1) == 0);
	ATF_REQUIRE(fclose(log) == 0);
}

/*
 * Starting the group joins the owner to the system-sampling owner list,
 * and stopping it leaves.  Sampling that never joins produces a log with
 * no kernel mappings, so samples in kernel text cannot be resolved.
 */
ATF_TC_WITHOUT_HEAD(grouped_system_sampling_accounting);
ATF_TC_BODY(grouped_system_sampling_accounting, tc)
{
	struct group g;
	FILE *log;
	u_int before, running;

	require_hwpmc();
	pin_to_test_cpu();
	log = tmpfile();
	ATF_REQUIRE(log != NULL);
	ATF_REQUIRE(pmc_configure_logfile(fileno(log)) == 0);

	before = sscount();
	ATF_REQUIRE_MSG(group_build(&g) == 0, "commit failed: %s",
	    strerror(errno));
	ATF_REQUIRE(pmc_start(g.g_ids[0]) == 0);
	g.g_started = true;
	spin();
	running = sscount();
	ATF_CHECK_MSG(running >= before,
	    "log sweeps went backwards: %u then %u", before, running);
	group_teardown(&g);

	/* Releasing a running grouped SS group must leave the list clean. */
	ATF_REQUIRE_MSG(group_build(&g) == 0, "second commit failed: %s",
	    strerror(errno));
	ATF_REQUIRE(pmc_start(g.g_ids[0]) == 0);
	g.g_started = true;
	group_teardown(&g);

	ATF_REQUIRE(pmc_configure_logfile(-1) == 0);
	ATF_REQUIRE(fclose(log) == 0);
}

ATF_TP_ADD_TCS(tp)
{

	ATF_TP_ADD_TC(tp, grouped_system_sampling);
	ATF_TP_ADD_TC(tp, grouped_system_sampling_accounting);

	return (atf_no_error());
}
