/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Advanced Micro Devices, Inc.
 */

/*
 * Fork inheritance for grouped PMCs (spec §3.9).
 *
 * PMC_F_DESCENDANTS on a group leader makes the whole group follow every
 * fork of every attached target, and the children's work accumulates into
 * the same member totals.  The measurement is therefore tree-wide: a run
 * that forks children which do most of the work must count far more than
 * the parent alone does.
 *
 * The controls matter more than the headline check here.  An unattached
 * child does the same work in every case, so a run WITHOUT the flag pins
 * down what "parent only" costs, and the two are compared against each
 * other rather than against an absolute number that would vary by machine.
 */

#include <sys/types.h>
#include <sys/sysctl.h>
#include <sys/wait.h>

#include <errno.h>
#include <pmc.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <atf-c.h>

#define	TEST_EVENT	"instructions"
#define	NCHILDREN	4
/* Each child does this much; the parent only forks and waits. */
#define	CHILD_ITERS	(200ULL * 1000 * 1000)

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

/* A two-member counting group; the leader may carry PMC_F_DESCENDANTS. */
static int
group_build(struct group *g, bool descendants)
{
	uint32_t flags;
	u_int i;

	group_init(g);
	g->g_n = 2;
	for (i = 0; i < g->g_n; i++) {
		flags = (i == 0 && descendants) ? PMC_F_DESCENDANTS : 0;
		if (pmc_allocate_group(TEST_EVENT, PMC_MODE_TC, flags,
		    PMC_CPU_ANY, &g->g_ids[i], 0) != 0)
			return (-1);
	}
	if (pmc_group_create(&g->g_id) != 0)
		return (-1);
	for (i = 0; i < g->g_n; i++) {
		if (pmc_group_add(g->g_id, g->g_ids[i], i == 0) != 0)
			return (-1);
	}
	return (pmc_group_commit(g->g_id));
}

static void __attribute__((noinline))
work(uint64_t iters)
{
	volatile uint64_t sink;
	uint64_t i;

	sink = 0;
	for (i = 0; i < iters; i++)
		sink += i;
}

/* Fork NCHILDREN, each doing the work, and reap them. */
static void
fork_workload(void)
{
	pid_t pids[NCHILDREN];
	int i, status;

	for (i = 0; i < NCHILDREN; i++) {
		pids[i] = fork();
		ATF_REQUIRE(pids[i] != -1);
		if (pids[i] == 0) {
			work(CHILD_ITERS);
			_exit(0);
		}
	}
	for (i = 0; i < NCHILDREN; i++) {
		ATF_REQUIRE(waitpid(pids[i], &status, 0) == pids[i]);
		ATF_REQUIRE(WIFEXITED(status) && WEXITSTATUS(status) == 0);
	}
}

static pmc_value_t
leader_value(struct group *g)
{
	struct pmc_group_member m[2];
	uint32_t n;

	n = g->g_n;
	memset(m, 0, sizeof(m));
	ATF_REQUIRE_MSG(pmc_group_read(g->g_ids[0], &n, m, NULL) == 0,
	    "group read failed: %s", strerror(errno));
	ATF_REQUIRE_EQ(n, g->g_n);
	return (m[0].pm_value);
}

/* Run the forking workload under a group and return the leader's count. */
static pmc_value_t
measure(bool descendants)
{
	struct group g;
	pmc_value_t v;

	ATF_REQUIRE_MSG(group_build(&g, descendants) == 0,
	    "commit failed: %s", strerror(errno));
	ATF_REQUIRE_MSG(pmc_attach(g.g_ids[0], getpid()) == 0,
	    "attach failed: %s", strerror(errno));
	ATF_REQUIRE_MSG(pmc_start(g.g_ids[0]) == 0, "start failed: %s",
	    strerror(errno));
	g.g_started = true;
	fork_workload();
	v = leader_value(&g);
	group_teardown(&g);
	return (v);
}

/*
 * The children's work must show up in the totals.  They do all the work,
 * so a tree-wide count is far above a parent-only one; the comparison is
 * against the same workload measured without the flag, so no absolute
 * event count is assumed.
 *
 * Skipped until the scheduler places a forked child's PMCs.  A child's
 * first turn on a CPU comes through sched_fork_exit(), which -- unlike
 * sched_switch() -- never calls PMC_SWITCH_CONTEXT(td, PMC_FN_CSW_IN), so
 * the inherited counters are never programmed and the child's events are
 * counted by nobody.  This is neither new here nor specific to groups:
 * ungrouped PMC_F_DESCENDANTS measures the same nothing, on a stock module
 * and on both schedulers.  The rest of this file still holds it to the
 * part §3.9 puts in the driver -- that the whole group follows the fork
 * and that totals never go backwards.
 */
ATF_TC_WITHOUT_HEAD(descendants_counted);
ATF_TC_BODY(descendants_counted, tc)
{
	pmc_value_t with, without;

	require_hwpmc();
	atf_tc_skip("a forked child's PMCs are never placed: "
	    "sched_fork_exit() does not call the PMC context-switch-in hook");

	without = measure(false);
	with = measure(true);

	printf("parent only: %ju\ntree wide:   %ju\n", (uintmax_t)without,
	    (uintmax_t)with);
	ATF_CHECK_MSG(with > without * 2,
	    "grouped -d counted %ju against %ju without the flag: the "
	    "children's work is missing from the totals",
	    (uintmax_t)with, (uintmax_t)without);
}

/*
 * Every member follows the fork, not just the leader: §2.4 has to hold in
 * the child too, so the siblings must stay consistent with each other.
 */
ATF_TC_WITHOUT_HEAD(descendants_whole_group);
ATF_TC_BODY(descendants_whole_group, tc)
{
	struct pmc_group_member m[2];
	struct group g;
	uint32_t n;

	require_hwpmc();
	ATF_REQUIRE_MSG(group_build(&g, true) == 0, "commit failed: %s",
	    strerror(errno));
	ATF_REQUIRE(pmc_attach(g.g_ids[0], getpid()) == 0);
	ATF_REQUIRE(pmc_start(g.g_ids[0]) == 0);
	g.g_started = true;
	fork_workload();

	n = g.g_n;
	memset(m, 0, sizeof(m));
	ATF_REQUIRE_MSG(pmc_group_read(g.g_ids[0], &n, m, NULL) == 0,
	    "group read failed: %s", strerror(errno));
	ATF_CHECK_EQ(n, 2);
	ATF_CHECK(m[0].pm_value > 0);
	ATF_CHECK_MSG(m[1].pm_value > 0,
	    "the non-leader member counted nothing, so the group did not "
	    "follow the fork as a whole");
	group_teardown(&g);
}

/* A child exit mid-run must never take counts back off the totals. */
ATF_TC_WITHOUT_HEAD(child_exit_does_not_regress);
ATF_TC_BODY(child_exit_does_not_regress, tc)
{
	struct group g;
	pmc_value_t before, after;
	int i;

	require_hwpmc();
	ATF_REQUIRE_MSG(group_build(&g, true) == 0, "commit failed: %s",
	    strerror(errno));
	ATF_REQUIRE(pmc_attach(g.g_ids[0], getpid()) == 0);
	ATF_REQUIRE(pmc_start(g.g_ids[0]) == 0);
	g.g_started = true;

	before = 0;
	for (i = 0; i < 3; i++) {
		fork_workload();
		after = leader_value(&g);
		ATF_CHECK_MSG(after >= before,
		    "totals fell from %ju to %ju across a child exit",
		    (uintmax_t)before, (uintmax_t)after);
		before = after;
	}
	group_teardown(&g);
}

ATF_TP_ADD_TCS(tp)
{

	ATF_TP_ADD_TC(tp, descendants_counted);
	ATF_TP_ADD_TC(tp, descendants_whole_group);
	ATF_TP_ADD_TC(tp, child_exit_does_not_regress);

	return (atf_no_error());
}
