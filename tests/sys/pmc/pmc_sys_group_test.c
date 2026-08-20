/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Advanced Micro Devices, Inc.
 */

/*
 * Grouped system-mode placement must publish and honor per-CPU row
 * occupancy, so that a physical counter is never live for two owners at
 * once.  The alpha implementation started rows without ever setting
 * phw_pmc and without checking it, so a second owner -- grouped or
 * ungrouped -- was placed straight on top of a live group, with no error
 * and no race required.
 *
 * An owner is per-process, so each test forks: the child allocates its own
 * PMCs and therefore gets its own owner descriptor.  The child reports
 * through its exit status rather than through ATF, whose machinery is not
 * fork-safe.  Row capacity is discovered at runtime by shrinking a group
 * until it commits, since a commit-time fit check is occupancy-blind and
 * so measures pure class capacity.
 */

#include <sys/param.h>
#include <sys/types.h>
#include <sys/sysctl.h>
#include <sys/wait.h>

#include <errno.h>
#include <pmc.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>

#include <atf-c.h>

#define	TEST_EVENT	"instructions"
#define	TEST_CPU	0

/* Child exit codes.  Anything else is a setup failure. */
#define	CHILD_OK		0	/* Second owner handled correctly. */
#define	CHILD_BUG_PLACED	10	/* Given a row already live. */
#define	CHILD_BUG_REJECTED	11	/* Refused a genuinely free row. */
#define	CHILD_ERR_COMMIT	2
#define	CHILD_ERR_START		3

struct sys_group {
	uint32_t	sg_groupid;
	pmc_id_t	sg_ids[PMC_GROUP_MAX_MEMBERS];
	u_int		sg_nmembers;
	bool		sg_started;
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
group_init(struct sys_group *g)
{
	u_int i;

	memset(g, 0, sizeof(*g));
	for (i = 0; i < nitems(g->sg_ids); i++)
		g->sg_ids[i] = PMC_ID_INVALID;
}

/*
 * Releasing a committed leader releases the whole group, so the remaining
 * members are already gone; release them anyway for the uncommitted case
 * and ignore the resulting stale-handle errors.
 */
static void
group_teardown(struct sys_group *g)
{
	u_int i;

	if (g->sg_started)
		(void)pmc_stop(g->sg_ids[0]);
	for (i = 0; i < g->sg_nmembers; i++) {
		if (g->sg_ids[i] != PMC_ID_INVALID)
			(void)pmc_release(g->sg_ids[i]);
	}
	group_init(g);
}

/*
 * Build and commit an nmembers-strong system-mode group bound to cpu, with
 * member 0 as the leader.  Returns 0, or -1 with errno set.
 */
static int
group_build(struct sys_group *g, int cpu, u_int nmembers)
{
	u_int i;

	group_init(g);
	g->sg_nmembers = nmembers;
	for (i = 0; i < nmembers; i++) {
		if (pmc_allocate_group(TEST_EVENT, PMC_MODE_SC, 0, cpu,
		    &g->sg_ids[i], 0) != 0)
			return (-1);
	}
	if (pmc_group_create(&g->sg_groupid) != 0)
		return (-1);
	for (i = 0; i < nmembers; i++) {
		if (pmc_group_add(g->sg_groupid, g->sg_ids[i], i == 0) != 0)
			return (-1);
	}
	return (pmc_group_commit(g->sg_groupid));
}

/*
 * The largest group that commits on cpu, which is the class row capacity:
 * the commit-time fit check ignores occupancy, so this does not depend on
 * what is currently placed.  Zero if system-mode groups are unavailable.
 */
static u_int
probe_group_capacity(int cpu)
{
	struct sys_group probe;
	u_int n;

	for (n = PMC_GROUP_MAX_MEMBERS; n > 0; n--) {
		if (group_build(&probe, cpu, n) == 0) {
			group_teardown(&probe);
			return (n);
		}
		group_teardown(&probe);
	}
	return (0);
}

static void
check_child(int status)
{

	ATF_REQUIRE_MSG(WIFEXITED(status), "second owner died on signal %d",
	    WIFSIGNALED(status) ? WTERMSIG(status) : 0);
	switch (WEXITSTATUS(status)) {
	case CHILD_OK:
		break;
	case CHILD_BUG_PLACED:
		atf_tc_fail("second owner was placed on a row already live "
		    "for the first owner");
	case CHILD_BUG_REJECTED:
		atf_tc_fail("second owner was refused a free row");
	default:
		atf_tc_fail("second owner setup failed, exit %d",
		    WEXITSTATUS(status));
	}
}

/* Claim the same rows the first owner is already running on. */
static int
child_full_collision(u_int cap)
{
	struct sys_group owner_b;
	int rc;

	if (group_build(&owner_b, TEST_CPU, cap) != 0) {
		/* Refusing at commit is already the right answer. */
		rc = errno == ENOSPC ? CHILD_OK : CHILD_ERR_COMMIT;
		group_teardown(&owner_b);
		return (rc);
	}
	errno = 0;
	if (pmc_start(owner_b.sg_ids[0]) == 0) {
		owner_b.sg_started = true;
		rc = CHILD_BUG_PLACED;
	} else
		rc = errno == ENOSPC ? CHILD_OK : CHILD_ERR_START;
	group_teardown(&owner_b);
	return (rc);
}

/* The same claim through the ungrouped allocation path. */
static int
child_ungrouped_alloc(u_int cap __unused)
{
	pmc_id_t id;

	if (pmc_allocate(TEST_EVENT, PMC_MODE_SC, 0, TEST_CPU, &id, 0) != 0)
		return (CHILD_OK);
	(void)pmc_release(id);
	return (CHILD_BUG_PLACED);
}

/* Claim the one row the first owner deliberately left free. */
static int
child_single_member(u_int cap __unused)
{
	struct sys_group owner_b;
	int rc;

	if (group_build(&owner_b, TEST_CPU, 1) != 0)
		rc = CHILD_BUG_REJECTED;
	else if (pmc_start(owner_b.sg_ids[0]) != 0)
		rc = CHILD_BUG_REJECTED;
	else {
		owner_b.sg_started = true;
		rc = CHILD_OK;
	}
	group_teardown(&owner_b);
	return (rc);
}

/*
 * Run fn as a second owner while the first owner holds nrows on TEST_CPU,
 * and check how the kernel answered it.
 */
static void
run_second_owner(u_int nrows, int (*fn)(u_int), u_int arg)
{
	struct sys_group owner_a;
	pid_t pid;
	int status;

	ATF_REQUIRE_MSG(group_build(&owner_a, TEST_CPU, nrows) == 0,
	    "first owner commit failed: %s", strerror(errno));
	ATF_REQUIRE_MSG(pmc_start(owner_a.sg_ids[0]) == 0,
	    "first owner start failed: %s", strerror(errno));
	owner_a.sg_started = true;

	pid = fork();
	ATF_REQUIRE(pid != -1);
	if (pid == 0)
		_exit(fn(arg));
	ATF_REQUIRE(waitpid(pid, &status, 0) == pid);

	group_teardown(&owner_a);
	check_child(status);
}

ATF_TC_WITHOUT_HEAD(sys_two_owners_full_collision);
ATF_TC_BODY(sys_two_owners_full_collision, tc)
{
	u_int cap;

	require_hwpmc();
	cap = probe_group_capacity(TEST_CPU);
	if (cap == 0)
		atf_tc_skip("no system-mode group capacity on CPU %d",
		    TEST_CPU);
	run_second_owner(cap, child_full_collision, cap);
}

ATF_TC_WITHOUT_HEAD(sys_grouped_vs_ungrouped_alloc);
ATF_TC_BODY(sys_grouped_vs_ungrouped_alloc, tc)
{
	u_int cap;

	require_hwpmc();
	cap = probe_group_capacity(TEST_CPU);
	if (cap == 0)
		atf_tc_skip("no system-mode group capacity on CPU %d",
		    TEST_CPU);
	run_second_owner(cap, child_ungrouped_alloc, 0);
}

ATF_TC_WITHOUT_HEAD(sys_two_owners_disjoint_rows);
ATF_TC_BODY(sys_two_owners_disjoint_rows, tc)
{
	u_int cap;

	require_hwpmc();
	cap = probe_group_capacity(TEST_CPU);
	if (cap < 2)
		atf_tc_skip("need at least two system-mode group rows on "
		    "CPU %d, have %u", TEST_CPU, cap);
	run_second_owner(cap - 1, child_single_member, 0);
}

ATF_TP_ADD_TCS(tp)
{

	ATF_TP_ADD_TC(tp, sys_two_owners_full_collision);
	ATF_TP_ADD_TC(tp, sys_grouped_vs_ungrouped_alloc);
	ATF_TP_ADD_TC(tp, sys_two_owners_disjoint_rows);

	return (atf_no_error());
}
