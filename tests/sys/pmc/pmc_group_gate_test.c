/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Advanced Micro Devices, Inc.
 */

/*
 * The v1.4 input gates.  Allocation accepts the inputs the revision makes
 * legal -- PMC_F_DESCENDANTS and system-wide sampling on a deferred PMC --
 * and commit rejects the two ways of misusing PMC_F_DESCENDANTS.
 *
 * The split matters for feature probing: allocation-time EOPNOTSUPP is the
 * canonical "this module does not have the feature" signal, so misuse of a
 * feature that IS present must come back as EINVAL from commit instead.  A
 * tool that saw EOPNOTSUPP here would report a working module as one
 * without grouping.
 */

#include <sys/types.h>
#include <sys/sysctl.h>

#include <errno.h>
#include <pmc.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>

#include <atf-c.h>

#define	TEST_EVENT	"instructions"
#define	TEST_CPU	0
#define	SAMPLE_COUNT	65536

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
require_root(void)
{

	if (geteuid() != 0)
		atf_tc_skip("system-mode PMCs require root");
}

/*
 * Commit a two-member group whose leader and non-leader carry the given
 * flags, and return the commit result (0 or -1 with errno set).  Everything
 * is released before returning.
 */
static int
commit_with_flags(enum pmc_mode mode, int cpu, uint32_t leader_flags,
    uint32_t member_flags, uint64_t count)
{
	uint32_t gid;
	pmc_id_t leader, member;
	int error, rv;

	leader = member = PMC_ID_INVALID;
	ATF_REQUIRE_MSG(pmc_allocate_group(TEST_EVENT, mode, leader_flags, cpu,
	    &leader, count) == 0,
	    "leader pmc_allocate_group failed: errno %d (%s)", errno,
	    strerror(errno));
	ATF_REQUIRE_MSG(pmc_allocate_group(TEST_EVENT, mode, member_flags, cpu,
	    &member, count) == 0,
	    "member pmc_allocate_group failed: errno %d (%s)", errno,
	    strerror(errno));
	ATF_REQUIRE_MSG(pmc_group_create(&gid) == 0,
	    "pmc_group_create failed: errno %d (%s)", errno, strerror(errno));
	ATF_REQUIRE_MSG(pmc_group_add(gid, leader, 1) == 0,
	    "leader pmc_group_add failed: errno %d (%s)", errno,
	    strerror(errno));
	ATF_REQUIRE_MSG(pmc_group_add(gid, member, 0) == 0,
	    "member pmc_group_add failed: errno %d (%s)", errno,
	    strerror(errno));
	rv = pmc_group_commit(gid);

	error = errno;
	if (leader != PMC_ID_INVALID)
		(void)pmc_release(leader);
	if (member != PMC_ID_INVALID)
		(void)pmc_release(member);
	errno = error;
	return (rv);
}

/* PMC_F_DESCENDANTS is accepted at allocate; the leader designation is later. */
ATF_TC_WITHOUT_HEAD(alloc_descendants_accepted);
ATF_TC_BODY(alloc_descendants_accepted, tc)
{
	pmc_id_t id;

	require_hwpmc();
	ATF_REQUIRE_MSG(pmc_allocate_group(TEST_EVENT, PMC_MODE_TC,
	    PMC_F_DESCENDANTS, PMC_CPU_ANY, &id, 0) == 0,
	    "deferred alloc with PMC_F_DESCENDANTS failed: %s",
	    strerror(errno));
	ATF_CHECK_EQ(pmc_release(id), 0);
}

/* A deferred system-sampling allocation is accepted. */
ATF_TC_WITHOUT_HEAD(alloc_system_sampling_accepted);
ATF_TC_BODY(alloc_system_sampling_accepted, tc)
{
	pmc_id_t id;

	require_hwpmc();
	require_root();
	ATF_REQUIRE_MSG(pmc_allocate_group(TEST_EVENT, PMC_MODE_SS, 0,
	    TEST_CPU, &id, SAMPLE_COUNT) == 0,
	    "deferred PMC_MODE_SS alloc failed: %s", strerror(errno));
	ATF_CHECK_EQ(pmc_release(id), 0);
}

/* A grouped sampling allocation may carry a nonzero count; counting may not. */
ATF_TC_WITHOUT_HEAD(alloc_count_gate_is_counting_only);
ATF_TC_BODY(alloc_count_gate_is_counting_only, tc)
{
	pmc_id_t id;

	require_hwpmc();
	ATF_REQUIRE_MSG(pmc_allocate_group(TEST_EVENT, PMC_MODE_TS, 0,
	    PMC_CPU_ANY, &id, SAMPLE_COUNT) == 0,
	    "grouped sampling alloc with nonzero count failed: %s",
	    strerror(errno));
	ATF_CHECK_EQ(pmc_release(id), 0);

	errno = 0;
	ATF_CHECK_EQ(pmc_allocate_group(TEST_EVENT, PMC_MODE_TC, 0,
	    PMC_CPU_ANY, &id, SAMPLE_COUNT), -1);
	ATF_CHECK_EQ(errno, EINVAL);
}

/* The flag is leader-only: on a non-leader, commit fails EINVAL. */
ATF_TC_WITHOUT_HEAD(commit_descendants_on_nonleader);
ATF_TC_BODY(commit_descendants_on_nonleader, tc)
{

	require_hwpmc();
	errno = 0;
	ATF_CHECK_EQ(commit_with_flags(PMC_MODE_TC, PMC_CPU_ANY, 0,
	    PMC_F_DESCENDANTS, 0), -1);
	ATF_CHECK_EQ(errno, EINVAL);
}

/* A CPU-bound group cannot follow a fork: commit fails EINVAL. */
ATF_TC_WITHOUT_HEAD(commit_descendants_on_system_leader);
ATF_TC_BODY(commit_descendants_on_system_leader, tc)
{

	require_hwpmc();
	require_root();
	errno = 0;
	ATF_CHECK_EQ(commit_with_flags(PMC_MODE_SC, TEST_CPU,
	    PMC_F_DESCENDANTS, 0, 0), -1);
	ATF_CHECK_EQ(errno, EINVAL);
}

/* The same group without the flag commits, so EINVAL above is the flag. */
ATF_TC_WITHOUT_HEAD(commit_descendants_on_virtual_leader);
ATF_TC_BODY(commit_descendants_on_virtual_leader, tc)
{

	require_hwpmc();
	ATF_CHECK_MSG(commit_with_flags(PMC_MODE_TC, PMC_CPU_ANY,
	    PMC_F_DESCENDANTS, 0, 0) == 0,
	    "virtual group with PMC_F_DESCENDANTS on the leader failed to "
	    "commit: %s", strerror(errno));
}

ATF_TP_ADD_TCS(tp)
{

	ATF_TP_ADD_TC(tp, alloc_descendants_accepted);
	ATF_TP_ADD_TC(tp, alloc_system_sampling_accepted);
	ATF_TP_ADD_TC(tp, alloc_count_gate_is_counting_only);
	ATF_TP_ADD_TC(tp, commit_descendants_on_nonleader);
	ATF_TP_ADD_TC(tp, commit_descendants_on_system_leader);
	ATF_TP_ADD_TC(tp, commit_descendants_on_virtual_leader);

	return (atf_no_error());
}
