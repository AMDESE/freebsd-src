/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Advanced Micro Devices, Inc.
 */

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

static void __attribute__((noinline))
spin(void)
{
	volatile uint64_t sink;
	uint64_t i;

	sink = 0;
	for (i = 0; i < 10 * 1000 * 1000; i++)
		sink += i;
}

static pmc_id_t
allocate_grouped(void)
{
	pmc_id_t id;

	ATF_REQUIRE_MSG(pmc_allocate_group("instructions", PMC_MODE_TC, 0,
	    PMC_CPU_ANY, &id, 0) == 0, "pmc_allocate_group failed: %s",
	    strerror(errno));
	return (id);
}

ATF_TC_WITHOUT_HEAD(committed_release);
ATF_TC_BODY(committed_release, tc)
{
	pmc_id_t fresh, leader, sibling;
	pmc_value_t value;
	uint32_t groupid;

	require_hwpmc();
	leader = allocate_grouped();
	sibling = allocate_grouped();
	ATF_REQUIRE_MSG(pmc_group_create(&groupid) == 0,
	    "pmc_group_create failed: %s", strerror(errno));
	ATF_REQUIRE_MSG(pmc_group_add(groupid, leader, 1) == 0,
	    "leader add failed: %s", strerror(errno));
	ATF_REQUIRE_MSG(pmc_group_add(groupid, sibling, 0) == 0,
	    "sibling add failed: %s", strerror(errno));
	ATF_REQUIRE_MSG(pmc_group_commit(groupid) == 0,
	    "pmc_group_commit failed: %s", strerror(errno));

	errno = 0;
	ATF_CHECK_EQ(pmc_release(sibling), -1);
	ATF_CHECK_EQ(errno, EBUSY);
	errno = 0;
	ATF_CHECK_EQ(pmc_detach(leader, getpid()), -1);
	ATF_CHECK_EQ(errno, EBUSY);
	errno = 0;
	ATF_CHECK_EQ(pmc_detach(sibling, getpid()), -1);
	ATF_CHECK_EQ(errno, EBUSY);
	ATF_REQUIRE_MSG(pmc_attach(leader, getpid()) == 0,
	    "leader attach failed: %s", strerror(errno));
	ATF_REQUIRE_MSG(pmc_start(leader) == 0,
	    "leader start failed: %s", strerror(errno));
	spin();

	ATF_REQUIRE_MSG(pmc_release(leader) == 0,
	    "leader release failed: %s", strerror(errno));
	errno = 0;
	ATF_CHECK_EQ(pmc_release(sibling), -1);
	ATF_CHECK_EQ(errno, EINVAL);
	errno = 0;
	ATF_CHECK_EQ(pmc_read(sibling, &value), -1);
	ATF_CHECK_EQ(errno, EINVAL);

	fresh = allocate_grouped();
	ATF_CHECK(fresh != sibling);
	errno = 0;
	ATF_CHECK_EQ(pmc_group_add(groupid, fresh, 1), -1);
	ATF_CHECK_EQ(errno, EINVAL);
	ATF_REQUIRE(pmc_release(fresh) == 0);
}

ATF_TC_WITHOUT_HEAD(uncommitted_release);
ATF_TC_BODY(uncommitted_release, tc)
{
	pmc_id_t leader, new_leader, probe, sibling;
	uint32_t groupid, released_groupid;

	require_hwpmc();
	leader = allocate_grouped();
	sibling = allocate_grouped();
	ATF_REQUIRE(pmc_group_create(&groupid) == 0);
	ATF_REQUIRE(pmc_group_add(groupid, leader, 1) == 0);
	ATF_REQUIRE(pmc_group_add(groupid, sibling, 0) == 0);

	ATF_REQUIRE_MSG(pmc_release(leader) == 0,
	    "uncommitted leader release failed: %s", strerror(errno));
	errno = 0;
	ATF_CHECK_EQ(pmc_group_commit(groupid), -1);
	ATF_CHECK_EQ(errno, EINVAL);

	new_leader = allocate_grouped();
	ATF_REQUIRE_MSG(pmc_group_add(groupid, new_leader, 1) == 0,
	    "replacement leader add failed: %s", strerror(errno));
	ATF_REQUIRE_MSG(pmc_group_commit(groupid) == 0,
	    "replacement leader commit failed: %s", strerror(errno));
	ATF_REQUIRE_MSG(pmc_release(new_leader) == 0,
	    "replacement leader release failed: %s", strerror(errno));

	probe = allocate_grouped();
	ATF_REQUIRE(pmc_group_create(&released_groupid) == 0);
	ATF_REQUIRE(pmc_group_add(released_groupid, probe, 0) == 0);
	ATF_REQUIRE_MSG(pmc_release(probe) == 0,
	    "last uncommitted member release failed: %s", strerror(errno));
	probe = allocate_grouped();
	errno = 0;
	ATF_CHECK_EQ(pmc_group_add(released_groupid, probe, 1), -1);
	ATF_CHECK_EQ(errno, EINVAL);
	ATF_REQUIRE(pmc_release(probe) == 0);
}

ATF_TC_WITHOUT_HEAD(owner_exit);
ATF_TC_BODY(owner_exit, tc)
{
	pmc_id_t id;
	uint32_t groupid;
	pid_t pid;
	int status;

	require_hwpmc();
	pid = fork();
	ATF_REQUIRE(pid >= 0);
	if (pid == 0) {
		if (pmc_allocate_group("instructions", PMC_MODE_TC, 0,
		    PMC_CPU_ANY, &id, 0) != 0 ||
		    pmc_group_create(&groupid) != 0 ||
		    pmc_group_add(groupid, id, 1) != 0 ||
		    pmc_group_commit(groupid) != 0 || pmc_attach(id, 0) != 0 ||
		    pmc_start(id) != 0)
			_exit(1);
		spin();
		_exit(0);
	}
	ATF_REQUIRE(waitpid(pid, &status, 0) == pid);
	ATF_REQUIRE(WIFEXITED(status));
	ATF_REQUIRE_EQ(WEXITSTATUS(status), 0);

	id = allocate_grouped();
	ATF_REQUIRE(pmc_group_create(&groupid) == 0);
	ATF_REQUIRE(pmc_group_add(groupid, id, 1) == 0);
	ATF_REQUIRE_MSG(pmc_group_commit(groupid) == 0,
	    "post-exit group commit failed: %s", strerror(errno));
	ATF_REQUIRE(pmc_release(id) == 0);
}

ATF_TC_WITHOUT_HEAD(uncommitted_nonleader_release);
ATF_TC_BODY(uncommitted_nonleader_release, tc)
{
	pmc_id_t leader, sibling;
	uint32_t groupid;

	require_hwpmc();
	leader = allocate_grouped();
	sibling = allocate_grouped();
	ATF_REQUIRE(pmc_group_create(&groupid) == 0);
	ATF_REQUIRE(pmc_group_add(groupid, leader, 1) == 0);
	ATF_REQUIRE(pmc_group_add(groupid, sibling, 0) == 0);
	ATF_REQUIRE_MSG(pmc_release(sibling) == 0,
	    "uncommitted sibling release failed: %s", strerror(errno));
	ATF_REQUIRE_MSG(pmc_group_commit(groupid) == 0,
	    "one-member commit failed: %s", strerror(errno));
	ATF_REQUIRE(pmc_release(leader) == 0);
}

ATF_TP_ADD_TCS(tp)
{

	ATF_TP_ADD_TC(tp, committed_release);
	ATF_TP_ADD_TC(tp, owner_exit);
	ATF_TP_ADD_TC(tp, uncommitted_release);
	ATF_TP_ADD_TC(tp, uncommitted_nonleader_release);
	return (atf_no_error());
}
