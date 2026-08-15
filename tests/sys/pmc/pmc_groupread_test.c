/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Advanced Micro Devices, Inc.
 */

#include <sys/param.h>
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

#define	SPIN_ITERS	(50ULL * 1000 * 1000)

struct test_group {
	uint32_t	tg_groupid;
	pmc_id_t	tg_ids[2];
	u_int		tg_nmembers;
	u_int		tg_leader;
	bool		tg_started;
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
cleanup_group(struct test_group *group)
{
	u_int i;

	if (group->tg_started)
		(void)pmc_stop(group->tg_ids[group->tg_leader]);
	(void)pmc_release(group->tg_ids[group->tg_leader]);
	for (i = 0; i < group->tg_nmembers; i++) {
		if (i != group->tg_leader)
			(void)pmc_release(group->tg_ids[i]);
	}
}

static int
setup_group(struct test_group *group, u_int nmembers, u_int leader)
{
	u_int i;
	int error;

	memset(group, 0, sizeof(*group));
	group->tg_nmembers = nmembers;
	group->tg_leader = leader;
	for (i = 0; i < nitems(group->tg_ids); i++)
		group->tg_ids[i] = PMC_ID_INVALID;
	for (i = 0; i < nmembers; i++) {
		if (pmc_allocate_group("instructions", PMC_MODE_TC, 0,
		    PMC_CPU_ANY, &group->tg_ids[i], 0) != 0)
			goto fail;
	}
	if (pmc_group_create(&group->tg_groupid) != 0)
		goto fail;
	for (i = 0; i < nmembers; i++) {
		if (pmc_group_add(group->tg_groupid, group->tg_ids[i],
		    i == leader) != 0)
			goto fail;
	}
	errno = 0;
	if (pmc_group_add(group->tg_groupid, group->tg_ids[leader], 1) == 0 ||
	    errno != EINVAL) {
		errno = EPROTO;
		goto fail;
	}
	if (pmc_group_commit(group->tg_groupid) != 0)
		goto fail;
	return (0);

fail:
	error = errno;
	for (i = 0; i < nmembers; i++) {
		if (group->tg_ids[i] != PMC_ID_INVALID)
			(void)pmc_release(group->tg_ids[i]);
	}
	errno = error;
	return (-1);
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
	for (i = 0; i < SPIN_ITERS; i++)
		sink += i;
}

ATF_TC_WITHOUT_HEAD(capacity_and_order);
ATF_TC_BODY(capacity_and_order, tc)
{
	struct pmc_group_member members[2], members_before[2];
	struct pmc_group_times times, times_before;
	struct test_group group;
	pmc_value_t value;
	uint64_t enabled, running;
	uint32_t n;
	int rv;

	require_hwpmc();
	ATF_REQUIRE_MSG(setup_group(&group, 2, 1) == 0,
	    "setup_group failed: %s", strerror(errno));

	memset(&times, 0xa5, sizeof(times));
	memcpy(&times_before, &times, sizeof(times));
	n = 0;
	ATF_CHECK_EQ(pmc_group_read(group.tg_ids[group.tg_leader], &n, NULL,
	    &times), 0);
	ATF_CHECK_EQ(n, 2);
	ATF_CHECK(memcmp(&times, &times_before, sizeof(times)) == 0);

	memset(members, 0xa5, sizeof(members));
	memcpy(members_before, members, sizeof(members));
	memset(&times, 0xa5, sizeof(times));
	memcpy(&times_before, &times, sizeof(times));
	n = 1;
	errno = 0;
	rv = pmc_group_read(group.tg_ids[group.tg_leader], &n, members,
	    &times);
	ATF_CHECK_EQ(rv, -1);
	ATF_CHECK_EQ(errno, E2BIG);
	ATF_CHECK_EQ(n, 2);
	ATF_CHECK(memcmp(members, members_before, sizeof(members)) == 0);
	ATF_CHECK(memcmp(&times, &times_before, sizeof(times)) == 0);

	memset(members, 0, sizeof(members));
	memset(&times, 0, sizeof(times));
	n = nitems(members);
	ATF_CHECK_EQ(pmc_group_read(group.tg_ids[group.tg_leader], &n,
	    members, &times), 0);
	ATF_CHECK_EQ(n, 2);
	ATF_CHECK_EQ(members[0].pm_pmcid, group.tg_ids[group.tg_leader]);
	ATF_CHECK_EQ(members[1].pm_pmcid, group.tg_ids[0]);
	ATF_CHECK_EQ(members[0].pm_value, 0);
	ATF_CHECK_EQ(members[1].pm_value, 0);
	ATF_CHECK_EQ(times.pgt_enabled, 0);
	ATF_CHECK_EQ(times.pgt_running, 0);
	ATF_CHECK_EQ(times.pgt_enabled_wall, 0);
	ATF_CHECK_EQ(times.pgt_wall, 0);
	ATF_CHECK_EQ(times.pgt_flags, PMC_GROUP_F_TIME_THREAD_NS);

	n = PMC_GROUP_MAX_MEMBERS + 1;
	errno = 0;
	ATF_CHECK_EQ(pmc_group_read(group.tg_ids[group.tg_leader], &n,
	    members, &times), -1);
	ATF_CHECK_EQ(errno, EINVAL);
	ATF_CHECK_EQ(n, PMC_GROUP_MAX_MEMBERS + 1);

	memset(members, 0xa5, sizeof(members));
	memcpy(members_before, members, sizeof(members));
	memset(&times, 0xa5, sizeof(times));
	memcpy(&times_before, &times, sizeof(times));
	n = nitems(members);
	errno = 0;
	ATF_CHECK_EQ(pmc_group_read(group.tg_ids[0], &n, members, &times), -1);
	ATF_CHECK_EQ(errno, ENOTTY);
	ATF_CHECK_EQ(n, nitems(members));
	ATF_CHECK(memcmp(members, members_before, sizeof(members)) == 0);
	ATF_CHECK(memcmp(&times, &times_before, sizeof(times)) == 0);

	value = 11;
	enabled = 22;
	running = 33;
	errno = 0;
	ATF_CHECK_EQ(pmc_read_pair(group.tg_ids[0], &value, &enabled,
	    &running), -1);
	ATF_CHECK_EQ(errno, EOPNOTSUPP);
	ATF_CHECK_EQ(value, 11);
	ATF_CHECK_EQ(enabled, 22);
	ATF_CHECK_EQ(running, 33);

	cleanup_group(&group);
}

ATF_TC_WITHOUT_HEAD(live_snapshot);
ATF_TC_BODY(live_snapshot, tc)
{
	struct pmc_group_member first, second, third;
	struct pmc_group_times first_times, second_times, third_times;
	struct test_group group;
	pmc_value_t pair_value;
	uint64_t pair_enabled, pair_running;
	uint32_t n;

	require_hwpmc();
	ATF_REQUIRE_MSG(setup_group(&group, 1, 0) == 0,
	    "setup_group failed: %s", strerror(errno));
	if (pmc_attach(group.tg_ids[0], getpid()) != 0) {
		ATF_CHECK_MSG(false, "pmc_attach failed: %s", strerror(errno));
		cleanup_group(&group);
		return;
	}
	if (pmc_start(group.tg_ids[0]) != 0) {
		ATF_CHECK_MSG(false, "pmc_start failed: %s", strerror(errno));
		cleanup_group(&group);
		return;
	}
	group.tg_started = true;

	spin();
	n = 1;
	if (pmc_group_read(group.tg_ids[0], &n, &first, &first_times) != 0) {
		ATF_CHECK_MSG(false, "first group read failed: %s",
		    strerror(errno));
		cleanup_group(&group);
		return;
	}
	n = 1;
	if (pmc_group_read(group.tg_ids[0], &n, &second,
	    &second_times) != 0) {
		ATF_CHECK_MSG(false, "second group read failed: %s",
		    strerror(errno));
		cleanup_group(&group);
		return;
	}
	spin();
	n = 1;
	if (pmc_group_read(group.tg_ids[0], &n, &third, &third_times) != 0) {
		ATF_CHECK_MSG(false, "third group read failed: %s",
		    strerror(errno));
		cleanup_group(&group);
		return;
	}

	ATF_CHECK_EQ(first.pm_pmcid, group.tg_ids[0]);
	ATF_CHECK(first.pm_value > 0);
	ATF_CHECK(second.pm_value >= first.pm_value);
	ATF_CHECK(third.pm_value > second.pm_value);
	ATF_CHECK(second.pm_value - first.pm_value < first.pm_value);
	ATF_CHECK(first_times.pgt_running <= first_times.pgt_enabled);
	ATF_CHECK(second_times.pgt_running <= second_times.pgt_enabled);
	ATF_CHECK(third_times.pgt_running <= third_times.pgt_enabled);
	ATF_CHECK(first_times.pgt_enabled_wall > 0);
	ATF_CHECK(first_times.pgt_wall > 0);
	ATF_CHECK(second_times.pgt_enabled >= first_times.pgt_enabled);
	ATF_CHECK(second_times.pgt_running >= first_times.pgt_running);
	ATF_CHECK(second_times.pgt_enabled_wall >=
	    first_times.pgt_enabled_wall);
	ATF_CHECK(second_times.pgt_wall >= first_times.pgt_wall);
	ATF_CHECK(third_times.pgt_enabled > second_times.pgt_enabled);
	ATF_CHECK(third_times.pgt_running > second_times.pgt_running);
	ATF_CHECK(third_times.pgt_enabled_wall >
	    second_times.pgt_enabled_wall);
	ATF_CHECK(third_times.pgt_wall > second_times.pgt_wall);
	ATF_CHECK_EQ(third_times.pgt_flags, PMC_GROUP_F_TIME_THREAD_NS);

	pair_value = 0;
	pair_enabled = 0;
	pair_running = 0;
	ATF_CHECK_EQ(pmc_read_pair(group.tg_ids[0], &pair_value,
	    &pair_enabled, &pair_running), 0);
	ATF_CHECK(pair_value >= third.pm_value);
	ATF_CHECK(pair_enabled >= third_times.pgt_enabled);
	ATF_CHECK(pair_running >= third_times.pgt_running);
	ATF_CHECK(pair_running <= pair_enabled);

	cleanup_group(&group);
}

ATF_TP_ADD_TCS(tp)
{

	ATF_TP_ADD_TC(tp, capacity_and_order);
	ATF_TP_ADD_TC(tp, live_snapshot);
	return (atf_no_error());
}
