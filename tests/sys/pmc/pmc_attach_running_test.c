/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Advanced Micro Devices, Inc.
 */

/*
 * Start a counting PMC on a target that already runs and never blocks.
 * No context switch loads the PMC, so the start must load it.
 */

#include <sys/param.h>
#include <sys/cpuset.h>
#include <sys/sysctl.h>
#include <sys/wait.h>

#include <errno.h>
#include <pmc.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>

#include <atf-c.h>

static const char *const hw_events[] = {
	"instructions", "inst_retired.any", "unhalted-core-cycles",
	"cycles", "cpu_clk_unhalted.thread", NULL
};

static void
require_hwpmc(void)
{

	if (geteuid() != 0)
		atf_tc_skip("attaching to another process requires root");
	if (pmc_init() != 0)
		atf_tc_skip("hwpmc(4) is not available");
}

static pmc_id_t
require_counting_pmc(void)
{
	pmc_id_t id;
	int k;

	for (k = 0; hw_events[k] != NULL; k++) {
		if (pmc_allocate(hw_events[k], PMC_MODE_TC, 0, PMC_CPU_ANY,
		    &id, 0) == 0)
			return (id);
	}
	atf_tc_skip("no hardware counting PMC is allocatable: %s",
	    strerror(errno));
}

/* Return a CPU this process may use other than 'other', or skip. */
static int
pick_cpu(int other)
{
	cpuset_t set;
	int cpu;

	ATF_REQUIRE(cpuset_getaffinity(CPU_LEVEL_WHICH, CPU_WHICH_PID, -1,
	    sizeof(set), &set) == 0);
	for (cpu = CPU_SETSIZE - 1; cpu >= 0; cpu--) {
		if (CPU_ISSET(cpu, &set) && cpu != other)
			return (cpu);
	}
	atf_tc_skip("needs two CPUs");
}

/* Return 0 or the error number. */
static int
set_cpu(pid_t pid, int cpu)
{
	cpuset_t set;

	CPU_ZERO(&set);
	CPU_SET(cpu, &set);
	if (cpuset_setaffinity(CPU_LEVEL_WHICH, CPU_WHICH_PID, pid,
	    sizeof(set), &set) != 0)
		return (errno);
	return (0);
}

static void
pin(pid_t pid, int cpu)
{
	int error;

	error = set_cpu(pid, cpu);
	ATF_REQUIRE_MSG(error == 0, "pin %d to CPU %d: %s", (int)pid, cpu,
	    strerror(error));
}

ATF_TC_WITHOUT_HEAD(start_on_running_target);
ATF_TC_BODY(start_on_running_target, tc)
{
	pmc_value_t v;
	pmc_id_t id;
	pid_t child;
	int cpu, status;

	require_hwpmc();
	cpu = pick_cpu(-1);
	pin(getpid(), pick_cpu(cpu));
	id = require_counting_pmc();

	ATF_REQUIRE((child = fork()) >= 0);
	if (child == 0) {
		volatile uint64_t sink = 0;

		/* Spin, and never block. */
		for (;;)
			sink++;
	}
	pin(child, cpu);
	/* Let the child reach its CPU and run there. */
	(void)usleep(200000);

	ATF_REQUIRE_MSG(pmc_attach(id, child) == 0, "pmc_attach: %s",
	    strerror(errno));
	ATF_REQUIRE_MSG(pmc_start(id) == 0, "pmc_start: %s",
	    strerror(errno));
	(void)usleep(500000);

	/* The exit saves the final count. */
	(void)kill(child, SIGKILL);
	ATF_REQUIRE(waitpid(child, &status, 0) == child);

	ATF_REQUIRE_MSG(pmc_read(id, &v) == 0, "pmc_read: %s",
	    strerror(errno));
	ATF_CHECK_MSG(v > 0,
	    "a PMC started on a running target counted nothing");

	(void)pmc_stop(id);
	ATF_CHECK(pmc_release(id) == 0);
}

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

#define	GROUP_MEMBERS	2
#define	GROUP_READS	4

/* Allocate and commit a group of counting members, or skip. */
static void
require_counting_group(pmc_id_t ids[GROUP_MEMBERS])
{
	uint32_t groupid;
	int i;

	if (!is_amd())
		atf_tc_skip("PMC grouping is supported only on AMD CPUs");
	for (i = 0; i < GROUP_MEMBERS; i++) {
		if (pmc_allocate_group("instructions", PMC_MODE_TC, 0,
		    PMC_CPU_ANY, &ids[i], 0) != 0)
			atf_tc_skip("no group member is allocatable: %s",
			    strerror(errno));
	}
	ATF_REQUIRE_MSG(pmc_group_create(&groupid) == 0,
	    "pmc_group_create: %s", strerror(errno));
	for (i = 0; i < GROUP_MEMBERS; i++)
		ATF_REQUIRE_MSG(pmc_group_add(groupid, ids[i], i == 0) == 0,
		    "pmc_group_add: %s", strerror(errno));
	ATF_REQUIRE_MSG(pmc_group_commit(groupid) == 0,
	    "pmc_group_commit: %s", strerror(errno));
}

/*
 * Start a group on a target that already runs and never blocks.  A live
 * group read must show counts before the target switches out.
 */
ATF_TC_WITHOUT_HEAD(group_start_on_running_target);
ATF_TC_BODY(group_start_on_running_target, tc)
{
	struct pmc_group_member members[GROUP_MEMBERS];
	struct pmc_group_times times;
	pmc_id_t ids[GROUP_MEMBERS];
	pid_t child;
	uint32_t n;
	int attach_error, cpu, i, pin_error, read_error, reads;
	int start_error, status;
	bool counted;

	require_hwpmc();
	cpu = pick_cpu(-1);
	pin(getpid(), pick_cpu(cpu));
	require_counting_group(ids);

	ATF_REQUIRE((child = fork()) >= 0);
	if (child == 0) {
		volatile uint64_t sink = 0;

		/* Spin, and never block. */
		for (;;)
			sink++;
	}

	/* Kill the child before any check can stop the test. */
	attach_error = read_error = start_error = 0;
	counted = false;
	reads = 0;
	pin_error = set_cpu(child, cpu);
	if (pin_error == 0) {
		/* Let the child reach its CPU and run there. */
		(void)usleep(200000);
		if (pmc_attach(ids[0], child) != 0)
			attach_error = errno;
		else if (pmc_start(ids[0]) != 0)
			start_error = errno;
	}
	for (i = 0; pin_error == 0 && attach_error == 0 &&
	    start_error == 0 && i < GROUP_READS; i++) {
		(void)usleep(10000);
		n = GROUP_MEMBERS;
		reads++;
		if (pmc_group_read(ids[0], &n, members, &times) != 0) {
			read_error = errno;
			break;
		}
		if (n == GROUP_MEMBERS && times.pgt_running > 0 &&
		    members[0].pm_value > 0 && members[1].pm_value > 0) {
			counted = true;
			break;
		}
	}
	(void)kill(child, SIGKILL);
	ATF_REQUIRE(waitpid(child, &status, 0) == child);

	ATF_REQUIRE_MSG(pin_error == 0, "pin %d to CPU %d: %s", (int)child,
	    cpu, strerror(pin_error));
	ATF_REQUIRE_MSG(attach_error == 0, "pmc_attach: %s",
	    strerror(attach_error));
	ATF_REQUIRE_MSG(start_error == 0, "pmc_start: %s",
	    strerror(start_error));
	ATF_REQUIRE_MSG(read_error == 0, "pmc_group_read: %s",
	    strerror(read_error));
	ATF_CHECK_MSG(counted,
	    "a group started on a running target counted nothing in %d "
	    "live reads", reads);
	(void)pmc_stop(ids[0]);
	ATF_CHECK(pmc_release(ids[0]) == 0);
}

ATF_TP_ADD_TCS(tp)
{

	ATF_TP_ADD_TC(tp, start_on_running_target);
	ATF_TP_ADD_TC(tp, group_start_on_running_target);

	return (atf_no_error());
}
