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
#include <sys/wait.h>

#include <errno.h>
#include <pmc.h>
#include <signal.h>
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

static void
pin(pid_t pid, int cpu)
{
	cpuset_t set;

	CPU_ZERO(&set);
	CPU_SET(cpu, &set);
	ATF_REQUIRE_MSG(cpuset_setaffinity(CPU_LEVEL_WHICH, CPU_WHICH_PID,
	    pid, sizeof(set), &set) == 0, "pin %d to CPU %d: %s", (int)pid,
	    cpu, strerror(errno));
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

ATF_TP_ADD_TCS(tp)
{

	ATF_TP_ADD_TC(tp, start_on_running_target);

	return (atf_no_error());
}
