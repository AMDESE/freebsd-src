/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Alexander Leidinger <netchild@FreeBSD.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other advertising materials provided with the
 *    distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/*
 * Regression tests for detaching a process-mode PMC from a target that
 * still has it loaded on hardware.  The runcount reference for a live
 * PMC is dropped only by the context-switch-out reclaim, which the
 * scheduler gates on P_HWPMC; detaching used to clear P_HWPMC without
 * draining the reference, so the following release spun forever in
 * pmc_wait_for_pmc_idle() - a panic ("waiting too long for pmc to be
 * free") on an INVARIANTS kernel, an unkillable loop holding the hwpmc
 * lock otherwise.
 *
 * The failure is deterministic: attaching a counting PMC to the current
 * (running) thread and detaching it before release always left the
 * reference live.  On a kernel that carries the fix these tests complete
 * immediately; run against a kernel that lacks it they would wedge, so
 * (like any hwpmc regression test) they ship alongside the fix.
 *
 * They need an allocatable process-mode counting event, and skip where
 * none is available (hwpmc(4) not loaded, or a VM without a vPMU).
 */

#include <sys/types.h>
#include <sys/cpuset.h>
#include <sys/wait.h>

#include <errno.h>
#include <pmc.h>
#include <pthread.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <atf-c.h>

static const char *counting_events[] = {
	"instructions",
	"cycles",
	"branches",
	"unhalted-core-cycles",
	"inst_retired.any",
	"cpu_clk_unhalted.thread",
	"ls_not_halted_cyc",
	NULL
};

static const char *counting_event;	/* set by alloc_counting_pmc() */
static volatile int spin_stop;

static void *
spinner(void *arg __unused)
{
	volatile unsigned long s __unused = 0;

	while (spin_stop == 0)
		s += 1;
	return (NULL);
}

static void
burn_cpu(void)
{
	volatile unsigned long s __unused = 0;
	int i;

	for (i = 0; i < 1000000; i++)
		s += i;
}

/* Allocate the first available process-mode counting PMC, or skip. */
static pmc_id_t
alloc_counting_pmc(void)
{
	pmc_id_t id;
	int i;

	if (pmc_init() != 0)
		atf_tc_skip("hwpmc(4) is not available: %s", strerror(errno));

	for (i = 0; counting_events[i] != NULL; i++) {
		id = PMC_ID_INVALID;
		if (pmc_allocate(counting_events[i], PMC_MODE_TC, 0,
		    PMC_CPU_ANY, &id, 0) == 0) {
			counting_event = counting_events[i];
			return (id);
		}
	}
	atf_tc_skip("no allocatable process-mode counting event "
	    "(no vPMU?)");
	return (PMC_ID_INVALID); /* not reached */
}

ATF_TC(detach_live_self);
ATF_TC_HEAD(detach_live_self, tc)
{
	atf_tc_set_md_var(tc, "descr",
	    "Detaching a running counting PMC from the current thread and "
	    "then releasing it does not leak its runcount reference");
	atf_tc_set_md_var(tc, "require.user", "root");
}
ATF_TC_BODY(detach_live_self, tc)
{
	pmc_id_t id;

	id = alloc_counting_pmc();

	ATF_REQUIRE(pmc_attach(id, 0) == 0);	/* 0 == current process */
	ATF_REQUIRE(pmc_start(id) == 0);
	burn_cpu();				/* make the counter live */

	/* Detach while still loaded on this CPU, then release. */
	ATF_REQUIRE(pmc_detach(id, 0) == 0);
	ATF_REQUIRE(pmc_release(id) == 0);	/* wedged before the fix */
}

ATF_TC(detach_live_multithread);
ATF_TC_HEAD(detach_live_multithread, tc)
{
	atf_tc_set_md_var(tc, "descr",
	    "Detaching a running process-scope counting PMC drains the "
	    "references held by the process' other threads");
	atf_tc_set_md_var(tc, "require.user", "root");
}
ATF_TC_BODY(detach_live_multithread, tc)
{
	pthread_t th[3];
	pmc_id_t id;
	int i, n;

	id = alloc_counting_pmc();

	/* Sibling threads keep the process-scope PMC live on other CPUs. */
	spin_stop = 0;
	for (n = 0; n < 3; n++)
		ATF_REQUIRE(pthread_create(&th[n], NULL, spinner, NULL) == 0);

	ATF_REQUIRE(pmc_attach(id, 0) == 0);
	ATF_REQUIRE(pmc_start(id) == 0);
	burn_cpu();
	/* Let the siblings be scheduled and pick up the PMC. */
	usleep(200 * 1000);

	ATF_REQUIRE(pmc_detach(id, 0) == 0);
	ATF_REQUIRE(pmc_release(id) == 0);	/* must drain the siblings */

	spin_stop = 1;
	for (i = 0; i < n; i++)
		pthread_join(th[i], NULL);
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

/*
 * Owner of PMC B.  Allocate B with the event of A, attach it to 'target'
 * and start it.  Send the row and the error, then hold B until the
 * parent closes 'hold'.
 */
static void
run_second_owner(pid_t target, int result, int hold, int unused[2])
{
	int reply[2];
	pmc_id_t b;
	char c;

	/* Close the parent ends: a read of 'hold' then sees EOF. */
	(void)close(unused[0]);
	(void)close(unused[1]);
	reply[0] = reply[1] = 0;
	if (pmc_allocate(counting_event, PMC_MODE_TC, 0, PMC_CPU_ANY, &b,
	    0) != 0)
		reply[1] = errno;
	else {
		reply[0] = PMC_ID_TO_ROWINDEX(b);
		if (pmc_attach(b, target) != 0 || pmc_start(b) != 0)
			reply[1] = errno;
	}
	(void)write(result, reply, sizeof(reply));
	(void)read(hold, &c, 1);
	_exit(0);
}

ATF_TC(detach_running_target_frees_row);
ATF_TC_HEAD(detach_running_target_frees_row, tc)
{
	atf_tc_set_md_var(tc, "descr",
	    "Detaching a PMC from a target that runs on another CPU removes "
	    "the PMC from that CPU before another PMC takes its row");
	atf_tc_set_md_var(tc, "require.user", "root");
}
ATF_TC_BODY(detach_running_target_frees_row, tc)
{
	pmc_id_t a;
	pid_t other, owner, target;
	int cpu, error, hold[2], parent_ends[2], reply[2], result[2], status;

	a = alloc_counting_pmc();
	cpu = pick_cpu(-1);
	ATF_REQUIRE(set_cpu(getpid(), pick_cpu(cpu)) == 0);
	ATF_REQUIRE(pipe(hold) == 0 && pipe(result) == 0);
	reply[0] = reply[1] = 0;
	owner = -1;
	/* A second target keeps A attached after the detach. */
	ATF_REQUIRE((other = fork()) >= 0);
	if (other == 0) {
		for (;;)
			(void)pause();
	}
	ATF_REQUIRE((target = fork()) >= 0);
	if (target == 0) {
		volatile uint64_t sink = 0;

		/* Spin, and never block. */
		for (;;)
			sink++;
	}

	/* Kill the children before any check can stop the test. */
	error = set_cpu(target, cpu);
	if (error == 0) {
		/* Let the target reach its CPU and run there. */
		(void)usleep(200000);
		if (pmc_attach(a, other) != 0 || pmc_attach(a, target) != 0 ||
		    pmc_start(a) != 0)
			error = errno;
	}
	if (error == 0) {
		/* Let the target switch in with A. */
		(void)usleep(100000);
		if (pmc_detach(a, target) != 0)
			error = errno;
	}
	parent_ends[0] = result[0];
	parent_ends[1] = hold[1];
	if (error == 0 && (owner = fork()) == 0)
		run_second_owner(target, result[1], hold[0], parent_ends);
	if (owner < 0 && error == 0)
		error = errno;
	/* A dead owner then gives EOF. */
	(void)close(result[1]);
	if (owner > 0 && read(result[0], reply, sizeof(reply)) !=
	    sizeof(reply))
		error = EIO;
	if (error == 0)
		(void)usleep(100000);
	/*
	 * Without the fix, A stays on the row of the target CPU.  The exit
	 * of the target then finds A where it expects B.
	 */
	(void)kill(target, SIGKILL);
	(void)kill(other, SIGKILL);
	(void)close(hold[1]);
	ATF_REQUIRE(waitpid(target, &status, 0) == target);
	ATF_REQUIRE(waitpid(other, &status, 0) == other);
	if (owner > 0)
		ATF_REQUIRE(waitpid(owner, &status, 0) == owner);
	ATF_REQUIRE_MSG(error == 0, "setup: %s", strerror(error));
	ATF_REQUIRE_MSG(reply[1] == 0, "PMC B: %s", strerror(reply[1]));
	(void)pmc_stop(a);
	ATF_CHECK(pmc_release(a) == 0);
	if (reply[0] != PMC_ID_TO_ROWINDEX(a))
		atf_tc_skip("PMC B used row %d, not row %d", reply[0],
		    PMC_ID_TO_ROWINDEX(a));
}

ATF_TP_ADD_TCS(tp)
{
	ATF_TP_ADD_TC(tp, detach_live_self);
	ATF_TP_ADD_TC(tp, detach_live_multithread);
	ATF_TP_ADD_TC(tp, detach_running_target_frees_row);

	return (atf_no_error());
}
