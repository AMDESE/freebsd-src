/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Advanced Micro Devices, Inc.
 */

#include <sys/param.h>
#include <sys/cpuset.h>
#include <sys/mman.h>
#include <sys/module.h>
#include <sys/sysctl.h>

#include <atf-c.h>
#include <errno.h>
#include <fcntl.h>
#include <pmc.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

/*
 * Deliberate installed-kernel tests only: lbr_hardware=true, as root.
 * Never load modules or administratively enable disabled PMC rows here.
 * The legacy AMD event table
 * maps FR_RETIRED_BRANCHES to core event 0xc2, with no unit-mask requirement.
 * Three simultaneous core rows suffice, including the allocation control.
 */
#define	EVENT		"k8-fr-retired-branches"
#define	USER_LBR	EVENT ",usr,lbr"
#define	KERNEL_LBR	EVENT ",os,lbr"
#define	SLOW_PERIOD	(UINT64_C(1) << 40)

struct context {
	pmc_id_t id[3];
	bool running[3];
	bool attached[3];
	bool logging;
	int logfd;
	int cpu[2];
	cpuset_t affinity;
	bool bound;
};

static void
metadata(atf_tc_t *tc, const char *description)
{
	atf_tc_set_md_var(tc, "descr", "%s", description);
	atf_tc_set_md_var(tc, "require.config", "lbr_hardware");
	atf_tc_set_md_var(tc, "is.exclusive", "true");
	atf_tc_set_md_var(tc, "timeout", "60");
}

/*
 * Used on ordinary completion and before every fatal assertion made after an
 * allocation.  Try every stop/release even if one fails.  Never deconfigure
 * the log while an unreleased handle might still be running.
 */
static void
cleanup(struct context *c)
{
	bool released;
	size_t i;
	int error;

	released = true;
	for (i = 0; i < nitems(c->id); i++) {
		if (!c->running[i])
			continue;
		error = pmc_stop(c->id[i]);
		ATF_CHECK_MSG(error == 0, "cleanup stop %#x: %s",
		    (unsigned)c->id[i], strerror(errno));
		if (error == 0)
			c->running[i] = false;
	}
	for (i = 0; i < nitems(c->id); i++) {
		if (c->id[i] == PMC_ID_INVALID)
			continue;
		error = pmc_release(c->id[i]);
		ATF_CHECK_MSG(error == 0, "cleanup release %#x: %s",
		    (unsigned)c->id[i], strerror(errno));
		if (error == 0) {
			c->id[i] = PMC_ID_INVALID;
			c->running[i] = c->attached[i] = false;
		} else
			released = false;
	}
	if (c->logging && released) {
		error = pmc_configure_logfile(-1);
		ATF_CHECK_MSG(error == 0, "cleanup logfile: %s", strerror(errno));
		if (error == 0)
			c->logging = false;
	}
	if (c->logfd != -1) {
		ATF_CHECK_MSG(close(c->logfd) == 0, "close logfile: %s",
		    strerror(errno));
		c->logfd = -1;
	}
	if (c->bound) {
		ATF_CHECK_MSG(cpuset_setaffinity(CPU_LEVEL_WHICH, CPU_WHICH_TID,
		    -1, sizeof(c->affinity), &c->affinity) == 0,
		    "restore test thread affinity: %s", strerror(errno));
		c->bound = false;
	}
}

static void
require_call(struct context *c, int result, const char *operation)
{
	int saved_errno;

	if (result == 0)
		return;
	saved_errno = errno;
	cleanup(c);
	ATF_REQUIRE_MSG(result == 0, "%s: errno %d (%s)", operation,
	    saved_errno, strerror(saved_errno));
}

#define	CALL(C, EXPR)	require_call((C), (EXPR), #EXPR)

static void
expect_error(struct context *c, int result, int saved_errno, int expected,
    const char *operation)
{
	if (result == -1 && saved_errno == expected)
		return;
	cleanup(c);
	ATF_REQUIRE_MSG(result == -1 && saved_errno == expected,
	    "%s: result %d, errno %d (%s), expected -1 and errno %d (%s)",
	    operation, result, saved_errno, strerror(saved_errno),
	    expected, strerror(expected));
}

static void
setup(const atf_tc_t *tc, struct context *c, unsigned int needed_cpus)
{
	struct pmc_pmcinfo *info;
	cpuset_t allowed, selected;
	size_t len, i;
	unsigned int found;
	int cpu, depth, npmcs, row, enabled;

	memset(c, 0, sizeof(*c));
	c->logfd = -1;
	for (i = 0; i < nitems(c->id); i++)
		c->id[i] = PMC_ID_INVALID;
	ATF_REQUIRE_MSG(atf_tc_get_config_var_as_bool_wd(tc,
	    "lbr_hardware", false), "explicit lbr_hardware=true is required");
	ATF_REQUIRE_MSG(geteuid() == 0, "root is required");
	ATF_REQUIRE_MSG(pmc_init() == 0,
	    "prerequisite: installed hwpmc support: %s", strerror(errno));
	len = sizeof(depth);
	ATF_REQUIRE_MSG(sysctlbyname("kern.hwpmc.amd_lbr_depth", &depth,
	    &len, NULL, 0) == 0,
	    "prerequisite: installed AMD LBR kernel support: %s",
	    strerror(errno));
	ATF_REQUIRE_MSG(len == sizeof(depth) && depth > 0,
	    "prerequisite: kern.hwpmc.amd_lbr_depth must be positive");
	ATF_REQUIRE_MSG(cpuset_getaffinity(CPU_LEVEL_WHICH, CPU_WHICH_TID,
	    -1, sizeof(allowed), &allowed) == 0,
	    "prerequisite: CPU affinity: %s", strerror(errno));
	found = 0;
	for (cpu = 0; cpu < CPU_SETSIZE && found < needed_cpus; cpu++) {
		if (!CPU_ISSET(cpu, &allowed))
			continue;
		npmcs = pmc_npmc(cpu);
		ATF_REQUIRE_MSG(npmcs > 0,
		    "prerequisite: CPU %d has no accessible PMCs", cpu);
		ATF_REQUIRE_MSG(pmc_pmcinfo(cpu, &info) == 0,
		    "prerequisite: CPU %d PMC information: %s",
		    cpu, strerror(errno));
		enabled = 0;
		for (row = 0; row < npmcs; row++)
			if (info->pm_pmcs[row].pm_enabled &&
			    info->pm_pmcs[row].pm_class == PMC_CLASS_K8)
				enabled++;
		free(info);
		if (enabled != 0)
			c->cpu[found++] = cpu;
	}
	ATF_REQUIRE_MSG(found == needed_cpus,
	    "prerequisite: need %u allowed CPUs with enabled AMD PMCs, found %u",
	    needed_cpus, found);
	/* Put self-attached TS on the CPU used by the first SS allocation. */
	c->affinity = allowed;
	CPU_ZERO(&selected);
	CPU_SET(c->cpu[0], &selected);
	CALL(c, cpuset_setaffinity(CPU_LEVEL_WHICH, CPU_WHICH_TID, -1,
	    sizeof(selected), &selected));
	c->bound = true;
}

static void
allocate(struct context *c, unsigned int slot, enum pmc_mode mode, int cpu,
    const char *event)
{
	int result;

	result = pmc_allocate(event, mode, 0, cpu, &c->id[slot],
	    PMC_IS_SAMPLING_MODE(mode) ? SLOW_PERIOD : 0);
	require_call(c, result,
	    "positive allocation control (event/filter/available core row)");
}

static void
release_slot(struct context *c, unsigned int slot)
{
	CALL(c, pmc_release(c->id[slot]));
	c->id[slot] = PMC_ID_INVALID;
	c->running[slot] = c->attached[slot] = false;
}

static void
reject_allocation(struct context *c, unsigned int slot, enum pmc_mode mode,
    int cpu, const char *event, int expected)
{
	int result, saved_errno;

	errno = 0;
	result = pmc_allocate(event, mode, 0, cpu, &c->id[slot],
	    PMC_IS_SAMPLING_MODE(mode) ? SLOW_PERIOD : 0);
	saved_errno = errno;
	/* If allocation unexpectedly succeeds, cleanup owns the returned ID. */
	expect_error(c, result, saved_errno, expected, event);
}

static void
configure_log(struct context *c)
{
	c->logfd = open("lbr.pmc", O_CREAT | O_TRUNC | O_RDWR, 0600);
	if (c->logfd == -1)
		require_call(c, -1, "open private ATF-work-directory logfile");
	CALL(c, pmc_configure_logfile(c->logfd));
	c->logging = true;
}

static void
start_slot(struct context *c, unsigned int slot, bool is_thread)
{
	if (is_thread && !c->attached[slot]) {
		CALL(c, pmc_attach(c->id[slot], 0));
		c->attached[slot] = true;
	}
	CALL(c, pmc_start(c->id[slot]));
	c->running[slot] = true;
}

static void
stop_slot(struct context *c, unsigned int slot)
{
	CALL(c, pmc_stop(c->id[slot]));
	c->running[slot] = false;
}

ATF_TC(lbr_counting_rejected);
ATF_TC_HEAD(lbr_counting_rejected, tc)
{
	metadata(tc, "LBR rejects TC and SC with EINVAL after counting controls");
}
ATF_TC_BODY(lbr_counting_rejected, tc)
{
	struct context c;
	enum pmc_mode mode;
	int cpu, thread;

	setup(tc, &c, 1);
	/* Prove the event's LBR sampling request is supported too. */
	allocate(&c, 0, PMC_MODE_TS, PMC_CPU_ANY, USER_LBR);
	release_slot(&c, 0);
	for (thread = 0; thread < 2; thread++) {
		mode = thread ? PMC_MODE_TC : PMC_MODE_SC;
		cpu = thread ? PMC_CPU_ANY : c.cpu[0];
		allocate(&c, 0, mode, cpu, EVENT ",usr");
		release_slot(&c, 0);
		reject_allocation(&c, 0, mode, cpu, USER_LBR, EINVAL);
	}
	cleanup(&c);
}

ATF_TC(lbr_matching_and_conflicting);
ATF_TC_HEAD(lbr_matching_and_conflicting, tc)
{
	metadata(tc, "Matching TS and same-CPU SS share reservations; "
	    "conflicts return EBUSY while existing handles remain usable");
}
ATF_TC_BODY(lbr_matching_and_conflicting, tc)
{
	struct context c;
	enum pmc_mode mode;
	int cpu, thread;

	setup(tc, &c, 1);
	for (thread = 0; thread < 2; thread++) {
		mode = thread ? PMC_MODE_TS : PMC_MODE_SS;
		cpu = thread ? PMC_CPU_ANY : c.cpu[0];
		allocate(&c, 0, mode, cpu, USER_LBR);
		allocate(&c, 1, mode, cpu, USER_LBR);
		/* Prove a third compatible row is available before EBUSY. */
		allocate(&c, 2, mode, cpu, USER_LBR);
		release_slot(&c, 2);
		reject_allocation(&c, 2, mode, cpu, KERNEL_LBR, EBUSY);
		CALL(&c, pmc_set(c.id[0], SLOW_PERIOD));
		CALL(&c, pmc_set(c.id[1], SLOW_PERIOD));
		release_slot(&c, 1);
		reject_allocation(&c, 2, mode, cpu, KERNEL_LBR, EBUSY);
		release_slot(&c, 0);
		allocate(&c, 2, mode, cpu, KERNEL_LBR);
		release_slot(&c, 2);
	}
	cleanup(&c);
}

ATF_TC(lbr_ts_ss_conflicts);
ATF_TC_HEAD(lbr_ts_ss_conflicts, tc)
{
	metadata(tc, "TS constrains SS and SS constrains TS at allocation");
}
ATF_TC_BODY(lbr_ts_ss_conflicts, tc)
{
	struct context c;
	enum pmc_mode first, second;
	int firstcpu, secondcpu, thread;

	setup(tc, &c, 1);
	for (thread = 0; thread < 2; thread++) {
		first = thread ? PMC_MODE_TS : PMC_MODE_SS;
		second = thread ? PMC_MODE_SS : PMC_MODE_TS;
		firstcpu = thread ? PMC_CPU_ANY : c.cpu[0];
		secondcpu = thread ? c.cpu[0] : PMC_CPU_ANY;
		allocate(&c, 0, first, firstcpu, USER_LBR);
		/* Different modes need distinct rows; prove both are available. */
		allocate(&c, 1, second, secondcpu, USER_LBR);
		release_slot(&c, 1);
		reject_allocation(&c, 1, second, secondcpu, KERNEL_LBR, EBUSY);
		CALL(&c, pmc_set(c.id[0], SLOW_PERIOD));
		allocate(&c, 1, second, secondcpu, USER_LBR);
		release_slot(&c, 1);
		release_slot(&c, 0);
	}
	cleanup(&c);
}

ATF_TC(lbr_self_requires_logfile);
ATF_TC_HEAD(lbr_self_requires_logfile, tc)
{
	metadata(tc, "Self-attached TS requires a logfile before start");
}
ATF_TC_BODY(lbr_self_requires_logfile, tc)
{
	struct context c;
	int result, saved_errno;

	setup(tc, &c, 1);
	allocate(&c, 0, PMC_MODE_TS, PMC_CPU_ANY, USER_LBR);
	CALL(&c, pmc_attach(c.id[0], 0));
	c.attached[0] = true;
	errno = 0;
	result = pmc_start(c.id[0]);
	saved_errno = errno;
	c.running[0] = result == 0;
	expect_error(&c, result, saved_errno, EDOOFUS, "self TS without logfile");
	configure_log(&c);
	start_slot(&c, 0, true);
	stop_slot(&c, 0);
	cleanup(&c);
}

ATF_TC(lbr_stopped_reservation);
ATF_TC_HEAD(lbr_stopped_reservation, tc)
{
	metadata(tc, "Stopped TS and SS retain their filter until release");
}
ATF_TC_BODY(lbr_stopped_reservation, tc)
{
	struct context c;
	enum pmc_mode mode;
	int cpu, thread;

	setup(tc, &c, 1);
	configure_log(&c);
	for (thread = 0; thread < 2; thread++) {
		mode = thread ? PMC_MODE_TS : PMC_MODE_SS;
		cpu = thread ? PMC_CPU_ANY : c.cpu[0];
		allocate(&c, 0, mode, cpu, USER_LBR);
		allocate(&c, 1, mode, cpu, USER_LBR);
		release_slot(&c, 1);
		start_slot(&c, 0, thread != 0);
		stop_slot(&c, 0);
		reject_allocation(&c, 1, mode, cpu, KERNEL_LBR, EBUSY);
		CALL(&c, pmc_set(c.id[0], SLOW_PERIOD));
		release_slot(&c, 0);
		allocate(&c, 1, mode, cpu, KERNEL_LBR);
		start_slot(&c, 1, thread != 0);
		stop_slot(&c, 1);
		release_slot(&c, 1);
	}
	cleanup(&c);
}

ATF_TC(lbr_simultaneous_ts_ss);
ATF_TC_HEAD(lbr_simultaneous_ts_ss, tc)
{
	metadata(tc, "Compatible TS and SS start, stop and restart together");
}
ATF_TC_BODY(lbr_simultaneous_ts_ss, tc)
{
	struct context c;

	setup(tc, &c, 1);
	allocate(&c, 0, PMC_MODE_TS, PMC_CPU_ANY, USER_LBR);
	allocate(&c, 1, PMC_MODE_SS, c.cpu[0], USER_LBR);
	configure_log(&c);
	start_slot(&c, 0, true);
	start_slot(&c, 1, false);
	stop_slot(&c, 0);
	stop_slot(&c, 1);
	start_slot(&c, 1, false);
	start_slot(&c, 0, true);
	stop_slot(&c, 1);
	stop_slot(&c, 0);
	cleanup(&c);
}

ATF_TC(lbr_disjoint_ss_filters);
ATF_TC_HEAD(lbr_disjoint_ss_filters, tc)
{
	metadata(tc, "Different SS filters coexist on two enabled CPUs");
}
ATF_TC_BODY(lbr_disjoint_ss_filters, tc)
{
	struct context c;

	setup(tc, &c, 2);
	allocate(&c, 0, PMC_MODE_SS, c.cpu[0], USER_LBR);
	allocate(&c, 1, PMC_MODE_SS, c.cpu[1], USER_LBR);
	release_slot(&c, 1);
	allocate(&c, 1, PMC_MODE_SS, c.cpu[1], KERNEL_LBR);
	configure_log(&c);
	start_slot(&c, 0, false);
	start_slot(&c, 1, false);
	stop_slot(&c, 1);
	stop_slot(&c, 0);
	cleanup(&c);
}

ATF_TC(lbr_copyout_rollback);
ATF_TC_HEAD(lbr_copyout_rollback, tc)
{
	metadata(tc, "Failed handle copyout returns EFAULT and releases "
	    "the TS or SS filter reservation");
}
ATF_TC_BODY(lbr_copyout_rollback, tc)
{
	struct pmc_op_pmcallocate *pa;
	struct module_stat ms;
	struct context c;
	enum pmc_mode mode;
	size_t pagesize;
	int cpu, modid, result, saved_errno, thread;

	setup(tc, &c, 1);
	modid = modfind("hwpmc");
	if (modid == -1)
		require_call(&c, -1, "find installed hwpmc syscall module");
	memset(&ms, 0, sizeof(ms));
	ms.version = sizeof(ms);
	CALL(&c, modstat(modid, &ms));
	pagesize = (size_t)getpagesize();
	for (thread = 0; thread < 2; thread++) {
		mode = thread ? PMC_MODE_TS : PMC_MODE_SS;
		cpu = thread ? PMC_CPU_ANY : c.cpu[0];
		pa = mmap(NULL, pagesize, PROT_READ | PROT_WRITE,
		    MAP_ANON | MAP_PRIVATE, -1, 0);
		if (pa == MAP_FAILED)
			require_call(&c, -1, "map allocation argument page");
		pa->pm_class = PMC_CLASS_K8;
		pa->pm_caps = PMC_CAP_USER | PMC_CAP_INTERRUPT | PMC_CAP_LBR;
		pa->pm_ev = PMC_EV_K8_FR_RETIRED_BRANCHES;
		pa->pm_mode = mode;
		pa->pm_cpu = cpu;
		pa->pm_count = SLOW_PERIOD;

		/* Prove this exact raw request succeeds before fault injection. */
		CALL(&c, syscall(ms.data.intval, PMC_OP_PMCALLOCATE, pa));
		c.id[0] = pa->pm_pmcid;
		release_slot(&c, 0);
		pa->pm_pmcid = PMC_ID_INVALID;
		CALL(&c, mprotect(pa, pagesize, PROT_READ));
		errno = 0;
		result = syscall(ms.data.intval, PMC_OP_PMCALLOCATE, pa);
		saved_errno = errno;
		CALL(&c, munmap(pa, pagesize));
		expect_error(&c, result, saved_errno, EFAULT,
		    "allocation handle copyout to a read-only page");

		/* A leaked user-only reservation would reject this with EBUSY. */
		allocate(&c, 0, mode, cpu, KERNEL_LBR);
		release_slot(&c, 0);
	}
	cleanup(&c);
}

ATF_TP_ADD_TCS(tp)
{
	ATF_TP_ADD_TC(tp, lbr_counting_rejected);
	ATF_TP_ADD_TC(tp, lbr_matching_and_conflicting);
	ATF_TP_ADD_TC(tp, lbr_ts_ss_conflicts);
	ATF_TP_ADD_TC(tp, lbr_self_requires_logfile);
	ATF_TP_ADD_TC(tp, lbr_stopped_reservation);
	ATF_TP_ADD_TC(tp, lbr_simultaneous_ts_ss);
	ATF_TP_ADD_TC(tp, lbr_disjoint_ss_filters);
	ATF_TP_ADD_TC(tp, lbr_copyout_rollback);
	return (atf_no_error());
}
