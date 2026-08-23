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
#include <sys/stat.h>
#include <sys/sysctl.h>
#include <sys/wait.h>

#include <errno.h>
#include <fcntl.h>
#include <pmc.h>
#include <pmclog.h>
#include <pthread.h>
#include <sched.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <atf-c.h>

#define	TEST_EVENT	"instructions"
#define	TEST_CPU	0
#define	SAMPLE_PERIOD	65536
#define	SPIN_ITERS	(400ULL * 1000 * 1000)

#define	TEST_SYSCTL_PREFIX	"kern.hwpmc.test."
#define	TEST_GROUP_STATE	TEST_SYSCTL_PREFIX "group_state"
#define	TEST_HOLD_GROUP_RESIDENT					\
    TEST_SYSCTL_PREFIX "hold_group_resident"
#define	TEST_HOLD_GROUP_RESIDENT_ACK				\
    TEST_SYSCTL_PREFIX "hold_group_resident_ack"
#define	TEST_HOLD_GROUP_EVICTED					\
    TEST_SYSCTL_PREFIX "hold_group_evicted"
#define	TEST_HOLD_GROUP_EVICTED_ACK				\
    TEST_SYSCTL_PREFIX "hold_group_evicted_ack"
#define	TEST_LIVE_PMC_TARGETS					\
    TEST_SYSCTL_PREFIX "live_pmc_targets"
#define	TEST_LIVE_GROUP_TARGETS					\
    TEST_SYSCTL_PREFIX "live_group_targets"
#define	TEST_LIVE_TARGET_PROCESSES				\
    TEST_SYSCTL_PREFIX "live_target_processes"
#define	TEST_LIVE_RESIDUAL_ENTRIES				\
    TEST_SYSCTL_PREFIX "live_residual_entries"
#define	TEST_LIVE_ROTATION_REFS					\
    TEST_SYSCTL_PREFIX "live_rotation_refs"
#define	TEST_LIVE_RUN_REFS					\
    TEST_SYSCTL_PREFIX "live_run_refs"
#define	TEST_FAIL_SYSTEM_START_AFTER				\
    TEST_SYSCTL_PREFIX "fail_system_start_after"
#define	TEST_PAUSE_SYSTEM_START_AFTER_FIRST			\
    TEST_SYSCTL_PREFIX "pause_system_start_after_first"
#define	TEST_PAUSE_SYSTEM_START_AFTER_FIRST_ACK			\
    TEST_SYSCTL_PREFIX "pause_system_start_after_first_ack"
#define	TEST_PAUSE_SYSTEM_START_MEMBER_COUNT			\
    TEST_SYSCTL_PREFIX "pause_system_start_member_count"
#define	TEST_PAUSE_SAMPLE_WORKER				\
    TEST_SYSCTL_PREFIX "pause_sample_worker"
#define	TEST_PAUSE_SAMPLE_WORKER_ACK				\
    TEST_SYSCTL_PREFIX "pause_sample_worker_ack"
#define	TEST_PAUSE_SAMPLE_SCHEDULE_OUT_ACK			\
    TEST_SYSCTL_PREFIX "pause_sample_schedule_out_ack"
#define	TEST_FAIL_CALLCHAIN_LOG_HANDLE				\
    TEST_SYSCTL_PREFIX "fail_callchain_log_handle"
#define	TEST_FAIL_CALLCHAIN_LOG_HANDLE_ACK			\
    TEST_SYSCTL_PREFIX "fail_callchain_log_handle_ack"
#define	TEST_SAMPLE_COUNTS					\
    TEST_SYSCTL_PREFIX "sample_counts"
#define	TEST_INJECT_KERNEL_SAMPLE					\
    TEST_SYSCTL_PREFIX "inject_kernel_sample"
#define	TEST_LOG_MARKER	0x47535046U
#define	TEST_OWNER_LOG_MARKER	0x47534f4cU
#define	TEST_LOG_FAILURE_MARKER	0x434c4641U
#define	TEST_LOG_DRAIN_MARKER	0x44524149U

struct group {
	uint32_t	g_id;
	pmc_id_t	g_ids[PMC_GROUP_MAX_MEMBERS];
	u_int		g_n;
	bool		g_started;
};

struct pmc_test_group_state {
	uint64_t	ptgs_handle;
	uint32_t	ptgs_running;
	uint32_t	ptgs_assigned;
	uint32_t	ptgs_sys_listed;
	uint32_t	ptgs_sscounted;
	uint32_t	ptgs_nevents;
	uint32_t	ptgs_running_members;
	uint32_t	ptgs_stopped_members;
	uint32_t	ptgs_allocated_members;
};

struct pmc_test_sample_counts {
	uint64_t	ptsc_handle;
	uint32_t	ptsc_pid;
	uint32_t	ptsc_tid;
	uint64_t	ptsc_accepted;
	uint64_t	ptsc_emitted;
	uint64_t	ptsc_dropped;
	uint64_t	ptsc_run_refs;
};

struct live_target_counts {
	uint64_t	ltc_pmc_targets;
	uint64_t	ltc_group_targets;
	uint64_t	ltc_target_processes;
	uint64_t	ltc_residual_entries;
	uint64_t	ltc_rotation_refs;
	uint64_t	ltc_run_refs;
};

struct group_snapshot {
	struct pmc_group_member	gs_members[PMC_GROUP_MAX_MEMBERS];
	struct pmc_group_times	gs_times;
	uint32_t		gs_nmembers;
};

struct group_build_failure {
	const char	*gbf_operation;
	u_int		gbf_member;
	int		gbf_errno;
};

struct owner_fork_log_counts {
	u_int		oflc_before;
	u_int		oflc_after;
	bool		oflc_marker_seen;
};

struct start_thread_result {
	pthread_mutex_t	str_lock;
	pmc_id_t	str_leader;
	int		str_rc;
	int		str_errno;
	bool		str_start_called;
	bool		str_done;
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

static int
sysctl_read_u32(const char *name, u_int *value)
{
	size_t length;

	length = sizeof(*value);
	return (sysctlbyname(name, value, &length, NULL, 0));
}

static int
sysctl_write_u32(const char *name, u_int value)
{

	return (sysctlbyname(name, NULL, NULL, &value, sizeof(value)));
}

static int
sysctl_read_u64(const char *name, uint64_t *value)
{
	size_t length;

	length = sizeof(*value);
	return (sysctlbyname(name, value, &length, NULL, 0));
}

static int
sysctl_write_u64(const char *name, uint64_t value)
{

	return (sysctlbyname(name, NULL, NULL, &value, sizeof(value)));
}

static void
require_hwpmc_test_support(void)
{
	uint64_t value;
	int error;

	errno = 0;
	error = sysctl_read_u64(TEST_LIVE_PMC_TARGETS, &value);
	if (error == 0)
		return;
	if (errno == ENOENT)
		atf_tc_skip("requires an HWPMCDEBUG module with test support");
	atf_tc_fail("reading %s failed: errno %d (%s)",
	    TEST_LIVE_PMC_TARGETS, errno, strerror(errno));
}

static int
read_group_state(pmc_id_t leader, struct pmc_test_group_state *state)
{
	uint64_t handle;
	size_t length;

	handle = leader;
	length = sizeof(*state);
	memset(state, 0, sizeof(*state));
	return (sysctlbyname(TEST_GROUP_STATE, state, &length, &handle,
	    sizeof(handle)));
}

static int
read_sample_counts(pmc_id_t pmcid, struct pmc_test_sample_counts *counts)
{
	uint64_t handle;
	size_t length;

	handle = pmcid;
	length = sizeof(*counts);
	memset(counts, 0, sizeof(*counts));
	return (sysctlbyname(TEST_SAMPLE_COUNTS, counts, &length, &handle,
	    sizeof(handle)));
}

static int
wait_test_sysctl_ack(const char *name, u_int expected)
{
	u_int value;
	int i;

	for (i = 0; i < 5000; i++) {
		if (sysctl_read_u32(name, &value) != 0)
			return (-1);
		if (value == expected)
			return (0);
		usleep(1000);
	}
	errno = ETIMEDOUT;
	return (-1);
}

static void
reset_system_start_test_hooks(void)
{

	(void)sysctl_write_u32(TEST_PAUSE_SYSTEM_START_AFTER_FIRST, 0);
	(void)sysctl_write_u32(TEST_PAUSE_SAMPLE_WORKER, 0);
	(void)sysctl_write_u32(TEST_FAIL_SYSTEM_START_AFTER, UINT_MAX);
	(void)sysctl_write_u32(TEST_FAIL_CALLCHAIN_LOG_HANDLE,
	    PMC_ID_INVALID);
}

static void
reset_system_stop_test_hooks(void)
{

	(void)sysctl_write_u32(TEST_HOLD_GROUP_RESIDENT, 0);
	(void)sysctl_write_u32(TEST_HOLD_GROUP_EVICTED, 0);
}

static int
start_thread_result_init(struct start_thread_result *result, pmc_id_t leader)
{
	int error;

	memset(result, 0, sizeof(*result));
	result->str_leader = leader;
	error = pthread_mutex_init(&result->str_lock, NULL);
	return (error);
}

static void *
start_group_thread(void *arg)
{
	struct start_thread_result *result;
	cpuset_t set;
	int error, rc;

	result = arg;
	CPU_ZERO(&set);
	CPU_SET(TEST_CPU, &set);
	if (cpuset_setaffinity(CPU_LEVEL_WHICH, CPU_WHICH_TID, -1,
	    sizeof(set), &set) != 0) {
		rc = -1;
		error = errno;
	} else {
		result->str_start_called = true;
		errno = 0;
		rc = pmc_start(result->str_leader);
		error = errno;
	}

	(void)pthread_mutex_lock(&result->str_lock);
	result->str_rc = rc;
	result->str_errno = error;
	result->str_done = true;
	(void)pthread_mutex_unlock(&result->str_lock);
	return (NULL);
}

static bool
start_thread_is_done(struct start_thread_result *result)
{
	bool done;

	(void)pthread_mutex_lock(&result->str_lock);
	done = result->str_done;
	(void)pthread_mutex_unlock(&result->str_lock);
	return (done);
}

static int
wait_test_sysctl_ack_or_start_done(const char *name, u_int expected,
    struct start_thread_result *result, bool *start_done)
{
	u_int value;
	int i;

	*start_done = false;
	for (i = 0; i < 5000; i++) {
		if (sysctl_read_u32(name, &value) != 0)
			return (-1);
		if (value == expected)
			return (0);
		if (start_thread_is_done(result)) {
			*start_done = true;
			return (0);
		}
		usleep(1000);
	}
	errno = ETIMEDOUT;
	return (-1);
}

static int
read_live_target_counts_raw(struct live_target_counts *counts,
    const char **failed_name)
{

	*failed_name = NULL;
	if (sysctl_read_u64(TEST_LIVE_PMC_TARGETS,
	    &counts->ltc_pmc_targets) != 0) {
		*failed_name = TEST_LIVE_PMC_TARGETS;
		return (-1);
	}
	if (sysctl_read_u64(TEST_LIVE_GROUP_TARGETS,
	    &counts->ltc_group_targets) != 0) {
		*failed_name = TEST_LIVE_GROUP_TARGETS;
		return (-1);
	}
	if (sysctl_read_u64(TEST_LIVE_TARGET_PROCESSES,
	    &counts->ltc_target_processes) != 0) {
		*failed_name = TEST_LIVE_TARGET_PROCESSES;
		return (-1);
	}
	if (sysctl_read_u64(TEST_LIVE_RESIDUAL_ENTRIES,
	    &counts->ltc_residual_entries) != 0) {
		*failed_name = TEST_LIVE_RESIDUAL_ENTRIES;
		return (-1);
	}
	if (sysctl_read_u64(TEST_LIVE_ROTATION_REFS,
	    &counts->ltc_rotation_refs) != 0) {
		*failed_name = TEST_LIVE_ROTATION_REFS;
		return (-1);
	}
	if (sysctl_read_u64(TEST_LIVE_RUN_REFS, &counts->ltc_run_refs) != 0) {
		*failed_name = TEST_LIVE_RUN_REFS;
		return (-1);
	}
	return (0);
}

static void
read_live_target_counts(struct live_target_counts *counts)
{
	const char *failed_name;

	ATF_REQUIRE_MSG(read_live_target_counts_raw(counts, &failed_name) == 0,
	    "reading %s failed: errno %d (%s)", failed_name, errno,
	    strerror(errno));
}

static bool
live_target_counts_equal(const struct live_target_counts *a,
    const struct live_target_counts *b)
{

	return (a->ltc_pmc_targets == b->ltc_pmc_targets &&
	    a->ltc_group_targets == b->ltc_group_targets &&
	    a->ltc_target_processes == b->ltc_target_processes &&
	    a->ltc_residual_entries == b->ltc_residual_entries &&
	    a->ltc_rotation_refs == b->ltc_rotation_refs &&
	    a->ltc_run_refs == b->ltc_run_refs);
}

static int
wait_live_target_counts(const struct live_target_counts *expected,
    struct live_target_counts *actual, const char **failed_name)
{
	int i;

	for (i = 0; i < 5000; i++) {
		if (read_live_target_counts_raw(actual, failed_name) != 0)
			return (-1);
		if (live_target_counts_equal(actual, expected))
			return (0);
		usleep(1000);
	}
	*failed_name = NULL;
	errno = ETIMEDOUT;
	return (-1);
}

static int
open_named_logfile(char *path, size_t path_size)
{
	int length;

	length = snprintf(path, path_size,
	    "/tmp/pmc-sys-sampling-test.XXXXXX");
	if (length < 0 || (size_t)length >= path_size) {
		errno = ENAMETOOLONG;
		return (-1);
	}
	return (mkstemp(path));
}

static int
scan_log_marker(const char *path, uint32_t marker, pmc_id_t sample_pmc,
    u_int *kernel_maps, bool *kernel_sample_seen,
    bool *mapping_before_kernel_sample, bool *marker_seen)
{
	struct pmclog_ev event;
	void *parser;
	int fd, saved_errno;

	*kernel_maps = 0;
	*kernel_sample_seen = false;
	*mapping_before_kernel_sample = false;
	*marker_seen = false;
	fd = open(path, O_RDONLY);
	if (fd < 0)
		return (-1);
	parser = pmclog_open(fd);
	if (parser == NULL) {
		saved_errno = errno;
		(void)close(fd);
		errno = saved_errno;
		return (-1);
	}

	memset(&event, 0, sizeof(event));
	while (pmclog_read(parser, &event) == 0) {
		if (event.pl_type == PMCLOG_TYPE_MAP_IN &&
		    event.pl_u.pl_mi.pl_pid == (pid_t)-1)
			(*kernel_maps)++;
		if (event.pl_type == PMCLOG_TYPE_CALLCHAIN &&
		    event.pl_u.pl_cc.pl_pmcid == sample_pmc &&
		    event.pl_u.pl_cc.pl_pid == UINT32_MAX &&
		    PMC_CALLCHAIN_CPUFLAGS_TO_USERMODE(
		    event.pl_u.pl_cc.pl_cpuflags) == 0) {
			*kernel_sample_seen = true;
			if (*kernel_maps > 0)
				*mapping_before_kernel_sample = true;
		}
		if (event.pl_type == PMCLOG_TYPE_USERDATA &&
		    event.pl_u.pl_u.pl_userdata == marker) {
			*marker_seen = true;
			break;
		}
	}
	if (event.pl_state == PMCLOG_ERROR) {
		saved_errno = EPROTO;
		pmclog_close(parser);
		(void)close(fd);
		errno = saved_errno;
		return (-1);
	}
	pmclog_close(parser);
	if (close(fd) != 0)
		return (-1);
	return (0);
}

static int
wait_log_marker(const char *path, uint32_t marker, pmc_id_t sample_pmc,
    u_int *kernel_maps, bool *kernel_sample_seen,
    bool *mapping_before_kernel_sample)
{
	bool marker_seen;
	int i;

	for (i = 0; i < 5000; i++) {
		if (scan_log_marker(path, marker, sample_pmc, kernel_maps,
		    kernel_sample_seen, mapping_before_kernel_sample,
		    &marker_seen) != 0)
			return (-1);
		if (marker_seen)
			return (0);
		usleep(1000);
	}
	errno = ETIMEDOUT;
	return (-1);
}

static int
scan_log_callchain_marker(const char *path, uint32_t marker,
    pmc_id_t sample_pmc, uint64_t *callchains, bool *marker_seen)
{
	struct pmclog_ev event;
	void *parser;
	int fd, saved_errno;

	*callchains = 0;
	*marker_seen = false;
	fd = open(path, O_RDONLY);
	if (fd < 0)
		return (-1);
	parser = pmclog_open(fd);
	if (parser == NULL) {
		saved_errno = errno;
		(void)close(fd);
		errno = saved_errno;
		return (-1);
	}

	memset(&event, 0, sizeof(event));
	while (pmclog_read(parser, &event) == 0) {
		if (event.pl_type == PMCLOG_TYPE_CALLCHAIN &&
		    event.pl_u.pl_cc.pl_pmcid == sample_pmc)
			(*callchains)++;
		if (event.pl_type == PMCLOG_TYPE_USERDATA &&
		    event.pl_u.pl_u.pl_userdata == marker) {
			*marker_seen = true;
			break;
		}
	}
	if (event.pl_state == PMCLOG_ERROR) {
		saved_errno = EPROTO;
		pmclog_close(parser);
		(void)close(fd);
		errno = saved_errno;
		return (-1);
	}
	pmclog_close(parser);
	if (close(fd) != 0)
		return (-1);
	return (0);
}

static int
wait_log_callchain_marker(const char *path, uint32_t marker,
    pmc_id_t sample_pmc, uint64_t *callchains)
{
	bool marker_seen;
	int i;

	for (i = 0; i < 5000; i++) {
		if (scan_log_callchain_marker(path, marker, sample_pmc,
		    callchains, &marker_seen) != 0)
			return (-1);
		if (marker_seen)
			return (0);
		usleep(1000);
	}
	errno = ETIMEDOUT;
	return (-1);
}

static int
scan_log_callchain_marker_exact(const char *path, uint32_t marker,
    pmc_id_t sample_pmc, uint32_t sample_pid, uint32_t sample_tid,
    uint64_t *callchains, bool *marker_seen)
{
	struct pmclog_ev event;
	void *parser;
	int fd, saved_errno;

	*callchains = 0;
	*marker_seen = false;
	fd = open(path, O_RDONLY);
	if (fd < 0)
		return (-1);
	parser = pmclog_open(fd);
	if (parser == NULL) {
		saved_errno = errno;
		(void)close(fd);
		errno = saved_errno;
		return (-1);
	}

	memset(&event, 0, sizeof(event));
	while (pmclog_read(parser, &event) == 0) {
		if (event.pl_type == PMCLOG_TYPE_CALLCHAIN &&
		    event.pl_u.pl_cc.pl_pmcid == sample_pmc &&
		    event.pl_u.pl_cc.pl_pid == sample_pid &&
		    event.pl_u.pl_cc.pl_tid == sample_tid)
			(*callchains)++;
		if (event.pl_type == PMCLOG_TYPE_USERDATA &&
		    event.pl_u.pl_u.pl_userdata == marker) {
			*marker_seen = true;
			break;
		}
	}
	if (event.pl_state == PMCLOG_ERROR) {
		saved_errno = EPROTO;
		pmclog_close(parser);
		(void)close(fd);
		errno = saved_errno;
		return (-1);
	}
	pmclog_close(parser);
	if (close(fd) != 0)
		return (-1);
	return (0);
}

static int
wait_log_callchain_marker_exact(const char *path, uint32_t marker,
    pmc_id_t sample_pmc, uint32_t sample_pid, uint32_t sample_tid,
    uint64_t *callchains)
{
	bool marker_seen;
	int i;

	for (i = 0; i < 5000; i++) {
		if (scan_log_callchain_marker_exact(path, marker, sample_pmc,
		    sample_pid, sample_tid, callchains, &marker_seen) != 0)
			return (-1);
		if (marker_seen)
			return (0);
		usleep(1000);
	}
	errno = ETIMEDOUT;
	return (-1);
}

static int
fork_sentinel(pid_t *child_pid)
{
	pid_t child, waited;
	int status;

	child = fork();
	if (child < 0)
		return (-1);
	if (child == 0)
		_exit(0);
	do {
		waited = waitpid(child, &status, 0);
	} while (waited < 0 && errno == EINTR);
	if (waited < 0)
		return (-1);
	if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) {
		errno = ECHILD;
		return (-1);
	}
	*child_pid = child;
	return (0);
}

static int
scan_owner_fork_log(const char *path, uint32_t marker, pid_t owner_pid,
    pid_t before_pid, pid_t after_pid, struct owner_fork_log_counts *counts)
{
	struct pmclog_ev event;
	void *parser;
	int fd, saved_errno;

	memset(counts, 0, sizeof(*counts));
	fd = open(path, O_RDONLY);
	if (fd < 0)
		return (-1);
	parser = pmclog_open(fd);
	if (parser == NULL) {
		saved_errno = errno;
		(void)close(fd);
		errno = saved_errno;
		return (-1);
	}

	memset(&event, 0, sizeof(event));
	while (pmclog_read(parser, &event) == 0) {
		if (event.pl_type == PMCLOG_TYPE_PROCFORK &&
		    event.pl_u.pl_f.pl_oldpid == owner_pid) {
			if (event.pl_u.pl_f.pl_newpid == before_pid)
				counts->oflc_before++;
			if (event.pl_u.pl_f.pl_newpid == after_pid)
				counts->oflc_after++;
		}
		if (event.pl_type == PMCLOG_TYPE_USERDATA &&
		    event.pl_u.pl_u.pl_userdata == marker) {
			counts->oflc_marker_seen = true;
			break;
		}
	}
	if (event.pl_state == PMCLOG_ERROR) {
		saved_errno = EPROTO;
		pmclog_close(parser);
		(void)close(fd);
		errno = saved_errno;
		return (-1);
	}
	pmclog_close(parser);
	if (close(fd) != 0)
		return (-1);
	return (0);
}

static int
wait_sample_reconciled(pmc_id_t pmcid,
    struct pmc_test_sample_counts *counts)
{
	int i;

	for (i = 0; i < 5000; i++) {
		if (read_sample_counts(pmcid, counts) != 0)
			return (-1);
		if (counts->ptsc_accepted ==
		    counts->ptsc_emitted + counts->ptsc_dropped &&
		    counts->ptsc_run_refs == 0)
			return (0);
		usleep(1000);
	}
	errno = ETIMEDOUT;
	return (-1);
}

static int
wait_sample_pending(pmc_id_t pmcid, struct pmc_test_sample_counts *counts)
{
	int i;

	for (i = 0; i < 5000; i++) {
		if (read_sample_counts(pmcid, counts) != 0)
			return (-1);
		if (counts->ptsc_accepted > 0 &&
		    counts->ptsc_emitted == 0 &&
		    counts->ptsc_dropped == 0 &&
		    counts->ptsc_run_refs == counts->ptsc_accepted)
			return (0);
		usleep(1000);
	}
	errno = ETIMEDOUT;
	return (-1);
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
group_build_n(struct group *g, u_int nmembers, bool sampling_leader,
    bool multiplex, struct group_build_failure *failure)
{
	enum pmc_mode mode;
	uint32_t flags;
	pmc_value_t count;
	u_int i;

	if (failure != NULL) {
		failure->gbf_operation = NULL;
		failure->gbf_member = UINT_MAX;
		failure->gbf_errno = 0;
	}
	group_init(g);
	g->g_n = nmembers;
	for (i = 0; i < nmembers; i++) {
		mode = sampling_leader && i == 0 ? PMC_MODE_SS : PMC_MODE_SC;
		flags = multiplex && i == 0 ? PMC_F_GROUP_MUX : 0;
		count = mode == PMC_MODE_SS ? SAMPLE_PERIOD : 0;
		if (pmc_allocate_group(TEST_EVENT, mode, flags, TEST_CPU,
		    &g->g_ids[i], count) != 0) {
			if (failure != NULL) {
				failure->gbf_operation = "pmc_allocate_group";
				failure->gbf_member = i;
				failure->gbf_errno = errno;
			}
			return (-1);
		}
	}
	if (pmc_group_create(&g->g_id) != 0) {
		if (failure != NULL) {
			failure->gbf_operation = "pmc_group_create";
			failure->gbf_errno = errno;
		}
		return (-1);
	}
	for (i = 0; i < g->g_n; i++) {
		if (pmc_group_add(g->g_id, g->g_ids[i], i == 0) != 0) {
			if (failure != NULL) {
				failure->gbf_operation = "pmc_group_add";
				failure->gbf_member = i;
				failure->gbf_errno = errno;
			}
			return (-1);
		}
	}
	if (pmc_group_commit(g->g_id) != 0) {
		if (failure != NULL) {
			failure->gbf_operation = "pmc_group_commit";
			failure->gbf_errno = errno;
		}
		return (-1);
	}
	return (0);
}

static int
group_build(struct group *g)
{

	return (group_build_n(g, 2, true, false, NULL));
}

static void
format_group_build_failure(char *buffer, size_t size, const char *description,
    const struct group_build_failure *failure)
{

	if (failure->gbf_member == UINT_MAX) {
		snprintf(buffer, size, "%s failed at %s: errno %d (%s)",
		    description, failure->gbf_operation, failure->gbf_errno,
		    strerror(failure->gbf_errno));
	} else {
		snprintf(buffer, size,
		    "%s failed at %s for member %u: errno %d (%s)",
		    description, failure->gbf_operation, failure->gbf_member,
		    failure->gbf_errno, strerror(failure->gbf_errno));
	}
}

static void
group_build_required(struct group *g)
{
	u_int i;

	group_init(g);
	g->g_n = 2;
	ATF_REQUIRE_MSG(pmc_allocate_group(TEST_EVENT, PMC_MODE_SS, 0,
	    TEST_CPU, &g->g_ids[0], SAMPLE_PERIOD) == 0,
	    "sampling-member allocation failed: %s", strerror(errno));
	ATF_REQUIRE_MSG(pmc_allocate_group(TEST_EVENT, PMC_MODE_SC, 0,
	    TEST_CPU, &g->g_ids[1], 0) == 0,
	    "counting-member allocation failed: %s", strerror(errno));
	ATF_REQUIRE_MSG(pmc_group_create(&g->g_id) == 0,
	    "group creation failed: %s", strerror(errno));
	for (i = 0; i < g->g_n; i++) {
		ATF_REQUIRE_MSG(pmc_group_add(g->g_id, g->g_ids[i], i == 0) == 0,
		    "group add for member %u failed: %s", i, strerror(errno));
	}
	ATF_REQUIRE_MSG(pmc_group_commit(g->g_id) == 0,
	    "group commit failed: %s", strerror(errno));
}

static u_int
probe_group_capacity(void)
{
	struct group probe;
	u_int n;

	for (n = PMC_GROUP_MAX_MEMBERS; n > 0; n--) {
		if (group_build_n(&probe, n, false, false, NULL) == 0) {
			group_teardown(&probe);
			return (n);
		}
		group_teardown(&probe);
	}
	return (0);
}

static int
read_group_snapshot(const struct group *g, struct group_snapshot *snapshot)
{

	memset(snapshot, 0, sizeof(*snapshot));
	snapshot->gs_nmembers = g->g_n;
	return (pmc_group_read(g->g_ids[0], &snapshot->gs_nmembers,
	    snapshot->gs_members, &snapshot->gs_times));
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

static void
pin_to_test_cpu_required(void)
{
	cpuset_t set;

	CPU_ZERO(&set);
	CPU_SET(TEST_CPU, &set);
	ATF_REQUIRE_MSG(cpuset_setaffinity(CPU_LEVEL_WHICH, CPU_WHICH_TID, -1,
	    sizeof(set), &set) == 0, "cannot pin to CPU %d: errno %d (%s)",
	    TEST_CPU, errno, strerror(errno));
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

static int __attribute__((noinline))
spin_until_test_ack(const char *name, u_int expected)
{
	volatile uint64_t sink;
	uint64_t i;
	u_int value;
	int attempt;

	sink = 0;
	for (attempt = 0; attempt < 5000; attempt++) {
		for (i = 0; i < 100000; i++)
			sink += i + (uint64_t)attempt;
		if (sysctl_read_u32(name, &value) != 0)
			return (-1);
		if (value == expected)
			return (0);
	}
	errno = ETIMEDOUT;
	return (-1);
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
 * Starting the group joins this owner to the system-sampling owner list, and
 * stopping it removes this owner.  Exact fork records in this owner's logfile
 * prove both transitions without relying on a global statistic.
 */
ATF_TC_WITHOUT_HEAD(grouped_system_sampling_accounting);
ATF_TC_BODY(grouped_system_sampling_accounting, tc)
{
	struct group_build_failure build_failure;
	struct group g;
	struct live_target_counts baseline, final_counts;
	struct owner_fork_log_counts fork_counts;
	struct pmc_test_group_state running_state, stopped_state;
	const char *failed_counter;
	char failure[768], log_path[PATH_MAX];
	pid_t after_pid, before_pid, owner_pid;
	bool logfile_configured;
	int log_fd, saved_errno;

	group_init(&g);
	memset(&baseline, 0, sizeof(baseline));
	memset(&final_counts, 0, sizeof(final_counts));
	failure[0] = '\0';
	log_path[0] = '\0';
	log_fd = -1;
	logfile_configured = false;

	require_hwpmc();
	require_hwpmc_test_support();
	pin_to_test_cpu_required();
	reset_system_stop_test_hooks();
	read_live_target_counts(&baseline);

	log_fd = open_named_logfile(log_path, sizeof(log_path));
	if (log_fd < 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "creating owner-lifecycle logfile failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	if (pmc_configure_logfile(log_fd) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "configuring owner-lifecycle logfile failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	logfile_configured = true;

	if (group_build_n(&g, 2, true, true, &build_failure) != 0) {
		format_group_build_failure(failure, sizeof(failure),
		    "building owner-lifecycle sampling group", &build_failure);
		goto cleanup;
	}
	if (sysctl_write_u32(TEST_HOLD_GROUP_RESIDENT, g.g_id) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "requesting resident hold for owner-lifecycle group %u "
		    "failed: errno %d (%s)", g.g_id, saved_errno,
		    strerror(saved_errno));
		goto cleanup;
	}
	if (pmc_start(g.g_ids[0]) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "starting owner-lifecycle group failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	g.g_started = true;
	if (wait_test_sysctl_ack(TEST_HOLD_GROUP_RESIDENT_ACK, g.g_id) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "resident hold for owner-lifecycle group %u was not "
		    "acknowledged: errno %d (%s)", g.g_id, saved_errno,
		    strerror(saved_errno));
		goto cleanup;
	}
	if (read_group_state(g.g_ids[0], &running_state) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "reading running owner-lifecycle state failed: errno %d "
		    "(%s)", saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	if (running_state.ptgs_running != 1 ||
	    running_state.ptgs_assigned != 1 ||
	    running_state.ptgs_sys_listed != 1 ||
	    running_state.ptgs_sscounted != 1 ||
	    running_state.ptgs_nevents != 2 ||
	    running_state.ptgs_running_members != 2) {
		snprintf(failure, sizeof(failure),
		    "owner-lifecycle group was not held running: running=%u "
		    "assigned=%u listed=%u sscounted=%u events=%u "
		    "members(running=%u stopped=%u allocated=%u)",
		    running_state.ptgs_running, running_state.ptgs_assigned,
		    running_state.ptgs_sys_listed,
		    running_state.ptgs_sscounted, running_state.ptgs_nevents,
		    running_state.ptgs_running_members,
		    running_state.ptgs_stopped_members,
		    running_state.ptgs_allocated_members);
		goto cleanup;
	}

	owner_pid = getpid();
	if (fork_sentinel(&before_pid) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "forking the running-owner sentinel failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	if (pmc_stop(g.g_ids[0]) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "stopping owner-lifecycle group failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	g.g_started = false;
	reset_system_stop_test_hooks();
	if (read_group_state(g.g_ids[0], &stopped_state) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "reading stopped owner-lifecycle state failed: errno %d "
		    "(%s)", saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	if (stopped_state.ptgs_running != 0 ||
	    stopped_state.ptgs_assigned != 0 ||
	    stopped_state.ptgs_sscounted != 0 ||
	    stopped_state.ptgs_nevents != 2 ||
	    stopped_state.ptgs_running_members != 0 ||
	    stopped_state.ptgs_stopped_members != 2) {
		snprintf(failure, sizeof(failure),
		    "owner-lifecycle stop left published sampling state: "
		    "running=%u assigned=%u listed=%u sscounted=%u events=%u "
		    "members(running=%u stopped=%u allocated=%u)",
		    stopped_state.ptgs_running, stopped_state.ptgs_assigned,
		    stopped_state.ptgs_sys_listed,
		    stopped_state.ptgs_sscounted, stopped_state.ptgs_nevents,
		    stopped_state.ptgs_running_members,
		    stopped_state.ptgs_stopped_members,
		    stopped_state.ptgs_allocated_members);
		goto cleanup;
	}
	if (fork_sentinel(&after_pid) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "forking the stopped-owner sentinel failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	if (pmc_writelog(TEST_OWNER_LOG_MARKER) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "writing owner-lifecycle log marker failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	if (pmc_flush_logfile() != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "flushing owner-lifecycle log marker failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	if (scan_owner_fork_log(log_path, TEST_OWNER_LOG_MARKER, owner_pid,
	    before_pid, after_pid, &fork_counts) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "parsing owner-lifecycle logfile failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	if (!fork_counts.oflc_marker_seen || fork_counts.oflc_before != 1 ||
	    fork_counts.oflc_after != 0) {
		snprintf(failure, sizeof(failure),
		    "owner sampling lifecycle log mismatch: marker=%u "
		    "running-fork=%u expected=1 stopped-fork=%u expected=0 "
		    "owner=%jd running-child=%jd stopped-child=%jd",
		    fork_counts.oflc_marker_seen ? 1 : 0,
		    fork_counts.oflc_before, fork_counts.oflc_after,
		    (intmax_t)owner_pid, (intmax_t)before_pid,
		    (intmax_t)after_pid);
		goto cleanup;
	}

cleanup:
	reset_system_stop_test_hooks();
	group_teardown(&g);
	if (logfile_configured && pmc_configure_logfile(-1) != 0 &&
	    failure[0] == '\0') {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "deconfiguring owner-lifecycle logfile failed: errno %d "
		    "(%s)", saved_errno, strerror(saved_errno));
	}
	if (log_fd >= 0 && close(log_fd) != 0 && failure[0] == '\0') {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "closing owner-lifecycle logfile failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
	}
	if (log_path[0] != '\0' && unlink(log_path) != 0 &&
	    failure[0] == '\0') {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "unlinking owner-lifecycle logfile failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
	}
	if (wait_live_target_counts(&baseline, &final_counts,
	    &failed_counter) != 0) {
		saved_errno = errno;
		if (failed_counter != NULL) {
			atf_tc_fail("%s; reading %s during cleanup failed: errno "
			    "%d (%s)", failure[0] != '\0' ? failure :
			    "owner lifecycle assertion failed", failed_counter,
			    saved_errno, strerror(saved_errno));
		}
		atf_tc_fail("%s; cleanup counters did not return to baseline: "
		    "errno %d (%s), pmc_target=%ju/%ju group_target=%ju/%ju "
		    "process=%ju/%ju residual=%ju/%ju rotation=%ju/%ju "
		    "run=%ju/%ju", failure[0] != '\0' ? failure :
		    "owner lifecycle assertion failed", saved_errno,
		    strerror(saved_errno),
		    (uintmax_t)final_counts.ltc_pmc_targets,
		    (uintmax_t)baseline.ltc_pmc_targets,
		    (uintmax_t)final_counts.ltc_group_targets,
		    (uintmax_t)baseline.ltc_group_targets,
		    (uintmax_t)final_counts.ltc_target_processes,
		    (uintmax_t)baseline.ltc_target_processes,
		    (uintmax_t)final_counts.ltc_residual_entries,
		    (uintmax_t)baseline.ltc_residual_entries,
		    (uintmax_t)final_counts.ltc_rotation_refs,
		    (uintmax_t)baseline.ltc_rotation_refs,
		    (uintmax_t)final_counts.ltc_run_refs,
		    (uintmax_t)baseline.ltc_run_refs);
	}
	if (failure[0] != '\0')
		atf_tc_fail("%s", failure);
}

static void
run_system_release_while_running(bool evicted)
{
	struct group sampling, competitor, fresh;
	struct group_build_failure build_failure;
	struct live_target_counts baseline, final_counts;
	struct owner_fork_log_counts fork_counts;
	struct pmc_test_group_state before_release, competitor_state, fresh_state;
	const char *failed_counter, *residency;
	char failure[1024], log_path[PATH_MAX];
	pid_t after_pid, before_pid, owner_pid;
	bool logfile_configured;
	u_int cap, expected_running, expected_stopped;
	int error, log_fd, saved_errno;

	group_init(&sampling);
	group_init(&competitor);
	group_init(&fresh);
	memset(&baseline, 0, sizeof(baseline));
	memset(&final_counts, 0, sizeof(final_counts));
	failure[0] = '\0';
	log_path[0] = '\0';
	before_pid = -1;
	after_pid = -1;
	log_fd = -1;
	logfile_configured = false;
	residency = evicted ? "evicted" : "resident";

	require_hwpmc();
	require_hwpmc_test_support();
	pin_to_test_cpu_required();
	reset_system_stop_test_hooks();

	cap = probe_group_capacity();
	if (cap < 2)
		atf_tc_skip("requires at least two system PMC rows on CPU %d",
		    TEST_CPU);
	read_live_target_counts(&baseline);

	log_fd = open_named_logfile(log_path, sizeof(log_path));
	if (log_fd < 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "creating %s direct-release logfile failed: errno %d (%s)",
		    residency, saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	if (pmc_configure_logfile(log_fd) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "configuring %s direct-release logfile failed: errno %d "
		    "(%s)", residency, saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	logfile_configured = true;

	if (group_build_n(&sampling, cap, true, true, &build_failure) != 0) {
		format_group_build_failure(failure, sizeof(failure),
		    "building full direct-release sampling group",
		    &build_failure);
		goto cleanup;
	}
	if (sysctl_write_u32(TEST_HOLD_GROUP_RESIDENT, sampling.g_id) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "requesting resident hold for direct-release group %u "
		    "failed: errno %d (%s)", sampling.g_id, saved_errno,
		    strerror(saved_errno));
		goto cleanup;
	}
	if (pmc_start(sampling.g_ids[0]) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "starting %s direct-release sampling group failed: "
		    "errno %d (%s)", residency, saved_errno,
		    strerror(saved_errno));
		goto cleanup;
	}
	sampling.g_started = true;
	if (wait_test_sysctl_ack(TEST_HOLD_GROUP_RESIDENT_ACK,
	    sampling.g_id) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "initial resident hold for direct-release group %u was not "
		    "acknowledged: errno %d (%s)", sampling.g_id, saved_errno,
		    strerror(saved_errno));
		goto cleanup;
	}

	if (evicted) {
		if (group_build_n(&competitor, cap, false, true,
		    &build_failure) != 0) {
			format_group_build_failure(failure, sizeof(failure),
			    "building full direct-release competitor",
			    &build_failure);
			goto cleanup;
		}
		if (pmc_start(competitor.g_ids[0]) != 0) {
			saved_errno = errno;
			snprintf(failure, sizeof(failure),
			    "starting deferred direct-release competitor failed: "
			    "errno %d (%s)", saved_errno,
			    strerror(saved_errno));
			goto cleanup;
		}
		competitor.g_started = true;
		if (read_group_state(competitor.g_ids[0],
		    &competitor_state) != 0) {
			saved_errno = errno;
			snprintf(failure, sizeof(failure),
			    "reading deferred direct-release competitor failed: "
			    "errno %d (%s)", saved_errno,
			    strerror(saved_errno));
			goto cleanup;
		}
		if (competitor_state.ptgs_running != 1 ||
		    competitor_state.ptgs_assigned != 0 ||
		    competitor_state.ptgs_sys_listed != 1 ||
		    competitor_state.ptgs_sscounted != 0 ||
		    competitor_state.ptgs_nevents != cap) {
			snprintf(failure, sizeof(failure),
			    "direct-release competitor did not start deferred: "
			    "running=%u assigned=%u listed=%u sscounted=%u "
			    "events=%u/%u", competitor_state.ptgs_running,
			    competitor_state.ptgs_assigned,
			    competitor_state.ptgs_sys_listed,
			    competitor_state.ptgs_sscounted,
			    competitor_state.ptgs_nevents, cap);
			goto cleanup;
		}
		if (sysctl_write_u32(TEST_HOLD_GROUP_EVICTED,
		    sampling.g_id) != 0) {
			saved_errno = errno;
			snprintf(failure, sizeof(failure),
			    "requesting evicted hold for direct-release group %u "
			    "failed: errno %d (%s)", sampling.g_id, saved_errno,
			    strerror(saved_errno));
			goto cleanup;
		}
		if (sysctl_write_u32(TEST_HOLD_GROUP_RESIDENT,
		    competitor.g_id) != 0) {
			saved_errno = errno;
			snprintf(failure, sizeof(failure),
			    "requesting resident hold for direct-release "
			    "competitor %u failed: errno %d (%s)",
			    competitor.g_id, saved_errno,
			    strerror(saved_errno));
			goto cleanup;
		}
		if (wait_test_sysctl_ack(TEST_HOLD_GROUP_EVICTED_ACK,
		    sampling.g_id) != 0) {
			saved_errno = errno;
			snprintf(failure, sizeof(failure),
			    "direct-release group %u was not held evicted: "
			    "errno %d (%s)", sampling.g_id, saved_errno,
			    strerror(saved_errno));
			goto cleanup;
		}
		if (wait_test_sysctl_ack(TEST_HOLD_GROUP_RESIDENT_ACK,
		    competitor.g_id) != 0) {
			saved_errno = errno;
			snprintf(failure, sizeof(failure),
			    "direct-release competitor %u did not become "
			    "resident: errno %d (%s)", competitor.g_id,
			    saved_errno, strerror(saved_errno));
			goto cleanup;
		}
	}

	if (read_group_state(sampling.g_ids[0], &before_release) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "reading held-%s direct-release state failed: errno %d "
		    "(%s)", residency, saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	expected_running = cap;
	expected_stopped = 0;
	if (before_release.ptgs_running != 1 ||
	    before_release.ptgs_assigned != (evicted ? 0U : 1U) ||
	    before_release.ptgs_sys_listed != 1 ||
	    before_release.ptgs_sscounted != 1 ||
	    before_release.ptgs_nevents != cap ||
	    before_release.ptgs_running_members != expected_running ||
	    before_release.ptgs_stopped_members != expected_stopped) {
		snprintf(failure, sizeof(failure),
		    "direct-release group was not held %s: running=%u "
		    "assigned=%u listed=%u sscounted=%u events=%u/%u "
		    "members(running=%u/%u stopped=%u/%u allocated=%u)",
		    residency, before_release.ptgs_running,
		    before_release.ptgs_assigned,
		    before_release.ptgs_sys_listed,
		    before_release.ptgs_sscounted,
		    before_release.ptgs_nevents, cap,
		    before_release.ptgs_running_members, expected_running,
		    before_release.ptgs_stopped_members, expected_stopped,
		    before_release.ptgs_allocated_members);
		goto cleanup;
	}

	owner_pid = getpid();
	if (fork_sentinel(&before_pid) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "forking the held-%s pre-release sentinel failed: "
		    "errno %d (%s)", residency, saved_errno,
		    strerror(saved_errno));
		goto cleanup;
	}
	errno = 0;
	error = pmc_release(sampling.g_ids[0]);
	saved_errno = errno;
	if (error != 0) {
		snprintf(failure, sizeof(failure),
		    "pmc_release on held-%s running group returned %d, "
		    "errno %d (%s)", residency, error, saved_errno,
		    strerror(saved_errno));
		goto cleanup;
	}
	sampling.g_started = false;
	reset_system_stop_test_hooks();

	if (fork_sentinel(&after_pid) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "forking the held-%s post-release sentinel failed: "
		    "errno %d (%s)", residency, saved_errno,
		    strerror(saved_errno));
		goto cleanup;
	}
	if (pmc_writelog(TEST_OWNER_LOG_MARKER) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "writing held-%s release marker failed: errno %d (%s)",
		    residency, saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	if (pmc_flush_logfile() != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "flushing held-%s release marker failed: errno %d (%s)",
		    residency, saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	if (scan_owner_fork_log(log_path, TEST_OWNER_LOG_MARKER, owner_pid,
	    before_pid, after_pid, &fork_counts) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "parsing held-%s release logfile failed: errno %d (%s)",
		    residency, saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	if (!fork_counts.oflc_marker_seen || fork_counts.oflc_before != 1 ||
	    fork_counts.oflc_after != 0) {
		snprintf(failure, sizeof(failure),
		    "held-%s release log mismatch: marker=%u "
		    "pre-release-fork=%u expected=1 post-release-fork=%u "
		    "expected=0 owner=%jd pre-child=%jd post-child=%jd",
		    residency, fork_counts.oflc_marker_seen ? 1 : 0,
		    fork_counts.oflc_before, fork_counts.oflc_after,
		    (intmax_t)owner_pid, (intmax_t)before_pid,
		    (intmax_t)after_pid);
		goto cleanup;
	}

	if (pmc_configure_logfile(-1) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "closing held-%s release logfile failed: errno %d (%s)",
		    residency, saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	logfile_configured = false;
	if (close(log_fd) != 0) {
		saved_errno = errno;
		log_fd = -1;
		snprintf(failure, sizeof(failure),
		    "closing held-%s release descriptor failed: errno %d (%s)",
		    residency, saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	log_fd = -1;
	if (unlink(log_path) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "unlinking held-%s release logfile failed: errno %d (%s)",
		    residency, saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	log_path[0] = '\0';

	if (competitor.g_started) {
		if (pmc_stop(competitor.g_ids[0]) != 0) {
			saved_errno = errno;
			snprintf(failure, sizeof(failure),
			    "stopping held-%s release competitor failed: "
			    "errno %d (%s)", residency, saved_errno,
			    strerror(saved_errno));
			goto cleanup;
		}
		competitor.g_started = false;
	}
	group_teardown(&competitor);

	if (group_build_n(&fresh, cap, false, true, &build_failure) != 0) {
		format_group_build_failure(failure, sizeof(failure),
		    "building fresh same-CPU group after direct release",
		    &build_failure);
		goto cleanup;
	}
	if (sysctl_write_u32(TEST_HOLD_GROUP_RESIDENT, fresh.g_id) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "requesting resident hold for fresh group %u failed: "
		    "errno %d (%s)", fresh.g_id, saved_errno,
		    strerror(saved_errno));
		goto cleanup;
	}
	if (pmc_start(fresh.g_ids[0]) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "starting fresh same-CPU group after held-%s release "
		    "failed: errno %d (%s)", residency, saved_errno,
		    strerror(saved_errno));
		goto cleanup;
	}
	fresh.g_started = true;
	if (wait_test_sysctl_ack(TEST_HOLD_GROUP_RESIDENT_ACK,
	    fresh.g_id) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "fresh group %u after held-%s release did not become "
		    "resident: errno %d (%s)", fresh.g_id, residency,
		    saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	if (read_group_state(fresh.g_ids[0], &fresh_state) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "reading fresh group after held-%s release failed: "
		    "errno %d (%s)", residency, saved_errno,
		    strerror(saved_errno));
		goto cleanup;
	}
	if (fresh_state.ptgs_running != 1 ||
	    fresh_state.ptgs_assigned != 1 ||
	    fresh_state.ptgs_sscounted != 0 ||
	    fresh_state.ptgs_nevents != cap ||
	    fresh_state.ptgs_running_members != cap) {
		snprintf(failure, sizeof(failure),
		    "fresh group after held-%s release was not resident: "
		    "running=%u assigned=%u listed=%u sscounted=%u "
		    "events=%u/%u members(running=%u stopped=%u allocated=%u)",
		    residency, fresh_state.ptgs_running,
		    fresh_state.ptgs_assigned, fresh_state.ptgs_sys_listed,
		    fresh_state.ptgs_sscounted, fresh_state.ptgs_nevents, cap,
		    fresh_state.ptgs_running_members,
		    fresh_state.ptgs_stopped_members,
		    fresh_state.ptgs_allocated_members);
		goto cleanup;
	}

cleanup:
	reset_system_stop_test_hooks();
	group_teardown(&fresh);
	group_teardown(&competitor);
	group_teardown(&sampling);
	if (logfile_configured && pmc_configure_logfile(-1) != 0 &&
	    failure[0] == '\0') {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "deconfiguring held-%s release logfile failed: errno %d "
		    "(%s)", residency, saved_errno, strerror(saved_errno));
	}
	if (log_fd >= 0 && close(log_fd) != 0 && failure[0] == '\0') {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "closing held-%s release logfile failed: errno %d (%s)",
		    residency, saved_errno, strerror(saved_errno));
	}
	if (log_path[0] != '\0' && unlink(log_path) != 0 &&
	    failure[0] == '\0') {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "unlinking held-%s release logfile failed: errno %d (%s)",
		    residency, saved_errno, strerror(saved_errno));
	}
	if (wait_live_target_counts(&baseline, &final_counts,
	    &failed_counter) != 0) {
		saved_errno = errno;
		if (failed_counter != NULL) {
			atf_tc_fail("%s; reading %s during cleanup failed: errno "
			    "%d (%s)", failure[0] != '\0' ? failure :
			    "direct-release assertion failed", failed_counter,
			    saved_errno, strerror(saved_errno));
		}
		atf_tc_fail("%s; cleanup counters did not return to baseline: "
		    "errno %d (%s), pmc_target=%ju/%ju group_target=%ju/%ju "
		    "process=%ju/%ju residual=%ju/%ju rotation=%ju/%ju "
		    "run=%ju/%ju", failure[0] != '\0' ? failure :
		    "direct-release assertion failed", saved_errno,
		    strerror(saved_errno),
		    (uintmax_t)final_counts.ltc_pmc_targets,
		    (uintmax_t)baseline.ltc_pmc_targets,
		    (uintmax_t)final_counts.ltc_group_targets,
		    (uintmax_t)baseline.ltc_group_targets,
		    (uintmax_t)final_counts.ltc_target_processes,
		    (uintmax_t)baseline.ltc_target_processes,
		    (uintmax_t)final_counts.ltc_residual_entries,
		    (uintmax_t)baseline.ltc_residual_entries,
		    (uintmax_t)final_counts.ltc_rotation_refs,
		    (uintmax_t)baseline.ltc_rotation_refs,
		    (uintmax_t)final_counts.ltc_run_refs,
		    (uintmax_t)baseline.ltc_run_refs);
	}
	if (failure[0] != '\0')
		atf_tc_fail("%s", failure);
}

ATF_TC_WITHOUT_HEAD(system_release_while_running_resident);
ATF_TC_BODY(system_release_while_running_resident, tc)
{

	run_system_release_while_running(false);
}

ATF_TC_WITHOUT_HEAD(system_release_while_running_evicted);
ATF_TC_BODY(system_release_while_running_evicted, tc)
{

	run_system_release_while_running(true);
}

/*
 * Grouped system sampling has the same no-log contract as a standalone
 * PMC_MODE_SS allocation.  The failed start must not consume the group: after
 * configuring a logfile, the same handles must start, stop, and release.
 */
ATF_TC_WITHOUT_HEAD(grouped_system_sampling_requires_log);
ATF_TC_BODY(grouped_system_sampling_requires_log, tc)
{
	struct group g;
	FILE *log;
	int error;

	require_hwpmc();
	pin_to_test_cpu();
	group_build_required(&g);

	errno = 0;
	error = pmc_start(g.g_ids[0]);
	ATF_REQUIRE_MSG(error == -1,
	    "pmc_start without a logfile unexpectedly succeeded");
	ATF_REQUIRE_MSG(errno == EDOOFUS,
	    "pmc_start without a logfile failed with errno %d (%s), expected "
	    "EDOOFUS", errno, strerror(errno));

	log = tmpfile();
	ATF_REQUIRE_MSG(log != NULL, "tmpfile failed: %s", strerror(errno));
	ATF_REQUIRE_MSG(pmc_configure_logfile(fileno(log)) == 0,
	    "logfile configuration after failed start failed: %s",
	    strerror(errno));
	ATF_REQUIRE_MSG(pmc_start(g.g_ids[0]) == 0,
	    "the same group was not reusable after no-log failure: %s",
	    strerror(errno));
	g.g_started = true;
	ATF_REQUIRE_MSG(pmc_stop(g.g_ids[0]) == 0,
	    "stop after retry failed: %s", strerror(errno));
	g.g_started = false;
	group_teardown(&g);
	ATF_REQUIRE_MSG(pmc_configure_logfile(-1) == 0,
	    "logfile close failed: %s", strerror(errno));
	ATF_REQUIRE_MSG(fclose(log) == 0, "fclose failed: %s", strerror(errno));
}

/*
 * System-sampling accounting and initial kernel mappings must be published
 * before the first group member starts.  A later-member failure must restore
 * the committed group to its reusable pre-start state.
 */
ATF_TC_WITHOUT_HEAD(grouped_system_sampling_start_rollback_after_preflight);
ATF_TC_BODY(grouped_system_sampling_start_rollback_after_preflight, tc)
{
	struct group g;
	struct group_build_failure build_failure;
	struct live_target_counts baseline, final_counts;
	struct pmc_test_group_state paused_state, rollback_state;
	struct pmc_test_sample_counts completed_samples;
	struct start_thread_result start_result;
	pthread_t start_thread;
	const char *failed_counter, *reason;
	char failure[768], red_failure[256], log_path[PATH_MAX];
	u_int kernel_maps, started_members;
	int error, log_fd, saved_errno;
	bool logfile_configured, result_initialized;
	bool kernel_sample_seen, mapping_before_kernel_sample, start_done;
	bool thread_created, thread_joined;

	group_init(&g);
	memset(&baseline, 0, sizeof(baseline));
	memset(&final_counts, 0, sizeof(final_counts));
	memset(&start_result, 0, sizeof(start_result));
	failure[0] = '\0';
	red_failure[0] = '\0';
	log_path[0] = '\0';
	log_fd = -1;
	logfile_configured = false;
	result_initialized = false;
	thread_created = false;
	thread_joined = false;

	require_hwpmc();
	require_hwpmc_test_support();
	pin_to_test_cpu_required();
	reset_system_start_test_hooks();
	read_live_target_counts(&baseline);

	log_fd = open_named_logfile(log_path, sizeof(log_path));
	if (log_fd < 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "creating named logfile failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	if (pmc_configure_logfile(log_fd) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "configuring named logfile failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	logfile_configured = true;

	errno = 0;
	if (group_build_n(&g, 2, true, false, &build_failure) != 0) {
		format_group_build_failure(failure, sizeof(failure),
		    "building two-member sampling group", &build_failure);
		goto cleanup;
	}
	if (sysctl_write_u32(TEST_PAUSE_SYSTEM_START_AFTER_FIRST,
	    g.g_id) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "requesting first-member start pause for group %u failed: "
		    "errno %d (%s)", g.g_id, saved_errno,
		    strerror(saved_errno));
		goto cleanup;
	}
	if (sysctl_write_u32(TEST_FAIL_SYSTEM_START_AFTER, 1) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "arming later-member start failure failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	error = start_thread_result_init(&start_result, g.g_ids[0]);
	if (error != 0) {
		snprintf(failure, sizeof(failure),
		    "pthread_mutex_init failed: error %d (%s)", error,
		    strerror(error));
		goto cleanup;
	}
	result_initialized = true;
	error = pthread_create(&start_thread, NULL, start_group_thread,
	    &start_result);
	if (error != 0) {
		snprintf(failure, sizeof(failure),
		    "pthread_create for pmc_start failed: error %d (%s)",
		    error, strerror(error));
		goto cleanup;
	}
	thread_created = true;

	if (wait_test_sysctl_ack_or_start_done(
	    TEST_PAUSE_SYSTEM_START_AFTER_FIRST_ACK, g.g_id, &start_result,
	    &start_done) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "first-member start pause for group %u was not "
		    "acknowledged: errno %d (%s)", g.g_id, saved_errno,
		    strerror(saved_errno));
		goto cleanup;
	}
	if (start_done) {
		if (!start_result.str_start_called) {
			snprintf(failure, sizeof(failure),
			    "pinning pmc_start thread to CPU %d failed: "
			    "errno %d (%s)", TEST_CPU, start_result.str_errno,
			    strerror(start_result.str_errno));
		} else {
			snprintf(failure, sizeof(failure),
			    "pmc_start returned before the first-member pause: "
			    "rc=%d errno=%d (%s)", start_result.str_rc,
			    start_result.str_errno,
			    strerror(start_result.str_errno));
		}
		goto cleanup;
	}
	if (sysctl_read_u32(TEST_PAUSE_SYSTEM_START_MEMBER_COUNT,
	    &started_members) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "reading paused member count failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	if (read_group_state(g.g_ids[0], &paused_state) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "reading first-member pause state failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	if (started_members != 1 || paused_state.ptgs_running != 1 ||
	    paused_state.ptgs_assigned != 1 ||
	    paused_state.ptgs_nevents != 2 ||
	    paused_state.ptgs_running_members != 1 ||
	    paused_state.ptgs_allocated_members != 1) {
		snprintf(failure, sizeof(failure),
		    "start pause did not isolate the intended path: "
		    "started=%u running=%u assigned=%u events=%u "
		    "members(running=%u stopped=%u allocated=%u)",
		    started_members, paused_state.ptgs_running,
		    paused_state.ptgs_assigned, paused_state.ptgs_nevents,
		    paused_state.ptgs_running_members,
		    paused_state.ptgs_stopped_members,
		    paused_state.ptgs_allocated_members);
		goto cleanup;
	}

	if (sysctl_write_u64(TEST_INJECT_KERNEL_SAMPLE, g.g_ids[0]) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "injecting deterministic kernel sample for group %u "
		    "failed: errno %d (%s)", g.g_id, saved_errno,
		    strerror(saved_errno));
		goto cleanup;
	}

	if (pmc_writelog(TEST_LOG_MARKER) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "writing logfile ordering marker failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	if (pmc_flush_logfile() != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "flushing logfile ordering marker failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	if (wait_log_marker(log_path, TEST_LOG_MARKER, g.g_ids[0],
	    &kernel_maps, &kernel_sample_seen,
	    &mapping_before_kernel_sample) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "logfile ordering marker was not readable: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	if (!kernel_sample_seen) {
		snprintf(failure, sizeof(failure),
		    "injected leader kernel sample was not present before the "
		    "logfile ordering marker");
		goto cleanup;
	}
	if (kernel_maps == 0 || !mapping_before_kernel_sample ||
	    paused_state.ptgs_sscounted != 1) {
		snprintf(red_failure, sizeof(red_failure),
		    "first hardware row started before preflight: "
		    "kernel_maps=%u sscounted=%u "
		    "mapping_before_kernel_sample=%u",
		    kernel_maps, paused_state.ptgs_sscounted,
		    mapping_before_kernel_sample ? 1 : 0);
	}

	if (sysctl_write_u32(TEST_PAUSE_SYSTEM_START_AFTER_FIRST, 0) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "releasing first-member start pause failed: "
		    "errno %d (%s)", saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	error = pthread_join(start_thread, NULL);
	if (error != 0) {
		snprintf(failure, sizeof(failure),
		    "pthread_join for pmc_start failed: error %d (%s)",
		    error, strerror(error));
		goto cleanup;
	}
	thread_joined = true;
	if (start_result.str_rc != -1 || start_result.str_errno != EIO) {
		if (start_result.str_rc == 0)
			g.g_started = true;
		snprintf(failure, sizeof(failure),
		    "later-member start failure returned rc=%d errno=%d "
		    "(%s), expected -1/EIO", start_result.str_rc,
		    start_result.str_errno, strerror(start_result.str_errno));
		goto cleanup;
	}
	if (wait_sample_reconciled(g.g_ids[0], &completed_samples) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "preflight test samples did not reconcile after rollback: "
		    "errno %d (%s)", saved_errno, strerror(saved_errno));
		goto cleanup;
	}

	if (read_group_state(g.g_ids[0], &rollback_state) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "reading post-failure group state failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	if (rollback_state.ptgs_running != 0 ||
	    rollback_state.ptgs_assigned != 0 ||
	    rollback_state.ptgs_sys_listed != 0 ||
	    rollback_state.ptgs_sscounted != 0 ||
	    rollback_state.ptgs_nevents != 2 ||
	    rollback_state.ptgs_running_members != 0 ||
	    rollback_state.ptgs_stopped_members != 0 ||
	    rollback_state.ptgs_allocated_members != 2) {
		if (red_failure[0] == '\0') {
			snprintf(red_failure, sizeof(red_failure),
			    "later-member failure did not restore reusable state: "
			    "running=%u assigned=%u listed=%u sscounted=%u "
			    "events=%u members(running=%u stopped=%u "
			    "allocated=%u)", rollback_state.ptgs_running,
			    rollback_state.ptgs_assigned,
			    rollback_state.ptgs_sys_listed,
			    rollback_state.ptgs_sscounted,
			    rollback_state.ptgs_nevents,
			    rollback_state.ptgs_running_members,
			    rollback_state.ptgs_stopped_members,
			    rollback_state.ptgs_allocated_members);
		}
	}

	if (sysctl_write_u32(TEST_FAIL_SYSTEM_START_AFTER, UINT_MAX) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "disarming later-member failure failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	errno = 0;
	if (pmc_start(g.g_ids[0]) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "same-group retry after rollback failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	g.g_started = true;
	if (pmc_stop(g.g_ids[0]) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "stopping same-group retry failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	g.g_started = false;

cleanup:
	if (thread_created && !thread_joined) {
		(void)sysctl_write_u32(TEST_PAUSE_SYSTEM_START_AFTER_FIRST, 0);
		(void)pthread_join(start_thread, NULL);
		if (start_result.str_rc == 0)
			g.g_started = true;
	}
	reset_system_start_test_hooks();
	if (result_initialized)
		(void)pthread_mutex_destroy(&start_result.str_lock);
	group_teardown(&g);
	if (logfile_configured && pmc_configure_logfile(-1) != 0 &&
	    failure[0] == '\0') {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "deconfiguring named logfile failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
	}
	if (log_fd >= 0 && close(log_fd) != 0 && failure[0] == '\0') {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "closing named logfile failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
	}
	if (log_path[0] != '\0' && unlink(log_path) != 0 &&
	    failure[0] == '\0') {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "unlinking named logfile failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
	}
	reason = failure[0] != '\0' ? failure :
	    (red_failure[0] != '\0' ? red_failure : "successful test path");
	if (wait_live_target_counts(&baseline, &final_counts,
	    &failed_counter) != 0) {
		saved_errno = errno;
		if (failed_counter != NULL) {
			atf_tc_fail("%s; reading %s during cleanup failed: errno "
			    "%d (%s)", reason, failed_counter, saved_errno,
			    strerror(saved_errno));
		}
		atf_tc_fail("%s; cleanup counters did not return to baseline: "
		    "errno %d (%s), pmc_target=%ju/%ju group_target=%ju/%ju "
		    "process=%ju/%ju residual=%ju/%ju rotation=%ju/%ju "
		    "run=%ju/%ju", reason, saved_errno,
		    strerror(saved_errno),
		    (uintmax_t)final_counts.ltc_pmc_targets,
		    (uintmax_t)baseline.ltc_pmc_targets,
		    (uintmax_t)final_counts.ltc_group_targets,
		    (uintmax_t)baseline.ltc_group_targets,
		    (uintmax_t)final_counts.ltc_target_processes,
		    (uintmax_t)baseline.ltc_target_processes,
		    (uintmax_t)final_counts.ltc_residual_entries,
		    (uintmax_t)baseline.ltc_residual_entries,
		    (uintmax_t)final_counts.ltc_rotation_refs,
		    (uintmax_t)baseline.ltc_rotation_refs,
		    (uintmax_t)final_counts.ltc_run_refs,
		    (uintmax_t)baseline.ltc_run_refs);
	}
	if (failure[0] != '\0')
		atf_tc_fail("%s", failure);
	if (red_failure[0] != '\0')
		atf_tc_fail("%s", red_failure);
}

/*
 * A sample accepted by the first member before a later member fails must enter
 * the same drain protocol used for normal system-group eviction.  The sample
 * must then be emitted or explicitly counted as dropped before rollback
 * returns.
 */
ATF_TC_WITHOUT_HEAD(grouped_system_sampling_partial_start_rollback_drains);
ATF_TC_BODY(grouped_system_sampling_partial_start_rollback_drains, tc)
{
	struct group g;
	struct group_build_failure build_failure;
	struct live_target_counts baseline, final_counts;
	struct pmc_test_group_state paused_state, rollback_state;
	struct pmc_test_sample_counts paused_samples, final_samples;
	struct start_thread_result start_result;
	pthread_t start_thread;
	const char *failed_counter, *reason;
	FILE *log;
	char failure[768], red_failure[256];
	u_int started_members;
	int error, saved_errno;
	bool logfile_configured, result_initialized;
	bool start_done, thread_created, thread_joined;

	group_init(&g);
	memset(&baseline, 0, sizeof(baseline));
	memset(&final_counts, 0, sizeof(final_counts));
	memset(&start_result, 0, sizeof(start_result));
	failure[0] = '\0';
	red_failure[0] = '\0';
	log = NULL;
	logfile_configured = false;
	result_initialized = false;
	thread_created = false;
	thread_joined = false;

	require_hwpmc();
	require_hwpmc_test_support();
	pin_to_test_cpu_required();
	reset_system_start_test_hooks();
	read_live_target_counts(&baseline);

	log = tmpfile();
	if (log == NULL) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "tmpfile for partial-start drain failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	if (pmc_configure_logfile(fileno(log)) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "configuring partial-start logfile failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	logfile_configured = true;

	errno = 0;
	if (group_build_n(&g, 2, true, false, &build_failure) != 0) {
		format_group_build_failure(failure, sizeof(failure),
		    "building partial-start sampling group", &build_failure);
		goto cleanup;
	}
	if (sysctl_write_u32(TEST_PAUSE_SYSTEM_START_AFTER_FIRST,
	    g.g_id) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "requesting partial-start pause for group %u failed: "
		    "errno %d (%s)", g.g_id, saved_errno,
		    strerror(saved_errno));
		goto cleanup;
	}
	if (sysctl_write_u32(TEST_PAUSE_SAMPLE_WORKER, g.g_id) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "requesting sample-worker pause for group %u failed: "
		    "errno %d (%s)", g.g_id, saved_errno,
		    strerror(saved_errno));
		goto cleanup;
	}
	if (sysctl_write_u32(TEST_FAIL_SYSTEM_START_AFTER, 1) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "arming partial-start failure failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	error = start_thread_result_init(&start_result, g.g_ids[0]);
	if (error != 0) {
		snprintf(failure, sizeof(failure),
		    "pthread_mutex_init failed: error %d (%s)", error,
		    strerror(error));
		goto cleanup;
	}
	result_initialized = true;
	error = pthread_create(&start_thread, NULL, start_group_thread,
	    &start_result);
	if (error != 0) {
		snprintf(failure, sizeof(failure),
		    "pthread_create for partial-start pmc_start failed: "
		    "error %d (%s)", error, strerror(error));
		goto cleanup;
	}
	thread_created = true;

	if (wait_test_sysctl_ack_or_start_done(
	    TEST_PAUSE_SYSTEM_START_AFTER_FIRST_ACK, g.g_id, &start_result,
	    &start_done) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "partial-start pause for group %u was not acknowledged: "
		    "errno %d (%s)", g.g_id, saved_errno,
		    strerror(saved_errno));
		goto cleanup;
	}
	if (start_done) {
		if (!start_result.str_start_called) {
			snprintf(failure, sizeof(failure),
			    "pinning partial-start thread to CPU %d failed: "
			    "errno %d (%s)", TEST_CPU, start_result.str_errno,
			    strerror(start_result.str_errno));
		} else {
			snprintf(failure, sizeof(failure),
			    "partial pmc_start returned before the first-member "
			    "pause: rc=%d errno=%d (%s)", start_result.str_rc,
			    start_result.str_errno,
			    strerror(start_result.str_errno));
		}
		goto cleanup;
	}
	if (sysctl_read_u32(TEST_PAUSE_SYSTEM_START_MEMBER_COUNT,
	    &started_members) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "reading partial-start member count failed: errno %d "
		    "(%s)", saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	if (read_group_state(g.g_ids[0], &paused_state) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "reading partial-start group state failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	if (started_members != 1 || paused_state.ptgs_running != 1 ||
	    paused_state.ptgs_assigned != 1 ||
	    paused_state.ptgs_nevents != 2 ||
	    paused_state.ptgs_running_members != 1 ||
	    paused_state.ptgs_allocated_members != 1) {
		snprintf(failure, sizeof(failure),
		    "partial-start pause did not isolate one running member: "
		    "started=%u running=%u assigned=%u events=%u "
		    "members(running=%u stopped=%u allocated=%u)",
		    started_members, paused_state.ptgs_running,
		    paused_state.ptgs_assigned, paused_state.ptgs_nevents,
		    paused_state.ptgs_running_members,
		    paused_state.ptgs_stopped_members,
		    paused_state.ptgs_allocated_members);
		goto cleanup;
	}

	if (spin_until_test_ack(TEST_PAUSE_SAMPLE_WORKER_ACK,
	    g.g_id) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "no accepted sample reached the paused worker for group "
		    "%u: errno %d (%s)", g.g_id, saved_errno,
		    strerror(saved_errno));
		goto cleanup;
	}
	if (read_sample_counts(g.g_ids[0], &paused_samples) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "reading paused sample counts failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	if (paused_samples.ptsc_handle != g.g_ids[0] ||
	    paused_samples.ptsc_accepted != 1 ||
	    paused_samples.ptsc_emitted != 0 ||
	    paused_samples.ptsc_dropped != 0 ||
	    paused_samples.ptsc_run_refs != 1) {
		snprintf(failure, sizeof(failure),
		    "sample pause did not hold exactly one accepted entry: "
		    "handle=%ju/%ju accepted=%ju emitted=%ju dropped=%ju "
		    "run_refs=%ju", (uintmax_t)paused_samples.ptsc_handle,
		    (uintmax_t)g.g_ids[0],
		    (uintmax_t)paused_samples.ptsc_accepted,
		    (uintmax_t)paused_samples.ptsc_emitted,
		    (uintmax_t)paused_samples.ptsc_dropped,
		    (uintmax_t)paused_samples.ptsc_run_refs);
		goto cleanup;
	}

	if (sysctl_write_u32(TEST_PAUSE_SYSTEM_START_AFTER_FIRST, 0) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "releasing partial-start pause failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	if (wait_test_sysctl_ack_or_start_done(
	    TEST_PAUSE_SAMPLE_SCHEDULE_OUT_ACK, g.g_id, &start_result,
	    &start_done) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "waiting for partial-start drain boundary failed: "
		    "errno %d (%s)", saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	if (start_done) {
		snprintf(red_failure, sizeof(red_failure),
		    "later-member start failure returned before the queued "
		    "sample entered the system-group drain protocol");
	}

	if (sysctl_write_u32(TEST_PAUSE_SAMPLE_WORKER, 0) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "releasing sample worker failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	error = pthread_join(start_thread, NULL);
	if (error != 0) {
		snprintf(failure, sizeof(failure),
		    "pthread_join for partial-start pmc_start failed: "
		    "error %d (%s)", error, strerror(error));
		goto cleanup;
	}
	thread_joined = true;
	if (start_result.str_rc != -1 || start_result.str_errno != EIO) {
		if (start_result.str_rc == 0)
			g.g_started = true;
		snprintf(failure, sizeof(failure),
		    "partial-start failure returned rc=%d errno=%d "
		    "(%s), expected -1/EIO", start_result.str_rc,
		    start_result.str_errno, strerror(start_result.str_errno));
		goto cleanup;
	}

	if (wait_sample_reconciled(g.g_ids[0], &final_samples) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "accepted sample did not reconcile after rollback: "
		    "errno %d (%s)", saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	if (final_samples.ptsc_accepted != 1 ||
	    final_samples.ptsc_emitted + final_samples.ptsc_dropped != 1 ||
	    final_samples.ptsc_run_refs != 0) {
		if (red_failure[0] == '\0') {
			snprintf(red_failure, sizeof(red_failure),
			    "rollback did not reconcile the accepted sample: "
			    "accepted=%ju emitted=%ju dropped=%ju run_refs=%ju",
			    (uintmax_t)final_samples.ptsc_accepted,
			    (uintmax_t)final_samples.ptsc_emitted,
			    (uintmax_t)final_samples.ptsc_dropped,
			    (uintmax_t)final_samples.ptsc_run_refs);
		}
	}

	if (read_group_state(g.g_ids[0], &rollback_state) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "reading partial-start rollback state failed: "
		    "errno %d (%s)", saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	if (rollback_state.ptgs_running != 0 ||
	    rollback_state.ptgs_assigned != 0 ||
	    rollback_state.ptgs_sys_listed != 0 ||
	    rollback_state.ptgs_sscounted != 0 ||
	    rollback_state.ptgs_running_members != 0 ||
	    rollback_state.ptgs_stopped_members != 0 ||
	    rollback_state.ptgs_allocated_members != 2) {
		if (red_failure[0] == '\0') {
			snprintf(red_failure, sizeof(red_failure),
			    "partial-start rollback left non-reusable state: "
			    "running=%u assigned=%u listed=%u sscounted=%u "
			    "members(running=%u stopped=%u allocated=%u)",
			    rollback_state.ptgs_running,
			    rollback_state.ptgs_assigned,
			    rollback_state.ptgs_sys_listed,
			    rollback_state.ptgs_sscounted,
			    rollback_state.ptgs_running_members,
			    rollback_state.ptgs_stopped_members,
			    rollback_state.ptgs_allocated_members);
		}
	}

	if (sysctl_write_u32(TEST_FAIL_SYSTEM_START_AFTER, UINT_MAX) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "disarming partial-start failure failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	errno = 0;
	if (pmc_start(g.g_ids[0]) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "same group was not reusable after sample rollback: "
		    "errno %d (%s)", saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	g.g_started = true;
	if (pmc_stop(g.g_ids[0]) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "stopping sample-rollback retry failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	g.g_started = false;

cleanup:
	if (thread_created && !thread_joined) {
		(void)sysctl_write_u32(TEST_PAUSE_SAMPLE_WORKER, 0);
		(void)sysctl_write_u32(TEST_PAUSE_SYSTEM_START_AFTER_FIRST, 0);
		(void)pthread_join(start_thread, NULL);
		if (start_result.str_rc == 0)
			g.g_started = true;
	}
	reset_system_start_test_hooks();
	if (result_initialized)
		(void)pthread_mutex_destroy(&start_result.str_lock);
	group_teardown(&g);
	if (logfile_configured && pmc_configure_logfile(-1) != 0 &&
	    failure[0] == '\0') {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "deconfiguring partial-start logfile failed: errno %d "
		    "(%s)", saved_errno, strerror(saved_errno));
	}
	if (log != NULL && fclose(log) != 0 && failure[0] == '\0') {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "closing partial-start logfile failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
	}
	reason = failure[0] != '\0' ? failure :
	    (red_failure[0] != '\0' ? red_failure : "successful test path");
	if (wait_live_target_counts(&baseline, &final_counts,
	    &failed_counter) != 0) {
		saved_errno = errno;
		if (failed_counter != NULL) {
			atf_tc_fail("%s; reading %s during cleanup failed: errno "
			    "%d (%s)", reason, failed_counter, saved_errno,
			    strerror(saved_errno));
		}
		atf_tc_fail("%s; cleanup counters did not return to baseline: "
		    "errno %d (%s), pmc_target=%ju/%ju group_target=%ju/%ju "
		    "process=%ju/%ju residual=%ju/%ju rotation=%ju/%ju "
		    "run=%ju/%ju", reason, saved_errno,
		    strerror(saved_errno),
		    (uintmax_t)final_counts.ltc_pmc_targets,
		    (uintmax_t)baseline.ltc_pmc_targets,
		    (uintmax_t)final_counts.ltc_group_targets,
		    (uintmax_t)baseline.ltc_group_targets,
		    (uintmax_t)final_counts.ltc_target_processes,
		    (uintmax_t)baseline.ltc_target_processes,
		    (uintmax_t)final_counts.ltc_residual_entries,
		    (uintmax_t)baseline.ltc_residual_entries,
		    (uintmax_t)final_counts.ltc_rotation_refs,
		    (uintmax_t)baseline.ltc_rotation_refs,
		    (uintmax_t)final_counts.ltc_run_refs,
		    (uintmax_t)baseline.ltc_run_refs);
	}
	if (failure[0] != '\0')
		atf_tc_fail("%s", failure);
	if (red_failure[0] != '\0')
		atf_tc_fail("%s", red_failure);
}

/*
 * One sample accepted before a normal system-group eviction must be emitted
 * before the row is reused.  Holding the worker after queue publication makes
 * the accepted entry and the eviction boundary deterministic.
 */
ATF_TC_WITHOUT_HEAD(system_sampling_drain_one_queued_sample);
ATF_TC_BODY(system_sampling_drain_one_queued_sample, tc)
{
	struct group sampling, competitor;
	struct group_build_failure build_failure;
	struct live_target_counts baseline, final_counts;
	struct pmc_test_group_state sampling_state, competitor_state;
	struct pmc_test_sample_counts paused_samples, final_samples;
	const char *failed_counter, *reason;
	char failure[1024], red_failure[384], log_path[PATH_MAX];
	uint64_t callchains;
	u_int cap;
	int log_fd, saved_errno;
	bool logfile_configured;

	group_init(&sampling);
	group_init(&competitor);
	memset(&baseline, 0, sizeof(baseline));
	memset(&final_counts, 0, sizeof(final_counts));
	failure[0] = '\0';
	red_failure[0] = '\0';
	log_path[0] = '\0';
	log_fd = -1;
	logfile_configured = false;

	require_hwpmc();
	require_hwpmc_test_support();
	pin_to_test_cpu_required();
	reset_system_start_test_hooks();
	reset_system_stop_test_hooks();

	cap = probe_group_capacity();
	if (cap < 2) {
		atf_tc_fail("system-group capacity probe returned %u rows on CPU %d; "
		    "expected at least two", cap, TEST_CPU);
	}
	read_live_target_counts(&baseline);

	log_fd = open_named_logfile(log_path, sizeof(log_path));
	if (log_fd < 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "creating drain logfile failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	if (pmc_configure_logfile(log_fd) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "configuring drain logfile failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	logfile_configured = true;

	if (group_build_n(&sampling, cap, true, true, &build_failure) != 0) {
		format_group_build_failure(failure, sizeof(failure),
		    "building full drain sampling group", &build_failure);
		goto cleanup;
	}
	if (sysctl_write_u32(TEST_HOLD_GROUP_RESIDENT, sampling.g_id) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "requesting resident hold for drain group %u failed: "
		    "errno %d (%s)", sampling.g_id, saved_errno,
		    strerror(saved_errno));
		goto cleanup;
	}
	if (sysctl_write_u32(TEST_PAUSE_SAMPLE_WORKER, sampling.g_id) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "requesting sample-worker pause for drain group %u failed: "
		    "errno %d (%s)", sampling.g_id, saved_errno,
		    strerror(saved_errno));
		goto cleanup;
	}
	if (pmc_start(sampling.g_ids[0]) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "starting drain sampling group failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	sampling.g_started = true;
	if (wait_test_sysctl_ack(TEST_HOLD_GROUP_RESIDENT_ACK,
	    sampling.g_id) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "drain sampling group %u did not become resident: "
		    "errno %d (%s)", sampling.g_id, saved_errno,
		    strerror(saved_errno));
		goto cleanup;
	}
	if (spin_until_test_ack(TEST_PAUSE_SAMPLE_WORKER_ACK,
	    sampling.g_id) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "no accepted sample reached the paused worker for drain "
		    "group %u: errno %d (%s)", sampling.g_id, saved_errno,
		    strerror(saved_errno));
		goto cleanup;
	}
	if (read_sample_counts(sampling.g_ids[0], &paused_samples) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "reading paused drain sample counts failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	if (paused_samples.ptsc_handle != sampling.g_ids[0] ||
	    paused_samples.ptsc_accepted != 1 ||
	    paused_samples.ptsc_emitted != 0 ||
	    paused_samples.ptsc_dropped != 0 ||
	    paused_samples.ptsc_run_refs != 1) {
		snprintf(failure, sizeof(failure),
		    "drain pause did not hold exactly one accepted entry: "
		    "handle=%ju/%ju accepted=%ju emitted=%ju dropped=%ju "
		    "run_refs=%ju", (uintmax_t)paused_samples.ptsc_handle,
		    (uintmax_t)sampling.g_ids[0],
		    (uintmax_t)paused_samples.ptsc_accepted,
		    (uintmax_t)paused_samples.ptsc_emitted,
		    (uintmax_t)paused_samples.ptsc_dropped,
		    (uintmax_t)paused_samples.ptsc_run_refs);
		goto cleanup;
	}

	if (group_build_n(&competitor, cap, false, true,
	    &build_failure) != 0) {
		format_group_build_failure(failure, sizeof(failure),
		    "building full drain competitor", &build_failure);
		goto cleanup;
	}
	if (pmc_start(competitor.g_ids[0]) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "starting deferred drain competitor failed: "
		    "errno %d (%s)", saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	competitor.g_started = true;
	if (read_group_state(competitor.g_ids[0], &competitor_state) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "reading deferred drain competitor failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	if (competitor_state.ptgs_running != 1 ||
	    competitor_state.ptgs_assigned != 0 ||
	    competitor_state.ptgs_sys_listed != 1 ||
	    competitor_state.ptgs_sscounted != 0 ||
	    competitor_state.ptgs_nevents != cap) {
		snprintf(failure, sizeof(failure),
		    "drain competitor did not start deferred: running=%u "
		    "assigned=%u listed=%u sscounted=%u events=%u/%u",
		    competitor_state.ptgs_running,
		    competitor_state.ptgs_assigned,
		    competitor_state.ptgs_sys_listed,
		    competitor_state.ptgs_sscounted,
		    competitor_state.ptgs_nevents, cap);
		goto cleanup;
	}

	if (sysctl_write_u32(TEST_HOLD_GROUP_EVICTED, sampling.g_id) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "requesting evicted hold for drain group %u failed: "
		    "errno %d (%s)", sampling.g_id, saved_errno,
		    strerror(saved_errno));
		goto cleanup;
	}
	if (sysctl_write_u32(TEST_HOLD_GROUP_RESIDENT, competitor.g_id) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "requesting resident hold for drain competitor %u failed: "
		    "errno %d (%s)", competitor.g_id, saved_errno,
		    strerror(saved_errno));
		goto cleanup;
	}
	if (wait_test_sysctl_ack(TEST_PAUSE_SAMPLE_SCHEDULE_OUT_ACK,
	    sampling.g_id) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "drain group %u did not reach the schedule-out boundary: "
		    "errno %d (%s)", sampling.g_id, saved_errno,
		    strerror(saved_errno));
		goto cleanup;
	}
	if (sysctl_write_u32(TEST_PAUSE_SAMPLE_WORKER, 0) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "releasing drain sample worker failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	if (wait_test_sysctl_ack(TEST_HOLD_GROUP_EVICTED_ACK,
	    sampling.g_id) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "drain sampling group %u was not held evicted: "
		    "errno %d (%s)", sampling.g_id, saved_errno,
		    strerror(saved_errno));
		goto cleanup;
	}
	if (wait_test_sysctl_ack(TEST_HOLD_GROUP_RESIDENT_ACK,
	    competitor.g_id) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "drain competitor %u did not become resident: "
		    "errno %d (%s)", competitor.g_id, saved_errno,
		    strerror(saved_errno));
		goto cleanup;
	}
	if (read_group_state(sampling.g_ids[0], &sampling_state) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "reading evicted drain group state failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	if (read_group_state(competitor.g_ids[0], &competitor_state) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "reading resident drain competitor state failed: "
		    "errno %d (%s)", saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	if (sampling_state.ptgs_running != 1 ||
	    sampling_state.ptgs_assigned != 0 ||
	    sampling_state.ptgs_sscounted != 1 ||
	    competitor_state.ptgs_running != 1 ||
	    competitor_state.ptgs_assigned != 1) {
		snprintf(failure, sizeof(failure),
		    "drain eviction state mismatch: sampling(running=%u "
		    "assigned=%u sscounted=%u) competitor(running=%u "
		    "assigned=%u)",
		    sampling_state.ptgs_running,
		    sampling_state.ptgs_assigned,
		    sampling_state.ptgs_sscounted,
		    competitor_state.ptgs_running,
		    competitor_state.ptgs_assigned);
		goto cleanup;
	}

	if (wait_sample_reconciled(sampling.g_ids[0], &final_samples) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "queued drain sample did not reconcile after eviction: "
		    "errno %d (%s)", saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	if (final_samples.ptsc_handle != sampling.g_ids[0] ||
	    final_samples.ptsc_accepted != 1 ||
	    final_samples.ptsc_accepted !=
	    final_samples.ptsc_emitted + final_samples.ptsc_dropped ||
	    final_samples.ptsc_run_refs != 0) {
		snprintf(failure, sizeof(failure),
		    "drain sample accounting was not internally consistent: "
		    "handle=%ju/%ju accepted=%ju emitted=%ju dropped=%ju "
		    "run_refs=%ju", (uintmax_t)final_samples.ptsc_handle,
		    (uintmax_t)sampling.g_ids[0],
		    (uintmax_t)final_samples.ptsc_accepted,
		    (uintmax_t)final_samples.ptsc_emitted,
		    (uintmax_t)final_samples.ptsc_dropped,
		    (uintmax_t)final_samples.ptsc_run_refs);
		goto cleanup;
	}

	if (pmc_writelog(TEST_LOG_DRAIN_MARKER) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "writing drain log marker failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	if (pmc_flush_logfile() != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "flushing drain log marker failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	if (wait_log_callchain_marker_exact(log_path, TEST_LOG_DRAIN_MARKER,
	    sampling.g_ids[0], paused_samples.ptsc_pid,
	    paused_samples.ptsc_tid, &callchains) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		   "drain log marker was not readable: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	if (final_samples.ptsc_emitted != 1 ||
	    final_samples.ptsc_dropped != 0 || callchains != 1) {
		snprintf(red_failure, sizeof(red_failure),
		    "queued system sample was dropped during eviction: "
		    "pid=%u tid=%u accepted=%ju emitted=%ju dropped=%ju "
		    "run_refs=%ju logged_callchains=%ju",
		    paused_samples.ptsc_pid, paused_samples.ptsc_tid,
		    (uintmax_t)final_samples.ptsc_accepted,
		    (uintmax_t)final_samples.ptsc_emitted,
		    (uintmax_t)final_samples.ptsc_dropped,
		    (uintmax_t)final_samples.ptsc_run_refs,
		    (uintmax_t)callchains);
	}

	if (pmc_stop(sampling.g_ids[0]) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "stopping evicted drain sampling group failed: "
		    "errno %d (%s)", saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	sampling.g_started = false;
	if (pmc_stop(competitor.g_ids[0]) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "stopping resident drain competitor failed: "
		    "errno %d (%s)", saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	competitor.g_started = false;

cleanup:
	reset_system_start_test_hooks();
	reset_system_stop_test_hooks();
	group_teardown(&competitor);
	group_teardown(&sampling);
	if (logfile_configured && pmc_configure_logfile(-1) != 0 &&
	    failure[0] == '\0') {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "deconfiguring drain logfile failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
	}
	if (log_fd >= 0 && close(log_fd) != 0 && failure[0] == '\0') {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "closing drain logfile failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
	}
	if (log_path[0] != '\0' && unlink(log_path) != 0 &&
	    failure[0] == '\0') {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "unlinking drain logfile failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
	}
	reason = failure[0] != '\0' ? failure :
	    (red_failure[0] != '\0' ? red_failure : "successful test path");
	if (wait_live_target_counts(&baseline, &final_counts,
	    &failed_counter) != 0) {
		saved_errno = errno;
		if (failed_counter != NULL) {
			atf_tc_fail("%s; reading %s during cleanup failed: "
			    "errno %d (%s)", reason, failed_counter,
			    saved_errno, strerror(saved_errno));
		}
		atf_tc_fail("%s; cleanup counters did not return to baseline: "
		    "errno %d (%s), pmc_target=%ju/%ju "
		    "group_target=%ju/%ju process=%ju/%ju "
		    "residual=%ju/%ju rotation=%ju/%ju run=%ju/%ju",
		    reason, saved_errno, strerror(saved_errno),
		    (uintmax_t)final_counts.ltc_pmc_targets,
		    (uintmax_t)baseline.ltc_pmc_targets,
		    (uintmax_t)final_counts.ltc_group_targets,
		    (uintmax_t)baseline.ltc_group_targets,
		    (uintmax_t)final_counts.ltc_target_processes,
		    (uintmax_t)baseline.ltc_target_processes,
		    (uintmax_t)final_counts.ltc_residual_entries,
		    (uintmax_t)baseline.ltc_residual_entries,
		    (uintmax_t)final_counts.ltc_rotation_refs,
		    (uintmax_t)baseline.ltc_rotation_refs,
		    (uintmax_t)final_counts.ltc_run_refs,
		    (uintmax_t)baseline.ltc_run_refs);
	}
	if (failure[0] != '\0')
		atf_tc_fail("%s", failure);
	if (red_failure[0] != '\0')
		atf_tc_fail("%s", red_failure);
}

/*
 * Supplemental low-rate, many-rotation drain stress (F-06).  The deterministic
 * single-sample case proves one queued sample survives one forced eviction;
 * this case proves the per-handle accounting stays lossless across many natural
 * rotations under load.  Two full-size MUX groups oversubscribe the PMU on one
 * CPU so neither is resident continuously, a bounded workload drives samples
 * over many short rotation windows, and the sampling handle must reconcile with
 * accepted == emitted + dropped and no leaked run reference.
 */
ATF_TC_WITHOUT_HEAD(system_sampling_drain_stress);
ATF_TC_BODY(system_sampling_drain_stress, tc)
{
	struct group sampling, competitor;
	struct group_build_failure build_failure;
	struct live_target_counts baseline, final_counts;
	struct pmc_test_sample_counts final_samples;
	const char *failed_counter, *reason;
	char failure[1024], red_failure[512], log_path[PATH_MAX];
	int mux_saved, mux_stress, log_fd, saved_errno, rounds;
	u_int cap;
	bool logfile_configured, mux_changed;
	size_t mux_len;

	group_init(&sampling);
	group_init(&competitor);
	memset(&baseline, 0, sizeof(baseline));
	memset(&final_counts, 0, sizeof(final_counts));
	failure[0] = '\0';
	red_failure[0] = '\0';
	log_path[0] = '\0';
	log_fd = -1;
	logfile_configured = false;
	mux_changed = false;
	mux_saved = 0;
	mux_stress = 5;

	require_hwpmc();
	require_hwpmc_test_support();
	pin_to_test_cpu_required();
	reset_system_start_test_hooks();
	reset_system_stop_test_hooks();

	cap = probe_group_capacity();
	if (cap < 2) {
		atf_tc_fail("system-group capacity probe returned %u rows on CPU "
		    "%d; expected at least two", cap, TEST_CPU);
	}
	read_live_target_counts(&baseline);

	/*
	 * Shorten the rotation window so a bounded workload crosses many
	 * eviction boundaries.  Restore the original value during cleanup.
	 */
	mux_len = sizeof(mux_saved);
	if (sysctlbyname("kern.hwpmc.mux_period_ms", &mux_saved, &mux_len,
	    &mux_stress, sizeof(mux_stress)) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "lowering kern.hwpmc.mux_period_ms failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	mux_changed = true;

	log_fd = open_named_logfile(log_path, sizeof(log_path));
	if (log_fd < 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "creating drain-stress logfile failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	if (pmc_configure_logfile(log_fd) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "configuring drain-stress logfile failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	logfile_configured = true;

	if (group_build_n(&sampling, cap, true, true, &build_failure) != 0) {
		format_group_build_failure(failure, sizeof(failure),
		    "building drain-stress sampling group", &build_failure);
		goto cleanup;
	}
	if (group_build_n(&competitor, cap, true, true, &build_failure) != 0) {
		format_group_build_failure(failure, sizeof(failure),
		    "building drain-stress competitor group", &build_failure);
		goto cleanup;
	}
	if (pmc_start(sampling.g_ids[0]) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "starting drain-stress sampling group failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	sampling.g_started = true;
	if (pmc_start(competitor.g_ids[0]) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "starting drain-stress competitor group failed: errno %d "
		    "(%s)", saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	competitor.g_started = true;

	/* Drive samples across many rotation windows. */
	for (rounds = 0; rounds < 8; rounds++)
		spin();

	if (pmc_stop(sampling.g_ids[0]) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "stopping drain-stress sampling group failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	sampling.g_started = false;
	if (pmc_stop(competitor.g_ids[0]) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "stopping drain-stress competitor group failed: errno %d "
		    "(%s)", saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	competitor.g_started = false;

	if (wait_sample_reconciled(sampling.g_ids[0], &final_samples) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "drain-stress samples did not reconcile: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
		goto cleanup;
	}

	/*
	 * The lossless invariant: every accepted sample is either emitted to the
	 * log or explicitly dropped, and no run reference is leaked across the
	 * many evictions this workload forced.
	 */
	if (final_samples.ptsc_handle != sampling.g_ids[0] ||
	    final_samples.ptsc_accepted !=
	    final_samples.ptsc_emitted + final_samples.ptsc_dropped ||
	    final_samples.ptsc_run_refs != 0) {
		snprintf(red_failure, sizeof(red_failure),
		    "drain-stress accounting not lossless: handle=%ju/%ju "
		    "accepted=%ju emitted=%ju dropped=%ju run_refs=%ju",
		    (uintmax_t)final_samples.ptsc_handle,
		    (uintmax_t)sampling.g_ids[0],
		    (uintmax_t)final_samples.ptsc_accepted,
		    (uintmax_t)final_samples.ptsc_emitted,
		    (uintmax_t)final_samples.ptsc_dropped,
		    (uintmax_t)final_samples.ptsc_run_refs);
	}
	printf("drain-stress: accepted=%ju emitted=%ju dropped=%ju "
	    "(mux_period_ms %d)\n", (uintmax_t)final_samples.ptsc_accepted,
	    (uintmax_t)final_samples.ptsc_emitted,
	    (uintmax_t)final_samples.ptsc_dropped, mux_stress);

cleanup:
	reset_system_start_test_hooks();
	reset_system_stop_test_hooks();
	group_teardown(&competitor);
	group_teardown(&sampling);
	if (mux_changed &&
	    sysctlbyname("kern.hwpmc.mux_period_ms", NULL, NULL, &mux_saved,
	    sizeof(mux_saved)) != 0 && failure[0] == '\0') {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "restoring kern.hwpmc.mux_period_ms failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
	}
	if (logfile_configured && pmc_configure_logfile(-1) != 0 &&
	    failure[0] == '\0') {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "deconfiguring drain-stress logfile failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
	}
	if (log_fd >= 0 && close(log_fd) != 0 && failure[0] == '\0') {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "closing drain-stress logfile failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
	}
	if (log_path[0] != '\0' && unlink(log_path) != 0 &&
	    failure[0] == '\0') {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "unlinking drain-stress logfile failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
	}
	reason = failure[0] != '\0' ? failure :
	    (red_failure[0] != '\0' ? red_failure : "successful test path");
	if (wait_live_target_counts(&baseline, &final_counts,
	    &failed_counter) != 0) {
		saved_errno = errno;
		if (failed_counter != NULL) {
			atf_tc_fail("%s; reading %s during cleanup failed: "
			    "errno %d (%s)", reason, failed_counter,
			    saved_errno, strerror(saved_errno));
		}
		atf_tc_fail("%s; cleanup counters did not return to baseline: "
		    "errno %d (%s)", reason, saved_errno, strerror(saved_errno));
	}
	if (failure[0] != '\0')
		atf_tc_fail("%s", failure);
	if (red_failure[0] != '\0')
		atf_tc_fail("%s", red_failure);
}

/*
 * A callchain record that cannot reserve log space was accepted by the sample
 * ring but was not emitted.  It must be attributed as an explicit drop.
 */
ATF_TC_WITHOUT_HEAD(callchain_log_reservation_failure_is_dropped);
ATF_TC_BODY(callchain_log_reservation_failure_is_dropped, tc)
{
	struct group g;
	struct group_build_failure build_failure;
	struct live_target_counts baseline, final_counts;
	struct pmc_test_sample_counts paused_samples, final_samples;
	const char *failed_counter, *reason;
	char failure[768], red_failure[384], log_path[PATH_MAX];
	uint64_t callchains;
	int log_fd, saved_errno;
	bool logfile_configured;

	group_init(&g);
	memset(&baseline, 0, sizeof(baseline));
	memset(&final_counts, 0, sizeof(final_counts));
	failure[0] = '\0';
	red_failure[0] = '\0';
	log_path[0] = '\0';
	log_fd = -1;
	logfile_configured = false;

	require_hwpmc();
	require_hwpmc_test_support();
	pin_to_test_cpu_required();
	reset_system_start_test_hooks();
	read_live_target_counts(&baseline);

	log_fd = open_named_logfile(log_path, sizeof(log_path));
	if (log_fd < 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "creating callchain-failure logfile failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	if (pmc_configure_logfile(log_fd) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "configuring callchain-failure logfile failed: "
		    "errno %d (%s)", saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	logfile_configured = true;

	errno = 0;
	if (group_build_n(&g, 2, true, false, &build_failure) != 0) {
		format_group_build_failure(failure, sizeof(failure),
		    "building callchain-failure sampling group",
		    &build_failure);
		goto cleanup;
	}
	if (sysctl_write_u32(TEST_PAUSE_SAMPLE_WORKER, g.g_id) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "requesting callchain-failure sample pause for group %u "
		    "failed: errno %d (%s)", g.g_id, saved_errno,
		    strerror(saved_errno));
		goto cleanup;
	}
	if (pmc_start(g.g_ids[0]) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "starting callchain-failure group failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	g.g_started = true;
	if (spin_until_test_ack(TEST_PAUSE_SAMPLE_WORKER_ACK,
	    g.g_id) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "no sample reached the callchain-failure pause for "
		    "group %u: errno %d (%s)", g.g_id, saved_errno,
		    strerror(saved_errno));
		goto cleanup;
	}
	if (wait_sample_pending(g.g_ids[0], &paused_samples) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "paused callchain-failure samples were not stable: "
		    "errno %d (%s)", saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	if (paused_samples.ptsc_handle != g.g_ids[0]) {
		snprintf(failure, sizeof(failure),
		    "paused callchain-failure handle=%ju, expected %ju",
		    (uintmax_t)paused_samples.ptsc_handle,
		    (uintmax_t)g.g_ids[0]);
		goto cleanup;
	}

	if (sysctl_write_u32(TEST_FAIL_CALLCHAIN_LOG_HANDLE,
	    g.g_ids[0]) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "arming callchain log reservation failure for handle "
		    "%ju failed: errno %d (%s)", (uintmax_t)g.g_ids[0],
		    saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	if (sysctl_write_u32(TEST_PAUSE_SAMPLE_WORKER, 0) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "releasing callchain-failure sample worker failed: "
		    "errno %d (%s)", saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	if (wait_test_sysctl_ack(TEST_FAIL_CALLCHAIN_LOG_HANDLE_ACK,
	    g.g_ids[0]) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "callchain log reservation failure for handle %ju was "
		    "not consumed: errno %d (%s)", (uintmax_t)g.g_ids[0],
		    saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	if (pmc_stop(g.g_ids[0]) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "stopping callchain-failure group failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	g.g_started = false;
	if (wait_sample_reconciled(g.g_ids[0], &final_samples) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "callchain-failure samples did not reconcile: "
		    "errno %d (%s)", saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	if (final_samples.ptsc_handle != g.g_ids[0] ||
	    final_samples.ptsc_accepted < paused_samples.ptsc_accepted ||
	    final_samples.ptsc_accepted !=
	    final_samples.ptsc_emitted + final_samples.ptsc_dropped ||
	    final_samples.ptsc_run_refs != 0) {
		snprintf(failure, sizeof(failure),
		    "callchain-failure sample accounting was not internally "
		    "consistent: handle=%ju/%ju accepted=%ju/%ju "
		    "emitted=%ju dropped=%ju run_refs=%ju",
		    (uintmax_t)final_samples.ptsc_handle,
		    (uintmax_t)g.g_ids[0],
		    (uintmax_t)final_samples.ptsc_accepted,
		    (uintmax_t)paused_samples.ptsc_accepted,
		    (uintmax_t)final_samples.ptsc_emitted,
		    (uintmax_t)final_samples.ptsc_dropped,
		    (uintmax_t)final_samples.ptsc_run_refs);
		goto cleanup;
	}

	if (pmc_writelog(TEST_LOG_FAILURE_MARKER) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "writing callchain-failure log marker failed: "
		    "errno %d (%s)", saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	if (pmc_flush_logfile() != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "flushing callchain-failure log marker failed: "
		    "errno %d (%s)", saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	if (wait_log_callchain_marker(log_path, TEST_LOG_FAILURE_MARKER,
	    g.g_ids[0], &callchains) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "callchain-failure log marker was not readable: "
		    "errno %d (%s)", saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	if (final_samples.ptsc_emitted != callchains ||
	    final_samples.ptsc_dropped == 0) {
		snprintf(red_failure, sizeof(red_failure),
		    "callchain reservation failure was misclassified: "
		    "accepted=%ju emitted=%ju dropped=%ju "
		    "logged_callchains=%ju",
		    (uintmax_t)final_samples.ptsc_accepted,
		    (uintmax_t)final_samples.ptsc_emitted,
		    (uintmax_t)final_samples.ptsc_dropped,
		    (uintmax_t)callchains);
	}

cleanup:
	reset_system_start_test_hooks();
	group_teardown(&g);
	if (logfile_configured && pmc_configure_logfile(-1) != 0 &&
	    failure[0] == '\0') {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "deconfiguring callchain-failure logfile failed: "
		    "errno %d (%s)", saved_errno, strerror(saved_errno));
	}
	if (log_fd >= 0 && close(log_fd) != 0 && failure[0] == '\0') {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "closing callchain-failure logfile failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
	}
	if (log_path[0] != '\0' && unlink(log_path) != 0 &&
	    failure[0] == '\0') {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "unlinking callchain-failure logfile failed: "
		    "errno %d (%s)", saved_errno, strerror(saved_errno));
	}
	reason = failure[0] != '\0' ? failure :
	    (red_failure[0] != '\0' ? red_failure : "successful test path");
	if (wait_live_target_counts(&baseline, &final_counts,
	    &failed_counter) != 0) {
		saved_errno = errno;
		if (failed_counter != NULL) {
			atf_tc_fail("%s; reading %s during cleanup failed: "
			    "errno %d (%s)", reason, failed_counter,
			    saved_errno, strerror(saved_errno));
		}
		atf_tc_fail("%s; cleanup counters did not return to baseline: "
		    "errno %d (%s), pmc_target=%ju/%ju "
		    "group_target=%ju/%ju process=%ju/%ju "
		    "residual=%ju/%ju rotation=%ju/%ju run=%ju/%ju",
		    reason, saved_errno, strerror(saved_errno),
		    (uintmax_t)final_counts.ltc_pmc_targets,
		    (uintmax_t)baseline.ltc_pmc_targets,
		    (uintmax_t)final_counts.ltc_group_targets,
		    (uintmax_t)baseline.ltc_group_targets,
		    (uintmax_t)final_counts.ltc_target_processes,
		    (uintmax_t)baseline.ltc_target_processes,
		    (uintmax_t)final_counts.ltc_residual_entries,
		    (uintmax_t)baseline.ltc_residual_entries,
		    (uintmax_t)final_counts.ltc_rotation_refs,
		    (uintmax_t)baseline.ltc_rotation_refs,
		    (uintmax_t)final_counts.ltc_run_refs,
		    (uintmax_t)baseline.ltc_run_refs);
	}
	if (failure[0] != '\0')
		atf_tc_fail("%s", failure);
	if (red_failure[0] != '\0')
		atf_tc_fail("%s", red_failure);
}

/*
 * Eviction changes residency, not the user-visible started state.  Stopping
 * an evicted group must clear the logical running and system-sampling state,
 * and the stopped group must not return when another group occupies the rows.
 */
ATF_TC_WITHOUT_HEAD(system_stop_while_evicted);
ATF_TC_BODY(system_stop_while_evicted, tc)
{
	struct group sampling, competitor, fresh;
	struct group_build_failure build_failure;
	struct group_snapshot stopped_before, stopped_after;
	struct live_target_counts baseline, final_counts;
	struct pmc_test_group_state before_stop, after_stop, competitor_state;
	struct pmc_test_group_state final_state;
	const char *failed_counter;
	FILE *log;
	char failure[768];
	bool logfile_configured;
	u_int cap, i;
	int error, saved_errno;

	group_init(&sampling);
	group_init(&competitor);
	group_init(&fresh);
	memset(&baseline, 0, sizeof(baseline));
	memset(&final_counts, 0, sizeof(final_counts));
	failure[0] = '\0';
	log = NULL;
	logfile_configured = false;

	require_hwpmc();
	require_hwpmc_test_support();
	pin_to_test_cpu_required();
	reset_system_stop_test_hooks();

	cap = probe_group_capacity();
	if (cap < 2)
		atf_tc_skip("requires at least two system PMC rows on CPU %d",
		    TEST_CPU);
	read_live_target_counts(&baseline);

	log = tmpfile();
	ATF_REQUIRE_MSG(log != NULL, "tmpfile failed: errno %d (%s)", errno,
	    strerror(errno));
	ATF_REQUIRE_MSG(pmc_configure_logfile(fileno(log)) == 0,
	    "logfile configuration failed: errno %d (%s)", errno,
	    strerror(errno));
	logfile_configured = true;

	errno = 0;
	if (group_build_n(&sampling, cap, true, true, &build_failure) != 0) {
		format_group_build_failure(failure, sizeof(failure),
		    "building full sampling group", &build_failure);
		goto fail_cleanup;
	}
	if (sysctl_write_u32(TEST_HOLD_GROUP_RESIDENT, sampling.g_id) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "requesting resident hold for group %u failed: errno %d (%s)",
		    sampling.g_id, saved_errno, strerror(saved_errno));
		goto fail_cleanup;
	}
	errno = 0;
	if (pmc_start(sampling.g_ids[0]) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "starting sampling group failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
		goto fail_cleanup;
	}
	sampling.g_started = true;
	if (wait_test_sysctl_ack(TEST_HOLD_GROUP_RESIDENT_ACK,
	    sampling.g_id) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "resident hold for sampling group %u was not acknowledged: "
		    "errno %d (%s)", sampling.g_id, saved_errno,
		    strerror(saved_errno));
		goto fail_cleanup;
	}

	errno = 0;
	if (group_build_n(&competitor, cap, false, true, &build_failure) != 0) {
		format_group_build_failure(failure, sizeof(failure),
		    "building full competitor under occupancy", &build_failure);
		goto fail_cleanup;
	}
	errno = 0;
	if (pmc_start(competitor.g_ids[0]) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "starting deferred competitor failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
		goto fail_cleanup;
	}
	competitor.g_started = true;
	if (read_group_state(competitor.g_ids[0], &competitor_state) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "reading deferred competitor state failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
		goto fail_cleanup;
	}
	if (competitor_state.ptgs_running != 1 ||
	    competitor_state.ptgs_assigned != 0 ||
	    competitor_state.ptgs_sys_listed != 1 ||
	    competitor_state.ptgs_sscounted != 0 ||
	    competitor_state.ptgs_nevents != cap) {
		snprintf(failure, sizeof(failure),
		    "competitor did not start deferred: running=%u assigned=%u "
		    "listed=%u sscounted=%u events=%u expected events=%u",
		    competitor_state.ptgs_running,
		    competitor_state.ptgs_assigned,
		    competitor_state.ptgs_sys_listed,
		    competitor_state.ptgs_sscounted,
		    competitor_state.ptgs_nevents, cap);
		goto fail_cleanup;
	}

	if (sysctl_write_u32(TEST_HOLD_GROUP_EVICTED, sampling.g_id) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "requesting evicted hold for group %u failed: errno %d (%s)",
		    sampling.g_id, saved_errno, strerror(saved_errno));
		goto fail_cleanup;
	}
	/*
	 * The resident gate has one request slot.  Replacing sampling.g_id with
	 * competitor.g_id releases the first group and holds the replacement.
	 */
	if (sysctl_write_u32(TEST_HOLD_GROUP_RESIDENT, competitor.g_id) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "requesting resident hold for competitor %u failed: errno %d "
		    "(%s)", competitor.g_id, saved_errno,
		    strerror(saved_errno));
		goto fail_cleanup;
	}
	if (wait_test_sysctl_ack(TEST_HOLD_GROUP_EVICTED_ACK,
	    sampling.g_id) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "sampling group %u was not deterministically evicted: errno "
		    "%d (%s)", sampling.g_id, saved_errno,
		    strerror(saved_errno));
		goto fail_cleanup;
	}
	if (wait_test_sysctl_ack(TEST_HOLD_GROUP_RESIDENT_ACK,
	    competitor.g_id) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "competitor %u did not become resident: errno %d (%s)",
		    competitor.g_id, saved_errno, strerror(saved_errno));
		goto fail_cleanup;
	}

	if (read_group_state(sampling.g_ids[0], &before_stop) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "reading evicted sampling-group state failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
		goto fail_cleanup;
	}
	if (before_stop.ptgs_handle != sampling.g_ids[0] ||
	    before_stop.ptgs_running != 1 ||
	    before_stop.ptgs_assigned != 0 ||
	    before_stop.ptgs_sys_listed != 1 ||
	    before_stop.ptgs_sscounted != 1 ||
	    before_stop.ptgs_nevents != cap ||
	    before_stop.ptgs_running_members != cap ||
	    before_stop.ptgs_stopped_members != 0 ||
	    before_stop.ptgs_allocated_members != 0) {
		snprintf(failure, sizeof(failure),
		    "evicted setup state is wrong: handle=%ju/%ju running=%u "
		    "assigned=%u listed=%u sscounted=%u events=%u/%u "
		    "members(running=%u stopped=%u)",
		    (uintmax_t)before_stop.ptgs_handle,
		    (uintmax_t)sampling.g_ids[0], before_stop.ptgs_running,
		    before_stop.ptgs_assigned, before_stop.ptgs_sys_listed,
		    before_stop.ptgs_sscounted, before_stop.ptgs_nevents, cap,
		    before_stop.ptgs_running_members,
		    before_stop.ptgs_stopped_members);
		goto fail_cleanup;
	}

	errno = 0;
	error = pmc_stop(sampling.g_ids[0]);
	saved_errno = errno;
	if (error != 0) {
		snprintf(failure, sizeof(failure),
		    "pmc_stop on evicted group returned %d, errno %d (%s)",
		    error, saved_errno, strerror(saved_errno));
		goto fail_cleanup;
	}
	if (read_group_state(sampling.g_ids[0], &after_stop) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "reading post-stop group state failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
		goto fail_cleanup;
	}
	if (after_stop.ptgs_running != 0 ||
	    after_stop.ptgs_assigned != 0 ||
	    after_stop.ptgs_sscounted != 0 ||
	    after_stop.ptgs_nevents != cap ||
	    after_stop.ptgs_running_members != 0 ||
	    after_stop.ptgs_stopped_members != cap) {
		snprintf(failure, sizeof(failure),
		    "pmc_stop returned success without stopping the evicted group: "
		    "before running=%u assigned=%u sscounted=%u "
		    "members(running=%u stopped=%u); after running=%u "
		    "assigned=%u listed=%u sscounted=%u "
		    "members(running=%u stopped=%u total=%u)",
		    before_stop.ptgs_running, before_stop.ptgs_assigned,
		    before_stop.ptgs_sscounted,
		    before_stop.ptgs_running_members,
		    before_stop.ptgs_stopped_members,
		    after_stop.ptgs_running, after_stop.ptgs_assigned,
		    after_stop.ptgs_sys_listed, after_stop.ptgs_sscounted,
		    after_stop.ptgs_running_members,
		    after_stop.ptgs_stopped_members, after_stop.ptgs_nevents);
		goto fail_cleanup;
	}
	sampling.g_started = false;

	reset_system_stop_test_hooks();
	errno = 0;
	if (pmc_configure_logfile(-1) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "closing logfile after a verified stop failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
		goto fail_cleanup;
	}
	logfile_configured = false;
	if (fclose(log) != 0) {
		saved_errno = errno;
		log = NULL;
		snprintf(failure, sizeof(failure),
		    "fclose after a verified stop failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
		goto fail_cleanup;
	}
	log = NULL;

	errno = 0;
	if (pmc_stop(competitor.g_ids[0]) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "stopping resident competitor failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
		goto fail_cleanup;
	}
	competitor.g_started = false;
	group_teardown(&competitor);

	if (read_group_snapshot(&sampling, &stopped_before) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "reading stopped group before replacement work failed: errno "
		    "%d (%s)", saved_errno, strerror(saved_errno));
		goto fail_cleanup;
	}
	errno = 0;
	if (group_build_n(&fresh, cap, false, true, &build_failure) != 0) {
		format_group_build_failure(failure, sizeof(failure),
		    "building fresh full-size group", &build_failure);
		goto fail_cleanup;
	}
	if (sysctl_write_u32(TEST_HOLD_GROUP_RESIDENT, fresh.g_id) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "requesting resident hold for fresh group %u failed: errno %d "
		    "(%s)", fresh.g_id, saved_errno, strerror(saved_errno));
		goto fail_cleanup;
	}
	errno = 0;
	if (pmc_start(fresh.g_ids[0]) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "starting fresh full-size group failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
		goto fail_cleanup;
	}
	fresh.g_started = true;
	if (wait_test_sysctl_ack(TEST_HOLD_GROUP_RESIDENT_ACK,
	    fresh.g_id) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "fresh group %u did not become resident: errno %d (%s)",
		    fresh.g_id, saved_errno, strerror(saved_errno));
		goto fail_cleanup;
	}

	spin();

	if (read_group_snapshot(&sampling, &stopped_after) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "reading stopped group after replacement work failed: errno "
		    "%d (%s)", saved_errno, strerror(saved_errno));
		goto fail_cleanup;
	}
	if (read_group_state(sampling.g_ids[0], &final_state) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "reading final stopped-group state failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
		goto fail_cleanup;
	}
	if (final_state.ptgs_running != 0 ||
	    final_state.ptgs_assigned != 0 ||
	    final_state.ptgs_sscounted != 0) {
		snprintf(failure, sizeof(failure),
		    "stopped group returned during replacement work: running=%u "
		    "assigned=%u sscounted=%u",
		    final_state.ptgs_running, final_state.ptgs_assigned,
		    final_state.ptgs_sscounted);
		goto fail_cleanup;
	}
	if (stopped_before.gs_nmembers != cap ||
	    stopped_after.gs_nmembers != cap ||
	    stopped_after.gs_times.pgt_enabled !=
	    stopped_before.gs_times.pgt_enabled ||
	    stopped_after.gs_times.pgt_running !=
	    stopped_before.gs_times.pgt_running) {
		snprintf(failure, sizeof(failure),
		    "stopped group time changed: members=%u/%u expected=%u "
		    "enabled=%ju/%ju running=%ju/%ju",
		    stopped_before.gs_nmembers, stopped_after.gs_nmembers, cap,
		    (uintmax_t)stopped_before.gs_times.pgt_enabled,
		    (uintmax_t)stopped_after.gs_times.pgt_enabled,
		    (uintmax_t)stopped_before.gs_times.pgt_running,
		    (uintmax_t)stopped_after.gs_times.pgt_running);
		goto fail_cleanup;
	}
	for (i = 1; i < cap; i++) {
		if (stopped_before.gs_members[i].pm_pmcid != sampling.g_ids[i] ||
		    stopped_after.gs_members[i].pm_pmcid != sampling.g_ids[i] ||
		    stopped_after.gs_members[i].pm_value !=
		    stopped_before.gs_members[i].pm_value) {
			snprintf(failure, sizeof(failure),
			    "stopped counting member %u changed: handle=%ju/%ju "
			    "expected=%ju value=%ju/%ju", i,
			    (uintmax_t)stopped_before.gs_members[i].pm_pmcid,
			    (uintmax_t)stopped_after.gs_members[i].pm_pmcid,
			    (uintmax_t)sampling.g_ids[i],
			    (uintmax_t)stopped_before.gs_members[i].pm_value,
			    (uintmax_t)stopped_after.gs_members[i].pm_value);
			goto fail_cleanup;
		}
	}

	reset_system_stop_test_hooks();
	group_teardown(&fresh);
	group_teardown(&sampling);
	if (wait_live_target_counts(&baseline, &final_counts,
	    &failed_counter) != 0) {
		saved_errno = errno;
		if (failed_counter != NULL)
			atf_tc_fail("reading %s during final cleanup failed: "
			    "errno %d (%s)", failed_counter, saved_errno,
			    strerror(saved_errno));
		atf_tc_fail("live counters did not return to baseline: errno %d "
		    "(%s); pmc_target=%ju/%ju group_target=%ju/%ju "
		    "process=%ju/%ju residual=%ju/%ju rotation=%ju/%ju "
		    "run=%ju/%ju", saved_errno, strerror(saved_errno),
		    (uintmax_t)final_counts.ltc_pmc_targets,
		    (uintmax_t)baseline.ltc_pmc_targets,
		    (uintmax_t)final_counts.ltc_group_targets,
		    (uintmax_t)baseline.ltc_group_targets,
		    (uintmax_t)final_counts.ltc_target_processes,
		    (uintmax_t)baseline.ltc_target_processes,
		    (uintmax_t)final_counts.ltc_residual_entries,
		    (uintmax_t)baseline.ltc_residual_entries,
		    (uintmax_t)final_counts.ltc_rotation_refs,
		    (uintmax_t)baseline.ltc_rotation_refs,
		    (uintmax_t)final_counts.ltc_run_refs,
		    (uintmax_t)baseline.ltc_run_refs);
	}
	return;

fail_cleanup:
	/*
	 * On the expected pre-fix failure, the sampling group is still logically
	 * running.  Release every group before closing the logfile so cleanup
	 * cannot turn the intended state mismatch into an sscount assertion.
	 */
	reset_system_stop_test_hooks();
	group_teardown(&fresh);
	group_teardown(&competitor);
	group_teardown(&sampling);
	if (logfile_configured)
		(void)pmc_configure_logfile(-1);
	if (log != NULL)
		(void)fclose(log);
	if (wait_live_target_counts(&baseline, &final_counts,
	    &failed_counter) != 0) {
		saved_errno = errno;
		if (failed_counter != NULL)
			atf_tc_fail("%s; reading %s during failure cleanup failed: "
			    "errno %d (%s)", failure, failed_counter,
			    saved_errno, strerror(saved_errno));
		atf_tc_fail("%s; cleanup counters did not return to baseline: "
		    "errno %d (%s), pmc_target=%ju/%ju group_target=%ju/%ju "
		    "process=%ju/%ju residual=%ju/%ju rotation=%ju/%ju "
		    "run=%ju/%ju", failure, saved_errno, strerror(saved_errno),
		    (uintmax_t)final_counts.ltc_pmc_targets,
		    (uintmax_t)baseline.ltc_pmc_targets,
		    (uintmax_t)final_counts.ltc_group_targets,
		    (uintmax_t)baseline.ltc_group_targets,
		    (uintmax_t)final_counts.ltc_target_processes,
		    (uintmax_t)baseline.ltc_target_processes,
		    (uintmax_t)final_counts.ltc_residual_entries,
		    (uintmax_t)baseline.ltc_residual_entries,
		    (uintmax_t)final_counts.ltc_rotation_refs,
		    (uintmax_t)baseline.ltc_rotation_refs,
		    (uintmax_t)final_counts.ltc_run_refs,
		    (uintmax_t)baseline.ltc_run_refs);
	}
	atf_tc_fail("%s", failure);
}

ATF_TP_ADD_TCS(tp)
{

	ATF_TP_ADD_TC(tp, grouped_system_sampling);
	ATF_TP_ADD_TC(tp, grouped_system_sampling_accounting);
	ATF_TP_ADD_TC(tp, system_release_while_running_resident);
	ATF_TP_ADD_TC(tp, system_release_while_running_evicted);
	ATF_TP_ADD_TC(tp, grouped_system_sampling_requires_log);
	ATF_TP_ADD_TC(tp,
	    grouped_system_sampling_start_rollback_after_preflight);
	ATF_TP_ADD_TC(tp,
	    grouped_system_sampling_partial_start_rollback_drains);
	ATF_TP_ADD_TC(tp, system_sampling_drain_one_queued_sample);
	ATF_TP_ADD_TC(tp, system_sampling_drain_stress);
	ATF_TP_ADD_TC(tp, callchain_log_reservation_failure_is_dropped);
	ATF_TP_ADD_TC(tp, system_stop_while_evicted);

	return (atf_no_error());
}
