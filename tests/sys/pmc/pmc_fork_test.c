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
#include <sys/stat.h>
#include <sys/sysctl.h>
#include <sys/wait.h>

#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <pmc.h>
#include <pmclog.h>
#include <poll.h>
#include <pthread.h>
#include <signal.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <atf-c.h>

#define	TEST_EVENT	"instructions"
#define	NCHILDREN	4
#define	MAX_ROW_OCCUPIERS	64
/* Each child does this much; the parent only forks and waits. */
#define	CHILD_ITERS	(200ULL * 1000 * 1000)
#define	CHILD_MIN_DELTA	(10ULL * 1000 * 1000)
#define	TEST_FORK_LOG_MARKER	0x47464d49U
#define	FORK_LOG_NGROUPS	2
#define	FORK_LOG_NMEMBERS	2
/*
 * The no-install test build uses the host's installed libpmc headers. Keep the
 * source-tree append-only record number local until that userland is installed
 * or linked directly.
 */
#define	TEST_PMCLOG_TYPE_GROUP_INHERIT_MISS	21

#define	CHILD_READY	'r'
#define	CHILD_CONTROL	'c'
#define	CHILD_WORK	'w'
#define	CHILD_EXIT	'x'
#define	TARGET_READY	'R'
#define	TARGET_FORK	'F'
#define	TARGET_FORKING	'f'
#define	TARGET_EXIT	'P'

#define	TEST_SYSCTL_PREFIX	"kern.hwpmc.test."
#define	TEST_FAIL_GROUP_TARGET_ALLOC					\
    TEST_SYSCTL_PREFIX "fail_group_target_alloc_after"
#define	TEST_FAIL_TARGET_LINK_ALLOC					\
    TEST_SYSCTL_PREFIX "fail_target_link_alloc_after"
#define	TEST_FAIL_ATTACH_AUTHORIZATION					\
    TEST_SYSCTL_PREFIX "fail_attach_authorization_after"
#define	TEST_HOLD_GROUP_RESIDENT					\
    TEST_SYSCTL_PREFIX "hold_group_resident"
#define	TEST_HOLD_GROUP_RESIDENT_ACK					\
    TEST_SYSCTL_PREFIX "hold_group_resident_ack"
#define	TEST_HOLD_GROUP_EVICTED						\
    TEST_SYSCTL_PREFIX "hold_group_evicted"
#define	TEST_HOLD_GROUP_EVICTED_ACK					\
    TEST_SYSCTL_PREFIX "hold_group_evicted_ack"
#define	TEST_PAUSE_DESCENDANTS_ATTACH					\
    TEST_SYSCTL_PREFIX "pause_descendants_attach"
#define	TEST_PAUSE_DESCENDANTS_ATTACH_ACK				\
    TEST_SYSCTL_PREFIX "pause_descendants_attach_ack"
#define	TEST_PAUSE_DESCENDANTS_TARGET_COUNT				\
    TEST_SYSCTL_PREFIX "pause_descendants_target_count"
#define	TEST_DESCENDANTS_EXIT_PID					\
    TEST_SYSCTL_PREFIX "descendants_exit_pid"
#define	TEST_DESCENDANTS_EXIT_ACK					\
    TEST_SYSCTL_PREFIX "descendants_exit_ack"
#define	TEST_LIVE_PMC_TARGETS						\
    TEST_SYSCTL_PREFIX "live_pmc_targets"
#define	TEST_LIVE_GROUP_TARGETS						\
    TEST_SYSCTL_PREFIX "live_group_targets"
#define	TEST_LIVE_TARGET_PROCESSES					\
    TEST_SYSCTL_PREFIX "live_target_processes"
#define	TEST_LIVE_RESIDUAL_ENTRIES					\
    TEST_SYSCTL_PREFIX "live_residual_entries"
#define	TEST_LIVE_ROTATION_REFS						\
    TEST_SYSCTL_PREFIX "live_rotation_refs"
#define	TEST_LIVE_RUN_REFS						\
    TEST_SYSCTL_PREFIX "live_run_refs"

struct group {
	uint32_t	g_id;
	pmc_id_t	g_ids[2];
	u_int		g_n;
	bool		g_started;
};

struct row_occupancy {
	pmc_id_t	ro_ids[MAX_ROW_OCCUPIERS];
	u_int		ro_n;
};

struct child_read_result {
	int		cr_control_rc;
	int		cr_control_errno;
	int		cr_leader_rc;
	int		cr_leader_errno;
	int		cr_sibling_rc;
	int		cr_sibling_errno;
	pmc_value_t	cr_control_value;
	pmc_value_t	cr_leader_value;
	pmc_value_t	cr_sibling_value;
};

struct fork_log_child_result {
	int		flcr_rc[FORK_LOG_NGROUPS][FORK_LOG_NMEMBERS];
	int		flcr_errno[FORK_LOG_NGROUPS][FORK_LOG_NMEMBERS];
	pmc_value_t	flcr_value[FORK_LOG_NGROUPS][FORK_LOG_NMEMBERS];
};

struct fork_log_counts {
	u_int		flc_procfork;
	u_int		flc_attach[FORK_LOG_NGROUPS][FORK_LOG_NMEMBERS];
	u_int		flc_detach[FORK_LOG_NGROUPS][FORK_LOG_NMEMBERS];
	bool		flc_marker_seen;
};

enum inherited_fork_result_stage {
	INHERITED_FORK_RESULT_CHILD = 0,
	INHERITED_FORK_RESULT_FAILPOINT,
	INHERITED_FORK_RESULT_FORK
};

struct inherited_fork_result {
	struct fork_log_child_result	ifr_reads;
	pid_t				ifr_pid;
	int				ifr_errno;
	enum inherited_fork_result_stage	ifr_stage;
};

struct inherited_fork_log_counts {
	u_int	iflc_procfork;
	u_int	iflc_parent_attach[FORK_LOG_NGROUPS][FORK_LOG_NMEMBERS];
	u_int	iflc_child_attach[FORK_LOG_NGROUPS][FORK_LOG_NMEMBERS];
	u_int	iflc_child_detach[FORK_LOG_NGROUPS][FORK_LOG_NMEMBERS];
	u_int	iflc_child_miss[FORK_LOG_NGROUPS];
	u_int	iflc_unmatched_child_miss;
	bool	iflc_marker_seen;
};

struct test_pmclog_group_inherit_miss {
	PMCLOG_ENTRY_HEADER
	uint32_t	pl_pmcid;
	uint32_t	pl_pid;
} __packed;

struct live_target_counts {
	uint64_t	ltc_pmc_targets;
	uint64_t	ltc_group_targets;
	uint64_t	ltc_target_processes;
	uint64_t	ltc_residual_entries;
	uint64_t	ltc_rotation_refs;
	uint64_t	ltc_run_refs;
};

struct attach_thread_result {
	pmc_id_t	atr_leader;
	pid_t		atr_target;
	int		atr_rc;
	int		atr_errno;
};

static ssize_t	read_full(int fd, void *buffer, size_t length);

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

static void
read_live_target_counts(struct live_target_counts *counts)
{

	ATF_REQUIRE_MSG(sysctl_read_u64(TEST_LIVE_PMC_TARGETS,
	    &counts->ltc_pmc_targets) == 0,
	    "reading %s failed: %s", TEST_LIVE_PMC_TARGETS, strerror(errno));
	ATF_REQUIRE_MSG(sysctl_read_u64(TEST_LIVE_GROUP_TARGETS,
	    &counts->ltc_group_targets) == 0,
	    "reading %s failed: %s", TEST_LIVE_GROUP_TARGETS, strerror(errno));
	ATF_REQUIRE_MSG(sysctl_read_u64(TEST_LIVE_TARGET_PROCESSES,
	    &counts->ltc_target_processes) == 0,
	    "reading %s failed: %s", TEST_LIVE_TARGET_PROCESSES,
	    strerror(errno));
	ATF_REQUIRE_MSG(sysctl_read_u64(TEST_LIVE_RESIDUAL_ENTRIES,
	    &counts->ltc_residual_entries) == 0,
	    "reading %s failed: %s", TEST_LIVE_RESIDUAL_ENTRIES,
	    strerror(errno));
	ATF_REQUIRE_MSG(sysctl_read_u64(TEST_LIVE_ROTATION_REFS,
	    &counts->ltc_rotation_refs) == 0,
	    "reading %s failed: %s", TEST_LIVE_ROTATION_REFS,
	    strerror(errno));
	ATF_REQUIRE_MSG(sysctl_read_u64(TEST_LIVE_RUN_REFS,
	    &counts->ltc_run_refs) == 0,
	    "reading %s failed: %s", TEST_LIVE_RUN_REFS, strerror(errno));
}

static void
check_live_counts_equal(const char *description,
    const struct live_target_counts *actual,
    const struct live_target_counts *expected)
{

	ATF_CHECK_MSG(actual->ltc_pmc_targets == expected->ltc_pmc_targets &&
	    actual->ltc_group_targets == expected->ltc_group_targets &&
	    actual->ltc_target_processes == expected->ltc_target_processes &&
	    actual->ltc_residual_entries == expected->ltc_residual_entries &&
	    actual->ltc_rotation_refs == expected->ltc_rotation_refs &&
	    actual->ltc_run_refs == expected->ltc_run_refs,
	    "%s: pmc_target %ju/%ju group_target %ju/%ju process %ju/%ju "
	    "residual %ju/%ju rotation_ref %ju/%ju run_ref %ju/%ju",
	    description,
	    (uintmax_t)actual->ltc_pmc_targets,
	    (uintmax_t)expected->ltc_pmc_targets,
	    (uintmax_t)actual->ltc_group_targets,
	    (uintmax_t)expected->ltc_group_targets,
	    (uintmax_t)actual->ltc_target_processes,
	    (uintmax_t)expected->ltc_target_processes,
	    (uintmax_t)actual->ltc_residual_entries,
	    (uintmax_t)expected->ltc_residual_entries,
	    (uintmax_t)actual->ltc_rotation_refs,
	    (uintmax_t)expected->ltc_rotation_refs,
	    (uintmax_t)actual->ltc_run_refs,
	    (uintmax_t)expected->ltc_run_refs);
}

static int
open_named_logfile(char *path, size_t path_size)
{
	int length;

	length = snprintf(path, path_size, "/tmp/pmc-fork-test.XXXXXX");
	if (length < 0 || (size_t)length >= path_size) {
		errno = ENAMETOOLONG;
		return (-1);
	}
	return (mkstemp(path));
}

static bool
find_fork_log_member(const struct group groups[FORK_LOG_NGROUPS],
    pmc_id_t pmcid, u_int *group_index, u_int *member_index)
{
	u_int group, member;

	for (group = 0; group < FORK_LOG_NGROUPS; group++) {
		for (member = 0; member < groups[group].g_n; member++) {
			if (groups[group].g_ids[member] != pmcid)
				continue;
			*group_index = group;
			*member_index = member;
			return (true);
		}
	}
	return (false);
}

static int
scan_fork_inherit_log(const char *path, pid_t parent_pid, pid_t child_pid,
    const struct group groups[FORK_LOG_NGROUPS],
    struct fork_log_counts *counts)
{
	struct pmclog_ev event;
	void *parser;
	pmc_id_t pmcid;
	pid_t pid;
	u_int group, member;
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
		    event.pl_u.pl_f.pl_oldpid == parent_pid &&
		    event.pl_u.pl_f.pl_newpid == child_pid)
			counts->flc_procfork++;
		if (event.pl_type == PMCLOG_TYPE_PMCATTACH) {
			pmcid = event.pl_u.pl_t.pl_pmcid;
			pid = event.pl_u.pl_t.pl_pid;
			if (pid == child_pid &&
			    find_fork_log_member(groups, pmcid, &group, &member))
				counts->flc_attach[group][member]++;
		}
		if (event.pl_type == PMCLOG_TYPE_PMCDETACH) {
			pmcid = event.pl_u.pl_d.pl_pmcid;
			pid = event.pl_u.pl_d.pl_pid;
			if (pid == child_pid &&
			    find_fork_log_member(groups, pmcid, &group, &member))
				counts->flc_detach[group][member]++;
		}
		if (event.pl_type == PMCLOG_TYPE_USERDATA &&
		    event.pl_u.pl_u.pl_userdata == TEST_FORK_LOG_MARKER) {
			counts->flc_marker_seen = true;
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

/*
 * Read the source-tree wire format directly so this kernel test does not depend
 * on the host's installed libpmc recognizing the new append-only record.
 */
static int
scan_inherited_fork_miss_log(const char *path, pid_t inherited_parent_pid,
    pid_t child_pid, const struct group groups[FORK_LOG_NGROUPS],
    struct inherited_fork_log_counts *counts)
{
	const struct pmclog_header *header;
	const struct pmclog_pmcattach *attach;
	const struct pmclog_pmcdetach *detach;
	const struct pmclog_procfork *procfork;
	const struct pmclog_userdata *userdata;
	const struct test_pmclog_group_inherit_miss *miss;
	struct stat sb;
	uint8_t *data;
	size_t length, offset, record_length;
	ssize_t nread;
	pmc_id_t pmcid;
	pid_t pid;
	u_int group, member, type;
	int fd, saved_errno;

	memset(counts, 0, sizeof(*counts));
	data = NULL;
	fd = open(path, O_RDONLY);
	if (fd < 0)
		return (-1);
	if (fstat(fd, &sb) != 0)
		goto fail;
	if (sb.st_size <= 0 || (uintmax_t)sb.st_size > SIZE_MAX) {
		errno = EPROTO;
		goto fail;
	}
	length = (size_t)sb.st_size;
	data = malloc(length);
	if (data == NULL)
		goto fail;
	nread = read_full(fd, data, length);
	if (nread != (ssize_t)length) {
		if (nread >= 0)
			errno = EPROTO;
		goto fail;
	}
	if (close(fd) != 0) {
		fd = -1;
		goto fail;
	}
	fd = -1;

	for (offset = 0; offset < length; offset += record_length) {
		if (length - offset < sizeof(*header))
			goto protocol_error;
		header = (const void *)(data + offset);
		if (!PMCLOG_HEADER_CHECK_MAGIC(header->pl_header))
			goto protocol_error;
		record_length = PMCLOG_HEADER_TO_LENGTH(header->pl_header);
		if (record_length < sizeof(*header) ||
		    record_length % sizeof(uint32_t) != 0 ||
		    record_length > length - offset)
			goto protocol_error;
		type = PMCLOG_HEADER_TO_TYPE(header->pl_header);

		switch (type) {
		case PMCLOG_TYPE_PROCFORK:
			if (record_length < sizeof(*procfork))
				goto protocol_error;
			procfork = (const void *)(data + offset);
			if ((pid_t)procfork->pl_oldpid == inherited_parent_pid &&
			    (pid_t)procfork->pl_newpid == child_pid)
				counts->iflc_procfork++;
			break;
		case PMCLOG_TYPE_PMCATTACH:
			if (record_length <
			    offsetof(struct pmclog_pmcattach, pl_pathname))
				goto protocol_error;
			attach = (const void *)(data + offset);
			pmcid = attach->pl_pmcid;
			pid = (pid_t)attach->pl_pid;
			if (!find_fork_log_member(groups, pmcid, &group, &member))
				break;
			if (pid == inherited_parent_pid)
				counts->iflc_parent_attach[group][member]++;
			else if (pid == child_pid)
				counts->iflc_child_attach[group][member]++;
			break;
		case PMCLOG_TYPE_PMCDETACH:
			if (record_length < sizeof(*detach))
				goto protocol_error;
			detach = (const void *)(data + offset);
			pmcid = detach->pl_pmcid;
			pid = (pid_t)detach->pl_pid;
			if (pid == child_pid &&
			    find_fork_log_member(groups, pmcid, &group, &member))
				counts->iflc_child_detach[group][member]++;
			break;
		case PMCLOG_TYPE_USERDATA:
			if (record_length < sizeof(*userdata))
				goto protocol_error;
			userdata = (const void *)(data + offset);
			if (userdata->pl_userdata == TEST_FORK_LOG_MARKER)
				counts->iflc_marker_seen = true;
			break;
		case TEST_PMCLOG_TYPE_GROUP_INHERIT_MISS:
			if (record_length < sizeof(*miss))
				goto protocol_error;
			miss = (const void *)(data + offset);
			if ((pid_t)miss->pl_pid != child_pid)
				break;
			pmcid = miss->pl_pmcid;
			if (find_fork_log_member(groups, pmcid, &group, &member) &&
			    member == 0)
				counts->iflc_child_miss[group]++;
			else
				counts->iflc_unmatched_child_miss++;
			break;
		default:
			break;
		}
		if (counts->iflc_marker_seen)
			break;
	}

	free(data);
	return (0);

protocol_error:
	errno = EPROTO;
fail:
	saved_errno = errno;
	if (fd >= 0)
		(void)close(fd);
	free(data);
	errno = saved_errno;
	return (-1);
}

static void *
attach_group_thread(void *arg)
{
	struct attach_thread_result *result;

	result = arg;
	errno = 0;
	result->atr_rc = pmc_attach(result->atr_leader, result->atr_target);
	result->atr_errno = errno;
	return (NULL);
}

static void
require_test_sysctl_write(const char *name, u_int value)
{

	ATF_REQUIRE_MSG(sysctl_write_u32(name, value) == 0,
	    "writing %s=%u failed: errno %d (%s)", name, value, errno,
	    strerror(errno));
}

static void
wait_test_sysctl_ack(const char *name, u_int expected)
{
	u_int value;
	int i;

	for (i = 0; i < 5000; i++) {
		ATF_REQUIRE_MSG(sysctl_read_u32(name, &value) == 0,
		    "reading %s failed: errno %d (%s)", name, errno,
		    strerror(errno));
		if (value == expected)
			return;
		usleep(1000);
	}
	atf_tc_fail("%s did not acknowledge value %u", name, expected);
}

static void
reset_fork_test_hooks(void)
{

	(void)sysctl_write_u32(TEST_FAIL_GROUP_TARGET_ALLOC, UINT_MAX);
	(void)sysctl_write_u32(TEST_FAIL_TARGET_LINK_ALLOC, UINT_MAX);
	(void)sysctl_write_u32(TEST_FAIL_ATTACH_AUTHORIZATION, UINT_MAX);
	(void)sysctl_write_u32(TEST_HOLD_GROUP_RESIDENT, 0);
	(void)sysctl_write_u32(TEST_HOLD_GROUP_EVICTED, 0);
	(void)sysctl_write_u32(TEST_PAUSE_DESCENDANTS_ATTACH, 0);
	(void)sysctl_write_u32(TEST_DESCENDANTS_EXIT_PID, 0);
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

static void
group_build_with_mux_required(struct group *g, bool descendants, bool mux)
{
	uint32_t flags;
	u_int i;

	group_init(g);
	g->g_n = 2;
	for (i = 0; i < g->g_n; i++) {
		flags = (i == 0 && descendants) ? PMC_F_DESCENDANTS : 0;
		if (i == 0 && mux)
			flags |= PMC_F_GROUP_MUX;
		ATF_REQUIRE_MSG(pmc_allocate_group(TEST_EVENT, PMC_MODE_TC, flags,
		    PMC_CPU_ANY, &g->g_ids[i], 0) == 0,
		    "member %u allocation failed: %s", i, strerror(errno));
	}
	ATF_REQUIRE_MSG(pmc_group_create(&g->g_id) == 0,
	    "group creation failed: %s", strerror(errno));
	for (i = 0; i < g->g_n; i++) {
		ATF_REQUIRE_MSG(
		    pmc_group_add(g->g_id, g->g_ids[i], i == 0) == 0,
		    "group add for member %u failed: %s", i, strerror(errno));
	}
	ATF_REQUIRE_MSG(pmc_group_commit(g->g_id) == 0,
	    "group commit failed: %s", strerror(errno));
}

static void
group_build_required(struct group *g, bool descendants)
{

	group_build_with_mux_required(g, descendants, false);
}

static void
group_build_one_required(struct group *g, bool mux)
{
	uint32_t flags;

	group_init(g);
	g->g_n = 1;
	flags = PMC_F_DESCENDANTS;
	if (mux)
		flags |= PMC_F_GROUP_MUX;
	ATF_REQUIRE_MSG(pmc_allocate_group(TEST_EVENT, PMC_MODE_TC, flags,
	    PMC_CPU_ANY, &g->g_ids[0], 0) == 0,
	    "member allocation failed: %s", strerror(errno));
	ATF_REQUIRE_MSG(pmc_group_create(&g->g_id) == 0,
	    "group creation failed: %s", strerror(errno));
	ATF_REQUIRE_MSG(pmc_group_add(g->g_id, g->g_ids[0], true) == 0,
	    "group leader add failed: %s", strerror(errno));
	ATF_REQUIRE_MSG(pmc_group_commit(g->g_id) == 0,
	    "group commit failed: %s", strerror(errno));
}

static void
row_occupancy_init(struct row_occupancy *occupancy)
{
	u_int i;

	memset(occupancy, 0, sizeof(*occupancy));
	for (i = 0; i < nitems(occupancy->ro_ids); i++)
		occupancy->ro_ids[i] = PMC_ID_INVALID;
}

/*
 * Reserve every row that can host TEST_EVENT.  The current ungrouped no-row
 * contract is EINVAL; any other failure is a setup error, not a reason to skip.
 */
static void
row_occupancy_fill_required(struct row_occupancy *occupancy)
{
	pmc_id_t id;
	int allocation_errno;

	row_occupancy_init(occupancy);
	while (occupancy->ro_n < nitems(occupancy->ro_ids)) {
		errno = 0;
		if (pmc_allocate(TEST_EVENT, PMC_MODE_TC, 0, PMC_CPU_ANY, &id,
		    0) == 0) {
			occupancy->ro_ids[occupancy->ro_n++] = id;
			continue;
		}
		allocation_errno = errno;
		ATF_REQUIRE_MSG(allocation_errno == EINVAL,
		    "occupier allocation %u failed with errno %d (%s), "
		    "expected no-row EINVAL", occupancy->ro_n,
		    allocation_errno, strerror(allocation_errno));
		ATF_REQUIRE_MSG(occupancy->ro_n > 0,
		    "could not reserve any %s rows", TEST_EVENT);
		return;
	}
	atf_tc_fail("allocated %u %s occupiers without reaching no-row EINVAL",
	    occupancy->ro_n, TEST_EVENT);
}

static void
row_occupancy_release_required(struct row_occupancy *occupancy)
{
	u_int i;

	for (i = 0; i < occupancy->ro_n; i++) {
		ATF_REQUIRE_MSG(pmc_release(occupancy->ro_ids[i]) == 0,
		    "releasing occupier %u failed: errno %d (%s)", i, errno,
		    strerror(errno));
		occupancy->ro_ids[i] = PMC_ID_INVALID;
	}
	occupancy->ro_n = 0;
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

static int
read_byte(int fd, char *value)
{
	ssize_t n;

	do {
		n = read(fd, value, 1);
	} while (n == -1 && errno == EINTR);
	if (n == 0)
		errno = EPIPE;
	return (n == 1 ? 0 : -1);
}

static int
write_byte(int fd, char value)
{
	ssize_t n;

	do {
		n = write(fd, &value, 1);
	} while (n == -1 && errno == EINTR);
	return (n == 1 ? 0 : -1);
}

static ssize_t
read_full(int fd, void *buffer, size_t length)
{
	char *cursor;
	ssize_t n;
	size_t done;

	cursor = buffer;
	done = 0;
	while (done < length) {
		n = read(fd, cursor + done, length - done);
		if (n == -1 && errno == EINTR)
			continue;
		if (n <= 0)
			return (n == 0 ? (ssize_t)done : -1);
		done += (size_t)n;
	}
	return ((ssize_t)done);
}

static int
write_full(int fd, const void *buffer, size_t length)
{
	const char *cursor;
	ssize_t n;
	size_t done;

	cursor = buffer;
	done = 0;
	while (done < length) {
		n = write(fd, cursor + done, length - done);
		if (n == -1 && errno == EINTR)
			continue;
		if (n <= 0)
			return (-1);
		done += (size_t)n;
	}
	return (0);
}

/*
 * A timeout is only a deadlock guard.  EOF is the synchronization condition:
 * exit1() runs the hwpmc process-exit hook before closing file descriptors.
 */
static void
require_pipe_eof(int fd, const char *description)
{
	struct pollfd pfd;
	char value;
	int poll_errno, poll_rc;

	memset(&pfd, 0, sizeof(pfd));
	pfd.fd = fd;
	pfd.events = POLLIN | POLLHUP;
	errno = 0;
	do {
		poll_rc = poll(&pfd, 1, 5000);
	} while (poll_rc == -1 && errno == EINTR);
	poll_errno = errno;
	ATF_REQUIRE_MSG(poll_rc == 1,
	    "%s pipe did not close: poll rc=%d errno=%d (%s)",
	    description, poll_rc, poll_errno, strerror(poll_errno));

	errno = 0;
	ATF_REQUIRE_MSG(read_byte(fd, &value) == -1 && errno == EPIPE,
	    "%s pipe produced data or remained open after exit: revents=%#x",
	    description, pfd.revents);
}

static void
gated_child(int command_fd, int ack_fd)
{
	char command;

	if (write_byte(ack_fd, CHILD_READY) != 0)
		_exit(2);
	if (raise(SIGSTOP) != 0)
		_exit(3);
	while (read_byte(command_fd, &command) == 0) {
		switch (command) {
		case CHILD_CONTROL:
			break;
		case CHILD_WORK:
			work(CHILD_ITERS);
			break;
		case CHILD_EXIT:
			if (write_byte(ack_fd, command) != 0)
				_exit(4);
			_exit(0);
		default:
			_exit(5);
		}
		if (write_byte(ack_fd, command) != 0)
			_exit(6);
	}
	/* The parent exits on an assertion failure, closing the command pipe. */
	_exit(0);
}

/*
 * Become an explicit group target, fork one descendant on command, and then
 * exit before that descendant.  The descendant is driven by a separate pipe
 * so its no-work and work phases remain deterministic after this process exits.
 */
static void
target_parent_with_child(int parent_command_fd, int child_command_fd,
    int ack_fd, int result_fd, pmc_id_t leader, pmc_id_t sibling)
{
	struct child_read_result result;
	pid_t child;
	char command;

	if (write_byte(ack_fd, TARGET_READY) != 0)
		_exit(2);
	if (read_byte(parent_command_fd, &command) != 0 ||
	    command != TARGET_FORK)
		_exit(3);
	if (write_byte(ack_fd, TARGET_FORKING) != 0)
		_exit(4);

	child = fork();
	if (child == -1)
		_exit(5);
	if (child == 0) {
		close(parent_command_fd);
		memset(&result, 0, sizeof(result));
		errno = 0;
		result.cr_leader_rc =
		    pmc_read(leader, &result.cr_leader_value);
		result.cr_leader_errno = errno;
		if (sibling != PMC_ID_INVALID) {
			errno = 0;
			result.cr_sibling_rc =
			    pmc_read(sibling, &result.cr_sibling_value);
			result.cr_sibling_errno = errno;
		}
		if (write_full(result_fd, &result, sizeof(result)) != 0)
			_exit(6);
		close(result_fd);
		if (write_byte(ack_fd, CHILD_READY) != 0)
			_exit(7);
		while (read_byte(child_command_fd, &command) == 0) {
			switch (command) {
			case CHILD_CONTROL:
				break;
			case CHILD_WORK:
				work(CHILD_ITERS);
				break;
			case CHILD_EXIT:
				if (write_byte(ack_fd, command) != 0)
					_exit(8);
				_exit(0);
			default:
				_exit(9);
			}
			if (write_byte(ack_fd, command) != 0)
				_exit(10);
		}
		_exit(0);
	}

	close(child_command_fd);
	close(result_fd);
	if (read_byte(parent_command_fd, &command) != 0 ||
	    command != TARGET_EXIT)
		_exit(11);
	_exit(0);
}

static pid_t
spawn_gated_child(int *command_fd, int *ack_fd)
{
	int command_pipe[2], ack_pipe[2];
	pid_t child;
	char ready;
	int status;

	ATF_REQUIRE_MSG(pipe(command_pipe) == 0,
	    "command pipe failed: %s", strerror(errno));
	ATF_REQUIRE_MSG(pipe(ack_pipe) == 0,
	    "ack pipe failed: %s", strerror(errno));
	child = fork();
	ATF_REQUIRE_MSG(child != -1, "fork failed: %s", strerror(errno));
	if (child == 0) {
		close(command_pipe[1]);
		close(ack_pipe[0]);
		gated_child(command_pipe[0], ack_pipe[1]);
	}
	close(command_pipe[0]);
	close(ack_pipe[1]);
	ATF_REQUIRE_MSG(read_byte(ack_pipe[0], &ready) == 0,
	    "child readiness read failed: %s", strerror(errno));
	ATF_REQUIRE_EQ(ready, CHILD_READY);
	ATF_REQUIRE_MSG(waitpid(child, &status, WUNTRACED) == child,
	    "wait for parked child failed: %s", strerror(errno));
	ATF_REQUIRE_MSG(WIFSTOPPED(status) && WSTOPSIG(status) == SIGSTOP,
	    "child did not park before attach: status %#x", status);
	*command_fd = command_pipe[1];
	*ack_fd = ack_pipe[0];
	return (child);
}

static void
run_child_phase(int command_fd, int ack_fd, char command)
{
	char ack;

	ATF_REQUIRE_MSG(write_byte(command_fd, command) == 0,
	    "child command %c failed: %s", command, strerror(errno));
	ATF_REQUIRE_MSG(read_byte(ack_fd, &ack) == 0,
	    "child acknowledgment for %c failed: %s", command,
	    strerror(errno));
	ATF_REQUIRE_EQ(ack, command);
}

static void
group_snapshot(struct group *g, pmc_value_t values[2])
{
	struct pmc_group_member members[2];
	uint32_t n;
	u_int i;

	n = g->g_n;
	memset(members, 0, sizeof(members));
	ATF_REQUIRE_MSG(pmc_group_read(g->g_ids[0], &n, members, NULL) == 0,
	    "group snapshot failed: %s", strerror(errno));
	ATF_REQUIRE_EQ(n, g->g_n);
	for (i = 0; i < g->g_n; i++) {
		ATF_REQUIRE_EQ(members[i].pm_pmcid, g->g_ids[i]);
		values[i] = members[i].pm_value;
	}
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

/*
 * A resident grouped fork must publish all member links together.  Reading the
 * leader first proves that the child reached the inheritance path; the sibling
 * read then distinguishes a whole-group publication from the old leader-only
 * row link.
 */
ATF_TC_WITHOUT_HEAD(descendants_child_reads_whole_group);
ATF_TC_BODY(descendants_child_reads_whole_group, tc)
{
	struct child_read_result result;
	struct group g;
	pmc_id_t control;
	ssize_t nread;
	pid_t child, waited;
	int result_pipe[2], status, wait_errno;

	require_hwpmc();
	group_build_required(&g, true);
	ATF_REQUIRE_MSG(pmc_attach(g.g_ids[0], getpid()) == 0,
	    "group attach failed: %s", strerror(errno));
	ATF_REQUIRE_MSG(pmc_start(g.g_ids[0]) == 0,
	    "group start failed: %s", strerror(errno));
	g.g_started = true;
	ATF_REQUIRE_MSG(pmc_allocate(TEST_EVENT, PMC_MODE_TC,
	    PMC_F_DESCENDANTS, PMC_CPU_ANY, &control, 0) == 0,
	    "ungrouped descendants control allocation failed: %s",
	    strerror(errno));
	ATF_REQUIRE_MSG(pmc_attach(control, getpid()) == 0,
	    "ungrouped descendants control attach failed: %s",
	    strerror(errno));
	ATF_REQUIRE_MSG(pmc_start(control) == 0,
	    "ungrouped descendants control start failed: %s",
	    strerror(errno));
	ATF_REQUIRE_MSG(pipe(result_pipe) == 0,
	    "result pipe failed: %s", strerror(errno));

	child = fork();
	ATF_REQUIRE_MSG(child != -1, "fork failed: %s", strerror(errno));
	if (child == 0) {
		close(result_pipe[0]);
		memset(&result, 0, sizeof(result));
		errno = 0;
		result.cr_control_rc =
		    pmc_read(control, &result.cr_control_value);
		result.cr_control_errno = errno;
		errno = 0;
		result.cr_leader_rc =
		    pmc_read(g.g_ids[0], &result.cr_leader_value);
		result.cr_leader_errno = errno;
		errno = 0;
		result.cr_sibling_rc =
		    pmc_read(g.g_ids[1], &result.cr_sibling_value);
		result.cr_sibling_errno = errno;
		if (write_full(result_pipe[1], &result, sizeof(result)) != 0)
			_exit(2);
		_exit(0);
	}

	close(result_pipe[1]);
	memset(&result, 0, sizeof(result));
	nread = read_full(result_pipe[0], &result, sizeof(result));
	close(result_pipe[0]);
	waited = waitpid(child, &status, 0);
	wait_errno = errno;
	(void)pmc_stop(control);
	(void)pmc_release(control);
	group_teardown(&g);

	ATF_REQUIRE_MSG(nread == (ssize_t)sizeof(result),
	    "child result read returned %zd of %zu bytes", nread,
	    sizeof(result));
	ATF_REQUIRE_MSG(waited == child, "waitpid failed: errno %d (%s)",
	    wait_errno, strerror(wait_errno));
	ATF_REQUIRE_MSG(WIFEXITED(status) && WEXITSTATUS(status) == 0,
	    "child exited abnormally: status %#x", status);
	ATF_REQUIRE_MSG(result.cr_control_rc == 0,
	    "ungrouped descendants control was not inherited: errno %d (%s)",
	    result.cr_control_errno, strerror(result.cr_control_errno));
	ATF_REQUIRE_MSG(result.cr_leader_rc == 0,
	    "child leader read did not reach inherited target: errno %d (%s)",
	    result.cr_leader_errno, strerror(result.cr_leader_errno));
	ATF_CHECK_MSG(result.cr_sibling_rc == 0,
	    "child inherited only the leader; sibling read failed: errno %d "
	    "(%s)", result.cr_sibling_errno,
	    strerror(result.cr_sibling_errno));
}

static void
fork_inherit_alloc_failure_atomic(const char *failpoint_name, u_int fail_after,
    const char *failure_kind)
{
	struct child_read_result result;
	struct live_target_counts initial, parent_only, child_live, child_gone;
	struct live_target_counts final;
	struct group g;
	ssize_t nread;
	pid_t child, waited;
	char release;
	u_int failpoint;
	int release_pipe[2], result_pipe[2], status, wait_errno;

	require_hwpmc();
	require_hwpmc_test_support();
	reset_fork_test_hooks();
	read_live_target_counts(&initial);

	group_build_required(&g, true);
	ATF_REQUIRE_MSG(pmc_attach(g.g_ids[0], getpid()) == 0,
	    "group attach failed: %s", strerror(errno));
	require_test_sysctl_write(TEST_HOLD_GROUP_RESIDENT, g.g_id);
	ATF_REQUIRE_MSG(pmc_start(g.g_ids[0]) == 0,
	    "group start failed: %s", strerror(errno));
	g.g_started = true;
	wait_test_sysctl_ack(TEST_HOLD_GROUP_RESIDENT_ACK, g.g_id);
	read_live_target_counts(&parent_only);

	require_test_sysctl_write(failpoint_name, fail_after);
	ATF_REQUIRE_MSG(pipe(result_pipe) == 0,
	    "result pipe failed: %s", strerror(errno));
	ATF_REQUIRE_MSG(pipe(release_pipe) == 0,
	    "release pipe failed: %s", strerror(errno));
	child = fork();
	ATF_REQUIRE_MSG(child != -1, "fork failed: %s", strerror(errno));
	if (child == 0) {
		close(result_pipe[0]);
		close(release_pipe[1]);
		memset(&result, 0, sizeof(result));
		errno = 0;
		result.cr_leader_rc =
		    pmc_read(g.g_ids[0], &result.cr_leader_value);
		result.cr_leader_errno = errno;
		errno = 0;
		result.cr_sibling_rc =
		    pmc_read(g.g_ids[1], &result.cr_sibling_value);
		result.cr_sibling_errno = errno;
		if (write_full(result_pipe[1], &result, sizeof(result)) != 0)
			_exit(2);
		if (read_byte(release_pipe[0], &release) != 0)
			_exit(3);
		_exit(release == CHILD_EXIT ? 0 : 4);
	}

	close(result_pipe[1]);
	close(release_pipe[0]);
	memset(&result, 0, sizeof(result));
	nread = read_full(result_pipe[0], &result, sizeof(result));
	close(result_pipe[0]);
	ATF_REQUIRE_MSG(sysctl_read_u32(failpoint_name, &failpoint) == 0,
	    "reading consumed %s failpoint failed: errno %d (%s)",
	    failure_kind, errno, strerror(errno));
	read_live_target_counts(&child_live);
	ATF_REQUIRE_MSG(write_byte(release_pipe[1], CHILD_EXIT) == 0,
	    "releasing child failed: %s", strerror(errno));
	close(release_pipe[1]);
	waited = waitpid(child, &status, 0);
	wait_errno = errno;
	read_live_target_counts(&child_gone);

	reset_fork_test_hooks();
	group_teardown(&g);
	read_live_target_counts(&final);

	ATF_REQUIRE_MSG(nread == (ssize_t)sizeof(result),
	    "child result read returned %zd of %zu bytes", nread,
	    sizeof(result));
	ATF_REQUIRE_MSG(waited == child, "waitpid failed: errno %d (%s)",
	    wait_errno, strerror(wait_errno));
	ATF_REQUIRE_MSG(WIFEXITED(status) && WEXITSTATUS(status) == 0,
	    "child exited abnormally: status %#x", status);
	ATF_REQUIRE_MSG(failpoint == UINT_MAX,
	    "%s failpoint was not consumed: value %u", failure_kind,
	    failpoint);
	ATF_CHECK_MSG(result.cr_leader_rc == -1 &&
	    result.cr_leader_errno == ESRCH,
	    "%s failure left leader access rc=%d errno=%d (%s)",
	    failure_kind, result.cr_leader_rc, result.cr_leader_errno,
	    strerror(result.cr_leader_errno));
	ATF_CHECK_MSG(result.cr_sibling_rc == -1 &&
	    result.cr_sibling_errno == ESRCH,
	    "%s failure left sibling access rc=%d errno=%d (%s)",
	    failure_kind, result.cr_sibling_rc, result.cr_sibling_errno,
	    strerror(result.cr_sibling_errno));
	ATF_CHECK_MSG(child_live.ltc_pmc_targets ==
	    parent_only.ltc_pmc_targets &&
	    child_live.ltc_group_targets == parent_only.ltc_group_targets &&
	    child_live.ltc_target_processes == parent_only.ltc_target_processes,
	    "%s failure changed live counts while child was alive: "
	    "pmc_target %ju/%ju group_target %ju/%ju process %ju/%ju",
	    failure_kind,
	    (uintmax_t)child_live.ltc_pmc_targets,
	    (uintmax_t)parent_only.ltc_pmc_targets,
	    (uintmax_t)child_live.ltc_group_targets,
	    (uintmax_t)parent_only.ltc_group_targets,
	    (uintmax_t)child_live.ltc_target_processes,
	    (uintmax_t)parent_only.ltc_target_processes);
	ATF_CHECK_MSG(child_gone.ltc_pmc_targets ==
	    parent_only.ltc_pmc_targets &&
	    child_gone.ltc_group_targets == parent_only.ltc_group_targets &&
	    child_gone.ltc_target_processes == parent_only.ltc_target_processes,
	    "child exit after %s failure did not restore the parent-only "
	    "baseline", failure_kind);
	ATF_CHECK_MSG(final.ltc_pmc_targets == initial.ltc_pmc_targets &&
	    final.ltc_group_targets == initial.ltc_group_targets &&
	    final.ltc_target_processes == initial.ltc_target_processes &&
	    final.ltc_residual_entries == initial.ltc_residual_entries &&
	    final.ltc_rotation_refs == initial.ltc_rotation_refs &&
	    final.ltc_run_refs == initial.ltc_run_refs,
	    "group teardown after %s failure did not restore initial live counts: "
	    "pmc_target %ju/%ju group_target %ju/%ju process %ju/%ju "
	    "residual %ju/%ju rotation_ref %ju/%ju run_ref %ju/%ju",
	    failure_kind,
	    (uintmax_t)final.ltc_pmc_targets,
	    (uintmax_t)initial.ltc_pmc_targets,
	    (uintmax_t)final.ltc_group_targets,
	    (uintmax_t)initial.ltc_group_targets,
	    (uintmax_t)final.ltc_target_processes,
	    (uintmax_t)initial.ltc_target_processes,
	    (uintmax_t)final.ltc_residual_entries,
	    (uintmax_t)initial.ltc_residual_entries,
	    (uintmax_t)final.ltc_rotation_refs,
	    (uintmax_t)initial.ltc_rotation_refs,
	    (uintmax_t)final.ltc_run_refs,
	    (uintmax_t)initial.ltc_run_refs);
}

/*
 * If the authoritative inherited edge cannot be allocated, no transient
 * leader link or child process descriptor may survive.  The child stays alive
 * while the parent checks the exact object-count baseline.
 */
ATF_TC_WITH_CLEANUP(descendants_fork_inherit_alloc_failure_atomic);
ATF_TC_HEAD(descendants_fork_inherit_alloc_failure_atomic, tc)
{

	atf_tc_set_md_var(tc, "require.user", "root");
}
ATF_TC_BODY(descendants_fork_inherit_alloc_failure_atomic, tc)
{

	fork_inherit_alloc_failure_atomic(TEST_FAIL_GROUP_TARGET_ALLOC, 0,
	    "group-target allocation");
}
ATF_TC_CLEANUP(descendants_fork_inherit_alloc_failure_atomic, tc)
{

	reset_fork_test_hooks();
}

/*
 * A later member-link allocation failure must roll back the already allocated
 * first link and the authoritative edge.  With two members, allowing one
 * allocation before failure proves that the failpoint reached the second link.
 */
ATF_TC_WITH_CLEANUP(descendants_fork_inherit_link_alloc_failure_atomic);
ATF_TC_HEAD(descendants_fork_inherit_link_alloc_failure_atomic, tc)
{

	atf_tc_set_md_var(tc, "require.user", "root");
}
ATF_TC_BODY(descendants_fork_inherit_link_alloc_failure_atomic, tc)
{

	fork_inherit_alloc_failure_atomic(TEST_FAIL_TARGET_LINK_ALLOC, 1,
	    "later target-link allocation");
}
ATF_TC_CLEANUP(descendants_fork_inherit_link_alloc_failure_atomic, tc)
{

	reset_fork_test_hooks();
}

/*
 * A fork-inheritance miss is not a detach: the failed group's child edge was
 * never published.  Allow exactly one of two descendants groups to inherit,
 * then use child-side handle lookup to identify the successful and missed
 * groups independently of list order.  The logfile must contain one PROCFORK
 * for this owner and child, one PMCATTACH per successful member, and no
 * PMCATTACH or PMCDETACH for any missed member before the marker.
 */
ATF_TC_WITH_CLEANUP(descendants_fork_miss_has_no_false_detach);
ATF_TC_HEAD(descendants_fork_miss_has_no_false_detach, tc)
{

	atf_tc_set_md_var(tc, "require.user", "root");
}
ATF_TC_BODY(descendants_fork_miss_has_no_false_detach, tc)
{
	struct fork_log_child_result child_result;
	struct fork_log_counts log_counts;
	struct live_target_counts initial, final;
	struct group groups[FORK_LOG_NGROUPS];
	struct pollfd pfd;
	char failure[1024], log_path[PATH_MAX], release;
	pid_t child, waited;
	ssize_t nread;
	u_int failpoint, group, member;
	int inherited_group, missed_group;
	int log_fd, poll_errno, poll_rc, release_pipe[2], result_pipe[2];
	int saved_errno, status;
	bool all_missing, all_read, logfile_configured;

	for (group = 0; group < FORK_LOG_NGROUPS; group++)
		group_init(&groups[group]);
	memset(&child_result, 0, sizeof(child_result));
	memset(&log_counts, 0, sizeof(log_counts));
	memset(&initial, 0, sizeof(initial));
	memset(&final, 0, sizeof(final));
	memset(&pfd, 0, sizeof(pfd));
	failure[0] = '\0';
	log_path[0] = '\0';
	child = -1;
	waited = -1;
	status = 0;
	failpoint = 0;
	log_fd = -1;
	release_pipe[0] = release_pipe[1] = -1;
	result_pipe[0] = result_pipe[1] = -1;
	logfile_configured = false;

	require_hwpmc();
	require_hwpmc_test_support();
	reset_fork_test_hooks();
	read_live_target_counts(&initial);

	log_fd = open_named_logfile(log_path, sizeof(log_path));
	if (log_fd < 0) {
		saved_errno = errno;
		log_path[0] = '\0';
		snprintf(failure, sizeof(failure),
		    "creating fork-inherit logfile failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	if (pmc_configure_logfile(log_fd) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "configuring fork-inherit logfile failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	logfile_configured = true;

	for (group = 0; group < FORK_LOG_NGROUPS; group++) {
		if (group_build(&groups[group], true) != 0) {
			saved_errno = errno;
			snprintf(failure, sizeof(failure),
			    "building descendants group %u failed: errno %d (%s)",
			    group, saved_errno, strerror(saved_errno));
			goto cleanup;
		}
		if (pmc_attach(groups[group].g_ids[0], getpid()) != 0) {
			saved_errno = errno;
			snprintf(failure, sizeof(failure),
			    "attaching descendants group %u failed: errno %d (%s)",
			    group, saved_errno, strerror(saved_errno));
			goto cleanup;
		}
	}

	if (sysctl_write_u32(TEST_FAIL_GROUP_TARGET_ALLOC, 1) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "arming fork group-target failpoint failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	if (pipe(result_pipe) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "creating child-result pipe failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	if (pipe(release_pipe) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "creating child-release pipe failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
		goto cleanup;
	}

	child = fork();
	if (child < 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "forking log target failed: errno %d (%s)", saved_errno,
		    strerror(saved_errno));
		goto cleanup;
	}
	if (child == 0) {
		close(result_pipe[0]);
		close(release_pipe[1]);
		close(log_fd);
		memset(&child_result, 0, sizeof(child_result));
		for (group = 0; group < FORK_LOG_NGROUPS; group++) {
			for (member = 0; member < FORK_LOG_NMEMBERS; member++) {
				errno = 0;
				child_result.flcr_rc[group][member] =
				    pmc_read(groups[group].g_ids[member],
				    &child_result.flcr_value[group][member]);
				child_result.flcr_errno[group][member] = errno;
			}
		}
		if (write_full(result_pipe[1], &child_result,
		    sizeof(child_result)) != 0)
			_exit(2);
		if (read_byte(release_pipe[0], &release) != 0)
			_exit(3);
		_exit(release == CHILD_EXIT ? 0 : 4);
	}

	close(result_pipe[1]);
	result_pipe[1] = -1;
	close(release_pipe[0]);
	release_pipe[0] = -1;
	pfd.fd = result_pipe[0];
	pfd.events = POLLIN | POLLHUP;
	errno = 0;
	do {
		poll_rc = poll(&pfd, 1, 5000);
	} while (poll_rc == -1 && errno == EINTR);
	poll_errno = errno;
	if (poll_rc != 1) {
		snprintf(failure, sizeof(failure),
		    "child result was not ready: poll rc=%d errno=%d (%s)",
		    poll_rc, poll_errno, strerror(poll_errno));
		goto cleanup;
	}
	nread = read_full(result_pipe[0], &child_result,
	    sizeof(child_result));
	if (nread != (ssize_t)sizeof(child_result)) {
		snprintf(failure, sizeof(failure),
		    "child result read returned %zd of %zu bytes", nread,
		    sizeof(child_result));
		goto cleanup;
	}
	close(result_pipe[0]);
	result_pipe[0] = -1;

	if (sysctl_read_u32(TEST_FAIL_GROUP_TARGET_ALLOC, &failpoint) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "reading consumed fork failpoint failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	if (pmc_writelog(TEST_FORK_LOG_MARKER) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "writing fork-inherit marker failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	if (pmc_flush_logfile() != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "flushing fork-inherit marker failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	if (scan_fork_inherit_log(log_path, getpid(), child, groups,
	    &log_counts) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "parsing fork-inherit logfile failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
		goto cleanup;
	}

	if (write_byte(release_pipe[1], CHILD_EXIT) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "releasing fork-log child failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	close(release_pipe[1]);
	release_pipe[1] = -1;
	do {
		waited = waitpid(child, &status, 0);
	} while (waited < 0 && errno == EINTR);
	if (waited != child) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "waiting for fork-log child failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
	}

cleanup:
	if (result_pipe[0] >= 0)
		(void)close(result_pipe[0]);
	if (result_pipe[1] >= 0)
		(void)close(result_pipe[1]);
	if (release_pipe[0] >= 0)
		(void)close(release_pipe[0]);
	if (release_pipe[1] >= 0) {
		(void)close(release_pipe[1]);
		release_pipe[1] = -1;
	}
	if (child > 0 && waited != child) {
		do {
			waited = waitpid(child, &status, 0);
		} while (waited < 0 && errno == EINTR);
	}
	reset_fork_test_hooks();
	for (group = 0; group < FORK_LOG_NGROUPS; group++)
		group_teardown(&groups[group]);
	if (logfile_configured && pmc_configure_logfile(-1) != 0 &&
	    failure[0] == '\0') {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "deconfiguring fork-inherit logfile failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
	}
	if (log_fd >= 0 && close(log_fd) != 0 && failure[0] == '\0') {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "closing fork-inherit logfile failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
	}
	if (log_path[0] != '\0' && unlink(log_path) != 0 &&
	    failure[0] == '\0') {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "unlinking fork-inherit logfile failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
	}
	read_live_target_counts(&final);
	check_live_counts_equal("fork-miss logging cleanup", &final, &initial);

	if (failure[0] != '\0')
		atf_tc_fail("%s", failure);
	ATF_REQUIRE_MSG(waited == child,
	    "fork-log child was not reaped: waited=%jd child=%jd",
	    (intmax_t)waited, (intmax_t)child);
	ATF_REQUIRE_MSG(WIFEXITED(status) && WEXITSTATUS(status) == 0,
	    "fork-log child exited abnormally: status %#x", status);
	ATF_REQUIRE_MSG(failpoint == UINT_MAX,
	    "fork group-target failpoint was not consumed: value %u",
	    failpoint);

	inherited_group = -1;
	missed_group = -1;
	for (group = 0; group < FORK_LOG_NGROUPS; group++) {
		all_read = true;
		all_missing = true;
		for (member = 0; member < FORK_LOG_NMEMBERS; member++) {
			all_read = all_read &&
			    child_result.flcr_rc[group][member] == 0;
			all_missing = all_missing &&
			    child_result.flcr_rc[group][member] == -1 &&
			    child_result.flcr_errno[group][member] == ESRCH;
		}
		if (all_read)
			inherited_group = (int)group;
		else if (all_missing)
			missed_group = (int)group;
	}
	ATF_REQUIRE_MSG(inherited_group >= 0 && missed_group >= 0 &&
	    inherited_group != missed_group,
	    "fork failpoint did not produce one whole inherited group and one "
	    "whole missed group: g0=[%d/%d,%d/%d] g1=[%d/%d,%d/%d]",
	    child_result.flcr_rc[0][0], child_result.flcr_errno[0][0],
	    child_result.flcr_rc[0][1], child_result.flcr_errno[0][1],
	    child_result.flcr_rc[1][0], child_result.flcr_errno[1][0],
	    child_result.flcr_rc[1][1], child_result.flcr_errno[1][1]);
	ATF_CHECK_MSG(log_counts.flc_marker_seen &&
	    log_counts.flc_procfork == 1 &&
	    log_counts.flc_attach[missed_group][0] == 0 &&
	    log_counts.flc_attach[missed_group][1] == 0 &&
	    log_counts.flc_detach[missed_group][0] == 0 &&
	    log_counts.flc_detach[missed_group][1] == 0 &&
	    log_counts.flc_attach[inherited_group][0] == 1 &&
	    log_counts.flc_attach[inherited_group][1] == 1 &&
	    log_counts.flc_detach[inherited_group][0] == 0 &&
	    log_counts.flc_detach[inherited_group][1] == 0,
	    "fork-miss log mismatch: marker=%u PROCFORK=%u "
	    "missed attach=[%u,%u] detach=[%u,%u], "
	    "inherited attach=[%u,%u] detach=[%u,%u]",
	    log_counts.flc_marker_seen ? 1 : 0, log_counts.flc_procfork,
	    log_counts.flc_attach[missed_group][0],
	    log_counts.flc_attach[missed_group][1],
	    log_counts.flc_detach[missed_group][0],
	    log_counts.flc_detach[missed_group][1],
	    log_counts.flc_attach[inherited_group][0],
	    log_counts.flc_attach[inherited_group][1],
	    log_counts.flc_detach[inherited_group][0],
	    log_counts.flc_detach[inherited_group][1]);
}
ATF_TC_CLEANUP(descendants_fork_miss_has_no_false_detach, tc)
{

	reset_fork_test_hooks();
}

/*
 * Repeat the partial-inheritance case when the forking process is itself an
 * inherited target.  The first fork's handle-scoped attach records prove that
 * both groups reached the intermediate process.  The second fork must identify
 * its one failed group with a distinct leader-handle/child-pid record.
 */
ATF_TC_WITH_CLEANUP(descendants_inherited_parent_fork_miss_is_identified);
ATF_TC_HEAD(descendants_inherited_parent_fork_miss_is_identified, tc)
{

	atf_tc_set_md_var(tc, "require.user", "root");
}
ATF_TC_BODY(descendants_inherited_parent_fork_miss_is_identified, tc)
{
	struct inherited_fork_result result;
	struct inherited_fork_log_counts log_counts;
	struct live_target_counts initial, final;
	struct group groups[FORK_LOG_NGROUPS];
	struct pollfd pfd;
	char failure[1024], log_path[PATH_MAX], release;
	pmc_id_t leaders[FORK_LOG_NGROUPS];
	pid_t child, inherited_parent, waited;
	ssize_t nread;
	u_int failpoint, group, member;
	int inherited_group, missed_group;
	int log_fd, poll_errno, poll_rc, release_pipe[2], result_pipe[2];
	int saved_errno, status;
	bool all_missing, all_read, grandchild_ready, logfile_configured;

	for (group = 0; group < FORK_LOG_NGROUPS; group++) {
		group_init(&groups[group]);
		leaders[group] = PMC_ID_INVALID;
	}
	memset(&result, 0, sizeof(result));
	memset(&log_counts, 0, sizeof(log_counts));
	memset(&initial, 0, sizeof(initial));
	memset(&final, 0, sizeof(final));
	memset(&pfd, 0, sizeof(pfd));
	failure[0] = '\0';
	log_path[0] = '\0';
	child = -1;
	inherited_parent = -1;
	waited = -1;
	status = 0;
	failpoint = 0;
	log_fd = -1;
	release_pipe[0] = release_pipe[1] = -1;
	result_pipe[0] = result_pipe[1] = -1;
	grandchild_ready = false;
	logfile_configured = false;

	require_hwpmc();
	require_hwpmc_test_support();
	reset_fork_test_hooks();
	read_live_target_counts(&initial);

	log_fd = open_named_logfile(log_path, sizeof(log_path));
	if (log_fd < 0) {
		saved_errno = errno;
		log_path[0] = '\0';
		snprintf(failure, sizeof(failure),
		    "creating inherited-parent logfile failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	if (pmc_configure_logfile(log_fd) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "configuring inherited-parent logfile failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	logfile_configured = true;

	for (group = 0; group < FORK_LOG_NGROUPS; group++) {
		if (group_build(&groups[group], true) != 0) {
			saved_errno = errno;
			snprintf(failure, sizeof(failure),
			    "building inherited-parent group %u failed: "
			    "errno %d (%s)", group, saved_errno,
			    strerror(saved_errno));
			goto cleanup;
		}
		if (pmc_attach(groups[group].g_ids[0], getpid()) != 0) {
			saved_errno = errno;
			snprintf(failure, sizeof(failure),
			    "attaching inherited-parent group %u failed: "
			    "errno %d (%s)", group, saved_errno,
			    strerror(saved_errno));
			goto cleanup;
		}
		leaders[group] = groups[group].g_ids[0];
	}

	if (pipe(result_pipe) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "creating inherited-parent result pipe failed: "
		    "errno %d (%s)", saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	if (pipe(release_pipe) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "creating inherited-parent release pipe failed: "
		    "errno %d (%s)", saved_errno, strerror(saved_errno));
		goto cleanup;
	}

	inherited_parent = fork();
	if (inherited_parent < 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "forking inherited parent failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	if (inherited_parent == 0) {
		close(result_pipe[0]);
		close(release_pipe[1]);
		close(log_fd);
		memset(&result, 0, sizeof(result));
		if (sysctl_write_u32(TEST_FAIL_GROUP_TARGET_ALLOC, 1) != 0) {
			result.ifr_stage = INHERITED_FORK_RESULT_FAILPOINT;
			result.ifr_errno = errno;
			(void)write_full(result_pipe[1], &result,
			    sizeof(result));
			_exit(2);
		}

		child = fork();
		if (child < 0) {
			result.ifr_stage = INHERITED_FORK_RESULT_FORK;
			result.ifr_errno = errno;
			(void)write_full(result_pipe[1], &result,
			    sizeof(result));
			_exit(3);
		}
		if (child == 0) {
			result.ifr_stage = INHERITED_FORK_RESULT_CHILD;
			result.ifr_pid = getpid();
			for (group = 0; group < FORK_LOG_NGROUPS; group++) {
				for (member = 0; member < FORK_LOG_NMEMBERS;
				    member++) {
					errno = 0;
					result.ifr_reads.flcr_rc[group][member] =
					    pmc_read(groups[group].g_ids[member],
					    &result.ifr_reads.flcr_value[group][member]);
					result.ifr_reads.flcr_errno[group][member] =
					    errno;
				}
			}
			if (write_full(result_pipe[1], &result,
			    sizeof(result)) != 0)
				_exit(4);
			if (read_byte(release_pipe[0], &release) != 0)
				_exit(5);
			_exit(release == CHILD_EXIT ? 0 : 6);
		}

		close(result_pipe[1]);
		close(release_pipe[0]);
		do {
			waited = waitpid(child, &status, 0);
		} while (waited < 0 && errno == EINTR);
		if (waited != child)
			_exit(7);
		if (!WIFEXITED(status))
			_exit(8);
		_exit(WEXITSTATUS(status));
	}

	close(result_pipe[1]);
	result_pipe[1] = -1;
	close(release_pipe[0]);
	release_pipe[0] = -1;
	pfd.fd = result_pipe[0];
	pfd.events = POLLIN | POLLHUP;
	errno = 0;
	do {
		poll_rc = poll(&pfd, 1, 5000);
	} while (poll_rc == -1 && errno == EINTR);
	poll_errno = errno;
	if (poll_rc != 1) {
		snprintf(failure, sizeof(failure),
		    "inherited-parent result was not ready: poll rc=%d "
		    "errno=%d (%s)", poll_rc, poll_errno,
		    strerror(poll_errno));
		goto cleanup;
	}
	nread = read_full(result_pipe[0], &result, sizeof(result));
	if (nread != (ssize_t)sizeof(result)) {
		snprintf(failure, sizeof(failure),
		    "inherited-parent result read returned %zd of %zu bytes",
		    nread, sizeof(result));
		goto cleanup;
	}
	close(result_pipe[0]);
	result_pipe[0] = -1;
	if (result.ifr_stage != INHERITED_FORK_RESULT_CHILD) {
		snprintf(failure, sizeof(failure),
		    "inherited-parent setup failed at stage %d: errno %d (%s)",
		    result.ifr_stage, result.ifr_errno,
		    strerror(result.ifr_errno));
		goto cleanup;
	}
	child = result.ifr_pid;
	if (child <= 0) {
		snprintf(failure, sizeof(failure),
		    "inherited-parent result contained invalid child pid %jd",
		    (intmax_t)child);
		goto cleanup;
	}
	grandchild_ready = true;

	if (sysctl_read_u32(TEST_FAIL_GROUP_TARGET_ALLOC, &failpoint) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "reading inherited-parent failpoint failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	if (pmc_writelog(TEST_FORK_LOG_MARKER) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "writing inherited-parent marker failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	if (pmc_flush_logfile() != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "flushing inherited-parent marker failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	if (scan_inherited_fork_miss_log(log_path, inherited_parent, child,
	    groups, &log_counts) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "parsing inherited-parent logfile failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
		goto cleanup;
	}

	grandchild_ready = false;
	if (write_byte(release_pipe[1], CHILD_EXIT) != 0) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "releasing inherited grandchild failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
		goto cleanup;
	}
	close(release_pipe[1]);
	release_pipe[1] = -1;
	do {
		waited = waitpid(inherited_parent, &status, 0);
	} while (waited < 0 && errno == EINTR);
	if (waited != inherited_parent) {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "waiting for inherited parent failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
	}

cleanup:
	if (result_pipe[0] >= 0)
		(void)close(result_pipe[0]);
	if (result_pipe[1] >= 0)
		(void)close(result_pipe[1]);
	if (release_pipe[0] >= 0)
		(void)close(release_pipe[0]);
	if (release_pipe[1] >= 0) {
		if (grandchild_ready)
			(void)write_byte(release_pipe[1], CHILD_EXIT);
		(void)close(release_pipe[1]);
		release_pipe[1] = -1;
	}
	if (inherited_parent > 0 && waited != inherited_parent) {
		do {
			waited = waitpid(inherited_parent, &status, 0);
		} while (waited < 0 && errno == EINTR);
	}
	reset_fork_test_hooks();
	for (group = 0; group < FORK_LOG_NGROUPS; group++)
		group_teardown(&groups[group]);
	if (logfile_configured && pmc_configure_logfile(-1) != 0 &&
	    failure[0] == '\0') {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "deconfiguring inherited-parent logfile failed: "
		    "errno %d (%s)", saved_errno, strerror(saved_errno));
	}
	if (log_fd >= 0 && close(log_fd) != 0 && failure[0] == '\0') {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "closing inherited-parent logfile failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
	}
	if (log_path[0] != '\0' && unlink(log_path) != 0 &&
	    failure[0] == '\0') {
		saved_errno = errno;
		snprintf(failure, sizeof(failure),
		    "unlinking inherited-parent logfile failed: errno %d (%s)",
		    saved_errno, strerror(saved_errno));
	}
	read_live_target_counts(&final);
	check_live_counts_equal("inherited-parent fork-miss cleanup", &final,
	    &initial);

	if (failure[0] != '\0')
		atf_tc_fail("%s", failure);
	ATF_REQUIRE_MSG(waited == inherited_parent,
	    "inherited parent was not reaped: waited=%jd parent=%jd",
	    (intmax_t)waited, (intmax_t)inherited_parent);
	ATF_REQUIRE_MSG(WIFEXITED(status) && WEXITSTATUS(status) == 0,
	    "inherited parent exited abnormally: status %#x", status);
	ATF_REQUIRE_MSG(failpoint == UINT_MAX,
	    "inherited-parent failpoint was not consumed: value %u",
	    failpoint);

	inherited_group = -1;
	missed_group = -1;
	for (group = 0; group < FORK_LOG_NGROUPS; group++) {
		all_read = true;
		all_missing = true;
		for (member = 0; member < FORK_LOG_NMEMBERS; member++) {
			all_read = all_read &&
			    result.ifr_reads.flcr_rc[group][member] == 0;
			all_missing = all_missing &&
			    result.ifr_reads.flcr_rc[group][member] == -1 &&
			    result.ifr_reads.flcr_errno[group][member] == ESRCH;
		}
		if (all_read)
			inherited_group = (int)group;
		else if (all_missing)
			missed_group = (int)group;
	}
	ATF_REQUIRE_MSG(inherited_group >= 0 && missed_group >= 0 &&
	    inherited_group != missed_group,
	    "inherited-parent failpoint did not produce one whole inherited "
	    "group and one whole missed group: "
	    "g0=[%d/%d,%d/%d] g1=[%d/%d,%d/%d]",
	    result.ifr_reads.flcr_rc[0][0],
	    result.ifr_reads.flcr_errno[0][0],
	    result.ifr_reads.flcr_rc[0][1],
	    result.ifr_reads.flcr_errno[0][1],
	    result.ifr_reads.flcr_rc[1][0],
	    result.ifr_reads.flcr_errno[1][0],
	    result.ifr_reads.flcr_rc[1][1],
	    result.ifr_reads.flcr_errno[1][1]);
	ATF_REQUIRE_MSG(log_counts.iflc_marker_seen,
	    "inherited-parent marker was not found");
	ATF_REQUIRE_MSG(log_counts.iflc_procfork == 1,
	    "inherited-parent child PROCFORK count=%u, expected 1",
	    log_counts.iflc_procfork);
	for (group = 0; group < FORK_LOG_NGROUPS; group++) {
		for (member = 0; member < FORK_LOG_NMEMBERS; member++) {
			ATF_REQUIRE_MSG(
			    log_counts.iflc_parent_attach[group][member] == 1,
			    "intermediate target group %u member %u "
			    "PMCATTACH count=%u, expected 1", group, member,
			    log_counts.iflc_parent_attach[group][member]);
			ATF_REQUIRE_MSG(
			    log_counts.iflc_child_detach[group][member] == 0,
			    "grandchild group %u member %u PMCDETACH count=%u, "
			    "expected 0", group, member,
			    log_counts.iflc_child_detach[group][member]);
		}
	}
	for (member = 0; member < FORK_LOG_NMEMBERS; member++) {
		ATF_REQUIRE_MSG(
		    log_counts.iflc_child_attach[inherited_group][member] == 1,
		    "inherited grandchild member %u PMCATTACH count=%u, "
		    "expected 1", member,
		    log_counts.iflc_child_attach[inherited_group][member]);
		ATF_REQUIRE_MSG(
		    log_counts.iflc_child_attach[missed_group][member] == 0,
		    "missed grandchild member %u PMCATTACH count=%u, expected 0",
		    member,
		    log_counts.iflc_child_attach[missed_group][member]);
	}
	ATF_CHECK_MSG(log_counts.iflc_child_miss[missed_group] == 1 &&
	    log_counts.iflc_child_miss[inherited_group] == 0 &&
	    log_counts.iflc_unmatched_child_miss == 0,
	    "inherited-parent miss log mismatch: failed leader %#x miss=%u, "
	    "successful leader %#x miss=%u, unmatched=%u; expected [1,0,0]",
	    leaders[missed_group],
	    log_counts.iflc_child_miss[missed_group],
	    leaders[inherited_group],
	    log_counts.iflc_child_miss[inherited_group],
	    log_counts.iflc_unmatched_child_miss);
}
ATF_TC_CLEANUP(descendants_inherited_parent_fork_miss_is_identified, tc)
{

	reset_fork_test_hooks();
}

/*
 * Fail a later target in an existing process-tree attachment after the root
 * target has already been prepared.  Nothing may be published, and the same
 * group must remain reusable after the transaction rolls back.
 */
static void
descendants_attach_failure_atomic(const char *failpoint_name,
    u_int fail_after, int expected_errno, const char *failure_kind)
{
	struct live_target_counts after_failure, final, initial, prepared;
	struct group g;
	pid_t child, waited;
	u_int failpoint;
	int ack_fd, attach_errno, attach_rc, command_fd, retry_errno, retry_rc;
	int status, wait_errno;

	require_hwpmc();
	require_hwpmc_test_support();
	reset_fork_test_hooks();
	read_live_target_counts(&initial);

	group_build_required(&g, true);
	child = spawn_gated_child(&command_fd, &ack_fd);
	read_live_target_counts(&prepared);

	require_test_sysctl_write(failpoint_name, fail_after);
	errno = 0;
	attach_rc = pmc_attach(g.g_ids[0], getpid());
	attach_errno = errno;
	ATF_REQUIRE_MSG(sysctl_read_u32(failpoint_name, &failpoint) == 0,
	    "reading consumed %s failpoint failed: errno %d (%s)",
	    failure_kind, errno, strerror(errno));
	read_live_target_counts(&after_failure);

	reset_fork_test_hooks();
	retry_rc = -2;
	retry_errno = 0;
	if (attach_rc == -1) {
		errno = 0;
		retry_rc = pmc_attach(g.g_ids[0], getpid());
		retry_errno = errno;
	}

	ATF_REQUIRE_MSG(kill(child, SIGCONT) == 0,
	    "could not release parked child: errno %d (%s)", errno,
	    strerror(errno));
	run_child_phase(command_fd, ack_fd, CHILD_EXIT);
	close(command_fd);
	close(ack_fd);
	waited = waitpid(child, &status, 0);
	wait_errno = errno;

	group_teardown(&g);
	read_live_target_counts(&final);

	ATF_CHECK_MSG(attach_rc == -1 && attach_errno == expected_errno,
	    "%s returned rc=%d errno=%d (%s), expected -1/%d (%s)",
	    failure_kind, attach_rc, attach_errno, strerror(attach_errno),
	    expected_errno, strerror(expected_errno));
	ATF_CHECK_MSG(failpoint == UINT_MAX,
	    "%s failpoint was not consumed: value %u", failure_kind,
	    failpoint);
	check_live_counts_equal("failed descendants attach changed live counts",
	    &after_failure, &prepared);
	ATF_CHECK_MSG(retry_rc == 0,
	    "same-group retry after %s returned rc=%d errno=%d (%s)",
	    failure_kind, retry_rc, retry_errno, strerror(retry_errno));
	ATF_REQUIRE_MSG(waited == child,
	    "waitpid after %s failed: errno %d (%s)", failure_kind,
	    wait_errno, strerror(wait_errno));
	ATF_REQUIRE_MSG(WIFEXITED(status) && WEXITSTATUS(status) == 0,
	    "child after %s exited abnormally: status %#x", failure_kind,
	    status);
	check_live_counts_equal("rollback/retry cleanup did not restore baseline",
	    &final, &initial);
}

ATF_TC_WITH_CLEANUP(descendants_attach_alloc_rollback);
ATF_TC_HEAD(descendants_attach_alloc_rollback, tc)
{

	atf_tc_set_md_var(tc, "require.user", "root");
}
ATF_TC_BODY(descendants_attach_alloc_rollback, tc)
{

	descendants_attach_failure_atomic(TEST_FAIL_GROUP_TARGET_ALLOC, 1,
	    ENOMEM, "later group-target allocation failure");
}
ATF_TC_CLEANUP(descendants_attach_alloc_rollback, tc)
{

	reset_fork_test_hooks();
}

ATF_TC_WITH_CLEANUP(descendants_attach_authorization_rollback);
ATF_TC_HEAD(descendants_attach_authorization_rollback, tc)
{

	atf_tc_set_md_var(tc, "require.user", "root");
}
ATF_TC_BODY(descendants_attach_authorization_rollback, tc)
{

	descendants_attach_failure_atomic(TEST_FAIL_ATTACH_AUTHORIZATION, 1,
	    EPERM, "later target authorization failure");
}
ATF_TC_CLEANUP(descendants_attach_authorization_rollback, tc)
{

	reset_fork_test_hooks();
}

/*
 * Hold attachment after the target-tree snapshot while the only enumerated
 * target attempts to fork.  The process-tree lock makes that fork linearize
 * after publication, so the new child must inherit the complete group.
 */
ATF_TC_WITH_CLEANUP(descendants_attach_race_fork);
ATF_TC_HEAD(descendants_attach_race_fork, tc)
{

	atf_tc_set_md_var(tc, "require.user", "root");
}
ATF_TC_BODY(descendants_attach_race_fork, tc)
{
	struct attach_thread_result attach;
	struct child_read_result child_read;
	struct group g;
	struct live_target_counts final, initial;
	pthread_t attach_thread;
	pid_t target_parent, waited;
	ssize_t nread;
	char ready;
	u_int paused_targets;
	int ack_pipe[2], child_command_pipe[2], parent_command_pipe[2];
	int result_pipe[2], status, thread_error, wait_errno;

	require_hwpmc();
	require_hwpmc_test_support();
	reset_fork_test_hooks();
	read_live_target_counts(&initial);
	group_build_required(&g, true);
	ATF_REQUIRE_MSG(pipe(parent_command_pipe) == 0,
	    "target-parent command pipe failed: %s", strerror(errno));
	ATF_REQUIRE_MSG(pipe(child_command_pipe) == 0,
	    "descendant command pipe failed: %s", strerror(errno));
	ATF_REQUIRE_MSG(pipe(ack_pipe) == 0,
	    "target acknowledgment pipe failed: %s", strerror(errno));
	ATF_REQUIRE_MSG(pipe(result_pipe) == 0,
	    "descendant result pipe failed: %s", strerror(errno));

	target_parent = fork();
	ATF_REQUIRE_MSG(target_parent != -1, "target fork failed: %s",
	    strerror(errno));
	if (target_parent == 0) {
		close(parent_command_pipe[1]);
		close(child_command_pipe[1]);
		close(ack_pipe[0]);
		close(result_pipe[0]);
		target_parent_with_child(parent_command_pipe[0],
		    child_command_pipe[0], ack_pipe[1], result_pipe[1],
		    g.g_ids[0], g.g_ids[1]);
	}

	close(parent_command_pipe[0]);
	close(child_command_pipe[0]);
	close(ack_pipe[1]);
	close(result_pipe[1]);
	ATF_REQUIRE_MSG(read_byte(ack_pipe[0], &ready) == 0,
	    "target readiness read failed: %s", strerror(errno));
	ATF_REQUIRE_EQ(ready, TARGET_READY);

	require_test_sysctl_write(TEST_PAUSE_DESCENDANTS_ATTACH, g.g_id);
	memset(&attach, 0, sizeof(attach));
	attach.atr_leader = g.g_ids[0];
	attach.atr_target = target_parent;
	thread_error = pthread_create(&attach_thread, NULL, attach_group_thread,
	    &attach);
	ATF_REQUIRE_MSG(thread_error == 0,
	    "creating attach thread failed: %s", strerror(thread_error));
	wait_test_sysctl_ack(TEST_PAUSE_DESCENDANTS_ATTACH_ACK, g.g_id);
	ATF_REQUIRE_MSG(sysctl_read_u32(TEST_PAUSE_DESCENDANTS_TARGET_COUNT,
	    &paused_targets) == 0,
	    "reading paused target count failed: errno %d (%s)", errno,
	    strerror(errno));

	ATF_REQUIRE_MSG(write_byte(parent_command_pipe[1], TARGET_FORK) == 0,
	    "requesting target fork failed: %s", strerror(errno));
	ATF_REQUIRE_MSG(read_byte(ack_pipe[0], &ready) == 0,
	    "fork-attempt acknowledgment failed: %s", strerror(errno));
	ATF_REQUIRE_EQ(ready, TARGET_FORKING);
	require_test_sysctl_write(TEST_PAUSE_DESCENDANTS_ATTACH, 0);
	thread_error = pthread_join(attach_thread, NULL);
	ATF_REQUIRE_MSG(thread_error == 0,
	    "joining attach thread failed: %s", strerror(thread_error));

	ATF_REQUIRE_MSG(read_byte(ack_pipe[0], &ready) == 0,
	    "descendant readiness read failed: %s", strerror(errno));
	ATF_REQUIRE_EQ(ready, CHILD_READY);
	memset(&child_read, 0, sizeof(child_read));
	nread = read_full(result_pipe[0], &child_read, sizeof(child_read));
	close(result_pipe[0]);

	run_child_phase(child_command_pipe[1], ack_pipe[0], CHILD_EXIT);
	close(child_command_pipe[1]);
	ATF_REQUIRE_MSG(write_byte(parent_command_pipe[1], TARGET_EXIT) == 0,
	    "releasing target parent failed: %s", strerror(errno));
	close(parent_command_pipe[1]);
	waited = waitpid(target_parent, &status, 0);
	wait_errno = errno;
	require_pipe_eof(ack_pipe[0], "fork-race target tree exit");
	close(ack_pipe[0]);

	group_teardown(&g);
	reset_fork_test_hooks();
	read_live_target_counts(&final);

	ATF_CHECK_MSG(paused_targets == 1,
	    "fork race paused with %u targets, expected only the parent",
	    paused_targets);
	ATF_CHECK_MSG(attach.atr_rc == 0,
	    "fork-race attach returned rc=%d errno=%d (%s)",
	    attach.atr_rc, attach.atr_errno, strerror(attach.atr_errno));
	ATF_CHECK_MSG(nread == (ssize_t)sizeof(child_read),
	    "fork-race child result read returned %zd of %zu bytes", nread,
	    sizeof(child_read));
	ATF_CHECK_MSG(child_read.cr_leader_rc == 0,
	    "post-linearization child leader read failed: errno %d (%s)",
	    child_read.cr_leader_errno,
	    strerror(child_read.cr_leader_errno));
	ATF_CHECK_MSG(child_read.cr_sibling_rc == 0,
	    "post-linearization child sibling read failed: errno %d (%s)",
	    child_read.cr_sibling_errno,
	    strerror(child_read.cr_sibling_errno));
	ATF_REQUIRE_MSG(waited == target_parent,
	    "waitpid for fork-race target failed: errno %d (%s)",
	    wait_errno, strerror(wait_errno));
	ATF_REQUIRE_MSG(WIFEXITED(status) && WEXITSTATUS(status) == 0,
	    "fork-race target exited abnormally: status %#x", status);
	check_live_counts_equal("fork-race cleanup did not restore baseline",
	    &final, &initial);
}
ATF_TC_CLEANUP(descendants_attach_race_fork, tc)
{

	reset_fork_test_hooks();
}

/*
 * Hold attachment after enumerating the owner and one parked child.  Make the
 * child enter pmc_process_exit() and snapshot P_HWPMC before publication, then
 * release the transaction.  Exit cleanup must still remove the child's newly
 * published edge while leaving the owner's edge intact.
 */
ATF_TC_WITH_CLEANUP(descendants_attach_race_exit);
ATF_TC_HEAD(descendants_attach_race_exit, tc)
{

	atf_tc_set_md_var(tc, "require.user", "root");
}
ATF_TC_BODY(descendants_attach_race_exit, tc)
{
	struct attach_thread_result attach;
	struct group g;
	struct live_target_counts after_exit, final, initial;
	pthread_t attach_thread;
	pid_t child, waited;
	uint32_t group_id;
	u_int owner_links, pause_request, paused_targets;
	int ack_fd, command_fd, status, thread_error, wait_errno;

	require_hwpmc();
	require_hwpmc_test_support();
	reset_fork_test_hooks();
	read_live_target_counts(&initial);
	group_build_required(&g, true);
	group_id = g.g_id;
	child = spawn_gated_child(&command_fd, &ack_fd);

	require_test_sysctl_write(TEST_PAUSE_DESCENDANTS_ATTACH, group_id);
	memset(&attach, 0, sizeof(attach));
	attach.atr_leader = g.g_ids[0];
	attach.atr_target = getpid();
	thread_error = pthread_create(&attach_thread, NULL, attach_group_thread,
	    &attach);
	ATF_REQUIRE_MSG(thread_error == 0,
	    "creating attach thread failed: %s", strerror(thread_error));
	wait_test_sysctl_ack(TEST_PAUSE_DESCENDANTS_ATTACH_ACK, group_id);
	ATF_REQUIRE_MSG(sysctl_read_u32(TEST_PAUSE_DESCENDANTS_TARGET_COUNT,
	    &paused_targets) == 0,
	    "reading paused target count failed: errno %d (%s)", errno,
	    strerror(errno));

	require_test_sysctl_write(TEST_DESCENDANTS_EXIT_PID, (u_int)child);
	ATF_REQUIRE_MSG(kill(child, SIGCONT) == 0,
	    "could not release parked child: errno %d (%s)", errno,
	    strerror(errno));
	run_child_phase(command_fd, ack_fd, CHILD_EXIT);
	close(command_fd);
	wait_test_sysctl_ack(TEST_DESCENDANTS_EXIT_ACK, (u_int)child);
	ATF_REQUIRE_MSG(sysctl_read_u32(TEST_PAUSE_DESCENDANTS_ATTACH,
	    &pause_request) == 0,
	    "reading descendants pause request failed: errno %d (%s)", errno,
	    strerror(errno));

	require_test_sysctl_write(TEST_PAUSE_DESCENDANTS_ATTACH, 0);
	thread_error = pthread_join(attach_thread, NULL);
	ATF_REQUIRE_MSG(thread_error == 0,
	    "joining attach thread failed: %s", strerror(thread_error));
	require_pipe_eof(ack_fd, "exit-race child");
	close(ack_fd);
	read_live_target_counts(&after_exit);

	/*
	 * The committed non-MUX group is assigned at commit, so the still-attached
	 * owner holds one ordinary row-link per member.  Capture that count before
	 * teardown clears g.g_n for the after_exit assertion below.
	 */
	owner_links = g.g_n;

	/*
	 * Release before waitpid() reaps the child, so a failing implementation
	 * can remove a stale edge without dereferencing a freed process.
	 */
	group_teardown(&g);
	reset_fork_test_hooks();
	waited = waitpid(child, &status, 0);
	wait_errno = errno;
	read_live_target_counts(&final);

	ATF_CHECK_MSG(paused_targets == 2,
	    "exit race paused with %u targets, expected owner and child",
	    paused_targets);
	ATF_CHECK_MSG(pause_request == group_id,
	    "descendants pause ended before child exit reached hwpmc: %u/%u",
	    pause_request, group_id);
	ATF_CHECK_MSG(attach.atr_rc == 0,
	    "exit-race attach returned rc=%d errno=%d (%s)", attach.atr_rc,
	    attach.atr_errno, strerror(attach.atr_errno));
	/*
	 * Only the owner's edge and descriptor survive the child's exit-race
	 * cleanup (group_target and process each initial+1).  A committed
	 * non-MUX group is assigned hardware rows at commit, so the still-attached
	 * owner legitimately holds one ordinary row-link per member; the child's
	 * links (the pre-fix leak this case targets) must be gone, leaving exactly
	 * initial + g.g_n.  A failing implementation that stranded the child's
	 * links would show initial + 2*g.g_n and still fail here.
	 */
	ATF_CHECK_MSG(
	    after_exit.ltc_pmc_targets == initial.ltc_pmc_targets + owner_links &&
	    after_exit.ltc_group_targets == initial.ltc_group_targets + 1 &&
	    after_exit.ltc_target_processes ==
	    initial.ltc_target_processes + 1 &&
	    after_exit.ltc_residual_entries == initial.ltc_residual_entries &&
	    after_exit.ltc_rotation_refs == initial.ltc_rotation_refs &&
	    after_exit.ltc_run_refs == initial.ltc_run_refs,
	    "exit cleanup left stale state before release: pmc_target %ju/%ju "
	    "group_target %ju/%ju process %ju/%ju residual %ju/%ju "
	    "rotation_ref %ju/%ju run_ref %ju/%ju",
	    (uintmax_t)after_exit.ltc_pmc_targets,
	    (uintmax_t)(initial.ltc_pmc_targets + owner_links),
	    (uintmax_t)after_exit.ltc_group_targets,
	    (uintmax_t)(initial.ltc_group_targets + 1),
	    (uintmax_t)after_exit.ltc_target_processes,
	    (uintmax_t)(initial.ltc_target_processes + 1),
	    (uintmax_t)after_exit.ltc_residual_entries,
	    (uintmax_t)initial.ltc_residual_entries,
	    (uintmax_t)after_exit.ltc_rotation_refs,
	    (uintmax_t)initial.ltc_rotation_refs,
	    (uintmax_t)after_exit.ltc_run_refs,
	    (uintmax_t)initial.ltc_run_refs);
	ATF_REQUIRE_MSG(waited == child,
	    "waitpid for exit-race child failed: errno %d (%s)", wait_errno,
	    strerror(wait_errno));
	ATF_REQUIRE_MSG(WIFEXITED(status) && WEXITSTATUS(status) == 0,
	    "exit-race child exited abnormally: status %#x", status);
	check_live_counts_equal("exit-race cleanup did not restore baseline",
	    &final, &initial);
}
ATF_TC_CLEANUP(descendants_attach_race_exit, tc)
{

	reset_fork_test_hooks();
}

/*
 * The explicit target is only one member of the authoritative target set.
 * Its exit must not stop a running group while an inherited child remains.
 * Keep the group deterministically evicted through the explicit target's exit
 * so the red test isolates anchor lifetime without exercising stale row links.
 * After a successful handoff, make the group resident and prove that the
 * surviving child contributes.
 */
ATF_TC_WITH_CLEANUP(descendants_parent_target_exits_before_child);
ATF_TC_HEAD(descendants_parent_target_exits_before_child, tc)
{

	atf_tc_set_md_var(tc, "require.user", "root");
}
ATF_TC_BODY(descendants_parent_target_exits_before_child, tc)
{
	struct child_read_result child_read;
	struct group g;
	struct live_target_counts child_gone, final, initial;
	struct row_occupancy occupancy;
	pmc_value_t post_exit_before, post_exit_control, post_exit_work;
	pmc_value_t control_delta, work_delta;
	pid_t target_parent;
	char ready, red_failure[256];
	int ack_pipe[2], child_command_pipe[2], parent_command_pipe[2];
	int result_pipe[2], restart_errno, restart_rc, status;

	red_failure[0] = '\0';
	require_hwpmc();
	require_hwpmc_test_support();
	reset_fork_test_hooks();
	read_live_target_counts(&initial);
	row_occupancy_fill_required(&occupancy);
	group_build_one_required(&g, true);
	require_test_sysctl_write(TEST_HOLD_GROUP_EVICTED, g.g_id);
	ATF_REQUIRE_MSG(pipe(parent_command_pipe) == 0,
	    "target-parent command pipe failed: %s", strerror(errno));
	ATF_REQUIRE_MSG(pipe(child_command_pipe) == 0,
	    "descendant command pipe failed: %s", strerror(errno));
	ATF_REQUIRE_MSG(pipe(ack_pipe) == 0,
	    "target acknowledgment pipe failed: %s", strerror(errno));
	ATF_REQUIRE_MSG(pipe(result_pipe) == 0,
	    "descendant result pipe failed: %s", strerror(errno));

	target_parent = fork();
	ATF_REQUIRE_MSG(target_parent != -1, "target fork failed: %s",
	    strerror(errno));
	if (target_parent == 0) {
		close(parent_command_pipe[1]);
		close(child_command_pipe[1]);
		close(ack_pipe[0]);
		close(result_pipe[0]);
		target_parent_with_child(parent_command_pipe[0],
		    child_command_pipe[0], ack_pipe[1], result_pipe[1],
		    g.g_ids[0], PMC_ID_INVALID);
	}

	close(parent_command_pipe[0]);
	close(child_command_pipe[0]);
	close(ack_pipe[1]);
	close(result_pipe[1]);
	ATF_REQUIRE_MSG(read_byte(ack_pipe[0], &ready) == 0,
	    "target readiness read failed: %s", strerror(errno));
	ATF_REQUIRE_EQ(ready, TARGET_READY);

	ATF_REQUIRE_MSG(pmc_attach(g.g_ids[0], target_parent) == 0,
	    "group attach to target parent failed: %s", strerror(errno));
	ATF_REQUIRE_MSG(pmc_start(g.g_ids[0]) == 0,
	    "group start failed: %s", strerror(errno));
	g.g_started = true;
	wait_test_sysctl_ack(TEST_HOLD_GROUP_EVICTED_ACK, g.g_id);

	ATF_REQUIRE_MSG(write_byte(parent_command_pipe[1], TARGET_FORK) == 0,
	    "requesting descendant fork failed: %s", strerror(errno));
	ATF_REQUIRE_MSG(read_byte(ack_pipe[0], &ready) == 0,
	    "descendant fork-attempt read failed: %s", strerror(errno));
	ATF_REQUIRE_EQ(ready, TARGET_FORKING);
	ATF_REQUIRE_MSG(read_byte(ack_pipe[0], &ready) == 0,
	    "descendant readiness read failed: %s", strerror(errno));
	ATF_REQUIRE_EQ(ready, CHILD_READY);
	ATF_REQUIRE_MSG(read_full(result_pipe[0], &child_read,
	    sizeof(child_read)) == (ssize_t)sizeof(child_read),
	    "descendant handle-read result was incomplete");
	close(result_pipe[0]);
	ATF_REQUIRE_MSG(child_read.cr_leader_rc == 0,
	    "descendant could not resolve the inherited leader before anchor "
	    "exit: errno %d (%s)", child_read.cr_leader_errno,
	    strerror(child_read.cr_leader_errno));

	ATF_REQUIRE_MSG(write_byte(parent_command_pipe[1], TARGET_EXIT) == 0,
	    "releasing target parent failed: %s", strerror(errno));
	close(parent_command_pipe[1]);
	ATF_REQUIRE_MSG(waitpid(target_parent, &status, 0) == target_parent,
	    "wait for target parent failed: %s", strerror(errno));
	ATF_REQUIRE_MSG(WIFEXITED(status) && WEXITSTATUS(status) == 0,
	    "target parent exited abnormally: status %#x", status);

	/*
	 * Pre-fix, parent exit stops the group and drops its only scheduling
	 * anchor.  A fresh start sees no attached target plus PMC_F_ATTACH_DONE
	 * and fails ESRCH at the automatic-attach gate before the group-layer
	 * target lookup.  With anchor handoff, the group is still logically
	 * running and start is the documented successful no-op.
	 */
	errno = 0;
	restart_rc = pmc_start(g.g_ids[0]);
	restart_errno = errno;
	if (restart_rc != 0) {
		snprintf(red_failure, sizeof(red_failure),
		    "group lost its live descendant anchor after parent exit: "
		    "pmc_start rc=%d errno=%d (%s), expected success",
		    restart_rc, restart_errno, strerror(restart_errno));
		goto cleanup;
	}

	require_test_sysctl_write(TEST_HOLD_GROUP_RESIDENT, g.g_id);
	require_test_sysctl_write(TEST_HOLD_GROUP_EVICTED, 0);
	row_occupancy_release_required(&occupancy);
	wait_test_sysctl_ack(TEST_HOLD_GROUP_RESIDENT_ACK, g.g_id);

	post_exit_before = leader_value(&g);
	run_child_phase(child_command_pipe[1], ack_pipe[0], CHILD_CONTROL);
	post_exit_control = leader_value(&g);
	run_child_phase(child_command_pipe[1], ack_pipe[0], CHILD_WORK);
	post_exit_work = leader_value(&g);
	ATF_REQUIRE_MSG(post_exit_control >= post_exit_before &&
	    post_exit_work >= post_exit_control,
	    "post-parent-exit count regressed: %ju -> %ju -> %ju",
	    (uintmax_t)post_exit_before, (uintmax_t)post_exit_control,
	    (uintmax_t)post_exit_work);
	control_delta = post_exit_control - post_exit_before;
	work_delta = post_exit_work - post_exit_control;
	ATF_CHECK_MSG(work_delta >= CHILD_MIN_DELTA &&
	    work_delta / 8 > control_delta,
	    "inherited child stopped contributing after its target parent "
	    "exited: work %ju control %ju", (uintmax_t)work_delta,
	    (uintmax_t)control_delta);

cleanup:
	run_child_phase(child_command_pipe[1], ack_pipe[0], CHILD_EXIT);
	close(child_command_pipe[1]);
	require_pipe_eof(ack_pipe[0], "descendant exit");
	close(ack_pipe[0]);
	read_live_target_counts(&child_gone);
	if (occupancy.ro_n != 0)
		row_occupancy_release_required(&occupancy);
	group_teardown(&g);
	reset_fork_test_hooks();
	read_live_target_counts(&final);

	check_live_counts_equal("last target exit did not restore live counts",
	    &child_gone, &initial);
	check_live_counts_equal("group release did not preserve live-count baseline",
	    &final, &initial);
	if (red_failure[0] != '\0')
		atf_tc_fail("%s", red_failure);
}
ATF_TC_CLEANUP(descendants_parent_target_exits_before_child, tc)
{

	reset_fork_test_hooks();
}

static void
release_running_group_with_live_child(bool resident)
{
	struct child_read_result child_read;
	struct group g;
	struct live_target_counts after_release, final, initial;
	struct row_occupancy occupancy;
	pid_t target_parent;
	char ready;
	int ack_pipe[2], child_command_pipe[2], parent_command_pipe[2];
	int release_errno, release_rc, result_pipe[2], status;

	require_hwpmc();
	require_hwpmc_test_support();
	reset_fork_test_hooks();
	read_live_target_counts(&initial);
	row_occupancy_init(&occupancy);
	if (!resident)
		row_occupancy_fill_required(&occupancy);
	group_build_with_mux_required(&g, true, true);
	require_test_sysctl_write(resident ? TEST_HOLD_GROUP_RESIDENT :
	    TEST_HOLD_GROUP_EVICTED, g.g_id);

	ATF_REQUIRE_MSG(pipe(parent_command_pipe) == 0,
	    "target-parent command pipe failed: %s", strerror(errno));
	ATF_REQUIRE_MSG(pipe(child_command_pipe) == 0,
	    "descendant command pipe failed: %s", strerror(errno));
	ATF_REQUIRE_MSG(pipe(ack_pipe) == 0,
	    "target acknowledgment pipe failed: %s", strerror(errno));
	ATF_REQUIRE_MSG(pipe(result_pipe) == 0,
	    "descendant result pipe failed: %s", strerror(errno));

	target_parent = fork();
	ATF_REQUIRE_MSG(target_parent != -1, "target fork failed: %s",
	    strerror(errno));
	if (target_parent == 0) {
		close(parent_command_pipe[1]);
		close(child_command_pipe[1]);
		close(ack_pipe[0]);
		close(result_pipe[0]);
		target_parent_with_child(parent_command_pipe[0],
		    child_command_pipe[0], ack_pipe[1], result_pipe[1],
		    g.g_ids[0], g.g_ids[1]);
	}

	close(parent_command_pipe[0]);
	close(child_command_pipe[0]);
	close(ack_pipe[1]);
	close(result_pipe[1]);
	ATF_REQUIRE_MSG(read_byte(ack_pipe[0], &ready) == 0,
	    "target readiness read failed: %s", strerror(errno));
	ATF_REQUIRE_EQ(ready, TARGET_READY);
	ATF_REQUIRE_MSG(pmc_attach(g.g_ids[0], target_parent) == 0,
	    "group attach failed: %s", strerror(errno));
	ATF_REQUIRE_MSG(pmc_start(g.g_ids[0]) == 0,
	    "group start failed: %s", strerror(errno));
	g.g_started = true;
	wait_test_sysctl_ack(resident ? TEST_HOLD_GROUP_RESIDENT_ACK :
	    TEST_HOLD_GROUP_EVICTED_ACK, g.g_id);

	ATF_REQUIRE_MSG(write_byte(parent_command_pipe[1], TARGET_FORK) == 0,
	    "requesting descendant fork failed: %s", strerror(errno));
	ATF_REQUIRE_MSG(read_byte(ack_pipe[0], &ready) == 0,
	    "descendant fork-attempt read failed: %s", strerror(errno));
	ATF_REQUIRE_EQ(ready, TARGET_FORKING);
	ATF_REQUIRE_MSG(read_byte(ack_pipe[0], &ready) == 0,
	    "descendant readiness read failed: %s", strerror(errno));
	ATF_REQUIRE_EQ(ready, CHILD_READY);
	ATF_REQUIRE_MSG(read_full(result_pipe[0], &child_read,
	    sizeof(child_read)) == (ssize_t)sizeof(child_read),
	    "descendant handle-read result was incomplete");
	close(result_pipe[0]);
	ATF_REQUIRE_MSG(child_read.cr_leader_rc == 0,
	    "live child could not read leader before release: errno %d (%s)",
	    child_read.cr_leader_errno, strerror(child_read.cr_leader_errno));
	ATF_REQUIRE_MSG(child_read.cr_sibling_rc == 0,
	    "live child could not read sibling before release: errno %d (%s)",
	    child_read.cr_sibling_errno, strerror(child_read.cr_sibling_errno));

	errno = 0;
	release_rc = pmc_release(g.g_ids[0]);
	release_errno = errno;
	if (release_rc == 0)
		group_init(&g);
	read_live_target_counts(&after_release);

	run_child_phase(child_command_pipe[1], ack_pipe[0], CHILD_EXIT);
	close(child_command_pipe[1]);
	ATF_REQUIRE_MSG(write_byte(parent_command_pipe[1], TARGET_EXIT) == 0,
	    "releasing target parent failed: %s", strerror(errno));
	close(parent_command_pipe[1]);
	ATF_REQUIRE_MSG(waitpid(target_parent, &status, 0) == target_parent,
	    "wait for target parent failed: %s", strerror(errno));
	ATF_REQUIRE_MSG(WIFEXITED(status) && WEXITSTATUS(status) == 0,
	    "target parent exited abnormally: status %#x", status);
	require_pipe_eof(ack_pipe[0], "release descendants");
	close(ack_pipe[0]);
	if (occupancy.ro_n != 0)
		row_occupancy_release_required(&occupancy);
	reset_fork_test_hooks();
	if (release_rc != 0)
		group_teardown(&g);
	read_live_target_counts(&final);

	ATF_REQUIRE_MSG(release_rc == 0,
	    "direct release while %s returned rc=%d errno=%d (%s)",
	    resident ? "resident" : "evicted", release_rc, release_errno,
	    strerror(release_errno));
	check_live_counts_equal("release with live child retained group state",
	    &after_release, &initial);
	check_live_counts_equal("release with live child cleanup did not restore "
	    "baseline", &final, &initial);
}

ATF_TC_WITH_CLEANUP(descendants_release_running_resident_with_live_child);
ATF_TC_HEAD(descendants_release_running_resident_with_live_child, tc)
{

	atf_tc_set_md_var(tc, "require.user", "root");
}
ATF_TC_BODY(descendants_release_running_resident_with_live_child, tc)
{

	release_running_group_with_live_child(true);
}
ATF_TC_CLEANUP(descendants_release_running_resident_with_live_child, tc)
{

	reset_fork_test_hooks();
}

ATF_TC_WITH_CLEANUP(descendants_release_running_evicted_with_live_child);
ATF_TC_HEAD(descendants_release_running_evicted_with_live_child, tc)
{

	atf_tc_set_md_var(tc, "require.user", "root");
}
ATF_TC_BODY(descendants_release_running_evicted_with_live_child, tc)
{

	release_running_group_with_live_child(false);
}
ATF_TC_CLEANUP(descendants_release_running_evicted_with_live_child, tc)
{

	reset_fork_test_hooks();
}

/*
 * A descendants attach is one group-level transaction over the existing
 * process tree.  Park the child before attach, then isolate a no-work control
 * and a child-only work phase while the child remains alive.  Every member
 * must show a substantial work-phase delta above the control noise.
 */
ATF_TC_WITHOUT_HEAD(descendants_existing_tree_attach_atomic);
ATF_TC_BODY(descendants_existing_tree_attach_atomic, tc)
{
	struct group g;
	pmc_value_t before[2], control[2], live_child[2], after_exit[2];
	pmc_value_t control_delta, work_delta;
	pid_t child;
	int ack_fd, attach_errno, command_fd, error, status;
	u_int i;

	require_hwpmc();
	group_build_required(&g, true);
	child = spawn_gated_child(&command_fd, &ack_fd);

	errno = 0;
	error = pmc_attach(g.g_ids[0], getpid());
	attach_errno = errno;
	ATF_REQUIRE_MSG(kill(child, SIGCONT) == 0,
	    "could not release parked child: %s", strerror(errno));
	ATF_REQUIRE_MSG(error == 0,
	    "existing-descendants attach failed at pmc_attach: errno %d (%s)",
	    attach_errno, strerror(attach_errno));
	ATF_REQUIRE_MSG(pmc_start(g.g_ids[0]) == 0,
	    "group start failed: %s", strerror(errno));
	g.g_started = true;

	group_snapshot(&g, before);
	run_child_phase(command_fd, ack_fd, CHILD_CONTROL);
	group_snapshot(&g, control);
	run_child_phase(command_fd, ack_fd, CHILD_WORK);
	group_snapshot(&g, live_child);
	ATF_REQUIRE_MSG(waitpid(child, &status, WNOHANG) == 0,
	    "child exited before the live-child snapshot");
	run_child_phase(command_fd, ack_fd, CHILD_EXIT);
	close(command_fd);
	close(ack_fd);
	ATF_REQUIRE_MSG(waitpid(child, &status, 0) == child,
	    "waitpid failed: %s", strerror(errno));
	ATF_REQUIRE_MSG(WIFEXITED(status) && WEXITSTATUS(status) == 0,
	    "child exited abnormally: status %#x", status);
	group_snapshot(&g, after_exit);

	for (i = 0; i < g.g_n; i++) {
		ATF_REQUIRE_MSG(control[i] >= before[i],
		    "member %u regressed during control: %ju -> %ju", i,
		    (uintmax_t)before[i], (uintmax_t)control[i]);
		ATF_REQUIRE_MSG(live_child[i] >= control[i],
		    "member %u regressed during child work: %ju -> %ju", i,
		    (uintmax_t)control[i], (uintmax_t)live_child[i]);
		ATF_REQUIRE_MSG(after_exit[i] >= live_child[i],
		    "member %u regressed across child exit: %ju -> %ju", i,
		    (uintmax_t)live_child[i], (uintmax_t)after_exit[i]);
		control_delta = control[i] - before[i];
		work_delta = live_child[i] - control[i];
		printf("member %u: control=%ju child-work=%ju post-exit=%ju\n", i,
		    (uintmax_t)control_delta, (uintmax_t)work_delta,
		    (uintmax_t)(after_exit[i] - live_child[i]));
		ATF_CHECK_MSG(work_delta >= CHILD_MIN_DELTA &&
		    work_delta / 8 > control_delta,
		    "member %u child-work delta %ju is not substantially above "
		    "control delta %ju", i, (uintmax_t)work_delta,
		    (uintmax_t)control_delta);
	}

	ATF_REQUIRE_MSG(pmc_stop(g.g_ids[0]) == 0,
	    "group stop failed: %s", strerror(errno));
	g.g_started = false;
	ATF_REQUIRE_MSG(pmc_release(g.g_ids[0]) == 0,
	    "group release failed: %s", strerror(errno));
	group_init(&g);
}

/*
 * Repeat the existing-tree transaction with no hardware row available at
 * commit or start.  Attachment must publish the full stable target set while
 * the group is deferred, and every member must begin counting the parked child
 * after the group is deterministically admitted.
 */
ATF_TC_WITH_CLEANUP(descendants_existing_tree_attach_deferred);
ATF_TC_HEAD(descendants_existing_tree_attach_deferred, tc)
{

	atf_tc_set_md_var(tc, "require.user", "root");
}
ATF_TC_BODY(descendants_existing_tree_attach_deferred, tc)
{
	struct group g;
	struct live_target_counts final, initial;
	struct row_occupancy occupancy;
	pmc_value_t after_exit[2], before[2], control[2], live_child[2];
	pmc_value_t control_delta, work_delta;
	pid_t child;
	int ack_fd, attach_errno, attach_rc, command_fd, status;
	u_int i;

	require_hwpmc();
	require_hwpmc_test_support();
	reset_fork_test_hooks();
	read_live_target_counts(&initial);
	row_occupancy_fill_required(&occupancy);
	group_build_with_mux_required(&g, true, true);
	require_test_sysctl_write(TEST_HOLD_GROUP_EVICTED, g.g_id);
	child = spawn_gated_child(&command_fd, &ack_fd);

	errno = 0;
	attach_rc = pmc_attach(g.g_ids[0], getpid());
	attach_errno = errno;
	if (attach_rc != 0) {
		ATF_REQUIRE_MSG(kill(child, SIGCONT) == 0,
		    "could not release child after attach failure: %s",
		    strerror(errno));
		run_child_phase(command_fd, ack_fd, CHILD_EXIT);
		close(command_fd);
		close(ack_fd);
		ATF_REQUIRE_MSG(waitpid(child, &status, 0) == child,
		    "wait after attach failure failed: %s", strerror(errno));
		row_occupancy_release_required(&occupancy);
		group_teardown(&g);
		reset_fork_test_hooks();
		atf_tc_fail("deferred existing-descendants attach returned "
		    "rc=%d errno=%d (%s), expected success", attach_rc,
		    attach_errno, strerror(attach_errno));
	}

	ATF_REQUIRE_MSG(pmc_start(g.g_ids[0]) == 0,
	    "deferred group start failed: errno %d (%s)", errno,
	    strerror(errno));
	g.g_started = true;
	wait_test_sysctl_ack(TEST_HOLD_GROUP_EVICTED_ACK, g.g_id);
	require_test_sysctl_write(TEST_HOLD_GROUP_RESIDENT, g.g_id);
	require_test_sysctl_write(TEST_HOLD_GROUP_EVICTED, 0);
	row_occupancy_release_required(&occupancy);
	wait_test_sysctl_ack(TEST_HOLD_GROUP_RESIDENT_ACK, g.g_id);

	ATF_REQUIRE_MSG(kill(child, SIGCONT) == 0,
	    "could not release parked child: %s", strerror(errno));
	group_snapshot(&g, before);
	run_child_phase(command_fd, ack_fd, CHILD_CONTROL);
	group_snapshot(&g, control);
	run_child_phase(command_fd, ack_fd, CHILD_WORK);
	group_snapshot(&g, live_child);
	ATF_REQUIRE_MSG(waitpid(child, &status, WNOHANG) == 0,
	    "child exited before the deferred live-child snapshot");
	run_child_phase(command_fd, ack_fd, CHILD_EXIT);
	close(command_fd);
	close(ack_fd);
	ATF_REQUIRE_MSG(waitpid(child, &status, 0) == child,
	    "waitpid failed: %s", strerror(errno));
	ATF_REQUIRE_MSG(WIFEXITED(status) && WEXITSTATUS(status) == 0,
	    "child exited abnormally: status %#x", status);
	group_snapshot(&g, after_exit);

	for (i = 0; i < g.g_n; i++) {
		ATF_REQUIRE_MSG(control[i] >= before[i],
		    "deferred member %u regressed during control: %ju -> %ju",
		    i, (uintmax_t)before[i], (uintmax_t)control[i]);
		ATF_REQUIRE_MSG(live_child[i] >= control[i],
		    "deferred member %u regressed during child work: %ju -> %ju",
		    i, (uintmax_t)control[i], (uintmax_t)live_child[i]);
		ATF_REQUIRE_MSG(after_exit[i] >= live_child[i],
		    "deferred member %u regressed across child exit: %ju -> %ju",
		    i, (uintmax_t)live_child[i], (uintmax_t)after_exit[i]);
		control_delta = control[i] - before[i];
		work_delta = live_child[i] - control[i];
		ATF_CHECK_MSG(work_delta >= CHILD_MIN_DELTA &&
		    work_delta / 8 > control_delta,
		    "deferred member %u child-work delta %ju is not "
		    "substantially above control delta %ju", i,
		    (uintmax_t)work_delta, (uintmax_t)control_delta);
	}

	ATF_REQUIRE_MSG(pmc_stop(g.g_ids[0]) == 0,
	    "deferred group stop failed: %s", strerror(errno));
	g.g_started = false;
	ATF_REQUIRE_MSG(pmc_release(g.g_ids[0]) == 0,
	    "deferred group release failed: %s", strerror(errno));
	group_init(&g);
	reset_fork_test_hooks();
	read_live_target_counts(&final);
	check_live_counts_equal("deferred attach cleanup did not restore baseline",
	    &final, &initial);
}
ATF_TC_CLEANUP(descendants_existing_tree_attach_deferred, tc)
{

	reset_fork_test_hooks();
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
	ATF_TP_ADD_TC(tp, descendants_child_reads_whole_group);
	ATF_TP_ADD_TC(tp, descendants_fork_inherit_alloc_failure_atomic);
	ATF_TP_ADD_TC(tp, descendants_fork_inherit_link_alloc_failure_atomic);
	ATF_TP_ADD_TC(tp, descendants_fork_miss_has_no_false_detach);
	ATF_TP_ADD_TC(tp, descendants_inherited_parent_fork_miss_is_identified);
	ATF_TP_ADD_TC(tp, descendants_attach_alloc_rollback);
	ATF_TP_ADD_TC(tp, descendants_attach_authorization_rollback);
	ATF_TP_ADD_TC(tp, descendants_attach_race_fork);
	ATF_TP_ADD_TC(tp, descendants_attach_race_exit);
	ATF_TP_ADD_TC(tp, descendants_parent_target_exits_before_child);
	ATF_TP_ADD_TC(tp, descendants_release_running_resident_with_live_child);
	ATF_TP_ADD_TC(tp, descendants_release_running_evicted_with_live_child);
	ATF_TP_ADD_TC(tp, descendants_existing_tree_attach_atomic);
	ATF_TP_ADD_TC(tp, descendants_existing_tree_attach_deferred);
	ATF_TP_ADD_TC(tp, child_exit_does_not_regress);

	return (atf_no_error());
}
