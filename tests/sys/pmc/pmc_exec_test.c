/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Advanced Micro Devices, Inc.
 */

/*
 * This file tests exec authorization for grouped process PMCs, when the
 * exec changes the credential.
 *
 * A setgid exec adds a group.  This group is not in the PMC owner's
 * credential.  Because of this, pmc_can_attach() must reject the new
 * credential.  The direct target is the control case.  The inherited
 * cases prove that reauthorization uses the authoritative group-target set.
 */

#include <sys/types.h>
#include <sys/mount.h>
#include <sys/stat.h>
#include <sys/sysctl.h>
#include <sys/wait.h>

#include <errno.h>
#include <fcntl.h>
#include <grp.h>
#include <limits.h>
#include <pmc.h>
#include <pwd.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <atf-c.h>

#include "pmc_exec_credential_common.h"

#define	TEST_EVENT			"instructions"
#define	TEST_OWNER_USER			"nobody"
#define	TEST_FOREIGN_GROUP		"wheel"
#define	TEST_HELPER_NAME		"pmc_exec_credential_helper"
#define	TEST_HELPER_PATH_RECORD		"pmc_exec_credential_helper.path"
#define	TEST_SUID_DIR_ENV		"PMC_EXEC_TEST_SUID_DIR"
#define	TEST_MAX_ROW_OCCUPIERS		64

#define	TEST_SYSCTL_PREFIX		"kern.hwpmc.test."
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

#define	OWNER_COMMAND_START	'S'
#define	OWNER_COMMAND_EXEC	'E'
#define	OWNER_COMMAND_CLEANUP	'C'

enum owner_message_kind {
	OWNER_MESSAGE_ERROR,
	OWNER_MESSAGE_PREPARED,
	OWNER_MESSAGE_STARTED,
	OWNER_MESSAGE_TARGET_DONE,
	OWNER_MESSAGE_DONE
};

enum owner_stage {
	OWNER_STAGE_NONE,
	OWNER_STAGE_SETGROUPS,
	OWNER_STAGE_SETGID,
	OWNER_STAGE_SETUID,
	OWNER_STAGE_PMC_INIT,
	OWNER_STAGE_OCCUPANCY_ALLOCATE,
	OWNER_STAGE_TARGET_PIPE,
	OWNER_STAGE_TARGET_FORK,
	OWNER_STAGE_TARGET_READY,
	OWNER_STAGE_LEADER_ALLOCATE,
	OWNER_STAGE_SIBLING_ALLOCATE,
	OWNER_STAGE_GROUP_CREATE,
	OWNER_STAGE_GROUP_ADD_LEADER,
	OWNER_STAGE_GROUP_ADD_SIBLING,
	OWNER_STAGE_GROUP_COMMIT,
	OWNER_STAGE_GROUP_ATTACH,
	OWNER_STAGE_WAIT_START,
	OWNER_STAGE_GROUP_START,
	OWNER_STAGE_WAIT_EXEC,
	OWNER_STAGE_TARGET_COMMAND,
	OWNER_STAGE_TARGET_WAIT,
	OWNER_STAGE_WAIT_CLEANUP,
	OWNER_STAGE_GROUP_RELEASE,
	OWNER_STAGE_OCCUPANCY_RELEASE
};

struct owner_message {
	uint32_t	om_kind;
	uint32_t	om_stage;
	int		om_rc;
	int		om_errno;
	uint32_t	om_group_id;
	pmc_id_t	om_leader;
	pmc_id_t	om_sibling;
	pid_t		om_target_pid;
	int		om_target_status;
};

struct row_occupancy {
	pmc_id_t	ro_ids[TEST_MAX_ROW_OCCUPIERS];
	u_int		ro_n;
};

struct live_target_counts {
	uint64_t	ltc_pmc_targets;
	uint64_t	ltc_group_targets;
	uint64_t	ltc_target_processes;
	uint64_t	ltc_residual_entries;
	uint64_t	ltc_rotation_refs;
	uint64_t	ltc_run_refs;
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
check_live_counts_equal(const struct live_target_counts *actual,
    const struct live_target_counts *expected)
{

	ATF_CHECK_MSG(actual->ltc_pmc_targets == expected->ltc_pmc_targets &&
	    actual->ltc_group_targets == expected->ltc_group_targets &&
	    actual->ltc_target_processes == expected->ltc_target_processes &&
	    actual->ltc_residual_entries == expected->ltc_residual_entries &&
	    actual->ltc_rotation_refs == expected->ltc_rotation_refs &&
	    actual->ltc_run_refs == expected->ltc_run_refs,
	    "cleanup changed live counts: pmc_target %ju/%ju "
	    "group_target %ju/%ju process %ju/%ju residual %ju/%ju "
	    "rotation_ref %ju/%ju run_ref %ju/%ju",
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

static void
reset_exec_test_hooks(void)
{

	(void)sysctl_write_u32(TEST_HOLD_GROUP_RESIDENT, 0);
	(void)sysctl_write_u32(TEST_HOLD_GROUP_EVICTED, 0);
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

static const char *
owner_stage_name(uint32_t stage)
{

	switch (stage) {
	case OWNER_STAGE_SETGROUPS:
		return ("setgroups");
	case OWNER_STAGE_SETGID:
		return ("setgid");
	case OWNER_STAGE_SETUID:
		return ("setuid");
	case OWNER_STAGE_PMC_INIT:
		return ("pmc_init");
	case OWNER_STAGE_OCCUPANCY_ALLOCATE:
		return ("row occupancy allocation");
	case OWNER_STAGE_TARGET_PIPE:
		return ("target pipe setup");
	case OWNER_STAGE_TARGET_FORK:
		return ("target fork");
	case OWNER_STAGE_TARGET_READY:
		return ("target readiness");
	case OWNER_STAGE_LEADER_ALLOCATE:
		return ("leader allocation");
	case OWNER_STAGE_SIBLING_ALLOCATE:
		return ("sibling allocation");
	case OWNER_STAGE_GROUP_CREATE:
		return ("group creation");
	case OWNER_STAGE_GROUP_ADD_LEADER:
		return ("leader group add");
	case OWNER_STAGE_GROUP_ADD_SIBLING:
		return ("sibling group add");
	case OWNER_STAGE_GROUP_COMMIT:
		return ("group commit");
	case OWNER_STAGE_GROUP_ATTACH:
		return ("group attach");
	case OWNER_STAGE_WAIT_START:
		return ("start command");
	case OWNER_STAGE_GROUP_START:
		return ("group start");
	case OWNER_STAGE_WAIT_EXEC:
		return ("exec command");
	case OWNER_STAGE_TARGET_COMMAND:
		return ("target exec command");
	case OWNER_STAGE_TARGET_WAIT:
		return ("target wait");
	case OWNER_STAGE_WAIT_CLEANUP:
		return ("cleanup command");
	case OWNER_STAGE_GROUP_RELEASE:
		return ("group release");
	case OWNER_STAGE_OCCUPANCY_RELEASE:
		return ("row occupancy release");
	default:
		return ("unknown owner stage");
	}
}

static void
owner_send_message(int fd, enum owner_message_kind kind,
    enum owner_stage stage, int rc, int error, uint32_t group_id,
    pmc_id_t leader, pmc_id_t sibling, pid_t target_pid, int target_status)
{
	struct owner_message message;

	memset(&message, 0, sizeof(message));
	message.om_kind = kind;
	message.om_stage = stage;
	message.om_rc = rc;
	message.om_errno = error;
	message.om_group_id = group_id;
	message.om_leader = leader;
	message.om_sibling = sibling;
	message.om_target_pid = target_pid;
	message.om_target_status = target_status;
	if (write_full(fd, &message, sizeof(message)) != 0)
		_exit(120);
}

static void
owner_fail(int fd, enum owner_stage stage, int rc, int error)
{

	owner_send_message(fd, OWNER_MESSAGE_ERROR, stage, rc, error, 0,
	    PMC_ID_INVALID, PMC_ID_INVALID, -1, 0);
	_exit(100 + stage);
}

static void
target_exec_failure(int result_fd, int error)
{
	struct pmc_exec_credential_result result;

	memset(&result, 0, sizeof(result));
	result.per_version = PMC_EXEC_CREDENTIAL_RESULT_VERSION;
	result.per_stage = PMC_EXEC_CREDENTIAL_STAGE_NONE;
	result.per_stage_errno = error;
	(void)write_full(result_fd, &result, sizeof(result));
	_exit(127);
}

static void
target_precheck_and_exec(const char *helper_path, int precheck_fd,
    int result_fd, pmc_id_t leader, pmc_id_t sibling)
{
	struct pmc_exec_target_precheck precheck;
	char fd_arg[32], leader_arg[32], sibling_arg[32];
	int error, length;

	memset(&precheck, 0, sizeof(precheck));
	errno = 0;
	precheck.petp_leader_rc =
	    pmc_read(leader, &precheck.petp_leader_value);
	precheck.petp_leader_errno = errno;
	errno = 0;
	precheck.petp_sibling_rc =
	    pmc_read(sibling, &precheck.petp_sibling_value);
	precheck.petp_sibling_errno = errno;
	if (write_full(precheck_fd, &precheck, sizeof(precheck)) != 0)
		target_exec_failure(result_fd, errno);
	close(precheck_fd);

	length = snprintf(fd_arg, sizeof(fd_arg), "%d", result_fd);
	if (length < 0 || length >= (int)sizeof(fd_arg))
		target_exec_failure(result_fd, EINVAL);
	length = snprintf(leader_arg, sizeof(leader_arg), "%ju",
	    (uintmax_t)leader);
	if (length < 0 || length >= (int)sizeof(leader_arg))
		target_exec_failure(result_fd, EINVAL);
	length = snprintf(sibling_arg, sizeof(sibling_arg), "%ju",
	    (uintmax_t)sibling);
	if (length < 0 || length >= (int)sizeof(sibling_arg))
		target_exec_failure(result_fd, EINVAL);
	execl(helper_path, TEST_HELPER_NAME, fd_arg, leader_arg, sibling_arg,
	    NULL);
	error = errno;
	target_exec_failure(result_fd, error);
}

static void
target_exec_relay(const char *source_helper_path, const char *helper_path,
    int ready_fd, int command_fd, int precheck_fd, int result_fd)
{
	char command_fd_arg[32], precheck_fd_arg[32], ready_fd_arg[32];
	char result_fd_arg[32];
	int error, length;

	length = snprintf(ready_fd_arg, sizeof(ready_fd_arg), "%d", ready_fd);
	if (length < 0 || length >= (int)sizeof(ready_fd_arg))
		target_exec_failure(result_fd, EINVAL);
	length = snprintf(command_fd_arg, sizeof(command_fd_arg), "%d",
	    command_fd);
	if (length < 0 || length >= (int)sizeof(command_fd_arg))
		target_exec_failure(result_fd, EINVAL);
	length = snprintf(precheck_fd_arg, sizeof(precheck_fd_arg), "%d",
	    precheck_fd);
	if (length < 0 || length >= (int)sizeof(precheck_fd_arg))
		target_exec_failure(result_fd, EINVAL);
	length = snprintf(result_fd_arg, sizeof(result_fd_arg), "%d", result_fd);
	if (length < 0 || length >= (int)sizeof(result_fd_arg))
		target_exec_failure(result_fd, EINVAL);
	execl(source_helper_path, TEST_HELPER_NAME,
	    PMC_EXEC_CREDENTIAL_RELAY_MODE, ready_fd_arg, command_fd_arg,
	    precheck_fd_arg, result_fd_arg, helper_path, NULL);
	error = errno;
	target_exec_failure(result_fd, error);
}

static pid_t
owner_spawn_direct_target(int owner_message_fd, int precheck_fd,
    int result_fd, const char *source_helper_path, const char *helper_path,
    int *command_fd)
{
	int command_pipe[2], ready_pipe[2];
	pid_t target;
	char ready;

	if (pipe(command_pipe) != 0 || pipe(ready_pipe) != 0)
		owner_fail(owner_message_fd, OWNER_STAGE_TARGET_PIPE, -1, errno);
	target = fork();
	if (target == -1)
		owner_fail(owner_message_fd, OWNER_STAGE_TARGET_FORK, -1, errno);
	if (target == 0) {
		close(command_pipe[1]);
		close(ready_pipe[0]);
		target_exec_relay(source_helper_path, helper_path,
		    ready_pipe[1], command_pipe[0], precheck_fd, result_fd);
	}

	close(command_pipe[0]);
	close(ready_pipe[1]);
	if (read_byte(ready_pipe[0], &ready) != 0)
		owner_fail(owner_message_fd, OWNER_STAGE_TARGET_READY, -1,
		    errno);
	if (ready != 'R')
		owner_fail(owner_message_fd, OWNER_STAGE_TARGET_READY, -1,
		    EPROTO);
	close(ready_pipe[0]);
	*command_fd = command_pipe[1];
	return (target);
}

static void
owner_fill_occupancy(int owner_message_fd, struct row_occupancy *occupancy)
{
	pmc_id_t pmcid;
	int error;

	memset(occupancy, 0, sizeof(*occupancy));
	while (occupancy->ro_n < nitems(occupancy->ro_ids)) {
		errno = 0;
		if (pmc_allocate(TEST_EVENT, PMC_MODE_TC, 0, PMC_CPU_ANY,
		    &pmcid, 0) == 0) {
			occupancy->ro_ids[occupancy->ro_n++] = pmcid;
			continue;
		}
		error = errno;
		if (error == EINVAL && occupancy->ro_n != 0)
			return;
		owner_fail(owner_message_fd, OWNER_STAGE_OCCUPANCY_ALLOCATE,
		    -1, error);
	}
	owner_fail(owner_message_fd, OWNER_STAGE_OCCUPANCY_ALLOCATE, -1,
	    EOVERFLOW);
}

static void
owner_release_occupancy(int owner_message_fd,
    struct row_occupancy *occupancy)
{
	u_int i;

	for (i = 0; i < occupancy->ro_n; i++) {
		errno = 0;
		if (pmc_release(occupancy->ro_ids[i]) != 0)
			owner_fail(owner_message_fd,
			    OWNER_STAGE_OCCUPANCY_RELEASE, -1, errno);
	}
	occupancy->ro_n = 0;
}

static void
owner_process(int command_fd, int message_fd, int precheck_fd, int result_fd,
    const char *source_helper_path, const char *helper_path, uid_t owner_uid,
    gid_t owner_gid, bool inherited, bool evicted)
{
	struct row_occupancy occupancy;
	struct pmc_exec_target_command target_command;
	uint32_t group_id;
	pmc_id_t leader, sibling;
	pid_t target, waited;
	char command;
	int status, target_command_fd;
	uint32_t flags;

	memset(&occupancy, 0, sizeof(occupancy));
	target = -1;
	target_command_fd = -1;
	leader = PMC_ID_INVALID;
	sibling = PMC_ID_INVALID;

	errno = 0;
	if (setgroups(1, &owner_gid) != 0)
		owner_fail(message_fd, OWNER_STAGE_SETGROUPS, -1, errno);
	errno = 0;
	if (setgid(owner_gid) != 0)
		owner_fail(message_fd, OWNER_STAGE_SETGID, -1, errno);
	errno = 0;
	if (setuid(owner_uid) != 0)
		owner_fail(message_fd, OWNER_STAGE_SETUID, -1, errno);
	errno = 0;
	if (pmc_init() != 0)
		owner_fail(message_fd, OWNER_STAGE_PMC_INIT, -1, errno);

	if (!inherited) {
		target = owner_spawn_direct_target(message_fd, precheck_fd,
		    result_fd, source_helper_path, helper_path,
		    &target_command_fd);
		close(precheck_fd);
		close(result_fd);
	}
	if (evicted)
		owner_fill_occupancy(message_fd, &occupancy);

	flags = PMC_F_DESCENDANTS | PMC_F_GROUP_MUX;
	errno = 0;
	if (pmc_allocate_group(TEST_EVENT, PMC_MODE_TC, flags, PMC_CPU_ANY,
	    &leader, 0) != 0)
		owner_fail(message_fd, OWNER_STAGE_LEADER_ALLOCATE, -1, errno);
	errno = 0;
	if (pmc_allocate_group(TEST_EVENT, PMC_MODE_TC, 0, PMC_CPU_ANY,
	    &sibling, 0) != 0)
		owner_fail(message_fd, OWNER_STAGE_SIBLING_ALLOCATE, -1, errno);
	errno = 0;
	if (pmc_group_create(&group_id) != 0)
		owner_fail(message_fd, OWNER_STAGE_GROUP_CREATE, -1, errno);
	errno = 0;
	if (pmc_group_add(group_id, leader, 1) != 0)
		owner_fail(message_fd, OWNER_STAGE_GROUP_ADD_LEADER, -1, errno);
	errno = 0;
	if (pmc_group_add(group_id, sibling, 0) != 0)
		owner_fail(message_fd, OWNER_STAGE_GROUP_ADD_SIBLING, -1,
		    errno);
	errno = 0;
	if (pmc_group_commit(group_id) != 0)
		owner_fail(message_fd, OWNER_STAGE_GROUP_COMMIT, -1, errno);
	errno = 0;
	if (pmc_attach(leader, inherited ? getpid() : target) != 0)
		owner_fail(message_fd, OWNER_STAGE_GROUP_ATTACH, -1, errno);

	owner_send_message(message_fd, OWNER_MESSAGE_PREPARED,
	    OWNER_STAGE_NONE, 0, 0, group_id, leader, sibling, target, 0);
	if (read_byte(command_fd, &command) != 0)
		owner_fail(message_fd, OWNER_STAGE_WAIT_START, -1, errno);
	if (command != OWNER_COMMAND_START)
		owner_fail(message_fd, OWNER_STAGE_WAIT_START, -1, EPROTO);
	errno = 0;
	if (pmc_start(leader) != 0)
		owner_fail(message_fd, OWNER_STAGE_GROUP_START, -1, errno);
	owner_send_message(message_fd, OWNER_MESSAGE_STARTED,
	    OWNER_STAGE_NONE, 0, 0, group_id, leader, sibling, target, 0);

	if (read_byte(command_fd, &command) != 0)
		owner_fail(message_fd, OWNER_STAGE_WAIT_EXEC, -1, errno);
	if (command != OWNER_COMMAND_EXEC)
		owner_fail(message_fd, OWNER_STAGE_WAIT_EXEC, -1, EPROTO);
	if (inherited) {
		target = fork();
		if (target == -1)
			owner_fail(message_fd, OWNER_STAGE_TARGET_FORK, -1,
			    errno);
		if (target == 0)
			target_precheck_and_exec(helper_path, precheck_fd,
			    result_fd, leader, sibling);
		close(precheck_fd);
		close(result_fd);
	} else {
		target_command.petc_leader = leader;
		target_command.petc_sibling = sibling;
		if (write_full(target_command_fd, &target_command,
		    sizeof(target_command)) != 0)
			owner_fail(message_fd, OWNER_STAGE_TARGET_COMMAND, -1,
			    errno);
		close(target_command_fd);
	}

	errno = 0;
	waited = waitpid(target, &status, 0);
	if (waited != target)
		owner_fail(message_fd, OWNER_STAGE_TARGET_WAIT, (int)waited,
		    errno);
	owner_send_message(message_fd, OWNER_MESSAGE_TARGET_DONE,
	    OWNER_STAGE_NONE, 0, 0, group_id, leader, sibling, target, status);

	if (read_byte(command_fd, &command) != 0)
		owner_fail(message_fd, OWNER_STAGE_WAIT_CLEANUP, -1, errno);
	if (command != OWNER_COMMAND_CLEANUP)
		owner_fail(message_fd, OWNER_STAGE_WAIT_CLEANUP, -1, EPROTO);
	errno = 0;
	if (pmc_release(leader) != 0)
		owner_fail(message_fd, OWNER_STAGE_GROUP_RELEASE, -1, errno);
	owner_release_occupancy(message_fd, &occupancy);
	owner_send_message(message_fd, OWNER_MESSAGE_DONE, OWNER_STAGE_NONE,
	    0, 0, group_id, leader, sibling, target, status);
	_exit(0);
}

static void
save_helper_path(const char *path)
{
	int fd;

	fd = open(TEST_HELPER_PATH_RECORD, O_WRONLY | O_CREAT | O_TRUNC, 0600);
	ATF_REQUIRE_MSG(fd != -1, "open %s failed: errno %d (%s)",
	    TEST_HELPER_PATH_RECORD, errno, strerror(errno));
	ATF_REQUIRE_MSG(write_full(fd, path, strlen(path)) == 0,
	    "writing %s failed: errno %d (%s)", TEST_HELPER_PATH_RECORD,
	    errno, strerror(errno));
	ATF_REQUIRE_MSG(close(fd) == 0, "closing %s failed: errno %d (%s)",
	    TEST_HELPER_PATH_RECORD, errno, strerror(errno));
}

static void
cleanup_helper_path(void)
{
	char path[PATH_MAX];
	ssize_t n;
	int fd;

	fd = open(TEST_HELPER_PATH_RECORD, O_RDONLY);
	if (fd == -1)
		return;
	n = read(fd, path, sizeof(path) - 1);
	(void)close(fd);
	if (n > 0) {
		path[n] = '\0';
		(void)unlink(path);
	}
	(void)unlink(TEST_HELPER_PATH_RECORD);
}

static void
install_setgid_helper(const char *source_path, const char *destination_dir,
    gid_t foreign_gid, char helper_path[PATH_MAX])
{
	struct stat st;
	struct statfs fs;
	char buffer[8192];
	ssize_t nread;
	int destination_fd, length, source_fd;

	source_fd = open(source_path, O_RDONLY);
	ATF_REQUIRE_MSG(source_fd != -1,
	    "opening helper %s failed: errno %d (%s)", source_path, errno,
	    strerror(errno));

	length = snprintf(helper_path, PATH_MAX,
	    "%s/.pmc-exec-credential.XXXXXX", destination_dir);
	ATF_REQUIRE_MSG(length >= 0 && length < PATH_MAX,
	    "helper destination path is too long");
	destination_fd = mkstemp(helper_path);
	ATF_REQUIRE_MSG(destination_fd != -1,
	    "mkstemp for setgid helper failed: errno %d (%s)", errno,
	    strerror(errno));
	save_helper_path(helper_path);
	ATF_REQUIRE_MSG(fstatfs(destination_fd, &fs) == 0,
	    "fstatfs for %s failed: errno %d (%s)", helper_path, errno,
	    strerror(errno));
	if ((fs.f_flags & MNT_NOSUID) != 0) {
		(void)close(source_fd);
		(void)close(destination_fd);
		cleanup_helper_path();
		atf_tc_skip("%s is on a nosuid filesystem", helper_path);
	}

	while ((nread = read(source_fd, buffer, sizeof(buffer))) != 0) {
		ATF_REQUIRE_MSG(nread > 0,
		    "reading helper %s failed: errno %d (%s)", source_path,
		    errno, strerror(errno));
		ATF_REQUIRE_MSG(write_full(destination_fd, buffer,
		    (size_t)nread) == 0,
		    "copying helper to %s failed: errno %d (%s)", helper_path,
		    errno, strerror(errno));
	}
	ATF_REQUIRE_MSG(close(source_fd) == 0,
	    "closing helper %s failed: errno %d (%s)", source_path, errno,
	    strerror(errno));
	ATF_REQUIRE_MSG(fchown(destination_fd, 0, foreign_gid) == 0,
	    "fchown helper %s failed: errno %d (%s)", helper_path, errno,
	    strerror(errno));
	ATF_REQUIRE_MSG(fchmod(destination_fd, 02555) == 0,
	    "fchmod helper %s failed: errno %d (%s)", helper_path, errno,
	    strerror(errno));
	ATF_REQUIRE_MSG(fsync(destination_fd) == 0,
	    "fsync helper %s failed: errno %d (%s)", helper_path, errno,
	    strerror(errno));
	ATF_REQUIRE_MSG(close(destination_fd) == 0,
	    "closing helper %s failed: errno %d (%s)", helper_path, errno,
	    strerror(errno));
	ATF_REQUIRE_MSG(stat(helper_path, &st) == 0,
	    "stat helper %s failed: errno %d (%s)", helper_path, errno,
	    strerror(errno));
	ATF_REQUIRE_MSG(S_ISREG(st.st_mode) && st.st_gid == foreign_gid &&
	    (st.st_mode & 07777) == 02555,
	    "helper %s has mode %#o uid %u gid %u, expected setgid gid %u",
	    helper_path, st.st_mode, st.st_uid, st.st_gid, foreign_gid);
}

static void
require_owner_message(int fd, enum owner_message_kind expected,
    struct owner_message *message)
{
	ssize_t nread;

	memset(message, 0, sizeof(*message));
	nread = read_full(fd, message, sizeof(*message));
	ATF_REQUIRE_MSG(nread == (ssize_t)sizeof(*message),
	    "owner message read returned %zd of %zu bytes", nread,
	    sizeof(*message));
	ATF_REQUIRE_MSG(message->om_kind != OWNER_MESSAGE_ERROR,
	    "owner failed at %s: rc=%d errno=%d (%s)",
	    owner_stage_name(message->om_stage), message->om_rc,
	    message->om_errno, strerror(message->om_errno));
	ATF_REQUIRE_MSG(message->om_kind == (uint32_t)expected,
	    "owner message kind %u, expected %u", message->om_kind, expected);
}

static void
check_helper_credentials(const struct pmc_exec_credential_result *result,
    uid_t owner_uid, gid_t owner_gid, gid_t foreign_gid)
{

	ATF_REQUIRE_MSG(
	    result->per_version == PMC_EXEC_CREDENTIAL_RESULT_VERSION,
	    "helper result version %u, expected %u", result->per_version,
	    PMC_EXEC_CREDENTIAL_RESULT_VERSION);
	ATF_REQUIRE_MSG(result->per_stage == PMC_EXEC_CREDENTIAL_STAGE_DONE,
	    "helper stopped at stage %u: errno %d (%s)", result->per_stage,
	    result->per_stage_errno, strerror(result->per_stage_errno));
	ATF_REQUIRE_MSG(result->per_pmc_init_rc == 0,
	    "helper pmc_init returned %d errno %d (%s)",
	    result->per_pmc_init_rc, result->per_pmc_init_errno,
	    strerror(result->per_pmc_init_errno));
	ATF_REQUIRE_MSG(result->per_ruid == owner_uid &&
	    result->per_euid == owner_uid && result->per_suid == owner_uid,
	    "helper uid credentials are r/e/s %u/%u/%u, expected %u",
	    result->per_ruid, result->per_euid, result->per_suid, owner_uid);
	ATF_REQUIRE_MSG(result->per_rgid == owner_gid &&
	    result->per_egid == foreign_gid &&
	    result->per_sgid == foreign_gid,
	    "helper gid credentials are r/e/s %u/%u/%u, expected %u/%u/%u",
	    result->per_rgid, result->per_egid, result->per_sgid, owner_gid,
	    foreign_gid, foreign_gid);
	ATF_REQUIRE_MSG(result->per_ngroups == 1 &&
	    result->per_groups[0] == owner_gid,
	    "helper supplementary groups are count=%d first=%u, expected %u",
	    result->per_ngroups, result->per_groups[0], owner_gid);
}

static void
run_credential_exec_case(const atf_tc_t *tc, bool inherited, bool evicted)
{
	struct pmc_exec_credential_result result;
	struct live_target_counts final, initial;
	struct owner_message message;
	struct pmc_exec_target_precheck precheck;
	struct passwd *owner;
	struct group *foreign;
	char helper_path[PATH_MAX], source_helper_path[PATH_MAX];
	const char *destination_dir, *hold_ack_name, *hold_name, *srcdir;
	pid_t owner_pid, waited;
	uid_t owner_uid;
	gid_t foreign_gid, owner_gid;
	ssize_t result_nread, precheck_nread;
	int command_pipe[2], message_pipe[2], precheck_pipe[2], result_pipe[2];
	int length, owner_status, wait_errno;

	require_hwpmc();
	require_hwpmc_test_support();
	reset_exec_test_hooks();
	read_live_target_counts(&initial);

	owner = getpwnam(TEST_OWNER_USER);
	ATF_REQUIRE_MSG(owner != NULL, "required user %s is missing",
	    TEST_OWNER_USER);
	owner_uid = owner->pw_uid;
	owner_gid = owner->pw_gid;
	foreign = getgrnam(TEST_FOREIGN_GROUP);
	ATF_REQUIRE_MSG(foreign != NULL, "required group %s is missing",
	    TEST_FOREIGN_GROUP);
	foreign_gid = foreign->gr_gid;
	ATF_REQUIRE_MSG(owner_uid != 0 && owner_gid != foreign_gid,
	    "test credentials do not create a foreign group: uid=%u gid=%u "
	    "foreign gid=%u", owner_uid, owner_gid, foreign_gid);
	srcdir = atf_tc_get_config_var(tc, "srcdir");
	length = snprintf(source_helper_path, sizeof(source_helper_path),
	    "%s/%s", srcdir, TEST_HELPER_NAME);
	ATF_REQUIRE_MSG(length >= 0 && length < (int)sizeof(source_helper_path),
	    "helper source path is too long");
	destination_dir = getenv(TEST_SUID_DIR_ENV);
	if (destination_dir == NULL || destination_dir[0] == '\0')
		destination_dir = srcdir;
	install_setgid_helper(source_helper_path, destination_dir, foreign_gid,
	    helper_path);

	ATF_REQUIRE_MSG(pipe(command_pipe) == 0,
	    "owner command pipe failed: errno %d (%s)", errno,
	    strerror(errno));
	ATF_REQUIRE_MSG(pipe(message_pipe) == 0,
	    "owner message pipe failed: errno %d (%s)", errno,
	    strerror(errno));
	ATF_REQUIRE_MSG(pipe(precheck_pipe) == 0,
	    "target precheck pipe failed: errno %d (%s)", errno,
	    strerror(errno));
	ATF_REQUIRE_MSG(pipe(result_pipe) == 0,
	    "helper result pipe failed: errno %d (%s)", errno,
	    strerror(errno));

	owner_pid = fork();
	ATF_REQUIRE_MSG(owner_pid != -1,
	    "owner fork failed: errno %d (%s)", errno, strerror(errno));
	if (owner_pid == 0) {
		close(command_pipe[1]);
		close(message_pipe[0]);
		close(precheck_pipe[0]);
		close(result_pipe[0]);
		owner_process(command_pipe[0], message_pipe[1],
		    precheck_pipe[1], result_pipe[1], source_helper_path,
		    helper_path, owner_uid, owner_gid, inherited, evicted);
	}

	close(command_pipe[0]);
	close(message_pipe[1]);
	close(precheck_pipe[1]);
	close(result_pipe[1]);
	require_owner_message(message_pipe[0], OWNER_MESSAGE_PREPARED,
	    &message);

	hold_name = evicted ? TEST_HOLD_GROUP_EVICTED :
	    TEST_HOLD_GROUP_RESIDENT;
	hold_ack_name = evicted ? TEST_HOLD_GROUP_EVICTED_ACK :
	    TEST_HOLD_GROUP_RESIDENT_ACK;
	require_test_sysctl_write(hold_name, message.om_group_id);
	ATF_REQUIRE_MSG(write_byte(command_pipe[1], OWNER_COMMAND_START) == 0,
	    "sending owner start command failed: errno %d (%s)", errno,
	    strerror(errno));
	require_owner_message(message_pipe[0], OWNER_MESSAGE_STARTED,
	    &message);
	wait_test_sysctl_ack(hold_ack_name, message.om_group_id);

	ATF_REQUIRE_MSG(write_byte(command_pipe[1], OWNER_COMMAND_EXEC) == 0,
	    "sending owner exec command failed: errno %d (%s)", errno,
	    strerror(errno));
	memset(&precheck, 0, sizeof(precheck));
	precheck_nread = read_full(precheck_pipe[0], &precheck,
	    sizeof(precheck));
	memset(&result, 0, sizeof(result));
	result_nread = read_full(result_pipe[0], &result, sizeof(result));
	require_owner_message(message_pipe[0], OWNER_MESSAGE_TARGET_DONE,
	    &message);

	require_test_sysctl_write(hold_name, 0);
	ATF_REQUIRE_MSG(write_byte(command_pipe[1],
	    OWNER_COMMAND_CLEANUP) == 0,
	    "sending owner cleanup command failed: errno %d (%s)", errno,
	    strerror(errno));
	require_owner_message(message_pipe[0], OWNER_MESSAGE_DONE, &message);
	close(command_pipe[1]);
	close(message_pipe[0]);
	close(precheck_pipe[0]);
	close(result_pipe[0]);
	errno = 0;
	waited = waitpid(owner_pid, &owner_status, 0);
	wait_errno = errno;
	read_live_target_counts(&final);
	reset_exec_test_hooks();
	cleanup_helper_path();

	ATF_REQUIRE_MSG(precheck_nread == (ssize_t)sizeof(precheck),
	    "target precheck read returned %zd of %zu bytes", precheck_nread,
	    sizeof(precheck));
	ATF_REQUIRE_MSG(precheck.petp_leader_rc == 0,
	    "leader was not readable before exec: errno %d (%s)",
	    precheck.petp_leader_errno, strerror(precheck.petp_leader_errno));
	ATF_REQUIRE_MSG(precheck.petp_sibling_rc == 0,
	    "sibling was not readable before exec: errno %d (%s)",
	    precheck.petp_sibling_errno, strerror(precheck.petp_sibling_errno));
	ATF_REQUIRE_MSG(result_nread == (ssize_t)sizeof(result),
	    "helper result read returned %zd of %zu bytes", result_nread,
	    sizeof(result));
	ATF_REQUIRE_MSG(WIFEXITED(message.om_target_status) &&
	    WEXITSTATUS(message.om_target_status) == 0,
	    "credential helper exited abnormally: status %#x",
	    message.om_target_status);
	ATF_REQUIRE_MSG(waited == owner_pid,
	    "waitpid for owner failed: errno %d (%s)", wait_errno,
	    strerror(wait_errno));
	ATF_REQUIRE_MSG(WIFEXITED(owner_status) &&
	    WEXITSTATUS(owner_status) == 0,
	    "owner exited abnormally: status %#x", owner_status);
	check_helper_credentials(&result, owner_uid, owner_gid, foreign_gid);
	check_live_counts_equal(&final, &initial);

	ATF_CHECK_MSG(result.per_leader_rc == -1 &&
	    result.per_leader_errno == ESRCH,
	    "leader read after unauthorized exec returned rc=%d errno=%d "
	    "(%s), expected -1/ESRCH", result.per_leader_rc,
	    result.per_leader_errno, strerror(result.per_leader_errno));
	ATF_CHECK_MSG(result.per_sibling_rc == -1 &&
	    result.per_sibling_errno == ESRCH,
	    "sibling read after unauthorized exec returned rc=%d errno=%d "
	    "(%s), expected -1/ESRCH", result.per_sibling_rc,
	    result.per_sibling_errno, strerror(result.per_sibling_errno));
}

ATF_TC_WITH_CLEANUP(credential_exec_direct_target);
ATF_TC_HEAD(credential_exec_direct_target, tc)
{

	atf_tc_set_md_var(tc, "require.user", "root");
}
ATF_TC_BODY(credential_exec_direct_target, tc)
{

	run_credential_exec_case(tc, false, false);
}
ATF_TC_CLEANUP(credential_exec_direct_target, tc)
{

	reset_exec_test_hooks();
	cleanup_helper_path();
}

ATF_TC_WITH_CLEANUP(credential_exec_inherited_resident);
ATF_TC_HEAD(credential_exec_inherited_resident, tc)
{

	atf_tc_set_md_var(tc, "require.user", "root");
}
ATF_TC_BODY(credential_exec_inherited_resident, tc)
{

	run_credential_exec_case(tc, true, false);
}
ATF_TC_CLEANUP(credential_exec_inherited_resident, tc)
{

	reset_exec_test_hooks();
	cleanup_helper_path();
}

ATF_TC_WITH_CLEANUP(credential_exec_inherited_evicted);
ATF_TC_HEAD(credential_exec_inherited_evicted, tc)
{

	atf_tc_set_md_var(tc, "require.user", "root");
}
ATF_TC_BODY(credential_exec_inherited_evicted, tc)
{

	run_credential_exec_case(tc, true, true);
}
ATF_TC_CLEANUP(credential_exec_inherited_evicted, tc)
{

	reset_exec_test_hooks();
	cleanup_helper_path();
}

ATF_TP_ADD_TCS(tp)
{

	ATF_TP_ADD_TC(tp, credential_exec_direct_target);
	ATF_TP_ADD_TC(tp, credential_exec_inherited_resident);
	ATF_TP_ADD_TC(tp, credential_exec_inherited_evicted);
	return (atf_no_error());
}
