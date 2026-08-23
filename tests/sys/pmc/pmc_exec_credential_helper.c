/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Advanced Micro Devices, Inc.
 */

#include <sys/types.h>

#include <errno.h>
#include <inttypes.h>
#include <limits.h>
#include <pmc.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "pmc_exec_credential_common.h"

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
write_byte(int fd, char value)
{
	ssize_t n;

	do {
		n = write(fd, &value, 1);
	} while (n == -1 && errno == EINTR);
	return (n == 1 ? 0 : -1);
}

static int
parse_fd(const char *text, int *fd)
{
	char *end;
	long value;

	errno = 0;
	value = strtol(text, &end, 10);
	if (errno != 0 || end == text || *end != '\0' || value < 0 ||
	    value > INT_MAX) {
		errno = EINVAL;
		return (-1);
	}
	*fd = (int)value;
	return (0);
}

static int
parse_pmc_id(const char *text, pmc_id_t *pmcid)
{
	char *end;
	uintmax_t value;

	errno = 0;
	value = strtoumax(text, &end, 0);
	if (errno != 0 || end == text || *end != '\0' ||
	    (uintmax_t)(pmc_id_t)value != value) {
		errno = EINVAL;
		return (-1);
	}
	*pmcid = (pmc_id_t)value;
	return (0);
}

static void
relay_failure(int result_fd, int error)
{
	struct pmc_exec_credential_result result;

	memset(&result, 0, sizeof(result));
	result.per_version = PMC_EXEC_CREDENTIAL_RESULT_VERSION;
	result.per_stage = PMC_EXEC_CREDENTIAL_STAGE_NONE;
	result.per_stage_errno = error;
	(void)write_full(result_fd, &result, sizeof(result));
	_exit(127);
}

static int
run_relay(int argc, char **argv)
{
	struct pmc_exec_target_command command;
	struct pmc_exec_target_precheck precheck;
	const char *helper_path;
	char fd_arg[32], leader_arg[32], sibling_arg[32];
	int command_fd, error, length, precheck_fd, ready_fd, result_fd;

	if (argc != 7 || parse_fd(argv[2], &ready_fd) != 0 ||
	    parse_fd(argv[3], &command_fd) != 0 ||
	    parse_fd(argv[4], &precheck_fd) != 0 ||
	    parse_fd(argv[5], &result_fd) != 0)
		return (2);
	helper_path = argv[6];

	if (write_byte(ready_fd, 'R') != 0)
		relay_failure(result_fd, errno);
	close(ready_fd);
	if (read_full(command_fd, &command, sizeof(command)) !=
	    (ssize_t)sizeof(command))
		relay_failure(result_fd, EPIPE);
	close(command_fd);

	if (pmc_init() != 0)
		relay_failure(result_fd, errno);
	memset(&precheck, 0, sizeof(precheck));
	errno = 0;
	precheck.petp_leader_rc =
	    pmc_read(command.petc_leader, &precheck.petp_leader_value);
	precheck.petp_leader_errno = errno;
	errno = 0;
	precheck.petp_sibling_rc =
	    pmc_read(command.petc_sibling, &precheck.petp_sibling_value);
	precheck.petp_sibling_errno = errno;
	if (write_full(precheck_fd, &precheck, sizeof(precheck)) != 0)
		relay_failure(result_fd, errno);
	close(precheck_fd);

	length = snprintf(fd_arg, sizeof(fd_arg), "%d", result_fd);
	if (length < 0 || length >= (int)sizeof(fd_arg))
		relay_failure(result_fd, EINVAL);
	length = snprintf(leader_arg, sizeof(leader_arg), "%ju",
	    (uintmax_t)command.petc_leader);
	if (length < 0 || length >= (int)sizeof(leader_arg))
		relay_failure(result_fd, EINVAL);
	length = snprintf(sibling_arg, sizeof(sibling_arg), "%ju",
	    (uintmax_t)command.petc_sibling);
	if (length < 0 || length >= (int)sizeof(sibling_arg))
		relay_failure(result_fd, EINVAL);
	execl(helper_path, "pmc_exec_credential_helper", fd_arg, leader_arg,
	    sibling_arg, NULL);
	error = errno;
	relay_failure(result_fd, error);
	return (127);
}

int
main(int argc, char **argv)
{
	struct pmc_exec_credential_result result;
	pmc_value_t value;
	pmc_id_t leader, sibling;
	int result_fd;

	if (argc > 1 && strcmp(argv[1], PMC_EXEC_CREDENTIAL_RELAY_MODE) == 0)
		return (run_relay(argc, argv));
	if (argc != 4 || parse_fd(argv[1], &result_fd) != 0 ||
	    parse_pmc_id(argv[2], &leader) != 0 ||
	    parse_pmc_id(argv[3], &sibling) != 0)
		return (2);

	result = (struct pmc_exec_credential_result){
		.per_version = PMC_EXEC_CREDENTIAL_RESULT_VERSION,
		.per_stage = PMC_EXEC_CREDENTIAL_STAGE_CREDENTIALS,
		.per_pmc_init_rc = -2,
		.per_leader_rc = -2,
		.per_sibling_rc = -2,
	};
	if (getresuid(&result.per_ruid, &result.per_euid,
	    &result.per_suid) != 0 ||
	    getresgid(&result.per_rgid, &result.per_egid,
	    &result.per_sgid) != 0) {
		result.per_stage_errno = errno;
		goto out;
	}

	result.per_stage = PMC_EXEC_CREDENTIAL_STAGE_GROUPS;
	result.per_ngroups = getgroups(0, NULL);
	if (result.per_ngroups < 0) {
		result.per_stage_errno = errno;
		goto out;
	}
	if (result.per_ngroups > PMC_EXEC_CREDENTIAL_MAX_GROUPS) {
		result.per_stage_errno = EOVERFLOW;
		goto out;
	}
	if (result.per_ngroups != 0 &&
	    getgroups(result.per_ngroups, result.per_groups) == -1) {
		result.per_stage_errno = errno;
		goto out;
	}

	result.per_stage = PMC_EXEC_CREDENTIAL_STAGE_PMC_INIT;
	errno = 0;
	result.per_pmc_init_rc = pmc_init();
	result.per_pmc_init_errno = errno;
	if (result.per_pmc_init_rc != 0) {
		result.per_stage_errno = result.per_pmc_init_errno;
		goto out;
	}

	result.per_stage = PMC_EXEC_CREDENTIAL_STAGE_READS;
	value = 0;
	errno = 0;
	result.per_leader_rc = pmc_read(leader, &value);
	result.per_leader_errno = errno;
	result.per_leader_value = value;
	value = 0;
	errno = 0;
	result.per_sibling_rc = pmc_read(sibling, &value);
	result.per_sibling_errno = errno;
	result.per_sibling_value = value;
	result.per_stage = PMC_EXEC_CREDENTIAL_STAGE_DONE;

out:
	if (write_full(result_fd, &result, sizeof(result)) != 0)
		return (3);
	return (result.per_stage == PMC_EXEC_CREDENTIAL_STAGE_DONE ? 0 : 1);
}
