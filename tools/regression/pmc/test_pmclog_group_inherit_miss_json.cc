/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Regression test for JSON conversion of a group-inheritance miss record.
 */

#include <sys/types.h>
#include <sys/wait.h>

#include <string>

#include <pmc.h>
#include <pmcformat.h>
#include <pmclog.h>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <unistd.h>

#define	TEST_PMCID	0x10203040U
#define	TEST_PID	4242
#define	TEST_TSC	0x0102030405060708ULL

static ssize_t
write_full(int fd, const void *buffer, size_t length)
{
	const char *cursor;
	ssize_t nwritten;
	size_t remaining;

	cursor = static_cast<const char *>(buffer);
	remaining = length;
	while (remaining != 0) {
		nwritten = write(fd, cursor, remaining);
		if (nwritten < 0) {
			if (errno == EINTR)
				continue;
			return (-1);
		}
		cursor += nwritten;
		remaining -= static_cast<size_t>(nwritten);
	}
	return (static_cast<ssize_t>(length));
}

int
main()
{
	static const char expected[] =
	    "{\"type\": \"pmcgroupinheritmiss\", "
	    "\"tsc\": \"72623859790382856\", "
	    "\"pmcid\": \"0x10203040\", \"pid\": \"4242\"}\n";
	struct pmclog_ev event;
	char output[sizeof(expected) + 32];
	std::string json;
	pid_t child, waited;
	ssize_t nread;
	size_t used;
	int pipefd[2], status;

	if (pipe(pipefd) != 0) {
		std::printf("FAIL: pipe: %s\n", std::strerror(errno));
		return (1);
	}

	child = fork();
	if (child < 0) {
		std::printf("FAIL: fork: %s\n", std::strerror(errno));
		return (1);
	}
	if (child == 0) {
		(void)close(pipefd[0]);
		std::memset(&event, 0, sizeof(event));
		event.pl_type = PMCLOG_TYPE_PMCGROUPINHERITMISS;
		event.pl_tsc = TEST_TSC;
		event.pl_u.pl_gim.pl_pmcid = TEST_PMCID;
		event.pl_u.pl_gim.pl_pid = TEST_PID;
		json = event_to_json(&event);
		if (write_full(pipefd[1], json.data(), json.size()) < 0)
			_exit(2);
		(void)close(pipefd[1]);
		_exit(0);
	}

	(void)close(pipefd[1]);
	used = 0;
	while (used < sizeof(output) - 1) {
		nread = read(pipefd[0], output + used, sizeof(output) - 1 - used);
		if (nread == 0)
			break;
		if (nread < 0) {
			if (errno == EINTR)
				continue;
			std::printf("FAIL: read: %s\n", std::strerror(errno));
			return (1);
		}
		used += static_cast<size_t>(nread);
	}
	output[used] = '\0';
	(void)close(pipefd[0]);

	do {
		waited = waitpid(child, &status, 0);
	} while (waited < 0 && errno == EINTR);
	if (waited != child) {
		std::printf("FAIL: waitpid: %s\n", std::strerror(errno));
		return (1);
	}
	if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) {
		std::printf("FAIL: event_to_json child status=%d, expected 0\n",
		    status);
		std::printf("Results: 0 pass, 1 fail\n");
		return (1);
	}
	if (std::strcmp(output, expected) != 0) {
		std::printf("FAIL: JSON=%s, expected=%s", output, expected);
		std::printf("Results: 0 pass, 1 fail\n");
		return (1);
	}

	std::printf("PASS: group-inherit-miss JSON matches\n");
	std::printf("Results: 1 pass, 0 fail\n");
	return (0);
}
