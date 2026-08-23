/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Regression test for parsing a grouped fork-inheritance miss record.
 */

#include <sys/types.h>

#include <pmc.h>
#include <pmclog.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "libpmcinternal.h"

#define	TEST_RECORD_TYPE	21
#define	TEST_PMCID		0x10203040U
#define	TEST_PID		4242U
#define	TEST_TSC		0x0102030405060708ULL

#define	TEST_RECORD_HEADER(type, length)					\
	((PMCLOG_HEADER_MAGIC << 24) | ((type) << 16) |			\
	    ((length) & 0xFFFF))

static int n_pass;
static int n_fail;

#define	CHECK(condition, fmt, ...) do {					\
	if (condition) {							\
		printf("PASS: " fmt "\n", ##__VA_ARGS__);			\
		n_pass++;							\
	} else {								\
		printf("FAIL: " fmt " (line %d)\n", ##__VA_ARGS__,		\
		    __LINE__);							\
		n_fail++;							\
	}									\
} while (0)

/*
 * pmclog.c references these event-name helpers from an unrelated parse case.
 * The synthetic record never calls them; local stubs keep this test linked to
 * the real parser translation unit without pulling in the rest of libpmc.
 */
const char *
_pmc_name_of_event(enum pmc_event event, enum pmc_cputype cpu)
{

	(void)event;
	(void)cpu;
	return (NULL);
}

const char *
pmc_pmu_event_get_by_idx(const char *cpuid, int index)
{

	(void)cpuid;
	(void)index;
	return (NULL);
}

int
main(void)
{
	struct pmclog_pmcgroupinheritmiss record;
	struct pmclog_ev event;
	void *parser;
	int rc;

	memset(&record, 0, sizeof(record));
	record.pl_header = TEST_RECORD_HEADER(TEST_RECORD_TYPE, sizeof(record));
	record.pl_tsc = TEST_TSC;
	record.pl_pmcid = TEST_PMCID;
	record.pl_pid = TEST_PID;

	parser = pmclog_open(PMCLOG_FD_NONE);
	if (parser == NULL) {
		printf("FAIL: pmclog_open returned NULL\n");
		return (1);
	}
	rc = pmclog_feed(parser, (char *)(void *)&record, sizeof(record));
	if (rc != 0) {
		printf("FAIL: pmclog_feed returned %d, expected 0\n", rc);
		pmclog_close(parser);
		return (1);
	}

	memset(&event, 0, sizeof(event));
	rc = pmclog_read(parser, &event);
	if (rc != 0) {
		printf("FAIL: group-inherit-miss parse returned %d state=%d, "
		    "expected 0/%d\n", rc, event.pl_state, PMCLOG_OK);
		printf("Results: 0 pass, 1 fail\n");
		pmclog_close(parser);
		return (1);
	}

	CHECK(event.pl_state == PMCLOG_OK, "state=%d, expected %d",
	    event.pl_state, PMCLOG_OK);
	CHECK(PMCLOG_TYPE_PMCGROUPINHERITMISS == TEST_RECORD_TYPE &&
	    event.pl_type == PMCLOG_TYPE_PMCGROUPINHERITMISS,
	    "type=%u enum=%u, expected %u", (u_int)event.pl_type,
	    PMCLOG_TYPE_PMCGROUPINHERITMISS, TEST_RECORD_TYPE);
	CHECK(event.pl_len == sizeof(record), "length=%d, expected %zu",
	    event.pl_len, sizeof(record));
	CHECK(event.pl_tsc == TEST_TSC, "TSC=%#jx, expected %#jx",
	    (uintmax_t)event.pl_tsc, (uintmax_t)TEST_TSC);
	CHECK(event.pl_u.pl_gim.pl_pmcid == TEST_PMCID,
	    "leader pmcid=%#x, expected %#x", event.pl_u.pl_gim.pl_pmcid,
	    TEST_PMCID);
	CHECK(event.pl_u.pl_gim.pl_pid == TEST_PID,
	    "child pid=%u, expected %u", event.pl_u.pl_gim.pl_pid, TEST_PID);

	pmclog_close(parser);
	printf("Results: %d pass, %d fail\n", n_pass, n_fail);
	return (n_fail == 0 ? 0 : 1);
}
