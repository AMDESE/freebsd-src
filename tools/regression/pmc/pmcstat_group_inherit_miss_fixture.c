/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Emit one deterministic grouped fork-inheritance miss record.
 */

#include <sys/types.h>
#include <sys/pmclog.h>

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define	TEST_PMCID	0x10203040U
#define	TEST_PID	4242U
#define	TEST_TSC	0x0102030405060708ULL

int
main(void)
{
	struct pmclog_pmcgroupinheritmiss record;

	memset(&record, 0, sizeof(record));
	record.pl_header = (PMCLOG_HEADER_MAGIC << 24) |
	    (PMCLOG_TYPE_PMCGROUPINHERITMISS << 16) | sizeof(record);
	record.pl_tsc = TEST_TSC;
	record.pl_pmcid = TEST_PMCID;
	record.pl_pid = TEST_PID;

	if (fwrite(&record, sizeof(record), 1, stdout) != 1) {
		perror("fwrite");
		return (1);
	}
	if (fflush(stdout) != 0) {
		perror("fflush");
		return (1);
	}
	return (0);
}
