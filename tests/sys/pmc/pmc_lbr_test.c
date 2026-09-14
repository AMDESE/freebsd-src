/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Advanced Micro Devices, Inc.
 */

#include <sys/types.h>
#include <sys/errno.h>

#include <atf-c.h>
#include <hwpmc_amd_lbr.h>
#include <stdint.h>

/*
 * Exercise the production allocation policy without PMC syscalls or hardware.
 * Calls are sequential, as they are under pmc_sx in the kernel.  The filter
 * values are opaque here; capability normalization belongs to the MD caller.
 */
#define	NCPU	3
#define	FILTER_A	UINT64_C(1)
#define	FILTER_B	UINT64_C(2)

struct reservations {
	struct amd_lbr_filter ts;
	struct amd_lbr_filter ss[NCPU];
};

static void
check_reservations(const struct reservations *actual,
    const struct reservations *expected)
{
	u_int cpu;

	ATF_CHECK_EQ(expected->ts.select, actual->ts.select);
	ATF_CHECK_EQ(expected->ts.refs, actual->ts.refs);
	for (cpu = 0; cpu < NCPU; cpu++) {
		ATF_CHECK_EQ_MSG(expected->ss[cpu].select, actual->ss[cpu].select,
		    "SS filter changed on CPU %u", cpu);
		ATF_CHECK_EQ_MSG(expected->ss[cpu].refs, actual->ss[cpu].refs,
		    "SS reference count changed on CPU %u", cpu);
	}
}

static void
reject_filter(struct reservations *r, int is_thread, u_int cpu,
    uint64_t select)
{
	struct reservations before;

	before = *r;
	ATF_REQUIRE_EQ(EBUSY, amd_lbr_filter_reserve(&r->ts, r->ss, NCPU,
	    is_thread, cpu, select));
	check_reservations(r, &before);
}

ATF_TC(lbr_ts_references);
ATF_TC_HEAD(lbr_ts_references, tc)
{
	atf_tc_set_md_var(tc, "descr",
	    "TS reservations share one filter until the last owner releases it");
}
ATF_TC_BODY(lbr_ts_references, tc)
{
	struct reservations r = {0}, empty = {0};
	u_int owner;

	for (owner = 0; owner < 3; owner++) {
		ATF_REQUIRE_EQ(0, amd_lbr_filter_reserve(&r.ts, r.ss, NCPU,
		    1, owner, FILTER_A));
		ATF_CHECK_EQ(owner + 1, r.ts.refs);
		ATF_CHECK_EQ(FILTER_A, r.ts.select);
	}
	reject_filter(&r, 1, 0, FILTER_B);

	/*
	 * There is no start/stop operation in this policy.  Allocated, stopped
	 * PMCs retain their references; only release may make a filter reusable.
	 */
	for (owner = 2; owner > 0; owner--) {
		amd_lbr_filter_release(&r.ts);
		ATF_CHECK_EQ(owner, r.ts.refs);
		ATF_CHECK_EQ(FILTER_A, r.ts.select);
		reject_filter(&r, 1, 0, FILTER_B);
	}
	amd_lbr_filter_release(&r.ts);
	check_reservations(&r, &empty);
	ATF_REQUIRE_EQ(0, amd_lbr_filter_reserve(&r.ts, r.ss, NCPU,
	    1, 0, FILTER_B));
	ATF_CHECK_EQ(FILTER_B, r.ts.select);
	ATF_CHECK_EQ(1, r.ts.refs);
	amd_lbr_filter_release(&r.ts);
	check_reservations(&r, &empty);
}

ATF_TC(lbr_ts_before_ss);
ATF_TC_HEAD(lbr_ts_before_ss, tc)
{
	atf_tc_set_md_var(tc, "descr",
	    "a TS reservation constrains SS on every CPU, including idle CPUs");
}
ATF_TC_BODY(lbr_ts_before_ss, tc)
{
	struct reservations r = {0}, empty = {0};
	u_int cpu;

	ATF_REQUIRE_EQ(0, amd_lbr_filter_reserve(&r.ts, r.ss, NCPU,
	    1, 0, FILTER_A));
	for (cpu = 0; cpu < NCPU; cpu++) {
		reject_filter(&r, 0, cpu, FILTER_B);
		ATF_REQUIRE_EQ(0, amd_lbr_filter_reserve(&r.ts, r.ss, NCPU,
		    0, cpu, FILTER_A));
		ATF_CHECK_EQ(1, r.ss[cpu].refs);
		ATF_CHECK_EQ(FILTER_A, r.ss[cpu].select);
	}
	ATF_CHECK_EQ(1, r.ts.refs);
	ATF_CHECK_EQ(FILTER_A, r.ts.select);
	amd_lbr_filter_release(&r.ts);
	/* Releasing TS must not release any SS owner's reservation. */
	reject_filter(&r, 1, 0, FILTER_B);
	for (cpu = 0; cpu < NCPU; cpu++)
		amd_lbr_filter_release(&r.ss[cpu]);
	check_reservations(&r, &empty);
}

ATF_TC(lbr_ss_before_ts);
ATF_TC_HEAD(lbr_ss_before_ts, tc)
{
	atf_tc_set_md_var(tc, "descr",
	    "TS allocation checks even the last CPU's SS reservation");
}
ATF_TC_BODY(lbr_ss_before_ts, tc)
{
	struct reservations r = {0}, empty = {0};

	ATF_REQUIRE_EQ(0, amd_lbr_filter_reserve(&r.ts, r.ss, NCPU,
	    0, NCPU - 1, FILTER_B));
	reject_filter(&r, 1, 0, FILTER_A);
	ATF_REQUIRE_EQ(0, amd_lbr_filter_reserve(&r.ts, r.ss, NCPU,
	    1, 0, FILTER_B));
	ATF_CHECK_EQ(FILTER_B, r.ts.select);
	ATF_CHECK_EQ(1, r.ts.refs);
	ATF_CHECK_EQ(FILTER_B, r.ss[NCPU - 1].select);
	ATF_CHECK_EQ(1, r.ss[NCPU - 1].refs);
	amd_lbr_filter_release(&r.ss[NCPU - 1]);
	reject_filter(&r, 0, NCPU - 1, FILTER_A);
	amd_lbr_filter_release(&r.ts);
	check_reservations(&r, &empty);
}

ATF_TC(lbr_disjoint_ss);
ATF_TC_HEAD(lbr_disjoint_ss, tc)
{
	atf_tc_set_md_var(tc, "descr",
	    "SS filters may differ across CPUs but share references on one CPU");
}
ATF_TC_BODY(lbr_disjoint_ss, tc)
{
	struct reservations r = {0}, empty = {0};

	ATF_REQUIRE_EQ(0, amd_lbr_filter_reserve(&r.ts, r.ss, NCPU,
	    0, 0, FILTER_A));
	ATF_REQUIRE_EQ(0, amd_lbr_filter_reserve(&r.ts, r.ss, NCPU,
	    0, NCPU - 1, FILTER_B));
	ATF_REQUIRE_EQ(0, amd_lbr_filter_reserve(&r.ts, r.ss, NCPU,
	    0, 0, FILTER_A));
	ATF_CHECK_EQ(2, r.ss[0].refs);
	ATF_CHECK_EQ(FILTER_A, r.ss[0].select);
	ATF_CHECK_EQ(1, r.ss[NCPU - 1].refs);
	ATF_CHECK_EQ(FILTER_B, r.ss[NCPU - 1].select);
	reject_filter(&r, 0, 0, FILTER_B);
	reject_filter(&r, 1, 0, FILTER_A);
	reject_filter(&r, 1, 0, FILTER_B);
	amd_lbr_filter_release(&r.ss[0]);
	ATF_CHECK_EQ(1, r.ss[0].refs);
	ATF_CHECK_EQ(FILTER_A, r.ss[0].select);
	reject_filter(&r, 0, 0, FILTER_B);
	amd_lbr_filter_release(&r.ss[0]);
	ATF_REQUIRE_EQ(0, amd_lbr_filter_reserve(&r.ts, r.ss, NCPU,
	    0, 0, FILTER_B));
	ATF_REQUIRE_EQ(0, amd_lbr_filter_reserve(&r.ts, r.ss, NCPU,
	    1, 0, FILTER_B));
	amd_lbr_filter_release(&r.ts);
	amd_lbr_filter_release(&r.ss[0]);
	amd_lbr_filter_release(&r.ss[NCPU - 1]);
	check_reservations(&r, &empty);
}

ATF_TC(lbr_allocation_rollback);
ATF_TC_HEAD(lbr_allocation_rollback, tc)
{
	atf_tc_set_md_var(tc, "descr",
	    "releasing a successful reservation restores existing owners "
	    "after a later allocation failure");
}
ATF_TC_BODY(lbr_allocation_rollback, tc)
{
	struct reservations r = {0}, before, empty = {0};

	/* Zero is a valid filter, not an indication that the slot is free. */
	ATF_REQUIRE_EQ(0, amd_lbr_filter_reserve(&r.ts, r.ss, NCPU,
	    1, 0, 0));
	ATF_REQUIRE_EQ(0, amd_lbr_filter_reserve(&r.ts, r.ss, NCPU,
	    0, 1, 0));
	reject_filter(&r, 1, 0, FILTER_A);
	reject_filter(&r, 0, 1, FILTER_A);
	before = r;

	ATF_REQUIRE_EQ(0, amd_lbr_filter_reserve(&r.ts, r.ss, NCPU,
	    1, 0, 0));
	amd_lbr_filter_release(&r.ts);
	check_reservations(&r, &before);
	ATF_REQUIRE_EQ(0, amd_lbr_filter_reserve(&r.ts, r.ss, NCPU,
	    0, 1, 0));
	amd_lbr_filter_release(&r.ss[1]);
	check_reservations(&r, &before);
	ATF_REQUIRE_EQ(0, amd_lbr_filter_reserve(&r.ts, r.ss, NCPU,
	    0, 2, 0));
	amd_lbr_filter_release(&r.ss[2]);
	check_reservations(&r, &before);

	amd_lbr_filter_release(&r.ts);
	amd_lbr_filter_release(&r.ss[1]);
	check_reservations(&r, &empty);
}

ATF_TP_ADD_TCS(tp)
{
	ATF_TP_ADD_TC(tp, lbr_ts_references);
	ATF_TP_ADD_TC(tp, lbr_ts_before_ss);
	ATF_TP_ADD_TC(tp, lbr_ss_before_ts);
	ATF_TP_ADD_TC(tp, lbr_disjoint_ss);
	ATF_TP_ADD_TC(tp, lbr_allocation_rollback);

	return (atf_no_error());
}
