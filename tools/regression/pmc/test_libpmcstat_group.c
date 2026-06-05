/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Advanced Micro Devices, Inc.
 *
 * Userland unit test for pmcstat_parse_event_group().
 *
 * Build:  cc -o test_libpmcstat_group test_libpmcstat_group.c -lpmcstat
 * Run:    ./test_libpmcstat_group   (no root, no kernel module needed)
 *
 * The test exercises the brace-list parser only; no PMU is touched.
 * Exit status:
 *     0  - all assertions pass
 *     1  - one or more cases failed
 */

#include <sys/types.h>
#include <sys/queue.h>

#include <errno.h>
#include <pmc.h>
#include <pmclog.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "libpmcstat.h"

static int n_pass;
static int n_fail;

#define	CHECK(cond, fmt, ...) do {					\
	if (cond) {							\
		printf("PASS: " fmt "\n", ##__VA_ARGS__);		\
		n_pass++;						\
	} else {							\
		printf("FAIL: " fmt " (line %d)\n",			\
		    ##__VA_ARGS__, __LINE__);				\
		n_fail++;						\
	}								\
} while (0)

static void
free_arr(char **a, size_t n)
{

	pmcstat_free_event_group(a, n);
}

/* {a,b,c} -> 3 elements, returns 0 (real group). */
static void
t_basic(void)
{
	char **out = NULL;
	size_t n = 0;
	int rv;

	rv = pmcstat_parse_event_group("{a,b,c}", &out, &n);
	CHECK(rv == 0, "basic: rv=%d, want 0", rv);
	CHECK(n == 3, "basic: n=%zu, want 3", n);
	if (rv == 0 && n == 3) {
		CHECK(strcmp(out[0], "a") == 0, "basic[0]=%s", out[0]);
		CHECK(strcmp(out[1], "b") == 0, "basic[1]=%s", out[1]);
		CHECK(strcmp(out[2], "c") == 0, "basic[2]=%s", out[2]);
	}
	free_arr(out, n);
}

/* {ev} (single element) -> rv==1 (degenerate, caller handles as non-group). */
static void
t_single_element(void)
{
	char **out = NULL;
	size_t n = 0;
	int rv;

	rv = pmcstat_parse_event_group("{instructions}", &out, &n);
	CHECK(rv == 1, "single: rv=%d, want 1", rv);
	free_arr(out, n);
}

/* No leading '{' -> rv==1 (not a brace list). */
static void
t_not_a_group(void)
{
	char **out = NULL;
	size_t n = 0;
	int rv;

	rv = pmcstat_parse_event_group("instructions", &out, &n);
	CHECK(rv == 1, "not_a_group: rv=%d, want 1", rv);
	CHECK(out == NULL, "not_a_group: out should be NULL");
	free_arr(out, n);
}

/* No closing '}' -> rv==-1 (parse error), errno=EINVAL. */
static void
t_no_close_brace(void)
{
	char **out = NULL;
	size_t n = 0;
	int rv;

	errno = 0;
	rv = pmcstat_parse_event_group("{a,b,c", &out, &n);
	CHECK(rv == -1, "no_close: rv=%d, want -1", rv);
	CHECK(errno == EINVAL, "no_close: errno=%d, want EINVAL", errno);
	free_arr(out, n);
}

/* {a, b , c} with whitespace around commas -> 3 trimmed elements. */
static void
t_whitespace(void)
{
	char **out = NULL;
	size_t n = 0;
	int rv;

	rv = pmcstat_parse_event_group("{ a , b ,  c }", &out, &n);
	CHECK(rv == 0, "ws: rv=%d, want 0", rv);
	CHECK(n == 3, "ws: n=%zu, want 3", n);
	if (rv == 0 && n == 3) {
		CHECK(strcmp(out[0], "a") == 0, "ws[0]=%s", out[0]);
		CHECK(strcmp(out[1], "b") == 0, "ws[1]=%s", out[1]);
		CHECK(strcmp(out[2], "c") == 0, "ws[2]=%s", out[2]);
	}
	free_arr(out, n);
}

/* Empty entries collapse: {,a,,b,} -> only "a" and "b". */
static void
t_empty_entries(void)
{
	char **out = NULL;
	size_t n = 0;
	int rv;

	rv = pmcstat_parse_event_group("{,a,,b,}", &out, &n);
	CHECK(rv == 0, "empty: rv=%d, want 0", rv);
	CHECK(n == 2, "empty: n=%zu, want 2", n);
	if (rv == 0 && n == 2) {
		CHECK(strcmp(out[0], "a") == 0, "empty[0]=%s", out[0]);
		CHECK(strcmp(out[1], "b") == 0, "empty[1]=%s", out[1]);
	}
	free_arr(out, n);
}

/* Colon qualifier inside event spec must be preserved untouched. */
static void
t_colon_preserved(void)
{
	char **out = NULL;
	size_t n = 0;
	int rv;

	rv = pmcstat_parse_event_group("{instructions:k,branches:u}",
	    &out, &n);
	CHECK(rv == 0, "colon: rv=%d, want 0", rv);
	CHECK(n == 2, "colon: n=%zu, want 2", n);
	if (rv == 0 && n == 2) {
		CHECK(strcmp(out[0], "instructions:k") == 0,
		    "colon[0]=%s", out[0]);
		CHECK(strcmp(out[1], "branches:u") == 0,
		    "colon[1]=%s", out[1]);
	}
	free_arr(out, n);
}

/* NULL inputs must return -1 with errno EINVAL, not crash. */
static void
t_null_inputs(void)
{
	char **out = NULL;
	size_t n = 0;
	int rv;

	errno = 0;
	rv = pmcstat_parse_event_group(NULL, &out, &n);
	CHECK(rv == -1, "null_spec: rv=%d, want -1", rv);
	CHECK(errno == EINVAL, "null_spec: errno=%d, want EINVAL", errno);

	errno = 0;
	rv = pmcstat_parse_event_group("{a,b}", NULL, &n);
	CHECK(rv == -1, "null_out: rv=%d, want -1", rv);
	CHECK(errno == EINVAL, "null_out: errno=%d, want EINVAL", errno);

	errno = 0;
	rv = pmcstat_parse_event_group("{a,b}", &out, NULL);
	CHECK(rv == -1, "null_n: rv=%d, want -1", rv);
	CHECK(errno == EINVAL, "null_n: errno=%d, want EINVAL", errno);
}

/* A leading-space spec like " {a,b}" should still parse as a group. */
static void
t_leading_ws(void)
{
	char **out = NULL;
	size_t n = 0;
	int rv;

	rv = pmcstat_parse_event_group("   {a,b}", &out, &n);
	CHECK(rv == 0, "lead_ws: rv=%d, want 0", rv);
	CHECK(n == 2, "lead_ws: n=%zu, want 2", n);
	free_arr(out, n);
}

/* pmcstat_add_one_event: verify it appends to args.pa_events with the
 * expected ev_groupid / ev_is_leader / ev_mode for option='p'. */
static void
t_add_one_event(void)
{
	struct pmcstat_args a;
	struct pmcstat_ev *ev;
	int seen;

	memset(&a, 0, sizeof(a));
	STAILQ_INIT(&a.pa_events);

	pmcstat_add_one_event('p', "instructions", &a, /*gid*/ 7,
	    /*is_leader*/ 1);
	pmcstat_add_one_event('p', "branches",     &a, /*gid*/ 7,
	    /*is_leader*/ 0);

	seen = 0;
	STAILQ_FOREACH(ev, &a.pa_events, ev_next) {
		CHECK(ev->ev_groupid == 7, "add: ev_groupid=%d", ev->ev_groupid);
		CHECK(ev->ev_mode == PMC_MODE_TC, "add: ev_mode=%d",
		    (int)ev->ev_mode);
		CHECK(ev->ev_pmcid == PMC_ID_INVALID, "add: ev_pmcid invalid");
		seen++;
	}
	CHECK(seen == 2, "add: saw %d events, want 2", seen);
	CHECK((a.pa_flags & FLAG_HAS_PROCESS_PMCS) != 0,
	    "add: FLAG_HAS_PROCESS_PMCS set");
	CHECK((a.pa_flags & FLAG_HAS_COUNTING_PMCS) != 0,
	    "add: FLAG_HAS_COUNTING_PMCS set");
}

int
main(void)
{

	t_basic();
	t_single_element();
	t_not_a_group();
	t_no_close_brace();
	t_whitespace();
	t_empty_entries();
	t_colon_preserved();
	t_null_inputs();
	t_leading_ws();
	t_add_one_event();

	printf("\nResults: %d pass, %d fail\n", n_pass, n_fail);
	return (n_fail == 0 ? 0 : 1);
}
