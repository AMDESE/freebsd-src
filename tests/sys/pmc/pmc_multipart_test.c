/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 The FreeBSD Foundation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <sys/param.h>
#include <sys/pmc.h>
#include <sys/pmclog.h>

#include <atf-c.h>
#include <pmclog.h>
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define	PMCLOG_TO_HEADER(T, L)					\
	((PMCLOG_HEADER_MAGIC << 24) | ((T) << 16) | ((L) & 0xffff))

static size_t
callchain_record_size(uint32_t npc)
{
	return (offsetof(struct pmclog_callchain, pl_pc) +
	    npc * sizeof(uintfptr_t));
}

static void
init_callchain_record(struct pmclog_callchain *rec, uint32_t npc,
    uint32_t cpuflags)
{
	size_t len;

	len = callchain_record_size(npc);
	memset(rec, 0, len);
	rec->pl_header = PMCLOG_TO_HEADER(PMCLOG_TYPE_CALLCHAIN, len);
	rec->pl_tsc = 0x12345678;
	rec->pl_pid = 42;
	rec->pl_tid = 43;
	rec->pl_pmcid = 44;
	rec->pl_cpuflags = cpuflags;
}

static void
parse_record(const struct pmclog_callchain *rec, uint32_t npc,
    struct pmclog_ev *ev)
{
	void *parser;
	size_t len;

	parser = pmclog_open(PMCLOG_FD_NONE);
	ATF_REQUIRE(parser != NULL);
	len = callchain_record_size(npc);
	ATF_REQUIRE_EQ(0, pmclog_feed(parser, (char *)(uintptr_t)rec, len));
	ATF_REQUIRE_EQ(0, pmclog_read(parser, ev));
	ATF_REQUIRE_EQ(PMCLOG_OK, ev->pl_state);
	ATF_REQUIRE_EQ(PMCLOG_TYPE_CALLCHAIN, ev->pl_type);
	pmclog_close(parser);
}

ATF_TC(multipart_word_arithmetic);
ATF_TC_HEAD(multipart_word_arithmetic, tc)
{
	atf_tc_set_md_var(tc, "descr",
	    "multipart headers and 64-bit payloads are counted in native "
	    "pointer-sized callchain slots");
}
ATF_TC_BODY(multipart_word_arithmetic, tc)
{
	ATF_CHECK_EQ(2, PMC_MULTIPART_HEADER_WORDS_FOR(4));
	ATF_CHECK_EQ(1, PMC_MULTIPART_HEADER_WORDS_FOR(8));
	ATF_CHECK_EQ(2, PMC_MULTIPART_64BIT_WORDS_FOR(1, 4));
	ATF_CHECK_EQ(1, PMC_MULTIPART_64BIT_WORDS_FOR(1, 8));
	ATF_CHECK_EQ(23, PMC_MULTIPART_SAMPLE_MIN_WORDS_FOR(4));
	ATF_CHECK_EQ(12, PMC_MULTIPART_SAMPLE_MIN_WORDS_FOR(8));
	ATF_CHECK_EQ(PMC_MULTIPART_HEADER_WORDS_FOR(sizeof(uintfptr_t)),
	    PMC_MULTIPART_HEADER_WORDS);
}

ATF_TC(multipart_offset_bounds);
ATF_TC_HEAD(multipart_offset_bounds, tc)
{
	atf_tc_set_md_var(tc, "descr",
	    "multipart offset helper handles full payloads, empty records, and "
	    "truncated payloads");
}
ATF_TC_BODY(multipart_offset_bounds, tc)
{
	uint32_t off, payload_words;
	uint8_t *hdr;
	uintfptr_t pc[PMC_MULTIPART_SAMPLE_MIN_WORDS + 1];

	memset(pc, 0, sizeof(pc));
	hdr = (uint8_t *)pc;
	payload_words = PMC_MULTIPART_PAYLOAD_WORDS(PMC_MULTIPART_MAX_PAYLOAD64);
	hdr[0] = PMC_CC_MULTIPART_IBS_OP;
	hdr[1] = payload_words;
	hdr[2] = PMC_CC_MULTIPART_CALLCHAIN;
	hdr[3] = 0;
	ATF_REQUIRE(pmclog_multipart_callchain_offset(pc,
	    PMC_MULTIPART_SAMPLE_MIN_WORDS, &off));
	ATF_CHECK_EQ(PMC_MULTIPART_SAMPLE_MIN_WORDS - 1, off);

	ATF_CHECK(!pmclog_multipart_callchain_offset(pc, 0, &off));
	ATF_CHECK(!pmclog_multipart_callchain_offset(pc,
	    PMC_MULTIPART_HEADER_WORDS - 1, &off));
	ATF_CHECK(!pmclog_multipart_callchain_offset(pc,
	    PMC_MULTIPART_SAMPLE_MIN_WORDS - 2, &off));
}

ATF_TC(pmclog_callchain_single_pc);
ATF_TC_HEAD(pmclog_callchain_single_pc, tc)
{
	atf_tc_set_md_var(tc, "descr",
	    "a callchain record without PMC_F_CALLCHAIN still carries its "
	    "single interrupt PC");
}
ATF_TC_BODY(pmclog_callchain_single_pc, tc)
{
	struct pmclog_callchain rec;
	struct pmclog_ev ev;

	init_callchain_record(&rec, 1, PMC_CALLCHAIN_TO_CPUFLAGS(3,
	    PMC_CC_F_USERSPACE));
	rec.pl_pc[0] = (uintfptr_t)0x1234abcd;
	parse_record(&rec, 1, &ev);
	ATF_CHECK_EQ(1, ev.pl_u.pl_cc.pl_npc);
	ATF_CHECK_EQ(3, PMC_CALLCHAIN_CPUFLAGS_TO_CPU(ev.pl_u.pl_cc.pl_cpuflags));
	ATF_CHECK(PMC_CALLCHAIN_CPUFLAGS_TO_USERMODE(ev.pl_u.pl_cc.pl_cpuflags));
	ATF_CHECK_EQ((uintfptr_t)0x1234abcd, ev.pl_u.pl_cc.pl_pc[0]);
}

ATF_TC(pmclog_callchain_kernel_and_full_depth);
ATF_TC_HEAD(pmclog_callchain_kernel_and_full_depth, tc)
{
	atf_tc_set_md_var(tc, "descr",
	    "the pmclog parser accepts kernel and full-depth callchain records");
}
ATF_TC_BODY(pmclog_callchain_kernel_and_full_depth, tc)
{
	struct pmclog_callchain *rec;
	struct pmclog_ev ev;
	uint32_t i;

	rec = calloc(1, sizeof(*rec));
	ATF_REQUIRE(rec != NULL);
	init_callchain_record(rec, PMC_CALLCHAIN_DEPTH_MAX,
	    PMC_CALLCHAIN_TO_CPUFLAGS(1, 0));
	for (i = 0; i < PMC_CALLCHAIN_DEPTH_MAX; i++)
		rec->pl_pc[i] = (uintfptr_t)(0x1000 + i);
	parse_record(rec, PMC_CALLCHAIN_DEPTH_MAX, &ev);
	ATF_CHECK_EQ(PMC_CALLCHAIN_DEPTH_MAX, ev.pl_u.pl_cc.pl_npc);
	ATF_CHECK_EQ(1, PMC_CALLCHAIN_CPUFLAGS_TO_CPU(ev.pl_u.pl_cc.pl_cpuflags));
	ATF_CHECK(!PMC_CALLCHAIN_CPUFLAGS_TO_USERMODE(ev.pl_u.pl_cc.pl_cpuflags));
	ATF_CHECK_EQ((uintfptr_t)(0x1000 + PMC_CALLCHAIN_DEPTH_MAX - 1),
	    ev.pl_u.pl_cc.pl_pc[PMC_CALLCHAIN_DEPTH_MAX - 1]);
	free(rec);
}

ATF_TC(pmclog_callchain_zero_pcs);
ATF_TC_HEAD(pmclog_callchain_zero_pcs, tc)
{
	atf_tc_set_md_var(tc, "descr",
	    "the pmclog parser accepts an empty callchain record without "
	    "reading stale PCs");
}
ATF_TC_BODY(pmclog_callchain_zero_pcs, tc)
{
	struct pmclog_callchain rec;
	struct pmclog_ev ev;

	init_callchain_record(&rec, 0, PMC_CALLCHAIN_TO_CPUFLAGS(7, 0));
	parse_record(&rec, 0, &ev);
	ATF_CHECK_EQ(0, ev.pl_u.pl_cc.pl_npc);
	ATF_CHECK_EQ(7, PMC_CALLCHAIN_CPUFLAGS_TO_CPU(ev.pl_u.pl_cc.pl_cpuflags));
	ATF_CHECK_EQ((uintfptr_t)0, ev.pl_u.pl_cc.pl_pc[0]);
}

ATF_TC(pmclog_callchain_multipart_minimum_capacity);
ATF_TC_HEAD(pmclog_callchain_multipart_minimum_capacity, tc)
{
	atf_tc_set_md_var(tc, "descr",
	    "a full multipart payload at the minimum sample depth leaves room "
	    "for one callchain PC");
}
ATF_TC_BODY(pmclog_callchain_multipart_minimum_capacity, tc)
{
	struct pmclog_callchain rec;
	struct pmclog_ev ev;
	uint32_t off, payload_words;
	uint8_t *hdr;

	init_callchain_record(&rec, PMC_MULTIPART_SAMPLE_MIN_WORDS,
	    PMC_CALLCHAIN_TO_CPUFLAGS(0, PMC_CC_F_MULTIPART));
	hdr = (uint8_t *)rec.pl_pc;
	payload_words = PMC_MULTIPART_PAYLOAD_WORDS(PMC_MULTIPART_MAX_PAYLOAD64);
	hdr[0] = PMC_CC_MULTIPART_IBS_FETCH;
	hdr[1] = payload_words;
	hdr[2] = PMC_CC_MULTIPART_CALLCHAIN;
	rec.pl_pc[PMC_MULTIPART_SAMPLE_MIN_WORDS - 1] = (uintfptr_t)0xfeedface;

	parse_record(&rec, PMC_MULTIPART_SAMPLE_MIN_WORDS, &ev);
	ATF_REQUIRE(pmclog_multipart_callchain_offset(ev.pl_u.pl_cc.pl_pc,
	    ev.pl_u.pl_cc.pl_npc, &off));
	ATF_CHECK_EQ(PMC_MULTIPART_SAMPLE_MIN_WORDS - 1, off);
	ATF_CHECK_EQ((uintfptr_t)0xfeedface, ev.pl_u.pl_cc.pl_pc[off]);
}

ATF_TC(pmclog_callchain_fragmented_record);
ATF_TC_HEAD(pmclog_callchain_fragmented_record, tc)
{
	atf_tc_set_md_var(tc, "descr",
	    "the pmclog parser waits for the remainder of a truncated record");
}
ATF_TC_BODY(pmclog_callchain_fragmented_record, tc)
{
	struct pmclog_callchain rec;
	struct pmclog_ev ev;
	void *parser;
	size_t len;

	init_callchain_record(&rec, 1, PMC_CALLCHAIN_TO_CPUFLAGS(2,
	    PMC_CC_F_USERSPACE));
	rec.pl_pc[0] = (uintfptr_t)0xabcdef;
	len = callchain_record_size(1);
	parser = pmclog_open(PMCLOG_FD_NONE);
	ATF_REQUIRE(parser != NULL);
	ATF_REQUIRE_EQ(0, pmclog_feed(parser, (char *)&rec, len - 1));
	ATF_CHECK_EQ(-1, pmclog_read(parser, &ev));
	ATF_CHECK_EQ(PMCLOG_REQUIRE_DATA, ev.pl_state);
	ATF_REQUIRE_EQ(0, pmclog_feed(parser, ((char *)&rec) + len - 1, 1));
	ATF_REQUIRE_EQ(0, pmclog_read(parser, &ev));
	ATF_REQUIRE_EQ(PMCLOG_OK, ev.pl_state);
	ATF_CHECK_EQ(1, ev.pl_u.pl_cc.pl_npc);
	ATF_CHECK_EQ((uintfptr_t)0xabcdef, ev.pl_u.pl_cc.pl_pc[0]);
	pmclog_close(parser);
}

ATF_TC(pmclog_rejects_malformed_lengths);
ATF_TC_HEAD(pmclog_rejects_malformed_lengths, tc)
{
	atf_tc_set_md_var(tc, "descr",
	    "pmclog rejects zero, oversized, and unaligned record lengths");
}
ATF_TC_BODY(pmclog_rejects_malformed_lengths, tc)
{
	struct pmclog_callchain rec;
	struct pmclog_ev ev;
	void *parser;
	size_t badlen;

	memset(&rec, 0, sizeof(rec));
	rec.pl_header = PMCLOG_TO_HEADER(PMCLOG_TYPE_CALLCHAIN, 0);
	parser = pmclog_open(PMCLOG_FD_NONE);
	ATF_REQUIRE(parser != NULL);
	ATF_REQUIRE_EQ(0, pmclog_feed(parser, (char *)&rec, sizeof(uint32_t)));
	ATF_CHECK_EQ(-1, pmclog_read(parser, &ev));
	ATF_CHECK_EQ(PMCLOG_ERROR, ev.pl_state);
	pmclog_close(parser);

	memset(&rec, 0, sizeof(rec));
	rec.pl_header = PMCLOG_TO_HEADER(PMCLOG_TYPE_CALLCHAIN,
	    sizeof(union pmclog_entry) + sizeof(uint32_t));
	parser = pmclog_open(PMCLOG_FD_NONE);
	ATF_REQUIRE(parser != NULL);
	ATF_REQUIRE_EQ(0, pmclog_feed(parser, (char *)&rec, sizeof(uint32_t)));
	ATF_CHECK_EQ(-1, pmclog_read(parser, &ev));
	ATF_CHECK_EQ(PMCLOG_ERROR, ev.pl_state);
	pmclog_close(parser);

	badlen = offsetof(struct pmclog_callchain, pl_pc) + 1;
	memset(&rec, 0, sizeof(rec));
	rec.pl_header = PMCLOG_TO_HEADER(PMCLOG_TYPE_CALLCHAIN, badlen);
	parser = pmclog_open(PMCLOG_FD_NONE);
	ATF_REQUIRE(parser != NULL);
	ATF_REQUIRE_EQ(0, pmclog_feed(parser, (char *)&rec, badlen));
	ATF_CHECK_EQ(-1, pmclog_read(parser, &ev));
	ATF_CHECK_EQ(PMCLOG_ERROR, ev.pl_state);
	pmclog_close(parser);
}

ATF_TP_ADD_TCS(tp)
{
	ATF_TP_ADD_TC(tp, multipart_word_arithmetic);
	ATF_TP_ADD_TC(tp, multipart_offset_bounds);
	ATF_TP_ADD_TC(tp, pmclog_callchain_single_pc);
	ATF_TP_ADD_TC(tp, pmclog_callchain_kernel_and_full_depth);
	ATF_TP_ADD_TC(tp, pmclog_callchain_zero_pcs);
	ATF_TP_ADD_TC(tp, pmclog_callchain_multipart_minimum_capacity);
	ATF_TP_ADD_TC(tp, pmclog_callchain_fragmented_record);
	ATF_TP_ADD_TC(tp, pmclog_rejects_malformed_lengths);

	return (atf_no_error());
}
