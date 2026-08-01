/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Advanced Micro Devices, Inc.
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

/*
 * ibs_errata_spoof - patch the CPUID data inside a pmc(1) performance log so
 * a capture from unaffected silicon decodes as AMD Family 19h Model 0Fh
 * (Zen3-B0). This exercises the IBS errata sanitization (#1197/#1238/#1293/
 * #1347) in usr.sbin/pmc, which is gated on the family/model decoded from the
 * CPUID leaves stored in the log header.
 *
 * The pmc log begins with a struct pmchdr_header, followed by a sequence of
 * (struct pmchdr_infohdr + payload) info blocks terminated by an info block
 * of type INFOHDR_TYPE_DONE, followed by the raw pmclog event stream.
 *
 * The CPUID info block (INFOHDR_TYPE_CPUID) payload is an array of uint32_t
 * words. The consumer (usr.sbin/pmc/view.cc, process_cpuidinfo) walks it as:
 *
 *   len = length / 4; offset = 0;
 *   while (offset < len) {
 *       maxleaf = words[offset];        // descriptor == root leaf's eax
 *       root    = maxleaf & 0xFFFF0000; // 0x00000000 or 0x80000000
 *       count   = maxleaf & 0x0000FFFF; // extra leaves after the root
 *       for (i = 0; i <= count; i++)    // leaf (root+i) = 4 words at offset+4*i
 *           eax=words[offset+4*i+0], ebx=+1, ecx=+2, edx=+3;
 *       offset += 4 * (count + 1);
 *   }
 *
 * The descriptor word words[offset] is ALSO the eax of the root leaf (x86
 * convention). Therefore we must NOT touch words[offset] (i==0's eax) - it
 * drives the parse loop. To spoof:
 *   - vendor  (leaf 0, root==0x00000000, i==0): patch ebx/ecx/edx.
 *   - fam/mdl (leaf 1, root==0x00000000, i==1): patch eax (words[offset+4]),
 *     which is safe because leaf 1 is not a root leaf.
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <err.h>

#include "headers.hh"

static const char *outpath;
static int outdone;

/*
 * Every error path below exits mid-stream, leaving an output file that still
 * carries a valid magic. Remove it so a truncated log is never mistaken for a
 * spoofed one.
 */
static void
cleanup(void)
{
	if (outpath != NULL && !outdone)
		unlink(outpath);
}

/*
 * Build the CPUID leaf-1 eax family/model/stepping signature. For
 * fam=0x19, model=0x0F, step=0 this yields 0x00a00ff0, which the consumer's
 * CPUID_TO_FAMILY/CPUID_TO_MODEL macros decode back to family 0x19, model
 * 0x0F.
 */
static uint32_t
mk_sig(unsigned fam, unsigned model, unsigned step)
{
	uint32_t base_family = (fam >= 0xF) ? 0xF : fam;
	uint32_t ext_family  = (fam >= 0xF) ? (fam - 0xF) : 0;
	uint32_t base_model  = model & 0xF;
	uint32_t ext_model   = (model >> 4) & 0xF;

	return (step & 0xF) | (base_model << 4) | (base_family << 8) |
	    (ext_model << 16) | (ext_family << 20);
}

static void
xfread(void *buf, size_t sz, FILE *fp, const char *what)
{
	if (sz == 0)
		return;
	if (fread(buf, 1, sz, fp) != sz)
		errx(1, "short read on %s", what);
}

static void
xfwrite(const void *buf, size_t sz, FILE *fp, const char *what)
{
	if (sz == 0)
		return;
	if (fwrite(buf, 1, sz, fp) != sz)
		errx(1, "short write on %s", what);
}

/*
 * Patch a CPUID info block payload in place: spoof AMD vendor in leaf 0 and
 * the Family 19h/Model 0Fh signature in leaf 1 of the standard run
 * (root == 0x00000000). AMD "AuthenticAMD" encodes as ebx="Auth", edx="enti",
 * ecx="cAMD" (see AMD_VENDOR_ID in the consumer).
 */
static void
patch_cpuid(uint32_t *words, size_t len)
{
	size_t offset = 0;

	while (offset < len) {
		uint32_t maxleaf = words[offset];
		uint32_t root = maxleaf & 0xFFFF0000;
		uint32_t count = maxleaf & 0x0000FFFF;

		if (root == 0x00000000) {
			/* Bounds: need leaf 0 (4 words) and leaf 1 (i==1). */
			if (offset + 3 >= len)
				errx(1, "truncated CPUID leaf 0");

			/* Leaf 0 vendor: ebx/ecx/edx (do NOT touch eax). */
			memcpy(&words[offset + 1], "Auth", 4); /* ebx */
			memcpy(&words[offset + 2], "cAMD", 4); /* ecx */
			memcpy(&words[offset + 3], "enti", 4); /* edx */

			if (count < 1 || offset + 4 >= len)
				errx(1, "CPUID run has no leaf 1 to patch");

			/* Leaf 1 eax: family/model signature (safe, not root). */
			words[offset + 4] = mk_sig(0x19, 0x0F, 0);
		}

		offset += 4 * (count + 1);
	}
}

int
main(int argc, char *argv[])
{
	FILE *in, *out;
	struct pmchdr_header hdr;
	struct pmchdr_infohdr info;
	uint32_t *payload;
	unsigned char copybuf[65536];
	size_t nwords;
	size_t n;

	if (argc != 3)
		errx(1, "usage: %s <in.pmc> <out.pmc>", argv[0]);

	in = fopen(argv[1], "rb");
	if (in == NULL)
		err(1, "open %s", argv[1]);
	out = fopen(argv[2], "wb");
	if (out == NULL)
		err(1, "open %s", argv[2]);
	outpath = argv[2];
	if (atexit(cleanup) != 0)
		err(1, "atexit");

	/* Header. */
	xfread(&hdr, sizeof(hdr), in, "header");
	if (hdr.magic != PMC_HEADER_MAGIC)
		errx(1, "%s: not a pmc log (magic 0x%08x)", argv[1], hdr.magic);
	if (hdr.version != PMC_HEADER_VERSION)
		errx(1, "%s: unsupported pmc log version %u (expected %u)",
		    argv[1], hdr.version, PMC_HEADER_VERSION);
	xfwrite(&hdr, sizeof(hdr), out, "header");

	/* Info blocks, up to and including INFOHDR_TYPE_DONE. */
	for (;;) {
		xfread(&info, sizeof(info), in, "infohdr");
		xfwrite(&info, sizeof(info), out, "infohdr");

		if (info.type == INFOHDR_TYPE_DONE)
			break;

		/*
		 * Allocate as uint32_t so the buffer is naturally aligned for
		 * the CPUID walk. Round the word count up to cover a payload
		 * whose byte length is not a multiple of 4.
		 */
		nwords = (info.length + 3) / 4;
		payload = calloc(nwords ? nwords : 1, sizeof(uint32_t));
		if (payload == NULL)
			err(1, "calloc");
		xfread(payload, info.length, in, "info payload");

		if (info.type == INFOHDR_TYPE_CPUID)
			patch_cpuid(payload, info.length / 4);

		xfwrite(payload, info.length, out, "info payload");
		free(payload);
	}

	/* Copy the rest of the file (pmclog event stream) verbatim. */
	while ((n = fread(copybuf, 1, sizeof(copybuf), in)) > 0)
		xfwrite(copybuf, n, out, "event stream");
	if (ferror(in))
		errx(1, "read error on %s", argv[1]);

	if (fclose(out) != 0)
		err(1, "close %s", argv[2]);
	outdone = 1;
	fclose(in);

	return (0);
}
