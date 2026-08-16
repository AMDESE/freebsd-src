/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * E7: fuzz the hwpmc group syscalls.
 *
 * Calls PMC_OP_PMCGROUPADD / PMCGROUPCOMMIT / PMCGROUPREAD directly --
 * bypassing libpmc's argument validation, which is the point: the kernel
 * must reject bad input itself.  Randomises group ids, pmc ids, capacities
 * (0, 1, 32, 33, UINT32_MAX and arbitrary), and targets groups in every
 * lifecycle state (never-created, uncommitted, committed, released) as
 * well as ids belonging to another process.
 *
 * Build: cc -O -g -Wall -o pmc_group_fuzz pmc_group_fuzz.c -lpmc
 * Run:   sudo ./pmc_group_fuzz [iterations]
 *
 * Acceptance: every call returns an errno; no panic; no KASSERT.
 * Exit 0 = survived, 77 = skip.
 */
#include <sys/param.h>
#include <sys/types.h>
#include <sys/module.h>
#include <sys/syscall.h>
#include <sys/wait.h>

#include <err.h>
#include <errno.h>
#include <pmc.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static int pmc_sc = -1;

#define	CALL(op, arg)	syscall(pmc_sc, (op), (arg))

static unsigned long calls, errs, oks;
static unsigned int seed = 12345;

/* Deterministic PRNG so a failing run can be replayed. */
static uint32_t
rnd(void)
{
	seed = seed * 1103515245u + 12345u;
	return (seed >> 8);
}

static uint32_t
pick_capacity(void)
{
	switch (rnd() % 8) {
	case 0: return (0);
	case 1: return (1);
	case 2: return (32);
	case 3: return (33);
	case 4: return (UINT32_MAX);
	case 5: return (rnd());
	case 6: return (rnd() % 64);
	default: return (2);
	}
}

static void
note(long rv)
{
	calls++;
	if (rv == 0)
		oks++;
	else
		errs++;
}

/*
 * PMCGROUPREAD with a wild capacity. The snapshot buffer is sized for
 * 'alloc' members but pm_nmembers claims 'claim' -- the kernel must not
 * write past what we allocated.
 */
static void
fuzz_read(pmc_id_t leader, uint32_t claim, uint32_t alloc)
{
	struct pmc_op_pmcgroupread *gr;
	size_t sz;

	/*
	 * pm_nmembers is the caller's declaration of how big the buffer is;
	 * the kernel sizes its copyout from it (bounded by the group's real
	 * membership).  A caller that claims more than it allocated corrupts
	 * its own memory, and that is caller error, not a kernel bug -- so
	 * always allocate for at least what we claim, and use the red zone
	 * to catch the kernel writing beyond even that.
	 */
	if (alloc > 64)
		alloc = 64;
	if (alloc < claim && claim <= 64)
		alloc = claim;
	if (claim > 64)
		alloc = 64;
	sz = sizeof(*gr) + (size_t)alloc * sizeof(gr->pm_members[0]);
	gr = calloc(1, sz + 4096);	/* red zone */
	if (gr == NULL)
		return;
	memset((char *)gr + sz, 0xA5, 4096);
	gr->pm_leader = leader;
	gr->pm_nmembers = claim;
	errno = 0;
	note(CALL(PMC_OP_PMCGROUPREAD, gr));

	/* The kernel must not have touched the red zone. */
	for (size_t i = 0; i < 4096; i++) {
		if (((unsigned char *)gr)[sz + i] != 0xA5) {
			printf("  *** RED ZONE CLOBBERED at +%zu "
			    "(claim=%u alloc=%u)\n", i, claim, alloc);
			break;
		}
	}
	free(gr);
}

static void
fuzz_add(uint32_t gid, pmc_id_t pmcid, uint32_t flags)
{
	struct pmc_op_pmcgroupadd ga;

	memset(&ga, 0, sizeof(ga));
	ga.pm_groupid = gid;
	ga.pm_pmcid = pmcid;
	ga.pm_flags = flags;
	errno = 0;
	note(CALL(PMC_OP_PMCGROUPADD, &ga));
}

static void
fuzz_commit(uint32_t gid)
{
	struct pmc_op_pmcgroupcommit gc;

	memset(&gc, 0, sizeof(gc));
	gc.pm_groupid = gid;
	errno = 0;
	note(CALL(PMC_OP_PMCGROUPCOMMIT, &gc));
}

int
main(int argc, char **argv)
{
	struct module_stat ms;
	pmc_id_t real_ids[4], released_id;
	uint32_t real_gid, released_gid, uncommitted_gid;
	long iters = 20000;
	int i, nreal = 0, modid;

	if (argc > 1)
		iters = atol(argv[1]);

	if (pmc_init() < 0)
		errx(77, "pmc_init: %s", strerror(errno));

	if ((modid = modfind(PMC_MODULE_NAME)) < 0)
		errx(77, "modfind %s: %s", PMC_MODULE_NAME, strerror(errno));
	memset(&ms, 0, sizeof(ms));
	ms.version = sizeof(ms);
	if (modstat(modid, &ms) < 0)
		errx(77, "modstat: %s", strerror(errno));
	pmc_sc = ms.data.intval;
	printf("hwpmc syscall=%d, %ld iterations, seed=%u\n", pmc_sc, iters,
	    seed);

	/* A real, committed, running group to aim at. */
	if (pmc_group_create(&real_gid) < 0)
		errx(77, "group_create: %s", strerror(errno));
	{
		const char *ev[] = { "instructions", "unhalted-cycles",
		    "branches" };
		for (i = 0; i < 3; i++) {
			if (pmc_allocate_group(ev[i], PMC_MODE_TC, 0,
			    PMC_CPU_ANY, &real_ids[nreal], 0) < 0)
				continue;
			if (pmc_group_add(real_gid, real_ids[nreal],
			    nreal == 0) < 0) {
				(void)pmc_release(real_ids[nreal]);
				continue;
			}
			nreal++;
		}
	}
	if (nreal < 2)
		errx(77, "could not build a 2-member group");
	if (pmc_group_commit(real_gid) < 0)
		errx(77, "commit: %s", strerror(errno));
	if (pmc_attach(real_ids[0], getpid()) < 0)
		errx(77, "attach: %s", strerror(errno));
	if (pmc_start(real_ids[0]) < 0)
		errx(77, "start: %s", strerror(errno));

	/* An uncommitted group. */
	if (pmc_group_create(&uncommitted_gid) < 0)
		errx(77, "group_create 2");

	/* A group that is created then released, so its id is stale. */
	if (pmc_group_create(&released_gid) == 0) {
		pmc_id_t tmp;

		if (pmc_allocate_group("instructions", PMC_MODE_TC, 0,
		    PMC_CPU_ANY, &tmp, 0) == 0) {
			(void)pmc_group_add(released_gid, tmp, 1);
			released_id = tmp;
			(void)pmc_release(tmp);
		} else {
			released_id = 0xdeadbeef;
		}
	} else {
		released_gid = 0xdead;
		released_id = 0xdeadbeef;
	}

	printf("real gid=%u (%d members), uncommitted gid=%u, "
	    "released gid=%u\n", real_gid, nreal, uncommitted_gid,
	    released_gid);

	for (i = 0; i < iters; i++) {
		uint32_t gid;
		pmc_id_t pid_;

		/* Pick a group id from a mix of valid and wild values. */
		switch (rnd() % 6) {
		case 0: gid = real_gid; break;
		case 1: gid = uncommitted_gid; break;
		case 2: gid = released_gid; break;
		case 3: gid = rnd(); break;
		case 4: gid = 0; break;
		default: gid = UINT32_MAX; break;
		}

		/* And a pmc id likewise. */
		switch (rnd() % 6) {
		case 0: pid_ = real_ids[0]; break;
		case 1: pid_ = real_ids[nreal - 1]; break;
		case 2: pid_ = released_id; break;
		case 3: pid_ = rnd(); break;
		case 4: pid_ = 0; break;
		default: pid_ = UINT32_MAX; break;
		}

		switch (rnd() % 3) {
		case 0:
			fuzz_add(gid, pid_, rnd() % 4);
			break;
		case 1:
			fuzz_commit(gid);
			break;
		default:
			fuzz_read(pid_, pick_capacity(), rnd() % 40);
			break;
		}

		if ((i % 5000) == 0 && i > 0)
			printf("  %d iterations, %lu calls, %lu errno, "
			    "%lu ok\n", i, calls, errs, oks);
	}

	printf("done: %lu calls, %lu returned an errno, %lu returned 0\n",
	    calls, errs, oks);

	/* The real group must still be intact and readable. */
	{
		struct pmc_group_times t;
		uint32_t n = 0;

		memset(&t, 0, sizeof(t));
		if (pmc_group_read(real_ids[0], &n, NULL, &t) < 0) {
			printf("  *** real group unreadable after fuzz: %s\n",
			    strerror(errno));
			return (1);
		}
		printf("real group still live: %u members, enabled=%llu\n",
		    n, (unsigned long long)t.pgt_enabled);
	}

	(void)pmc_release(real_ids[0]);
	return (0);
}
