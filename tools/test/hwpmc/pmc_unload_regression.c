/*-
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 Advanced Micro Devices, Inc.
 */

#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdio.h>

/* Compile the production unload guard and module dispatch with fake locks. */
#define MOD_LOAD 1
#define MOD_UNLOAD 2
#define MOD_SHUTDOWN 3
#define PMCDBG0(...) ((void)0)
#define PMCDBG2(...) ((void)0)
struct module { int unused; };
struct sx { bool held; };
struct pmc_owner { struct pmc_owner *next; };
struct pmc_ownerhash { struct pmc_owner *first; };
static struct sx pmc_sx;
static struct pmc_ownerhash owners[2], *pmc_ownerhash = owners;
static unsigned long pmc_ownerhashmask = 1;
#define SX_XLOCKED 1
#define CPU_FOREACH(cpu) for ((cpu) = 0; (cpu) < 1; (cpu)++)
#define DPCPU_ID_SET(cpu, name, value) ((void)0)
static void *pmc_hook;
static void *pmc_intr;
static unsigned cleanups, tries, blocking_locks;
#define LIST_EMPTY(head) ((head)->first == NULL)

static int
sx_try_xlock(struct sx *lock)
{
	tries++;
	if (lock->held)
		return (0);
	lock->held = true;
	return (1);
}

static void
sx_xlock(struct sx *lock)
{
	blocking_locks++;
	assert(!lock->held);
	lock->held = true;
}

static void
sx_xunlock(struct sx *lock)
{
	assert(lock->held);
	lock->held = false;
}

static void
sx_assert(struct sx *lock, int what)
{
	(void)what;
	assert(lock->held);
}

static void
pmc_cleanup(void)
{
	int cpu;

#include "cleanup.inc"
	cleanups++;
	sx_xunlock(&pmc_sx);
}

static int pmc_initialize(void) { return (0); }
#include "unload.inc"
#include "load.inc"

static void
reset(void)
{
	owners[0].first = owners[1].first = NULL;
	pmc_ownerhash = owners;
	pmc_sx.held = false;
	pmc_hook = pmc_intr = &owners;
	cleanups = tries = blocking_locks = 0;
}

int
main(void)
{
	struct pmc_owner owner = { NULL };

	/* A running, stopped or logfile-only owner must veto ordinary unload. */
	for (unsigned bucket = 0; bucket < 2; bucket++) {
		reset();
		owners[bucket].first = &owner;
		assert(load(NULL, MOD_UNLOAD, NULL) == EBUSY);
		assert(pmc_hook && pmc_intr && cleanups == 0 && !pmc_sx.held);
		assert(tries == 1 && blocking_locks == 0);
	}
	/* An allocation holding pmc_sx cannot race an owner-free check. */
	reset();
	pmc_sx.held = true;
	assert(load(NULL, MOD_UNLOAD, NULL) == EBUSY);
	assert(pmc_hook && pmc_intr && cleanups == 0 && pmc_sx.held);
	assert(tries == 1 && blocking_locks == 0);
	/* Idle unload succeeds; shutdown is not subject to the owner veto. */
	reset();
	assert(load(NULL, MOD_UNLOAD, NULL) == 0);
	assert(!pmc_hook && !pmc_intr && cleanups == 1 && !pmc_sx.held);
	assert(tries == 1 && blocking_locks == 0);
	reset();
	owners[0].first = &owner;
	assert(load(NULL, MOD_SHUTDOWN, NULL) == 0);
	assert(cleanups == 1 && blocking_locks == 1);
	puts("PMC unload: owners/contention vetoed, idle unload and shutdown pass");
	return (0);
}
