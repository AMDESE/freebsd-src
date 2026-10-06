/*-
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 Advanced Micro Devices, Inc.
 */

#include <assert.h>
#include <stdint.h>
#include <stdio.h>
typedef unsigned int u_int;
typedef uint64_t pmc_value_t;
#define KASSERT(c, msg) assert(c)
#define PMCDBG2(...) ((void)0)
#define PMCDBG4(...) ((void)0)
#define PMC_MODE_TC 1
#define PMC_MODE_TS 2
#define PMC_TO_MODE(pm) ((pm)->mode)
#define PMC_IS_VIRTUAL_MODE(mode) 1
#define PMC_TO_ROWINDEX(pm) ((pm)->row)
#define PMC_PCPU_SAVED(cpu, ri) 0
#define mtx_pool_lock_spin(a, b) ((void)0)
#define mtx_pool_unlock_spin(a, b) ((void)0)
#define counter_u64_fetch(p) (*(p))
#define counter_u64_add(p, v) (*(p) += (v))
struct pmc {
	int mode, row;
	uint64_t *pm_runcount;
	struct {
		int pps_cpustate, pps_stalled;
	} pm_pcpu_state[1];
	struct {
		uint64_t pm_savedvalue;
	} pm_gv;
};
struct pmc_process {
	struct {
		struct pmc *pp_pmc;
		uint64_t pp_pmcval;
	} pp_pmcs[1];
};
struct pmc_classdep {
	int pcd_num;
	int (*pcd_get_config)(int, int, struct pmc **);
	int (*pcd_stop_pmc)(int, int, struct pmc *);
	int (*pcd_read_pmc)(int, int, struct pmc *, pmc_value_t *);
	int (*pcd_config_pmc)(int, int, struct pmc *);
	int (*pcd_stop_all)(int);
};
static uint64_t runcount = 1;
static struct pmc pm_storage = {.mode = PMC_MODE_TC,
				.row = 0,
				.pm_runcount = &runcount,
				.pm_pcpu_state[0].pps_cpustate = 1};
static struct pmc *loaded = &pm_storage;
static int stop_calls, read_calls;
static int getcfg(int c, int r, struct pmc **p)
{
	*p = loaded;
	return 0;
}
static int stop(int c, int r, struct pmc *p)
{
	stop_calls++;
	return 0;
}
static int readpm(int c, int r, struct pmc *p, uint64_t *v)
{
	read_calls++;
	*v = 1234;
	return 0;
}
static int config(int c, int r, struct pmc *p)
{
	loaded = p;
	return 0;
}
static int stopall(int c) { return 0; }
static struct md {
	int pmd_nclass, pmd_npmc;
	struct pmc_classdep pmd_classdep[1];
} md_storage = {1, 1, {{1, getcfg, stop, readpm, config, stopall}}},
  *md = &md_storage;
static struct pmc_classdep *pmc_ri_to_classdep(struct md *m, int r, int *adj)
{
	*adj = 0;
	return &m->pmd_classdep[0];
}
static uint64_t pmc_delta(struct pmc_classdep *p, uint64_t n, uint64_t o)
{
	return n - o;
}

#include "exit.inc"

static void
check_exit(int batch, int mode, int stalled)
{
	int cpu = 0, ri, adjri;
	struct pmc *pm;
	struct pmc_classdep *pcd;
	pmc_value_t newvalue, tmp, expected;
	struct pmc_process pp_storage = {.pp_pmcs[0].pp_pmc = &pm_storage},
			   *pp = &pp_storage;

	runcount = 1;
	loaded = &pm_storage;
	stop_calls = read_calls = 0;
	pm_storage.mode = mode;
	pm_storage.pm_pcpu_state[0].pps_cpustate = 1;
	pm_storage.pm_pcpu_state[0].pps_stalled = stalled;
	pm_storage.pm_gv.pm_savedvalue = 0;
	md->pmd_classdep[0].pcd_stop_all = batch ? stopall : NULL;
	pmc_process_csw_stop_all(cpu);
#include "exit_loop.inc"
	expected = !stalled && mode == PMC_MODE_TC ? 1234 : 0;
	assert(stop_calls == !stalled);
	assert(read_calls == (!stalled && mode == PMC_MODE_TC));
	assert(pm_storage.pm_gv.pm_savedvalue == expected &&
	    pp->pp_pmcs[0].pp_pmcval == expected);
	assert(runcount == 0 && loaded == NULL);
}

int
main(void)
{
	int batch, stalled, mode;

	for (batch = 0; batch < 2; batch++)
		for (stalled = 0; stalled < 2; stalled++)
			for (mode = PMC_MODE_TC; mode <= PMC_MODE_TS; mode++)
				check_exit(batch, mode, stalled);
	puts("PMC exit: 8 batch/non-batch, TC/TS, stalled/live cases passed");
	return (0);
}
