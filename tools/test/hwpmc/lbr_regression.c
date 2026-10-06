/*-
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 Advanced Micro Devices, Inc.
 */

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
typedef unsigned int u_int;
typedef long register_t;
typedef uint64_t pmc_value_t;
#define __diagused
#define KASSERT(c, msg)		 assert(c)
#define CRITICAL_ASSERT(td)	 ((void)0)
#define atomic_load_int(p)	 (*(p))
#define atomic_store_int(p, v)	 (*(p) = (v))
#define atomic_interrupt_fence() ((void)0)
#define PMCDBG1(...)		 ((void)0)
#define PMCDBG3(...)		 ((void)0)
#define PMC_CC_MULTIPART_LBR	 4
#define PMC_HR			 0
#define PMC_STATE_RUNNING	 1
#define PMC_CAP_LBR		 (1U << 14)
#define PMC_CAP_USER		 (1U << 1)
#define PMC_CAP_SYSTEM		 (1U << 2)
#define PMC_MODE_SS		 1
#define PMC_MODE_TS		 2
#define PMC_TO_MODE(pm)		 ((pm)->mode)
#define PMC_IS_SYSTEM_MODE(m)	 ((m) == PMC_MODE_SS)
#define PMC_IS_SAMPLING_MODE(m)	 (1)
#define TRAPF_USERMODE(tf)	 1
#define min(a, b)		 ((a) < (b) ? (a) : (b))
#define DPCPU_SET(n, v)		 ((n) = (v))
#define DPCPU_GET(n)		 (n)
#define counter_u64_add(p, v)	 (*(p) += (v))
#include "sys/dev/hwpmc/hwpmc_amd.h"
#include "sys/dev/hwpmc/hwpmc_amd_lbr.h"
struct trapframe {
	int unused;
};
struct pmc_cpu {
	int unused;
};
struct pmc {
	uint32_t pm_caps;
	int mode;
	int pm_state;
	struct {
		struct {
			uint64_t pm_amd_evsel, pm_amd_lbr_select;
		} pm_amd;
	} pm_md;
	struct {
		pmc_value_t pm_reloadcount;
	} pm_sc;
};
struct pmc_hw {
	struct pmc *phw_pmc;
};
struct pmc_multipart {
	char pl_type, pl_length;
	uint64_t pl_mpdata[32];
};
struct amd_cpu {
	struct pmc_hw pc_amdpmcs[16];
	uint32_t pc_lbr_mask;
	uint64_t pc_lbr_select, pc_lbr_hw_select, pc_lbr_extn_cfg;
	bool pc_lbr_suspended, pc_lbr_hw_on, pc_lbr_dirty;
	volatile u_int pc_lbr_busy;
};
static int curcpu;
static struct amd_cpu cpu_storage;
static struct amd_cpu *cpu_ptrs[1] = { &cpu_storage }, **amd_pcpu = cpu_ptrs;
static struct pmc_cpu pc_storage, *pmc_pcpu[1] = { &pc_storage };
static int amd_lbr_depth = 1, amd_core_npmcs = 1;
static bool amd_lbr_freeze = true;
static u_int amd_lbr_va_width = 48;
static uint64_t amd_global_cntr_mask = 1;
static uint64_t empty_total, processed, ignored, *amd_lbr_empty = &empty_total;
static struct {
	uint64_t *pm_intr_processed, *pm_intr_ignored;
} pmc_stats = { &processed, &ignored };
static struct {
	uint32_t pm_evsel, pm_perfctr;
} amd_pmcdesc[16] = { { AMD_PMC_CORE_BASE, AMD_PMC_CORE_BASE + 1 } };
static uint32_t nmi_counter;
static uint64_t global_status, extn_cfg, ring_from, ring_to;
static bool mutate_ring;
static uint64_t sampled_from, sampled_to;
static int sample_words, branch_reads, samples;
static int critical_nesting;
static register_t
intr_disable(void)
{
	return 0;
}
static void
intr_restore(register_t f)
{
	(void)f;
}
static int
pmc_cpu_max(void)
{
	return 1;
}
static int
pmc_ibs_intr(struct trapframe *tf)
{
	(void)tf;
	return 0;
}
static void
amd_v2_freeze_core(int cpu)
{
	(void)cpu;
}
static void
amd_v2_thaw_core(int cpu)
{
	(void)cpu;
}
static uint64_t
rdmsr(uint32_t msr)
{
	if (msr == AMD_MSR_SAMP_BR_FROM || msr == AMD_MSR_SAMP_BR_FROM + 1)
		branch_reads++;
	if (msr == AMD_PMC_GLOBAL_STATUS)
		return global_status;
	if (msr == AMD_MSR_SAMP_BR_FROM + 1)
		return ring_to;
	if (msr == AMD_MSR_SAMP_BR_FROM) {
		/* Recording advances between separate To and From reads unless
		 * frozen. */
		if (mutate_ring && (extn_cfg & AMD_DBG_EXTN_CFG_LBRV2EN) &&
		    !(global_status & AMD_PMC_GLOBAL_STATUS_LBRS_FROZEN)) {
			ring_from = 0x2222;
			ring_to = AMD_LBR_TO_VALID | 0x2223;
		}
		return ring_from;
	}
	return 0;
}
static void
wrmsr(uint32_t msr, uint64_t v)
{
	if (msr == AMD_MSR_DBG_EXTN_CFG)
		extn_cfg = v;
	if (msr == AMD_PMC_GLOBAL_STATUS_CLR)
		global_status &= ~v;
	if (msr == AMD_MSR_SAMP_BR_FROM)
		ring_from = v;
	if (msr == AMD_MSR_SAMP_BR_FROM + 1)
		ring_to = v;
}
static int
pmc_process_interrupt_mp(int r, struct pmc *pm, struct trapframe *tf,
    struct pmc_multipart *mp)
{
	(void)r;
	(void)pm;
	(void)tf;
	sample_words = mp->pl_length;
	samples++;
	if (sample_words) {
		sampled_from = mp->pl_mpdata[0];
		sampled_to = mp->pl_mpdata[1];
	}
	return 0;
}
static int
pmc_process_interrupt(int r, struct pmc *pm, struct trapframe *tf)
{
	(void)r;
	(void)pm;
	(void)tf;
	return 0;
}

/* These simplified structs and callbacks surround the actual hook fragments. */
#define P_HWPMC			     1
#define PMC_FN_CSW_IN		     1
#define PMC_FN_CSW_OUT		     2
#define PMC_FN_THR_EXIT		     3
#define PMC_FN_THR_EXIT_LOG	     4
#define PMC_PROC_IS_USING_PMCS(p)    ((p)->p_flag & P_HWPMC)
#define PMC_SYSTEM_CSW_ACTIVE()	     (pmc_ss_csw_count > 0)
#define PMC_SYSTEM_SAMPLING_ACTIVE() (pmc_ss_count > 0)
struct proc {
	int p_flag;
};
struct thread {
	struct proc *td_proc;
};
static int pmc_ss_csw_count, pmc_ss_count, switch_in_calls, switch_out_calls;
static void
critical_enter(void)
{
	critical_nesting++;
}
static void
critical_exit(void)
{
	assert(critical_nesting > 0);
	critical_nesting--;
}
static void amd_lbr_csw(struct pmc_cpu *, bool);
static void
hook(struct thread *td, int cmd, void *arg)
{
	(void)td;
	(void)arg;
	if (cmd == PMC_FN_CSW_IN) {
		switch_in_calls++;
		amd_lbr_csw(&pc_storage, true);
	}
	if (cmd == PMC_FN_CSW_OUT) {
		switch_out_calls++;
		amd_lbr_csw(&pc_storage, false);
	}
}
#define PMC_CALL_HOOK_UNLOCKED(td, cmd, arg) hook(td, cmd, arg)
#define PMC_SWITCH_CONTEXT(td, cmd)	     hook(td, cmd, NULL)
#include "lbr.inc"
static void
first_run(struct thread *td)
{
	struct proc *p = td->td_proc;
	(void)p;
#include "fork.inc"
}
static void
exiting_thread(struct thread *td)
{
#include "thread.inc"
}

static void
check_snapshot(void)
{
	struct pmc pm = { .pm_caps = PMC_CAP_LBR,
		.mode = PMC_MODE_SS,
		.pm_state = PMC_STATE_RUNNING,
		.pm_sc.pm_reloadcount = 100 };
	struct trapframe tf = { 0 };
	unsigned int reset;

	cpu_storage.pc_amdpmcs[0].phw_pmc = &pm;
	amd_lbr_activate(0, 0, &pm);
	for (reset = 0; reset < 2; reset++) {
		global_status = 1 | AMD_PMC_GLOBAL_STATUS_LBRS_FROZEN;
		if (reset == 0)
			amd_lbr_exec(&pc_storage);
		else {
			amd_lbr_csw(&pc_storage, false);
			amd_lbr_csw(&pc_storage, true);
		}
		assert(global_status == 1 && cpu_storage.pc_lbr_hw_on);
		ring_from = 0x1111;
		ring_to = AMD_LBR_TO_VALID | 0x1112;
		mutate_ring = true;
		branch_reads = 0;
		sample_words = -1;
		assert(amd_intr_v2(&tf) == 1);
		assert(sample_words == 0 && branch_reads == 0);
		assert(global_status == 0 && empty_total == reset + 1);
	}
	/* Both hardware-frozen and software-stopped rings remain coherent. */
	global_status = 1 | AMD_PMC_GLOBAL_STATUS_LBRS_FROZEN;
	ring_from = 0x3333;
	ring_to = AMD_LBR_TO_VALID | 0x3334;
	assert(amd_intr_v2(&tf) == 1);
	assert(sample_words == 2 && AMD_LBR_IP(sampled_from) == 0x3333 &&
	    AMD_LBR_IP(sampled_to) == 0x3334);
	amd_lbr_freeze = false;
	global_status = 1;
	ring_from = 0x4444;
	ring_to = AMD_LBR_TO_VALID | 0x4445;
	assert(amd_intr_v2(&tf) == 1);
	assert(sample_words == 2 && AMD_LBR_IP(sampled_from) == 0x4444 &&
	    AMD_LBR_IP(sampled_to) == 0x4445);
	assert(samples == 4 && empty_total == 2);
	amd_lbr_deactivate(0, 0, &pm);
	puts("LBR snapshots: reset/late-NMI empty, frozen/software-stop "
	     "coherent");
}

static void
check_lifecycle(void)
{
	struct pmc pm = { .pm_caps = PMC_CAP_LBR, .mode = PMC_MODE_SS };
	struct proc p = { 0 };
	struct thread td = { &p };

	amd_lbr_activate(0, 0, &pm);
	pmc_ss_csw_count = pmc_ss_count = 1;
	/* A normal switch-out followed by first execution must resume LBR. */
	amd_lbr_csw(&pc_storage, false);
	first_run(&td);
	assert(switch_in_calls == 1 && !cpu_storage.pc_lbr_suspended &&
	    cpu_storage.pc_lbr_hw_on);
	assert(ring_from == 0 && ring_to == 0);
	/* An unmonitored thread exits directly to another first-run thread. */
	ring_from = 0x5555;
	ring_to = AMD_LBR_TO_VALID | 0x5556;
	exiting_thread(&td);
	assert(switch_out_calls == 1 && !cpu_storage.pc_lbr_hw_on);
	first_run(&td);
	assert(switch_in_calls == 2 && cpu_storage.pc_lbr_hw_on);
	assert(ring_from == 0 && ring_to == 0 && critical_nesting == 0);
	/* Plain SS sampling does not request these extra context hooks. */
	pmc_ss_csw_count = 0;
	first_run(&td);
	exiting_thread(&td);
	assert(switch_in_calls == 2 && switch_out_calls == 1);
	/* The existing P_HWPMC exit path must invoke CSW_OUT only once. */
	p.p_flag = P_HWPMC;
	pmc_ss_csw_count = 1;
	exiting_thread(&td);
	assert(switch_out_calls == 2);
	amd_lbr_deactivate(0, 0, &pm);
	puts("LBR lifecycle: first-run resumes, exit-to-first-run wipes old "
	     "ring");
}

int
main(int argc, char **argv)
{
	if (argc == 1 || strcmp(argv[1], "snapshot") == 0)
		check_snapshot();
	if (argc == 1 || strcmp(argv[1], "lifecycle") == 0)
		check_lifecycle();
	return (0);
}
