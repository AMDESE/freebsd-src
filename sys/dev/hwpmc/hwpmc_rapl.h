/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Advanced Micro Devices, Inc.
 *
 * AMD/Intel RAPL energy counters exposed as an hwpmc(4) PMC class.
 */

#ifndef _DEV_HWPMC_RAPL_H_
#define	_DEV_HWPMC_RAPL_H_ 1

#ifdef	_KERNEL

struct pmc_mdep;

/*
 * Maximum number of RAPL counter rows per CPU. One row per supported energy
 * event: package, "cores" and DRAM. AMD exposes package + cores (2); Intel
 * adds DRAM (3). The actual count is determined per-vendor at initialize time
 * and stored in the class-dep's pcd_num.
 */
#define	RAPL_MAX_NPMCS	3

/*
 * Prototypes.
 */
int	pmc_rapl_initialize(struct pmc_mdep *_md, int _maxcpu, int _classindex);
void	pmc_rapl_finalize(struct pmc_mdep *_md);

#endif	/* _KERNEL */
#endif	/* _DEV_HWPMC_RAPL_H_ */
