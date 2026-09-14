/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Advanced Micro Devices, Inc.
 */

#ifndef _DEV_HWPMC_AMD_LBR_H_
#define _DEV_HWPMC_AMD_LBR_H_

#include <sys/types.h>
#include <sys/errno.h>

/* Allocation reservations, serialized by pmc_sx; independent of running state. */
struct amd_lbr_filter {
	uint64_t select;
	u_int refs;
};

static __inline int
amd_lbr_filter_reserve(struct amd_lbr_filter *ts, struct amd_lbr_filter *ss,
    u_int ncpu, int is_thread, u_int cpu, uint64_t select)
{
	u_int i;

	if (ts->refs != 0 && ts->select != select)
		return (EBUSY);
	if (is_thread) {
		for (i = 0; i < ncpu; i++) {
			if (ss[i].refs != 0 && ss[i].select != select)
				return (EBUSY);
		}
		if (ts->refs++ == 0)
			ts->select = select;
	} else {
		if (ss[cpu].refs != 0 && ss[cpu].select != select)
			return (EBUSY);
		if (ss[cpu].refs++ == 0)
			ss[cpu].select = select;
	}
	return (0);
}

/* The caller must hold a reservation in this slot. */
static __inline void
amd_lbr_filter_release(struct amd_lbr_filter *slot)
{
	if (--slot->refs == 0)
		slot->select = 0;
}

#endif /* _DEV_HWPMC_AMD_LBR_H_ */
