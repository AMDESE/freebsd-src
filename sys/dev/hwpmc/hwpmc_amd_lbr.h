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

/*
 * Branch record IP handling.  The hardware IP field is bits [57:0], but only
 * the bits covered by the implemented linear-address width (CPUID
 * 0x80000008 EAX[15:8]) are architecturally meaningful.  Do not assume the
 * bits above that width are a copy of the sign bit: sign-extend from the
 * implemented width, as the Linux driver does.
 */
#define	AMD_LBR_IP_BITS		58
#define	AMD_LBR_IP_MASK		((1ULL << AMD_LBR_IP_BITS) - 1)
#define	AMD_LBR_VA_BITS_MIN	48
#define	AMD_LBR_VA_BITS_DEFAULT	48

/* Clamp a CPUID linear-address width to the range the IP field can hold. */
static __inline u_int
amd_lbr_va_bits(u_int cpuid_bits)
{
	if (cpuid_bits < AMD_LBR_VA_BITS_MIN)
		return (AMD_LBR_VA_BITS_DEFAULT);
	if (cpuid_bits > AMD_LBR_IP_BITS)
		return (AMD_LBR_IP_BITS);
	return (cpuid_bits);
}

/* True if a raw From/To word holds a supervisor-half (kernel) address. */
static __inline int
amd_lbr_ip_is_kernel(uint64_t raw, u_int va_bits)
{
	return (((raw & AMD_LBR_IP_MASK) >> (va_bits - 1)) != 0);
}

/*
 * Rewrite the IP field of a raw From/To word so that sign-extending it from
 * bit 57 yields the canonical address.  The flag bits [63:58] are preserved,
 * so consumers can decode the record without knowing the recording CPU's
 * linear-address width.
 */
static __inline uint64_t
amd_lbr_canonicalize(uint64_t raw, u_int va_bits)
{
	uint64_t ip;
	u_int shift;

	shift = 64 - va_bits;
	ip = (uint64_t)((int64_t)(raw << shift) >> shift);
	return ((raw & ~AMD_LBR_IP_MASK) | (ip & AMD_LBR_IP_MASK));
}

#endif /* _DEV_HWPMC_AMD_LBR_H_ */
