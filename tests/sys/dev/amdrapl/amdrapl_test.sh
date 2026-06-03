#!/usr/libexec/atf-sh
#
# SPDX-License-Identifier: BSD-2-Clause
#
# Copyright (c) 2026 Advanced Micro Devices, Inc.
#
# Functional tests for the amd_rapl(4) cumulative energy counters.
#
# These exercise the dev.amd_rapl.0.*_energy_uj sysctls exported by the
# driver. Every test skips cleanly when the driver (or AMD RAPL hardware)
# is absent, so the suite is safe to run in generic CI.

PKG_OID="dev.amd_rapl.0.package_energy_uj"
CORE_OID="dev.amd_rapl.0.cores_energy_uj"
UNIT_OID="dev.amd_rapl.0.energy_unit"
MAX_OID="dev.amd_rapl.0.max_energy_uj"

# Loadable-module filename (the driver is amd_rapl(4); its .ko is amdrapl.ko).
MODNAME="amdrapl"

# Skip the calling test unless the amd_rapl package energy sysctl is present.
require_amdrapl()
{
	if ! sysctl -n "${PKG_OID}" >/dev/null 2>&1; then
		atf_skip "amd_rapl(4) energy sysctls not present" \
		    "(no AMD RAPL hardware or driver not loaded)"
	fi
}

# Skip the calling test unless the per-core energy sysctl is present. The
# package and per-core domains are probed independently, so a host may export
# one without the other.
require_cores()
{
	if ! sysctl -n "${CORE_OID}" >/dev/null 2>&1; then
		atf_skip "amd_rapl(4) per-core energy sysctl not present" \
		    "(no AMD RAPL hardware, driver not loaded, or core domain absent)"
	fi
}

pkg_uj()
{
	sysctl -n "${PKG_OID}"
}

# First (core 0) field of the comma-separated per-core list.
core0_uj()
{
	sysctl -n "${CORE_OID}" | cut -d, -f1
}

# Integer sum of all comma-separated fields of a sysctl's value.
sum_csv()
{
	sysctl -n "$1" | awk -F, '{s = 0; for (i = 1; i <= NF; i++) s += $i; \
	    printf "%d\n", s}'
}

# Number of comma-separated fields in a sysctl's value.
field_count()
{
	sysctl -n "$1" | awk -F, '{print NF}'
}

# Fail the calling test unless every comma-separated field of the named
# sysctl is exactly 0 or 1.
require_boolean_list()
{
	bad=$(sysctl -n "$1" | awk -F, '{for (i = 1; i <= NF; i++) \
	    if ($i != 0 && $i != 1) { print $i; exit }}')
	if [ -n "${bad}" ]; then
		atf_fail "$1 has a non-boolean field: ${bad}"
	fi
}

atf_test_case package_energy_monotonic
package_energy_monotonic_head()
{
	atf_set "descr" "package_energy_uj never decreases between reads"
}
package_energy_monotonic_body()
{
	require_amdrapl
	v1=$(pkg_uj)
	sleep 2
	v2=$(pkg_uj)
	if [ "${v2}" -lt "${v1}" ]; then
		atf_fail "package_energy_uj decreased: ${v1} -> ${v2}"
	fi
}

atf_test_case package_energy_advances
package_energy_advances_head()
{
	atf_set "descr" "package_energy_uj strictly increases on live hardware"
}
package_energy_advances_body()
{
	require_amdrapl
	v1=$(pkg_uj)
	sleep 2
	v2=$(pkg_uj)
	if [ "${v2}" -le "${v1}" ]; then
		atf_fail "package consumed no energy over 2s: ${v1} -> ${v2}"
	fi
}

atf_test_case cores_energy_monotonic
cores_energy_monotonic_head()
{
	atf_set "descr" "per-core energy_uj (core 0) never decreases"
}
cores_energy_monotonic_body()
{
	require_cores
	v1=$(core0_uj)
	sleep 2
	v2=$(core0_uj)
	if [ "${v2}" -lt "${v1}" ]; then
		atf_fail "cores_energy_uj[0] decreased: ${v1} -> ${v2}"
	fi
}

atf_test_case cores_field_count_matches_cores
cores_field_count_matches_cores_head()
{
	atf_set "descr" "per-core energy list has one field per physical core"
}
cores_field_count_matches_cores_body()
{
	require_cores
	ncores=$(sysctl -n kern.smp.cores 2>/dev/null)
	if [ -z "${ncores}" ]; then
		atf_skip "kern.smp.cores unavailable; cannot check per-core count"
	fi
	# One field per physical core. The F1 regression emits one field per
	# logical CPU, ~2x kern.smp.cores on an SMT-enabled part. kern.smp.cores
	# (mp_ncores) can itself legitimately undercount the driver's per-core dedup
	# when a core's primary SMT thread is individually disabled but a sibling
	# survives, so flag only a near-2x blowup rather than requiring strict
	# equality.
	fields=$(sysctl -n "${CORE_OID}" | awk -F, '{print NF}')
	if [ "${fields}" -lt 1 ] || [ "${fields}" -gt $((ncores + ncores / 2)) ]; then
		atf_fail "per-core field count ${fields} vs physical cores" \
		    "${ncores} (SMT double-count would make this ~2x)"
	fi
}

atf_test_case cores_sum_within_package
cores_sum_within_package_head()
{
	atf_set "descr" "summed per-core energy delta does not exceed the package delta"
}
cores_sum_within_package_body()
{
	require_amdrapl
	require_cores
	# The core domain is a subset of the package domain, so the summed
	# per-core energy gained over an interval must not exceed the package
	# energy gained over the same interval. A per-SMT-sibling double-count of
	# the core domain can push the core sum past the package total. This bound
	# is host-independent: it needs no knowledge of the core or socket count.
	pc1=$(sum_csv "${CORE_OID}")
	pp1=$(sum_csv "${PKG_OID}")
	sleep 3
	pc2=$(sum_csv "${CORE_OID}")
	pp2=$(sum_csv "${PKG_OID}")
	dcore=$((pc2 - pc1))
	dpkg=$((pp2 - pp1))
	if [ "${dpkg}" -le 0 ]; then
		atf_skip "no package energy accumulated; cannot bound cores"
	fi
	if [ "${dcore}" -gt "${dpkg}" ]; then
		atf_fail "core energy delta ${dcore} uj exceeds package" \
		    "${dpkg} uj (an SMT double-count would do this)"
	fi
}

atf_test_case package_power_sane
package_power_sane_head()
{
	atf_set "descr" "energy-derived package power is within a plausible range"
}
package_power_sane_body()
{
	require_amdrapl
	v1=$(pkg_uj)
	sleep 3
	v2=$(pkg_uj)
	duj=$((v2 - v1))
	if [ "${duj}" -le 0 ]; then
		atf_fail "no energy accumulated over 3s (delta=${duj} uj)"
	fi
	# Integer watts = microjoules / 1e6 / seconds.
	watts=$((duj / 1000000 / 3))
	if [ "${watts}" -gt 2000 ]; then
		atf_fail "implausible package power: ${watts}W" \
		    "(delta=${duj} uj over 3s)"
	fi
}

atf_test_case energy_unit_sane
energy_unit_sane_head()
{
	atf_set "descr" "energy_unit is within the architectural field range"
}
energy_unit_sane_body()
{
	require_amdrapl
	unit=$(sysctl -n "${UNIT_OID}")
	# (value >> 8) & 0x1f is a 5-bit field; a zero unit (1 J/count) is bogus.
	if [ "${unit}" -lt 1 ] || [ "${unit}" -gt 31 ]; then
		atf_fail "energy_unit out of range: ${unit}"
	fi
}

atf_test_case max_energy_consistent
max_energy_consistent_head()
{
	atf_set "descr" "max_energy_uj equals UINT32_MAX scaled by the energy unit"
}
max_energy_consistent_body()
{
	require_amdrapl
	unit=$(sysctl -n "${UNIT_OID}")
	maxuj=$(sysctl -n "${MAX_OID}")
	# Mirror the kernel: microjoules of the 0xffffffff maximum counter value.
	expected=$((4294967295 * 1000000 / (1 << unit)))
	if [ "${maxuj}" -ne "${expected}" ]; then
		atf_fail "max_energy_uj=${maxuj} but expected ${expected}" \
		    "for energy_unit=${unit}"
	fi
}

atf_test_case package_domain_parity
package_domain_parity_head()
{
	atf_set "descr" "package energy_uj, mwatt, and lapsed lists agree on field count"
}
package_domain_parity_body()
{
	require_amdrapl
	# All three package lists are printed from the same per-domain array, so
	# they must expose the same number of comma-separated fields. A drift
	# here means one printer walked a different length than the others.
	nuj=$(field_count "${PKG_OID}")
	nmw=$(field_count "dev.amd_rapl.0.package_mwatt")
	nlp=$(field_count "dev.amd_rapl.0.package_energy_lapsed")
	if [ "${nuj}" -ne "${nmw}" ] || [ "${nuj}" -ne "${nlp}" ]; then
		atf_fail "package field counts disagree:" \
		    "energy_uj=${nuj} mwatt=${nmw} lapsed=${nlp}"
	fi
}

atf_test_case cores_domain_parity
cores_domain_parity_head()
{
	atf_set "descr" "per-core energy_uj, mwatt, and lapsed lists agree on field count"
}
cores_domain_parity_body()
{
	require_cores
	# As with the package domain, the three per-core lists are emitted from a
	# single backing array and must therefore share a field count.
	nuj=$(field_count "${CORE_OID}")
	nmw=$(field_count "dev.amd_rapl.0.cores_mwatt")
	nlp=$(field_count "dev.amd_rapl.0.cores_energy_lapsed")
	if [ "${nuj}" -ne "${nmw}" ] || [ "${nuj}" -ne "${nlp}" ]; then
		atf_fail "per-core field counts disagree:" \
		    "energy_uj=${nuj} mwatt=${nmw} lapsed=${nlp}"
	fi
}

atf_test_case lapsed_is_boolean
lapsed_is_boolean_head()
{
	atf_set "descr" "every *_energy_lapsed field is the 0/1 sticky flag"
}
lapsed_is_boolean_body()
{
	# The lapsed lists are per-domain sticky flags, so each field must be a
	# bare 0 or 1. Probe both domains independently; skip cleanly if neither
	# is present.
	checked=0
	if sysctl -n dev.amd_rapl.0.package_energy_lapsed >/dev/null 2>&1; then
		require_boolean_list dev.amd_rapl.0.package_energy_lapsed
		checked=1
	fi
	if sysctl -n dev.amd_rapl.0.cores_energy_lapsed >/dev/null 2>&1; then
		require_boolean_list dev.amd_rapl.0.cores_energy_lapsed
		checked=1
	fi
	if [ "${checked}" -eq 0 ]; then
		atf_skip "no amd_rapl(4) lapsed sysctls present"
	fi
}

atf_test_case load_unload_cycle cleanup
load_unload_cycle_head()
{
	atf_set "descr" "kldunload/kldload tears down and rebuilds the sysctl tree without panic"
	atf_set "require.user" "root"
}
load_unload_cycle_body()
{
	# energy_unit is registered unconditionally on attach, so it is the
	# canonical "driver is attached" probe (package/core OIDs are domain-gated).
	if ! sysctl -n "${UNIT_OID}" >/dev/null 2>&1; then
		atf_skip "amd_rapl(4) not attached; nothing to cycle"
	fi
	# Only a loadable module can be cycled. A driver compiled into the kernel
	# has no .ko to kldunload, so skip rather than fail on such hosts.
	if ! kldstat -n "${MODNAME}" >/dev/null 2>&1; then
		atf_skip "amd_rapl(4) is built into the kernel; cannot kldunload"
	fi
	# Teardown is the riskiest path (callout drain, cpuset/sysctl free). A
	# panic here takes the test host down, which is itself the signal.
	if ! kldunload "${MODNAME}"; then
		atf_fail "kldunload ${MODNAME} failed"
	fi
	if sysctl -n "${UNIT_OID}" >/dev/null 2>&1; then
		atf_fail "${UNIT_OID} still present after kldunload"
	fi
	if ! kldload "${MODNAME}"; then
		atf_fail "kldload ${MODNAME} failed to reattach"
	fi
	if ! sysctl -n "${UNIT_OID}" >/dev/null 2>&1; then
		atf_fail "${UNIT_OID} did not reappear after kldload"
	fi
}
load_unload_cycle_cleanup()
{
	# Restore the loaded state for later tests regardless of where the body
	# bailed out. Reloading an already-loaded module is a harmless no-op.
	kldload "${MODNAME}" >/dev/null 2>&1 || true
}

atf_init_test_cases()
{
	atf_add_test_case package_energy_monotonic
	atf_add_test_case package_energy_advances
	atf_add_test_case cores_energy_monotonic
	atf_add_test_case cores_field_count_matches_cores
	atf_add_test_case cores_sum_within_package
	atf_add_test_case package_power_sane
	atf_add_test_case energy_unit_sane
	atf_add_test_case max_energy_consistent
	atf_add_test_case package_domain_parity
	atf_add_test_case cores_domain_parity
	atf_add_test_case lapsed_is_boolean
	atf_add_test_case load_unload_cycle
}
