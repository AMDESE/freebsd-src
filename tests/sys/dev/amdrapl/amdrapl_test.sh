#!/usr/libexec/atf-sh
#
# SPDX-License-Identifier: BSD-2-Clause
#
# Copyright (c) 2026 Advanced Micro Devices, Inc.
#
# Functional tests for the amd_rapl(4) energy counters. Tests skip cleanly when
# the driver or hardware is absent. Reading the counters is root-only, so the
# value-reading tests set require.user root.

PKG_OID="dev.amd_rapl.0.package_energy_uj"
CORE_OID="dev.amd_rapl.0.cores_energy_uj"
UNIT_OID="dev.amd_rapl.0.energy_unit"
MAX_OID="dev.amd_rapl.0.max_energy_uj"

# .ko filename (driver is amd_rapl(4), module is amdrapl.ko).
MODNAME="amdrapl"

# Skip unless the package energy sysctl is present (privileged OID).
require_amdrapl()
{
	if ! sysctl -n "${PKG_OID}" >/dev/null 2>&1; then
		atf_skip "amd_rapl(4) energy sysctls not present" \
		    "(no AMD RAPL hardware or driver not loaded)"
	fi
}

# Skip unless the per-core energy sysctl is present; domains are probed
# independently, so a host may export one and not the other. Privileged OID.
require_cores()
{
	if ! sysctl -n "${CORE_OID}" >/dev/null 2>&1; then
		atf_skip "amd_rapl(4) per-core energy sysctl not present" \
		    "(no AMD RAPL hardware, driver not loaded, or core domain absent)"
	fi
}

# Skip unless attached. energy_unit is registered on attach and world-readable.
require_attached()
{
	if ! sysctl -n "${UNIT_OID}" >/dev/null 2>&1; then
		atf_skip "amd_rapl(4) not attached"
	fi
}

# Raw comma-separated per-package list.
pkg_uj()
{
	sysctl -n "${PKG_OID}"
}

# Core 0 field of the comma-separated per-core list.
core0_uj()
{
	sysctl -n "${CORE_OID}" | cut -d, -f1
}

# Integer sum of a sysctl's comma-separated fields.
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

# Per-field deltas of two same-length CSV lists ($2 - $1), one per line.
csv_deltas()
{
	echo "$1 $2" | awk '{n = split($1, a, ","); split($2, b, ","); \
	    for (i = 1; i <= n; i++) printf "%d\n", b[i] - a[i]}'
}

# Fail unless every comma-separated field of the named sysctl is 0 or 1.
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
	atf_set "require.user" "root"
}
package_energy_monotonic_body()
{
	require_amdrapl
	v1=$(pkg_uj)
	sleep 2
	v2=$(pkg_uj)
	# Check each package field; the lists are CSV on multi-socket hosts.
	for d in $(csv_deltas "${v1}" "${v2}"); do
		if [ "${d}" -lt 0 ]; then
			atf_fail "package_energy_uj decreased: ${v1} -> ${v2}"
		fi
	done
}

atf_test_case package_energy_advances
package_energy_advances_head()
{
	atf_set "descr" "package_energy_uj strictly increases on live hardware"
	atf_set "require.user" "root"
}
package_energy_advances_body()
{
	require_amdrapl
	v1=$(pkg_uj)
	sleep 2
	v2=$(pkg_uj)
	for d in $(csv_deltas "${v1}" "${v2}"); do
		if [ "${d}" -le 0 ]; then
			atf_fail "a package consumed no energy over 2s:" \
			    "${v1} -> ${v2}"
		fi
	done
}

atf_test_case cores_energy_monotonic
cores_energy_monotonic_head()
{
	atf_set "descr" "per-core energy_uj (core 0) never decreases"
	atf_set "require.user" "root"
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
	atf_set "require.user" "root"
}
cores_field_count_matches_cores_body()
{
	require_cores
	ncores=$(sysctl -n kern.smp.cores 2>/dev/null)
	if [ -z "${ncores}" ]; then
		atf_skip "kern.smp.cores unavailable; cannot check per-core count"
	fi
	# One field per physical core; an SMT double-count blows this up ~2x.
	# kern.smp.cores can itself undercount the dedup, so flag only a near-2x
	# blowup, not strict equality.
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
	atf_set "require.user" "root"
}
cores_sum_within_package_body()
{
	require_amdrapl
	require_cores
	# Cores are a subset of the package, so the summed core delta must not
	# exceed the package delta; an SMT double-count would push it past.
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
	atf_set "require.user" "root"
}
package_power_sane_body()
{
	require_amdrapl
	v1=$(pkg_uj)
	sleep 3
	v2=$(pkg_uj)
	for duj in $(csv_deltas "${v1}" "${v2}"); do
		if [ "${duj}" -le 0 ]; then
			atf_fail "no energy accumulated over 3s (delta=${duj} uj)"
		fi
		# Integer watts = microjoules / 1e6 / seconds.
		watts=$((duj / 1000000 / 3))
		if [ "${watts}" -gt 2000 ]; then
			atf_fail "implausible package power: ${watts}W" \
			    "(delta=${duj} uj over 3s)"
		fi
	done
}

atf_test_case energy_unit_sane
energy_unit_sane_head()
{
	atf_set "descr" "energy_unit is within the architectural field range"
}
energy_unit_sane_body()
{
	require_attached
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
	require_attached
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
	atf_set "require.user" "root"
}
package_domain_parity_body()
{
	require_amdrapl
	# All three package lists share one per-domain array, so field counts agree.
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
	atf_set "require.user" "root"
}
cores_domain_parity_body()
{
	require_cores
	# Like the package domain, the three per-core lists share one array.
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
	# Each lapsed field must be a bare 0 or 1. Probe both domains; skip if neither.
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
	# energy_unit is the canonical "attached" probe (domain OIDs are gated).
	if ! sysctl -n "${UNIT_OID}" >/dev/null 2>&1; then
		atf_skip "amd_rapl(4) not attached; nothing to cycle"
	fi
	# Built-in driver has no .ko to cycle; skip rather than fail.
	if ! kldstat -n "${MODNAME}" >/dev/null 2>&1; then
		atf_skip "amd_rapl(4) is built into the kernel; cannot kldunload"
	fi
	# Teardown is the riskiest path; a panic here takes the host down.
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
	# Restore the loaded state for later tests (reload is a harmless no-op).
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
