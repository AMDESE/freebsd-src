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

# Skip the calling test unless the amd_rapl energy sysctls are present.
require_amdrapl()
{
	if ! sysctl -n "${PKG_OID}" >/dev/null 2>&1; then
		atf_skip "amd_rapl(4) energy sysctls not present" \
		    "(no AMD RAPL hardware or driver not loaded)"
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
	require_amdrapl
	v1=$(core0_uj)
	sleep 2
	v2=$(core0_uj)
	if [ "${v2}" -lt "${v1}" ]; then
		atf_fail "cores_energy_uj[0] decreased: ${v1} -> ${v2}"
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

atf_init_test_cases()
{
	atf_add_test_case package_energy_monotonic
	atf_add_test_case package_energy_advances
	atf_add_test_case cores_energy_monotonic
	atf_add_test_case package_power_sane
}
