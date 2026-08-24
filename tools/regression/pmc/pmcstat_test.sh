#!/bin/sh
#
# pmcstat(8) regression tests for -b / {a,b,c} group syntax.
#
# Must run as root with hwpmc(4) loaded on an AMD CPU.  A workload
# program is needed; we use /usr/bin/yes piped to /dev/null so each
# test has a few seconds of busy CPU to collect counts on.
#
# Each test prints a "PASS:" or "FAIL:" line and an exit-status row
# at the end.  The script returns 0 only if every case passes.
#
# Usage:
#     sudo sh tools/regression/pmc/pmcstat_test.sh
#

set -u

PMCSTAT=${PMCSTAT:-/usr/sbin/pmcstat}
WORKLOAD_SECS=${WORKLOAD_SECS:-3}
LOGDIR=$(mktemp -d -t pmcstat_test) || exit 1
SUMMARY="${LOGDIR}/summary.log"
PASS=0
FAIL=0

trap "rm -rf ${LOGDIR}" EXIT

note() { printf '%s\n' "$*" | tee -a "${SUMMARY}"; }
fail() { note "FAIL: $*"; FAIL=$((FAIL + 1)); }
pass() { note "PASS: $*"; PASS=$((PASS + 1)); }

# Run a CPU-bound workload for event counting.
busy_workload() {
	# `yes` is CPU-bound; redirect its stdout/stderr.
	yes >/dev/null 2>&1 &
	pid=$!
	sleep "${WORKLOAD_SECS}"
	kill -9 "${pid}" 2>/dev/null
	wait "${pid}" 2>/dev/null
}

require_root() {
	if [ "$(id -u)" -ne 0 ]; then
		echo "ERROR: must run as root" >&2
		exit 2
	fi
}

require_hwpmc() {
	if ! kldstat -q -m hwpmc; then
		if ! kldload hwpmc 2>/dev/null; then
			echo "ERROR: hwpmc(4) is not loaded and could not be loaded" >&2
			exit 2
		fi
	fi
}

require_amd() {
	cpuid=$(sysctl -n kern.hwpmc.cpuid 2>/dev/null || echo unknown)
	case "${cpuid}" in
	*AuthenticAMD*|*HygonGenuine*) ;;
	*) echo "SKIP: AMD-only test, cpuid=${cpuid}"; exit 77 ;;
	esac
}

# t_basic_group: verify group of three events in process counting mode.
t_basic_group() {
	out="${LOGDIR}/basic.out"
	${PMCSTAT} -b \
	    -p '{instructions,unhalted-cycles,ls_dispatch.ld_st_dispatch}' \
	    -- sh -c /usr/bin/true \
	    >"${out}" 2>&1
	rc=$?
	if [ "${rc}" -ne 0 ]; then
		fail "basic_group: pmcstat exit=${rc}; see ${out}"
		return
	fi
	if ! grep -q '[0-9]' "${out}"; then
		fail "basic_group: no numeric counter output in ${out}"
		return
	fi
	pass "basic_group"
}

# t_two_groups: verify two process counting groups.
t_two_groups() {
	out="${LOGDIR}/two.out"
	${PMCSTAT} -b  \
	    -p '{instructions,unhalted-cycles}' \
	    -p '{ls_alloc_mab_count, ls_not_halted_cyc, ls_dispatch.ld_st_dispatch}' \
	    -- sh -c /usr/bin/true \
	    >"${out}" 2>&1
	rc=$?
	if [ "${rc}" -ne 0 ]; then
		fail "two_groups: pmcstat exit=${rc}; see ${out}"
		return
	fi
	pass "two_groups"
}

# t_oversubscribe: verify oversubscribed single group fails.
t_oversubscribe() {
	out="${LOGDIR}/mux.out"
	${PMCSTAT} -b -O /dev/null \
	    -p '{instructions,unhalted-cycles,ls_alloc_mab_count, ls_not_halted_cyc, ls_dispatch.ld_st_dispatch,ls_stlf, ls_int_taken}' \
	    -- sh -c /usr/bin/true \
	    >"${out}" 2>&1
	rc=$?
	if [ "${rc}" -eq 0 ]; then
		fail "oversubscribe: should have rejected"
		return
	fi
	if [ "${rc}" -ge 128 ]; then
		fail "oversubscribe: signal exit=${rc}"
		return
	fi
	pass "oversubscribe"
}

# t_system_mode: verify system-wide counting group on a single CPU.
t_system_mode() {
	out="${LOGDIR}/sys.out"
	${PMCSTAT} -b -c 0 \
	    -s '{instructions,unhalted-cycles}' \
	    -- sleep "${WORKLOAD_SECS}" \
	    >"${out}" 2>&1
	rc=$?
	if [ "${rc}" -ne 0 ]; then
		fail "system_mode: pmcstat exit=${rc}; see ${out}"
		return
	fi
	pass "system_mode"
}

# t_sampling_group: verify process sampling group.
t_sampling_group() {
	log="${LOGDIR}/samples.log"
	out="${LOGDIR}/sampling.out"
	${PMCSTAT} -b \
	    -P '{instructions,unhalted-cycles,ls_dispatch.ld_st_dispatch}' \
	    -O "${log}" \
	    -- sh -c 'i=0; while [ $i -lt 200000 ]; do i=$((i+1)); done' \
	    >"${out}" 2>&1
	rc=$?
	if [ "${rc}" -ne 0 ]; then
		fail "sampling_group: pmcstat exit=${rc}; see ${out}"
		return
	fi
	if [ ! -s "${log}" ]; then
		fail "sampling_group: empty log ${log}"
		return
	fi
	pass "sampling_group"
}

# t_compat_no_b: verify brace syntax without -b fails cleanly.
t_compat_no_b() {
	out="${LOGDIR}/compat_no_b.out"
	${PMCSTAT} \
	    -p '{instructions,unhalted-cycles}' \
	    -- /usr/bin/true \
	    >"${out}" 2>&1
	rc=$?
	# Without -b the literal "{instructions,...}" string is invalid
	# event-spec syntax, so pmc_allocate must fail; pmcstat reports it
	# via err(EX_OSERR, ...).  Exit must be non-zero, NOT a SIGSEGV.
	if [ "${rc}" -eq 0 ]; then
		fail "compat_no_b: should have rejected brace without -b"
		return
	fi
	if [ "${rc}" -ge 128 ]; then
		fail "compat_no_b: pmcstat killed by signal (exit=${rc})"
		return
	fi
	pass "compat_no_b"
}

# t_compat_legacy: verify non-grouped events operate normally.
t_compat_legacy() {
	out="${LOGDIR}/legacy.out"
	${PMCSTAT} \
	    -p instructions -p unhalted-cycles \
	    -- sh -c /usr/bin/true \
	    >"${out}" 2>&1
	rc=$?
	if [ "${rc}" -ne 0 ]; then
		fail "compat_legacy: exit=${rc}; see ${out}"
		return
	fi
	pass "compat_legacy"
}

# t_single_brace: verify single-element group operates as single event.
t_single_brace() {
	out="${LOGDIR}/single.out"
	${PMCSTAT} -b \
	    -p '{instructions}' \
	    -- /usr/bin/true \
	    >"${out}" 2>&1
	rc=$?
	if [ "${rc}" -ne 0 ]; then
		fail "single_brace: exit=${rc}; see ${out}"
		return
	fi
	pass "single_brace"
}

# t_malformed_brace: verify missing closing brace fails cleanly.
t_malformed_brace() {
	out="${LOGDIR}/malformed.out"
	${PMCSTAT} -b \
	    -p '{instructions,unhalted-cycles' \
	    -- /usr/bin/true \
	    >"${out}" 2>&1
	rc=$?
	if [ "${rc}" -eq 0 ]; then
		fail "malformed_brace: should have rejected"
		return
	fi
	if [ "${rc}" -ge 128 ]; then
		fail "malformed_brace: signal exit=${rc}"
		return
	fi
	pass "malformed_brace"
}

# t_mixed: verify grouped and ungrouped events in same command.
t_mixed() {
	out="${LOGDIR}/mixed.out"
	${PMCSTAT} -b \
	    -p '{instructions,unhalted-cycles}' \
	    -p ls_dispatch.ld_st_dispatch \
	    -- sh -c /usr/bin/true \
	    >"${out}" 2>&1
	rc=$?
	if [ "${rc}" -ne 0 ]; then
		fail "mixed: exit=${rc}; see ${out}"
		return
	fi
	pass "mixed"
}

main() {
	require_root
	require_hwpmc
	require_amd

	note "pmcstat(8) regression tests; logdir=${LOGDIR}"

	t_compat_legacy
	t_basic_group
	t_two_groups
	t_system_mode
	t_sampling_group
	t_single_brace
	t_mixed
	t_compat_no_b
	t_malformed_brace
	t_oversubscribe

	note "------------------------------------"
	note "PASS: ${PASS}  FAIL: ${FAIL}"
	[ "${FAIL}" -eq 0 ]
}

main "$@"
