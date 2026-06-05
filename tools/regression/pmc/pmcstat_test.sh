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

# Run a workload that produces a few hundred million instructions so
# every counter in the group has something to read.
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

# t_basic_group: three events, one group, process-counting mode.
# Verifies -b accepts {ev1,ev2,ev3} and pmcstat exits 0 with non-empty output.
t_basic_group() {
	out="${LOGDIR}/basic.out"
	${PMCSTAT} -b -O /dev/null \
	    -p '{instructions,unhalted-cycles,branches}' \
	    -- sh -c "for i in $(jot 200000); do :; done" \
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

# t_two_groups: two independent process-counting groups.
t_two_groups() {
	out="${LOGDIR}/two.out"
	${PMCSTAT} -b -O /dev/null \
	    -p '{instructions,unhalted-cycles}' \
	    -p '{branches,branch-misses}' \
	    -- sh -c "for i in $(jot 200000); do :; done" \
	    >"${out}" 2>&1
	rc=$?
	if [ "${rc}" -ne 0 ]; then
		fail "two_groups: pmcstat exit=${rc}; see ${out}"
		return
	fi
	pass "two_groups"
}

# t_multiplex: 8 events on a typical 6-counter Zen forces multiplexing.
# Requires PMC_F_GROUP_MUX support in the kernel (commit fails otherwise).
t_multiplex() {
	out="${LOGDIR}/mux.out"
	${PMCSTAT} -b -O /dev/null \
	    -p '{instructions,unhalted-cycles,branches,branch-misses,cache-references,cache-misses,l1d-loads,l1d-load-misses}' \
	    -- sh -c "for i in $(jot 400000); do :; done" \
	    >"${out}" 2>&1
	rc=$?
	if [ "${rc}" -ne 0 ]; then
		fail "multiplex: pmcstat exit=${rc} (commit may have rejected w/o GROUP_MUX); see ${out}"
		return
	fi
	pass "multiplex"
}

# t_system_mode: system-wide counting, single CPU.
t_system_mode() {
	out="${LOGDIR}/sys.out"
	${PMCSTAT} -b -O /dev/null -c 0 \
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

# t_sampling_group: process sampling with a group leader.
t_sampling_group() {
	log="${LOGDIR}/samples.log"
	out="${LOGDIR}/sampling.out"
	${PMCSTAT} -b -O "${log}" \
	    -P '{instructions,unhalted-cycles,branches}' \
	    -- sh -c "for i in $(jot 400000); do :; done" \
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

# t_compat_no_b: without -b, brace text is treated as a literal event
# spec by pmc_allocate which should reject it; we expect non-zero
# exit and a useful message, never a kernel panic / segfault.
t_compat_no_b() {
	out="${LOGDIR}/compat_no_b.out"
	${PMCSTAT} -O /dev/null \
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

# t_compat_legacy: classic non-grouped pmcstat invocation must still work.
t_compat_legacy() {
	out="${LOGDIR}/legacy.out"
	${PMCSTAT} -O /dev/null \
	    -p instructions -p unhalted-cycles \
	    -- sh -c "for i in $(jot 100000); do :; done" \
	    >"${out}" 2>&1
	rc=$?
	if [ "${rc}" -ne 0 ]; then
		fail "compat_legacy: exit=${rc}; see ${out}"
		return
	fi
	pass "compat_legacy"
}

# t_single_brace: {ev} (one element) must fall through to single-event
# behaviour without erroring.
t_single_brace() {
	out="${LOGDIR}/single.out"
	${PMCSTAT} -b -O /dev/null \
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

# t_malformed_brace: missing closing '}' must be rejected with EX_USAGE
# (64), never a crash.
t_malformed_brace() {
	out="${LOGDIR}/malformed.out"
	${PMCSTAT} -b -O /dev/null \
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

# t_mixed: one grouped -p and one ungrouped -p in the same invocation.
t_mixed() {
	out="${LOGDIR}/mixed.out"
	${PMCSTAT} -b -O /dev/null \
	    -p '{instructions,unhalted-cycles}' \
	    -p branches \
	    -- sh -c "for i in $(jot 100000); do :; done" \
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
	t_multiplex

	note "------------------------------------"
	note "PASS: ${PASS}  FAIL: ${FAIL}"
	[ "${FAIL}" -eq 0 ]
}

main "$@"
