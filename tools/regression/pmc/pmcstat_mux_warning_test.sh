#!/bin/sh
#
# Check that pmcstat sends the long-sampling-period warning only after
# multiplexing starts.
#

set -u

: "${PMCSTAT:?set PMCSTAT to the pmcstat binary under test}"
: "${PMCSTAT_WARNING_SHIM:?set PMCSTAT_WARNING_SHIM to the preload shim}"

if [ ! -x "${PMCSTAT}" ]; then
	echo "ERROR: pmcstat binary is not executable: ${PMCSTAT}" >&2
	exit 2
fi
if [ ! -r "${PMCSTAT_WARNING_SHIM}" ]; then
	echo "ERROR: preload shim is not readable: ${PMCSTAT_WARNING_SHIM}" >&2
	exit 2
fi

tmpdir=$(mktemp -d -t pmcstat_mux_warning) || exit 2
trap 'rm -rf "${tmpdir}"' EXIT

pass=0
fail=0

run_case()
{
	name=$1
	enabled=$2
	running=$3
	expected=$4
	out="${tmpdir}/${name}.out"

	PMCSTAT_TEST_ENABLED=${enabled} \
	    PMCSTAT_TEST_RUNNING=${running} \
	    LD_PRELOAD=${PMCSTAT_WARNING_SHIM} \
	    "${PMCSTAT}" -b -n 100 \
	    -S '{instructions,unhalted-cycles}' \
	    -l 0.01 -O /dev/null >"${out}" 2>&1
	rc=$?
	if [ "${rc}" -ne 0 ]; then
		echo "FAIL: ${name}: pmcstat exit=${rc}; output follows"
		sed 's/^/  /' "${out}"
		fail=$((fail + 1))
		return
	fi

	count=$(grep -c \
	    'kern.hwpmc.mux_period_ms rotation window can retire' \
	    "${out}" || true)
	if [ "${count}" -ne "${expected}" ]; then
		echo "FAIL: ${name}: warning count=${count}, expected ${expected}"
		sed 's/^/  /' "${out}"
		fail=$((fail + 1))
		return
	fi

	echo "PASS: ${name}: warning count=${count}"
	pass=$((pass + 1))
}

# This case catches a bug: the warning appears only because the braces set PMC_F_GROUP_MUX.
run_case resident 100 100 0

# This case catches a bug: the code deletes the warning instead of adding a condition to it.
run_case multiplexed 100 50 2

echo "Results: ${pass} pass, ${fail} fail"
[ "${fail}" -eq 0 ]
