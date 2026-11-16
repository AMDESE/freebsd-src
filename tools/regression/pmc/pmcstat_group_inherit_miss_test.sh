#!/bin/sh

set -eu

pmcstat=${PMCSTAT:-/usr/sbin/pmcstat}
fixture=${PMCGROUP_MISS_FIXTURE:-./pmcstat_group_inherit_miss_fixture}
tmpdir=$(mktemp -d -t pmcstat_group_miss) || {
	echo "FAIL: mktemp -d"
	echo "Results: 0 pass, 1 fail"
	exit 1
}
logfile="${tmpdir}/input.log"
stdout="${tmpdir}/stdout"
stderr="${tmpdir}/stderr"

cleanup()
{
	rm -f "${logfile}" "${stdout}" "${stderr}"
	rmdir "${tmpdir}" 2>/dev/null || :
}
trap cleanup EXIT
trap 'exit 1' HUP INT TERM

if ! "${fixture}" >"${logfile}"; then
	echo "FAIL: fixture failed"
	echo "Results: 0 pass, 1 fail"
	exit 1
fi

set +e
"${pmcstat}" -R "${logfile}" >"${stdout}" 2>"${stderr}"
rc=$?
set -e

if [ "${rc}" -ne 0 ]; then
	echo "FAIL: pmcstat -R exited ${rc}, expected 0"
	sed 's/^/stderr: /' "${stderr}"
	echo "Results: 0 pass, 1 fail"
	exit 1
fi
if [ -s "${stderr}" ]; then
	echo "FAIL: pmcstat -R wrote unexpected stderr"
	sed 's/^/stderr: /' "${stderr}"
	echo "Results: 0 pass, 1 fail"
	exit 1
fi

expected='group-inherit-miss 0x10203040 4242    72623859790382856'
line_count=$(wc -l <"${stdout}" | tr -d ' ')
actual=$(sed -n '1p' "${stdout}")
if [ "${line_count}" -ne 1 ] || [ "${actual}" != "${expected}" ]; then
	printf "FAIL: output='%s', expected='%s'\n" "${actual}" "${expected}"
	echo "Results: 0 pass, 1 fail"
	exit 1
fi

echo "PASS: group-inherit-miss text matches"
echo "Results: 1 pass, 0 fail"
