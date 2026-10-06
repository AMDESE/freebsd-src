#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-2-Clause
# Copyright (c) 2026 Advanced Micro Devices, Inc.

"""Compile production LBR/lifecycle code with fake MSRs and class callbacks.

Run from any directory: python3 tools/test/hwpmc/lbr_regression.py
Requires a C compiler, but no PMC hardware, module, root access, or kernel
installation. Use --sanitize for AddressSanitizer/UndefinedBehaviorSanitizer.
The scheduler fragments are taken from the real first-run and exit call sites,
not duplicated policy expressions. Temporary sources and binaries are removed.
"""

import argparse
import os
import re
import shlex
import subprocess
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
FIXTURES = Path(__file__).resolve().parent


def function(source, name):
    """Read a complete top-level C function, including its return type."""
    match = re.search(r"^" + re.escape(name) + r"\([^;]*?\n\{", source, re.MULTILINE)
    if match is None:
        raise ValueError(f"function not found: {name}")
    start = source.rfind("\n", 0, match.start() - 1) + 1
    end = source.index("\n}\n", match.end()) + 3
    return source[start:end]


def hwpmc_block(source):
    match = re.search(
        r"^#ifdef\s+HWPMC_HOOKS\n(.*?)^#endif", source, re.MULTILINE | re.DOTALL
    )
    return match.group(1) if match else ""


def generate(directory):
    amd = (ROOT / "sys/dev/hwpmc/hwpmc_amd.c").read_text()
    mod = (ROOT / "sys/dev/hwpmc/hwpmc_mod.c").read_text()
    functions = [
        "amd_lbr_reset",
        "amd_lbr_sync",
        "amd_lbr_update_begin",
        "amd_lbr_update_end",
        "amd_lbr_activate",
        "amd_lbr_deactivate",
        "amd_lbr_curcpu",
        "amd_lbr_csw",
        "amd_lbr_exec",
        "amd_lbr_read",
        "amd_intr_v2",
    ]
    (directory / "lbr.inc").write_text(
        "\n".join(function(amd, name) for name in functions)
    )
    fork = function((ROOT / "sys/kern/kern_fork.c").read_text(), "fork_exit")
    thread = function((ROOT / "sys/kern/kern_thread.c").read_text(), "thread_exit")
    (directory / "fork.inc").write_text(hwpmc_block(fork))
    (directory / "thread.inc").write_text(hwpmc_block(thread))
    exit_fn = function(mod, "pmc_process_exit")
    start = exit_fn.index("\tfor (ri = 0; ri < md->pmd_npmc; ri++) {")
    end = exit_fn.index("\n\t/*\n\t * Inform the MD layer", start)
    batch = function(mod, "pmc_process_csw_stop_all")
    if "pmc_process_csw_out_prepare(cpu)" in batch:
        batch = function(mod, "pmc_process_csw_out_prepare") + batch
    (directory / "exit.inc").write_text(batch)
    (directory / "exit_loop.inc").write_text(exit_fn[start:end])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sanitize", action="store_true")
    parser.add_argument("--case", choices=["snapshot", "lifecycle", "exit"])
    args = parser.parse_args()
    flags = ["-std=gnu11", "-O2", "-I", str(ROOT)]
    if args.sanitize:
        flags += ["-O1", "-g", "-fsanitize=address,undefined"]
    with tempfile.TemporaryDirectory(prefix="hwpmc-lbr-") as name:
        directory = Path(name)
        generate(directory)
        fixtures = ["lbr_regression", "pmc_exit_regression"]
        if args.case:
            fixtures = [
                "pmc_exit_regression" if args.case == "exit" else "lbr_regression"
            ]
        for fixture in fixtures:
            binary = directory / fixture
            command = shlex.split(os.environ.get("CC", "cc")) + flags
            command += ["-I", name, str(FIXTURES / (fixture + ".c")), "-o", str(binary)]
            subprocess.run(command, check=True)
            command = [str(binary)]
            if args.case and fixture == "lbr_regression":
                command.append(args.case)
            subprocess.run(command, check=True)


if __name__ == "__main__":
    main()
