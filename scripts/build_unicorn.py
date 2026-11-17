#!/usr/bin/env python3
"""Optional: build a faster, more correct libunicorn for SIMEMU (Unicorn 2.1.4 +
scripts/unicorn-2.1.4-simemu.patch).

The emulator works with any stock Unicorn 2.1.x (Homebrew, distro packages, the Python wheel);
simemu-tc1791 corrects the translator defects it can from code hooks (`isa_fix.rs`). The patch
fixes TriCore ISA defects in the translator itself (reported through `uc_ctl` 0x5152; the board
then skips its own hooks for them):

  * psw_read() keeps PSW[24] (RM[0]); fpu_set_state() maps PSW.RM to the softfloat rounding mode;
    UPDFL updates PSW[31:24] under its mask; RET keeps PSW.RM (TC1.3.1/TC1.6); a PSW register
    write updates the FPU rounding mode. Stock Unicorn gets the FPU rounding mode wrong and
    cannot be corrected from hooks, so stock and patched builds produce different
    `crates/simemu-tc1791/tests/perf_determinism.rs` digests;
  * SRO LD.H scales off4 by 2; MTCR ICR keeps the read-only ICR.PIPN, and MFCR/MTCR ICR use the
    TCG global that ENABLE/DISABLE/RESTORE update (same results as the board's hooks, faster).

Speed-ups, TriCore target only (they do not change what the firmware executes):

  * find_memory_mapping(): one-entry MRU cache (dropped on every address-space change);
  * TriCore JI/JLI/CALLI/FCALLI/RET chain to the next block via lookup_and_goto_ptr instead of
    returning to the cpu_exec loop (as cross-page jumps already do);
  * context save areas (CALL/RET/RFE/BISR/SVLCX/RSLCX): one TLB probe for the 64 bytes instead
    of 16 full softmmu accesses when the area is plain RAM and no memory hook could observe the
    accesses (the self-modifying-code bitmap check is made correct for such wide writes);
  * TLB re-protection after a translation checks only the page's own TLB slots;
  * single-address UC_HOOK_CODE hooks are found by address instead of walking the whole list;
  * the "exit requested?" check after every guest load/store moves into the softmmu slow path,
    the only place where callbacks (MMIO, memory hooks) can request it;
  * optional inline block accounting (`uc_ctl` 0x5151, used by simemu-tc1791 when available):
    the per-block instruction count is added by the translated code instead of a Rust callback.

Exact instruction timing (`uc_ctl` 0x5153, used by simemu-tc1791 when available; it gives the
Unicorn backend the time semantics of the native core, at a cost in speed, see ARCHITECTURE.md §3.1):

  * a peripheral access sees the count of instructions retired before its instruction
    (`simemu_io_begin`; the block prologue counted the whole block);
  * an instruction that accesses a peripheral ends its translation block (found at its first access,
    which restarts it at the end of a block, as QEMU's deterministic I/O rule cpu_io_recompile), so
    a stop requested by the access takes effect at the next instruction;
  * the block hook can have a block re-run limited to N instructions (`helper_simemu_truncate`,
    cflags instruction count), so a quantum ends at exactly its budget.

Requirements: Python 3, git, CMake >= 3.5, a C compiler (Xcode clang / gcc / MSVC); Ninja optional.

    python3 scripts/build_unicorn.py            # -> target/unicorn-simemu/build
    # macOS / Linux: link it statically
    UNICORN_LIB_DIR=$PWD/target/unicorn-simemu/build UNICORN_STATIC=1 cargo build --release
    # Windows (dynamic; build.rs copies unicorn.dll next to the binaries)
    set UNICORN_LIB_DIR=%CD%\\target\\unicorn-simemu\\build\\Release   (folder with unicorn.lib/.dll)
"""
import argparse
import os
import shutil
import subprocess
import sys

TAG = "2.1.4"
URL = "https://github.com/unicorn-engine/unicorn.git"
HERE = os.path.dirname(os.path.abspath(__file__))


def run(cmd, cwd=None):
    print("+", " ".join(cmd), flush=True)
    subprocess.run(cmd, cwd=cwd, check=True)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default=os.path.join(HERE, "..", "target", "unicorn-simemu"), help="work directory")
    ap.add_argument("--stock", action="store_true", help="build unpatched (for comparisons)")
    a = ap.parse_args()
    for tool in ("git", "cmake"):
        if not shutil.which(tool):
            print(f"error: required build tool '{tool}' not found in PATH", file=sys.stderr)
            return 1
    patch = os.path.join(HERE, "unicorn-2.1.4-simemu.patch")
    if not a.stock and not os.path.isfile(patch):
        print(f"error: patch file not found: {patch}", file=sys.stderr)
        return 1
    out = os.path.abspath(a.out)
    src, build = os.path.join(out, "src"), os.path.join(out, "build")
    try:
        if not os.path.isdir(src):
            run(["git", "clone", "--depth", "1", "--branch", TAG, URL, src])
        applied = subprocess.run(["git", "apply", "--reverse", "--check", patch], cwd=src, capture_output=True).returncode == 0
        if a.stock or not applied:
            run(["git", "checkout", "--", "."], cwd=src)  # pristine TAG, also drops an older patch version
        if not a.stock and not applied:
            run(["git", "apply", patch], cwd=src)
        cfg = ["cmake", "-S", src, "-B", build, "-DCMAKE_BUILD_TYPE=Release", "-DUNICORN_ARCH=tricore",
               "-DUNICORN_BUILD_TESTS=OFF", "-DCMAKE_POLICY_VERSION_MINIMUM=3.5"]
        if shutil.which("ninja"):
            cfg += ["-G", "Ninja"]
        run(cfg)
        run(["cmake", "--build", build, "--config", "Release", "--parallel"])
    except subprocess.CalledProcessError as exc:
        print(f"error: command failed with exit code {exc.returncode}: {' '.join(exc.cmd)}", file=sys.stderr)
        return exc.returncode or 1
    print(f"\nlibunicorn built in {build}")
    if sys.platform == "win32":
        print(f"set UNICORN_LIB_DIR=<folder of unicorn.lib / unicorn.dll under {build}>")
    else:
        print(f"export UNICORN_LIB_DIR={build} UNICORN_STATIC=1")
    return 0


if __name__ == "__main__":
    sys.exit(main())
