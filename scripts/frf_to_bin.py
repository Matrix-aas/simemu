#!/usr/bin/env python3
"""Optional CLI helper delegating VAG ODIS `.frf` / `.odx` unpacking to the native Rust `simemu unpack-frf` command.

Note: SimEmu loads and executes `.frf` and `.odx` containers natively in memory (`simemu_damos::frf`),
so pre-unpacking `.frf` files to `.bin` is optional.
"""
import argparse
import pathlib
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("frf", type=pathlib.Path, help="input VAG ODIS .frf or .odx container")
    ap.add_argument(
        "-o",
        "--out",
        type=pathlib.Path,
        help="output 4 MB .bin path (default: vault/firmwares/ecu/stock/<stem>.bin)",
    )
    a = ap.parse_args()

    out = a.out or (
        ROOT
        / "vault"
        / "firmwares"
        / "ecu"
        / "stock"
        / (a.frf.stem.removeprefix("FL_").removeprefix("fl_") + ".bin")
    )
    exe_name = "simemu.exe" if sys.platform == "win32" else "simemu"
    release_bin = ROOT / "target" / "release" / exe_name
    if release_bin.is_file():
        cmd = [str(release_bin), "unpack-frf", str(a.frf), str(out)]
    else:
        cmd = ["cargo", "run", "--release", "--bin", "simemu", "--", "unpack-frf", str(a.frf), str(out)]
    return subprocess.call(cmd, cwd=ROOT)


if __name__ == "__main__":
    sys.exit(main())
