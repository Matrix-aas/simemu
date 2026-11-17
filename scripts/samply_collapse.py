#!/usr/bin/env python3
"""Collapse a `samply record --save-only --unstable-presymbolicate` profile into folded stacks.

    samply_collapse.py PROFILE.json.gz [--thread SUBSTR] [--top N] > out.folded
    inferno-flamegraph out.folded > out.svg

Folded output (root;...;leaf count, inline frames expanded) goes to stdout; with --top the N
hottest functions by self and by total samples are printed to stderr. See docs/PERFORMANCE.md.
"""
import argparse
import bisect
import collections
import gzip
import json
import sys


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("profile", help="samply profile (.json or .json.gz)")
    ap.add_argument("--thread", default="", help="only threads whose name contains this")
    ap.add_argument("--top", type=int, default=0, help="print top N functions by self/total samples to stderr")
    a = ap.parse_args()
    syms_path = a.profile.removesuffix(".gz").removesuffix(".json") + ".json.syms.json"
    try:
        if a.profile.endswith(".gz"):
            with gzip.open(a.profile, "rt", encoding="utf-8") as fp:
                prof = json.load(fp)
        else:
            with open(a.profile, "r", encoding="utf-8") as fp:
                prof = json.load(fp)
    except (OSError, json.JSONDecodeError) as exc:
        print(f"error: cannot read profile {a.profile!r}: {exc}", file=sys.stderr)
        return 1
    try:
        with open(syms_path, "r", encoding="utf-8") as fp:
            syms = json.load(fp)
    except (OSError, json.JSONDecodeError) as exc:
        print(
            f"error: cannot read symbol sidecar {syms_path!r} "
            f"(record with `samply record --save-only --unstable-presymbolicate`): {exc}",
            file=sys.stderr,
        )
        return 1
    st = syms["string_table"]
    by_lib = {}
    for d in syms["data"]:
        table = d["symbol_table"]
        rvas = [e["rva"] for e in table]
        by_lib[d["debug_name"]] = (rvas, table)

    def names(lib, addr):
        """Frames for `addr` in `lib`, outermost first (inline expansion)."""
        if lib not in by_lib:
            return [f"{lib}+{addr:#x}"]
        rvas, table = by_lib[lib]
        i = bisect.bisect_right(rvas, addr) - 1
        if i < 0 or addr >= table[i]["rva"] + table[i]["size"]:
            return [f"{lib}+{addr:#x}"]
        e = table[i]
        fr = e.get("frames")
        if not fr:
            return [st[e["symbol"]]]
        # frames are innermost first; the outermost is the symbol itself
        return [st[e["symbol"]]] + [st[f["function"]] for f in reversed(fr[:-1])]

    folded = collections.Counter()
    self_c, total_c = collections.Counter(), collections.Counter()
    libs = [l["debugName"] for l in prof["libs"]]
    for t in prof["threads"]:
        if a.thread not in t["name"]:
            continue
        ft, stt, res = t["frameTable"], t["stackTable"], t["resourceTable"]
        fn, strs = t["funcTable"], t["stringArray"]
        cache = {}

        def frame_names(f):
            if f in cache:
                return cache[f]
            r = fn["resource"][ft["func"][f]]
            lib = libs[res["lib"][r]] if r is not None and r >= 0 else "?"
            addr = ft["address"][f]
            n = names(lib, addr) if addr is not None and addr >= 0 else [strs[fn["name"][ft["func"][f]]]]
            cache[f] = n
            return n

        for s in t["samples"]["stack"]:
            chain = []
            while s is not None:
                chain.append(stt["frame"][s])
                s = stt["prefix"][s]
            path = []
            for f in reversed(chain):
                path.extend(frame_names(f))
            if not path:
                continue
            path = [p.replace(";", ":") for p in path]
            folded[";".join([t["name"]] + path)] += 1
            self_c[path[-1]] += 1
            for p in set(path):
                total_c[p] += 1
    for k, v in folded.items():
        print(f"{k} {v}")
    if a.top:
        n = sum(self_c.values()) or 1
        print(f"samples {n}", file=sys.stderr)
        print("-- self --", file=sys.stderr)
        for k, v in self_c.most_common(a.top):
            print(f"{100 * v / n:6.2f}%  {k}", file=sys.stderr)
        print("-- total --", file=sys.stderr)
        for k, v in total_c.most_common(a.top):
            print(f"{100 * v / n:6.2f}%  {k}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
