#!/usr/bin/env python3
"""Static TriCore instruction census of the firmware images (rizin/capstone as decoder).

Code discovery, per image:
  * regions: flash code ranges + RAM code (PSPR) images rebuilt from the firmware's own
    flash->RAM copy tables ({dst, src, len, 1} records). PCP code (copy records with a CMEM
    destination 0xF006xxxx) is excluded everywhere: it is not TriCore code.
  * tier 1 (reachable): recursive descent from the reset vector, following direct branches/calls
    and constants built with MOVH/ADDI/LEA/ADDIH that reach CALLI/JI/JLI or MTCR BIV/BTV
    (vector tables: BTV 8 classes, BIV 256 priorities, 32-byte spacing, TC1.6 vol1 5-16/6-...).
  * tier 2 (pointer-seeded): additionally every aligned data word that points into a code region
    and whose decoded run reaches a block end without an invalid encoding (task tables, callbacks).
  * linear: plain linear sweep of the code regions (upper bound, includes data decoded as code).

Formats come from the TC1.6 manual encoding table (parse_manual.py) keyed by (mnemonic, op1).

Usage: insn_census.py <manual_tc16.json> <out.json> <out.md>
"""
import json
import os
import re
import struct
import subprocess
import sys
from collections import Counter, defaultdict

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../.."))
SCRATCH = os.path.join(ROOT, "build", "census")
RZ_CFG = [
    "e scr.color=0", "e asm.lines=false", "e asm.comments=false", "e asm.flags=false",
    "e asm.xrefs=false", "e asm.functions=false", "e asm.bytes=true", "e asm.lines.bb=false",
    "e asm.var=false", "e asm.marks=false", "e asm.cmt.right=false", "e scr.utf8=false",
    "e asm.analysis=false", "e asm.sub.jmp=false",
]
LINE = re.compile(r"^0x([0-9a-f]+)\s+([0-9a-f]+)\s+(\S+)\s*(.*)$")
ADDR = re.compile(r"0x([0-9a-f]+)\s*$")

STOP = {"ret", "rfe", "rfm", "ji", "j", "ja", "loopu", "fret", "debug_stop"}
CALL = {"call", "calla", "calli", "jl", "jla", "jli", "fcall", "fcalla", "fcalli"}
INDIRECT = {"calli", "ji", "jli", "fcalli"}
COND = {
    "jeq", "jne", "jge", "jge.u", "jlt", "jlt.u", "jz", "jnz", "jgez", "jgtz", "jlez", "jltz",
    "jz.t", "jnz.t", "jz.a", "jnz.a", "jeq.a", "jne.a", "jned", "jnei", "loop",
}
CSFR_BIV, CSFR_BTV = 0xFE20, 0xFE24
# mnemonics whose first operand is not a destination register
NO_DEST = ("st", "j", "call", "mtcr", "loop", "cachea", "cachei", "ldmst", "swap", "cmpswap", "fcall")


def rz_run(blob_path, base, cmds):
    script = os.path.join(SCRATCH, "cmd.rz")
    with open(script, "w") as f:
        f.write("\n".join(RZ_CFG + cmds) + "\n")
    out = subprocess.run(
        ["rizin", "-a", "tricore", "-b", "32", "-m", hex(base), "-q", "-i", script, blob_path],
        capture_output=True, text=True, check=True).stdout
    table = {}
    for l in out.splitlines():
        m = LINE.match(l.strip())
        if m:
            a = int(m.group(1), 16)
            table.setdefault(a, (len(m.group(2)) // 2, m.group(2), m.group(3), m.group(4).strip()))
    return table


class Region:
    def __init__(self, img, base, data):
        self.base, self.data = base, data
        self.path = os.path.join(SCRATCH, f"{img}_{base:08x}.bin")
        with open(self.path, "wb") as f:
            f.write(data)
        self.table = {}

    def contains(self, a):
        return self.base <= a < self.base + len(self.data)

    def linear(self):
        self.table.update(rz_run(self.path, self.base, [f"pD {len(self.data)} @ {self.base:#x}"]))

    def decode_more(self, addrs):
        for i in range(0, len(addrs), 4000):
            t = rz_run(self.path, self.base, [f"pd 48 @ {a:#x}" for a in addrs[i:i + 4000]])
            for a, v in t.items():
                self.table.setdefault(a, v)


def norm(a):
    """Cached (0x8...) and non-cached (0xA...) flash views are the same code."""
    return a & 0xDFFFFFFF if a >> 28 in (0x8, 0xA) else a


def copy_records(blob):
    """{dst, src, len, 1} flash->RAM copy-table records."""
    out = []
    for i in range(0, len(blob) - 16, 4):
        d, s, n, one = struct.unpack_from("<4I", blob, i)
        if one == 1 and 0 < n < 0x10000 and s >> 24 in (0x80, 0xA0) and d >> 16 in (0xC000, 0xD400, 0xF006):
            out.append((d, norm(s), n))
    return out


def imm(tok):
    tok = tok.strip().lstrip("#").rstrip(",")
    try:
        return int(tok, 0)
    except ValueError:
        return None


class Image:
    def __init__(self, name, path, flash_maps, entry, pspr_dst):
        self.name, self.entry = name, entry
        blob = open(path, "rb").read()
        self.recs = copy_records(blob)
        self.pcp = [(s, s + n) for d, s, n in self.recs if d >> 16 == 0xF006]
        self.regions = []
        for off, size, base in flash_maps:
            data = bytearray(blob[off:off + size])
            for lo, hi in self.pcp:  # PCP code is not TriCore code
                for a in range(max(lo, base), min(hi, base + size)):
                    data[a - base] = 0xFF
            self.regions.append(Region(name, base, bytes(data)))
        ram = [(d, s, n) for d, s, n in self.recs if d >> 16 == pspr_dst >> 16]
        if ram:
            lo, hi = min(d for d, _, _ in ram), max(d + n for d, _, n in ram)
            img = bytearray(hi - pspr_dst)
            for d, s, n in ram:
                r = next((r for r in self.regions if r.contains(s)), None)
                if r:
                    img[d - pspr_dst:d - pspr_dst + n] = r.data[s - r.base:s - r.base + n]
            self.regions.append(Region(name, pspr_dst, bytes(img)))
        self.blob = blob

    def region(self, a):
        return next((r for r in self.regions if r.contains(a)), None)

    def insn(self, a):
        r = self.region(a)
        return r.table.get(a) if r else None

    def pointer_seeds(self):
        seeds = set()
        for r in self.regions:
            d = r.data
            for i in range(0, len(d) - 3, 4):
                v = norm(struct.unpack_from("<I", d, i)[0])
                if v & 1 == 0 and v != r.base + i and self.region(v):
                    seeds.add(v)
        return seeds

    def descend(self, seeds, validate, known=()):
        """Recursive descent. Returns (visited instruction addresses, extra seeds from constants)."""
        seen, work, const_seeds = set(known), list(seeds), set()
        verdict = {}
        while work:
            missing = defaultdict(set)
            nxt = []
            for a in work:
                if a in seen or not self.region(a):
                    continue
                if validate:
                    v = verdict.get(a)
                    if v is None:
                        v = self.valid_run(a, seen)
                    if isinstance(v, int) and not isinstance(v, bool):  # needs decoding at v
                        missing[self.region(v)].add(v)
                        nxt.append(a)
                        continue
                    verdict[a] = v
                    if not v:
                        continue
                if self.insn(a) is None:
                    missing[self.region(a)].add(a)
                    nxt.append(a)
                    continue
                nxt.extend(self.walk(a, seen, const_seeds))
            for r, addrs in missing.items():
                r.decode_more(sorted(addrs))
                for a in addrs:  # undecodable: give up on it
                    r.table.setdefault(a, (2, "", "invalid", ""))
            work = [a for a in nxt if a not in seen]
        return seen, const_seeds

    def valid_run(self, a, known, limit=256):
        """Seed validation: the run decodes without an invalid encoding, does not start inside a known
        instruction, is not padding, keeps its direct branch targets inside code regions and reaches a
        block end. Returns True/False, or an address that must be decoded first."""
        if (a - 2) in known and self.insn(a - 2) and self.insn(a - 2)[0] == 4:
            return False
        for _ in range(limit):
            i = self.insn(a)
            if i is None:
                return a if self.region(a) else False
            size, hexb, m, ops = i
            if m == "invalid" or m.startswith("unk") or hexb in ("0000", "ffff", "ffffffff", "00000000"):
                return False
            t = ADDR.search(ops)
            if t and (m in CALL or m in COND or m in ("j", "ja", "loopu")) and not self.region(norm(int(t.group(1), 16))):
                return False
            if m in STOP or a in known:
                return True
            a += size
        return True

    def walk(self, a, seen, const_seeds):
        """Linear walk of one run; returns branch targets to visit."""
        out, regs = [], {}
        while a not in seen:
            i = self.insn(a)
            if i is None:
                out.append(a)
                break
            size, _, m, ops = i
            if m == "invalid":
                break
            seen.add(a)
            tgt = ADDR.search(ops)
            t = norm(int(tgt.group(1), 16)) if tgt else None
            if m in INDIRECT or (m == "mtcr" and ops):
                reg = ops.split(",")[-1].strip() if m == "mtcr" else ops.strip()
                v = regs.get(reg)
                if m == "mtcr":
                    csfr = (imm(ops.split(",")[0]) or 0) & 0xFFFF
                    if v is not None and csfr in (CSFR_BIV, CSFR_BTV):
                        n = 8 if csfr == CSFR_BTV else 256
                        const_seeds.update(norm(v + 32 * k) for k in range(n))
                elif v is not None:
                    const_seeds.add(norm(v))
            self.track(m, ops, regs)
            if m in CALL or m in COND:
                if t is not None and m not in INDIRECT:
                    out.append(t)
            elif m in ("j", "ja", "loopu") and t is not None:
                out.append(t)
            if m in STOP:
                break
            a += size
        return out

    @staticmethod
    def track(m, ops, regs):
        o = [x.strip() for x in ops.split(",")]
        if m in ("movh", "movh.a") and len(o) == 2:
            v = imm(o[1])
            regs[o[0]] = (v << 16) & 0xFFFFFFFF if v is not None else None
        elif m in ("addi", "addih", "lea", "addih.a") and len(o) >= 2:
            if m == "lea":  # lea aX, [aY]#off
                mm = re.match(r"\[(a\d+)\]#?(-?0x[0-9a-f]+|-?\d+)?", o[1])
                base = regs.get(mm.group(1)) if mm else None
                off = imm(mm.group(2)) if mm and mm.group(2) else 0
            else:
                base, off = regs.get(o[1]), imm(o[2]) if len(o) > 2 else None
                if m.startswith("addih") and off is not None:
                    off <<= 16
            regs[o[0]] = (base + off) & 0xFFFFFFFF if base is not None and off is not None else None
        elif m in ("mov.a", "mov.d", "mov.aa", "mov") and len(o) == 2 and o[1] in regs:
            regs[o[0]] = regs[o[1]]
        elif m in CALL:
            regs.clear()  # callee clobbers the lower context
        elif o and o[0][:1] in ("d", "a", "e") and o[0][1:].isdigit() and not m.startswith(NO_DEST):
            regs[o[0]] = None
            if o[0][0] == "e":
                n = int(o[0][1:])
                regs[f"d{n}"] = regs[f"d{n + 1}"] = None


def fmt_of(enc, m, word, size):
    op1 = word & 0xFF
    key = m.upper()
    for k in (op1, op1 & 0x7F, op1 & 0x3F):
        f = enc.get((key, k, size))
        if f:
            return "/".join(sorted(f))
    return "?"


def main(manual, out_json, out_md):
    os.makedirs(SCRATCH, exist_ok=True)
    man = json.load(open(manual))
    enc = defaultdict(set)
    for e in man["encodings"]:
        enc[(e["mnemonic"], e["op1"], e["size"])].add(e["format"])
    # code sectors only: PFlash0, PFlash1 behind the CAL sector 0x80800000..0x8087FFFF (data)
    simos_maps = [(0, 0x200000, 0x80000000), (0x280000, 0x180000, 0x80880000)]
    images = []
    stock_dir = os.path.join(ROOT, "vault", "firmwares", "ecu", "stock")
    if os.path.isdir(stock_dir):
        for f in sorted(os.listdir(stock_dir)):
            if f.endswith(".bin"):
                images.append((f"SIMOS18 {f[:-4]} (TC1791)", os.path.join("vault/firmwares/ecu/stock", f), simos_maps, 0x80000000, 0xC0000000))
    dq200_dir = os.path.join(ROOT, "vault", "firmwares", "tcu", "dq200")
    if os.path.isdir(dq200_dir):
        for f in sorted(os.listdir(dq200_dir)):
            p = os.path.join(dq200_dir, f)
            if f.endswith(".bin") and os.path.getsize(p) == 0x2A0000:
                images.append(("DQ200_G2 0CW300043H (TC1784)", os.path.join("vault/firmwares/tcu/dq200", f),
                               [(0, 0x20000, 0x80000000), (0x40000, 0x260000, 0x80040000)], 0x80000000, 0xD4000000))
                break
    report = {"images": [], "mnemonics": {}, "formats": {}}
    per_mnem = defaultdict(lambda: defaultdict(Counter))
    per_fmt = defaultdict(Counter)
    for name, path, maps, entry, pspr in images:
        img = Image(name, os.path.join(ROOT, path), maps, entry, pspr)
        for r in img.regions:
            r.linear()
        t1, consts = img.descend([entry], validate=False)
        tried = set()
        while consts - t1 - tried:  # constants reaching CALLI/JI/JLI/BIV/BTV feed back into tier 1
            new = consts - t1 - tried
            tried |= new
            s, more = img.descend(sorted(new), validate=False, known=t1)
            t1 |= s
            consts |= more
        seeds = sorted(img.pointer_seeds() - t1)
        t2_only, _ = img.descend(seeds, validate=True, known=t1)
        t2_only -= t1
        tiers = {"tier1": t1, "tier2": t1 | t2_only}
        linear = {a for r in img.regions for a in r.table}
        stats = {"name": name, "file": path, "pcp_excluded": [f"{lo:#x}..{hi:#x}" for lo, hi in img.pcp],
                 "ram_code_regions": [f"{r.base:#x}+{len(r.data):#x}" for r in img.regions[len(maps):]]}
        for tier, addrs in [("tier1", t1), ("tier2", tiers["tier2"]), ("linear", linear)]:
            c = Counter()
            for a in addrs:
                i = img.insn(a)
                if not i or i[2] == "invalid":
                    continue
                size, hexb, m, _ = i
                word = int.from_bytes(bytes.fromhex(hexb), "little")
                f = fmt_of(enc, m, word, size)
                c[m] += 1
                per_mnem[m][tier][name] += 1
                per_mnem[m]["_fmt"][f"{f}:{size * 8}"] += 1 if tier == "tier2" else 0
                if tier == "tier2":
                    per_fmt[f][name] += 1
            stats[tier] = {"instructions": sum(c.values()), "mnemonics": len(c)}
        report["images"].append(stats)
        print(stats, file=sys.stderr)
    for m, d in per_mnem.items():
        report["mnemonics"][m] = {
            "tier1": dict(d["tier1"]), "tier2": dict(d["tier2"]), "linear": dict(d["linear"]),
            "formats": {k: v for k, v in d["_fmt"].items() if v},
            "in_tc16_manual": m.upper() in man["mnemonics"],
        }
    report["formats"] = {f: dict(c) for f, c in per_fmt.items()}
    write_md(report, out_md, [i[0] for i in images])
    json.dump(report, open(out_json, "w"), indent=1, sort_keys=True)


GROUP_FILES = {"A": "alu.rs", "B": "bitmul.rs", "C": "ldst.rs", "D": "ctrl.rs", "E": "fpu_div.rs"}


def group_of_mnemonics():
    """mnemonic (manual spelling) -> group letter, from `pub const MNEMONICS` of each group module."""
    out = {}
    for g, f in GROUP_FILES.items():
        src = open(os.path.join(ROOT, "crates/tricore-core/src/groups", f)).read()
        body = src[src.index("pub const MNEMONICS"):]
        body = body[:body.index("];")]
        for m in re.findall(r'"([^"]+)"', body):
            out[m] = g
    return out


def write_md(rep, path, names):
    short = {n: f"I{k}" for k, n in enumerate(names)}
    grp = group_of_mnemonics()
    for m, d in rep["mnemonics"].items():
        d["group"] = grp.get(m.upper(), "-")
    L = ["# TriCore instruction census of the firmware images", "",
         "Generated by `crates/tricore-core/tools/insn_census.py` (rizin 0.9.1 / capstone decoder, formats from",
         "the TC1.6 Vol.2 encoding boxes via `parse_manual.py`). Raw data: `insn_census.json`. Regenerate:",
         "`python3 crates/tricore-core/tools/parse_manual.py <pdftotext -layout of tcarch_vol2_a4.pdf> isa_tc16_encodings.json`",
         "then `python3 crates/tricore-core/tools/insn_census.py docs/tricore_core/isa_tc16_encodings.json",
         "docs/tricore_core/insn_census.json docs/tricore_core/insn_census.md` (about 3 min; scratch in `build/census/`).", "",
         "* **tier1** - reachable by recursive descent from the reset vector (direct branches/calls, constants",
         "  reaching CALLI/JI/JLI, BIV/BTV vector tables). High confidence, but misses code reached only through",
         "  function-pointer tables.",
         "* **tier2** - tier1 plus runs seeded from every data word pointing into a code region whose run decodes",
         "  without an invalid encoding up to a block end. This is the census used for prioritising work.",
         "* **linear** - linear sweep of all code regions (upper bound; includes data decoded as code).",
         "* PCP code (flash->CMEM copy records, `0xF006xxxx`) is excluded. RAM code (PSPR) is rebuilt from the",
         "  firmware's flash->RAM copy tables and analysed at its execution address.", "",
         "## Images", "", "| id | image | tier1 insns | tier2 insns | tier2 mnemonics | linear insns | PCP excluded | RAM code |",
         "|---|---|---|---|---|---|---|---|"]
    for s in rep["images"]:
        L.append(f"| {short[s['name']]} | {s['name']} | {s['tier1']['instructions']} | {s['tier2']['instructions']} | "
                 f"{s['tier2']['mnemonics']} | {s['linear']['instructions']} | {', '.join(s['pcp_excluded']) or '-'} | "
                 f"{', '.join(s['ram_code_regions']) or '-'} |")
    L += ["", "## Encoding formats (tier2, static occurrences)", "", "| format | " + " | ".join(short[n] for n in names) + " |",
          "|---|" + "---|" * len(names)]
    for f, c in sorted(rep["formats"].items(), key=lambda kv: -sum(kv[1].values())):
        L.append(f"| {f} | " + " | ".join(str(c.get(n, 0)) for n in names) + " |")
    L += ["", "## Mnemonics (tier2, sorted by total static occurrences)", "",
          "`formats` = format:bits as resolved from (mnemonic, op1) against the TC1.6 manual; `?` = no manual match",
          "(capstone mnemonic differs from the manual or the op1 is not documented for that mnemonic).", "",
          "| mnemonic | group | total | " + " | ".join(short[n] for n in names) + " | tier1 total | formats |",
          "|---|---|---|" + "---|" * len(names) + "---|---|"]
    rows = sorted(rep["mnemonics"].items(), key=lambda kv: -sum(kv[1]["tier2"].values()))
    for m, d in rows:
        tot = sum(d["tier2"].values())
        if not tot:
            continue
        L.append(f"| {m} | {d['group']} | {tot} | " + " | ".join(str(d['tier2'].get(n, 0)) for n in names) +
                 f" | {sum(d['tier1'].values())} | {', '.join(f'{k} ({v})' for k, v in sorted(d['formats'].items(), key=lambda kv: -kv[1]))} |")
    gsum = defaultdict(Counter)
    for m, d in rep["mnemonics"].items():
        for n, c in d["tier2"].items():
            gsum[d["group"]][n] += c
    L[L.index("## Encoding formats (tier2, static occurrences)"):L.index("## Encoding formats (tier2, static occurrences)")] = [
        "## Instruction groups (tier2 static occurrences per image)", "",
        "Group partition (every manual mnemonic in exactly one group; source: `pub const MNEMONICS` of",
        "`crates/tricore-core/src/groups/*.rs`): A `alu.rs`, B `bitmul.rs`, C `ldst.rs`, D `ctrl.rs`, E `fpu_div.rs`;",
        "`-` = capstone mnemonic that is not in the TC1.6/TC1.3.1 manuals (see below).", "",
        "| group | " + " | ".join(short[n] for n in names) + " |", "|---|" + "---|" * len(names)] + [
        f"| {g} | " + " | ".join(str(gsum[g].get(n, 0)) for n in names) + " |" for g in sorted(gsum)] + [""]
    rep["group_totals"] = {g: dict(c) for g, c in gsum.items()}
    nonman = sorted(m for m, d in rep["mnemonics"].items() if d["group"] == "-" and d["tier2"])
    L += ["", "## Tier-2 mnemonics that are not in the manuals", "",
          "Decoded by capstone in tier-2 runs but absent from the TC1.6 V1.0 / TC1.3.1 instruction lists (TC1.6.1+",
          "encodings or capstone naming); with single-digit counts these are almost certainly data decoded as code. On",
          "the real core they are IOPC/UOPC traps.", "",
          ", ".join(f"`{m}` ({sum(rep['mnemonics'][m]['tier2'].values())})" for m in nonman) or "none", ""]
    lin = sorted((m for m, d in rep["mnemonics"].items() if not d["tier2"] and d["linear"]),
                 key=lambda m: -sum(rep["mnemonics"][m]["linear"].values()))
    L += ["", "## Seen only in the linear sweep (most likely data decoded as code)", "",
          ", ".join(f"`{m}` ({sum(rep['mnemonics'][m]['linear'].values())})" for m in lin), ""]
    open(path, "w").write("\n".join(L) + "\n")


if __name__ == "__main__":
    if os.environ.get("CENSUS_MD_ONLY"):  # regenerate the markdown from an existing JSON
        rep = json.load(open(sys.argv[2]))
        write_md(rep, sys.argv[3], [i["name"] for i in rep["images"]])
        json.dump(rep, open(sys.argv[2], "w"), indent=1, sort_keys=True)
    else:
        main(*sys.argv[1:4])
