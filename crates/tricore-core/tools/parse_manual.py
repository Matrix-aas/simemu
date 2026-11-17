#!/usr/bin/env python3
"""Extract the instruction encoding table from an Infineon TriCore instruction-set manual.

Input: `pdftotext -layout` output of the manual (Volume 2). Every instruction page carries one
encoding box per variant:

    ABSD[c], D[b] (RR)                      <- syntax line, format in parentheses
    31  28 27  20 19 18 17 16 15 12 11 8 7 0 <- bit-position header
      c    1CH    -  -  b  -  0BH            <- field line: fixed fields as hex (..H) / binary (..B)

The last hex token of the field line is op1; the remaining fixed tokens are op2/fixed fields in
high-to-low bit order. The page label is taken from the page footer that follows the entry.

Usage: parse_manual.py <manual.txt> <out.json>
"""
import json
from collections import Counter
import re
import sys

FORMATS = (
    "ABS ABSB B BIT BO BOL BRC BRN BRR RC RCPW RCR RCRR RCRW RLC RR RR1 RR2 RRPW RRR RRR1 RRR2 "
    "RRRR RRRW SYS SB SBC SBR SBRN SC SLR SLRO SR SRC SRO SRR SRRS SSR SSRO"
).split()
SYNTAX = re.compile(r"^\s*(\S.*?)\s*\((" + "|".join(FORMATS) + r")\)\s*(\(.*\))?\s*$")
FOOTER = re.compile(r"User.{0,3}Manual.*?\s(\d+ ?- ?\d+)\s")
HEADER = re.compile(r"^(31|15)\s+\d")
SHORTNAME = re.compile(r"\b([A-Z][A-Z0-9]*(?:\.[A-Z0-9]+)*)(?:\s+\(16-bit\))?\s+(\d+)\b")
# TC1.3.1 layout: `ABS . . . . . 3 - 22`
SHORTNAME_DOTS = re.compile(r"^([A-Z][A-Z0-9]*(?:\.[A-Z0-9]+)*)(?:\s+\(16-bit\))?\s*(?:\.\s*)+(\d+) - \d+")
MNEM = re.compile(r"^[A-Z][A-Z0-9]*(?:\.[A-Z0-9]+)*")
FIXED = re.compile(r"\b([0-9A-F]+H|[01]+B)\b")


def known_mnemonics(lines):
    """Mnemonics listed in 'List of Instructions by Shortname' (two-column index)."""
    start = next(i for i, l in enumerate(lines) if l.strip().startswith("List of Instructions by Shortname")
                 and i > len(lines) // 2)
    out = set()
    for l in lines[start + 1:]:
        if l.strip().startswith("List of Instructions by Longname"):
            break
        m = SHORTNAME_DOTS.match(l.strip())
        for mn, _ in [m.groups()] if m else SHORTNAME.findall(l):
            out.add(mn)
    return out


def split_mnemonic(text, known):
    """`ABSD[c], D[b]` -> (`ABS`, `D[c], D[b]`): longest known mnemonic prefix."""
    m = MNEM.match(text)
    cand = m.group(0) if m else text.split()[0]
    for n in range(len(cand), 0, -1):
        # `JNED[a]` is JNE D[a], `MOV.AA[a]` is MOV.A A[a]: an operand never starts with '['
        if cand[:n] in known and not text[n:].startswith("["):
            return cand[:n], text[n:].strip()
    return cand, text[len(cand):].strip()


def main(src, dst):
    # split on newlines only: pdftotext emits form feeds between pages
    lines = open(src, encoding="utf-8", errors="replace").read().replace("\f", "").split("\n")
    # TC1.3.1 pages carry a wide left margin: remove the dominant indentation
    margin = Counter(len(l) - len(l.lstrip(" ")) for l in lines if l.strip()).most_common(1)[0][0]
    lines = [l[margin:] if l[:margin].isspace() else l.lstrip(" ") for l in lines]
    known = known_mnemonics(lines)
    footers = [(i, m.group(1)) for i, l in enumerate(lines) if (m := FOOTER.search(l))]
    entries = []
    for i, l in enumerate(lines):
        m = SYNTAX.match(l)
        if not m or l.startswith(" " * 20):
            continue
        mnem, ops = split_mnemonic(m.group(1), known)
        if mnem not in known:
            continue
        # field line: first non-empty line after the bit header
        j, seen_header, field = i + 1, False, None
        while j < min(i + 16, len(lines)):
            s = lines[j].strip()
            j += 1
            if not seen_header:
                seen_header = bool(HEADER.match(s))
            elif any(t.endswith("H") for t in FIXED.findall(s)):
                field = s
                break
        fixed = FIXED.findall(field or "")
        hexes = [t for t in fixed if t.endswith("H")]
        page = next((p.replace(" ", "") for k, p in footers if k > i), None)
        entries.append({
            "mnemonic": mnem,
            "operands": ops,
            "format": m.group(2),
            "mode": (m.group(3) or "").strip("()") or None,
            "size": 2 if m.group(2).startswith("S") and m.group(2) != "SYS" else 4,
            "op1": int(hexes[-1][:-1], 16) if hexes else None,
            "fixed": fixed[:-1] if hexes else fixed,
            "page": page,
            "line": i + 1,
        })
    json.dump({"source": src, "mnemonics": sorted(known), "encodings": entries}, open(dst, "w"), indent=1)
    missing = sorted(known - {e["mnemonic"] for e in entries})
    print(f"{len(entries)} encodings, {len(known)} mnemonics; without encoding box: {missing}")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
