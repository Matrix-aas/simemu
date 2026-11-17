#!/usr/bin/env python3
"""
Fast indexer & reader for Continental SIMOS18 Funktionsrahmen (27,194 pages, 220 MB).
Never loads the entire document text into memory at once; uses PyMuPDF (MuPDF C engine)
for O(1) page access and TOC searching.

Usage:
  python scripts/fr_tool.py chapters
  python scripts/fr_tool.py toc <query>
  python scripts/fr_tool.py entry <exact name>   (TOC entry + its def/use children)
  python scripts/fr_tool.py read <start_page> [end_page]
"""
import argparse
import os
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
def default_pdf() -> pathlib.Path:
    if env := os.environ.get("SIMEMU_FR_PDF", "").strip():
        return pathlib.Path(env)
    for rel in (
        pathlib.Path("vault/docs/continental/Simos18 Funktionsrahmen.pdf"),
        pathlib.Path("vault/docs/Simos18 Funktionsrahmen.pdf"),
    ):
        cand = ROOT / rel
        if cand.is_file():
            return cand
    return ROOT / "vault" / "docs" / "continental" / "Simos18 Funktionsrahmen.pdf"


DEFAULT_PDF = default_pdf()


def open_pdf(pdf_path: pathlib.Path):
    try:
        import pymupdf
    except ImportError:
        print("error: PyMuPDF is not installed (run: pip install pymupdf)", file=sys.stderr)
        sys.exit(1)
    if not pdf_path.is_file():
        print(
            f"error: Funktionsrahmen PDF not found at {pdf_path} "
            "(pass --pdf <path> or set $SIMEMU_FR_PDF)",
            file=sys.stderr,
        )
        sys.exit(1)
    return pymupdf.open(pdf_path)


def cmd_chapters(doc):
    toc = doc.get_toc()
    for lvl, title, page in toc:
        if lvl == 1:
            print(f"[p.{page:5d}] {title}")


def cmd_toc(doc, query: str):
    q = query.lower()
    toc = doc.get_toc()
    hits = 0
    for lvl, title, page in toc:
        if q in title.lower():
            indent = "  " * (lvl - 1)
            print(f"[p.{page:5d}] {indent}{title}")
            hits += 1
            if hits >= 120:
                print(f"... (truncated at {hits} matches)")
                break


def cmd_entry(doc, name: str):
    """Exact TOC entry (e.g. a variable) with its child entries (def/use chapters and pages)."""
    toc = doc.get_toc()
    for i, (lvl, title, page) in enumerate(toc):
        if title.strip() == name:
            print(f"[p.{page:5d}] {title}")
            for clvl, ctitle, cpage in toc[i + 1:]:
                if clvl <= lvl:
                    break
                print(f"[p.{cpage:5d}] {'  ' * (clvl - lvl)}{ctitle}")


def cmd_read(doc, start_page: int, end_page: int):
    if len(doc) == 0:
        return
    start_idx = max(0, min(len(doc) - 1, start_page - 1))
    end_idx = max(start_idx, min(len(doc) - 1, end_page - 1))
    for idx in range(start_idx, end_idx + 1):
        print(f"==================== PAGE {idx + 1} ====================")
        print(doc[idx].get_text("text"))


def main() -> int:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pdf", type=pathlib.Path, default=DEFAULT_PDF, help="path to Funktionsrahmen PDF (or $SIMEMU_FR_PDF)")
    sub = ap.add_subparsers(dest="command")
    sub.add_parser("chapters", help="list top-level chapters")
    p_toc = sub.add_parser("toc", help="case-insensitive substring search in TOC")
    p_toc.add_argument("query", nargs="+")
    p_ent = sub.add_parser("entry", help="exact TOC entry + child def/use pages")
    p_ent.add_argument("name", nargs="+")
    p_rd = sub.add_parser("read", help="print text of page range (1-based inclusive)")
    p_rd.add_argument("start_page", type=int)
    p_rd.add_argument("end_page", type=int, nargs="?")

    a = ap.parse_args()
    if not a.command:
        ap.print_help()
        return 0

    doc = open_pdf(a.pdf)
    if a.command == "chapters":
        cmd_chapters(doc)
    elif a.command == "toc":
        cmd_toc(doc, " ".join(a.query))
    elif a.command == "entry":
        cmd_entry(doc, " ".join(a.name))
    elif a.command == "read":
        cmd_read(doc, a.start_page, a.end_page if a.end_page is not None else a.start_page)
    return 0


if __name__ == "__main__":
    sys.exit(main())
