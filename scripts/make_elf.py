#!/usr/bin/env python3
"""
Builds standard ELF32 (EM_TRICORE = 44) images from the unmodified SIMOS18.1
Flash and EEPROM/DFlash binary dumps so that Ghidra, REA, Rizin, Cutter, and
QEMU can load them with exact memory segment mappings without modifying a
single byte of the firmware.
"""
import argparse
import os
import pathlib
import struct
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
BUILD_DIR = ROOT / "build"


def find_by_sizes(subdirs: tuple[str, ...], sizes: tuple[int, ...], exts: tuple[str, ...] = (".bin", ".eep")) -> pathlib.Path | None:
    bases = []
    if env_dir := os.environ.get("SIMEMU_IMAGE_DIR", "").strip():
        bases.extend(pathlib.Path(p) for p in env_dir.split(os.pathsep) if p)
    bases.append(ROOT)
    for b in bases:
        for sub in subdirs:
            d = b / sub if sub else b
            if not d.is_dir():
                continue
            for cand in sorted(d.iterdir()):
                if cand.is_file() and cand.suffix.lower() in exts:
                    try:
                        if cand.stat().st_size in sizes:
                            return cand
                    except OSError:
                        continue
    return None


FLASH_PATH = (
    find_by_sizes(("vault/firmwares/ecu/bench", "vault/firmwares/ecu/stock", "firmwares/ecu/bench", ""), (0x400000,), (".bin",))
    or (ROOT / "vault" / "firmwares" / "ecu" / "bench" / "simos18_bench_flash.bin")
)
EEPROM_PATH = (
    find_by_sizes(("vault/firmwares/ecu/bench", "firmwares/ecu/bench", ""), (0x40000, 0x18000), (".bin", ".eep"))
    or (ROOT / "vault" / "firmwares" / "ecu" / "bench" / "simos18_bench_dflash.bin")
)


def build_tricore_elf32(
    segments: list[tuple[int, bytes, int, str]],
    entry_point: int = 0x80000000,
) -> bytes:
    """
    Create a little-endian ELF32 executable for Infineon TriCore (EM_TRICORE = 44).
    segments: list of (vaddr, data_bytes, flags, section_name)
      flags: PF_X=1, PF_W=2, PF_R=4
    """
    EI_MAG = b"\x7fELF"
    ELFCLASS32 = 1
    ELFDATA2LSB = 1
    EV_CURRENT = 1
    ET_EXEC = 2
    EM_TRICORE = 44

    ehdr_size = 52
    phdr_size = 32
    shdr_size = 40
    num_phdrs = len(segments)

    # Section 0: null, Section 1..N: segments, Section N+1: .shstrtab
    shstrtab = bytearray(b"\x00")
    sec_name_offsets = []
    for _, _, _, name in segments:
        sec_name_offsets.append(len(shstrtab))
        shstrtab.extend(name.encode("ascii") + b"\x00")
    shstrtab_name_off = len(shstrtab)
    shstrtab.extend(b".shstrtab\x00")

    data_offset = (ehdr_size + num_phdrs * phdr_size + 0xFFF) & ~0xFFF
    phdrs = bytearray()
    shdrs = bytearray(shdr_size)  # NULL section header
    payload = bytearray()

    cur_offset = data_offset
    for idx, (vaddr, data, flags, _name) in enumerate(segments):
        filesz = len(data)
        memsz = filesz
        PT_LOAD = 1
        phdr = struct.pack(
            "<IIIIIIII",
            PT_LOAD,
            cur_offset,
            vaddr,
            vaddr,
            filesz,
            memsz,
            flags,
            0x1000,
        )
        phdrs.extend(phdr)

        SHT_PROGBITS = 1
        SHF_ALLOC = 0x2
        SHF_EXECINSTR = 0x4
        SHF_WRITE = 0x1
        sh_flags = SHF_ALLOC
        if flags & 1:
            sh_flags |= SHF_EXECINSTR
        if flags & 2:
            sh_flags |= SHF_WRITE

        shdr = struct.pack(
            "<IIIIIIIIII",
            sec_name_offsets[idx],
            SHT_PROGBITS,
            sh_flags,
            vaddr,
            cur_offset,
            filesz,
            0,
            0,
            4,
            0,
        )
        shdrs.extend(shdr)

        pad = cur_offset - (data_offset + len(payload))
        if pad > 0:
            payload.extend(b"\x00" * pad)
        payload.extend(data)
        cur_offset = (data_offset + len(payload) + 0xFFF) & ~0xFFF

    shstrtab_offset = data_offset + len(payload)
    payload.extend(shstrtab)
    while (data_offset + len(payload)) % 4 != 0:
        payload.append(0)

    shoff = data_offset + len(payload)
    SHT_STRTAB = 3
    shstr_shdr = struct.pack(
        "<IIIIIIIIII",
        shstrtab_name_off,
        SHT_STRTAB,
        0,
        0,
        shstrtab_offset,
        len(shstrtab),
        0,
        0,
        1,
        0,
    )
    shdrs.extend(shstr_shdr)

    num_shdrs = len(segments) + 2
    shstrndx = num_shdrs - 1

    e_ident = struct.pack(
        "<4sBBBBB7s",
        EI_MAG,
        ELFCLASS32,
        ELFDATA2LSB,
        EV_CURRENT,
        0,
        0,
        b"\x00" * 7,
    )
    ehdr = struct.pack(
        "<16sHHIIIIIHHHHHH",
        e_ident,
        ET_EXEC,
        EM_TRICORE,
        EV_CURRENT,
        entry_point,
        ehdr_size,
        shoff,
        0,  # e_flags
        ehdr_size,
        phdr_size,
        num_phdrs,
        shdr_size,
        num_shdrs,
        shstrndx,
    )

    out = bytearray(ehdr)
    out.extend(phdrs)
    out.extend(b"\x00" * (data_offset - len(out)))
    out.extend(payload)
    out.extend(shdrs)
    return bytes(out)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--flash", type=pathlib.Path, default=FLASH_PATH, help="4 MB SIMOS18.1 PFlash binary")
    ap.add_argument("--eeprom", type=pathlib.Path, default=EEPROM_PATH, help="SIMOS18.1 DFlash/EEPROM binary")
    ap.add_argument("--out-dir", type=pathlib.Path, default=BUILD_DIR, help="output directory for generated ELF files")
    a = ap.parse_args()

    if not a.flash.is_file():
        print(
            f"error: PFlash binary not found at {a.flash} (pass --flash <path> or set $SIMEMU_IMAGE_DIR)",
            file=sys.stderr,
        )
        return 1
    if not a.eeprom.is_file():
        print(
            f"error: EEPROM/DFlash binary not found at {a.eeprom} (pass --eeprom <path> or set $SIMEMU_IMAGE_DIR)",
            file=sys.stderr,
        )
        return 1

    flash = a.flash.read_bytes()
    eeprom = a.eeprom.read_bytes()
    if len(flash) < 0x300000:
        print(f"error: {a.flash} is too small ({len(flash)} bytes; expected >= 3 MB PFlash)", file=sys.stderr)
        return 1
    a.out_dir.mkdir(parents=True, exist_ok=True)

    PF_R = 4
    PF_W = 2
    PF_X = 1

    # 1. SBOOT-only ELF (first 64KB at 0x80000000) for fast Ghidra/REA analysis
    sboot_elf = build_tricore_elf32(
        segments=[
            (0x80000000, flash[:0x10000], PF_R | PF_X, ".sboot"),
        ],
        entry_point=0x800073AC,
    )
    sboot_path = a.out_dir / "simos18_sboot.elf"
    sboot_path.write_bytes(sboot_elf)
    print(f"Wrote {sboot_path} ({len(sboot_elf)} bytes)")

    # Parse ASW C runtime __copy_table at 0x8012028C to populate PSPR (0xC0000000) and DSPR (0xD0000000)
    pspr = bytearray(0x10000)
    dspr = bytearray(0x20000)
    tbl_off = 0x12028C
    while tbl_off + 16 <= len(flash):
        action, dst, src, length = struct.unpack_from("<IIII", flash, tbl_off)
        if action == 0:
            break
        if action == 1 and 0x80000000 <= src < 0x80200000 and src - 0x80000000 + length <= len(flash):
            src_off = src - 0x80000000
            if 0xC0000000 <= dst < 0xC0010000 and dst - 0xC0000000 + length <= len(pspr):
                rel = dst - 0xC0000000
                pspr[rel : rel + length] = flash[src_off : src_off + length]
            elif 0xD0000000 <= dst < 0xD0020000 and dst - 0xD0000000 + length <= len(dspr):
                rel = dst - 0xD0000000
                dspr[rel : rel + length] = flash[src_off : src_off + length]
        tbl_off += 16

    # 2. Full SIMOS18.1 ELF (Cached PFlash0 0x80000000 + PFlash1 0x80800000 + DFlash/EEPROM 0xAF000000 + PSPR/DSPR)
    full_elf = build_tricore_elf32(
        segments=[
            (0x80000000, flash[:0x010000], PF_R | PF_X, ".sboot"),
            (0x80010000, flash[0x010000:0x03C000], PF_R | PF_X, ".cboot"),
            (0x8003C000, flash[0x03C000:0x200000], PF_R | PF_X, ".asw_pflash0"),
            (0x80800000, flash[0x200000:0x280000], PF_R, ".cal_pflash1"),
            (0xA0800000, flash[0x200000:0x280000], PF_R, ".cal_uncached"),
            (0x80880000, flash[0x280000:0x300000], PF_R | PF_X, ".asw_pflash1"),
            (0xAF000000, eeprom, PF_R | PF_W, ".dflash_eeprom"),
            (0xC0000000, bytes(pspr), PF_R | PF_W | PF_X, ".pspr_rtos"),
            (0xD0000000, bytes(dspr), PF_R | PF_W, ".dspr_ram"),
        ],
        entry_point=0x80000000,
    )
    full_path = a.out_dir / "simos18_full.elf"
    full_path.write_bytes(full_elf)
    print(f"Wrote {full_path} ({len(full_elf)} bytes)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
