# SIMOS18.1 (TC1791) — GPTA / PCP crank-cam acquisition and actuator outputs: RE design document

Firmware: `8V0906264K__0003.bin` / donor `8V0906259J` (cal `SC8S50`). Goal: honest GPTA/PCP/IO emulation so the
unmodified firmware synchronises to a 60-2 crank + 2 cam Hall sensors and its spark / injection / PWM commands can be read at register level.

## 0. Evidence conventions

| Tag | Meaning |
|---|---|
| **VERIFIED-S** | Static: read in the flash image with rizin (`rizin -a tricore`, PFlash1 mapped at `0x80800000`). Address given. |
| **VERIFIED-T** | Trace: the value was written by the unmodified firmware during a 30 M-instruction boot in the existing Unicorn platform (scratch copy of `simemu-tc1791` with one `eprintln!` added to `hook_mmio_f000`, see Appendix A). PC of the store given. |
| **VERIFIED-FR** | Funktionsrahmen page (fr_tool PDF page index). Note: FR header is baseline `SC8L50Y0`, ours is `SC8S50`; NC_* constants may differ. |
| **VERIFIED-DS** | TC1791 data sheet V1.1 (`vault/docs/infineon/infineon-tc1791-ds-en.pdf`, page = PDF page). |
| **HYP** | Hypothesis / recollection of the TC1796/TC1797 GPTA v5 user manual (not available in the repo). Every bit-field decode of GPTA/PCP registers is HYP unless stated. |

GPTA kernel bases (HYP from TC1796/97 UM, but every handler/driver below is self-consistent with it): GPTA0 `0xF0001800`, GPTA1 `0xF0002000`, LTCA2 `0xF0002800`.
Register offsets used throughout (HYP names, offsets confirmed by consistent use in firmware):

| Off | Name | Off | Name |
|---|---|---|---|
| 0x000 | CLC | 0x0C0..0D4 | PLLCTR, PLLMTI, PLLCNT, PLLSTP, PLLREV, PLLDTR |
| 0x00C | FDR (fractional divider) | 0x0D8 | CKBCTR (clock bus) |
| 0x010+8g / 0x014+8g | SRSCg / SRSSg (g=0..3; W1C/W1S request flags) | 0x0E0/0E4/0E8 | GTCTR0 / GTREV0 / GTTIM0 (GT0) |
| 0x038 / 0x03C / 0x040 | MRACTL / MRADIN / MRADOUT (mux register array) | 0x0F0/0F4/0F8 | GTCTR1 / GTREV1 / GTTIM1 (GT1) |
| 0x044 | FPCSTAT | 0x100+8n / 0x104+8n | GTCCTRn / GTCXRn, n=0..31 |
| 0x048+8k / 0x04C+8k | FPCCTRk / FPCTIMk, k=0..5 | 0x200+8n / 0x204+8n | LTCCTRn / LTCXRn, n=0..63 |
| 0x078 | PDLCTR | 0x400 (GPTA0 only) | "EDCTR" kernel/timebase run control (HYP name) |
| 0x080+16d | DCMCTRd / DCMTIMd(+4) / DCMCAVd(+8) / DCMCOVd(+C) | 0x7FC−4·m | SRCm (service request node m) |

**Unified cell index used by every Continental driver (VERIFIED-S, e.g. `0x801124F4`, `0x800A2606`, `0x80111212`):**
`addr = 0xF0001900 + 8·cell`, `cell = 256·kernel + (n for GTCn | 32+n for LTCn)`, kernel 0=GPTA0, 1=GPTA1, 2=LTCA2.
In this document a cell is written e.g. `G0.GTC12`, `G1.LTC24`, `L2.LTC32`.

---

## 1. Key findings (summary)

1. **The crank, cam, ignition and injection hardware events are serviced by the PCP (Peripheral Control Processor), not by the TriCore.**
   23 GPTA/LTCA2 service requests are programmed with TOS=PCP (`SRC = 0x4400 | channel`, VERIFIED-S `0x800DDEF4`, VERIFIED-T `0x800DDF22`).
   PCP channel code (~12.4 KB) is copied from flash `0x8012DE00..0x80130F1E` to CMEM, PCP data structures live in PRAM `0xF0050000..`.
   **An honest emulator must run a PCP2 interpreter** — Unicorn has no PCP. Without it, no tooth will ever be processed whatever the GPTA model does.
2. Clocks: fGPTA = **90 MHz** (HYP, strongly supported), GT0 time base = **5.625 MHz** (firmware constant), LTC PWM timers 5.625 MHz (inferred from exact periods), STM = **30 MHz** (1 ms OS tick = 30 000 STM ticks).
   The current emulator assumption "STM 100 MHz / CPU 240 MHz" is not what this firmware programs (§2).
3. Crank chain: **G0.FPC0 → G0.DCM0 → G0.PLL (256 microticks/tooth) → G0.GT1 (24-bit angle base, mirrored on G1.GT1)**;
   DCM0 SRN → PCP ch 0x3F, PLL SRN → PCP ch 0x3E; angle events to the OS via **G0.GTC30/31 → CPU ISR `0x800893DC`**.
4. Cams: **G0.FPC3 → G0.GTC28** (cam index 0) and **G0.FPC5 → G0.GTC26** (cam index 1), both edges, PCP ch 0x3A / 0x3B.
5. Ignition: **G0.GTC12..15** (4 coils), PCP ch 0x2C/0x2D, PRAM blocks `0xF0052500/2600/2700/2800` (VERIFIED-S config table `0x8004F930`).
6. Angle-scheduled outputs: **G1.GTC08..11**, **G1.GTC12..15**, **G0.GTC24**, **G1.GTC24**, **G1.GTC26** (config `0x800418CC`). The injection assignment guessed here is refuted by §12.3: G1.GTC08..15 = valve-lift (AVS) actuators, G1.GTC26 = HPFP valve; the injectors are G0.GTC04/17/16/05.
7. 22 fixed-frequency PWM channels on LTC quads (1 kHz, 1.6 kHz, 16 kHz, 160 Hz, 300 Hz, 625 Hz, 800 Hz, 975 Hz — §6.4).
8. Existing emulator bug: `sensors_io.rs` writes "tooth period" values to `0xF0001840/44/48` = **MRADOUT / FPCSTAT / FPCCTR0** (FPCCTR0 is firmware-configured to `0x00930046`). Must be removed.

---

## 2. Clocks and time bases

| Item | Value | Evidence |
|---|---|---|
| SCU PLL | PLLCON0 final `0x01048E20` (NDIV field 0x47 → ×72, PDIV 1 → /2), PLLCON1 K2DIV ramp 0x23→0x0B→0x04→**0x03** (/4) | VERIFIED-T `0x800056B8..0x80005748`. Decode HYP; with fOSC=20 MHz (HYP) → fVCO 720 MHz, fPLL **180 MHz** |
| fFPI = fGPTA | **90 MHz** | HYP from CCUCON0/1 = `0x00010001` (VERIFIED-T `0x8000560E/0x80005632`) + the three consistency checks below |
| STM | STM_CLC = `0x314` (RMC=3) → fSTM = 30 MHz; RTA-OS reprograms STM_CMP0 in steps of exactly **30 000** → 1 ms tick | VERIFIED-T `0x80005524`, STM_CMP0 deltas |
| GPTA0 FDR | `0x43FF` (DM=1, STEP=0x3FF → ÷1) | VERIFIED-T `0x801087C8`; decode HYP |
| GT0 rate | **5 625 000 Hz** — firmware constants `0x0055D4A8` (=5 624 999) and `0x0015752A` (=1 406 250 = 5.625 MHz/4) | VERIFIED-S `0x8004F9B0` |
| FR time unit → GT0 | HAL converts FR 200 ns units (5 MHz) ×90/80, µs ×90/16; `0x989680` = 10 000 000 used as "infinite" | VERIFIED-S `0x8008EDC8`, `0x8008EDE4`, `0x8008EEB0`; FR p.7932 (200 ns unit) |
| GT1 | 24-bit angle counter, reset to `0x00FFFFFF` in **both** GPTA0 and GPTA1 at PLL (re)start | VERIFIED-T `0x8008967C`, `0x80089688` |
| Angle unit | 1/256 tooth = 0.0234375 °CRK; PLLMTI = `0x100` | VERIFIED-T `0x8008983A`; FR p.7929, 8000, 11983 |

Consequence for the emulator: GTTIM0 must advance 5.625 ticks/µs of *virtual* time; the virtual time must be the same clock the STM uses (30 MHz). FR-level timing (e.g. 50 µs filters, 10 ms crank delay) only comes out right if both are consistent.

---

## 3. GPTA register map written by the firmware (boot to RTOS, no crank signal)

Order = execution order. All values VERIFIED-T (PC = store instruction). Decodes HYP unless marked.

### 3.1 Kernel / clock setup (`0x80108630`, config `0x8005EBEC`)

| Address | Value | PC | Meaning |
|---|---|---|---|
| F0001800 | 0 | 801087C0 | GPTA0 CLC: enable |
| F000180C | 0x43FF | 801087C8 | GPTA0 FDR ÷1 → fGPTA = fFPI |
| F0001C00 | 0 → **0x70F** | 8010863A, 801086E2 | "EDCTR": HYP bit0 G0.GT0 run, bit1 G0.GT1 run, bit2 G1.GT0 run, bit3 G1.GT1 run, bits 8..10 GPTA0/GPTA1/LTCA2 enable. Crank PLL init clears bits 1+3 (`0x705`, PC 80087140) while reloading both GT1, then sets `0x70F` again (80087132) — synchronous restart of the two angle bases |
| F00018E8 / F00018F8 / F00020E8 / F00020F8 | 0 | 8010863C..48 | GTTIM0/1 of both kernels cleared |
| F00018D8 | **0x0000C846** | 8008714E | G0 CKBCTR: nibbles 6,4,8,C (from table `0x8005EBB8`: {sel,val} pairs). HYP: clock-bus dividers 2^6, 2^4, 2^8, 2^12 → 1.406 MHz, **5.625 MHz**, 351.6 kHz, 21.97 kHz |
| F00020D8 | **0x0001C846** | 8008714E | G1 CKBCTR, extra nibble4 = 1 (HYP: G1 clock line fed from G0 PLL microticks) |
| F00018E0 | 0x4E | 801086D6 | G0 GTCTR0: HYP clock-line field [7:4]=4 → 5.625 MHz (**time base**) |
| F00018F0 | 0x1E | 801086D6 | G0 GTCTR1: line 1 (HYP: PLL microticks → **angle base**) |
| F00020E0 / F00020F0 | 0x4E / 0x3E | 801086D6 | G1 GT0 time base (same 5.625 MHz); G1 GT1 on line 3 (HYP: G0 PLL microticks via clock bus) |

### 3.2 Crank front end (driver `0x800895A8..0x800898E4`, PCP ch 0x3E/0x3F)

| Address | Value(s) | PC | Meaning |
|---|---|---|---|
| F0001880 | 0 | 80089718 | DCMCTR0 reset |
| F0001810 | 7 | 80089738 | SRSC0 bits 0..2 = DCM0 rising/falling/compare flags cleared (bit layout `3·d+{0,1,2}` VERIFIED-S from shift code `0x8008971A..0x80089738` and ISR `0x8008DED4`) |
| F00019F0 / F00019F8 | 0 | 800DC8EA / 80089764 | G0.GTC30/31 reset |
| F00018C0..D4 | 0 | 8008976A..8A | PLLCTR, PLLMTI, PLLCNT, PLLSTP, PLLREV, PLLDTR cleared |
| F0001848 / F000184C | 0x00100000 / 0 | 80089798 / 8008979A | FPCCTR0 / FPCTIM0 (IPS field [21:19]=2 preset at `0x80110AFA`, byte store `0x10`) |
| F0001878 | 0 (twice) | 80089816/2A | PDLCTR: PDL not used |
| F00018C0 | **0x4** | 80089832 | PLLCTR |
| F00018C4 | **0x100** | 8008983A | PLLMTI = 256 microticks per tooth |
| F00018CC | **0xFF00** | 80089840 | PLLSTP |
| F00018D0 | **0x1090** | 80089856 | PLLREV |
| F00018C8 / F00018D4 | 0 | 8008985C/60 | PLLCNT / PLLDTR |
| F000188C | 0 | 80089864 | DCMCOV0 |
| F0001848 | **0x00930046** | 80089890 | FPCCTR0: CMP=0x46 (70), MOD [18:16]=3, IPS=2, CLK [24:23]=1 (field split HYP; CMP/IPS/CLK split supported by cam code `0x80088D84..DB2`) |
| F00019F8 | 0x420 | 800898D0 | G0.GTC31 CTR (angle-event compare on GT1, XR written by PCP — its address `0xF00019FC` is in the PCP crank pointer list) |
| F0001880 | **0x129** | 800898D6 | DCMCTR0 (both-edge capture with request; decode HYP) |
| F00019F4 / F00019F0 / F00019F4 | 0x7FFF / **0x47F** / 0x100 | 80089288/9A/9E | G0.GTC30 XR/CTR |

### 3.3 Cam front end (driver `0x80088C7C..0x80088E90`, per-cam struct `0xD000642C + 0x20·i`)

| Address | Value(s) | PC | Meaning |
|---|---|---|---|
| F0001860 (FPCCTR3) | 0x01900046 → 0x01910000 → 0x01900046 | 80088DB2/E38/E88 | cam 0 filter: CMP 70 (·0.711 µs = 49.8 µs ≈ FR NC_T_CAM_EDGE_FIL 50 µs, FR p.7917/7935), CLK=3 |
| F00019E0 / F00019E8 | 0x830 → 0x4830 → **0x4838** / 0x31 | 80088DFE/E06/E76/D1C | G0.GTC28 capture (both edges, `mode 3` written by `0x80088DB8` when d5==3), REN set last; GTC29 = 0x31 |
| F0001870 (FPCCTR5) | same sequence | 80088DB2.. | cam 1 filter |
| F00019D0 / F00019D8 | … **0x4838** / 0x31 | 80088D1C.. | G0.GTC26 capture, GTC27 = 0x31 |
| F0001864 / F0001874 | 0 | 80088CDA | FPCTIM3/5 cleared |
| F0001818 | 0x10000000 / 0x04000000 | 80088D0C | SRSC1 bit 28 / 26 = GTC28 / GTC26 flag clear |

Cam struct dump after boot (VERIFIED-T, DSPR): cam0 `{ptr 0xF0050800, filt 0x0119, mode 3, cell 0x1C=GTC28, fpc 3}`, cam1 `{ptr 0xF0050840, filt 0x0119, mode 3, cell 0x1A=GTC26, fpc 5}`.
Intake vs. exhaust: **HYP** cam0 = intake (FR p.7922: intake has sync priority; FR array order IN, EX).

### 3.4 Remaining fixed configuration (abridged; full list in Appendix B)

| Block | Writes (VERIFIED-T) | Interpretation (HYP) |
|---|---|---|
| G0.LTC60/61 | CTR 0x102 / 0x19, XR 0 (`0x80112536..4C`, driver `0x801124F4`, cfg `0xD000A0CC`) | OS hardware counter; SRC37 → ISR `0xC0002100` (RTA-OS, PSPR) |
| G1.LTC60/61 | XR rewritten each task (`0x800A5BC0/C8`) | second OS/timer service |
| Input-capture quads G0.LTC48..51, G0.LTC52..55, G1.LTC16..19, G1.LTC20..23, G1.LTC36..39, G1.LTC40..43 | `{LTCk=0x13, k+1=0x18|0x28, k+2=0x1020|0x1010, k+3=0x3831, XR(k+3)=0x55D4/0xCDF/0x2BF2/0xE484/0x55D4/0x5E69}` (`0x800DCFC4..0x800DD018`) | PWM/frequency inputs (duty+period capture); CPU ISRs `0x8008E020`, `0x8008DF38`, `0x8008DF74`, `0x8008E05C`, `0x8008DFB0`, `0x8008DFE8` read exactly LTC k+1 and k+3 |
| G1.FPC0, G1.FPC4, G1.DCM0, G1.DCM3 | FPCCTR0 `0x00181780`, FPCCTR4 `0x00101780`, FPCCTR5 `0x00280000`, DCMCTR0/3 = 4 (`0x800F8348..90`) | slow digital edge inputs; ISRs `0x8008DED4` (DCM0), `0x8008DF04` (DCM3) dispatch callbacks |
| G0.GTC00/01 | CTR 0x5036 / 0x101E (`0x800DD8FA/8CA`) | SRC06 → PCP ch 0x25; SRC06 address stored in every ignition PRAM block (likely SW-triggered PCP channel) |
| G0.GTC12..15 | CTR **0x5808** (REN) + SRSC1 bit clear (`0x80101660/7C`) | ignition cells, initial level |
| G1.GTC00..03, G1.GTC08..15, G1.GTC26, G0.GTC04/05/16/17 | CTR 0x5800 | outputs parked inactive-high (0x5800) |
| G0.GTC21/23/24, G1.GTC24 | CTR 0x5000 | outputs parked inactive-low (0x5000) (polarity byte, `0x800DC402..412`) |
| `0xF0001F00..0C` | `0x00080000, 0x00555502, 0x50251086, 0x00000555` (`0x80110B14`) | **unknown** GPTA0-space registers (offset 0x700..0x70C) — 2-bit packed fields; candidate: output/IO sharing selection. TODO |
| MRA (mux register array) | GPTA0 54 words, GPTA1 54 words, LTCA2 32 words via MRACTL=0/2, MRADIN×n, MRACTL=1 (`0x801109BC`); SBOOT/CBOOT write the same counts first (`0x80006AC6`, `0x8001F25C`) | Input/output/clock line routing (OMCR/LIMCR/GIMCR…). Raw words in Appendix B; **not decoded** (needs TC1796/97 UM "Multiplexer Register Array" ordering) |

### 3.5 Service-request node assignment — VERIFIED-S from firmware arithmetic

The generic output driver computes the SRC address of a cell (`0x800DC41C..0x800DC464`):

```
0x800dc420  extr.u d0, d1, #0, #8        ; n = cell & 0xFF
0x800dc426  jge.u  #0x1f, d0, gtc        ; n<32 -> GTC
0x800dc42a  add    d15, d0, #-0x20 ; sha d15,#-2   ; (n-32)/4
0x800dc434  jlt.u  d1, #0x220, gpta      ; LTCA2 if cell>=0x220
0x800dc438  rsub   d15, d15, #0x25       ; LTCA2: slot = 37-(k/4)
0x800dc442  rsub   d15, d15, #0xf        ; GPTA : slot = 15-(k/4)
0x800dc44e  rsub   d15, d0, #0x1f        ; GTC  : slot = 31-(n/2)
0x800dc454  lea    a4, #0xf0001f68 ; + kernel*0x800 + slot*4
```

| Source | SRN | SRC address (GPTA0 / GPTA1 / LTCA2) |
|---|---|---|
| DCMd | SRC d (0..3) | `0x…7FC − 4d` (VERIFIED-S: ISR `0x8008DED4` reads G1.DCMCTR0 for G1 SRC00, `0x8008DF04` reads DCMCTR3 for SRC03) |
| PLL | SRC04 | `0x…7EC` (VERIFIED-S: in PCP crank pointer list next to PLL regs) |
| GTCn | SRC(6 + n/2) | `0x…7E4 − 4·(n/2)` |
| LTCn (GPTA0/1) | SRC(22 + n/4) | `0x…7A4 − 4·(n/4)` |
| LTCn (LTCA2) | SRC(n/4) | `0xF0002FFC − 4·(n/4)` |

---

## 4. The PCP is part of the I/O path

| Fact | Evidence |
|---|---|
| PRAM cleared `0xF0050000..0xF0053FFF`, CMEM `0xF0060000..0xF0067FFF` filled | VERIFIED-S `0x80108726..0x8010875C` |
| PCP regs: `0xF0043F20 = 0x01000000`, `0xF0043F00 (PCP_CLC) = 0`, `0xF0043F10 (PCP_CS) = 0x21` | VERIFIED-T `0x80108768..0x8010877C` (names HYP) |
| Image copy table (dst, src, len) at `0x80125B00`: PRAM `F0050800←8012BA00 (0xA80)`, `F0051300←8012C500 (0xF0)`, `F0051400←8012C600 (0x15B8)`, `F0052A00←8012DC00 (0x200)`; CMEM `F0060100←8012DE00 (0x79C)`, `F00608A0←8012E5A0 (0x1D74)`, `F0062618←80130318 (0x7E)`, `F0062698←80130398 (0x2A6)`, `F0062940←80130640 (0x7CA)`, `F0063110←80130E10 (0x10E)` | VERIFIED-S |
| SRC→PCP routing: table `0x8004F84C` (23 × {SRC addr, u8 channel, u8 prio-group}), loop writes `SRC = 0x4400 | ch` (bit 14 CLRR, bit 10 TOS=PCP — bit names HYP) and `PRAM[ch·32+4] = group<<24` (later overwritten by driver inits) | VERIFIED-S `0x800DDEF4..0x800DDF22`, VERIFIED-T |
| Channel context = 8 words at `PRAM + 32·ch`. Word0 layout **[31:16] PC (16-bit word address, CMEM byte = 2·PC), [15:8] DPTR (PRAM 256-byte data page), [7:0] flags** | VERIFIED-T by 4 independent matches: ch3F PC 0x0450 → `F00608A0` (start of a copied code segment), DPTR 0x09 → PRAM `0x900` = crank block; ch3A/3B DPTR 0x08 → cam blocks `0x800/0x840`; ch2C/2D PC 0x14A0 → `F0062940` (segment start), DPTR 0x25/0x27 → ignition blocks `0x2500/0x2700`; ch04 DPTR 0x2A → `F0052A00` block |

PCP channels in use (channel = SRPN, group byte = HYP channel priority CPPN):

| PCP ch | Group | SRC (kernel / node) | Cells | Function | PRAM page / R7 (word0) |
|---|---|---|---|---|---|
| 0x3F | 0x3F | `F0001FFC` G0 SRC00 | DCM0 | **crank tooth edge** | 0x09 / `04500940` |
| 0x3E | 0x3F | `F0001FEC` G0 SRC04 | PLL | **crank PLL (microtick end / gap)** | 0x09 / `0AB50940` |
| 0x3A | 0x3F | `F0001FAC` G0 SRC20 | GTC28/29 | **cam 0 edge** | 0x08 / `03EC0840` |
| 0x3B | 0x3F | `F0001FB0` G0 SRC19 | GTC26/27 | **cam 1 edge** | 0x08 / `03EC0840` |
| 0x2C | 0x2D | `F0001FC8` G0 SRC13 | GTC14/15 | **ignition cyl-ch 0/1** | 0x25 / `14A02560` |
| 0x2D | 0x2D | `F0001FCC` G0 SRC12 | GTC12/13 | **ignition cyl-ch 2/3** | 0x27 / `14A02760` |
| 0x25 | 0x2D | `F0001FE4` G0 SRC06 | GTC00/01 | ignition helper (HYP) | 0x13 / `11C81360` |
| 0x1D | 0x2D | `F0001FD4` G0 SRC10 | GTC08/09 | angle output (G0.GTC08 ptr at PRAM 0xD00) | 0x0D / `10DA0D60` |
| 0x29 / 0x28 | 0x2D | `F0001FBC` SRC16 / `F0001FB8` SRC17 | GTC20/21, GTC22/23 | angle outputs (ptrs at PRAM 0xE20/0xF20) | — |
| 0x1F | 0x2D | `F0001F90` G0 SRC27 | LTC20..23 | ? | — |
| 0x32 / 0x33 | 0x2D | `F00027C0` G1 SRC15 / `F00027B8` G1 SRC17 | G1 GTC18/19, GTC22/23 | ? | — |
| 0x1E | 0x2D | `F0002778` G1 SRC33 | G1 LTC44..47 | ? | — |
| 0x06 | 0x13 | `F00027D8` G1 SRC09 | G1 GTC06/07 | angle-output block base (PRAM 0x1400) | 0x14 / `03301460` |
| 0x08..0x0B | 0x13 | `F00027D4/D0/CC/C8` G1 SRC10..13 | **G1 GTC08..15** | **injection (HYP)** | 0x15..0x17 |
| 0x0D | 0x13 | `F0001FB4` G0 SRC18 | G0 GTC24/25 | HPFP/aux angle output (HYP) | 0x17 / `02251760` |
| 0x0E / 0x0F | 0x13 | `F00027B0` / `F00027B4` G1 SRC19/18 | G1 GTC26/27, GTC24/25 | HPFP/aux angle output (HYP) | — |
| 0x04 | 0x13 | `F0002FF0` L2 SRC03 | L2 LTC12..15 | timer scheduler; PRAM 0x2A00 holds L2 SRC00..03 + G1 SRC21 addrs | 0x2A / `13242A60` |

Hardware addresses the PCP crank channel touches (pointer list in PRAM crank block, VERIFIED-T `0xF00509BC..F8`):
`FPCSTAT, FPCCTR0, DCMCTR0, DCMTIM0, DCMCOV0, SRC00, PLLCTR, PLLMTI, PLLSTP, PLLCNT, PLLREV, SRC04, SRSC0, SRSS0, GTTIM1, GTC31.XR` (+ `DCMCAV0`, `FPCCTR0` at `0xF0050A68`).
Ignition blocks hold `GTCn.CTR, SRSC1, GTTIM0, GTTIM1, SRSS1, SRC06` (VERIFIED-T `0xF0052500..0xF005281C`).
Angle-output blocks hold G1 GTC06/08..15/24/26 and G0 GTC24 CTR addresses (VERIFIED-T `0xF0051488..0xF00518B8`).

PCP ISA: not decoded here (rizin has no PCP2 support). The interpreter must be written from the TC1796/TC1797 UM "PCP2 instruction set" chapter. Entry: on a PCP-routed SRN with SRPN = ch, restore context from `PRAM+32·ch`, start at `2·PC` in CMEM, use DPTR page for `[R7-relative]` data, save context on `EXIT`.

---

## 5. Crank / cam acquisition — what must be reproduced

### 5.1 FR requirements (VERIFIED-FR, from ENSD chapter)

| Item | Value | FR page |
|---|---|---|
| Wheel | 60 teeth, 2 missing, 1 gap; ACPS, **falling edge active** | 7923, 7934 |
| Sensor | CPDD (direction): pulse width after active edge forward 15–62.2 µs, backward 62.4–150 µs (DAMOS `c_t_crk_sho_level_*` @0x80805AB0..ABC); evaluated < 3000 rpm | 7916, 7949, 7952, 7918 |
| Filters | crank and cam 50 µs; ≥51.2 µs between active edges | 7917, 7935 |
| Gap | `c_crk_gap_det` = 2.0, `c_crk_gap_vld` = 6.0 (DAMOS @0x80805A14/15); confirmed on edge #2 after the gap | 7944, 7945, 7963 |
| Position | NC_OFS_TDC0_REF_CRK = 78 °CRK (gap → TDC0); segment start 72 °CRK before each TDC | 7924, 7925 |
| Cams | 8 edges/720° each, both edges used. IN_1 edges °CRK after TDC0: 46, 80, 226, 260, 406, 542, 586, 2. EX_1: 179, 213, 359, 495, 539, 675, 719, 33 (baseline values) | 7920–7925 |
| Sync | `STATE_CRK_BAS/ACQ` 0..0xB, target **0xB SYN_VLD_GAP**; `STATE_EPO_BAS` → 3 EPO_CRK_SYN … 5 CAM_SYN; `LV_SYN_ENG`, `LV_SYN_VLD` (needed for DI injection/ignition); 10 cam edges to validate (`id_nr_vld_cam_syn_crk` @0x80805A06 = 10) | 7931, 7965, 8013, 8044, 7903 |
| HAL | `HAL_EPO_CrkTicksPerTooth`, `HAL_EPO_ToothCnt` (24-bit CTR_CRK_INT_BAS), `HAL_EPO_CrkAtomicSetEventEnable(ch, pos, next, en)`, OS triggers SEG/HALF_SEG/GAP/SYNC/CAM_* | 7997–8003 |

TriCore-side crank HAL (VERIFIED-S): config pointer `*(0xD0006478) = 0x8004F35C` → static params `0x8004F354 = {FPC 0, DCM 0, GTC 0x1E (=30/31), PCP 0x3E, PCP 0x3F}`, 12 angle-event channels `0x8004F2F4`, flag "mirror GT1 into GPTA1" (byte `+0xE` = 1).
Runtime block pointer `*(0xD000647C) = 0xF0050900` (**PCP PRAM**). Called from ENSD init `0x800EB7F4..0x800EB812` (VERIFIED-S): `0x8008EE30(c_crk_gap_det, c_crk_gap_vld)` → block `+0x54/+0x58`; `0x8008EE44(c_fac_fil_crk_tooth_act @0x80805A19, c_fac_crk_back @0x80805A18)` → `+0x5C/+0x7C`; `0x8008EDC8(0, c_t_crk_dly)` → `+0x0C` = value(200 ns units)·90/80·16.
Angle-event ISR `0x800893DC` (G0.GTC30/31, SRPN 0x39) walks the pending-channel mask `0xD0006544` and calls the OS callbacks of table `0x8004F2F4`.

### 5.2 Hardware data flow to emulate

```
physics crank angle θ(t) ─► tooth edge generator (60-2, falling active, CPDD pulse ≈30–45 µs low) ─► G0.FPC0 (filter, CMP=70)
   ─► G0.DCM0: capture GTTIM0 on rising/falling into DCMCAV0/DCMCOV0, set SRSS0 bit0/1 ─► SRC00 (TOS=PCP) ─► PCP ch 0x3F
   ─► G0.PLL : per tooth emit PLLMTI(=256) microticks spread over the predicted period (PLLSTP/PLLREV/PLLDTR maintained by PCP),
               PLL request ─► SRC04 ─► PCP ch 0x3E
   ─► microtick clock line ─► G0.GT1 += 1 per microtick, and G1.GT1 identically (same clock line, same reload)
   ─► GTC compare on GT1 (G0.GTC31.XR written by PCP) ─► SRC21 (CPU, SRPN 0x39) ─► ISR 0x800893DC ─► OS angle tasks
cam Hall level (physics cam angle + phaser) ─► G0.FPC3/FPC5 ─► G0.GTC28/GTC26 capture (both edges) ─► SRC20/SRC19 ─► PCP 0x3A/0x3B
```

Missing-tooth behaviour: the generator simply omits 2 edges; the PCP code detects the gap from DCM period ratios (`c_crk_gap_det`), there is no PDL involvement (PDLCTR=0).
Which IN pin carries CKP/CMP: **unknown** (MRA undecoded). Inject at the FPC input (post-mux) — FPC0, FPC3, FPC5 of GPTA0 — which is hardware-honest as long as the MRA routing is not modelled.

---

## 6. Outputs

### 6.1 Ignition — G0.GTC12..15 (VERIFIED-S)

Config `*(0xD0009BB0) = 0x8004F998` → channel table `0x8004F930` (16 bytes each):

| ch | cell | PCP ch | last two bytes | PRAM block |
|---|---|---|---|---|
| 0 | G0.GTC15 | 0x2C | 07 02 | 0xF0052500 |
| 1 | G0.GTC14 | 0x2C | 08 03 | 0xF0052600 |
| 2 | G0.GTC13 | 0x2D | 09 00 | 0xF0052700 |
| 3 | G0.GTC12 | 0x2D | 0A 01 | 0xF0052800 |

Path: IGRE ASW (`0x800BC1C0`, uses `c_ang_dif_ign_psn_eng` @0x808087D6) → adapter `0x8009B548/5B0/5C8/5E4` (`SetIgn*`/`GetIgnAngle`/`GetIgnDwell`/`GetIgnPsnEng`) → driver `0x800A2558..0x800A2970` → on (re)start writes `GTCn.XR = t−1`, `GTCn.CTR = (PRAM+0x7C)|0x3F`, `GTCn.XR = t` (t from `0x800A2398`; `0x800A267A..0x800A268C`, HYP: forced immediate output action) and PRAM state (`+0x54/58/64/68/8C`, state codes 8/9/0xA polled at `0x800A25B2..D8`); PCP ch 0x2C/0x2D service subsequent compare events.
Driver also references PCP code entries `0xF0062698`, `0xF00626F6`, `0xF00627F0` (`0x8004F97C..84`).
Mapping ch → cylinder: HYP the trailing byte (02,03,00,01) is the cylinder/firing slot. ASIC: ATIC140 provides coil diagnostics only (FR p.12569–12578).
**Decoding a spark:** the coil current flows while the GTC output is in the active level; the falling transition (end of dwell) is the spark (FR p.12567).
Spark angle = GT1 value at the output transition → `θ = (GT1·360/(60·256))` relative to the PLL reference, or directly the physics crank angle at the event time; dwell = ΔGT0 / 5.625 MHz.

### 6.2 Angle-scheduled outputs — injection / HPFP (driver `0x800DC188`, cfg `*(0xD00007F4) = 0x80041950`, PRAM `0xF0051400`)

Channel table `0x800418CC` (11 × 12 bytes: {sub-event list, cell, ?, polarity, ?, n_sub}) — VERIFIED-S:

| ch | Cell | Sub-events | Idle level | HYP function |
|---|---|---|---|---|
| 0–3 | G1.GTC08..11 | 4 | 1 (0x5800) | ~~injector cyl 0..3~~ AVS actuator pulse timing per logical cylinder (current on/hit/off callbacks 0x8009AEB0/0x8009AF1C/0x8009AF88); the coils themselves are ATIC140 stages 16..19 switched by MSC DD bits (docs/SIMOS18_COMBUSTION.md §6b) |
| 4–7 | G1.GTC12..15 | 3 | 1 | ~~second injector signal~~ ~~second actuator set~~ AVS recoil-diagnosis windows (callback 0x8009A138, hit + 600 … 740 °; ibid.) |
| 8 | G0.GTC24 | 6 | 0 (0x5000) | HPFP N276 / aux |
| 9 | G1.GTC26 | 17 | 1 | HPFP N276 peak/hold chopping (FR: VCV PWM 500/4000 Hz, angles in 1/256 tooth, p.11983) |
| 10 | G1.GTC24 | 2 | 0 | aux |

Injector power stage: ATIC121 (NC_PBK_IV_TYP = 121, 2 banks, FR p.12921), configured over SPI (`ACTION_INFR_SetInjAtic21Param`, FR p.12983–12995). SPI command format: **unknown**. Two SSC-like kernels are active at `0xF0310000` / `0xF0310100` (CON toggles `0x4000/0xC0xx`, data width fields 0x0F/0x17/0x1F, VERIFIED-T `0x800A8D6C/8C`) — HYP: these are the SSC channels to ATIC/MoC.

### 6.3 Physical GPTA output pins configured at boot (port IOCR decode HYP, pin functions VERIFIED-DS p.34ff)

| Pin | Alt | Line |
|---|---|---|
| P2.8 | O1 | GPTA0 OUT0 |
| P2.14 | O1 | GPTA0 OUT6 |
| P9.6 / P9.7 | O1 | GPTA0 OUT54 / OUT55 |
| P4.1 / P4.3 / P4.4 | O2 | GPTA1 OUT25 / OUT27 / OUT28 |
| P5.2 | O2 | GPTA0 OUT74 |

Only 8 GPTA pins are driven directly. HYP: the 4 GPTA0 pins (OUT0, OUT6, OUT54, OUT55) are the 4 ignition logic outputs. Most PWM loads go through the smart power stages over **MSC0/MSC1** (both configured, `0xF0000800`/`0xF0000900`, FDR `0x43FE`, `DSDSL 0x0200000D`, MSC1 command register `0x91C` written 147× during boot, VERIFIED-T). TC17xx MSC can serialise GPTA OUT lines (ALTIN) — HYP that `MSC0 +0x24 = 0xC03F3CCF`, `MSC1 +0x24 = 0xFF0F000F` select those sources.

### 6.4 Fixed-period PWM generators (driver `0x801111BC`, cfg `*(0xD0000744)`, 28 B/channel; VERIFIED-T)

Each channel uses an LTC quad: LTC k = reset timer (`0x1103/0x1903/0x5103/0x5903/0x5B03`), k+1 = period compare (`0x2031`, XR = period−2), k+2/k+3 = edge compares (`0x3011/0x3021` or `0x7011/0x7021`, `0x3811/0x7811` variants = polarity/mode), XR(k+2) = duty edge (0xFFFF = 0 %). Period in 5.625 MHz ticks (`0x15F7+2 = 5625` = exactly 1.000 ms supports the clock).

| Timer cell | Period ticks | f (Hz) | CPU SRN (dispatcher `0xC0001C78(idx)`) | Candidate (HYP) |
|---|---|---|---|---|
| G1.LTC24, G1.LTC28, L2.LTC32 | 5625 | **1000** | idx 0, 1, 3 | e-wastegate V465 (`c_frq_pwm_bpa` = 1000 Hz @0x80802C3A), ETC? |
| G0.LTC56 | 9000 | 625 | 2 | ? |
| L2.LTC40, L2.LTC56 | 5769 | 975 | 4, (none) | ? |
| L2.LTC16, L2.LTC20, G1.LTC00, G1.LTC04, G1.LTC08 | 3515 | 1600 | 5, (none), (none), 0xC, 0xE | cam phasers / oil pump N428 / thermostat N493 |
| L2.LTC24, G1.LTC12 | 351 | 16 000 | 8, 0xD | ETC H-bridge (FR p.24830: H-bridge PWM ±100 %, freq not stated) |
| G0.LTC28, G1.LTC56, G1.LTC32, G0.LTC04 | 35156 | 160 | 7, 9, 0xB, 0x13 | VCP N205/N318 (DAMOS `ip_frq_pwm_vcp` 150 Hz — mismatch, check) |
| G0.LTC12, G0.LTC40, L2.LTC36 | 7031 | 800 | 0xF, 0x10, 0x15 | ? |
| G0.LTC24, L2.LTC28 | 18750 | 300 | 0x11, 0x12 | ? |

**Correction (§12.9): the timer clock of each quad is its LIMCR clock line, not always CLK4.** Decoded LIMCR (Appendix B
MRA words; clock bus GPTA0 CKBCTR 0xC846, GPTA1 0x1C846, LTCA2 uses the GPTA1 bus, TC1796 UM p.24-256): CLK0 = fGPTA 90 MHz,
CLK4 = fGPTA/16 = 5.625 MHz, CLK6 = fGPTA/256 = 351.56 kHz, CLK7 = fGPTA/4096 = 21.97 kHz. G0.LTC4/12/16/28/40, G1.LTC0/4/8/12/32/56,
L2.LTC16/20/24/36 ← CLK6; G0.LTC8/24, G1.LTC16/20/24/28/52, L2.LTC28/32 ← CLK4; G0.LTC44/56, L2.LTC40/56 ← CLK0; G0.LTC48/52,
G1.LTC36/40 ← CLK7 (input measurement quads); G0.LTC60 ← CLK2. So e.g. G0.LTC28 / G0.LTC04 (period 35156) run at **10 Hz**, not
160 Hz (N80 purge on G0.LTC4 quad = 10 Hz, §12.8), L2.LTC16 (3515) = 100 Hz (λ heater, VERIFIED-T), L2.LTC56 (5769) = 15.6 kHz
(§12.3). The frequencies in the table above assume CLK4 for every cell and are only right for the CLK4 rows.

Duty encoding: `duty = (XR(k+3) − XR(k+2)) / period` or `XR(k+2)/period` depending on mode (HYP). Runtime duty writes seen: `0x800A88CA` (XR of k+3) e.g. G0.LTC15.XR = `0x18AC/0x1B76`, G1.LTC11.XR = `0x15E`, L2.LTC59.XR = `0x23F/0x480/0x1445`.
Actuator ↔ channel mapping must be closed by experiment (§9 test T6). Main relay, starter relays, fan: not located (likely MSC/GPIO). LPFP J538: PWM to EFP control unit (FR 43.22), channel unknown.

---

## 7. Interrupt sources

CPU vectoring: BIV = `0xC0000000` (PSPR), BTV `0xC0007B00` (VERIFIED by current emulator boot). RTA-OS Cat-2 table `0x8004F594` (58 × {SRC, descriptor, flags}); **SRPN = table index + 1**, written as `0x4000|SRPN` then enabled `0x1000|SRPN` (`0x800DDE34`, `0x8008D556`). Descriptor `{fn, arg}`; `0x800971AC` = indirect wrapper calling `ptr[0](ptr[1])`.

| SRC | Node | Cells | SRPN | Handler |
|---|---|---|---|---|
| F0001FA8 | G0 SRC21 | GTC30/31 | 0x39 | **`0x800893DC`** crank angle events |
| F0001F68 | G0 SRC37 | LTC60–63 | 0x30 | `0xC0002100` (RTA-OS counter) |
| F0001FD0 | G0 SRC11 | GTC10/11 | 0x0F | `0x8008BB2C` |
| F0001F70 / F0001F74 | G0 SRC35 / 34 | LTC52–55 / 48–51 | 0x23 / 0x27 | `0x8008DF38` / `0x8008E020` (input capture) |
| F0001F9C | G0 SRC24 | LTC08–11 | 0x29 | `0x8008E098` |
| F00027FC / F00027F0 | G1 SRC00 / 03 | DCM0 / DCM3 | 0x2E / 0x2F | `0x8008DED4` / `0x8008DF04` |
| F00027E4 / F00027E0 | G1 SRC06 / 07 | GTC00–03 | 0x0D / 0x0E | `0x8008DE6C` / `0x8008DEA0` |
| F0002794, F0002790, F0002780, F000277C | G1 SRC26/27/31/32 | LTC16–23, 36–43 | 0x24/0x28/0x25/0x26 | `0x8008DF74`, `0x8008E05C`, `0x8008DFB0`, `0x8008DFE8` |
| F00027A8 | G1 SRC21 | GTC30/31 | 0x38 | `0xC000086C(3)` |
| F0002FFC / F8 / F4 | L2 SRC00/01/02 | LTC00–11 | 0x07/0x08/0x2D | `0xC000086C(0/1/2)` |
| PWM SRNs (F000278C, F0002788, F0001F6C, F0002FDC, F0002FD4, F0002FEC, F0001F88, F0002FE4, F000276C, F0002784, F00027A0, F0002798, F000279C, F0001F98, F0001F7C, F0001F8C, F0002FE0, F0001FA0, F0002FD8) | LTC quads of §6.4 | — | 0x10..0x22 | `0xC0001C78(idx 0..0x15)` |
| F00009FC | MSC1 | — | 0x09 | `0xC0000814(1)` |
| F0003EE0..EFC | DMA ch | — | 0x2A..0x35 (EE4=2A, EE0=2B, EF0=2C, EFC=31, EF8=32, EF4=33, EEC=34, EE8=35; VERIFIED-T) | various (DMA, already used by MoC SSC path) |
| F00103E4 / F00103EC | ? | — | 0x06 / 0x05 | `0x800971B4` — HYP PCP→CPU requests |

PCP-routed SRNs: 23 entries of §4 (TOS=PCP, SRPN = PCP channel). They must **not** be delivered to the CPU.

---

## 8. RAM / PRAM observables

| Address | Content | Evidence |
|---|---|---|
| `0xD0006478` → `0x8004F35C` | crank static cfg | VERIFIED-T/S |
| `0xD000647C` → `0xF0050900` | crank runtime block (PCP page 0x09) | VERIFIED-T |
| `0xD0006544` (u16) | pending angle-event channel mask | VERIFIED-S `0x800893F6` |
| `0xD000642C`, `0xD000644C` | cam structs (→ PRAM `0xF0050800/0840`) | VERIFIED-T |
| `0xD0009BB0/B4/B8` | ignition cfg / runtime / PRAM `0xF0052B00` | VERIFIED-T |
| `0xD00007F4/F8/FC` | angle-output cfg `0x80041950` / PRAM `0xF0051400` / `0xD0006D74` | VERIFIED-T |
| `0xD0000744/48` | PWM cfg / runtime | VERIFIED-S |
| `0xD0012408` (u16) | OBD PID 0x0C engine speed buffer (DID `F40C`) | VERIFIED-S table `0x8005CE88` |
| `0xD000E569` (u8) | OBD PID 0x0E timing advance | same table |
| `0xD000F373` (u8) | OBD PID 0x11 throttle position | same table |
| `0xD000EBA5` (u8), `0xD000F3CA` (u8) | OBD PID 0x0B MAP, 0x0D vehicle speed | same table |

DAMOS `SC8S50.json` contains **no RAM variables** (calibrations only, CAL 0x80800400..0x8087CB0C). `N_32`, `STATE_CRK_ACQ`, `LV_SYN_ENG`… must be located by following the HAL getters (TODO, method: watch which DSPR bytes change state 0→0xB during the first synchronisation in the emulator).

---

## 9. Implementation plan (dependency order)

**P0 — clean-up.** Delete the writes to `0xF0001840/44/48` in `sensors_io.rs` (they clobber MRADOUT/FPCSTAT/FPCCTR0).

**P1 — virtual clock.** One `u64 vt_ps` (picoseconds) shared by STM (30 MHz), GT0 (5.625 MHz), PCP and the crank generator. Keep a `gpta_hz = 90e6` and `stm_hz = 30e6` calibration knob. Make STM follow this (today: 100 MHz).

**P2 — GPTA register model (new `crates/simemu-tc1791/src/gpta.rs`).** Trap reads/writes `0xF0001800..0xF0002FFF`:
```rust
struct Gpta { k: [Kernel; 3] }                       // GPTA0, GPTA1, LTCA2
struct Kernel { fpc: [Fpc; 6], dcm: [Dcm; 4], pll: Pll, gt: [Gt; 2],
                gtc: [Cell; 32], ltc: [Cell; 64], srss: [u32; 4], src: [u32; 54], mra: Vec<u32> }
struct Cell { ctr: u32, xr: u32, out: bool }        // ctr bits kept raw; decode lazily
```
Semantics needed first: SRSCg/SRSSg write-1-to-clear/set; SRC registers (SRE, SRR, CLRR, SETR, TOS, SRPN) with CLRR/SETR read as 0; GTTIM0/1 computed from `vt`; MRA write counter (store, no decode yet).

**P3 — interrupt router.** On SRR∧SRE: TOS=CPU → existing ICU (`irq_pending`); TOS=PCP → PCP request queue keyed by SRPN.

**P4 — PCP2 interpreter (new `pcp.rs`).** Memory: PRAM `0xF0050000+16K`, CMEM `0xF0060000+32K` (already host-backed), FPI accesses → normal MMIO path. Channel start: context `PRAM+32·ch`, PC=`word0>>16` (×2 bytes), DPTR=`(word0>>8)&0xFF`. Arbitration: highest pending SRPN > current priority. Validate on boot image: every PC of §4 lands inside a copied CMEM segment.

**P5 — crank front end.** Edge generator from physics θ: per tooth emit falling edge (active) and rising edge after `t_low` (forward 30–45 µs, CPDD). Omit 2 teeth at the gap; gap at 78 °CRK before TDC0. Per edge: FPC0 (pass-through + optional CMP delay), DCM0 capture of GTTIM0 into CAV/COV, SRSS0 bits, SRC00. PLL: on each tooth, after PCP updates PLLSTP/PLLREV, spread `PLLMTI` microticks over the predicted period; increment G0.GT1 and G1.GT1; raise SRC04 per PLL rules.

**P6 — GTC/LTC behaviour.** Capture (latch GTx on input edge), compare (when `GTx` crosses `XR`, act on OUT per CTR action field, set request), reset-timer LTC (period) — implement as *event scheduling*: when XR/CTR is written, compute the virtual time at which GT0/GT1 reaches XR (GT1 from predicted rpm, re-evaluated each tooth) and post an event; never poll per instruction.

**P7 — cam front end.** From physics cam angles (incl. phaser) generate Hall levels with the 8-edge patterns of §5.1; feed G0.FPC3 → GTC28 (cam0) and G0.FPC5 → GTC26 (cam1) captures; SRC20/SRC19.

**P8 — output decoder (read-only taps).**
`IgnitionTap`: on each OUT transition of G0.GTC12..15 record `(cyl_ch, vt, θ, GT1)`; spark = end of active level; publish `spark_deg_btdc[cyl]`, `dwell_ms[cyl]` to physics.
`InjTap`: G1.GTC08..15 active intervals → SOI angle / pulse width; `PwmTap`: per LTC quad, period = `XR(k+1)+2`, duty from `XR(k+2)/XR(k+3)` + CTR mode.

**P9 — power-stage serial links.** MSC0/1 downstream frame decoder (output bits); SSC `0xF0310000/0x100` ATIC121/140 command logger. Needed before low-side loads (relays, fan, N276 enable) can be read honestly.

**P10 — MRA decode.** Implement OMCR/LIMCR/GIMCR decoding (needs UM ordering) to replace the FPC-level injection with pin-level injection and to map OUT lines to pins.

---

## 10. Test plan

| # | Test | Pass criterion |
|---|---|---|
| T1 | Boot regression (30 M insns, no crank) | GPTA register snapshot equals Appendix B; 23 SRCs have TOS=PCP; PRAM contexts equal §4 |
| T2 | PCP smoke | Software-set SRC00 (SETR) → PCP ch 0x3F runs from CMEM `0xF00608A0`, exits, no TriCore fault; PCP writes only addresses from its PRAM pointer list |
| T3 | Crank only, 200 rpm cranking then 800 rpm, 20 revs | ch 0x3F runs once per DCM0 request (≥ 58 per rev; 116 if both edges request — DCMCTR0 decode open); `GTTIM1` advances 60·256 = 15 360 per rev; ISR `0x800893DC` called at every programmed angle event; PCP writes `PLLSTP/PLLREV` each tooth |
| T4 | Crank + both cams, 800 rpm, ≥ 4 revs | ignition driver starts writing `0xF0001960..0xF000197C` (GTC12..15 CTR/XR) — first hard proof of `LV_SYN_ENG`; OBD buffer `0xD0012408` → 800·4 = 3200 ± 1 % |
| T5 | Spark decode | `IgnitionTap` angle vs OBD PID 0x0E (`0xD000E569`, A/2 − 64 °BTDC) within 0.75 °; 4 sparks per 720° in firing order 1-3-4-2 |
| T6 | Actuator map | For each PWM quad of §6.4 sweep the corresponding DAMOS test/override or physics input and correlate duty; record final mapping in this doc |
| T7 | Negative | Remove 1 cam edge per cycle → firmware keeps crank sync but sets cam error (no `LV_SYN_VLD`); reverse-rotation pulse width (100 µs) → crank direction error |

---

## 11. Open items (all HYP or unknown)

1. PCP2 instruction set + exact PCP_CS/ICR semantics (needed for P4).
2. Bit-level semantics of FPCCTR/DCMCTR/PLLCTR/GTCCTR/LTCCTR, EDCTR and offsets `0x700..0x70C` (`0xF0001F00..0C`).
3. MRA word ordering → IN pin of CKP/CMP and OUT-line → pin map.
4. Injection vs HPFP assignment of the 11 angle-output channels; cylinder numbering of ignition channels.
5. ATIC121 / ATIC140 SPI command formats; MSC frame layout.
6. Addresses of `N_32`, `STATE_CRK_ACQ`, `LV_SYN_ENG`, `LV_SYN_VLD`.
7. fOSC = 20 MHz assumption behind the 180/90 MHz PLL decode (the 5.625 MHz and 30 MHz conclusions do not depend on it).

---

### 12.5 Lambda chain, lasting idle, sixth pass (2026-11-02)

Method: REA (rizin provider) disassembly with evidence ids, FR via `scripts/fr_tool.py`, DAMOS values read from the image,
probes / RAM peeks / DSPR diffs on the running scenario, OBD over UDS (read-only). Details of the IC: BOARD_HW doc §6.

| Item | Result | Evidence |
|---|---|---|
| ATIC142 | Protocol decoded (init, 26-word measurement frame, config download/read-back, control words), modelled in `atic142.rs`; accepted, configuration SUCCESS, no LAMB_IC errors | BOARD_HW §6; REA `ev_e9a9dfd8…`, `ev_2fd9602f…`, `ev_4b84417b…`, `ev_f2265c1e…` |
| Sensor physics | `simemu-physics/src/lambda_sensor.rs`: mass-based plug flow manifold → sensor (0.8 L), LSU-class cell (Ip = inverse of `ip_lamb_lsl_inv` with the firmware's T/p corrections, Ri(T) = inverse of `ip_ttip_mes_lsl`, matching LSU 4.9 300 Ω @ 780 °C), PTC heater + tip thermal model, catalyst O₂ storage, binary cell (`ip_ttip_mes_ls`, LSF-class EMF table UNVERIFIED). 28 ns per 1 ms plant step | unit tests `lambda_sensor::tests::*` |
| Heaters | Up = BSW PWM ch 5 = L2.LTC19 (100 Hz), down = ch 9 = G1.LTC59 (10 Hz), low = on, duty = (b+1)/256 of `0xD000C56C`/`0xD000C56B` → `EcuOutputs::lambda_heater_duty` | VERIFIED-S (`0x8019B258`, `0x8019B2CC`, c_vb_max_prot_lsh_lsl/ls) / VERIFIED-T (dew-point bytes 5/20 = decoded 2.3/8.2 %) |
| Why λ control never started | Lambda signal acquisition `0x8019AD40` is skipped while `0xD0000DA5` ≠ 0 (cleared only with FID `0x3CD`, `0x8019C3F8..`); FID `0x3CD` inhibited by RLY_MAIN_PLAUS_ON (failed: after-relay voltage `0xD001518A` = 0 < 5 V, `0x801DF674`) | VERIFIED-S/T (FID table `0x808353D6`/`0x808305B6`, status byte `0xD000D9D2` = 0xFF) |
| Board inputs added | KL87 sense ADC2.CH5 (GetVRlyMainBas `0x800A66A0`, id `0x8007`); 5 V sensor references A/B/C ADC0.CH1/2/3 at half scale (GetVccBas `0x800A65F4`, ids `0xE/0xD/0xC`). Without the references, once the relay fault stopped masking them, VCC_L_0..2 failed and the firmware dropped crank synchronisation (no start) | VERIFIED-S/T (A/B runs, ERRM status diff) |
| Engine-off timer | T_ES_H (`0xD0012490`) = PIC value at power-up, T_ES (`0xD0012488`) = /60 — the firmware takes the PIC clock as the engine-off time directly; now a bench input (`StudioSession::eot_clock_s`, scenario default 300 s warm / 12 h cold, applied before the ECU powers up together with the warm plant). It changes the dew-point and start-enrichment paths: 0 s → upstream dew point never ended in 25 s, stall 12 s; 1 h → 16 s; 47 h → 17 s (before the board-input fixes) | VERIFIED-T (DSPR diff PIC 0 vs 3600 s) |
| Scenario (warm, SC8S50, 40 s, both cores) | Engine **runs on its own 37 s** (self-sustained from 3 s), 30–40 s: 998–1348 rpm (native) / 1221–1322 rpm (Unicorn). Dew point → MES (`B9E8`) → Ri range 1→4 → firmware heater loop holds the tip at 778 °C (set-point 785 °C) → pump controller on (`C` bit 10) → firmware λ (OBD 0x34) follows the plant (median ratio 0.995 / 1.006) → **closed loop**: PID 0x03 = 2 with STFT −5…−24 % from ≈ 18 s | `engine_start_scenario` (asserted) |
| Remaining blockers | (1) Mixture rich (λ 0.7–0.9) since the supplies were emulated: the rail stays at 17–23 MPa with N276 at its minimum-delivery angle (rail set-point 3–3.5 MPa) — injector characteristic above 15 MPa (`ip_ti_4_h`, plant fitted to `ip_ti_4_l` only) and pump/rail behaviour not checked; the controller hits its trim (≈ −22 %) and the status falls to 4 (open loop due to conditions, cause not decoded). (2) Idle speed ≈ 1000–1350 rpm vs 700–800 rpm target; firmware charge (absolute load 14 % ≈ 85–90 mg) vs plant 124–144 mg/stroke at idle MAP 30 kPa: plant VE at idle ≈ 84 % looks high — not changed without the matching calibration (SC8S50 is a 2.0 TSI CJX calibration on a 1.8 plant). (3) Newly unmasked faults: CONV_PLAUS, PORT_FB_EL_L_1, RVC_DIG_COM, VCV_OC, ETC_PWM_2 | session traces, OBD |
| SC8O40 (`8V0906264H__0005`) | Immobiliser power-class check: the bench DFlash record has B = `0xBA` (adapted in the Golf), the SC8O40 list `imo_idxtun01..05_c_vw` accepts only `0x5C` → immobiliser locked, no 0x010 challenge, ignition and injection cut; the later reset is the firmware's own start monitoring (LV_ERR_ST_MON → MU). Code `0x801261E0`, list CAL `0x8087944C`; same on SC8S50 code at `0x8011C414` | REA `ev_474f2844…`, `ev_343d938a…`, FR p.12852 (separate agent) |
| MPI | Port injectors on G1.GTC00/03/02/01 = cyl 1/2/3/4, active low; driver reaches RUN; at warm idle the firmware stays DI-only (mode 0) because the MPI warm-up inhibit holds (modelled oil 30 °C, IAT 8 °C) — not the cause of the old lean drift. Decoder recipe in the agent report (MILESTONE §5) | VERIFIED-S/T (separate agent) |

### 12.6 Fuel system, MPI and the idle blockers, seventh pass (2026-11-02)

Method: REA (rizin provider) on ELFs of the FACTORY (259J) and SC8O40 264H images (`scripts/make_elf.py` segments, PFlash0/1 +
CAL alias), FR via `scripts/fr_tool.py` (FUSL 43.15/43.29, CBMD 7.4, FMSP 41.19, INJR 50.1, TQLO 71, INTC 52.61, ENLU 27),
DAMOS values read from each image, firmware measured values over UDS 0x22 (DIDs of `log_tools::channels`, both images) and
the Motor_20 frame on CAN, observation probes, the reference CJSA log. Bench configuration is a scenario parameter
(`SIMEMU_ENGINE`, `SIMEMU_TURBO`), not tied to the image; the runs below: 259J on `cjxc` (2.0 + IS38), 264H on `cjsa`.

| Item | Result | Evidence |
|---|---|---|
| Rail at 17–23 MPa (root cause) | The plant latched the N276 closing from `on + delay` until the next plunger TDC. The firmware's zero-delivery pulse (259J, idle ≈ 1300 rpm: current on 22.5 °CRK before plunger TDC, **off 6.1 °CRK later**) reached its 3 ms closing delay 1° *after* TDC, the latch held through BDC → a full stroke every lobe → relief valve. A normally-open digital inlet valve only closes while the plunger rises; once closed it is held by the chamber pressure above `c_fup_min_lock_vcv` (25 bar 259J @0x80808654 / 20 bar 264H @0x808085BC, "self locking of the VCV"); the armature returns after the current ends | GTC log of G1.GTC26 (VERIFIED-T); FR p.12026 "currentless open VCV", p.12047 fig. 43.15.59–61 (ANG_CUR_ON = close range + mechanical delay, ANG_CUR_OFF); Bosch HDP5 principle (Reif, Gasoline Engine Management 2015) |
| HPFP model (plant) | Coil current [on, off) per lobe (`EcuOutputs::hpfp_off_deg`, decoded from the same low phase); armature pulled `hpfp_vcv_close_delay_ms(VB)` after current-on, released `hpfp_vcv_release_ms` (0.5 ms, UNVERIFIED) after current-off; inlet closes at the first rising-plunger instant with the armature pulled (energised at BDC = full stroke), self-locked above `hpfp_vcv_lock_mpa`. Loss `ip_mfp_ofs`, efficiency and stroke per profile | `engine::tests::hpfp_valve_closes_only_in_the_up_stroke` |
| Pump hardware per engine | 259J (2.0): `c_mfp_max` 232 mg, VCV type 0 (`lc_swi_vcv_typ` 0), delay = `ip_t_dly_vb_add` 3.0 ms @ 14 V (VERIFIED-T: lead 22.5° at 1300 rpm = 2.9 ms), efficiency 0.807, loss `ip_mfp_ofs` 13–90 mg. 264H (1.8): `c_mfp_max` 174.5 mg, type 1, delay `ip_t_dly_vcv_close_add_3` 2.552 ms × `ip_fac_vb_t_dur_vcv_ctl` + `ip_t_dly_vb_add` = 1.30 ms @ 14 V (HYP combination per fig. 43.15.59), efficiency 0.82, loss 2.5–37 mg. `c_dpl`: 259J 2.0 l @0x80809B84, 264H 1.8 l (@0x80809AC8 by neighbour offset), 264K 1.8 l | DAMOS (VERIFIED-S); profiles `ea888_20t_cjxc_is38` / `ea888_18t_cjsa_is12` |
| DI injector per engine | CJXC = 259J `ip_ti_4_l` fit (18.98 cm³/s @ 10 MPa, t0 0.037 ms, as before); CJSA = SC8O40 `ip_ti_4_l` fit (14.12 cm³/s, t0 0.017 ms; @0x80849D92) | `engine::tests::injector_matches_the_calibrated_characteristic` (both maps, ±3 %) |
| VCV_OC | VCV diagnosis `0x808BB868` (FUSL 43.29, FR p.12174–12180) reads GetElDiagVcvHss = `0x808AD95C` (ATIC140 diag getter `0x800A9F68` id **0x119** = chip 1 stage 25) and GetElDiagVcvLss = `0x808AD978` (id **0x118** = stage 24). "Due to the design of ATIC 140" the good case is OC-condition + OC-error on **both** switches; all-valid-no-error on both (what the model reported) is the open-circuit symptom. Model: the coil bridges HS 25 / LS 24 — both off-state open-load checks see a floating output → code (1,1,0); a broken coil → each output at its own bias → (0,0,1). The bias mechanism is UNVERIFIED (no data sheet) | REA `ev_5adcf54f…` (getters), `ev_356c6ea5…`; `atic::tests::atic140_bridged_vcv_coil_reads_open_load_on_both_stages_when_intact`; run: VCV_OC gone, rail controlled |
| Rail vs set-point | 259J/CJXC: FUP follows FUP_SP (DIDs 0x2027 / 0x293B) — 15.3/13.7 MPa right after the start (the set-point ramps down faster than injection empties the rail: minimum delivery cannot remove fuel), then 10.2 / 10.2 MPa from 2 s on its own; PUMP_VOL (0x209A) 5–11 % while controlling. The 3–3.5 MPa of §12.5 was a DAMOS row, not the runtime set-point | `engine_start_scenario` `rail while running` (asserted ±1 MPa) |
| Other unmasked faults (separate agent, REA) | CONV_PLAUS: the second conversion of PVS_2 is ADC2.CH0 (id 0x800C, check `0x808C84B8`, ≤ 0.298 V) — now fed. PORT_FB_EL_L_1: flap potentiometer G336 = ADC2.CH8 (id 0x8005, window 0.2/4.8 V, inverted by `ip_v_port_1_cor`, open 1.20 V / closed 3.80 V at the pin) — fed from the decoded flap command (N316 not decoded → open). LTS_EL_L_TCR: radiator-outlet sensor G83 = ADC1.CH6 (id 0x4004, `0x808E558C`) — fed with the G62 characteristic (`ip_tlts_mes[1]` within 1.6 °C). ETC_PWM_2 was never failed: the ERRM status byte 1 bit 1 is also set while a test is running (failed = bit 3, HYP from observed values). Left (bench / other models): COM_*_TOUT (absent chassis ECUs), VP_BPA_L_1 (wastegate sensor), CAM_STAT_VCP_IN_1 (phasers), RVC_DIG_COM (SENT receiver `0xF0321000` not modelled: driver `0x800A8C50` status 1), OIL_SENS_PLAUS (oil level/temperature PULS sensor not modelled) | `studio::io` constants; `engine_start_scenario` asserts that only these remain (SC8S50) |
| Charge heating in the manifold | The reference CJSA log reads IAT 39.7 °C at 16 kg/h idle with 12.7 °C ambient (88 °C coolant): the plant's 4 % pick-up gave 8 °C. Model: series conductance head flange → manifold wall → air (UA ∝ ṁ^0.8), fitted to the log at idle (1.9 W/K), flange 4 W/K UNVERIFIED → +3 K at low-speed full load | `engine::tests::manifold_heat_pickup_matches_the_warm_idle_log` |
| MPI release | CBMD 7.4.2.7 (FR p.1707): `id_tia_toil_mpi_inh` (TIA × TOIL, 1 = release) and `ip_maf_int_ast_mpi_inh_thd` (0 g at TCO_ST ≥ 20 °C), delay `c_t_dly_tia_toil_mpi_inh` 5 s. With the manifold air at 33–35 °C (TOIL model 30 °C) the firmware switches the warm idle to MPI at ≈ 5.5 s on both images | DIDs 0x13AC (TI_1_MES_MPI 3.1–3.4 ms), 0x202F (TOIL), 0x1001 (TIA) |
| MPI injector and port film (plant) | Decode: G1.GTC00/03/02/01 low phases → `EcuOutputs::port_injection` (same cycle rule as DI), gated by the ATIC140 enables (power stage HYP). Injector = the calibration's characteristic: `ip_fac_mff_ti_stnd_h_rng_mpi[0]` 0.434 ms/mg at ΔP 5.0 bar (`ip_fac_ti_prs_mpi` = 1.0 at 5004 hPa, √ law within 3 %), dead time `ip_ti_add_dly_mpi[0]` (0.47–2.56 ms vs VB; identical in 259J and 264H); supply = the LP system (ΔP to the manifold). Port film x–τ (Aquino 1981 SAE 810494, Hires & Overington 1981 SAE 810495): x 0.30 = `ip_fac_mff_wf_mpi[0][1]` at idle, τ 0.19 s at 90 °C from `ip_fac_mwf_vap_im[1]` × temp. corr. ≈ 0.8 per cycle (HYP reading), Clausius–Clapeyron in the head temperature, IVD knob 0 (UNVERIFIED); the port charge is inducted with the fresh air of each cylinder. Check: at the MPI switch the firmware commands 3.1 ms for its ≈ 110 mg → 6.4 mg intended, the plant cylinders receive 6.5–6.7 mg; the reference log (CJSA idle, 3.58 ms, 5.89 bar, 34 kPa, λ 1.0, STFT −16 %) is consistent with the same characteristic within 2 % | `engine::tests::port_injection_follows_the_calibrated_characteristic_and_the_x_tau_film`; `io::tests::injection_pulses_*` |
| Decode defect fixed | `injection_pulses` held the last pulse set for up to 1440 °CRK: a cycle without pulses (fuel cut-off, cylinder cut-off, the DI → MPI switch) re-injected the previous cycle in the plant. Now only the cycle that ended at the last firing TDC is reported | `io::tests::injection_pulses_report_the_last_complete_cycle` (new case) |
| Idle target | N_SP_IS is broadcast in Motor_20 (0x121) `MO_Solldrehz_Leerlauf` = byte 5, 10 rpm/bit (FR p.18057/18069; 0x64 = 1000 rpm before start → 760 rpm warm on 259J, 700 on 264H) | CAN (VERIFIED-T) |
| **Blocker 1: idle torque** | 259J/CJXC idles at 1150–1300 rpm vs N_SP_IS 760–990 with spark **−8 … −9 °BTDC** and the throttle at 2.4–2.9 °: the firmware is at its minimum torque realisation (TQ_DRIV_REQ −29 Nm, TQ_ENG ≈ 0 over DIDs 0x4380 / 0x437C) and still over-speeds. The calibration's reference (MBT) ignition `ip_iga_ref_ivvt_vvl_port_h[0][0][0]` @0x80840D58 is ≥ 40 °BTDC at 700–1500 rpm / 75–175 mg, 36 ° at 2000/175 mg, 15 ° at 2000/440 mg, 18 ° at 3000/420 mg; the plant's MBT (`bench_tests::explore_mbt_vs_calibration`) is 6 ° / 14 ° / 18 ° / 18 ° / 26 ° at the same points: its burn has almost no load dependence, so the firmware's 48 ° retard from its MBT (`ip_eff_iga` ≈ 0.2) costs the plant only ≈ 6 % torque. Plant friction + pumping (21 Nm at 1200 rpm, 105 mg) is close to the firmware's `ip_tqfr_cbk_vvl_port_close` (≈ 20 Nm) — not the cause. A ρ^-1/2 density term (Keck / Tabaczynski, Heywood §9.5.3) moved idle MBT only to 18 ° because the flame-development term `(S_L_REF/S_L)^0.6` grows with earlier spark — not adopted; the combustion model needs a recalibration against `ip_iga_ref` / `ip_eff_iga` | explore test; DAMOS; UDS |
| **Blocker 2: air charge** | At the same MAP (firmware 26.2 kPa vs plant 26.2 kPa over DID 0x39C0) and IAT, the firmware's MAF_CYL (DID 0x2032) is ≈ 0.86 × the plant's air (2.8–3.3 g/s vs 3.2–3.9 g/s, integer kg/h): plant VE at idle 0.72–0.74 vs ≈ 0.62 of the calibration's model; cams at rest (phasers not decoded) — the firmware's frame matches the plant's centrelines (`ip_vo_eff_scav_1` axes: intake 125 °, exhaust −114 °). On MPI the firmware fuels its air: plant λ 1.07–1.16, and with the late spark the engine stalls ≈ 2 s after the switch (259J: self-run ≈ 5 s) | DIDs; `engine_start_scenario` |
| SC8O40 on CJSA | With the bridged VCV load the 264H firmware drives N276 at the start (time-based start pulses, the same pattern as 259J: 452 ° then 93 ° / 38 ° / 34 °, then ≈ 43 ° at 102.75 → 145.5 °CRK = near-full delivery), the rail reaches 23 MPa (relief) in cranking — without it (main a53c62e) the rail stayed at 0.5–0.7 MPa (VCV_OC reaction). After the start the VCV stays in its open phase (FR p.12019: no high-pressure start request / start phase / pressure above set-point), the rail decays 23 → 15.5 MPa. Same angle-output table (`0x800418CC`, ch 9 = G1.GTC26). The engine reaches 1075 rpm and stalls at ≈ 4 s: the run-up is very rich (in-cylinder λ 0.4–0.54: DI start pulses 73 °/31 ° BTDC 1.11/0.91 ms at 21–23 MPa ≈ 31 mg, then MPI 9.2 → 3.7 ms) while the manifold empties (MAP 82 → 34 kPa in 0.4 s); misfires follow. Not resolved: whether the start fuel is the firmware's intent for these plant conditions (port film at cranking: the firmware's model has x ≈ 0.38–0.48 at high MAP and a per-cycle vaporisation — slower than the plant's time-based τ at 250 rpm) | `engine_start_scenario` with `SIMEMU_IMAGE=images/8V0906264H__0005.bin SIMEMU_ENGINE=cjsa`, IO report |

Open: combustion recalibration (MBT vs load), air-charge (VE) vs the firmware's model with the phasers driven (other agent), the
264H run-up mixture, the oil-level/temperature (PULS) and SENT sensors, which DTCs 003B42 / 003B43 / 003C70 are (the
ERRM → DTC map is not decoded).

## Appendix A — reproducing the trace

Scratch copy (outside the repo) of `crates/unicorn-tricore-sys` + `crates/simemu-tc1791`; in `Simos18Board::hook_mmio_f000` write branch add:

```rust
let mut pc: u32 = 0;
unsafe { uc_reg_read(uc, TriCoreReg::PC as c_int, (&mut pc as *mut u32) as *mut c_void); }
eprintln!("W {:08X} {:08X} {:08X} {}", pc, addr, val, _size);
```

and a 15-line `main` that runs `run_boot_sequence(30_000_000)` on the stock flash/EEPROM and dumps `0xF0001800..0xF0002FFF`, PRAM, CMEM and DSPR. 25 864 writes outside PRAM/CMEM/MultiCAN; final PC `0x8001F564`, RTA-OS running.

## Appendix B — raw data

GPTA final state after boot (non-zero only, VERIFIED-T):

```
G0: 80C=43FF 810=7 818=04000000 820=01000000 82C=20000000 838=1 848=00930046 860=01900046 870=01900046 880=129
    8C0=4 8C4=100 8CC=FF00 8D0=1090 8D8=C846 8E0=4E 8F0=1E 8F8=FFFFFF
    GTC: 00=5036 01=101E 04=5800 05=5800 12..15=5808 16=5800 17=5800 21=5000 23=5000 24=5000 26=4838 27=31 28=4838 29=31
         30=047F(XR 100) 31=420
    LTC: 04=1103 05=2031 05XR=8952 06=7811 06XR=FFFF 07=7821 | 08=12 09=20 10=22 | 12=1303 13=2031 13XR=1B75 14=7811 14XR=FFFF 15=7821 15XR=1B76
         24=110B 25=2031 25XR=493C 26=7811 26XR=FFFF 27=7821 | 28=1103 29=2031 29XR=8952 30=7811 30XR=FFFF 31=7821
         40=1103 41=2031 41XR=1B75 42=7811 42XR=FFFF 43=7821 | 44=103 44XR=FFFF 45=1031 45XR=168 46=7831 46XR=B5
         48=13 49=28 50=1010 51=3831 51XR=55D4 | 52=13 53=18 54=1020 55=3831 55XR=CDF
         56=5903 56XR=FFFF 57=2031 57XR=2326 58=3011 58XR=2327 59=3021 59XR=2327 | 60=102 61=19
    C00=70F F00=00080000 F04=00555502 F08=50251086 F0C=00000555
G1: 818=40 820=1000 838=1 848=00181780 868=00101780 870=00280000 880=4 8B0=4 8D8=1C846 8E0=4E 8F0=3E 8F8=FFFFFF
    GTC: 00..03=5800 06=043E(XR 1000) 08..15=5800 24=5000 26=5800
    LTC: 00=5903 01=2031 01XR=DB9 02=3011 02XR=DBA 03=3021 | 04=5103 05=2031 05XR=DB9 06=3811 06XR=DBA 07=3821
         08=5B03 09=2031 09XR=DB9 10=3011 10XR=DBA 11=3021 11XR=15E | 12=590B 13=2031 13XR=15D 14=3011 14XR=15E 15=3021
         16=13 17=18 18=1020 19=3831 19XR=2BF2 | 20=13 21=18 22=1020 23=3831 23XR=E484
         24=590B 25=2031 25XR=15F7 26=3011 26XR=15F8 27=3021 | 28=590B 29=2031 29XR=15F7 30=3011 30XR=15F8 31=3021
         32=1103 33=2031 33XR=8952 34=7811 34XR=FFFF 35=7821 | 36=13 37=18 38=1020 39=3831 39XR=55D4
         40=13 41=28 42=1010 43=3831 43XR=5E69 | 56=1103 57=2031 57XR=8952 58=7811 58XR=FFFF 59=7821 | 60=D02 61=432
L2: 820=01000000 828=1 838=1
    LTC: 16=1103 17=2031 17XR=DB9 18=7811 18XR=FFFF 19=7821 | 20=5903 21=2031 21XR=DB9 22=3011 22XR=DBA 23=3021
         24=590B 25=2031 25XR=15D 26=3011 26XR=15E 27=3021 | 28=110B 29=2031 29XR=493C 30=7811 30XR=FFFF 31=7821
         32=590B 33=2031 33XR=15F7 34=3011 34XR=15F8 35=3021 | 36=1103 37=2031 37XR=1B75 38=7811 38XR=FFFF 39=7821
         40=1903 41=2031 41XR=1687 42=7011 42XR=FFFF 43=7021 | 56=5B03 57=2031 57XR=1687 58=3011 58XR=1688 59=3021 59XR=1445
```

MRA words written by ASW (`0x801109EA`, after MRACTL=2; MRACTL=1 at end), VERIFIED-T:

```
GPTA0 (54): 70000000 00300000 04050607 0 0 00001700 00002300 00000013 0 00001300 23170000 00260000 0 0 00001700 0
            01000000 0 0 0 0 0 0 0 0 0 0 00260000 00050000 00010104 B2 B0 009393B7 008484B7 B0 B6 0 0 B6 B4 0
            B6 B6 008080B4 B6 0 80C0C3C3 C5C50000 0 0 0 0 0 0
GPTA1 (54): 0 0 0 0 03010200 0 00000002 0 0 0 00001317 0 17230000 00001313 0 00002300 0 0 0 0 0 0 00000017 24001300
            0 0 0 0 0 0 00008382 B6 009595B4 0 0 008484B7 009696B7 B6 B4 B4 008282B4 009090B4 B6 B6 B6 B6 0 0
            92929200 91919100 0 0 0 0
LTCA2 (32): 0 23 00230000 27 23 00131700 0 17000013 0 0 0 0 0 0 0 0
            0 B0 0 0 0 B0 B6 B4 B4 B6 B6 B6 0 0 0 0
```

---

## 12. Board integration results (engine I/O, 2026-11-01)

Implementation: `crates/simemu-tc1791/src/engine_io.rs` (GPTA + PCP in the board, sensor inputs, output taps),
studio co-simulation `crates/simemu-cli/src/studio/{io.rs,session.rs}`.

| Item | Result | Evidence |
|---|---|---|
| Clocks | fGPTA = fFPI / GPTA0 FDR divider (90 MHz); fPCP = fCPU / (CCUCON0[19:16]+1) = 90 MHz (field HYP: TC1791 DS p.84/117/121 lists fPCP as a separate clock, 120 MHz on 240 MHz parts) | `engine_io` module docs |
| Crank sync | firmware synchronises to a motored 60-2 + 2×8 cam signal ~80 ms after the crank front end is armed (time-gated: same at 1000 and 3000 rpm); GT1 = 15360 micro ticks/rev; PCP ch 0x3F once per tooth after the start delay | `tests/engine_io_crank_sync.rs` (VERIFIED-T) |
| Ignition | G0.GTC15/12/14/13 = coil of cylinder 1/2/3/4; idle high, **dwell = low phase**, spark = rising edge. Start ignition 6.7° BTDC (board stimulus) / 9.7° BTDC (plant sensor voltages), dwell 1.28–1.35 ms at 1000 rpm, firing order 1-3-4-2 | crank-sync test, `studio::io` test (VERIFIED-T) |
| Output multiplexer | MRA FIFO = 30 OMCR words (H/L, group 14..0) + 16 LIMCR + 8 GIMCR. Decode: OUT0/6/54/55 (pins P2.8/P2.14/P9.6/P9.7) = G0.GTC04/05/16/17 (the four cells parked at 0x5800); OG6 OUT108..111 = G0.GTC15/14/13/12 | `omcr_sources` (TC1796 UM p.24-97..99, 24-110, 24-194..197) |
| GPTA → MSC | MMXCTR00/01/10/11 = GPTA0 +0x700..0x70C (`0x00080000 0x00555502 0x50251086 0x00000555`). With the TC1796 encoding (0 GPTA0, 1 GPTA1, 2 LTCA2) GTC13/GTC12 dwell switches ATIC140#1 stages 22/23 off via MSC1 ALTINH.6/.7 (VERIFIED-T); GTC15/14 (OUT108/109) would come from GPTA1 — inconsistent, so the TC1791 MMXCTR encoding is UNVERIFIED. LTCA2-sourced ALTIN lines are not decoded (read low) | `EngineIo::msc_altin` |
| Injection | G1.GTC08..15 never left their parked level (motored crank, immobiliser accepted, no start request) | — |

### 12.1 Start path (2026-11-01, second pass)

| Item | Result | Evidence |
|---|---|---|
| KL50 wire | **P4.2** = IN26 of GPTA0/GPTA1/LTCA2 (DS p.39). Two readers: (1) port level, BSW getter `0x800A5B04` (pin id 0x42) → ENSS (init `0x800ECF94` → LV_ST_REQ_KL50 `0xD0000F12/F17`); (2) the "Hardware-Startwunschspeicher" (FR p.8620) = **G1.LTC60** free-running timer, LIMCR IN26, CTR 0x902 (ILM=1: counts fGPTA while the pin is high, AIL=0), overflow toggles OUT27 = **P4.3** (IOCR alt O2) which **G1.LTC61** counts back on IN27 (CTR 0x032, both edges). BSW `0x800A5B1C` reads `(LTC61.XR<<16 | LTC60.XR)·1000/90 MHz` [ms]; ENSS `0x801AD304` compares successive readings (`c_t_str_swi_st_req_diff_min` 6 ms, hysteresis 30 ms), writes the discharged value back (`0x800A5BA0`), state in `0xD0001FE0` + LMU `0xB0001F3C/40/44`. Earlier brute force failed because GPTA inputs were only fed from the crank/cam path and level-gated LTC timers were not modelled; a floating P4.2 (pull-up) reads as a permanent start request | VERIFIED-S/T (`gpta::tests::ltc_level_gated_timer_and_pin_loop_count_high_time`) |
| KL50R | starter terminal 50 feedback **P1.9** (getter `0x800A5AFC` → LV_ST_RESP `0xD0000F18`, `lc_st_resp_can_avl` = 0 hardwired); floating high = "starter engaged" clears start-release bit 9 | VERIFIED-S |
| P/N interlock | **P0.3**, ACTION_INFR_GetPnCluSwi2 = `0x800A6880`: LV_PN_CLU_SWI_2 = !P0.3 (low = P/N), FR p.25941 | VERIFIED-S/T (release bit 4 follows it) |
| Brake | brake light switch analog **ADC0.CH12** (logical id 3, getter `0x800A5600` → DRPD `0x80196C58`, threshold `c_vp_thd_bls_act` 3.5 V at the ADC pin, enabled above `c_vp_thd_bls_bts` 6 V supply); brake test switch **P9.4**, LV_PIN_BTS = !P9.4 (getter `0x800A55D4`) | VERIFIED-S |
| KL15 | **ADC2.CH4** (logical 0x8008 → PWSL `0x801073DA`), divider NC_V_IGK_BRI_RATIO 5.70, on-threshold 4.0 V (FR p.23942); NC_STATE_V_IGK_ST_PRES = 0 (no analog KL50). KL15 off → after-run, relays released 1.9/2.7 s later; KL15 on again → CAN + relays resume without reset | VERIFIED-T |
| Start release | stFgStart (`0xD0012452`) & `c_st_rls_mask` 0x6FD: bit 0 plausible request (`0xD0000F27` = brake OK ∧ driveline open ∧ KL15 time), 2 KL15, 3 no after-run, 4 driveline open, 5 start delay, 6 repeat lock, 7/8 engine stopped, 9 starter off ≥ 1.5 s, 10 no KL15 fault. Starter on at `0x801AC89C` (both relays) | VERIFIED-T |
| Starter relays | ATIC140#1 data bits **6 and 7** (MSC1 DD, wrappers `0x800A5B80`/`0x800A5B8C`, called only by ENSS `0x801AC89C..0x801ACBEC` and init `0x800ED0B6`): EN7_8 ("stages 7/8" in the chip's 1-based numbering) gates bits 6/7 | VERIFIED-S/T |
| Main relays | two port-driven relays, driver `0x800AB5D0`, table `0x80041990` {pin A, pin B, feedback}: relay 0 = P13.1+P13.0 (fb P13.3), relay 1 = P7.1+P2.7 (fb P7.0); energised = both pins high; released 2.7 s / 1.9 s after KL15 off. J271 vs J757 assignment UNVERIFIED | VERIFIED-T |
| Other DD stages | ATIC140#1 bit 2 electric oil pump/N428 (`0x808D15E0`), bit 3 secondary air, bit 4 piston-cooling jets valve (`0x808DB93E`, `lc_oil_2_pist_ivs`), bit 5 / #0 bits 12, 14 DMTL/NVLD (tank leak), bit 10 recirculation (`0x80193A9A`); #0 bits 12/13 AEB/ATB | VERIFIED-S (consumer calibrations) |
| BSW PWM channels | table `*(0xD0000744)` (28 B/ch, cell index at +0): 0xE = **EFP J538** (SetPwmEfp `0x800A6338`, SetFrqPwmEfp `0x800A6320` 20/100 Hz) = G1.LTC8..11 → OUT77 → MSC0 ALTINH.5 → ATIC140#0 bit 21; 0xD = electric coolant pump CWP (G1.LTC12..15 → OUT76 → ATIC140#0 bit 20; corrected in §12.8/§12.9 — the radiator fan is channel 0xF = G0.LTC12 quad); 0x3 = boost actuator BPA (`c_frq_pwm_bpa`, L2.LTC32); 0x5/0x9 lambda heaters; 0xA electric oil pump; 0xC "ECRAS"; 0x15 wastegate/BPA path | VERIFIED-S/T |
| Analog map | buffers `*(0xD00006E4+4k)`; ADC0 i ↔ CH(15−i), ADC1 [13,9,8,7,6,5,4,3,2,0], ADC2 [13,12,11,10,9,8,7,5,4,3,2,1,0]. By OBD DID response to a 2.5→1.0 V step: TPS1 ADC0.CH5, pedal D/E ADC0.CH7/CH8, IAT ADC0.CH10, MAP ADC0.CH11 (153 kPa @2.5 V, 51 kPa @1.0 V), baro ADC1.CH3 (66/32 kPa), rail ADC2.CH3 (14.0/3.5 MPa), CLT ADC2.CH7; TPS2 ADC0.CH6, fuel LP ADC2.CH2 by consumer | VERIFIED-T/S (`sensors_io::UNVERIFIED_ANALOG_MAP` docs) |
| Scenario (warm engine, P, brake at 0.5 s, KL50 1.0–3.0 s) | relays on at KL15; EFP pre-run 100 % (LP 6 bar); starter on 1.12 s (≈0.1 s after KL50, not before); plant hangs on the first compression (~7 V) 1.3–1.9 s, then cranks 255–310 rpm; firmware synchronises and fires **one** spark (cyl 1, 2.2 s), the angle outputs G0.GTC24 / G1.GTC24 / G1.GTC26 pulse once per cycle; **no injection** (G1.GTC08..15 parked) also with an external 5 MPa rail supply or a bench motor holding 270/1000 rpm; starter released at KL50 release; fault memory: 63 entries, none failed/pending. Next: why ignition stops after the first spark in the session context (the bare board + plant at 1000 rpm fires all four coils continuously) | `studio::engine_run` (ignored test) |

### 12.2 Start path, third pass (2026-11-01, native core + patched Unicorn)

| Item | Result | Evidence |
|---|---|---|
| Single spark (old Unicorn, session) | The session booted the ECU (16 M instructions) before the first plant exchange, so the crank/cam levels were applied only after the firmware had armed its GPTA captures: phantom edges, one sync, `N` (OBD buffer `0xD0012408`) back to 0 within 0.5 s. Fix: `io::power_on_ecu` applies all sensor levels at board time 0, then boots (sensors are wired before the MC leaves reset) | session vs. bare board with/without the late levels (VERIFIED-T) |
| Single spark (corrected cores, `plant_edges_*` test) | Not an input problem: with the crank already turning while the front end is armed ("flying start" from power-on) the firmware syncs, fires one coil and drops `N` to 0 for start angles 0/30/150/180 °CRK; other angles fire all four. The board-wheel crank test, plant wheel or not, `feed_ecu` or not, behaves the same; starting the crank after arming (real start) syncs at every angle (12 angles tested) | angle scan, `SIMEMU_CPU=native` (VERIFIED-T). Open: whether a real ECU resynchronises after a flying start |
| Cranking ignition | Session on native and patched Unicorn: all four coils fire continuously while cranking (start ignition 6.7° BTDC) and while motored at 1000 rpm | `engine_start_scenario` |
| Immobiliser in the session | `can_immo_rx 0x8009B644` runs on the cluster's 0x011 answer; `0xD0010428/2A/2B` = 1; the ECU stops re-challenging; IMO interface bytes `0xD0010861/69/6B` set (vs `0xD001086A/6C` without the answer) | probes/RAM reads (VERIFIED-T) |
| ATIC121 identification | Job = driver init state 1 (`0x800F7478`): R0B, W18=2, R01, R02, R0B, R00; parser `0x80099BC4`: chip id = reply to read 0x01, revision = low nibble of reply to read 0x02. The model answered at 0x00/0x01 → id 1/rev 0 stored, job repeated 139× and abandoned. Fixed in `atic.rs`; driver now reaches states 9/10 and configures the chip | SPI trace + disassembly (VERIFIED-S/T) |
| Injector boost voltage | VBOOST (`0xD0019580`, 0.5 V/bit, diag `0x801FD3D4` vs `c_vboost_mmv_diag_1/2/3` = 30/50/75 V) is ADC2.CH1 with 18.2 V/V (2.5 V → 45.5 V, 1.0 V → 18.0 V); was 0 V. Fed from a boost-converter model (65 V UNVERIFIED, regulated while the main relays are on) | channel bisection (VERIFIED-T) |
| HPFP N276 | Angle-output ch 9 = G1.GTC26, idle high, current feed = low phase; at start current on 55 °CRK after each pump TDC, off 19° before the next; pump TDC at k·180 °CRK (`c_ang_ref_vcv` = 0, DAMOS 0x8080860E). Decoded to `hpfp_close_deg`; the rail builds from 0.7 to 11.5 MPa in 0.25 s of cranking and the firmware then retards current-on to 158° | VERIFIED-T/S |
| High-pressure start threshold | `ip_fup_min_inj_h_prs` 40 bar warm (45/140/80 bar below 35 °C), fuel pressure setpoint before injection `ip_fup_sp_st_no_inj` 100 bar at 90 °C | DAMOS (VERIFIED-S) |
| Injection | Still none: PCP channels 0x08..0x0B never start, angle-output blocks ch 0..7 (`*(0xD00007FC)[ch]`, PRAM 0xF0051500..) never get a request, also with rail > 4 MPa, VBOOST 65 V, 13.8 V bench supply, cold start, or a motoring drive at 1000 rpm. INJR error-handler bank/cylinder inhibit bytes `0xD000E631/E639` stay 0. Open: STATE_INJ_DR / LV_INJ_ENA_MON / CBMD injection mode not located | — |

### 12.3 Injection, fourth pass (2026-11-01): root cause, outputs, first combustions

Method: basic-block coverage (observation probes on every block leader of a code range, `SIMEMU_PROBES`), RAM peeks,
store watchpoints on the native core (diagnostic build only) and Ghidra/REA decompilation of the functions found;
every link below was followed from the value that blocked injection back to its source.

| Item | Result | Evidence |
|---|---|---|
| Injector outputs | §1 item 6 / §6.2 were wrong: G1.GTC08..15 (angle-output channels 0..7) are the **valve-lift (AVS) actuators** [corrected in the tenth pass: channels 0..3 time the actuator pulses, 4..7 are the recoil-diagnosis windows; the actuator coils are ATIC140 stages 16..19, docs/SIMOS18_COMBUSTION.md §6b]. They are scheduled only by the VVL controller `0x800CAB24` (`c_n_min_vvl_ctl`, `lc_ctr_vvl_swi_clr`, `c_ctr_abc_max_*_susp`) through `0x800BBE74` / `0x800BBF20` (cfg `0x80042C60`: ch 0..3; cfg `0x80042C04`: ch 4..7 + port pins 0x10/0x11). The DI injectors are the ATIC121 timing inputs **G0.GTC04/17/16/05 = cylinders 1/2/3/4** (OUT0/OUT55/OUT54/OUT6 = P2.8/P9.7/P9.6/P2.14, the four direct pins of §6.3/§12), active low | injection HAL cfg `*(0xD000654C)` = `0x8005D8FC` → `0x8005D88C`: cells {4, 16, 5, 17} parked `0x5800` in firing order 1-3-4-2, banks {G0.GTC20/21: 4, 16} / {G0.GTC22/23: 5, 17}, SRCs → PCP 0x1D/0x29/0x1F/0x28/0x1E, PRAM page 0x0D; PRAM blocks 0xE20/0xF20 hold the GTC04/GTC17 CTR addresses (VERIFIED-S/T); each cell pulses before exactly one cylinder's firing TDC (VERIFIED-T) |
| Injection driver state | INFR_INJ ReqMode `0x8008A05C` (state `0xD0006588`) reaches 5 = RUN at KL15; INJR sends setpoints (SetPulse* adapters `0x8009B8AC`.. run) but `ACTION_INJR_SendBswHpdi` `0x808A2FB4` → INFR_INJ_SetInhibit `0x8009B778` → `0x8008B210` writes inhibit = 1 for every cylinder (`*(0xD000658C)[cyl]+8`) | probes, peeks (VERIFIED-T) |
| Inhibit chain | lv_inh_now = bit of 0xD000E6A4/A5 (LF_INJ_INH_SP queue, `0x808A2BA2`) ← 0xD000E6A0/A1 = LF_INJ_INH_STAT \| SEQ_FCUT pattern 0xD00096BC (`0x8089BA54`) ← sequencer `0x8089BB44` (TQDR "Sequential fuel cut-off", FR p.25228) fed with NR_PAT_SCC = **4** (0xD000F38C, `0x801E5A62` = ID_NR_PAT_SCC(EFF_SCC_SP)) ← LV_AUTH_TQ_PAT 0xD000167A (`0x801E58D2`, FR p.25394) ← LV_TQ_SCC_REQ 0xD000169D (`0x801EA62E`, TQSP, FR p.25664) ← 0xD000163F (`0x801E2CD6`) ← **0xD001C0CF** (`0x800742C6`): ECM2 error reaction `0x80073FB4` for level-2 error flag **0xD001C033 = LV_ERR_CONV_MON** (`0x80073F90`) | store watchpoints + disassembly (VERIFIED-T/S); static cylinder shut-off LF_INJ_INH_STAT 0xD000E63B (`0x8089C06C`) itself clears when the rail passes 4 MPa |
| Root cause: A/D-conversion monitoring | ECM2 17.5 (FR p.4939–4945): an auto-generated, first-order **hardware-filtered PWM** voltage is converted by the MC (VP_MC_AN_DIG_MON 0xD001CB24) and the MU (VP_MU 0xD001CB26); errors: alive check (MC value must move > `c_vp_min_thd_alv_chk_mon` 0.01 V between 40 ms samples, at most `c_ctr_min_thd_alv_chk_mon` = 10 stuck samples), range (0..`c_vp_max_thd_rng_mon` 3.15 V), MC/MU difference < `c_vp_acq_conv_hys_mon` 0.31 V. Both values were 0 → LV_ERR_VP_ALV_CHK_MON 0xD001C097 = 1 at ~0.55 s → C033 → fuel cut of all cylinders. No DTC (level-2 reaction) — the 63-entry fault memory showed nothing | `0x80073C00` decompiled; peeks (VERIFIED-T) |
| Test-voltage hardware | PWM: ACTION_INFR_SetPwmDucyADCMon `0x800629A4` → BSW PWM `0x800A8758` **channel 0x14** → table `*(0xD0000744)` = `0x80042400` entry 0x14 = cell 0x258 = **LTCA2 LTC56 quad** (0x5903/0x2031/0x3011/0x3021; period 5769 ticks — 15.6 kHz with the model's LTCA2 LIMCR decode CLK0 = fGPTA, HYP; §6.4 assumed 975 Hz), duty 2..91 % in 10 % steps every 40 ms. MC input: VP_ADC_MC `0x8006E2D8` → getter `0x800A70FC`(id 1) = **ADC0.CH14**. MU input: its own 8-bit conversion (RX of `0xBFE0`, `monitor_device`) | VERIFIED-S/T; pin of LTC59 and the RC values: UNVERIFIED |
| Defect 1: GPTA CUD | The BSW PWM driver updates a duty by writing the inactive compare cell and setting **CUD by read-modify-write without CUDCLR** (`0x800A88A4..0x800A88E8`). The model ignored CUD = 1 unless CUDCLR = 1, so no compare-pair PWM ever left its boot duty. TC1796 UM p.24-181: CUD rwh, set by writing 1, cleared by CUDCLR = 1 & CUD = 0 or by hardware at the reset overflow | `gpta::tests::ltc_pwm_duty_update_by_coherent_update_without_cudclr` (fails with the old semantics) |
| Defect 2: no RC path | New board node `engine_io::RcNode` behind L2.LTC59 (exact first-order RC, high level VDDP = 3.3 V, TC1791 DS p.87; consistent with the 3.15 V range limit at 91 % duty; τ = 10 ms UNVERIFIED) drives ADC0.CH14 and the MU's analog input | `engine_io::tests::adc_monitor_rc_node_follows_the_pwm_duty`; `tests/adc_monitoring.rs` (both cores: VP_MC/VP_MU follow the ramp 0..0x4CD0, agree within the tolerance, no error flag) |
| Injection while cranking | With both fixes NR_PAT_SCC = 0, inhibit 0, PCP 0x28/0x29 run per INJBAS, the injector inputs pulse. Warm start, 240 rpm, rail 23 MPa: **two pulses per cylinder and cycle at 85° and 43° BTDC firing, 0.95 / 0.74 ms**, decaying to 0.57 ms; after the start SOI moves to ~300° / 76° BTDC (1.0–1.7 ms) | `SIMEMU_GTC_LOG`, `studio::engine_run` (VERIFIED-T) |
| Plant decode | `studio::io::decode_ecu_outputs` → `EcuOutputs.injection[cyl]` = pulses of the last complete cycle (≤ 3, SOI before firing TDC, electrical width), gated by the ATIC121 enables (NDIS0 high, LS_DIS0 low); one cycle of latency like the coils | `io::tests::injection_pulses_report_the_last_complete_cycle` |
| Scenario | First decoded injection 2.0 s, **first combustions 2.56 s**, up to ~1070 rpm (native and patched Unicorn agree), then the engine stalls (~4 s): plant λ 0.3–0.4 (fuel 21–23 mg against 140 mg air at MAP 30 kPa) | `studio::engine_run` (asserts injection on every cylinder and first combustions) |
| Next blocker (idle) | The firmware targets λ 1.0 (F444 = 0x7F22) with an absolute load of 75 % (F443 = 0x00BF) while MAP is 29 kPa (F40B), and commands the throttle to 1.6–2 % (F44C) — the plant throttle stays at the 6.7 % limp-home opening because the ETC H-bridge (ATIC111) is not decoded. Sensor characteristics disagree with the firmware: IAT reads −26 °C (F40F = 0x0E) at 5 °C ambient, coolant 80 °C (F405) at 90 °C (NTC models UNVERIFIED). Plant injector 15 cm³/s @ 10 MPa + 0.25 ms vs the firmware's own characteristic `ip_ti_4_h` (0x80849FF4: 25→50 mg in 1.268→2.464 ms at 21.5 MPa ≈ 19 cm³/s @ 10 MPa, offset ≈ 0.06 ms): the plant delivers ~65 % of the intended mass (profile not changed) | UDS 0x22 during the scenario (`SIMEMU_UDS_DID`), DAMOS |

### 12.4 After-start idle, fifth pass (2026-11-01): throttle, sensors, injector, HPFP

Method: SC8S50 calibrations read from the factory image with the DAMOS JSON (map storage is column by column,
`columnDir`), disassembly of the BSW drivers (linear listing of PFlash0; CAL is reached through `a1` =
0xA0808000, startup 0x80007780), observation probes / port and GPTA register peeks, UDS 0x22 F4xx, DSPR
snapshots (`studio::engine_run` diagnostics). Corrections to §12.3: F44C is in % (0x19 = 9.8 %), not 1.6–2 %;
the `ip_ti_4_h` row quoted there is the 21.5 MPa row of a 14–21.5 MPa map (the standard profile is `ip_ti_4_l`).

| Item | Result | Evidence |
|---|---|---|
| ETC H-bridge = ATIC111 bridge 0 | H-bridge driver vtable `*(0xD00007D8)` = 0x8004F4EC → type table 0x8004F4A4 (+0x3C SetDuty 0x801F5854, +0x0C enable 0x801F54F4), chip config 0x8004F49C: 3 bridges on SPI handle 0x2180, entries 0x8004F46C {BSW PWM channel, direction pin, disable pin, SPI command 0x0008, invert flag}: bridge 0 = PWM channel 0, direction **P14.4** (pin id 0x304), disable P6.7 (0x67 = MTC_DIS); bridge 1 = channel 1, P4.5, P13.9; bridge 2 = channel 3, P13.6, P4.9. The SPI command stays 0x0008 on all three bridges (only mode bits 3:2; no duty over SPI) | disassembly; ATIC111 frames traced (one change at init) |
| ETC duty path | THRO ACTION_INFR_SetPwmDucyEtc (FR p.25114, 2 ms) → 0x801F5854(handle 0, ±0x4000 = ±100 %): magnitude → BSW PWM 0x800A8758(channel 0) → 0x800A8830 (LTC quad, CUD) ; on a sign change the period interrupt of the PWM timer is enabled (0x800A8980 sets REN of the timer cell) and the ISR 0x800AC0C4 writes the direction pin one period later (0x800A7ADC). Channel 0 = **G1.LTC24 quad** (table 0x80042400: cell 0x138, 1 kHz); output cell **G1.LTC27 → GPTA1 OUT25 = P4.1** (OMCR, `EngineIo::out_lines_of`); **high = on**: 0x00FB (1.5 %) → 15 µs high per 1 ms, ±0x4000 → static high; P14.4 high = PWM_ETC ≥ 0 (opening) — P14 OUT bit 4 alternates with the ±100 % TPS start check | probes 0x801F5854/58F4 (D5 = duty), GPTA tap of G1.LTC27, P14 OUT peeks (VERIFIED-S/T) |
| Decode | `EcuOutputs.etc_duty` = ±(high fraction of G1.LTC27 over its last period) with the sign of P14.4, `None` while the bridge is disabled (EN = MU NDIS1, DIS = P6.7); `ltc_duty` now treats an output quiet for > 1.5 periods as static (the former 100 ms rule reported a stale duty for up to 100 ms after a jump to 0/100 %) | `studio::io` |
| Throttle actuator (plant) | DC motor + gear + limp-home spring pair, Coulomb friction, stops at 0 / 90 ° (`c_tps_max` 90 °TPS @0x8080C058), limp-home 7.0 ° (`c_tps_lih_ini` @0x8080C050 = `c_vp_tps_sp_lih_1` 0.8333 V @0x8080C0C6). The firmware's position controller (`c_kp_etc` 110 %/°, `ip_kp_cor_etc` 120 → 82.5 %/° over 10–16 V, D on the measured angle `c_kdf_etc` 6.5 per 2 ms) limit-cycled ±2 ° at 43 Hz (bang-bang) with a fast actuator (J 4e-4 kg·m², 0.14 N·m/A) and settles (±0.05 °, ≈ 18 % duty holding 12 °) with J 1.2e-3 kg·m², 0.10 N·m/A, 1.6 Ω (UNVERIFIED values; the closed-loop stability of the production controller is the indirect evidence) | `engine::tests::throttle_actuator_opens_returns_and_holds`, session trace at 1 ms |
| Throttle flow | The plant uses the calibrated reduced area `ip_ar_thr_bas` @0x8084BE3C (0.06 cm² closed, 0.45 cm² at 7 °, 20 cm² at 80 °) instead of the geometric Ø56 mm plate model, which was 2.8–4.4× smaller at idle openings | VERIFIED-S |
| Sensor characteristics | Inverse of the firmware's conversions: IAT `ip_tlts_air_mes[0]` @0x80810ADA, coolant `ip_tlts_mes[0]` @0x8082357A, MAP `ip_prs_im_mes` @0x808099E4, baro `ip_amp_mes` @0x80809878, PUT `ip_prs_up_thr_mes` @0x80809A18, LP fuel `ip_fup_efp` @0x808018AE, rail `c_vp_fup_mes_ofs`/`c_fup_mes_slop` 0.5 V + 7.0 MPa/V, TPS `c_vp_tps_ad_bol_1/2` 0.5/4.5 V + `c_tps_slop` 21 °/V, pedal `c_vp_pvs_slop_1/2_mon` 36.6/73.1 %/V. **PUT = ADC1.CH2** (getter id 0x4008 in 0x800FE934 next to MAP id 4 and AMP id 0x4007; 1.5 V on ADC1.CH2 → OBD PID 0x70 sensor A 92.5 kPa) — it was not fed (the firmware read 20 kPa) | `sensors::tests::sensor_voltages_invert_the_firmware_tables`; `engine_run::tests::sensor_characteristics_round_trip_through_the_firmware` (both cores): CLT 89/89.9 °C, IAT 8/7.6 °C, MAP 99/99.4, baro 99/99.4, PUT 99.38/99.41 kPa, rail 7.97/7.97 MPa, TPS1, pedal D/E (VERIFIED-T) |
| Injector | `ip_ti_4_l` @0x8084A214 (standard current profile, 0.5–15 MPa): ti = m / (q·√(ΔP/10 MPa)) + t0 fits 10–50 mg / 4–15 MPa with q = 14.14 mg/ms (18.98 cm³/s at 745 kg/m³), t0 = 0.037 ms, max residual 0.057 ms (was 15 cm³/s, 0.25 ms) | `engine::tests::injector_matches_the_calibrated_characteristic` |
| HPFP | N276 closes `ip_t_dly_vb_add` @0x808085AC after current-on (3.0 ms at 14 V, 5.3 ms at 6 V: the firmware's idle current-on 22 °CRK before plunger TDC = zero delivery — without the delay the plant pump kept the rail on the 23 MPa relief valve); stroke = `c_mfp_max` 232 mg @0x8080865E; delivered = 0.807 (`ip_fac_fuel_pump_eff_*` @0x8083C026/66/A6) × displaced volume − `ip_mfp_ofs` @0x8083C22E (13–90 mg/stroke); HP volume 50 cm³ UNVERIFIED (25 cm³ → 2–4 MPa overshoot per stroke against the 1.5 mg/bar P part `ip_fuel_mass_req_p_ctl` @0x8080189E) | VERIFIED-S (calibrations), session traces |
| Idle targets | `ip_n_sp_is_bas[0..3]` 700 rpm at 80–90 °C; after start `ip_n_sp_is_t_ast` 800 rpm up to 5.5 s, 680 at 15 s, 660 at 25 s; rail setpoint `ip_fup_sp_bas_sel[0]` 3.0–3.5 MPa at 100 mg/stroke | DAMOS |
| Scenario (warm, 11 s) | Throttle to the firmware's start setpoint ≈ 12 ° while cranking, first combustions 2.6–3.0 s, then the engine runs **on its own for ≥ 8 s**: 5–10 s window 1135–1364 rpm (native) / 922–1371 rpm (patched Unicorn), throttle 3.1–3.8 ° (ETC closed loop), MAP 29–31 kPa, rail 23 → 6 MPa, mean exhaust λ 1.11 / 1.17. 63 fault-memory entries, none failed/pending/confirmed | `studio::engine_run` asserts this window |
| **Next blocker** | Open-loop mixture drift: the firmware fuels for an air mass of ≈ 112–114 mg/stroke (DSPR floats 0xD000A05C / 0xD000AE74 track it; 112.6 mg × 1/14.7 = the injected 7.66 mg at λ_sp 1.0 — HYP identification), the plant inducts 120–130 mg (throttle mass flow 16.8–18.1 kg/h); λ drifts from 1.0 (7 s) to 1.2–1.3 (11–16 s) and the engine stalls at ≈ 17.5 s (native). Nothing corrects it: the wide-band sensor interface (CJ125 over SPI, UA/UR inputs, heater PWM channels 5/9) is not emulated, so λ control stays open loop. [Superseded by §12.5: the IC is an ATIC142 over SPI, no UA/UR ADC inputs; closed loop reached.] Also open: the speed stays at 1150–1300 rpm above the 800/700 rpm setpoint (torque/friction model vs TQLO not compared), cam phasers N205/N318 and bridges 1/2 not decoded | RAM snapshots `SIMEMU_RAM_DUMP`, `SIMEMU_CYL_REPORT` |

### 12.6 Boost: electric wastegate actuator, ATIC111 bridges 1/2, diverter (2026-11-02, seventh pass)

Method: REA (rizin) disassembly — `ev_0c628027…` (H-bridge SetDuty 0x801F5854), `ev_60f7dbf2…` (BPA INFR wrappers, position-
controller tail call, N249 setter); FR CHRA 9.25–9.29 / CHRG 10.13–10.26 via `fr_tool.py`; traces on the native core of an
instrumented scratch copy (not in the tree); RAM addresses resolved with a0 = 0xD0018000, a1 = 0xA0808000 (startup 0x80007778).

| Item | Result | Evidence |
|---|---|---|
| ATIC111 bridges | Config entries 0x8004F46C (16 B: +0 BSW PWM channel, +2 direction pin, +4 disable pin, +6 SPI command 0x0008, +8 = 1 direction not inverted, +9 = 0 disable active high): **bridge 0 = throttle** (ch 0, P14.4, P6.7), **bridge 1 = rotary coolant valve N493** (ch 1, P4.5, P13.9; ENTE 34.25, FR p.9095), **bridge 2 = wastegate actuator V465 "BPA"** (ch 3, P13.6, P4.9). Each bridge has its **own** disable pin; the model gated all three with P6.7 (VERIFIED-T: P6.7 high while P4.9 low and the wastegate PWM running) → `power_stages::PIN_BRIDGE_DIS` | VERIFIED-S/T |
| BPA driver | INFR wrappers forced to handle 2: DisableBpaDr 0x800A53B8, EnableBpaDr 0x800A53DC, status 0x800A5400, SetPwmDucyBpa 0x800A5448, SetPwmFrqBpa 0x800A5468 (`c_frq_pwm_bpa` 1000 Hz), GetVpBpa 0x800A5430; variant gate `c_conf_pin_var` @0x80801C7C = 0 → H-bridge (1 = pneumatic actuator on BSW channel 0x15) | VERIFIED-S |
| Duty path | CHRA position controller (FR 9.26, 1 ms; Kp `ip_fac_kp_bpa_ctl` 17.08→11.38 over 10–16 V, KDF 13, KI 0.00293) ends at 0x8017BDEE..0x8017BE0A in SetPwmDucyBpa(0, PWM_BPA/2) (±100 % → ±0x4000). PWM = L2.LTC32 quad, output **L2.LTC35, high = on, 1 kHz** (−14.86 % commanded → 0.148 ms high per 1 ms); **+ = towards open**, P13.6 high = positive | VERIFIED-T |
| Position sensor | **ADC1.CH5** (logical 0x4005), 0–5 V, 152.6 µV/bit (raw12 << 3); POSN_BPA = (VP − VP_BPA_AD_BOL)·FAC_BPA_SLOP: 0 % = flap seated (high voltage), 100 % = open. Bench ECU learned BOL 3.931 V (0xD0011DDE), slope −33.33 %/V (constant 3.0 V span, `lc_vp_bpa_dif_tol_bol_con` = 1); BOL accepted 2.73–4.52 V; range check 0.20–4.80 V (fault → limp-home 95 % with zero PWM) | VERIFIED-S/T |
| Firmware's demands on the actuator | static hold "noise" mode −15…−25 % (`ip_pwm_bpa_noise`) while POSN < 35 %; seat learning at −40 % within 0.3 s (`c_pwm_ini_ad_bol_bpa`, `c_t_vp_bpa_ad_acq`); limp-home = zero duty → the de-energised actuator must go open (HYP, strongly implied) | VERIFIED-S |
| RAM (SC8S50) | PUT_SP 0xD0011EEE, PUT_MES 0xD0013FB4, POSN_SP_BPA 0xD0011D96, POSN_BPA 0xD0011D8A (i16, 100/16384 %), PWM_BPA 0xD0011DBA, VP_BPA 0xD0011DDC. Standard UDS DIDs: 0x2029 PUT_SP / 0x202A PUT (hPa), 0x39A2 / 0x39A3 POSN_BPA / POSN_SP_BPA (0.01 %; SimosTools shows 100 − x = **100 closed**, the old `channels.rs` note was inverted) | VERIFIED-S |
| N249 | ATIC140#1 stage 10 (SetCmdRcl 0x800A549C), energised = open (LV_CMD_RCL_OPEN, FR 9.7) | VERIFIED-S |
| Not decoded | N493: its position comes back over SENT (0x800A5D9C, SENT driver vtable 0xD00007DC, logical channel 0); no SENT model → RVC_DIG_COM, the firmware runs it in its limp-home ramp (+35 → +12.5 %) | VERIFIED-S/T |
| Plant | `simemu_physics::wastegate`: DC motor + gear against a return spring towards open, exhaust Δp on the flap, Coulomb friction, stops, swing-valve flow law, linear sensor from the profile (`TurboHardware::wg_actuator`, values UNVERIFIED but checked against the firmware's demands above) | `wastegate::tests::actuator_closes_holds_and_springs_back` |
| Closed loop | key on: bridge enabled, firmware POSN_BPA = plant within 1 % (`io_air::tests::wastegate_bridge_and_position_sensor_close_the_loop_at_key_on`); engine dyno 3000 rpm WOT (2.0 TSI CJXC + IS38 profile, SC800S50): PUT follows PUT_SP to 1.2 % mean, POSN_BPA follows POSN_SP_BPA to 2.0 %-points mean, MAP ≈ 240 kPa (`dyno_run::engine_dyno_wot_boost_and_knock`) | VERIFIED-T |

#### 12.6.1 P00AF `PWM_BPA_H` on stock SC8O40 after a 5 °C start (2026-11-07)

Image `8V0906264H__0005.bin` (SC8O40), a0 = 0xD0018000, a1 = 0xA0808000, a9 = 0xD000C000; FR 9.13 / 9.18 / 9.25; calibration
names and values from `definitions/SC8O40.json`.

| Item | Result | Evidence |
|---|---|---|
| Monitor | CHRA actuator diagnosis (FR 9.18), code 0x800BCF00…0x800BD140, 10 ms: PWM_BPA_MMV (0xD0015C28) = first-order low pass of PWM_BPA (0xD0011E76, ±100 % = ±0x8000) with `c_crlc_pwm_bpa` 0.199 while VB_MMV (0xD000F594) > `c_vb_min_pwm_bpa_diag` 9.04 V; symptom PWM_BPA_H (0xD0000B46) = MMV > `c_pwm_bpa_max_diag` 98 % unless the position-deviation symptom (0xD0000B44, `ip_posn_dif_bpa_err_max` 100 % → never) is set; PWM_BPA_L the mirror (< −98 %). Debounce 0xD0015BBE: +1 per symptom cycle, −127 otherwise, max 40 → **0.4 s of a saturated position controller**; error index 0x3BC. Enable: `c_t_ast_bpa_ctl_diag` 4 s after start, TAM > −39.75 °C, TCE > −40 °C | VERIFIED-S; FR p.2169 (VERIFIED-FR) |
| Position scale | SC8O40 does not learn the open stop: `lc_fac_bpa_slop_sub` = 1 → slope −26.5 %/V (`c_fac_bpa_slop_sub`, 0x801915A8; learned slope would be 0x19999999/(VP_TOL − VP_BOL) = 100 % at the top limit), `lc_vp_bpa_ad_tol_sub` = 1 → VP_TOL = `c_vp_bpa_tol_sub` 0.870 V; seat VP_BOL learned (0xD0011E9A, 3.925–3.931 V on the bench). POSN_BPA 0xD0011E46, POSN_SP_BPA 0xD0011E52 (i16, 100/16384 %), VP_BPA 0xD0011E98 | VERIFIED-S/T |
| Set-point limit | `lc_vp_sp_bpa_max` = 1 → POSN_SP_BPA ≤ (`c_vp_sp_bpa_max` 0.870 V − VP_BOL)·slope, clamped to `c_posn_sp_bpa_max` 95 % (0x800BBEFC). With BOL 3.925 V: **80.96 %**. SC8S50 (bench) and 264K use 0.570 V; the learned-top-limit plausibility window of SC8O40 is `c_vp_bpa_tol_sub` −0.30/+0.65 V = 0.57–1.52 V (FR 9.13.1, ±tolerances on the substitute) | VERIFIED-S |
| What set it | cold 5 °C drive, phase "40 %" (≈ 2400 rpm, 100 km/h, TCE 25 °C): the firmware opens the wastegate fully, POSN_SP_BPA = 80.9 % (= the voltage limit); the plant actuator stopped at 0.931 V = 79.2 % in the firmware's scale; the I share ramped PWM_BPA from +50 to +100 % in 2.3 s against the stop (plant −7.1 A, stalled), MMV > 98 % → P00AF status 0xA7 after 0.4 s. Warm drives never command the limit in this scenario | VERIFIED-T (RAM probes) |
| Plant before | sensor 3.931 V (seat) … 0.931 V (open stop) — the open stop had been chosen as "100 % of SC8S50's constant 3.0 V span", i.e. exactly at the S50 set-point limit and 0.06 V short of SC8O40's | `wastegate.rs` |
| Fix | IS12 actuator travel extended to an open stop of **0.70 V** (UNVERIFIED value inside the firmware's bounds: beyond the 0.870 V limit, above the 0.57 V plausibility floor; workshop sources give ≈ 0.2–0.8 V open, 3.4–3.9 V closed); same sensor gain, linkage ratio and seat spring torque, so nothing changes up to the former stop (the flow is saturated there). Result: the controller holds 81.3 % with −5 % duty; no P00AF on `bench_s50` / `octavia_fl` at 5 °C | `fault_audit_scenario` (VERIFIED-T) |
| Not examined | right after start (3.0–5.4 s, warm and cold) PWM_BPA is −100 % while the plant actuator carries ≈ 0.1 A and stays open; no fault results (seat learning comes from DFlash) | — |

### 12.7 Knock acquisition ("knock by soft") and knock control (2026-11-02)

Method: REA `ev_e02bdaa91fa5…` (FADC init 0x8010E2B0), `ev_7697558aaa3c…` (FADC StartGroup 0x800A6B34), `ev_2cf235c14c69…`
(knock DID getters); FR KNCK 53.2 / 53.18 (p.22133–22140, 22356–22370); TC1796 UM FADC chapter p.2052–2114; observation-only
probe traces (native, crank motored 2000 rpm).

| Item | Result | Evidence |
|---|---|---|
| Sensor / channel | **one** knock input: knock driver config 0x80042388 (1 input) → 0x8004237C {FADC ch 1}; every cylinder maps to it (`c_knks_cyl_conf` 0); FADC ch 2 (group 1, DMA ch 15) configured, never started | VERIFIED-S/T |
| Analog input | ACR1 = 0x5 per window: GAIN ×2, ENP = 1, ENN = 0 → single-ended FAIN1P − VFAREF/2, 0 → 3.3 V = code 0 → 1023; boot offset calibration GCR CALMODE 01 + FMR.SCRF1, expects 0x200 | VERIFIED-T |
| Window gate | CFGR1 = 0x30631887: GSEL 7, GM 10 (timer + conversions only while GS7 high), CTM 10, CTF 1 (fCT = fFADC, FDR 0x43FF), CTREL 99, INP 3. GS7 = GPTA0 OUT26 ← **G0.GTC24** (OMCR); G0.GTC24 high −9.7…+8.3° ATDC (pre-window) and +10.1…+43.1° ATDC (main = `ip_knkwb` 10.125° / `ip_knkwd` 33°) in every segment, scheduled by the angle-output driver 0x80088A64 (ch 8) | VERIFIED-T; GS7 ↔ OUT26 HYP (TC1796 UM Fig. 26-16) |
| Data reduction | FCR0 0x3512 (ch 1, ADDL 2, MAVL 1), FCR1 0xB202 (input FRR0, ADDL 2, IEN, INP 3): 12-tap FIR [1,1,1,2,2,2,2,2,2,1,1,1], 9:1 → FRR1 (gain 18) | VERIFIED-T values, UM 26.1.6 semantics |
| Samples to RAM | DMA ch 11 (CHCR 0x00302000 PRSEL 2 continuous 16 bit, ADRCR 0xF080, SADR FRR1 0xF0100494, CHICR 0x668 → DMA_SRC6 at TCOUNT 0); main buffer 0xD0010BF8 (TREL 400), pre 0xD0011238 (167); window-end callback 0x800AC300 ≈ 18 µs after the gate falls takes the count from DADR, drops the first sample | VERIFIED-T; FADC SR3 → ch 11 PRSEL 2 HYP |
| Software filter | 0x800C6828 (FPU loop 0x800C6B58..0x800C6C60): 3 band-passes y = x_k − x_{k−2} + F1·y_{k−1} + F2·y_{k−2} with integration of |y| = FR p.22137–22140; filter tables `NC_KNKS_FIL_FAC_1` (@0x80044020 / 0x80043F20) = 2cos(2πf/100 kHz) → the firmware assumes 100 kHz output | VERIFIED-S |
| Calibration (SC8S50) | `lc_knks_fil_tab` 1; filter 1 6.94 kHz (Q 9) → 7.6–13.1 kHz at high rpm, filters 2/3 13.7–16.6 / 19–23.7 kHz; gain `ip_knks_gain` 4–38 ± 8 adaptation; ITC 65–120 µs; NL target 0.51 → 1.0 V; threshold `ip_knks_thd_fac` 1.53–1.94; retard −1.5…−3° per event, max −8.6…−14.6°; sensor diagnosis "basic" KNKS < 0.117…0.312 V at N ≥ 2016 rpm, MAF ≥ 229 mg, TCO ≥ 59 °C | VERIFIED-S |
| Observables | per-cylinder retard u8 0xD0019888..8B (0x80 = 0°, 0.375°/bit), **DIDs 0x200A/0x200B/0x200C/0x200D = cylinders 1/2/3/4** ((raw − 128)·37.5 → /100 °); KNKS u16[4] 0xD0014274, NL u16[4] 0xD00142B2 (5/65536 V), segment order = firing order 1-3-4-2 | VERIFIED-S/T |
| Board model | `simemu_tc1791::knock_fadc`: register model (GCR triggers, FMR software conversion, CFGR gating/timer, ACR input configuration, filter blocks, RCH/CRR/IRR/FRR), conversions produced lazily at exactly the converter's instants while gated (channel-timer divider CTREL + 2 at fCLC/fCT = 1, UM p.26-44: 891 kHz — 1 % below the firmware's assumption, UNVERIFIED for the TC1791), FRR1 → DMA per output. Front end: AC-coupled sensor on a VFAREF/2 bias (firmware's filter initial value = mid-scale), gain 1 UNVERIFIED | `knock_fadc::tests::*` |
| Plant | `simemu_physics::knock_sensor`: chamber acoustic modes (Draper α₁₀…α₁₁, c from the burned gas) per knock event, block structural modes rung by valve seating / piston slap / DI needle / machinery impacts, KS4 sensor (26 mV/g, 30 kHz resonance, Bosch Motorsport KS4-P data sheet); damped-sinusoid atoms, no fixed-rate synthesis; magnitudes UNVERIFIED (profile `EngineProfile::knock`) | `knock_sensor::tests::*` |
| Result | Before: no samples → KNKS = 0 → sensor diagnosis → knock limp-home (NL = 1.0 V, spark −7…−15° ATDC at 3000 rpm WOT, 200 Nm). Now: NL 0.2–0.5 V, no limp-home, spark +4…−5° ATDC, 340–385 Nm; RON 95: retard 0.0° on every cylinder; RON 78 (test-cell fuel switch): 99–118 plant knock events per cylinder in 8 s, firmware retard −2…−4.5° per cylinder, followed by its own boost/load reduction | `dyno_run::engine_dyno_wot_boost_and_knock` (VERIFIED-T) |
| Detection vs knock statistics (merge check 2026-11-02) | Detection = KNKS_EGY = KNKS / KNKS_THD (= NL × `ip_knks_thd_fac`) > `c_knks_egy_thd` 1.0; NL is a floating mean of KNKS while no knock is detected (`c_nl_fil_gain` 0.043, on increase `ip_nl_fil_gain_inc` 0.039; frozen only `c_cycnr_knk_det` = 1 cycle after a detection, + `c_nl_inc` 0.039 V). With a plant whose knocking cycles all ring alike (no cycle-to-cycle scatter), NL absorbs sustained knock within ~1 s: only the onset can be detected, and whether it is depends on the background noise of those few cycles (after the fuel-system fix 1860fd3 the engine idles before the dyno pick-up, the VCP hold adapts and the warm intake cam holds — at 74dad1f it limit-cycled 5…57° and made the knock intermittent and variable). Seen as: native core 0.0° retard / Unicorn −2.2° on one cylinder decaying to 0 in 4 s, both with ~175 events/cylinder in 8 s. With log-normal knock-intensity scatter in the plant (`cylinder::KNOCK_SCATTER_SIGMA`, UNVERIFIED σ 0.7): retard on every cylinder, renewed throughout the knocking phase (−3.4° native, −5.6° Unicorn), RON 95 still 0.0° | FR p.22196–22204 (VERIFIED-FR), calibration values VERIFIED-S, runs VERIFIED-T |

### 12.8 Cam phasers and ATIC140 solenoid stages (2026-11-02)

Method: REA `ev_d6f52d10…` (VCP pulse/hold driver), `ev_210cb888…` (VCP handle table, diagnosis ids, DID getters), `ev_81d74936…`
(N428, N522, port flap, diverter, purge, flap sensor); FR VVTI 79 (p.26731), ENLU 27, AIRM/INSY; probe runs with the plant phaser
forced (observation of CAM_VCP).

| Item | Result | Evidence |
|---|---|---|
| N205 / N318 | handle table 0x80042C7C: cam 0 (intake) → BSW PWM 0x11 = G0.LTC24 quad, output **G0.LTC27**; cam 1 (exhaust) → 0x12 = L2.LTC28 quad, output **L2.LTC31**; 150 Hz (`ip_frq_pwm_vcp`); **low = current on** (low fraction of G0.LTC27 = firmware hold duty within 0.1 %); loads = ATIC140#1 stages 0/1 (diagnosis ids 0x100/0x101). Duty ↑ = active direction (intake advance / exhaust retard = CAM_VCP ↑); control = hold duty + timed 90 %/10 % pulses (`c_pwm_act_vcp` / `c_pwm_pas_vcp`) | VERIFIED-T (G0.LTC27), VERIFIED-S |
| Hold feed-forward | `ip_pwm_hld_vcp` @0x8086DCF4 (coil °C × VB): 76.3/60.3/47.6/41.5 % at 8.5/11.1/14/16 V, 90 °C (≈ 6.6 V mean coil voltage = constant current); 28 % at −10 °C, 14 V | VERIFIED-S |
| VVTI observables | CAM_VCP actual 0xD00156B0/B2, set point 0xD001569C/9E (i16, 1/128 °CRK); DIDs 0x201E/0x201A actual, 0x201D/0x2019 set point (0.1 °). Index 0 = intake = cam 0 (G0.GTC28); intake + = advance, exhaust + = retard (rest = locked: intake retarded, exhaust advanced) | VERIFIED-S/T |
| Enable | oil −15…160 °C, VB 8.53–16.15 V, N ≤ 8160 rpm, after `id_dly_st_vcp_end` (0.7–2.5 s); no oil-pressure input | VERIFIED-S |
| Other stages | ATIC140#1: 2 = N428 (energised = low oil-pressure stage; P164E = electrical open/short of this stage), 4 = N522 (energised = jets off, `lc_oil_2_pist_ivs`), 10 = N249, 11 = N316 port flaps (!PORT_DEAC: energised = flaps **open**); flap sensor ADC2.CH8 (closed 1.196 V / open 3.799 V); N80 purge = G0.LTC7 (10 Hz, stage 9, not decoded: no plant consumer); BSW channels 8/0xD are the electric coolant pumps (CWP), the fan is channel 0xF (G0.LTC15) — §12.1 "0xD = fan" is wrong | VERIFIED-S |
| Plant | phaser hydraulics now per `ValveGear::phaser` (`PhaserHw`): coil current d·V_B/R(T) against a null fitted to `ip_pwm_hld_vcp`, spool overlap, response 15 ms, 800 °CA/s hot at 3 bar, viscous slow-down (−20 °C ≈ 5× slower = `ip_vel_ivs_*`), oil-pressure dependence; port-flap actuator lag + sensor from the profile | `valvetrain::tests::phaser_holds_at_the_firmware_hold_duty_and_moves_at_its_speed` |
| Result | measurement chain exact (firmware CAM_VCP = plant ± 1°). Oil at 30 °C: intake reaches its 40° set point and holds it (17/17 samples within 2°), exhaust reaches 8/9° (`dyno_run::engine_dyno_cam_phasing`). **Warm engine: limit cycle ±15°** — the oil-temperature sender is not emulated, the firmware runs on its substitute TOIL ≈ 28–30 °C (DID 0x202F) for the coil temperature, so its hold duty (≈ 0.32 + adaptation) is that of a cold coil while the plant's coil is at 90 °C (needs 0.476) | VERIFIED-T |
| Commanded hold duty (2026-11-02) | Holding PWM (FR VVTI 79.16) traced with observation probes at 0x8017A0D4..0x8017A178 (30 °C engine, idle and 3000 rpm WOT): x of `ip_pwm_hld_vcp` = coil substitute temperature TMAG 24 °C (FR 79.15: from TOIL, vehicle speed, ambient), y = VB 14.2 V → 37.05 %; + adaptation table value −3.4 % intake / −4.7 % exhaust present from the first activation (bench ECU NVM); + `ip_fac_pwm_hld_ofs_vcp[0]`(TMAG) 0.52 × `ip_pwm_hld_ofs_vcp[0]`(rpm, mg/stk) — intake map −5…−10 % (−7 % over 1000–4000 rpm), exhaust map 0 → intake hold ≈ 30 % (idle) / 29 % (WOT). The plant's null had been fitted to the base map alone (38 % at a 30 °C coil): ~8 % of duty above what the firmware holds with, so the cam ran back between the 90 % pulses and only the hold adaptation could catch it (Unicorn: converged; native core: limit cycle → `engine_dyno_cam_phasing` 6/17 at 9c90b9d, 12/17 at 74dad1f and on a pre-merge reconstruction). Also `ip_vel_ivs_act_vcp` expects 0.25–0.28 (0 °C) / 0.53–0.91 (40 °C) of the hot phaser speed, the plant had 0.6 / 0.95. Plant now: null = base + offset × factor + NVM adaptation, viscous slow-down fitted to `ip_vel_ivs_act_vcp` (`profiles::PhaserHw`) → 17/17 (Unicorn) and 16/17 (native), warm intake cam held on both cores | VERIFIED-S/T (probes), REA rizin `ev_4f6b4dff…` |
| Board defects found | LTCA2 PWM cells run at 0.625× the commanded frequency (L2.LTC31: 93.75 Hz for a 150 Hz command; duty unaffected); `EngineIo::out_lines_of` panics for kernel 2; MSC1 MMXCTR field decode inconsistent (GPTA→ATIC140 stage 0/1 never switches) — not fixed. **§12.9: the 0.625× reading is not reproduced (single-period artefact of the VCP pulse/hold duty changes, both kernels); `out_lines_of` fixed** | VERIFIED-T (separate agent) |

### 12.9 Oil temperature, SENT / rotary coolant valve, oil-pressure switches, board defects (2026-11-02, eighth pass)

Method: REA (rizin) evidence `ev_a7c68729…` (DID 0x202F handler), `ev_b26793e1…`/`ev_dcb0c50b…`/`ev_d5db9aba…` (TOIL selection),
`ev_88c4ee1b…` (TOIL_MDL init), `ev_e4b275fe…` (SENT ISRs), `ev_5cc69a27…` (SENT channel table), `ev_fa3ffb3f…` (SENT driver
vtable), `ev_009e5506…` (ENTE RVC sensor signal), `ev_3f6b312e…` (ATIC111 bridge table), `ev_5371471b…` (oil-pressure switch
getters); FR via `fr_tool.py`; DAMOS values from the image; write watchpoints / MMIO logs / DSPR diffs of a temporary diagnostic
harness (not in the tree); plant runs in the studio session (native core).

| Item | Result | Evidence |
|---|---|---|
| DID 0x202F | handler `0x801BC33C`: (byte `[a9+0x5AE]`)·10 + 2331 → 0.1 K; a9 = 0xD000C000 → **TOIL = 0xD000C5AE** (°C + 40). `log_tools::channels` had 0xD000C6A3 (corrected) | VERIFIED-S/T |
| TOIL source | **SC8S50 uses the oil-temperature model, not the sender**: TOIL_CLC (`0x808DF3E4..0x808DF4A2`, FR 27.7.3, p.7288–7293) tests CLF_TOIL_SRV_INTL_DI_CONF (`0x80804F82` = 1, bit 0 "disables TOIL_SRV_INTL usage always") → TOIL_RAW = TOIL_MDL (`0xD000C5AF`), then a gradient limit of `c_toil_grd_lim` 1 °C/s. TOIL_SRV_INTL (sender, `0xD000C5B2`, °C + 60) is only used for the WIV/oil-level functions | VERIFIED-S/FR |
| TOIL_MDL start value | init `0x800DBD68..0x800DBDB8` (FR 27.9.1.1): TOIL_MDL = TOIL_ST (`0xD00189A2`) from `0x800DBA48`: warm-start path from **TOIL_STOP** (`0xD000C5B7`, NVM; 24 °C in the bench DFlash) + `ip_toil_st_add`(TCO_ST, TCO_STOP) → 30 °C; the TCO-based path needs TOIL_STOP > `c_toil_ini_lim` 87 °C. The engine-off time does not change it (300 s and 2 h: TOIL_MDL 30 °C). Running at 3000 rpm WOT the model rises to 78 °C in 150 s | VERIFIED-S/T |
| Cam phaser hold duty | the coil substitute temperature of VVTI 79.15 (FR p.26921–26923) = f(TOIL, VS, TAM) → the "cold coil" hold duty on a hot plant is the firmware's own model state, not a missing sensor. A hot bench start with this ECU is inconsistent with its NVM history (TOIL_STOP 24 °C) until the firmware's model has warmed. `dyno_run::engine_dyno_cam_phasing_hot` (plant 90 °C, 3000 rpm WOT, 240 s, native): intake cam error per 20 s window — TOIL 30–54 °C: max 22–32°, 3–4/40 samples within 2° (limit cycle); TOIL 77 °C: max 0.2°, 40/40; TOIL 86–87 °C: max 2.7°, 49/61 within 2°, all within 3° | VERIFIED-FR/T |
| Oil level / temperature sender G266 (superseded: §12.11 — decoded and emulated) | wired to the ECU (`lc_loil_sens_sys_conf` = 1), thermal (TOG) PWM sender, FR ENLU 27.3 (p.7193–7208): BSW measures low/high times at each rising edge. **Input = IN12 = P3.4** (DS p.36) → G1.LTC40 quad (CLK7 period/pulse capture, struct `0xD0006A80`): a square wave on IN12 changes T_EDGE_FALL_LOIL `0xD000C5AD` and the ENLU variables, on IN20/IN51/IN38 (the other three CLK7 quads, structs `0xD0006A70/50/60`) it does not (deterministic DSPR diff). **Not emulated**: the TOG pulse code (reference 48–52 ms, temperature 25–88 ms, cool-off time = level) is not decoded yet; without it the firmware sets OIL_SENS_EL (DTC 0x3B42) and after 30 s OIL_SENS_PLAUS (0x3B43) | VERIFIED-T (pin), FR/DAMOS (protocol bounds) |
| SENT module | `0xF0321000`, 8 channels (DS p.21), FDR 0x43FF; logical channels 0/1/2 = hw 0/2/3, 6 data nibbles, no serial messages; only hw 0 enabled. Data ISR (SRPN 0x36, SRC `0x9FC`) reads RDR `0x080+4·ch` (D1 in bits 3:0) and RSR `+0x0C` (CRC 3:0, status 11:8), acknowledges `+0x28`; error ISR (0x37, `0x9F8`) masks INTSTAT with 0x13E0. The node routing INP = 0x100000 → data on node 0 = SRC 0x9FC is confirmed by the running firmware. Model `simemu_tc1791::sent` (module docs: register names/bits HYP, tick decode HYP) | VERIFIED-S/T |
| N493 position sensor | INFR GetDigRvcSens `0x800A5D9C`: DIG_SIG_RVC = D1<<8 \| D2<<4 \| D3, status nibble bit 0 → sensor error, driver CRC flag → bit 1, other receive errors → bit 2 (FR 34.22, p.9081); ENTE `0x8017DD30`: DIG_SIG_RVC `0xD00124B2`, BOL/TOL `0xD00124AE/B0` (learned 140/3957 in the bench NVM; defaults `c_dig_rvc_bol/tol` 145/3948), RVC_AV `0xD0012574` (1/64 °, 0..263°) | VERIFIED-S/T |
| N493 actuator | ATIC111 bridge 1: BSW PWM channel 1 (G1.LTC28 quad, output G1.LTC31), direction P4.5, disable P13.9 (entry `0x8004F47C`); **P4.5 high (firmware positive duty) = towards smaller angle** — with this sign the firmware's position loop holds RVC_AV within ±1° of its set point while it moves the valve 170° → 58° (radiator opening as TCO rises at WOT); with the opposite sign it drove the valve into the stops at 100 % | VERIFIED-T |
| Plant | `simemu_physics::cooling_valve`: motor + self-locking worm gear, stops 0°/263° (FR p.9082), SENT counts 145..3948 (the firmware's default adaptation values), radiator branch 0° open → 85° closed (UNVERIFIED), rest 160°; drives `thermal`'s radiator flow (the wax fail-safe stays) | `cooling_valve::tests::*` |
| Result | RVC_DIG_COM (DTC 0x3C70) no longer set (engine_start_scenario fault memory A/B); STATE_DIAG_RVC_SENS 0; ~1.6 kHz frames, no receive errors (`tests/sent_rvc.rs`, both cores) | VERIFIED-T |
| Oil-pressure switches (superseded: §12.11 — P6.14 = low, P7.5 = high switch) | ENLU ACQ `0x808DF9E0` (FR 27.12, p.7348; `c_idx_cod_poil_det_conf` 0x7FF enables both): INFR `0x800A5A78` = **P6.14** → `0xD0000E4A`, `0x800A5A70` = **P7.5** → `0xD0000E4B`, raw pin levels; `0x800A5A80` = !**P6.15** (piston-jet switch, ENLU plausibility `0x808DE4F2`). Fed from the plant's gallery pressure in `studio::io_air` (high / low switch assignment by RAM order HYP, normally-open to ground and the EA888 Gen3 nominal thresholds 2.85 / 0.7 bar UNVERIFIED); no oil-pressure DTC in the start scenario. P6.15 is not fed (no jet-gallery pressure output in the plant) | VERIFIED-S (pins) |
| "LTCA2 PWM at 0.625×" | **not reproduced**: L2.LTC31 and G0.LTC27 (both 150 Hz VCP quads, identical CTR words) run at 150.00 Hz on both cores, bare board and running session; edges per second drop on *both* kernels (123–146/s) only while the firmware's pulse/hold phases change the duty, and a single high+low measurement across such a change reads 81–130 Hz on either kernel — the source of the 93.75 Hz reading. No clock-path change in `gpta.rs`; determinism digests unchanged by this item | VERIFIED-T |
| `out_lines_of` | panicked for LTCA2 (kernel 2 has no decoded output multiplexer); returns no lines now | `engine_io::tests::ltca2_has_no_decoded_output_lines` |
| MSC1 multiplexer | still UNVERIFIED: GPTA0 OMCR routes the intake VCP cell **G0.LTC27 → GPTA0 OUT88**, whose only consumer is MSC1 ALTINL.0, but MMXCTR10 field 0 = 2 (TC1796: LTCA2); reading 2 as GPTA0 instead would put G0.GTC24 (knock window) on ALTINL.3 (field 2, secondary-air stage) — also implausible. The LTCA2 output multiplexer (first 16 of its 32 MRA words) must be decoded to settle it; the VCP stages stay tapped at the cells | VERIFIED-T (OMCR), open |
| Studio tester ISO-TP | with the SENT receiver running, the Unicorn core lost every multi-frame UDS answer after a request at 1.50 s (`engine_run::sensor_characteristics_round_trip_through_the_firmware`): the ECU sent the First Frame, the tester's Flow Control (injected in the same 1 ms exchange) was accepted by the receive MO, no Consecutive Frame followed — the FC reached the ECU before its ISO-TP sender waited for one. The tester now answers one exchange later (`session::bridge_can_frames`), as a real tester does after the bus time and its N_Br; native/baseline were unaffected | VERIFIED-T |
| BSW channel 0xD / 0xF | 0xD = electric coolant pump (CWP), not the radiator fan (§12.1 corrected). **0xF = radiator cooling fan J293 (ECF)**: BSW table `0x80042400` entry `0x800425A4` (`ev_828823eb…`), setters `SetPwmDucyEcf` `0x800A6200` (`ev_f0ee78f7…`) and `SetPwmFrqEcf` `0x800A63C8` (`ev_9d60cc6d…`) → G0.LTC12 quad (`CLK6`, period `0x1B77` = 7031 → 50.0 Hz), output **G0.LTC15**, **high = low-side stage on** like EFP 0xE (standby `26/256` = 10.156 % → G0.LTC15 high fraction 10.16 %), J293 curve 26/256..230/256 → 0..100 % (`fans_nengcorfan1var2_m_vw`, `fans_rfan1devoutadj_t_vw`, ceiling `fans_rrgnmaxcodem_c_vw` 253/256; FR ENTE 34.20/34.21 pp. 9058–9076, CUSF 11.56 pp. 3340–3350) — decoded in `studio::io_air` → `EcuOutputs::fan_duty` | VERIFIED-S/T/FR |
| N80 purge valve | G0.LTC4 quad (CLK6, period 35156 = 10 Hz), output G0.LTC7, ATIC140#1 stage 9: not decoded (no plant canister/purge path; whether the purge diagnosis needs one is not checked) | open |

#### 12.9.1 Rotary coolant valve monitors RVC_DIF / RVC_JAM_RNG and the bridge PWM decode (SC8O40 264H, 2026-11-07)

Trigger: `fault_audit_scenario` with a 5 °C soak on `bench_s50` (264H) — P00B7 `RVC_DIF` (0x457D) + `RVC_JAM_RNG_0` (0x3C6E),
status 0x27; not on `octavia_fl` in the same run. Method: rizin on a 264H ELF (ASW globals `0x808835CC`: a0 = 0xD0018000,
a1 = 0x80808000, a9 = 0xD000C000), FR ENTE 34.25/34.67/34.70 via `fr_tool.py`, DAMOS SC8O40 values from the image, read-only
RAM / plant / GPTA-tap print in the studio exchange loop (temporary, not in the tree).

| Item | Result | Evidence |
|---|---|---|
| RAM (264H) | RVC_AV 0xD001260A, RVC_SP 0xD0012612, RVC_DIF 0xD001260E, RVCPWM 0xD0012618 (s16, 0x4000 = 100 %), LV_RVC_CUR_ON 0xD0000FF8, STATE_RVC_AD [a9+0x7E8] = 0xD000C7E8, TCO for the jam condition [a9+0x7F0] (°C + 40), all angles 1/64° | VERIFIED-S/T |
| RVC_DIF (FR 34.70 p.9426–9427) | `0x800DBEF4`, 10 ms: with LV_IGK, STATE_RVC_AD ∈ {END, TOL_END} (5/6), no jerk (0xD0000F5A) and FID 0x2D7: RVC_SP_DIF = \|SP − PRED\|, thd = `id_rvc_psn_thd`(RVC_SP_DIF) (@0x808227F2, axis 0x8082272E, step lookup 0x8008DBE0), symptom \|PRED − AV\| > thd; PRED = low-pass(`c_crlc_rvc_av_pred` 0.04 per 10 ms ≈ τ 0.25 s) of SP; error index 0xF9. RAM: PRED 0xD001662C, thd 0xD0016636, LF_STATE_DGO_RVC_DIF 0xD00165A2 | VERIFIED-FR/S |
| RVC_JAM_RNG_i (FR 34.67 p.9411–9416) | `0x800DBD0C`, 10 ms, i = 0..3 over `c_rvc_av_bol/tol_rvc_jam_rng` [0, 100.0, 135.0, 240.0] … [100, 135, 240, 270]°: condition = AV in (BOL, TOL] ∧ no reset/inhibit (FID 0x2D9 + i) ∧ (TCO > `c_tco_thd_rvc_jam_rng` 11 °C ∨ STATE_RVC_AD ≥ 5); pending = LV_RVC_CUR_ON ∧ \|REF − SP\| > `c_rvc_dif_sp_rvc_jam_rng` 5° ∧ \|REF − AV\| ≤ `c_rvc_dif_av_rvc_jam_rng_sym` 4° (+ ofs 0); symptom after `c_t_dly_rvc_jam_rng_sym` 0.5 s pending; REF = AV whenever the valve has moved > 4° (pass). RAM: REF[i] 0xD001663A, timer[i] 0xD0016670 (10 ms ticks), LF_STATE_DGO_RVC_JAM_RNG[i] 0xD00125BC | VERIFIED-S/FR |
| Power stage | `0x800AA3C0`: LV_RVC_CUR_ON off after `c_t_deac_rvc_opm` 0.1 s (@0x808066D4, timer 0xD0016650) when \|RVC_DIF\| < `ip_thd_rvc_dif`(AV) 0.8° and \|RVCPWM\| < `ip_thd_rvcpwm`(AV) 10–12 %; RVCPWM → INFR `0x808A3144` → bridge 1 driver (vtable `*(0xD00007DC)`); stage on/off `0x808A30A0` / `0x808A3084` | VERIFIED-S |
| Failing sequence (bench_s50, 5 °C) | 74.5 s: valve reaches SP 88.75°, stage off, G1.LTC31 static low. 87.10 s: SP → 114 → 134.5°, LV_RVC_CUR_ON = 1, RVCPWM −100 %, **G1.LTC31 static high since 87.098 s** — but the decoded `EcuOutputs::rotary_valve_duty` = 0.0, the plant valve stays at 92.3° (firmware AV 89.45°) → 87.61 s JAM_RNG_0 = 3 (fail), 88.26 s RVC_DIF = 3 | VERIFIED-T |
| Root cause | board glue `studio::io::ltc_duty`: duty = high / (high + low) of the *last completed phases*, static only when quiet for > 1.5 such periods. After the stage had been off, the last completed low phase was the 12.58 s pause, so on the switch to static high the "period" was 12.58 s and the duty 0.13 ms / 12.58 s ≈ 0 for up to 19 s. The valve got no voltage while the firmware drove 100 % | VERIFIED-T |
| Why not octavia_fl | same 264H calibration and decoder; the vehicle (`Vehicle::Bench` 1385 kg / Cd 0.29 / r 0.3115 m vs Octavia A7 FL 1335 kg / 0.302 / 0.307 m) only changes the drive trajectory, hence the level of G1.LTC31 when the stage switched off and how long it stayed off. In the octavia_fl run the 86.09 s move was decoded as +100 % (output state before the stop not printed — HYP: it stopped high, so the static rule applied) | VERIFIED-T (bench_s50), HYP (octavia_fl) |
| Fix | `io::ltc_duty_bounded` (used for N493): a completed phase longer than the slowest PWM period the firmware configures on an LTC quad (G0.LTC4 quad, N80, 10 Hz = 100 ms, §6.4) is not a PWM phase → the output is static at its current level; a resumed PWM is misread for at most one of its periods. The same defect is in `ltc_duty` for ETC, wastegate, cam phasers, EFP and λ heaters; switching them over changed only the wastegate in a decisive way (`octavia_is38`, 5 °C: LTCA2 LTC35 static high for 85 ms at cranking, old 2 % / new 100 %; with the new rule on that cell alone the engine stalls at 3.2 s, deterministic, 2 of 2) — left to the air-path work | VERIFIED-T |
| Plant note (not changed) | `cooling_valve::RotaryValve::step` tests stiction after dividing the drive impulse by the implicit back-EMF factor (1 + h·k²/R/J ≈ 1.21 at h = 0.2 ms), i.e. breakaway at ≈ 1.21 × friction; at ignition-on the valve parked ≈ 0.9° from SP with 17 % duty. Not a cause of these faults; friction and motor constants are UNVERIFIED anyway | VERIFIED-T |


### 12.10 Режимы сгорания EA888 Gen3: MPI и FSI (DI), восьмой проход (2026-11-02)

Метод. FR только через `scripts/fr_tool.py` (CBMD 7, FMSP 41, INJR 50, IGSP 48, IGRE 47, TQSP 72.14, ENSS 33.13,
EXTC 38.5; базовая версия FR — `SC8L50Y0`, константы `NC_*` в образах могут отличаться). DAMOS: `SC8S50.json` на
доноре `259J` / стоке `8V0906264K__0003.bin` и `SC8O40.json` на стоке `8V0906264H__0005.bin` (264H, семейство CJSA); адрес в памяти =
`0x80800000 + адрес DAMOS`. Код: дизассемблирование образа 259J / 264K (rizin-листинг), декомпиляция Ghidra 12.1
(язык `tricore:LE:32:tc176x`) функций `0x800B8D6C` (FMSP 41.19, S50) и `0x8009075C` (тот же модуль, 264H). REA этот ELF
(EM_TRICORE) не открывает (`unsupported ELF architecture`), поэтому Ghidra запускалась напрямую (analyzeHeadless).
Глобальные базы TriCore: `a0 = 0xD0018000`, `a1 = 0x80808000`, `a9 = 0xD000C000` (как в `aref.py`, совпадают с
адресами DAMOS). Метка **PUB** — публикация VW/Audi о серии EA888 Gen3: описывает двигатель, а не эти калибровки.

Критерий правильности для плёнки во впуске — совпадение с моделью и режимами самой заводской прошивки по FR/коду/калибровке.

Итог в одной фразе: по обеим заводским калибровкам последовательность — **сначала FSI** (пуск, после пуска, прогрев,
нагрев катализатора), **потом MPI** (тёплый холостой ход и частичная нагрузка), **снова FSI** на высокой нагрузке; одновременная
работа MPI+DI в рабочей точке в этих калибровках не запрашивается. Формулировка «сначала MPI, потом FSI» источниками
не подтверждается.

#### 12.10.1 Как прошивка выбирает режим (CBMD)

| Звено | Что делает | Доказательство |
|---|---|---|
| Запросы | Модули ставят/снимают запросы `ACTION_CBMD_SetReq/ClrReq(IDX_REQ)`: TQSP «Torque based combustion mode request» (рабочая точка), ENSS (пуск), EXTC (нагрев катализатора), FUSL, KNCK, MISF, GEOL | FR p.62 (список пользователей `ACTION_CBMD_SetReq`), p.25696, p.8559, p.10985 (VERIFIED-FR) |
| Приоритет | Из активных незапрещённых запросов выигрывает максимальный `C_NR_CMB_MOD_PRI_REQ[req]`; режим = `NC_NR_CMB_MOD_SEL[req]` | FR p.1760–1762; код 259J `0x80188BD6..0x80188C9C`: max-поиск по `0x80818848`, режим из `0x808184E0`, запись `IDX_CMB_MOD_REQ` в `0xD000C373` (VERIFIED-S) |
| Запреты | Битовое поле `LF_CMB_MOD_INH`; запрещённый запрос выпадает, побеждает следующий по приоритету (резервные запросы TQSP `ID_IDX_CMB_MOD_OPP_FLB`) | FR p.1763–1771, p.1756, p.25696 (VERIFIED-FR); проверка «бит запроса = 1, запрет = 0» `and d15,#3; eq d15,#1` в `0x80188C46` (VERIFIED-S) |
| Переход | `IDX_CMB_MOD_TAR` → после задержки `T_DLY_TRAN_CMB_SP` и времени `T_TRAN_CMB_SP` фактор `FAC_TRAN_CMB_SP` = 1 → `IDX_CMB_MOD_CUR` | FR p.1722–1730, p.1772–1775 (VERIFIED-FR) |
| Раздел топлива | `FAC_MFF_MPL_INJ_HOM[4]` по `IDX_CMB_MOD_CUR`: слот 0 = MPI, слоты 1..3 = импульсы DI | FR p.11496, p.11543; код `0x80095DA8` (switch), `0x80096696`/`0x80096906` (VERIFIED-S) |
| Синхронизация | Переключение зажигания синхронизируется с переключением впрыска по цилиндрам | FR p.12585 «the ignition switch has to be synchronized with the injection switch» (VERIFIED-FR) |

Номера режимов (`IDX_CMB_MOD`, 0..38) восстановлены из switch'а `ACTION_FMSP_CalcFacMffSplitType` (259J, `0x80095DA8`):
каждая ветка читает карты раздела с именем режима в DAMOS. Ветки 0/4/14 пишут `FAC = [0, 0x80, 0]` (весь бензин в 1-й
импульс DI, `0x80096696`), ветки 8/18/0xFD пишут `[0x80, 0, 0]` (весь бензин в MPI, `0x80096906`) — VERIFIED-S.
Имена режимов 2, 3, 6, 7, 15–24 взяты по картам, на которые ссылаются ветки, остальные (4–7, 14–17) — по порядку `NC_IDX_*`
в FR p.1661–1682 (HYP).

| `IDX_CMB_MOD` | Режим | Топливо (слоты MPI / DI1 / DI2 / DI3) | Доказательство |
|---|---|---|---|
| 0 | OPP_1 | DI 1 импульс (впуск, гомогенно) | `0x80096696` (VERIFIED-S) |
| 1 / 5 | OPP_2 / SCAV_OPP_2 | DI 2 импульса | ветка читает `ip_fac_opp_2_pls_2` `0x8083649E` (VERIFIED-S) |
| 2 / 6 | OPP_2_S_1 / SCAV_OPP_2_S_1 | DI 1-й + 3-й (поздний, «S») | `ip_fac_opp_2_s_1_pls_3` `0x808365A0` (VERIFIED-S) |
| 3 / 7 | OPP_3_HOMS_S / SCAV_… | DI 3 импульса | `ip_fac_opp_3_homs_s_pls_2` `0x808366A2` (VERIFIED-S) |
| 8 | OPP_MPI_1 | только MPI | `0x80096906`; `C_IDX_TQ_CMB[8] = 4` (VERIFIED-S) |
| 9 | OPP_MPI_2 | MPI + DI1 | `ip_fac_opp_mpi_2_pls_1` `0x8083ABD8` (VERIFIED-S) |
| 10..13 | OPP_MPI_2_S, OPP_MPI_3, OPP_MPI_3_S_1, OPP_MPI_4_S_2 | MPI + DI (2–3 импульса) | карты `ip_fac_opp_mpi_*` (VERIFIED-S) |
| 14..17 | CH_1, CH_2, CH_2_S_1, CH_3_S_2 | DI, нагрев катализатора | 15 читает `ip_fac_ch_2_pls_2` (VERIFIED-S), прочие HYP |
| 18 / 19 | CH_MPI_1 / CH_MPI_2_S | MPI / MPI+DI при нагреве катализатора | 18 = ветка «только MPI», 19 читает `ip_fac_ch_mpi_2_s_pls_3` (VERIFIED-S) |
| 20..24 | CH_SA_* (вторичный воздух) | DI / MPI+DI | порядок ветвей (HYP) |
| 25..38 | пусковые и прочие (ST_*, STST_*) | — | не разбирались |

Таблица «запрос → режим» `NC_NR_CMB_MOD_SEL` (259J `0x808184E0`, 264H `0x80818358`, одинаковая; VERIFIED-S):
запрос 0..7 → режим 0..7; **8 → 0** (холостой ход, `NC_IDX_REQ_IS_OPP_1`, код TQSP `mov.u d4,#8; call SetReq` в
`0x801EB4A6/0x801EB4BE`); **9 → 8** (OPP_MPI_1); 10 → 8 (резервный MPI); 11..15 → 9..13; 16 → 8; 17.. → 14.. .
Приоритеты `C_NR_CMB_MOD_PRI_REQ` (одинаковы в 259J `0x80818848` и 264H `0x808186C0`): запрос 0 → 5, 2 → 8, 3 → 9,
6 → 20, 8 → 3, **9 → 11**, 10 → 4 (VERIFIED-S).

#### 12.10.2 Выбор режима по состояниям двигателя

| Состояние | Режим по калибровке | Условия и значения (259J / 264H) | Доказательство |
|---|---|---|---|
| Пуск, высокое давление | DI, 2 или 3 «S»-импульса (такт сжатия) | `ID_IDX_CMB_MOD_ST_H(TCO_ST)`: 259J `0x80805E38` = 5 (ST_H_4_MPI_S_2) при ≤ −35 °C, 4 (ST_H_3_S_2) при −31.5, 3 (ST_H_3_S, S/S/S) от −25.5 °C, **2 (ST_H_2_S, S/S) от 35.25 °C**; 264H `0x80805DEE`: 3 до 30.0 °C, 2 от 30.75 °C | DAMOS (VERIFIED-S), коды состояний FR p.8558–8561 (VERIFIED-FR). Семантика ID-карты (ступенька по нижней точке) — HYP |
| Пуск, низкое давление | DI 1 импульс гомогенно (ST_L_1) | `ID_IDX_CMB_MOD_ST_L` = 1 при любой TCO_ST в обоих образах; MPI-пуски (2 = ST_L_1_MPI, 3 = ST_L_2_MPI) не используются | DAMOS (VERIFIED-S), FR p.8559 |
| Наблюдение на стенде | 264H на CJSA: пусковые импульсы DI 73°/31° до ВМТ (2 импульса) | §12.6, строка «SC8O40 on CJSA» (VERIFIED-T, прежний проход) |
| После пуска (EXITST) | все ST-запросы снимаются; далее запросы рабочей точки; пока MPI запрещён — DI 1 импульс (OPP_1, запрос 0, приоритет 5) | FR p.8561 «At the EXIT START event all ST and STST combustion modes are deleted»; приоритеты выше | VERIFIED-FR/-S |
| Прогрев: запрет MPI | MPI запрещён (`NC_IDX_INH_MPI_WUP_CST`), пока не выполнено: `ID_TIA_TOIL_MPI_INH(TIA_IM, TOIL)` = 1 с задержкой `C_T_DLY_TIA_TOIL_MPI_INH` = 5 с **и** интеграл воздуха после пуска ≥ `IP_MAF_INT_AST_MPI_INH_THD(TCO_ST)` | Карта (обе одинаковы, `0x808024C4` / `0x808024BE`): 1 при TOIL ≥ 85 °C; при TOIL ≥ 20 °C только если TIA ≥ 18 °C; при TOIL 0 °C — 0. Порог интеграла: 7500 g (−35 °C), 1243 g (0 °C), 330 g (9.75 °C), **0 g от 20.25 °C** | FR p.1708 fig. 7.4.27 (VERIFIED-FR); DAMOS (VERIFIED-S); ориентация осей (x = TIA, y = TOIL по порядку в FR) — HYP; на стенде (§12.6) переход на MPI ≈ 5.5 с при TIA 33–35 °C / TOIL 30 °C (VERIFIED-T) |
| Прогрев: окно MPI по TCO | Запрос OPP_MPI_1 (регион 4 у 259J, регион 5 у 264H) имеет `IP_TQI_THD_MPLH_OPP_MAX` = 0 при TCO ≤ 21.0 °C → MPI не запрашивается на холодном двигателе | 259J: 0 при 21.0 °C, 120…175 Нм от 21.8 °C; 264H: 0 при 20.2 °C, 120…165 Нм от 21.0 °C | DAMOS `0x8086A096` / регион 5 264H (VERIFIED-S) |
| Нагрев катализатора | DI, 2 импульса (HPDI) | `ID_IDX_MOD_REQ_INJ_CH` = 1 (HPDI) во всей области (оба образа); `ID_NR_IV_PLS_REQ_CH` = 2 (259J; в DAMOS 264H нет); код запроса `ID_IDX_CMB_MOD_REQ_TEG_CH[HPDI][2]` = 4 (оба). Доля позднего импульса `IP_FAC_CH_2_S_1_PLS_3` 0.30–0.40 (259J) | FR p.10985–10997 (VERIFIED-FR); DAMOS (VERIFIED-S). Какой именно режим (CH_2 или CH_2_S_1) кодирует «4» — HYP (CH_2_S_1) |
| **Тёплый холостой ход** | **только MPI (OPP_MPI_1)** | Активны запрос 0 (OPP_1, весь диапазон 0–1024 Нм, приоритет 5), запрос 9 (OPP_MPI_1, TQI 0…120 Нм при 800 об/мин, приоритет 11) и запрос 8 холостого хода (OPP_1, приоритет 3, `C_STATE_SEL_IS_REQ_CMB` = 1). Побеждает MPI. Если MPI запрещён — DI 1 импульс (OPP_1) | TQSP: `C_IDX_CMB_MOD_OPP_STND` 259J {0,2,3,6,9,12} `0x8080C9D5`, 264H {0,2,3,6,7,9} `0x8080C891`; пороги `IP_TQI_THD_MPLH_OPP_MIN/MAX`, `C_N_THD_MPLH_OPP_MAX` (VERIFIED-S); логика приоритетов (VERIFIED-S). TQI холостого хода < 120 Нм — HYP (индицированный момент ≈ потери) |
| Частичная нагрузка | MPI до TQI 120 Нм (800 об/мин) … 175 Нм (1500–3500) у 259J, N ≤ 3600; у 264H 120…165 Нм, N ≤ 8000 | Выше — запрос MPI снимается | DAMOS (VERIFIED-S) |
| Высокая нагрузка | DI 2 импульса (OPP_2_S_1, 1-й + поздний 3-й) | 259J регион 1: TQI ≥ 110…350 Нм (TCO 84.8 °C), N ≤ 4000; доля 3-го импульса `IP_FAC_OPP_2_S_1_PLS_3` 0.5 до MAF 400 мг, 0.2–0.3 при 750–1050 мг. 264H регион 1: TQI ≥ 110…350 Нм при TCO ≥ 70.5 °C | DAMOS (VERIFIED-S) |
| Высокая нагрузка, холоднее | DI 3 импульса (OPP_3_HOMS_S) | 259J регион 2: TQI ≥ 155…175 Нм, N ≤ 2600, только при TCO < 84.8 °C; у 264H регион 2 выключен (`C_N_THD_MPLH_OPP_MAX` = 0) | DAMOS (VERIFIED-S) |
| Продувка (scavenging) | DI 2 импульса SCAV_OPP_2_S_1, приоритет 20 | N ≤ 2500; доп. условие TQI_SP ≥ `C_TQI_SP_MIN_REQ_CMB_SCAV` 150 Нм (259J) / 250 Нм (264H), `C_NR_CASE_CMB_MOD_TQI_REQ_SCAV` = 4 | DAMOS (VERIFIED-S); как именно случай 4 связан с регионом — не проверено |
| Полная нагрузка | MPI дополнительно запрещается при TQI выше предела MPI + `C_TQ_OFS_CMB_MPI_FL` 50 Нм и с поправкой на давление/температуру окружающей среды | FR p.25701–25703 (VERIFIED-FR); значения `IP_TQ_OFS_INH_MPI_AM_COR` не разбирались |
| Отсечка (PUC) и возобновление | Режим не меняется, впрыск отсекается. Плёнка во впуске за отсечку испаряется (`LC_MFF_ADD_WF_INH_INJ_ENA` = 1: при невыполненном впрыске MPI добавка в плёнку = 0). Запрет MPI после PUC (7.4.2.11) снимается по интегралу воздуха ≥ `IP_MAF_INT_THD_CLR_MPI_INH` = 0 г (оба образа) → фактически не действует (HYP) | FR p.1714, p.11673 «During PUC, the manifold wall-film becomes cut down until it has completely disappeared» (VERIFIED-FR); DAMOS (VERIFIED-S) |
| Долгая работа на MPI | Кратковременный переход на DI для промывки (охлаждения) форсунок DI: MPI запрещается, когда интеграл воздуха на MPI > `IP_MAF_INT_INH_MPI_INJ_PURGE(N, MAF)` (4 400…50 400 г), снимается через `C_T_MAF_INT_CLR_INH_MPI` = 10 с работы на DI; участвуют `C_VS_INH_MPI_INJ_PURGE` 20 км/ч, `C_FUP_INH_MPI_INJ_PURGE` 190 бар (259J) / 210 бар (264H), разрешение `ID_MAF_N_INH_MPI_INJ_PURGE` (259J: 0 при MAF ≤ 150 мг/такт, т.е. на холостом ходу выключено; 264H: 1 везде) | FR p.1710–1712 (VERIFIED-FR); DAMOS (VERIFIED-S). Направление сравнения со скоростью и роль FUP — не проверено |
| Детонация | MPI запрещается, если коррекция УОЗ от детонации ниже `IP_IGA_KNK_MPI_THD` (−0.4…−3 °ПКВ), гистерезис −2.25 °, задержка 3 с | FR p.1703 (VERIFIED-FR), DAMOS (VERIFIED-S) |
| Аварийный | Отказ форсунок одной системы → запрет её режимов; резервные запросы TQSP (`ID_IDX_CMB_MOD_OPP_FLB`: у 264H резерв OPP_1 = запрос 10 → MPI, резерв MPI = OPP_1) | FR p.1704–1706, p.25711 (VERIFIED-FR); DAMOS (VERIFIED-S) |

Режимы со смешанной подачей MPI+DI в рабочей точке (OPP_MPI_2…4) в обеих калибровках не используются: у 259J единственный
такой запрос (12 → OPP_MPI_2_S) стоит в регионе с `C_N_THD_MPLH_OPP_MAX` = 0, у 264H в списке нет (VERIFIED-S). Карты для них
откалиброваны (у 259J `IP_FAC_OPP_MPI_2_PLS_1` = 0.40 доля DI, предел `_MAX` 0.50, `IP_RATIO_OPP_MPI_2_WUP` = 0), но не
вызываются.

PUB (Audi SSP 606 «Moteurs TFSI de 1,8l et 2,0l Audi de la ligne EA888 (3ème génération)», 11/2012, франц. изд., с. 49–51):
на холодном двигателе (охлаждающая жидкость ниже ≈ 45 °C, с учётом температуры масла) — всегда DI; каждый пуск — DI;
пуск — тройной впрыск в такте сжатия; нагрев катализатора — двойной впрыск (впуск + сжатие), позднее зажигание, заслонки
закрыты; тёплый двигатель (> 45 °C), частичная нагрузка — MPI; более высокая нагрузка — двойной DI; при долгой работе на MPI
кратковременно включается FSI против закоксовывания форсунок; при отказе одной системы работает другая. Audi Technology
Portal «1.8 TFSI»: «Indirect injection supplements FSI direct fuel injection in the part-load range… FSI fuel injection is
active when starting and at higher loads». Схема совпадает с калибровками качественно; численный порог ≈ 45 °C в SSP не
совпадает с окном прошивки (TCO ≈ 21–22 °C + условие TIA/TOIL) — SSP описывает калибровку Audi, не 259J/264H.
Не просмотрены: VW SSP 522 (EA888 Gen3 2.0 TSI); Heiduk, Kuhn, Stichlmeir et al., «The new 1.8 l TFSI Engine from Audi,
part 2: Mixture Formation, Combustion Method and Turbocharging», MTZ worldwide 72 (2011) 58–64 (платный доступ); доклады
Венского моторного симпозиума по EA888 Gen3.

#### 12.10.3 Переход между режимами

| Пункт | Значение | Доказательство |
|---|---|---|
| Число уставок перехода | `NC_NR_CMB_SP` = 1 (карты `ID_T_TRAN_CMB_SP`/`ID_T_DLY_TRAN_CMB_SP` имеют 1 столбец) → факторы `FAC_TRAN_CMB_SP_STD/_TQ_ADD/_TQ_SLOW` берутся из одного `FAC_TRAN_CMB_SP` | DAMOS (VERIFIED-S), FR p.1774 fig. 7.10.5 (VERIFIED-FR) |
| Время перехода | `T_TRAN` = 0 во всех наборах (ступенька) | `ID_T_TRAN_CMB_SP` 259J `0x808024BA` (VERIFIED-S) |
| Задержка | набор 1 = 1.0 с, набор 2 = 2.0 с, набор 3 = 2.5 с; выбор `ID_IDX_T_TRAN_CMB_SP[CUR][TAR]` (259J `0x80817203`, 264H `0x8081711B`, одинаковы): OPP_1 ↔ OPP_MPI_1 — наборы 1 и 2 | DAMOS (VERIFIED-S). Какая ось CUR, какая TAR — HYP: при x = TAR переход DI → MPI задерживается на 1.0 с, MPI → DI на 2.0 с |
| Ступенчатое переключение топлива | Промежуточный режим RAMP_MPL_TRAN с линейным переносом доли MPI/DI в обеих калибровках не используется: `ID_STATE_CMB_MOD_SEL_MPL_TRAN` = 0 везде, `IP_FAC_MFF_MPL_INJ_RAMP_INC/DEC` = 0 (259J) | FR p.11543–11550 (VERIFIED-FR); DAMOS (VERIFIED-S) |
| Прочие пути перехода | `ID_STATE_CMB_MOD_SEL_TQ_TRAN` (через модель момента), `_FAST_TRAN`, `_GB_INTV`, `_PRI_INH_TRAN` = 0 везде (259J) | DAMOS (VERIFIED-S) |
| Поправка топлива при переходе MPI ↔ SDI | `IP_FAC_MFF_ADD_TRAN_MOD_N_MAF/_TCO` (FMSP 41.24); `C_CTR_TRAN_CMB_MAX` = 0 (259J) → счётчик перехода сразу останавливается, поправка фактически выключена (HYP); сами карты в DAMOS отсутствуют | FR p.11741–11743 (VERIFIED-FR), DAMOS (VERIFIED-S) |
| Обновление плёнки при смене режима | `LC_MFF_WF_CYL_UPD_MOD_TRAN` = 1: при смене `IDX_CMB_MOD_CUR` пересчитываются все цилиндры (`LF_MFF_WF_CYL_UPD` = 0xFF); набор калибровок плёнки `IDX_CAL_WF = ID_IDX_CAL_WF[IDX_CMB_MOD_CUR]` = 1 для режимов 8–13, 18, 19, 22–24, 38 (MPI) и 0 для DI | код 259J `0x800B8D6C..0x800B8DE6`; DAMOS `0x808376C2`/`0x80837270` (VERIFIED-S) |

#### 12.10.4 Модель плёнки во впуске (FMSP 41.19) — формула для сравнения с физикой

Модуль «Wall film compensation» ведёт две плёнки во впуске (IM, источник — MPI) и две в камере сгорания (CMBC, источник —
DI) на каждый цилиндр (`NC_NR_IM_WF_FIL` = `NC_NR_CMBC_WF_FIL` = 2, `NC_NR_CAL_WF` = 2; размеры массивов по шагу адресов —
VERIFIED-S). Работает после окончания пуска (`LV_ST_END` = 1), периодичность SEG/HLFSEG, обнуление при RST/EXITST
(FR p.11636–11637, VERIFIED-FR). Плёнка цилиндра обновляется, когда его бит стоит в `LF_MWF_CYL_UPD =
ID_MWF_CYL_UPD[IDX_CAL_WF](CTR_SEG_HALF)`: набор 0 — `[4,0,8,0,1,0,2,0]`, набор 1 — `[8,0,1,0,2,0,4,0]` → **один раз за
рабочий цикл на цилиндр** (VERIFIED-S, обе калибровки одинаковы).

Обновление IM-плёнки цилиндра c, плёнки i ∈ {0, 1} (код 259J `0x800B9DC6..0x800B9F40`, 264H `0x80090F32..`; FR
fig. 41.19.40 p.11663):

```
x_i = IP_FAC_MFF_WF_MPI[i][IDX_CAL_WF](N_32, MAP) · IP_FAC_MFF_WF_IM_TEMP_COR(TIA_IM, TCO)        (FAC_MFF_WF_MPI)
f_i = IP_FAC_MWF_VAP_IM[i](N_32, MAP)             · IP_FAC_MWF_VAP_IM_TEMP_COR(TIA_IM, TCO)       (FAC_MWF_VAP_IM)
a_i = x_i · m_MPI           m_MPI = MFF_HOM_SP[c][0]; a_i = 0, если MPI-впрыск не выполнен и LC_MFF_ADD_WF_INH_INJ_ENA = 1;
                            если режим цилиндра не менялся (LV_CMB_MOD_CYL_NOT_SWI[c] ≠ 0), берётся a_i прошлого события
                            (MFF_ADD_IM_PREV), новое a_i запоминается
S_i = M_i + a_i
v_i = k_i(S_i) · f_i · S_i  k_i = IP_MFF_VAP_WF_IM[i](S_i)                                          (MFF_VAP_IM)
M_i ← max(S_i − v_i, 0)                                                                             (MWF_IM)
```

Неявно подразумеваемая масса топлива в цилиндре: `m_cyl = (1 − Σx_i)·m_MPI + Σ v_i` (HYP, следует из того, что поправка
строится по разности `a_i − v_i`). Время в модели не участвует: доля испарения задана **на рабочий цикл**, поэтому
эквивалентная постоянная времени масштабируется с оборотами: `τ_i = −T_cyc / ln(1 − k_i·f_i)`.

Поправка `MFF_ADD_WF_CYL` (FR fig. 41.19.33, код 259J `0x800B9xxx`, декомпиляция строки 540–647):

```
DIF_i = wdif±_i · (a_i − v_i(M_i + a_i)),  wdif = IP_FAC_MFF_DIF_VAP_IM_POS/NEG[i](N, MAF)   (MFF_DIF_VAP_IM)
Δm    = w±_AST(TCO, CYC_CAST) · w±[IDX_CAL_WF](N, MAF) · (Σ DIF_IM + Σ DIF_CMBC) / D
D     = sat16(Σ EFF_IM + Σ EFF_CMBC) + (1 − Σx_MPI)·FAC_MFF_MPI + (1 − Σx_DI)·(1 − FAC_MFF_MPI)
```

`EFF` — производная испарения по впрыску, считается конечной разностью с шагом `C_MFF_ADD_EFF_CLC`; в обоих образах
`C_MFF_ADD_EFF_CLC` = 0, тогда код пишет `EFF = 0xFFFF` (≈ 1.0): ветки `0x800B95B8` (IM) и `0x800B9A18` (CMBC) после
`ld.hu d0,[a1]#0x174; ne d0,d0,#0` в `0x800B9294` (VERIFIED-S). Знаменатель при этом насыщается до ≈ 1 и
**Δm ≈ w·Σ wdif·(a − v)** (HYP: насыщение сумм прочитано по декомпиляции Ghidra). `IP_FAC_MFF_ADD_WF_POS/NEG[1]` = 1.0 на
холостом ходу (оба образа). Плавное включение после пуска `IP_FAC_MFF_ADD_WF_AST_POS/NEG(TCO, CYC_CAST)`: 0 до 20 циклов,
1.0 с 40 циклов (259J); 0 до 40, 1.0 с 60 циклов (264H). Деактивация по малой поправке выключена
(`LC_MFF_ADD_WF_DEACT_ENA` = 0, `C_FAC_MFF_ADD_WF_CLC_THD` = 0). Все — DAMOS (VERIFIED-S).

Опорная плёнка (только для инициализации `MWF` на первом сегменте после пуска, FR p.11646, 11660):
`MWF_REF_i = IP_MFF_VAP_WF_IM_IVS[i](a_i / f_i) − a_i` (HYP по декомпиляции). Обратные кривые не согласованы с прямыми:
наклон `IVS[0]` = 2.03, `IVS[1]` = 5.09 (соответствуют k = 0.49 / 0.20), а прямые `k` = 0.80 / 0.15 (обе калибровки,
VERIFIED-S). После пуска на DI `a_i = 0` → `MWF_REF = 0`, поэтому на работу это не влияет (HYP).

**Значения на тёплом холостом ходу** (режим OPP_MPI_1 → `IDX_CAL_WF` = 1; TCO 90 °C — ось TCO кончается на 80.3 °C, берётся
крайняя строка; TIA_IM 35 °C; билинейная интерполяция по DAMOS, скрипт `/tmp/dualinj/film_idle.py`):

| Величина | 259J (SC8S50) | 264H (SC8O40) |
|---|---|---|
| `IP_MFF_VAP_WF_IM[0]` / `[1]` (k, постоянны по массе плёнки) | 0.80 / 0.15 (`0x8083B580`/`0x8083B598`) | 0.80 / 0.15 (`0x8083B12E`/`0x8083B146`) |
| `IP_FAC_MWF_VAP_IM[0..1]` (N 704–800, MAP 300–400 гПа) | 1.00 | 1.00 |
| `IP_FAC_MWF_VAP_IM_TEMP_COR` (TIA 30 / 35 / 40 °C) | 0.816 / 0.822 / 0.828 | то же |
| `IP_FAC_MFF_WF_IM_TEMP_COR` (TIA 30 / 35 / 40 °C) | 0.631 / 0.622 / 0.614 | то же |
| x0 (быстрая плёнка), 704 об/мин·300 гПа / 750·350 / 800·400 | 0.206 / 0.195 / 0.185 | 0.205 / 0.193 / 0.181 |
| x1 (медленная плёнка) | 0.157 / 0.157 / 0.156 | 0.150 / 0.145 / 0.139 |
| Σx | 0.36 / 0.35 / 0.34 | 0.36 / 0.34 / 0.32 |
| k0·f0 / k1·f1 за цикл | 0.658 / 0.123 | 0.658 / 0.123 |
| τ0 / τ1 (704 / 750 / 800 об/мин) | 159 / 149 / 140 мс; 1.30 / 1.22 / 1.14 с | то же |
| Равновесная плёнка M_i / m_MPI = x_i(1 − k_i f_i)/(k_i f_i) | 0.10 + 1.11 | 0.10 + 1.03…0.99 |
| `IP_FAC_MFF_DIF_VAP_IM_POS[0]` / `[1]` (704 об/мин, MAF 98–180 мг) | 1.73…1.61 / 0.65…0.66 | 1.73…1.61 / 0.77…0.78 |
| `IP_FAC_MFF_DIF_VAP_IM_NEG[0]` / `[1]` | **0** / 1.05…1.02 | **0** / 0.92…0.79 |

Проверка на скачке DI → MPI (M = 0, 750 об/мин, 259J): в первом цикле в цилиндр попадает
`1 − 0.195·(1 − 0.658) − 0.157·(1 − 0.123) = 0.796` впрыснутого; поправка `1.73·0.067 + 0.65·0.138 = 0.205` от m_MPI —
совпадает с дефицитом 0.204 (расчёт по формулам выше). На скачке MPI → DI быстрая плёнка (≈ 0.10·m) не компенсируется
(вес NEG = 0), испарение медленной (≈ 0.12/цикл от ≈ 1.1·m) вычитается из DI с весом ≈ 1.0.

Сверка с моделью стенда (`crates/simemu-physics/src/profiles.rs`, `PORT_FILM_EA888_GEN3`: одна плёнка x = 0.30,
τ = 0.19 с при 90 °C):
- `[0][1]` в `ip_fac_mff_wf_mpi[0][1]` — это [плёнка 0][набор калибровок 1] (шаг 0x40 = одна карта 8×8 на набор, 0x80 на
  плёнку; код `0x800B8EC4..0x800B8F9E`, VERIFIED-S), набор 1 = MPI-режимы — выбор верный. Но 0.30 — сырое значение без
  температурной поправки `FAC_MFF_WF_IM_TEMP_COR` ≈ 0.62 при 90 °C; эффективное x0 ≈ 0.19.
- Вторая (медленная) плёнка прошивки (x1 ≈ 0.15, τ ≈ 1.2 с, равновесная масса ≈ 1.0–1.1·m_MPI) в модели стенда отсутствует.
- «0.8 за цикл» в комментарии — это `IP_MFF_VAP_WF_IM[0]` (k0 = 0.8), а не `ip_fac_mwf_vap_im[1]` (= 1.0); эффективная доля —
  k0·f0 = 0.8·1.0·0.82 = 0.66 за цикл → τ0 ≈ 0.15 с при 750 об/мин; для плёнки 1 — 0.12 за цикл → τ1 ≈ 1.2 с.
- В прошивке доля испарения постоянна на цикл (τ ∝ 1/N), температурная зависимость — через `TIA_IM` и `TCO` по картам;
  в модели стенда τ — во времени с Клапейроном–Клаузиусом по температуре стенки.
- Для сравнения «физика ↔ прошивка» на тёплом холостом ходу: модель стенда должна давать в цилиндр ≈ 0.80 скачка в первом
  цикле, ≈ 0.95 через ≈ 3 цикла (быстрая плёнка) и остаток ≈ 0.12·m с τ ≈ 1.2 с; при выполнении этого поправка прошивки
  `MFF_ADD_WF_CYL` компенсирует плёнку без остаточной ошибки λ (HYP, расчёт по формулам; на прогоне не проверено).

Модель плёнки DI (камера сгорания): та же структура с `IP_FAC_MFF_WF_HPDI[i][cal](N_32, MFF_BAS)`,
`IP_FAC_MWF_VAP_CMBC[i]`, `IP_MFF_VAP_WF_CMBC[i]`, источник — Σ импульсов DI (`MFF_HOM_SP[c][1..3]`, код `0x800B9F42..`);
значения не разбирались.

#### 12.10.5 Зажигание и момент при смене режима

| Пункт | Значение | Доказательство |
|---|---|---|
| Индекс модели момента | `C_IDX_TQ_CMB[IDX_CMB_MOD]`: OPP_1 → 0, OPP_2_S_1 → 2, OPP_3_HOMS_S → 3, **OPP_MPI_1 → 4**, CH_1 → 5, CH_2…CH_3_S_2 и CH_MPI_2_S → 6, прочие → 0 (264H: SCAV_OPP_2_S_1 → 2, SCAV_OPP_3 → 3) | 259J `0x80863BEC`, 264H `0x808633D8` (VERIFIED-S) |
| Поправка УОЗ на режим | `IP_IGA_REF_CMB_MOD_COR[IDX_TQ_CMB_CUR]` и `IP_IGA_BAS_CMB_MOD_COR[…]` (N_32 × MAF_FG_CYL); для цели перехода — то же по `IDX_TQ_CMB_TAR` | FR p.12817–12825 (VERIFIED-FR) |
| MPI относительно DI (индекс 4 против 0) на холостом ходу (704 об/мин, MAF 100–125 мг) | 259J: REF 0…+1.5 °ПКВ, BAS 0…+1.5; 264H: REF +0.8, BAS +0.8 (индекс 0 = 0 везде) | DAMOS 259J `0x80840958`/`0x8083DA8C`, 264H `0x80840506`/`0x8083D63A` (VERIFIED-S); знак «+ = раньше» — HYP |
| Когда меняется | В момент смены `IDX_CMB_MOD_CUR`, синхронно с переключением впрыска по цилиндрам; резерв момента (`FAC_TRAN_CMB_SP_TQ_ADD`) идёт тем же фактором (ступенька после задержки 12.9.3) | FR p.12585, p.1774 (VERIFIED-FR); значения T_DLY (VERIFIED-S) |
| Поправка топлива на режим | `IP_FAC_MFF_BAS_MPLH[ID_IDX_CAL_FAC_MFF_BAS_MPLH[mode]]`: OPP_MPI_1 → карта 3; ≈ 1.0 (1.1 только при 700–800 об/мин и MAF 70 мг) | FR p.11742 (VERIFIED-FR), DAMOS 259J (VERIFIED-S) |

Конец впрыска MPI в рабочей точке `IP_EOI_BAS_MPI_OPP(MAF, N)` (°ПКВ, 0…720): на холостом ходу 160 ° (259J) / 125 ° (264H);
от какой метки отсчитывается угол — не проверено (FR p.12951 и далее не разбирался).

#### 12.10.6 RAM-переменные для чтения во время прогона

Адреса из кода (статически); имена — по FR 41.19 / 7.8 и роли в коде. «HYP» — имя присвоено по аналогии. Чтение по UDS
`ReadMemoryByAddress (0x23)` прошивкой не проверено; в эмуляторе доступно наблюдательной пробой (AGENTS §2.1).

| Переменная | Тип, масштаб | 259J (SC8S50) | 264H (SC8O40) |
|---|---|---|---|
| `IDX_CMB_MOD_CUR` | u8, индекс режима (таблица 12.9.1) | `0xD000C36E` | `0xD000C46B` |
| `IDX_CMB_MOD_REQ` | u8 | `0xD000C373` | `0xD000C470` |
| `IDX_CMB_REQ_SET` (победивший запрос) | u8 | `0xD00187B0` | `0xD00187D3` |
| `NR_CMB_MOD_PRI_REQ` (HYP) | u8 | `0xD000C37B` | — |
| `MFF_HOM_SP_CLC[pls]` (HYP), pls 0 = MPI, 1..3 = DI | u16 × 0.02119478 мг | `0xD00135A8` | `0xD0013626` |
| `MFF_HOM_SP[cyl][pls]` | u16 × 0.02119478 мг, `cyl·8 + pls·2` | `0xD0013588` | `0xD0013606` |
| `MFF_HOM_SP_WF_MPI` / `_HPDI` | u16 × 0.02119478 мг | `0xD0017028` / `0xD0017026` (запись `0x800BA37A`/`0x800BA382`) | — |
| Доля MPI `FAC_MFF_MPI` (копия модуля плёнки) | u8 × 1/128 | `0xD00192B1` (источник `0xD000DD43`, HYP: `FAC_MFF_MPL_INJ_HOM[0]`) | источник `0xD000DE87` (HYP) |
| `IDX_CAL_WF` / `LF_MFF_WF_CYL_UPD` / `LF_MWF_CYL_UPD` | u8 | `0xD00192D7` / `0xD00192D9` / `0xD00192DA` | `0xD00192EB` / `0xD00192ED` / `0xD00192EE` (HYP) |
| `FAC_MFF_WF_MPI[2]` / `FAC_MWF_VAP_IM[2]` | u8 × 1/256 / × 1/128 | `0xD00192CD` / `0xD00192D2` | `0xD00192E1` / `0xD00192E6` (HYP) |
| `FAC_MFF_WF_IM_TEMP_COR` / `FAC_MWF_VAP_IM_TEMP_COR` | u8 × 1/128 | `0xD00192CC` / `0xD00192D4` | — |
| `MWF_IM[cyl][fil]` | u32 × 646.804e-9 мг, `cyl·8 + fil·4` | `0xD000AB1C` | `0xD000AC50` |
| `MFF_VAP_IM[cyl][fil]` | u32 × 646.804e-9 мг | `0xD000AADC` | `0xD000AC10` |
| `MWF_CMBC` / `MFF_VAP_CMBC` | u32 | `0xD000AAFC` / `0xD000AABC` | `0xD000AC30` / `0xD000ABF0` |
| `MFF_ADD_IM_PREV[cyl][fil]` | u16 × 0.02119478 мг, `cyl·4 + fil·2` | `0xD0016FEA` | `0xD0017024` |
| `MWF_REF_IM` / `MWF_REF_CMBC` | u16 × 0.04238956 мг | `0xD0017040` / `0xD0017030` | — / `0xD001706A` (HYP) |
| `MFF_DIF_VAP_IM` / `EFF_MFF_ADD_WF_IM` / `MFF_VAP_CLC_IM` (итоговые копии) | s16 / u16 / u32 | `0xD0017016` / `0xD0016FB6` / `0xD000AA9C` | — |
| `MFF_ADD_WF_CYL[cyl]` (поправка на цилиндр) | s16 × 0.02119478 мг | `0xD0013578` (без обнуления: `0xD0016FFC`, HYP `…_DEACT`) | `0xD00135F6` (HYP) |
| `MFF_ADD_WF` (текущий цилиндр) | s16 × 0.02119478 мг | `0xD0013576` | `0xD00135F4` |
| `LV_CMB_MOD_CYL_NOT_SWI[cyl]` / `LV_MWF_INI` | u8 | `0xD000226B` / `0xD0002273` | `0xD00022AA` / `0xD00022B2` |
| Входы плёнки: `N_32` / `MAP` | u8 × 32 об/мин / u16 × 0.0829 гПа | `0xD000C61C` / `0xD0013E9E` | — |
| Измеряемые DID (прежний проход) | `TI_1_MES_MPI` 0x13AC, TOIL 0x202F, TIA 0x1001 | §12.6 (VERIFIED-T) | то же |

Модуль плёнки 264H новее (дополнительные переменные, иная раскладка RAM); адреса 264H получены из той же структуры цикла
(`0x8009075C`, Ghidra) и отмечены HYP там, где роль не подтверждена второй ссылкой.

#### 12.10.7 Не проверено / открыто

- Ориентация осей `ID_IDX_T_TRAN_CMB_SP` (какая задержка 1.0 с, какая 2.0 с) и `ID_TIA_TOIL_MPI_INH` (TIA/TOIL).
- Семантика ступенчатых ID-карт (нижняя точка или ближайшая) для пусковых таблиц и `ID_TIA_TOIL_MPI_INH`.
- Индицированный момент `TQI_REQ_CMB` на тёплом холостом ходу (нужен прогон: прочитать `IDX_CMB_MOD_CUR` и момент).
- Номера режимов 4–7, 14–17, 20–24 (по порядку в FR) и код «4» в `ID_IDX_CMB_MOD_REQ_TEG_CH`.
- Где `MFF_ADD_WF_CYL` прибавляется к уставке (FMSP 41.13/41.14, p.11551–11563) и делится ли поправка по долям MPI/DI.
- Плёнка DI (CMBC) — значения; поправка на возобновление после отсечки (FMSP 41.21, p.11673) — значения.
- Отсчёт угла `IP_EOI_BAS_MPI_OPP`; на модели — момент осаждения порции MPI относительно открытия клапана.
- Публикации: VW SSP 522, MTZ 2011 (Heiduk et al.), доклады Венского симпозиума — не просмотрены.

#### 12.10.8 Plant против модели плёнки прошивки и прогон с прошивкой (2026-11-02)

- Plant: две плёнки на канал (`PortFilmHw`: тарелка клапана — ОЖ + 60 K, стенка — ОЖ; x 0.19 / 0.15, τ 0.15 / 1.2 с в
  опорной точке ОЖ 90 °C, 35 кПа, 3 г/с; зависимость τ — Клаузиус–Клапейрон, ∝ p_im, ∝ ṁ^−0.8). Опорные значения — модель
  этой прошивки (§12.10.4) как идентификация физики канала калибровщиком; прежняя одна плёнка (x 0.30 — сырое значение
  карты без температурного множителя 0.62, τ 0.19 с) давала на ступени DI → MPI недодачу ≈ 10 % в первые циклы, а
  компенсация медленной плёнки прошивкой затем переобогащала.
- Часть впрыска мимо плёнок plant теперь отдаёт в ближайший такт, как модель прошивки (поршневое течение в канале,
  `PortFuel::induct`); прежний перемешанный канал задерживал 25–30 % этой части на следующие такты — λ 2.2–2.7 после
  переключений ЭБУ DI → MPI на сборке DQ250 (docs/SIMOS18_COMBUSTION.md §14). Адреса 264H `MFF_HOM_SP` `0xD0013606` и
  `IDX_CMB_MOD_CUR` `0xD000C46B` подтверждены прогоном (VERIFIED-T, там же).
- Прогон `engine_start_scenario` (SC800S50 259J / `cjxc`, тёплый, 40 с, native): прошивка переходит на MPI на 5.8 с, через
  0.5–3 с λ цилиндров plant 0.97–1.00, двигатель не глохнет; тёплый ХХ на MPI 30+ с в пределах уставки ±80 об/мин
  (docs/SIMOS18_COMBUSTION.md §5). Гипотеза «двигатель глохнет из-за плёнки на переходе DI → MPI» подтвердилась лишь
  частично: главными были (1) пропуски искры в plant при опережении от цикла к циклу (декодированный угол — конец заряда
  предыдущего цикла) и (2) чувствительность момента к УОЗ (MBT plant 6 ° вместо ≈ 15 ° карты при фазах ХХ).
- Остаток STFT +12 % на тёплом ХХ устранён (девятый проход): +1…+2 % (259J) / +2…+3 % (264H) — подогрев свежего заряда в
  канале и состав газа вентиляции картера, разбор модели наполнения INSY 51.13–51.16 и RAM `0xD0017846`/`0xD001784A`:
  docs/SIMOS18_COMBUSTION.md §6.

### 12.11 Oil level / temperature sender G266 and the oil-pressure switches (2026-11-02, eleventh pass)

Method: static RE of the SC800S50 image with REA/rizin (evidence `ev_4eb37ac0…`, `ev_73f1ee3d…`; a0 = 0xD0018000,
a1 = 0x80808000, a9 = 0xD000C000), FR via `fr_tool.py` (ENLU 27.3 p.7193–7208, 27.11 p.7334–7346, 27.19 p.7444–7467),
DAMOS values read from each image, board test `simemu-tc1791/tests/oil_sensors.rs`, studio scenarios (native core).
Supersedes the G266 / oil-pressure-switch rows of §12.9. DTC names: the image's own ERRM name and DTC tables
(`drive_run::ecu_error_table`): 0x3B42 = OIL_SENS_EL (P0198), 0x3B43 = OIL_SENS_PLAUS (P0196), 0x426A = POIL_PIST_L (P168D),
0x3B55 = POIL_L_WARN (P164D) — identical in SC8S50 and SC8O40.

| Item | Result | Evidence |
|---|---|---|
| G266 capture (BSW) | ISR 0x8008DFE8 (G1.LTC41/43 events) → callback 0x8008DCBC: PWM-input driver 0x80089D54(ch 5) returns period (CLK7 ticks) and duty (Q14); period·10000/21972 → 0.1 ms, HIGH = period·duty ≫ 14 (`[a0-0x60c8]` = 0xD0011F38), LOW = period − HIGH (0xD0011F3C), edge counter 0xD0011F3E, valid flag 0xD0011F3A; 0xFFFF on timeout. Getter ACTION_INFR_GetOilSens 0x800A5A6C → 0x8008DD98. The event comes at the **falling** edge: HIGH = high time just ended, LOW = the low time before it (VERIFIED-T: last capture of a known waveform = heating pulse 30 ms + preceding low 50 ms) | VERIFIED-S/T |
| G266 decoder (ENLU 27.3, 10 ms) | 0x801A36F0, `lc_loil_sens_sys_conf` = 1 (pin). New capture (counter changed, no 0xFFFF) → T_EDGE_FALL_LOIL (0xD000C5AD, 10 ms ticks) = 0. With (H_old, L_old) the previous capture and L the current low: start phase ("fast temperatures") when H_old/L_old ∈ [0.95, 1.05] and H+L < 88 ms on consecutive captures → T_TEMP_SIG_LOIL = H_old + L_old; normal: \|H_old − L_old\| ≤ `c_t_sig_loil_ratio_max` 5 ms and L > `c_t_sig_l_loil_min` 60 ms, then H_old ∈ 48–52 ms = **reference** (T_REF_SIG_LEN_LOIL_TMP = H_old + L_old), H_old ∈ 2–45.5 ms = **temperature** (T_TEMP_SIG_LOIL_TMP = (H_old + L_old)·100 ms / T_REF); T_COOL_LOIL_TMP = (H_old + L_old + L)·100 ms / T_REF, T_HEAT_LOIL = H; LV_LOIL_TOG_SIG toggles ref/temp. Frame that satisfies it: **heating pulse (high) → 50 % pulse (low, high) → low until the cool-off time**, cool-off time = end of one heating pulse to the start of the next | VERIFIED-S (code, `c_t_sig_*` S50 0x80804F3C..0x80804F50), FR p.7195–7207 |
| G266 diagnosis (A0GZ, 100 ms) | 0x808DE214: OIL_SENS_EL when T_EDGE_FALL_LOIL > `c_t_edge_fall_loil_max_diag_el` 2.5 s (delay `c_t_dly_oil_sens_el` 60 s); OIL_SENS_PLAUS unless T_COOL_TMP ∈ [130, 1020] ms, T_TEMP_TMP ∈ [21, 89.5] ms, T_REF_TMP ∈ [95, 105] ms and T_REF/T_TEMP > 1.1 (Q15 0x8CCD; delay 30 s) | VERIFIED-S (S50 0x80804ED2..0x80804F56), FR p.7189 |
| Sender characteristic | the firmware's own sender curves (SC8S50): `ip_toil_srv_intl[0]` @0x8081FCE4 temperature time 23 ms → −40 °C … 35 ms → 0 °C … 65 ms → 100 °C … 84.5 ms → 165 °C; `ip_loil_srv_intl[0]` @0x8081F716 cool-off time 150 ms → 120 mm … 450 ms → 80 mm … 1050 ms → 0 mm (long cooling = low level). TOIL_SRV_INTL is not used for TOIL (CLF_TOIL_SRV_INTL_DI_CONF bit 0 = 1 in SC8S50 @0x80804F82, SC8O40 @0x80804F5A, SC8O30 @0x80804F4E) | VERIFIED-S |
| Sender model | `simemu_physics::sensors::OilLevelSender` (KL15-supplied, released output = high): power-up low 100 ms, 10 fast temperature periods, then frames heating 30 ms + 50 % pulse (reference 100 ms / temperature period alternating) + low to the cool-off time; inverses of the two curves; sump level from `lubrication` (100 mm at the MAX fill with the engine stopped, 20 mm/l, oil held in the circuit and drawn up with rpm). Heating length, power-up sequence, sump geometry UNVERIFIED; HIGH = heating follows the Hella thermal sender description (heated element = output high) | test `g266_frame_codes_temperature_and_level` |
| Board test | SC8S50 board, sender at 90 °C / 80 mm: T_REF_TMP 99.9 ms, T_TEMP_TMP 62.1 ms (sent 62.06), T_COOL_TMP 450.3 ms (sent 450), T_EDGE_FALL 26 (< 250) | `tests/oil_sensors.rs` (VERIFIED-T) |
| Oil-pressure switches | `c_idx_cod_poil_det_conf` (u32 @0x80804E60 S50 / @0x80804E38 O40) = 0x7FF: bit 8 swaps, bits 9/10 invert H/L (FR p.7341; init 0x800E9C24, OPM 0x808DF7A8) → LV_POIL_SWI_L (0xD0000E3B) = !P6.14, LV_POIL_SWI_H (0xD0000E37) = !P7.5: P6.14 = reduced-pressure switch F378, P7.5 = oil pressure switch F22, closing to ground at pressure. §12.9 had them the other way round (RAM order); the warnings were inhibited while the oil sensor was in error, so it only showed (POIL_L_WARN 0x3B55) once G266 worked | VERIFIED-S/T |
| Piston-jet switch | P6.15 (0x800A5A80 = !pin) = LV_POIL_SWI_PIST (1 = pressure, FR p.7443) = F447 on the jet gallery behind N522; POIL_PIST_L (FR 27.19.2.5, 0x808DE4F2..): cooling on (LV_OIL_2_PIST_DIAG = 0), N ≥ `ip_n_toil_poil_pist`(TOIL), no pressure for `ip_t_toil_n_poil_pist`. Plant: `Lubrication::p_jets_pa` follows the gallery through the open valve (N522 de-energised), leaks down (0.3 s, UNVERIFIED) behind the closed one | VERIFIED-S (code), switch point UNVERIFIED |
| Result | SC8S50 + CJXC (90 s): OIL_SENS_PLAUS gone; 0x3B42 status 0x65 → 0x24 (test passed this cycle, history bits from the bench NVM; at 40 s it is still 0x65 = not completed, delay 60 s); no OIL_SENS_* / POIL_* in ERRM. SC8O40 + CJSA `vehicle_drive`: 0x3B43 and 0x426A gone, no failed ECU DTC | scenario logs (native) |
| Side effect | `dyno_cam_phasing` (native) 10/17 within 2° (811e1b3: 17/17; Unicorn 17/17): the firmware's actuator commands (N428 high stage, N522) and cam set points are identical; the switch flags have no consumer outside ENLU warnings / diagnosis / CAN; a G266 heating pulse of 31 instead of 30 ms (recorded only in T_HEAT_LOIL) gives 17/17 — trajectory sensitivity of the cam loop (cam parked inside the plant spool's overlap band), not an oil-pressure path. Open | VERIFIED-T (A/B runs) |

Sources for the hardware (web): Hella "Thermischer Ölniveausensor" TI 2006 (heated element = output high, high time ~ oil
temperature, low time ~ level); Hella PULS brief 2106 (open-collector output, ECU pull-up); EA888 Gen3 switches F378 / F22 /
F447 and N522 / N428 from forum and parts data — switch points are not datasheet values. URLs:
https://www.hella.com/tibi/ti/de/electronics/ti_d_electronics_thermischer_oelniveausensor.pdf,
https://www.hella.com/resources-soe/assets/documents/2106-KI-Oelniveausensoren-FORVIA-HELLA-EN.pdf,
https://www.elektroda.com/rtvforum/topic3567109.html (long cooling = underfill),
https://vwts.ru/articles/engine/engine-2-0-ea888-gen-3-bz-fitting-locations.html (F1/F378/F447/N522/N428 on EA888 Gen3),
https://audizine.com/threads/oil-pressure-troubleshooting.907336/ (F378 0.7 bar, F22 2.55 bar, closed = active),
https://www.fcpeuro.com/blog/dealing-with-low-oil-pressure-tsi (F378 0.5–0.8, F22 2.3–3.0 bar),
https://www.briskoda.net/forums/profile/142291-theclient/content/page/13/?all_activity=1 (F447 on the jet circuit),
https://www.autoteileprofi.de/originalteile/oe-06l919081 (06L919081 equivalents 0.3–0.6 bar),
https://www.tomorrowstechnician.com/audi-vw-oil-coolers-importance-and-operation-2/ (N522 de-energised = jets on; search
excerpt only, page not readable here).
