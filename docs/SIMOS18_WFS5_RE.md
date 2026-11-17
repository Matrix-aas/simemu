# SIMOS18.1 WFS5 (Immo 5) credentials: reverse-engineering notes

Target: SC800S50 / `8V0906264K__0003.bin` & donor `8V0906259J`, TC1791 (file `0..0x1FFFFF` maps to `0x80000000`, `0x200000..` maps to `0x80800000`) and
donor DFlash dump `simos18_dflash.bin` (DFlash, file offset 0 maps to `0xAF000000`).
Code: `crates/simemu-cli/src/studio/immo_keys.rs` (`simos18_wfs5_partner(eeprom, flash)`).

## 1. Result

| Field | Value | Source |
|---|---|---|
| K0 (AES-128) | 16-byte partner key in NVM | DFlash IMMO data record `[1..17)` (decrypted with MCU UID) |
| Power class B | `0xBA` (donor `259J`) / `0x5C` (`SC8O40`) / `0x6A` (`264K`) | data record `[0x23]` |
| ID_kombi | `54 3D` (donor NVM; `immo_lab` derives deterministic `LAB-…` ID) | data record `[0x25..0x27)` |
| ID_ecu | `F9 3B` (u16 LE `0x3BF9` ×3) | PFlash `0x80001150` (FAZIT/production block at `0x80001100`) |
| VIN | 17-byte ASCII (`immo_lab`: `LABEMU…`) | data record `[0x11..0x22)` and channel bytes `[83..100)` |
| C4, c7p, c7a | `9C F7 FE B7`, `01..07`, `08..0E` | built in code (see §5); identical to the cluster spec |

The donor ECU NVM was originally adapted to a VW Golf 7 R (`B=0xBA`, `ID_kombi 54 3D`), whereas the DQ200_G2 donor EEPROM
came from a Skoda Octavia A7 FL (`B=0x5C`, `ID_kombi D3 2E`). Each partner keeps its own credentials, so the virtual
cluster can answer both, and by default `immo_lab` binds both NVM images in memory to a shared deterministic `LAB-…` key set.

## 2. Where the key material lives

* **IMMO NVM = ECME channels 6, 7, 8** (three redundant copies, 100 bytes each, encrypted) and channel 5 (19-byte
  statistics record, encrypted). Channel callbacks (descriptor table `0x8005F6AC`, stride `0x2C`):
  restore `0x801150A0/0x801150D4/0x80115108` (ch6/7/8), `0x8011513C` (ch5); write `0x801151C4…`; default `0x80115154…`.
  RAM images: `0xD001D0C4`, `0xD001D106`, `0xD001D148` (66 bytes each = state 13 + data 53), `0xD001D18A` (ch5, 19).
* Plain 100-byte channel image (write callback `0x801151C4`):
  `[0..13)` state record, `[13..66)` data record, `[66..83)` zero, `[83..100)` VIN copy for CBOOT.
  Matches Funktionsrahmen 49.2.2.5 ("0..65 OEM IMMO DATA, 66..82 unused, 83..99 VIN").
* **Data record** (53 bytes, built at `0x8011C4CC` from the adapter mirror `0xD00107B7..`):

  | off | len | meaning | adapter RAM |
  |---|---|---|---|
  | 0x00 | 1 | 0 | |
  | 0x01 | 16 | **K0** | `0xD00107D1` (getter `0x8011BC2C`) |
  | 0x11 | 17 | **VIN** | `0xD00107E5` |
  | 0x22 | 1 | lib `0x521` | `0xD00107C8` |
  | 0x23 | 1 | **power class B** | `0xD00107CB` (getter `0x8011BBEC`) |
  | 0x24 | 1 | IMMO state code from `0xD00107F6`: 1 or 0x0A → `0x55`, 3 → `0xA5`, 4 → `0x5A`, 0x58 → `0x55`/`0xAA` by the UID check `0x801162D4`, else (2 = adapted) `0xAA` | |
  | 0x25 | 2 | **ID_kombi** | `0xD00107CF` (getter `0x8011BC1C`) |
  | 0x27..0x2A | 4 | `0xD0010803`, `0x7CE`, `0x802`, `0x7CC` (meaning not traced) | |
  | 0x2B | 1 | `0xA5` if `0xD0010808` else `0x5A` | |
  | 0x2C | 1 | `(0x735 ? 0x0A : 0x05) \| (0x7C1 ? 0xA0 : 0x50)` (RAM `0xD0010735`/`0xD00107C1`) | |
  | 0x2D | 1 | `0x40·[0x7B4] + 0x20·[0x7C7] + 0x10·[0x7C6]` (download flags) | |
  | 0x2F | 4 | write counter (LE) | `0xD0010710` |
  | 0x33 | 2 | CRC-16/CCITT-FALSE LE over `[0..0x33)` (`0x8011F468`) | |

* **State record** (13 bytes, `0x8011C658`): `[0]` 0xAA/0x55, `[1..5)` = last challenge P (`0x8011F068`, mirrored at
  `0xD00107E1`) — **not an ID** (it differs between ch6/7 and ch8), `[7..11)` counter `0xD0010720`, `[11..13)` CRC16.
  The same IMO-lib record format is in the DQ200 SPI EEPROM (`0x1E8` data, `0x258` state). Consequence:
  `Wfs5ImmoPartner::from_dq200_eeprom` takes `ID_tcu` from `0x259` = first two bytes of a stale challenge.
  Harmless for answering (the answer uses K, B, ID_kombi only); only `verify_challenge` on TCU frames is affected.
* **ID_ecu**: `0x800DB13C` walks the table `0x8005D668` and calls `0x8011524C(a4 = 0x80001150)`, a 2-of-3 vote over
  three u16 → `0xD001D19E`; `0x801160D8` returns its low/high byte for the TX frame.

## 3. DFlash FEE format (TC1791 DF0 `0xAF000000..0xAF017FFF`, DF1 `0xAF080000..`, here erased)

* The first byte of every 0x100 page is a page marker (`0x00/0x10/0x20/0x30`), not data.
* Records grow upwards; each is `payload (8-byte units) | trailer[8]`; the trailer is solved in §8.2.
  `units` counts physical 8-byte words including page-marker bytes; the FEE scans **backwards** from the last
  written word (reads via `0x800F95F0`, memcpy `0x80086FB4`). The newest instance of a channel wins.
* Management pages (channel `0x30`) at `0x17900..0x17C00` and bank headers at `0x17E00/0x17F00`.
* The dump holds 124/182/91 copies of ch6/7/8. The extractor tries every trailer, decrypts, keeps the
  copies where both record CRCs are valid (all 397 here), and takes the highest (data, state) counter.

## 4. NVM encryption: `anvmcryptlight` (CBOOT, strings `based on anvmcryptlight.c r1.0`)

* Key schedule `0x80017B2C(a4 = 0xD0000000)`, called once at start-up through `[0x8001405C]` from `0x80106D82`.
  It takes **UID[0..8]**, where UID = the 12-byte TC1791 chip ID that sits at DSPR `0xD0000000` when the ASW
  starts. The factory copy is at `0x80014200` (`44 C8 91 07 D7 43 DC 79 0C 0C 00 20`, the same string as the dump
  file names). The ASW checks the two for equality at `0x800DB788` and hashes the RAM copy at `0x8017BAD0`.
  ```
  k[6] (0xB0000B3A) = u0^u1^u4, u1^u2^u3, u0^u2^u5, u0^u3^u4, u1^u3^u6, u1^u4^u5
  m[4] (0xB0000B4E) = u4^u5^u6, u2^u5^u7, u4^u6^u7, u0^u6^u7
  ```
* Per 16-bit word at "EEPROM address" A (= channel id, +2 per word; ECME passes the channel index, FR 19.2.1.7):
  48-bit state `A_hi A_lo A_hi A_lo k0 k1` (`0x800176BC`), then 32 key bits `k[2..6]^m` are shifted in with
  `bit ^ f(state)`, then 16 output bits = `shift(fb(state)); out = f(state)` (`0x8001794C`), XORed onto the word
  (`0x80017C64`; with a 3-way redundancy vote at `0x80017AB8`).
* `f` (`0x80017540`) is a Hitag2-shaped filter with tables (as bit masks) TA=`0x287B` (`0x8001523C`),
  TB=`0x267D` (`0x8001524C`), TC=`0x7956EB79` (`0x8001525C`) on state bits (MSB=0)
  {2,3,5,6} {8,12,14,15} {17,21,23,26} {28,29,31,33} {34,43,44,46}. The feedback `0x80017620` uses the Hitag2 taps
  {0,2,3,6,7,8,16,22,23,26,30,41,42,43,46,47}.

## 5. WFS5 in the IMO library (`0x80116000..0x8011E300`, RAM `0xD00103AE..0xD0010830`)

* `0x80117644(variant)` returns C4. The variant comes from cal `imo_idxlab_c_vw` (`0x8087A084` = 3), which selects
  `9C F7 FE B7`. Lab variants: `0x0D` gives `B3 FE 92 96`, `0x10` gives `85 41 22 3F`.
* Challenge `0x80117808`: P = `0x8011F068(0..3)` → ring `0xD00103EA`; block `0xD00103FC` = `C4 | B | P | 01..07`.
* AES `0x801174B2`: key = 16 bytes from `0x8011BC2C` (= K0), sliced AES `0x80120214`/`0x8011FE94`
  (S-box `0x8006143C`). CRC-32 `0x8011F4E0` (table `0x8006103C`) gives c, stored LE in `0xD00103E2`.
* TX frame `0x80117CC8` (`0xD00103F4`) = `P | c0 c1 (c2^ID_ecu[0]) (c3^ID_ecu[1])`.
* Expected answer `0x80117320`: block `C4 | B | ~P | 08..0E`, CRC stored at `0xD001041A`.
* RX: `can_immo_rx 0x8009B644` reads 8 bytes, then `0x80117518` copies them to `0xD00103C1` and sets flag
  `0xD00103BF`. Check at `0x80117198`: `0x42A` = (a0,a1 ok), `0x42B` = (rx2^a2, rx3^a3 == ID_kombi from
  `0x8011BC1C`), `0x428` = both && (S & 1).
* `0xD00107F6` is the adapter's NVM IMMO state (`2` = adapted/normal here). **`0x58` is not "authenticated".**
  `0x58` is written at `0x8011BEF4` and `0x8011C90E` when `0x801162D4 → 0x80187768` reports a missing or invalid chip UID.
  Many paths treat it as "immobiliser inactive". The bench bypass works by making the getter claim exactly that.

## 6. Verification

1. **Decryption**: all 397 ch6/7/8 copies (and ch5) decrypt to records with valid CRC-16, and to an ASCII VIN.
2. **Emulator, unmodified firmware** (scratch harness `build/immo_re/scratch`; runtime-only fixes listed in §7):
   * the firmware's own anvmcrypt keystream equals ours (`addr 0x0B → 6B 96`, `0x0D → DA FA`, `0x11 → 2F 95`);
   * the IMO adapter loads K0/B/ID_kombi/VIN from the DFlash by itself;
   * every `0x010` frame it transmits passes `verify_challenge` with the extracted K0, B and ID_ecu
     (e.g. `9F D3 2E 28 5A 05 34 DD`, `6A FA 02 41 95 A7 50 64`);
   * the firmware's expected-answer CRC for P=`6A FA 02 41` (RAM `0xD001041A` = `08 6A B1 2F`) equals ours;
   * our `answer_challenge` frame, handed to the IMO RX buffer, gives `0x42A=0x42B=0x428=1`, and the ECU
     stops re-challenging. Negative controls: a wrong ID_kombi gives `0x42B=0` and keeps re-challenging;
     a key with one bit flipped fails `verify_challenge` and gives `0x42A=0`.
3. The unit tests in `immo_keys.rs` contain these firmware vectors.

Not verified: a downstream "engine released" flag. The state stays `2`, which is correct (see §5).
Delivery of `0x011` through the emulated MultiCAN was not verified either (§7.3).

## 7. Emulator issues found (`crates/simemu-tc1791`, not fixed here)

1. **Chip UID at `0xD0000000` is missing.** Nothing in PFlash writes it: no static writer was found, and the
   only dynamic writes are the non-destructive RAM test at `0x8011171A`, which saves and restores it. It is
   start-up state. Seed `0xD0000000..0xD000000B` with `flash[0x14200..0x1420C]` before `run_boot_sequence`.
   Without it, NVM decryption fails, the VIN becomes `EEPROM-READ-ERROR` (`0x8011B776`) and the state becomes `0x63`.
2. **Unicorn TriCore bug: `LD.BU D[a], [+A[b]]off10`** (BO, op1 `0x09`, op2 `0x11`) **sign-extends** like
   `LD.B`: `0xAB` loads as `0xFFFFFFAB`. The other modes (`[a+]`, `[a]off`, `LD.HU` pre-inc) are correct.
   This breaks anvmcrypt (filter/shift) and the IMO AES. FIPS-197 C.1 gives `5D0A4554…` instead of `69C4E0D8…`.
   A workaround that masks the destination to 8 bits after each such instruction (1509 sites; list in
   `build/immo_re/ldbu_preinc.txt`) makes both correct. This needs a fix like `hook_fix_tc1784_imask`.
3. `inject_can_rx` accepts `0x011` (1 MO matched), but `can_immo_rx 0x8009B644` is never executed, so
   `0xD00103BF` never gets set. The CAN RX dispatch into the IMMO handler needs a look.

## 8. Lab binding (project rule 2026-11-02, `AGENTS.md` §2.4 exception 5)

Code: `crates/simemu-cli/src/studio/immo_lab.rs` (binding, tests), FEE/cipher helpers in `immo_keys.rs`. RE through REA
(`execute_rizin_command` on ELFs built from each image), FR chapter 49 (p.12848–12873).

### 8.1 What the firmware checks (ECU)

* NVM: ECME channels 6/7/8 (§2), `anvmcryptlight` keyed by the chip UID (§4), CRC-16 per record. The IMO restores the
  three channels and uses the copy with the highest data counter. FR 49.2.2.5: the IMO lib ≥ 6.1.0 writes the state and
  data record together into all three channels (`ImoIfS_Imo_strDatDat_VW`); the VIN copy `[83..100)` is refreshed on
  every write.
* Power class: check `0x8011C414` (SC8S50; SC8O40 `0x801261E0`, SC8O30 `0x80125AA0`, SC8H83 `0x80112E9C`). States
  `0x63/1/0x58/0x0A` skip it (B := 0); otherwise the adapted B (`0xD00107CB`) is looked up in the 5-byte RAM list
  (`0xD00107FD`, SC8O40 `0xD0010835`, SC8H83 `0xD0010867`); found → index 1..5 (`0xD0010809`); not found (or, in SC8S50/SC8O40, an
  all-zero list; SC8H83 `0x80112F0A` skips the check then) → IMMO state `0x42` (the `0x010` challenge is never sent; previous finding: ignition and injection
  cut, start monitoring resets). FR p.12852: `NC_IMMO_PCLASS_NR = 5`, error `NC_IDX_ERR_ECU_6` "IMMO P-CLASS ERROR".
* The RAM list is copied at start-up from the calibration struct `data_imodat` (`imo_idxlab_c_vw`, then
  `imo_idxtun01..05_c_vw`; SC8S50 init `0x8011ABB8` reads the struct pointer at `0x80061AB4`). Located per image by
  code signature (`immo_lab::power_classes`), cross-checked with DAMOS where it has the names:

  | Image | Calibration | struct (`imo_idxlab_c_vw`) | variant | list `imo_idxtun01..05` | DAMOS |
  |---|---|---|---|---|---|
  | Donor `8V0906259J` | SC8S50 | `0x8087A084` (ptr `0x80061AB4`) | 3 | `AC B7 B9 BA 00` | SC8S50.json ✓ |
  | `8V0906264K__0003.bin` | SC8S50 | `0x8087A084` | 3 | `6A 00 00 00 00` | (same layout) |
  | `8V0906264H__0005.bin`, `J__0003` | SC8O40 | `0x8087944C` (ptr `0x80065FFC`) | 3 | `5C 00 00 00 00` | not in SC8O40.json |
  | `8V0906264F__0003.bin` | SC8H83 | `0x8087AEDC` (ptr `0x800634E4` = `0xA087AEDC`) | 3 | `6A 00 00 00 00` | not in SC8H83.json |

  Variant 3 = C4 `9C F7 FE B7` in all images. Index of B may select the power variant (`ACTION_SYCF_SetPowClasIdx`,
  FR p.12866, only with `USE_REVERSE_VAR_COD`) — not checked.
* Adaptation path (evaluated, not driven): the IMO download state machine `0x80118BFA..` decrypts the GeKo blob
  (`0xD0010484..0xD00104B3`, 3 AES blocks) with the **current** K (`0x8011BC2C` → `0x8011FD7C`), checks CRC-32
  (`0x8011F4E0`, big-endian at `+0x34`), mode byte `+0x24` (1 clear K, 2/3 set K from `+0x11`); the data download
  (`0xD0010548..`, B at `+0x52`, lib byte at `+0x57`) fills `0xD0010508..0xD0010547`; `0x8011AFD8` copies the
  changed fields into the adapter mirror and the NVM write follows (callback `0x801151C4`). Driving it needs the
  UDS login and blob format of two IMO libraries (ECU, TCU) — not done; records are written directly instead.
* Immo-off patches: `movh.a a15,#0xd001; mov.u d2,#0x58; ret` as the state getter (e.g. `0x8011BC60` in SC8S50 or
  `0x801252EC` in SC8O30, detected read-only by `immo_lab::immo_off_patch`). With it the firmware never challenges; the binding is written in NVM but not
  checked by a patched firmware.

### 8.2 FEE record trailer (ECME, `0x800F8xxx..0x800FBxxx`)

Builder `0x800F9210`, parser `0x800F9098`, check `0x800F8C48`/`0x800F9110`:

```
trailer = [ (h<<4)|~c>>12, ~c>>4, (~c<<4)|units>>8, units, id>>8, id, c>>8, c ]       h = 0 in all records
id      = channel<<8 | 0x46                                                          (0x800F85F8)
c       = CRC16ARC( CRC16ARC(0xA55A, payload), seed_le32 | units_le16 | id_le16 )    (table 0x80040E08, loop 0x800870DC)
payload = the units*8 bytes before the trailer without the page-marker byte of every 0x100 page
seed    = bank sequence: bank-header record id 0xFFFE (DF0+0x17E00/0x17F00), raw bytes 4..8 BE = 0x9C3 here;
          the header itself is checksummed over its 8 raw bytes (marker included)
```

All 630 consistent trailers of the bench DF0 verify (test `fee_trailers_match_the_firmware_checksum`). DF0 is
almost full (one free page, `0x17D00`); appending new instances would require a bank swap to DF1, which the
emulation does not attempt. Side finding: at start-up the firmware erases DF1 with `…5554:80, 5554:AA, AAA8:55,
AF080000:40` ("erase physical sector") and enters `5554:5E`; `flash.rs` reports both as sequence errors (3 per boot;
DF1 is already erased, so no data effect) — not fixed here.

### 8.3 DQ200_G2 (TCU)

* Data record copies at EEPROM `0x1E8`, `0x220`, `0x268` (56-byte slots, plain, same 53-byte layout, CRC-16 LE at
  `+0x33`), state copies at `0x258`, `0x2A0`, `0x2B0`. Read by `0x8004A2BA` into RAM `0xD0000A02/0xA37/0xA6C`,
  CRC `0x8004E8C2`, the highest counter wins (`0x8004A3C6..`).
* Power class: same check `0x8004B3A0` on the RAM list `0xD0000B74`, which is only zeroed (`0x80049F70`, no other
  writer); an all-zero list skips the check (`0x8004B412`). The TCU accepts any B → one cluster B for both slaves.

### 8.4 Binding (data only; dump files untouched; in memory at every image load)

1. Keys: `block_i = AES-128("SIMEMU-IMMO-LAB1", UID[0..12] | 00 00 00 i)`; slot A = block 1, slot B = block 2;
   block 3 → ID_kombi `[0..2)`, VIN `LABEMU` + 11 hex digits of `[2..8)`, key-set id `LAB-` + `[12..16)`.
   K0/K1 cut from A/B as the MQB cluster does (`fpke-immo-wfs5.md` §1.2). Bench UID → `LAB-127F2466`,
   ID_kombi `47 05`, VIN `LABEMU9F200CED3B7`.
2. B: the dump's B if it is in the loaded calibration's list, else the first entry. Independent of the bench engine.
3. ECU: the last FEE instance of channels 6/7/8 (+ identical copies written with it) gets the newest record with
   K0, VIN, B, ID_kombi, data counter + 1, CRC-16, VIN copy; re-encrypted; trailer CRC recomputed. Read back with the
   firmware's own decryption before use. ID_ecu (PFlash `0x80001150`) unchanged.
4. TCU: all three data copies with K1, VIN, B, ID_kombi, counter = max + 1, CRC-16; read back.
5. Cost: < 30 ms per load (unit tests run in 0.03 s); nothing at run time.

### 8.5 Verification (2026-11-02, native core; Unicorn see MILESTONE)

`immo_lab::tests::board_level_lab_binding` (no plant, 3 s, every `0x010` checked with `verify_challenge` against
the lab K0/B/ID_ecu) and `lab_binding_e2e` (studio session, CJSA plant, warm, KL50 1–6 s, 8 s):

| Image | B (cal addr) | 0x010 lab / wrong K0 / dump (3 s) | Session: lab key | Session: wrong K0/K1 |
|---|---|---|---|---|
| Donor `259J` SC8S50 | `0xBA` (`0x8087A088`) | 1 / 54 / 1 | 1 challenge, `0x428/2A/2B = 1`, runs to 1525 rpm, fires to the end, TCU state 7 | 73 challenges, flags 0, fires until ~3.9 s then ignition and injection cut, TCU never 7 |
| `K__0003` SC8S50 | `0x6A` (`0x8087A085`) | 1 / 54 / 0 | 1 challenge, runs to 1323 rpm, TCU 7 | 74 challenges, cut at ~4 s |
| `H__0005` SC8O40 | `0x5C` (`0x8087944D`) | 1 / 54 / 0 | 1 challenge, sparks + injection, 802 rpm, TCU 7 | 100 challenges, no injection at all |
| `J__0003` SC8O40 | `0x5C` | 1 / 54 / 0 | same as H__0005 | same |
| `F__0003` SC8H83 | `0x6A` (`0x8087AEDD`) | 1 / 54 / 0 | studio loads and runs after ECM3 table check | — |

Unicorn (patched 2.1.4) gives the same verdicts on every row (e.g. SC8S50 1 challenge, flags 1/1/1, 1494 rpm; wrong key 73 challenges, cut at ~3.9 s).

`engine_start_scenario` with `SIMEMU_IMAGE=images/8V0906264H__0005.bin` (session default plant CJSA IS12): immobiliser
released (sparks on all cylinders from 2.2 s, injection pulses, up to 805 rpm), the engine stalls after ~6 s because
the HPFP valve N276 is never driven (rail 0.5–0.7 MPa) — fuel system, not immobiliser. The scenario's own immobiliser
assert uses SC8S50 RAM addresses (`0x8009B644`, `0xD0010428`) and fails on SC8O40 for that reason.

Not verified: the IMMO DTCs; why the lab-bound ECU challenges again every ~10 s (2 in 12 s, 4 in 40 s; with a wrong
key it is every ~50 ms); the meaning of data-record bytes `0x27..0x2A`.
