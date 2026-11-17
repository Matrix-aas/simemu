# SIMOS18.1 board: external chips, PCB routing, and firmware interfaces

Firmware reference: factory FRF images `8V0906264H__0005.bin` (`SC8O40`) and `8V0906264K__0003.bin` (`SC8S50`), cross-checked
against unmodified SC800S50 donor (`8V0906259J`, cal `SC8S50`).

Hardware reference (`PHOTO`): opened physical **Continental SIMOS18.1 ECU** from Skoda Octavia III A7 FL 1.8 TSI
(`VW AG 06K 907 425 B`, HW `H13`, `0004`, `Continental 5WA18024 01`, `13.07.19`, bare PCB silkscreen
**`A2C80473700-04` / `mbo`**):
- `vault/hardware/simos18_06K907425B_H13/IMG_2507.DNG` *(local RAW archive, gitignored)*: 3024×4032 orthogonal Apple ProRAW overhead scan of the complete top side of PCB
  `A2C80473700-04`.
- `vault/hardware/simos18_06K907425B_H13/20261111_184622..184728.jpg` *(local RAW archive, gitignored)*: 8 angled macro photographs of all top-side ASICs, MOSFETs, diodes,
  shunts, and crystals.
- Connector & harness cross-reference: `docs/SIMOS18_PINOUT.md` (`T105a` / `T91a` pinout reference).
- *(Supersedes an earlier partial third-party PCB photo which had
  obscured the lower half of the PCB and led to the earlier assumption that `ATIC121`, `ATIC111 ×3`, `ATIC142`, and
  `ATIC140 #2` were on the bottom side — **all of them sit on the top side** of `A2C80473700-04`, PHOTO.)*

Evidence labels follow `docs/SIMOS18_GPTA_IO_RE.md` §0 (`VERIFIED-S`, `VERIFIED-T`, `VERIFIED-FR`, `VERIFIED-DS`,
`HYP`, `UNVERIFIED`); additionally **PHOTO** = read or traced directly on `vault/hardware/simos18_06K907425B_H13/`, **DAMOS** = name/value in
`definitions/SC8S50.json` (read from the image at the DAMOS address).

Session tooling: REA rizin evidence `ev_a718c30de288…` (job start `0x800A8CE8`), `ev_a8a0fac6fdd7…` (job descriptor and
callback tables `0x80042774..0x800429DC`), `ev_cb9221a5c403…` & `ev_577101765e0c…` (`SCU_CHIPID` decoder
`0x80112478..0x801124F0`), `ev_7fde7f07797e…` & `ev_55d11e31e47c…` (BSW DIO input getters `0x800A5550..0x800A6900` and
`ASC0` search); traces = MMIO log + an SPI frame log added temporarily to `BoardSpiBus` (boot 2–16 s, KL15 off at 3 s);
RAM read back from the running emulator.

---

## 1. Chip & power-stage inventory (`06K 907 425 B` HW `H13`, PCB `A2C80473700-04`)

| Photo marking (`PHOTO`) | Package & location (top side) | Identification & rating | Function | MCU interface | Firmware / doc ref | Emulator model |
|---|---|---|---|---|---|---|
| `20.000` (`S:93`) | 2-pad SMD crystal above MCU | 20.000 MHz fundamental crystal | `f_OSC` | `XTAL1`/`XTAL2` (balls `K20`/`K19`, DS p.27) | PLL 720 MHz VCO → 180 MHz CPU / 30 MHz STM (`stm_icu.rs`) | `FOSC_HZ` (`PHOTO` + `VERIFIED-T`) |
| `Infineon TriCore SAK-TC1791S-384 F200EP AB` (`84PKGS885B1`, `G1918`) | PG-LBGA-292, center-right | Infineon AUDO MAX `TC1791S`, 3 MB PFlash, 200 MHz, SHE, `AB` step (`CHIPID = 0x9600AA02`) | Main 32-bit MCU + PCP2 | — | §1.1 (`0x80112486`) | `simemu-tc1791` |
| `A2C00757700 / ATIC140S B1 / 8M25321` (×2) | PowerQFP-64 w/ 2 side heat slugs (upper-mid & lower-mid) | Continental `ATIC140S` rev B1 *(older boards: `A2C0005956`)* | 2× 26-ch low-side / PWM / smart-coil trigger arrays (upper: `T91a` relays/pump/fan/λ heaters; lower: `T105a` solenoids/MPI/AVS + 4× `20.0` Ω coil triggers) | `MSC0` / `MSC1` | `atic.rs`; SOPC table FR p.5679; §1.3, §9 | modelled (`atic.rs`, `power_stages.rs`) |
| `SC900697AE / A2C00704400 / ATIC139/A2 / CTEQ1919E` | LQFP-64, top-right | Freescale/NXP custom Continental `ATIC139` system basis ASIC | **MU** (ECM3 level-3 safety monitor) + **LS** (3 diagnosed low-side stages) + **PS** (switch-mode pre-regulator + 2 switched sensor/5 V supplies) | `SSC1` / `SLSO12` (`P2.2`, DS p.33) | §1.3, §3 | MU: `monitor_device.rs`; LS/PS: `atic139.rs` |
| `Infineon 2N06L64` (`6AK905`) ×2 + `ON B36` (`MBRS360`) ×2 + `LTP 95` / `SJ 94` | 2× DPAK + 2× SMC + SMB around `ATIC139` | `IPD50N06S4L-64` / `2N06L64` (**60 V**, 6.4 mΩ logic N-MOSFET) + `MBRS360` (**60 V**, 3 A Schottky) | **Switch-mode pre-regulator of `ATIC139` PS block** (60 V rating rules out 65 V GDI boost, `PHOTO`) | Driven by `ATIC139` | §1.3, §3 | supply rail implicit |
| `ST A2C00057927 / ATIC121 B3 / 9900Y V1 AK / 920 MYS` | Exposed-pad TQFP-64, bottom-right | STMicroelectronics Continental `ATIC121` rev B3 | 4-cyl GDI peak-and-hold injector & `N276` HPFP controller + 65 V boost controller | `SSC0` / `SLSO00` (`P2.0`) + GPTA0 `GTC04/17/16/05` + `G1.GTC26` | §1.3, §2.1, `atic.rs` | modelled (`atic.rs`) |
| `Infineon 30N10L25` + `R022 F` (`L254H`) + `3R3` + SMC diode | DPAK + 2512 shunt + 0805 + SMC, right of `ATIC121`/`ATIC142` | **100 V**, 30 A, 25 mΩ logic N-MOSFET + **22 mΩ 1%** shunt + 3.3 Ω gate resistor (+ underside inductor & bulk cap) | **65 V GDI DC/DC boost converter** (`VBOOST` sensed on `ADC2.CH1`) | Driven by `ATIC121` | §1.3, `studio::io::VBOOST_ADC` | `injector_boost_v` (`studio/io.rs`) |
| `ON VNL10` (`64 / 15ANLG`) ×2 + `ON VNL17` (`U / 620TG`) ×2 | 4× DPAK below `ATIC121` | Automotive high-side/bank power MOSFET switches | GDI injector boost/battery bank switches feeding `N30..N33` (`T105a/1,2,22,23,43,64,65,85`) | Driven by `ATIC121` | §1.3 | modelled via `ATIC121` |
| `ON VNL17` (`U / 620TG`) + 2× DPAK + `R022 F` (`L254H`) + `LTP 95` | 3× DPAK + 2512 shunt + SMB, between `ATIC140 #2` & `ATIC111` | Power MOSFET stage + **22 mΩ 1%** current-sense shunt + freewheeling diode | **`N276` HPFP fuel-pressure metering valve** current-regulated HS/LS stage (`T105a/92, 93`, `HYP` supported by `PHOTO` adjacency & 22 mΩ shunt) | `G1.GTC26` / `ATIC121` | §1.3, GPTA doc §12 | `hpfp_close_deg` / `hpfp_off_deg` |
| `ST A2C00693000 / UM31C6 / 9CV920 / MYS V1 99` (×3) | 3× PowerSSO-24, bottom-left | STMicroelectronics Continental `ATIC111` 5 A H-bridge | **Upper**: `J338` throttle (`T105a/90,91`, Bridge 0); **Bot-Left**: `N493` coolant valve (`T105a/86,87`, Bridge 1); **Bot-Right**: `V465` wastegate (`T105a/88,89`, Bridge 2) | `SSC1` / `SLSO13` (`P2.3`, 3-chip daisy chain) + `PWM`/`DIR`/`DIS` | §1.2, §2.1, §9 | modelled (`atic.rs`) |
| `TI A2C49811B1 / 9G15AA` | TSSOP-24, right of `ATIC121` | Texas Instruments Continental `ATIC142` O2 front-end ASIC | Wide-band upstream λ (`G39` LSU 4.9) + binary downstream λ (`G130` on `flexIO`) | `SSC1` / `SLSO14` (`P2.4`, 250 kHz) | §6, FR EGCP 22.5 | modelled (`atic142.rs`) |
| `Microchip 12LF1822 E/SN 1915 QDD` | SOIC-8, top-center right of `ATIC140 #1` | `PIC12LF1822` XLP 8-bit MCU (`LFINTOSC`) | Battery-backed external engine-off timer (`T_ES`) | `SSC0` / `SLSO06` (`P2.6`), 8-bit SPI; pins 2 & 5 NC (`PHOTO`) | §4, FR ENSS 33.4 | modelled (`pic_eot.rs`) |
| `NXP TJA1043T / CRW00041 / TnD19201` + `H9264` + 2× `61R9` | SOIC-14 + choke + 2× 0805, mid-left *(2nd CAN footprint unpopulated beside it)* | High-speed CAN transceiver w/ split termination (`61.9 × 2 = 123.8 Ω`) | Powertrain CAN (`T91a/79` CANH, `T91a/80` CANL) | `CAN0` (`P6.9`/`P6.8`) or `CAN1` (`P6.11`/`P6.10`) via inner layer | §5 | frame-level CAN (`multican.rs`) |
| `Melexis MLX14610 / CB-1914 / A2C819000 / S27472` | SSOP-16, mid-left below `TJA1043T` *(2nd SSOP-16 unpopulated below it)* | Melexis custom Continental switch-conditioning ASIC `A2C819000` | High-voltage vehicle switch wetting & level shifter (`T91a/61, 62, 75, 76, 77` → 4 logic outputs via `103` 4×10 kΩ array) | 4× `TC1791` DIO inputs (`HYP`: `P0.3`, `P0.12`, `P9.4`, `P4.7`) | §1.4 | DIO pins (`power_stages.rs` / `studio::io`) |
| `TI 2903Q1 / 94KG4` | SOIC-8, right of unpopulated SSOP-16 | Texas Instruments `LM2903-Q1` dual comparator | Dual threshold comparator (`2491` 2.49 kΩ, `223` 22 kΩ, `104` 100 kΩ network) | `TC1791` digital / capture inputs (`HYP`) | §1.4 | — |
| `V32 / A702 / 1011` + `AOH` ×2 + `TR8` ×2 | VSSOP-8 (bottom-left of MCU) + SOT-23-6 + SOT-23 | Dual analog op-amp/conditioner + small-signal stages | Analog / FADC knock (`G61` `FAIN1P`) & sensor conditioning (`HYP`) | `FADC` / `ADC` | §1.4, §9 | `knock_fadc.rs` |
| *(Bottom side — not on top photo)* | Underside of `A2C80473700-04` | MEMS barometric pressure sensor (`BAP`); GDI boost inductor & bulk electrolytic cap | Atmospheric pressure `ADC1.CH3` (under housing Gore-Tex vent, `ecu_body.png`); 65 V boost energy storage (heavy TH leads at `x=2570,2730`) | `ADC1.CH3`; boost node | `sensors_io.rs`; §1.3 | `baro` (`sensors_io.rs`) |

---

### 1.1 MCU `SAK-TC1791S-384 F200EP AB` vs bench `TC1791F-512F240EP` and `SCU_CHIPID`

1. **Why `06K 907 425 B` uses `SAK-TC1791S-384 F200EP AB` (`PHOTO` + `VERIFIED-DS`, `infineon-tc1791-ds-en.pdf` pp. 8–19):**
   - **200 MHz speed grade (`F200`) matches the 180 MHz firmware clock:** SIMOS18.1 programs the PLL (`20 MHz × 72 / (2 × 4)`)
     to `f_CPU = 180 MHz`, `f_PCP = 180 MHz`, `f_FPI = 90 MHz`, `f_STM = 30 MHz` (`stm_icu.rs`). The 200 MHz `F200`
     grade (`-40..+125 °C`) covers 180 MHz with 20 MHz margin without requiring the 240 MHz `F240` bin.
   - **3 MB PFlash (`-384` = 384 × 8 KB = 3072 KB) fits the entire SIMOS18.1 Flash layout:** `TC1791S-384` populates
     `2 MB` PFlash0 (`0x80000000..0x801FFFFF`: `SBOOT` + `CBOOT` + `ASW1` + `ASW2`) and `1 MB` PFlash1
     (`0x80800000..0x808FFFFF`: 512 KB `CAL` + 512 KB `ASW3`). In every SIMOS18.1 image inspected
     (`8V0906264H__0005.bin`, `8V0906264K__0003.bin`, donor `8V0906259J`), the fourth
     megabyte (`0x300000..0x3FFFFF`, i.e. `0x80900000..0x809FFFFF` present only on `-512`) contains **zero non-erased
     bytes** (`VERIFIED-S`).
   - **Identical SRAM & DFlash (`VERIFIED-DS` Table 1, p. 9):** Both `TC1791S-384F200EP` and `TC1791F-512F240EP` provide
     192 KB DFlash (2×96 KB DF0/DF1) + 2×8 KB Key Flash, 128 KB DSPR, 32 KB PSPR, 16 KB ICACHE, 16 KB DCACHE, 128 KB
     FPI SRAM, 16 KB PRAM, 32 KB CMEM, and 16 KB BROM (`-384` bonds 44 ADC lines vs 48 on `-512`).
   - **`S` feature package:** includes the **SHE (`Secure Hardware Extension`)** cryptographic peripheral (`VERIFIED-DS`
     pp. 14, 16), absent on `TC1791F`.

2. **Exact `SCU_CHIPID` (`0xF0000640`) values (`VERIFIED-DS` Tables 4 & 10, pp. 74, 76):**
   - `SAK-TC1791F-512F240EP` (`AB` step): **`0x8700_9502`** (bit 31 = `EP` exposed pad, bit 28 = `0` for 240 MHz,
     bits `[26:24] = 7` for 4 MB PFlash, `CHID [15:8] = 0x95` `TC1791F`, `CHREV [7:0] = 0x02` `AB` step — confirming
     `CHIPID_VALUE` in `scu.rs:27` is exact per DS Table 4).
   - `SAK-TC1791S-384F200EP` (`AB` step): **`0x9600_AA02`** (bit 31 = `EP`, bit 28 = `1` for 200 MHz, bits `[26:24] = 6`
     for 3 MB PFlash, `CHID [15:8] = 0xAA` `TC1791S`, `CHREV [7:0] = 0x02` `AB` step).

3. **Firmware `SCU_CHIPID` decoder (`0x80112478..0x801124F0`, `VERIFIED-S`, REA `ev_cb9221a5c403…`, `ev_577101765e0c…`):**
   - Reads `0xF0000640`, extracts `d15 = CHIPID[16:8]` (`CHID`) and `d0 = (CHIPID[29:24] << 9) | d15`, then executes:
     ```text
     0x8011249e: eq   d15, d15, #0xaa
     0x801124a2: cadd d0, d15, d0, #-0x15   ; if CHID == 0xAA (TC1791S), subtract 0x15 -> 0x95 (TC1791F)
     0x801124aa: and  d8, d0, #0xff
     ```
   - `CHREV` (`0x02`, `AB` step) and the upper Flash/frequency bits (`0x96` vs `0x87`) are masked off; both `0x8700_9502`
     and `0x9600_AA02` normalize to `d8 == 0x95` (`TC1791`), taking the `0x95`/`0x98` branch (`0x801124AC..0x801124EC`)
     that unlocks ENDINIT (`0x800A92A8`), copies the BROM Die Temperature Sensor calibration word from `*(u16*)0xD0000018`
     into `SCU_DTSCON` (`0xF00005E4`), and relocks ENDINIT (`0x800A92D0`).

4. **BGA-292 physical orientation on PCB (`PHOTO` + `VERIFIED-DS` Fig. 7 p. 27):**
   - Rotating DS Figure 7 (top view) **90° clockwise** aligns with `IMG_2507.DNG`: ball `A1` sits at the bottom-left
     white silkscreen dot (`x=1835, y=2055`); `XTAL1`/`XTAL2` (`K20`/`K19`) sit at top-center (`x=2015, y=1670`) directly
     beneath the `20.000` MHz crystal; the left edge (`rows A..F`) faces the connectors and carries digital/SPI/CAN/SENT
     ports (`P0, P2/SLSO, P5, P6, P8, P9`); the right edge (`rows U..Y`) carries the 44/48 analog inputs (`AN0..AN47`)
     plus `P4, P10, P14`.

---

### 1.2 Connector pin-grid geometry on PCB (`T91a` vs `T105a`, `PHOTO`)

In `IMG_2507.DNG` (`3024×4032`), the right-angle through-hole connector header runs down the left edge (`x = 372..615`,
5 vertical columns `c = 0..4` at `x ≈ [372, 433, 495, 555, 615]`, shortest-to-longest bent tails corresponding to
columns 1..5 in `ecu_connectors.png`):

- **Upper group (`y = 895..1978`) = `T91a` (91 pins = 6 large power blades + 5×17 small pins):**
  - **6 large power blades** (`y = 1978` adjacent to the inter-connector gap, `y = 1875` below the small pins):
    - `c = 0` (`x = 372`): **`T91a/1` (`y=1978`) & `T91a/2` (`y=1875`)** — both bonded directly into the main top-layer
      **GND plane** (Terminal 31) via 4-spoke thermal reliefs (`PHOTO`).
    - `c = 2` (`x = 495`): **`T91a/3` (`y=1978`) & `T91a/4` (`y=1875`)** — isolated clearance antipads (inner layer).
    - `c = 4` (`x = 615`): **`T91a/5` (`y=1978`) & `T91a/6` (`y=1875`)** — isolated clearance antipads (Terminal 87
      switched main-relay power input from `J271`, routed on inner copper planes).
  - **85 small signal pins:** `pin = 7 + 17·c + r`, where row `r = 0..16` counts **upward** from `y = 1770` (`r = 0`:
    pins `7, 24, 41, 58, 75`) to `y = 900` (`r = 16`: top row `23, 40, 57, 74, 91` — matching molded housing numbers
    `40 | 57 | 74` on `ecu_connectors.png` and verified by `T91a/79` `(c=4, r=4)` = `CANH` and `T91a/80` `(c=4, r=5)` =
    `CANL`).
- **Lower group (`y = 2245..3350`) = `T105a` (105 pins = 5×21 small pins):**
  - `pin = 1 + 21·c + r`, where row `r = 0..20` counts **upward** from `y = 3350` (`r = 0`: bottom row
    `1, 22, 43, 64, 85`) to `y = 2245` (`r = 20`: top row `21, 42, 63, 84, 105`).
  - Verified by two independent copper signatures (`PHOTO`):
    1. **GDI injector pins `N30..N33`** (`T105a/1, 2, 22, 23, 43, 64, 65, 85` = all at `r = 0..1` along the very bottom
       of `T105a`), where the standard `x = 755` 0603 signal-filter MLCC row stops and is replaced by 7 larger
       high-voltage snubber MLCCs along the bottom board edge (`y = 3480`).
    2. **3× `ATIC111` H-bridge heavy copper pours** landing directly on **`T105a/86..91`** (`c = 4, r = 1..6`, §9).

---

### 1.3 Power stages contrasted: `ATIC139` pre-regulator, `ATIC121` 65 V GDI boost, `N276` HPFP, and `ATIC140S`

1. **`ATIC139/A2` 60 V switch-mode pre-regulator (`PHOTO`, `20261111_184650.jpg` & `184728.jpg`):**
   - Immediately below and right of `SC900697AE / ATIC139/A2` sit **2× Infineon `2N06L64`** (`6AK905`, **60 V**, 6.4 mΩ
     logic-level N-MOSFETs, DPAK), **2× ON Semi `B36`** (`MBRS360`, **60 V**, 3 A SMC Schottky rectifiers), a `151`
     (150 Ω) resistor, Vishay `LTP 95` / `SJ 94` SMB diodes, and five 1210 ceramic capacitors.
   - Because both MOSFETs and Schottkys are 60 V devices (and the actual 100 V GDI boost converter sits next to
     `ATIC121` at the opposite corner), this circuit is **definitively the switch-mode pre-regulator of `ATIC139`'s
     `PS` supply block**, settling the earlier ambiguity.

2. **`ATIC121 B3` 65 V GDI DC/DC boost converter & injector bank switches (`PHOTO`, `20261111_184722.jpg`):**
   - **Boost converter** (right of `ATIC121`/`ATIC142`): **Infineon `30N10L25`** (**100 V**, 30 A, 25 mΩ N-MOSFET,
     DPAK) driven via a **`3R3` (3.3 Ω)** gate resistor, sensed across a **`R022 F / L254H` (22 mΩ, 1%)** 2512 shunt
     resistor, feeding an SMC boost rectifier diode and two pairs of heavy through-hole leads along the right board edge
     (`x = 2570, 2730`) for the underside boost inductor and bulk electrolytic capacitor.
   - **GDI bank switches** (below `ATIC121`): **2× ON `VNL10` (`64 / 15ANLG`) + 2× ON `VNL17` (`U / 620TG`)** DPAK
     MOSFETs (boost/battery high-side bank selection for the two GDI cylinder banks `N30..N33`).

3. **`N276` HPFP metering-valve discrete stage (`PHOTO` + `HYP`, `20261111_184706.jpg` & `184715.jpg`):**
   - Between `ATIC140 #2` and the upper `ATIC111` (`x = 1180..1580, y = 2800..3220`, directly opposite `T105a/92, 93` =
     `N276` HPFP valve lines A & B) sit **1× ON `VNL17` (`U / 620TG`)** DPAK, **2× additional DPAK MOSFETs**, a second
     identical **`R022 F / L254H` (22 mΩ, 1%)** current-sense shunt, and a Vishay **`LTP 95`** freewheeling diode —
     forming the high-current peak-and-hold stage for `N276`.

4. **Upper vs lower `ATIC140S B1` (`A2C00757700`, `PHOTO`):**
   - **Upper `ATIC140S B1`** (facing `T91a`): surrounded by `152` (1.5 kΩ ×4) and `302` (3.0 kΩ) networks + `EHD 95`
     diodes (`T91a` main/aux/starter relays, `J538` fuel-pump PWM, `J293` fan PWM, `N82` valve, `G39`/`Z29` λ heaters).
   - **Lower `ATIC140S B1`** (facing `T105a`): features **four `20.0` (20.0 Ω) 0805 series resistors** along its upper
     edge (`PHOTO` — matching the 4 active smart ignition coil trigger lines `N70, N127, N291, N292` on
     `T105a/76, 79, 57, 62`), plus **2× ON `U2D` (`MURS220T3G`, 2 A 200 V ultrafast SMC clamping diodes)**, `EHD 95`
     diodes, and a **`R01` (10 mΩ)** current shunt (`T105a` MPI injectors, cam phasers, AVS, engine solenoids).

---

### 1.4 `Melexis MLX14610` (`A2C819000`), `TI LM2903-Q1`, analog front-ends, and precision resistors (`PHOTO`)

- **`Melexis MLX14610 / CB-1914 / A2C819000 / S27472` (SSOP-16, `x=890..1025, y=1695..1815`):**
  - Settles the blurry `MLX1461O(?) CB-1510` reading from the old third-party image (`PHOTO`).
  - **Connector-side routing (`PHOTO`):** five left-side input pins trace on the top layer via `x = 730` filter MLCCs
    directly to **`T91a/62`**, **`T91a/61`** (brake/clutch switch circuits `F/F47/G476`, accompanied by two **`302` =
    3.0 kΩ** wetting/pull resistors), and **`T91a/77, 76, 75`** (adjacent vehicle switch inputs).
  - **MCU-side routing (`PHOTO` + `VERIFIED-S`):** four parallel logic traces exit rightward past a **`103` (4×10 kΩ)**
    8-pin resistor array into vias heading toward the `TC1791` left-edge port balls (`HYP` pairing with the BSW active-low
    switch getters: `P9.4` brake test switch `0x800A55D4`, `P0.3` P/N-clutch starter interlock `0x800A6880`, `P0.12`
    `0x800A6890`, `P4.7` `0x800A6830`; REA `ev_7fde7f07797e…`). No SPI chip-select or `ASC0` LIN line connects to it.
  - A **second unpopulated SSOP-16 footprint** sits immediately below-right (`x=1015..1145, y=1870..2010`) for vehicle
    variants requiring additional conditioned switch inputs.
- **`TI 2903Q1 / 94KG4` (`LM2903-Q1`, SOIC-8, `x=1355..1495, y=1985..2095`):** automotive dual differential comparator
  with precision threshold resistors (`2491` = 2.49 kΩ, `223` = 22 kΩ ×2, `104` = 100 kΩ ×2, MELF resistors) and an
  adjacent 6-pin SOT-363 device.
- **Analog / knock front-end candidates (`PHOTO` + `HYP`):** an 8-pin VSSOP-8 marked **`V32 / A702 / 1011`** sits at the
  bottom-left analog corner of `TC1791` (`x=1750..1825, y=2365..2455`) alongside SOT-23-6 **`AOH`** (also at
  `x=2585, y=1925`), SOT-23 **`TR8`** (2×), and 5× SOT-363 dual clamping arrays (`A4t` `BAV99DW` ×3, `3Ft` ×2) above
  `ATIC140 #2`.
- **Precision 0.1% thin-film blue resistors (`PHOTO`):**
  - Upper `T91a` sensor bank (`x=800, y=1000..1180`): **`2151` (2.15 kΩ)**, **`2611` (2.61 kΩ)**, and a third blue
    resistor alongside `472` (4.7 kΩ), each wired individually to adjacent `T91a` filter MLCCs (NTC temperature sensor
    5 V pull-up precision references for `G83`/`G299`, `HYP`).
  - Next to `T91a/5, 6` (`KL87` blades, `x=915, y=2050`): **`2211` (2.21 kΩ)**.
  - Above `ATIC142` / adjacent to the `VBOOST` bulk-cap through-hole (`x=2450, y=2680`): **`1743` (174 kΩ)** near `103`
    (10 kΩ), `363` (36 kΩ), `223` (22 kΩ) (`HYP` candidate `(174 kΩ + 10 kΩ)/10 kΩ = 18.4` for the `18.2×` `VBOOST`
    sense divider; inner-layer continuity `UNVERIFIED`).
  - Supply sense dividers for `KL15` (`T91a/50` → `ADC2.CH4`, `NC_V_IGK_BRI_RATIO = 5.701171875` `VERIFIED-FR` p.23942),
    `KL87` (`T91a/5,6` → `ADC2.CH5`), and `KL30` (`T91a/86`) route across inner PCB layers to the right-edge ADC balls,
    so complete two-resistor ratios for `KL87`/`KL30` cannot be proven from top-side copper alone (`UNVERIFIED`).

---

## 2. Interfaces the firmware uses

### 2.1 SSC jobs (BSW SPI scheduler)

Job start `0x800A8CE8` (bus `d4`, job `d5`): bus structs at `*(0xD000086C)` = `0xD0006BC4 + 0x28·bus`, per-bus config
`0x80042A4C` (bus 0: SSC0, 3 jobs) / `0x80042A64` (bus 1: SSC1, 8 jobs), job descriptors `0x8004291C` / `0x8004294C`
(16 bytes: buffer offset, length, width, segmentation, two callback-chain pointers), RAM job table `0xD0006C14` /
`0xD0006C54` (SSOC mask, BR, CON). VERIFIED-S/-T (RAM dump after boot). End-of-job callbacks are chains
`{fn, arg}` ending in the parser (`0x800971AC/B4` are generic dispatchers).

| Bus/job | SLSO (pin) | Rate, format | Words | TX (traced) | End-of-job → parser | Device | Emulator before → now |
|---|---|---|---|---|---|---|---|
| 0/0 | 0 | 1 MHz, 16 bit MSB | 10 | ATIC121 init | — | ATIC121 (`ST A2C00057927`) | modelled |
| 0/1 | 0 | 1 MHz, 16 bit MSB | 6 | `B816 B816 B819 B801…` | `0x80099B44` | ATIC121 | modelled |
| 0/2 | 6 (P2.6) | 1 MHz, **8 bit** MSB | 6, bursts 2+4 | `51 00 / 00 00 00 00`, once `D2`, `5D` | `0x800ABD40` | PIC12LF1822 | zeros → `pic_eot.rs` |
| 1/0 | 2 (P2.2) | 1 MHz, 16 bit MSB | 6, bursts 2+4 | `AAAA 177A / 1773 2665 177A 2665` | PSPR `0xC0002364` → `0x800A9BE8` | ATIC139 LS (`SC900697AE`) | zeros → `atic139.rs` |
| 1/1 | 2 | 1 MHz, 16 bit MSB | 4, bursts 2+2 | `AAAA 00FD/00FE/00FB / F001 F001` | PSPR `0xC0002340` → `0x800A96EC` | ATIC139 PS | zeros → `atic139.rs` |
| 1/2 | 2 | 1 MHz, 16 bit MSB | 7 | MU protocol | `0x80062FD0` | ATIC139 MU | modelled |
| 1/3 | 3 (P2.3) | 1 MHz, 16 bit **LSB** | 4 | `5555 1000 1000 1000` (wire order) | table `0x80041AF8` → `0x800ABE98` | ATIC111 ×3 (`ST A2C00693000`) | modelled |
| 1/4 | 4 (P2.4) | **250 kHz** (BR 0xB3), 16 bit | 6 | `C000 A000 B1E8 0666 4000 4000` until accepted | PSPR `0xC0002388` | ATIC142 (`TI A2C49811B1`, §6) | zeros → `atic142.rs` |
| 1/5..7 | 4 | 250 kHz | 13 + 13 / 9 | measurement frame / configuration download (§6) | `0xC0002388` / `0xC000265C` | ATIC142 | `atic142.rs` |

P2.5 is configured as SLSO15 (IOCR `0xA0`) but no job uses it (HYP: second lambda IC of other variants,
`NC_NR_LAMB_IC`). SSC2: CLC enabled only; SSC3: untouched. Each SSC job of the ATIC139 and the PIC is deliberately split
into two chip-select bursts by the job start (the select line is released between them, VERIFIED-T); both devices keep
the reply across the select edge (the parsers read the reply to the last word of burst 1 in burst 2).

### 2.2 Other peripherals with off-chip partners

| Peripheral | Firmware use (VERIFIED-T, MMIO log) | Partner | Model |
|---|---|---|---|
| MSC0 / MSC1 | DC/DD/USR programmed, command + data frames | ATIC140S #0 / #1 (`A2C00757700`) | modelled |
| MultiCAN node 0 / 1 | NBTR `0x2A8B`, NPCR not written (RXSEL 0 = P6.8 / P6.10, VERIFIED-DS p.48–49); nodes 2/3 only initialised | `TJA1043T` (+ unpopulated 2nd transceiver footprint, `PHOTO`) | frame level (`multican.rs`) |
| ASC0 (`0xF0000A00`) | only CLC = `0x100` written (REA `ev_7fde7f07797e…`), no other access in boot + after-run | unused in normal operation (test / K-line / LIN option, HYP) | — |
| SENT module at `0xF0321000` (8 channels, DS p.21; **SENT0 input = `P8.0/IN40` ball `F1`, VERIFIED-DS p.52** — *correcting earlier `P5.0` typo*) | logical channels 0/1/2 = hw 0/2/3 configured (CPDR 5, CFDR `0x9F1`/`0x9D7`, RCR `0x10620`, 6 data nibbles), only hw 0 enabled; data ISR SRPN 0x36 (SRC `0x9FC`), error ISR 0x37 (`0x9F8`) | hw 0 = position sensor of the rotary coolant valve N493 (GPTA doc §12.9) | `sent.rs` (receiver + J2716 sensor side) |
| FADC `0xF0100400` | channels 1..3 configured, ch 1 = knock sensor `G61` (`FAIN1P`) | `G61` piezo knock sensor + front-end (§9) | `knock_fadc.rs` |
| Ports | DIO read `0x800A79E0` / write `0x800A7ADC` (id = port·16+pin, P12+ = (port−11)·256+pin, `0x1000`/`0x2000`+n = MSC0/1 data bit n) | relays, KL50, oil switches, `MLX14610` conditioned switches | partial (`power_stages.rs`) |

---

## 3. ATIC139 = MU + LS + PS on SSC1/SLSO2 (`SC900697AE / A2C00704400`)

Evidence that the "MU" is a function block of the Freescale/NXP `ATIC139/A2` (`SC900697AE`):
1. The MU job and both `0xAAAA` jobs use the same select line SLSO2 (SSOC mask `0x04`, VERIFIED-T); there is no
   additional port-pin chip select in the job or in the trace.
2. The BSW driver descriptor `0x8004CBF0` holds the MU fixed-word builder `0x80081724` together with the init routines of
   both `0xAAAA` jobs (`0x801127FC` → PS struct `0xD000B258`, `0x80112998` → LS struct `0xD000A118`) (VERIFIED-S).
3. The image contains exactly two ATIC139 failure names, `MCC_ATIC_139_LS` (`0x80045BA8`) and `MCC_ATIC_139_PS`
   (`0x80045BB8`) (VERIFIED-S); DAMOS IDX_ERR `0xA6` / `0xA7`, OBD P0603, customer DTC `0x5653` / `0x5654`. Every other
   SPI select line is accounted for by another chip (§2.1).
4. FR p.5688: the MU "might just as well be realized by an ASIC" and drives the power-stage disable lines and the MC
   reset (VERIFIED-FR); **PHOTO**: `ATIC139/A2` is directly flanked by 2× Infineon `2N06L64` (60 V MOSFETs) and 2× ON
   `B36` (`MBRS360`, 60 V Schottkys) forming its switch-mode pre-regulator (§1.3).

Which `0xAAAA` job is "LS" and which "PS" is HYP (3 diagnosed channels = low-side stages, 2 switched outputs = supply
outputs). PS output 1 is switched off 0.4 s after the second relay at the end of the after-run (`00FE` → `00FB` at
6.0 s, KL15 off at 3.0 s, VERIFIED-T) — HYP: power-latch / sensor-supply output.

Protocol and decoders: `crates/simemu-tc1791/src/atic139.rs` header. The model answers with the header echo and the
"healthy, all diagnoses executed" codes derived from the parsers; undecoded reply bits are 0 (UNVERIFIED).

---

## 4. PIC12LF1822 = engine-off timer on SSC0/SLSO6

FR ENSS 33.4 "Engine off duration" (p.8334–8338, VERIFIED-FR): `STATE_T_ES_RD` / `STATE_T_ES_VLD_RD` = "status /
validity of external clock read from PIC"; `STATE_T_ES_TYP_INFO`: 1 = engine-off time from PIC, 2 = from ATIC
(`ACTION_INFR_GetTEsTmrAtic` exists, i.e. an ATIC can provide the timer on other boards). DAMOS (SC8S50):
`LC_STATE_T_ES_CLC_SWI` = 1 (PIC, not CAN), `LC_STATE_T_ES_TYPE_REQ` = 0 (PIC), `C_T_ES_HW_MAX` = 172000 s,
`LC_T_ES_HW_RST_AST` = 1. Failures `T_ES*` → OBD P2610 (customer `0x59DA`, `0x53AB`, …).

Protocol (driver struct `0xD000B288`, parser `0x800ABD40`, command selector `0x800ABB18`, state machine `0x800ABA04`
with the limit `0x15180·(p+1)` s): `crates/simemu-tc1791/src/pic_eot.rs` header. The firmware reads `0x51` every
10 ms (p = 1 → 48 h range). An 8-bit-only SPI slave matches the PIC's MSSP; no other chip on the board answers 8-bit
frames.

**Engine-off time (VERIFIED-T, DSPR diff of two runs with the PIC at 0 s and 3600 s).** The firmware takes the PIC value at
power-up directly as the engine-off time: T_ES_H `0xD0012490` = 3600 s, T_ES `0xD0012488` = 60 min (raw copy T_ES_H_RD
`0xD0008E68`, ENSS `0x801A83D4`). The PIC is reset 2 s after each start (`c_t_es_ast_rst_dly`, LC_T_ES_HW_RST_AST = 1). It feeds
the dew-point detection (EXTD 39.7: start temperatures from NVM for short stops) and the start enrichment, so the emulator
treats it as a bench input set before power-up (`StudioSession::eot_clock_s`).

**Physical 8-pin SOIC-8 trace verification (`PHOTO`, `20261111_184641.jpg` & `IMG_2507.DNG`):**

| Pin | `PIC12LF1822` function | Observed PCB copper connection (`PHOTO`) |
|---:|---|---|
| 1 | `VDD` | Permanent 3.3 V standby rail + 0603 decoupling MLCC + circular ICSP test pad |
| 2 | `RA5` | **NC** — routed solely to an isolated circular test pad with **zero vias and zero outgoing tracks** (confirms internal `LFINTOSC` is used, no external crystal) |
| 3 | `RA4 / SDO` (`APFCON.SDOSEL=1`) | Top-layer trace heading up-left to via → `SSC0 MRST0` (MISO) |
| 4 | `RA3 / MCLR_N / SS_N` (`APFCON.SSSEL=1`) | Circular ICSP `VPP` test pad + via → `SSC0 SLSO06` (`P2.6`) |
| 5 | `RA2` | **NC** — isolated pad with **zero vias and zero outgoing tracks** |
| 6 | `RA1 / SCK / ICSPCLK` | Circular ICSP `CLK` test pad + via → `SSC0 SCLK0` |
| 7 | `RA0 / SDI / ICSPDAT` | Circular ICSP `DAT` test pad + via → `SSC0 MTSR0` (MOSI) |
| 8 | `VSS` | Direct thermal relief spokes into top-layer **GND plane** |

Because all 8 pins are accounted for (`VDD`, `VSS`, 4 SPI wires, 2 unconnected pins), **there is physically no wake-up or
alarm line from `PIC12LF1822` to `ATIC139` or `TC1791`** (`PHOTO` + `VERIFIED-T`), settling Open Question #2 regarding wake
wiring. PWSL "wake up signal" (FR 64.6) is a separate KL15-type vehicle input.

---

## 5. TJA1043T CAN transceiver (`PHOTO` + `VERIFIED-S/-T`)

**Firmware side (`VERIFIED-T` + `VERIFIED-DS`):** CAN nodes 0/1 are configured on `P6.8/P6.9` and `P6.10/P6.11` (IOCR
`0x90` on `P6.9`/`P6.11` = alternate output 1 = `TXDCAN`, `RXSEL 0 = P6.8 / P6.10` DS pp. 48–49; nodes 2/3 only
initialised). No static DIO pin id of `0x800A79E0`/`0x800A7ADC` and no port write in boot + KL15 off + 13 s after-run
switches a CAN transceiver mode.

**PCB copper trace verification (`PHOTO`, `IMG_2507.DNG` `x=1095..1430, y=1290..1650`):**
- **Single populated transceiver + unpopulated 2nd CAN footprint:** `NXP TJA1043T` (`CRW00041 / TnD19201`, SOIC-14,
  mounted 180° rotated with **Pin 1 at top-right**) sits at `x=1095..1265, y=1435..1580`. Immediately to its right and
  above (`x=1255..1425, y=1295..1545`) sits a **complete unpopulated 2nd SOIC-14 CAN transceiver + common-mode choke +
  split-termination footprint**, explaining why BSW initializes both MultiCAN node 0 and node 1 while `06K 907 425 B`
  exposes only one physical CAN bus (`T91a/79, 80`).
- **14-pin connection map of the populated `TJA1043T` (`PHOTO`):**
  - **Pin 1 (`TXD`, top-right)** & **Pin 4 (`RXD`, top 4th from right):** thin logic tracks route to inner-layer vias
    heading toward the `TC1791` `P6.8..P6.11` corner (which of `CAN0` vs `CAN1` connects to the populated footprint is
    routed on inner layers, `UNVERIFIED`).
  - **Pin 2 (`GND`, top 2nd from right):** thermal spoke directly into top-layer GND plane + ground via.
  - **Pin 3 (`VCC`, +5 V)** & **Pin 5 (`VIO`, +3.3 V):** decoupled locally by two 0603 MLCCs to GND above the IC and fed
    from inner supply vias.
  - **Pin 6 (`EN`, top 2nd from left)** & **Pin 7 (`INH`, top-left):** each routes straight up to an inner-layer via.
  - **Pin 8 (`ERR_N`, bottom-left):** routes straight down to an inner-layer via (not read by BSW static DIO).
  - **Pin 9 (`WAKE`, bottom 2nd from left):** tied through the adjacent **`472` (4.7 kΩ)** resistor (local wake unused).
  - **Pin 10 (`VBAT`, bottom 3rd from left):** wide trace down to an 0805 decoupling MLCC to GND + battery supply via.
  - **Pin 11 (`SPLIT`, bottom 4th from left):** trace runs up underneath the SOIC-14 body, loops over choke `H9264`, and
    connects to the **center tap between the two `61R9` (61.9 Ω, 1%) termination resistors** (`123.8 Ω` split
    termination) + split capacitor to GND.
  - **Pin 12 (`CANL`, bottom 5th from left)** & **Pin 13 (`CANH`, bottom 6th from left):** differential pair runs left
    underneath the IC body into common-mode choke **`H9264`**, whose left pads connect across the `61R9` split
    termination directly to **`T91a/80` (`CANL`)** and **`T91a/79` (`CANH`)**.
  - **Pin 14 (`STB_N`, bottom-right):** connects down through an 0402 strap/resistor + test pad to an inner-layer supply/
    control via. Together with the absence of any firmware standby GPIO toggle (`VERIFIED-T`), `TJA1043T` operates
    permanently in Normal mode while board rails are up; a transceiver mode state machine has no observable effect.

---

## 6. Wide-band lambda IC ATIC142 on SSC1/SLSO4 (`TI A2C49811B1`) — modelled (`atic142.rs`)

**Chip (`PHOTO` + `VERIFIED-FR`).** Physically identified on the top side next to `ATIC121 B3` (`x=2330..2500,
y=2880..3080`, `20261111_184722.jpg` & `IMG_2507.DNG`) as **Texas Instruments custom Continental ASIC
`A2C49811B1 / 9G15AA`** in a **TSSOP-24** package, flanked by two SMB clamping diodes and precision biasing passives.
FR EGCP 22.5 "O2 sensor (lin, up) SPI interface ASW - BSW" (p.5964–5968, VERIFIED-FR) names it `ATIC142` (not a Bosch
CJ125: no CJ125 command words occur in the traffic). It serves **both** sensors: the wide-band upstream sensor (`G39`,
`T91a/41, 43, 44, 74`, two-cell LSU 4.9 class, own pump-current controller, Ri measurement by an AC current) and the
binary downstream sensor (`G130`, `T91a/25, 26`) on its flexIO pin (FR 22.2: GetWrafMeasLin / GetWrafMeasFlexIO). No λ
signal comes over the ADC. 32 configuration words `C_STATE_LAMB_IC_CONF_VALUE` (SC8S50: `F000 0016 0000 0000 0000
0079 … 05A8 0DD5 0000 0F55 05F9 0FB0 0FFF 0079 …`, DAMOS) are downloaded at start and read back cyclically.

**Interface (VERIFIED-T/-S).** SSC1, SLSO4 = P2.4 (IOCR `0xA0`, DS p.33), 16 bit MSB first, BR `0xB3` → 250 kHz. Jobs of
bus 1 (handles = bus·0x2000 + job·0x80; device config `0x800445D0`: +0x10/+0x14/+0x18 → handles `0x2200`/`0x2280`/
`0x2380`; `NC_NR_LAMB_IC` = 1 at `0x800445F4`). One word latency like ATIC139 (reply to TX k = RX k+1, kept across
select edges). A reply carries a response ID in bits 15:12 and 12 data bits, or 16 data bits for high-resolution reads.

| Job | Words | Use | TX | End of job |
|---|---|---|---|---|
| 4 (`0x2200`) | 6 | init handshake, every 10 ms until accepted | `C000 A000 B1E8 0666 4000 4000` (`0x80042FA0`) | `0xC0002388` state 0/2 |
| 5+6 (`0x2280`) | 13+13 = 26 | measurement frame, every 10 ms (`0x801F4C00`) | see below | `0xC0002388` state 1 |
| 7 (`0x2380`) | 9 | configuration download `D54c` + 8 words, c = 0..3 | template `B000 C000 …` `0x80042F8C` | `0xC000265C` |

Measurement frame (built by init `0x80113470..0x80113760` + cyclic `0x801F4C00`; mode 0 from cfg+0xC = 2, Ex00 reads
because cfg+0xD = 1; slot index tables `0x80042FAC` + 0x18·mode): `6000 7000 4000 Cxxx Bxxx DD4i DD4i E100 1400 1600
1C00 1700 E100 1800 EA00 E500 1B00 1900 E300 2xxx 3xxx 80xx 90xx Axxx F000 E100` (REA `ev_2fd9602f…`, `ev_4b84417b…`).

| Word | Meaning | Evidence |
|---|---|---|
| `1r00` / `Er00` | read register r (reply ID r, 12 bits) / high-resolution read (16 bits) | handler `0x8012A786` (ID checks), REA `ev_e9a9dfd8…` |
| `E100`, `E300` | IPLSL_RAW / IPLSL_MMV_SEG pump current, i16 & 0xFFFE · 366.39 nA | INFR `0x800A5878` = GetWrafMeasLin (REA `ev_f2265c1e…`) |
| `1400`, `1700`, `1800`, `1600` | VSLSL, VPLSL, VRCLSL (signed), VGLSL (unsigned), 8.144 · 152.588 µV = 1.2427 mV/bit | same |
| `E500` | V_R_IT_LSL Ri AC amplitude, 14 bits in 15:2, 0.3107 mV/bit | same |
| `1900`, `EA00`, `1B00`, `1C00` | binary sensor V_LS_SENS, V_R_IT_LS, flexIO pin, VG pin | INFR `0x800A57B8` = GetWrafMeasFlexIO |
| `4000` | applied switch state (FR p.5933 read-back layout) | GetWrafProt `0x800A5994` |
| `6000` / `7000` | DIAG_EL / DIAG_LIM bit vectors, accumulated and read-and-clear in the BSW | `0x801F4F08` / `0x801F4FB8` |
| `Bxxx` | switch request in the read-back layout, from STATE_LSL/LS_PROT_STR_REQ (MES = `0x9E8` for this sensor type) | ASW `0x801A125E..0x801A135E` → setter `0x801F5084` |
| `Cxxx` | control flags: bit 0 = 0xD0001D39 (constant-current mode), bit 3, 8, 11 = 0xD0001D38/3A/3B, bits 2:1 = 2-bit state, **bit 10 = 0xD0001D36 ∧ ¬0xD0001D39** (setter `0x801F50DC` clears bit 10 when bit 0 is set) | packer `0x801A1360..0x801A13A6` |
| `80xx` / `90xx` | Icp + Ri AC range of the linear / binary sensor: byte = R_IT_RNG << 5 + boost << 4 + Icp (3.75 µA steps) | packer `0x801A11F6..0x801A123E` |
| `Axxx`, `3xxx`, `2xxx` | IPLSL_MAN (current when the controller is off), Ip-controller correction, segment-averaging length | SetWrafCtlLin `0x801F4AF8` |
| `D54c` + 8, `DD40 \| (i&7)<<3 \| i>>3` | configuration write chunk c / read-back of word i (16-bit compare) | `0x801F4D92`, `0x8012A8C4` |

Ri AC currents per range code (what the ASW divides by): linear sensor 30/60/120/240/480/1000/3000 µA (`0x801A1752..0x801A17DE`,
= FR 22.6; FR 22.5's C_R_IT_RNG_LSL description lists another set), binary sensor 30/120/480/1000/2000/5000 µA
(`0x801A14C2..0x801A14FC`, = FR 22.5). Cold/hot ranges 1/4 (linear), 1/2 (binary) from `c_idx_acri_*`.

**Init acceptance** (`0xC0002388` state 0): RX[1] ID C with `0x0D0F` clear, RX[3] = `B1E8`, RX[5] ID 4 with `0x1E8` clear — so
the `B1E8` switch request written in the same frame must not yet show in the read-back: the model applies a `Bxxx`
request at the next chip select (HYP: switching time).

**Model** (`crates/simemu-tc1791/src/atic142.rs`): protocol as above; front end fed by the plant every 1 ms
(`studio::io::feed_lambda_sensors`): pump current = the cell's limiting current while VIP/VG/VN are connected and `C` bit 10
is set (HYP: LV_IPLSL_ENA), Nernst voltage = 450 mV set-point (LSU class) or the open-cell EMF, plus Icp·Ri; Ri reading =
Ri · I_ac(range); disconnected pins read 0 V; no fault injection (DIAG words 0). UNVERIFIED: VG = 2.5 V, the pump-cell
resistance, the controller's own limits and dynamics, segment averaging (MMV_SEG = raw).

**Firmware reaction (VERIFIED-T, `studio::engine_run`, SC8S50, warm start).** Accepted at power-up (driver state `+0x26` = 1),
configuration download SUCCESS (`+0x27` = 3), no COM/CONF errors. The ASW holds the switches open until the dew-point
phase ends (heater at `c_v_efc_red_lsh_lsl` ≈ 2 V), then requests MES (`B9E8`), measures Ri (range 1 → 4), closes the heater
loop at its 785 °C set-point (plant tip 778–781 °C, Ri ≈ 303 Ω), enables the pump controller (`C500`/`CD00`) and reports
closed loop on OBD PID 0x03 (see GPTA doc §12.5).

**Heaters (VERIFIED-S/T).** Upstream heater = BSW PWM channel 5 = LTCA2 LTC16 quad, output L2.LTC19, 100 Hz; downstream =
channel 9 = G1.LTC56 quad, output G1.LTC59, 10 Hz (ASW `0x8019B258` / `0x8019B2CC` check c_vb_max_prot_lsh_lsl / _ls and call
the INFR setters `0x800A59E4` / `0x800A59D0`). Duty = (b + 1)/256 of the byte `0xD000C56C` (up) / `0xD000C56B` (down) =
V_efc²/V_b²; **low = heater on** (dew-point phase: bytes 5/20 = 2.3/8.2 % = the low fractions of the two cells). The power
stage behind the cells (`Z29` on `T91a/11` and upstream `G39` heater) sits on the `T91a` side (HYP: upper `ATIC140S B1`
high-current low-side stages, §1.3).

**Board supplies found on the way (VERIFIED-S/T).** With the IC accepted, lambda signal acquisition still stayed off: the
operating strategy `0x8019AD40` returns while `0xD0000DA5` ≠ 0, cleared only if FID `0x3CD` is granted (`0x8019C3F8..`;
FID status = signed byte `0xD000D605 + FID` ≥ 0, `0x800F4EAC`). FID `0x3CD` is inhibited (event→FID table `0x808353D6` /
`0x808305B6`, `0x800F4F14`) by the heater/sensor circuit events, LAMB_IC_COM/CONF, … and **RLY_MAIN_PLAUS_ON**, which was
failed: it compares V_RLY_MAIN_MES_H_RES (`0xD001518A`, average of `0x801074A8`) with c_v_rly_main_plaus_on_diag 5.0 V
(`0x801DF674`). The after-relay supply KL87 is **ADC2.CH5** (GetVRlyMainBas `0x800A66A0`, logical `0x8007`) and was never fed.
Feeding it unmasked further supply diagnoses: the 5 V sensor references A/B/C are **ADC0.CH1/CH2/CH3** at half scale
(GetVccBas `0x800A65F4`, logical `0xE/0xD/0xC`; 0 V → VCC_L_0..2, 5.0 V at the pin → VCC_H_0..2, 2.5 V → none) — without them
the firmware lost crank synchronisation. Both are fed by `studio::io` now (KL87 divider assumed equal to KL15, UNVERIFIED).
Still failed after these (not investigated, none inhibits FID `0x3CD`): CONV_PLAUS *(since fixed via `PVS2_ADC2 = ADC2.CH0`)*,
PORT_FB_EL_L_1, RVC_DIG_COM, VCV_OC, ETC_PWM_2.

---

## 7. Silent faults (zero replies) — what the firmware concluded

ERRM status array: 2 bytes per IDX_ERR at `0xD00125E4` (writer `0x800947C8`; byte 1 bit 0 = reported, bit 1 =
failed). IDX_ERR = position of `C_ERR_CLAS_<name>` in its DAMOS table.

| Device | With zero replies | Firmware conclusion | With the new model |
|---|---|---|---|
| ATIC139 LS/PS | RX[1] ≠ `0xAAAA` | LS state `+0x1C` = 2, PS `+0x18` = 2; `MCC_ATIC_139_LS`/`_PS` failed (status `0x1F 0x7B`, read at 2 s and 5 s) | states 3, both reported without failure (`0x08 0x05`) — test `board_chips_e2e` |
| PIC | RX0 ≠ `0x2x`, RX5 type ≠ `0xC0` | `STATE_T_ES_VLD_RD` = 4 (invalid), ASW issued extra `0xD2`/`0x5D` reads; `T_ES*` diagnoses not executed within 16 s | validity 1, firmware reads a 1 Hz clock — test `board_chips_e2e` |
| ATIC142 | zeros | `LAMB_IC_*` not tested in 16 s; IC never accepted, no λ data | accepted, configuration SUCCESS, Ri/heater/pump-current loop running (§6) |

An intermediate model that answered the header but returned invalid status words (MU words in the LS slots, zero PS
status) made the firmware set `RLY_MAIN_EL_SCP` (main relay, electrical short to battery) instead of the MCC errors:
one of the ATIC139 LS/PS channels carries the main-relay output diagnosis (VERIFIED-T; which channel was not separated).

UDS `19 02 FF` after 5 s returned 63 stored DTCs, all with status `0x40`/`0x50` (not completed) both before and after:
the MCC failures stayed inside ERRM (not stored as DTC within 5 s). Remaining failures of a bare board run (no plant):
`FUP_L`, `FUP_EFP_L` (no fuel pressure), `PVS_L_1/2` (no pedal voltages), `RLY_MAIN_PLAUS_ON` — inputs, not chips.

---

## 9. Knock-sensor input, ATIC111 H-bridges, cam-phaser and flap stages

Details and firmware evidence: `docs/SIMOS18_GPTA_IO_RE.md` §12.6–§12.11.

**Knock input → FADC (`crates/simemu-tc1791/src/knock_fadc.rs`).** One piezo knock sensor (`G61` on `T105a/97, 98`, EA888
Gen3, KS4 class: 26 mV/g at 5 kHz, main resonance 30 kHz, 1150 pF — Bosch Motorsport KS4-P data sheet) on FADC channel 1,
single-ended FAIN1P against VFAREF/2 with the FADC's ×2 gain (3.23 mV/code). The board's input stage (`HYP`: 8-pin VSSOP-8
`V32 / A702` at the MCU analog corner, §1.4; transfer gain UNVERIFIED) is modelled as AC coupling onto a VFAREF/2 = 1.65 V
bias with gain 1 (`KNOCK_FE_GAIN`): the firmware's own filter starts from mid-scale (511.5 codes), so the bias is mid-rail.
The window gate is a GPTA output (G0.GTC24 → GPTA0 OUT26 → FADC GS7, HYP for the TC1791), the samples go through the FADC
filter blocks (12-tap FIR, 9:1) and DMA channel 11 into RAM; the CPU filters them. Cost: conversions are produced only while
the gate is open (≈ 30 % of the time, 891 kHz), one complex rotation per live vibration atom per conversion.

**ATIC111 H-bridges (`ST A2C00693000` ×3, `PHOTO` + `VERIFIED-S`).** All three PowerSSO-24 H-bridge ASICs sit in the
bottom-left corner of the top side (`IMG_2507.DNG`, `20261111_184702..184715.jpg`), with heavy top-layer output copper pours
routing directly to `T105a/86..91`:

| Logical bridge | Actuator | Physical IC on PCB (`PHOTO`) | Traced `T105a` output pins (`PHOTO`) | Firmware `PWM` / `DIR` / `DIS` (`VERIFIED-S`) |
|---:|---|---|---|---|
| Bridge 0 | `J338 / G186` Throttle valve motor | **Upper `ATIC111`** (`x=845..1050, y=2855..3055`) | **`T105a/90` & `T105a/91`** | `G1.LTC27` (`P4.1`) / `P14.4` / `P6.7` (`MTC_DIS`) |
| Bridge 1 | `N493` Rotary coolant valve | **Bottom-Left `ATIC111`** (`x=835..1035, y=3360..3560`) | **`T105a/86` & `T105a/87`** | `G1.LTC31` (`P4.4`) / `P4.5` / `P13.9` |
| Bridge 2 | `V465` Wastegate actuator (`BPA`) | **Bottom-Right `ATIC111`** (`x=1105..1310, y=3375..3575`) | **`T105a/88` & `T105a/89`** | `L2.LTC35` (pin undecoded) / `P13.6` / `P4.9` |

Disable pins are active-high (`high = disabled`); hardware `EN = MU NDIS1` (UNVERIFIED for every chip). The SPI signal leads
(`SLSO13 = P2.3`, `SCLK1`, `MRST → MTSR` daisy chain) drop immediately into vias next to the PowerSSO-24 pads and route on
inner PCB layers, so **which SPI daisy-chain stage (`0/1/2`) corresponds to which physical bridge remains `UNVERIFIED`**; the
emulator's SPI status keeps `MTC_DIS`.

**Analog & digital sensor inputs.** ADC1.CH5 = wastegate position sensor `G581` (`T105a/41`, 0–5 V ratiometric, seat 3.931 V
learned, −33.33 %/V); ADC2.CH8 = port-flap position sensor `G336` (1.196 V closed / 3.799 V open nominal). SENT hw 0
(`P8.0/IN40`) = position sensor of `N493` (`sent.rs`, plant `cooling_valve.rs`). **Oil-pressure switches** (`T105a/72, 73, 74`):
codeword `c_idx_cod_poil_det_conf` = `0x7FF` (SC8S50, SC8O40) swaps and inverts H/L (bit 8 swap, bits 9/10 invert H/L; OPM
`0x808DF7A8`, init `0x800E9C24`): `LV_POIL_SWI_L = !P6.14` = reduced-pressure switch `F378`, `LV_POIL_SWI_H = !P7.5` = oil
pressure switch `F22` (VERIFIED-S). Piston-jet switch `F447` on `P6.15` (getter `0x800A5A80 = !pin = LV_POIL_SWI_PIST`, 1 =
pressure) follows jet-gallery pressure behind `N522`. All three close to ground at pressure; switch points 0.65 / 2.65 / 0.45
bar = mid-band of EA888 Gen3 replacement parts (UNVERIFIED). **Oil level / temperature sender `G266`** (`T105a/83`) on IN12 =
`P3.4` (`engine_io::LINE_G266`): BSW captures period and duty of the G1.LTC40 quad at each falling edge (CLK7 = 21.97 kHz;
`0x8008DCBC`); ENLU 27.3 (`0x801A36F0`) decodes the TOG frame (`tests/oil_sensors.rs`, VERIFIED-T). `G266` does **not** feed
`TOIL` on SC8S50/SC8O40/SC8O30 (`CLF_TOIL_SRV_INTL_DI_CONF` bit 0 = 1).

**ATIC140#1 low-side stages decoded** (MSC1 data bit n): 0/1 cam phasers N205/N318 (`T105a/105, 104`, signal tapped at
G0.LTC27 / L2.LTC31, low = on, because the TC1791 GPTA→MSC multiplexer encoding is unverified), 2 N428 oil-pump stage
(`T105a/17`), 4 N522 piston jets (`T105a/7`), 6/7 starter relays (`T91a/87, 88`), 9 = N80 purge (`T105a/3`, not decoded: no
plant consumer), 10 N249 diverter (`T105a/66`), 11 N316 port flaps (`T105a/53`), 16..19 exhaust AVS actuators (`F366..F373`,
both chips).

---

## 8. Open questions — settled by `simos-images/` vs remaining measurements

### Settled by the `06K 907 425 B` (`H13`, PCB `A2C80473700-04`) inspection (`PHOTO` + `VERIFIED-DS/-S`)
- **Former #5 (Unidentified IC, `ATIC139` discretes, and "bottom-side" ASICs) — SETTLED:**
  - All major ASICs (`ATIC121 B3` `ST A2C00057927`, `3× ATIC111` `ST A2C00693000`, `ATIC142` `TI A2C49811B1`, and both
    `ATIC140S B1` `A2C00757700`) sit on the **top side** of `A2C80473700-04`; only the BAP MEMS sensor and GDI boost
    inductor / bulk electrolytic capacitor sit on the bottom side (§1).
  - The 60 V `2N06L64` ×2 + `B36` (`MBRS360`) ×2 stage next to `ATIC139/A2` is definitively `ATIC139`'s switch-mode
    pre-regulator, while the 65 V GDI DC/DC boost converter is the `30N10L25` (100 V) + `R022 F` (22 mΩ) stage next to
    `ATIC121 B3` (§1.3).
  - `MLX1461O(?) CB-1510` is **`Melexis MLX14610 / CB-1914 / A2C819000`** (SSOP-16), conditioning `T91a/61, 62, 75, 76, 77`
    switch inputs into 4 logic lines toward `TC1791` (§1.4).
- **Former #2 (PIC wake/alarm pin) — SETTLED:** Pins 1..8 of `PIC12LF1822` are completely traced on top-layer copper (§4):
  pins 2 (`RA5`) and 5 (`RA2`) are unconnected test pads with zero vias, proving no physical wake/alarm line exists.
- **Former #3 (`TJA1043T` CAN bus & split termination) — PARTIALLY SETTLED:** Single populated `TJA1043T` (+ unpopulated 2nd
  CAN footprint), `H9264` choke, and `2× 61R9` (`123.8 Ω`) split termination to `SPLIT` (pin 11) and `T91a/79, 80` are
  verified (§5); inner-layer destination of `EN` (pin 6), `INH` (pin 7), `ERR_N` (pin 8), `STB_N` (pin 14), and `CAN0` vs
  `CAN1` pair would only require DMM continuity if ever needed.
- **`MCU` variant & `SCU_CHIPID` — SETTLED:** `SAK-TC1791S-384 F200EP AB` (`0x9600_AA02`) vs `TC1791F-512F240EP`
  (`0x8700_9502`) and firmware normalization `0xAA → 0x95` at `0x8011249E` (§1.1).
- **`ATIC111` physical bridge-to-actuator assignment — SETTLED:** Upper = `J338` (`T105a/90,91`), Bottom-Left = `N493`
  (`T105a/86,87`), Bottom-Right = `V465` (`T105a/88,89`) (§9).

### Still open (requires bottom-side photo, DMM inner-layer continuity, or live scope / logic-analyser capture)
1. **ATIC139 LS/PS (`SSC1/SLSO12`):** logic-analyser capture on a running ECU — real reply words (undecoded status bits,
   `0xF001` reply), DMM continuity from `ATIC139` pins to `T91a` (which channel carries main-relay `J271` diagnosis), and
   whether PS switches off the MCU rail at the end of after-run.
2. **PIC12LF1822 (`SSC0/SLSO06`):** SPI capture — real firmware version nibble, `0xD0..0xD2` diagnostic replies, mode bits
   `m` of `0x5x`, initial count after battery connect, and `LFINTOSC` drift over 1 h.
3. **ATIC111 SPI daisy-chain order & `V465` PWM pin:** DMM continuity on `MRST → MTSR` between the 3 `ATIC111` chips (inner
   layers) to determine which SPI shift stage (`0/1/2`) reports which bridge's fault bits; decode the MCU pin of `L2.LTC35`
   (`V465` PWM); measure `V465` mechanical stroke time, return spring rest position, and open-stop voltage.
4. **ATIC142 (`SSC1/SLSO14`):** live 250 kHz SPI capture with engine running — real `DIAG_EL`/`DIAG_LIM` words, `VGLSL`
   virtual-ground level (assumed 2.5 V), `C` bit 10 enable semantics, and `Bxxx` switch timing.
5. **Board supply dividers & λ heater stages (former #6):** DMM resistance measurement from `T91a/5,6` (`KL87` → `ADC2.CH5`)
   and `T91a/86` (`KL30`) to ADC pins / GND across inner layers, and continuity from `T91a/11` (`Z29`) / `G39` heater pins
   to the upper `ATIC140S B1` pins; confirm whether `1743` (174 kΩ) + `103` (10 kΩ) forms the `VBOOST` (`ADC2.CH1`) divider.
6. **Bottom-side PCB photo:** confirm package/part number of the barometric pressure (`BAP`) MEMS sensor and exact ratings of
   the GDI boost inductor and bulk electrolytic capacitor(s).
7. **Knock front-end (`G61` → `FAIN1P`):** scope `FAIN1P` (`V32/A702` output) with engine running (bias, gain, anti-alias
   corner) and confirm GPTA0 `OUT26 → GS7` gate wiring / channel-timer divider (`CTREL + 1` vs `+ 2`) via DMA sample count.
8. **Oil level/temp sender `G266` (`P3.4`), SENT (`N493` on `P8.0/IN40`), and oil-pressure switch points (`F378/F22/F447`):**
   live vehicle scope captures of the TOG pulse train and SENT fast/slow frames, plus mechanical switch-point tolerances.
