# SimEmu

**English | [Русский](README.ru.md)**

Hardware-level emulator for the **Continental SIMOS18.1** engine ECU (Infineon SAK-TC1791, TriCore 1.6) alongside the
**DSG7 DQ200_G2** (`0CW`, Infineon SAK-TC1784, TriCore 1.3.1) and **DSG6 DQ250 MQB** (`0D9`, Infineon SAK-TC1766, TriCore 1.3)
gearbox mechatronics, running in closed-loop co-simulation with a crank-angle-resolved **EA888 Gen3** (1.8 / 2.0 TSI)
engine, turbocharger, driveline, vehicle, and physical acoustics plant.

Factory stock and modified firmware images execute **byte-for-byte unmodified** (`Zero-Patch`) on SimEmu's pure-Rust
`tricore-core` CPU engine (enabled by default in CLI and Studio, achieving **~1.4–1.7× real-time** full ECU + TCU + physics
co-simulation) or against the optional Unicorn 2.1.4 differential oracle. Rust models only CPU architecture, on-chip MCU
peripherals, board ASICs, CAN bus transceivers, and physical laws (`Zero-Fake-Logic`) — every control decision (throttle,
spark advance, DI/MPI fuel injection, boost, wastegate, starter, gear selection, clutch pressures) is made by the unmodified
OEM firmware itself.

- Russian documentation: [`README.ru.md`](README.ru.md)
- **Firmware, memory regions (`rom/sboot.bin`, `rom/dflash.bin`), WFS5 keys & ECU/TCU boot guide:** [`docs/FIRMWARE_AND_BOOT_GUIDE.md`](docs/FIRMWARE_AND_BOOT_GUIDE.md) ([Русский](docs/FIRMWARE_AND_BOOT_GUIDE.ru.md))
- Architecture, crate graph & memory maps: [`ARCHITECTURE.md`](ARCHITECTURE.md)
- Verified milestones, benchmarks & roadmap: [`MILESTONE.md`](MILESTONE.md)
- Engineering invariants & contributor guide: [`CONTRIBUTING.md`](CONTRIBUTING.md)
- License terms: [`LICENSE`](LICENSE)

---

## Repository Layout

```text
simemu/
├── Cargo.toml             # Cargo workspace manifest (8 crates)
├── LICENSE                # PolyForm Noncommercial 1.0.0 + Additional Terms (© 2026 Matrix)
├── README.md              # English documentation (default)
├── README.ru.md           # Russian documentation
├── ARCHITECTURE.md        # Crate graph, MCU memory maps, peripherals & data flow
├── MILESTONE.md           # Verified results, benchmarks & known limitations
├── CONTRIBUTING.md        # Zero-Patch / Zero-Fake-Logic invariants & PR checklist
├── AGENTS.md              # Evidence rules and hardware specs for contributors & AI agents
├── rom/
│   ├── sboot.bin          # Default SIMOS18.1 supplier BootROM (SBOOT 112 KB for .frf / OBD reads)
│   └── dflash.bin         # Scrubbed SIMOS18.1 DFlash / FEE NVM image (256 KB, lab WFS5 keys)
├── crates/
│   ├── tricore-core/      # Pure-Rust Infineon TriCore 1.3 / 1.3.1 / 1.6 CPU core
│   ├── unicorn-tricore-sys/ # Optional FFI bindings to patched libunicorn 2.1.x (dev oracle)
│   ├── simemu-tc1791/     # SIMOS18.1 (TC1791), DQ200_G2 (TC1784) & DQ250 (TC1766) boards + ASICs
│   ├── simemu-physics/    # Crank-angle EA888 Gen3 combustion, turbo, dual-clutch gearbox & chassis
│   ├── simemu-acoustics/  # Physics-driven 1D duct & mechanical acoustic synthesis
│   ├── simemu-damos/      # DAMOS/A2L codec, native VAG .frf/.odx unpacker & CRC / ECM3 checksums
│   ├── simemu-bridge/     # MQB chassis CAN encoder & UDP telemetry bridge
│   └── simemu-cli/        # `simemu` CLI, browser Co-Simulation Studio & TCP Lawicel/SLCAN server
├── definitions/           # DAMOS calibration JSON schemas (SC8O40, SC8S50, SC8O30, SC8H83, SC8O20)
├── docs/                  # Clean-room hardware specifications, pinouts & RE reports
├── scripts/               # ELF builders, Unicorn oracle build script, profiling & E2E scenario runner
└── vault/                 # [gitignored] Local directory for user-supplied firmwares (.frf / .bin)
```

---

## Supplying Firmware Images (`vault/firmwares/`)

SimEmu **does not distribute** third-party OEM application firmware or proprietary manufacturer PDFs (`vault/` is gitignored).
Without firmware files, the entire workspace builds cleanly on pure Rust and all unit tests pass out of the box
(`cargo test --release --lib`); firmware-dependent integration tests print `SKIP`.

Place your own VAG `.frf` containers or `.bin` dumps under `vault/firmwares/` (release archives already include this folder
skeleton; you can also point `$SIMEMU_IMAGE_DIR` to any external directory):

```text
rom/
├── sboot.bin              # Bundled default SIMOS18.1 BootROM (112 KB, used when a dump has no SBOOT)
└── dflash.bin             # Bundled scrubbed SIMOS18.1 DFlash / FEE image (256 KB, used when no user DFlash is supplied)
vault/
└── firmwares/
    ├── ecu/
    │   ├── frf/           # Official VAG ODIS flash containers (.frf) — unpacked natively in RAM
    │   ├── stock/         # Optional: pre-unpacked 4 MB factory SIMOS18.1 images (.bin)
    │   ├── bench/         # Optional: user's own SIMOS18.1 bench DFlash 256 KB / PFlash 4 MB / sboot.bin
    │   └── custom/        # Tuned / custom SIMOS18.1 images (.bin or .frf)
    └── tcu/
        ├── dq200/         # Full bench read of DSG7 DQ200_G2 mechatronic (TC1784 micro + 8 KB EEPROM)
        └── dq250/         # Full Flash read of DSG6 DQ250 MQB mechatronic (1.5 MB TC1766 .bin)
```

### Required & Optional Files

1. **SIMOS18.1 ECU Flash (`vault/firmwares/ecu/frf/` or `custom/`)**:
   - **Native `.frf` / `.odx` support:** SimEmu decrypts and decompresses VAG ODIS `.frf` and `.odx` containers directly in pure Rust (`simemu_damos::frf`) — no external tools or Python scripts needed.
   - Drop an `.frf` container from VAG Flashdaten into `vault/firmwares/ecu/frf/`:
     - **1.8 TSI EA888 Gen3 (`CJSA` / `CJSB`, Octavia A7):** `FL_8V0906264H__0005.frf` (default in CLI and Studio) or `FL_8V0906264J__0003.frf` (`SC8O40`);
     - **2.0 TSI EA888 Gen3:** `FL_8V0906264K__0003.frf` (`SC8S50`) or `FL_8V0906264F__0003.frf` (`SC8H83`).
   - **Supplier BootROM (`SBOOT`, `0x80000000..0x8001BFFF`) is already bundled (`rom/sboot.bin`, 112 KB):** `.frf` containers and OBD reads never carry `SBOOT` or its shared crypto table (`0x80014088`). SimEmu applies strict priority at load time:
     1. **User's own `SBOOT` inside the loaded `.bin`:** if a loaded `4 MB` or `3 MB` BSL `.bin` dump already contains a non-empty `SBOOT`, **its own `SBOOT` is always preserved 100% untouched**;
     2. **User's bench donor:** for `.frf`/`.odx` and OBD-only reads (where `SBOOT` is `0x00`/`0xFF`), SimEmu first checks for a user-supplied `sboot.bin` or bench dump in `vault/firmwares/ecu/bench/` (and `$SIMEMU_IMAGE_DIR`);
     3. **Bundled `rom/sboot.bin`:** otherwise, SimEmu supplies `rom/sboot.bin` in memory automatically (you can also replace `rom/sboot.bin` on disk without recompiling).
   - To export an unpacked 4 MB `.bin` to disk, run `simemu unpack-frf <input.frf> [output.bin]`.

2. **SIMOS18.1 ECU `DFlash / EEPROM` (`rom/dflash.bin` already bundled, `256 KB`)**:
   - **No separate DFlash dump is required!** SimEmu ships a clean, 100% depersonalized SIMOS18.1 FEE DFlash image (`rom/dflash.bin`, `256 KB`, bank `DF0` = first `96 KB`) pre-populated with deterministic laboratory WFS5 records (`LABEMU…`). Dropping **just a single `.frf` file** into `vault/firmwares/ecu/frf/` is enough to start the engine and run full closed-loop Studio co-simulation out of the box.
   - If you place your own `256 KB` (or `96 KB` `DF0`) bench DFlash dump in `vault/firmwares/ecu/bench/` (or `$SIMEMU_IMAGE_DIR`), your dump automatically overrides `rom/dflash.bin`. See [`docs/FIRMWARE_AND_BOOT_GUIDE.md`](docs/FIRMWARE_AND_BOOT_GUIDE.md) for the full memory map and boot walkthrough.

3. **DSG7 DQ200 Gen2 Mechatronic (`0CW`, `vault/firmwares/tcu/dq200/`)**:
   - Full bench read of a `J743` (`GSG DQ200G2`, `SAK-TC1784`, e.g. `0CW300043H_0002`, PCMflash Module 58):
     - **Micro (PFlash)** — `2.625 MB` (`0x2A0000` bytes);
     - **EEPROM (ST95640)** — `8 KB` (`8192` bytes).

4. **DSG6 DQ250 MQB Mechatronic (`0D9`, `vault/firmwares/tcu/dq250/`)**:
   - Only a **single `1.5 MB` Flash read** (`0x180000` or `0x178000` bytes, `SAK-TC1766`, e.g. `0D9300011R` SW 4319 or `0D9300012F` SW 4930) is required.
   - **No DQ250 EEPROM dump is needed:** on first launch, Studio initializes an erased 8 KB EEPROM, binds lab WFS5 credentials, and runs a background workshop session where the unmodified TCU firmware itself performs UDS coding and fork/clutch basic settings (`0x0380` / `0x04D4`), persisting the adapted EEPROM in `build/studio_state/`.

---

## Building from Source

- **Rust 1.88+** (Cargo workspace, 2021 edition).
- **Zero external C/C++ dependencies:** by default, `simemu` compiles as a **100% pure-Rust** standalone executable powered by `tricore-core` (`CpuBackend::Native`) on Windows, Linux, and macOS.

```sh
cargo build --release                          # full build (studio + bridge features)
cargo build --release --no-default-features    # minimal headless CLI
```

### Optional (Developers Only): Unicorn 2.1.4 Differential Oracle

The patched C `libunicorn` 2.1.4 backend is gated behind the optional `--features unicorn` Cargo feature and is used strictly during development to verify instruction-for-instruction determinism of `tricore-core`:

```sh
python scripts/build_unicorn.py       # builds patched Unicorn 2.1.4 -> target/unicorn-simemu/build/
cargo test --release --workspace --all-features
```

---

## CLI Usage & Language Selection (`--lang en|ru`)

By default, all console output and the Studio web UI start in **English (`en`)**. Pass **`--lang ru`** (or shorthand **`--ru`**, `-L ru`, or set `SIMEMU_LANG=ru`) anywhere on the command line to switch console output and Studio's default UI language to **Russian**:

```sh
simemu                                          # print CLI help in English (default)
simemu --lang ru                                # print CLI help in Russian (or: simemu --ru)

simemu studio [port] [--slcan <port|off> | --no-slcan] # web studio http://127.0.0.1:8080 + SLCAN :28750
simemu --ru studio                              # web studio + console logs in Russian

simemu can-server [port] [image.bin|frf] [eeprom]   # headless Lawicel/SLCAN TCP bench server (:28750)
simemu boot [insns] [image.bin|image.frf]           # cold-boot SIMOS18.1 ECU (.bin or .frf directly)
simemu unpack-frf <input.frf|odx> [output.bin]      # unpack VAG ODIS .frf/.odx natively to 4 MB .bin
simemu damos [query] [--image <bin|frf>]            # search DAMOS parameters and read physical values
simemu tcu-info                                     # inspect DQ200_G2 TCU image and run boot trace
simemu can-table                                    # dump SIMOS18.1 CAN TX/RX descriptor tables
simemu render-audio [start|rev|misfire|knock|all|rtf] [out_dir]
simemu log-report <log.csv>                         # SimosTools CSV log forensics -> build/log_report/
simemu cal-diff [A.bin|frf A.json B.bin|frf B.json] # DAMOS calibration diff -> build/caldiff/
simemu replay <log.csv> [--image …] [--profile …]   # closed-loop log replay -> build/replay/
```

Studio binds exclusively to loopback `127.0.0.1`:
- `http://127.0.0.1:8080` — virtual engine/chassis dyno workstation, live DAMOS calibration editor, oscilloscope, pre-flash tune safety verifier, and UDS diagnostic console (`EN / RU` switcher is also available in the top bar);
- `socket://127.0.0.1:28750` — live **Lawicel / SLCAN over TCP** virtual CAN adapter compatible with external diagnostic tools, loggers, and flashers (`python-can`, `udsoncan`, `SavvyCAN`, `SimosTools`, `VW_Flash`).

---

## Running Tests

Always run emulation tests with `--release`:

```sh
# Unit tests across all crates (runs cleanly even without vault/ firmwares)
cargo test --release --lib

# Full workspace test suite (automatically discovers firmwares in vault/firmwares/)
cargo test --release

# Full co-simulation real-time guard (ECU + TCU + physics, target RTF > 1.0x)
cargo test --release -p simemu-cli --lib cosim_realtime_guard -- --ignored --nocapture

# Hardware boot & power-stage regressions across all images in vault/firmwares/
cargo test --release -p simemu-tc1791 --test boot_regression --test power_stages_e2e --test perf_determinism -- --nocapture
```

---

## License

SimEmu © 2026 Matrix is licensed under the **PolyForm Noncommercial License 1.0.0** with **Additional Terms** (the full text in [`LICENSE`](LICENSE) governs):

- Open source — free to read, study, and modify;
- Free for personal, academic, research, and noncommercial use, **as well as paid vehicle repair and diagnostics** (workshops and technicians may use SimEmu when diagnosing, repairing, testing, coding, or adapting customer vehicles);
- Selling, licensing, renting, or charging for access to SimEmu (standalone or bundled) is prohibited;
- Modified versions and derivative works must be distributed with complete source code under the exact same license terms and keep all `Required Notice:` lines intact.
