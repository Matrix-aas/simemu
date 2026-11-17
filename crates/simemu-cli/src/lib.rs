pub mod can_server;
#[path = "studio/immo_keys.rs"]
pub mod immo_keys;
#[path = "studio/immo_lab.rs"]
pub mod immo_lab;
pub mod log_tools;
pub mod paths;
pub mod slcan;
#[cfg(feature = "studio")]
pub mod studio;

#[cfg(feature = "bridge")]
use simemu_bridge::Simos18CanDescriptor;
use simemu_damos::{is_ru, set_lang, tr, DamosDatabase, Lang};
use simemu_tc1791::{Dq200TcuBoard, Dq200TcuImageInfo, Simos18Board};
use std::env;
use std::fs;
use std::path::{Path, PathBuf};
use std::time::Instant;
use unicorn_tricore_sys::TriCoreReg;

const DEFAULT_DAMOS_REL: &str = "definitions/SC8O40.json";

pub fn cmd_boot(root: &Path, max_insns: usize, image: Option<&str>) -> Result<(), String> {
    let ru = is_ru();
    let flash_path = paths::resolve_firmware_or_err(root, image.unwrap_or(paths::DEFAULT_ECU_FLASH))?;
    let (eeprom_path, eeprom) = paths::read_dflash_or_default(root)?;
    let eeprom_desc = format!("{} (256 KB)", eeprom_path.display());

    println!("==========================================================================");
    println!(
        "{}",
        tr(
            "  SIMEMU — Native Continental SIMOS18.1 (Infineon TC1791, fCPU 180 MHz)",
            "  SIMEMU — Нативный ЭБУ Continental SIMOS18.1 (Infineon TC1791, fCPU 180 МГц)"
        )
    );
    println!("==========================================================================");
    println!("  Flash : {} (4 MB)", flash_path.display());
    println!("  EEPROM: {}", eeprom_desc);

    let mut flash = paths::read_firmware_from_path(root, &flash_path)
        .map_err(|e| format!("{}: {e}", tr("Failed to read Flash", "Не удалось прочитать Flash")))?;
    // Reads without the supplier bootloader get SBOOT from the factory dump of the same hardware (or rom/sboot.bin).
    if flash.len() == 0x40_0000 && flash[..0x1_C000].iter().all(|&b| b == 0 || b == 0xFF) {
        let (_, base) = paths::read_sboot_donor(root)?;
        flash[..0x1_C000].copy_from_slice(&base[..0x1_C000]);
    }
    let _ = simemu_damos::apply_post_programming(&mut flash)?;

    let t_init = Instant::now();
    let mut board = Simos18Board::new(&flash, &eeprom)?;
    if ru {
        println!(
            "  Плата инициализирована (ядро {:?}) за {:.2?}",
            board.backend(),
            t_init.elapsed()
        );
    } else {
        println!(
            "  Board initialized ({:?} CPU) with zero-copy host memory mappings in {:.2?}",
            board.backend(),
            t_init.elapsed()
        );
    }

    let t_boot = Instant::now();
    let final_pc = board.run_boot_sequence(max_insns)?;
    let dt = t_boot.elapsed();

    let bist1 = board.hw().read_lmu_u32(0xB000_014C);
    let bist2 = board.hw().read_lmu_u32(0xB000_0150);
    let fcx = board.read_reg(TriCoreReg::FCX);
    let lcx = board.read_reg(TriCoreReg::LCX);
    let biv = board.read_reg(TriCoreReg::BIV);
    let btv = board.read_reg(TriCoreReg::BTV);
    let sp = board.read_reg(TriCoreReg::A10);

    println!(
        "\n{}",
        tr(
            "[Boot & Hardware Self-Test Summary]",
            "[Сводка загрузки и аппаратной самопроверки]"
        )
    );
    println!("  {:17}: {:.3?}", tr("Execution Time", "Время выполнения"), dt);
    println!(
        "  {:17}: 0x{:08X} ({})",
        tr("Current PC", "Текущий PC"),
        final_pc,
        Simos18Board::classify_region(final_pc)
    );
    println!(
        "  {:17}: {}",
        tr("Warm Resets", "Тёплых сбросов"),
        board.hw().sw_reset_count
    );
    println!(
        "  BIST Signatures  : 0x{:08X} / 0x{:08X} (@ 0xB000014C..0xB0000150)",
        bist1, bist2
    );
    println!(
        "  CPU CSFR State   : SP=0x{:08X} | FCX=0x{:08X} | LCX=0x{:08X} | BIV=0x{:08X} | BTV=0x{:08X}",
        sp, fcx, lcx, biv, btv
    );
    println!(
        "  CPU Global Regs  : A0=0x{:08X} | A1=0x{:08X} | A8=0x{:08X} | A9=0x{:08X} | ICR=0x{:08X}",
        board.read_reg(TriCoreReg::A0),
        board.read_reg(TriCoreReg::A1),
        board.read_reg(TriCoreReg::A8),
        board.read_reg(TriCoreReg::A9),
        board.read_reg(TriCoreReg::ICR),
    );
    println!(
        "  STM 64-bit Timer : {} ticks (TIM0=0x{:08X}, TIM1=0x{:08X})",
        board.hw().stm_ticks(),
        board.hw().stm_ticks() as u32,
        (board.hw().stm_ticks() >> 4) as u32
    );
    println!(
        "  MMIO Throughput  : {} reads, {} writes (native zero-GIL C-ABI hooks)",
        board.hw().mmio_read_count, board.hw().mmio_write_count
    );
    println!(
        "  Hardware IRQs    : {} dispatched via ICU (STM_CMP0=0x{:08X}, STM_SRC0=0x{:04X}, CPU_SRC0=0x{:04X})",
        board.hw().irq_dispatch_count,
        board.hw().stm.cmp[0],
        board.hw().read_f000_u32(0xF000_02FC),
        board.hw().read_f7e0_u32(0xF7E0_FFF8)
    );
    println!(
        "  RTOS Scheduler   : ReadyMask=0x{:08X}, CurMask=0x{:08X}, SchedFlags=0x{:08X}, TaskMask=0x{:08X}",
        board.hw().read_dspr_u32(0xD000_A204),
        board.hw().read_dspr_u32(0xD000_A208),
        board.hw().read_dspr_u32(0xD000_B000),
        board.hw().read_dspr_u32(0xD000_AFFC),
    );
    println!("  RTA-OS ISRs      : {} entries", board.hw().rtos_isr_counts.len());
    for (&fn_addr, &cnt) in &board.hw().rtos_isr_counts {
        println!("    ISR  @ 0x{:08X} -> {} calls", fn_addr, cnt);
    }
    println!("  RTA-OS Tasks     : {} entries", board.hw().rtos_task_counts.len());
    for (&fn_addr, &cnt) in &board.hw().rtos_task_counts {
        println!("    Task @ 0x{:08X} -> {} runs", fn_addr, cnt);
    }
    println!(
        "  MultiCAN+ TX     : {} frames transmitted by ECU",
        board.hw().can_tx_queue.len()
    );

    println!(
        "\n[Hardware Boot Milestones Reached ({})]",
        board.hw().milestones.len()
    );
    for m in &board.hw().milestones {
        println!(
            "  [Reset #{}] {:20} @ 0x{:08X} (STM={} ticks)",
            m.reset_cycle, m.name, m.pc, m.stm_ticks
        );
    }

    Ok(())
}

pub fn cmd_tcu_info(root: &Path) -> Result<(), String> {
    let micro_path = paths::resolve_firmware_or_err(root, paths::DEFAULT_DQ200_FLASH)?;
    let eeprom_path = paths::resolve_firmware_or_err(root, paths::DEFAULT_DQ200_EEPROM)?;
    let info = Dq200TcuImageInfo::inspect_files(&micro_path, Some(&eeprom_path))?;

    println!("==========================================================================");
    println!("  SIMEMU — Core DSG7 DQ200 Gen2 (0CW) TCU Firmware & Hardware Inspector");
    println!("==========================================================================");
    println!("  MCU Target      : {}", info.mcu_model);
    println!(
        "  PFlash / EEPROM : {} bytes (2.75 MB) / {} bytes (8 KB)",
        info.flash_size_bytes, info.eeprom_size_bytes
    );
    println!("  Cal Header      : {}", info.cal_header);
    println!("  Calibration ID  : {}", info.sw_version_tag);
    println!("  ASW Build ID    : {}", info.asw_build_tag);
    println!("  CBoot Version   : {}", info.cboot_version);
    println!("  SBOOT Entry     : 0x{:08X} -> j 0x80010000 (TEMIC ACCESS OK)", info.sboot_entry_va);
    println!("  CBoot Entry     : 0x{:08X} -> j 0x80015B40 ({})", info.cboot_entry_va, info.cboot_version);
    println!("  ASW Entry       : 0x{:08X} -> j 0x80092BF0 ({})", info.asw_entry_va, info.asw_build_tag);
    println!("\n[Memory Map]");
    println!("  0x80000000..0x8000D000 : Primary SBOOT (TEMIC Seed/Key + Boot Check)");
    println!("  0x80010000..0x8001FFFF : Secondary CBoot (UDS Flash Loader V0006)");
    println!("  0x80020000..0x8003FFFF : Calibration Sector (Shift Maps, K1/K2 Kiss Points, Torque Cap)");
    println!("  0x80040000..0x8013FFFF : Application ASW (Mechatronic Pressure/Clutch & Shift State Machine)");

    println!("\n[Native Unicorn TriCore TC1784 Boot Trace (Zero-Patch)]");
    let flash_bytes = fs::read(&micro_path).map_err(|e| e.to_string())?;
    let eeprom_bytes = fs::read(&eeprom_path).map_err(|e| e.to_string())?;
    let mut tcu = Dq200TcuBoard::new(&flash_bytes, &eeprom_bytes)?;

    for step in 1..=20 {
        match tcu.run_boot_trace(10, 25_000) {
            Ok(pc) => {
                println!(
                    "  Step {:2} (250k insns) -> PC=0x{:08X} ({:12}) | Resets={} | STM={} | MMIO R/W={}/{} | CAN TX={}",
                    step,
                    pc,
                    Dq200TcuBoard::classify_region(pc),
                    tcu.cpu.hw().sw_reset_count,
                    tcu.cpu.hw().stm_ticks(),
                    tcu.cpu.hw().mmio_read_count,
                    tcu.cpu.hw().mmio_write_count,
                    tcu.cpu.hw().can_tx_queue.len()
                );
            }
            Err(e) => {
                println!("  [HALT] {}", e);
                break;
            }
        }
    }
    if !tcu.cpu.hw().can_tx_queue.is_empty() {
        println!("\n[DQ200_G2 MultiCAN+ Transmitted MQB Frames ({} total)]", tcu.cpu.hw().can_tx_queue.len());
        for (i, f) in tcu.cpu.hw().can_tx_queue.iter().enumerate().take(20) {
            let hex: Vec<String> = f.data[..(f.dlc as usize).min(8)]
                .iter()
                .map(|b| format!("{:02X}", b))
                .collect();
            println!(
                "  #{:02} | MO={:<3} | ID=0x{:03X} | DLC={} | Data=[{}]",
                i + 1,
                f.mo_index,
                f.arb_id,
                f.dlc,
                hex.join(" ")
            );
        }
    }
    Ok(())
}

#[cfg(feature = "bridge")]
pub fn cmd_can_table(root: &Path) -> Result<(), String> {
    let flash = paths::read_firmware(root, paths::DEFAULT_ECU_FLASH)?;
    let descriptors = Simos18CanDescriptor::parse_all_from_flash(&flash);
    println!(
        "SIMOS18.1 CAN descriptor tables: TX @ 0x8005BBE4 (48-byte entries), RX @ 0x8005C038 (44-byte entries), {} valid entries:",
        descriptors.len()
    );
    println!(
        "{:<4} | {:<3} | {:<10} | {:<3} | {:<7} | {:<10} | {:<10} | {:<7}",
        "Idx", "Bus", "CAN ID", "DLC", "Timeout", "Handler Fn", "Sig Table", "Signals"
    );
    println!("{}", "-".repeat(78));
    for d in &descriptors {
        println!(
            "{:<4} | {:<3} | 0x{:08X} | {:<3} | {:>7} | 0x{:08X} | 0x{:08X} | {:>7}",
            d.index,
            d.bus_id,
            d.can_id,
            d.dlc,
            d.timeout_ticks,
            d.handler_fn,
            d.signal_table_ptr,
            d.signal_count
        );
    }
    Ok(())
}

pub fn cmd_damos(root: &Path, args: &[String]) -> Result<(), String> {
    let mut query = "put";
    let mut image_opt: Option<&str> = None;
    let mut damos_opt: Option<&str> = None;
    let mut i = 0;
    while i < args.len() {
        match args[i].as_str() {
            "--image" => {
                image_opt = Some(args.get(i + 1).ok_or("--image requires a path")?.as_str());
                i += 2;
            }
            "--damos" => {
                damos_opt = Some(args.get(i + 1).ok_or("--damos requires a path")?.as_str());
                i += 2;
            }
            q if !q.starts_with('-') => {
                query = q;
                i += 1;
            }
            other => {
                return Err(format!(
                    "unknown argument {other:?} (usage: simemu damos [query] [--image <bin|frf>] [--damos <json>])"
                ))
            }
        }
    }
    let flash = paths::read_firmware(root, image_opt.unwrap_or(paths::DEFAULT_ECU_FLASH))?;
    let damos_path = match damos_opt {
        Some(p) => {
            let pb = PathBuf::from(p);
            if pb.is_file() { pb } else { root.join(p) }
        }
        None => {
            let box_code = flash
                .get(0x20_0002..0x20_0008)
                .map(|b| String::from_utf8_lossy(b).trim().to_string())
                .unwrap_or_default();
            let auto = root.join(format!("definitions/{box_code}.json"));
            if auto.is_file() { auto } else { root.join(DEFAULT_DAMOS_REL) }
        }
    };
    let t0 = Instant::now();
    let db = DamosDatabase::load_from_json(&damos_path)?;
    println!(
        "Loaded {} DAMOS parameters from {} in {:.2?}",
        db.parameters.len(),
        damos_path.display(),
        t0.elapsed()
    );
    let hits = db.search(query, 25);
    println!("Showing top {} matches for '{}':", hits.len(), query);
    for p in hits {
        let val_str = db
            .read_physical_value(p, &flash)
            .map(|v| format!("{:.4} {}", v, p.unit))
            .unwrap_or_else(|| "N/A".to_string());
        println!(
            "  {:32} | @ 0x{:08X} ({:<5}) | {:16} | {}",
            p.id, p.cpu_address, p.kind, val_str, p.title
        );
    }
    Ok(())
}

pub fn cmd_unpack_frf(root: &Path, args: &[String]) -> Result<(), String> {
    if args.is_empty() || args.first().is_some_and(|a| matches!(a.as_str(), "-h" | "--help")) {
        println!("Usage: simemu unpack-frf <input.frf|input.odx> [output.bin]");
        return if args.is_empty() {
            Err("missing <input.frf> argument".into())
        } else {
            Ok(())
        };
    }
    if args.len() > 2 {
        return Err(format!(
            "unexpected argument {:?} (usage: simemu unpack-frf <input.frf|input.odx> [output.bin])",
            args[2]
        ));
    }
    let in_path = paths::resolve_firmware_or_err(root, &args[0])?;
    let out_path = match args.get(1) {
        Some(p) => PathBuf::from(p),
        None => {
            let stem = in_path
                .file_stem()
                .and_then(|s| s.to_str())
                .unwrap_or("unpacked");
            let clean = stem.strip_prefix("FL_").or_else(|| stem.strip_prefix("fl_")).unwrap_or(stem);
            PathBuf::from(format!("{clean}.bin"))
        }
    };
    let donor = paths::read_sboot_donor(root).ok();
    let raw = fs::read(&in_path).map_err(|e| format!("{}: {e}", in_path.display()))?;
    let t0 = Instant::now();
    let (mut img, report) = if simemu_damos::is_odx_path(&in_path) && !simemu_damos::is_frf_bytes(&raw) {
        let xml = String::from_utf8(raw)
            .unwrap_or_else(|e| String::from_utf8_lossy(&e.into_bytes()).into_owned());
        let name = in_path
            .file_name()
            .map(|s| s.to_string_lossy().into_owned())
            .unwrap_or_default();
        simemu_damos::unpack_odx_into_image(&xml, &name, donor.as_ref().map(|(_, b)| b.as_slice()))?
    } else {
        simemu_damos::unpack_frf_into_image(&raw, donor.as_ref().map(|(_, b)| b.as_slice()))?
    };
    let _ = simemu_damos::apply_post_programming(&mut img)?;
    if let Some(parent) = out_path.parent().filter(|p| !p.as_os_str().is_empty()) {
        fs::create_dir_all(parent).map_err(|e| format!("{}: {e}", parent.display()))?;
    }
    fs::write(&out_path, &img).map_err(|e| format!("{}: {e}", out_path.display()))?;
    println!(
        "Unpacked {} -> {} ({} bytes) in {:.2?}",
        in_path.display(),
        out_path.display(),
        img.len(),
        t0.elapsed()
    );
    println!(
        "  ODX: {} | Box: {} | Part: {} | Blocks: {}",
        report.odx_name,
        report.box_code,
        report.part_number,
        report.blocks_written.join(", ")
    );
    match &donor {
        Some((dp, _)) => println!("  SBOOT donor: {}", dp.display()),
        None => println!("  SBOOT donor: none (0x00000..0x1BFFF left 0xFF; calibration-only until bench SBOOT is supplied)"),
    }
    Ok(())
}

const DEFAULT_BOOT_INSNS: usize = 5_000_000;

fn looks_like_image_arg(arg: &str) -> bool {
    Path::new(arg)
        .extension()
        .and_then(|e| e.to_str())
        .is_some_and(paths::is_ecu_image_ext)
        || arg.contains('/')
        || arg.contains('\\')
}

/// Parses `simemu boot [max_insns] [image.bin|image.frf]`, allowing `simemu boot FL_8V0906264H__0005.frf` directly
/// while rejecting malformed instruction counts or unexpected extra arguments.
pub fn parse_boot_args(args: &[String]) -> Result<(usize, Option<&str>), String> {
    match args.first().map(|s| s.as_str()) {
        None => Ok((DEFAULT_BOOT_INSNS, None)),
        Some(first) => match first.parse::<usize>() {
            Ok(max_insns) => {
                if args.len() > 2 {
                    return Err(format!(
                        "unexpected argument {:?} (usage: simemu boot [insns] [image.bin|image.frf])",
                        args[2]
                    ));
                }
                Ok((max_insns, args.get(1).map(|s| s.as_str())))
            }
            Err(_) if looks_like_image_arg(first) => {
                if args.len() > 1 {
                    return Err(format!(
                        "unexpected argument {:?} after image {:?} (usage: simemu boot [insns] [image.bin|image.frf])",
                        args[1], first
                    ));
                }
                Ok((DEFAULT_BOOT_INSNS, Some(first)))
            }
            Err(_) => Err(format!(
                "invalid instruction count {first:?} (usage: simemu boot [insns] [image.bin|image.frf])"
            )),
        },
    }
}

pub fn parse_can_server_args(args: &[String]) -> Result<(u16, Option<&str>, Option<&str>), String> {
    match args.first().map(|s| s.as_str()) {
        None => Ok((slcan::DEFAULT_SLCAN_PORT, None, None)),
        Some(first) => match first.parse::<u16>() {
            Ok(port) => {
                if args.len() > 3 {
                    return Err(format!(
                        "unexpected argument {:?} (usage: simemu can-server [port] [image.bin|image.frf] [eeprom.bin])",
                        args[3]
                    ));
                }
                Ok((
                    port,
                    args.get(1).map(|s| s.as_str()),
                    args.get(2).map(|s| s.as_str()),
                ))
            }
            Err(_) if looks_like_image_arg(first) => {
                if args.len() > 2 {
                    return Err(format!(
                        "unexpected argument {:?} (usage: simemu can-server [port] [image.bin|image.frf] [eeprom.bin])",
                        args[2]
                    ));
                }
                Ok((
                    slcan::DEFAULT_SLCAN_PORT,
                    Some(first),
                    args.get(1).map(|s| s.as_str()),
                ))
            }
            Err(_) => Err(format!(
                "invalid can-server port {first:?} (usage: simemu can-server [port] [image.bin|image.frf] [eeprom.bin])"
            )),
        },
    }
}

pub fn cli_usage() -> &'static str {
    if is_ru() {
        "Использование: simemu [--lang <en|ru>] <команда> [опции]\n\
         \n\
         Глобальные флаги:\n\
         \x20 --lang <en|ru>, -L <en|ru>, --en, --ru            Язык консоли и веб-студии (по умолчанию: en)\n\
         \x20 -h, --help                                        Показать эту справку\n\
         \x20 -V, --version                                     Показать версию\n\
         \n\
         Команды:\n\
         \x20 studio [port] [--slcan <port|off> | --no-slcan]   Веб-стенд ко-симуляции (по умолчанию: http://127.0.0.1:8080)\n\
         \x20 boot [insns] [image.bin|image.frf]                Холодная загрузка ЭБУ SIMOS18.1 (по умолчанию: 5_000_000 инстр.)\n\
         \x20 unpack-frf <input.frf|input.odx> [output.bin]     Нативная распаковка VAG ODIS .frf/.odx в 4 МБ .bin\n\
         \x20 can-server [port] [image.bin|frf] [eeprom.bin]    Автономный TCP-сервер Lawicel/SLCAN (по умолчанию: 127.0.0.1:28750)\n\
         \x20 damos [query] [--image <bin|frf>] [--damos <json>] Поиск параметров калибровки в DAMOS\n\
         \x20 tcu-info                                          Паспорт образа ТКУ DQ200_G2 и трасса загрузки\n\
         \x20 can-table                                         Дамп таблиц CAN TX/RX из прошивки SIMOS18.1\n\
         \x20 render-audio [start|rev|misfire|knock|all|rtf] [out_dir]\n\
         \x20 log-report <log.csv> [--image <bin|frf>] [--damos <json>]\n\
         \x20 cal-diff [A.bin|frf A.json B.bin|frf B.json]\n\
         \x20 replay <log.csv> [--image <bin|frf>] [--profile <slug>] [--from s] [--to s]"
    } else {
        "Usage: simemu [--lang <en|ru>] <command> [options]\n\
         \n\
         Global options:\n\
         \x20 --lang <en|ru>, -L <en|ru>, --en, --ru            Console and Studio UI language (default: en)\n\
         \x20 -h, --help                                        Print this help message\n\
         \x20 -V, --version                                     Print version\n\
         \n\
         Commands:\n\
         \x20 studio [port] [--slcan <port|off> | --no-slcan]   Co-simulation web studio (default: http://127.0.0.1:8080)\n\
         \x20 boot [insns] [image.bin|image.frf]                Boot SIMOS18.1 ECU (default: 5_000_000 insns)\n\
         \x20 unpack-frf <input.frf|input.odx> [output.bin]     Unpack VAG ODIS .frf/.odx natively to 4 MB .bin\n\
         \x20 can-server [port] [image.bin|frf] [eeprom.bin]    Headless Lawicel/SLCAN TCP bench server (default: 127.0.0.1:28750)\n\
         \x20 damos [query] [--image <bin|frf>] [--damos <json>] Query DAMOS calibration parameters\n\
         \x20 tcu-info                                          Inspect DQ200_G2 TCU image and run short boot trace\n\
         \x20 can-table                                         Dump SIMOS18.1 CAN TX/RX descriptor tables (feature: bridge)\n\
         \x20 render-audio [start|rev|misfire|knock|all|rtf] [out_dir]\n\
         \x20 log-report <log.csv> [--image <bin|frf>] [--damos <json>]\n\
         \x20 cal-diff [A.bin|frf A.json B.bin|frf B.json]\n\
         \x20 replay <log.csv> [--image <bin|frf>] [--profile <slug>] [--from s] [--to s]"
    }
}

/// Strips global language flags (`--lang <en|ru>`, `--lang=<en|ru>`, `-L <en|ru>`, `--en`, `--ru`)
/// from `raw_args[1..]` and returns `(cli_lang, remaining_args_without_argv0)`.
pub fn extract_global_lang_args(raw_args: &[String]) -> Result<(Option<Lang>, Vec<String>), String> {
    let tail = raw_args.get(1..).unwrap_or(&[]);
    let mut chosen: Option<Lang> = None;
    let mut rest = Vec::with_capacity(tail.len());
    let mut i = 0;
    let mut passthrough = false;
    while i < tail.len() {
        let a = tail[i].as_str();
        if passthrough {
            rest.push(tail[i].clone());
            i += 1;
            continue;
        }
        if a == "--" {
            passthrough = true;
            rest.push(tail[i].clone());
            i += 1;
            continue;
        }
        if a == "--lang" || a == "-L" {
            let val = tail
                .get(i + 1)
                .ok_or_else(|| format!("{a} requires a value ('en' or 'ru')"))?;
            chosen = Some(Lang::parse(val)?);
            i += 2;
        } else if let Some(val) = a.strip_prefix("--lang=").or_else(|| a.strip_prefix("-L=")) {
            chosen = Some(Lang::parse(val)?);
            i += 1;
        } else if a == "--ru" {
            chosen = Some(Lang::Ru);
            i += 1;
        } else if a == "--en" {
            chosen = Some(Lang::En);
            i += 1;
        } else {
            rest.push(tail[i].clone());
            i += 1;
        }
    }
    Ok((chosen, rest))
}

pub fn run_cli() -> Result<(), String> {
    // The native TriCore core is the default CPU; SIMEMU_CPU=unicorn selects Unicorn (read by every
    // board constructor). Set before any thread exists.
    if env::var_os("SIMEMU_CPU").is_none() {
        env::set_var("SIMEMU_CPU", "native");
    }
    let root = paths::find_workspace_root();
    let raw_args: Vec<String> = env::args().collect();
    let env_lang = env::var("SIMEMU_LANG").ok();
    run_cli_with_args(&root, &raw_args, env_lang.as_deref())
}

pub fn run_cli_with_args(
    root: &Path,
    raw_args: &[String],
    env_lang: Option<&str>,
) -> Result<(), String> {
    let (cli_lang, rest) = extract_global_lang_args(raw_args)?;
    let effective_lang = match cli_lang {
        Some(l) => l,
        None => match env_lang.map(str::trim).filter(|s| !s.is_empty()) {
            Some(v) => Lang::parse(v)?,
            None => Lang::En,
        },
    };
    set_lang(effective_lang);

    let Some(sub) = rest.first().map(|s| s.as_str()) else {
        println!("{}", cli_usage());
        return Ok(());
    };
    let sub_args = &rest[1..];

    match sub {
        "-h" | "--help" | "help" => {
            println!("{}", cli_usage());
            Ok(())
        }
        "-V" | "--version" | "version" => {
            println!("simemu {}", env!("CARGO_PKG_VERSION"));
            Ok(())
        }
        "boot" => {
            if sub_args.first().is_some_and(|a| matches!(a.as_str(), "-h" | "--help")) {
                println!(
                    "{}",
                    tr(
                        "Usage: simemu [--lang <en|ru>] boot [insns] [image.bin|image.frf]",
                        "Использование: simemu [--lang <en|ru>] boot [insns] [image.bin|image.frf]"
                    )
                );
                return Ok(());
            }
            let (max_insns, image) = parse_boot_args(sub_args)?;
            cmd_boot(root, max_insns, image)
        }
        "unpack-frf" => cmd_unpack_frf(root, sub_args),
        "tcu-info" => cmd_tcu_info(root),
        "can-server" => {
            if sub_args.first().is_some_and(|a| matches!(a.as_str(), "-h" | "--help")) {
                println!(
                    "{}",
                    tr(
                        "Usage: simemu [--lang <en|ru>] can-server [port] [image.bin|image.frf] [eeprom.bin]",
                        "Использование: simemu [--lang <en|ru>] can-server [port] [image.bin|image.frf] [eeprom.bin]"
                    )
                );
                return Ok(());
            }
            let (port, image, eeprom) = parse_can_server_args(sub_args)?;
            can_server::cmd_can_server(root, port, image, eeprom)
        }
        #[cfg(feature = "bridge")]
        "can-table" => cmd_can_table(root),
        "damos" => cmd_damos(root, sub_args),
        #[cfg(feature = "studio")]
        "studio" => {
            if sub_args.first().is_some_and(|a| matches!(a.as_str(), "-h" | "--help")) {
                println!(
                    "{}",
                    tr(
                        "Usage: simemu [--lang <en|ru>] studio [port] [--slcan <port|off> | --no-slcan]",
                        "Использование: simemu [--lang <en|ru>] studio [port] [--slcan <port|off> | --no-slcan]"
                    )
                );
                return Ok(());
            }
            studio::run_studio_server_with_args(root, sub_args)
        }
        #[cfg(feature = "studio")]
        "render-audio" => {
            if sub_args.first().is_some_and(|a| matches!(a.as_str(), "-h" | "--help")) {
                println!(
                    "{}",
                    tr(
                        "Usage: simemu [--lang <en|ru>] render-audio [start|rev|misfire|knock|all|rtf] [out_dir]",
                        "Использование: simemu [--lang <en|ru>] render-audio [start|rev|misfire|knock|all|rtf] [out_dir]"
                    )
                );
                return Ok(());
            }
            let which = sub_args.first().map(|s| s.as_str()).unwrap_or("all");
            let out = sub_args.get(1).map(PathBuf::from).unwrap_or_else(|| root.join("build/audio"));
            studio::audio::cmd_render_audio(root, which, &out)
        }
        "log-report" => log_tools::cmd_log_report(root, sub_args),
        "cal-diff" => log_tools::cmd_cal_diff(root, sub_args),
        #[cfg(feature = "studio")]
        "replay" => studio::log_replay::cmd_replay(root, sub_args),
        other => Err(if is_ru() {
            format!("неизвестная команда {other:?}\n\n{}", cli_usage())
        } else {
            format!("unknown subcommand {other:?}\n\n{}", cli_usage())
        }),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cli_boot_and_can_server_arg_parsers_are_panic_free_and_ergonomic() {
        let v = |s: &[&str]| s.iter().map(|x| x.to_string()).collect::<Vec<_>>();
        assert_eq!(parse_boot_args(&v(&[])).unwrap(), (5_000_000, None));
        assert_eq!(parse_boot_args(&v(&["100000"])).unwrap(), (100_000, None));
        assert_eq!(
            parse_boot_args(&v(&["100000", "custom.bin"])).unwrap(),
            (100_000, Some("custom.bin"))
        );
        assert_eq!(
            parse_boot_args(&v(&["FL_8V0906264H__0005.frf"])).unwrap(),
            (5_000_000, Some("FL_8V0906264H__0005.frf"))
        );
        assert_eq!(
            parse_boot_args(&v(&["vault/firmwares/ecu/stock/8V0906264H__0005.bin"])).unwrap(),
            (5_000_000, Some("vault/firmwares/ecu/stock/8V0906264H__0005.bin"))
        );
        assert!(parse_boot_args(&v(&["not_a_number"])).is_err());
        assert!(parse_boot_args(&v(&["custom.bin", "extra"])).is_err());
        assert!(parse_boot_args(&v(&["100", "a.bin", "extra"])).is_err());

        assert_eq!(
            parse_can_server_args(&v(&["29000", "ecu.bin", "eep.bin"])).unwrap(),
            (29000, Some("ecu.bin"), Some("eep.bin"))
        );
        assert_eq!(
            parse_can_server_args(&v(&["FL_8V0906264H__0005.frf", "eep.bin"])).unwrap(),
            (slcan::DEFAULT_SLCAN_PORT, Some("FL_8V0906264H__0005.frf"), Some("eep.bin"))
        );
        assert!(parse_can_server_args(&v(&["99999"])).is_err());
        assert!(parse_can_server_args(&v(&["29000", "a.bin", "b.bin", "extra"])).is_err());
    }

    #[test]
    fn zero_arg_invocation_and_lang_flags_never_panic_and_default_to_english() {
        let v = |s: &[&str]| s.iter().map(|x| x.to_string()).collect::<Vec<_>>();
        let tmp = env::temp_dir().join(format!("simemu_cli_zero_arg_{}", std::process::id()));
        let _ = fs::remove_dir_all(&tmp);
        fs::create_dir_all(&tmp).unwrap();

        // Regression test for `.\simemu.exe` with zero arguments (and empty slice).
        run_cli_with_args(&tmp, &v(&[]), None).unwrap();
        run_cli_with_args(&tmp, &v(&["simemu.exe"]), None).unwrap();
        assert_eq!(simemu_damos::lang(), Lang::En);
        assert!(cli_usage().starts_with("Usage: simemu"));

        // Russian flag via `--lang ru`, `--lang=ru`, `-L ru`, `--ru`, and `SIMEMU_LANG=ru`.
        run_cli_with_args(&tmp, &v(&["simemu.exe", "--lang", "ru"]), None).unwrap();
        assert_eq!(simemu_damos::lang(), Lang::Ru);
        assert!(cli_usage().starts_with("Использование: simemu"));

        run_cli_with_args(&tmp, &v(&["simemu.exe", "--en", "--help"]), Some("ru")).unwrap();
        assert_eq!(simemu_damos::lang(), Lang::En);

        run_cli_with_args(&tmp, &v(&["simemu.exe", "boot", "--lang=ru", "--help"]), None).unwrap();
        assert_eq!(simemu_damos::lang(), Lang::Ru);

        run_cli_with_args(&tmp, &v(&["simemu.exe", "-L", "en", "-V"]), None).unwrap();
        assert_eq!(simemu_damos::lang(), Lang::En);

        // Subcommands with zero arguments in an empty directory return clean `Err`, never panic.
        assert!(run_cli_with_args(&tmp, &v(&["simemu.exe", "boot"]), None).is_err());
        assert!(run_cli_with_args(&tmp, &v(&["simemu.exe", "unpack-frf"]), None).is_err());
        assert!(run_cli_with_args(&tmp, &v(&["simemu.exe", "log-report"]), None).is_err());
        assert!(run_cli_with_args(&tmp, &v(&["simemu.exe", "--lang"]), None).is_err());
        assert!(run_cli_with_args(&tmp, &v(&["simemu.exe", "--lang", "de"]), None).is_err());

        set_lang(Lang::En);
        let _ = fs::remove_dir_all(&tmp);
    }
}
