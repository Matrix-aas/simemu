//! Diagnostic (run with `--release --ignored --nocapture`): a board on the Unicorn and on the
//! native backend side by side under the stimuli of `backend_diff.rs`, comparing the ordered streams
//! of peripheral accesses and interrupt entries with their instruction counts
//! (`Simos18Board::event_log_mut`). Prints the first event that differs, with the preceding events
//! and quantum ends of each backend. `SIMEMU_LOCK_MS` (default 3000) bounds the run,
//! `SIMEMU_LOCK_CTX` (default 12) the context.

mod common;

use simemu_tc1791::{CanFrame, CpuBackend, Dq200TcuBoard, Simos18Board, Wfs5ImmoPartner};
use std::collections::VecDeque;

const FACTORY: &str = "8V0906264K__0003.bin";
const EEPROM: &str = common::ECU_DFLASH;
const TCU_FLASH: &str = common::DQ200_FLASH;
const TCU_EEPROM: &str = common::DQ200_EEPROM;

fn find(name: &str) -> Option<Vec<u8>> {
    common::find_image(name)
}

fn env(k: &str, d: u64) -> u64 {
    std::env::var(k).ok().and_then(|v| v.parse().ok()).unwrap_or(d)
}

type Ev = (u64, u8, u32, u64);

fn show(e: &Ev) -> String {
    match e.1 {
        3 => format!("{:>11} --- quantum end: {} of budget {}", e.0, e.2, e.3),
        k => format!("{:>11} {:3} {:08X} {:#x}", e.0, ["R", "W", "IRQ"][k as usize], e.2, e.3),
    }
}

/// Events of one backend not yet compared, plus the recent history (quantum ends included).
#[derive(Default)]
struct Side {
    pending: VecDeque<Ev>,
    hist: VecDeque<Ev>,
}

impl Side {
    /// Next comparable event (quantum ends go to the history only).
    fn front(&mut self, ctx: usize) -> Option<Ev> {
        while let Some(e) = self.pending.front().copied() {
            if e.1 != 3 {
                return Some(e);
            }
            self.pop(ctx);
        }
        None
    }
    fn pop(&mut self, ctx: usize) {
        if let Some(e) = self.pending.pop_front() {
            self.hist.push_back(e);
            if self.hist.len() > ctx {
                self.hist.pop_front();
            }
        }
    }
}

/// Run `frame(board, i)` on both boards for every 10 ms frame and compare their event streams.
fn lockstep(name: &str, mut b: [&mut Simos18Board; 2], mut frame: impl FnMut(&mut Simos18Board, u64)) {
    for x in b.iter_mut() {
        *x.event_log_mut() = Some(Vec::new());
    }
    let ctx = env("SIMEMU_LOCK_CTX", 12) as usize;
    let (mut u, mut n) = (Side::default(), Side::default());
    let mut matched = 0u64;
    for i in 0..env("SIMEMU_LOCK_MS", 3000) / 10 {
        for x in b.iter_mut() {
            frame(x, i);
        }
        u.pending.extend(b[0].event_log_mut().as_mut().unwrap().drain(..));
        n.pending.extend(b[1].event_log_mut().as_mut().unwrap().drain(..));
        while let (Some(eu), Some(en)) = (u.front(ctx), n.front(ctx)) {
            if eu != en {
                println!("{name}: first difference after {matched} equal events (frame {i}). Unicorn:");
                u.hist.iter().chain(u.pending.iter().take(ctx)).for_each(|e| println!("  {}", show(e)));
                println!("native:");
                n.hist.iter().chain(n.pending.iter().take(ctx)).for_each(|e| println!("  {}", show(e)));
                return;
            }
            u.pop(ctx);
            n.pop(ctx);
            matched += 1;
        }
    }
    println!("{name}: no difference in {matched} events");
    final_state(name, &b);
}

/// Compare the end state of both boards part by part (what a digest would fold together).
fn final_state(name: &str, b: &[&mut Simos18Board; 2]) {
    use unicorn_tricore_sys::TriCoreReg as R;
    let regs = |x: &Simos18Board| {
        let mut v: Vec<(String, u32)> = (0..16).flat_map(|i| [(format!("D{i}"), x.read_reg(R::d(i))), (format!("A{i}"), x.read_reg(R::a(i)))]).collect();
        v.push(("PSW(arch)".into(), x.psw()));
        for r in [R::PC, R::PSW, R::PCXI, R::ICR, R::FCX, R::LCX, R::BIV, R::BTV, R::ISP] {
            v.push((format!("{r:?}"), x.read_reg(r)));
        }
        v
    };
    for (a, c) in regs(b[0]).iter().zip(regs(b[1])) {
        if a.1 != c.1 {
            println!("{name}: {} Unicorn {:#010x} native {:#010x}", a.0, a.1, c.1);
        }
    }
    let (u, n) = (b[0].hw(), b[1].hw());
    for (what, x, y) in [("DSPR", &u.dspr[..], &n.dspr[..]), ("PSPR", &u.pspr[..], &n.pspr[..]), ("LMU", &u.lmu[..], &n.lmu[..]), ("DFlash", &u.dflash[..], &n.dflash[..])] {
        if let Some(i) = x.iter().zip(y).position(|(p, q)| p != q) {
            println!("{name}: {what} differs first at +{i:#x}");
        }
    }
    let ms = |h: &simemu_tc1791::Tc1791HardwareState| h.milestones.iter().map(|m| (m.name, m.pc, m.stm_ticks)).collect::<Vec<_>>();
    for (x, y) in ms(u).iter().zip(ms(n)) {
        if *x != y {
            println!("{name}: milestone Unicorn {x:x?} native {y:x?}");
        }
    }
    if u.rtos_task_counts != n.rtos_task_counts || u.rtos_isr_counts != n.rtos_isr_counts {
        println!("{name}: RTOS dispatch counts differ");
    }
    println!("{name}: end state compared");
}

#[test]
#[ignore]
fn ecu_event_streams_lockstep() {
    if !CpuBackend::Unicorn.is_available() {
        eprintln!("SKIP: built without `--features unicorn`");
        return;
    }
    let (Some(flash), Some(eeprom)) = (find(FACTORY), find(EEPROM)) else {
        eprintln!("SKIP: SC800S50 images not found");
        return;
    };
    let mut b = [CpuBackend::Unicorn, CpuBackend::Native].map(|k| Simos18Board::new_on(&flash, &eeprom, k).unwrap());
    for x in &mut b {
        x.set_adc_input_voltage(2, 4, 2.5).unwrap();
    }
    let [u, n] = &mut b;
    lockstep("SIMOS18.1 SC800S50", [u, n], |x, i| {
        if i % 10 == 5 {
            let f = CanFrame { node: 0, mo_index: 0, arb_id: 0x3C0, extended: false, dlc: 4, data: [0, 0, 0x03, 0, 0, 0, 0, 0], timestamp_us: 0 };
            x.inject_can_rx(&f);
        }
        x.step_realtime_frame(0.010).unwrap();
    });
}

#[test]
#[ignore]
fn tcu_event_streams_lockstep() {
    if !CpuBackend::Unicorn.is_available() {
        eprintln!("SKIP: built without `--features unicorn`");
        return;
    }
    let (Some(flash), Some(eeprom)) = (find(TCU_FLASH), find(TCU_EEPROM)) else {
        eprintln!("SKIP: DQ200_G2 images not found");
        return;
    };
    let mut t = [CpuBackend::Unicorn, CpuBackend::Native].map(|k| Dq200TcuBoard::new_on(&flash, &eeprom, k).unwrap());
    for x in &mut t {
        x.inject_full_mechatronic_plant(13.85, 52.0, 40.0, [0.0; 4], 0.0, 0.0, 780.0, 0.0, 0.0);
    }
    let immo = Wfs5ImmoPartner::from_dq200_images(&flash, &eeprom);
    let [u, n] = &mut t;
    lockstep("DQ200_G2", [&mut u.cpu, &mut n.cpu], |x, _| {
        x.run_for(0.010).unwrap();
        for f in x.take_can_tx() {
            if f.arb_id == 0x012 {
                let a = CanFrame { node: 0, mo_index: 0, arb_id: 0x013, extended: false, dlc: 8, data: immo.answer_challenge(&f.data), timestamp_us: 0 };
                x.inject_can_rx(&a);
            }
        }
    });
}
