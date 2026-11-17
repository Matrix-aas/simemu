//! MultiCAN module (4 nodes, 128 message objects, list-based MO allocation).
//!
//! Modelled per the MultiCAN register set used by the firmware (init 0x8010EB3C / SBOOT 0x80006E50):
//! panel commands (PANCTR: init lists, static/dynamic allocate, insert before/behind), list
//! registers, node control/status, MO control/status with SET/RESET pairs, transmit when
//! `MSGVAL & TXRQ & TXEN0 & TXEN1 & DIR` and the node (list x+1 <-> node x) is out of INIT,
//! acceptance filtering with MOAMR/MIDE delivering to the first matching receive MO in list order,
//! NEWDAT/MSGLST/RXPND/TXPND, MO interrupt pointers to the CAN SRCs and MSPND/MSID.
//! Simplification: a frame is transmitted (and appears in `can_tx_queue`) at the instant all
//! transmit conditions hold; bus timing/arbitration between nodes is not modelled.

use crate::memory::{CanFrame, Tc1791HardwareState};
use crate::stm_icu::can_src;

pub const CAN_BASE: u32 = 0xF000_4000;

// MOSTAT bits
pub const RXPND: u32 = 1 << 0;
pub const TXPND: u32 = 1 << 1;
pub const RXUPD: u32 = 1 << 2;
pub const NEWDAT: u32 = 1 << 3;
pub const MSGLST: u32 = 1 << 4;
pub const MSGVAL: u32 = 1 << 5;
pub const RXEN: u32 = 1 << 7;
pub const TXRQ: u32 = 1 << 8;
pub const TXEN0: u32 = 1 << 9;
pub const TXEN1: u32 = 1 << 10;
pub const DIR: u32 = 1 << 11;

#[derive(Default, Clone, Copy)]
pub struct Mo {
    pub fcr: u32,
    pub fgpr: u32,
    pub ipr: u32,
    pub amr: u32,
    pub datal: u32,
    pub datah: u32,
    pub ar: u32,
    /// MOSTAT bits 11:0 (list bits/pointers are derived from the list model).
    pub stat: u32,
}

pub struct MultiCan {
    pub mo: [Mo; 128],
    pub lists: [Vec<u8>; 16],
    pub mo_list: [u8; 128],
    pub panctr: u32,
    pub mspnd: [u32; 8],
    pub msimask: u32,
    pub mcr: u32,
    pub ncr: [u32; 4],
    pub nsr: [u32; 4],
    pub nipr: [u32; 4],
    pub tx_count: u64,
    pub rx_count: u64,
    pub rx_dropped: u64,
    pub panel_commands: u64,
    /// Implemented message objects, nodes and the offset of MO0 from the module base
    /// (TC1791: 128 / 4 / 0x1000; TC1766: 64 / 2 / 0x400).
    pub num_mo: usize,
    pub num_nodes: usize,
    pub mo_off: u32,
}

impl Default for MultiCan {
    fn default() -> Self {
        Self::new(128, 4, 0x1000)
    }
}

impl MultiCan {
    pub fn new(num_mo: usize, num_nodes: usize, mo_off: u32) -> Self {
        let num_mo = num_mo.min(128);
        let num_nodes = num_nodes.min(4);
        let mut c = Self {
            mo: [Mo::default(); 128],
            lists: Default::default(),
            mo_list: [0; 128],
            panctr: 0,
            mspnd: [0; 8],
            msimask: 0,
            mcr: 0,
            ncr: [1; 4],
            nsr: [0; 4],
            nipr: [0; 4],
            tx_count: 0,
            rx_count: 0,
            rx_dropped: 0,
            panel_commands: 0,
            num_mo,
            num_nodes,
            mo_off,
        };
        c.init_lists();
        c
    }

    fn init_lists(&mut self) {
        for l in self.lists.iter_mut() {
            l.clear();
        }
        self.lists[0] = (0..self.num_mo as u8).collect();
        self.mo_list = [0; 128];
    }

    fn unlink(&mut self, mo: u8) {
        let l = self.mo_list[mo as usize] as usize;
        self.lists[l].retain(|&m| m != mo);
    }

    fn place(&mut self, mo: u8, list: usize, pos: usize) {
        self.unlink(mo);
        let pos = pos.min(self.lists[list].len());
        self.lists[list].insert(pos, mo);
        self.mo_list[mo as usize] = list as u8;
    }

    /// Execute a panel command; returns the new PANCTR value (BUSY/RBUSY clear).
    pub fn panel(&mut self, v: u32) -> u32 {
        self.panel_commands += 1;
        let cmd = v & 0xFF;
        let mut ar1 = (v >> 16) & 0xFF;
        let ar2 = (v >> 24) & 0xFF;
        let pos_of = |c: &Self, mo: u8| {
            let l = c.mo_list[mo as usize] as usize;
            (l, c.lists[l].iter().position(|&m| m == mo).unwrap_or(0))
        };
        match cmd {
            0x01 => self.init_lists(),
            0x02 => {
                let mo = (ar1 & 0x7F) as usize;
                let l = (ar2 & 0xF) as usize;
                if mo < self.num_mo {
                    let end = self.lists[l].len();
                    self.place(mo as u8, l, end);
                } else {
                    ar1 = 0x80; // ERR: invalid MO index
                }
            }
            0x03 => {
                if let Some(&mo) = self.lists[0].first() {
                    let l = (ar2 & 0xF) as usize;
                    let end = self.lists[l].len();
                    self.place(mo, l, end);
                    ar1 = mo as u32;
                } else {
                    ar1 = 0x80; // ERR: no free MO in list 0
                }
            }
            0x05 | 0x07 => {
                let rf = (ar2 & 0x7F) as usize;
                if rf >= self.num_mo {
                    ar1 = 0x80;
                } else if let Some(&mo) = self.lists[0].first() {
                    let rf = rf as u8;
                    if mo != rf {
                        self.unlink(mo);
                        let (l, p) = pos_of(self, rf);
                        let p = if cmd == 0x05 { p } else { p + 1 };
                        self.place(mo, l, p);
                    }
                    ar1 = mo as u32;
                } else {
                    ar1 = 0x80; // ERR: no free MO in list 0
                }
            }
            0x04 | 0x06 => {
                let mo = (ar1 & 0x7F) as usize;
                let rf = (ar2 & 0x7F) as usize;
                if mo >= self.num_mo || rf >= self.num_mo {
                    ar1 = 0x80;
                } else if mo != rf {
                    let (mo, rf) = (mo as u8, rf as u8);
                    self.unlink(mo);
                    let (l, p) = pos_of(self, rf);
                    let p = if cmd == 0x04 { p } else { p + 1 };
                    self.place(mo, l, p);
                }
            }
            _ => {}
        }
        (v & 0xFF00_00FF & !0x0000_0300) | (ar1 << 16)
    }

    pub fn list_reg(&self, l: usize) -> u32 {
        let v = &self.lists[l];
        match (v.first(), v.last()) {
            (Some(&b), Some(&e)) => b as u32 | (e as u32) << 8 | ((v.len() as u32 - 1) & 0xFF) << 16,
            _ => 1 << 24,
        }
    }

    pub fn mostat(&self, n: usize) -> u32 {
        let l = self.mo_list[n] as usize;
        let list = &self.lists[l];
        let p = list.iter().position(|&m| m as usize == n).unwrap_or(0);
        let prev = if p == 0 { n as u32 } else { list[p - 1] as u32 };
        let next = list.get(p + 1).map_or(n as u32, |&m| m as u32);
        (self.mo[n].stat & 0xFFF) | (l as u32) << 12 | prev << 16 | next << 24
    }

    fn node_active(&self, node: usize) -> bool {
        node < self.num_nodes && self.ncr[node] & 0x11 == 0
    }
}

/// MOAR identifier field for a frame (extended: bits 28:0, standard: bits 28:18).
fn frame_moar_id(f: &CanFrame) -> u32 {
    if f.extended {
        f.arb_id & 0x1FFF_FFFF
    } else {
        (f.arb_id & 0x7FF) << 18
    }
}

impl Tc1791HardwareState {
    fn can_mo_irq(&mut self, n: usize, inp: u32) {
        let mpn = (self.can.mo[n].ipr >> 8) & 0xFF;
        self.can.mspnd[(mpn >> 5) as usize & 7] |= 1 << (mpn & 31);
        self.src_set_srr(can_src(inp & 0xF));
    }

    fn can_node_irq(&mut self, node: usize) {
        if self.can.ncr[node] & 0x2 != 0 {
            let trinp = (self.can.nipr[node] >> 8) & 0xF;
            self.src_set_srr(can_src(trinp));
        }
    }

    /// Transmit MO `n` if all hardware conditions hold.
    fn can_try_tx(&mut self, n: usize) {
        let need = MSGVAL | TXRQ | TXEN0 | TXEN1 | DIR;
        let l = self.can.mo_list[n] as usize;
        if self.can.mo[n].stat & need != need || !(1..=self.can.num_nodes).contains(&l) || !self.can.node_active(l - 1) {
            return;
        }
        let node = l - 1;
        let mo = self.can.mo[n];
        let extended = mo.ar & 0x2000_0000 != 0;
        let arb_id = if extended { mo.ar & 0x1FFF_FFFF } else { (mo.ar >> 18) & 0x7FF };
        let dlc = (((mo.fcr >> 24) & 0xF) as u8).min(8);
        let mut data = [0u8; 8];
        data[..4].copy_from_slice(&mo.datal.to_le_bytes());
        data[4..].copy_from_slice(&mo.datah.to_le_bytes());
        let ts = (self.stm_now() as u128 * self.stm.cycles_per_tick().max(1) as u128 * 1_000_000
            / self.scu.fcpu_hz().max(1) as u128) as u64;
        self.can_tx_queue.push(CanFrame {
            node: node as u8,
            mo_index: n as u8,
            arb_id,
            extended,
            dlc,
            data,
            timestamp_us: ts,
        });
        self.can.tx_count += 1;
        let st = &mut self.can.mo[n].stat;
        *st = (*st & !(TXRQ | NEWDAT)) | TXPND;
        self.can.nsr[node] |= 1 << 3; // TXOK
        if mo.fcr & (1 << 17) != 0 {
            self.can_mo_irq(n, (mo.ipr >> 4) & 0xF);
        }
        self.can_node_irq(node);
    }

    /// Deliver a received frame on `frame.node` to the first matching receive MO of that node's
    /// list. Returns the number of MOs updated (0 or 1).
    pub fn inject_can_rx(&mut self, frame: &CanFrame) -> usize {
        let node = frame.node as usize;
        if !self.can.node_active(node) {
            self.can.rx_dropped += 1;
            return 0;
        }
        let fid = frame_moar_id(frame);
        let matched = self.can.lists[node + 1].iter().copied().map(usize::from).find(|&n| {
            let mo = self.can.mo[n];
            if mo.stat & (MSGVAL | RXEN | DIR) != (MSGVAL | RXEN) {
                return false;
            }
            let mo_ext = mo.ar & 0x2000_0000 != 0;
            if mo.amr & 0x2000_0000 != 0 && mo_ext != frame.extended {
                return false;
            }
            let mut mask = mo.amr & 0x1FFF_FFFF;
            if !frame.extended {
                mask &= 0x1FFC_0000;
            }
            (mo.ar ^ fid) & mask == 0
        });
        if let Some(n) = matched {
            let mo = self.can.mo[n];
            let mut stat = mo.stat | RXUPD;
            if stat & NEWDAT != 0 {
                stat |= MSGLST;
            }
            let d = &frame.data;
            let mo = &mut self.can.mo[n];
            mo.datal = u32::from_le_bytes([d[0], d[1], d[2], d[3]]);
            mo.datah = u32::from_le_bytes([d[4], d[5], d[6], d[7]]);
            mo.fcr = (mo.fcr & 0xF0FF_FFFF) | ((frame.dlc.min(15) as u32) << 24);
            mo.ar = (mo.ar & 0xC000_0000) | (frame.extended as u32) << 29 | fid;
            mo.stat = (stat & !RXUPD) | NEWDAT | RXPND;
            let (fcr, ipr) = (mo.fcr, mo.ipr);
            self.can.rx_count += 1;
            self.can.nsr[node] |= 1 << 4; // RXOK
            if fcr & (1 << 16) != 0 {
                self.can_mo_irq(n, ipr & 0xF);
            }
            self.can_node_irq(node);
            return 1;
        }
        self.can.rx_dropped += 1;
        0
    }

    pub(crate) fn can_read(&self, addr: u32) -> Option<u32> {
        let off = addr - CAN_BASE;
        let c = &self.can;
        let nodes_end = 0x200 + 0x100 * c.num_nodes as u32;
        let mo_range = c.mo_off..c.mo_off + 0x20 * c.num_mo as u32;
        Some(match off {
            0x100..=0x13C => c.list_reg(((off - 0x100) / 4) as usize),
            0x140..=0x15C => c.mspnd[((off - 0x140) / 4) as usize],
            0x180..=0x19C => {
                let v = c.mspnd[((off - 0x180) / 4) as usize] & c.msimask;
                if v == 0 { 0x20 } else { v.trailing_zeros() }
            }
            0x1C0 => c.msimask,
            0x1C4 => c.panctr,
            0x1C8 => c.mcr,
            0x1CC => 0,
            _ if (0x200..nodes_end).contains(&off) => {
                let node = ((off - 0x200) >> 8) as usize;
                match off & 0xFF {
                    0x00 => c.ncr[node],
                    0x04 => c.nsr[node],
                    0x08 => c.nipr[node],
                    _ => return None,
                }
            }
            _ if mo_range.contains(&off) => {
                let n = ((off - c.mo_off) >> 5) as usize;
                let m = &c.mo[n];
                match off & 0x1F {
                    0x00 => m.fcr,
                    0x04 => m.fgpr,
                    0x08 => m.ipr,
                    0x0C => m.amr,
                    0x10 => m.datal,
                    0x14 => m.datah,
                    0x18 => m.ar,
                    _ => c.mostat(n),
                }
            }
            _ => return None,
        })
    }

    pub(crate) fn can_write(&mut self, addr: u32, val: u32, mask: u32) -> bool {
        let off = addr - CAN_BASE;
        let merge = |old: u32| (old & !mask) | (val & mask);
        let nodes_end = 0x200 + 0x100 * self.can.num_nodes as u32;
        let mo_range = self.can.mo_off..self.can.mo_off + 0x20 * self.can.num_mo as u32;
        match off {
            0x100..=0x13C | 0x180..=0x19C => {} // read-only
            0x140..=0x15C => {
                let i = ((off - 0x140) / 4) as usize;
                self.can.mspnd[i] &= val | !mask; // write 0 clears, 1 has no effect
            }
            0x1C0 => self.can.msimask = merge(self.can.msimask),
            0x1C4 => {
                let v = merge(self.can.panctr);
                self.can.panctr = self.can.panel(v);
            }
            0x1C8 => self.can.mcr = merge(self.can.mcr),
            0x1CC => {
                let it = val & mask & 0xFFFF;
                for i in 0..16 {
                    if it & (1 << i) != 0 {
                        self.src_set_srr(can_src(i));
                    }
                }
            }
            _ if (0x200..nodes_end).contains(&off) => {
                let node = ((off - 0x200) >> 8) as usize;
                match off & 0xFF {
                    0x00 => {
                        let was = self.can.node_active(node);
                        let mut v = merge(self.can.ncr[node]);
                        if v & 0x10 != 0 {
                            // CANDIS: the node leaves the bus and hardware sets INIT (bus idle).
                            v |= 0x01;
                        }
                        self.can.ncr[node] = v;
                        if !was && self.can.node_active(node) {
                            for m in self.can.lists[node + 1].clone() {
                                self.can_try_tx(m as usize);
                            }
                        }
                    }
                    0x04 => {
                        // LEC writable; TXOK/RXOK/ALERT/LLE/LOE cleared by writing 0.
                        let w = val | !mask;
                        let clr = 0x33F & !w & !0x7;
                        let lec = if mask & 0x7 != 0 { val & 7 } else { self.can.nsr[node] & 7 };
                        self.can.nsr[node] = (self.can.nsr[node] & !clr & !7) | lec;
                    }
                    0x08 => self.can.nipr[node] = merge(self.can.nipr[node]),
                    _ => return false,
                }
            }
            _ if mo_range.contains(&off) => {
                let n = ((off - self.can.mo_off) >> 5) as usize;
                let m = &mut self.can.mo[n];
                match off & 0x1F {
                    0x00 => m.fcr = merge(m.fcr),
                    0x04 => m.fgpr = merge(m.fgpr),
                    0x08 => m.ipr = merge(m.ipr),
                    0x0C => m.amr = merge(m.amr),
                    0x10 => m.datal = merge(m.datal),
                    0x14 => m.datah = merge(m.datah),
                    0x18 => m.ar = merge(m.ar),
                    _ => {
                        // MOCTR: bits 11:0 RESET, bits 27:16 SET (unwritten lanes are 0).
                        let v = val & mask;
                        let res = v & 0xFFF;
                        let set = (v >> 16) & 0xFFF;
                        let both = res & set;
                        m.stat = ((m.stat & !(res & !both)) | (set & !both)) & 0xFFF;
                        self.can_try_tx(n);
                    }
                }
            }
            _ => return false,
        }
        true
    }

    /// Return MultiCAN to its reset state (lists initialised, all nodes in INIT).
    pub(crate) fn can_reset(&mut self) {
        self.can = match self.variant {
            crate::memory::BoardVariant::Dq250Tc1766 => MultiCan::new(64, 2, 0x400),
            _ => MultiCan::default(),
        };
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::memory::BoardVariant;
    use crate::Simos18Board;

    fn board() -> Simos18Board {
        Simos18Board::new_variant(&vec![0u8; 0x40_0000], &vec![0u8; 0x4_0000], BoardVariant::Simos18Tc1791)
            .unwrap()
    }

    fn w(b: &mut Simos18Board, a: u32, v: u32) {
        b.hw_mut().can_write(a, v, 0xFFFF_FFFF);
    }

    fn mo(n: u32, r: u32) -> u32 {
        CAN_BASE + 0x1000 + 0x20 * n + r
    }

    fn setup_rx(b: &mut Simos18Board, n: u32, list: u32, id: u32, amr: u32) {
        w(b, CAN_BASE + 0x1C4, (list << 24) | (n << 16) | 2); // static allocate
        w(b, mo(n, 0x18), (id & 0x7FF) << 18);
        w(b, mo(n, 0x0C), amr);
        w(b, mo(n, 0x00), 1 << 16); // RXIE
        w(b, mo(n, 0x08), 1); // RXINP = 1
        w(b, mo(n, 0x1C), 0x00A0_0800); // SETMSGVAL | SETRXEN, RESDIR
    }

    fn frame(id: u32, d0: u8) -> CanFrame {
        CanFrame { node: 0, mo_index: 0, arb_id: id, extended: false, dlc: 8, data: [d0; 8], timestamp_us: 0 }
    }

    #[test]
    fn rx_acceptance_first_mo_in_list_and_msglst() {
        let mut b = board();
        w(&mut b, CAN_BASE + 0x200, 0); // node 0 out of INIT
        setup_rx(&mut b, 5, 1, 0x100, 0x3FFC_0000);
        setup_rx(&mut b, 3, 1, 0x100, 0x3FFC_0000); // later in list 1
        setup_rx(&mut b, 7, 1, 0x200, 0x3FFC_0000);
        let hw = b.hw_mut();
        assert_eq!(hw.inject_can_rx(&frame(0x100, 0x11)), 1);
        assert_eq!(hw.can.mo[5].stat & (NEWDAT | RXPND), NEWDAT | RXPND);
        assert_eq!(hw.can.mo[3].stat & NEWDAT, 0, "only one MO receives");
        assert_eq!(hw.can.mo[5].stat & MSGLST, 0);
        hw.inject_can_rx(&frame(0x100, 0x22));
        assert_ne!(hw.can.mo[5].stat & MSGLST, 0, "overwrite while NEWDAT set => MSGLST");
        assert_eq!(hw.can.mo[5].datal, 0x2222_2222);
        assert_eq!(hw.inject_can_rx(&frame(0x300, 0)), 0);
        // RXIE + RXINP=1 -> CAN SRC1 requested
        assert_ne!(hw.src_get(can_src(1)) & crate::stm_icu::SRC_SRR, 0);
        // a node in INIT does not receive
        let mut f = frame(0x200, 1);
        f.node = 1;
        assert_eq!(hw.inject_can_rx(&f), 0);
    }

    #[test]
    fn rx_mask_and_dlc_over_8_does_not_panic() {
        let mut b = board();
        w(&mut b, CAN_BASE + 0x200, 0);
        setup_rx(&mut b, 1, 1, 0x7F0, 0x3FC0_0000); // mask ignores ID bits 3:0
        let mut f = frame(0x7F5, 0x33);
        f.dlc = 15;
        assert_eq!(b.hw_mut().inject_can_rx(&f), 1);
        assert_eq!((b.hw().can.mo[1].fcr >> 24) & 0xF, 15, "DLC 9..15 is legal and stored");
        assert_eq!(b.hw().can.mo[1].datah, 0x3333_3333);
    }

    #[test]
    fn tx_requires_all_conditions_and_node_running() {
        let mut b = board();
        w(&mut b, CAN_BASE + 0x1C4, (2 << 24) | (9 << 16) | 2); // MO9 -> list 2 (node 1)
        w(&mut b, mo(9, 0x18), 0x123 << 18);
        w(&mut b, mo(9, 0x00), 4 << 24);
        w(&mut b, mo(9, 0x10), 0xDDCC_BBAA);
        w(&mut b, mo(9, 0x1C), 0x0E20_0000); // SETMSGVAL|SETTXEN0|SETTXEN1|SETDIR
        w(&mut b, mo(9, 0x1C), 0x0100_0000); // SETTXRQ
        assert!(b.hw().can_tx_queue.is_empty(), "node 1 still in INIT");
        w(&mut b, CAN_BASE + 0x300, 0);
        let hw = b.hw();
        assert_eq!(hw.can_tx_queue.len(), 1);
        let f = hw.can_tx_queue[0];
        assert_eq!((f.node, f.arb_id, f.dlc), (1, 0x123, 4));
        assert_eq!(&f.data[..4], &[0xAA, 0xBB, 0xCC, 0xDD]);
        assert_eq!(hw.can.mo[9].stat & (TXRQ | TXPND), TXPND);
        // MOSTAT reflects list 2
        assert_eq!((hw.can.mostat(9) >> 12) & 0xF, 2);
    }

    #[test]
    fn panel_lists() {
        let mut c = MultiCan::default();
        c.panel(0x0100_0002); // MO0 -> list 1
        c.panel(0x0101_0002); // MO1 -> list 1
        assert_eq!(c.list_reg(1), (1 << 8) | (1 << 16));
        assert_eq!(c.mostat(0) >> 16 & 0xFF, 0);
        assert_eq!(c.mostat(0) >> 24, 1);
        let r = c.panel(0x0100_0003); // dynamic allocate -> list 1
        assert_eq!((r >> 16) & 0xFF, 2);
        assert_eq!(c.lists[1], vec![0, 1, 2]);
    }

    #[test]
    fn panel_rejects_oob_mo_on_64_mo_variant_and_dynamic_insert_order() {
        let mut c = MultiCan::new(64, 3, 0x1000);
        assert_eq!(c.num_mo, 64);
        // OOB MO 64 and 127 on a 64-MO controller (TC1766) must set PANCTR.ERR (PANARG1 bit 7 = 0x80) without panicking
        assert_eq!((c.panel((1 << 24) | (64 << 16) | 2) >> 16) & 0xFF, 0x80);
        assert_eq!((c.panel((64 << 24) | (1 << 16) | 6) >> 16) & 0xFF, 0x80);
        assert_eq!((c.panel((1 << 24) | (127 << 16) | 4) >> 16) & 0xFF, 0x80);

        // Dynamic insert before (0x05): first move MO 5 to list 1 so lists[0] = [0,1,2,3,4,6,...],
        // then dynamically allocate from lists[0] (picks MO 0) and insert before MO 3 (also in lists[0]).
        c.panel((1 << 24) | (5 << 16) | 2);
        c.panel((3 << 24) | 0x05); // picks first free MO 0 from lists[0], unlinks 0, inserts before MO 3 in lists[0]
        assert_eq!(&c.lists[0][..5], &[1, 2, 0, 3, 4], "MO 0 must be unlinked before computing pos_of(MO 3)");
    }
}
