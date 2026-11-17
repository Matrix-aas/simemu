//! ST M95640 (64 Kbit = 8 KB) SPI serial EEPROM, as fitted on the DQ200_G2 mechatronic board
//! (SSC0 of the TC1784, chip select on a GPIO, see `tc1784_io`).
//!
//! Modelled per the M95640 datasheet: instruction set WREN 06h, WRDI 04h, RDSR 05h, WRSR 01h,
//! READ 03h, WRITE 02h; 16-bit address (A12..A0 used, upper bits ignored); status register
//! SRWD(7) BP1(3) BP0(2) WEL(1) WIP(0); 32-byte pages (a WRITE that runs past the page end wraps to
//! the page start); WRITE/WRSR are latched while CS is low and committed on the CS rising edge,
//! then the internal write cycle runs for `WRITE_CYCLE_S` with WIP = 1 (only RDSR is accepted);
//! WEL is reset at the end of every WRITE/WRSR cycle and by WRDI. Block protection BP1:BP0 =
//! 01 upper quarter, 10 upper half, 11 whole array. The W pin is not wired in this model (held
//! high), so SRWD has no effect.

pub const SIZE: usize = 0x2000;
pub const PAGE: usize = 32;
/// Write cycle time tW (datasheet maximum 5 ms).
pub const WRITE_CYCLE_S: f64 = 5e-3;

const SR_WIP: u8 = 0x01;
const SR_WEL: u8 = 0x02;
const SR_BP: u8 = 0x0C;
const SR_SRWD: u8 = 0x80;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum State {
    /// CS high (deselected).
    Idle,
    /// CS low, waiting for the instruction byte.
    Cmd,
    Addr { cmd: u8, hi: Option<u8> },
    Read(u16),
    /// WRITE data phase: page base, offset of the next byte in the page.
    Write { page: u16, off: u8 },
    Rdsr,
    Wrsr,
    /// Instruction complete or ignored; further bytes are don't-care until CS rises.
    Done,
}

pub struct St95640 {
    pub mem: Vec<u8>,
    sr: u8,
    state: State,
    /// Pending page write: (page base, data latched per byte).
    pending: Option<(u16, [Option<u8>; PAGE])>,
    pending_sr: Option<u8>,
    /// Simulated time (s) at which the running internal write cycle ends.
    busy_until: f64,
    pub writes_committed: u64,
    /// Start addresses of the READ instructions (diagnostic, bounded).
    pub read_log: Vec<u16>,
}

impl St95640 {
    pub fn new(image: &[u8]) -> Self {
        let mut mem = vec![0xFF; SIZE];
        let n = image.len().min(SIZE);
        mem[..n].copy_from_slice(&image[..n]);
        Self { mem, sr: 0, state: State::Idle, pending: None, pending_sr: None, busy_until: 0.0, writes_committed: 0, read_log: Vec::new() }
    }

    fn busy(&mut self, now: f64) -> bool {
        if self.sr & SR_WIP != 0 && now >= self.busy_until {
            self.sr &= !(SR_WIP | SR_WEL);
        }
        self.sr & SR_WIP != 0
    }

    pub fn status(&mut self, now: f64) -> u8 {
        self.busy(now);
        self.sr
    }

    fn protected(&self, addr: u16) -> bool {
        let a = addr as usize & (SIZE - 1);
        match (self.sr & SR_BP) >> 2 {
            1 => a >= SIZE * 3 / 4,
            2 => a >= SIZE / 2,
            3 => true,
            _ => false,
        }
    }

    /// Chip select level change (`low` = selected). A rising edge commits a latched WRITE/WRSR.
    pub fn set_cs(&mut self, low: bool, now: f64) {
        match (low, self.state) {
            (true, State::Idle) => self.state = State::Cmd,
            (true, _) => {}
            (false, State::Idle) => {}
            (false, _) => {
                self.state = State::Idle;
                let busy = self.busy(now);
                if let Some((page, data)) = self.pending.take() {
                    if !busy && self.sr & SR_WEL != 0 {
                        for (i, b) in data.iter().enumerate() {
                            if let Some(b) = b {
                                let a = page as usize + i;
                                if !self.protected(a as u16) {
                                    self.mem[a] = *b;
                                }
                            }
                        }
                        self.start_cycle(now);
                    }
                }
                if let Some(v) = self.pending_sr.take() {
                    if !busy && self.sr & SR_WEL != 0 {
                        self.sr = (self.sr & !(SR_BP | SR_SRWD)) | (v & (SR_BP | SR_SRWD));
                        self.start_cycle(now);
                    }
                }
            }
        }
    }

    fn start_cycle(&mut self, now: f64) {
        self.sr |= SR_WIP;
        self.busy_until = now + WRITE_CYCLE_S;
        self.writes_committed += 1;
    }

    /// Full-duplex byte exchange while CS is low: returns the byte driven on Q (MISO).
    /// Q is high impedance when deselected or not outputting; the board pull-up reads 0xFF.
    pub fn xfer(&mut self, d: u8, now: f64) -> u8 {
        match self.state {
            State::Idle | State::Done => 0xFF,
            State::Cmd => {
                let busy = self.busy(now);
                self.state = match d {
                    0x05 => State::Rdsr,
                    _ if busy => State::Done, // during tW only RDSR is executed
                    0x06 => {
                        self.sr |= SR_WEL;
                        State::Done
                    }
                    0x04 => {
                        self.sr &= !SR_WEL;
                        State::Done
                    }
                    0x01 => State::Wrsr,
                    0x02 | 0x03 => State::Addr { cmd: d, hi: None },
                    _ => State::Done,
                };
                0xFF
            }
            State::Addr { cmd, hi: None } => {
                self.state = State::Addr { cmd, hi: Some(d) };
                0xFF
            }
            State::Addr { cmd, hi: Some(h) } => {
                let addr = (u16::from_be_bytes([h, d]) as usize & (SIZE - 1)) as u16;
                self.state = if cmd == 0x03 {
                    if self.read_log.len() < 4096 {
                        self.read_log.push(addr);
                    }
                    State::Read(addr)
                } else {
                    let page = addr & !(PAGE as u16 - 1);
                    self.pending = Some((page, [None; PAGE]));
                    State::Write { page, off: (addr as usize % PAGE) as u8 }
                };
                0xFF
            }
            State::Read(a) => {
                self.state = State::Read((a + 1) & (SIZE as u16 - 1));
                self.mem[a as usize]
            }
            State::Write { page, off } => {
                if let Some((_, data)) = self.pending.as_mut() {
                    data[off as usize] = Some(d);
                }
                self.state = State::Write { page, off: (off + 1) % PAGE as u8 };
                0xFF
            }
            State::Rdsr => self.status(now),
            State::Wrsr => {
                self.pending_sr = Some(d);
                self.state = State::Done;
                0xFF
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cmd(e: &mut St95640, bytes: &[u8], t: f64) -> Vec<u8> {
        e.set_cs(true, t);
        let r = bytes.iter().map(|&b| e.xfer(b, t)).collect();
        e.set_cs(false, t);
        r
    }

    #[test]
    fn read_write_page_wrap_wel_wip() {
        let mut e = St95640::new(&[0x11; SIZE]);
        assert_eq!(cmd(&mut e, &[0x03, 0x01, 0xFF, 0, 0], 0.0)[3..], [0x11, 0x11]);
        // WRITE without WREN is ignored
        cmd(&mut e, &[0x02, 0x00, 0x10, 0xAB], 0.0);
        assert_eq!(e.mem[0x10], 0x11);
        // WREN, WRITE across the page end wraps to the page start; committed on CS rise
        cmd(&mut e, &[0x06], 0.0);
        assert_eq!(cmd(&mut e, &[0x05, 0], 0.0)[1] & SR_WEL, SR_WEL);
        e.set_cs(true, 0.0);
        for b in [0x02, 0x00, 0x3F, 0xA1, 0xA2] {
            e.xfer(b, 0.0);
        }
        assert_eq!(e.mem[0x3F], 0x11, "not committed before CS rises");
        e.set_cs(false, 0.0);
        assert_eq!((e.mem[0x3F], e.mem[0x20], e.mem[0x40]), (0xA1, 0xA2, 0x11));
        // WIP during tW; READ ignored while busy; WEL/WIP clear afterwards
        assert_eq!(cmd(&mut e, &[0x05, 0], 1e-3)[1] & 3, 3);
        assert_eq!(cmd(&mut e, &[0x03, 0x00, 0x3F, 0], 1e-3)[3], 0xFF);
        assert_eq!(cmd(&mut e, &[0x05, 0], 6e-3)[1] & 3, 0);
        assert_eq!(cmd(&mut e, &[0x03, 0x00, 0x3F, 0], 6e-3)[3], 0xA1);
    }

    #[test]
    fn wrsr_block_protect_and_wrdi() {
        let mut e = St95640::new(&[]);
        cmd(&mut e, &[0x06], 0.0);
        cmd(&mut e, &[0x01, 0x0C], 0.0); // BP = 11: whole array protected
        assert_eq!(cmd(&mut e, &[0x05, 0], 0.01)[1], 0x0C);
        cmd(&mut e, &[0x06], 0.01);
        cmd(&mut e, &[0x02, 0x00, 0x00, 0x55], 0.01);
        assert_eq!(e.mem[0], 0xFF);
        cmd(&mut e, &[0x06], 0.02);
        cmd(&mut e, &[0x04], 0.02);
        assert_eq!(cmd(&mut e, &[0x05, 0], 0.02)[1] & SR_WEL, 0);
    }
}
