//! One servo bus segment: a single-wire half-duplex UART at 6 Mbps. The
//! segment scans IDs 0 to 14 for servos that answer, then polls every servo it
//! found once per 2 ms tick through a bench test mode chosen over the USB
//! console.

use core::cell::Cell;

use embassy_stm32::mode::Async;
use embassy_stm32::usart::{self, Uart};
use embassy_futures::join::join;
use embassy_futures::yield_now;
use embassy_time::{with_timeout, Duration, Instant, Ticker};

use crate::j288::{self, Command, Mode, State};

/// Bench test modes, the same for every servo.
#[derive(Clone, Copy, PartialEq, Eq, Default)]
pub enum TestMode {
    /// Mode 0: windings shorted, a damped brake. Nothing moves.
    #[default]
    Stop,
    /// Mode 1, kd only: the joint feels viscous and can be turned by hand.
    Damp,
    /// Mode 1, hold the position the joint had when the mode was chosen.
    Hold,
    /// Mode 1, a slow sine about the position the joint had when the mode was chosen.
    Wiggle,
    /// One clear-faults frame, then Stop.
    ClearFaults,
}

/// The console's choice, with counters so segments notice a re-selection or a
/// rescan request.
#[derive(Clone, Copy)]
pub struct Control {
    pub mode: TestMode,
    pub generation: u32,
    pub rescan: u32,
    /// Run each transaction as a polled register loop with interrupts off (the
    /// default), or through embassy's DMA read and write. On the bench the DMA
    /// receive overran on about 98% of transactions on all three UARTs while the
    /// polled loop missed nothing; the DMA path stays selectable to investigate.
    pub polled: bool,
    /// Polled path only: drive the data pin push-pull while transmitting and
    /// release it (open drain) for the reply (the default), or open drain
    /// throughout, which needs an external pull-up to make our rising edges.
    /// Active drive needs no pull-up, so a replying servo only has to pull the
    /// line low against the other servos' own weak pull-ups: on the bench a 1 k
    /// pull-up already cost replies with three servos on a segment, and none got
    /// through with five, while active drive ran five per segment cleanly.
    pub active_drive: bool,
}

impl Default for Control {
    fn default() -> Self {
        Self { mode: TestMode::Stop, generation: 0, rescan: 0, polled: true, active_drive: true }
    }
}

/// The bus allows IDs 0 to 14; 15 is broadcast and never answers.
pub const MAX_SERVOS: usize = 15;

// Gains from the bench tools (scripts/j288/explore.py position test): gentle
// enough for a loose servo on the bench.
const KP: f32 = 0.5; // N.m/rad, output side
const KD: f32 = 0.05; // N.m.s/rad
const WIGGLE_AMPLITUDE: f32 = 0.3; // rad
const WIGGLE_HZ: f32 = 0.5;
/// Above this a servo drops to Stop until a mode is chosen again.
const SPEED_LIMIT: f32 = 10.0; // rad/s
const PERIOD: Duration = Duration::from_micros(2000);
/// Command out (33 us at 6 Mbps) and reply back (43 us) normally take about 100 us.
const TRANSACTION_TIMEOUT: Duration = Duration::from_micros(1000);
/// Without a valid reply for this long, assume the servo lost power or latched
/// its frame timeout, and clear the latch before driving it again.
const RECOVER_AFTER: Duration = Duration::from_millis(200);
const RECOVER_FOR: Duration = Duration::from_millis(500);
/// Frames per ID during a scan; an ID counts as present on its first valid reply.
const SCAN_TRIES: u32 = 3;
/// A segment that found no servos scans again this often.
const EMPTY_RESCAN: Duration = Duration::from_secs(1);

/// One servo's link statistics and last state.
#[derive(Clone, Copy, Default)]
pub struct Stats {
    pub id: u8,
    pub sent: u32,
    pub replies: u32,
    pub no_reply: u32,
    pub bad_frames: u32,
    pub uart_errors: u32,
    pub noise: u32,
    pub framing: u32,
    pub overrun: u32,
    /// Our own frame did not read back as sent: the bridge's side of the line is wrong.
    pub echo_mismatch: u32,
    pub round_trip_us: u32,
    pub round_trip_max_us: u32,
    pub speed_trips: u32,
    pub recovering: bool,
    pub last: Option<State>,
}

/// What a segment reports: its scan result and one `Stats` per servo found.
#[derive(Clone, Copy, Default)]
pub struct SegmentStats {
    pub scans: u32,
    pub scanning: bool,
    pub count: usize,
    pub servos: [Stats; MAX_SERVOS],
    /// IDs that gave only corrupt replies in the last scan, as a bit mask: two
    /// servos sharing an ID answer together and garble each other.
    pub garbled: u16,
    /// From the last scan: frames sent, frames whose echo read back wrong, and the
    /// fewest and most bytes received for a frame (20 is the echo alone, 46 a
    /// full reply).
    pub scan_frames: u32,
    pub scan_echo_bad: u32,
    pub scan_bytes_min: usize,
    pub scan_bytes_max: usize,
}

/// Per-servo control state that only the segment loop needs.
#[derive(Clone, Copy)]
struct Track {
    origin: Option<f32>,
    tripped: bool,
    last_reply: Instant,
    recover_until: Instant,
}

enum Outcome {
    Reply(State),
    /// Fewer bytes than echo and reply arrived (polled path: how many did).
    NoReply(usize),
    Uart(usart::Error),
    BadFrame,
}

/// One exchange: send `cmd` and read its echo and the reply. The receiver stays
/// on while we transmit (readback), and one read takes our own 20-byte echo
/// followed by the servo's 26-byte reply, so there is no gap at the turnaround
/// for the reply to fall into. Returns the outcome and whether the echo matched.
async fn exchange(uart: &mut Uart<'_, Async>, raw: Raw, ctl: &Control, cmd: &Command) -> (Outcome, bool) {
    let frame = j288::encode(cmd);
    let mut rx_buf = [0u8; j288::COMMAND_LEN + j288::REPLY_LEN];
    let result = if ctl.polled {
        let (n, flags) = raw.transact(&frame, &mut rx_buf, ctl.active_drive);
        if flags & ISR_ORE != 0 {
            Ok(Err(usart::Error::Overrun))
        } else if flags & ISR_FE != 0 {
            Ok(Err(usart::Error::Framing))
        } else if flags & ISR_NE != 0 {
            Ok(Err(usart::Error::Noise))
        } else if n < rx_buf.len() {
            Err(n)
        } else {
            Ok(Ok(()))
        }
    } else {
        let (tx, rx) = uart.split_ref();
        with_timeout(
            TRANSACTION_TIMEOUT,
            join(rx.read(&mut rx_buf), async {
                yield_now().await;
                tx.write(&frame).await
            }),
        )
        .await
        .map(|(read, write)| write.and(read))
        .map_err(|_| 0)
    };
    let echo_ok = rx_buf[..j288::COMMAND_LEN] == frame;
    let reply: &[u8; j288::REPLY_LEN] = rx_buf[j288::COMMAND_LEN..].try_into().unwrap();
    let outcome = match result {
        Err(n) => Outcome::NoReply(n),
        Ok(Err(e)) => Outcome::Uart(e),
        // A reply from another ID would also be a bad frame here.
        Ok(Ok(())) => match j288::decode(reply) {
            Ok(state) if state.id == cmd.id => Outcome::Reply(state),
            _ => Outcome::BadFrame,
        },
    };
    (outcome, echo_ok)
}

/// Asks every ID with stop frames (timeout bit clear) and records which answer.
async fn scan(uart: &mut Uart<'_, Async>, raw: Raw, ctl: &Control, seg: &mut SegmentStats) {
    seg.count = 0;
    seg.garbled = 0;
    seg.scan_frames = 0;
    seg.scan_echo_bad = 0;
    seg.scan_bytes_min = usize::MAX;
    seg.scan_bytes_max = 0;
    for id in 0..MAX_SERVOS as u8 {
        let mut corrupt = false;
        for _ in 0..SCAN_TRIES {
            let (outcome, echo_ok) = exchange(uart, raw, ctl, &Command::stop(id, false)).await;
            seg.scan_frames += 1;
            if !echo_ok {
                seg.scan_echo_bad += 1;
            }
            let bytes = match outcome {
                Outcome::NoReply(n) => n,
                _ => j288::COMMAND_LEN + j288::REPLY_LEN,
            };
            seg.scan_bytes_min = seg.scan_bytes_min.min(bytes);
            seg.scan_bytes_max = seg.scan_bytes_max.max(bytes);
            match outcome {
                Outcome::Reply(state) => {
                    seg.servos[seg.count] = Stats { id, last: Some(state), ..Stats::default() };
                    seg.count += 1;
                    corrupt = false;
                    break;
                }
                Outcome::BadFrame | Outcome::Uart(_) => corrupt = true,
                Outcome::NoReply(_) => {}
            }
            yield_now().await;
        }
        if corrupt {
            seg.garbled |= 1 << id;
        }
    }
    seg.scans += 1;
}

pub async fn run(
    uart: &mut Uart<'_, Async>,
    control: &Cell<Control>,
    shared: &Cell<SegmentStats>,
    raw: Raw,
) {
    let mut seg = SegmentStats::default();
    let mut tracks = [Track {
        origin: None,
        tripped: false,
        last_reply: Instant::MIN,
        recover_until: Instant::MIN,
    }; MAX_SERVOS];
    let mut seen_generation = u32::MAX;
    let mut seen_rescan = control.get().rescan;
    let mut mode_start = Instant::now();
    let mut last_scan = Instant::MIN;
    let mut need_scan = true;
    let mut ticker = Ticker::every(PERIOD);

    loop {
        let ctl = control.get();
        if ctl.rescan != seen_rescan {
            seen_rescan = ctl.rescan;
            need_scan = true;
        }
        if seg.count == 0 && Instant::now() - last_scan > EMPTY_RESCAN {
            need_scan = true;
        }
        if need_scan {
            need_scan = false;
            seg.scanning = true;
            shared.set(seg);
            scan(uart, raw, &ctl, &mut seg).await;
            seg.scanning = false;
            last_scan = Instant::now();
            for (track, stats) in tracks.iter_mut().zip(seg.servos.iter()).take(seg.count) {
                *track = Track {
                    origin: stats.last.map(|s| s.position),
                    tripped: false,
                    last_reply: last_scan,
                    recover_until: last_scan + RECOVER_FOR,
                };
            }
            shared.set(seg);
            ticker.reset();
        }

        ticker.next().await;
        let now = Instant::now();
        let new_mode = ctl.generation != seen_generation;
        if new_mode {
            seen_generation = ctl.generation;
            mode_start = now;
        }
        let t = (now - mode_start).as_micros() as f32 * 1e-6;

        for (stats, track) in seg.servos.iter_mut().zip(tracks.iter_mut()).take(seg.count) {
            let id = stats.id;
            if new_mode {
                track.origin = stats.last.map(|s| s.position);
                track.tripped = false;
            }
            if now - track.last_reply > RECOVER_AFTER && now >= track.recover_until {
                track.recover_until = now + RECOVER_FOR;
            }
            stats.recovering = now < track.recover_until;

            let cmd = if stats.recovering {
                // Stop frames with the timeout bit clear reset a latched frame timeout.
                Command::stop(id, false)
            } else if track.tripped {
                Command::stop(id, true)
            } else {
                let foc = |position: f32, kp: f32| Command {
                    id,
                    mode: Mode::Foc,
                    torque: 0.0,
                    speed: 0.0,
                    position,
                    kp,
                    kd: KD,
                    timeout: true,
                };
                match (ctl.mode, track.origin) {
                    (TestMode::Damp, _) => foc(0.0, 0.0),
                    (TestMode::Hold, Some(p)) => foc(p, KP),
                    (TestMode::Wiggle, Some(p)) => foc(
                        p + WIGGLE_AMPLITUDE * libm::sinf(2.0 * core::f32::consts::PI * WIGGLE_HZ * t),
                        KP,
                    ),
                    (TestMode::ClearFaults, _) if now - mode_start < PERIOD * 2 => Command {
                        mode: Mode::ClearFaults,
                        ..Command::stop(id, true)
                    },
                    // Hold and Wiggle wait for a reply to anchor on.
                    _ => Command::stop(id, true),
                }
            };

            let t0 = Instant::now();
            stats.sent += 1;
            let (outcome, echo_ok) = exchange(uart, raw, &ctl, &cmd).await;
            if !echo_ok {
                stats.echo_mismatch += 1;
            }
            match outcome {
                Outcome::NoReply(_) => stats.no_reply += 1,
                Outcome::Uart(e) => {
                    stats.uart_errors += 1;
                    match e {
                        usart::Error::Noise => stats.noise += 1,
                        usart::Error::Framing => stats.framing += 1,
                        usart::Error::Overrun => stats.overrun += 1,
                        _ => {}
                    }
                }
                Outcome::BadFrame => stats.bad_frames += 1,
                Outcome::Reply(state) => {
                    let rt = (Instant::now() - t0).as_micros() as u32;
                    stats.replies += 1;
                    stats.round_trip_us = rt;
                    stats.round_trip_max_us = stats.round_trip_max_us.max(rt);
                    stats.last = Some(state);
                    track.last_reply = now;
                    if track.origin.is_none() {
                        track.origin = Some(state.position);
                    }
                    if !track.tripped
                        && matches!(ctl.mode, TestMode::Hold | TestMode::Wiggle)
                        && state.speed.abs() > SPEED_LIMIT
                    {
                        track.tripped = true;
                        stats.speed_trips += 1;
                    }
                }
            }
        }
        shared.set(seg);
    }
}

// USART and LPUART register offsets and ISR bits (RM0440): the same in both.
const CR1: usize = 0x00;
const CR3: usize = 0x08;
const ISR: usize = 0x1C;
const ICR: usize = 0x20;
const RDR: usize = 0x24;
const TDR: usize = 0x28;
const ISR_FE: u32 = 1 << 1;
const ISR_NE: u32 = 1 << 2;
const ISR_ORE: u32 = 1 << 3;
const ISR_RXNE: u32 = 1 << 5;
const ISR_TC: u32 = 1 << 6;
const ISR_TXE: u32 = 1 << 7;
const ICR_TCCF: u32 = 1 << 6;
/// GPIO output type register: a set bit is open drain.
const GPIO_OTYPER: usize = 0x04;
/// Polling iterations with nothing received before giving up, about 250 us.
const SPIN_LIMIT: u32 = 2000;

/// A segment's USART or LPUART by base address, and its data pin's GPIO port
/// base and pin number, for the polled loop.
#[derive(Clone, Copy)]
pub struct Raw {
    pub uart: usize,
    pub gpio: usize,
    pub pin: u8,
}

impl Raw {
    fn reg(self, offset: usize) -> *mut u32 {
        (self.uart + offset) as *mut u32
    }

    /// Switches the data pin between push-pull and open drain.
    unsafe fn set_open_drain(self, open_drain: bool) {
        let otyper = (self.gpio + GPIO_OTYPER) as *mut u32;
        unsafe {
            let v = otyper.read_volatile();
            otyper.write_volatile(if open_drain { v | 1 << self.pin } else { v & !(1 << self.pin) });
        }
    }

    /// Sends `frame` and collects the echo and reply into `buf` by polling the
    /// registers with interrupts off. With `active_drive` the pin is push-pull
    /// from the first byte until transmission completes, then open drain again
    /// for the reply. Returns the bytes received and any error flags seen.
    fn transact(self, frame: &[u8], buf: &mut [u8], active_drive: bool) -> (usize, u32) {
        critical_section::with(|_| unsafe {
            let (cr1, cr3, isr, icr) = (self.reg(CR1), self.reg(CR3), self.reg(ISR), self.reg(ICR));
            let (rdr, tdr) = (self.reg(RDR), self.reg(TDR));
            // No DMA requests and no error interrupt; transmitter and receiver on.
            cr3.write_volatile(cr3.read_volatile() & !((1 << 7) | (1 << 6) | 1));
            cr1.write_volatile(cr1.read_volatile() | (1 << 3) | (1 << 2));
            icr.write_volatile(0b1111 | ICR_TCCF);
            let _ = rdr.read_volatile();
            if active_drive {
                self.set_open_drain(false);
            }
            let mut driving = active_drive;

            let (mut received, mut sent, mut spins, mut flags) = (0usize, 0usize, 0u32, 0u32);
            while received < buf.len() && spins < SPIN_LIMIT {
                let status = isr.read_volatile();
                // Release the line as soon as the last stop bit is out: the servo
                // starts its reply tens of microseconds later.
                if driving && sent == frame.len() && status & ISR_TC != 0 {
                    self.set_open_drain(true);
                    driving = false;
                }
                if status & (ISR_FE | ISR_NE | ISR_ORE) != 0 {
                    flags |= status & (ISR_FE | ISR_NE | ISR_ORE);
                    icr.write_volatile(0b1110);
                }
                if status & ISR_RXNE != 0 {
                    buf[received] = rdr.read_volatile() as u8;
                    received += 1;
                    spins = 0;
                } else {
                    spins += 1;
                }
                if sent < frame.len() && status & ISR_TXE != 0 {
                    tdr.write_volatile(frame[sent] as u32);
                    sent += 1;
                }
            }
            if driving {
                self.set_open_drain(true);
            }
            (received, flags)
        })
    }
}
