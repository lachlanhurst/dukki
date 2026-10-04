//! One servo bus segment: a single-wire half-duplex UART at 6 Mbps, polled at
//! 500 Hz. For now each segment drives one servo through a bench test mode
//! chosen over the USB console.

use core::cell::Cell;

use embassy_stm32::mode::Async;
use embassy_stm32::usart::{self, Uart};
use embassy_futures::join::join;
use embassy_futures::yield_now;
use embassy_time::{with_timeout, Duration, Instant, Ticker};

use crate::j288::{self, Command, Mode, State};

/// Bench test modes, the same for every segment.
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

/// The console's choice, with a counter so segments notice a re-selection.
#[derive(Clone, Copy)]
pub struct Control {
    pub mode: TestMode,
    pub generation: u32,
    /// Run each transaction as a polled register loop with interrupts off (the
    /// default), or through embassy's DMA read and write. On the bench the DMA
    /// receive overran on about 98% of transactions on all three UARTs while the
    /// polled loop missed nothing; the DMA path stays selectable to investigate.
    pub polled: bool,
}

impl Default for Control {
    fn default() -> Self {
        Self { mode: TestMode::Stop, generation: 0, polled: true }
    }
}

// Gains from the bench tools (scripts/j288/explore.py position test): gentle
// enough for a loose servo on the bench.
const KP: f32 = 0.5; // N.m/rad, output side
const KD: f32 = 0.05; // N.m.s/rad
const WIGGLE_AMPLITUDE: f32 = 0.3; // rad
const WIGGLE_HZ: f32 = 0.5;
/// Above this the segment drops to Stop until a mode is chosen again.
const SPEED_LIMIT: f32 = 10.0; // rad/s
const PERIOD: Duration = Duration::from_micros(2000);
/// Command out (33 us at 6 Mbps) and reply back (43 us) normally take about 120 us.
const TRANSACTION_TIMEOUT: Duration = Duration::from_micros(1000);
/// Without a valid reply for this long, assume the servo lost power or latched
/// its frame timeout, and clear the latch before driving it again.
const RECOVER_AFTER: Duration = Duration::from_millis(200);
const RECOVER_FOR: Duration = Duration::from_millis(500);

#[derive(Clone, Copy, Default)]
pub struct Stats {
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

pub async fn run(
    uart: &mut Uart<'_, Async>,
    id: u8,
    control: &Cell<Control>,
    shared: &Cell<Stats>,
    raw: Raw,
) {
    let mut ticker = Ticker::every(PERIOD);
    let mut stats = Stats::default();
    let mut seen_generation = u32::MAX;
    let mut origin: Option<f32> = None;
    let mut mode_start = Instant::now();
    let mut tripped = false;
    let mut last_reply = Instant::MIN;
    let mut recover_until = Instant::now() + RECOVER_FOR;

    loop {
        ticker.next().await;
        let now = Instant::now();
        let ctl = control.get();
        if ctl.generation != seen_generation {
            seen_generation = ctl.generation;
            origin = stats.last.map(|s| s.position);
            mode_start = now;
            tripped = false;
        }
        if now - last_reply > RECOVER_AFTER && now >= recover_until {
            recover_until = now + RECOVER_FOR;
        }
        stats.recovering = now < recover_until;

        let cmd = if stats.recovering {
            // Stop frames with the timeout bit clear reset a latched frame timeout.
            Command::stop(id, false)
        } else if tripped {
            Command::stop(id, true)
        } else {
            let t = (now - mode_start).as_micros() as f32 * 1e-6;
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
            match (ctl.mode, origin) {
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

        let frame = j288::encode(&cmd);
        // The receiver stays on while we transmit (readback), and one read takes our
        // own 20-byte echo followed by the servo's 26-byte reply. The read is armed
        // before the first byte goes out, so there is no gap at the turnaround for
        // the reply to fall into.
        let mut rx_buf = [0u8; j288::COMMAND_LEN + j288::REPLY_LEN];
        let t0 = Instant::now();
        stats.sent += 1;
        let result = if ctl.polled {
            let (n, flags) = raw.transact(&frame, &mut rx_buf);
            if flags & ISR_ORE != 0 {
                Ok(Err(usart::Error::Overrun))
            } else if flags & ISR_FE != 0 {
                Ok(Err(usart::Error::Framing))
            } else if flags & ISR_NE != 0 {
                Ok(Err(usart::Error::Noise))
            } else if n < rx_buf.len() {
                Err(())
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
            .map_err(|_| ())
        };
        let echo_ok = rx_buf[..j288::COMMAND_LEN] == frame;
        if !echo_ok {
            stats.echo_mismatch += 1;
        }
        let reply: &[u8; j288::REPLY_LEN] = rx_buf[j288::COMMAND_LEN..].try_into().unwrap();
        match result {
            Err(_) => stats.no_reply += 1,
            Ok(Err(e)) => {
                stats.uart_errors += 1;
                match e {
                    usart::Error::Noise => stats.noise += 1,
                    usart::Error::Framing => stats.framing += 1,
                    usart::Error::Overrun => stats.overrun += 1,
                    _ => {}
                }
            }
            Ok(Ok(())) => match j288::decode(reply) {
                Ok(state) => {
                    let rt = (Instant::now() - t0).as_micros() as u32;
                    stats.replies += 1;
                    stats.round_trip_us = rt;
                    stats.round_trip_max_us = stats.round_trip_max_us.max(rt);
                    stats.last = Some(state);
                    last_reply = now;
                    if origin.is_none() {
                        origin = Some(state.position);
                    }
                    if !tripped
                        && matches!(ctl.mode, TestMode::Hold | TestMode::Wiggle)
                        && state.speed.abs() > SPEED_LIMIT
                    {
                        tripped = true;
                        stats.speed_trips += 1;
                    }
                }
                Err(_) => stats.bad_frames += 1,
            },
        }
        shared.set(stats);
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
const ISR_TXE: u32 = 1 << 7;
/// Polling iterations with nothing received before giving up, about 250 us.
const SPIN_LIMIT: u32 = 2000;

/// A segment's USART or LPUART by base address, for the polled diagnostic.
#[derive(Clone, Copy)]
pub struct Raw(pub usize);

impl Raw {
    fn reg(self, offset: usize) -> *mut u32 {
        (self.0 + offset) as *mut u32
    }

    /// Sends `frame` and collects the echo and reply into `buf` by polling the
    /// registers with interrupts off. Returns the bytes received and any error
    /// flags seen.
    fn transact(self, frame: &[u8], buf: &mut [u8]) -> (usize, u32) {
        critical_section::with(|_| unsafe {
            let (cr1, cr3, isr, icr) = (self.reg(CR1), self.reg(CR3), self.reg(ISR), self.reg(ICR));
            let (rdr, tdr) = (self.reg(RDR), self.reg(TDR));
            // No DMA requests and no error interrupt; transmitter and receiver on.
            cr3.write_volatile(cr3.read_volatile() & !((1 << 7) | (1 << 6) | 1));
            cr1.write_volatile(cr1.read_volatile() | (1 << 3) | (1 << 2));
            icr.write_volatile(0b1111);
            let _ = rdr.read_volatile();

            let (mut received, mut sent, mut spins, mut flags) = (0usize, 0usize, 0u32, 0u32);
            while received < buf.len() && spins < SPIN_LIMIT {
                let status = isr.read_volatile();
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
            (received, flags)
        })
    }
}
