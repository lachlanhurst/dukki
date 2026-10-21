//! The servo bus segments: single-wire half-duplex UARTs at 6 Mbps, served
//! together by one loop (`run`). Each segment scans IDs 0 to 14 for servos that
//! answer, then polls every servo it found once per 2 ms tick, and at once when
//! a command arrives. The segments' exchanges run side by side, so a round takes
//! as long as the busiest segment's alone. Each servo follows the compute
//! module's latest command (`link.rs`); before any command has ever arrived it
//! follows the bench test mode chosen over the USB console instead, and once
//! commands stop for the watchdog period every servo is stopped.

use core::cell::Cell;

use cortex_m::peripheral::DWT;
use embassy_stm32::mode::Async;
use embassy_stm32::usart::{self, Uart};
use embassy_futures::join::join;
use embassy_futures::select::{select, Either};
use embassy_futures::yield_now;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{with_timeout, Duration, Instant, Ticker};

use duck_bridge_proto::{self as proto, Action, ServoMode};

use crate::j288::{self, Command, Mode, State};
use crate::link::{HostCommand, WATCHDOG};

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
    /// Start a round as soon as a command arrives (the default), or wait for the next tick.
    pub immediate: bool,
    /// Bumped by the console to zero the segments' maxima and the servos' counters.
    pub stats_reset: u32,
    /// How the polled loop runs the lines in a slot.
    pub overlap: Overlap,
}

/// How the polled loop runs the segments' exchanges in one slot.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Overlap {
    /// All lines start together.
    Parallel,
    /// Each line starts [`STAGGER_US`] after the one before, so no two transmit at once.
    Staggered,
    /// One line after another, as before the lines were served together.
    Serial,
}

/// A command frame takes 33 µs at 6 Mbps; this keeps the lines' transmissions apart.
pub const STAGGER_US: u32 = 40;

impl Default for Control {
    fn default() -> Self {
        Self { mode: TestMode::Stop, generation: 0, rescan: 0, polled: true, active_drive: true, immediate: true, stats_reset: 0, overlap: Overlap::Parallel }
    }
}

/// Segments the bridge drives: A, B and C on this robot, and D on UART4 (PC10) for one that
/// needs a fourth line. A segment with nothing on it rescans every ID once a second: 45 short
/// exchanges, each yielding to the other tasks, and nothing between scans.
pub const SEGMENTS: usize = 4;

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
/// Replies kept per servo for the velocity estimate: about 15 ms at the bridge's
/// polling rate, matching the servo's own filter so the two speeds are comparable.
const VELOCITY_WINDOW: usize = 8;
/// A gap in replies longer than this restarts the velocity estimate rather than
/// differencing across it.
const VELOCITY_GAP_US: u64 = 50_000;
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
    /// Bytes received on the last exchange without a full reply: 20 is the echo alone.
    pub no_reply_bytes: u8,
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
    /// Bridge clock at the last good reply, µs.
    pub last_reply_us: u64,
    /// From position differences over [`VELOCITY_WINDOW`] replies, rad/s.
    pub velocity: f32,
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
    /// Passes over the segment's servos, and the last one's duration in µs.
    pub rounds: u32,
    pub round_us: u32,
    /// From a host command arriving to its first exchange on this segment, and to the exchange
    /// that delivered it to the segment's last servo, µs: the last command's, and the most seen
    /// since the console last reset them.
    pub apply_first_us: u32,
    pub apply_last_us: u32,
    pub apply_first_max_us: u32,
    pub apply_last_max_us: u32,
}

/// Per-servo control state that only the segment loop needs.
#[derive(Clone, Copy)]
struct Track {
    origin: Option<f32>,
    tripped: bool,
    last_reply: Instant,
    recover_until: Instant,
    /// The last [`proto::ServoCommand::action_id`] acted on. 0 never triggers an
    /// action, so a compute module counts its actions from 1.
    action_id: u8,
    /// The sequence number of the last host command sent to this servo.
    carried: Option<u32>,
    /// Recent (bridge µs, position) replies, oldest first once full.
    samples: [(u64, f32); VELOCITY_WINDOW],
    sample_count: usize,
    sample_head: usize,
}

impl Track {
    const fn new() -> Self {
        Self {
            origin: None,
            tripped: false,
            last_reply: Instant::MIN,
            recover_until: Instant::MIN,
            action_id: 0,
            carried: None,
            samples: [(0, 0.0); VELOCITY_WINDOW],
            sample_count: 0,
            sample_head: 0,
        }
    }

    /// Records a reply and returns the velocity across the window.
    fn velocity(&mut self, t_us: u64, position: f32) -> f32 {
        if self.sample_count > 0 {
            let newest = self.samples[(self.sample_head + VELOCITY_WINDOW - 1) % VELOCITY_WINDOW];
            if t_us.saturating_sub(newest.0) > VELOCITY_GAP_US {
                self.sample_count = 0;
            }
        }
        self.samples[self.sample_head] = (t_us, position);
        self.sample_head = (self.sample_head + 1) % VELOCITY_WINDOW;
        self.sample_count = (self.sample_count + 1).min(VELOCITY_WINDOW);
        if self.sample_count < 2 {
            return 0.0;
        }
        let oldest = self.samples[(self.sample_head + VELOCITY_WINDOW - self.sample_count) % VELOCITY_WINDOW];
        let dt = t_us.saturating_sub(oldest.0) as f32 * 1e-6;
        if dt > 0.0 { (position - oldest.1) / dt } else { 0.0 }
    }
}

enum Outcome {
    Reply(State),
    /// Fewer bytes than echo and reply arrived (polled path: how many did).
    NoReply(usize),
    Uart(usart::Error),
    BadFrame,
}

/// Echo and reply, as one read.
const RX_LEN: usize = j288::COMMAND_LEN + j288::REPLY_LEN;

/// What an exchange's bytes amount to, and whether the echo matched. `result` is the transfer's:
/// `Err(n)` when only `n` bytes arrived, `Ok(Err(e))` for a UART error.
fn outcome(
    frame: &[u8; j288::COMMAND_LEN],
    rx_buf: &[u8; RX_LEN],
    result: Result<Result<(), usart::Error>, usize>,
    id: u8,
) -> (Outcome, bool) {
    let echo_ok = rx_buf[..j288::COMMAND_LEN] == *frame;
    let reply: &[u8; j288::REPLY_LEN] = rx_buf[j288::COMMAND_LEN..].try_into().unwrap();
    let outcome = match result {
        Err(n) => Outcome::NoReply(n),
        Ok(Err(e)) => Outcome::Uart(e),
        // A reply from another ID would also be a bad frame here.
        Ok(Ok(())) => match j288::decode(reply) {
            Ok(state) if state.id == id => Outcome::Reply(state),
            _ => Outcome::BadFrame,
        },
    };
    (outcome, echo_ok)
}

/// A polled transfer's result from its byte count and error flags.
fn polled_result(n: usize, flags: u32) -> Result<Result<(), usart::Error>, usize> {
    if flags & ISR_ORE != 0 {
        Ok(Err(usart::Error::Overrun))
    } else if flags & ISR_FE != 0 {
        Ok(Err(usart::Error::Framing))
    } else if flags & ISR_NE != 0 {
        Ok(Err(usart::Error::Noise))
    } else if n < RX_LEN {
        Err(n)
    } else {
        Ok(Ok(()))
    }
}

/// One exchange through embassy's DMA read and write, the console's alternative to the polled
/// loop. The receiver stays on while we transmit (readback), and one read takes our own 20-byte
/// echo followed by the servo's 26-byte reply, so there is no gap at the turnaround for the
/// reply to fall into.
async fn exchange_dma(uart: &mut Uart<'_, Async>, frame: &[u8; j288::COMMAND_LEN], id: u8) -> (Outcome, bool) {
    let mut rx_buf = [0u8; RX_LEN];
    let (tx, rx) = uart.split_ref();
    let result = with_timeout(
        TRANSACTION_TIMEOUT,
        join(rx.read(&mut rx_buf), async {
            yield_now().await;
            tx.write(frame).await
        }),
    )
    .await
    .map(|(read, write)| write.and(read))
    .map_err(|_| 0);
    outcome(frame, &rx_buf, result, id)
}

/// Whose commands the servos follow.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Driver {
    /// No command has arrived since the bridge started: the bench console's test mode.
    Console,
    /// The compute module, which commanded within the watchdog period.
    Host,
    /// The compute module stopped commanding: every servo stopped.
    Watchdog,
}

/// Raised by the link when a command arrives, so the segments apply it at once rather than on
/// their next tick.
pub type CommandArrived = Signal<NoopRawMutex, ()>;

/// A segment's line: its UART for the DMA path, its registers and pin for the polled loop, and
/// where it publishes what it found.
pub struct Line<'a, 'd> {
    pub uart: &'a mut Uart<'d, Async>,
    pub raw: Raw,
    pub shared: &'a Cell<SegmentStats>,
}

/// How far a segment's scan has got: the ID being asked, how often, and whether it gave only
/// corrupt replies so far.
#[derive(Clone, Copy)]
struct Scan {
    id: u8,
    tries: u32,
    corrupt: bool,
}

/// Everything a segment keeps between rounds.
struct Seg {
    stats: SegmentStats,
    tracks: [Track; MAX_SERVOS],
    /// Scanning: one ID is asked per slot, in place of polling.
    scan: Option<Scan>,
    last_scan: Instant,
    /// The newest host command, when it arrived, and how many of the segment's servos it has
    /// reached.
    delivering: Option<(u32, u64, usize)>,
}

impl Seg {
    fn new() -> Self {
        Self {
            stats: SegmentStats::default(),
            tracks: [Track::new(); MAX_SERVOS],
            scan: None,
            last_scan: Instant::MIN,
            delivering: None,
        }
    }

    /// Asks every ID with stop frames (timeout bit clear) and records which answer, a slot at a
    /// time.
    fn begin_scan(&mut self) {
        let s = &mut self.stats;
        s.count = 0;
        s.garbled = 0;
        s.scan_frames = 0;
        s.scan_echo_bad = 0;
        s.scan_bytes_min = usize::MAX;
        s.scan_bytes_max = 0;
        s.scanning = true;
        self.scan = Some(Scan { id: 0, tries: 0, corrupt: false });
    }

    /// Records one scan frame's outcome and moves to the next try or ID.
    fn scan_step(&mut self, outcome: Outcome, echo_ok: bool) {
        let Some(mut sc) = self.scan else { return };
        let s = &mut self.stats;
        s.scan_frames += 1;
        if !echo_ok {
            s.scan_echo_bad += 1;
        }
        let bytes = match outcome {
            Outcome::NoReply(n) => n,
            _ => RX_LEN,
        };
        s.scan_bytes_min = s.scan_bytes_min.min(bytes);
        s.scan_bytes_max = s.scan_bytes_max.max(bytes);
        sc.tries += 1;
        let next = match outcome {
            Outcome::Reply(state) => {
                s.servos[s.count] = Stats { id: sc.id, last: Some(state), ..Stats::default() };
                s.count += 1;
                sc.corrupt = false;
                true
            }
            Outcome::BadFrame | Outcome::Uart(_) => {
                sc.corrupt = true;
                sc.tries >= SCAN_TRIES
            }
            Outcome::NoReply(_) => sc.tries >= SCAN_TRIES,
        };
        if next {
            if sc.corrupt {
                s.garbled |= 1 << sc.id;
            }
            sc = Scan { id: sc.id + 1, tries: 0, corrupt: false };
        }
        if (sc.id as usize) < MAX_SERVOS {
            self.scan = Some(sc);
            return;
        }
        self.scan = None;
        s.scans += 1;
        s.scanning = false;
        let now = Instant::now();
        self.last_scan = now;
        for (track, stats) in self.tracks.iter_mut().zip(s.servos.iter()).take(s.count) {
            *track = Track {
                origin: stats.last.map(|s| s.position),
                last_reply: now,
                recover_until: now + RECOVER_FOR,
                ..Track::new()
            };
        }
    }
}

/// What one servo is told this slot.
fn servo_command(
    ctl: &Control,
    driver: Driver,
    host_cmd: &HostCommand,
    track: &mut Track,
    recovering: bool,
    id: u8,
    since_mode: Duration,
) -> Command {
    if recovering {
        // Stop frames with the timeout bit clear reset a latched frame timeout.
        return Command::stop(id, false);
    }
    match driver {
        Driver::Watchdog => Command::stop(id, true),
        Driver::Host => {
            let sc = host_cmd.command.servos[id as usize];
            let action = if sc.action_id != track.action_id {
                track.action_id = sc.action_id;
                sc.action
            } else {
                Action::None
            };
            match (action, sc.mode) {
                (Action::ClearFaults, _) => Command { mode: Mode::ClearFaults, ..Command::stop(id, true) },
                (Action::Reset, _) => Command { mode: Mode::Reset, ..Command::stop(id, true) },
                (Action::None, ServoMode::Stop) => Command::stop(id, true),
                (Action::None, ServoMode::Foc) => Command {
                    id,
                    mode: Mode::Foc,
                    torque: sc.torque,
                    speed: sc.velocity,
                    position: sc.position,
                    kp: sc.kp,
                    kd: sc.kd,
                    timeout: true,
                },
            }
        }
        Driver::Console if track.tripped => Command::stop(id, true),
        Driver::Console => {
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
            let t = since_mode.as_micros() as f32 * 1e-6;
            match (ctl.mode, track.origin) {
                (TestMode::Damp, _) => foc(0.0, 0.0),
                (TestMode::Hold, Some(p)) => foc(p, KP),
                (TestMode::Wiggle, Some(p)) => foc(
                    p + WIGGLE_AMPLITUDE * libm::sinf(2.0 * core::f32::consts::PI * WIGGLE_HZ * t),
                    KP,
                ),
                (TestMode::ClearFaults, _) if since_mode < PERIOD * 2 => Command {
                    mode: Mode::ClearFaults,
                    ..Command::stop(id, true)
                },
                // Hold and Wiggle wait for a reply to anchor on.
                _ => Command::stop(id, true),
            }
        }
    }
}

/// Every segment, served together. A round is a run of slots; in each slot every segment has one
/// exchange, with the next servo it polls or, while it scans, the next ID it asks, and on the
/// polled path the lines run side by side in one interrupts-off loop ([`transact_all`]). So a
/// round over five servos a segment takes five exchange times, not fifteen. A round starts on
/// the 2 ms tick, or at once when a command arrives; every slot takes the latest command.
pub async fn run<const N: usize>(
    mut lines: [Line<'_, '_>; N],
    control: &Cell<Control>,
    host: &Cell<HostCommand>,
    arrived: &CommandArrived,
) {
    // The polled loop's timeouts count CPU cycles.
    let mut cp = unsafe { cortex_m::Peripherals::steal() };
    cp.DCB.enable_trace();
    cp.DWT.enable_cycle_counter();

    let mut segs: [Seg; N] = core::array::from_fn(|_| {
        let mut s = Seg::new();
        s.begin_scan();
        s
    });
    let mut seen_generation = u32::MAX;
    let mut seen_rescan = control.get().rescan;
    let mut seen_stats_reset = control.get().stats_reset;
    let mut mode_start = Instant::now();
    let mut ticker = Ticker::every(PERIOD);

    loop {
        let ctl = control.get();
        if ctl.rescan != seen_rescan {
            seen_rescan = ctl.rescan;
            segs.iter_mut().for_each(Seg::begin_scan);
        }
        for s in &mut segs {
            if s.scan.is_none() && s.stats.count == 0 && Instant::now() - s.last_scan > EMPTY_RESCAN {
                s.begin_scan();
            }
        }
        for (s, line) in segs.iter().zip(lines.iter()) {
            line.shared.set(s.stats);
        }

        // With servos to poll, wait for the tick or a command; a scan alone runs flat out.
        if segs.iter().any(|s| s.scan.is_none() && s.stats.count > 0) {
            // A new command starts a round at once; the ticker then counts its 2 ms from there,
            // so routine rounds do not crowd in behind it.
            if ctl.immediate {
                if let Either::Second(()) = select(ticker.next(), arrived.wait()).await {
                    ticker.reset();
                }
            } else {
                ticker.next().await;
                arrived.reset();
            }
        }
        if ctl.stats_reset != seen_stats_reset {
            seen_stats_reset = ctl.stats_reset;
            for s in &mut segs {
                s.stats.apply_first_max_us = 0;
                s.stats.apply_last_max_us = 0;
                for stats in &mut s.stats.servos {
                    *stats = Stats { id: stats.id, last: stats.last, last_reply_us: stats.last_reply_us, velocity: stats.velocity, ..Stats::default() };
                }
            }
        }
        let round_start = Instant::now();
        let new_mode = ctl.generation != seen_generation;
        if new_mode {
            seen_generation = ctl.generation;
            mode_start = round_start;
            for s in &mut segs {
                for (track, stats) in s.tracks.iter_mut().zip(s.stats.servos.iter()).take(s.stats.count) {
                    track.origin = stats.last.map(|s| s.position);
                    track.tripped = false;
                }
            }
        }
        let slots = segs
            .iter()
            .map(|s| if s.scan.is_some() { 1 } else { s.stats.count })
            .max()
            .unwrap_or(0);

        for slot in 0..slots {
            // The latest command for every slot: one arriving mid-round reaches the next servo
            // polled on each segment rather than waiting for the next round.
            let host_cmd = host.get();
            let now = Instant::now();
            let driver = match host_cmd.received_us {
                None => Driver::Console,
                Some(at) if now.as_micros().saturating_sub(at) <= WATCHDOG.as_micros() => Driver::Host,
                Some(_) => Driver::Watchdog,
            };

            // Each line's frame, and which polled servo it is for.
            let mut frames = [[0u8; j288::COMMAND_LEN]; N];
            let mut ids = [0u8; N];
            let mut active = [false; N];
            let mut polled: [Option<usize>; N] = [None; N];
            let mut delivered = [false; N];
            for (i, s) in segs.iter_mut().enumerate() {
                let cmd = if let Some(sc) = s.scan {
                    Command::stop(sc.id, false)
                } else if slot < s.stats.count {
                    let (stats, track) = (&mut s.stats.servos[slot], &mut s.tracks[slot]);
                    if now - track.last_reply > RECOVER_AFTER && now >= track.recover_until {
                        track.recover_until = now + RECOVER_FOR;
                    }
                    stats.recovering = now < track.recover_until;
                    let cmd = servo_command(&ctl, driver, &host_cmd, track, stats.recovering, stats.id, now - mode_start);
                    // Delivery of a new host command: when it reached the segment's first
                    // servo and, once the slot is done, its last.
                    if let (Driver::Host, Some(at)) = (driver, host_cmd.received_us) {
                        let seq = host_cmd.command.seq;
                        if track.carried != Some(seq) {
                            track.carried = Some(seq);
                            let reached = match s.delivering {
                                Some((d, _, n)) if d == seq => n + 1,
                                _ => {
                                    s.stats.apply_first_us = now.as_micros().saturating_sub(at) as u32;
                                    s.stats.apply_first_max_us = s.stats.apply_first_max_us.max(s.stats.apply_first_us);
                                    1
                                }
                            };
                            s.delivering = Some((seq, at, reached));
                            delivered[i] = reached == s.stats.count;
                        }
                    }
                    s.stats.servos[slot].sent += 1;
                    polled[i] = Some(slot);
                    cmd
                } else {
                    continue;
                };
                frames[i] = j288::encode(&cmd);
                ids[i] = cmd.id;
                active[i] = true;
            }

            // The exchanges.
            let mut results: [Option<(Outcome, bool)>; N] = core::array::from_fn(|_| None);
            let t0 = Instant::now();
            if ctl.polled {
                let raws: [Raw; N] = core::array::from_fn(|i| lines[i].raw);
                let mut bufs = [[0u8; RX_LEN]; N];
                let done = match ctl.overlap {
                    Overlap::Parallel => transact_all(&raws, &active, &frames, &mut bufs, ctl.active_drive, 0),
                    Overlap::Staggered => {
                        transact_all(&raws, &active, &frames, &mut bufs, ctl.active_drive, STAGGER_US * CYCLES_PER_US)
                    }
                    Overlap::Serial => {
                        let mut done = [(0, 0); N];
                        for i in 0..N {
                            let only: [bool; N] = core::array::from_fn(|j| j == i && active[j]);
                            if only[i] {
                                done[i] = transact_all(&raws, &only, &frames, &mut bufs, ctl.active_drive, 0)[i];
                            }
                        }
                        done
                    }
                };
                for i in 0..N {
                    if active[i] {
                        let (n, flags) = done[i];
                        results[i] = Some(outcome(&frames[i], &bufs[i], polled_result(n, flags), ids[i]));
                    }
                }
            } else {
                for i in 0..N {
                    if active[i] {
                        results[i] = Some(exchange_dma(lines[i].uart, &frames[i], ids[i]).await);
                    }
                }
            }
            let rt = (Instant::now() - t0).as_micros() as u32;
            let replied_us = Instant::now().as_micros();

            for (i, (s, result)) in segs.iter_mut().zip(results).enumerate() {
                let Some((outcome, echo_ok)) = result else { continue };
                let Some(k) = polled[i] else {
                    s.scan_step(outcome, echo_ok);
                    continue;
                };
                let (stats, track) = (&mut s.stats.servos[k], &mut s.tracks[k]);
                if !echo_ok {
                    stats.echo_mismatch += 1;
                }
                match outcome {
                    Outcome::NoReply(n) => {
                        stats.no_reply += 1;
                        stats.no_reply_bytes = n as u8;
                    }
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
                        stats.replies += 1;
                        stats.round_trip_us = rt;
                        stats.round_trip_max_us = stats.round_trip_max_us.max(rt);
                        stats.last = Some(state);
                        stats.last_reply_us = replied_us;
                        stats.velocity = track.velocity(replied_us, state.position);
                        track.last_reply = now;
                        if track.origin.is_none() {
                            track.origin = Some(state.position);
                        }
                        if !track.tripped
                            && driver == Driver::Console
                            && matches!(ctl.mode, TestMode::Hold | TestMode::Wiggle)
                            && state.speed.abs() > SPEED_LIMIT
                        {
                            track.tripped = true;
                            stats.speed_trips += 1;
                        }
                    }
                }
                if let (true, Some((_, at, _))) = (delivered[i], s.delivering) {
                    s.stats.apply_last_us = replied_us.saturating_sub(at) as u32;
                    s.stats.apply_last_max_us = s.stats.apply_last_max_us.max(s.stats.apply_last_us);
                }
            }
            // The polled loop never awaits, so without this a round would hold off the link
            // task, and a state request with it, for its whole length.
            yield_now().await;
        }

        let round_us = (Instant::now() - round_start).as_micros() as u32;
        for (s, line) in segs.iter_mut().zip(lines.iter_mut()) {
            if s.scan.is_none() && s.stats.count > 0 {
                s.stats.rounds = s.stats.rounds.wrapping_add(1);
                s.stats.round_us = round_us;
            }
            line.shared.set(s.stats);
        }
    }
}

/// The protocol's view of one servo, from what the segment holds.
pub fn servo_state(segment: u8, stats: &Stats, now_us: u64) -> proto::ServoState {
    let mut s = proto::ServoState {
        segment,
        replies: stats.replies as u16,
        failures: (stats.sent - stats.replies) as u16,
        velocity: stats.velocity,
        age_us: now_us.saturating_sub(stats.last_reply_us).min(u16::MAX as u64) as u16,
        ..Default::default()
    };
    if stats.recovering {
        s.flags |= proto::servo_flags::RECOVERING;
    }
    if let Some(st) = stats.last {
        s.mode = st.mode;
        if st.timeout {
            s.flags |= proto::servo_flags::TIMEOUT_LATCHED;
        }
        s.temp_c = st.temp;
        s.winding_c = st.winding_temp;
        s.volts_half = (st.volts * 2.0) as u8;
        s.warning = st.warning;
        s.error = st.error;
        s.position = st.position;
        s.speed = st.speed;
        s.torque = st.torque;
        s.output_encoder = st.output_encoder_raw;
    } else {
        s.age_us = u16::MAX;
    }
    s
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
/// CPU cycles per µs, for the polled loop's timeouts.
const CYCLES_PER_US: u32 = 168;
/// A polled line is done once nothing has moved on it for this long after its frame went out.
/// A servo starts its reply some tens of µs after the frame.
const IDLE_CYCLES: u32 = 150 * CYCLES_PER_US;
/// And in any case after this long: 46 bytes at 6 Mbps take 77 µs.
const CAP_CYCLES: u32 = 500 * CYCLES_PER_US;

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

    /// Turns the UART's 8-byte FIFOs on, with the UART disabled while it changes (RM0440).
    pub fn enable_fifo(self) {
        const UE: u32 = 1;
        const FIFOEN: u32 = 1 << 29;
        unsafe {
            let cr1 = self.reg(CR1);
            let v = cr1.read_volatile();
            cr1.write_volatile(v & !UE);
            cr1.write_volatile((v & !UE) | FIFOEN);
            cr1.write_volatile(v | FIFOEN);
        }
    }

    /// Whether the UART's FIFOs are on.
    pub fn fifo_enabled(self) -> bool {
        unsafe { self.reg(CR1).read_volatile() & (1 << 29) != 0 }
    }

    /// Switches the data pin between push-pull and open drain.
    unsafe fn set_open_drain(self, open_drain: bool) {
        let otyper = (self.gpio + GPIO_OTYPER) as *mut u32;
        unsafe {
            let v = otyper.read_volatile();
            otyper.write_volatile(if open_drain { v | 1 << self.pin } else { v & !(1 << self.pin) });
        }
    }
}

/// A line in [`transact_all`].
#[derive(Clone, Copy, Default)]
struct LineProgress {
    sent: usize,
    received: usize,
    flags: u32,
    driving: bool,
    done: bool,
    /// Cycle count at the last byte in or out, or the end of transmission.
    last: u32,
}

/// Sends each active line's frame and collects its echo and reply, all lines side by side in one
/// loop polling the registers with interrupts off. With `active_drive` each pin is push-pull from
/// its first byte until its transmission completes, then open drain again for the reply. A line
/// is done when its buffer is full, when nothing has moved on it for [`IDLE_CYCLES`] since its
/// frame went out, or after [`CAP_CYCLES`]. Returns each line's bytes received and error flags.
fn transact_all<const N: usize>(
    raws: &[Raw; N],
    active: &[bool; N],
    frames: &[[u8; j288::COMMAND_LEN]; N],
    bufs: &mut [[u8; RX_LEN]; N],
    active_drive: bool,
    stagger: u32,
) -> [(usize, u32); N] {
    critical_section::with(|_| unsafe {
        let start = DWT::cycle_count();
        let mut lines = [LineProgress { last: start, ..Default::default() }; N];
        // Line i starts transmitting `i * stagger` cycles in, counting active lines only.
        let mut starts = [0u32; N];
        let mut k = 0;
        for i in 0..N {
            if active[i] {
                starts[i] = k * stagger;
                k += 1;
            }
        }
        for i in 0..N {
            if !active[i] {
                lines[i].done = true;
                continue;
            }
            let raw = raws[i];
            let (cr1, cr3, icr, rdr) = (raw.reg(CR1), raw.reg(CR3), raw.reg(ICR), raw.reg(RDR));
            // No DMA requests and no error interrupt; transmitter and receiver on.
            cr3.write_volatile(cr3.read_volatile() & !((1 << 7) | (1 << 6) | 1));
            cr1.write_volatile(cr1.read_volatile() | (1 << 3) | (1 << 2));
            icr.write_volatile(0b1111 | ICR_TCCF);
            // Empty anything left from before, FIFO included.
            while raw.reg(ISR).read_volatile() & ISR_RXNE != 0 {
                let _ = rdr.read_volatile();
            }
            if active_drive {
                raw.set_open_drain(false);
                lines[i].driving = true;
            }
        }

        loop {
            let now = DWT::cycle_count();
            let mut all_done = true;
            for i in 0..N {
                let line = &mut lines[i];
                if line.done {
                    continue;
                }
                all_done = false;
                let raw = raws[i];
                let status = raw.reg(ISR).read_volatile();
                // Release the line as soon as the last stop bit is out: the servo starts its
                // reply tens of microseconds later.
                if line.sent == j288::COMMAND_LEN && status & ISR_TC != 0 && line.driving {
                    raw.set_open_drain(true);
                    line.driving = false;
                    line.last = now;
                }
                if status & (ISR_FE | ISR_NE | ISR_ORE) != 0 {
                    line.flags |= status & (ISR_FE | ISR_NE | ISR_ORE);
                    raw.reg(ICR).write_volatile(0b1110);
                }
                // Drain the receive FIFO and fill the transmit one as far as they go: one byte a
                // pass would leave gaps inside the outgoing frame whenever a pass over all the
                // lines takes longer than a byte (1.7 µs), and the servos do not answer a frame
                // with gaps in it.
                while line.received < RX_LEN && raw.reg(ISR).read_volatile() & ISR_RXNE != 0 {
                    bufs[i][line.received] = raw.reg(RDR).read_volatile() as u8;
                    line.received += 1;
                    line.last = now;
                }
                if line.received == RX_LEN {
                    line.done = true;
                    continue;
                }
                while line.sent < j288::COMMAND_LEN
                    && now.wrapping_sub(start) >= starts[i]
                    && raw.reg(ISR).read_volatile() & ISR_TXE != 0
                {
                    raw.reg(TDR).write_volatile(frames[i][line.sent] as u32);
                    line.sent += 1;
                    line.last = now;
                }
                let idle = line.sent == j288::COMMAND_LEN && !line.driving && now.wrapping_sub(line.last) > IDLE_CYCLES;
                if idle || now.wrapping_sub(start) > CAP_CYCLES + starts[i] {
                    line.done = true;
                }
            }
            if all_done {
                break;
            }
        }
        for i in 0..N {
            if lines[i].driving {
                raws[i].set_open_drain(true);
            }
        }
        core::array::from_fn(|i| (lines[i].received, lines[i].flags))
    })
}
