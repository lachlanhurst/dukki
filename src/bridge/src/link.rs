//! The link to the compute module: USART1, full duplex, PA9 (TX) to the CM4
//! carrier's header pin 18 and PA10 (RX) from pin 16, speaking
//! `duck-bridge-proto`.
//!
//! The compute module sends a `Command` and a `StateRequest` every control
//! tick. Commands are stored for the segments, which apply them on their next
//! pass; a state request is answered at once from what the segments and the IMU
//! already hold, so the reply never waits on a servo. Reception is a circular
//! DMA buffer, which keeps receiving through the polled servo loop's
//! interrupts-off windows: an interrupt-driven receiver would lose bytes there.

use core::cell::Cell;

use duck_bridge_proto::{self as proto, FrameDecoder, Kind, Message};
use embassy_stm32::mode::Async;
use embassy_stm32::usart::{RingBufferedUartRx, UartTx};
use embassy_time::{Duration, Instant};

use crate::imu;
use crate::segment::{self, SegmentStats};

/// 168 MHz / 2 Mbps is an integer USART divider (84). 4 Mbps (42) is the planned
/// step up once the neck harness has been checked on a scope (hardware.md 5.5).
pub const LINK_BAUD: u32 = 2_000_000;

/// With no command for this long the segments stop every servo. Five ticks at
/// 50 Hz, ten at 100 Hz: long enough to ride out a scheduling hiccup on the
/// compute module, short enough that a crashed robotd cannot leave the robot
/// holding a stale pose.
pub const WATCHDOG: Duration = Duration::from_millis(100);

/// The compute module's latest command and when it arrived.
#[derive(Clone, Copy)]
pub struct HostCommand {
    pub command: proto::Command,
    /// Bridge clock when it arrived, µs; `None` until the first one.
    pub received_us: Option<u64>,
}

impl Default for HostCommand {
    fn default() -> Self {
        Self { command: proto::Command::default(), received_us: None }
    }
}

/// What the link has seen, for the bench console.
#[derive(Clone, Copy, Default)]
pub struct LinkStats {
    pub frames: u32,
    pub commands: u32,
    pub state_requests: u32,
    pub info_requests: u32,
    /// Frames the decoder rejected (CRC, version, length).
    pub frame_errors: u32,
    /// Receive overruns and other UART errors.
    pub uart_errors: u32,
    /// Well-formed frames whose payload did not decode, or of a kind the bridge
    /// does not accept.
    pub bad_messages: u32,
    /// Bytes handed to the transmit DMA.
    pub tx_bytes: u32,
}

enum Request {
    Command(proto::Command),
    State(proto::StateRequest),
    Info,
}

pub async fn run(
    tx: &mut UartTx<'_, Async>,
    rx: &mut RingBufferedUartRx<'_>,
    host: &Cell<HostCommand>,
    segs: &[Cell<SegmentStats>; segment::SEGMENTS],
    imu_shared: &Cell<imu::Snapshot>,
    stats: &Cell<LinkStats>,
) {
    let mut decoder = FrameDecoder::new();
    let mut chunk = [0u8; 256];
    let mut out = [0u8; proto::MAX_FRAME];
    let mut state_seq: u32 = 0;
    let mut ls = LinkStats::default();

    loop {
        let n = match rx.read(&mut chunk).await {
            Ok(n) => n,
            Err(_) => {
                // The ring buffer stops on an error and restarts on the next read.
                ls.uart_errors += 1;
                stats.set(ls);
                continue;
            }
        };
        let mut taken = 0;
        while taken < n {
            taken += decoder.push(&chunk[taken..n]);
            loop {
                let request = match decoder.next_frame() {
                    None => break,
                    Some(frame) => {
                        ls.frames += 1;
                        match frame.kind {
                            Kind::Command => proto::Command::decode(&frame).map(Request::Command),
                            Kind::StateRequest => proto::StateRequest::decode(&frame).map(Request::State),
                            Kind::InfoRequest => proto::InfoRequest::decode(&frame).map(|_| Request::Info),
                            _ => Err(proto::Error::WrongKind),
                        }
                    }
                };
                match request {
                    Err(_) => ls.bad_messages += 1,
                    Ok(Request::Command(command)) => {
                        ls.commands += 1;
                        host.set(HostCommand { command, received_us: Some(Instant::now().as_micros()) });
                    }
                    Ok(Request::State(req)) => {
                        ls.state_requests += 1;
                        state_seq = state_seq.wrapping_add(1);
                        let errors = (decoder.errors + ls.uart_errors + ls.bad_messages) as u16;
                        let state = build_state(state_seq, req, &host.get(), segs, &imu_shared.get(), errors);
                        if let Ok(len) = state.encode(&mut out) {
                            let _ = tx.write(&out[..len]).await;
                            ls.tx_bytes += len as u32;
                        }
                    }
                    Ok(Request::Info) => {
                        ls.info_requests += 1;
                        if let Ok(len) = build_info(segs).encode(&mut out) {
                            let _ = tx.write(&out[..len]).await;
                            ls.tx_bytes += len as u32;
                        }
                    }
                }
            }
        }
        ls.frame_errors = decoder.errors;
        stats.set(ls);
    }
}

fn build_state(
    seq: u32,
    req: proto::StateRequest,
    host: &HostCommand,
    segs: &[Cell<SegmentStats>; segment::SEGMENTS],
    imu_snap: &imu::Snapshot,
    link_errors: u16,
) -> proto::State {
    let now_us = Instant::now().as_micros();
    let mut state = proto::State {
        seq,
        request_host_time_us: req.host_time_us,
        bridge_time_us: now_us,
        link_errors,
        imu: imu::imu_state(imu_snap, now_us),
        ..Default::default()
    };
    if let Some(at) = host.received_us {
        let age = now_us.saturating_sub(at);
        state.command_seq = host.command.seq;
        state.command_age_us = age.min(u32::MAX as u64) as u32;
        state.flags |= proto::state_flags::COMMANDED;
        if age > WATCHDOG.as_micros() {
            state.flags |= proto::state_flags::WATCHDOG;
        }
    }
    for s in &mut state.servos {
        s.segment = proto::ABSENT;
    }
    let mut rounds = u32::MAX;
    for (index, cell) in segs.iter().enumerate() {
        let seg = cell.get();
        for stats in &seg.servos[..seg.count] {
            if let Some(slot) = state.servos.get_mut(stats.id as usize) {
                *slot = segment::servo_state(index as u8, stats, now_us);
            }
        }
        if seg.count > 0 {
            rounds = rounds.min(seg.rounds);
            state.round_us = state.round_us.max(seg.round_us.min(u16::MAX as u32) as u16);
        }
    }
    state.rounds = if rounds == u32::MAX { 0 } else { rounds };
    state
}

fn build_info(segs: &[Cell<SegmentStats>; segment::SEGMENTS]) -> proto::Info {
    let mut info = proto::Info {
        protocol_version: proto::PROTOCOL_VERSION,
        sysclk_mhz: 168,
        link_baud: LINK_BAUD,
        watchdog_ms: WATCHDOG.as_millis() as u16,
        segments: [proto::ABSENT; proto::NUM_SERVOS],
        ..Default::default()
    };
    let version = concat!("bridge ", env!("CARGO_PKG_VERSION")).as_bytes();
    info.firmware[..version.len()].copy_from_slice(version);
    for (index, cell) in segs.iter().enumerate() {
        let seg = cell.get();
        for stats in &seg.servos[..seg.count] {
            if let Some(slot) = info.segments.get_mut(stats.id as usize) {
                *slot = index as u8;
            }
        }
    }
    info
}
