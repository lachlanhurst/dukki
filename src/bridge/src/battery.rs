//! Low-battery detection for the status display's warning.
//!
//! The pack voltage for now is the highest supply reading any servo reports, in
//! the J288's 0.5 V steps. The highest because harness drop under load only
//! lowers a servo's reading, so the highest is the closest to the pack. To be
//! replaced by the bridge's own divider on PA0 (hardware.md section 10), which
//! the docs plan for and the firmware does not read yet; only `pack_volts`
//! needs to change.
//!
//! Servo current sags the pack: a burst of hard movement can pull it a volt or
//! two under its resting value for a second or two. The decision is therefore
//! made on an exponential average with a 10 s time constant, which a 2 V dip
//! lasting 2 s moves by under 0.4 V, while a pack that sits low on average,
//! dips included, still trips it. Once raised, the warning stays until the
//! average is back above `CLEAR_V`. A LiPo recovers a few tenths of a volt as
//! soon as the load eases, so clearing at the warning threshold itself would let
//! the warning flicker on and off; `CLEAR_V` is a pack that has been charged or
//! swapped.

use core::cell::Cell;

use embassy_time::Instant;

use crate::segment::{self, SegmentStats};

/// 6S at 3.50 V per cell, empty under load (hardware.md section 10, and
/// `BATTERY_EMPTY_V` for robotd).
pub const LOW_V: f32 = 21.0;
/// 6S at about 3.83 V per cell: a recharged or fresh pack, not a rested empty one.
pub const CLEAR_V: f32 = 23.0;
/// Time constant of the average the decision is made on.
const TAU_S: f32 = 10.0;

/// The pack voltage, or `None` while no servo is answering.
pub fn pack_volts(segs: &[Cell<SegmentStats>; segment::SEGMENTS]) -> Option<f32> {
    let mut volts: Option<f32> = None;
    for cell in segs {
        let seg = cell.get();
        for st in seg.servos[..seg.count].iter().filter_map(|s| s.last) {
            volts = Some(volts.map_or(st.volts, |v| v.max(st.volts)));
        }
    }
    volts
}

#[derive(Default)]
pub struct LowBattery {
    warning: bool,
    /// The averaged voltage and when it was last updated; `None` until the first reading.
    average: Option<(f32, Instant)>,
}

impl LowBattery {
    /// Feeds the latest reading and returns whether to warn. No reading (no servo
    /// answering) neither raises nor clears the warning.
    pub fn update(&mut self, volts: Option<f32>, now: Instant) -> bool {
        let Some(v) = volts else {
            return self.warning;
        };
        let average = match self.average {
            None => v,
            Some((avg, at)) => {
                let dt = (now - at).as_micros() as f32 / 1e6;
                avg + (v - avg) * (1.0 - libm::expf(-dt / TAU_S))
            }
        };
        self.average = Some((average, now));
        if self.warning {
            self.warning = average < CLEAR_V;
        } else {
            self.warning = average < LOW_V;
        }
        self.warning
    }

    /// The averaged voltage the decision is made on.
    pub fn average(&self) -> Option<f32> {
        self.average.map(|(v, _)| v)
    }
}
