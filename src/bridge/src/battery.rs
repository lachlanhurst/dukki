//! Low-battery detection for the status display's warning.
//!
//! The pack voltage is the switched rail on PA0, through the carrier's 100 k /
//! 10 k divider (power.rs), which reads the pack whenever the power is on. The
//! servos' own supply readings were used before: they come in 0.5 V steps, and
//! a servo that loses power keeps its last reply, taken as the rail fell, so
//! the warning came on after every power-off. With the power off there is no
//! reading and no warning. The rail is not counted until it has been live for
//! `SETTLE`, so the soft start's ramp does not start the average low.
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

use embassy_time::{Duration, Instant};

use crate::power::Power;

/// 6S at 3.50 V per cell, empty under load (hardware.md section 10, and
/// `BATTERY_EMPTY_V` for robotd).
pub const LOW_V: f32 = 21.0;
/// 6S at about 3.83 V per cell: a recharged or fresh pack, not a rested empty one.
pub const CLEAR_V: f32 = 23.0;
/// Time constant of the average the decision is made on.
const TAU_S: f32 = 10.0;

/// How long the rail must have been live before it counts.
const SETTLE: Duration = Duration::from_secs(1);

/// The pack voltage, or `None` while the power is off.
pub fn pack_volts(power: &Cell<Power>, now: Instant) -> Option<f32> {
    let p = power.get();
    p.live_since.filter(|&since| now - since >= SETTLE).map(|_| p.rail_v)
}

#[derive(Default)]
pub struct LowBattery {
    warning: bool,
    /// The averaged voltage and when it was last updated; `None` until the first reading.
    average: Option<(f32, Instant)>,
}

impl LowBattery {
    /// Feeds the latest reading and returns whether to warn. No reading (the power
    /// is off) clears the warning and the average.
    pub fn update(&mut self, volts: Option<f32>, now: Instant) -> bool {
        let Some(v) = volts else {
            *self = Self::default();
            return false;
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
