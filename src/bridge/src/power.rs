//! The power hold on the carrier PCB's soft power switch (bridge-pcb.md section
//! 6.5). A press of the button turns the soft start on for about a second; PC4
//! (PWR_HOLD) keeps it on after that, and releasing PC4 turns everything off,
//! the bridge included when it runs from the pack. PA0 reads the switched rail
//! through its 100 k / 10 k divider, which says whether the robot is powered.
//!
//! The rules (bridge-pcb.md 6.5):
//! - At boot the hold goes on if the button is pressed or the rail is live (C15
//!   kept the switch on through a reset), so USB alone does not power the robot.
//! - The press that turned the power on does not count towards a shutdown; only
//!   a fresh press can start the 3 s hold (button.rs).
//! - After a committed shutdown the hold is released once the head reports off,
//!   and the button does nothing more. If the bridge is still running a few
//!   seconds later it is on USB, or the switch is bridged always on (JP2), or
//!   this is the breadboard, where PC4 goes nowhere: the display then says to
//!   release the button or unplug the pack.
//! - On USB, with the rail seen dead, a fresh press turns the power on again.
//!
//! The task also pets the independent watchdog, so a hung bridge resets and lets
//! the hold go rather than keep the robot on until the pack is unplugged.

use core::cell::Cell;

use embassy_stm32::adc::{Adc, AnyAdcChannel, SampleTime};
use embassy_stm32::gpio::Output;
use embassy_stm32::peripherals::{ADC1, IWDG};
use embassy_stm32::wdg::IndependentWatchdog;
use embassy_time::{Duration, Instant, Ticker};

use crate::button::{self, Button};
use crate::link::{self, Head, HeadStatus, HostCommand};

const POLL: Duration = Duration::from_millis(10);
/// Longer than any stretch the firmware holds the core, by a wide margin. A watchdog
/// reset leaves PC4 floating; C15 holds the switch on for at least 0.22 s, and the
/// hold is back on within a few milliseconds of the reset (`at_boot`).
pub const WATCHDOG_US: u32 = 1_000_000;
/// Above this the switched rail is live. The J288's minimum is 6.4 V; with the
/// switch off the divider pulls the rail to 0 V.
const RAIL_LIVE_V: f32 = 6.0;
/// After the hold is released, how long before the display says why the bridge is
/// still running. On the pack it is off within about a second of the button's release.
pub const STILL_RUNNING: Duration = Duration::from_secs(3);

/// The switched rail's voltage from PA0: 3.3 V full scale, divided by 11.
pub struct Rail<'d> {
    pub adc: Adc<'d, ADC1>,
    pub pin: AnyAdcChannel<'d, ADC1>,
}

impl Rail<'_> {
    pub fn volts(&mut self) -> f32 {
        // The divider's 100 nF makes the source stiff, so a short sample is plenty.
        let raw = self.adc.blocking_read(&mut self.pin, SampleTime::CYCLES47_5);
        raw as f32 * 3.3 / 4095.0 * 11.0
    }
}

#[derive(Clone, Copy, Default, PartialEq)]
pub enum State {
    /// The hold is off: USB only, nothing pressed.
    #[default]
    Off,
    /// The hold is on.
    On,
    /// A shutdown is committed and the hold stays on until the head is off.
    ShuttingDown { since: Instant },
    /// The hold is released after a shutdown. The bridge is still running only on
    /// USB, with the switch bridged always on, or on the breadboard.
    Released { since: Instant },
}

#[derive(Clone, Copy, Default)]
pub struct Power {
    pub state: State,
    /// The switched rail at the last poll.
    pub rail_v: f32,
    /// Since when the rail has been live, if it is.
    pub live_since: Option<Instant>,
    /// What the boot decision saw: the button down, and the rail.
    pub boot_button: bool,
    pub boot_rail_v: f32,
}

/// Decides the hold at boot, before anything else is set up: on if the button is
/// pressed or the rail is live.
pub fn at_boot(hold: &mut Output<'_>, button_down: bool, rail: &mut Rail<'_>) -> Power {
    let rail_v = rail.volts();
    let on = button_down || rail_v > RAIL_LIVE_V;
    if on {
        hold.set_high();
    }
    Power {
        state: if on { State::On } else { State::Off },
        rail_v,
        live_since: (rail_v > RAIL_LIVE_V).then(Instant::now),
        boot_button: button_down,
        boot_rail_v: rail_v,
    }
}

pub async fn run(
    mut hold: Output<'_>,
    mut rail: Rail<'_>,
    iwdg: embassy_stm32::Peri<'_, IWDG>,
    shared: &Cell<Power>,
    button: &Cell<Button>,
    host: &Cell<HostCommand>,
    head: &Cell<HeadStatus>,
) {
    let mut watchdog = IndependentWatchdog::new(iwdg, WATCHDOG_US);
    watchdog.unleash();
    let mut ticker = Ticker::every(POLL);
    let mut p = shared.get();
    loop {
        ticker.next().await;
        watchdog.pet();
        let now = Instant::now();
        p.rail_v = rail.volts();
        let live = p.rail_v > RAIL_LIVE_V;
        p.live_since = if live { p.live_since.or(Some(now)) } else { None };
        let mut b = button.get();
        let fresh_press = b.held_since.is_some() && !b.ignored;
        p.state = match p.state {
            State::Off if fresh_press => {
                // On USB: this press turns the power on, and does not count as a hold.
                b.ignored = true;
                b.held_since = None;
                hold.set_high();
                State::On
            }
            State::On if b.fired => {
                // The hold completed: button.rs has made the request.
                State::ShuttingDown { since: b.requested_at.unwrap_or(now) }
            }
            State::ShuttingDown { since } => {
                let release = match link::head(&host.get(), &head.get(), now.as_micros()) {
                    Head::Off => true,
                    // The head acknowledged: wait for it to go quiet.
                    Head::ShuttingDown => false,
                    // No compute module has spoken since the bridge started: nothing to wait for.
                    Head::NoLink => true,
                    // Running or stalled without acknowledging: give it as long as the
                    // request lasts, then cut the power anyway.
                    Head::Running(_) | Head::Stalled => now - since >= button::REQUEST_LAPSE,
                };
                if release {
                    hold.set_low();
                    b.locked = true;
                    State::Released { since: now }
                } else {
                    State::ShuttingDown { since }
                }
            }
            State::Released { .. } if !live && !b.down => {
                // Still running with the rail dead and the button up: on USB. A fresh press
                // may turn the power on again.
                b.locked = false;
                State::Off
            }
            state => state,
        };
        button.set(b);
        shared.set(p);
    }
}
