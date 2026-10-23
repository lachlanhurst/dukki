//! The shutdown button: a normally open switch from PB5 (header P1-20) to
//! ground, held up by the pin's internal pull-up. On the carrier PCB the button
//! itself switches the pack to the soft power switch, and Q3 pulls PB5 low while
//! it is held and for about a second after (C14), so a press reads the same.
//!
//! Polled every 10 ms, which costs the servo loop a few microseconds a poll. A
//! change of level counts once it has read the same for `DEBOUNCE`, so contact
//! bounce and a brushed lead do nothing. Holding the button for `HOLD` asks
//! the compute module to shut down (link.rs sets the request flag in its
//! replies). It fires once per press: keep holding and nothing more happens,
//! release early and nothing has happened at all.
//!
//! A press already down at boot does not count until it is released: on the
//! carrier PCB it is the press that turned the power on. Nor does the press that
//! turns the power on from USB (power.rs), and once the power hold is released
//! after a shutdown no press counts at all.
//!
//! A request nobody acknowledges lapses after `REQUEST_LAPSE`, so a press while
//! robotd is not running cannot shut the head down on its next start.

use core::cell::Cell;

use embassy_stm32::gpio::Input;
use embassy_time::{Duration, Instant, Ticker};

const POLL: Duration = Duration::from_millis(10);
const DEBOUNCE: Duration = Duration::from_millis(30);
pub const HOLD: Duration = Duration::from_secs(3);
pub const REQUEST_LAPSE: Duration = Duration::from_secs(10);

#[derive(Clone, Copy, Default)]
pub struct Button {
    /// The debounced button is down, whether or not this press counts.
    pub down: bool,
    /// This press does not count: it was down at boot or turned the power on.
    pub ignored: bool,
    /// No press counts: the power hold is released after a shutdown (power.rs).
    pub locked: bool,
    /// Since when the debounced button has been down, if it is.
    pub held_since: Option<Instant>,
    /// This press has been held long enough and has made its request.
    pub fired: bool,
    /// When the last hold completed: the shutdown request, until acknowledged or lapsed.
    pub requested_at: Option<Instant>,
    /// Holds completed since the bridge started.
    pub requests: u32,
}

impl Button {
    /// Whether the compute module should be asked to shut down now.
    pub fn request_pending(&self, now: Instant) -> bool {
        self.requested_at.is_some_and(|at| now - at < REQUEST_LAPSE)
    }
}

pub async fn run(pin: Input<'_>, shared: &Cell<Button>) {
    let mut ticker = Ticker::every(POLL);
    // The raw level and since when it has read that way, and the debounced state.
    let mut raw_down = pin.is_low();
    let mut raw_since = Instant::now();
    let mut down = false;
    if raw_down {
        let mut b = shared.get();
        b.ignored = true;
        shared.set(b);
    }
    loop {
        ticker.next().await;
        let now = Instant::now();
        let level_down = pin.is_low();
        if level_down != raw_down {
            raw_down = level_down;
            raw_since = now;
        }
        if raw_down != down && now - raw_since >= DEBOUNCE {
            down = raw_down;
            let mut b = shared.get();
            b.down = down;
            if down {
                b.ignored |= b.locked;
            } else {
                b.ignored = false;
            }
            b.held_since = (down && !b.ignored).then_some(raw_since);
            b.fired = false;
            shared.set(b);
        }
        let mut b = shared.get();
        if down && !b.ignored && !b.fired && b.held_since.is_some_and(|t| now - t >= HOLD) {
            b.requested_at = Some(now);
            b.requests = b.requests.wrapping_add(1);
            b.fired = true;
            shared.set(b);
        }
    }
}
