//! The shutdown button: a normally open switch from PB5 (header P1-20) to
//! ground, held up by the pin's internal pull-up.
//!
//! Polled every 10 ms, which costs the servo loop a few microseconds a poll. A
//! change of level counts once it has read the same for `DEBOUNCE`, so contact
//! bounce and a brushed lead do nothing. Holding the button for `HOLD` asks
//! the compute module to shut down (link.rs sets the request flag in its
//! replies). It fires once per press: keep holding and nothing more happens,
//! release early and nothing has happened at all.
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
            b.held_since = down.then_some(raw_since);
            b.fired = false;
            shared.set(b);
        }
        let mut b = shared.get();
        if down && !b.fired && b.held_since.is_some_and(|t| now - t >= HOLD) {
            b.requested_at = Some(now);
            b.requests = b.requests.wrapping_add(1);
            b.fired = true;
            shared.set(b);
        }
    }
}
