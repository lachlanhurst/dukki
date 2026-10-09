//! Status display: a 0.96 inch 128 x 64 OLED with an SSD1315 controller, which
//! takes the SSD1306 command set, on I2C1 (SCL PA15, SDA PB7) at 400 kHz.
//!
//! The servo segments' polled exchanges keep the core busy for most of every
//! 2 ms tick, so this task must never hold it for long. Every transfer goes by
//! DMA, and the task yields between small steps: formatting a line, drawing a
//! few characters (embedded-graphics sets a glyph's 60 pixels one at a time, so
//! a whole line in one go held the core for about 1 ms), and sending a 128-byte
//! page. Only lines whose text changed are redrawn, and only their pages sent.
//!
//! When the pack runs low (battery.rs) the status gives way to a full-screen
//! warning, a battery outline over "LOW BATTERY", flashing between dark and
//! light every 0.5 s. The image is drawn once at startup into its own buffer and
//! sent by DMA when the warning starts; the flashing is the controller's invert
//! command, one two-byte write, so nothing is redrawn while it flashes.
//! What still blocks is inside embassy's I2C
//! driver: after each DMA transfer it spins until the last byte has left the
//! shift register (about 45 µs at 400 kHz), and before each start it spins
//! while the bus is busy, for up to the I2C timeout. So each page write first
//! waits for an idle bus here, yielding, and gives up rather than enter that
//! spin. A display that is missing or stops answering is retried every few
//! seconds and costs the servos nothing meanwhile.

use core::cell::Cell;
use core::fmt::Write;
use core::future::{poll_fn, Future};
use core::pin::pin;

use display_interface::{AsyncWriteOnlyDataCommand, DataFormat, DisplayError};
use embassy_futures::yield_now;
use embassy_stm32::i2c::{I2c, Master};
use embassy_stm32::mode::Async;
use embassy_time::{Duration, Instant, Timer};
use embedded_graphics::mono_font::ascii::{FONT_10X20, FONT_6X10};
use embedded_graphics::mono_font::{MonoTextStyle, MonoTextStyleBuilder};
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{PrimitiveStyle, Rectangle};
use embedded_graphics::text::{Baseline, Text};
use ssd1306::prelude::*;
use ssd1306::Ssd1306Async;

use crate::battery::{self, LowBattery};
use crate::segment::{self, SegmentStats};
use crate::{imu, link};

/// The module's I2C address; a few are jumpered to 0x3D.
const ADDRESS: u8 = 0x3C;
/// For the whole of one page write, about 2.9 ms of bus time at 400 kHz.
pub const I2C_TIMEOUT: Duration = Duration::from_millis(20);
/// Redraw period, and the warning's flash period.
const PERIOD: Duration = Duration::from_millis(500);
/// Wait between attempts while the display is missing or failing.
const RETRY: Duration = Duration::from_secs(3);
/// How long a write waits for the bus to go idle, yielding, before giving up.
const IDLE_WAIT: Duration = Duration::from_millis(2);
/// Characters per line in the 6 x 10 font, and lines on the screen.
const COLUMNS: usize = 21;
const ROWS: usize = 6;
/// Characters drawn between yields.
const CHARS_PER_STEP: usize = 3;

#[derive(Clone, Copy, Default)]
pub struct DisplayStats {
    /// Initialised and the last frame went out.
    pub up: bool,
    pub frames: u32,
    pub errors: u32,
    /// Longest the task held the core in one go, µs: the cost to every other task.
    pub slice_max_us: u32,
    /// Wall time of the last frame's transfer, µs, nearly all of it awaiting DMA.
    pub flush_us: u32,
    /// Longest single character draw, µs.
    pub char_max_us: u32,
    /// The averaged pack voltage the low-battery decision is made on, and the decision.
    pub battery_v: Option<f32>,
    pub battery_low: bool,
}

/// One I2C write per command batch or 128-byte page. display-interface-i2c, the
/// adapter ssd1306 offers, splits data into 16-byte writes, eight times as
/// many transfers and as many of the driver's end-of-transfer spins.
struct Bus<'d> {
    i2c: I2c<'d, Async, Master>,
}

impl Bus<'_> {
    async fn write(&mut self, control: u8, bytes: &[u8]) -> Result<(), DisplayError> {
        let mut frame = [0u8; 129];
        for chunk in bytes.chunks(128) {
            frame[0] = control;
            frame[1..=chunk.len()].copy_from_slice(chunk);
            wait_idle().await?;
            self.i2c.write(ADDRESS, &frame[..=chunk.len()]).await.map_err(|_| DisplayError::BusWriteError)?;
            yield_now().await;
        }
        Ok(())
    }
}

/// Waits for the I2C1 bus to go idle, yielding, so the driver never spins on BUSY.
async fn wait_idle() -> Result<(), DisplayError> {
    let deadline = Instant::now() + IDLE_WAIT;
    while embassy_stm32::pac::I2C1.isr().read().busy() {
        if Instant::now() > deadline {
            return Err(DisplayError::BusWriteError);
        }
        yield_now().await;
    }
    Ok(())
}

impl AsyncWriteOnlyDataCommand for Bus<'_> {
    async fn send_commands(&mut self, cmd: DataFormat<'_>) -> Result<(), DisplayError> {
        match cmd {
            DataFormat::U8(bytes) => self.write(0x00, bytes).await,
            _ => Err(DisplayError::DataFormatNotImplemented),
        }
    }

    async fn send_data(&mut self, buf: DataFormat<'_>) -> Result<(), DisplayError> {
        match buf {
            DataFormat::U8(bytes) => self.write(0x40, bytes).await,
            _ => Err(DisplayError::DataFormatNotImplemented),
        }
    }
}

/// A line of text that `write!` can format into, cut at the screen width.
#[derive(Clone, Copy, PartialEq)]
struct TextLine {
    buf: [u8; COLUMNS],
    len: usize,
}

impl TextLine {
    const EMPTY: Self = Self { buf: [0; COLUMNS], len: 0 };

    /// Spaces to the screen width, so drawing the line also erases what it replaces.
    fn pad(&mut self) {
        self.buf[self.len..].fill(b' ');
        self.len = COLUMNS;
    }

    fn as_str(&self) -> &str {
        core::str::from_utf8(&self.buf[..self.len]).unwrap_or("")
    }
}

impl Write for TextLine {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let n = s.len().min(COLUMNS - self.len);
        self.buf[self.len..self.len + n].copy_from_slice(&s.as_bytes()[..n]);
        self.len += n;
        Ok(())
    }
}

pub async fn run(
    i2c: I2c<'_, Async, Master>,
    imu_shared: &Cell<imu::Snapshot>,
    segs: &[Cell<SegmentStats>; segment::SEGMENTS],
    host: &Cell<link::HostCommand>,
    stats: &Cell<DisplayStats>,
) {
    let mut display =
        Ssd1306Async::new(Bus { i2c }, DisplaySize128x64, DisplayRotation::Rotate0).into_buffered_graphics_mode();
    // Polled through `timed`, so every stretch the task holds the core is measured.
    let mut slice_max_us = 0;
    let task = async {
        // A static, not a local: the whole firmware runs as one task built on the stack at
        // startup, and its size is what limits the stack (main.rs).
        let warning = cortex_m::singleton!(: Canvas = Canvas { buf: [0; 1024] }).unwrap();
        draw_warning(warning).await;
        let mut low = LowBattery::default();
        loop {
            // init clears the buffer and marks all of it changed, so the flush blanks the panel.
            if display.init().await.is_err() || display.flush().await.is_err() {
                note_error(stats);
                Timer::after(RETRY).await;
                continue;
            }
            let mut shown = [TextLine::EMPTY; ROWS];
            // Whether the panel holds the warning image, and whether it is inverted now.
            let mut warning_shown = false;
            let mut inverted = false;
            loop {
                let warn = low.update(battery::pack_volts(segs), Instant::now());
                let mut s = stats.get();
                s.battery_v = low.average();
                s.battery_low = warn;
                stats.set(s);

                let result = if warn {
                    flash_warning(&mut display, warning, &mut warning_shown, &mut inverted).await
                } else {
                    if warning_shown {
                        // The panel holds the warning; redraw the whole status screen.
                        warning_shown = false;
                        display.clear_buffer();
                        shown = [TextLine::EMPTY; ROWS];
                    }
                    let result = if inverted { display.set_invert(false).await } else { Ok(()) };
                    inverted = false;
                    match result {
                        Ok(()) => show_status(&mut display, &mut shown, imu_shared, segs, host, stats).await,
                        Err(e) => Err(e),
                    }
                };
                if result.is_err() {
                    note_error(stats);
                    break;
                }
                Timer::after(PERIOD).await;
            }
            Timer::after(RETRY).await;
        }
    };
    timed(task, &mut slice_max_us, stats).await
}

/// Redraws the status lines that changed since `shown`.
async fn show_status(
    display: &mut Display<'_>,
    shown: &mut [TextLine; ROWS],
    imu_shared: &Cell<imu::Snapshot>,
    segs: &[Cell<SegmentStats>; segment::SEGMENTS],
    host: &Cell<link::HostCommand>,
    stats: &Cell<DisplayStats>,
) -> Result<(), DisplayError> {
    let lines = status_lines(imu_shared, segs, host).await;
    if lines == *shown {
        return Ok(());
    }
    let start = Instant::now();
    draw(display, &lines, shown, stats).await?;
    *shown = lines;
    let mut s = stats.get();
    s.up = true;
    s.frames = s.frames.wrapping_add(1);
    s.flush_us = (Instant::now() - start).as_micros() as u32;
    stats.set(s);
    Ok(())
}

/// Sends the warning image if the panel does not hold it yet, then flips the
/// panel between dark and light: one invert command per call.
async fn flash_warning(
    display: &mut Display<'_>,
    warning: &Canvas,
    warning_shown: &mut bool,
    inverted: &mut bool,
) -> Result<(), DisplayError> {
    if !*warning_shown {
        display.set_draw_area((0, 0), (128, 64)).await?;
        display.draw(&warning.buf).await?;
        *warning_shown = true;
    }
    *inverted = !*inverted;
    display.set_invert(*inverted).await
}

/// A 128 x 64 image in the controller's layout: byte `page * 128 + x`, bit `y % 8`.
struct Canvas {
    buf: [u8; 1024],
}

impl Canvas {
    fn set(&mut self, x: i32, y: i32, on: bool) {
        if (0..128).contains(&x) && (0..64).contains(&y) {
            let i = (y / 8) as usize * 128 + x as usize;
            let bit = 1 << (y % 8);
            if on {
                self.buf[i] |= bit;
            } else {
                self.buf[i] &= !bit;
            }
        }
    }
}

impl OriginDimensions for Canvas {
    fn size(&self) -> Size {
        Size::new(128, 64)
    }
}

impl DrawTarget for Canvas {
    type Color = BinaryColor;
    type Error = core::convert::Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for Pixel(p, color) in pixels {
            self.set(p.x, p.y, color.is_on());
        }
        Ok(())
    }

    fn fill_solid(&mut self, area: &Rectangle, color: Self::Color) -> Result<(), Self::Error> {
        for p in area.points() {
            self.set(p.x, p.y, color.is_on());
        }
        Ok(())
    }
}

/// The low-battery warning: a battery outline with a sliver of charge, over
/// "LOW BATTERY". Drawn at startup a piece at a time, yielding between pieces.
async fn draw_warning(canvas: &mut Canvas) {
    let on = BinaryColor::On;
    let _ = Rectangle::new(Point::new(20, 4), Size::new(80, 34))
        .into_styled(PrimitiveStyle::with_stroke(on, 3))
        .draw(canvas);
    yield_now().await;
    // The terminal, and the little charge left.
    let _ = Rectangle::new(Point::new(100, 14), Size::new(6, 14)).into_styled(PrimitiveStyle::with_fill(on)).draw(canvas);
    let _ = Rectangle::new(Point::new(26, 10), Size::new(10, 22)).into_styled(PrimitiveStyle::with_fill(on)).draw(canvas);
    yield_now().await;
    let style = MonoTextStyle::new(&FONT_10X20, on);
    let text = "LOW BATTERY";
    for (i, ch) in text.char_indices() {
        let at = Point::new(9 + i as i32 * 10, 42);
        let _ = Text::with_baseline(&text[i..i + ch.len_utf8()], at, style, Baseline::Top).draw(canvas);
        yield_now().await;
    }
}

type Display<'d> = Ssd1306Async<Bus<'d>, DisplaySize128x64, ssd1306::mode::BufferedGraphicsModeAsync<DisplaySize128x64>>;

/// Redraws the lines that differ from `shown`, a few characters at a time with a
/// yield between, then sends the pages they touched.
async fn draw(
    display: &mut Display<'_>,
    lines: &[TextLine; ROWS],
    shown: &[TextLine; ROWS],
    stats: &Cell<DisplayStats>,
) -> Result<(), DisplayError> {
    // A background colour makes each glyph overwrite its whole cell.
    let style: MonoTextStyle<BinaryColor> = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .background_color(BinaryColor::Off)
        .build();
    for (row, (line, old)) in lines.iter().zip(shown).enumerate() {
        if line == old {
            continue;
        }
        let text = line.as_str();
        for (i, ch) in text.char_indices() {
            let start = Instant::now();
            let glyph = &text[i..i + ch.len_utf8()];
            let at = Point::new(i as i32 * 6, row as i32 * 10 + 2);
            Text::with_baseline(glyph, at, style, Baseline::Top).draw(display)?;
            let us = (Instant::now() - start).as_micros() as u32;
            let mut s = stats.get();
            if us > s.char_max_us {
                s.char_max_us = us;
                stats.set(s);
            }
            if i % CHARS_PER_STEP == CHARS_PER_STEP - 1 {
                yield_now().await;
            }
        }
        yield_now().await;
    }
    display.flush().await
}

fn note_error(stats: &Cell<DisplayStats>) {
    let mut s = stats.get();
    s.up = false;
    s.errors = s.errors.wrapping_add(1);
    stats.set(s);
}

/// What the screen shows: link state, servos found, the IMU's attitude, and
/// the lowest supply voltage and hottest temperature across the servos.
/// Yields after each line, since core::fmt is slow at formatting floats.
async fn status_lines(
    imu_shared: &Cell<imu::Snapshot>,
    segs: &[Cell<SegmentStats>; segment::SEGMENTS],
    host: &Cell<link::HostCommand>,
) -> [TextLine; ROWS] {
    let mut lines = [TextLine::EMPTY; ROWS];
    let now = Instant::now();
    let _ = write!(lines[0], "Dukki bridge {:>5}s", now.as_secs());
    yield_now().await;

    let _ = match host.get().received_us {
        None => write!(lines[1], "link: no host"),
        Some(at) if now.as_micros().saturating_sub(at) <= link::WATCHDOG.as_micros() => {
            write!(lines[1], "link: host")
        }
        Some(_) => write!(lines[1], "link: WATCHDOG"),
    };
    yield_now().await;

    let _ = write!(lines[2], "servos");
    let mut volts_min = f32::MAX;
    let mut temp_max = i32::MIN;
    for (name, cell) in ["A", "B", "C", "D"].iter().zip(segs) {
        let seg = cell.get();
        let _ = write!(lines[2], " {}{}", name, seg.count);
        for st in seg.servos[..seg.count].iter().filter_map(|s| s.last) {
            volts_min = volts_min.min(st.volts);
            temp_max = temp_max.max(st.temp as i32).max(st.winding_temp as i32);
        }
    }
    yield_now().await;

    let s = imu_shared.get();
    let _ = match s.status {
        imu::Status::Running | imu::Status::BusError => {
            let e = imu::euler_deg(imu::quaternion(&s.block));
            write!(lines[3], "IMU r{:+6.1} p{:+6.1}", e[0], e[1])
        }
        imu::Status::Starting => write!(lines[3], "IMU starting"),
        imu::Status::NotFound => write!(lines[3], "IMU not found"),
        imu::Status::ConfigFailed => write!(lines[3], "IMU config failed"),
    };
    if s.status == imu::Status::BusError {
        let _ = write!(lines[4], "IMU BUS ERROR");
    }
    yield_now().await;

    if temp_max != i32::MIN {
        let _ = write!(lines[5], "{:4.1} V  max {} C", volts_min, temp_max);
    } else {
        let _ = write!(lines[5], "no servo replies");
    }
    for line in &mut lines {
        line.pad();
    }
    lines
}

/// Runs `fut`, recording the longest single poll: how long the task held the core.
async fn timed<F: Future>(fut: F, max_us: &mut u32, stats: &Cell<DisplayStats>) -> F::Output {
    let mut fut = pin!(fut);
    poll_fn(|cx| {
        let start = Instant::now();
        let result = fut.as_mut().poll(cx);
        let us = (Instant::now() - start).as_micros() as u32;
        if us > *max_us {
            *max_us = us;
            let mut s = stats.get();
            s.slice_max_us = us;
            stats.set(s);
        }
        result
    })
    .await
}
