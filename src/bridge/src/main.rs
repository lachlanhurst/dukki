//! Dukki servo bridge firmware for the WeAct STM32G474 core board (QFN48,
//! STM32G474CEU6).
//!
//! For now this is a bench test of the trunk IMU with on-chip fusion. ST's
//! `lsm6dsv16x-rs` driver configures the LSM6DSV16X on SPI3 (hardware.md
//! section 5.8) for the block the compute side decodes (section 6.1): gyro at
//! +-500 dps and the SFLP game rotation vector, both at 120 Hz through the
//! chip's FIFO, with INT1 (PA8) on the FIFO threshold. The latest 12-byte block
//! and decoded values are printed at 10 Hz on a USB CDC serial port. Reading the
//! IMU and printing run separately, so a slow or closed port never stalls the
//! FIFO. The blue LED on PC6 toggles with each printed line.

#![no_std]
#![no_main]

use core::cell::Cell;
use core::fmt::Write;

use embassy_executor::Spawner;
use embassy_futures::join::join3;
use embassy_stm32::exti::{self, ExtiInput};
use embassy_stm32::gpio::{Level, Output, Pull, Speed};
use embassy_stm32::rcc::{self, mux};
use embassy_stm32::spi::{self, Spi};
use embassy_stm32::time::Hertz;
use embassy_stm32::usb::{self, Driver};
use embassy_stm32::{bind_interrupts, dma, interrupt, peripherals};
use embassy_time::{with_timeout, Delay, Duration, Timer};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::Builder;
use embedded_hal_async::delay::DelayNs;
use embedded_hal_bus::spi::ExclusiveDevice;
use lsm6dsv16x_rs::asynchronous as lsm6dsv16x;
use lsm6dsv16x::prelude::*;
use panic_halt as _;
use st_mems_bus::asynchronous::BusOperation;

bind_interrupts!(struct Irqs {
    USB_LP => usb::InterruptHandler<peripherals::USB>;
    EXTI9_5 => exti::InterruptHandler<interrupt::typelevel::EXTI9_5>;
    DMA1_CHANNEL1 => dma::InterruptHandler<peripherals::DMA1_CH1>;
    DMA1_CHANNEL2 => dma::InterruptHandler<peripherals::DMA1_CH2>;
});

/// FIFO entries per INT1: at 120 Hz each for gyro, game rotation, gravity and
/// gyro bias the FIFO fills at 480 entries/s, so 4 entries is about 8 ms.
const FIFO_WATERMARK: u8 = 4;

/// What the IMU side has seen, shared with the printer.
#[derive(Clone, Copy, Default)]
struct Snapshot {
    who_am_i: u8,
    status: Status,
    /// The block the compute side decodes: gyro x, y, z as i16 raw counts at
    /// +-500 dps, then the game rotation x, y, z as half floats. All-zero
    /// quaternion bytes until the first fused sample.
    block: [u8; 12],
    gravity_mg: [f32; 3],
    gbias_mdps: [f32; 3],
    gyro_samples: u32,
    rotation_samples: u32,
    fifo_overruns: u32,
    int1_timeouts: u32,
}

#[derive(Clone, Copy, Default, PartialEq)]
enum Status {
    #[default]
    Starting,
    NotFound,
    ConfigFailed,
    BusError,
    Running,
}

/// A fixed buffer that `write!` can format into.
struct Line {
    buf: [u8; 256],
    len: usize,
}

impl Line {
    fn new() -> Self {
        Self { buf: [0; 256], len: 0 }
    }
    fn as_bytes(&self) -> &[u8] {
        &self.buf[..self.len]
    }
}

impl Write for Line {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let n = s.len().min(self.buf.len() - self.len);
        self.buf[self.len..self.len + n].copy_from_slice(&s.as_bytes()[..n]);
        self.len += n;
        Ok(())
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = embassy_stm32::Config::default();
    // USB runs from HSI48, trimmed by the CRS against the host's start-of-frame packets.
    config.rcc.hsi48 = Some(rcc::Hsi48Config { sync_from_usb: true });
    config.rcc.mux.clk48sel = mux::Clk48sel::HSI48;
    let p = embassy_stm32::init(config);

    let mut led = Output::new(p.PC6, Level::Low, Speed::Low);

    let mut spi_config = spi::Config::default();
    spi_config.frequency = Hertz(8_000_000);
    spi_config.mode = spi::MODE_3;
    let spi = Spi::new(p.SPI3, p.PC10, p.PB5, p.PC11, p.DMA1_CH1, p.DMA1_CH2, Irqs, spi_config);
    // High at boot: the chip stays in I2C mode until CS first goes low.
    let cs = Output::new(p.PA15, Level::High, Speed::Medium);
    let spi_device = ExclusiveDevice::new_no_delay(spi, cs).unwrap();
    let mut int1 = ExtiInput::new(p.PA8, p.EXTI8, Pull::Down, Irqs);

    // USB CDC ACM serial port.
    let driver = Driver::new(p.USB, Irqs, p.PA12, p.PA11);
    let mut usb_config = embassy_usb::Config::new(0x1209, 0x0001);
    usb_config.manufacturer = Some("Dukki");
    usb_config.product = Some("Dukki bridge");
    usb_config.serial_number = Some("bridge-imu-test");
    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut control_buf = [0; 64];
    let mut state = State::new();
    let mut builder = Builder::new(
        driver,
        usb_config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut [],
        &mut control_buf,
    );
    let mut class = CdcAcmClass::new(&mut builder, &mut state, 64);
    let mut usb = builder.build();

    let shared = Cell::new(Snapshot::default());

    let imu = async {
        let mut sensor = Lsm6dsv16x::new_spi(spi_device, Delay);
        Timer::after_millis(20).await; // boot time

        let mut snap = Snapshot::default();
        loop {
            snap.who_am_i = sensor.device_id_get().await.unwrap_or(0);
            if snap.who_am_i == ID {
                break;
            }
            snap.status = Status::NotFound;
            shared.set(snap);
            Timer::after_millis(500).await;
        }
        if configure(&mut sensor).await.is_err() {
            snap.status = Status::ConfigFailed;
            shared.set(snap);
            return;
        }
        snap.status = Status::Running;
        shared.set(snap);

        loop {
            // INT1 is high while the FIFO is at or above the watermark. A timeout means
            // INT1 is not firing; poll the FIFO anyway.
            if with_timeout(Duration::from_millis(100), int1.wait_for_high()).await.is_err() {
                snap.int1_timeouts += 1;
            }
            if drain_fifo(&mut sensor, &mut snap).await.is_err() {
                snap.status = Status::BusError;
            }
            shared.set(snap);
        }
    };

    let printer = async {
        loop {
            class.wait_connection().await;
            let _ = report(&mut class, &shared, &mut led).await;
        }
    };

    join3(usb.run(), imu, printer).await;
}

/// Gyro +-500 dps, accel +-4 g, both 120 Hz high-performance; SFLP game rotation at
/// 120 Hz; gyro, game rotation, gravity and gyro bias into the FIFO in stream mode;
/// FIFO threshold on INT1.
async fn configure<B: BusOperation, T: DelayNs>(
    sensor: &mut Lsm6dsv16x<B, T, MainBank>,
) -> Result<(), Error<B::Error>> {
    sensor.reset_set(Reset::RestoreCtrlRegs).await?;
    while sensor.reset_get().await? != Reset::Ready {}
    // CS glitches cannot drop the chip back to I2C once this is set.
    sensor.ui_i2c_i3c_mode_set(UiI2cI3cMode::Disable).await?;
    sensor.block_data_update_set(1).await?;
    sensor.xl_full_scale_set(XlFullScale::_4g).await?;
    sensor.gy_full_scale_set(GyFullScale::_500dps).await?;

    sensor.fifo_watermark_set(FIFO_WATERMARK).await?;
    sensor.fifo_gy_batch_set(FifoBatch::_120hz).await?;
    let mut sflp = FifoSflpRaw::default();
    sflp.game_rotation = 1;
    sflp.gravity = 1;
    sflp.gbias = 1;
    sensor.fifo_sflp_batch_set(sflp).await?;
    sensor.fifo_mode_set(FifoMode::StreamMode).await?;

    let mut route = PinIntRoute::default();
    route.fifo_th = 1;
    sensor.pin_int1_route_set(&route).await?;

    sensor.xl_data_rate_set(Odr::_120hz).await?;
    sensor.gy_data_rate_set(Odr::_120hz).await?;
    sensor.sflp_data_rate_set(SflpDataRate::_120hz).await?;
    // The game rotation vector estimates gyro bias itself; start it from zero.
    sensor.sflp_game_gbias_set(&SflpGbias::default()).await?;
    sensor.sflp_game_rotation_set(1).await?;
    Ok(())
}

/// Reads every entry in the FIFO into the snapshot.
async fn drain_fifo<B: BusOperation, T: DelayNs>(
    sensor: &mut Lsm6dsv16x<B, T, MainBank>,
    snap: &mut Snapshot,
) -> Result<(), Error<B::Error>> {
    let status = sensor.fifo_status_get().await?;
    if status.fifo_ovr == 1 {
        snap.fifo_overruns += 1;
    }
    for _ in 0..status.fifo_level {
        let raw = sensor.fifo_out_raw_get().await?;
        let d = raw.data;
        let w = |i: usize| i16::from_le_bytes([d[i], d[i + 1]]);
        match raw.tag {
            Tag::GyNcTag => {
                snap.block[0..6].copy_from_slice(&d);
                snap.gyro_samples += 1;
            }
            Tag::SflpGameRotationVectorTag => {
                // Three half floats, w implied positive: copied as is.
                snap.block[6..12].copy_from_slice(&d);
                snap.rotation_samples += 1;
            }
            Tag::SflpGravityVectorTag => {
                snap.gravity_mg = [w(0), w(2), w(4)].map(from_sflp_to_mg);
            }
            Tag::SflpGyroscopeBiasTag => {
                snap.gbias_mdps = [w(0), w(2), w(4)].map(from_fs125_to_mdps);
            }
            _ => {}
        }
    }
    Ok(())
}

/// Game rotation half floats to a unit quaternion (x, y, z, w), w positive.
fn quaternion(block: &[u8; 12]) -> [f32; 4] {
    let h = |i: usize| from_half_to_single_precision(u16::from_le_bytes([block[i], block[i + 1]]));
    let (mut x, mut y, mut z) = (h(6), h(8), h(10));
    let mut sumsq = x * x + y * y + z * z;
    if sumsq > 1.0 {
        let n = libm::sqrtf(sumsq);
        (x, y, z) = (x / n, y / n, z / n);
        sumsq = 1.0;
    }
    [x, y, z, libm::sqrtf(1.0 - sumsq)]
}

/// Roll, pitch and yaw in degrees (ZYX), for reading the quaternion by eye.
fn euler_deg([x, y, z, w]: [f32; 4]) -> [f32; 3] {
    let roll = libm::atan2f(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
    let pitch = libm::asinf((2.0 * (w * y - z * x)).clamp(-1.0, 1.0));
    let yaw = libm::atan2f(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
    [roll, pitch, yaw].map(|r| r * 180.0 / core::f32::consts::PI)
}

async fn report<'d>(
    class: &mut CdcAcmClass<'d, Driver<'d, peripherals::USB>>,
    shared: &Cell<Snapshot>,
    led: &mut Output<'_>,
) -> Result<(), EndpointError> {
    let mut line = Line::new();
    let _ = write!(line, "\r\nDukki bridge IMU fusion test (lsm6dsv16x-rs, SFLP game rotation)\r\n");
    write_line(class, &line).await?;
    loop {
        Timer::after_millis(100).await;
        led.toggle();
        let s = shared.get();
        let mut line = Line::new();
        match s.status {
            Status::Starting => {
                let _ = write!(line, "starting\r\n");
            }
            Status::NotFound => {
                let _ = write!(line, "WHO_AM_I = 0x{:02X}, expected 0x{:02X}\r\n", s.who_am_i, ID);
            }
            Status::ConfigFailed => {
                let _ = write!(line, "configuration failed (SPI error)\r\n");
            }
            Status::BusError | Status::Running => {
                let q = quaternion(&s.block);
                let e = euler_deg(q);
                let g = |i: usize| i16::from_le_bytes([s.block[i], s.block[i + 1]]) as f32 * 0.0175;
                let _ = write!(
                    line,
                    "q {:+.4} {:+.4} {:+.4} {:+.4}  rpy {:+7.2} {:+7.2} {:+7.2}  grav mg {:+6.0} {:+6.0} {:+6.0}  \
                     gyro dps {:+7.2} {:+7.2} {:+7.2}  bias mdps {:+6.0} {:+6.0} {:+6.0}  \
                     n {}/{}  ovr {}  int1 to {}{}\r\n",
                    q[0], q[1], q[2], q[3], e[0], e[1], e[2],
                    s.gravity_mg[0], s.gravity_mg[1], s.gravity_mg[2],
                    g(0), g(2), g(4),
                    s.gbias_mdps[0], s.gbias_mdps[1], s.gbias_mdps[2],
                    s.gyro_samples, s.rotation_samples, s.fifo_overruns, s.int1_timeouts,
                    if s.status == Status::BusError { "  BUS ERROR" } else { "" }
                );
            }
        }
        write_line(class, &line).await?;
    }
}

/// Writes a line in packets of at most 64 bytes, the CDC endpoint size.
async fn write_line<'d>(
    class: &mut CdcAcmClass<'d, Driver<'d, peripherals::USB>>,
    line: &Line,
) -> Result<(), EndpointError> {
    for chunk in line.as_bytes().chunks(64) {
        class.write_packet(chunk).await?;
    }
    // A full final packet needs a zero-length packet to end the transfer.
    if line.len % 64 == 0 {
        class.write_packet(&[]).await?;
    }
    Ok(())
}
