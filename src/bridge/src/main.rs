//! Dukki servo bridge firmware for the WeAct STM32G474 core board (QFN48,
//! STM32G474CEU6).
//!
//! For now this is a bench test of the trunk IMU: it reads the LSM6DSV16X over
//! SPI3 (hardware.md section 5.8), waits on INT1 for data ready, and prints the
//! readings at 10 Hz on a USB CDC serial port. The blue LED on PC6 toggles with
//! each printed line, or blinks fast if the IMU does not answer.

#![no_std]
#![no_main]

use core::fmt::Write;

use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_stm32::exti::{self, ExtiInput};
use embassy_stm32::gpio::{Level, Output, Pull, Speed};
use embassy_stm32::rcc::{self, mux};
use embassy_stm32::spi::{self, Spi};
use embassy_stm32::time::Hertz;
use embassy_stm32::usb::{self, Driver};
use embassy_stm32::{bind_interrupts, interrupt, peripherals};
use embassy_time::{with_timeout, Duration, Timer};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::Builder;
use panic_halt as _;

bind_interrupts!(struct Irqs {
    USB_LP => usb::InterruptHandler<peripherals::USB>;
    EXTI9_5 => exti::InterruptHandler<interrupt::typelevel::EXTI9_5>;
});

// LSM6DSV16X registers
const IF_CFG: u8 = 0x03;
const INT1_CTRL: u8 = 0x0D;
const WHO_AM_I: u8 = 0x0F;
const CTRL1: u8 = 0x10;
const CTRL2: u8 = 0x11;
const CTRL3: u8 = 0x12;
const CTRL6: u8 = 0x15;
const CTRL8: u8 = 0x17;
const OUT_TEMP_L: u8 = 0x20;
const WHO_AM_I_VALUE: u8 = 0x70;

// Accel at +-4 g (0.122 mg/LSB), gyro at +-2000 dps (70 mdps/LSB), both at 120 Hz.
const ACCEL_MG_PER_LSB: f32 = 0.122;
const GYRO_MDPS_PER_LSB: f32 = 70.0;
const ODR_120HZ: u8 = 0x06;

struct Imu<'d> {
    spi: Spi<'d, embassy_stm32::mode::Blocking, spi::mode::Master>,
    cs: Output<'d>,
}

impl Imu<'_> {
    fn read(&mut self, reg: u8, buf: &mut [u8]) {
        self.cs.set_low();
        let _ = self.spi.blocking_write(&[reg | 0x80]);
        let _ = self.spi.blocking_read(buf);
        self.cs.set_high();
    }

    fn read_reg(&mut self, reg: u8) -> u8 {
        let mut b = [0u8];
        self.read(reg, &mut b);
        b[0]
    }

    fn write_reg(&mut self, reg: u8, value: u8) {
        self.cs.set_low();
        let _ = self.spi.blocking_write(&[reg & 0x7F, value]);
        self.cs.set_high();
    }

    /// Temperature, gyro and accel in one burst from OUT_TEMP_L (0x20 to 0x2D).
    fn read_sample(&mut self) -> Sample {
        let mut b = [0u8; 14];
        self.read(OUT_TEMP_L, &mut b);
        let w = |i: usize| i16::from_le_bytes([b[i], b[i + 1]]);
        Sample {
            temp_c: 25.0 + w(0) as f32 / 256.0,
            gyro_dps: [w(2), w(4), w(6)].map(|v| v as f32 * GYRO_MDPS_PER_LSB / 1000.0),
            accel_g: [w(8), w(10), w(12)].map(|v| v as f32 * ACCEL_MG_PER_LSB / 1000.0),
        }
    }
}

struct Sample {
    temp_c: f32,
    gyro_dps: [f32; 3],
    accel_g: [f32; 3],
}

/// A fixed buffer that `write!` can format into.
struct Line {
    buf: [u8; 192],
    len: usize,
}

impl Line {
    fn new() -> Self {
        Self { buf: [0; 192], len: 0 }
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
    spi_config.frequency = Hertz(4_000_000);
    spi_config.mode = spi::MODE_3;
    let mut imu = Imu {
        spi: Spi::new_blocking(p.SPI3, p.PC10, p.PB5, p.PC11, spi_config),
        // High at boot: the chip stays in I2C mode until CS first goes low.
        cs: Output::new(p.PA15, Level::High, Speed::Medium),
    };
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

    let app = async {
        // Give the chip its 10 ms boot time, then check it is there.
        Timer::after_millis(20).await;
        let who = imu.read_reg(WHO_AM_I);
        let found = who == WHO_AM_I_VALUE;
        if found {
            imu.write_reg(IF_CFG, 0x01); // I2C_I3C_disable: CS glitches cannot drop it back to I2C
            imu.write_reg(CTRL3, 0x44); // BDU, IF_INC
            imu.write_reg(CTRL8, 0x01); // accel +-4 g
            imu.write_reg(CTRL6, 0x04); // gyro +-2000 dps
            imu.write_reg(CTRL1, ODR_120HZ); // accel high-performance, 120 Hz
            imu.write_reg(CTRL2, ODR_120HZ); // gyro high-performance, 120 Hz
            imu.write_reg(INT1_CTRL, 0x01); // INT1_DRDY_XL
        }

        loop {
            class.wait_connection().await;
            let _ = send_banner(&mut class, who, found).await;
            if !found {
                // Nothing to read: blink fast and keep reporting the ID until the port closes.
                loop {
                    led.toggle();
                    Timer::after_millis(100).await;
                    if led.is_set_high() {
                        let who = imu.read_reg(WHO_AM_I);
                        let mut line = Line::new();
                        let _ = write!(line, "WHO_AM_I = 0x{:02X}, expected 0x{:02X}\r\n", who, WHO_AM_I_VALUE);
                        if class.write_packet(line.as_bytes()).await.is_err() {
                            break;
                        }
                    }
                }
                continue;
            }
            let _ = stream(&mut class, &mut imu, &mut int1, &mut led).await;
        }
    };

    join(usb.run(), app).await;
}

async fn send_banner<'d>(
    class: &mut CdcAcmClass<'d, Driver<'d, peripherals::USB>>,
    who: u8,
    found: bool,
) -> Result<(), EndpointError> {
    let mut line = Line::new();
    let _ = write!(
        line,
        "\r\nDukki bridge IMU test. WHO_AM_I = 0x{:02X} ({})\r\n",
        who,
        if found { "LSM6DSV16X found" } else { "no LSM6DSV16X" }
    );
    write_line(class, &line).await
}

async fn stream<'d>(
    class: &mut CdcAcmClass<'d, Driver<'d, peripherals::USB>>,
    imu: &mut Imu<'_>,
    int1: &mut ExtiInput<'_, embassy_stm32::mode::Async>,
    led: &mut Output<'_>,
) -> Result<(), EndpointError> {
    let mut samples: u32 = 0;
    let mut int1_timeouts: u32 = 0;
    loop {
        // Data ready on INT1 at 120 Hz. A timeout means INT1 is not wired or not firing;
        // read anyway so the rest of the chain can still be checked.
        if with_timeout(Duration::from_millis(50), int1.wait_for_high()).await.is_err() {
            int1_timeouts += 1;
        }
        let s = imu.read_sample();
        samples += 1;
        // Wait for INT1 to fall again: reading the accel output clears data ready.
        let _ = with_timeout(Duration::from_millis(5), int1.wait_for_low()).await;

        if samples % 12 == 0 {
            led.toggle();
            let mut line = Line::new();
            let _ = write!(
                line,
                "acc g {:+6.3} {:+6.3} {:+6.3}  gyro dps {:+8.2} {:+8.2} {:+8.2}  temp {:5.1} C  n {}  int1 timeouts {}\r\n",
                s.accel_g[0], s.accel_g[1], s.accel_g[2],
                s.gyro_dps[0], s.gyro_dps[1], s.gyro_dps[2],
                s.temp_c, samples, int1_timeouts
            );
            write_line(class, &line).await?;
        }
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
