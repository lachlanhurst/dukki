//! Dukki servo bridge firmware for the WeAct STM32G474 core board (QFN48,
//! STM32G474CEU6).
//!
//! For now a bench test of everything on the bridge (hardware.md section 5.8):
//! the trunk IMU with SFLP fusion on SPI3, and one J288 servo on each of the
//! three bus segments (A: USART2 on PB3, B: USART3 on PB9, C: LPUART1 on PA2),
//! single-wire at 6 Mbps, each transaction a polled register loop. A USB CDC
//! serial console reports both at 5 Hz and takes single-key commands that set
//! the servos' test mode. The blue LED on PC6 toggles with each report.

#![no_std]
#![no_main]

mod imu;
mod j288;
mod segment;

use core::cell::Cell;
use core::fmt::Write;

use embassy_executor::Spawner;
use embassy_futures::join::{join, join3, join4};
use embassy_stm32::exti::{self, ExtiInput};
use embassy_stm32::gpio::{Level, Output, Pull, Speed};
use embassy_stm32::rcc::{self, mux};
use embassy_stm32::spi::{self, Spi};
use embassy_stm32::time::Hertz;
use embassy_stm32::usart::{self, HalfDuplexReadback, OutputConfig, Uart};
use embassy_stm32::usb::{self, Driver};
use embassy_stm32::{bind_interrupts, dma, interrupt, peripherals};
use embassy_time::{Delay, Timer};
use embassy_usb::class::cdc_acm::{CdcAcmClass, Receiver, Sender, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::Builder;
use embedded_hal_bus::spi::ExclusiveDevice;
use lsm6dsv16x_rs::asynchronous::prelude::Lsm6dsv16x;
use panic_halt as _;

use segment::{Control, Raw, Stats, TestMode};

bind_interrupts!(struct Irqs {
    USB_LP => usb::InterruptHandler<peripherals::USB>;
    EXTI9_5 => exti::InterruptHandler<interrupt::typelevel::EXTI9_5>;
    USART2 => usart::InterruptHandler<peripherals::USART2>;
    USART3 => usart::InterruptHandler<peripherals::USART3>;
    LPUART1 => usart::InterruptHandler<peripherals::LPUART1>;
    DMA1_CHANNEL1 => dma::InterruptHandler<peripherals::DMA1_CH1>;
    DMA1_CHANNEL2 => dma::InterruptHandler<peripherals::DMA1_CH2>;
    DMA1_CHANNEL3 => dma::InterruptHandler<peripherals::DMA1_CH3>;
    DMA1_CHANNEL4 => dma::InterruptHandler<peripherals::DMA1_CH4>;
    DMA1_CHANNEL5 => dma::InterruptHandler<peripherals::DMA1_CH5>;
    DMA1_CHANNEL6 => dma::InterruptHandler<peripherals::DMA1_CH6>;
    DMA1_CHANNEL7 => dma::InterruptHandler<peripherals::DMA1_CH7>;
    DMA1_CHANNEL8 => dma::InterruptHandler<peripherals::DMA1_CH8>;
});

/// Servo ID on every segment: all three test servos are still at the factory ID.
const SERVO_ID: u8 = 0;
const SEGMENT_NAMES: [&str; 3] = ["A", "B", "C"];

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
    // 168 MHz from the 8 MHz crystal (PLL 8 / 1 x 42 / 2): every USART divider is an
    // integer at 6 Mbps (hardware.md section 5.3). Above 150 MHz needs boost mode.
    config.rcc.hse = Some(rcc::Hse { freq: Hertz(8_000_000), mode: rcc::HseMode::Oscillator });
    config.rcc.pll = Some(rcc::Pll {
        source: rcc::PllSource::HSE,
        prediv: rcc::PllPreDiv::DIV1,
        mul: rcc::PllMul::MUL42,
        divp: None,
        divq: None,
        divr: Some(rcc::PllRDiv::DIV2),
    });
    config.rcc.sys = rcc::Sysclk::PLL1_R;
    config.rcc.boost = true;
    // USB runs from HSI48, trimmed by the CRS against the host's start-of-frame packets.
    config.rcc.hsi48 = Some(rcc::Hsi48Config { sync_from_usb: true });
    config.rcc.mux.clk48sel = mux::Clk48sel::HSI48;
    let p = embassy_stm32::init(config);

    let mut led = Output::new(p.PC6, Level::Low, Speed::Low);

    // Trunk IMU on SPI3.
    let mut spi_config = spi::Config::default();
    spi_config.frequency = Hertz(8_000_000);
    spi_config.mode = spi::MODE_3;
    let spi = Spi::new(p.SPI3, p.PC10, p.PB5, p.PC11, p.DMA1_CH1, p.DMA1_CH2, Irqs, spi_config);
    // High at boot: the chip stays in I2C mode until CS first goes low.
    let cs = Output::new(p.PA15, Level::High, Speed::Medium);
    let sensor = Lsm6dsv16x::new_spi(ExclusiveDevice::new_no_delay(spi, cs).unwrap(), Delay);
    let mut int1 = ExtiInput::new(p.PA8, p.EXTI8, Pull::Down, Irqs);

    // Servo segments: single-wire half duplex on the TX pin, open drain with the
    // segment's external pull-up (hardware.md section 5.8, direct drive). No internal
    // pull-up: each J288 pulls the line towards 5 V itself, and an FT pin only
    // sustains more than 4 V with its internal pull resistors off (DS12288 table 14).
    let mut uart_config = usart::Config::default();
    uart_config.baudrate = 6_000_000;
    uart_config.tx_config = OutputConfig::OpenDrain;
    // Readback: each transaction reads its own echo and the reply in one transfer.
    let rb = HalfDuplexReadback::Readback;
    let mut seg_a =
        Uart::new_half_duplex(p.USART2, p.PB3, p.DMA1_CH3, p.DMA1_CH4, Irqs, uart_config, rb).unwrap();
    let mut seg_b =
        Uart::new_half_duplex(p.USART3, p.PB9, p.DMA1_CH5, p.DMA1_CH6, Irqs, uart_config, rb).unwrap();
    let mut seg_c =
        Uart::new_half_duplex(p.LPUART1, p.PA2, p.DMA1_CH7, p.DMA1_CH8, Irqs, uart_config, rb).unwrap();

    // USB CDC ACM serial console.
    let driver = Driver::new(p.USB, Irqs, p.PA12, p.PA11);
    let mut usb_config = embassy_usb::Config::new(0x1209, 0x0001);
    usb_config.manufacturer = Some("Dukki");
    usb_config.product = Some("Dukki bridge");
    usb_config.serial_number = Some("bridge-bench");
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
    let class = CdcAcmClass::new(&mut builder, &mut state, 64);
    let mut usb = builder.build();
    let (mut tx, mut rx) = class.split();

    let imu_shared = Cell::new(imu::Snapshot::default());
    let control = Cell::new(Control::default());
    let seg_shared = [Cell::new(Stats::default()), Cell::new(Stats::default()), Cell::new(Stats::default())];

    let servos = join3(
        // Raw: the base addresses of USART2, USART3 and LPUART1 for the polled loop.
        segment::run(&mut seg_a, SERVO_ID, &control, &seg_shared[0], Raw(0x4000_4400)),
        segment::run(&mut seg_b, SERVO_ID, &control, &seg_shared[1], Raw(0x4000_4800)),
        segment::run(&mut seg_c, SERVO_ID, &control, &seg_shared[2], Raw(0x4000_8000)),
    );
    let console = join(
        async {
            loop {
                tx.wait_connection().await;
                let _ = report(&mut tx, &imu_shared, &seg_shared, &control, &mut led).await;
            }
        },
        async {
            loop {
                rx.wait_connection().await;
                let _ = commands(&mut rx, &control).await;
            }
        },
    );

    join4(usb.run(), imu::run(sensor, &mut int1, &imu_shared), servos, console).await;
}

/// Single-key commands from the console.
async fn commands<'d>(
    rx: &mut Receiver<'d, Driver<'d, peripherals::USB>>,
    control: &Cell<Control>,
) -> Result<(), EndpointError> {
    let mut buf = [0u8; 64];
    loop {
        let n = rx.read_packet(&mut buf).await?;
        for &key in &buf[..n] {
            if key == b'p' {
                let c = control.get();
                control.set(Control { polled: !c.polled, ..c });
                continue;
            }
            let mode = match key {
                b's' => TestMode::Stop,
                b'd' => TestMode::Damp,
                b'h' => TestMode::Hold,
                b'w' => TestMode::Wiggle,
                b'c' => TestMode::ClearFaults,
                _ => continue,
            };
            let c = control.get();
            control.set(Control { mode, generation: c.generation.wrapping_add(1), ..c });
        }
    }
}

fn mode_name(mode: TestMode) -> &'static str {
    match mode {
        TestMode::Stop => "stop",
        TestMode::Damp => "damp",
        TestMode::Hold => "hold",
        TestMode::Wiggle => "wiggle",
        TestMode::ClearFaults => "clear faults",
    }
}

async fn report<'d>(
    tx: &mut Sender<'d, Driver<'d, peripherals::USB>>,
    imu_shared: &Cell<imu::Snapshot>,
    seg_shared: &[Cell<Stats>; 3],
    control: &Cell<Control>,
    led: &mut Output<'_>,
) -> Result<(), EndpointError> {
    let mut line = Line::new();
    let _ = write!(
        line,
        "\r\nDukki bridge bench test. Keys: s stop, d damp, h hold, w wiggle, c clear faults, p polled or DMA bus\r\n"
    );
    write_line(tx, &line).await?;
    let mut tick: u32 = 0;
    loop {
        Timer::after_millis(200).await;
        tick += 1;
        led.toggle();

        let mut line = Line::new();
        let c = control.get();
        let _ = write!(
            line,
            "-- mode {}  bus {}\r\n",
            mode_name(c.mode),
            if c.polled { "polled" } else { "DMA" }
        );
        write_line(tx, &line).await?;

        for (name, cell) in SEGMENT_NAMES.iter().zip(seg_shared) {
            let s = cell.get();
            let mut line = Line::new();
            let ok = if s.sent > 0 { s.replies as f32 * 100.0 / s.sent as f32 } else { 0.0 };
            let _ = write!(
                line,
                "{} ok {:5.1}% ({} sent, {} none, {} bad, {} uart: {} noise {} framing {} overrun, {} echo)  rt {}/{} us",
                name, ok, s.sent, s.no_reply, s.bad_frames, s.uart_errors, s.noise, s.framing, s.overrun, s.echo_mismatch,
                s.round_trip_us, s.round_trip_max_us
            );
            match s.last {
                Some(st) => {
                    let _ = write!(
                        line,
                        "  pos {:+7.3} spd {:+6.2} tq {:+6.3}  {:4.1} V  {}/{} C  mode {} to {} err {:#x} warn {}",
                        st.position, st.speed, st.torque, st.volts, st.temp, st.winding_temp,
                        st.mode, st.timeout as u8, st.error, st.warning
                    );
                }
                None => {
                    let _ = write!(line, "  no reply yet");
                }
            }
            if s.recovering {
                let _ = write!(line, "  [clearing timeout]");
            }
            if s.speed_trips > 0 {
                let _ = write!(line, "  speed trips {}", s.speed_trips);
            }
            let _ = write!(line, "\r\n");
            write_line(tx, &line).await?;
        }

        if tick % 5 == 0 {
            let s = imu_shared.get();
            let mut line = Line::new();
            match s.status {
                imu::Status::Running | imu::Status::BusError => {
                    let e = imu::euler_deg(imu::quaternion(&s.block));
                    let _ = write!(
                        line,
                        "IMU rpy {:+7.2} {:+7.2} {:+7.2}  n {}/{}  ovr {}  int1 to {}{}\r\n",
                        e[0], e[1], e[2], s.gyro_samples, s.rotation_samples, s.fifo_overruns, s.int1_timeouts,
                        if s.status == imu::Status::BusError { "  BUS ERROR" } else { "" }
                    );
                }
                imu::Status::Starting => {
                    let _ = write!(line, "IMU starting\r\n");
                }
                imu::Status::NotFound => {
                    let _ = write!(line, "IMU WHO_AM_I 0x{:02X}, not found\r\n", s.who_am_i);
                }
                imu::Status::ConfigFailed => {
                    let _ = write!(line, "IMU configuration failed\r\n");
                }
            }
            write_line(tx, &line).await?;
        }
    }
}

/// Writes a line in packets of at most 64 bytes, the CDC endpoint size.
async fn write_line<'d>(
    tx: &mut Sender<'d, Driver<'d, peripherals::USB>>,
    line: &Line,
) -> Result<(), EndpointError> {
    for chunk in line.as_bytes().chunks(64) {
        tx.write_packet(chunk).await?;
    }
    // A full final packet needs a zero-length packet to end the transfer.
    if line.len % 64 == 0 {
        tx.write_packet(&[]).await?;
    }
    Ok(())
}
