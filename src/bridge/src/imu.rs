//! Trunk IMU: the LSM6DSV16X through ST's `lsm6dsv16x-rs` driver, configured
//! for the block the compute side decodes (hardware.md section 6.1): gyro at
//! +-500 dps and the SFLP game rotation vector, both at 120 Hz through the
//! chip's FIFO, with INT1 on the FIFO threshold.

use core::cell::Cell;

use embassy_stm32::exti::ExtiInput;
use embassy_stm32::mode::Async;
use embassy_time::{with_timeout, Duration, Instant, Timer};
use embedded_hal_async::delay::DelayNs;
use lsm6dsv16x_rs::asynchronous as lsm6dsv16x;
use lsm6dsv16x::prelude::*;
use st_mems_bus::asynchronous::BusOperation;

/// FIFO entries per INT1: at 120 Hz each for gyro, game rotation, gravity and
/// gyro bias the FIFO fills at 480 entries/s, so 4 entries is about 8 ms.
const FIFO_WATERMARK: u8 = 4;

/// What the IMU side has seen, shared with the reporter.
#[derive(Clone, Copy, Default)]
pub struct Snapshot {
    pub who_am_i: u8,
    pub status: Status,
    /// The block the compute side decodes: gyro x, y, z as i16 raw counts at
    /// +-500 dps, then the game rotation x, y, z as half floats. All-zero
    /// quaternion bytes until the first fused sample.
    pub block: [u8; 12],
    pub gravity_mg: [f32; 3],
    pub gbias_mdps: [f32; 3],
    /// Bridge clock at the last FIFO drain, µs.
    pub drained_us: u64,
    pub gyro_samples: u32,
    pub rotation_samples: u32,
    pub fifo_overruns: u32,
    pub int1_timeouts: u32,
}

#[derive(Clone, Copy, Default, PartialEq)]
pub enum Status {
    #[default]
    Starting,
    NotFound,
    ConfigFailed,
    BusError,
    Running,
}

/// Finds and configures the chip, then drains its FIFO on every INT1 forever,
/// publishing to `shared`.
pub async fn run<B: BusOperation, T: DelayNs>(
    mut sensor: Lsm6dsv16x<B, T, MainBank>,
    int1: &mut ExtiInput<'_, Async>,
    shared: &Cell<Snapshot>,
) {
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
        snap.drained_us = Instant::now().as_micros();
        shared.set(snap);
    }
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
pub fn quaternion(block: &[u8; 12]) -> [f32; 4] {
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
pub fn euler_deg([x, y, z, w]: [f32; 4]) -> [f32; 3] {
    let roll = libm::atan2f(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
    let pitch = libm::asinf((2.0 * (w * y - z * x)).clamp(-1.0, 1.0));
    let yaw = libm::atan2f(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
    [roll, pitch, yaw].map(|r| r * 180.0 / core::f32::consts::PI)
}

/// The protocol's view of the IMU.
pub fn imu_state(s: &Snapshot, now_us: u64) -> duck_bridge_proto::ImuState {
    use duck_bridge_proto::imu_status;
    duck_bridge_proto::ImuState {
        status: match s.status {
            Status::Starting => imu_status::STARTING,
            Status::NotFound => imu_status::NOT_FOUND,
            Status::ConfigFailed => imu_status::CONFIG_FAILED,
            Status::BusError => imu_status::BUS_ERROR,
            Status::Running => imu_status::RUNNING,
        },
        block: s.block,
        age_us: now_us.saturating_sub(s.drained_us).min(u32::MAX as u64) as u32,
        gravity_mg: s.gravity_mg,
        gbias_mdps: s.gbias_mdps,
        gyro_samples: s.gyro_samples,
        rotation_samples: s.rotation_samples,
        fifo_overruns: s.fifo_overruns as u16,
        int1_timeouts: s.int1_timeouts as u16,
    }
}
