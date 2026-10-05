//! Unitree J288 / S288 servo protocol: 20-byte command frames, 26-byte replies,
//! Unitree's CRC32.
//!
//! A port of `bam/unitree/digital_servo.py`, which was checked against
//! Unitree's own demo (j288-testing.md). Frame fields are rotor-side integers;
//! this module converts to and from output-side SI units with the exact gear
//! ratio 70070/243. The tests at the bottom compare against frames from the
//! Python driver.

// The whole protocol is here; the bench firmware does not use every field yet.
#![allow(dead_code)]

/// Rotor turns per output turn.
pub const GEAR_RATIO: f32 = 70070.0 / 243.0;

/// Command and reply frame lengths.
pub const COMMAND_LEN: usize = 20;
pub const REPLY_LEN: usize = 26;

// Protocol resolution, output side: kp in 1/1280000 N.m/rad, kd in
// 1/128000000 N.m.s/rad, torque in 1/256000 N.m and speed in 2*pi/2.56 rad/s,
// all rotor-side.
const KP_STEP: f32 = GEAR_RATIO * GEAR_RATIO / 1_280_000.0;
const KD_STEP: f32 = GEAR_RATIO * GEAR_RATIO / 128_000_000.0;
const TORQUE_STEP: f32 = GEAR_RATIO / 256_000.0;
const SPEED_STEP: f32 = 2.0 * core::f32::consts::PI / 2.56 / GEAR_RATIO;
/// Rotor position counts per output radian, in f64: a multi-turn position
/// needs more than f32's 24 bits to round to the same count as the reference.
const POSITION_COUNTS_PER_RAD: f64 =
    (70070.0 / 243.0) * 32768.0 / (2.0 * core::f64::consts::PI);

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(u8)]
pub enum Mode {
    /// Locked: shorts the windings, a viscous brake.
    Stop = 0,
    /// FOC closed loop: tau = tau_ff + kp (p_des - p) + kd (w_des - w).
    Foc = 1,
    /// Clear latched faults (sent with tau = -256 raw, everything else 0).
    ClearFaults = 6,
    /// Reset, about 0.3 s during which commands are ignored.
    Reset = 7,
}

/// A command in output-side SI units.
#[derive(Clone, Copy, Debug)]
pub struct Command {
    pub id: u8,
    pub mode: Mode,
    /// Feed-forward torque [N.m].
    pub torque: f32,
    /// Desired speed [rad/s].
    pub speed: f32,
    /// Desired multi-turn position [rad], relative to the servo's power-up position.
    pub position: f32,
    /// [N.m/rad]
    pub kp: f32,
    /// [N.m.s/rad]
    pub kd: f32,
    /// Timeout protection: the servo stops about 1 s after the last frame and
    /// latches until a frame arrives with this bit clear.
    pub timeout: bool,
}

impl Command {
    pub const fn stop(id: u8, timeout: bool) -> Self {
        Self { id, mode: Mode::Stop, torque: 0.0, speed: 0.0, position: 0.0, kp: 0.0, kd: 0.0, timeout }
    }
}

/// A reply in output-side SI units. It reports the state from before the
/// command it answers was applied.
#[derive(Clone, Copy, Default, Debug)]
pub struct State {
    pub id: u8,
    pub mode: u8,
    pub timeout: bool,
    /// Housing temperature [°C].
    pub temp: i8,
    /// Winding temperature [°C]. Rises far faster than the housing.
    pub winding_temp: u8,
    /// Supply voltage [V], 0.5 V steps.
    pub volts: f32,
    /// Motor torque from the current, before gearbox losses [N.m].
    pub torque: f32,
    /// The servo's filtered speed estimate, about 15 ms lag [rad/s].
    pub speed: f32,
    /// Multi-turn position from the rotor encoder [rad].
    pub position: f32,
    /// 13-bit output encoder, 0 to 2 pi [rad]. Only trustworthy at rest.
    pub output_encoder: f32,
    /// The same, as the raw 13-bit count.
    pub output_encoder_raw: u16,
    pub error: u32,
    pub warning: u8,
}

/// Unitree's CRC32: MSB first, polynomial 0x04C11DB7, initial value
/// 0xFFFFFFFF, no final xor, fed one little-endian 32-bit word at a time
/// (high byte first). The data length must be a multiple of four.
pub fn crc32(data: &[u8]) -> u32 {
    let mut crc: u32 = 0xFFFF_FFFF;
    for word in data.chunks_exact(4) {
        for &b in [word[3], word[2], word[1], word[0]].iter() {
            crc ^= (b as u32) << 24;
            for _ in 0..8 {
                crc = if crc & 0x8000_0000 != 0 { (crc << 1) ^ 0x04C1_1DB7 } else { crc << 1 };
            }
        }
    }
    crc
}

fn clamp_round(x: f32, lo: i32, hi: i32) -> i32 {
    // Round half away from zero; the reference uses round half to even, which
    // differs only on exact halves.
    let r = (if x >= 0.0 { x + 0.5 } else { x - 0.5 }) as i64;
    r.clamp(lo as i64, hi as i64) as i32
}

pub fn encode(cmd: &Command) -> [u8; COMMAND_LEN] {
    let mut f = [0u8; COMMAND_LEN];
    f[0] = 0xFE;
    f[1] = 0xEE;
    f[2] = (cmd.id & 0x0F) | ((cmd.mode as u8 & 0x07) << 4) | ((cmd.timeout as u8) << 7);
    f[3] = 0;
    let (tor, spd, pos, kp, kd) = if cmd.mode == Mode::ClearFaults {
        (-256, 0, 0, 0, 0)
    } else {
        let pos = cmd.position as f64 * POSITION_COUNTS_PER_RAD;
        let pos = (if pos >= 0.0 { pos + 0.5 } else { pos - 0.5 }) as i64;
        (
            clamp_round(cmd.torque / TORQUE_STEP, -32768, 32767),
            clamp_round(cmd.speed / SPEED_STEP, -32768, 32767),
            pos.clamp(i32::MIN as i64, i32::MAX as i64) as i32,
            clamp_round(cmd.kp / KP_STEP, 0, 32767),
            clamp_round(cmd.kd / KD_STEP, 0, 32767),
        )
    };
    f[4..6].copy_from_slice(&(tor as i16).to_le_bytes());
    f[6..8].copy_from_slice(&(spd as i16).to_le_bytes());
    f[8..12].copy_from_slice(&pos.to_le_bytes());
    f[12..14].copy_from_slice(&(kp as i16).to_le_bytes());
    f[14..16].copy_from_slice(&(kd as i16).to_le_bytes());
    let crc = crc32(&f[..16]);
    f[16..20].copy_from_slice(&crc.to_le_bytes());
    f
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum ParseError {
    Header,
    Crc,
}

/// Parses a reply. The CRC covers bytes 2 to 21, not the header.
pub fn decode(f: &[u8; REPLY_LEN]) -> Result<State, ParseError> {
    if f[0] != 0xFC || f[1] != 0xEE {
        return Err(ParseError::Header);
    }
    if u32::from_le_bytes([f[22], f[23], f[24], f[25]]) != crc32(&f[2..22]) {
        return Err(ParseError::Crc);
    }
    let i16_at = |i: usize| i16::from_le_bytes([f[i], f[i + 1]]);
    let pos = i32::from_le_bytes([f[10], f[11], f[12], f[13]]);
    let out = u16::from_le_bytes([f[18], f[19]]);
    Ok(State {
        id: f[2] & 0x0F,
        mode: (f[2] >> 4) & 0x07,
        timeout: f[2] >> 7 != 0,
        temp: f[3] as i8,
        winding_temp: f[4],
        volts: f[5] as f32 / 2.0,
        torque: i16_at(6) as f32 * TORQUE_STEP,
        speed: i16_at(8) as f32 * SPEED_STEP,
        position: (pos as f64 / POSITION_COUNTS_PER_RAD) as f32,
        output_encoder: (out & 0x1FFF) as f32 * 2.0 * core::f32::consts::PI / 8192.0,
        output_encoder_raw: out & 0x1FFF,
        error: u32::from_le_bytes([f[14], f[15], f[16], f[17]]),
        warning: ((out >> 13) & 0x07) as u8,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn hex(s: &str) -> Vec<u8> {
        (0..s.len()).step_by(2).map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap()).collect()
    }

    fn cmd(id: u8, mode: Mode, torque: f32, speed: f32, position: f32, kp: f32, kd: f32, timeout: bool) -> Command {
        Command { id, mode, torque, speed, position, kp, kd, timeout }
    }

    // Frames from bam/unitree/digital_servo.py build_command and clear_faults_command.
    #[test]
    fn commands_match_reference() {
        assert_eq!(encode(&Command::stop(0, true)).to_vec(), hex("feee80000000000000000000000000001d99fb6e"));
        assert_eq!(
            encode(&cmd(3, Mode::Foc, 0.1, -0.5, 1.2345, 0.5, 0.05, true)).to_vec(),
            hex("feee93005900c5ffd2531c0008004d00b4071e78")
        );
        assert_eq!(
            encode(&cmd(14, Mode::Foc, -0.2, 2.0, -3.0, 2.0, 0.01, false)).to_vec(),
            hex("feee1e004effeb001b29bbff1f000f0020727b59")
        );
        assert_eq!(
            encode(&cmd(0, Mode::ClearFaults, 0.0, 0.0, 0.0, 0.0, 0.0, false)).to_vec(),
            hex("feee600000ff0000000000000000000043fbe01e")
        );
    }

    // A reply built and parsed by the Python driver: id 3, mode 1, timeout set,
    // 31 °C, winding 40 °C, 24.5 V, torque 100 raw, speed -50 raw, position
    // 123456 raw, output encoder 4096, warning 1.
    #[test]
    fn reply_matches_reference() {
        let f: [u8; REPLY_LEN] = hex("fcee931f28316400ceff40e201000000000000300000236751fd").try_into().unwrap();
        let s = decode(&f).unwrap();
        assert_eq!((s.id, s.mode, s.timeout, s.temp, s.winding_temp), (3, 1, true, 31, 40));
        assert_eq!((s.volts, s.error, s.warning), (24.5, 0, 1));
        assert!((s.torque - 0.112_638_25).abs() < 1e-6);
        assert!((s.speed + 0.425_582_8).abs() < 1e-6);
        assert!((s.position - 0.082_094_92).abs() < 1e-6);
        assert!((s.output_encoder - core::f32::consts::PI).abs() < 1e-6);

        let mut bad = f;
        bad[10] ^= 1;
        assert_eq!(decode(&bad).unwrap_err(), ParseError::Crc);
        bad[0] = 0;
        assert_eq!(decode(&bad).unwrap_err(), ParseError::Header);
    }
}
