//! LIS2DH / LIS2DH12 accelerometer over I2C.
//!
//! Purpose here is motion gating, not measurement: a threshold interrupt on
//! INT1 keeps the MCU asleep and lets the sensor decide when to wake it. The
//! device is never placed in high-performance mode - low-power 10 Hz is
//! ~3 µA and there is no reason to spend more on a boolean.
//!
//! Generic over the I2C bus so host tests can drive it with a fake register
//! model (same approach as the C++ test suite).

use embedded_hal::i2c::I2c;

pub const WHO_AM_I: u8 = 0x0F;
pub const CTRL_REG1: u8 = 0x10;
pub const CTRL_REG2: u8 = 0x11;
pub const CTRL_REG4: u8 = 0x1F;
pub const CTRL_REG5: u8 = 0x20;
pub const STATUS_REG: u8 = 0x1E;
pub const INT1_CFG: u8 = 0x09;
pub const INT1_SRC: u8 = 0x0D;
pub const INT1_THS: u8 = 0x07;
pub const INT1_DUR: u8 = 0x08;
pub const OUT_X_L: u8 = 0x28;

/// CTRL_REG1: LPen=1 low-power, BDU=1 block update, XYZ_EN=111, ODR=10 Hz -
/// the lowest rate that still gates reliably.
pub const CTRL1_LP_10HZ: u8 = 0x2A;
/// CTRL_REG4: ±2 g so a chair bump is in range without clipping.
pub const CTRL4_2G: u8 = 0x08;
/// INT1_CFG: latch on any of the three axes.
pub const INT1_ANY_AXIS: u8 = 0xFD;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error<B> {
    Bus(B),
    Timeout,
}

pub struct Lis2dh<B> {
    bus: B,
    addr: u8,
}

impl<B: I2c> Lis2dh<B> {
    pub fn new(bus: B, addr: u8) -> Self {
        Self { bus, addr }
    }

    pub fn bus(&mut self) -> &mut B {
        &mut self.bus
    }

    fn write_reg(&mut self, reg: u8, val: u8) -> Result<(), B::Error> {
        self.bus.write(self.addr, &[reg, val])
    }

    fn read_reg(&mut self, reg: u8) -> Result<u8, B::Error> {
        use embedded_hal::i2c::Operation;
        let mut out = [0u8; 1];
        self.bus.transaction(self.addr, &mut [
            Operation::Write(&[reg]),
            Operation::Read(&mut out),
        ])?;
        Ok(out[0])
    }

    /// True when WHO_AM_I matches a known LIS2DH family value.
    pub fn probe(&mut self) -> Result<bool, B::Error> {
        let who = self.read_reg(WHO_AM_I)?;
        Ok(matches!(who, 0x32 | 0x33 | 0x2A))
    }

    /// Low-power 10 Hz mode with a threshold interrupt on INT1.
    ///
    /// `threshold` is in 1/16 g units (0x10 = 1 g). Start at 0x08 (0.5 g) and
    /// tune against the real chair - below ~0x04, floor vibration
    /// false-triggers constantly. `duration` = consecutive samples before
    /// firing, 0 = 1.
    pub fn configure(&mut self, threshold: u8, duration: u8) -> Result<(), B::Error> {
        self.write_reg(CTRL_REG1, CTRL1_LP_10HZ)?;
        self.write_reg(CTRL_REG4, CTRL4_2G)?;
        self.write_reg(CTRL_REG2, 0x00)?; // no high-pass filter
        self.write_reg(INT1_CFG, INT1_ANY_AXIS)?;
        self.write_reg(INT1_THS, threshold)?;
        self.write_reg(INT1_DUR, duration)?;
        self.write_reg(CTRL_REG5, 0x00)?; // INT1 routed to the INT1 pad
        Ok(())
    }

    fn data_ready(&mut self) -> Result<bool, B::Error> {
        Ok(self.read_reg(STATUS_REG)? & 0x08 != 0)
    }

    /// Reads acceleration in mg. Polls the status register at most
    /// `max_polls` times; each poll is one I2C round trip (~100 µs), so
    /// 20_000 is a few seconds worst case.
    pub fn accel(&mut self, max_polls: u32) -> Result<[i16; 3], Error<B::Error>> {
        for _ in 0..max_polls {
            if self.data_ready().map_err(Error::Bus)? {
                return self.read_accel().map_err(Error::Bus);
            }
        }
        Err(Error::Timeout)
    }

    fn read_accel(&mut self) -> Result<[i16; 3], B::Error> {
        use embedded_hal::i2c::Operation;
        let mut b = [0u8; 6];
        self.bus.transaction(self.addr, &mut [
            Operation::Write(&[OUT_X_L]),
            Operation::Read(&mut b),
        ])?;

        // 12-bit two's complement, left-justified in each 16-bit word; 1
        // mg/LSB in the ±2 g range. Sign extension must come from bit 11 of
        // the 12-bit value, not from the 16-bit word: shifting -1000 mg right
        // by 4 yields 3096, not -1000, because the device emits 0x418 << 4
        // rather than a sign-extended 16-bit word.
        let mut out = [0i16; 3];
        for (i, slot) in out.iter_mut().enumerate() {
            let w = b[i * 2] as u16 | ((b[i * 2 + 1] as u16) << 8);
            let v = (w >> 4) as u16;
            let v = if v & 0x0800 != 0 { v | 0xF000 } else { v };
            *slot = v as i16;
        }
        Ok(out)
    }

    /// Vector magnitude in mg, ~1000 when still, -1 on a failed read.
    pub fn magnitude(&mut self, max_polls: u32) -> i32 {
        match self.accel(max_polls) {
            Ok([x, y, z]) => isqrt((x as i64 * x as i64 + y as i64 * y as i64 + z as i64 * z as i64) as u64) as i32,
            Err(_) => -1,
        }
    }

    /// Latched threshold interrupt. Reading INT1_SRC clears it (ST side
    /// effect).
    pub fn motion_interrupt(&mut self) -> Result<bool, B::Error> {
        Ok(self.read_reg(INT1_SRC)? & 0x01 != 0)
    }

    /// Clears the latched interrupt (the read has the clear side effect).
    pub fn clear_interrupt(&mut self) -> Result<(), B::Error> {
        self.read_reg(INT1_SRC)?;
        Ok(())
    }

    /// Polling fallback / cross-check, so a missed interrupt does not
    /// silently read as parked. Orientation-independent: gravity fixes the
    /// vector length at 1 g regardless of tilt. Requires `samples`
    /// consecutive magnitudes deviating more than `idle_threshold_mg` from
    /// 1 g, bounded by `max_polls` status reads.
    pub fn is_moving(
        &mut self,
        idle_threshold_mg: i32,
        samples: u8,
        max_polls: u32,
    ) -> Result<bool, B::Error> {
        let mut hits = 0u8;
        let mut polls = 0u32;
        while hits < samples && polls < max_polls {
            if !self.data_ready()? {
                polls += 1;
                continue;
            }
            polls += 1;
            match self.read_accel()? {
                [x, y, z] => {
                    let mag = isqrt(
                        (x as i64 * x as i64 + y as i64 * y as i64 + z as i64 * z as i64) as u64,
                    ) as i32;
                    let delta = (mag - 1000).abs();
                    if delta > idle_threshold_mg {
                        hits += 1;
                    } else {
                        hits = 0; // must be consecutive
                    }
                }
            }
        }
        Ok(hits >= samples)
    }

    /// Human-readable WHO_AM_I family name.
    pub fn who_am_i_name(v: u8) -> &'static str {
        match v {
            0x32 => "LIS2DH12",
            0x33 => "LIS3DH-family clone",
            0x2A => "LIS2DU-family clone",
            _ => "unknown",
        }
    }
}

/// Integer square root - no flash, no float.
fn isqrt(m: u64) -> u64 {
    let mut m = m;
    let mut root: u64 = 0;
    let mut bit: u64 = 1 << 62;
    while bit > m {
        bit >>= 2;
    }
    while bit != 0 {
        if m >= root + bit {
            m -= root + bit;
            root = (root >> 1) + bit;
        } else {
            root >>= 1;
        }
        bit >>= 2;
    }
    root
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_hal::i2c::ErrorType;
    use std::cell::RefCell;
    use std::rc::Rc;

    /// Fake LIS2DH register model behind an I2C impl, mirroring the C++ test
    /// suite's approach.
    struct Model {
        regs: [u8; 256],
    }


    impl Model {
        fn new() -> Self {
            Self { regs: [0; 256] }
        }

        fn set24(&mut self, reg: u8, v: [i16; 3]) {
            for (i, axis) in v.iter().enumerate() {
                let w = (*axis as u16 as i32) << 4; // left-justify 12-bit
                let w = w as u16;
                self.regs[(reg as usize) + i * 2] = (w & 0xFF) as u8;
                self.regs[(reg as usize) + i * 2 + 1] = (w >> 8) as u8;
            }
        }
    }

    struct FakeI2c {
        model: Rc<RefCell<Model>>,
        fail: bool,
    }

    #[derive(Debug)]
    struct NoopError;

    impl embedded_hal::i2c::Error for NoopError {
        fn kind(&self) -> embedded_hal::i2c::ErrorKind {
            embedded_hal::i2c::ErrorKind::Other
        }
    }

    impl ErrorType for FakeI2c {
        type Error = NoopError;
    }

    impl I2c for FakeI2c {
        fn transaction(
            &mut self,
            _address: u8,
            operations: &mut [embedded_hal::i2c::Operation<'_>],
        ) -> Result<(), Self::Error> {
            if self.fail {
                return Err(NoopError);
            }
            let mut next_reg: Option<u8> = None;
            for op in operations {
                match op {
                    embedded_hal::i2c::Operation::Write(w) => {
                        // Register write: [reg, val]; register select: [reg].
                        if w.len() == 2 {
                            self.model.borrow_mut().regs[w[0] as usize] = w[1];
                            next_reg = None;
                        } else if w.len() == 1 {
                            next_reg = Some(w[0]);
                        }
                    }
                    embedded_hal::i2c::Operation::Read(r) => {
                        let start = next_reg.take().unwrap_or(0) as usize;
                        let regs = self.model.borrow().regs;
                        for (i, slot) in r.iter_mut().enumerate() {
                            *slot = regs[start + i];
                        }
                    }
                }
            }
            Ok(())
        }
    }

    fn imu() -> (Lis2dh<FakeI2c>, Rc<RefCell<Model>>) {
        let model = Rc::new(RefCell::new(Model::new()));
        model.borrow_mut().regs[WHO_AM_I as usize] = 0x33;
        // Data-ready bit set, so reads do not poll out.
        model.borrow_mut().regs[STATUS_REG as usize] = 0x08;
        (
            Lis2dh::new(
                FakeI2c {
                    model: model.clone(),
                    fail: false,
                },
                0x19,
            ),
            model,
        )
    }

    #[test]
    fn probe_accepts_known_family() {
        let (mut imu, _) = imu();
        assert!(imu.probe().unwrap());
    }

    #[test]
    fn probe_rejects_unknown() {
        let (mut imu, model) = imu();
        model.borrow_mut().regs[WHO_AM_I as usize] = 0x00;
        assert!(!imu.probe().unwrap());
    }

    #[test]
    fn configure_writes_expected_registers() {
        let (mut imu, model) = imu();
        imu.configure(0x08, 0).unwrap();
        let m = model.borrow();
        assert_eq!(m.regs[CTRL_REG1 as usize], CTRL1_LP_10HZ);
        assert_eq!(m.regs[CTRL_REG4 as usize], CTRL4_2G);
        assert_eq!(m.regs[INT1_THS as usize], 0x08);
        assert_eq!(m.regs[INT1_DUR as usize], 0);
    }

    #[test]
    fn accel_sign_extends_from_bit11() {
        let (mut imu, model) = imu();
        // -1000 mg in 12-bit: 0xC18. Left-justified into 16 bits: 0xC180.
        model.borrow_mut().set24(OUT_X_L, [-1000, 0, 1000]);
        assert_eq!(imu.accel(10).unwrap(), [-1000, 0, 1000]);
    }

    #[test]
    fn magnitude_still_is_about_1000() {
        let (mut imu, model) = imu();
        model.borrow_mut().set24(OUT_X_L, [3, 4, 998]);
        let mag = imu.magnitude(10);
        assert_eq!(mag, 998, "floor(sqrt(996029))");
    }

    #[test]
    fn magnitude_reports_minus_one_on_bus_error() {
        let model = Rc::new(RefCell::new(Model::new()));
        model.borrow_mut().regs[WHO_AM_I as usize] = 0x33;
        model.borrow_mut().regs[STATUS_REG as usize] = 0x08;
        let mut imu = Lis2dh::new(
            FakeI2c {
                model,
                fail: true,
            },
            0x19,
        );
        assert_eq!(imu.magnitude(10), -1);
    }

    #[test]
    fn motion_interrupt_reads_latch_bit() {
        let (mut imu, model) = imu();
        model.borrow_mut().regs[INT1_SRC as usize] = 0x01;
        assert!(imu.motion_interrupt().unwrap());
        // Reading the source also clears in the model (we emulate the side
        // effect by checking the API contract only).
    }

    #[test]
    fn is_moving_false_when_still() {
        let (mut imu, model) = imu();
        model.borrow_mut().set24(OUT_X_L, [0, 0, 1000]);
        assert!(!imu.is_moving(250, 3, 100).unwrap());
    }

    #[test]
    fn is_moving_true_when_beyond_threshold() {
        let (mut imu, model) = imu();
        model.borrow_mut().set24(OUT_X_L, [1300, 0, 0]);
        assert!(imu.is_moving(250, 3, 100).unwrap());
    }

    #[test]
    fn is_moving_false_on_bus_error() {
        let model = Rc::new(RefCell::new(Model::new()));
        model.borrow_mut().regs[WHO_AM_I as usize] = 0x33;
        let mut imu = Lis2dh::new(
            FakeI2c {
                model,
                fail: true,
            },
            0x19,
        );
        assert!(imu.is_moving(250, 3, 100).is_err());
    }

    #[test]
    fn isqrt_matches_known_values() {
        assert_eq!(isqrt(0), 0);
        assert_eq!(isqrt(1_000_000), 1000);
        assert_eq!(isqrt(2_000_000), 1414); // floor(sqrt(2e6)) = 1414
    }
}
