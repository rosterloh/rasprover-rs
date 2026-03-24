//! IMU signal processing: per-axis Kalman filtering and Mahony 9-DOF pose fusion.
//!
//! Processing pipeline:
//! 1. [`KalmanFilter`] — one instance per sensor axis, smooths raw sensor noise.
//! 2. [`MahonyFilter`] — fuses filtered acc + gyr + mag into quaternion attitude,
//!    outputting roll / pitch / yaw in degrees.
//! 3. [`ImuProcessor`] — top-level type combining both stages.

use libm::{asinf, atan2f, sqrtf};

const DEG_TO_RAD: f32 = core::f32::consts::PI / 180.0;
const RAD_TO_DEG: f32 = 180.0 / core::f32::consts::PI;

/// Single-axis discrete Kalman filter for smoothing noisy sensor readings.
///
/// Tuning parameters:
/// - `q` (process noise covariance): how quickly the true value can change.
///   Higher → tracks fast changes but passes more noise.
/// - `r` (measurement noise covariance): how noisy the raw reading is.
///   Higher → smoother but slower to respond.
pub struct KalmanFilter {
    q: f32,
    r: f32,
    x: f32, // state estimate
    p: f32, // estimate error covariance
}

impl KalmanFilter {
    pub const fn new(q: f32, r: f32) -> Self {
        Self { q, r, x: 0.0, p: 1.0 }
    }

    pub fn update(&mut self, z: f32) -> f32 {
        // Predict
        self.p += self.q;
        // Update
        let k = self.p / (self.p + self.r);
        self.x += k * (z - self.x);
        self.p *= 1.0 - k;
        self.x
    }
}

/// Nine independent Kalman filters — one per axis per sensor.
struct AxisFilters {
    acc: [KalmanFilter; 3],
    gyr: [KalmanFilter; 3],
    mag: [KalmanFilter; 3],
}

impl AxisFilters {
    fn new() -> Self {
        Self {
            acc: [
                KalmanFilter::new(0.001, 0.1),
                KalmanFilter::new(0.001, 0.1),
                KalmanFilter::new(0.001, 0.1),
            ],
            gyr: [
                KalmanFilter::new(0.003, 0.03),
                KalmanFilter::new(0.003, 0.03),
                KalmanFilter::new(0.003, 0.03),
            ],
            mag: [
                KalmanFilter::new(0.001, 0.5),
                KalmanFilter::new(0.001, 0.5),
                KalmanFilter::new(0.001, 0.5),
            ],
        }
    }

    fn update(
        &mut self,
        acc: [f32; 3],
        gyr: [f32; 3],
        mag: [f32; 3],
    ) -> ([f32; 3], [f32; 3], [f32; 3]) {
        (
            [
                self.acc[0].update(acc[0]),
                self.acc[1].update(acc[1]),
                self.acc[2].update(acc[2]),
            ],
            [
                self.gyr[0].update(gyr[0]),
                self.gyr[1].update(gyr[1]),
                self.gyr[2].update(gyr[2]),
            ],
            [
                self.mag[0].update(mag[0]),
                self.mag[1].update(mag[1]),
                self.mag[2].update(mag[2]),
            ],
        )
    }
}

/// Mahony complementary filter for 9-DOF attitude estimation.
///
/// Fuses accelerometer + gyroscope + magnetometer into a quaternion attitude,
/// then outputs roll / pitch / yaw in degrees. Falls back to 6-DOF (acc + gyr)
/// when the magnetometer reading is degenerate (near-zero magnitude).
///
/// Tuning:
/// - `kp` (proportional gain, default 2.0): higher → faster convergence, more noise sensitivity
/// - `ki` (integral gain, default 0.005): corrects slow gyroscope bias drift
///
/// Reference: Mahony et al., "Nonlinear Complementary Filters on the Special
/// Orthogonal Group", IEEE TAC, 2008.
pub struct MahonyFilter {
    kp: f32,
    ki: f32,
    // Quaternion (w, x, y, z)
    q0: f32,
    q1: f32,
    q2: f32,
    q3: f32,
    // Accumulated integral error terms
    ex_int: f32,
    ey_int: f32,
    ez_int: f32,
}

impl MahonyFilter {
    pub fn new(kp: f32, ki: f32) -> Self {
        Self {
            kp,
            ki,
            q0: 1.0,
            q1: 0.0,
            q2: 0.0,
            q3: 0.0,
            ex_int: 0.0,
            ey_int: 0.0,
            ez_int: 0.0,
        }
    }

    /// Update with 9-DOF sensor data.
    ///
    /// - `acc`: accelerometer in g
    /// - `gyr`: gyroscope in **deg/s** (converted internally to rad/s)
    /// - `mag`: magnetometer in µT
    /// - `dt`: elapsed time in seconds since the last call
    ///
    /// Returns `(roll, pitch, yaw)` in degrees.
    pub fn update(
        &mut self,
        acc: [f32; 3],
        gyr: [f32; 3],
        mag: [f32; 3],
        dt: f32,
    ) -> (f32, f32, f32) {
        let mut gx = gyr[0] * DEG_TO_RAD;
        let mut gy = gyr[1] * DEG_TO_RAD;
        let mut gz = gyr[2] * DEG_TO_RAD;

        let mut ax = acc[0];
        let mut ay = acc[1];
        let mut az = acc[2];

        let mut mx = mag[0];
        let mut my = mag[1];
        let mut mz = mag[2];

        // Normalise accelerometer — skip attitude update if stationary/degenerate.
        let norm_a = sqrtf(ax * ax + ay * ay + az * az);
        if norm_a < 1e-6 {
            return self.euler_angles();
        }
        let inv_a = 1.0 / norm_a;
        ax *= inv_a;
        ay *= inv_a;
        az *= inv_a;

        // Normalise magnetometer — fall back to 6-DOF if degenerate.
        let norm_m = sqrtf(mx * mx + my * my + mz * mz);
        if norm_m < 1e-6 {
            return self.update_6dof(ax, ay, az, gx, gy, gz, dt);
        }
        let inv_m = 1.0 / norm_m;
        mx *= inv_m;
        my *= inv_m;
        mz *= inv_m;

        let q0 = self.q0;
        let q1 = self.q1;
        let q2 = self.q2;
        let q3 = self.q3;

        // Reference direction of earth's magnetic field in the navigation frame,
        // projected back into the body frame via the current quaternion.
        let hx = 2.0 * mx * (0.5 - q2 * q2 - q3 * q3)
            + 2.0 * my * (q1 * q2 - q0 * q3)
            + 2.0 * mz * (q0 * q2 + q1 * q3);
        let hy = 2.0 * mx * (q1 * q2 + q0 * q3)
            + 2.0 * my * (0.5 - q1 * q1 - q3 * q3)
            + 2.0 * mz * (q2 * q3 - q0 * q1);
        let bx = sqrtf(hx * hx + hy * hy);
        let bz = 2.0 * mx * (q1 * q3 - q0 * q2)
            + 2.0 * my * (q0 * q1 + q2 * q3)
            + 2.0 * mz * (0.5 - q1 * q1 - q2 * q2);

        // Estimated direction of gravity and magnetic north in the body frame.
        let vx = 2.0 * (q1 * q3 - q0 * q2);
        let vy = 2.0 * (q0 * q1 + q2 * q3);
        let vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

        let wx = 2.0 * bx * (0.5 - q2 * q2 - q3 * q3) + 2.0 * bz * (q1 * q3 - q0 * q2);
        let wy = 2.0 * bx * (q1 * q2 - q0 * q3) + 2.0 * bz * (q0 * q1 + q2 * q3);
        let wz = 2.0 * bx * (q0 * q2 + q1 * q3) + 2.0 * bz * (0.5 - q1 * q1 - q2 * q2);

        // Error: cross product between estimated and measured direction vectors.
        let ex = (ay * vz - az * vy) + (my * wz - mz * wy);
        let ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
        let ez = (ax * vy - ay * vx) + (mx * wy - my * wx);

        self.apply_correction(&mut gx, &mut gy, &mut gz, ex, ey, ez, dt);
        self.integrate_quaternion(gx, gy, gz, dt);
        self.euler_angles()
    }

    /// 6-DOF fallback (acc + gyr only) used when magnetometer is unavailable.
    fn update_6dof(
        &mut self,
        ax: f32,
        ay: f32,
        az: f32,
        mut gx: f32,
        mut gy: f32,
        mut gz: f32,
        dt: f32,
    ) -> (f32, f32, f32) {
        let q0 = self.q0;
        let q1 = self.q1;
        let q2 = self.q2;
        let q3 = self.q3;

        // Estimated direction of gravity in the body frame.
        let vx = 2.0 * (q1 * q3 - q0 * q2);
        let vy = 2.0 * (q0 * q1 + q2 * q3);
        let vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

        let ex = ay * vz - az * vy;
        let ey = az * vx - ax * vz;
        let ez = ax * vy - ay * vx;

        self.apply_correction(&mut gx, &mut gy, &mut gz, ex, ey, ez, dt);
        self.integrate_quaternion(gx, gy, gz, dt);
        self.euler_angles()
    }

    fn apply_correction(
        &mut self,
        gx: &mut f32,
        gy: &mut f32,
        gz: &mut f32,
        ex: f32,
        ey: f32,
        ez: f32,
        dt: f32,
    ) {
        if self.ki > 0.0 {
            self.ex_int += self.ki * ex * dt;
            self.ey_int += self.ki * ey * dt;
            self.ez_int += self.ki * ez * dt;
            *gx += self.ex_int;
            *gy += self.ey_int;
            *gz += self.ez_int;
        }
        *gx += self.kp * ex;
        *gy += self.kp * ey;
        *gz += self.kp * ez;
    }

    fn integrate_quaternion(&mut self, gx: f32, gy: f32, gz: f32, dt: f32) {
        let q0 = self.q0;
        let q1 = self.q1;
        let q2 = self.q2;
        let q3 = self.q3;
        let half_dt = 0.5 * dt;

        self.q0 += half_dt * (-q1 * gx - q2 * gy - q3 * gz);
        self.q1 += half_dt * (q0 * gx + q2 * gz - q3 * gy);
        self.q2 += half_dt * (q0 * gy - q1 * gz + q3 * gx);
        self.q3 += half_dt * (q0 * gz + q1 * gy - q2 * gx);

        // Re-normalise quaternion to prevent drift.
        let norm =
            sqrtf(self.q0 * self.q0 + self.q1 * self.q1 + self.q2 * self.q2 + self.q3 * self.q3);
        if norm > 1e-6 {
            let inv_norm = 1.0 / norm;
            self.q0 *= inv_norm;
            self.q1 *= inv_norm;
            self.q2 *= inv_norm;
            self.q3 *= inv_norm;
        }
    }

    fn euler_angles(&self) -> (f32, f32, f32) {
        let q0 = self.q0;
        let q1 = self.q1;
        let q2 = self.q2;
        let q3 = self.q3;

        // Clamp asin argument to [-1, 1] to handle floating-point rounding.
        let sin_pitch = 2.0 * (q0 * q2 - q3 * q1);
        let sin_pitch = if sin_pitch < -1.0 {
            -1.0_f32
        } else if sin_pitch > 1.0 {
            1.0_f32
        } else {
            sin_pitch
        };

        let roll = atan2f(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (q1 * q1 + q2 * q2));
        let pitch = asinf(sin_pitch);
        let yaw = atan2f(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3));

        (roll * RAD_TO_DEG, pitch * RAD_TO_DEG, yaw * RAD_TO_DEG)
    }

    /// Returns the current attitude quaternion `[w, x, y, z]`.
    pub fn quaternion(&self) -> [f32; 4] {
        [self.q0, self.q1, self.q2, self.q3]
    }
}

/// Filtered IMU data including orientation estimate.
pub struct ImuData {
    /// Kalman-filtered accelerometer readings in g.
    pub acc: [f32; 3],
    /// Kalman-filtered gyroscope readings in deg/s.
    pub gyr: [f32; 3],
    /// Kalman-filtered magnetometer readings in µT.
    pub mag: [f32; 3],
    /// Temperature in °C (unfiltered — low noise by design).
    pub tmp: f32,
    /// Roll angle in degrees (rotation around X axis).
    pub roll: f32,
    /// Pitch angle in degrees (rotation around Y axis).
    pub pitch: f32,
    /// Yaw / heading angle in degrees (rotation around Z axis, magnetic north = 0°).
    pub yaw: f32,
}

/// Combines per-axis Kalman pre-filtering with Mahony 9-DOF pose fusion.
///
/// Usage:
/// ```ignore
/// let mut processor = ImuProcessor::new();
/// let data = processor.update(raw_acc, raw_gyr, raw_mag, raw_tmp, dt_seconds);
/// ```
pub struct ImuProcessor {
    filters: AxisFilters,
    mahony: MahonyFilter,
}

impl ImuProcessor {
    pub fn new() -> Self {
        Self {
            filters: AxisFilters::new(),
            mahony: MahonyFilter::new(2.0, 0.005),
        }
    }

    /// Process a raw IMU reading.
    ///
    /// - `acc` / `gyr` / `mag`: raw sensor arrays in native units (g, deg/s, µT)
    /// - `tmp`: temperature in °C
    /// - `dt`: elapsed seconds since the previous call (use 0.0 on first call)
    pub fn update(
        &mut self,
        acc: [f32; 3],
        gyr: [f32; 3],
        mag: [f32; 3],
        tmp: f32,
        dt: f32,
    ) -> ImuData {
        let (facc, fgyr, fmag) = self.filters.update(acc, gyr, mag);
        let (roll, pitch, yaw) = self.mahony.update(facc, fgyr, fmag, dt);
        ImuData { acc: facc, gyr: fgyr, mag: fmag, tmp, roll, pitch, yaw }
    }
}
