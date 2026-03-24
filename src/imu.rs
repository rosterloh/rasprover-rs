/// A 1-D discrete Kalman filter.
///
/// Apply one instance per sensor axis. The filter tracks a single state
/// variable (the "true" signal) and reduces noise by balancing the process
/// model against noisy measurements.
///
/// Tuning parameters:
/// - `q` (process noise covariance) — how much the true value can change
///   between samples. Larger → faster tracking, less smoothing.
/// - `r` (measurement noise covariance) — how noisy the raw sensor reading
///   is. Larger → more smoothing, slower response.
#[derive(Clone, Copy)]
pub struct KalmanFilter {
    /// Current state estimate.
    x: f32,
    /// Estimate error covariance.
    p: f32,
    /// Process noise covariance.
    q: f32,
    /// Measurement noise covariance.
    r: f32,
}

impl KalmanFilter {
    pub fn new(q: f32, r: f32) -> Self {
        Self { x: 0.0, p: 1.0, q, r }
    }

    /// Feed a raw measurement and return the filtered estimate.
    pub fn update(&mut self, z: f32) -> f32 {
        // — Predict —
        // State is assumed constant between steps; only covariance grows.
        let p_pred = self.p + self.q;

        // — Update —
        let k = p_pred / (p_pred + self.r); // Kalman gain
        self.x += k * (z - self.x);
        self.p = (1.0 - k) * p_pred;

        self.x
    }
}

/// Per-axis Kalman-filtered output of a 9-DOF IMU.
#[derive(Clone, defmt::Format)]
pub struct FilteredImuData {
    pub acc: [f32; 3],
    pub gyr: [f32; 3],
    pub mag: [f32; 3],
    pub tmp: f32,
}

/// Kalman filters for all axes of a 9-DOF IMU (accelerometer, gyroscope,
/// magnetometer).
///
/// Default noise parameters are conservative starting points; tune `q` and
/// `r` to match the actual sensor noise floor and desired responsiveness.
pub struct ImuKalmanFilter {
    acc: [KalmanFilter; 3],
    gyr: [KalmanFilter; 3],
    mag: [KalmanFilter; 3],
}

impl ImuKalmanFilter {
    /// Construct filters with sensor-appropriate default noise parameters.
    ///
    /// | Sensor | Q (process noise) | R (measurement noise) |
    /// |--------|------------------|-----------------------|
    /// | Accel  | 0.001            | 0.1                   |
    /// | Gyro   | 0.003            | 0.03                  |
    /// | Mag    | 0.001            | 0.5                   |
    pub fn new() -> Self {
        Self {
            acc: [KalmanFilter::new(0.001, 0.1); 3],
            gyr: [KalmanFilter::new(0.003, 0.03); 3],
            mag: [KalmanFilter::new(0.001, 0.5); 3],
        }
    }

    /// Apply Kalman filtering to one set of raw IMU readings.
    ///
    /// Temperature is passed through unchanged (no filtering applied).
    pub fn update(
        &mut self,
        acc: [f32; 3],
        gyr: [f32; 3],
        mag: [f32; 3],
        tmp: f32,
    ) -> FilteredImuData {
        FilteredImuData {
            acc: [
                self.acc[0].update(acc[0]),
                self.acc[1].update(acc[1]),
                self.acc[2].update(acc[2]),
            ],
            gyr: [
                self.gyr[0].update(gyr[0]),
                self.gyr[1].update(gyr[1]),
                self.gyr[2].update(gyr[2]),
            ],
            mag: [
                self.mag[0].update(mag[0]),
                self.mag[1].update(mag[1]),
                self.mag[2].update(mag[2]),
            ],
            tmp,
        }
    }
}

impl Default for ImuKalmanFilter {
    fn default() -> Self {
        Self::new()
    }
}
