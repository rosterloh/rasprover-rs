//! Tests for the IMU processing pipeline: KalmanFilter and MahonyFilter.
//!
//! All tests are pure computation — no sensor hardware required.

#![no_std]
#![no_main]

esp_bootloader_esp_idf::esp_app_desc!();

#[cfg(test)]
#[embedded_test::tests(executor = esp_rtos::embassy::Executor::new())]
mod tests {
    use libm::sqrtf;
    use rasprover_rs::imu::{ImuProcessor, KalmanFilter, MahonyFilter};

    #[init]
    fn init() {
        let peripherals = esp_hal::init(esp_hal::Config::default());
        let timg1 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG1);
        esp_rtos::start(timg1.timer0);
        rtt_target::rtt_init_defmt!();
    }

    // ── KalmanFilter ──────────────────────────────────────────────────────────

    #[test]
    async fn kalman_converges_toward_constant_input() {
        let mut kf = KalmanFilter::new(0.001, 0.1);
        let target = 5.0_f32;
        let mut output = 0.0_f32;
        for _ in 0..200 {
            output = kf.update(target);
        }
        // After many updates the estimate must be very close to the input.
        let diff = output - target;
        defmt::assert!(diff > -0.01 && diff < 0.01, "Kalman did not converge");
    }

    #[test]
    async fn kalman_high_measurement_noise_moves_slowly() {
        // Large r → trust the measurement less → slow convergence.
        let mut kf = KalmanFilter::new(0.001, 100.0);
        let mut output = 0.0_f32;
        for _ in 0..10 {
            output = kf.update(10.0);
        }
        // After only 10 updates with high noise weighting the estimate should
        // still be well below the target.
        defmt::assert!(output < 5.0, "Expected slow convergence with high r");
    }

    #[test]
    async fn kalman_low_measurement_noise_moves_quickly() {
        // Small r → trust the measurement more → fast convergence.
        let mut kf = KalmanFilter::new(0.001, 0.001);
        let mut output = 0.0_f32;
        for _ in 0..10 {
            output = kf.update(10.0);
        }
        defmt::assert!(output > 8.0, "Expected fast convergence with low r");
    }

    #[test]
    async fn kalman_output_is_finite() {
        let mut kf = KalmanFilter::new(0.001, 0.1);
        for _ in 0..1000 {
            let out = kf.update(1.23);
            defmt::assert!(!out.is_nan() && !out.is_infinite(), "Kalman output non-finite");
        }
    }

    // ── MahonyFilter ─────────────────────────────────────────────────────────

    #[test]
    async fn mahony_initial_quaternion_is_identity() {
        let mf = MahonyFilter::new(2.0, 0.005);
        let q = mf.quaternion();
        defmt::assert_eq!(q[0], 1.0_f32); // w
        defmt::assert_eq!(q[1], 0.0_f32); // x
        defmt::assert_eq!(q[2], 0.0_f32); // y
        defmt::assert_eq!(q[3], 0.0_f32); // z
    }

    #[test]
    async fn mahony_quaternion_stays_normalised() {
        let mut mf = MahonyFilter::new(2.0, 0.005);
        // Gravity pointing down, no rotation, no magnetic field (6-DOF fallback).
        let acc = [0.0_f32, 0.0, 1.0];
        let gyr = [0.0_f32; 3];
        let mag = [0.0_f32; 3]; // degenerate → 6-DOF path
        for _ in 0..100 {
            mf.update(acc, gyr, mag, 0.01);
        }
        let q = mf.quaternion();
        let norm = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        let diff = norm - 1.0;
        defmt::assert!(
            diff > -1e-4 && diff < 1e-4,
            "Quaternion norm drifted from 1.0"
        );
    }

    #[test]
    async fn mahony_degenerate_accelerometer_does_not_panic() {
        let mut mf = MahonyFilter::new(2.0, 0.005);
        // Zero accelerometer vector is degenerate; filter must still produce
        // finite output and a normalised quaternion.
        let (roll, pitch, yaw) =
            mf.update([0.0; 3], [0.0; 3], [20.0, 0.0, 0.0], 0.01);
        defmt::assert!(!roll.is_nan() && !pitch.is_nan() && !yaw.is_nan());
        let q = mf.quaternion();
        let norm = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        let diff = norm - 1.0;
        defmt::assert!(diff > -1e-4 && diff < 1e-4);
    }

    #[test]
    async fn mahony_degenerate_magnetometer_uses_6dof_fallback() {
        let mut mf = MahonyFilter::new(2.0, 0.005);
        // Magnetometer is zero → must not panic and output must be finite.
        let (roll, pitch, yaw) =
            mf.update([0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0; 3], 0.01);
        defmt::assert!(!roll.is_nan() && !pitch.is_nan() && !yaw.is_nan());
    }

    #[test]
    async fn mahony_level_attitude_converges_to_near_zero_roll_pitch() {
        let mut mf = MahonyFilter::new(2.0, 0.005);
        // Gravity on +Z, no rotation, valid magnetometer on +X (pointing north).
        let acc = [0.0_f32, 0.0, 1.0];
        let gyr = [0.0_f32; 3];
        let mag = [40.0_f32, 0.0, 0.0];
        let mut roll = 0.0_f32;
        let mut pitch = 0.0_f32;
        for _ in 0..500 {
            let angles = mf.update(acc, gyr, mag, 0.01);
            roll = angles.0;
            pitch = angles.1;
        }
        defmt::assert!(
            roll > -2.0 && roll < 2.0,
            "Roll did not converge near zero"
        );
        defmt::assert!(
            pitch > -2.0 && pitch < 2.0,
            "Pitch did not converge near zero"
        );
    }

    // ── ImuProcessor ─────────────────────────────────────────────────────────

    #[test]
    async fn imu_processor_update_produces_finite_output() {
        let mut proc = ImuProcessor::new();
        let data = proc.update(
            [0.0, 0.0, 1.0], // acc: gravity on Z
            [0.0; 3],        // gyr: stationary
            [40.0, 0.0, 0.0], // mag: north on X
            25.0,            // temperature
            0.01,            // dt 10 ms
        );
        defmt::assert!(!data.roll.is_nan() && !data.pitch.is_nan() && !data.yaw.is_nan());
        defmt::assert!(!data.tmp.is_nan());
        for v in data.acc {
            defmt::assert!(!v.is_nan() && !v.is_infinite());
        }
    }

    #[test]
    async fn imu_processor_repeated_updates_stay_finite() {
        let mut proc = ImuProcessor::new();
        for _ in 0..200 {
            let data = proc.update(
                [0.01, 0.0, 0.98],
                [0.5, -0.3, 0.1],
                [38.0, 2.0, -5.0],
                24.5,
                0.01,
            );
            defmt::assert!(!data.roll.is_nan() && !data.pitch.is_nan() && !data.yaw.is_nan());
        }
    }
}
