#![no_main]
#![no_std]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use defmt::{debug, error, info};
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_executor::Spawner;
use embassy_time::{Instant, Timer};
use esp_hal::{clock::CpuClock, rng::Rng, timer::timg::TimerGroup};
use icm20948_async::{AccRange, GyrRange, I2cAddress, IcmBuilder};
use network::NetworkState;
use panic_rtt_target as _;
use rasprover_rs::{board, display, imu, motors, network};

extern crate alloc;

esp_bootloader_esp_idf::esp_app_desc!();

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_rtos::main]
async fn main(spawner: Spawner) {
    rtt_target::rtt_init_defmt!();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let p = esp_hal::init(config);
    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 98768);

    let timg0 = TimerGroup::new(p.TIMG0);
    esp_rtos::start(timg0.timer0);

    let pins = board::Peripherals {
        i2c_sda: p.GPIO32,
        i2c_scl: p.GPIO33,
        i2c0: p.I2C0,
        motor_ain1: p.GPIO21.into(),
        motor_ain2: p.GPIO17.into(),
        motor_bin1: p.GPIO22.into(),
        motor_bin2: p.GPIO23.into(),
        motor_pwma: p.GPIO25.into(),
        motor_pwmb: p.GPIO26.into(),
        ledc: p.LEDC,
        pcnt: p.PCNT,
        enc_left: p.GPIO35.into(),
        enc_right: p.GPIO16.into(),
    };

    let mut motors = motors::Motors::new(
        pins.ledc,
        pins.motor_ain1,
        pins.motor_ain2,
        pins.motor_pwma,
        pins.motor_bin1,
        pins.motor_bin2,
        pins.motor_pwmb,
        pins.pcnt,
        pins.enc_left,
        pins.enc_right,
    );
    info!("Motors initialized");
    motors.set_velocity(50, 50);
    Timer::after_millis(500).await;
    motors.stop();

    board::init(pins.i2c0, pins.i2c_sda, pins.i2c_scl);
    display::set_line(0, concat!("RaspRover v", env!("CARGO_PKG_VERSION")));
    spawner.spawn(display::display_task()).ok();

    let rng = Rng::new();
    network::init(p.WIFI, rng, spawner);

    let imu_i2c = I2cDevice::new(board::get_i2c_bus());
    let mut imu = IcmBuilder::new_i2c(imu_i2c, embassy_time::Delay)
        .set_address(I2cAddress::X68)
        .acc_range(AccRange::Gs8)
        .gyr_range(GyrRange::Dps500)
        .initialize_9dof()
        .await
        .unwrap();
    info!("ICM-20948 initialised");

    let mut imu_processor = imu::ImuProcessor::new();
    let mut last_imu_instant = Instant::now();

    let mut net_state_rx = network::NET_STATE.receiver().unwrap();
    let mut last_net_state: Option<NetworkState> = None;

    loop {
        if let Some(state) = net_state_rx.try_get() {
            if Some(&state) != last_net_state.as_ref() {
                last_net_state = Some(state.clone());
            }

            if let NetworkState::Up(_) = state {
                let now = Instant::now();
                let dt = now.duration_since(last_imu_instant).as_micros() as f32 / 1_000_000.0;
                last_imu_instant = now;

                match imu.read_9dof().await {
                    Ok(data) => {
                        let processed =
                            imu_processor.update(data.acc, data.gyr, data.mag, data.tmp, dt);
                        debug!(
                            "Accel [{} {} {}] Gyro [{} {} {}] Mag [{} {} {}] Temp {} °C | Roll {} Pitch {} Yaw {}",
                            processed.acc[0],
                            processed.acc[1],
                            processed.acc[2],
                            processed.gyr[0],
                            processed.gyr[1],
                            processed.gyr[2],
                            processed.mag[0],
                            processed.mag[1],
                            processed.mag[2],
                            processed.tmp,
                            processed.roll,
                            processed.pitch,
                            processed.yaw,
                        );
                    }
                    Err(_) => error!("IMU read error"),
                }
            } else {
                motors.stop();
            }
        }

        Timer::after_millis(100).await;
    }
}
