#![no_main]
#![no_std]

mod i2c_bus;
mod motors;
mod network;
mod pins;

use ariel_os::{
    debug::log::{debug, error, info},
    i2c::controller::I2cDevice,
    time::{Delay, Timer},
};
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use icm20948_async::{AccRange, GyrRange, IcmBuilder, I2cAddress};
use network::NetworkState;
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306Async};

#[ariel_os::task(autostart, peripherals)]
async fn main(peripherals: pins::Peripherals) {
    let mut motors = motors::Motors::new(
        peripherals.ledc,
        peripherals.motor_ain1,
        peripherals.motor_ain2,
        peripherals.motor_pwma,
        peripherals.motor_bin1,
        peripherals.motor_bin2,
        peripherals.motor_pwmb,
    );
    info!("Motors initialized");
    motors.set_velocity(50, 50);
    Timer::after_millis(500).await;
    motors.stop();

    i2c_bus::init(peripherals.i2c_sda, peripherals.i2c_scl);

    // --- Display ---
    let display_i2c = I2cDevice::new(i2c_bus::I2C_BUS.get().unwrap());
    let interface = I2CDisplayInterface::new(display_i2c);
    let mut display = Ssd1306Async::new(interface, DisplaySize128x32, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();

    display.init().await.unwrap();

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();

    let ver_line = concat!("RaspRover v", env!("CARGO_PKG_VERSION"));
    Text::with_baseline(ver_line, Point::new(0, 0), text_style, Baseline::Top)
        .draw(&mut display)
        .unwrap();
    Text::with_baseline("Connecting...", Point::new(0, 14), text_style, Baseline::Top)
        .draw(&mut display)
        .unwrap();
    display.flush().await.unwrap();

    // --- IMU ---
    let imu_i2c = I2cDevice::new(i2c_bus::I2C_BUS.get().unwrap());
    let mut imu = IcmBuilder::new_i2c(imu_i2c, Delay)
        .set_address(I2cAddress::X68)
        .acc_range(AccRange::Gs8)
        .gyr_range(GyrRange::Dps500)
        .initialize_9dof()
        .await
        .unwrap();

    info!("ICM-20948 initialized");

    let mut net_rx = network::NET_STATE.receiver().unwrap();
    let mut last_net_state: Option<NetworkState> = None;

    // --- Sensor loop ---
    loop {
        // Non-blocking check for network state change
        if let Some(state) = net_rx.try_get() {
            if Some(&state) != last_net_state.as_ref() {
                display.clear(BinaryColor::Off).unwrap();
                Text::with_baseline(ver_line, Point::new(0, 0), text_style, Baseline::Top)
                    .draw(&mut display)
                    .unwrap();
                let status: &str = match &state {
                    NetworkState::Connecting => "Connecting...",
                    NetworkState::Up(ip) => ip.as_str(),
                    NetworkState::Down => "Disconnected",
                };
                Text::with_baseline(status, Point::new(0, 14), text_style, Baseline::Top)
                    .draw(&mut display)
                    .unwrap();
                display.flush().await.unwrap();
                last_net_state = Some(state);
            }
        }

        match imu.read_9dof().await {
            Ok(data) => debug!(
                "Accel [{} {} {}] Gyro [{} {} {}] Mag [{} {} {}] Temp {} °C",
                data.acc[0],
                data.acc[1],
                data.acc[2],
                data.gyr[0],
                data.gyr[1],
                data.gyr[2],
                data.mag[0],
                data.mag[1],
                data.mag[2],
                data.tmp
            ),
            Err(_) => error!("IMU read error"),
        }
        Timer::after_millis(100).await;
    }
}
