#![no_main]
#![no_std]

mod i2c_bus;
mod pins;

use core::fmt::Write as _;

use ariel_os::{
    debug::log::{debug, info, error},
    i2c::controller::I2cDevice,
    net,
    time::{Delay, Timer},
};
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use heapless::String;
use icm20948_async::{AccRange, GyrRange, IcmBuilder, I2cAddress};
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306Async};

#[ariel_os::task(autostart, peripherals)]
async fn main(peripherals: pins::Peripherals) {
    i2c_bus::init(peripherals);

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

    // --- Network ---
    let stack = net::network_stack().await.unwrap();
    stack.wait_config_up().await;

    if let Some(config) = stack.config_v4() {
        let mut ip_str: String<18> = String::new();
        write!(ip_str, "IP: {}", config.address.address()).unwrap();

        display.clear(BinaryColor::Off).unwrap();
        Text::with_baseline(ver_line, Point::new(0, 0), text_style, Baseline::Top)
            .draw(&mut display)
            .unwrap();
        Text::with_baseline(ip_str.as_str(), Point::new(0, 14), text_style, Baseline::Top)
            .draw(&mut display)
            .unwrap();
        display.flush().await.unwrap();

        info!("IP: {}", ip_str.as_str());
    }

    // --- Sensor loop ---
    loop {
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
