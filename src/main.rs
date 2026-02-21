#![no_main]
#![no_std]

mod pins;

use core::fmt::Write as _;

use ariel_os::{
    debug::log::info,
    hal,
    i2c::controller::{Kilohertz, highest_freq_in},
    net,
};
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use heapless::String;
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306Async};

#[ariel_os::task(autostart, peripherals)]
async fn main(peripherals: pins::Peripherals) {
    let mut i2c_config = hal::i2c::controller::Config::default();
    i2c_config.frequency = const { highest_freq_in(Kilohertz::kHz(100)..=Kilohertz::kHz(400)) };

    let i2c = pins::DisplayI2c::new(peripherals.i2c_sda, peripherals.i2c_scl, i2c_config);

    let interface = I2CDisplayInterface::new(i2c);
    let mut display = Ssd1306Async::new(interface, DisplaySize128x32, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();

    display.init().await.unwrap();

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();

    // Line 1: firmware version, shown immediately on boot
    let ver_line = concat!("RaspRover v", env!("CARGO_PKG_VERSION"));
    Text::with_baseline(ver_line, Point::new(0, 0), text_style, Baseline::Top)
        .draw(&mut display)
        .unwrap();
    Text::with_baseline("Connecting...", Point::new(0, 14), text_style, Baseline::Top)
        .draw(&mut display)
        .unwrap();
    display.flush().await.unwrap();

    // Line 2: IP address once the network stack has a DHCP lease
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

    info!("Display ready on {}", ariel_os::buildinfo::BOARD);
}
