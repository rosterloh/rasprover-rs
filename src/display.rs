use core::cell::RefCell;

use ariel_os::{i2c::controller::I2cDevice, time::Timer};
use embassy_sync::{
    blocking_mutex::{raw::CriticalSectionRawMutex, Mutex},
    signal::Signal,
};
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use heapless::String;
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306Async};

type Line = String<21>;

static LINES: Mutex<CriticalSectionRawMutex, RefCell<[Line; 3]>> =
    Mutex::new(RefCell::new([String::new(), String::new(), String::new()]));

static DIRTY: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Write a string to display line 0, 1, or 2. Non-blocking; safe to call from any task.
pub fn set_line(idx: usize, text: &str) {
    LINES.lock(|lines| {
        let mut lines = lines.borrow_mut();
        lines[idx].clear();
        lines[idx].push_str(text).ok();
    });
    DIRTY.signal(());
}

const LINE_Y: [i32; 3] = [0, 11, 22];

#[ariel_os::task(autostart)]
async fn display_task() {
    // Wait for main to call i2c_bus::init()
    let bus = loop {
        if let Some(bus) = crate::i2c_bus::I2C_BUS.get() {
            break bus;
        }
        Timer::after_millis(1).await;
    };

    let display_i2c = I2cDevice::new(bus);
    let interface = I2CDisplayInterface::new(display_i2c);
    let mut display = Ssd1306Async::new(interface, DisplaySize128x32, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().await.unwrap();

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();

    loop {
        DIRTY.wait().await;

        let snapshot = LINES.lock(|lines| lines.borrow().clone());

        display.clear(BinaryColor::Off).unwrap();
        for (i, line) in snapshot.iter().enumerate() {
            if !line.is_empty() {
                Text::with_baseline(
                    line.as_str(),
                    Point::new(0, LINE_Y[i]),
                    text_style,
                    Baseline::Top,
                )
                .draw(&mut display)
                .unwrap();
            }
        }
        display.flush().await.unwrap();
    }
}
