use core::cell::RefCell;

use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_sync::{
    blocking_mutex::{Mutex, raw::CriticalSectionRawMutex},
    signal::Signal,
};
use embedded_graphics::{
    mono_font::{MonoTextStyleBuilder, ascii::FONT_6X10},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use heapless::String;
use ssd1306::{I2CDisplayInterface, Ssd1306Async, prelude::*};

type Line = String<21>;

static LINES: Mutex<CriticalSectionRawMutex, RefCell<[Line; 3]>> =
    Mutex::new(RefCell::new([String::new(), String::new(), String::new()]));

static DIRTY: Signal<CriticalSectionRawMutex, ()> = Signal::new();

pub fn set_line(idx: usize, text: &str) {
    LINES.lock(|lines| {
        let mut lines = lines.borrow_mut();
        lines[idx].clear();
        lines[idx].push_str(text).ok();
    });
    DIRTY.signal(());
}

const LINE_Y: [i32; 3] = [0, 11, 22];

#[embassy_executor::task]
pub async fn display_task() {
    let display_i2c = I2cDevice::new(crate::board::get_i2c_bus());
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
