extern crate alloc;

use alloc::boxed::Box;
use core::{
    cell::RefCell,
    sync::atomic::{AtomicI32, Ordering},
};
use esp_hal::{
    gpio::{DriveMode, Input, InputConfig, Level, Output, OutputConfig, OutputPin, Pull},
    handler,
    interrupt::Priority,
    ledc::{
        LSGlobalClkSource, Ledc, LowSpeed,
        channel::{self, ChannelIFace},
        timer::{self, TimerIFace},
    },
    pcnt::{
        Pcnt,
        channel::{CtrlMode, EdgeMode},
        unit::{Counter, Unit},
    },
    peripherals::{LEDC, PCNT},
    time::Rate,
};
use critical_section::Mutex;
use esp_hal::gpio::InputPin;

const LIMIT: i16 = 30_000;

static UNIT_LEFT: Mutex<RefCell<Option<Unit<'static, 0>>>> = Mutex::new(RefCell::new(None));
static UNIT_RIGHT: Mutex<RefCell<Option<Unit<'static, 1>>>> = Mutex::new(RefCell::new(None));
static TICKS_LEFT: AtomicI32 = AtomicI32::new(0);
static TICKS_RIGHT: AtomicI32 = AtomicI32::new(0);

#[handler(priority = Priority::Priority2)]
fn encoder_isr() {
    critical_section::with(|cs| {
        let mut left = UNIT_LEFT.borrow_ref_mut(cs);
        if let Some(u) = left.as_mut() {
            if u.interrupt_is_set() {
                let ev = u.events();
                if ev.high_limit {
                    TICKS_LEFT.fetch_add(LIMIT as i32, Ordering::Relaxed);
                } else if ev.low_limit {
                    TICKS_LEFT.fetch_add(-(LIMIT as i32), Ordering::Relaxed);
                }
                u.reset_interrupt();
            }
        }
        let mut right = UNIT_RIGHT.borrow_ref_mut(cs);
        if let Some(u) = right.as_mut() {
            if u.interrupt_is_set() {
                let ev = u.events();
                if ev.high_limit {
                    TICKS_RIGHT.fetch_add(LIMIT as i32, Ordering::Relaxed);
                } else if ev.low_limit {
                    TICKS_RIGHT.fetch_add(-(LIMIT as i32), Ordering::Relaxed);
                }
                u.reset_interrupt();
            }
        }
    });
}

pub struct Motors {
    channel_a: channel::Channel<'static, LowSpeed>,
    channel_b: channel::Channel<'static, LowSpeed>,
    ain1: Output<'static>,
    ain2: Output<'static>,
    bin1: Output<'static>,
    bin2: Output<'static>,
    _enc_left: Input<'static>,
    _enc_right: Input<'static>,
    counter_left: Counter<'static, 0>,
    counter_right: Counter<'static, 1>,
}

impl Motors {
    pub fn new(
        ledc_p: LEDC<'static>,
        ain1_pin: impl OutputPin + 'static,
        ain2_pin: impl OutputPin + 'static,
        pwma_pin: impl OutputPin + 'static,
        bin1_pin: impl OutputPin + 'static,
        bin2_pin: impl OutputPin + 'static,
        pwmb_pin: impl OutputPin + 'static,
        pcnt_p: PCNT<'static>,
        enc_left_pin: impl InputPin + 'static,
        enc_right_pin: impl InputPin + 'static,
    ) -> Self {
        let mut ledc = Ledc::new(ledc_p);
        ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);

        let mut timer = ledc.timer::<LowSpeed>(timer::Number::Timer0);
        timer
            .configure(timer::config::Config {
                duty: timer::config::Duty::Duty8Bit,
                clock_source: timer::LSClockSource::APBClk,
                frequency: Rate::from_hz(20_000),
            })
            .unwrap();

        // Leak the timer into a 'static reference so channels can hold it.
        let timer_ref: &'static timer::Timer<'static, LowSpeed> = Box::leak(Box::new(timer));

        let mut channel_a = ledc.channel(channel::Number::Channel0, pwma_pin);
        channel_a
            .configure(channel::config::Config {
                timer: timer_ref,
                duty_pct: 0,
                drive_mode: DriveMode::PushPull,
            })
            .unwrap();

        let mut channel_b = ledc.channel(channel::Number::Channel1, pwmb_pin);
        channel_b
            .configure(channel::config::Config {
                timer: timer_ref,
                duty_pct: 0,
                drive_mode: DriveMode::PushPull,
            })
            .unwrap();

        let cfg = OutputConfig::default();
        let ain1 = Output::new(ain1_pin, Level::Low, cfg);
        let ain2 = Output::new(ain2_pin, Level::Low, cfg);
        let bin1 = Output::new(bin1_pin, Level::Low, cfg);
        let bin2 = Output::new(bin2_pin, Level::Low, cfg);

        // Forget ledc to prevent any Drop side-effects on the peripheral token;
        // channels and timer continue to operate via their 'static register references.
        core::mem::forget(ledc);

        // --- PCNT encoder setup ---
        let mut pcnt = Pcnt::new(pcnt_p);
        pcnt.set_interrupt_handler(encoder_isr);

        let enc_cfg = InputConfig::default().with_pull(Pull::Up);
        let enc_left = Input::new(enc_left_pin, enc_cfg);
        let enc_right = Input::new(enc_right_pin, enc_cfg);
        let sig_left = enc_left.peripheral_input();
        let sig_right = enc_right.peripheral_input();

        // Unit 0: left encoder
        let u0 = pcnt.unit0;
        u0.set_low_limit(Some(-LIMIT)).unwrap();
        u0.set_high_limit(Some(LIMIT)).unwrap();
        u0.set_filter(Some((10u16 * 80).min(1023))).unwrap(); // 10 µs glitch filter at 80 MHz APB
        u0.clear();
        u0.channel0.set_edge_signal(sig_left);
        u0.channel0.set_ctrl_mode(CtrlMode::Keep, CtrlMode::Keep);
        u0.channel0
            .set_input_mode(EdgeMode::Increment, EdgeMode::Increment);
        u0.listen();
        u0.resume();
        let counter_left = u0.counter.clone();
        critical_section::with(|cs| UNIT_LEFT.borrow_ref_mut(cs).replace(u0));

        // Unit 1: right encoder
        let u1 = pcnt.unit1;
        u1.set_low_limit(Some(-LIMIT)).unwrap();
        u1.set_high_limit(Some(LIMIT)).unwrap();
        u1.set_filter(Some(10 * 80)).unwrap();
        u1.clear();
        u1.channel0.set_edge_signal(sig_right);
        u1.channel0.set_ctrl_mode(CtrlMode::Keep, CtrlMode::Keep);
        u1.channel0
            .set_input_mode(EdgeMode::Increment, EdgeMode::Increment);
        u1.listen();
        u1.resume();
        let counter_right = u1.counter.clone();
        critical_section::with(|cs| UNIT_RIGHT.borrow_ref_mut(cs).replace(u1));
        // unit0 and unit1 are moved into statics; Rust auto-drops remaining fields.
        // Each Unit holds its own GenericPeripheralGuard so PCNT stays clocked.

        Self {
            channel_a,
            channel_b,
            ain1,
            ain2,
            bin1,
            bin2,
            _enc_left: enc_left,
            _enc_right: enc_right,
            counter_left,
            counter_right,
        }
    }

    /// Set motor velocities. Values are clamped to -100..=100.
    /// Positive = forward, negative = reverse, zero = coast.
    pub fn set_velocity(&mut self, left: i32, right: i32) {
        self.drive_a(left);
        self.drive_b(right);
    }

    pub fn stop(&mut self) {
        self.set_velocity(0, 0);
    }

    /// Total accumulated encoder ticks for the left motor (both edges counted).
    pub fn ticks_left(&self) -> i32 {
        self.counter_left.get() as i32 + TICKS_LEFT.load(Ordering::Relaxed)
    }

    /// Total accumulated encoder ticks for the right motor (both edges counted).
    pub fn ticks_right(&self) -> i32 {
        self.counter_right.get() as i32 + TICKS_RIGHT.load(Ordering::Relaxed)
    }

    /// Reset both encoder tick counts to zero.
    pub fn reset_encoders(&self) {
        TICKS_LEFT.store(0, Ordering::Relaxed);
        TICKS_RIGHT.store(0, Ordering::Relaxed);
        critical_section::with(|cs| {
            if let Some(u) = UNIT_LEFT.borrow_ref(cs).as_ref() {
                u.clear();
            }
            if let Some(u) = UNIT_RIGHT.borrow_ref(cs).as_ref() {
                u.clear();
            }
        });
    }

    fn drive_a(&mut self, speed: i32) {
        let duty = speed.unsigned_abs().min(100) as u8;
        if speed > 0 {
            self.ain1.set_high();
            self.ain2.set_low();
        } else if speed < 0 {
            self.ain1.set_low();
            self.ain2.set_high();
        } else {
            self.ain1.set_low();
            self.ain2.set_low();
        }
        self.channel_a.set_duty(duty).unwrap();
    }

    fn drive_b(&mut self, speed: i32) {
        let duty = speed.unsigned_abs().min(100) as u8;
        if speed > 0 {
            self.bin1.set_high();
            self.bin2.set_low();
        } else if speed < 0 {
            self.bin1.set_low();
            self.bin2.set_high();
        } else {
            self.bin1.set_low();
            self.bin2.set_low();
        }
        self.channel_b.set_duty(duty).unwrap();
    }
}
