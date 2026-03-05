extern crate alloc;

use alloc::boxed::Box;
use esp_hal::{
    gpio::{DriveMode, Level, Output, OutputConfig, OutputPin},
    ledc::{
        LSGlobalClkSource, Ledc, LowSpeed,
        channel::{self, ChannelIFace},
        timer::{self, TimerIFace},
    },
    peripherals::LEDC,
    time::Rate,
};

pub struct Motors {
    channel_a: channel::Channel<'static, LowSpeed>,
    channel_b: channel::Channel<'static, LowSpeed>,
    ain1: Output<'static>,
    ain2: Output<'static>,
    bin1: Output<'static>,
    bin2: Output<'static>,
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

        Self {
            channel_a,
            channel_b,
            ain1,
            ain2,
            bin1,
            bin2,
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
