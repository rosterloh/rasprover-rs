use ariel_os::hal::{i2c, peripherals};

#[cfg(context = "espressif-esp32-devkitc")]
pub type I2cBus = i2c::controller::I2C0;

#[cfg(context = "espressif-esp32-devkitc")]
ariel_os::hal::define_peripherals!(Peripherals {
    i2c_sda: GPIO32,
    i2c_scl: GPIO33,
    motor_ain1: GPIO21,
    motor_ain2: GPIO17,
    motor_pwma: GPIO25,
    motor_bin1: GPIO22,
    motor_bin2: GPIO23,
    motor_pwmb: GPIO26,
    ledc: LEDC,
    pcnt: PCNT,
    enc_left: GPIO35, // AENCB 34
    enc_right: GPIO16, // BENCA 27
});
