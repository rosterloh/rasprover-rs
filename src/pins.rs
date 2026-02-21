use ariel_os::hal::{i2c, peripherals};

#[cfg(context = "espressif-esp32-devkitc")]
pub type DisplayI2c = i2c::controller::I2C0;

#[cfg(context = "espressif-esp32-devkitc")]
ariel_os::hal::define_peripherals!(Peripherals {
    i2c_sda: GPIO32,
    i2c_scl: GPIO33,
});
