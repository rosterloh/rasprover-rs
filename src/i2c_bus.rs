use ariel_os::{
    hal,
    i2c::controller::{Kilohertz, highest_freq_in},
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use once_cell::sync::OnceCell;

pub static I2C_BUS: OnceCell<Mutex<CriticalSectionRawMutex, hal::i2c::controller::I2c>> =
    OnceCell::new();

pub fn init(peripherals: crate::pins::Peripherals) {
    let mut config = hal::i2c::controller::Config::default();
    config.frequency = const { highest_freq_in(Kilohertz::kHz(100)..=Kilohertz::kHz(400)) };

    let bus = crate::pins::I2cBus::new(peripherals.i2c_sda, peripherals.i2c_scl, config);
    let _ = I2C_BUS.set(Mutex::new(bus));
}
