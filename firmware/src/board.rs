use core::sync::atomic::{AtomicPtr, Ordering};

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use esp_hal::{
    Async,
    gpio::AnyPin,
    i2c::master::{BusTimeout, Config, I2c},
    peripherals::{GPIO32, GPIO33, I2C0, LEDC, PCNT},
    time::Rate,
};
use static_cell::StaticCell;

pub type SharedI2c = Mutex<CriticalSectionRawMutex, I2c<'static, Async>>;

static I2C_BUS_CELL: StaticCell<SharedI2c> = StaticCell::new();
static I2C_BUS_PTR: AtomicPtr<SharedI2c> = AtomicPtr::new(core::ptr::null_mut());

/// Returns a reference to the shared I2C bus. Panics if called before `board::init()`.
pub fn get_i2c_bus() -> &'static SharedI2c {
    let ptr = I2C_BUS_PTR.load(Ordering::Acquire);
    // Safety: pointer is written once in init() before any consumers are spawned.
    unsafe { &*ptr }
}

pub struct Peripherals {
    pub i2c_sda:    GPIO32<'static>,
    pub i2c_scl:    GPIO33<'static>,
    pub i2c0:       I2C0<'static>,
    pub motor_ain1: AnyPin<'static>,
    pub motor_ain2: AnyPin<'static>,
    pub motor_bin1: AnyPin<'static>,
    pub motor_bin2: AnyPin<'static>,
    pub motor_pwma: AnyPin<'static>,
    pub motor_pwmb: AnyPin<'static>,
    pub ledc:       LEDC<'static>,
    pub pcnt:       PCNT<'static>,
    pub enc_left:   AnyPin<'static>,
    pub enc_right:  AnyPin<'static>,
}

pub fn init(i2c0: I2C0<'static>, sda: GPIO32<'static>, scl: GPIO33<'static>) {
    let i2c_config = Config::default()
        .with_frequency(Rate::from_khz(400))
        .with_timeout(BusTimeout::Maximum);
    let i2c_bus = I2c::new(i2c0, i2c_config)
        .unwrap()
        .with_sda(sda)
        .with_scl(scl)
        .into_async();
    let bus_ref = I2C_BUS_CELL.init(Mutex::new(i2c_bus));
    I2C_BUS_PTR.store(bus_ref as *mut _, Ordering::Release);
}
