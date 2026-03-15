extern crate alloc;

use core::fmt::Write as _;

use alloc::string::ToString;
use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_net::{Runner, Stack, StackResources};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, watch::Watch};
use embassy_time::Timer;
use esp_hal::rng::Rng;
use esp_radio::Controller;
use esp_radio::wifi::{ClientConfig, ModeConfig, WifiController, WifiDevice, WifiEvent};
use heapless::String;
use static_cell::StaticCell;

const SSID: &str = env!("CONFIG_WIFI_NETWORK");
const PASSWORD: &str = env!("CONFIG_WIFI_PASSWORD");

#[derive(Clone, PartialEq, Eq)]
pub enum NetworkState {
    Connecting,
    Up(String<18>),
    Down,
}

static WIFI_RADIO: StaticCell<Controller> = StaticCell::new();
pub static NET_STATE: Watch<CriticalSectionRawMutex, NetworkState, 2> = Watch::new();

pub fn init(
    wifi: esp_hal::peripherals::WIFI<'static>,
    rng: Rng,
    spawner: Spawner,
) -> Stack<'static> {
    let esp_radio_ctrl =
        &*WIFI_RADIO.init(esp_radio::init().expect("Failed to initialise Wi-Fi/BLE controller"));
    let (controller, interfaces) = esp_radio::wifi::new(&esp_radio_ctrl, wifi, Default::default())
        .expect("Failed to initialise Wi-Fi controller");
    let seed = (rng.random() as u64) << 32 | rng.random() as u64;
    let wifi_device: WifiDevice<'static> = interfaces.sta;

    static RESOURCES: StaticCell<StackResources<4>> = StaticCell::new();
    let (stack, runner) = embassy_net::new(
        wifi_device,
        embassy_net::Config::dhcpv4(Default::default()),
        RESOURCES.init_with(StackResources::new),
        seed,
    );

    spawner.spawn(wifi_connection_task(controller)).ok();
    spawner.spawn(net_runner_task(runner)).ok();
    spawner.spawn(network_monitor(stack)).ok();

    stack
}

#[embassy_executor::task]
async fn wifi_connection_task(mut controller: WifiController<'static>) {
    loop {
        if matches!(controller.is_connected(), Ok(true)) {
            controller.wait_for_event(WifiEvent::StaDisconnected).await;
            info!("wifi: disconnected");
        }

        let config = ClientConfig::default()
            .with_ssid(SSID.to_string())
            .with_password(PASSWORD.to_string());
        controller.set_config(&ModeConfig::Client(config)).unwrap();

        if !matches!(controller.is_started(), Ok(true)) {
            controller.start_async().await.unwrap();
            info!("wifi: started");
        }

        info!("wifi: connecting to {}", SSID);
        match controller.connect_async().await {
            Ok(()) => info!("wifi: connected"),
            Err(e) => {
                warn!("wifi: connect failed: {:?}", e);
                Timer::after_millis(500).await;
            }
        }
    }
}

#[embassy_executor::task]
async fn net_runner_task(mut runner: Runner<'static, WifiDevice<'static>>) {
    runner.run().await;
}

#[embassy_executor::task]
async fn network_monitor(stack: Stack<'static>) {
    let sender = NET_STATE.sender();

    loop {
        info!("network: connecting...");
        crate::display::set_line(1, "Connecting...");
        sender.send(NetworkState::Connecting);
        stack.wait_config_up().await;

        if let Some(config) = stack.config_v4() {
            let mut ip_str: String<18> = String::new();
            write!(ip_str, "{}", config.address.address()).unwrap();
            info!("network: up, IP: {}", ip_str.as_str());
            crate::display::set_line(1, ip_str.as_str());
            sender.send(NetworkState::Up(ip_str));
        }

        loop {
            Timer::after_secs(5).await;
            if !stack.is_config_up() {
                warn!("network: down, will reconnect");
                crate::display::set_line(1, "Disconnected");
                sender.send(NetworkState::Down);
                break;
            }
        }
    }
}
