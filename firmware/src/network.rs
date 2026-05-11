extern crate alloc;

use core::fmt::Write as _;

use alloc::string::String as AllocString;
use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_net::{Runner, Stack, StackResources};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, watch::Watch};
use embassy_time::Timer;
use esp_hal::rng::Rng;
use esp_radio::wifi::{Config, Interface, WifiController, sta::StationConfig};
use heapless::String;
use static_cell::StaticCell;

const SSID: &str = env!("CONFIG_WIFI_NETWORK");
const PASSWORD: &str = env!("CONFIG_WIFI_PASSWORD");

#[derive(Clone, PartialEq, Eq, defmt::Format)]
pub enum NetworkState {
    Connecting,
    Up(String<18>),
    Down,
}

pub static NET_STATE: Watch<CriticalSectionRawMutex, NetworkState, 2> = Watch::new();

pub fn init(
    wifi: esp_hal::peripherals::WIFI<'static>,
    rng: Rng,
    spawner: Spawner,
) -> Stack<'static> {
    let (controller, interfaces) = esp_radio::wifi::new(wifi, Default::default())
        .expect("Failed to initialise Wi-Fi controller");
    let seed = (rng.random() as u64) << 32 | rng.random() as u64;
    let wifi_device: Interface<'static> = interfaces.station;

    static RESOURCES: StaticCell<StackResources<4>> = StaticCell::new();
    let (stack, runner) = embassy_net::new(
        wifi_device,
        embassy_net::Config::dhcpv4(Default::default()),
        RESOURCES.init_with(StackResources::new),
        seed,
    );

    spawner.spawn(wifi_connection_task(controller).unwrap());
    spawner.spawn(net_runner_task(runner).unwrap());
    spawner.spawn(network_monitor(stack).unwrap());

    stack
}

#[embassy_executor::task]
async fn wifi_connection_task(mut controller: WifiController<'static>) {
    let station_config = Config::Station(
        StationConfig::default()
            .with_ssid(SSID)
            .with_password(AllocString::from(PASSWORD)),
    );

    if let Err(e) = controller.set_config(&station_config) {
        warn!("wifi: set_config failed: {:?}", e);
        return;
    }
    info!("wifi: configured for {}", SSID);

    loop {
        if controller.is_connected() {
            let _ = controller.wait_for_disconnect_async().await;
            info!("wifi: disconnected");
        }

        info!("wifi: connecting to {}", SSID);
        match controller.connect_async().await {
            Ok(_) => info!("wifi: connected"),
            Err(e) => {
                warn!("wifi: connect failed: {:?}", e);
                Timer::after_millis(500).await;
            }
        }
    }
}

#[embassy_executor::task]
async fn net_runner_task(mut runner: Runner<'static, Interface<'static>>) {
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
