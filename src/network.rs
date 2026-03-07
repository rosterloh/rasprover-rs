use core::fmt::Write as _;

use ariel_os::{
    debug::log::{info, warn},
    net,
    time::Timer,
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, watch::Watch};
use heapless::String;

#[derive(Clone, PartialEq, Eq)]
pub enum NetworkState {
    Connecting,
    Up(String<18>),
    Down,
}

// N=2: supports up to 2 concurrent receivers
pub static NET_STATE: Watch<CriticalSectionRawMutex, NetworkState, 2> = Watch::new();

#[ariel_os::task(autostart)]
async fn network_monitor() {
    let stack = net::network_stack().await.unwrap();
    let sender = NET_STATE.sender();

    loop {
        info!("network: connecting...");
        sender.send(NetworkState::Connecting);
        stack.wait_config_up().await;

        if let Some(config) = stack.config_v4() {
            let mut ip_str: String<18> = String::new();
            write!(ip_str, "{}", config.address.address()).unwrap();
            info!("network: up, IP: {}", ip_str.as_str());
            sender.send(NetworkState::Up(ip_str));
        }

        // Poll until link/config drops
        loop {
            Timer::after_secs(5).await;
            if !stack.is_config_up() {
                warn!("network: down, will reconnect");
                sender.send(NetworkState::Down);
                break;
            }
        }
        // Loop back → will immediately send Connecting again
    }
}
