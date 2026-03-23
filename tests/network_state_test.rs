//! Tests for the NetworkState enum in the network module.

#![no_std]
#![no_main]

extern crate alloc;

esp_bootloader_esp_idf::esp_app_desc!();

#[cfg(test)]
#[embedded_test::tests(executor = esp_rtos::embassy::Executor::new())]
mod tests {
    use defmt::{assert_eq, assert_ne};
    use heapless::String;
    use rasprover_rs::network::NetworkState;

    #[init]
    fn init() {
        let peripherals = esp_hal::init(esp_hal::Config::default());
        let timg1 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG1);
        esp_rtos::start(timg1.timer0);
        rtt_target::rtt_init_defmt!();
    }

    #[test]
    async fn connecting_state_equals_itself() {
        assert_eq!(NetworkState::Connecting, NetworkState::Connecting);
    }

    #[test]
    async fn down_state_equals_itself() {
        assert_eq!(NetworkState::Down, NetworkState::Down);
    }

    #[test]
    async fn connecting_and_down_are_not_equal() {
        assert_ne!(NetworkState::Connecting, NetworkState::Down);
    }

    #[test]
    async fn up_state_with_same_ip_is_equal() {
        let mut ip: String<18> = String::new();
        ip.push_str("192.168.1.1").ok();
        let a = NetworkState::Up(ip.clone());
        let b = NetworkState::Up(ip);
        assert_eq!(a, b);
    }

    #[test]
    async fn up_state_with_different_ips_is_not_equal() {
        let mut ip1: String<18> = String::new();
        ip1.push_str("192.168.1.1").ok();
        let mut ip2: String<18> = String::new();
        ip2.push_str("10.0.0.1").ok();
        assert_ne!(NetworkState::Up(ip1), NetworkState::Up(ip2));
    }

    #[test]
    async fn up_state_is_not_equal_to_connecting() {
        let mut ip: String<18> = String::new();
        ip.push_str("192.168.1.1").ok();
        assert_ne!(NetworkState::Up(ip), NetworkState::Connecting);
    }

    #[test]
    async fn up_state_is_not_equal_to_down() {
        let mut ip: String<18> = String::new();
        ip.push_str("192.168.1.1").ok();
        assert_ne!(NetworkState::Up(ip), NetworkState::Down);
    }

    #[test]
    async fn network_state_can_be_cloned() {
        let state = NetworkState::Connecting;
        let cloned = state.clone();
        assert_eq!(state, cloned);
    }

    #[test]
    async fn up_state_clone_preserves_ip() {
        let mut ip: String<18> = String::new();
        ip.push_str("10.0.0.42").ok();
        let original = NetworkState::Up(ip);
        let cloned = original.clone();
        assert_eq!(original, cloned);
    }
}
