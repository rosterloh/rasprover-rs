//! Smoke tests for the display module's line-state management.
//!
//! These tests verify the `set_line` API behaves correctly across edge cases
//! without requiring any display hardware to be connected.

#![no_std]
#![no_main]

esp_bootloader_esp_idf::esp_app_desc!();

#[cfg(test)]
#[embedded_test::tests(executor = esp_rtos::embassy::Executor::new())]
mod tests {
    use rasprover_rs::display::set_line;

    #[init]
    fn init() {
        let peripherals = esp_hal::init(esp_hal::Config::default());
        let timg1 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG1);
        esp_rtos::start(timg1.timer0);
        rtt_target::rtt_init_defmt!();
    }

    #[test]
    async fn set_line_0_does_not_panic() {
        set_line(0, "Hello World");
    }

    #[test]
    async fn set_line_1_does_not_panic() {
        set_line(1, "Status line");
    }

    #[test]
    async fn set_line_2_does_not_panic() {
        set_line(2, "Bottom line");
    }

    #[test]
    async fn set_line_with_empty_string_does_not_panic() {
        set_line(0, "");
        set_line(1, "");
        set_line(2, "");
    }

    #[test]
    async fn set_line_truncates_text_longer_than_21_chars() {
        // Line capacity is 21 chars; push_str silently truncates — must not panic.
        set_line(0, "This text is definitely longer than twenty-one characters");
    }

    #[test]
    async fn overwriting_a_line_does_not_panic() {
        set_line(0, "First value");
        set_line(0, "Second value");
    }

    #[test]
    async fn all_lines_can_be_set_independently() {
        set_line(0, "Line zero");
        set_line(1, "Line one");
        set_line(2, "Line two");
    }
}
