# CLAUDE.md

## Project Overview

RaspRover-RS is embedded firmware for the [Waveshare RaspRover](https://www.waveshare.com/wiki/RaspRover) robot platform, targeting an Espressif ESP32 DevKit-C (Xtensa architecture). It uses [esp-rtos](https://github.com/ariel-os/esp-hal) with [Embassy](https://embassy.dev/book/) for async/await support. All esp-hal family crates are pinned to the ariel-os fork at `rev = 531c629`.

## Build Commands

Standard Cargo is used directly — no meta-build system required.

```bash
# Build only
cargo build --release

# Build and flash to connected ESP32
cargo run --release
```

WiFi credentials are configured via environment variables (with fallback defaults in `.cargo/config.toml`):
```bash
export CONFIG_WIFI_NETWORK="your-ssid"
export CONFIG_WIFI_PASSWORD="your-password"
```

## Toolchain

This project requires the **ESP Rust toolchain** (not stable/nightly), which supports the Xtensa instruction set used by ESP32:
- Toolchain: `esp` (`RUSTUP_TOOLCHAIN=esp`)
- Target: `xtensa-esp32-none-elf`

Rust Analyzer is configured via `.vscode/settings.json` with the correct `RUSTFLAGS` and environment variables for the ESP32 target. If using VS Code, these settings are picked up automatically.

## Architecture

The codebase is `no_std` + `no_main` — standard library and Rust runtime are unavailable. All async code runs in the Embassy executor provided by `esp-rtos`. However, `alloc` **is** available — the heap is configured with `esp_alloc::heap_allocator!`, so `Box`, `Vec`, etc. work fine.

**Entry point pattern:**
```rust
#![no_main]
#![no_std]

extern crate alloc;

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(spawner: Spawner) {
    rtt_target::rtt_init_defmt!();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let p = esp_hal::init(config);
    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 98768);

    let timg0 = TimerGroup::new(p.TIMG0);
    esp_rtos::start(timg0.timer0);

    // application code
}
```

`esp_rtos::start(...)` must be called **inside** the async main function after heap init. `SoftwareInterruptControl` is no longer required — `esp_rtos::start` takes only the timer.

**Key dependency structure:**
- `esp-rtos` — async runtime (Embassy executor + timer integration)
- `esp-radio` — WiFi via `esp_radio::wifi`
- `embassy-net` — TCP/IP networking
- `embassy-sync` — synchronisation primitives (`Watch`, `Mutex`, etc.)
- All esp-hal family crates are patched to `git = "https://github.com/ariel-os/esp-hal", rev = "531c629..."`

**Cargo configuration** (`.cargo/config.toml`) is self-contained — no external includes. Sets the build target, linker arguments, `build-std`, and WiFi credential defaults.

## Logging

Uses `defmt` structured logging via RTT. Log level is controlled by the `DEFMT_LOG` environment variable (default: `info` in `.cargo/config.toml`). Import macros directly from `defmt`:
```rust
use defmt::{debug, error, info, warn};
use panic_rtt_target as _;
info!("message {}", value);
```

Initialize RTT at the top of `main` before any logging:
```rust
rtt_target::rtt_init_defmt!();
```

The panic handler is provided by `panic-rtt-target`. The `rtt-target` crate (with `features = ["defmt"]`) provides the transport.

## Peripheral Patterns

**Peripherals struct** (`src/pins.rs`) is a plain Rust struct. Use concrete peripheral types from `esp_hal::peripherals` for typed pins and `AnyPin<'static>` (via `.into()`) for interchangeable pins:
```rust
pub struct Peripherals {
    pub i2c_sda: GPIO32<'static>,
    pub i2c_scl: GPIO33<'static>,
    pub motor_ain1: AnyPin<'static>,  // p.GPIO21.into()
    // ...
}
```

**All peripherals have `'static` lifetime** — `GPIO21<'static>`, `LEDC<'static>`, etc. This propagates through esp-hal types (`Output<'static>`, `Channel<'static, LowSpeed>`, etc.).

**PWM (LEDC)**: Use `esp_hal::ledc` directly with the `unstable` feature. The `[patch]` in `Cargo.toml` redirects to the ariel-os fork. LEDC source is in `~/.cargo/git/checkouts/esp-hal-*/531c629/esp-hal/src/ledc/`.

**Self-referential HAL structs**: `Channel<'a, LowSpeed>` holds a `&'a dyn TimerIFace` so the timer must outlive the channel. Use `Box::leak(Box::new(timer))` to get a `&'static Timer` — this is safe given the always-available heap.

**I2C**: Use `esp_hal::i2c::master::I2c` directly:
```rust
I2c::new(i2c0, config).unwrap().with_sda(sda).with_scl(scl).into_async()
```
Wrap with `embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice` for sharing across tasks.

**Rng**: `Rng::new()` — no arguments.

## WiFi Pattern

WiFi is initialized in `network::init()`:
```rust
let (controller, interfaces) = esp_radio::wifi::new(wifi, Config::default()).unwrap();
let wifi_device: WifiDevice<'static> = interfaces.station;
```
- `controller.is_connected()` — check connection status
- `controller.set_config(&ModeConfig::Station(config))` — configure
- `StationConfig::default().with_ssid(String).with_password(String)` — uses `alloc::string::String`

## Module Design Patterns

**Encapsulate subsystems as spawned tasks** — each major subsystem (network, display, etc.) owns its lifecycle via `#[embassy_executor::task]` functions and a dedicated `src/<subsystem>.rs` module. `main` coordinates but does not inline subsystem logic.

**Use `Watch` for state broadcasting** — `embassy_sync::watch::Watch` is the preferred primitive for publishing subsystem state to other tasks:
- Delivers the **current** value to late subscribers (no missed events on join)
- `try_get()` enables non-blocking polling in tight loops
- Declare as a `pub static` in the owning module; consumers call `.receiver().unwrap()`
- Size N to the number of concurrent receivers needed

```rust
// In the owning module (e.g. src/network.rs):
pub static NET_STATE: Watch<CriticalSectionRawMutex, NetworkState, 2> = Watch::new();

// In a consumer:
let mut net_rx = network::NET_STATE.receiver().unwrap();
if let Some(state) = net_rx.try_get() { /* react */ }
```

**Carry data in enum variants** — state enums should embed associated payload directly (e.g. `Up(String<18>)`) rather than storing it separately.

**Track last-seen state to avoid redundant work** — consumers that perform side effects should cache the last processed state and only act when it changes:
```rust
let mut last_state: Option<NetworkState> = None;
if Some(&state) != last_state.as_ref() {
    // redraw / update
    last_state = Some(state);
}
```

## References

- [Embassy Book](https://embassy.dev/book/)
- [esp-hal docs](https://docs.esp-rs.org/esp-hal/)
- [Embedded Rust Book](https://docs.rust-embedded.org/book/)
