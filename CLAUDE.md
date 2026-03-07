# CLAUDE.md

## MANDATORY: Use td for Task Management

Run td usage --new-session at conversation start (or after /clear). This tells you what to work on next.

Sessions are automatic (based on terminal/agent context). Optional:
- td session "name" to label the current session
- td session --new to force a new session in the same context

Use td usage -q after first read.

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

RaspRover-RS is embedded firmware for the [Waveshare RaspRover](https://www.waveshare.com/wiki/RaspRover) robot platform, targeting an Espressif ESP32 DevKit-C (Xtensa architecture). It uses [Ariel OS](https://ariel-os.github.io/ariel-os/dev/docs/book/introduction.html) v0.3.0 as the embedded OS framework, which wraps [Embassy](https://embassy.dev/book/) for async/await support.

## Build Commands

The project uses **Laze** as a meta-build system on top of Cargo. Standard `cargo build` will not work directly.

```bash
# Build and flash to connected ESP32
laze build -b espressif-esp32-devkitc run

# Build only (no flash)
laze build -b espressif-esp32-devkitc
```

Before building, WiFi credentials must be configured:
```bash
cp .env.example .env
# Edit .env with your WiFi SSID and password
source .env
```

## Toolchain

This project requires the **ESP Rust toolchain** (not stable/nightly), which supports the Xtensa instruction set used by ESP32:
- Toolchain: `esp` (`RUSTUP_TOOLCHAIN=esp`)
- Target: `xtensa-esp32-none-elf`

Rust Analyzer is configured via `.vscode/settings.json` with the correct `RUSTFLAGS` and environment variables for the ESP32 target. If using VS Code, these settings are picked up automatically.

## Architecture

The codebase is `no_std` + `no_main` — standard library and Rust runtime are unavailable. All code runs in an async executor provided by Ariel OS. However, `alloc` **is** available — the build enables `ariel-os/alloc` and the ESP32 heap is configured, so `Box`, `Vec`, etc. work fine.

**Entry point pattern:**
```rust
#[ariel_os::task(autostart)]
async fn main() {
    // application code
}
```

**Key dependency structure:**
- `ariel-os` (path: `build/imports/ariel-os/src/ariel-os`) — the top-level OS crate
- `ariel-os-boards` — board support for ESP32 DevKit-C
- Both are vendored locally under `build/imports/ariel-os/` (imported by Laze from GitHub)

**Cargo configuration** (`.cargo/config.toml`) includes the Ariel OS cargo config via an unstable `config-include` feature. This sets the build target, linker arguments, and other embedded build settings.

**Feature selection** for Ariel OS capabilities (threading, networking, storage, etc.) is done in `laze-project.yml` via `selects:` and in `Cargo.toml` via `features = []` on the `ariel-os` dependency.

## Logging

Uses `defmt` structured logging. Log level is controlled by the `DEFMT_LOG` environment variable (default: `info` in VS Code settings). Use macros from `ariel_os::debug::log`:
```rust
use ariel_os::debug::log::info;
info!("message {}", value);
```

## Linting

Clippy is configured in VS Code settings with the ESP32 build flags. To run manually, you need the full set of `RUSTFLAGS` and environment variables matching those in `.vscode/settings.json`.

## cfg Contexts

The build system passes `--cfg context="..."` flags for the target board, chip family, and OS. These are used throughout Ariel OS for conditional compilation. Unexpected cfg warnings are expected and configured to `warn` level in `Cargo.toml`.

## Peripheral Patterns

**Peripherals struct** (`src/pins.rs`) uses `ariel_os::hal::define_peripherals!` which maps friendly names to `hal::peripherals::GPIOxx` types. Each field is a concrete type like `GPIO21<'static>`. Pass individual fields to functions rather than the whole struct when only some pins are needed.

**Ariel OS provides `'static` peripherals** — all peripheral tokens (`GPIO21<'static>`, `LEDC<'static>`, etc.) have `'static` lifetime, which propagates through esp-hal types (`Output<'static>`, `Channel<'static, LowSpeed>`, etc.).

**PWM (LEDC)**: Ariel OS v0.3.0 has no PWM abstraction. Use `esp_hal::ledc` directly with the `unstable` feature. Add `esp-hal = { version = "1.0.0", default-features = false, features = ["unstable"] }` to `Cargo.toml` — the `[patch]` in `ariel-os-cargo.toml` automatically redirects it to the ariel-os fork (`rev = 531c629`). LEDC source is in `~/.cargo/git/checkouts/esp-hal-*/531c629/esp-hal/src/ledc/`.

**Self-referential HAL structs**: `Channel<'a, LowSpeed>` holds a `&'a dyn TimerIFace` so the timer must outlive the channel. Use `Box::leak(Box::new(timer))` to get a `&'static Timer` — this is safe and idiomatic given the always-available heap.

**`i2c_bus::init`** takes individual `GPIO32<'static>` and `GPIO33<'static>` pins (not the whole `Peripherals` struct) to allow partial moves when other pins are claimed first.

## References

- [Ariel OS Manual](https://ariel-os.github.io/ariel-os/dev/docs/book/introduction.html)
- [Ariel OS examples](build/imports/ariel-os/examples/) — useful reference for adding features
- [Embassy Book](https://embassy.dev/book/)
- [Embedded Rust Book](https://docs.rust-embedded.org/book/)
