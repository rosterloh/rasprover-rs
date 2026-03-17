# RaspRover Firmware

### Setup

Setup according to the [Rust on ESP Getting Started](https://docs.espressif.com/projects/rust/book/getting-started/index.html) guide.

```bash
cargo install espup --locked
espup install
cargo install espflash --locked
cargo install --locked probe-rs-tools
```

Copy the [.env.example](.env.example) file to `.env` and make your modifications. Source `~/export-esp.sh` and `.env` before building.

### Building

```bash
source ~/export-esp.sh
source .env
cargo build --release
```

### Flashing

```bash
source .env
cargo run --release
```

### Useful Links

- [Waveshare Wiki](https://www.waveshare.com/wiki/RaspRover)
- [Rust on ESP Book](https://docs.espressif.com/projects/rust/book/)
- [Embassy Book](https://embassy.dev/book/)
- [Embedded Rust Book](https://docs.rust-embedded.org/book/)
