# RaspRover Firmware

### Setup

Setup according to the [Ariel OS getting started](https://ariel-os.github.io/ariel-os/dev/docs/book/getting-started.html) guide.
Copy the [.env.example](.env.example) file to `.env` and make your modifications. Source `.env` before building.

### Building

```bash
laze build -b espressif-esp32-devkitc run
```

### Useful Links

- [Waveshare Wiki](https://www.waveshare.com/wiki/RaspRover)
- [Ariel OS manual](https://ariel-os.github.io/ariel-os/dev/docs/book/introduction.html)
- [Embedded Rust Book](https://docs.rust-embedded.org/book/)
- [Embassy Book](https://embassy.dev/book/)