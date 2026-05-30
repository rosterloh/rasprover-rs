# RaspRover

Monorepo for the [Waveshare RaspRover](https://www.waveshare.com/wiki/RaspRover) mobile robot platform.

## Repository layout

```
firmware/   ESP32 DevKit-C embedded firmware (Rust, no_std + Embassy)
robot/      ROS 2 workspace for the Raspberry Pi host (Lyrical Luth)
data/       Mechanical CAD and reference documents
```

The ESP32 ROS Driver Board handles low-level motor control, IMU, display, and WiFi.
The Raspberry Pi runs the ROS 2 stack for navigation, perception, and simulation.

## Prerequisites

Install [pixi](https://pixi.sh) and [rustup](https://rustup.rs), then clone the repo:

```bash
git clone https://github.com/rosterloh/rasprover-rs
cd rasprover-rs
```

The two environments are managed independently — install only what you need.

## Firmware (ESP32)

The firmware uses the Espressif `esp` Xtensa toolchain, which lives outside pixi. Install it once:

```bash
pixi run -e firmware setup-toolchain
source ~/export-esp.sh
```

Configure WiFi credentials (optional — defaults are set in `firmware/.cargo/config.toml`):

```bash
export CONFIG_WIFI_NETWORK="your-ssid"
export CONFIG_WIFI_PASSWORD="your-password"
```

| Task | Command |
|------|---------|
| Install environment | `pixi install -e firmware` |
| Build | `pixi run -e firmware build` |
| Build & flash | `pixi run -e firmware flash` |
| Serial monitor | `pixi run -e firmware monitor` |
| Save binary image | `pixi run -e firmware firmware-image` |
| Erase flash | `pixi run -e firmware erase` |
| Binary size stats | `pixi run -e firmware stats` |

Logs stream over RTT. Level is controlled by `DEFMT_LOG` (default: `info`).

### Firmware modules

| Module | Description |
|--------|-------------|
| `board` | Shared I2C bus and peripheral initialisation |
| `display` | SSD1306 OLED via I2C |
| `imu` | MPU-6050 with Kalman / Mahony filter |
| `motors` | PWM motor control via LEDC |
| `network` | WiFi station + DHCP, OTA updates |

## Robot (ROS 2)

The ROS 2 workspace uses [RoboStack Lyrical](https://robostack.github.io) via pixi.

```bash
pixi install -e ros
pixi run -e ros build       # first build (also generates robot/install/setup.sh)
```

> The `ros` environment activation sources `robot/install/setup.sh`, which is generated
> by colcon. Run `build` once before opening a `pixi shell -e ros`.

| Task | Command |
|------|---------|
| Install environment | `pixi install -e ros` |
| Build workspace | `pixi run -e ros build` |
| Clean workspace | `pixi run -e ros clean` |
| Run on hardware | `pixi run -e ros start` |
| Run simulation | `pixi run -e ros simulation` |
| Run Gazebo | `pixi run -e ros gazebo` |
| Open RViz | `pixi run -e ros rviz` |

### ROS packages

| Package | Description |
|---------|-------------|
| `rasprover_bringup` | Launch files for hardware and simulation |
| `rasprover_description` | URDF, meshes, and RViz config |
| `rasprover_gz` | Gazebo simulation integration |

## Links

- [Waveshare RaspRover Wiki](https://www.waveshare.com/wiki/RaspRover)
- [Rust on ESP Book](https://docs.espressif.com/projects/rust/book/)
- [Embassy Book](https://embassy.dev/book/)
- [Embedded Rust Book](https://docs.rust-embedded.org/book/)
- [RoboStack](https://robostack.github.io)
- [pixi](https://pixi.sh/latest/)
