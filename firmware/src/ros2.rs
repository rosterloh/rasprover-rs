//! ROS 2 telemetry over Zenoh — IMU publish spike.
//!
//! Connects to a `rmw_zenoh` router over TCP and publishes the fused IMU
//! attitude as a `sensor_msgs/msg/Imu` on `/imu`, using the `no_std`
//! [`zenoh-ros2-nostd`](https://github.com/Baker-Tanaka/zenoh_ros2_nostd)
//! library. This is an exploratory spike — see `CLAUDE.md`.
//!
//! Architecture mirrors the library's esp32c3 example:
//! - [`session_task`] owns the Zenoh session lifecycle (TCP → handshake →
//!   spin → reconnect).
//! - [`IMU_PUB`] is a `static` publisher any task can [`publish`] into; the
//!   session task drains it inside `Node::spin`.
//!
//! # Running it
//!
//! 1. Point the firmware at your network and router (defaults in
//!    `.cargo/config.toml`):
//!    ```sh
//!    export CONFIG_WIFI_NETWORK="your-ssid"
//!    export CONFIG_WIFI_PASSWORD="your-password"
//!    export CONFIG_ZENOH_ROUTER="192.168.1.42:7447"   # the Pi's ip:port
//!    ```
//! 2. On the Pi host, run a Zenoh router so ROS 2 nodes share the same graph:
//!    ```sh
//!    export RMW_IMPLEMENTATION=rmw_zenoh_cpp
//!    ros2 run rmw_zenoh_cpp rmw_zenohd
//!    ```
//! 3. Flash the firmware (`pixi run -e firmware flash`) and echo the topic:
//!    ```sh
//!    RMW_IMPLEMENTATION=rmw_zenoh_cpp ros2 topic echo /imu sensor_msgs/msg/Imu
//!    ```
//!
//! The `/imu` key expression embeds `IMU_TYPE_HASH`; it must match the host's
//! `sensor_msgs/msg/Imu` RIHS01 hash or `echo` sees nothing.

use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_net::{IpEndpoint, Ipv4Address, Stack, tcp::TcpSocket};
use embassy_time::{Duration, Instant, Timer, with_timeout};
use serde::Serialize;
use zenoh_ros2_nostd::{
    ros2::{Publisher, TopicKeyExpr, node::NodeBuilder},
    session::ReconnectPolicy,
    transport::protocol::ZenohId,
};

const DEG_TO_RAD: f64 = core::f64::consts::PI / 180.0;
const G_TO_MS2: f64 = 9.806_65;

// ── ROS 2 topic definition ───────────────────────────────────────────────────

/// rmw_zenoh DDS-mangled type name for `sensor_msgs/msg/Imu`.
const IMU_TYPE_NAME: &str = "sensor_msgs::msg::dds_::Imu_";

/// RIHS01 type hash for `sensor_msgs/msg/Imu`.
///
/// Content-addressed per REP-2011, so it is identical across ROS distros with
/// an unchanged `Imu` definition — verified equal for Kilted and Lyrical
/// (`share/sensor_msgs/msg/Imu.json` → `type_hashes[sensor_msgs/msg/Imu]`).
const IMU_TYPE_HASH: &str =
    "RIHS01_7d9a00ff131080897a5ec7e26e315954b8eae3353c3f995c55faf71574000b5b";

const IMU_TOPIC: TopicKeyExpr = TopicKeyExpr::new(0, "imu", IMU_TYPE_NAME, IMU_TYPE_HASH);

/// Device Zenoh ID — must be unique per device on the network.
const DEVICE_ZID: [u8; 8] = [0xBA, 0xBE, 0xCA, 0xFE, 0x00, 0x01, 0x02, 0x03];

/// CDR buffer capacity for an `Imu` message (must be ≥ serialized size and
/// ≤ the node's 512-byte drain buffer). Serialized `Imu` is ~340 bytes.
const IMU_CDR_CAP: usize = 512;

// ── sensor_msgs/Imu CDR layout ───────────────────────────────────────────────
// Fields are serialized in declaration order; the library's CDR serializer
// applies OMG alignment automatically (f64 → 8-byte boundaries).

#[derive(Serialize)]
struct Time {
    sec: i32,
    nanosec: u32,
}

#[derive(Serialize)]
struct Header {
    stamp: Time,
    frame_id: &'static str,
}

#[derive(Serialize)]
struct Quaternion {
    x: f64,
    y: f64,
    z: f64,
    w: f64,
}

#[derive(Serialize)]
struct Vector3 {
    x: f64,
    y: f64,
    z: f64,
}

/// `sensor_msgs/msg/Imu`.
#[derive(Serialize)]
pub struct Imu {
    header: Header,
    orientation: Quaternion,
    orientation_covariance: [f64; 9],
    angular_velocity: Vector3,
    angular_velocity_covariance: [f64; 9],
    linear_acceleration: Vector3,
    linear_acceleration_covariance: [f64; 9],
}

impl Imu {
    /// Build an `Imu` from fused sensor values.
    ///
    /// - `quat`: attitude quaternion `[w, x, y, z]` (Mahony convention)
    /// - `gyr_dps`: angular velocity in deg/s (converted to rad/s)
    /// - `acc_g`: linear acceleration in g (converted to m/s²)
    fn from_sensors(quat: [f32; 4], gyr_dps: [f32; 3], acc_g: [f32; 3]) -> Self {
        let now = Instant::now();
        let stamp = Time {
            sec: now.as_secs() as i32,
            nanosec: (now.as_micros() % 1_000_000) as u32 * 1_000,
        };
        Self {
            header: Header { stamp, frame_id: "imu_link" },
            orientation: Quaternion {
                x: quat[1] as f64,
                y: quat[2] as f64,
                z: quat[3] as f64,
                w: quat[0] as f64,
            },
            orientation_covariance: [0.0; 9],
            angular_velocity: Vector3 {
                x: gyr_dps[0] as f64 * DEG_TO_RAD,
                y: gyr_dps[1] as f64 * DEG_TO_RAD,
                z: gyr_dps[2] as f64 * DEG_TO_RAD,
            },
            angular_velocity_covariance: [0.0; 9],
            linear_acceleration: Vector3 {
                x: acc_g[0] as f64 * G_TO_MS2,
                y: acc_g[1] as f64 * G_TO_MS2,
                z: acc_g[2] as f64 * G_TO_MS2,
            },
            linear_acceleration_covariance: [0.0; 9],
        }
    }
}

// ── Static publisher ─────────────────────────────────────────────────────────

static IMU_PUB: Publisher<Imu, IMU_CDR_CAP, 4> = Publisher::new(IMU_TOPIC);

/// Publish a fused IMU reading. Awaits if the publish queue is full.
pub async fn publish(quat: [f32; 4], gyr_dps: [f32; 3], acc_g: [f32; 3]) {
    if IMU_PUB.send(&Imu::from_sensors(quat, gyr_dps, acc_g)).await.is_err() {
        warn!("ros2: IMU publish failed");
    }
}

// ── embedded-io-async 0.6 transport adapter ──────────────────────────────────
// `zenoh-ros2-nostd` v1.0 bounds `NodeBuilder::open` on embedded-io-async 0.6,
// but embassy-net 0.9's `TcpSocket` implements the 0.7 traits. This shim
// re-exposes the 0.6 `Read + Write` by delegating to `TcpSocket`'s inherent
// async methods. Delete once the library tracks embedded-io-async 0.7.

use embedded_io_06::{ErrorKind, ErrorType};
use embedded_io_async_06::{Read, Write};

/// Wraps `embassy_net::tcp::Error` so it satisfies embedded-io 0.6's `Error`.
#[derive(Debug)]
struct TcpError(embassy_net::tcp::Error);

impl embedded_io_06::Error for TcpError {
    fn kind(&self) -> ErrorKind {
        ErrorKind::Other
    }
}

struct TcpAdapter<'s, 'a> {
    sock: &'s mut TcpSocket<'a>,
}

impl ErrorType for TcpAdapter<'_, '_> {
    type Error = TcpError;
}

impl Read for TcpAdapter<'_, '_> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.sock.read(buf).await.map_err(TcpError)
    }
}

impl Write for TcpAdapter<'_, '_> {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.sock.write(buf).await.map_err(TcpError)
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        self.sock.flush().await.map_err(TcpError)
    }
}

// ── Session lifecycle ────────────────────────────────────────────────────────

/// Spawn the Zenoh session task. Call once after the network stack is created.
pub fn init(stack: Stack<'static>, spawner: Spawner) {
    spawner.spawn(session_task(stack).unwrap());
}

#[embassy_executor::task]
async fn session_task(stack: Stack<'static>) {
    let (ip, port) = parse_router_addr(env!("CONFIG_ZENOH_ROUTER"));
    let endpoint = IpEndpoint::from((Ipv4Address::from(ip), port));
    let mut reconnect = ReconnectPolicy::default_policy();

    loop {
        while stack.config_v4().is_none() {
            Timer::after_millis(500).await;
        }

        let mut rx = [0u8; 4096];
        let mut tx = [0u8; 4096];
        let mut socket = TcpSocket::new(stack, &mut rx, &mut tx);
        socket.set_timeout(Some(Duration::from_secs(30)));

        info!("ros2: TCP connecting to router...");
        match with_timeout(Duration::from_secs(10), socket.connect(endpoint)).await {
            Ok(Ok(())) => info!("ros2: TCP connected"),
            _ => {
                warn!("ros2: TCP connect failed");
                reconnect.wait_and_advance().await;
                continue;
            }
        }

        let builder = NodeBuilder::new(ZenohId::from_bytes(&DEVICE_ZID))
            .name("rasprover")
            .domain_id(0);
        let mut node = match builder.open(TcpAdapter { sock: &mut socket }).await {
            Ok(n) => n,
            Err(_) => {
                warn!("ros2: handshake failed");
                reconnect.wait_and_advance().await;
                continue;
            }
        };

        node.register_publisher(IMU_PUB.as_drain());
        reconnect.reset();
        info!("ros2: node ready, publishing /imu");

        let mut rx_buf = [0u8; 4096];
        node.spin(&mut rx_buf).await;

        warn!("ros2: session ended, reconnecting");
        reconnect.wait_and_advance().await;
    }
}

// ── Router address parsing (no_std, no alloc) ─────────────────────────────────

/// Parse `"a.b.c.d:port"` into `([u8; 4], u16)`. Panics on malformed input —
/// the value is a compile-time constant, so a startup panic is acceptable.
fn parse_router_addr(addr: &str) -> ([u8; 4], u16) {
    let colon = addr.bytes().rposition(|b| b == b':').expect("CONFIG_ZENOH_ROUTER: missing ':'");
    let port: u16 = addr[colon + 1..].parse().expect("CONFIG_ZENOH_ROUTER: invalid port");

    let mut octets = [0u8; 4];
    let mut count = 0;
    for part in addr[..colon].split('.') {
        assert!(count < 4, "CONFIG_ZENOH_ROUTER: too many IP octets");
        octets[count] = part.parse().expect("CONFIG_ZENOH_ROUTER: invalid IP octet");
        count += 1;
    }
    assert!(count == 4, "CONFIG_ZENOH_ROUTER: IP must have 4 octets");
    (octets, port)
}
