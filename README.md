# ESP WiFi Sniffer (Rust)

A privacy-focused ESP32 WiFi sniffer system that detects and tracks WiFi devices using RSSI-based trilateration. Built with Rust for the ESP32, with a web-based real-time visualization dashboard.

## Features

- **Privacy-First**: MAC addresses are SHA-256 hashed before transmission for GDPR compliance
- **Real-time Tracking**: Multiple ESP32 stations detect WiFi probe requests and publish RSSI data via MQTT
- **Trilateration**: Advanced positioning algorithm using gradient descent optimization to calculate device positions
- **Web Dashboard**: Real-time visualization of detected devices and their positions
- **Low Latency**: Optimized packet processing with configurable rate limiting

## Architecture

```
┌─────────────┐     ┌─────────────┐     ┌─────────────┐
│   ESP32     │     │   ESP32     │     │   ESP32     │
│  Station 1  │     │  Station 2  │     │  Station 3  │
│             │     │             │     │             │
│  Sniffs WiFi│     │  Sniffs WiFi│     │  Sniffs WiFi│
│  Hash MAC   │     │  Hash MAC   │     │  Hash MAC   │
└──────┬──────┘     └──────┬──────┘     └──────┬──────┘
       │                   │                   │
       └───────────────────┴───────────────────┘
                           │
                    ┌──────▼──────┐
                    │    MQTT     │
                    │   Broker    │
                    └──────┬──────┘
                           │
                    ┌──────▼──────┐
                    │  Web GUI    │
                    │             │
                    │ Triangulate │
                    │ Visualize   │
                    └─────────────┘
```

## Privacy & GDPR Compliance

This system is designed with privacy in mind:

- **MAC Address Hashing**: All MAC addresses are hashed using SHA-256 on the ESP32 before transmission
- **No PII Storage**: Only hashed identifiers are stored and transmitted
- **No Raw Packet Logging**: Raw 802.11 frames are never logged or stored
- **Local Processing**: All data stays within your local network

## Requirements

### Hardware
- 3+ ESP32 development boards (ESP32, ESP32-C3, ESP32-S3, etc.)
- USB cables for flashing and power
- WiFi network for MQTT communication

### Software
- Rust (with ESP32 toolchain)
- Cargo
- espflash or cargo-espflash
- Docker (for MQTT broker, optional)

## Installation

### 1. Clone the Repository

```bash
git clone https://github.com/patrickhaahr/esp-sniffer-rs.git
cd esp-sniffer-rs
```

### 2. Install ESP32 Rust Toolchain

Follow the [esp-rs installation guide](https://docs.esp-rs.org/book/installation/index.html):

```bash
cargo install espup
espup install
. $HOME/export-rs.sh  # Or add to your shell profile
```

### 3. Set Up MQTT Broker

Start the included Mosquitto MQTT broker:

```bash
docker-compose up -d
```

Or use your own MQTT broker and update the configuration.

### 4. Configure Environment Variables

Copy the example environment file and configure for each ESP32:

```bash
cp .env.example .env
```

Edit `.env` with your settings:

```bash
WIFI_SSID=your_network_name
WIFI_PASS=your_network_password
MQTT_BROKER=mqtt://192.168.1.100:1883
STATION_ID=station1  # Unique ID for each ESP32
```

### 5. Configure Station Positions

Edit `web/config.toml` to match your physical setup:

```toml
[room]
width = 5.0   # Room width in meters
height = 9.0  # Room height in meters

[[stations]]
id = "station1"          # Must match STATION_ID in .env
x = 0.5                  # X position in meters
y = 0.5                  # Y position in meters
label = "Station 1"
rssi_at_1m = -45.0       # Calibration: RSSI at 1 meter
path_loss_exponent = 3.0 # Indoor path loss (2.0-4.0)
```

## Usage

### Flash ESP32 Stations

For each ESP32, update the `STATION_ID` in `.env` and flash:

```bash
cargo fr
```

This command will:
1. Build the firmware in release mode
2. Flash to ESP32
3. Open serial monitor

**Note**: `cargo fr` is a custom alias defined in `.cargo/config.toml` that expands to `cargo run --release --bin esp-sniffer-rs`.


### View Real-time Data

1. Open browser to `http://localhost:3000`
2. You'll see a 2D visualization of the room with:
   - Station positions (fixed markers)
   - Detected devices (moving circles)
   - Device trails showing movement history
   - RSSI values and signal strength indicators

## CLI Commands

| Command | Description |
|---------|-------------|
| `cargo fr` | Flash firmware to ESP32 with serial monitor |
| `cargo br` | Build ESP32 firmware only (release mode) |
| `cargo web` | Run web GUI (cross-platform) |
| `cargo web-l` | Run web GUI (Linux-optimized) |
| `cargo web-m` | Run web GUI (macOS Apple Silicon) |
| `cargo web-w` | Run web GUI (Windows) |

## How It Works

### ESP32 Sniffer

1. **WiFi Promiscuous Mode**: ESP32 enters monitor mode to capture 802.11 management frames
2. **MAC Extraction**: Source MAC addresses are extracted from probe requests and data frames
3. **Privacy Hashing**: MAC addresses are hashed with SHA-256 immediately
4. **RSSI Measurement**: Signal strength (RSSI) is recorded for each frame
5. **MQTT Publishing**: Hashed MAC + RSSI + timestamp sent to MQTT broker

### Trilateration Algorithm

The web dashboard uses advanced positioning:

1. **RSSI to Distance**: Converts signal strength to estimated distance using log-distance path loss model:
   ```
   distance = 10^((rssi_at_1m - rssi) / (10 * path_loss_exponent))
   ```

2. **Gradient Descent**: Minimizes position error using weighted non-linear least squares

3. **Position Smoothing**: Exponential moving average reduces jitter in real-time tracking

4. **Fallback**: Uses weighted centroid when fewer than 3 stations detect a device

## Configuration

### ESP32 Sniffer Configuration

Located in `src/sniffer.rs`:

```rust
const SEND_RATE: u32 = 10;  // Send 1 in every 10 packets to MQTT
const CHANNEL_CAPACITY: usize = 32;  // Event queue size
```

### Triangulation Configuration

Located in `web/config.toml` or programmatically:

```toml
[triangulation]
smoothing_factor = 0.4           # 0.0 = no smoothing, 1.0 = no update
max_iterations = 50              # Gradient descent iterations
convergence_threshold = 0.01     # Stop when position change < 0.01m
learning_rate = 0.5              # Gradient descent step size
min_stations = 3                 # Minimum stations for trilateration
max_reading_age_secs = 10        # Ignore readings older than 10s
min_rssi = -90                   # Ignore weak signals
max_distance = 50.0              # Ignore unrealistic distance estimates
```

## Troubleshooting

### ESP32 Connection Issues

```bash
# Check serial port
ls /dev/ttyUSB* /dev/ttyACM*

# Flash with verbose output
cargo fr

# Or manually with espflash
espflash flash --release --monitor target/xtensa-esp32-espidf/release/esp-sniffer-rs
```

### MQTT Connection Issues

```bash
# Test MQTT broker
mosquitto_sub -h <broker-ip> -t "sniffer/#" -v

# Check if ESP32 is publishing
mosquitto_sub -h <broker-ip> -t "sniffer/+/device" -v
```

### Poor Positioning Accuracy

1. **Calibrate RSSI**: Measure actual RSSI at 1 meter and update `rssi_at_1m`
2. **Adjust Path Loss**: Increase `path_loss_exponent` for environments with more obstacles
3. **Station Placement**: Position stations in a triangle/polygon for best results
4. **Reduce Smoothing**: Lower `smoothing_factor` for faster position updates

