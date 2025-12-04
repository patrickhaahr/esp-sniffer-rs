# ESP32 WiFi Sniffer

A privacy-focused ESP32 WiFi sniffer system that detects and tracks WiFi devices using RSSI-based trilateration. Built with Rust for the ESP32, with a web-based real-time visualization dashboard.

## Web Dashboard Examples

![Single Station Dashboard](docs/station1_only_gui.png)
*Single ESP32 station connected to the system*

![Full System Dashboard](docs/generated_image_gui.jpg)
*AI-generated example showing all three ESP32 stations connected (screenshot unavailable)*

## Features

- **Privacy-First**: MAC addresses are SHA-256 hashed before transmission for GDPR compliance
- **Secure by Default**: All communications encrypted with TLS 1.3 (HTTPS, WSS, MQTTS)
- **Real-time Tracking**: Multiple ESP32 stations detect WiFi probe requests and publish RSSI data via MQTT
- **Trilateration**: Advanced positioning algorithm using gradient descent optimization to calculate device positions
- **Web Dashboard**: Real-time visualization of detected devices and their positions
- **Low Latency**: Optimized packet processing with configurable rate limiting

## Architecture

```
┌──────────────┐     ┌──────────────┐     ┌──────────────┐
│    ESP32     │     │    ESP32     │     │    ESP32     │
│   Station 1  │     │   Station 2  │     │   Station 3  │
│              │     │              │     │              │
│  Sniffs WiFi │     │  Sniffs WiFi │     │  Sniffs WiFi │
│   Hash MAC   │     │   Hash MAC   │     │   Hash MAC   │
└──────┬───────┘     └──────┬───────┘     └──────┬───────┘
       │                    │                    │
       └────────────────────┴────────────────────┘
                            │
                     ┌──────▼───────┐
                     │     MQTT     │
                     │    Broker    │
                     └──────┬───────┘
                            │
                     ┌──────▼───────┐
                     │    Web GUI   │
                     │    (HTTPS)   │
                     └──────────────┘
```

## Quick Start Summary

```bash
# 1. Configure environment
cp .env.example .env
# Edit .env with your SERVER_IP, WIFI_SSID, WIFI_PASS, MQTT_BROKER=mqtts://...

# 2. Generate TLS certificates
./genssl.sh

# 3. Start MQTT broker
docker-compose up -d

# 4. Flash ESP32 (repeat for each station)
# Edit STATION_ID in .env, then:
cargo fr

# 5. Start web GUI
cargo web-l 

# 6. Open browser to https://localhost:3000
# 7. Monitor MQTT: mosquitto_sub -h $SERVER_IP -p 8883 --cafile ./certs/ca.crt -t '#' -u elev1 -P password -v
```

## Security

All communications are encrypted with TLS 1.3:

- **ESP32 → MQTT**: MQTTS on port 8883 with CA certificate verification
- **Web GUI → MQTT**: MQTTS on port 8883 with CA certificate verification  
- **Browser → Web GUI**: HTTPS on port 3000 with self-signed certificate
- **WebSocket**: WSS automatically over HTTPS

## Privacy & GDPR Compliance

This system is designed with privacy in mind:

- **MAC Address Hashing**: All MAC addresses are hashed using SHA-256 on the ESP32 before transmission
- **No PII Storage**: Only hashed identifiers are stored and transmitted
- **No Raw Packet Logging**: Raw 802.11 frames are never logged or stored
- **Local Processing**: All data stays within your local network

## Installation

### 1. Install ESP32 Rust Toolchain

Follow the [esp-rs installation guide](https://docs.espressif.com/projects/rust/book/getting-started/toolchain.html)
Follow the [esp-idf-template installation guide](https://github.com/esp-rs/esp-idf-template?tab=readme-ov-file#prerequisites) if needed


### 2. Clone the Repository
    
```bash
git clone https://github.com/patrickhaahr/esp32-wifi-sniffer.git
cd esp32-wifi-sniffer
```

### 3. Generate TLS Certificates

**Important**: Set your server IP in `.env` first, then generate certificates:

```bash
# Copy and configure environment file
cp .env.example .env

# Edit .env with your server IP (the machine running MQTT broker and web GUI)
SERVER_IP=192.168.1.100 
WIFI_SSID=your_network_name
WIFI_PASS=your_network_password
MQTT_BROKER=mqtts://192.168.1.100:8883  
MQTT_USERNAME=elev1 
MQTT_PASSWORD=password 
STATION_ID=station1

# Generate TLS certificates and MQTT password file
./genssl.sh
```

The script creates:
- `certs/ca.crt` - Root CA certificate (for ESP32 clients)
- `certs/server.crt` - Server certificate (shared by MQTT broker and web GUI)
- `certs/server.key` - Server private key
- `mosquitto/config/passwd` - MQTT password file with user credentials

### 4. Start MQTT Broker

Start the included Mosquitto MQTT broker with TLS and authentication:

```bash
docker-compose up -d
```

The broker will listen on port 8883 with TLS encryption and username/password authentication.

### 5. Verify MQTT TLS Connection with Authentication

Test the MQTT broker with TLS and authentication:

```bash
# View all MQTT messages with TLS and authentication
mosquitto_sub -h 192.168.1.100 -p 8883 --cafile ./certs/ca.crt -u elev1 -P password -t '#' -v
```

Replace `192.168.1.100` with your `SERVER_IP`. The credentials (`elev1`/`password`) are configured in your `.env` file.

### 5. Configure Station Positions

Edit `web/config.toml` to match your physical setup:

```toml
[room]
width = 5.0              # Room width in meters
height = 9.0             # Room height in meters   

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

**Note**: `cargo fr` is a custom alias defined in `.cargo/config.toml` that expands to `cargo run --release --bin esp32-wifi-sniffer`.


### View Real-time Data

```bash
cargo web-*  # replace "*" with: l = Linux, m = MacOS, w = Windows
```

1. Open browser to `https://localhost:3000` (accept self-signed certificate warning)
2. You'll see a 2D visualization of the room with:
   - Station positions (fixed markers)
   - Detected devices (moving circles)
   - Device trails showing movement history
   - RSSI values and signal strength indicators
   - Real-time triangulation positioning

## CLI Commands

| Command     | Description                                 |
|-------------|---------------------------------------------|
| `cargo fr`    | Flash firmware to ESP32 with serial monitor |
| `cargo br`    | Build ESP32 firmware only (release mode)    |
| `cargo web-l` | Run web GUI (Linux)                         |
| `cargo web-m` | Run web GUI (MacOS)                         |
| `cargo web-w` | Run web GUI (Windows)                       |

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
