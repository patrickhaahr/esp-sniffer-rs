use anyhow::Result;
use esp_idf_svc::mqtt::client::{EspMqttClient, EventPayload, MqttClientConfiguration, QoS};
use log::{error, info, warn};
use std::sync::mpsc::{self, Receiver, SyncSender, TrySendError};
use std::time::Duration;

/// MQTT broker configuration
const MQTT_BROKER: &str = env!("MQTT_BROKER");
const MQTT_TOPIC_PREFIX: &str = "sniffer";

/// Bounded channel capacity - prevents memory exhaustion
const CHANNEL_CAPACITY: usize = 32;

/// Device detection event to publish (fixed size, no heap allocation)
/// MAC address is stored as a SHA-256 hash for privacy
#[derive(Debug, Clone, Copy)]
pub struct DeviceEvent {
    pub mac_hash: [u8; 32],
    pub rssi: i8,
    pub channel: u8,
    pub timestamp: u64,
}

/// MQTT publisher that receives events from a channel and publishes them
pub struct MqttPublisher {
    client: EspMqttClient<'static>,
    rx: Receiver<DeviceEvent>,
    station_id: String,
}

impl MqttPublisher {
    /// Create new MQTT publisher
    pub fn new(station_id: &str, rx: Receiver<DeviceEvent>) -> Result<Self> {
        info!("Connecting to MQTT broker: {}", MQTT_BROKER);

        let mqtt_config = MqttClientConfiguration {
            client_id: Some(station_id),
            ..Default::default()
        };

        let client = EspMqttClient::new_cb(
            MQTT_BROKER,
            &mqtt_config,
            move |event| {
                match event.payload() {
                    EventPayload::Connected(_) => {
                        info!("MQTT connected");
                    }
                    EventPayload::Disconnected => {
                        info!("MQTT disconnected");
                    }
                    EventPayload::Error(e) => {
                        error!("MQTT error: {:?}", e);
                    }
                    _ => {}
                }
            },
        )?;

        info!("MQTT client created for station: {}", station_id);

        Ok(Self {
            client,
            rx,
            station_id: station_id.to_string(),
        })
    }

    /// Run the publisher loop - receives events and publishes to MQTT
    pub fn run(&mut self) -> Result<()> {
        info!("MQTT publisher running...");

        loop {
            // Block waiting for events with timeout
            match self.rx.recv_timeout(Duration::from_secs(5)) {
                Ok(event) => {
                    self.publish_event(&event)?;
                }
                Err(mpsc::RecvTimeoutError::Timeout) => {
                    // No events, just continue
                }
                Err(mpsc::RecvTimeoutError::Disconnected) => {
                    error!("Event channel disconnected");
                    break;
                }
            }
        }

        Ok(())
    }

    /// Publish a device event to MQTT
    fn publish_event(&mut self, event: &DeviceEvent) -> Result<()> {
        // Format hashed MAC address as hex string (64 chars for 32 bytes)
        let mut mac_hex = String::with_capacity(64);
        for byte in &event.mac_hash {
            mac_hex.push_str(&format!("{:02x}", byte));
        }

        // Use a fixed-size buffer to avoid heap allocation
        let mut payload = [0u8; 200];  // Increased size for longer hash
        let payload_str = format!(
            r#"{{"mac_hash":"{}","rssi":{},"channel":{},"timestamp":{},"station":"{}"}}"#,
            mac_hex, event.rssi, event.channel, event.timestamp, self.station_id
        );

        let len = payload_str.len().min(payload.len());
        payload[..len].copy_from_slice(&payload_str.as_bytes()[..len]);

        // Use static topic to avoid allocation
        let topic = format!("{}/{}/device", MQTT_TOPIC_PREFIX, self.station_id);

        // Try to enqueue, ignore errors (MQTT outbox full)
        if let Err(e) = self.client.enqueue(
            &topic,
            QoS::AtMostOnce,
            false,
            &payload[..len],
        ) {
            // Log occasionally, don't spam
            static SKIP_COUNT: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);
            let skipped = SKIP_COUNT.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
            if skipped % 100 == 0 {
                error!("MQTT enqueue failed ({}): {:?}", skipped, e);
            }
        }

        Ok(())
    }

    /// Publish station heartbeat
    pub fn publish_heartbeat(&mut self, packet_count: u32) -> Result<()> {
        let topic = format!("{}/{}/heartbeat", MQTT_TOPIC_PREFIX, self.station_id);

        let payload = format!(
            r#"{{"station":"{}","packets":{}}}"#,
            self.station_id, packet_count
        );

        self.client.enqueue(
            &topic,
            QoS::AtMostOnce,
            false,
            payload.as_bytes(),
        )?;

        Ok(())
    }
}

/// Create bounded event channel for passing device detections
/// Returns a SyncSender that will drop events when channel is full
pub fn create_event_channel() -> (SyncSender<DeviceEvent>, Receiver<DeviceEvent>) {
    mpsc::sync_channel(CHANNEL_CAPACITY)
}
