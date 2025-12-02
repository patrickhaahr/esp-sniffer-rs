use esp_idf_svc::sys::{
    esp_wifi_set_promiscuous,
    esp_wifi_set_promiscuous_rx_cb,
    esp_wifi_set_promiscuous_filter,
    esp_wifi_set_channel,
    esp_timer_get_time,
    wifi_promiscuous_pkt_t,
    wifi_promiscuous_pkt_type_t,
    wifi_promiscuous_filter_t,
    wifi_second_chan_t_WIFI_SECOND_CHAN_NONE,
    WIFI_PROMIS_FILTER_MASK_MGMT,
    WIFI_PROMIS_FILTER_MASK_DATA,
    ESP_OK,
};
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::Mutex;
use std::sync::mpsc::SyncSender;
use sha2::{Sha256, Digest};
use crate::mqtt::DeviceEvent;

/// Packet counter for statistics
static PACKET_COUNT: AtomicU32 = AtomicU32::new(0);
static DROPPED_COUNT: AtomicU32 = AtomicU32::new(0);
static SENT_COUNT: AtomicU32 = AtomicU32::new(0);

/// Rate limit: only send 1 event per N packets to avoid overwhelming MQTT
/// Lower value = faster updates (more MQTT messages)
const SEND_RATE: u32 = 10;

/// Global event sender for the callback
static EVENT_SENDER: Mutex<Option<SyncSender<DeviceEvent>>> = Mutex::new(None);

/// Set the event sender for publishing device detections
pub fn set_event_sender(sender: SyncSender<DeviceEvent>) {
    if let Ok(mut guard) = EVENT_SENDER.lock() {
        *guard = Some(sender);
    }
}

/// IEEE 802.11 MAC Header (simplified)
/// Offsets: addr1 @ 4, addr2 @ 10, addr3 @ 16
#[repr(C, packed)]
pub struct Ieee80211MacHeader {
    pub frame_control: u16,
    pub duration: u16,
    pub addr1: [u8; 6],  // Receiver/Destination Address
    pub addr2: [u8; 6],  // Transmitter/Source Address
    pub addr3: [u8; 6],  // BSSID or other
    pub seq_ctrl: u16,
}

/// MAC address wrapper for display
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct MacAddress(pub [u8; 6]);

impl MacAddress {
    pub fn from_bytes(bytes: &[u8; 6]) -> Self {
        MacAddress(*bytes)
    }

    /// Check if this is a broadcast address (FF:FF:FF:FF:FF:FF)
    pub fn is_broadcast(&self) -> bool {
        self.0 == [0xFF; 6]
    }

    /// Check if this is a multicast address (first byte has LSB set)
    pub fn is_multicast(&self) -> bool {
        self.0[0] & 0x01 != 0
    }

    /// Hash the MAC address using SHA-256 and return as hex string
    pub fn hash(&self) -> [u8; 32] {
        let mut hasher = Sha256::new();
        hasher.update(&self.0);
        let result = hasher.finalize();
        let mut hash = [0u8; 32];
        hash.copy_from_slice(&result);
        hash
    }
}

impl core::fmt::Display for MacAddress {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(
            f,
            "{:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}",
            self.0[0], self.0[1], self.0[2], self.0[3], self.0[4], self.0[5]
        )
    }
}

/// Sniffed packet information
#[derive(Debug, Clone)]
pub struct SniffedPacket {
    pub source_mac: MacAddress,
    pub dest_mac: MacAddress,
    pub bssid: MacAddress,
    pub rssi: i8,
    pub channel: u8,
    pub packet_type: u32,
    pub length: u32,
}

/// Promiscuous mode RX callback
/// WARNING: Called directly in WiFi driver task - keep it minimal!
unsafe extern "C" fn promiscuous_rx_callback(
    buf: *mut ::core::ffi::c_void,
    pkt_type: wifi_promiscuous_pkt_type_t,
) {
    if buf.is_null() {
        return;
    }

    // Cast to packet structure
    let pkt = buf as *const wifi_promiscuous_pkt_t;
    let rx_ctrl = &(*pkt).rx_ctrl;

    // Extract RSSI (signal strength in dBm)
    let rssi = rx_ctrl.rssi() as i8;

    // Get payload length
    let sig_len = rx_ctrl.sig_len();

    // Get channel
    let channel = rx_ctrl.channel() as u8;

    // Skip if payload too small for MAC header (minimum 24 bytes)
    if sig_len < 24 {
        return;
    }

    // Get pointer to payload (IEEE 802.11 frame)
    let payload_ptr = (*pkt).payload.as_ptr();

    // Parse MAC header
    let mac_header = payload_ptr as *const Ieee80211MacHeader;
    let source_mac = MacAddress((*mac_header).addr2);

    // Skip broadcast/multicast for device tracking
    if source_mac.is_broadcast() || source_mac.is_multicast() {
        return;
    }

    // Increment packet counter
    let count = PACKET_COUNT.fetch_add(1, Ordering::SeqCst);

    // Get timestamp in microseconds
    let timestamp = esp_timer_get_time() as u64;

    // Rate limit: only send 1 in every SEND_RATE packets
    if count % SEND_RATE == 0 {
        // Send event to MQTT publisher (non-blocking, drops if full)
        if let Ok(guard) = EVENT_SENDER.try_lock() {
            if let Some(sender) = guard.as_ref() {
                // Hash the MAC address for privacy
                let mac_hash = source_mac.hash();
                
                let event = DeviceEvent {
                    mac_hash,
                    rssi,
                    channel,
                    timestamp,
                };
                // Use try_send to avoid blocking - drop event if channel full
                if sender.try_send(event).is_ok() {
                    SENT_COUNT.fetch_add(1, Ordering::Relaxed);
                } else {
                    DROPPED_COUNT.fetch_add(1, Ordering::Relaxed);
                }
            }
        }
    }

    // Log every 100th packet to avoid flooding
    if count % 100 == 0 {
        log::info!(
            "[{}] Type={}, RSSI={}dBm, Ch={}, Src={}",
            count, pkt_type, rssi, channel, source_mac
        );
    }
}

/// Initialize WiFi promiscuous mode sniffer
/// Note: When connected to WiFi, sniffs on the AP's channel (cannot change)
pub fn start_sniffer() -> anyhow::Result<()> {
    log::info!("Starting promiscuous mode sniffer");

    unsafe {
        // Don't set channel - use whatever channel the AP is on
        // esp_wifi_set_channel fails when connected to an AP

        // Configure promiscuous filter (capture management and data frames)
        let filter = wifi_promiscuous_filter_t {
            filter_mask: WIFI_PROMIS_FILTER_MASK_MGMT | WIFI_PROMIS_FILTER_MASK_DATA,
        };
        let ret = esp_wifi_set_promiscuous_filter(&filter);
        if ret != ESP_OK {
            anyhow::bail!("Failed to set promiscuous filter: {}", ret);
        }
        log::info!("Promiscuous filter configured");

        // Register the callback
        let ret = esp_wifi_set_promiscuous_rx_cb(Some(promiscuous_rx_callback));
        if ret != ESP_OK {
            anyhow::bail!("Failed to set promiscuous callback: {}", ret);
        }
        log::info!("Promiscuous callback registered");

        // Enable promiscuous mode
        let ret = esp_wifi_set_promiscuous(true);
        if ret != ESP_OK {
            anyhow::bail!("Failed to enable promiscuous mode: {}", ret);
        }

        log::info!("Promiscuous mode enabled");
    }

    Ok(())
}

/// Stop the sniffer
pub fn stop_sniffer() -> anyhow::Result<()> {
    unsafe {
        let ret = esp_wifi_set_promiscuous(false);
        if ret != ESP_OK {
            anyhow::bail!("Failed to disable promiscuous mode: {}", ret);
        }

        let ret = esp_wifi_set_promiscuous_rx_cb(None);
        if ret != ESP_OK {
            anyhow::bail!("Failed to clear promiscuous callback: {}", ret);
        }
    }

    log::info!("Promiscuous mode disabled");
    Ok(())
}

/// Get current packet count
pub fn get_packet_count() -> u32 {
    PACKET_COUNT.load(Ordering::SeqCst)
}

/// Get dropped event count (channel was full)
pub fn get_dropped_count() -> u32 {
    DROPPED_COUNT.load(Ordering::Relaxed)
}

/// Get sent event count
pub fn get_sent_count() -> u32 {
    SENT_COUNT.load(Ordering::Relaxed)
}

/// Reset packet counter
pub fn reset_packet_count() {
    PACKET_COUNT.store(0, Ordering::SeqCst);
}
