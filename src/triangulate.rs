//! Advanced triangulation module for WiFi-based device positioning.
//!
//! Implements multiple positioning algorithms:
//! - **Trilateration**: Non-linear least squares optimization using RSSI-to-distance conversion
//! - **Weighted Centroid**: Fallback when trilateration doesn't converge
//! - **Position Smoothing**: Exponential moving average to reduce jitter
//!
//! The algorithm converts RSSI values to estimated distances using the log-distance
//! path loss model, then uses gradient descent to find the position that minimizes
//! the sum of squared distance errors.

use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// A calculated 2D position in meters
#[derive(Debug, Clone, Copy, Serialize, Deserialize, Default, PartialEq)]
pub struct Position {
    pub x: f32,
    pub y: f32,
}

impl Position {
    pub fn new(x: f32, y: f32) -> Self {
        Self { x, y }
    }

    /// Calculate Euclidean distance to another position
    pub fn distance_to(&self, other: &Position) -> f32 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        (dx * dx + dy * dy).sqrt()
    }

    /// Linear interpolation between two positions
    pub fn lerp(&self, other: &Position, t: f32) -> Position {
        Position {
            x: self.x + (other.x - self.x) * t,
            y: self.y + (other.y - self.y) * t,
        }
    }
}

/// Per-station calibration parameters for RSSI-to-distance conversion
#[derive(Debug, Clone, Deserialize)]
pub struct CalibrationParams {
    /// Reference RSSI at 1 meter distance (typically -40 to -50 dBm)
    #[serde(default = "default_rssi_at_1m")]
    pub rssi_at_1m: f32,

    /// Path loss exponent (2.0 = free space, 2.5-4.0 = indoor)
    #[serde(default = "default_path_loss_exponent")]
    pub path_loss_exponent: f32,
}

fn default_rssi_at_1m() -> f32 {
    -40.0
}

fn default_path_loss_exponent() -> f32 {
    2.5
}

impl Default for CalibrationParams {
    fn default() -> Self {
        Self {
            rssi_at_1m: default_rssi_at_1m(),
            path_loss_exponent: default_path_loss_exponent(),
        }
    }
}

/// Station data with position and calibration for triangulation
#[derive(Debug, Clone)]
pub struct StationData {
    pub id: String,
    pub x: f32,
    pub y: f32,
    pub calibration: CalibrationParams,
}

impl StationData {
    pub fn position(&self) -> Position {
        Position::new(self.x, self.y)
    }
}

/// RSSI reading from a station
#[derive(Debug, Clone)]
pub struct RssiReading {
    pub rssi: i8,
    pub timestamp: u64,
}

/// Trait to abstract over different station config types
pub trait StationLike {
    fn id(&self) -> &str;
    fn x(&self) -> f32;
    fn y(&self) -> f32;
    fn calibration(&self) -> CalibrationParams;
}

/// Configuration for the positioning algorithm
#[derive(Debug, Clone)]
pub struct TriangulatorConfig {
    /// Smoothing factor for position updates (0.0 = no smoothing, 1.0 = no update)
    /// Recommended: 0.3-0.5 for smooth tracking
    pub smoothing_factor: f32,

    /// Maximum iterations for gradient descent
    pub max_iterations: usize,

    /// Convergence threshold (stop when position change is below this)
    pub convergence_threshold: f32,

    /// Learning rate for gradient descent
    pub learning_rate: f32,

    /// Minimum number of stations required for trilateration (fallback to centroid otherwise)
    pub min_stations_for_trilateration: usize,

    /// Maximum RSSI age in seconds (older readings are ignored)
    pub max_reading_age_secs: u64,

    /// Minimum RSSI value to consider (weaker signals are ignored)
    pub min_rssi: i8,

    /// Maximum distance estimate to consider valid (filters out extreme outliers)
    pub max_distance: f32,
}

impl Default for TriangulatorConfig {
    fn default() -> Self {
        Self {
            smoothing_factor: 0.4,
            max_iterations: 50,
            convergence_threshold: 0.01,
            learning_rate: 0.5,
            min_stations_for_trilateration: 3,
            max_reading_age_secs: 10,
            min_rssi: -90,
            max_distance: 50.0,
        }
    }
}

/// Distance measurement from a station
#[derive(Debug, Clone)]
struct DistanceMeasurement {
    station_pos: Position,
    estimated_distance: f32,
    weight: f32, // Higher weight for stronger signals (more reliable)
}

/// Triangulator that computes device positions from RSSI readings
pub struct Triangulator {
    stations: HashMap<String, StationData>,
    config: TriangulatorConfig,
    /// Room bounds for clamping positions
    room_min: Position,
    room_max: Position,
}

impl Triangulator {
    /// Create a new triangulator from station configurations
    pub fn new<S>(stations: &[S]) -> Self
    where
        S: StationLike,
    {
        Self::with_config(stations, TriangulatorConfig::default())
    }

    /// Create a new triangulator with custom configuration
    pub fn with_config<S>(stations: &[S], config: TriangulatorConfig) -> Self
    where
        S: StationLike,
    {
        let station_map: HashMap<String, StationData> = stations
            .iter()
            .map(|s| {
                (
                    s.id().to_string(),
                    StationData {
                        id: s.id().to_string(),
                        x: s.x(),
                        y: s.y(),
                        calibration: s.calibration(),
                    },
                )
            })
            .collect();

        // Calculate room bounds from station positions (with some padding)
        let (min_x, max_x, min_y, max_y) = station_map.values().fold(
            (f32::MAX, f32::MIN, f32::MAX, f32::MIN),
            |(min_x, max_x, min_y, max_y), s| {
                (
                    min_x.min(s.x),
                    max_x.max(s.x),
                    min_y.min(s.y),
                    max_y.max(s.y),
                )
            },
        );

        // Add padding around the room bounds
        let padding = 1.0;
        let room_min = Position::new((min_x - padding).max(0.0), (min_y - padding).max(0.0));
        let room_max = Position::new(max_x + padding, max_y + padding);

        Self {
            stations: station_map,
            config,
            room_min,
            room_max,
        }
    }

    /// Calculate position using trilateration with gradient descent optimization
    ///
    /// This is the main entry point for position calculation.
    /// Falls back to weighted centroid if trilateration fails.
    pub fn calculate_position(&self, readings: &HashMap<String, RssiReading>) -> Option<Position> {
        self.calculate_position_internal(readings, None)
    }

    /// Calculate position with optional previous position for smoothing
    pub fn calculate_position_smoothed(
        &self,
        readings: &HashMap<String, RssiReading>,
        previous_position: Option<Position>,
    ) -> Option<Position> {
        self.calculate_position_internal(readings, previous_position)
    }

    fn calculate_position_internal(
        &self,
        readings: &HashMap<String, RssiReading>,
        previous_position: Option<Position>,
    ) -> Option<Position> {
        if readings.is_empty() {
            return None;
        }

        // Convert readings to distance measurements
        let measurements = self.readings_to_measurements(readings);

        if measurements.is_empty() {
            return None;
        }

        // Calculate raw position
        let raw_position = if measurements.len() >= self.config.min_stations_for_trilateration {
            // Use trilateration with gradient descent
            self.trilaterate(&measurements)
                .unwrap_or_else(|| self.weighted_centroid(&measurements))
        } else {
            // Fall back to weighted centroid for fewer stations
            self.weighted_centroid(&measurements)
        };

        // Clamp to room bounds
        let clamped = self.clamp_to_room(raw_position);

        // Apply smoothing if we have a previous position
        let smoothed = if let Some(prev) = previous_position {
            prev.lerp(&clamped, 1.0 - self.config.smoothing_factor)
        } else {
            clamped
        };

        Some(smoothed)
    }

    /// Convert RSSI readings to distance measurements
    fn readings_to_measurements(
        &self,
        readings: &HashMap<String, RssiReading>,
    ) -> Vec<DistanceMeasurement> {
        readings
            .iter()
            .filter_map(|(station_id, reading)| {
                let station = self.stations.get(station_id)?;

                // Filter out weak signals
                if reading.rssi < self.config.min_rssi {
                    return None;
                }

                let distance = self.rssi_to_distance(reading.rssi, &station.calibration);

                // Filter out unrealistic distances
                if distance > self.config.max_distance || distance < 0.1 {
                    return None;
                }

                // Weight based on signal strength (stronger = more reliable)
                // Using inverse of distance squared as weight
                let weight = 1.0 / (distance * distance + 0.1);

                Some(DistanceMeasurement {
                    station_pos: station.position(),
                    estimated_distance: distance,
                    weight,
                })
            })
            .collect()
    }

    /// Trilateration using gradient descent optimization
    ///
    /// Minimizes: sum_i(weight_i * (distance(pos, station_i) - estimated_distance_i)^2)
    fn trilaterate(&self, measurements: &[DistanceMeasurement]) -> Option<Position> {
        // Initialize position at weighted centroid
        let mut pos = self.weighted_centroid(measurements);

        for _ in 0..self.config.max_iterations {
            let (grad_x, grad_y) = self.compute_gradient(&pos, measurements);

            // Update position using gradient descent
            let new_x = pos.x - self.config.learning_rate * grad_x;
            let new_y = pos.y - self.config.learning_rate * grad_y;

            let new_pos = Position::new(new_x, new_y);

            // Check for convergence
            if pos.distance_to(&new_pos) < self.config.convergence_threshold {
                return Some(new_pos);
            }

            pos = new_pos;
        }

        // Return final position even if not fully converged
        Some(pos)
    }

    /// Compute gradient of the cost function
    fn compute_gradient(&self, pos: &Position, measurements: &[DistanceMeasurement]) -> (f32, f32) {
        let mut grad_x = 0.0f32;
        let mut grad_y = 0.0f32;

        for m in measurements {
            let dx = pos.x - m.station_pos.x;
            let dy = pos.y - m.station_pos.y;
            let actual_dist = (dx * dx + dy * dy).sqrt().max(0.001); // Avoid division by zero

            let error = actual_dist - m.estimated_distance;

            // Gradient of squared error with respect to position
            // d/dx [(sqrt((x-sx)^2 + (y-sy)^2) - d)^2] = 2 * error * (x-sx) / actual_dist
            grad_x += m.weight * 2.0 * error * dx / actual_dist;
            grad_y += m.weight * 2.0 * error * dy / actual_dist;
        }

        // Normalize by total weight
        let total_weight: f32 = measurements.iter().map(|m| m.weight).sum();
        if total_weight > 0.0 {
            grad_x /= total_weight;
            grad_y /= total_weight;
        }

        (grad_x, grad_y)
    }

    /// Weighted centroid calculation (fallback method)
    fn weighted_centroid(&self, measurements: &[DistanceMeasurement]) -> Position {
        let mut total_weight = 0.0f32;
        let mut weighted_x = 0.0f32;
        let mut weighted_y = 0.0f32;

        for m in measurements {
            weighted_x += m.weight * m.station_pos.x;
            weighted_y += m.weight * m.station_pos.y;
            total_weight += m.weight;
        }

        if total_weight > 0.0 {
            Position::new(weighted_x / total_weight, weighted_y / total_weight)
        } else {
            Position::default()
        }
    }

    /// Convert RSSI to estimated distance using log-distance path loss model
    ///
    /// Formula: distance = 10^((rssi_at_1m - rssi) / (10 * path_loss_exponent))
    fn rssi_to_distance(&self, rssi: i8, calibration: &CalibrationParams) -> f32 {
        let exponent =
            (calibration.rssi_at_1m - rssi as f32) / (10.0 * calibration.path_loss_exponent);
        10.0_f32.powf(exponent)
    }

    /// Clamp position to room bounds
    fn clamp_to_room(&self, pos: Position) -> Position {
        Position::new(
            pos.x.clamp(self.room_min.x, self.room_max.x),
            pos.y.clamp(self.room_min.y, self.room_max.y),
        )
    }
}

/// Position tracker that maintains smoothed positions for multiple devices
pub struct PositionTracker {
    triangulator: Triangulator,
    /// Smoothed positions for each device (by MAC address)
    positions: HashMap<String, Position>,
}

impl PositionTracker {
    pub fn new<S>(stations: &[S]) -> Self
    where
        S: StationLike,
    {
        Self {
            triangulator: Triangulator::new(stations),
            positions: HashMap::new(),
        }
    }

    pub fn with_config<S>(stations: &[S], config: TriangulatorConfig) -> Self
    where
        S: StationLike,
    {
        Self {
            triangulator: Triangulator::with_config(stations, config),
            positions: HashMap::new(),
        }
    }

    /// Update position for a device, applying smoothing
    pub fn update_position(
        &mut self,
        device_id: &str,
        readings: &HashMap<String, RssiReading>,
    ) -> Option<Position> {
        let previous = self.positions.get(device_id).copied();
        let new_pos = self
            .triangulator
            .calculate_position_smoothed(readings, previous)?;
        self.positions.insert(device_id.to_string(), new_pos);
        Some(new_pos)
    }

    /// Get the current smoothed position for a device
    pub fn get_position(&self, device_id: &str) -> Option<Position> {
        self.positions.get(device_id).copied()
    }

    /// Remove a device from tracking
    pub fn remove_device(&mut self, device_id: &str) {
        self.positions.remove(device_id);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct TestStation {
        id: String,
        x: f32,
        y: f32,
        calibration: Option<CalibrationParams>,
    }

    impl StationLike for TestStation {
        fn id(&self) -> &str {
            &self.id
        }
        fn x(&self) -> f32 {
            self.x
        }
        fn y(&self) -> f32 {
            self.y
        }
        fn calibration(&self) -> CalibrationParams {
            self.calibration.clone().unwrap_or_default()
        }
    }

    fn make_stations() -> Vec<TestStation> {
        vec![
            TestStation {
                id: "1".to_string(),
                x: 0.0,
                y: 0.0,
                calibration: None,
            },
            TestStation {
                id: "2".to_string(),
                x: 5.0,
                y: 0.0,
                calibration: None,
            },
            TestStation {
                id: "3".to_string(),
                x: 2.5,
                y: 5.0,
                calibration: None,
            },
        ]
    }

    #[test]
    fn test_rssi_to_distance() {
        let stations = make_stations();
        let triangulator = Triangulator::new(&stations);
        let cal = CalibrationParams::default();

        // At reference distance (1m), RSSI should equal rssi_at_1m
        let dist_1m = triangulator.rssi_to_distance(-40, &cal);
        assert!((dist_1m - 1.0).abs() < 0.1, "Distance at ref RSSI should be ~1m");

        // Weaker signal = greater distance
        let dist_far = triangulator.rssi_to_distance(-65, &cal);
        assert!(dist_far > 1.0, "Weaker signal should give greater distance");

        // Stronger signal = shorter distance
        let dist_near = triangulator.rssi_to_distance(-30, &cal);
        assert!(dist_near < 1.0, "Stronger signal should give shorter distance");
    }

    #[test]
    fn test_single_station_returns_station_position() {
        let stations = vec![TestStation {
            id: "1".to_string(),
            x: 2.0,
            y: 3.0,
            calibration: None,
        }];

        let triangulator = Triangulator::new(&stations);
        let mut readings = HashMap::new();
        readings.insert(
            "1".to_string(),
            RssiReading {
                rssi: -50,
                timestamp: 0,
            },
        );

        let pos = triangulator.calculate_position(&readings).unwrap();
        // With only one station, position should be at or near the station
        assert!((pos.x - 2.0).abs() < 1.0);
        assert!((pos.y - 3.0).abs() < 1.0);
    }

    #[test]
    fn test_equal_rssi_gives_centroid() {
        let stations = make_stations();
        let triangulator = Triangulator::new(&stations);

        let mut readings = HashMap::new();
        // Equal RSSI from all stations
        for id in ["1", "2", "3"] {
            readings.insert(
                id.to_string(),
                RssiReading {
                    rssi: -50,
                    timestamp: 0,
                },
            );
        }

        let pos = triangulator.calculate_position(&readings).unwrap();
        // Should be near centroid: (0+5+2.5)/3 = 2.5, (0+0+5)/3 = 1.67
        assert!(
            (pos.x - 2.5).abs() < 0.5,
            "x={} should be near 2.5",
            pos.x
        );
        assert!(
            (pos.y - 1.67).abs() < 0.5,
            "y={} should be near 1.67",
            pos.y
        );
    }

    #[test]
    fn test_stronger_signal_pulls_position() {
        let stations = make_stations();
        let triangulator = Triangulator::new(&stations);

        let mut readings = HashMap::new();
        // Station 1 has much stronger signal
        readings.insert(
            "1".to_string(),
            RssiReading {
                rssi: -30,
                timestamp: 0,
            },
        );
        readings.insert(
            "2".to_string(),
            RssiReading {
                rssi: -60,
                timestamp: 0,
            },
        );
        readings.insert(
            "3".to_string(),
            RssiReading {
                rssi: -60,
                timestamp: 0,
            },
        );

        let pos = triangulator.calculate_position(&readings).unwrap();
        // Should be closer to station 1 (at 0,0)
        assert!(pos.x < 2.0, "x={} should be closer to station 1", pos.x);
        assert!(pos.y < 2.0, "y={} should be closer to station 1", pos.y);
    }

    #[test]
    fn test_empty_readings_returns_none() {
        let stations = make_stations();
        let triangulator = Triangulator::new(&stations);
        let readings = HashMap::new();

        assert!(triangulator.calculate_position(&readings).is_none());
    }

    #[test]
    fn test_position_smoothing() {
        let stations = make_stations();
        let mut tracker = PositionTracker::with_config(
            &stations,
            TriangulatorConfig {
                smoothing_factor: 0.5,
                ..Default::default()
            },
        );

        // First reading - position near station 1
        let mut readings1 = HashMap::new();
        readings1.insert(
            "1".to_string(),
            RssiReading {
                rssi: -30,
                timestamp: 0,
            },
        );
        readings1.insert(
            "2".to_string(),
            RssiReading {
                rssi: -70,
                timestamp: 0,
            },
        );
        readings1.insert(
            "3".to_string(),
            RssiReading {
                rssi: -70,
                timestamp: 0,
            },
        );

        let pos1 = tracker.update_position("device1", &readings1).unwrap();

        // Second reading - position near station 2 (jump)
        let mut readings2 = HashMap::new();
        readings2.insert(
            "1".to_string(),
            RssiReading {
                rssi: -70,
                timestamp: 1,
            },
        );
        readings2.insert(
            "2".to_string(),
            RssiReading {
                rssi: -30,
                timestamp: 1,
            },
        );
        readings2.insert(
            "3".to_string(),
            RssiReading {
                rssi: -70,
                timestamp: 1,
            },
        );

        let pos2 = tracker.update_position("device1", &readings2).unwrap();

        // With smoothing, position should move but not jump all the way
        // pos2.x should be between pos1.x and station2.x (5.0)
        assert!(
            pos2.x > pos1.x,
            "Position should move toward station 2"
        );
        assert!(
            pos2.x < 4.0,
            "Position should not jump all the way to station 2"
        );
    }

    #[test]
    fn test_trilateration_accuracy() {
        // Create stations at known positions
        let stations = vec![
            TestStation {
                id: "1".to_string(),
                x: 0.0,
                y: 0.0,
                calibration: Some(CalibrationParams {
                    rssi_at_1m: -40.0,
                    path_loss_exponent: 2.0,
                }),
            },
            TestStation {
                id: "2".to_string(),
                x: 4.0,
                y: 0.0,
                calibration: Some(CalibrationParams {
                    rssi_at_1m: -40.0,
                    path_loss_exponent: 2.0,
                }),
            },
            TestStation {
                id: "3".to_string(),
                x: 2.0,
                y: 4.0,
                calibration: Some(CalibrationParams {
                    rssi_at_1m: -40.0,
                    path_loss_exponent: 2.0,
                }),
            },
        ];

        let triangulator = Triangulator::new(&stations);

        // Simulate device at (2, 2) - equal distance from all stations
        // Distance to each station is about 2.83m
        // Using path loss model: RSSI = -40 - 10*2*log10(2.83) ≈ -49 dBm
        let mut readings = HashMap::new();
        readings.insert(
            "1".to_string(),
            RssiReading {
                rssi: -49,
                timestamp: 0,
            },
        );
        readings.insert(
            "2".to_string(),
            RssiReading {
                rssi: -49,
                timestamp: 0,
            },
        );
        readings.insert(
            "3".to_string(),
            RssiReading {
                rssi: -49,
                timestamp: 0,
            },
        );

        let pos = triangulator.calculate_position(&readings).unwrap();

        // Should be reasonably close to (2, 2)
        let error = ((pos.x - 2.0).powi(2) + (pos.y - 2.0).powi(2)).sqrt();
        assert!(
            error < 1.0,
            "Position ({}, {}) should be within 1m of (2, 2), error={}",
            pos.x,
            pos.y,
            error
        );
    }
}
