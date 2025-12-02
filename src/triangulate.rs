//! Triangulation module for calculating device positions from RSSI readings.
//!
//! Uses weighted centroid algorithm where each station's position is weighted
//! by a function of its RSSI reading.

use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// A calculated 2D position in meters
#[derive(Debug, Clone, Copy, Serialize, Deserialize, Default, PartialEq)]
pub struct Position {
    pub x: f32,
    pub y: f32,
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
    -45.0
}

fn default_path_loss_exponent() -> f32 {
    3.0
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

/// Triangulator that computes device positions from RSSI readings
pub struct Triangulator {
    stations: HashMap<String, StationData>,
}

impl Triangulator {
    /// Create a new triangulator from station configurations
    ///
    /// # Arguments
    /// * `stations` - Slice of station configs with position and optional calibration
    pub fn new<S>(stations: &[S]) -> Self
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

        Self {
            stations: station_map,
        }
    }

    /// Calculate position using weighted centroid algorithm
    ///
    /// Formula: Position = sum(weight_i * position_i) / sum(weight_i)
    ///
    /// Returns None if no valid readings or no matching stations
    pub fn calculate_position(&self, readings: &HashMap<String, RssiReading>) -> Option<Position> {
        if readings.is_empty() {
            return None;
        }

        let mut total_weight = 0.0f32;
        let mut weighted_x = 0.0f32;
        let mut weighted_y = 0.0f32;

        for (station_id, reading) in readings {
            if let Some(station) = self.stations.get(station_id) {
                let weight = self.calculate_weight(reading.rssi);

                if weight > 0.0 && weight.is_finite() {
                    weighted_x += weight * station.x;
                    weighted_y += weight * station.y;
                    total_weight += weight;
                }
            }
        }

        if total_weight > 0.0 {
            Some(Position {
                x: weighted_x / total_weight,
                y: weighted_y / total_weight,
            })
        } else {
            None
        }
    }

    /// Calculate position, filtering out stale readings
    ///
    /// # Arguments
    /// * `readings` - All readings for the device
    /// * `max_age_secs` - Maximum age of readings to consider
    /// * `current_time` - Current timestamp for age calculation
    pub fn calculate_position_with_max_age(
        &self,
        readings: &HashMap<String, RssiReading>,
        max_age_secs: u64,
        current_time: u64,
    ) -> Option<Position> {
        let filtered: HashMap<String, RssiReading> = readings
            .iter()
            .filter(|(_, r)| current_time.saturating_sub(r.timestamp) < max_age_secs)
            .map(|(k, v)| (k.clone(), v.clone()))
            .collect();

        self.calculate_position(&filtered)
    }

    /// Calculate weight from RSSI using exponential weighting
    ///
    /// Exponential weighting: 10^(rssi/20)
    /// This gives stronger signals exponentially higher weights
    fn calculate_weight(&self, rssi: i8) -> f32 {
        // Exponential weighting: 10^(rssi/20)
        // -40 dBm -> 0.01, -60 dBm -> 0.001, -80 dBm -> 0.0001
        // Stronger signals get exponentially higher weights
        10.0_f32.powf(rssi as f32 / 20.0)
    }

    /// Convert RSSI to estimated distance using log-distance path loss model
    ///
    /// Formula: distance = 10^((rssi_at_1m - rssi) / (10 * path_loss_exponent))
    #[allow(dead_code)]
    fn rssi_to_distance(&self, rssi: i8, calibration: &CalibrationParams) -> f32 {
        let exponent =
            (calibration.rssi_at_1m - rssi as f32) / (10.0 * calibration.path_loss_exponent);
        10.0_f32.powf(exponent)
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
        assert_eq!(pos.x, 2.0);
        assert_eq!(pos.y, 3.0);
    }

    #[test]
    fn test_equal_rssi_gives_midpoint() {
        let stations = vec![
            TestStation {
                id: "1".to_string(),
                x: 0.0,
                y: 0.0,
                calibration: None,
            },
            TestStation {
                id: "2".to_string(),
                x: 4.0,
                y: 0.0,
                calibration: None,
            },
        ];

        let triangulator = Triangulator::new(&stations);
        let mut readings = HashMap::new();
        readings.insert(
            "1".to_string(),
            RssiReading {
                rssi: -50,
                timestamp: 0,
            },
        );
        readings.insert(
            "2".to_string(),
            RssiReading {
                rssi: -50,
                timestamp: 0,
            },
        );

        let pos = triangulator.calculate_position(&readings).unwrap();
        assert!((pos.x - 2.0).abs() < 0.01);
        assert!((pos.y - 0.0).abs() < 0.01);
    }

    #[test]
    fn test_stronger_signal_pulls_position() {
        let stations = vec![
            TestStation {
                id: "1".to_string(),
                x: 0.0,
                y: 0.0,
                calibration: None,
            },
            TestStation {
                id: "2".to_string(),
                x: 4.0,
                y: 0.0,
                calibration: None,
            },
        ];

        let triangulator = Triangulator::new(&stations);
        let mut readings = HashMap::new();
        readings.insert(
            "1".to_string(),
            RssiReading {
                rssi: -40,
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

        let pos = triangulator.calculate_position(&readings).unwrap();
        assert!(pos.x < 2.0); // Should be closer to station 1
    }

    #[test]
    fn test_empty_readings_returns_none() {
        let stations = vec![TestStation {
            id: "1".to_string(),
            x: 0.0,
            y: 0.0,
            calibration: None,
        }];

        let triangulator = Triangulator::new(&stations);
        let readings = HashMap::new();

        assert!(triangulator.calculate_position(&readings).is_none());
    }

    #[test]
    fn test_unknown_station_ignored() {
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
        readings.insert(
            "unknown".to_string(),
            RssiReading {
                rssi: -30,
                timestamp: 0,
            },
        );

        let pos = triangulator.calculate_position(&readings).unwrap();
        // Should only use station 1, ignoring unknown
        assert_eq!(pos.x, 2.0);
        assert_eq!(pos.y, 3.0);
    }

    #[test]
    fn test_three_stations_triangulation() {
        let stations = vec![
            TestStation {
                id: "1".to_string(),
                x: 0.0,
                y: 0.0,
                calibration: None,
            },
            TestStation {
                id: "2".to_string(),
                x: 4.0,
                y: 0.0,
                calibration: None,
            },
            TestStation {
                id: "3".to_string(),
                x: 2.0,
                y: 4.0,
                calibration: None,
            },
        ];

        let triangulator = Triangulator::new(&stations);
        let mut readings = HashMap::new();
        readings.insert(
            "1".to_string(),
            RssiReading {
                rssi: -50,
                timestamp: 0,
            },
        );
        readings.insert(
            "2".to_string(),
            RssiReading {
                rssi: -50,
                timestamp: 0,
            },
        );
        readings.insert(
            "3".to_string(),
            RssiReading {
                rssi: -50,
                timestamp: 0,
            },
        );

        let pos = triangulator.calculate_position(&readings).unwrap();
        // Equal weights should give centroid: (0+4+2)/3 = 2, (0+0+4)/3 = 1.33
        assert!((pos.x - 2.0).abs() < 0.01);
        assert!((pos.y - 4.0 / 3.0).abs() < 0.01);
    }
}
