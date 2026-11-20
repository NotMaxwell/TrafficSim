//! Corridor configuration as written in scenario files: length, lanes by
//! mile, speed limits by mile, end names, and interchanges.

use serde::{Deserialize, Serialize};

use crate::road::Dir;

/// A mid-corridor interchange. Ramps can be one-directional (partial
/// interchanges) and carry their own demand.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct Interchange {
    pub mile: f64,
    pub name: String,
    pub eb_off: bool,
    pub eb_on: bool,
    pub wb_off: bool,
    pub wb_on: bool,
    /// Peak on-ramp demand, veh/h (None = scenario default).
    pub on_vph_eb: Option<f64>,
    pub on_vph_wb: Option<f64>,
    /// Share of passing traffic leaving here (None = scenario default).
    pub exit_share_eb: Option<f64>,
    pub exit_share_wb: Option<f64>,
}

impl Default for Interchange {
    fn default() -> Self {
        Interchange {
            mile: 0.0,
            name: String::new(),
            eb_off: true,
            eb_on: true,
            wb_off: true,
            wb_on: true,
            on_vph_eb: None,
            on_vph_wb: None,
            exit_share_eb: None,
            exit_share_wb: None,
        }
    }
}

/// Lane count over a stretch of corridor (`dir` = None: both directions).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct LaneSegment {
    pub from_mi: f64,
    pub to_mi: f64,
    pub lanes: u8,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub dir: Option<Dir>,
}

/// Speed limit over a stretch of corridor (both directions).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct LimitSegment {
    pub from_mi: f64,
    pub to_mi: f64,
    pub mph: f64,
}

/// Geometry settings for the corridor (part of the scenario file).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct CorridorGeometry {
    pub corridor_mi: f64,
    pub lanes: u8,
    pub loop_lanes: u8,
    pub loop_m: f64,
    pub ramp_zone_m: f64,
    pub speed_limit_mph: f64,
    pub loop_limit_mph: f64,
    /// Simple full interchanges at these miles (named "Mile m").
    pub interchanges_mi: Vec<f64>,
    /// Named interchanges with optional per-direction ramps and demand.
    pub interchanges: Vec<Interchange>,
    pub lane_segments: Vec<LaneSegment>,
    pub limit_segments: Vec<LimitSegment>,
    /// Names of the ends: mile 0 (west) and the far end (east).
    pub city_a: String,
    pub city_b: String,
    /// City ends are freeway terminals (multi-lane entry/exit). When false
    /// they are ordinary single-lane ramps like the interchanges.
    pub city_terminal: bool,
}

impl Default for CorridorGeometry {
    fn default() -> Self {
        CorridorGeometry {
            corridor_mi: 20.0,
            lanes: 3,
            loop_lanes: 2,
            loop_m: 500.0,
            ramp_zone_m: 300.0,
            speed_limit_mph: 70.0,
            loop_limit_mph: 35.0,
            interchanges_mi: Vec::new(),
            interchanges: Vec::new(),
            lane_segments: Vec::new(),
            limit_segments: Vec::new(),
            city_a: "City A".into(),
            city_b: "City B".into(),
            city_terminal: true,
        }
    }
}
