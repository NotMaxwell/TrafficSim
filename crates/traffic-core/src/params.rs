//! Tunable model parameters, grouped by concept. All values are SI
//! (m, s, m/s, m/s²). Every struct has documented defaults from the design
//! doc and deserializes with missing fields filled from those defaults.

use serde::{Deserialize, Serialize};

use crate::idm::Idm;

const KMH: f64 = 1.0 / 3.6;

/// Human driver population and the Human Driver Model extensions.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct HumanParams {
    /// Baseline IDM (v0 is the mainline desired speed before spread).
    pub idm: Idm,
    /// Relative spread of v0 (uniform ±).
    pub v0_spread: f64,
    /// Desired time gap range (uniform).
    pub t_min: f64,
    pub t_max: f64,
    /// MOBIL politeness range (uniform).
    pub polite_min: f64,
    pub polite_max: f64,
    /// Car length, m.
    pub length: f64,
    /// White acceleration noise σ, m/s² (0 = off). Small perturbation source.
    pub accel_noise: f64,

    /// Reaction delay on/off and its lognormal distribution.
    pub delay: bool,
    pub tau_median: f64,
    pub tau_sigma: f64,
    pub tau_min: f64,
    pub tau_max: f64,

    /// Multi-anticipation: max leaders watched (1 to 3) and range, m.
    pub anticipation_max: u8,
    pub perception_range: f64,
    /// Mirror range for lane changes, m behind / ahead.
    pub mirror_back: f64,
    pub mirror_front: f64,

    /// Estimation errors (Wiener processes).
    pub estimation: bool,
    /// Relative gap error (≈5% of gap).
    pub gap_error: f64,
    /// Inverse-TTC error for closing speed, 1/s.
    pub rate_error: f64,
    /// Correlation time of estimation errors, s.
    pub error_corr_time: f64,

    /// Distraction: mean interval and duration range, s.
    pub distraction: bool,
    pub distraction_interval: f64,
    pub distraction_min: f64,
    pub distraction_max: f64,

    /// Crashes freeze both vehicles for this long, s, before they are towed.
    pub crash_clear_time: f64,
}

impl Default for HumanParams {
    fn default() -> Self {
        HumanParams {
            idm: Idm { v0: 120.0 * KMH, t: 1.5, s0: 2.0, a: 1.4, b: 2.0, delta: 4.0 },
            v0_spread: 0.10,
            t_min: 1.0,
            t_max: 2.0,
            polite_min: 0.0,
            polite_max: 0.5,
            length: 5.0,
            accel_noise: 0.0,
            delay: true,
            tau_median: 1.1,
            tau_sigma: 0.3,
            tau_min: 0.7,
            tau_max: 2.5,
            anticipation_max: 3,
            perception_range: 200.0,
            mirror_back: 60.0,
            mirror_front: 100.0,
            estimation: true,
            gap_error: 0.05,
            rate_error: 0.01,
            error_corr_time: 20.0,
            distraction: true,
            distraction_interval: 300.0,
            distraction_min: 1.0,
            distraction_max: 3.0,
            crash_clear_time: 900.0,
        }
    }
}

impl HumanParams {
    /// Ideal drivers: no delay, no noise, full perception. Used by tests that
    /// isolate a single mechanism (e.g. MOBIL safety).
    pub fn ideal() -> Self {
        HumanParams {
            delay: false,
            estimation: false,
            distraction: false,
            anticipation_max: 1,
            mirror_back: 1000.0,
            mirror_front: 1000.0,
            ..Default::default()
        }
    }
}

/// Automated (connected) vehicles.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct AutoParams {
    /// IDM envelope (v0 = speed limit, no spread).
    pub idm: Idm,
    /// CACC time gap, s.
    pub t_cacc: f64,
    /// ACC fallback time gap, s.
    pub t_acc: f64,
    /// CACC gains (Ploeg-style: spacing error and its derivative).
    pub kp: f64,
    pub kd: f64,
    /// Feed-forward weight on the leader's commanded acceleration.
    pub kff: f64,
    /// Radar range, m.
    pub radar_range: f64,
    /// Actuator lag time constant, s.
    pub actuator_tau: f64,
    pub politeness: f64,
    pub threshold: f64,
    /// Wave look-ahead distance, m (0 = off).
    pub lookahead: f64,
    /// Extra time gap opened when a slowdown is detected ahead, s.
    pub buffer_gap: f64,
    /// Stern-style wave dampener when following a human in ACC mode.
    pub dampener: bool,
    pub length: f64,
}

impl Default for AutoParams {
    fn default() -> Self {
        AutoParams {
            idm: Idm { v0: 112.0 * KMH, t: 0.6, s0: 2.0, a: 1.4, b: 2.0, delta: 4.0 },
            t_cacc: 0.6,
            t_acc: 1.1,
            kp: 0.2,
            kd: 0.7,
            kff: 1.0,
            radar_range: 200.0,
            actuator_tau: 0.2,
            politeness: 0.5,
            threshold: 0.1,
            lookahead: 2000.0,
            buffer_gap: 0.4,
            dampener: true,
            length: 5.0,
        }
    }
}

/// Heavy vehicles: longer, slower, and they block the view of cars behind.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct TruckParams {
    pub length: f64,
    pub v0: f64,
    pub a: f64,
    pub b: f64,
    pub t: f64,
}

impl Default for TruckParams {
    fn default() -> Self {
        TruckParams { length: 15.0, v0: 95.0 * KMH, a: 0.7, b: 1.5, t: 1.8 }
    }
}

/// Lane-changing (MOBIL) settings shared by all drivers.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct MobilParams {
    /// Human switching threshold, m/s².
    pub threshold: f64,
    /// Keep-right bias, m/s².
    pub bias_right: f64,
    /// Max braking a lane change may impose on the new follower, m/s².
    pub b_safe: f64,
    /// Seconds between lane-change checks per vehicle (staggered by id).
    pub check_interval: f64,
    /// Distance before an exit at which drivers start moving right, m.
    pub exit_prepare: f64,
    /// Duration of the drawn lateral move, s (visual only).
    pub change_duration: f64,
}

impl Default for MobilParams {
    fn default() -> Self {
        MobilParams {
            threshold: 0.2,
            bias_right: 0.15,
            b_safe: 4.0,
            check_interval: 1.0,
            exit_prepare: 2000.0,
            change_duration: 2.0,
        }
    }
}

/// Everything a world needs besides geometry and demand.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct Params {
    pub human: HumanParams,
    pub auto: AutoParams,
    pub truck: TruckParams,
    pub mobil: MobilParams,
}
