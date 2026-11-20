//! Scenario files (TOML via serde). A scenario fully determines a run:
//! geometry, population, demand, model parameters, auto shares and events.
//! Parsing and printing are pure string functions; file I/O lives in the
//! app and the CLI.

use serde::{Deserialize, Serialize};

use crate::bus::BusParams;
use crate::demand::DemandConfig;
use crate::params::{AutoParams, HumanParams, MobilParams, Params, TruckParams};
use crate::road::{CorridorGeometry, Road};

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum RoadConfig {
    /// Single-direction ring for validation experiments.
    Ring {
        length_m: f64,
        #[serde(default = "one")]
        lanes: u8,
        #[serde(default = "default_ring_limit")]
        speed_limit_kmh: f64,
    },
    /// The City A <-> City B corridor.
    Corridor(CorridorGeometry),
}

fn one() -> u8 {
    1
}

fn default_ring_limit() -> f64 {
    120.0
}

impl RoadConfig {
    pub fn build(&self) -> Road {
        match self {
            RoadConfig::Ring { length_m, lanes, speed_limit_kmh } => {
                Road::ring(*length_m, (*lanes).max(1), speed_limit_kmh / 3.6)
            }
            RoadConfig::Corridor(g) => Road::corridor(g),
        }
    }
}

/// Initial vehicles placed at t = 0.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct Population {
    /// Ring: number of vehicles, spread evenly over the lanes.
    pub count: usize,
    /// Corridor: initial density on the mainline, veh/km/lane.
    pub initial_density: f64,
    /// Initial speed, m/s (negative = equilibrium speed for the spacing).
    pub initial_speed: f64,
    pub truck_share: f64,
}

impl Default for Population {
    fn default() -> Self {
        Population { count: 0, initial_density: 0.0, initial_speed: -1.0, truck_share: 0.0 }
    }
}

/// Auto share for the left and right panes of the split view.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct Mix {
    pub left: f64,
    pub right: f64,
}

impl Default for Mix {
    fn default() -> Self {
        Mix { left: 0.0, right: 1.0 }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct Scenario {
    pub name: String,
    pub description: String,
    pub seed: u64,
    /// Fixed time step, s.
    pub dt: f64,
    /// Default run length for headless runs, s.
    pub duration_s: f64,
    pub road: RoadConfig,
    pub population: Population,
    pub demand: DemandConfig,
    pub mix: Mix,
    pub human: HumanParams,
    pub auto: AutoParams,
    pub truck: TruckParams,
    pub mobil: MobilParams,
    pub bus: BusParams,
}

impl Default for Scenario {
    fn default() -> Self {
        Scenario {
            name: "Untitled".into(),
            description: String::new(),
            seed: 1,
            dt: 0.1,
            duration_s: 600.0,
            road: RoadConfig::Ring { length_m: 1000.0, lanes: 1, speed_limit_kmh: 120.0 },
            population: Population::default(),
            demand: DemandConfig::default(),
            mix: Mix::default(),
            human: HumanParams::default(),
            auto: AutoParams::default(),
            truck: TruckParams::default(),
            mobil: MobilParams::default(),
            bus: BusParams::default(),
        }
    }
}

impl Scenario {
    pub fn from_toml(text: &str) -> Result<Scenario, String> {
        toml::from_str(text).map_err(|e| e.to_string())
    }

    pub fn to_toml(&self) -> Result<String, String> {
        toml::to_string_pretty(self).map_err(|e| e.to_string())
    }

    pub fn params(&self) -> Params {
        Params {
            human: self.human.clone(),
            auto: self.auto.clone(),
            truck: self.truck.clone(),
            mobil: self.mobil.clone(),
        }
    }
}
