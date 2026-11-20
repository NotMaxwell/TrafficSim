//! traffic-core: highway traffic simulation engine.
//!
//! Pure library: no GUI, no files, no printing. A `World` is advanced by
//! `step(&mut world)` with a fixed time step; everything random comes from
//! keyed streams (see `rng`), so runs are bit-for-bit repeatable.

pub mod auto;
pub mod bus;
pub mod demand;
pub mod human;
pub mod idm;
pub mod mobil;
pub mod params;
pub mod rng;
pub mod road;
pub mod scenario;
pub mod sim;
pub mod snapshot;
pub mod step;
pub mod vehicles;

use std::hash::{Hash, Hasher};

pub use params::Params;
pub use road::Road;
pub use scenario::Scenario;
pub use sim::{Side, Sim};
pub use step::step;
pub use vehicles::{Kind, Vehicles};

pub fn version() -> &'static str {
    env!("CARGO_PKG_VERSION")
}

/// Running counters. `spawned = removed + present` always holds.
#[derive(Clone, Debug, Default, PartialEq, Eq, Hash)]
pub struct Stats {
    pub spawned: u64,
    pub removed: u64,
    pub autos_spawned: u64,
    pub collisions: u64,
    pub lane_changes: u64,
    /// Lane changes that forced the new follower to brake beyond b_safe.
    pub unsafe_lane_changes: u64,
    pub hard_brakes: u64,
}

/// One simulated world (one pane of the split view).
#[derive(Clone, Debug)]
pub struct World {
    pub road: Road,
    pub params: Params,
    pub veh: Vehicles,
    pub seed: u64,
    pub dt: f64,
    pub time: f64,
    pub tick: u64,
    /// Probability that a new vehicle is an auto.
    pub auto_share: f64,
    pub truck_share: f64,
    pub stats: Stats,
    pub demand: demand::Demand,
    pub bus: bus::Bus,
    pub(crate) next_id: u64,
}

impl World {
    /// Build a world from a scenario with the given auto share.
    pub fn new(sc: &Scenario, auto_share: f64) -> World {
        let road = sc.road.build();
        let demand = demand::Demand::new(&sc.demand, &road, sc.seed);
        let mut w = World {
            demand,
            bus: bus::Bus::new(&sc.bus, sc.dt, sc.seed),
            veh: Vehicles::new(road.lanes),
            road,
            params: sc.params(),
            seed: sc.seed,
            dt: sc.dt,
            time: 0.0,
            tick: 0,
            auto_share: auto_share.clamp(0.0, 1.0),
            truck_share: sc.population.truck_share,
            stats: Stats::default(),
            next_id: 0,
        };
        demand::populate(&mut w, &sc.population);
        w.veh.reindex();
        w
    }

    /// Hash of the full dynamic state, for determinism checks.
    pub fn state_hash(&self) -> u64 {
        let mut h = std::collections::hash_map::DefaultHasher::new();
        self.tick.hash(&mut h);
        self.stats.hash(&mut h);
        let mut order: Vec<usize> = (0..self.veh.len()).collect();
        order.sort_by_key(|&i| self.veh.id[i]);
        for i in order {
            self.veh.id[i].hash(&mut h);
            self.veh.s[i].to_bits().hash(&mut h);
            self.veh.v[i].to_bits().hash(&mut h);
            self.veh.a[i].to_bits().hash(&mut h);
            self.veh.lane[i].hash(&mut h);
        }
        h.finish()
    }

    /// Turn vehicle `slot` into an auto (keeps its position and speed).
    pub fn make_auto(&mut self, slot: usize) {
        let ag = &mut self.veh.agent[slot];
        let mut idm = self.params.auto.idm;
        idm.v0 = self.road.limit;
        ag.driver = vehicles::Driver {
            idm,
            politeness: self.params.auto.politeness,
            threshold: self.params.auto.threshold,
            tau: 0.0,
            anticipation: 1,
        };
        ag.auto = auto::AutoState { v_avg: self.veh.v[slot], v_cmd: self.veh.v[slot], ..Default::default() };
        self.veh.kind[slot] = Kind::Auto;
    }

    /// Called just before a vehicle leaves through off-ramp `ramp`.
    pub(crate) fn on_exit(&mut self, _slot: usize, _ramp: u16) {}

    /// Vehicles waiting in ramp queues.
    pub fn queued(&self) -> usize {
        self.demand.queued()
    }

    /// Advance by `n` ticks.
    pub fn run_ticks(&mut self, n: u64) {
        for _ in 0..n {
            step(self);
        }
    }

    /// Advance by `seconds` of simulated time.
    pub fn run_for(&mut self, seconds: f64) {
        let n = (seconds / self.dt).round() as u64;
        self.run_ticks(n);
    }

    /// Mean speed over all vehicles, m/s.
    pub fn mean_speed(&self) -> f64 {
        if self.veh.is_empty() {
            return 0.0;
        }
        self.veh.v.iter().sum::<f64>() / self.veh.len() as f64
    }
}
