//! Spawning and removal: initial population, per-vehicle traits, ramp
//! arrivals (Poisson, per-ramp random stream), ramp queues, routing to an
//! exit, and removal at off-ramp gores.
//!
//! Ids are assigned at *arrival*, in a fixed ramp order, so vehicle #4812
//! arrives at the same second with the same traits in both worlds.

use std::collections::VecDeque;

use rand::Rng;
use rand_chacha::ChaCha8Rng;
use serde::{Deserialize, Serialize};

use crate::auto::AutoState;
use crate::human::HumanState;
use crate::idm::Idm;
use crate::rng::{self, Purpose, VehRng};
use crate::road::{RampKind, Road};
use crate::scenario::Population;
use crate::vehicles::{Agent, Driver, Kind, NO_RAMP};
use crate::World;

/// Demand settings (part of the scenario file). Rates in veh/h.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct DemandConfig {
    pub city_a_vph: f64,
    pub city_b_vph: f64,
    /// Each interchange on-ramp.
    pub interchange_vph: f64,
    /// Share of city-bound traffic that wraps through the far end loop.
    pub through_share: f64,
    /// Share of passing traffic leaving at each interchange.
    pub interchange_exit_share: f64,
    /// Clock time at t = 0, hours.
    pub start_hour: f64,
    /// Time-of-day multipliers [(hour, factor)], piecewise linear.
    pub profile_a: Vec<[f64; 2]>,
    pub profile_b: Vec<[f64; 2]>,
    /// Global scale (UI slider).
    pub multiplier: f64,
}

impl Default for DemandConfig {
    fn default() -> Self {
        DemandConfig {
            city_a_vph: 0.0,
            city_b_vph: 0.0,
            interchange_vph: 0.0,
            through_share: 0.1,
            interchange_exit_share: 0.1,
            start_hour: 7.0,
            profile_a: Vec::new(),
            profile_b: Vec::new(),
            multiplier: 1.0,
        }
    }
}

/// Piecewise-linear profile lookup (wraps over 24 h). Empty = 1.
pub fn profile_at(profile: &[[f64; 2]], hour: f64) -> f64 {
    if profile.is_empty() {
        return 1.0;
    }
    let h = hour.rem_euclid(24.0);
    let n = profile.len();
    for k in 0..n {
        let [h0, f0] = profile[k];
        let [h1, f1] = profile[(k + 1) % n];
        let h1 = if k + 1 == n { h1 + 24.0 } else { h1 };
        let hh = if h < h0 { h + 24.0 } else { h };
        if hh >= h0 && hh < h1 {
            return f0 + (f1 - f0) * (hh - h0) / (h1 - h0);
        }
    }
    profile[0][1]
}

/// Runtime state of one on-ramp.
#[derive(Clone, Debug)]
pub struct RampState {
    pub ramp: u16,
    pub base_vph: f64,
    pub city_a: bool,
    /// Extra demand from surge events, veh/h.
    pub surge_vph: f64,
    rng: ChaCha8Rng,
    /// Waiting vehicles: (id, arrival time).
    pub queue: VecDeque<(u64, f64)>,
    pub arrived: u64,
    pub entered: u64,
}

#[derive(Clone, Debug, Default)]
pub struct Demand {
    pub cfg: DemandConfig,
    pub ramps: Vec<RampState>,
    /// Vehicles removed per off-ramp id.
    pub exited: Vec<u64>,
}

impl Demand {
    pub fn new(cfg: &DemandConfig, road: &Road, seed: u64) -> Demand {
        let ramps = road
            .on_ramps()
            .map(|r| {
                let city_a = r.city && r.mile == 0.0;
                let base_vph = match (r.city, city_a) {
                    (true, true) => cfg.city_a_vph,
                    (true, false) => cfg.city_b_vph,
                    _ => cfg.interchange_vph,
                };
                RampState {
                    ramp: r.id,
                    base_vph,
                    city_a,
                    surge_vph: 0.0,
                    rng: rng::stream(seed, Purpose::Arrivals, r.id as u64),
                    queue: VecDeque::new(),
                    arrived: 0,
                    entered: 0,
                }
            })
            .collect();
        Demand { cfg: cfg.clone(), ramps, exited: vec![0; road.ramps.len()] }
    }

    /// Current arrival rate of ramp state `k` at simulated time `t`, veh/h.
    pub fn rate(&self, k: usize, t: f64) -> f64 {
        let r = &self.ramps[k];
        let hour = self.cfg.start_hour + t / 3600.0;
        let prof = if r.city_a { &self.cfg.profile_a } else { &self.cfg.profile_b };
        (r.base_vph * profile_at(prof, hour) * self.cfg.multiplier + r.surge_vph).max(0.0)
    }

    pub fn queued(&self) -> usize {
        self.ramps.iter().map(|r| r.queue.len()).sum()
    }
}

/// Draw a new vehicle's kind, traits and length from its id-keyed streams.
/// Every number is always drawn in the same order, so a given id gets the
/// same personality in every world regardless of auto share.
pub fn make_agent(w: &World, id: u64, origin: u16, exit: u16) -> (Kind, Agent, f64) {
    let p = &w.params;
    let mut r = rng::stream(w.seed, Purpose::Traits, id);
    let is_truck = r.random::<f64>() < w.truck_share;
    let u_v0: f64 = r.random();
    let u_t: f64 = r.random();
    let u_pol: f64 = r.random();
    let z_tau = rng::normal(&mut r);
    let u_ant: f64 = r.random();
    let kind = if rng::uniform(w.seed, Purpose::Kind, id) < w.auto_share { Kind::Auto } else { Kind::Human };

    let h = &p.human;
    let mut driver = match kind {
        Kind::Human => {
            let mut idm = h.idm;
            idm.v0 *= 1.0 + h.v0_spread * (2.0 * u_v0 - 1.0);
            idm.t = h.t_min + (h.t_max - h.t_min) * u_t;
            let tau = if h.delay { (h.tau_median * (h.tau_sigma * z_tau).exp()).clamp(h.tau_min, h.tau_max) } else { 0.0 };
            // Drivers who react slowly keep at least their reaction time as
            // headway (otherwise τ > T makes rear-end crashes routine).
            idm.t = idm.t.max(tau * h.gap_per_tau);
            let max_ant = h.anticipation_max.clamp(1, 3);
            let min_ant = h.anticipation_min.clamp(1, max_ant);
            let span = max_ant - min_ant + 1;
            Driver {
                idm,
                politeness: h.polite_min + (h.polite_max - h.polite_min) * u_pol,
                threshold: p.mobil.threshold,
                tau,
                anticipation: min_ant + ((u_ant * span as f64) as u8).min(span - 1),
            }
        }
        Kind::Auto => {
            let mut idm = p.auto.idm;
            idm.v0 = w.road.limit;
            Driver { idm, politeness: p.auto.politeness, threshold: p.auto.threshold, tau: 0.0, anticipation: 1 }
        }
    };
    let mut len = match kind {
        Kind::Human => h.length,
        Kind::Auto => p.auto.length,
    };
    if is_truck {
        let t = &p.truck;
        driver.idm = Idm { v0: t.v0.min(driver.idm.v0), a: t.a, b: t.b, t: t.t.max(driver.idm.t), ..driver.idm };
        len = t.length;
    }
    let agent = Agent {
        driver,
        truck: is_truck,
        origin,
        exit,
        spawn_t: w.time,
        crashed_until: -1.0,
        forced: None,
        human: HumanState::default(),
        auto: AutoState::default(),
        rng: VehRng::new(w.seed, id),
        last_change: f64::NEG_INFINITY,
        odo: 0.0,
        s_prev: 0.0,
        hard_braking: false,
    };
    (kind, agent, len)
}

/// Add vehicle `id` to the world and return its slot. The lane index is
/// stale until the caller runs `reindex`.
pub fn add_vehicle(w: &mut World, id: u64, s: f64, lane: u8, v: f64, origin: u16, exit: u16) -> usize {
    let (kind, mut agent, len) = make_agent(w, id, origin, exit);
    agent.s_prev = s;
    agent.auto.v_avg = v;
    agent.auto.v_cmd = v;
    w.stats.spawned += 1;
    if kind == Kind::Auto {
        w.stats.autos_spawned += 1;
    }
    w.veh.push(id, s, v, lane, len, kind, agent)
}

fn new_id(w: &mut World) -> u64 {
    let id = w.next_id;
    w.next_id += 1;
    id
}

/// Pick an exit for a vehicle that is at `s`: interchanges take a share of
/// passing traffic, the first city exit takes all but the through share,
/// the second city exit takes everyone left.
pub fn choose_exit(road: &Road, seed: u64, id: u64, s: f64, cfg: &DemandConfig) -> u16 {
    let exits = road.exits_downstream(s);
    if exits.is_empty() {
        return NO_RAMP;
    }
    let mut r = rng::stream(seed, Purpose::Route, id);
    let mut cities = 0;
    for e in &exits {
        let p = if e.city {
            cities += 1;
            if cities == 1 { 1.0 - cfg.through_share } else { 1.0 }
        } else {
            cfg.interchange_exit_share
        };
        if r.random::<f64>() < p {
            return e.id;
        }
    }
    exits.last().map_or(NO_RAMP, |e| e.id)
}

/// Place the initial vehicles.
pub fn populate(w: &mut World, pop: &Population) {
    if w.road.is_corridor() {
        populate_corridor(w, pop.initial_density);
        return;
    }
    let lanes = w.road.lanes as usize;
    let n = pop.count;
    if n == 0 {
        return;
    }
    let per_lane = n.div_ceil(lanes);
    let spacing = w.road.length / per_lane as f64;
    for k in 0..n {
        let lane = (k % lanes) as u8 + 1;
        let idx = k / lanes;
        let s = w.road.wrap(idx as f64 * spacing + (lane - 1) as f64 * spacing / lanes as f64);
        let v = if pop.initial_speed >= 0.0 {
            pop.initial_speed
        } else {
            let gap = spacing - w.params.human.length;
            w.params.human.idm.equilibrium_speed(gap, w.road.limit.min(w.params.human.idm.v0))
        };
        let id = new_id(w);
        add_vehicle(w, id, s, lane, v, NO_RAMP, NO_RAMP);
    }
}

fn populate_corridor(w: &mut World, density: f64) {
    if density <= 0.0 {
        return;
    }
    let spacing = 1000.0 / density;
    let gap = spacing - w.params.human.length;
    let v = w.params.human.idm.equilibrium_speed(gap, w.road.limit);
    let c = w.road.corridor;
    let starts = [0.0, c + w.road.loop_len];
    for lane in 1..=w.road.lanes {
        for &start in &starts {
            let mut x = 400.0 + spacing * (lane as f64 - 1.0) / w.road.lanes as f64;
            while x < c - 600.0 {
                let s = start + x;
                if w.road.lane_exists(s, lane) {
                    let id = new_id(w);
                    let exit = choose_exit(&w.road, w.seed, id, s, &w.demand.cfg);
                    add_vehicle(w, id, s, lane, v, NO_RAMP, exit);
                }
                x += spacing;
            }
        }
    }
}

/// Step 1a: Poisson arrivals into ramp queues (one draw per ramp per tick,
/// independent of world state).
pub fn arrivals(w: &mut World) {
    let t = w.time;
    for k in 0..w.demand.ramps.len() {
        let p = w.demand.rate(k, t) * w.dt / 3600.0;
        let u: f64 = w.demand.ramps[k].rng.random();
        if u < p {
            let id = new_id(w);
            let r = &mut w.demand.ramps[k];
            r.queue.push_back((id, t));
            r.arrived += 1;
        }
    }
}

/// Step 1b: move queued vehicles onto the road where there is room.
/// Returns true if any vehicle entered (lane index needs a rebuild).
pub fn enter(w: &mut World) -> bool {
    let mut any = false;
    for k in 0..w.demand.ramps.len() {
        let Some(&(id, _)) = w.demand.ramps[k].queue.front() else { continue };
        let ramp = w.road.ramps[w.demand.ramps[k].ramp as usize].clone();
        let s = ramp.zone_start;
        let lanes: Vec<u8> = if ramp.all_lanes { (1..=w.road.lanes).filter(|&l| w.road.lane_exists(s, l)).collect() } else { vec![0] };
        let v_max = 0.8 * w.road.limit_at(s);
        let best = lanes.iter().filter_map(|&lane| entry_speed(w, lane, s, v_max).map(|v| (lane, v))).max_by(|a, b| a.1.total_cmp(&b.1));
        if let Some((lane, v)) = best {
            w.demand.ramps[k].queue.pop_front();
            w.demand.ramps[k].entered += 1;
            let exit = choose_exit(&w.road, w.seed, id, s, &w.demand.cfg);
            add_vehicle(w, id, s, lane, v, ramp.id, exit);
            w.veh.reindex();
            any = true;
        }
    }
    any
}

/// Speed at which a vehicle can enter `lane` at `s`, or None if there is
/// no room. The vehicle enters at (about) the leader's speed, and only when
/// the gap ahead fits that speed with half the desired time gap and the
/// vehicle behind would not have to brake hard.
fn entry_speed(w: &World, lane: u8, s: f64, v_max: f64) -> Option<f64> {
    let p = &w.params.human.idm;
    let len = w.params.human.length;
    let (lead, fol) = w.veh.around(lane, s, usize::MAX);
    let mut v = v_max;
    if let Some(l) = lead {
        let gap = w.road.dist(s, w.veh.s[l]) - w.veh.len[l];
        if gap < 300.0 {
            v = v.min(w.veh.v[l] + 1.0);
            if gap < p.s0 + 0.5 * p.t * v + 2.0 {
                return None;
            }
        }
    }
    if let Some(f) = fol {
        let gap = w.road.dist(w.veh.s[f], s) - len;
        if gap < p.s0 + 1.0 {
            return None;
        }
        let ag = &w.veh.agent[f];
        let acc = ag.driver.idm.interaction(w.veh.v[f], gap, w.veh.v[f] - v);
        if gap < 500.0 && acc < -w.params.mobil.b_safe * 0.5 {
            return None;
        }
    }
    Some(v.max(0.0))
}

/// Step 6b: remove vehicles that crossed their exit's gore in an exit lane;
/// vehicles that missed their exit are re-routed to the next one.
pub fn exits(w: &mut World) -> bool {
    let mut removed = false;
    let mut i = 0;
    while i < w.veh.len() {
        let ex = w.veh.agent[i].exit;
        let crashed = w.veh.agent[i].crashed();
        let gore = w.road.gore(ex);
        let crossed = gore.is_some_and(|g| crossed(&w.road, w.veh.agent[i].s_prev, w.veh.s[i], g));
        let towed = crashed && w.veh.agent[i].crashed_until <= w.time;
        if towed || (crossed && (w.veh.lane[i] == 0 || w.road.ramps[ex as usize].all_lanes)) {
            if !towed {
                w.demand.exited[ex as usize] += 1;
                w.on_exit(i, ex);
            }
            w.veh.remove(i);
            w.stats.removed += 1;
            removed = true;
            continue;
        }
        if crossed {
            // Missed it: take the next exit downstream.
            let s = w.veh.s[i];
            let next = w.road.exits_downstream(s).into_iter().find(|r| r.id != ex && r.kind == RampKind::Off).map(|r| r.id);
            w.veh.agent[i].exit = next.unwrap_or(NO_RAMP);
        }
        i += 1;
    }
    removed
}

/// Did a move from `a` to `b` (forward, less than half a ring) pass `x`?
pub fn crossed(road: &Road, a: f64, b: f64, x: f64) -> bool {
    let total = road.dist(a, b);
    let dx = road.dist(a, x);
    total < road.length * 0.5 && dx > 0.0 && dx <= total
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn profile_interpolates_and_wraps() {
        let p = [[6.0, 1.0], [8.0, 3.0], [20.0, 1.0]];
        assert_eq!(profile_at(&p, 7.0), 2.0);
        assert_eq!(profile_at(&p, 8.0), 3.0);
        assert!((profile_at(&p, 2.0) - 1.0).abs() < 1e-9);
        assert_eq!(profile_at(&[], 5.0), 1.0);
    }
}
