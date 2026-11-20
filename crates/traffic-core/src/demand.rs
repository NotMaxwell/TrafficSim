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

use crate::idm::Idm;
use crate::rng::{self, Purpose};
use crate::road::{RampKind, Road};
use crate::scenario::Population;
pub use crate::spawn::{add_vehicle, make_agent};
use crate::vehicles::NO_RAMP;
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
    /// Eastbound entry (uses profile A); westbound uses profile B.
    pub eastbound: bool,
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
                let default = match (r.city, r.mile == 0.0) {
                    (true, true) => cfg.city_a_vph,
                    (true, false) => cfg.city_b_vph,
                    _ => cfg.interchange_vph,
                };
                RampState {
                    ramp: r.id,
                    base_vph: r.vph.unwrap_or(default),
                    eastbound: r.dir == crate::road::Dir::East,
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
        let prof = if r.eastbound { &self.cfg.profile_a } else { &self.cfg.profile_b };
        (r.base_vph * profile_at(prof, hour) * self.cfg.multiplier + r.surge_vph).max(0.0)
    }

    pub fn queued(&self) -> usize {
        self.ramps.iter().map(|r| r.queue.len()).sum()
    }
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
            e.exit_share.unwrap_or(cfg.interchange_exit_share)
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
        let exit = choose_exit(&w.road, w.seed, id, s, &w.demand.cfg);
        let (_, agent, len) = make_agent(w, id, ramp.id, exit);
        let best = lanes
            .iter()
            .filter_map(|&lane| entry_speed(w, lane, s, v_max, len, &agent.driver.idm).map(|v| (lane, v)))
            .max_by(|a, b| a.1.total_cmp(&b.1));
        if let Some((lane, v)) = best {
            w.demand.ramps[k].queue.pop_front();
            w.demand.ramps[k].entered += 1;
            add_vehicle(w, id, s, lane, v, ramp.id, exit);
            w.veh.reindex();
            any = true;
        }
    }
    any
}

/// Speed at which a vehicle of length `len` can enter `lane` at `s`, or
/// None if there is no room. It enters at (about) the leader's speed, only
/// when the gap ahead fits that speed with half its desired time gap, and
/// only if the vehicle behind keeps a short time gap and would not have to
/// brake hard even reacting ~1.2 s late.
fn entry_speed(w: &World, lane: u8, s: f64, v_max: f64, len: f64, idm: &Idm) -> Option<f64> {
    let (lead, fol) = w.veh.around(lane, s, usize::MAX);
    let mut v = v_max;
    if let Some(l) = lead {
        let gap = w.road.dist(s, w.veh.s[l]) - w.veh.len[l];
        if gap < 300.0 {
            v = v.min(w.veh.v[l] + 1.0);
            if gap < idm.s0 + 0.5 * idm.t * v + 2.0 {
                return None;
            }
        }
    }
    if let Some(f) = fol {
        let gap = w.road.dist(w.veh.s[f], s) - len;
        let vf = w.veh.v[f];
        if gap < 500.0 {
            if gap < 2.0 + 0.5 * vf {
                return None;
            }
            let closing = (vf - v).max(0.0);
            let ag = &w.veh.agent[f];
            let acc = ag.driver.idm.interaction(vf, (gap - 1.2 * closing).max(0.1), vf - v);
            if acc < -w.params.mobil.b_safe * 0.5 {
                return None;
            }
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
