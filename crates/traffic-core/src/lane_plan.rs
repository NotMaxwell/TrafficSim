//! Lane planning: how far ahead drivers act on lane ends, the lane the
//! hive mind assigns each auto, and courtesy yielding to signalling cars.

use crate::mobil::idm_acc;
use crate::step::lane_end_within;
use crate::vehicles::{Kind, NO_RAMP};
use crate::World;

/// Lane-drop warning signs are posted about a mile ahead, m.
pub const SIGN_DISTANCE: f64 = 1600.0;

/// How far ahead a driver acts on a lane that ends (lane drop, closure):
/// humans see about 600 m; connected autos know it from the bus up to
/// their look-ahead distance and merge out early while gaps are plentiful.
pub fn lane_end_horizon(w: &World, i: usize) -> f64 {
    let p = &w.params.auto;
    if w.veh.kind[i] == Kind::Auto && p.lookahead > 600.0 && w.bus.up_at(w.veh.s[i]) { p.lookahead } else { 600.0 }
}

/// The lane the hive mind assigns to auto `i` at its position: spread
/// evenly over the lanes present, fixed per vehicle (so it never weaves).
/// Vehicles whose aux-lane exit is close are kept in the right lane.
pub fn assigned_lane(w: &World, i: usize) -> u8 {
    let s = w.veh.s[i];
    let ex = w.veh.agent[i].exit;
    if ex != NO_RAMP {
        let r = &w.road.ramps[ex as usize];
        if !r.all_lanes && w.road.dist(s, r.zone_end) < 2.0 * w.params.mobil.exit_prepare {
            return 1;
        }
    }
    // Lanes that stay open over the look-ahead (closures and lane drops
    // the hive mind already knows about are planned around).
    let h = lane_end_horizon(w, i);
    let n = w.road.lanes_at(s).max(1);
    let open: Vec<u8> = (1..=n).filter(|&l| lane_end_within(w, i, l, s, h).is_none_or(|d| exits_within(w, i, d))).collect();
    let open = if open.is_empty() { (1..=n).collect() } else { open };
    // Trucks keep to the lowest open lane so cars (which never overtake)
    // are not held to truck speed; cars are spread over the open lanes
    // above it, fixed per vehicle.
    if w.veh.agent[i].truck || open.len() == 1 {
        return open[0];
    }
    let cars = &open[1..];
    cars[(crate::rng::mix(w.veh.id[i] ^ 0x5AFE) % cars.len() as u64) as usize]
}

/// Does vehicle `i` leave the road (at a freeway terminal, on any lane)
/// within `d` metres? Then a lane ending beyond that does not matter.
pub fn exits_within(w: &World, i: usize, d: f64) -> bool {
    let ex = w.veh.agent[i].exit;
    ex != NO_RAMP && {
        let r = &w.road.ramps[ex as usize];
        r.all_lanes && w.road.dist(w.veh.s[i], r.zone_end) < d
    }
}

/// Courtesy: a vehicle signalling into `slot`'s lane just ahead of it, if
/// yielding to it needs no more than comfortable braking. Returns
/// (gap, speed) of that vehicle.
pub fn yield_target(w: &World, slot: usize) -> Option<(f64, f64)> {
    let lane = w.veh.lane[slot];
    let s = w.veh.s[slot];
    let mut best: Option<(f64, f64)> = None;
    for adj in [lane as i16 - 1, lane as i16 + 1] {
        if !(0..=w.road.lanes as i16).contains(&adj) {
            continue;
        }
        let (lead, _) = w.veh.around(adj as u8, s, slot);
        let Some(n) = lead else { continue };
        if w.veh.lane[n] as i16 + w.veh.agent[n].signal as i16 != lane as i16 {
            continue;
        }
        let gap = w.road.dist(s, w.veh.s[n]) - w.veh.len[n];
        if !(0.0..=YIELD_RANGE).contains(&gap) {
            continue;
        }
        // Zipper courtesy: drivers brake harder (up to b_safe) for a car
        // that must merge (its lane ends) than for a discretionary change.
        let limit = if w.veh.agent[n].signal_forced { w.params.mobil.b_safe } else { w.veh.agent[slot].driver.idm.b };
        if idm_acc(w, slot, gap, w.veh.v[n]) < -limit {
            continue;
        }
        if best.is_none_or(|(g, _)| gap < g) {
            best = Some((gap, w.veh.v[n]));
        }
    }
    best
}

/// How far ahead a signalling vehicle is noticed for yielding, m.
const YIELD_RANGE: f64 = 50.0;
