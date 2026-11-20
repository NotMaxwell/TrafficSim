//! The tick: one fixed time step of a world, in the order given by the
//! design doc. Sub-steps live in their own modules; this file only
//! sequences them and holds the shared leader query and integrator.

use crate::idm::{self, FREE_GAP};
use crate::vehicles::Kind;
use crate::World;

/// What sits ahead of a vehicle in its lane: the nearest of the leader
/// vehicle and the end of the lane (treated as a stopped obstacle).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Lead {
    /// Bumper-to-bumper gap, m (FREE_GAP if nothing in range).
    pub gap: f64,
    pub v: f64,
    pub a: f64,
    /// Leader slot, None for a lane end or free road.
    pub slot: Option<usize>,
}

impl Lead {
    pub const FREE: Lead = Lead { gap: FREE_GAP, v: 0.0, a: 0.0, slot: None };
}

/// Gap from a vehicle at `s` to the rear of vehicle `leader`.
pub fn gap_to(w: &World, s: f64, leader: usize) -> f64 {
    w.road.dist(s, w.veh.s[leader]) - w.veh.len[leader]
}

/// Lane-end obstacle distance for `slot` in `lane` at position `s`.
/// Exiting vehicles in their own off-ramp lane do not see the gore as an
/// obstacle: they leave the road there.
pub fn lane_end(w: &World, slot: usize, lane: u8, s: f64) -> Option<f64> {
    let d = w.road.lane_end_ahead(s, lane, FREE_GAP)?;
    if lane == 0
        && let Some(r) = w.road.ramp_zone_at(s)
        && r.id == w.veh.agent[slot].exit
    {
        return None;
    }
    Some(d)
}

/// The obstacle ahead of `slot` in its current lane.
pub fn lead_of(w: &World, slot: usize) -> Lead {
    let s = w.veh.s[slot];
    let mut lead = match w.veh.leader(slot) {
        Some(l) => {
            let gap = gap_to(w, s, l);
            if gap < FREE_GAP {
                Lead { gap, v: w.veh.v[l], a: w.veh.a[l], slot: Some(l) }
            } else {
                Lead::FREE
            }
        }
        None => Lead::FREE,
    };
    if let Some(d) = lane_end(w, slot, w.veh.lane[slot], s)
        && d < lead.gap
    {
        lead = Lead { gap: d, v: 0.0, a: 0.0, slot: None };
    }
    lead
}

/// Desired speed of `slot` at its current position, honouring the local
/// speed limit (humans scale their personal v0 by the limit ratio).
pub fn desired_speed(w: &World, slot: usize) -> f64 {
    let s = w.veh.s[slot];
    let limit = w.road.limit_at(s);
    let d = &w.veh.agent[slot].driver;
    match w.veh.kind[slot] {
        Kind::Human => d.idm.v0 * limit / w.road.limit,
        Kind::Auto => d.idm.v0.min(limit),
    }
}

/// Advance the world by one tick.
pub fn step(w: &mut World) {
    let n = w.veh.len();
    let mut acc = vec![0.0; n];
    for (i, a) in acc.iter_mut().enumerate() {
        *a = accel_ideal(w, i);
    }
    let sigma = w.params.human.accel_noise;
    if sigma > 0.0 {
        for (i, a) in acc.iter_mut().enumerate() {
            if w.veh.kind[i] == Kind::Human && !w.veh.agent[i].crashed() {
                *a += sigma * w.veh.agent[i].rng.normal();
            }
        }
    }
    integrate(w, &acc);
    w.veh.reindex();
    detect_collisions(w);
    w.time += w.dt;
    w.tick += 1;
}

/// Plain IDM acceleration on true state (no delay, no noise).
pub fn accel_ideal(w: &World, i: usize) -> f64 {
    if w.veh.agent[i].crashed() {
        return 0.0;
    }
    let lead = lead_of(w, i);
    let v = w.veh.v[i];
    w.veh.agent[i].driver.idm.accel(v, desired_speed(w, i), lead.gap, v - lead.v)
}

/// Ballistic update of every vehicle with accelerations `acc`.
pub fn integrate(w: &mut World, acc: &[f64]) {
    let dt = w.dt;
    for (i, &a_cmd) in acc.iter().enumerate() {
        let ag = &mut w.veh.agent[i];
        ag.s_prev = w.veh.s[i];
        if ag.crashed() {
            w.veh.v[i] = 0.0;
            w.veh.a[i] = 0.0;
            continue;
        }
        let a = a_cmd.clamp(-idm::B_MAX, 5.0);
        let (dx, v) = idm::ballistic(w.veh.v[i], a, dt);
        w.veh.s[i] = w.road.wrap(w.veh.s[i] + dx);
        w.veh.v[i] = v;
        w.veh.a[i] = a;
        ag.odo += dx;
        let lat = w.veh.lat[i];
        if lat != 0.0 {
            let step = (dt / w.params.mobil.change_duration) as f32;
            w.veh.lat[i] = if lat.abs() <= step { 0.0 } else { lat - step * lat.signum() };
        }
    }
}

/// Any overlap between consecutive vehicles in a lane is a crash: both stop
/// and become a blockage until towed.
pub fn detect_collisions(w: &mut World) {
    let clear = w.params.human.crash_clear_time;
    for lane in 0..w.veh.lanes.len() {
        let n = w.veh.lanes[lane].len();
        if n < 2 {
            continue;
        }
        for k in 0..n {
            let f = w.veh.lanes[lane][k] as usize;
            let l = w.veh.lanes[lane][(k + 1) % n] as usize;
            if gap_to(w, w.veh.s[f], l) >= 0.0 {
                continue;
            }
            let (fc, lc) = (w.veh.agent[f].crashed(), w.veh.agent[l].crashed());
            if fc && lc {
                continue;
            }
            w.stats.collisions += 1;
            let until = w.time + clear;
            for x in [f, l] {
                w.veh.agent[x].crashed_until = until;
                w.veh.v[x] = 0.0;
                w.veh.a[x] = 0.0;
            }
            // Separate the bodies so the blockage is well defined.
            w.veh.s[f] = w.road.wrap(w.veh.s[l] - w.veh.len[l] - 0.01);
        }
    }
}
