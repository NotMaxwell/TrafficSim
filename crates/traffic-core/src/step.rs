//! The tick: one fixed time step of a world, in the order given by the
//! design doc. Sub-steps live in their own modules; this file only
//! sequences them and holds the shared leader query and integrator.

use crate::idm::{self, FREE_GAP};
use crate::auto::AutoState;
use crate::{auto, bus, demand, events, human, metrics, mobil};
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
    lane_end_within(w, slot, lane, s, FREE_GAP)
}

/// Like `lane_end`, looking up to `horizon` metres ahead.
pub fn lane_end_within(w: &World, slot: usize, lane: u8, s: f64, horizon: f64) -> Option<f64> {
    let closed = if w.events.closures.is_empty() { None } else { w.events.closure_ahead(&w.road, s, lane, horizon) };
    let mut d = w.road.lane_end_ahead(s, lane, horizon);
    if lane == 0
        && let Some(r) = w.road.ramp_zone_at(s)
        && r.id == w.veh.agent[slot].exit
    {
        d = None;
    }
    match (d, closed) {
        (Some(a), Some(b)) => Some(a.min(b)),
        (a, b) => a.or(b),
    }
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
    let weather = if w.events.weather.is_empty() { 1.0 } else { w.events.weather_at(&w.road, s).0 };
    weather
        * match w.veh.kind[slot] {
            Kind::Human => d.idm.v0 * limit / w.road.limit,
            Kind::Auto => d.idm.v0.min(limit),
        }
}

/// Advance the world by one tick.
pub fn step(w: &mut World) {
    // 7. Scheduled events start / expire (applied first so they take
    //    effect this tick in both worlds alike).
    events::apply(w);
    // 1. Arrivals and entries from ramp queues.
    demand::arrivals(w);
    demand::enter(w);
    // 4. Lane changes (applied immediately, index kept sorted).
    mobil::lane_changes(w);
    // 3. Perception (after lane changes, so a driver who just changed
    //    lanes looks at the new lane).
    human::perceive(w);
    // 2. Autos publish to the state bus.
    bus::publish(w);
    // 5. Accelerations (auto controller state is applied afterwards so
    //    every vehicle decides on the same snapshot).
    let mut auto_states = Vec::new();
    let acc: Vec<f64> = (0..w.veh.len())
        .map(|i| {
            let (a, st) = accel(w, i);
            if let Some(st) = st {
                auto_states.push((i, st));
            }
            a
        })
        .collect();
    for (i, st) in auto_states {
        w.veh.agent[i].auto = st;
    }
    // 6. Integrate, remove exiting vehicles, re-sort.
    integrate(w, &acc);
    demand::exits(w);
    w.veh.reindex();
    detect_collisions(w);
    // 8. Detectors and the space-time grid.
    metrics::sample(w);
    w.time += w.dt;
    w.tick += 1;
}

/// Acceleration of vehicle `i` for this tick, plus the new controller
/// state for autos.
pub fn accel(w: &World, i: usize) -> (f64, Option<AutoState>) {
    let ag = &w.veh.agent[i];
    if ag.crashed() {
        return (0.0, None);
    }
    let v0 = desired_speed(w, i);
    let mut st = None;
    let mut a = match w.veh.kind[i] {
        Kind::Human => {
            let mut a = human::accel(w, i, v0);
            let sigma = w.params.human.accel_noise;
            if sigma > 0.0 {
                // Deterministic per-vehicle noise; the RNG advances in
                // `integrate`, keeping this function side-effect free.
                let mut r = ag.rng;
                a += sigma * r.normal();
            }
            a
        }
        Kind::Auto => {
            let out = auto::accel(w, i);
            st = Some(out.state);
            out.a
        }
    };
    if let Some(f) = ag.forced
        && w.time < f.until
    {
        // Scheduled hard brake: slow to the target and hold it (never
        // faster than the controller would allow).
        let v = w.veh.v[i];
        let hold = if v > f.target { -f.decel } else { ((f.target - v) / 2.0).min(0.5) };
        a = a.min(hold);
    }
    (a, st)
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
            // Wreckage skids to a stop.
            let (dx, v) = idm::ballistic(w.veh.v[i], -CRASH_DECEL, dt);
            w.veh.s[i] = w.road.wrap(w.veh.s[i] + dx);
            w.veh.v[i] = v;
            w.veh.a[i] = if v > 0.0 { -CRASH_DECEL } else { 0.0 };
            continue;
        }
        if w.params.human.accel_noise > 0.0 && w.veh.kind[i] == Kind::Human {
            ag.rng.normal();
        }
        if ag.forced.is_some_and(|f| w.time >= f.until) {
            ag.forced = None;
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

/// Deceleration of vehicles after a crash, m/s².
pub const CRASH_DECEL: f64 = 8.0;

/// Any overlap between consecutive vehicles in a lane is a crash: both skid
/// to a stop and become a blockage until towed.
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
            w.metrics.crashes.push((w.time, w.veh.s[l]));
            let until = w.time + clear;
            // Momentum is shared on impact; both then skid.
            let v = 0.5 * (w.veh.v[f] + w.veh.v[l]);
            for x in [f, l] {
                w.veh.agent[x].crashed_until = until;
                w.veh.v[x] = v;
            }
            // Separate the bodies so the blockage is well defined.
            w.veh.s[f] = w.road.wrap(w.veh.s[l] - w.veh.len[l] - 0.01);
        }
    }
}
