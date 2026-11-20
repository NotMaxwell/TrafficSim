//! Scheduled and injected events. Events live in a schedule shared by both
//! worlds of the split view, so they hit both at the same time and place.

use serde::{Deserialize, Serialize};

use crate::bus::Outage;
use crate::road::{Dir, MILE, Road};
use crate::vehicles::Forced;
use crate::World;

/// Where an event happens: a corridor direction + mile marker, or a raw
/// ring coordinate s (validation rings).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(untagged)]
pub enum Pos {
    Corridor { dir: Dir, mile: f64 },
    Ring { s: f64 },
}

impl Pos {
    pub fn s(&self, road: &Road) -> f64 {
        match *self {
            Pos::Corridor { dir, mile } if road.is_corridor() => road.from_corridor(dir, mile),
            Pos::Corridor { mile, .. } => road.wrap(mile * MILE),
            Pos::Ring { s } => road.wrap(s),
        }
    }

    /// Ring coordinate `meters` further downstream (or upstream if negative).
    fn span(&self, road: &Road, other_mile: f64) -> (f64, f64) {
        let a = self.s(road);
        let b = match *self {
            Pos::Corridor { dir, .. } => Pos::Corridor { dir, mile: other_mile }.s(road),
            Pos::Ring { s } => road.wrap(s + (other_mile * MILE)),
        };
        // Order along the direction of travel.
        if road.dist(a, b) < road.length * 0.5 { (a, b) } else { (b, a) }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum EventKind {
    /// The vehicle nearest `at` brakes to `target_kmh` and holds it.
    HardBrake { at: Pos, target_kmh: f64, duration_s: f64, decel: f64 },
    /// Vehicles at `at` in `lanes` crash; the lanes are blocked until cleared.
    Crash { at: Pos, lanes: Vec<u8>, clear_s: f64 },
    /// Work zone: `lanes` closed from `at` to `to_mile`.
    LaneClosure { at: Pos, to_mile: f64, lanes: Vec<u8>, duration_s: f64 },
    /// Extra demand on the named on-ramp.
    DemandSurge { ramp: String, extra_vph: f64, duration_s: f64 },
    /// Rain or fog: lower desired speed, longer gaps.
    Weather { at: Pos, to_mile: f64, speed_factor: f64, gap_factor: f64, duration_s: f64 },
    /// V2V bus down on a stretch (or everywhere if `whole_corridor`).
    CommsOutage { at: Pos, to_mile: f64, whole_corridor: bool, duration_s: f64 },
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct EventSpec {
    /// Start time, s.
    pub t: f64,
    #[serde(flatten)]
    pub kind: EventKind,
}

/// Lanes closed on [s0, s1) until `until`.
#[derive(Clone, Debug, PartialEq)]
pub struct Closure {
    pub s0: f64,
    pub s1: f64,
    pub lanes: Vec<u8>,
    pub until: f64,
    pub crash: bool,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WeatherZone {
    pub s0: f64,
    pub s1: f64,
    pub speed_factor: f64,
    pub gap_factor: f64,
    pub until: f64,
}

#[derive(Clone, Debug, Default)]
pub struct Events {
    pub schedule: Vec<EventSpec>,
    next: usize,
    pub closures: Vec<Closure>,
    pub weather: Vec<WeatherZone>,
    surges: Vec<(usize, f64, f64)>,
    /// Started events (time, kind label, s), for markers and analysis.
    pub log: Vec<(f64, String, f64)>,
}

impl Events {
    pub fn new(mut schedule: Vec<EventSpec>) -> Events {
        schedule.sort_by(|a, b| a.t.total_cmp(&b.t));
        Events { schedule, ..Default::default() }
    }

    /// Add an event (kept sorted; events already due fire next tick).
    pub fn add(&mut self, e: EventSpec) {
        let k = self.schedule[self.next..].partition_point(|x| x.t <= e.t) + self.next;
        self.schedule.insert(k, e);
    }

    pub fn in_closure(&self, road: &Road, s: f64, lane: u8) -> bool {
        self.closures.iter().any(|c| c.lanes.contains(&lane) && road.dist(c.s0, s) < road.dist(c.s0, c.s1))
    }

    /// Distance from s to the start of a closure of `lane` ahead, if any.
    pub fn closure_ahead(&self, road: &Road, s: f64, lane: u8, horizon: f64) -> Option<f64> {
        self.closures
            .iter()
            .filter(|c| c.lanes.contains(&lane))
            .map(|c| road.dist(s, c.s0))
            .filter(|&d| d <= horizon)
            .min_by(|a, b| a.total_cmp(b))
    }

    /// Weather multipliers (speed, gap) at s.
    pub fn weather_at(&self, road: &Road, s: f64) -> (f64, f64) {
        self.weather
            .iter()
            .find(|z| road.dist(z.s0, s) < road.dist(z.s0, z.s1))
            .map_or((1.0, 1.0), |z| (z.speed_factor, z.gap_factor))
    }
}

/// Step 7: start due events and expire finished ones.
pub fn apply(w: &mut World) {
    let t = w.time;
    let ev = &mut w.events;
    ev.closures.retain(|c| c.until > t);
    ev.weather.retain(|z| z.until > t);
    let mut k = 0;
    while k < ev.surges.len() {
        let (r, extra, until) = ev.surges[k];
        if until <= t {
            w.demand.ramps[r].surge_vph -= extra;
            ev.surges.swap_remove(k);
        } else {
            k += 1;
        }
    }
    while w.events.next < w.events.schedule.len() && w.events.schedule[w.events.next].t <= t {
        let e = w.events.schedule[w.events.next].kind.clone();
        w.events.next += 1;
        start(w, e);
    }
}

fn start(w: &mut World, e: EventKind) {
    let t = w.time;
    let road = &w.road;
    let label = match &e {
        EventKind::HardBrake { .. } => "hard brake",
        EventKind::Crash { .. } => "crash",
        EventKind::LaneClosure { .. } => "lane closure",
        EventKind::DemandSurge { .. } => "demand surge",
        EventKind::Weather { .. } => "weather",
        EventKind::CommsOutage { .. } => "comms outage",
    };
    let mut where_s = 0.0;
    match e {
        EventKind::HardBrake { at, target_kmh, duration_s, decel } => {
            let s = at.s(road);
            where_s = s;
            let nearest = (0..w.veh.len())
                .filter(|&i| !w.veh.agent[i].crashed() && w.veh.lane[i] > 0)
                .min_by(|&a, &b| w.road.dist(w.veh.s[a], s).min(w.road.dist(s, w.veh.s[a])).total_cmp(&w.road.dist(w.veh.s[b], s).min(w.road.dist(s, w.veh.s[b]))));
            if let Some(i) = nearest {
                w.veh.agent[i].forced = Some(Forced { target: target_kmh / 3.6, decel: decel.max(0.5), until: t + duration_s });
            }
        }
        EventKind::Crash { at, lanes, clear_s } => {
            let s = at.s(road);
            where_s = s;
            let (s0, s1) = (w.road.wrap(s - 10.0), w.road.wrap(s + 30.0));
            // Vehicles in the crash zone are involved.
            let mut involved = 0;
            for i in 0..w.veh.len() {
                if lanes.contains(&w.veh.lane[i]) && w.road.dist(s0, w.veh.s[i]) < 40.0 {
                    w.veh.agent[i].crashed_until = t + clear_s;
                    involved += 1;
                }
            }
            if involved > 0 {
                w.stats.collisions += 1;
            }
            w.metrics.crashes.push((t, s));
            w.events.closures.push(Closure { s0, s1, lanes, until: t + clear_s, crash: true });
        }
        EventKind::LaneClosure { at, to_mile, lanes, duration_s } => {
            let (s0, s1) = at.span(road, to_mile);
            where_s = s0;
            w.events.closures.push(Closure { s0, s1, lanes, until: t + duration_s, crash: false });
        }
        EventKind::DemandSurge { ramp, extra_vph, duration_s } => {
            let found = w.demand.ramps.iter().position(|r| w.road.ramps[r.ramp as usize].name == ramp);
            if let Some(k) = found {
                where_s = w.road.ramps[w.demand.ramps[k].ramp as usize].zone_start;
                w.demand.ramps[k].surge_vph += extra_vph;
                w.events.surges.push((k, extra_vph, t + duration_s));
            }
        }
        EventKind::Weather { at, to_mile, speed_factor, gap_factor, duration_s } => {
            let (s0, s1) = at.span(road, to_mile);
            where_s = s0;
            w.events.weather.push(WeatherZone { s0, s1, speed_factor, gap_factor, until: t + duration_s });
        }
        EventKind::CommsOutage { at, to_mile, whole_corridor, duration_s } => {
            let (s0, s1) = if whole_corridor { (0.0, w.road.length) } else { at.span(road, to_mile) };
            where_s = s0;
            w.bus.outages.push(Outage { s0, s1, until: t + duration_s });
        }
    }
    w.events.log.push((t, label.to_string(), where_s));
}
