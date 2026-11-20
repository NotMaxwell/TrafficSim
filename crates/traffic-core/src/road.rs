//! Road geometry: one closed 1D coordinate s ∈ [0, L) with integer lanes.
//!
//! Lane 0 is the auxiliary lane (ramp merge / diverge zones only). Mainline
//! lanes are 1..=lanes, numbered from the right in the direction of travel.
//!
//! Corridor layout (C = corridor length, Lp = end-loop length):
//!   [0, C)            eastbound, City A (mile 0) -> City B (mile 20)
//!   [C, C+Lp)         City B end loop (fewer lanes, lower limit)
//!   [C+Lp, 2C+Lp)     westbound, City B -> City A
//!   [2C+Lp, 2C+2Lp)   City A end loop

use serde::{Deserialize, Serialize};

pub const MILE: f64 = 1609.344;
pub const MPH: f64 = MILE / 3600.0;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum RampKind {
    On,
    Off,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Dir {
    East,
    West,
}

/// An on- or off-ramp with its auxiliary-lane zone [zone_start, zone_end).
/// Off-ramp vehicles are removed at the gore point (zone_end).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Ramp {
    pub id: u16,
    pub kind: RampKind,
    pub name: String,
    pub dir: Dir,
    /// Mile marker (0 = City A, 20 = City B).
    pub mile: f64,
    pub zone_start: f64,
    pub zone_end: f64,
    /// City end ramps (true) vs mid-corridor interchanges (false).
    pub city: bool,
    /// Freeway terminal: vehicles enter / leave on any mainline lane at
    /// `zone_start` (on) or `zone_end` (off) instead of using lane 0.
    pub all_lanes: bool,
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct Span {
    start: f64,
    end: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Road {
    pub length: f64,
    /// Number of mainline lanes (highest lane index).
    pub lanes: u8,
    pub ramps: Vec<Ramp>,
    /// Corridor length C and loop length Lp (0 for a plain ring).
    pub corridor: f64,
    pub loop_len: f64,
    /// Lanes that exist on the whole ring (no ends).
    full: Vec<bool>,
    /// Where each lane exists when not full; sorted, non-overlapping.
    spans: Vec<Vec<Span>>,
    /// Speed limits: (start, end, limit m/s); default `limit` elsewhere.
    limits: Vec<(f64, f64, f64)>,
    pub limit: f64,
}

impl Road {
    /// Single-direction ring (validation scenarios). All lanes everywhere.
    pub fn ring(length: f64, lanes: u8, limit: f64) -> Road {
        let mut full = vec![true; lanes as usize + 1];
        full[0] = false;
        Road {
            length,
            lanes,
            ramps: Vec::new(),
            corridor: 0.0,
            loop_len: 0.0,
            full,
            spans: vec![Vec::new(); lanes as usize + 1],
            limits: Vec::new(),
            limit,
        }
    }

    /// The City A <-> City B corridor closed into a ring.
    pub fn corridor(cfg: &CorridorGeometry) -> Road {
        let c = cfg.corridor_mi * MILE;
        let lp = cfg.loop_m;
        let len = 2.0 * (c + lp);
        let lanes = cfg.lanes.clamp(1, 6);
        let loop_lanes = cfg.loop_lanes.clamp(1, lanes);
        let z = cfg.ramp_zone_m;
        let wb0 = c + lp;
        let mut road = Road::ring(len, lanes, cfg.speed_limit_mph * MPH);
        road.corridor = c;
        road.loop_len = lp;
        road.limits = vec![(c, c + lp, cfg.loop_limit_mph * MPH), (2.0 * c + lp, len, cfg.loop_limit_mph * MPH)];
        for lane in (loop_lanes + 1)..=lanes {
            road.full[lane as usize] = false;
            road.spans[lane as usize] = vec![Span { start: 0.0, end: c }, Span { start: wb0, end: wb0 + c }];
        }
        let terminal = cfg.city_terminal;
        let mut add = |kind, name: String, dir, mile, start: f64, city: bool| {
            let id = road.ramps.len() as u16;
            let all_lanes = city && terminal;
            road.ramps.push(Ramp { id, kind, name, dir, mile, zone_start: start, zone_end: start + z, city, all_lanes });
        };
        let off = 100.0;
        add(RampKind::On, "City A on (EB)".into(), Dir::East, 0.0, off, true);
        for &m in &cfg.interchanges_mi {
            let x = m * MILE;
            add(RampKind::Off, format!("Mile {m} off (EB)"), Dir::East, m, x - 150.0 - z, false);
            add(RampKind::On, format!("Mile {m} on (EB)"), Dir::East, m, x + 150.0, false);
        }
        add(RampKind::Off, "City B off (EB)".into(), Dir::East, cfg.corridor_mi, c - off - z, true);
        add(RampKind::On, "City B on (WB)".into(), Dir::West, cfg.corridor_mi, wb0 + off, true);
        let mut mids: Vec<f64> = cfg.interchanges_mi.clone();
        mids.sort_by(|a, b| b.total_cmp(a));
        for m in mids {
            let x = wb0 + (c - m * MILE);
            add(RampKind::Off, format!("Mile {m} off (WB)"), Dir::West, m, x - 150.0 - z, false);
            add(RampKind::On, format!("Mile {m} on (WB)"), Dir::West, m, x + 150.0, false);
        }
        add(RampKind::Off, "City A off (WB)".into(), Dir::West, 0.0, wb0 + c - off - z, true);
        let mut aux: Vec<Span> =
            road.ramps.iter().filter(|r| !r.all_lanes).map(|r| Span { start: r.zone_start, end: r.zone_end }).collect();
        aux.sort_by(|a, b| a.start.total_cmp(&b.start));
        road.spans[0] = aux;
        road
    }

    pub fn is_corridor(&self) -> bool {
        self.corridor > 0.0
    }

    /// Forward distance from `from` to `to` along the ring, in [0, L).
    pub fn dist(&self, from: f64, to: f64) -> f64 {
        (to - from).rem_euclid(self.length)
    }

    pub fn wrap(&self, s: f64) -> f64 {
        s.rem_euclid(self.length)
    }

    pub fn lane_exists(&self, s: f64, lane: u8) -> bool {
        let l = lane as usize;
        l < self.full.len() && (self.full[l] || self.spans[l].iter().any(|sp| s >= sp.start && s < sp.end))
    }

    /// Distance to the end of `lane` ahead of s, if within `horizon`.
    /// Returns Some(0) when the lane does not exist at s.
    pub fn lane_end_ahead(&self, s: f64, lane: u8, horizon: f64) -> Option<f64> {
        let l = lane as usize;
        if l >= self.full.len() {
            return Some(0.0);
        }
        if self.full[l] {
            return None;
        }
        match self.spans[l].iter().find(|sp| s >= sp.start && s < sp.end) {
            Some(sp) => {
                let d = sp.end - s;
                (d <= horizon).then_some(d)
            }
            None => Some(0.0),
        }
    }

    /// Speed limit at s.
    pub fn limit_at(&self, s: f64) -> f64 {
        self.limits.iter().find(|(a, b, _)| s >= *a && s < *b).map_or(self.limit, |l| l.2)
    }

    /// The ramp whose zone contains s, if any.
    pub fn ramp_zone_at(&self, s: f64) -> Option<&Ramp> {
        self.ramps.iter().find(|r| !r.all_lanes && s >= r.zone_start && s < r.zone_end)
    }

    /// Point where vehicles leave through `ramp` (the gore).
    pub fn gore(&self, ramp: u16) -> Option<f64> {
        self.ramps.get(ramp as usize).map(|r| r.zone_end)
    }

    pub fn on_ramps(&self) -> impl Iterator<Item = &Ramp> {
        self.ramps.iter().filter(|r| r.kind == RampKind::On)
    }

    /// Off-ramps in downstream order starting from position s.
    pub fn exits_downstream(&self, s: f64) -> Vec<&Ramp> {
        let mut v: Vec<&Ramp> = self.ramps.iter().filter(|r| r.kind == RampKind::Off).collect();
        v.sort_by(|a, b| self.dist(s, a.zone_end).total_cmp(&self.dist(s, b.zone_end)));
        v
    }

    /// Map s to (direction, mile marker) for drawing the corridor strip.
    /// Loop sections return None.
    pub fn to_corridor(&self, s: f64) -> Option<(Dir, f64)> {
        if !self.is_corridor() {
            return None;
        }
        let c = self.corridor;
        let wb0 = c + self.loop_len;
        if s < c {
            Some((Dir::East, s / MILE))
        } else if s >= wb0 && s < wb0 + c {
            Some((Dir::West, (c - (s - wb0)) / MILE))
        } else {
            None
        }
    }

    /// Inverse of `to_corridor`.
    pub fn from_corridor(&self, dir: Dir, mile: f64) -> f64 {
        let c = self.corridor;
        let x = (mile * MILE).clamp(0.0, c - 1.0);
        match dir {
            Dir::East => x,
            Dir::West => c + self.loop_len + (c - x),
        }
    }
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
    pub interchanges_mi: Vec<f64>,
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
            city_terminal: true,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn corridor_layout() {
        let r = Road::corridor(&CorridorGeometry::default());
        assert!((r.length - 2.0 * (20.0 * MILE + 500.0)).abs() < 1e-6);
        assert_eq!(r.ramps.len(), 4);
        assert!(r.lane_exists(1000.0, 3));
        assert!(!r.lane_exists(r.corridor + 10.0, 3));
        assert!(r.lane_exists(r.corridor + 10.0, 2));
        assert!(!r.lane_exists(150.0, 0), "city terminals have no aux lane");
        let g = CorridorGeometry { city_terminal: false, ..Default::default() };
        let r2 = Road::corridor(&g);
        assert!(r2.lane_exists(150.0, 0));
        assert!(!r2.lane_exists(5000.0, 0));
        assert_eq!(r.lane_end_ahead(r.corridor - 50.0, 3, 100.0), Some(50.0));
        assert_eq!(r.lane_end_ahead(r.corridor - 50.0, 1, 100.0), None);
        let (d, m) = r.to_corridor(r.from_corridor(Dir::West, 5.0)).unwrap();
        assert_eq!(d, Dir::West);
        assert!((m - 5.0).abs() < 1e-9);
    }

    #[test]
    fn interchanges_add_four_ramps_each() {
        let g = CorridorGeometry { interchanges_mi: vec![7.0, 13.0], ..Default::default() };
        let r = Road::corridor(&g);
        assert_eq!(r.ramps.len(), 12);
        let exits = r.exits_downstream(0.0);
        assert_eq!(exits.len(), 6);
        assert!(exits[0].mile == 7.0 && exits[0].dir == Dir::East);
    }
}
