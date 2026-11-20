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

pub use crate::corridor::{CorridorGeometry, Interchange, LaneSegment, LimitSegment};

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
    /// On-ramp demand at peak, veh/h (None = scenario default).
    pub vph: Option<f64>,
    /// Off-ramp: share of passing traffic that leaves here (None = default).
    pub exit_share: Option<f64>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct Span {
    start: f64,
    end: f64,
}

/// Sort spans and join overlapping or touching ones.
fn merge_spans(mut v: Vec<Span>) -> Vec<Span> {
    v.sort_by(|a, b| a.start.total_cmp(&b.start));
    let mut out: Vec<Span> = Vec::new();
    for sp in v {
        match out.last_mut() {
            Some(last) if sp.start <= last.end + 1e-6 => last.end = last.end.max(sp.end),
            _ => out.push(sp),
        }
    }
    out
}

#[derive(Clone, Debug, PartialEq)]
pub struct Road {
    pub length: f64,
    /// Number of mainline lanes (highest lane index).
    pub lanes: u8,
    pub ramps: Vec<Ramp>,
    /// Names of the corridor ends (mile 0 and the far end).
    pub city_a: String,
    pub city_b: String,
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
            city_a: "City A".into(),
            city_b: "City B".into(),
            corridor: 0.0,
            loop_len: 0.0,
            full,
            spans: vec![Vec::new(); lanes as usize + 1],
            limits: Vec::new(),
            limit,
        }
    }

    /// The corridor closed into a ring: eastbound, end loop, westbound,
    /// end loop. Lane counts and speed limits can vary by mile; ramps come
    /// from the city ends and the interchange list.
    pub fn corridor(cfg: &CorridorGeometry) -> Road {
        let c = cfg.corridor_mi * MILE;
        let lp = cfg.loop_m;
        let len = 2.0 * (c + lp);
        let wb0 = c + lp;
        let default_lanes = cfg.lanes.clamp(1, 6);
        let max_lanes = cfg.lane_segments.iter().map(|g| g.lanes.clamp(1, 6)).fold(default_lanes, u8::max);
        let loop_lanes = cfg.loop_lanes.clamp(1, max_lanes);
        let z = cfg.ramp_zone_m;
        let mut road = Road::ring(len, max_lanes, cfg.speed_limit_mph * MPH);
        road.corridor = c;
        road.loop_len = lp;
        road.city_a = cfg.city_a.clone();
        road.city_b = cfg.city_b.clone();
        let to_s = |dir: Dir, mile: f64| match dir {
            Dir::East => (mile * MILE).clamp(0.0, c),
            Dir::West => wb0 + (c - (mile * MILE).clamp(0.0, c)),
        };
        // Speed limits: per-mile segments (both directions), then the loops.
        for g in &cfg.limit_segments {
            for dir in [Dir::East, Dir::West] {
                let (a, b) = (to_s(dir, g.from_mi), to_s(dir, g.to_mi));
                road.limits.push((a.min(b), a.max(b), g.mph * MPH));
            }
        }
        road.limits.push((c, c + lp, cfg.loop_limit_mph * MPH));
        road.limits.push((2.0 * c + lp, len, cfg.loop_limit_mph * MPH));

        // Lane counts: default, overridden by segments; loops have loop_lanes.
        let lanes_at_mile = |dir: Dir, mile: f64| {
            cfg.lane_segments
                .iter()
                .rev()
                .find(|g| g.dir.is_none_or(|d| d == dir) && mile >= g.from_mi.min(g.to_mi) && mile < g.from_mi.max(g.to_mi))
                .map_or(default_lanes, |g| g.lanes.clamp(1, 6))
        };
        let mut cuts: Vec<f64> = vec![0.0, cfg.corridor_mi];
        for g in &cfg.lane_segments {
            cuts.extend([g.from_mi, g.to_mi]);
        }
        cuts.retain(|m| (0.0..=cfg.corridor_mi).contains(m));
        cuts.sort_by(|a, b| a.total_cmp(b));
        cuts.dedup();
        for lane in 2..=max_lanes {
            let mut spans = Vec::new();
            for dir in [Dir::East, Dir::West] {
                for w in cuts.windows(2) {
                    if lanes_at_mile(dir, 0.5 * (w[0] + w[1])) >= lane {
                        let (a, b) = (to_s(dir, w[0]), to_s(dir, w[1]));
                        spans.push(Span { start: a.min(b), end: a.max(b) });
                    }
                }
            }
            if lane <= loop_lanes {
                spans.push(Span { start: c, end: c + lp });
                spans.push(Span { start: 2.0 * c + lp, end: len });
            }
            let spans = merge_spans(spans);
            if spans.len() == 1 && spans[0].start <= 0.0 && spans[0].end >= len {
                continue; // everywhere
            }
            road.full[lane as usize] = false;
            road.spans[lane as usize] = spans;
        }

        // Ramps.
        let terminal = cfg.city_terminal;
        let mut add = |kind, name: String, dir, mile, start: f64, city: bool, vph: Option<f64>, exit_share: Option<f64>| {
            let id = road.ramps.len() as u16;
            let all_lanes = city && terminal;
            road.ramps.push(Ramp { id, kind, name, dir, mile, zone_start: start, zone_end: start + z, city, all_lanes, vph, exit_share });
        };
        let mut xs: Vec<Interchange> = cfg.interchanges.clone();
        xs.extend(cfg.interchanges_mi.iter().map(|&m| Interchange { mile: m, name: format!("Mile {m}"), ..Default::default() }));
        xs.sort_by(|a, b| a.mile.total_cmp(&b.mile));
        let off = 100.0;
        let (ca, cb) = (&cfg.city_a, &cfg.city_b);
        add(RampKind::On, format!("{ca} on (EB)"), Dir::East, 0.0, off, true, None, None);
        for x in &xs {
            let p = x.mile * MILE;
            if x.eb_off {
                add(RampKind::Off, format!("{} off (EB)", x.name), Dir::East, x.mile, p - 150.0 - z, false, None, x.exit_share_eb);
            }
            if x.eb_on {
                add(RampKind::On, format!("{} on (EB)", x.name), Dir::East, x.mile, p + 150.0, false, x.on_vph_eb, None);
            }
        }
        add(RampKind::Off, format!("{cb} off (EB)"), Dir::East, cfg.corridor_mi, c - off - z, true, None, None);
        add(RampKind::On, format!("{cb} on (WB)"), Dir::West, cfg.corridor_mi, wb0 + off, true, None, None);
        for x in xs.iter().rev() {
            let p = wb0 + (c - x.mile * MILE);
            if x.wb_off {
                add(RampKind::Off, format!("{} off (WB)", x.name), Dir::West, x.mile, p - 150.0 - z, false, None, x.exit_share_wb);
            }
            if x.wb_on {
                add(RampKind::On, format!("{} on (WB)", x.name), Dir::West, x.mile, p + 150.0, false, x.on_vph_wb, None);
            }
        }
        add(RampKind::Off, format!("{ca} off (WB)"), Dir::West, 0.0, wb0 + c - off - z, true, None, None);
        // Auxiliary lanes; overlapping ramp zones join into one lane.
        let aux: Vec<Span> = road.ramps.iter().filter(|r| !r.all_lanes).map(|r| Span { start: r.zone_start, end: r.zone_end }).collect();
        road.spans[0] = merge_spans(aux);
        road
    }

    /// Mainline lanes present at s.
    pub fn lanes_at(&self, s: f64) -> u8 {
        (1..=self.lanes).filter(|&l| self.lane_exists(s, l)).count() as u8
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
        let spans = &self.spans[l];
        match spans.iter().find(|sp| s >= sp.start && s < sp.end) {
            Some(sp) => {
                let mut d = sp.end - s;
                // A span running to the end of the ring continues at s = 0.
                if sp.end >= self.length - 1e-6
                    && let Some(first) = spans.first().filter(|f| f.start <= 1e-6 && f.end < sp.start)
                {
                    d += first.end;
                }
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
    fn lane_segments_and_wrapping_lane_ends() {
        let g = CorridorGeometry {
            lane_segments: vec![
                LaneSegment { from_mi: 7.0, to_mi: 9.0, lanes: 2, dir: None },
                LaneSegment { from_mi: 19.0, to_mi: 20.0, lanes: 2, dir: None },
            ],
            ..Default::default()
        };
        let r = Road::corridor(&g);
        assert_eq!(r.lanes_at(r.from_corridor(Dir::East, 5.0)), 3);
        assert_eq!(r.lanes_at(r.from_corridor(Dir::East, 8.0)), 2);
        assert_eq!(r.lanes_at(r.from_corridor(Dir::West, 8.0)), 2);
        assert_eq!(r.lanes_at(r.from_corridor(Dir::West, 19.5)), 2);
        // Eastbound lane 3 ends at mile 7.
        let s = r.from_corridor(Dir::East, 6.9);
        let d = r.lane_end_ahead(s, 3, 1000.0).unwrap();
        assert!((d - 0.1 * MILE).abs() < 1.0);
        // Lane 2 continues through the loops and across s = 0.
        let near_end = r.length - 10.0;
        assert_eq!(r.lane_end_ahead(near_end, 2, 1e9), None);
        assert!(!r.lane_exists(near_end, 3), "the end loops have two lanes");
    }

    #[test]
    fn overlapping_ramp_zones_join_into_one_aux_lane() {
        let g = CorridorGeometry {
            interchanges: vec![Interchange { mile: 7.0, name: "A".into(), ..Default::default() }, Interchange { mile: 7.4, name: "B".into(), ..Default::default() }],
            ..Default::default()
        };
        let r = Road::corridor(&g);
        let on_a = r.ramps.iter().find(|x| x.name == "A on (EB)").unwrap();
        let off_b = r.ramps.iter().find(|x| x.name == "B off (EB)").unwrap();
        assert!(on_a.zone_end > off_b.zone_start, "test needs overlapping zones");
        let d = r.lane_end_ahead(on_a.zone_start + 1.0, 0, 1e9).unwrap();
        assert!((d - (off_b.zone_end - on_a.zone_start - 1.0)).abs() < 1e-6);
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
