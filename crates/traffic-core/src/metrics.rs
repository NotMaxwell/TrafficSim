//! Metrics collected inside the engine: virtual loop detectors, a
//! space-time speed grid, per-minute summary rows, trips and crashes.
//! The frontend only draws what these produce. CSV is produced and parsed
//! as strings (no file I/O here).

use std::collections::VecDeque;
use std::sync::Arc;

use crate::road::{Dir, MILE};
use crate::vehicles::NO_RAMP;
use crate::World;

/// Space-time grid cell size.
pub const CELL_M: f64 = 100.0;
pub const CELL_S: f64 = 5.0;
/// Grid history: one hour of 5 s columns.
pub const MAX_COLS: usize = 720;
/// Detector aggregation window, s.
pub const WINDOW_S: f64 = 60.0;

#[derive(Clone, Debug, PartialEq)]
pub struct Detector {
    pub s: f64,
    pub label: String,
    /// Total vehicles counted.
    pub count: u64,
    win_count: u32,
    win_inv_v: f64,
}

/// One detector window: flow per lane, density per lane, space-mean speed.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DetSample {
    pub t: f64,
    pub det: u16,
    pub flow_vphpl: f64,
    pub density_vpkpl: f64,
    pub speed_kmh: f64,
}

/// One summary row per simulated minute.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Row {
    pub t: f64,
    pub vehicles: f64,
    pub mean_speed_kmh: f64,
    /// Vehicles leaving at off-ramps, veh/h.
    pub throughput_vph: f64,
    /// City-to-city travel time over the last 10 min, minutes (NaN if none).
    pub travel_med_min: f64,
    pub travel_p90_min: f64,
    /// Hard-braking onsets (< -3 m/s²) in this minute.
    pub hard_brakes: f64,
    /// Cumulative collisions.
    pub collisions: f64,
    pub queued: f64,
    /// Positive work per vehicle-km (relative fuel proxy), m²/s² per km.
    pub fuel_proxy: f64,
}

pub const ROW_FIELDS: [&str; 10] = [
    "t_s",
    "vehicles",
    "mean_speed_kmh",
    "throughput_vph",
    "travel_med_min",
    "travel_p90_min",
    "hard_brakes",
    "collisions",
    "queued",
    "fuel_proxy",
];

impl Row {
    fn values(&self) -> [f64; 10] {
        [
            self.t,
            self.vehicles,
            self.mean_speed_kmh,
            self.throughput_vph,
            self.travel_med_min,
            self.travel_p90_min,
            self.hard_brakes,
            self.collisions,
            self.queued,
            self.fuel_proxy,
        ]
    }

    fn from_values(v: &[f64]) -> Row {
        Row {
            t: v[0],
            vehicles: v[1],
            mean_speed_kmh: v[2],
            throughput_vph: v[3],
            travel_med_min: v[4],
            travel_p90_min: v[5],
            hard_brakes: v[6],
            collisions: v[7],
            queued: v[8],
            fuel_proxy: v[9],
        }
    }
}

/// Rows as CSV (f64 printed in shortest round-trip form).
pub fn rows_to_csv(rows: &[Row]) -> String {
    let mut out = ROW_FIELDS.join(",");
    out.push('\n');
    for r in rows {
        let line: Vec<String> = r.values().iter().map(|x| x.to_string()).collect();
        out.push_str(&line.join(","));
        out.push('\n');
    }
    out
}

pub fn rows_from_csv(text: &str) -> Result<Vec<Row>, String> {
    let mut lines = text.lines();
    let header = lines.next().ok_or("empty CSV")?;
    if header.split(',').collect::<Vec<_>>() != ROW_FIELDS {
        return Err(format!("unexpected header: {header}"));
    }
    lines
        .filter(|l| !l.trim().is_empty())
        .map(|l| {
            let v: Result<Vec<f64>, _> = l.split(',').map(|x| x.trim().parse::<f64>()).collect();
            let v = v.map_err(|e| format!("{e} in line: {l}"))?;
            if v.len() != ROW_FIELDS.len() {
                return Err(format!("expected {} fields: {l}", ROW_FIELDS.len()));
            }
            Ok(Row::from_values(&v))
        })
        .collect()
}

pub fn detector_samples_to_csv(m: &Metrics) -> String {
    let mut out = String::from("t_s,detector,position,flow_vphpl,density_vpkpl,speed_kmh\n");
    for d in &m.samples {
        let label = &m.detectors[d.det as usize].label;
        out.push_str(&format!("{},{},{},{},{},{}\n", d.t, d.det, label, d.flow_vphpl, d.density_vpkpl, d.speed_kmh));
    }
    out
}

#[derive(Clone, Debug)]
pub struct Metrics {
    pub detectors: Vec<Detector>,
    pub samples: Vec<DetSample>,
    /// Space-time grid: mean speed (m/s) per 100 m cell, one column per 5 s.
    /// NaN = no vehicle in the cell.
    pub grid: VecDeque<Arc<Vec<f32>>>,
    /// Time at the end of the oldest column.
    pub grid_t0: f64,
    pub grid_version: u64,
    acc_sum: Vec<f32>,
    acc_n: Vec<u16>,
    pub rows: Vec<Row>,
    /// Completed city-to-city trips: (exit time, travel time s).
    pub trips: VecDeque<(f64, f64)>,
    pub trips_total: u64,
    /// Crash locations: (time, s).
    pub crashes: Vec<(f64, f64)>,
    last_exits: u64,
    last_hard: u64,
    last_collisions: u64,
    work: f64,
    dist: f64,
}

impl Metrics {
    /// Empty metrics, replaced by `new` once the world exists.
    pub(crate) fn placeholder() -> Metrics {
        Metrics {
            detectors: Vec::new(),
            samples: Vec::new(),
            grid: VecDeque::new(),
            grid_t0: 0.0,
            grid_version: 0,
            acc_sum: vec![0.0],
            acc_n: vec![0],
            rows: Vec::new(),
            trips: VecDeque::new(),
            trips_total: 0,
            crashes: Vec::new(),
            last_exits: 0,
            last_hard: 0,
            last_collisions: 0,
            work: 0.0,
            dist: 0.0,
        }
    }

    pub fn new(w: &World) -> Metrics {
        let road = &w.road;
        let mut detectors = Vec::new();
        if road.is_corridor() {
            for dir in [Dir::East, Dir::West] {
                let mut m = 0.5;
                while m < road.corridor / MILE {
                    let label = format!("{} mi {m}", if dir == Dir::East { "EB" } else { "WB" });
                    detectors.push(Detector { s: road.from_corridor(dir, m), label, count: 0, win_count: 0, win_inv_v: 0.0 });
                    m += 1.0;
                }
            }
        } else {
            for k in 0..4 {
                let s = road.length * k as f64 / 4.0 + 1.0;
                detectors.push(Detector { s, label: format!("ring {s:.0} m"), count: 0, win_count: 0, win_inv_v: 0.0 });
            }
        }
        detectors.sort_by(|a, b| a.s.total_cmp(&b.s));
        let cells = (road.length / CELL_M).ceil() as usize;
        Metrics {
            detectors,
            samples: Vec::new(),
            grid: VecDeque::new(),
            grid_t0: 0.0,
            grid_version: 0,
            acc_sum: vec![0.0; cells],
            acc_n: vec![0; cells],
            rows: Vec::new(),
            trips: VecDeque::new(),
            trips_total: 0,
            crashes: Vec::new(),
            last_exits: 0,
            last_hard: 0,
            last_collisions: 0,
            work: 0.0,
            dist: 0.0,
        }
    }

    pub fn cells(&self) -> usize {
        self.acc_sum.len()
    }
}

/// Did a forward move from a to b pass x (ring-aware)?
fn passed(len: f64, a: f64, b: f64, x: f64) -> bool {
    let total = (b - a).rem_euclid(len);
    let dx = (x - a).rem_euclid(len);
    total < len * 0.5 && dx > 0.0 && dx <= total
}

/// Step 8: sample detectors and the grid; once a minute, write a row.
pub fn sample(w: &mut World) {
    let len = w.road.length;
    let dt = w.dt;
    let m = &mut w.metrics;
    let nd = m.detectors.len();
    for i in 0..w.veh.len() {
        let (a, b, v) = (w.veh.agent[i].s_prev, w.veh.s[i], w.veh.v[i]);
        // Detectors: first one strictly after a.
        if nd > 0 && a != b {
            let mut k = m.detectors.partition_point(|d| d.s <= a) % nd;
            for _ in 0..nd {
                let d = &mut m.detectors[k];
                if !passed(len, a, b, d.s) {
                    break;
                }
                d.count += 1;
                d.win_count += 1;
                d.win_inv_v += 1.0 / v.max(0.5);
                k = (k + 1) % nd;
            }
        }
        let c = ((b / CELL_M) as usize).min(m.acc_sum.len() - 1);
        m.acc_sum[c] += v as f32;
        m.acc_n[c] += 1;
        let acc = w.veh.a[i];
        if acc > 0.0 {
            m.work += acc * v * dt;
        }
        m.dist += v * dt;
        let ag = &mut w.veh.agent[i];
        let hard = acc < -3.0;
        if hard && !ag.hard_braking {
            w.stats.hard_brakes += 1;
        }
        ag.hard_braking = hard;
    }
    let t = w.time + dt;
    let tick = w.tick + 1;
    let per_col = (CELL_S / dt).round() as u64;
    if tick.is_multiple_of(per_col) {
        let col: Vec<f32> = m.acc_sum.iter().zip(&m.acc_n).map(|(&s, &n)| if n > 0 { s / n as f32 } else { f32::NAN }).collect();
        m.acc_sum.iter_mut().for_each(|x| *x = 0.0);
        m.acc_n.iter_mut().for_each(|x| *x = 0);
        m.grid.push_back(Arc::new(col));
        if m.grid.len() > MAX_COLS {
            m.grid.pop_front();
        }
        m.grid_t0 = t - CELL_S * (m.grid.len() as f64 - 1.0);
        m.grid_version += 1;
    }
    let per_win = (WINDOW_S / dt).round() as u64;
    if tick.is_multiple_of(per_win) {
        minute(w, t);
    }
}

fn minute(w: &mut World, t: f64) {
    let lanes_at = |s: f64| (1..=w.road.lanes).filter(|&l| w.road.lane_exists(s, l)).count().max(1) as f64;
    let lanes: Vec<f64> = w.metrics.detectors.iter().map(|d| lanes_at(d.s)).collect();
    let m = &mut w.metrics;
    for (k, d) in m.detectors.iter_mut().enumerate() {
        let n = d.win_count as f64;
        let flow = n * 3600.0 / WINDOW_S / lanes[k];
        let speed = if n > 0.0 { n / d.win_inv_v * 3.6 } else { f64::NAN };
        let density = if n > 0.0 { flow / speed } else { 0.0 };
        m.samples.push(DetSample { t, det: k as u16, flow_vphpl: flow, density_vpkpl: density, speed_kmh: speed });
        d.win_count = 0;
        d.win_inv_v = 0.0;
    }
    let exits: u64 = w.demand.exited.iter().sum();
    while m.trips.front().is_some_and(|&(te, _)| te < t - 600.0) {
        m.trips.pop_front();
    }
    let mut tt: Vec<f64> = m.trips.iter().map(|&(_, x)| x / 60.0).collect();
    tt.sort_by(|a, b| a.total_cmp(b));
    let q = |p: f64| if tt.is_empty() { f64::NAN } else { tt[((tt.len() - 1) as f64 * p).round() as usize] };
    let n = w.veh.len();
    m.rows.push(Row {
        t,
        vehicles: n as f64,
        mean_speed_kmh: if n > 0 { w.veh.v.iter().sum::<f64>() / n as f64 * 3.6 } else { 0.0 },
        throughput_vph: (exits - m.last_exits) as f64 * 3600.0 / WINDOW_S,
        travel_med_min: q(0.5),
        travel_p90_min: q(0.9),
        hard_brakes: (w.stats.hard_brakes - m.last_hard) as f64,
        collisions: w.stats.collisions as f64,
        queued: w.demand.queued() as f64,
        fuel_proxy: if m.dist > 0.0 { m.work / (m.dist / 1000.0) } else { 0.0 },
    });
    m.last_exits = exits;
    m.last_hard = w.stats.hard_brakes;
    m.last_collisions = w.stats.collisions;
}

/// Record a completed trip when a vehicle leaves through `ramp`.
pub fn on_exit(w: &mut World, slot: usize, ramp: u16) {
    let ag = &w.veh.agent[slot];
    if ag.origin == NO_RAMP {
        return;
    }
    let (o, e) = (&w.road.ramps[ag.origin as usize], &w.road.ramps[ramp as usize]);
    if o.city && e.city && o.mile != e.mile {
        let tt = w.time - ag.spawn_t;
        w.metrics.trips.push_back((w.time, tt));
        w.metrics.trips_total += 1;
    }
}
