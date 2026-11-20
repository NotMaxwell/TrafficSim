//! Parameter sweeps (headless): run one world per auto share in parallel
//! and summarise each run in one row.

use rayon::prelude::*;

use crate::{Scenario, World};

#[derive(Clone, Debug, PartialEq)]
pub struct SweepRow {
    pub auto_share: f64,
    /// Mean vehicles leaving per hour over the run (after warm-up).
    pub throughput_vph: f64,
    pub travel_med_min: f64,
    pub travel_p90_min: f64,
    pub mean_speed_kmh: f64,
    pub hard_brakes_per_h: f64,
    pub collisions: u64,
    pub fuel_proxy: f64,
    pub queued_end: usize,
    pub unsafe_lane_changes: u64,
}

pub const SWEEP_FIELDS: [&str; 10] = [
    "auto_share",
    "throughput_vph",
    "travel_med_min",
    "travel_p90_min",
    "mean_speed_kmh",
    "hard_brakes_per_h",
    "collisions",
    "fuel_proxy",
    "queued_end",
    "unsafe_lane_changes",
];

/// Run `seconds` for each share (in parallel). The first `warmup` seconds
/// are excluded from the averages.
pub fn sweep(sc: &Scenario, shares: &[f64], seconds: f64, warmup: f64) -> Vec<SweepRow> {
    shares
        .par_iter()
        .map(|&share| {
            let mut w = World::new(sc, share);
            w.run_for(seconds);
            summarise(&w, share, warmup)
        })
        .collect()
}

pub fn summarise(w: &World, share: f64, warmup: f64) -> SweepRow {
    let rows: Vec<_> = w.metrics.rows.iter().filter(|r| r.t > warmup).collect();
    let n = rows.len().max(1) as f64;
    let mean = |f: fn(&crate::metrics::Row) -> f64| {
        let v: Vec<f64> = rows.iter().map(|r| f(r)).filter(|x| x.is_finite()).collect();
        if v.is_empty() { f64::NAN } else { v.iter().sum::<f64>() / v.len() as f64 }
    };
    SweepRow {
        auto_share: share,
        throughput_vph: mean(|r| r.throughput_vph),
        travel_med_min: mean(|r| r.travel_med_min),
        travel_p90_min: mean(|r| r.travel_p90_min),
        mean_speed_kmh: mean(|r| r.mean_speed_kmh),
        hard_brakes_per_h: rows.iter().map(|r| r.hard_brakes).sum::<f64>() / n * 60.0,
        collisions: w.stats.collisions,
        fuel_proxy: w.metrics.rows.last().map_or(0.0, |r| r.fuel_proxy),
        queued_end: w.queued(),
        unsafe_lane_changes: w.stats.unsafe_lane_changes,
    }
}

pub fn sweep_to_csv(rows: &[SweepRow]) -> String {
    let mut out = SWEEP_FIELDS.join(",");
    out.push('\n');
    for r in rows {
        out.push_str(&format!(
            "{},{},{},{},{},{},{},{},{},{}\n",
            r.auto_share,
            r.throughput_vph,
            r.travel_med_min,
            r.travel_p90_min,
            r.mean_speed_kmh,
            r.hard_brakes_per_h,
            r.collisions,
            r.fuel_proxy,
            r.queued_end,
            r.unsafe_lane_changes
        ));
    }
    out
}
