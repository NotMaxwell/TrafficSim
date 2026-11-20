//! Shared helpers for milestone gate tests.
#![allow(dead_code)]

use traffic_core::{Scenario, World};

pub fn scenario(name: &str) -> Scenario {
    let path = format!("{}/../../scenarios/{name}.toml", env!("CARGO_MANIFEST_DIR"));
    let text = std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    Scenario::from_toml(&text).unwrap_or_else(|e| panic!("{path}: {e}"))
}

pub fn std_dev(xs: &[f64]) -> f64 {
    let n = xs.len() as f64;
    let m = xs.iter().sum::<f64>() / n;
    (xs.iter().map(|x| (x - m).powi(2)).sum::<f64>() / n).sqrt()
}

/// Position of the slowest region on a single-lane ring: mean speed per
/// 10 m cell, smoothed over neighbours, argmin.
pub fn jam_location(w: &World) -> f64 {
    let cell = 10.0;
    let n = (w.road.length / cell).ceil() as usize;
    let mut sum = vec![0.0; n];
    let mut cnt = vec![0.0; n];
    for i in 0..w.veh.len() {
        let c = ((w.veh.s[i] / cell) as usize).min(n - 1);
        sum[c] += w.veh.v[i];
        cnt[c] += 1.0;
    }
    // Fill empty cells (free road) with a high speed, then smooth.
    let raw: Vec<f64> = (0..n).map(|c| if cnt[c] > 0.0 { sum[c] / cnt[c] } else { 99.0 }).collect();
    let sm: Vec<f64> = (0..n).map(|c| (raw[(c + n - 1) % n] + 2.0 * raw[c] + raw[(c + 1) % n]) / 4.0).collect();
    let k = (0..n).min_by(|&a, &b| sm[a].total_cmp(&sm[b])).unwrap();
    (k as f64 + 0.5) * cell
}

/// Mean velocity of the jam over `seconds`, m/s (negative = upstream).
pub fn jam_velocity(w: &mut World, seconds: f64) -> f64 {
    let mut x = jam_location(w);
    let mut travelled = 0.0;
    let steps = seconds as usize;
    for _ in 0..steps {
        w.run_for(1.0);
        let nx = jam_location(w);
        let mut d = nx - x;
        let l = w.road.length;
        if d > l / 2.0 {
            d -= l;
        } else if d < -l / 2.0 {
            d += l;
        }
        travelled += d;
        x = nx;
    }
    travelled / seconds
}
