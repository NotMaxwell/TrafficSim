//! Wave analysis on the space-time grid: shockwave speed by
//! cross-correlation of speed series at neighbouring locations, and wave
//! amplification of the latest hard-brake event.

use crate::metrics::{CELL_M, CELL_S, Metrics};
use crate::road::{Dir, Road};

/// Speed series of grid cell `c` over the last `n` columns (NaN -> None).
fn series(m: &Metrics, c: usize, n: usize) -> Vec<Option<f64>> {
    let start = m.grid.len().saturating_sub(n);
    m.grid.iter().skip(start).map(|col| col.get(c).copied().filter(|v| v.is_finite()).map(|v| v as f64)).collect()
}

fn corr(a: &[Option<f64>], b: &[Option<f64>], lag: isize) -> Option<f64> {
    let mut xs = Vec::new();
    for (i, x) in a.iter().enumerate() {
        let j = i as isize + lag;
        if j < 0 || j as usize >= b.len() {
            continue;
        }
        if let (Some(x), Some(y)) = (x, b[j as usize]) {
            xs.push((*x, y));
        }
    }
    if xs.len() < 12 {
        return None;
    }
    let n = xs.len() as f64;
    let (mx, my) = (xs.iter().map(|p| p.0).sum::<f64>() / n, xs.iter().map(|p| p.1).sum::<f64>() / n);
    let (mut sxy, mut sxx, mut syy) = (0.0, 0.0, 0.0);
    for (x, y) in xs {
        sxy += (x - mx) * (y - my);
        sxx += (x - mx).powi(2);
        syy += (y - my).powi(2);
    }
    (sxx > 1.0 && syy > 1.0).then(|| sxy / (sxx * syy).sqrt())
}

/// Shockwave speed (km/h, negative = upstream) in one corridor direction
/// over the last `window_s` seconds, from pairs of cells 1 km apart.
/// None when there are no clear waves.
pub fn shockwave_speed(m: &Metrics, road: &Road, dir: Option<Dir>, window_s: f64) -> Option<f64> {
    let n = (window_s / CELL_S) as usize;
    if m.grid.len() < n / 2 {
        return None;
    }
    let (s0, s1) = match (dir, road.is_corridor()) {
        (Some(Dir::East), true) => (0.0, road.corridor),
        (Some(Dir::West), true) => (road.corridor + road.loop_len, 2.0 * road.corridor + road.loop_len),
        _ => (0.0, road.length),
    };
    let gap_cells = (1000.0 / CELL_M) as usize;
    let max_lag = (240.0 / CELL_S) as isize;
    let mut lags = Vec::new();
    let mut c = (s0 / CELL_M) as usize + 1;
    let end = ((s1 / CELL_M) as usize).saturating_sub(gap_cells + 1);
    while c < end {
        let (up, down) = (series(m, c, n), series(m, c + gap_cells, n));
        // Only locations with real slowdowns.
        let low = |s: &[Option<f64>]| s.iter().flatten().any(|&v| v < 12.0);
        if low(&up) && low(&down) {
            // Upstream sees the wave `lag` columns after downstream.
            let best = (-max_lag..=max_lag).filter_map(|l| corr(&down, &up, l).map(|r| (l, r))).max_by(|a, b| a.1.total_cmp(&b.1));
            if let Some((l, r)) = best
                && r > 0.6
                && l != 0
            {
                lags.push(l as f64);
            }
        }
        c += gap_cells / 2;
    }
    if lags.len() < 3 {
        return None;
    }
    lags.sort_by(|a, b| a.total_cmp(b));
    let lag = lags[lags.len() / 2] * CELL_S;
    Some(-1000.0 / lag * 3.6)
}

/// Amplification of the wave from a disturbance at `s` that started at
/// `t0`: speed dip 2 km upstream divided by the dip at the trigger, over
/// the following `window_s`. Above 1 = growing (unstable).
pub fn amplification(m: &Metrics, road: &Road, s: f64, t0: f64, window_s: f64) -> Option<f64> {
    let col_at = |t: f64| ((t - m.grid_t0) / CELL_S).round() as isize;
    let (k0, k1) = (col_at(t0), col_at(t0 + window_s));
    if k0 < 2 || k1 as usize >= m.grid.len() {
        return None;
    }
    let cell = |s: f64| (road.wrap(s) / CELL_M) as usize;
    let dip = |c: usize| {
        let before: Vec<f64> = (k0 - 12..k0).filter_map(|k| m.grid.get(k as usize)?.get(c).copied()).filter(|v| v.is_finite()).map(|v| v as f64).collect();
        let reference = before.iter().sum::<f64>() / before.len().max(1) as f64;
        let low = (k0..=k1).filter_map(|k| m.grid.get(k as usize)?.get(c).copied()).filter(|v| v.is_finite()).fold(f32::INFINITY, f32::min);
        (reference - low as f64).max(0.0)
    };
    let trig = dip(cell(s));
    (trig > 0.5).then(|| dip(cell(s - 2000.0)) / trig)
}
