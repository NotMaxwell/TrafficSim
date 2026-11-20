//! Human Driver Model (Treiber, Kesting & Helbing 2006): IDM on delayed,
//! noisy, partial inputs.
//!
//! Each tick a human perceives up to three leaders within range (a truck
//! blocks the view past it), with Wiener-process errors on gap and closing
//! speed, and pushes the percept into a ring buffer. The controller reads
//! the percept from t − τ (linear interpolation between ticks). During a
//! distraction the percept is frozen.

use crate::step::lane_end;
use crate::vehicles::Kind;
use crate::World;

/// Delay buffer length: 2.5 s at dt = 0.1, plus two for interpolation.
pub const BUF: usize = 27;

/// What a human perceived in one tick. `gap[j]` is the per-vehicle gap to
/// leader j+1, i.e. (bumper gap − intermediate lengths) / (j+1), so the
/// equilibrium of the multi-leader model equals the single-leader one.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Percept {
    pub gap: [f32; 3],
    pub dv: [f32; 3],
    pub n: u8,
    pub v: f32,
}

#[derive(Clone, Debug, PartialEq)]
pub struct HumanState {
    pub buf: [Percept; BUF],
    pub head: u8,
    /// Valid entries; 0 means "reset" (fill the whole buffer on next push).
    pub filled: u8,
    /// Wiener processes for gap and closing-speed estimation errors.
    pub w_gap: f64,
    pub w_rate: f64,
    pub distracted_until: f64,
}

impl Default for HumanState {
    fn default() -> Self {
        HumanState { buf: [Percept::default(); BUF], head: 0, filled: 0, w_gap: 0.0, w_rate: 0.0, distracted_until: -1.0 }
    }
}

impl HumanState {
    pub fn push(&mut self, p: Percept) {
        if self.filled == 0 {
            self.buf = [p; BUF];
            self.filled = BUF as u8;
            return;
        }
        self.head = ((self.head as usize + 1) % BUF) as u8;
        self.buf[self.head as usize] = p;
    }

    pub fn latest(&self) -> Percept {
        self.buf[self.head as usize]
    }

    /// Percept from `ticks` ago (fractional, linearly interpolated).
    pub fn delayed(&self, ticks: f64) -> Percept {
        let ticks = ticks.clamp(0.0, (BUF - 2) as f64);
        let k = ticks.floor() as usize;
        let f = (ticks - k as f64) as f32;
        let at = |k: usize| self.buf[(self.head as usize + BUF - k) % BUF];
        let (a, b) = (at(k), at(k + 1));
        if f == 0.0 || a.n != b.n {
            return a;
        }
        let mut p = a;
        for j in 0..a.n as usize {
            p.gap[j] = a.gap[j] + (b.gap[j] - a.gap[j]) * f;
            p.dv[j] = a.dv[j] + (b.dv[j] - a.dv[j]) * f;
        }
        p.v = a.v + (b.v - a.v) * f;
        p
    }
}

/// True (noise-free) percept of vehicle `i`.
fn observe(w: &World, i: usize) -> Percept {
    let h = &w.params.human;
    let s = w.veh.s[i];
    let v = w.veh.v[i];
    let mut p = Percept { v: v as f32, ..Default::default() };
    let n_max = w.veh.agent[i].driver.anticipation.clamp(1, 3) as usize;
    let end = lane_end(w, i, w.veh.lane[i], s);
    let mut lengths = 0.0;
    for j in 0..n_max {
        let Some(l) = w.veh.nth_leader(i, j + 1) else { break };
        if l == i {
            break;
        }
        let dist = w.road.dist(s, w.veh.s[l]);
        let gap = dist - w.veh.len[l];
        if dist > h.perception_range || end.is_some_and(|d| d < gap) {
            break;
        }
        p.gap[j] = ((gap - lengths) / (j + 1) as f64) as f32;
        p.dv[j] = (v - w.veh.v[l]) as f32;
        p.n = j as u8 + 1;
        lengths += w.veh.len[l];
        if w.veh.agent[l].truck {
            break; // occlusion: nothing visible past a truck
        }
    }
    if let Some(d) = end
        && (p.n == 0 || (d as f32) < p.gap[0])
        && d < h.perception_range
    {
        p = Percept { gap: [d as f32, 0.0, 0.0], dv: [v as f32, 0.0, 0.0], n: 1, v: v as f32 };
    }
    p
}

/// Step 3: every human perceives and stores into the delay buffer.
pub fn perceive(w: &mut World) {
    let h = w.params.human.clone();
    let dt = w.dt;
    let decay = (-dt / h.error_corr_time.max(dt)).exp();
    let kick = (1.0 - decay * decay).sqrt();
    for i in 0..w.veh.len() {
        if w.veh.kind[i] != Kind::Human || w.veh.agent[i].crashed() {
            continue;
        }
        let truth = observe(w, i);
        let t = w.time;
        let ag = &mut w.veh.agent[i];
        let hs = &mut ag.human;
        if h.distraction {
            // Drivers look away only when traffic ahead is calm: nobody
            // closer than 2 s, not closing in. Peripheral vision ends a
            // distraction when the leader looms (true time-to-collision < 3 s).
            let calm = truth.n == 0 || (truth.gap[0] as f64 > 10.0 + 2.0 * truth.v as f64 && truth.dv[0] < 1.0);
            let looming = truth.n > 0 && truth.dv[0] > 0.0 && (truth.gap[0] / truth.dv[0]) < 3.0;
            if t >= hs.distracted_until && calm && ag.rng.uniform() < dt / h.distraction_interval {
                hs.distracted_until = t + h.distraction_min + (h.distraction_max - h.distraction_min) * ag.rng.uniform();
            } else if t < hs.distracted_until && looming {
                hs.distracted_until = t;
            }
            if t < hs.distracted_until && hs.filled > 0 {
                let mut p = hs.latest();
                p.v = truth.v; // own speed is still felt
                hs.push(p);
                continue;
            }
        }
        let mut p = truth;
        if h.estimation {
            hs.w_gap = decay * hs.w_gap + kick * ag.rng.normal();
            hs.w_rate = decay * hs.w_rate + kick * ag.rng.normal();
            for j in 0..p.n as usize {
                let g = p.gap[j] as f64;
                p.gap[j] = (g * (h.gap_error * hs.w_gap).exp()) as f32;
                p.dv[j] = (p.dv[j] as f64 - h.rate_error * g * hs.w_rate) as f32;
            }
        }
        hs.push(p);
    }
}

/// Step 5 (humans): IDM with multi-anticipation on the delayed percept.
///
/// Temporal anticipation (HDM): drivers know their own current speed and
/// extrapolate each delayed gap over their reaction time assuming constant
/// speeds, s ≈ s(t−τ) − τ·Δv(t−τ). They still react to changes only τ late.
pub fn accel(w: &World, i: usize, v0: f64) -> f64 {
    let ag = &w.veh.agent[i];
    let d = &ag.driver;
    let mut p = if d.tau > 0.0 { ag.human.delayed(d.tau / w.dt) } else { ag.human.latest() };
    let v = if d.tau > 0.0 && w.params.human.temporal_anticipation {
        for j in 0..p.n as usize {
            p.gap[j] = (p.gap[j] - (d.tau as f32) * p.dv[j] / (j as f32 + 1.0)).max(0.1);
        }
        w.veh.v[i]
    } else {
        p.v as f64
    };
    let n = p.n as usize;
    let mut a = d.idm.free(v, v0);
    if n > 0 {
        // Weights ∝ 1/j², normalised: nearer leaders matter most.
        let norm: f64 = (1..=n).map(|j| 1.0 / (j * j) as f64).sum();
        for j in 0..n {
            let c = 1.0 / ((j + 1) * (j + 1)) as f64 / norm;
            a += c * d.idm.interaction(v, p.gap[j] as f64, p.dv[j] as f64);
        }
    }
    a.clamp(-crate::idm::B_MAX, d.idm.a)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn delayed_reads_history_with_interpolation() {
        let mut h = HumanState::default();
        for k in 0..30 {
            h.push(Percept { v: k as f32, n: 0, ..Default::default() });
        }
        assert_eq!(h.delayed(0.0).v, 29.0);
        assert_eq!(h.delayed(5.0).v, 24.0);
        assert!((h.delayed(5.5).v - 23.5).abs() < 1e-6);
    }
}
