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
    /// Courtesy target (signalling car ahead in the next lane, already
    /// filtered to what the driver is willing to brake for): gap, Δv.
    /// gap < 0 means none.
    pub yield_gap: f32,
    pub yield_dv: f32,
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
    /// Current direct leader and when it became the leader (a new car
    /// directly ahead = a cut-in).
    pub leader: u64,
    pub leader_since: f64,
}

impl Default for HumanState {
    fn default() -> Self {
        HumanState {
            buf: [Percept::default(); BUF],
            head: 0,
            filled: 0,
            w_gap: 0.0,
            w_rate: 0.0,
            distracted_until: -1.0,
            leader: u64::MAX,
            leader_since: f64::NEG_INFINITY,
        }
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
        if a.yield_gap >= 0.0 && b.yield_gap >= 0.0 {
            p.yield_gap = a.yield_gap + (b.yield_gap - a.yield_gap) * f;
            p.yield_dv = a.yield_dv + (b.yield_dv - a.yield_dv) * f;
        }
        p
    }
}

/// True (noise-free) percept of vehicle `i`.
fn observe(w: &World, i: usize) -> Percept {
    let h = &w.params.human;
    let s = w.veh.s[i];
    let v = w.veh.v[i];
    let mut p = Percept { v: v as f32, yield_gap: -1.0, ..Default::default() };
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
        p = Percept { gap: [d as f32, 0.0, 0.0], dv: [v as f32, 0.0, 0.0], n: 1, v: v as f32, ..p };
    }
    // Courtesy: make room for a signalling car just ahead in the next lane.
    if let Some((gap, vn)) = crate::mobil::yield_target(w, i) {
        p.yield_gap = gap as f32;
        p.yield_dv = (v - vn) as f32;
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
        let leader = w.veh.leader(i).map_or(u64::MAX, |l| w.veh.id[l]);
        let ag = &mut w.veh.agent[i];
        let hs = &mut ag.human;
        if leader != hs.leader {
            hs.leader = leader;
            hs.leader_since = t;
        }
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
            if p.yield_gap >= 0.0 {
                p.yield_gap = (p.yield_gap as f64 * (h.gap_error * hs.w_gap).exp()) as f32;
            }
            for j in 0..p.n as usize {
                let g = p.gap[j] as f64;
                p.gap[j] = (g * (h.gap_error * hs.w_gap).exp()) as f32;
                p.dv[j] = (p.dv[j] as f64 - h.rate_error * g * hs.w_rate) as f32;
            }
        }
        hs.push(p);
    }
}

/// Reaction delay for braking when a car has just cut in ahead at crawling
/// speed, s. The merging car was signalling right beside the driver, so
/// the response is an expected one (Green 2000: 0.70-0.75 s).
pub const PRIMED_TAU: f64 = 0.7;
/// How long after a cut-in the primed response applies, s.
const PRIMED_FOR: f64 = 3.0;
/// Deceleration that counts as a clear braking cue, m/s².
const PRIMED_CUE: f64 = 1.0;
/// Below this speed drivers are fully primed, m/s.
const PRIMED_BELOW: f64 = 8.0;

/// Step 5 (humans): IDM with multi-anticipation on the delayed percept.
///
/// Temporal anticipation (HDM): drivers know their own current speed and
/// extrapolate each delayed gap over their reaction time assuming constant
/// speeds, s ≈ s(t−τ) − τ·Δv(t−τ). They still react to changes only τ late,
/// except right after a car cuts in at crawling speed (primed braking).
pub fn accel(w: &World, i: usize, v0: f64) -> f64 {
    let ag = &w.veh.agent[i];
    let mut drv = ag.driver;
    if !w.events.weather.is_empty() {
        drv.idm.t *= w.events.weather_at(&w.road, w.veh.s[i]).1;
    }
    let a = accel_with_delay(w, i, &drv, drv.tau, v0);
    let v = w.veh.v[i];
    let cut_in = w.time - ag.human.leader_since < PRIMED_FOR;
    if cut_in && drv.tau > PRIMED_TAU && v < PRIMED_BELOW {
        // Only a clear braking cue (brake-light strength) gets the primed
        // response; small creeping changes still take the full delay.
        let primed = accel_with_delay(w, i, &drv, PRIMED_TAU, v0);
        if primed < a && primed < -PRIMED_CUE {
            return a + (primed - a) * (1.0 - v / PRIMED_BELOW);
        }
    }
    a
}

fn accel_with_delay(w: &World, i: usize, d: &crate::vehicles::Driver, tau: f64, v0: f64) -> f64 {
    let ag = &w.veh.agent[i];
    let mut p = if tau > 0.0 { ag.human.delayed(tau / w.dt) } else { ag.human.latest() };
    let v = if tau > 0.0 && w.params.human.temporal_anticipation {
        for j in 0..p.n as usize {
            p.gap[j] = (p.gap[j] - (tau as f32) * p.dv[j] / (j as f32 + 1.0)).max(0.1);
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
    // Courtesy yielding is an extra constraint, at most b_safe braking.
    if p.yield_gap >= 0.0 {
        let ay = d.idm.free(v, v0) + d.idm.interaction(v, p.yield_gap as f64, p.yield_dv as f64);
        a = a.min(ay.max(-w.params.mobil.b_safe));
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
