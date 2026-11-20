//! Intelligent Driver Model (longitudinal control).
//!
//! dv/dt = a [1 - (v/v0)^δ - (s*/s)^2],  s* = s0 + vT + vΔv / (2√(ab)).
//! Above the desired speed (e.g. entering a lower speed limit) the free term
//! uses the IIDM form so the vehicle slows down at about b, not violently.

use serde::{Deserialize, Serialize};

/// Hard physical braking limit, m/s².
pub const B_MAX: f64 = 9.0;
/// Gaps beyond this are treated as free road, m.
pub const FREE_GAP: f64 = 1000.0;

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Idm {
    /// Desired speed, m/s.
    pub v0: f64,
    /// Desired time gap, s.
    pub t: f64,
    /// Jam distance, m.
    pub s0: f64,
    /// Max acceleration, m/s².
    pub a: f64,
    /// Comfortable deceleration, m/s².
    pub b: f64,
    /// Acceleration exponent.
    pub delta: f64,
}

impl Default for Idm {
    fn default() -> Self {
        Idm { v0: 120.0 / 3.6, t: 1.5, s0: 2.0, a: 1.4, b: 2.0, delta: 4.0 }
    }
}

impl Idm {
    /// Free-road term for own speed `v` and desired speed `v0`.
    pub fn free(&self, v: f64, v0: f64) -> f64 {
        let v0 = v0.max(0.1);
        if v <= v0 {
            self.a * (1.0 - (v / v0).powf(self.delta))
        } else {
            -self.b * (1.0 - (v0 / v).powf(self.a * self.delta / self.b))
        }
    }

    /// Desired gap s* for own speed `v` and closing speed `dv = v - v_leader`.
    pub fn s_star(&self, v: f64, dv: f64) -> f64 {
        self.s0 + (v * self.t + v * dv / (2.0 * (self.a * self.b).sqrt())).max(0.0)
    }

    /// Interaction (braking) term for a leader at bumper-to-bumper `gap`.
    pub fn interaction(&self, v: f64, gap: f64, dv: f64) -> f64 {
        if gap >= FREE_GAP {
            return 0.0;
        }
        let r = self.s_star(v, dv) / gap.max(0.05);
        -self.a * r * r
    }

    /// Full IDM acceleration, clamped to physical limits.
    pub fn accel(&self, v: f64, v0: f64, gap: f64, dv: f64) -> f64 {
        (self.free(v, v0) + self.interaction(v, gap, dv)).clamp(-B_MAX, self.a)
    }

    /// Equilibrium speed for a given gap (bisection), used for initial states.
    pub fn equilibrium_speed(&self, gap: f64, v0: f64) -> f64 {
        let (mut lo, mut hi) = (0.0, v0);
        for _ in 0..60 {
            let mid = 0.5 * (lo + hi);
            if self.free(mid, v0) + self.interaction(mid, gap, 0.0) > 0.0 {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        0.5 * (lo + hi)
    }
}

/// Ballistic update with speed clamped at zero.
/// Returns (distance travelled, new speed).
pub fn ballistic(v: f64, a: f64, dt: f64) -> (f64, f64) {
    let v_new = v + a * dt;
    if v_new >= 0.0 {
        (v * dt + 0.5 * a * dt * dt, v_new)
    } else {
        // Stops inside the step: travel only until standstill.
        (if a < 0.0 { -0.5 * v * v / a } else { 0.0 }, 0.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn free_road_accelerates_toward_v0() {
        let m = Idm::default();
        assert!((m.accel(0.0, m.v0, FREE_GAP, 0.0) - m.a).abs() < 1e-9);
        assert!(m.accel(m.v0, m.v0, FREE_GAP, 0.0).abs() < 1e-9);
        assert!(m.accel(m.v0 * 1.5, m.v0, FREE_GAP, 0.0) < 0.0);
        assert!(m.accel(m.v0 * 1.5, m.v0, FREE_GAP, 0.0) > -m.b);
    }

    #[test]
    fn equilibrium_gap_gives_zero_accel() {
        let m = Idm::default();
        let v = m.equilibrium_speed(40.0, m.v0);
        assert!(m.accel(v, m.v0, 40.0, 0.0).abs() < 1e-6);
    }

    #[test]
    fn ballistic_never_goes_negative() {
        let (dx, v) = ballistic(1.0, -9.0, 0.5);
        assert_eq!(v, 0.0);
        assert!((dx - 1.0 / 18.0).abs() < 1e-12);
    }
}
