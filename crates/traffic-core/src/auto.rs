//! Automated vehicle control stack.
//!
//! 1. CACC (Ploeg et al.): h·u̇ = −u + kp·e + kd·ė + kff·u_leader, with
//!    spacing error e = gap − s0 − h·v and ė = Δv − h·a. Feeding forward
//!    the leader's command (from the bus) makes platoons string-stable.
//! 2. ACC fallback (radar only, IDM with a 1.1 s gap) behind humans, when
//!    the leader is not on the bus, or when the bus is down.
//! 3. Wave look-ahead: ease toward slow traffic seen up to 2 km ahead on
//!    the bus and open a buffer gap.
//! 4. Wave dampener (Stern et al. 2018) when following a human: track a
//!    running average speed instead of every oscillation of the leader.
//! 5. Emergency braking from kinematics, then a first-order actuator lag.

use crate::bus::{BIN, BusSnap};
use crate::idm::{B_MAX, FREE_GAP};
use crate::step::{Lead, desired_speed, lead_of};
use crate::vehicles::Kind;
use crate::World;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum AutoMode {
    #[default]
    Acc,
    Cacc,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AutoState {
    pub mode: AutoMode,
    /// CACC controller state (commanded acceleration), m/s².
    pub u: f64,
    /// Running average of own speed (wave dampener), m/s.
    pub v_avg: f64,
    /// Dampener speed command, m/s.
    pub v_cmd: f64,
    /// Planned lateral move published on the bus.
    pub intent: i8,
    /// A slowdown was detected ahead (buffer gap active).
    pub slowdown_ahead: bool,
    /// Running mean of the leader's acceleration and of its magnitude:
    /// their difference is large only when the leader oscillates.
    pub lead_a_mean: f64,
    pub lead_a_abs: f64,
    /// Dampener engaged (with hysteresis).
    pub damping: bool,
}

impl AutoState {
    /// Update the stop-and-go detector: the leader's acceleration keeps
    /// changing sign (steady acceleration out of a queue does not count).
    fn update_damping(&mut self) {
        let index = self.lead_a_abs - self.lead_a_mean.abs();
        if self.damping { self.damping = index > 0.05 } else { self.damping = index > 0.15 }
    }
}

/// Result of the controller for one tick.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AutoOut {
    /// Actual acceleration after the actuator lag.
    pub a: f64,
    pub state: AutoState,
}

/// Slowest bin speed within `range` ahead of s, with its distance.
fn slowest_ahead(snap: &BusSnap, s: f64, range: f64, road_len: f64) -> Option<(f64, f64)> {
    let nb = snap.bin_n.len();
    if nb == 0 {
        return None;
    }
    let k0 = (s / BIN) as usize;
    let mut best: Option<(f64, f64)> = None;
    for j in 1..=((range / BIN) as usize) {
        let k = (k0 + j) % nb;
        if let Some(v) = snap.bin_speed(k) {
            let d = (k as f64 * BIN + BIN * 0.5 - s).rem_euclid(road_len);
            if best.is_none_or(|(bv, _)| v < bv) {
                best = Some((v, d));
            }
        }
    }
    best
}

/// Controller for auto `i`.
pub fn accel(w: &World, i: usize) -> AutoOut {
    let p = &w.params.auto;
    let ag = &w.veh.agent[i];
    let mut st = ag.auto;
    st.intent = ag.signal;
    let s = w.veh.s[i];
    let v = w.veh.v[i];
    let a_prev = w.veh.a[i];
    let v0 = desired_speed(w, i);
    let idm = ag.driver.idm;
    let dt = w.dt;
    let gap_factor = if w.events.weather.is_empty() { 1.0 } else { w.events.weather_at(&w.road, s).1 };
    st.v_avg += dt / 30.0 * (v - st.v_avg);

    let mut lead = lead_of(w, i);
    if lead.gap > p.radar_range {
        lead = Lead::FREE;
    }
    let k = dt / 30.0;
    st.lead_a_mean += k * (lead.a - st.lead_a_mean);
    st.lead_a_abs += k * (lead.a.abs() - st.lead_a_abs);
    st.update_damping();
    let yielding = crate::mobil::yield_target(w, i);
    let bus_ok = w.bus.up_at(s);
    let view = if bus_ok { w.bus.view() } else { None };

    // 3. Look-ahead: slow traffic downstream on the bus.
    let range = if w.bus.params.range_m > 0.0 { p.lookahead.min(w.bus.params.range_m) } else { p.lookahead };
    let mut a_la = f64::INFINITY;
    st.slowdown_ahead = false;
    if p.lookahead > 0.0
        && let Some(snap) = view
        && let Some((v_ds, d)) = slowest_ahead(snap, s, range, w.road.length)
        && v_ds < v - 3.0
    {
        st.slowdown_ahead = true;
        let x = ((d - 300.0) / (range - 300.0).max(1.0)).clamp(0.0, 1.0);
        let v_target = v_ds + (v - v_ds) * (0.4 + 0.6 * x);
        a_la = ((v_target - v) / 4.0).clamp(-1.5, 0.0);
    }

    // 1./2. Car following.
    let leader_entry = lead.slot.filter(|&l| w.veh.kind[l] == Kind::Auto).and_then(|l| view.and_then(|snap| snap.get(w.veh.id[l])));
    let mut a_cmd = if let (Some(e), true) = (leader_entry, lead.gap < FREE_GAP) {
        let h = p.t_cacc * gap_factor + if st.slowdown_ahead { p.buffer_gap } else { 0.0 };
        if st.mode != AutoMode::Cacc {
            st.u = a_prev;
        }
        st.mode = AutoMode::Cacc;
        let err = lead.gap - idm.s0 - h * v;
        let derr = (lead.v - v) - h * a_prev;
        st.u += dt / h * (-st.u + p.kp * err + p.kd * derr + p.kff * e.u);
        st.u = st.u.clamp(-B_MAX, idm.a);
        st.u
    } else {
        st.mode = AutoMode::Acc;
        let acc_idm = crate::idm::Idm { t: p.t_acc * gap_factor, ..idm };
        let mut a = acc_idm.accel(v, v0, lead.gap, v - lead.v);
        // 4. Dampener behind a human leader that is oscillating.
        if p.dampener && st.damping && lead.slot.is_some_and(|l| w.veh.kind[l] == Kind::Human) && lead.gap < FREE_GAP {
            let (g_l, g_u) = (idm.s0 + 0.8 * v + 2.0, idm.s0 + 2.5 * v + 10.0);
            let catch = ((lead.gap - g_l) / (g_u - g_l)).clamp(0.0, 1.0);
            let target = (st.v_avg + 1.5 * catch).min(v0);
            let safe = ((lead.gap - idm.s0) / (g_l - idm.s0).max(1.0)).clamp(0.0, 1.0);
            st.v_cmd = safe * target + (1.0 - safe) * lead.v.min(target);
            a = a.min(((st.v_cmd - v) / 2.0).clamp(-B_MAX, idm.a));
        }
        st.u = a;
        a
    };
    a_cmd = a_cmd.min(idm.free(v, v0)).min(a_la);
    // Merge coordinator: line up with a gap, or make room for a merger.
    let mc = crate::merge::command(w, i);
    if let Some(a_seek) = mc.seek {
        // Seeking replaces car following, within the IDM safety envelope.
        let env = crate::idm::Idm { t: p.t_cacc, ..idm };
        a_cmd = a_seek.min(env.accel(v, v0, lead.gap, v - lead.v)).min(a_la);
    }
    if let Some(a_m) = mc.limit {
        a_cmd = a_cmd.min(a_m);
    }
    // Courtesy: also make room for a vehicle signalling into this lane.
    if let Some((gap, vn)) = yielding {
        let gentle = crate::idm::Idm { t: p.t_acc, ..idm };
        a_cmd = a_cmd.min(gentle.accel(v, v0, gap, v - vn).max(-idm.b));
    }

    // 5. Safety envelope: far inside the desired gap (cut-ins, merges),
    //    the IDM's braking strategy takes over.
    let h = if st.mode == AutoMode::Cacc { p.t_cacc } else { p.t_acc };
    if lead.gap < 0.5 * (idm.s0 + h * v) {
        let env = crate::idm::Idm { t: h, ..idm };
        a_cmd = a_cmd.min(env.accel(v, v0, lead.gap, v - lead.v));
    }
    // Emergency braking from kinematics, allowing for the actuator lag:
    // stop within the gap plus the leader's own stopping distance if it is
    // braking, or match its speed within the gap if it is not.
    if lead.gap < FREE_GAP {
        let room = (lead.gap - 0.5 * idm.s0 - (v - lead.v).max(0.0) * p.actuator_tau).max(0.1);
        let need = if lead.a < -0.5 {
            let lead_stop = lead.v * lead.v / (2.0 * -lead.a);
            v * v / (2.0 * (room + lead_stop))
        } else if v > lead.v {
            (v - lead.v).powi(2) / (2.0 * room)
        } else {
            0.0
        };
        if need > idm.b {
            a_cmd = a_cmd.min(-need * 1.1);
        }
    }
    a_cmd = a_cmd.clamp(-B_MAX, idm.a);
    let a = a_prev + (dt / p.actuator_tau.max(dt)).min(1.0) * (a_cmd - a_prev);
    AutoOut { a, state: st }
}
