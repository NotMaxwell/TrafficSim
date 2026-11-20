//! Merge coordinator for automated vehicles, used for every forced merge:
//! on-ramps, lane drops and closed lanes.
//!
//! With the merge known well ahead (bus), a merging auto picks the nearest
//! gap in the target lane that fits it and drifts into line with it using
//! small speed adjustments, so nobody has to brake. Autos in the target
//! lane help gently: each opens space for the first merger just ahead of
//! it, braking hard only when that merger is about to run out of lane.
//! Autos also make room for merging humans.

use crate::step::{desired_speed, lane_end};
use crate::vehicles::Kind;
use crate::World;

/// How far ahead an auto looks for a merger to make room for, m.
const SLOT_RANGE: f64 = 90.0;
/// Gentle yielding while the merger still has room, m/s².
const GENTLE_YIELD: f64 = 1.0;
/// Hardest braking used to open a gap for a merger near its lane end, m/s².
const MAX_YIELD_DECEL: f64 = 2.5;
/// A merger closer than this to its lane end gets firm help, m.
const URGENT_END: f64 = 250.0;
/// How far forward/back a merger searches for a gap, m.
const SEEK_RANGE: f64 = 150.0;

/// Is `m` waiting to move into lane `into` because it has to?
fn forced_into(w: &World, m: usize, into: u8) -> bool {
    let ag = &w.veh.agent[m];
    !ag.crashed() && w.veh.lane[m] as i16 + ag.signal as i16 == into as i16 && (ag.signal_forced || w.veh.lane[m] == 0)
}

/// The merger auto `i` is assigned to make room for, if any.
pub fn assigned_merger(w: &World, i: usize) -> Option<usize> {
    let lane = w.veh.lane[i];
    if lane == 0 || w.veh.kind[i] != Kind::Auto {
        return None;
    }
    let s = w.veh.s[i];
    let lead_d = w.veh.leader(i).map_or(f64::INFINITY, |l| w.road.dist(s, w.veh.s[l]));
    let mut best: Option<(f64, usize)> = None;
    for adj in [lane as i16 - 1, lane as i16 + 1] {
        if !(0..=w.road.lanes as i16).contains(&adj) {
            continue;
        }
        let Some(m) = w.veh.around(adj as u8, s, i).0 else { continue };
        let d = w.road.dist(s, w.veh.s[m]);
        // Zipper: a merger stuck at its lane end right beside us cannot
        // move into a gap we open by braking; we go ahead and it slots in
        // behind us instead.
        let beside = d - w.veh.len[m] < w.veh.agent[m].driver.idm.s0;
        let stuck = lane_end(w, m, w.veh.lane[m], w.veh.s[m]).is_some_and(|e| e < 15.0);
        if beside && stuck {
            continue;
        }
        if d <= SLOT_RANGE && d < lead_d && forced_into(w, m, lane) && best.is_none_or(|(bd, _)| d < bd) {
            best = Some((d, m));
        }
    }
    best.map(|(_, m)| m)
}

/// Signed distance from a to b along the ring, in [-L/2, L/2).
fn signed(w: &World, a: f64, b: f64) -> f64 {
    let d = w.road.dist(a, b);
    if d > w.road.length * 0.5 { d - w.road.length } else { d }
}

/// Gap seeking for a merging auto: acceleration that lines it up with the
/// nearest gap in `target` that fits it, or None if already lined up.
fn seek_gap(w: &World, i: usize, target: u8) -> Option<f64> {
    let list = w.veh.lanes.get(target as usize)?;
    let s = w.veh.s[i];
    let v = w.veh.v[i];
    let len = w.veh.len[i];
    let s0 = w.veh.agent[i].driver.idm.s0;
    let need = len + 2.0 * (s0 + 0.4 * v); // room for the car plus margins
    let n = list.len();
    if n == 0 {
        return None;
    }
    let k = list.partition_point(|&j| w.veh.s[j as usize] <= s);
    // Someone in the target lane is opening a gap for us: settle just
    // behind the car ahead of it, at its speed (zipper).
    let (lead_t, fol_t) = w.veh.around(target, s, i);
    if fol_t.is_some_and(|f| assigned_merger(w, f) == Some(i)) {
        let l = lead_t?;
        let slot = w.road.wrap(w.veh.s[l] - w.veh.len[l] - (s0 + 0.4 * v));
        let e = signed(w, s, slot).min(0.0);
        let dv = w.veh.v[l] - v;
        if e > -0.5 && dv.abs() < 0.5 {
            return None;
        }
        return Some((0.15 * e + 0.6 * dv).clamp(-1.5, 1.0));
    }
    let mut best: Option<(f64, f64)> = None; // (position error, gap speed)
    for off in 0..8.min(n) {
        for idx in [k + off, k + n - off] {
            let (fi, li) = ((idx + n - 1) % n, idx % n);
            let (f, l) = (list[fi] as usize, list[li] as usize);
            if f == i || l == i || (n == 1 && f == l) {
                continue;
            }
            let room = w.road.dist(w.veh.s[f], w.veh.s[l]) - w.veh.len[l];
            if room < need {
                continue;
            }
            // Nearest point in the gap where our front bumper fits with
            // margins on both sides.
            let margin = s0 + 0.4 * v;
            let lo = signed(w, s, w.road.wrap(w.veh.s[f] + len + margin));
            let hi = signed(w, s, w.road.wrap(w.veh.s[l] - w.veh.len[l] - margin));
            let e = if lo > hi { 0.5 * (lo + hi) } else { 0.0_f64.clamp(lo, hi) };
            if e.abs() > SEEK_RANGE {
                continue;
            }
            if best.is_none_or(|(be, _)| e.abs() < be.abs()) {
                best = Some((e, 0.5 * (w.veh.v[f] + w.veh.v[l])));
            }
        }
    }
    // No gap fits nearby and nobody is opening one: drift back gently
    // relative to the target lane until a gap or a helper turns up.
    let Some((e, v_gap)) = best else {
        let v_lane = lead_t.or(fol_t).map_or(v, |j| w.veh.v[j]);
        return (v > v_lane - 3.0).then_some(-0.5);
    };
    if e.abs() < 0.5 {
        return None; // lined up: MOBIL will take the gap
    }
    Some((0.15 * e + 0.6 * (v_gap - v)).clamp(-1.5, 1.0))
}

/// What the coordinator asks of auto `i` this tick.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MergeCommand {
    /// Upper limit on acceleration (making room for a merger).
    pub limit: Option<f64>,
    /// Desired acceleration to line up with a gap (replaces car-following
    /// but stays within the safety envelope).
    pub seek: Option<f64>,
}

pub fn command(w: &World, i: usize) -> MergeCommand {
    let s = w.veh.s[i];
    let v = w.veh.v[i];
    let mut cmd = MergeCommand::default();
    if let Some(m) = assigned_merger(w, i) {
        let gap = (w.road.dist(s, w.veh.s[m]) - w.veh.len[m]).max(0.1);
        let idm = crate::idm::Idm { t: w.params.auto.t_cacc, ..w.veh.agent[i].driver.idm };
        let urgent = lane_end(w, m, w.veh.lane[m], w.veh.s[m]).is_some_and(|d| d < URGENT_END);
        let floor = if urgent { MAX_YIELD_DECEL } else { GENTLE_YIELD };
        cmd.limit = Some(idm.accel(v, desired_speed(w, i), gap, v - w.veh.v[m]).max(-floor));
    }
    let lane = w.veh.lane[i];
    let sig = w.veh.agent[i].signal;
    if sig != 0 {
        let target = (lane as i16 + sig as i16).max(0) as u8;
        if forced_into(w, i, target) {
            cmd.seek = seek_gap(w, i, target);
        }
    }
    cmd
}
