//! Lane changing (MOBIL) with mandatory changes for exits, lane ends,
//! ramp merges and blockages.
//!
//! A driver changes lanes when own gain − politeness × (losses imposed on
//! the old and new followers) exceeds a threshold, and only if the new
//! follower would not have to brake harder than b_safe. Decisions are
//! applied one at a time in slot order, each against the already-updated
//! lane index, so two vehicles can never take the same gap in one tick.

use crate::idm::FREE_GAP;
use crate::step::{desired_speed, lane_end};
use crate::vehicles::{Kind, NO_RAMP};
use crate::World;

/// Lateral need from routing and road layout.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Need {
    /// Preferred direction: -1 = right, +1 = left, 0 = none.
    pub dir: i8,
    /// Extra incentive toward `dir`, m/s².
    pub strength: f64,
    /// Moves away from `dir` are forbidden.
    pub hard: bool,
    /// Check every tick instead of on the staggered schedule.
    pub urgent: bool,
}

/// Whatever is ahead of a vehicle at `s` in `lane`: (gap, leader speed).
/// Includes the end of the lane as a stopped obstacle.
pub fn lead_in(w: &World, i: usize, lane: u8, s: f64) -> (Option<usize>, f64, f64) {
    let (lead, _) = w.veh.around(lane, s, i);
    let (mut gap, mut v) = (FREE_GAP, 0.0);
    if let Some(l) = lead {
        let g = w.road.dist(s, w.veh.s[l]) - w.veh.len[l];
        if g < FREE_GAP {
            (gap, v) = (g, w.veh.v[l]);
        }
    }
    if let Some(d) = lane_end(w, i, lane, s)
        && d < gap
    {
        return (None, d, 0.0);
    }
    (lead, gap, v)
}

/// IDM acceleration of vehicle `j` behind an obstacle at `gap` moving at `v_l`.
pub fn idm_acc(w: &World, j: usize, gap: f64, v_l: f64) -> f64 {
    if w.veh.agent[j].crashed() {
        return 0.0;
    }
    let v = w.veh.v[j];
    w.veh.agent[j].driver.idm.accel(v, desired_speed(w, j), gap, v - v_l)
}

/// Routing / geometry needs of vehicle `i`.
pub fn need_of(w: &World, i: usize) -> Need {
    let s = w.veh.s[i];
    let lane = w.veh.lane[i];
    let mp = &w.params.mobil;
    // Lane ends ahead (on-ramp merge lane, lane drop, closure).
    if let Some(d) = lane_end(w, i, lane, s)
        && d < 600.0
    {
        let beyond = s + d + 10.0;
        let dir = if lane == 0 || !w.road.lane_exists(w.road.wrap(beyond), lane - 1) { 1 } else { -1 };
        let strength = if lane == 0 { 3.0 } else { 0.5 + 3.0 * (1.0 - d / 600.0) };
        return Need { dir, strength, hard: true, urgent: lane == 0 || d < 300.0 };
    }
    // A blockage (crash) just ahead: get out of this lane.
    if let Some(l) = w.veh.leader(i)
        && w.veh.agent[l].crashed()
        && w.road.dist(s, w.veh.s[l]) < 250.0
    {
        let dir = if lane > 1 { -1 } else { 1 };
        return Need { dir, strength: 2.0, hard: false, urgent: true };
    }
    // Heading for an off-ramp that uses the auxiliary lane.
    let ex = w.veh.agent[i].exit;
    if ex != NO_RAMP {
        let r = &w.road.ramps[ex as usize];
        if !r.all_lanes {
            let d = w.road.dist(s, r.zone_end);
            if lane == 0 && d < r.zone_end - r.zone_start + 1.0 {
                return Need { dir: 0, strength: 0.0, hard: true, urgent: false };
            }
            if d < mp.exit_prepare {
                let x = (1.0 - d / mp.exit_prepare).clamp(0.0, 1.0);
                let in_zone = d <= r.zone_end - r.zone_start;
                return Need { dir: -1, strength: 0.6 + 2.4 * x, hard: true, urgent: in_zone };
            }
        }
    }
    Need::default()
}

/// Is `target` a lane vehicle `i` may move into at its position?
fn lane_allowed(w: &World, i: usize, target: u8) -> bool {
    let s = w.veh.s[i];
    if target > w.road.lanes || !w.road.lane_exists(s, target) {
        return false;
    }
    if target == 0 {
        // Only exiting vehicles, inside their own diverge zone.
        return w.road.ramp_zone_at(s).is_some_and(|r| r.id == w.veh.agent[i].exit);
    }
    // Don't move into a lane that is about to end.
    lane_end(w, i, target, s).is_none_or(|d| d > 80.0)
}

/// Best lane change for `i`, if any: (target lane, incentive).
pub fn decide(w: &World, i: usize, need: Need) -> Option<u8> {
    let s = w.veh.s[i];
    let lane = w.veh.lane[i];
    let ag = &w.veh.agent[i];
    let mp = &w.params.mobil;
    let human = w.veh.kind[i] == Kind::Human;
    let (back_range, front_range) =
        if human { (w.params.human.mirror_back, w.params.human.mirror_front) } else { (FREE_GAP, FREE_GAP) };

    let (_, gap_c, v_c) = lead_in(w, i, lane, s);
    let a_cur = idm_acc(w, i, gap_c, v_c);
    // Old follower: gains if we leave.
    let old_f = w.veh.follower(i).filter(|&f| w.road.dist(w.veh.s[f], s) < back_range);
    let (mut f_old_gain, mut f_old_base) = (0.0, 0.0);
    if let Some(f) = old_f {
        let g_now = w.road.dist(w.veh.s[f], s) - w.veh.len[i];
        f_old_base = idm_acc(w, f, g_now, w.veh.v[i]);
        let (_, g_after, v_after) = match w.veh.leader(i) {
            Some(l) if l != f => (Some(l), w.road.dist(w.veh.s[f], w.veh.s[l]) - w.veh.len[l], w.veh.v[l]),
            _ => (None, FREE_GAP, 0.0),
        };
        f_old_gain = idm_acc(w, f, g_after.min(FREE_GAP), v_after);
    }

    let mut best: Option<(u8, f64)> = None;
    for dir in [-1i8, 1] {
        if need.hard && need.dir != 0 && dir != need.dir {
            continue;
        }
        if need.hard && need.dir == 0 {
            continue;
        }
        let t = lane as i16 + dir as i16;
        if !(0..=w.road.lanes as i16).contains(&t) {
            continue;
        }
        let t = t as u8;
        if !lane_allowed(w, i, t) {
            continue;
        }
        let (lead, fol) = w.veh.around(t, s, i);
        // Leader in the target lane.
        let (_, mut gap_n, mut v_n) = lead_in(w, i, t, s);
        if let Some(l) = lead
            && w.road.dist(s, w.veh.s[l]) > front_range
            && gap_n >= w.road.dist(s, w.veh.s[l]) - w.veh.len[l]
        {
            (gap_n, v_n) = (FREE_GAP, 0.0);
        }
        if gap_n < 0.5 {
            continue;
        }
        let a_new = idm_acc(w, i, gap_n, v_n);
        if a_new < -mp.b_safe {
            continue;
        }
        // New follower: safety criterion and its loss.
        let mut f_new_loss = 0.0;
        if let Some(f) = fol {
            let d_back = w.road.dist(w.veh.s[f], s);
            if d_back < back_range {
                let gap_b = d_back - w.veh.len[i];
                if gap_b < 0.5 {
                    continue;
                }
                let a_f_new = idm_acc(w, f, gap_b, w.veh.v[i]);
                if a_f_new < -mp.b_safe {
                    continue;
                }
                let (_, g_old, v_old) = lead_in(w, f, t, w.veh.s[f]);
                f_new_loss = a_f_new - idm_acc(w, f, g_old, v_old);
            }
        }
        let toward = need.dir == dir;
        let p = if toward { ag.driver.politeness * (1.0 - (need.strength / 3.0).min(1.0)) } else { ag.driver.politeness };
        let mut gain = a_new - a_cur + p * (f_new_loss + (f_old_gain - f_old_base));
        if toward {
            gain += need.strength;
        } else if need.dir != 0 {
            gain -= need.strength;
        }
        if t != 0 && lane != 0 {
            gain += if dir < 0 { mp.bias_right } else { -mp.bias_right };
        }
        let th = if toward && need.strength > 1.0 { 0.0 } else { ag.driver.threshold };
        if gain > th && best.is_none_or(|(_, g)| gain > g) {
            best = Some((t, gain));
        }
    }
    best.map(|(t, _)| t)
}

/// Step 4: lane-change decisions, applied immediately one by one.
pub fn lane_changes(w: &mut World) {
    let check = (w.params.mobil.check_interval / w.dt).round().max(1.0) as u64;
    let b_safe = w.params.mobil.b_safe;
    for i in 0..w.veh.len() {
        let ag = &w.veh.agent[i];
        if ag.crashed() || ag.forced.is_some() {
            continue;
        }
        let need = need_of(w, i);
        let due = (w.veh.id[i] + w.tick).is_multiple_of(check) || need.urgent;
        let cooldown = if need.urgent { 1.0 } else { 5.0 };
        if !due || w.time - ag.last_change < cooldown {
            continue;
        }
        let Some(t) = decide(w, i, need) else { continue };
        let old = w.veh.lane[i];
        w.veh.move_lane(i, t);
        w.veh.lat[i] = t as f32 - old as f32;
        let ag = &mut w.veh.agent[i];
        ag.last_change = w.time;
        ag.human.filled = 0;
        w.stats.lane_changes += 1;
        // Audit with true state (humans may have missed someone in the mirror).
        if let Some(f) = w.veh.follower(i)
            && f != i
        {
            let gap = w.road.dist(w.veh.s[f], w.veh.s[i]) - w.veh.len[i];
            if idm_acc(w, f, gap, w.veh.v[i]) < -b_safe {
                w.stats.unsafe_lane_changes += 1;
            }
        }
    }
}
