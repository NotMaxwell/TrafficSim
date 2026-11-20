//! Ramp merge coordinator (zipper merge) for automated vehicles.
//!
//! In each on-ramp zone, every mainline auto in lane 1 is assigned the
//! first merging vehicle directly ahead of it (between it and its own
//! leader) and opens a gap for it. Merging autos that find a vehicle
//! alongside which is not yielding drop back behind it. Merging vehicles
//! publish their intent (turn signal) on the bus, so the slot is known in
//! advance. Humans benefit too: autos yield to merging humans.

use crate::road::RampKind;
use crate::step::desired_speed;
use crate::World;

/// How far ahead a mainline auto looks for a merger to yield to, m.
const SLOT_RANGE: f64 = 90.0;
/// Upstream distance before a ramp zone where yielding starts, m.
const APPROACH: f64 = 150.0;
/// Hardest braking used to open a gap, m/s².
const MAX_YIELD_DECEL: f64 = 2.5;

/// The merger (lane-0 vehicle) that mainline auto `i` is assigned to make
/// room for: the first signalling vehicle ahead in lane 0, within range,
/// with nobody in lane 1 between them.
pub fn assigned_merger(w: &World, i: usize) -> Option<usize> {
    if w.veh.lane[i] != 1 || w.veh.kind[i] != crate::vehicles::Kind::Auto {
        return None;
    }
    let s = w.veh.s[i];
    w.road.ramps.iter().find(|r| {
        r.kind == RampKind::On && !r.all_lanes && w.road.dist(r.zone_start - APPROACH, s) < r.zone_end - r.zone_start + APPROACH
    })?;
    let (ahead, _) = w.veh.around(0, s, i);
    let m = ahead?;
    let d = w.road.dist(s, w.veh.s[m]);
    if d > SLOT_RANGE || w.veh.agent[m].signal != 1 || w.veh.agent[m].crashed() {
        return None;
    }
    if let Some(l) = w.veh.leader(i)
        && w.road.dist(s, w.veh.s[l]) < d
    {
        return None;
    }
    Some(m)
}

/// Extra acceleration limit for auto `i` from the merge coordinator.
pub fn constraint(w: &World, i: usize) -> Option<f64> {
    let s = w.veh.s[i];
    let v = w.veh.v[i];
    match w.veh.lane[i] {
        1 => {
            let m = assigned_merger(w, i)?;
            let gap = (w.road.dist(s, w.veh.s[m]) - w.veh.len[m]).max(0.1);
            let p = &w.params.auto;
            let idm = crate::idm::Idm { t: p.t_cacc, ..w.veh.agent[i].driver.idm };
            Some(idm.accel(v, desired_speed(w, i), gap, v - w.veh.v[m]).max(-MAX_YIELD_DECEL))
        }
        0 => {
            let r = w.road.ramp_zone_at(s)?;
            if r.kind != RampKind::On {
                return None;
            }
            // Someone alongside in lane 1 who is not making room: drop back.
            let (lead1, fol1) = w.veh.around(1, s, i);
            let s0 = w.veh.agent[i].driver.idm.s0;
            // ...unless it is the auto assigned to open the gap for us.
            let alongside_behind =
                fol1.is_some_and(|f| w.road.dist(w.veh.s[f], s) < w.veh.len[i] + s0 && assigned_merger(w, f) != Some(i));
            let alongside_ahead = lead1.is_some_and(|l| w.road.dist(s, w.veh.s[l]) - w.veh.len[l] < s0);
            let other = match (alongside_behind, alongside_ahead) {
                (true, _) => fol1,
                (_, true) => lead1,
                _ => None,
            }?;
            Some(((w.veh.v[other] - 2.0 - v) / 2.0).clamp(-MAX_YIELD_DECEL, 0.0))
        }
        _ => None,
    }
}
