//! Creating vehicles: initial population, per-vehicle traits, and (for the
//! corridor) ramp arrivals and removal at off-ramps.

use rand::Rng;

use crate::auto::AutoState;
use crate::human::HumanState;
use crate::idm::Idm;
use crate::rng::{self, Purpose, VehRng};
use crate::scenario::Population;
use crate::vehicles::{Agent, Driver, Kind, NO_RAMP};
use crate::World;

/// Draw a new vehicle's kind, traits and length from its id-keyed streams.
/// Every number is always drawn in the same order, so a given id gets the
/// same personality in every world regardless of auto share.
pub fn make_agent(w: &World, id: u64, origin: u16, exit: u16) -> (Kind, Agent, f64) {
    let p = &w.params;
    let mut r = rng::stream(w.seed, Purpose::Traits, id);
    let is_truck = r.random::<f64>() < w.truck_share;
    let u_v0: f64 = r.random();
    let u_t: f64 = r.random();
    let u_pol: f64 = r.random();
    let z_tau = rng::normal(&mut r);
    let u_ant: f64 = r.random();
    let kind = if rng::uniform(w.seed, Purpose::Kind, id) < w.auto_share { Kind::Auto } else { Kind::Human };

    let h = &p.human;
    let mut driver = match kind {
        Kind::Human => {
            let mut idm = h.idm;
            idm.v0 *= 1.0 + h.v0_spread * (2.0 * u_v0 - 1.0);
            idm.t = h.t_min + (h.t_max - h.t_min) * u_t;
            let tau = if h.delay { (h.tau_median * (h.tau_sigma * z_tau).exp()).clamp(h.tau_min, h.tau_max) } else { 0.0 };
            let max_ant = h.anticipation_max.clamp(1, 3);
            Driver {
                idm,
                politeness: h.polite_min + (h.polite_max - h.polite_min) * u_pol,
                threshold: p.mobil.threshold,
                tau,
                anticipation: 1 + ((u_ant * max_ant as f64) as u8).min(max_ant - 1),
            }
        }
        Kind::Auto => {
            let mut idm = p.auto.idm;
            idm.v0 = w.road.limit;
            Driver { idm, politeness: p.auto.politeness, threshold: p.auto.threshold, tau: 0.0, anticipation: 1 }
        }
    };
    let mut len = match kind {
        Kind::Human => h.length,
        Kind::Auto => p.auto.length,
    };
    if is_truck {
        let t = &p.truck;
        driver.idm = Idm { v0: t.v0.min(driver.idm.v0), a: t.a, b: t.b, t: t.t.max(driver.idm.t), ..driver.idm };
        len = t.length;
    }
    let agent = Agent {
        driver,
        truck: is_truck,
        origin,
        exit,
        spawn_t: w.time,
        crashed_until: -1.0,
        forced: None,
        human: HumanState::default(),
        auto: AutoState::default(),
        rng: VehRng::new(w.seed, id),
        last_change: f64::NEG_INFINITY,
        odo: 0.0,
        s_prev: 0.0,
        hard_braking: false,
    };
    (kind, agent, len)
}

/// Add a vehicle to the world. Returns its slot. The lane index is stale
/// until the caller runs `reindex`.
pub fn add_vehicle(w: &mut World, s: f64, lane: u8, v: f64, origin: u16, exit: u16) -> usize {
    let id = w.next_id;
    w.next_id += 1;
    let (kind, mut agent, len) = make_agent(w, id, origin, exit);
    agent.s_prev = s;
    agent.auto.v_avg = v;
    agent.auto.v_cmd = v;
    w.stats.spawned += 1;
    if kind == Kind::Auto {
        w.stats.autos_spawned += 1;
    }
    w.veh.push(id, s, v, lane, len, kind, agent)
}

/// Place the initial vehicles.
pub fn populate(w: &mut World, pop: &Population) {
    if w.road.is_corridor() {
        return;
    }
    // Ring: spread `count` vehicles evenly, round-robin over lanes.
    let lanes = w.road.lanes as usize;
    let n = pop.count;
    if n == 0 {
        return;
    }
    let per_lane = n.div_ceil(lanes);
    let spacing = w.road.length / per_lane as f64;
    for k in 0..n {
        let lane = (k % lanes) as u8 + 1;
        let idx = k / lanes;
        let s = w.road.wrap(idx as f64 * spacing + (lane - 1) as f64 * spacing / lanes as f64);
        let v = if pop.initial_speed >= 0.0 {
            pop.initial_speed
        } else {
            let gap = spacing - w.params.human.length;
            w.params.human.idm.equilibrium_speed(gap, w.road.limit.min(w.params.human.idm.v0))
        };
        add_vehicle(w, s, lane, v, NO_RAMP, NO_RAMP);
    }
}
