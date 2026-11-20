//! Milestone 5 gate: reaction delay, perception limits, estimation noise,
//! distraction, collisions.

mod common;
use common::*;
use traffic_core::demand::add_vehicle;
use traffic_core::scenario::{Population, RoadConfig};
use traffic_core::vehicles::{Forced, NO_RAMP};
use traffic_core::{Scenario, World};

fn ring(length: f64, count: usize) -> Scenario {
    Scenario {
        name: "human ring".into(),
        seed: 11,
        road: RoadConfig::Ring { length_m: length, lanes: 1, speed_limit_kmh: 120.0 },
        population: Population { count, ..Default::default() },
        ..Default::default()
    }
}

/// Lag between a leader starting to brake and the follower responding.
fn response_lag(tau: Option<f64>) -> f64 {
    let mut sc = ring(5000.0, 0);
    let h = &mut sc.human;
    h.v0_spread = 0.0;
    h.estimation = false;
    h.distraction = false;
    h.anticipation_max = 1;
    match tau {
        Some(t) => (h.tau_median, h.tau_min, h.tau_max) = (t, t, t),
        None => h.delay = false,
    }
    let mut w = World::new(&sc, 0.0);
    add_vehicle(&mut w, 0, 100.0, 1, 20.0, NO_RAMP, NO_RAMP);
    add_vehicle(&mut w, 1, 140.0, 1, 20.0, NO_RAMP, NO_RAMP);
    w.veh.reindex();
    let leader = w.veh.id.iter().position(|&id| id == 1).unwrap();
    w.veh.agent[leader].forced = Some(Forced { target: 20.0, decel: 1.0, until: 1e9 });
    w.run_for(90.0);
    let t0 = w.time;
    let leader = w.veh.id.iter().position(|&id| id == 1).unwrap();
    w.veh.agent[leader].forced = Some(Forced { target: 5.0, decel: 4.0, until: t0 + 20.0 });
    let fol = w.veh.id.iter().position(|&id| id == 0).unwrap();
    let base = w.veh.a[fol];
    for _ in 0..60 {
        w.run_ticks(1);
        let fol = w.veh.id.iter().position(|&id| id == 0).unwrap();
        if w.veh.a[fol] < base - 0.3 {
            return w.time - t0;
        }
    }
    panic!("follower never responded");
}

#[test]
fn braking_response_lags_by_tau() {
    let instant = response_lag(None);
    // Pipeline latency: leader speed changes one tick after its command,
    // and the follower's response crosses the detection threshold a tick later.
    assert!(instant <= 0.35, "without delay the response should be immediate ({instant:.2} s)");
    for tau in [0.8, 1.2, 2.0] {
        let lag = response_lag(Some(tau));
        eprintln!("tau {tau}: response lag {lag:.2} s (baseline {instant:.2} s)");
        assert!(
            (lag - instant - tau).abs() <= 0.15,
            "τ = {tau}: lag {lag:.2} s should be τ + {instant:.2} s"
        );
    }
}

/// Rush-hour ring: 35 veh/km on 5 km. Human delays (default τ spread) and
/// multi-anticipation; estimation noise and distraction off and a common
/// time gap so the ring starts in a clean equilibrium. One vehicle dips
/// 3 m/s; we follow the disturbance through the 100 vehicles behind it.
/// Returns per-follower (dip, time of minimum, position of minimum).
fn brake_wave() -> (Vec<(f64, f64, f64)>, f64, World) {
    let mut sc = ring(5000.0, 175);
    let h = &mut sc.human;
    h.distraction = false;
    h.estimation = false;
    h.v0_spread = 0.0;
    (h.t_min, h.t_max) = (1.5, 1.5);
    h.tau_max = 1.5;
    let mut w = World::new(&sc, 0.0);
    w.run_for(120.0);
    let v_eq = w.mean_speed();
    let braker = 0usize;
    let t0 = w.time;
    w.veh.agent[braker].forced = Some(Forced { target: v_eq - 3.0, decel: 2.0, until: t0 + 5.0 });
    let mut chain = Vec::new();
    let mut k = braker;
    for _ in 0..100 {
        k = w.veh.follower(k).unwrap();
        chain.push(w.veh.id[k]);
    }
    let slot = |w: &World, id: u64| w.veh.id.iter().position(|&x| x == id).unwrap();
    let base: Vec<f64> = chain.iter().map(|&id| w.veh.agent[slot(&w, id)].odo).collect();
    let behind: Vec<f64> = chain.iter().map(|&id| w.road.dist(w.veh.s[slot(&w, id)], w.veh.s[braker])).collect();
    let mut best = vec![(f64::INFINITY, 0.0, 0.0); chain.len()];
    // 160 s: long enough for the first pass 2 km upstream, short enough
    // that the disturbance has not lapped the ring.
    for _ in 0..1600 {
        w.run_ticks(1);
        for (c, &id) in chain.iter().enumerate() {
            let i = slot(&w, id);
            if w.veh.v[i] < best[c].0 {
                best[c] = (w.veh.v[i], w.time - t0, -behind[c] + (w.veh.agent[i].odo - base[c]));
            }
        }
    }
    let out = best.into_iter().map(|(vmin, t, x)| (v_eq - vmin, t, x)).collect();
    (out, v_eq, w)
}

#[test]
fn wave_amplification_above_one_at_rush_hour_density() {
    let (dips, v_eq, w) = brake_wave();
    assert!(v_eq > 8.0 && v_eq < 22.0, "rush-hour equilibrium speed {:.1} km/h", v_eq * 3.6);
    // Speed dip at the trigger is the imposed 3 m/s; ≈ 2 km upstream at
    // 28.6 m spacing is the 70th follower.
    let near = 3.0;
    let far: f64 = dips[65..75].iter().map(|d| d.0).sum::<f64>() / 10.0;
    let amp = far / near;
    eprintln!("amplification: trigger dip {near:.2} m/s, 2 km upstream {far:.2} m/s, ratio {amp:.2}");
    assert!(amp > 1.0, "human waves should grow: trigger dip {near:.2}, 2 km upstream {far:.2}, ratio {amp:.2}");
    assert_eq!(w.stats.collisions, 0, "no crashes in this controlled experiment");
}

#[test]
fn shockwave_speed_upstream_10_to_20_kmh() {
    // A vehicle stops for 20 s at rush-hour density; the resulting jam's
    // head (where drivers pull away) travels upstream. Track where and
    // when each follower leaves the jam and regress position on time.
    let mut sc = ring(5000.0, 175);
    sc.human.distraction = false;
    let mut w = World::new(&sc, 0.0);
    w.run_for(60.0);
    let t0 = w.time;
    w.veh.agent[0].forced = Some(Forced { target: 0.0, decel: 4.0, until: t0 + 20.0 });
    let slot = |w: &World, id: u64| w.veh.id.iter().position(|&x| x == id).unwrap();
    let mut chain = Vec::new();
    let mut k = 0;
    for _ in 0..60 {
        k = w.veh.follower(k).unwrap();
        chain.push(w.veh.id[k]);
    }
    let x0: Vec<f64> = chain.iter().map(|&id| -w.road.dist(w.veh.s[slot(&w, id)], w.veh.s[0]) - w.veh.agent[slot(&w, id)].odo).collect();
    let mut stopped = vec![false; chain.len()];
    let mut exit: Vec<Option<(f64, f64)>> = vec![None; chain.len()];
    for _ in 0..2500 {
        w.run_ticks(1);
        for (c, &id) in chain.iter().enumerate() {
            let i = slot(&w, id);
            let v = w.veh.v[i];
            if v < 1.0 {
                stopped[c] = true;
            } else if stopped[c] && exit[c].is_none() && v > 5.0 {
                exit[c] = Some((w.time - t0, x0[c] + w.veh.agent[i].odo));
            }
        }
    }
    let pts: Vec<(f64, f64)> = exit[10..50].iter().flatten().copied().collect();
    assert!(pts.len() >= 30, "the jam should reach most followers ({} of 40)", pts.len());
    let n = pts.len() as f64;
    let (mt, mx) = (pts.iter().map(|p| p.0).sum::<f64>() / n, pts.iter().map(|p| p.1).sum::<f64>() / n);
    let slope = pts.iter().map(|p| (p.0 - mt) * (p.1 - mx)).sum::<f64>() / pts.iter().map(|p| (p.0 - mt).powi(2)).sum::<f64>();
    let kmh = slope * 3.6;
    eprintln!("jam head speed: {kmh:.1} km/h from {} followers", pts.len());
    assert!((-20.0..=-10.0).contains(&kmh), "jam head speed {kmh:.1} km/h (expected 10 to 20 km/h upstream)");
    assert_eq!(w.stats.collisions, 0);
}

#[test]
fn distraction_and_noise_are_seeded_and_crashes_freeze_vehicles() {
    // Full human model on the Sugiyama ring with long reaction times:
    // deterministic, and any crash leaves both vehicles stopped.
    let mut sc = scenario("sugiyama_ring");
    sc.human = traffic_core::params::HumanParams { idm: sc.human.idm, length: sc.human.length, ..Default::default() };
    sc.human.tau_min = 1.8;
    sc.human.tau_median = 2.2;
    let mut a = World::new(&sc, 0.0);
    let mut b = World::new(&sc, 0.0);
    a.run_for(900.0);
    b.run_for(900.0);
    assert_eq!(a.state_hash(), b.state_hash());
    for i in 0..a.veh.len() {
        if a.veh.agent[i].crashed() {
            assert_eq!(a.veh.v[i], 0.0);
        }
    }
    let _ = std_dev(&a.veh.v);
}
