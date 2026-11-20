//! Milestone 7 gate: state bus, CACC, ACC fallback, look-ahead.

mod common;
use common::*;
use traffic_core::bus::Outage;
use traffic_core::scenario::{Population, RoadConfig};
use traffic_core::vehicles::Forced;
use traffic_core::{Scenario, Sim, World};

fn ring(length: f64, count: usize) -> Scenario {
    Scenario {
        name: "auto ring".into(),
        seed: 21,
        road: RoadConfig::Ring { length_m: length, lanes: 1, speed_limit_kmh: 120.0 },
        population: Population { count, ..Default::default() },
        ..Default::default()
    }
}

#[test]
fn amplification_below_one_at_full_automation() {
    // Same experiment as the human gate (35 veh/km), plus 60 veh/km where
    // autos are in the car-following regime rather than at the limit.
    for count in [175, 300] {
        amplification(count);
    }
}

fn amplification(count: usize) {
    let sc = ring(5000.0, count);
    let mut w = World::new(&sc, 1.0);
    w.run_for(120.0);
    let v_eq = w.mean_speed();
    let t0 = w.time;
    w.veh.agent[0].forced = Some(Forced { target: v_eq - 3.0, decel: 2.0, until: t0 + 5.0 });
    let mut chain = Vec::new();
    let mut k = 0;
    for _ in 0..(2000 * count / 5000 + 5) {
        k = w.veh.follower(k).unwrap();
        chain.push(w.veh.id[k]);
    }
    let mut vmin = vec![f64::INFINITY; chain.len()];
    let braker = w.veh.id[0];
    let mut v_braker = f64::INFINITY;
    for _ in 0..1600 {
        w.run_ticks(1);
        let b = w.veh.id.iter().position(|&x| x == braker).unwrap();
        v_braker = v_braker.min(w.veh.v[b]);
        for (c, &id) in chain.iter().enumerate() {
            let i = w.veh.id.iter().position(|&x| x == id).unwrap();
            vmin[c] = vmin[c].min(w.veh.v[i]);
        }
    }
    // Actual dip of the braking vehicle (it can undershoot its target).
    let near = v_eq - v_braker;
    // ≈ 2 km upstream.
    let k2 = (2000.0 * count as f64 / 5000.0) as usize;
    let chain_len = chain.len();
    assert!(k2 + 5 <= chain_len);
    let far = (k2 - 5..k2 + 5).map(|c| v_eq - vmin[c]).sum::<f64>() / 10.0;
    let first = v_eq - vmin[0];
    eprintln!("autos at {} veh/km: v_eq {:.1} km/h, trigger dip {near:.2}, first follower {first:.2}, 2 km upstream {far:.3}, ratio {:.3}", count / 5, v_eq * 3.6, far / near);
    assert!(far / near < 1.0, "auto platoon should damp the wave: 2 km upstream dip {far:.3} vs trigger {near}");
    assert!(first <= near * 1.05, "first follower should not amplify the dip ({first:.2} vs {near:.2})");
    assert_eq!(w.stats.collisions, 0);
}

#[test]
fn single_auto_damps_ring_waves() {
    // Stern et al. 2018: 22 vehicles on a 260 m ring in stop-and-go; one
    // vehicle switches to auto (wave dampener) and the waves die down.
    let mut sc = scenario("sugiyama_ring");
    sc.road = RoadConfig::Ring { length_m: 260.0, lanes: 1, speed_limit_kmh: 50.0 };
    let mut human_only = World::new(&sc, 0.0);
    let mut with_auto = World::new(&sc, 0.0);
    human_only.run_for(300.0);
    with_auto.run_for(300.0);
    assert_eq!(human_only.state_hash(), with_auto.state_hash());
    with_auto.make_auto(0);
    let (mut sd_h, mut sd_a, mut n) = (0.0, 0.0, 0);
    for t in 0..600 {
        human_only.run_for(1.0);
        with_auto.run_for(1.0);
        if t >= 300 {
            sd_h += std_dev(&human_only.veh.v);
            sd_a += std_dev(&with_auto.veh.v);
            n += 1;
        }
    }
    let (sd_h, sd_a) = (sd_h / n as f64, sd_a / n as f64);
    eprintln!("speed sd: humans only {sd_h:.2} m/s, with one auto {sd_a:.2} m/s");
    assert!(sd_h > 1.0, "the human ring should be in stop-and-go (sd {sd_h:.2})");
    assert!(sd_a < 0.6 * sd_h, "one auto should damp the waves: sd {sd_a:.2} vs {sd_h:.2}");
    assert!(with_auto.mean_speed() > 0.9 * human_only.mean_speed(), "without costing much throughput");
    assert_eq!(with_auto.stats.collisions, 0);
}

#[test]
fn comms_outage_causes_no_collisions() {
    let mut sc = scenario("city_a_to_b");
    sc.mix.left = 1.0;
    sc.mix.right = 1.0;
    let mut sim = Sim::new(&sc);
    sim.run_for(600.0);
    let cacc = |w: &World| w.veh.agent.iter().filter(|a| a.auto.mode == traffic_core::auto::AutoMode::Cacc).count();
    assert!(cacc(&sim.left) > 500, "platoons should be running CACC ({})", cacc(&sim.left));
    // Left: corridor-wide outage. Right: outage on the eastbound half only.
    let t = sim.time();
    sim.left.bus.up = false;
    sim.right.bus.outages.push(Outage { s0: 0.0, s1: sim.right.road.corridor, until: t + 300.0 });
    // Stress it: hard brakes in the middle of dense platoons during the outage.
    for w in [&mut sim.left, &mut sim.right] {
        for slot in [10, 500, 1000] {
            w.veh.agent[slot].forced = Some(Forced { target: 5.0, decel: 4.0, until: t + 10.0 });
        }
    }
    sim.run_for(30.0);
    assert_eq!(cacc(&sim.left), 0, "with the bus down every auto falls back to ACC");
    sim.run_for(270.0);
    sim.left.bus.up = true;
    sim.run_for(300.0);
    assert_eq!(sim.left.stats.collisions, 0);
    assert_eq!(sim.right.stats.collisions, 0);
    assert!(cacc(&sim.left) > 500, "CACC resumes when the bus returns");
}
