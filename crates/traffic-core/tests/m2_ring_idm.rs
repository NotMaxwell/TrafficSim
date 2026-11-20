//! Milestone 2 gate: single-lane ring, IDM, ballistic integration, seeded RNG.

mod common;
use common::*;
use traffic_core::World;

#[test]
fn sugiyama_phantom_jam_forms_and_moves_backward() {
    let sc = scenario("sugiyama_ring");
    let mut w = World::new(&sc, 0.0);
    assert_eq!(w.veh.len(), 22);
    w.run_for(400.0);
    let min_v = w.veh.v.iter().cloned().fold(f64::INFINITY, f64::min);
    let sd = std_dev(&w.veh.v);
    let mean = w.mean_speed();
    assert!(min_v < 1.0, "a jam should bring some car nearly to a stop (min {min_v:.2} m/s)");
    assert!(sd > 1.5, "speeds should be strongly non-uniform (sd {sd:.2}, mean {mean:.2})");
    assert!(mean > 2.0, "traffic still flows overall (mean {mean:.2})");
    let vj = jam_velocity(&mut w, 120.0);
    assert!(vj < -1.0, "the jam should travel backward, got {vj:.2} m/s");
    assert_eq!(w.stats.collisions, 0);
}

#[test]
fn same_seed_gives_identical_output() {
    let sc = scenario("sugiyama_ring");
    let mut a = World::new(&sc, 0.0);
    let mut b = World::new(&sc, 0.0);
    a.run_for(300.0);
    b.run_for(300.0);
    assert_eq!(a.state_hash(), b.state_hash());
    assert_eq!(a.veh.s, b.veh.s);

    let mut other = sc.clone();
    other.seed += 1;
    let mut c = World::new(&other, 0.0);
    c.run_for(300.0);
    assert_ne!(a.state_hash(), c.state_hash(), "a different seed should change the noise");
}

#[test]
fn speeds_never_negative_and_no_overlaps() {
    let sc = scenario("sugiyama_ring");
    let mut w = World::new(&sc, 0.0);
    for _ in 0..6000 {
        w.run_ticks(1);
        for i in 0..w.veh.len() {
            assert!(w.veh.v[i] >= 0.0, "negative speed at t={}", w.time);
            let l = w.veh.leader(i).unwrap();
            let gap = w.road.dist(w.veh.s[i], w.veh.s[l]) - w.veh.len[l];
            assert!(gap >= 0.0, "overlap {gap} at t={}", w.time);
        }
    }
    assert_eq!(w.stats.collisions, 0);
}

#[test]
fn uniform_flow_is_steady_without_noise() {
    // Control: the same ring at low density with no noise stays uniform.
    let mut sc = scenario("sugiyama_ring");
    sc.human.accel_noise = 0.0;
    sc.population.count = 8;
    sc.population.initial_speed = -1.0;
    let mut w = World::new(&sc, 0.0);
    w.run_for(300.0);
    assert!(std_dev(&w.veh.v) < 0.05);
}
