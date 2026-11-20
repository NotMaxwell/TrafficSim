//! Milestone 6 gate: two worlds stepped in parallel with a shared seed.

mod common;
use common::*;
use traffic_core::Sim;

#[test]
fn identical_settings_give_identical_panes() {
    let mut sc = scenario("city_a_to_b");
    sc.mix.left = 0.0;
    sc.mix.right = 0.0;
    let mut sim = Sim::new(&sc);
    for _ in 0..30 {
        sim.run_for(20.0);
        assert_eq!(sim.left.state_hash(), sim.right.state_hash(), "panes diverged at t={}", sim.time());
    }
    assert_eq!(sim.left.stats, sim.right.stats);
    assert!(sim.left.stats.lane_changes > 0 && sim.left.stats.spawned > 1000);
}

#[test]
fn parallel_run_matches_serial_run_bit_for_bit() {
    let mut sc = scenario("city_a_to_b");
    sc.mix.left = 0.0;
    sc.mix.right = 0.5;
    let mut par = Sim::new(&sc);
    let mut ser = Sim::new(&sc);
    for _ in 0..3000 {
        par.step();
        ser.step_serial();
    }
    assert_eq!(par.left.state_hash(), ser.left.state_hash());
    assert_eq!(par.right.state_hash(), ser.right.state_hash());
    assert_eq!(par.left.veh.s, ser.left.veh.s);
}

#[test]
fn common_random_numbers_across_panes() {
    // Same vehicle id arrives at the same time with the same traits in
    // both worlds, whatever the mix.
    let mut sc = scenario("city_a_to_b");
    sc.mix.left = 0.0;
    sc.mix.right = 1.0;
    let mut sim = Sim::new(&sc);
    sim.run_for(300.0);
    let arrivals = |w: &traffic_core::World| w.demand.ramps.iter().map(|r| r.arrived).collect::<Vec<_>>();
    assert_eq!(arrivals(&sim.left), arrivals(&sim.right));
    assert_eq!(sim.left.stats.spawned - sim.left.veh.len() as u64 - sim.left.stats.removed, 0);
}
