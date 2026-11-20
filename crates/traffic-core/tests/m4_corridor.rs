//! Milestone 4 gate: multi-lane MOBIL, 40-mile ring, city ramps, demand.

mod common;
use common::*;
use traffic_core::params::HumanParams;
use traffic_core::road::RampKind;
use traffic_core::World;

fn corridor_ideal() -> traffic_core::Scenario {
    let mut sc = scenario("city_a_to_b");
    sc.human = HumanParams::ideal();
    sc
}

#[test]
fn vehicle_count_is_conserved() {
    let sc = corridor_ideal();
    let mut w = World::new(&sc, 0.0);
    let initial = w.stats.spawned;
    assert!(initial > 1000, "corridor should start populated ({initial})");
    for _ in 0..20 {
        w.run_for(60.0);
        assert_eq!(w.stats.spawned, w.stats.removed + w.veh.len() as u64, "spawned = removed + present");
        let entered: u64 = w.demand.ramps.iter().map(|r| r.entered).sum();
        assert_eq!(w.stats.spawned, initial + entered);
        for r in &w.demand.ramps {
            assert_eq!(r.arrived, r.entered + r.queue.len() as u64, "arrivals = entered + queued");
        }
    }
    assert!(w.stats.removed > 500, "vehicles should be leaving at exits ({})", w.stats.removed);
    let interchange_exits: u64 = w
        .road
        .ramps
        .iter()
        .filter(|r| r.kind == RampKind::Off && !r.city)
        .map(|r| w.demand.exited[r.id as usize])
        .sum();
    assert!(interchange_exits > 50, "interchange off-ramps should be used ({interchange_exits})");
}

#[test]
fn no_lane_change_forces_braking_beyond_b_safe() {
    let sc = corridor_ideal();
    let mut w = World::new(&sc, 0.0);
    w.run_for(1800.0);
    assert!(w.stats.lane_changes > 1000, "expected plenty of lane changes, got {}", w.stats.lane_changes);
    assert_eq!(w.stats.unsafe_lane_changes, 0);
    assert_eq!(w.stats.collisions, 0);
}

#[test]
fn free_flow_ramp_throughput_matches_demand() {
    let mut sc = corridor_ideal();
    sc.demand.city_a_vph = 1800.0;
    sc.demand.city_b_vph = 1800.0;
    sc.demand.interchange_vph = 600.0;
    sc.demand.profile_a.clear();
    sc.demand.profile_b.clear();
    sc.population.initial_density = 8.0;
    let mut w = World::new(&sc, 0.0);
    let hours = 2.0;
    w.run_for(hours * 3600.0);
    for (k, r) in w.demand.ramps.iter().enumerate() {
        let expected = w.demand.rate(k, 0.0) * hours;
        let err = (r.entered as f64 - expected).abs() / expected;
        let name = &w.road.ramps[r.ramp as usize].name;
        assert!(err < 0.05, "{name}: entered {} vs demand {expected:.0} ({:.1}%)", r.entered, err * 100.0);
        assert!(r.queue.len() < 10, "{name}: queue {} in free flow", r.queue.len());
    }
    let mean_kmh = w.mean_speed() * 3.6;
    assert!(mean_kmh > 85.0, "free flow should stay fast, mean {mean_kmh:.0} km/h");
}
