//! Milestone 8 gate: merge coordinator and mixed-traffic slider.

mod common;
use common::*;
use traffic_core::road::RampKind;
use traffic_core::scenario::RoadConfig;
use traffic_core::{Side, Sim, World};

/// Heavy merging: interchange ramps busy and city ends as ordinary
/// single-lane ramps, so every entering vehicle merges from lane 0.
fn merge_heavy() -> traffic_core::Scenario {
    let mut sc = scenario("city_a_to_b");
    if let RoadConfig::Corridor(g) = &mut sc.road {
        g.city_terminal = false;
    }
    sc.demand.city_a_vph = 1500.0;
    sc.demand.city_b_vph = 1500.0;
    sc.demand.interchange_vph = 900.0;
    sc.demand.profile_a.clear();
    sc.demand.profile_b.clear();
    sc
}

fn merges(w: &World) -> u64 {
    w.demand.ramps.iter().filter(|r| !w.road.ramps[r.ramp as usize].all_lanes).map(|r| r.entered).sum()
}

#[test]
fn zero_merge_collisions_at_full_automation() {
    let sc = merge_heavy();
    let mut w = World::new(&sc, 1.0);
    w.run_for(3600.0);
    let m = merges(&w);
    eprintln!("100% autos: {m} merges, {} collisions, queued {}", w.stats.collisions, w.queued());
    assert!(m > 5000, "expected heavy merging ({m})");
    assert_eq!(w.stats.collisions, 0);
    assert!(w.queued() < 50, "ramps should keep up ({} queued)", w.queued());
    assert!(w.road.ramps.iter().filter(|r| r.kind == RampKind::On).count() == 6);
}

#[test]
fn coordinator_helps_merging() {
    // Same demand: autos merge with shorter ramp queues than humans.
    let sc = merge_heavy();
    let mut sim = Sim::new(&sc);
    sim.set_auto_share(Side::Left, 0.0);
    sim.set_auto_share(Side::Right, 1.0);
    sim.run_for(2400.0);
    let (qh, qa) = (sim.left.queued(), sim.right.queued());
    eprintln!("queued after 40 min: humans {qh}, autos {qa}");
    assert!(qa <= qh, "autos should not queue more than humans ({qa} vs {qh})");
}

#[test]
fn auto_share_of_spawns_matches_slider() {
    let mut sc = scenario("city_a_to_b");
    sc.demand.multiplier = 2.0;
    // A shorter corridor runs faster; arrivals depend only on demand.
    if let RoadConfig::Corridor(g) = &mut sc.road {
        g.corridor_mi = 5.0;
        g.interchanges_mi.clear();
    }
    sc.population.initial_density = 5.0;
    for share in [0.3, 0.75] {
        let mut w = World::new(&sc, 0.0);
        w.run_for(60.0);
        w.auto_share = share;
        let (s0, a0) = (w.stats.spawned, w.stats.autos_spawned);
        while w.stats.spawned - s0 < 10_000 {
            w.run_for(30.0);
        }
        let n = (w.stats.spawned - s0) as f64;
        let frac = (w.stats.autos_spawned - a0) as f64 / n;
        eprintln!("slider {share}: {frac:.4} autos over {n} spawns");
        assert!((frac - share).abs() < 0.02, "auto share {frac:.3} vs slider {share}");
    }
}

#[test]
fn single_av_ring_scenario_damps_waves_in_the_right_pane() {
    let sc = scenario("single_av_ring");
    let mut sim = Sim::new(&sc);
    sim.run_for(300.0);
    let (mut l, mut r) = (0.0, 0.0);
    for _ in 0..300 {
        sim.run_for(1.0);
        l += std_dev(&sim.left.veh.v);
        r += std_dev(&sim.right.veh.v);
    }
    eprintln!("ring speed sd: left (humans) {:.2}, right (one auto) {:.2}", l / 300.0, r / 300.0);
    assert_eq!(sim.right.stats.autos_spawned, 1);
    assert!(r < 0.6 * l);
    assert_eq!(sim.left.stats.collisions + sim.right.stats.collisions, 0);
}
