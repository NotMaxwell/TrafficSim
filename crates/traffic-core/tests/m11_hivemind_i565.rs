//! Hive-mind lane discipline, bus-informed early merging, and the I-565
//! scenario.

mod common;
use common::*;
use traffic_core::events::{EventKind, EventSpec, Pos};
use traffic_core::road::Dir;
use traffic_core::step::desired_speed;
use traffic_core::{Sim, World};

#[test]
fn autos_make_no_discretionary_lane_changes() {
    let sc = scenario("i565_decatur_huntsville");
    let mut sim = Sim::new(&sc);
    sim.run_for(1800.0);
    let (h, a) = (&sim.left.stats, &sim.right.stats);
    eprintln!(
        "humans: {} lane changes ({} discretionary); autos: {} lane changes ({} discretionary), {:.2} per auto",
        h.lane_changes,
        h.discretionary_human,
        a.lane_changes,
        a.discretionary_auto,
        a.lane_changes as f64 / a.autos_spawned as f64
    );
    assert!(h.discretionary_human > 1000, "humans overtake and keep right");
    assert_eq!(a.discretionary_auto, 0, "the hive mind never changes lanes without a reason");
    assert!(a.lane_changes < h.lane_changes / 2);
}

/// Autos learn about a closed lane from the bus and leave it well before
/// reaching it, so traffic passes the crash without slowing down.
#[test]
fn autos_clear_a_closed_lane_early_with_no_slowdown() {
    let mut sc = scenario("city_a_to_b");
    sc.demand.city_a_vph = 3000.0;
    sc.demand.profile_a.clear();
    sc.demand.profile_b.clear();
    sc.events.push(EventSpec {
        t: 600.0,
        kind: EventKind::Crash { at: Pos::Corridor { dir: Dir::East, mile: 10.0 }, lanes: vec![1], clear_s: 3600.0 },
    });
    for (share, label) in [(1.0, "autos"), (0.0, "humans")] {
        let mut w = World::new(&sc, share);
        let crash = w.road.from_corridor(Dir::East, 10.0);
        // Speed relative to each vehicle's own desired speed (a truck at its
        // 95 km/h cruise is not slowed down), within 1 km upstream.
        let speed_near = |w: &World| {
            let v: Vec<f64> = (0..w.veh.len())
                .filter(|&i| !w.veh.agent[i].crashed() && w.road.dist(w.veh.s[i], crash) < 1000.0 && w.veh.lane[i] > 0)
                .map(|i| w.veh.v[i] / desired_speed(w, i))
                .collect();
            (v.iter().sum::<f64>() / v.len().max(1) as f64, v.iter().cloned().fold(f64::INFINITY, f64::min))
        };
        w.run_for(540.0);
        let (before, _) = speed_near(&w);
        w.run_for(360.0); // crash at 600 s, then 5 minutes to settle
        let (mut worst_mean, mut worst_min, mut in_lane) = (f64::INFINITY, f64::INFINITY, 0);
        for _ in 0..90 {
            w.run_for(10.0);
            let (m, lo) = speed_near(&w);
            worst_mean = worst_mean.min(m);
            worst_min = worst_min.min(lo);
            in_lane += (0..w.veh.len())
                .filter(|&i| w.veh.lane[i] == 1 && !w.veh.agent[i].crashed() && w.road.dist(w.veh.s[i], crash) < 500.0)
                .count();
        }
        eprintln!(
            "{label}: speed / desired speed within 1 km upstream: {:.0}% before, worst {:.0}% after; slowest vehicle {:.0}%; samples in the closed lane within 500 m: {in_lane}",
            before * 100.0,
            worst_mean * 100.0,
            worst_min * 100.0
        );
        if share == 1.0 {
            assert_eq!(in_lane, 0, "autos should have left the closed lane 500 m before the crash");
            assert!(worst_mean > 0.95 * before, "no slowdown approaching the crash");
            assert!(worst_min > 0.8 * before, "nobody brakes hard for it");
            let crashes: Vec<_> = w.metrics.crashes.iter().map(|c| (c.0.round(), w.road.to_corridor(c.1).map(|x| (x.1 * 100.0).round() / 100.0))).collect();
            assert!(crashes.iter().all(|c| c.0 == 600.0), "only the scripted crash: {crashes:?}");
        }
    }
}

#[test]
fn i565_geometry_matches_the_real_highway() {
    let sc = scenario("i565_decatur_huntsville");
    let w = World::new(&sc, 0.0);
    let r = &w.road;
    assert_eq!(r.city_a, "Decatur (I-65)");
    assert!((r.corridor / traffic_core::road::MILE - 22.1).abs() < 1e-9);
    for (dir, mile, lanes) in [(Dir::East, 4.0, 3), (Dir::East, 8.0, 2), (Dir::West, 8.0, 2), (Dir::East, 12.0, 3), (Dir::West, 20.5, 2)] {
        assert_eq!(r.lanes_at(r.from_corridor(dir, mile)), lanes, "{dir:?} mile {mile}");
    }
    let mph = |dir, mile| r.limit_at(r.from_corridor(dir, mile)) / traffic_core::road::MPH;
    assert!((mph(Dir::East, 10.0) - 70.0).abs() < 1e-6 && (mph(Dir::West, 20.0) - 65.0).abs() < 1e-6);
    // Exit 13 is a westbound exit and an eastbound entrance only.
    let names: Vec<&str> = r.ramps.iter().map(|x| x.name.as_str()).collect();
    assert!(names.contains(&"Exit 13 Madison Blvd on (EB)") && names.contains(&"Exit 13 Madison Blvd off (WB)"));
    assert!(!names.contains(&"Exit 13 Madison Blvd off (EB)") && !names.contains(&"Exit 13 Madison Blvd on (WB)"));
    assert_eq!(r.ramps.iter().filter(|x| x.name.starts_with("Exit")).count(), 4 * 14 - 2);
}

#[test]
fn i565_morning_peak_runs_cleanly() {
    let sc = scenario("i565_decatur_huntsville");
    let mut sim = Sim::new(&sc);
    sim.run_for(3600.0);
    let (h, a) = (&sim.left, &sim.right);
    eprintln!(
        "after 1 h: humans {:.0} km/h, {} crashes, {} queued; autos {:.0} km/h, {} crashes, {} queued",
        h.mean_speed() * 3.6,
        h.stats.collisions,
        h.queued(),
        a.mean_speed() * 3.6,
        a.stats.collisions,
        a.queued()
    );
    assert_eq!(a.stats.collisions, 0);
    assert!(a.mean_speed() > h.mean_speed());
    assert!(a.stats.removed > h.stats.removed, "autos deliver more trips");
    assert!(h.stats.collisions <= 5, "human crashes stay rare ({})", h.stats.collisions);
    for w in [h, a] {
        assert!((0..w.veh.len()).all(|i| w.road.lane_exists(w.veh.s[i], w.veh.lane[i])));
    }
}
