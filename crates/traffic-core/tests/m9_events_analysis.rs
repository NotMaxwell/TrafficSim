//! Milestone 9 gate: events, detectors, space-time grid, CSV export.

mod common;
use common::*;
use traffic_core::events::{EventKind, EventSpec, Pos};
use traffic_core::metrics::{rows_from_csv, rows_to_csv};
use traffic_core::road::{Dir, RampKind};
use traffic_core::{Scenario, World};

fn steady(sc: &mut Scenario) {
    sc.demand.city_a_vph = 3000.0;
    sc.demand.city_b_vph = 1500.0;
    sc.demand.profile_a.clear();
    sc.demand.profile_b.clear();
}

/// Eastbound exits (veh/h) over [t0, t1), sampled from per-ramp counters.
fn eb_exit_rate(w: &mut World, t0: f64, t1: f64) -> f64 {
    let eb: Vec<usize> = w.road.ramps.iter().filter(|r| r.kind == RampKind::Off && r.dir == Dir::East).map(|r| r.id as usize).collect();
    let count = |w: &World| eb.iter().map(|&k| w.demand.exited[k]).sum::<u64>();
    w.run_for(t0 - w.time);
    let c0 = count(w);
    w.run_for(t1 - t0);
    (count(w) - c0) as f64 * 3600.0 / (t1 - t0)
}

#[test]
fn crash_drops_throughput_and_it_recovers_after_clearing() {
    let mut sc = scenario("city_a_to_b");
    steady(&mut sc);
    sc.events.push(EventSpec {
        t: 1800.0,
        kind: EventKind::Crash { at: Pos::Corridor { dir: Dir::East, mile: 10.0 }, lanes: vec![1, 2], clear_s: 900.0 },
    });
    let mut w = World::new(&sc, 0.0);
    let before = eb_exit_rate(&mut w, 1200.0, 1800.0);
    let during = eb_exit_rate(&mut w, 2100.0, 2700.0);
    let after = eb_exit_rate(&mut w, 4800.0, 5400.0);
    eprintln!("eastbound exits veh/h: before {before:.0}, during crash {during:.0}, after clearing {after:.0}");
    assert!(during < 0.85 * before, "a two-lane blockage should cut throughput");
    assert!(after > 0.9 * before, "throughput should recover once cleared");
    assert!(w.events.log.iter().any(|e| e.1 == "crash"));
    assert!(w.events.closures.is_empty(), "the crash should be cleared");
}

#[test]
fn detector_counts_match_vehicle_passages() {
    let sc = scenario("city_a_to_b");
    let mut w = World::new(&sc, 0.5);
    let dets: Vec<f64> = w.metrics.detectors.iter().map(|d| d.s).collect();
    let mut manual = vec![0u64; dets.len()];
    let len = w.road.length;
    for _ in 0..6000 {
        let before: std::collections::HashMap<u64, f64> = (0..w.veh.len()).map(|i| (w.veh.id[i], w.veh.s[i])).collect();
        w.run_ticks(1);
        for i in 0..w.veh.len() {
            let Some(&a) = before.get(&w.veh.id[i]) else { continue };
            let b = w.veh.s[i];
            let total = (b - a).rem_euclid(len);
            for (k, &x) in dets.iter().enumerate() {
                let dx = (x - a).rem_euclid(len);
                if dx > 0.0 && dx <= total {
                    manual[k] += 1;
                }
            }
        }
    }
    let counted: Vec<u64> = w.metrics.detectors.iter().map(|d| d.count).collect();
    assert_eq!(counted, manual);
    assert!(counted.iter().sum::<u64>() > 10_000);
    // Fundamental-diagram samples exist and are sane.
    let fd = &w.metrics.samples;
    assert!(fd.len() >= 10 * dets.len());
    assert!(fd.iter().all(|s| s.flow_vphpl >= 0.0 && s.flow_vphpl < 4000.0));
}

#[test]
fn csv_round_trips() {
    let sc = scenario("city_a_to_b");
    let mut w = World::new(&sc, 0.3);
    w.run_for(900.0);
    let rows = &w.metrics.rows;
    assert_eq!(rows.len(), 15);
    let back = rows_from_csv(&rows_to_csv(rows)).unwrap();
    assert_eq!(back.len(), rows.len());
    let same = |a: f64, b: f64| a.to_bits() == b.to_bits() || (a.is_nan() && b.is_nan());
    for (a, b) in rows.iter().zip(&back) {
        let (fa, fb) = (format!("{a:?}"), format!("{b:?}"));
        assert!(
            same(a.t, b.t) && same(a.mean_speed_kmh, b.mean_speed_kmh) && same(a.travel_med_min, b.travel_med_min) && same(a.fuel_proxy, b.fuel_proxy),
            "{fa} != {fb}"
        );
    }
    assert!(rows_from_csv("bad,header\n1,2").is_err());
}

#[test]
fn events_hit_both_panes_and_scenarios_with_events_round_trip() {
    let mut sc = scenario("city_a_to_b");
    sc.events.push(EventSpec {
        t: 120.0,
        kind: EventKind::Weather { at: Pos::Corridor { dir: Dir::West, mile: 15.0 }, to_mile: 5.0, speed_factor: 0.7, gap_factor: 1.3, duration_s: 600.0 },
    });
    sc.events.push(EventSpec { t: 60.0, kind: EventKind::DemandSurge { ramp: "City A on (EB)".into(), extra_vph: 1200.0, duration_s: 300.0 } });
    let text = sc.to_toml().unwrap();
    let back = Scenario::from_toml(&text).unwrap();
    assert_eq!(back.events, sc.events);
    let mut sim = traffic_core::Sim::new(&back);
    sim.inject(EventKind::HardBrake { at: Pos::Corridor { dir: Dir::East, mile: 4.0 }, target_kmh: 20.0, duration_s: 8.0, decel: 4.0 });
    sim.inject(EventKind::LaneClosure { at: Pos::Corridor { dir: Dir::East, mile: 12.0 }, to_mile: 13.0, lanes: vec![3], duration_s: 600.0 });
    sim.run_for(200.0);
    for w in [&sim.left, &sim.right] {
        let kinds: Vec<&str> = w.events.log.iter().map(|e| e.1.as_str()).collect();
        assert_eq!(kinds, ["hard brake", "lane closure", "demand surge", "weather"]);
        assert_eq!(w.events.weather.len(), 1);
        assert_eq!(w.events.closures.len(), 1);
    }
    // Nobody drives in the closed lane inside the work zone after a while.
    sim.run_for(200.0);
    let w = &sim.left;
    let c = &w.events.closures[0];
    let inside = (0..w.veh.len()).filter(|&i| w.veh.lane[i] == 3 && w.road.dist(c.s0, w.veh.s[i]) < w.road.dist(c.s0, c.s1) - 50.0).count();
    assert_eq!(inside, 0, "vehicles in the closed lane");
    let _ = std_dev(&[1.0]);
}

#[test]
fn bundled_scenarios_parse() {
    for name in ["city_a_to_b", "rush_hour_incidents", "sugiyama_ring", "single_av_ring"] {
        let sc = scenario(name);
        let w = World::new(&sc, 0.5);
        assert!(!w.veh.is_empty() || w.road.is_corridor(), "{name}");
    }
    assert_eq!(scenario("rush_hour_incidents").events.len(), 4);
}
