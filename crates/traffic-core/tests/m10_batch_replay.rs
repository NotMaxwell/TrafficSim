//! Milestone 10 gate: headless sweeps; save then load reproduces a run.

mod common;
use common::*;
use traffic_core::events::{EventKind, Pos};
use traffic_core::road::Dir;
use traffic_core::sweep::{sweep, sweep_to_csv};
use traffic_core::{Scenario, Side, Sim};

#[test]
fn auto_share_sweep_0_to_100_completes_headless() {
    let sc = scenario("city_a_to_b");
    let shares: Vec<f64> = (0..=10).map(|k| k as f64 / 10.0).collect();
    let rows = sweep(&sc, &shares, 900.0, 300.0);
    assert_eq!(rows.len(), 11);
    for (r, s) in rows.iter().zip(&shares) {
        assert_eq!(r.auto_share, *s);
        assert!(r.throughput_vph > 1000.0 && r.mean_speed_kmh > 20.0, "{r:?}");
    }
    let csv = sweep_to_csv(&rows);
    assert_eq!(csv.lines().count(), 12);
    eprintln!("{csv}");
}

#[test]
fn save_then_load_reproduces_the_run() {
    let sc = scenario("city_a_to_b");
    let mut original = Sim::new(&sc);
    original.run_for(120.0);
    original.inject(EventKind::HardBrake { at: Pos::Corridor { dir: Dir::East, mile: 3.0 }, target_kmh: 15.0, duration_s: 8.0, decel: 4.0 });
    original.set_auto_share(Side::Left, 0.4);
    original.run_for(200.0);
    original.inject(EventKind::Crash { at: Pos::Corridor { dir: Dir::West, mile: 12.0 }, lanes: vec![2], clear_s: 300.0 });
    original.set_auto_share(Side::Right, 0.6);
    original.run_for(280.0);

    let saved = original.scenario.to_toml().unwrap();
    let loaded = Scenario::from_toml(&saved).unwrap();
    assert_eq!(loaded.events.len(), 2);
    assert_eq!(loaded.mix_timeline.len(), 2);
    let mut replay = Sim::new(&loaded);
    replay.run_for(600.0);
    assert_eq!(replay.time(), original.time());
    assert_eq!(replay.left.state_hash(), original.left.state_hash());
    assert_eq!(replay.right.state_hash(), original.right.state_hash());
    assert_eq!(replay.left.auto_share, 0.4);
    assert_eq!(replay.right.auto_share, 0.6);
}
