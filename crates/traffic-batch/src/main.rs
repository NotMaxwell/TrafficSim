//! traffic-batch: headless runs and parameter sweeps.
//!
//!   traffic-batch run <scenario.toml> [--minutes N] [--out DIR]
//!   traffic-batch sweep <scenario.toml> [--minutes N] [--step PCT] [--warmup MIN] [--out FILE]
//!   traffic-batch validate

use std::path::{Path, PathBuf};
use std::process::ExitCode;
use std::time::Instant;

use traffic_core::metrics::{detector_samples_to_csv, rows_to_csv};
use traffic_core::sweep::{SweepRow, summarise, sweep, sweep_to_csv};
use traffic_core::{Scenario, Sim};

const USAGE: &str = "usage:
  traffic-batch run <scenario.toml> [--minutes N] [--out DIR]
      Run both panes of the scenario headless; write summary and detector CSVs.
  traffic-batch sweep <scenario.toml> [--minutes N] [--step PCT] [--warmup MIN] [--out FILE]
      Run one world per auto share (0..100% in PCT steps) in parallel; write one CSV row each.
  traffic-batch validate
      Print the key validation numbers from the bundled ring scenarios.";

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let res = match args.first().map(String::as_str) {
        Some("run") => run(&args[1..]),
        Some("sweep") => do_sweep(&args[1..]),
        Some("validate") => validate(),
        _ => Err(USAGE.to_string()),
    };
    match res {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("{e}");
            ExitCode::FAILURE
        }
    }
}

fn opt<T: std::str::FromStr>(args: &[String], key: &str) -> Result<Option<T>, String> {
    match args.iter().position(|a| a == key) {
        None => Ok(None),
        Some(i) => {
            let v = args.get(i + 1).ok_or(format!("{key} needs a value"))?;
            v.parse().map(Some).map_err(|_| format!("bad value for {key}: {v}"))
        }
    }
}

fn load(args: &[String]) -> Result<(Scenario, PathBuf), String> {
    let path = PathBuf::from(args.first().filter(|a| !a.starts_with("--")).ok_or(USAGE)?);
    let text = std::fs::read_to_string(&path).map_err(|e| format!("{}: {e}", path.display()))?;
    let sc = Scenario::from_toml(&text).map_err(|e| format!("{}: {e}", path.display()))?;
    Ok((sc, path))
}

fn write(path: &Path, text: &str) -> Result<(), String> {
    if let Some(dir) = path.parent() {
        std::fs::create_dir_all(dir).map_err(|e| format!("{}: {e}", dir.display()))?;
    }
    std::fs::write(path, text).map_err(|e| format!("{}: {e}", path.display()))
}

fn print_row(label: &str, r: &SweepRow) {
    println!(
        "{label:>8} {:>6.0} veh/h  travel {:>5.1}/{:>5.1} min  {:>5.1} km/h  {:>6.0} hard brakes/h  {:>3} crashes  fuel {:>6.0}  queued {}",
        r.throughput_vph, r.travel_med_min, r.travel_p90_min, r.mean_speed_kmh, r.hard_brakes_per_h, r.collisions, r.fuel_proxy, r.queued_end
    );
}

fn run(args: &[String]) -> Result<(), String> {
    let (sc, path) = load(args)?;
    let minutes: f64 = opt(args, "--minutes")?.unwrap_or(sc.duration_s / 60.0);
    let stem = path.file_stem().and_then(|s| s.to_str()).unwrap_or("run");
    let out: PathBuf = opt(args, "--out")?.unwrap_or_else(|| PathBuf::from("exports").join(stem));
    let t0 = Instant::now();
    let mut sim = Sim::new(&sc);
    sim.run_for(minutes * 60.0);
    println!("{}: {minutes} min simulated in {:.1} s", sc.name, t0.elapsed().as_secs_f64());
    for (name, w) in [("left", &sim.left), ("right", &sim.right)] {
        print_row(&format!("{name} {:.0}%", w.auto_share * 100.0), &summarise(w, w.auto_share, 0.0));
        write(&out.join(format!("{name}_summary.csv")), &rows_to_csv(&w.metrics.rows))?;
        write(&out.join(format!("{name}_detectors.csv")), &detector_samples_to_csv(&w.metrics))?;
    }
    write(&out.join("scenario.toml"), &sim.scenario.to_toml()?)?;
    println!("wrote {}", out.display());
    Ok(())
}

fn do_sweep(args: &[String]) -> Result<(), String> {
    let (sc, path) = load(args)?;
    let minutes: f64 = opt(args, "--minutes")?.unwrap_or(sc.duration_s / 60.0);
    let step: f64 = opt(args, "--step")?.unwrap_or(10.0);
    let warmup: f64 = opt(args, "--warmup")?.unwrap_or((minutes / 6.0).min(15.0));
    if !(step > 0.0 && step <= 100.0) {
        return Err("--step must be in (0, 100]".into());
    }
    let stem = path.file_stem().and_then(|s| s.to_str()).unwrap_or("sweep");
    let out: PathBuf = opt(args, "--out")?.unwrap_or_else(|| PathBuf::from("exports").join(format!("{stem}_sweep.csv")));
    let n = (100.0 / step).round() as usize;
    let shares: Vec<f64> = (0..=n).map(|k| (k as f64 * step / 100.0).min(1.0)).collect();
    println!("{}: {} runs of {minutes} min (warm-up {warmup} min) on {} threads", sc.name, shares.len(), rayon::current_num_threads());
    let t0 = Instant::now();
    let rows = sweep(&sc, &shares, minutes * 60.0, warmup * 60.0);
    for r in &rows {
        print_row(&format!("{:.0}%", r.auto_share * 100.0), r);
    }
    write(&out, &sweep_to_csv(&rows))?;
    println!("wrote {} in {:.1} s", out.display(), t0.elapsed().as_secs_f64());
    Ok(())
}

fn validate() -> Result<(), String> {
    let dir = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../scenarios");
    for name in ["sugiyama_ring", "single_av_ring"] {
        let (sc, _) = load(&[dir.join(format!("{name}.toml")).display().to_string()])?;
        let mut sim = Sim::new(&sc);
        sim.run_for(600.0);
        let sd = |w: &traffic_core::World| {
            let m = w.mean_speed();
            (w.veh.v.iter().map(|v| (v - m).powi(2)).sum::<f64>() / w.veh.len() as f64).sqrt()
        };
        println!(
            "{:<28} left: mean {:>5.1} km/h, speed sd {:.2} m/s | right: mean {:>5.1} km/h, speed sd {:.2} m/s",
            sc.name,
            sim.left.mean_speed() * 3.6,
            sd(&sim.left),
            sim.right.mean_speed() * 3.6,
            sd(&sim.right)
        );
    }
    println!("Full validation suite: cargo test --release -p traffic-core");
    Ok(())
}
