//! Simulation thread. Owns the worlds, steps them in (scaled) real time,
//! and publishes an immutable `Frame` for the UI roughly 60 times a second.
//! The UI only sends commands and draws the latest frame.

use std::sync::mpsc::{Receiver, Sender, channel};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use traffic_core::snapshot::WorldSnap;
use traffic_core::events::EventKind;
use traffic_core::road::Dir;
use traffic_core::{Road, Scenario, Side, Sim, World, metrics, waves};

pub enum Cmd {
    Play(bool),
    Speed(f64),
    Load(Box<Scenario>),
    SetShare(Side, f64),
    Event(EventKind),
    /// Write CSVs of both worlds into this directory.
    Export(std::path::PathBuf),
    /// Save the scenario as run so far (events, slider changes) to a file.
    Save(std::path::PathBuf),
    StepOnce,
    Quit,
}

/// Wave analysis of one world, refreshed about once a second.
#[derive(Clone, Copy, Debug, Default)]
pub struct Analysis {
    /// Shockwave speed, km/h (negative = upstream), per direction.
    pub shock_eb: Option<f64>,
    pub shock_wb: Option<f64>,
    /// Amplification of the latest hard-brake disturbance.
    pub amplification: Option<f64>,
}

impl Analysis {
    fn of(w: &World) -> Analysis {
        let m = &w.metrics;
        let (eb, wb) = if w.road.is_corridor() { (Some(Dir::East), Some(Dir::West)) } else { (None, None) };
        let amplification = w
            .events
            .log
            .iter()
            .rev()
            .find(|e| e.1 == "hard brake")
            .and_then(|e| waves::amplification(m, &w.road, e.2, e.0, 600.0_f64.min(w.time - e.0 - 10.0)));
        Analysis {
            shock_eb: waves::shockwave_speed(m, &w.road, eb, 900.0),
            shock_wb: if w.road.is_corridor() { waves::shockwave_speed(m, &w.road, wb, 900.0) } else { None },
            amplification,
        }
    }
}

/// Everything the UI draws, published as one immutable unit.
#[derive(Default)]
pub struct Frame {
    pub worlds: Vec<WorldSnap>,
    /// Vehicles waiting in ramp queues, per world.
    pub queued: [usize; 2],
    pub road: Option<Arc<Road>>,
    pub playing: bool,
    /// Measured sim-seconds per real second.
    pub actual_speed: f64,
    /// Mean wall time of one tick (all worlds), ms.
    pub tick_ms: f64,
    pub analysis: [Analysis; 2],
    /// Result of the last CSV export.
    pub export_msg: Option<String>,
}

pub struct Runner {
    tx: Sender<Cmd>,
    frame: Arc<Mutex<Arc<Frame>>>,
    handle: Option<JoinHandle<()>>,
}

impl Runner {
    pub fn start(sc: Scenario, repaint: egui::Context) -> Runner {
        let (tx, rx) = channel();
        let frame = Arc::new(Mutex::new(Arc::new(Frame::default())));
        let out = frame.clone();
        let handle = std::thread::Builder::new()
            .name("sim".into())
            .spawn(move || SimThread::new(sc, rx, out, repaint).run())
            .expect("spawn sim thread");
        Runner { tx, frame, handle: Some(handle) }
    }

    pub fn send(&self, cmd: Cmd) {
        let _ = self.tx.send(cmd);
    }

    pub fn frame(&self) -> Arc<Frame> {
        self.frame.lock().map(|f| f.clone()).unwrap_or_default()
    }
}

impl Drop for Runner {
    fn drop(&mut self) {
        let _ = self.tx.send(Cmd::Quit);
        if let Some(h) = self.handle.take() {
            let _ = h.join();
        }
    }
}

struct SimThread {
    sim: Sim,
    road: Arc<Road>,
    rx: Receiver<Cmd>,
    out: Arc<Mutex<Arc<Frame>>>,
    repaint: egui::Context,
    playing: bool,
    speed: f64,
    measured: (Instant, f64),
    actual_speed: f64,
    tick_ms: f64,
    analysis: [Analysis; 2],
    analysed: Instant,
    export_msg: Option<String>,
}

impl SimThread {
    fn new(sc: Scenario, rx: Receiver<Cmd>, out: Arc<Mutex<Arc<Frame>>>, repaint: egui::Context) -> Self {
        let mut t = SimThread {
            sim: Sim::new(&sc),
            road: Arc::new(sc.road.build()),
            rx,
            out,
            repaint,
            playing: true,
            speed: 1.0,
            measured: (Instant::now(), 0.0),
            actual_speed: 0.0,
            tick_ms: 0.0,
            analysis: Default::default(),
            analysed: Instant::now(),
            export_msg: None,
        };
        t.load(sc);
        t
    }

    fn load(&mut self, sc: Scenario) {
        self.sim = Sim::new(&sc);
        self.analysis = Default::default();
        self.export_msg = None;
        self.road = Arc::new(self.sim.left.road.clone());
        self.measured = (Instant::now(), 0.0);
    }

    fn sim_time(&self) -> f64 {
        self.sim.time()
    }

    fn step_all(&mut self) {
        self.sim.step();
    }

    fn run(mut self) {
        let mut last = Instant::now();
        let mut last_publish = Instant::now() - Duration::from_secs(1);
        let mut owed = 0.0_f64;
        loop {
            while let Ok(cmd) = self.rx.try_recv() {
                match cmd {
                    Cmd::Play(p) => self.playing = p,
                    Cmd::Speed(s) => self.speed = s.clamp(0.05, 1000.0),
                    Cmd::Load(sc) => {
                        self.load(*sc);
                        owed = 0.0;
                        last_publish -= Duration::from_secs(1);
                    }
                    Cmd::SetShare(side, v) => self.sim.set_auto_share(side, v),
                    Cmd::Event(e) => self.sim.inject(e),
                    Cmd::Export(dir) => self.export_msg = Some(self.export(&dir)),
                    Cmd::Save(path) => {
                        let res = self.sim.scenario.to_toml().and_then(|t| std::fs::write(&path, t).map_err(|e| e.to_string()));
                        self.export_msg = Some(match res {
                            Ok(()) => format!("Scenario saved to {}", path.display()),
                            Err(e) => format!("Save failed: {e}"),
                        });
                    }
                    Cmd::StepOnce => self.step_all(),
                    Cmd::Quit => return,
                }
            }
            let now = Instant::now();
            let real = now.duration_since(last).as_secs_f64();
            last = now;
            let dt = self.sim.left.dt;
            if self.playing {
                // Never owe more than a quarter second of wall time: when
                // the sim cannot keep up it slows down instead of spiralling.
                owed = (owed + real * self.speed).min(0.25 * self.speed.max(1.0));
            } else {
                owed = 0.0;
            }
            let budget = Instant::now();
            let mut ran = 0u32;
            while owed >= dt && budget.elapsed() < Duration::from_millis(14) {
                self.step_all();
                owed -= dt;
                ran += 1;
            }
            if ran > 0 {
                let per = budget.elapsed().as_secs_f64() * 1000.0 / ran as f64;
                self.tick_ms = 0.9 * self.tick_ms + 0.1 * per;
            }
            if owed >= dt {
                owed = owed.min(dt * 4.0);
            }
            let since = self.measured.0.elapsed().as_secs_f64();
            if since >= 0.5 {
                self.actual_speed = (self.sim_time() - self.measured.1) / since;
                self.measured = (Instant::now(), self.sim_time());
            }
            if last_publish.elapsed() >= Duration::from_millis(15) {
                self.publish();
                last_publish = Instant::now();
            }
            if ran == 0 {
                std::thread::sleep(Duration::from_millis(2));
            }
        }
    }

    /// Write summary rows, detector samples and the scenario (with every
    /// injected event, so the run can be replayed) for both worlds.
    fn export(&self, dir: &std::path::Path) -> String {
        let run = dir.join(format!("run-t{:05.0}", self.sim.time()));
        let mut files = vec![(run.join("scenario.toml"), self.sim.scenario.to_toml().unwrap_or_default())];
        for (name, w) in [("left", &self.sim.left), ("right", &self.sim.right)] {
            files.push((run.join(format!("{name}_summary.csv")), metrics::rows_to_csv(&w.metrics.rows)));
            files.push((run.join(format!("{name}_detectors.csv")), metrics::detector_samples_to_csv(&w.metrics)));
        }
        let res = std::fs::create_dir_all(&run).and_then(|_| files.iter().try_for_each(|(p, c)| std::fs::write(p, c)));
        match res {
            Ok(()) => format!("Saved to {}", run.display()),
            Err(e) => format!("Export failed: {e}"),
        }
    }

    fn publish(&mut self) {
        if self.analysed.elapsed() >= Duration::from_secs(1) {
            self.analysis = [Analysis::of(&self.sim.left), Analysis::of(&self.sim.right)];
            self.analysed = Instant::now();
        }
        let worlds = [&self.sim.left, &self.sim.right]
            .into_iter()
            .map(|w| {
                let mut s = WorldSnap::default();
                w.snapshot_into(&mut s);
                s
            })
            .collect();
        let frame = Frame {
            worlds,
            queued: [self.sim.left.queued(), self.sim.right.queued()],
            road: Some(self.road.clone()),
            playing: self.playing,
            actual_speed: self.actual_speed,
            tick_ms: self.tick_ms,
            analysis: self.analysis,
            export_msg: self.export_msg.clone(),
        };
        if let Ok(mut f) = self.out.lock() {
            *f = Arc::new(frame);
        }
        self.repaint.request_repaint();
    }
}
