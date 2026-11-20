//! Simulation thread. Owns the worlds, steps them in (scaled) real time,
//! and publishes an immutable `Frame` for the UI roughly 60 times a second.
//! The UI only sends commands and draws the latest frame.

use std::sync::mpsc::{Receiver, Sender, channel};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use traffic_core::snapshot::WorldSnap;
use traffic_core::{Road, Scenario, Side, Sim};

pub enum Cmd {
    Play(bool),
    Speed(f64),
    Load(Box<Scenario>),
    SetShare(Side, f64),
    StepOnce,
    Quit,
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
        };
        t.load(sc);
        t
    }

    fn load(&mut self, sc: Scenario) {
        self.sim = Sim::new(&sc);
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

    fn publish(&mut self) {
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
        };
        if let Ok(mut f) = self.out.lock() {
            *f = Arc::new(frame);
        }
        self.repaint.request_repaint();
    }
}
