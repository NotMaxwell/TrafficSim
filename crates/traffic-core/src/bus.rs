//! Shared state bus for automated vehicles (V2V).
//!
//! Every tick each auto publishes {id, s, lane, v, a, u, intent}. Readers
//! see the snapshot published `latency` ago (the history is a short queue),
//! with optional packet loss, a broadcast range, and outages. Snapshots
//! also carry mean speed per 500 m bin so "what is happening 2 km ahead"
//! is a cheap lookup.

use std::collections::VecDeque;

use rand::Rng;
use rand_chacha::ChaCha8Rng;
use serde::{Deserialize, Serialize};

use crate::rng::{self, Purpose};
use crate::vehicles::Kind;
use crate::World;

pub const BIN: f64 = 500.0;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct BusParams {
    /// V2V message delay, s.
    pub latency_s: f64,
    /// Probability a message is lost.
    pub loss: f64,
    /// Broadcast range, m (0 = whole corridor).
    pub range_m: f64,
    /// Roadside sensors publish human vehicles too (positions only).
    pub roadside_sensing: bool,
}

impl Default for BusParams {
    fn default() -> Self {
        BusParams { latency_s: 0.1, loss: 0.0, range_m: 0.0, roadside_sensing: false }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BusEntry {
    pub id: u64,
    pub s: f64,
    pub lane: u8,
    pub v: f64,
    pub a: f64,
    /// Commanded acceleration (CACC feed-forward signal).
    pub u: f64,
    /// Planned lateral move: -1 right, +1 left, 0 none.
    pub intent: i8,
    pub auto: bool,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct BusSnap {
    /// Sorted by id.
    pub entries: Vec<BusEntry>,
    pub bin_v: Vec<f32>,
    pub bin_n: Vec<u16>,
}

impl BusSnap {
    pub fn get(&self, id: u64) -> Option<&BusEntry> {
        self.entries.binary_search_by_key(&id, |e| e.id).ok().map(|k| &self.entries[k])
    }

    /// Mean speed in bin `k`, if anything published there.
    pub fn bin_speed(&self, k: usize) -> Option<f64> {
        let n = *self.bin_n.get(k)?;
        (n > 0).then(|| self.bin_v[k] as f64 / n as f64)
    }
}

/// A segment of road where the bus is down until `until`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Outage {
    pub s0: f64,
    pub s1: f64,
    pub until: f64,
}

#[derive(Clone, Debug)]
pub struct Bus {
    pub params: BusParams,
    history: VecDeque<BusSnap>,
    lat_ticks: usize,
    rng: ChaCha8Rng,
    pub outages: Vec<Outage>,
    /// Corridor-wide kill switch.
    pub up: bool,
}

impl Bus {
    pub fn new(params: &BusParams, dt: f64, seed: u64) -> Bus {
        Bus {
            params: params.clone(),
            history: VecDeque::new(),
            lat_ticks: (params.latency_s / dt).round().max(0.0) as usize,
            rng: rng::stream(seed, Purpose::Bus, 0),
            outages: Vec::new(),
            up: true,
        }
    }

    /// The snapshot readers see now (published `latency` ago).
    pub fn view(&self) -> Option<&BusSnap> {
        self.history.front()
    }

    /// Is the bus working at position s?
    pub fn up_at(&self, s: f64) -> bool {
        self.up && !self.outages.iter().any(|o| if o.s0 <= o.s1 { s >= o.s0 && s < o.s1 } else { s >= o.s0 || s < o.s1 })
    }
}

/// Step 2: every auto (and, with roadside sensing, every human) publishes.
pub fn publish(w: &mut World) {
    let time = w.time;
    w.bus.outages.retain(|o| o.until > time);
    let nbins = (w.road.length / BIN).ceil() as usize;
    let mut snap = BusSnap { entries: Vec::new(), bin_v: vec![0.0; nbins], bin_n: vec![0; nbins] };
    let roadside = w.bus.params.roadside_sensing;
    let loss = w.bus.params.loss;
    for i in 0..w.veh.len() {
        let auto = w.veh.kind[i] == Kind::Auto;
        if !auto && !roadside {
            continue;
        }
        let s = w.veh.s[i];
        if auto && !w.bus.up_at(s) {
            continue;
        }
        if loss > 0.0 && w.bus.rng.random::<f64>() < loss {
            continue;
        }
        let ag = &w.veh.agent[i];
        snap.entries.push(BusEntry {
            id: w.veh.id[i],
            s,
            lane: w.veh.lane[i],
            v: w.veh.v[i],
            a: w.veh.a[i],
            // An override (hard-brake event) is what the car really does.
            u: if auto && ag.forced.is_none() { ag.auto.u } else { w.veh.a[i] },
            intent: ag.auto.intent,
            auto,
        });
        let k = ((s / BIN) as usize).min(nbins - 1);
        snap.bin_v[k] += w.veh.v[i] as f32;
        snap.bin_n[k] += 1;
    }
    snap.entries.sort_unstable_by_key(|e| e.id);
    w.bus.history.push_back(snap);
    while w.bus.history.len() > w.bus.lat_ticks + 1 {
        w.bus.history.pop_front();
    }
}
