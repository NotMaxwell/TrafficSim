//! Render snapshot: the minimal per-vehicle data the UI draws, copied out
//! of a world once per frame (not once per tick).

use std::sync::Arc;

use crate::metrics::Row;
use crate::vehicles::Kind;
use crate::{Stats, World};

pub const FLAG_AUTO: u8 = 1;
pub const FLAG_TRUCK: u8 = 2;
pub const FLAG_BRAKING: u8 = 4;
pub const FLAG_CRASHED: u8 = 8;
pub const FLAG_CACC: u8 = 16;
pub const FLAG_DISTRACTED: u8 = 32;
pub const FLAG_SIGNAL_LEFT: u8 = 64;
pub const FLAG_SIGNAL_RIGHT: u8 = 128;

/// 16 bytes per vehicle.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct VehDraw {
    pub s: f32,
    pub v: f32,
    pub len: f32,
    pub lat: f32,
    pub lane: u8,
    pub flags: u8,
    pub id: u32,
}

/// Metrics the plots need, copied once per frame (the grid columns are
/// shared, so copying them is cheap).
#[derive(Clone, Debug, Default, PartialEq)]
pub struct MetricsSnap {
    pub rows: Vec<Row>,
    /// Fundamental diagram points (density veh/km/lane, flow veh/h/lane)
    /// from the last 30 minutes of detector windows.
    pub fd: Vec<[f32; 2]>,
    pub grid: Vec<Arc<Vec<f32>>>,
    pub grid_t0: f64,
    pub grid_version: u64,
    pub crashes: Vec<(f64, f64)>,
    /// Started events: (time, label, s).
    pub events: Vec<(f64, String, f64)>,
    /// Active closures: (s0, s1, lanes, is_crash).
    pub closures: Vec<(f64, f64, Vec<u8>, bool)>,
    /// Active weather zones and comms outages: (s0, s1).
    pub weather: Vec<(f64, f64)>,
    pub outages: Vec<(f64, f64)>,
    pub trips: u64,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct WorldSnap {
    pub time: f64,
    pub tick: u64,
    pub vehicles: Vec<VehDraw>,
    pub stats: Stats,
    pub mean_speed: f64,
    pub auto_share: f64,
    pub metrics: MetricsSnap,
}

impl World {
    /// Copy the drawable state into `out`, reusing its allocation.
    pub fn snapshot_into(&self, out: &mut WorldSnap) {
        out.time = self.time;
        out.tick = self.tick;
        out.stats = self.stats.clone();
        out.mean_speed = self.mean_speed();
        out.auto_share = self.auto_share;
        let m = &self.metrics;
        let since = self.time - 1800.0;
        out.metrics = MetricsSnap {
            rows: m.rows.clone(),
            fd: m
                .samples
                .iter()
                .rev()
                .take_while(|d| d.t >= since)
                .filter(|d| d.flow_vphpl > 0.0)
                .map(|d| [d.density_vpkpl as f32, d.flow_vphpl as f32])
                .collect(),
            grid: m.grid.iter().cloned().collect(),
            grid_t0: m.grid_t0,
            grid_version: m.grid_version,
            crashes: m.crashes.clone(),
            events: self.events.log.clone(),
            closures: self.events.closures.iter().map(|c| (c.s0, c.s1, c.lanes.clone(), c.crash)).collect(),
            weather: self.events.weather.iter().map(|z| (z.s0, z.s1)).collect(),
            outages: self.bus.outages.iter().map(|o| (o.s0, o.s1)).collect(),
            trips: m.trips_total,
        };
        out.vehicles.clear();
        out.vehicles.extend((0..self.veh.len()).map(|i| {
            let ag = &self.veh.agent[i];
            let mut flags = 0;
            if self.veh.kind[i] == Kind::Auto {
                flags |= FLAG_AUTO;
                if ag.auto.mode == crate::auto::AutoMode::Cacc {
                    flags |= FLAG_CACC;
                }
            }
            if ag.truck {
                flags |= FLAG_TRUCK;
            }
            if self.veh.a[i] < -0.5 || (self.veh.v[i] < 0.1 && self.veh.a[i] <= 0.0) {
                flags |= FLAG_BRAKING;
            }
            if ag.crashed() {
                flags |= FLAG_CRASHED;
            }
            if ag.human.distracted_until > self.time {
                flags |= FLAG_DISTRACTED;
            }
            match ag.signal {
                1 => flags |= FLAG_SIGNAL_LEFT,
                -1 => flags |= FLAG_SIGNAL_RIGHT,
                _ => {}
            }
            VehDraw {
                s: self.veh.s[i] as f32,
                v: self.veh.v[i] as f32,
                len: self.veh.len[i] as f32,
                lat: self.veh.lat[i],
                lane: self.veh.lane[i],
                flags,
                id: self.veh.id[i] as u32,
            }
        }));
    }
}
