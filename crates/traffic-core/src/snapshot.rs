//! Render snapshot: the minimal per-vehicle data the UI draws, copied out
//! of a world once per frame (not once per tick).

use crate::vehicles::Kind;
use crate::{Stats, World};

pub const FLAG_AUTO: u8 = 1;
pub const FLAG_TRUCK: u8 = 2;
pub const FLAG_BRAKING: u8 = 4;
pub const FLAG_CRASHED: u8 = 8;
pub const FLAG_CACC: u8 = 16;
pub const FLAG_DISTRACTED: u8 = 32;

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

#[derive(Clone, Debug, Default, PartialEq)]
pub struct WorldSnap {
    pub time: f64,
    pub tick: u64,
    pub vehicles: Vec<VehDraw>,
    pub stats: Stats,
    pub mean_speed: f64,
    pub auto_share: f64,
}

impl World {
    /// Copy the drawable state into `out`, reusing its allocation.
    pub fn snapshot_into(&self, out: &mut WorldSnap) {
        out.time = self.time;
        out.tick = self.tick;
        out.stats = self.stats.clone();
        out.mean_speed = self.mean_speed();
        out.auto_share = self.auto_share;
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
