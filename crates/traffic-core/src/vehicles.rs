//! Vehicle store: hot per-tick fields as a struct of arrays, cold per-driver
//! data in `Agent`, and a per-lane index sorted by position so leaders and
//! followers are array neighbours.

use serde::{Deserialize, Serialize};

use crate::auto::AutoState;
use crate::human::HumanState;
use crate::idm::Idm;
use crate::rng::VehRng;

/// "No ramp" marker for origin / exit.
pub const NO_RAMP: u16 = u16::MAX;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum Kind {
    Human,
    Auto,
}

/// Per-driver traits, drawn once at spawn from the vehicle's trait stream.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Driver {
    /// IDM parameters; `idm.v0` is the desired speed on a mainline whose
    /// limit is `road.limit` (scaled for other limits).
    pub idm: Idm,
    pub politeness: f64,
    pub threshold: f64,
    /// Reaction delay, s (0 for ideal drivers).
    pub tau: f64,
    /// Leaders watched (multi-anticipation).
    pub anticipation: u8,
}

/// A scheduled override (hard-brake event): brake to `target` speed and hold
/// it until `until`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Forced {
    pub target: f64,
    pub decel: f64,
    pub until: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Agent {
    pub driver: Driver,
    pub truck: bool,
    pub origin: u16,
    pub exit: u16,
    pub spawn_t: f64,
    /// Time at which a crashed vehicle is towed; negative = not crashed.
    pub crashed_until: f64,
    pub forced: Option<Forced>,
    pub human: HumanState,
    pub auto: AutoState,
    pub rng: VehRng,
    /// Time of the last lane change.
    pub last_change: f64,
    /// Distance travelled, m.
    pub odo: f64,
    /// Position before this tick's move (for crossing tests).
    pub s_prev: f64,
    /// Was braking hard (< -3 m/s²) last tick (for counting onsets).
    pub hard_braking: bool,
}

impl Agent {
    pub fn crashed(&self) -> bool {
        self.crashed_until >= 0.0
    }
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Vehicles {
    pub id: Vec<u64>,
    pub s: Vec<f64>,
    pub v: Vec<f64>,
    pub a: Vec<f64>,
    pub lane: Vec<u8>,
    pub len: Vec<f64>,
    pub kind: Vec<Kind>,
    /// Drawn lateral offset in lanes, decays to 0 after a lane change.
    pub lat: Vec<f32>,
    pub agent: Vec<Agent>,
    /// Slots per lane, sorted by (s, id). Rebuilt by `reindex`.
    pub lanes: Vec<Vec<u32>>,
    /// Index of each slot inside its lane list.
    pub pos: Vec<u32>,
}

impl Vehicles {
    pub fn new(max_lane: u8) -> Self {
        Vehicles { lanes: vec![Vec::new(); max_lane as usize + 1], ..Default::default() }
    }

    pub fn len(&self) -> usize {
        self.id.len()
    }

    pub fn is_empty(&self) -> bool {
        self.id.is_empty()
    }

    #[allow(clippy::too_many_arguments)]
    pub fn push(&mut self, id: u64, s: f64, v: f64, lane: u8, len: f64, kind: Kind, agent: Agent) -> usize {
        self.id.push(id);
        self.s.push(s);
        self.v.push(v);
        self.a.push(0.0);
        self.lane.push(lane);
        self.len.push(len);
        self.kind.push(kind);
        self.lat.push(0.0);
        self.agent.push(agent);
        self.pos.push(0);
        self.id.len() - 1
    }

    /// Swap-remove a slot. The lane index is stale until `reindex`.
    pub fn remove(&mut self, slot: usize) {
        self.id.swap_remove(slot);
        self.s.swap_remove(slot);
        self.v.swap_remove(slot);
        self.a.swap_remove(slot);
        self.lane.swap_remove(slot);
        self.len.swap_remove(slot);
        self.kind.swap_remove(slot);
        self.lat.swap_remove(slot);
        self.agent.swap_remove(slot);
        self.pos.swap_remove(slot);
    }

    /// Rebuild the per-lane sorted index. Order barely changes between
    /// ticks, so the sort runs on nearly sorted data.
    pub fn reindex(&mut self) {
        for l in &mut self.lanes {
            l.clear();
        }
        for i in 0..self.id.len() {
            self.lanes[self.lane[i] as usize].push(i as u32);
        }
        let (s, id) = (&self.s, &self.id);
        for l in &mut self.lanes {
            l.sort_unstable_by(|&x, &y| {
                let (x, y) = (x as usize, y as usize);
                s[x].total_cmp(&s[y]).then(id[x].cmp(&id[y]))
            });
            for (k, &slot) in l.iter().enumerate() {
                self.pos[slot as usize] = k as u32;
            }
        }
    }

    /// Move a slot to another lane, keeping both lane lists sorted.
    pub fn move_lane(&mut self, slot: usize, to: u8) {
        let from = self.lane[slot] as usize;
        let k = self.pos[slot] as usize;
        self.lanes[from].remove(k);
        for j in k..self.lanes[from].len() {
            let x = self.lanes[from][j] as usize;
            self.pos[x] = j as u32;
        }
        let (s, id) = (self.s[slot], self.id[slot]);
        let list = &self.lanes[to as usize];
        let k = list.partition_point(|&j| {
            let j = j as usize;
            self.s[j] < s || (self.s[j] == s && self.id[j] < id)
        });
        self.lanes[to as usize].insert(k, slot as u32);
        for j in k..self.lanes[to as usize].len() {
            let x = self.lanes[to as usize][j] as usize;
            self.pos[x] = j as u32;
        }
        self.lane[slot] = to;
    }

    /// Next vehicle ahead in the same lane (wrapping around the ring).
    pub fn leader(&self, slot: usize) -> Option<usize> {
        let l = &self.lanes[self.lane[slot] as usize];
        if l.len() < 2 {
            return None;
        }
        Some(l[(self.pos[slot] as usize + 1) % l.len()] as usize)
    }

    /// Next vehicle behind in the same lane (wrapping).
    pub fn follower(&self, slot: usize) -> Option<usize> {
        let l = &self.lanes[self.lane[slot] as usize];
        if l.len() < 2 {
            return None;
        }
        Some(l[(self.pos[slot] as usize + l.len() - 1) % l.len()] as usize)
    }

    /// Leader and follower a vehicle at `s` would have in `lane`, ignoring
    /// `exclude` (normally the asking vehicle itself).
    pub fn around(&self, lane: u8, s: f64, exclude: usize) -> (Option<usize>, Option<usize>) {
        let Some(l) = self.lanes.get(lane as usize) else { return (None, None) };
        let n = l.len();
        if n == 0 || (n == 1 && l[0] as usize == exclude) {
            return (None, None);
        }
        let k = l.partition_point(|&j| self.s[j as usize] <= s);
        let mut lead = k % n;
        if l[lead] as usize == exclude {
            lead = (lead + 1) % n;
        }
        let mut fol = (k + n - 1) % n;
        if l[fol] as usize == exclude {
            fol = (fol + n - 1) % n;
        }
        (Some(l[lead] as usize), Some(l[fol] as usize))
    }

    /// The k-th leader ahead in the same lane (k = 1 is the direct leader).
    pub fn nth_leader(&self, slot: usize, k: usize) -> Option<usize> {
        let l = &self.lanes[self.lane[slot] as usize];
        if k >= l.len() {
            return None;
        }
        Some(l[(self.pos[slot] as usize + k) % l.len()] as usize)
    }
}
