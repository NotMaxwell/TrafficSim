//! Split view: two worlds built from the same scenario (same seed, demand
//! and event schedule) that differ only in auto share. They are stepped in
//! lockstep; each tick runs both in parallel (rayon) and the next tick
//! starts only when both are done.

use crate::{Scenario, World, step};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Side {
    Left,
    Right,
}

#[derive(Clone, Debug)]
pub struct Sim {
    pub scenario: Scenario,
    pub left: World,
    pub right: World,
}

impl Sim {
    pub fn new(sc: &Scenario) -> Sim {
        Sim {
            scenario: sc.clone(),
            left: World::with_auto_ids(sc, sc.mix.left, &sc.mix.left_auto_ids),
            right: World::with_auto_ids(sc, sc.mix.right, &sc.mix.right_auto_ids),
        }
    }

    pub fn world(&self, side: Side) -> &World {
        match side {
            Side::Left => &self.left,
            Side::Right => &self.right,
        }
    }

    pub fn world_mut(&mut self, side: Side) -> &mut World {
        match side {
            Side::Left => &mut self.left,
            Side::Right => &mut self.right,
        }
    }

    pub fn time(&self) -> f64 {
        self.left.time
    }

    /// One tick of both worlds in parallel.
    pub fn step(&mut self) {
        let (l, r) = (&mut self.left, &mut self.right);
        rayon::join(|| step(l), || step(r));
    }

    /// One tick of both worlds on the calling thread (reference for tests).
    pub fn step_serial(&mut self) {
        step(&mut self.left);
        step(&mut self.right);
    }

    pub fn run_for(&mut self, seconds: f64) {
        let n = (seconds / self.left.dt).round() as u64;
        for _ in 0..n {
            self.step();
        }
    }

    /// Inject an event now into both worlds (same time, same place).
    pub fn inject(&mut self, kind: crate::events::EventKind) {
        let t = self.time();
        let spec = crate::events::EventSpec { t, kind };
        self.scenario.events.push(spec.clone());
        self.left.events.add(spec.clone());
        self.right.events.add(spec);
    }

    /// Change the auto share of one side. Only new arrivals are affected,
    /// so the change spreads through the corridor over time.
    pub fn set_auto_share(&mut self, side: Side, share: f64) {
        self.world_mut(side).auto_share = share.clamp(0.0, 1.0);
        match side {
            Side::Left => self.scenario.mix.left = share,
            Side::Right => self.scenario.mix.right = share,
        }
    }
}
