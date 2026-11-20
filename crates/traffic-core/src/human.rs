//! Human driver state (reaction-delay buffer, noise, distraction).

/// Delay buffer length: 25 ticks of history (2.5 s at dt = 0.1) plus one.
pub const BUF: usize = 26;

/// What a human perceived in one tick: up to three leaders' gaps and
/// closing speeds, plus own speed.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Percept {
    pub gap: [f32; 3],
    pub dv: [f32; 3],
    pub n: u8,
    pub v: f32,
}

#[derive(Clone, Debug, PartialEq)]
pub struct HumanState {
    pub buf: [Percept; BUF],
    pub head: u8,
    pub filled: u8,
    /// Wiener processes for gap and closing-speed estimation errors.
    pub w_gap: f64,
    pub w_rate: f64,
    pub distracted_until: f64,
}

impl Default for HumanState {
    fn default() -> Self {
        HumanState { buf: [Percept::default(); BUF], head: 0, filled: 0, w_gap: 0.0, w_rate: 0.0, distracted_until: -1.0 }
    }
}
