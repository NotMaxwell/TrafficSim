//! Automated vehicle state.

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum AutoMode {
    #[default]
    Acc,
    Cacc,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct AutoState {
    pub mode: AutoMode,
    /// CACC controller state (commanded acceleration), m/s².
    pub u: f64,
    /// Running average of own speed (wave dampener), m/s.
    pub v_avg: f64,
    /// Dampener speed command, m/s.
    pub v_cmd: f64,
}
