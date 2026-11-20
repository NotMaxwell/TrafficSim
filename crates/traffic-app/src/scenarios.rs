//! Scenarios bundled into the executable (the files in /scenarios).

pub const BUILTIN: &[(&str, &str)] = &[("Sugiyama ring", include_str!("../../../scenarios/sugiyama_ring.toml"))];

pub fn load(text: &str) -> traffic_core::Scenario {
    traffic_core::Scenario::from_toml(text).unwrap_or_else(|e| panic!("bundled scenario is invalid: {e}"))
}
