//! Scenarios bundled into the executable (the files in /scenarios).

pub const BUILTIN: &[(&str, &str)] = &[
    ("City A ↔ City B corridor", include_str!("../../../scenarios/city_a_to_b.toml")),
    ("Sugiyama ring (phantom jam)", include_str!("../../../scenarios/sugiyama_ring.toml")),
    ("Single auto on a ring (Stern)", include_str!("../../../scenarios/single_av_ring.toml")),
];

pub fn load(text: &str) -> traffic_core::Scenario {
    traffic_core::Scenario::from_toml(text).unwrap_or_else(|e| panic!("bundled scenario is invalid: {e}"))
}
