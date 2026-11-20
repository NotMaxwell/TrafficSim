# Pinned dependencies

All versions are pinned exactly in the workspace `Cargo.toml`. egui/eframe/wgpu
break APIs between minor releases, so code must target these versions.

| Crate | Version | Docs | Notes |
| --- | --- | --- | --- |
| eframe | 0.36.2 | https://docs.rs/eframe/0.36.2 | wgpu backend (wgpu 30). `App::ui(&mut self, ui: &mut egui::Ui, frame)` replaces the old `update(ctx, frame)`; `App::logic(ctx, frame)` runs before each `ui`. |
| egui | 0.36.2 | https://docs.rs/egui/0.36.2 | Panels: `egui::Panel::left/right/top/bottom(id).show(ui, ..)` and `CentralPanel::default().show(ui, ..)` take a `&mut Ui`, not a `&Context`. `SidePanel`/`TopBottomPanel` are gone; `default_width` is now `default_size`. |
| egui_plot | 0.37.0 | https://docs.rs/egui_plot/0.37.0 | 0.37 is the release built on egui 0.36 (0.36 uses egui 0.35). |
| rayon | 1.12.0 | https://docs.rs/rayon/1.12.0 | Steps the two worlds in parallel (`rayon::join`) and batch sweeps. |
| rand | 0.9.5 | https://docs.rs/rand/0.9.5 | `Rng::random`, `random_range`, `SeedableRng::seed_from_u64`. |
| rand_chacha | 0.9.0 | https://docs.rs/rand_chacha/0.9.0 | `ChaCha8Rng` per-purpose streams (see `rng.rs`). |
| serde | 1.0.229 | https://docs.rs/serde | Scenario (de)serialization. |
| toml | 1.1.8 | https://docs.rs/toml/1.1.8 | `toml::from_str`, `toml::to_string_pretty`. |

Toolchain: Rust 1.98 (edition 2024).
