//! Highway Traffic Simulator: desktop GUI.

mod capture;
mod runner;
mod scenarios;
mod theme;
mod view_road;

use runner::{Cmd, Runner};
use view_road::ColorBy;

fn main() -> eframe::Result {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_title("Highway Traffic Simulator")
            .with_app_id("traffic-sim")
            .with_inner_size([1600.0, 1000.0])
            .with_min_inner_size([900.0, 600.0]),
        ..Default::default()
    };
    eframe::run_native("Highway Traffic Simulator", options, Box::new(|cc| Ok(Box::new(App::new(cc)))))
}

struct App {
    runner: Runner,
    scenario_idx: usize,
    speed: f64,
    color_by: ColorBy,
    fps: f32,
    capture: Option<capture::Capture>,
}

impl App {
    fn new(cc: &eframe::CreationContext<'_>) -> Self {
        // Optional command line: --scenario <index> --speed <x>
        let args: Vec<String> = std::env::args().collect();
        let arg = |k: &str| args.iter().position(|a| a == k).and_then(|i| args.get(i + 1));
        let scenario_idx = arg("--scenario").and_then(|s| s.parse().ok()).unwrap_or(0).min(scenarios::BUILTIN.len() - 1);
        let speed = arg("--speed").and_then(|s| s.parse().ok()).unwrap_or(1.0);
        let sc = scenarios::load(scenarios::BUILTIN[scenario_idx].1);
        let runner = Runner::start(sc, cc.egui_ctx.clone());
        runner.send(Cmd::Speed(speed));
        App { runner, scenario_idx, speed, color_by: ColorBy::Speed, fps: 0.0, capture: capture::Capture::from_args(&args) }
    }

    fn top_bar(&mut self, ui: &mut egui::Ui, frame: &runner::Frame) {
        ui.horizontal_wrapped(|ui| {
            let label = if frame.playing { "⏸ Pause" } else { "▶ Play" };
            if ui.button(label).clicked() {
                self.runner.send(Cmd::Play(!frame.playing));
            }
            if ui.button("Step").on_hover_text("Advance one 0.1 s tick").clicked() {
                self.runner.send(Cmd::StepOnce);
            }
            ui.separator();
            ui.label("Speed");
            let r = ui.add(egui::Slider::new(&mut self.speed, 0.25..=50.0).logarithmic(true).suffix("×").max_decimals(2));
            if r.changed() {
                self.runner.send(Cmd::Speed(self.speed));
            }
            for s in [1.0, 10.0] {
                if ui.small_button(format!("{s}×")).clicked() {
                    self.speed = s;
                    self.runner.send(Cmd::Speed(s));
                }
            }
            ui.separator();
            let mut idx = self.scenario_idx;
            egui::ComboBox::from_id_salt("scenario")
                .selected_text(scenarios::BUILTIN[idx].0)
                .show_ui(ui, |ui| {
                    for (i, (name, _)) in scenarios::BUILTIN.iter().enumerate() {
                        ui.selectable_value(&mut idx, i, *name);
                    }
                });
            if idx != self.scenario_idx || ui.button("⟲ Reset").clicked() {
                self.scenario_idx = idx;
                self.runner.send(Cmd::Load(Box::new(scenarios::load(scenarios::BUILTIN[idx].1))));
            }
            ui.separator();
            ui.selectable_value(&mut self.color_by, ColorBy::Speed, "Color: speed");
            ui.selectable_value(&mut self.color_by, ColorBy::Kind, "Color: type");
            ui.separator();
            let t = frame.worlds.first().map_or(0.0, |w| w.time);
            ui.strong(&frame.scenario);
            ui.monospace(format_time(t));
            ui.separator();
            ui.label(format!("{:.1}× actual · tick {:.2} ms · {:.0} fps", frame.actual_speed, frame.tick_ms, self.fps));
        });
    }
}

pub fn format_time(t: f64) -> String {
    let s = t as u64;
    format!("{:02}:{:02}:{:02}.{}", s / 3600, (s / 60) % 60, s % 60, ((t.fract()) * 10.0) as u32)
}

impl eframe::App for App {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        let dt = ui.input(|i| i.stable_dt).max(1e-4);
        self.fps = 0.95 * self.fps + 0.05 / dt;
        if let Some(c) = &mut self.capture {
            c.update(ui.ctx());
        }
        let frame = self.runner.frame();
        egui::Panel::top("top").show(ui, |ui| self.top_bar(ui, &frame));
        egui::CentralPanel::default().show(ui, |ui| {
            if let (Some(road), Some(w)) = (&frame.road, frame.worlds.first()) {
                view_road::ring_view(ui, w, road, self.color_by);
            }
        });
    }
}
