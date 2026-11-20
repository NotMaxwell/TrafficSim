//! Highway Traffic Simulator: desktop GUI.

mod capture;
mod pane;
mod runner;
mod scenarios;
mod theme;
mod view_detail;
mod view_road;
mod view_strip;

use runner::{Cmd, Runner};
use traffic_core::Side;
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
    share: [f64; 2],
    color_by: ColorBy,
    fps: f32,
    capture: Option<capture::Capture>,
    camera: view_strip::Camera,
}

impl App {
    fn new(cc: &eframe::CreationContext<'_>) -> Self {
        // Optional command line: --scenario <index> --speed <x> --mile <m> --zoom <half width m>
        let args: Vec<String> = std::env::args().collect();
        let arg = |k: &str| args.iter().position(|a| a == k).and_then(|i| args.get(i + 1));
        let scenario_idx = arg("--scenario").and_then(|s| s.parse().ok()).unwrap_or(0).min(scenarios::BUILTIN.len() - 1);
        let speed = arg("--speed").and_then(|s| s.parse().ok()).unwrap_or(1.0);
        let sc = scenarios::load(scenarios::BUILTIN[scenario_idx].1);
        let share = [sc.mix.left, sc.mix.right];
        let mut camera = view_strip::Camera::default();
        if let Some(m) = arg("--mile").and_then(|s| s.parse().ok()) {
            camera.mile = m;
        }
        if let Some(z) = arg("--zoom").and_then(|s| s.parse().ok()) {
            camera.half_m = z;
        }
        let runner = Runner::start(sc, cc.egui_ctx.clone());
        runner.send(Cmd::Speed(speed));
        App {
            runner,
            scenario_idx,
            speed,
            share,
            color_by: ColorBy::Speed,
            fps: 0.0,
            capture: capture::Capture::from_args(&args),
            camera,
        }
    }

    fn load(&mut self, idx: usize) {
        self.scenario_idx = idx;
        let sc = scenarios::load(scenarios::BUILTIN[idx].1);
        self.share = [sc.mix.left, sc.mix.right];
        self.camera = Default::default();
        self.runner.send(Cmd::Load(Box::new(sc)));
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
            let r = ui.add(egui::Slider::new(&mut self.speed, 0.25..=100.0).logarithmic(true).suffix("×").max_decimals(2));
            if r.changed() {
                self.runner.send(Cmd::Speed(self.speed));
            }
            for s in [1.0, 10.0, 50.0] {
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
                self.load(idx);
            }
            ui.separator();
            let t = frame.worlds.first().map_or(0.0, |w| w.time);
            ui.monospace(format_time(t));
            ui.label(egui::RichText::new(format!("{:.1}× actual · tick {:.2} ms · {:.0} fps", frame.actual_speed, frame.tick_ms, self.fps)).weak());
        });
    }

    fn controls(&mut self, ui: &mut egui::Ui) {
        ui.heading("Controls");
        ui.add_space(4.0);
        ui.label(egui::RichText::new("Auto share (new arrivals)").strong());
        for (k, (side, style)) in [(Side::Left, &pane::LEFT), (Side::Right, &pane::RIGHT)].into_iter().enumerate() {
            ui.horizontal(|ui| {
                ui.label(egui::RichText::new(style.title).color(style.color).strong());
                let r = ui.add(egui::Slider::new(&mut self.share[k], 0.0..=1.0).custom_formatter(|v, _| format!("{:.0}%", v * 100.0)));
                if r.changed() {
                    self.runner.send(Cmd::SetShare(side, self.share[k]));
                }
            });
        }
        ui.add_space(8.0);
        ui.label(egui::RichText::new("Vehicle colour").strong());
        ui.horizontal(|ui| {
            ui.selectable_value(&mut self.color_by, ColorBy::Speed, "Speed");
            ui.selectable_value(&mut self.color_by, ColorBy::Kind, "Type");
        });
        legend(ui, self.color_by);
    }
}

fn legend(ui: &mut egui::Ui, by: ColorBy) {
    let item = |ui: &mut egui::Ui, c: egui::Color32, t: &str| {
        ui.horizontal(|ui| {
            let (r, _) = ui.allocate_exact_size(egui::vec2(14.0, 10.0), egui::Sense::hover());
            ui.painter().rect_filled(r, 2.0, c);
            ui.label(egui::RichText::new(t).small());
        });
    };
    match by {
        ColorBy::Speed => {
            for (f, t) in [(0.0, "stopped"), (0.4, "slow"), (0.7, "moderate"), (1.0, "at limit")] {
                item(ui, view_road::speed_color(f, 1.0), t);
            }
        }
        ColorBy::Kind => {
            item(ui, theme::HUMAN, "human car");
            item(ui, theme::TRUCK, "truck");
            item(ui, theme::AUTO, "auto (connected)");
        }
    }
    item(ui, theme::BRAKE, "brake light");
    item(ui, theme::CRASH, "crash");
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
        egui::Panel::left("controls").resizable(true).default_size(230.0).show(ui, |ui| self.controls(ui));
        egui::CentralPanel::default().show(ui, |ui| {
            let (Some(road), [left, right]) = (&frame.road, frame.worlds.as_slice()) else { return };
            let queued = [frame.queued[0], frame.queued[1]];
            ui.columns(2, |cols| {
                for (k, (snap, style)) in [(left, &pane::LEFT), (right, &pane::RIGHT)].into_iter().enumerate() {
                    let ui = &mut cols[k];
                    pane::header(ui, style, snap, queued[k]);
                    let strip_h = (ui.available_height() * 0.22).clamp(70.0, 130.0);
                    if let Some(m) = pane::road(ui, road, snap, &mut self.camera, self.color_by, strip_h) {
                        self.camera.mile = m;
                        self.camera.follow = None;
                    }
                }
            });
        });
    }
}
