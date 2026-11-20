//! Event panel: build an event and inject it into both worlds at once.

use traffic_core::Road;
use traffic_core::events::{EventKind, Pos};
use traffic_core::road::{Dir, MILE, RampKind};

const KINDS: [&str; 6] = ["Hard brake", "Crash", "Lane closure", "Demand surge", "Weather", "Comms outage"];

pub struct EventForm {
    kind: usize,
    dir: Dir,
    pub mile: f64,
    lanes: [bool; 7],
    target_kmh: f64,
    duration_min: f64,
    length_mi: f64,
    extra_vph: f64,
    ramp: usize,
    speed_factor: f64,
    gap_factor: f64,
    whole: bool,
}

impl Default for EventForm {
    fn default() -> Self {
        EventForm {
            kind: 0,
            dir: Dir::East,
            mile: 10.0,
            lanes: [false, true, true, false, false, false, false],
            target_kmh: 20.0,
            duration_min: 15.0,
            length_mi: 1.0,
            extra_vph: 1500.0,
            ramp: 0,
            speed_factor: 0.7,
            gap_factor: 1.4,
            whole: true,
        }
    }
}

impl EventForm {
    /// Draw the form; returns an event when "Inject" is pressed.
    pub fn ui(&mut self, ui: &mut egui::Ui, road: &Road, camera_mile: f64) -> Option<EventKind> {
        egui::ComboBox::from_id_salt("event_kind").selected_text(KINDS[self.kind]).show_ui(ui, |ui| {
            for (i, k) in KINDS.iter().enumerate() {
                ui.selectable_value(&mut self.kind, i, *k);
            }
        });
        let corridor = road.is_corridor();
        let max_mile = if corridor { road.corridor / MILE } else { road.length / MILE };
        let needs_place = self.kind != 3 && !(self.kind == 5 && self.whole);
        if needs_place {
            if corridor {
                ui.horizontal(|ui| {
                    ui.selectable_value(&mut self.dir, Dir::East, "EB");
                    ui.selectable_value(&mut self.dir, Dir::West, "WB");
                    if ui.small_button("at camera").on_hover_text("Use the detail camera's position").clicked() {
                        self.mile = camera_mile;
                    }
                });
                ui.add(egui::Slider::new(&mut self.mile, 0.0..=max_mile).text("mile").max_decimals(2));
            } else {
                let mut m = self.mile * MILE;
                ui.add(egui::Slider::new(&mut m, 0.0..=road.length).text("position (m)").max_decimals(0));
                self.mile = m / MILE;
            }
        }
        let lanes_ui = |ui: &mut egui::Ui, lanes: &mut [bool; 7]| {
            ui.horizontal(|ui| {
                ui.label("lanes");
                for l in 1..=road.lanes as usize {
                    ui.checkbox(&mut lanes[l], format!("{l}"));
                }
            })
            .response
            .on_hover_text("Lane 1 is the rightmost lane");
        };
        match self.kind {
            0 => {
                ui.add(egui::Slider::new(&mut self.target_kmh, 0.0..=80.0).text("slow to km/h"));
                ui.add(egui::Slider::new(&mut self.duration_min, 0.05..=2.0).text("hold (min)"));
            }
            1 => {
                lanes_ui(ui, &mut self.lanes);
                ui.add(egui::Slider::new(&mut self.duration_min, 1.0..=60.0).text("clear after (min)"));
            }
            2 => {
                lanes_ui(ui, &mut self.lanes);
                ui.add(egui::Slider::new(&mut self.length_mi, 0.1..=3.0).text("length (mi)"));
                ui.add(egui::Slider::new(&mut self.duration_min, 1.0..=120.0).text("duration (min)"));
            }
            3 => {
                let ramps: Vec<&str> = road.ramps.iter().filter(|r| r.kind == RampKind::On).map(|r| r.name.as_str()).collect();
                if ramps.is_empty() {
                    ui.label("No on-ramps on this road.");
                    return None;
                }
                self.ramp = self.ramp.min(ramps.len() - 1);
                egui::ComboBox::from_id_salt("surge_ramp").selected_text(ramps[self.ramp]).show_ui(ui, |ui| {
                    for (i, r) in ramps.iter().enumerate() {
                        ui.selectable_value(&mut self.ramp, i, *r);
                    }
                });
                ui.add(egui::Slider::new(&mut self.extra_vph, 100.0..=4000.0).text("extra veh/h"));
                ui.add(egui::Slider::new(&mut self.duration_min, 1.0..=120.0).text("duration (min)"));
            }
            4 => {
                ui.add(egui::Slider::new(&mut self.length_mi, 0.5..=20.0).text("length (mi)"));
                ui.add(egui::Slider::new(&mut self.speed_factor, 0.3..=1.0).text("speed ×"));
                ui.add(egui::Slider::new(&mut self.gap_factor, 1.0..=2.5).text("gap ×"));
                ui.add(egui::Slider::new(&mut self.duration_min, 1.0..=120.0).text("duration (min)"));
            }
            _ => {
                ui.checkbox(&mut self.whole, "whole corridor");
                if !self.whole {
                    ui.add(egui::Slider::new(&mut self.length_mi, 0.5..=20.0).text("length (mi)"));
                }
                ui.add(egui::Slider::new(&mut self.duration_min, 0.5..=60.0).text("duration (min)"));
            }
        }
        if !ui.button("⚡ Inject into both panes").clicked() {
            return None;
        }
        let at = if corridor { Pos::Corridor { dir: self.dir, mile: self.mile } } else { Pos::Ring { s: self.mile * MILE } };
        // End of a stretch, downstream of `at` in the direction of travel.
        let to_mile = match (corridor, self.dir) {
            (true, Dir::East) => (self.mile + self.length_mi).min(max_mile),
            (true, Dir::West) => (self.mile - self.length_mi).max(0.0),
            _ => self.length_mi,
        };
        let lanes: Vec<u8> = (1..=road.lanes).filter(|&l| self.lanes[l as usize]).collect();
        let dur = self.duration_min * 60.0;
        Some(match self.kind {
            0 => EventKind::HardBrake { at, target_kmh: self.target_kmh, duration_s: dur, decel: 4.0 },
            1 => EventKind::Crash { at, lanes, clear_s: dur },
            2 => EventKind::LaneClosure { at, to_mile, lanes, duration_s: dur },
            3 => {
                let name = road.ramps.iter().filter(|r| r.kind == RampKind::On).nth(self.ramp)?.name.clone();
                EventKind::DemandSurge { ramp: name, extra_vph: self.extra_vph, duration_s: dur }
            }
            4 => EventKind::Weather { at, to_mile, speed_factor: self.speed_factor, gap_factor: self.gap_factor, duration_s: dur },
            _ => EventKind::CommsOutage { at, to_mile, whole_corridor: self.whole, duration_s: dur },
        })
    }
}
