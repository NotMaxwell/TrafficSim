//! One pane of the split view: header with live numbers, then the road
//! views for that world.

use egui::{Color32, RichText};
use traffic_core::Road;
use traffic_core::snapshot::WorldSnap;

use crate::view_road::{self, ColorBy};
use crate::view_strip::{self, Camera};
use crate::{theme, view_detail};

pub struct PaneStyle {
    pub title: &'static str,
    pub color: Color32,
}

pub const LEFT: PaneStyle = PaneStyle { title: "LEFT", color: theme::LEFT };
pub const RIGHT: PaneStyle = PaneStyle { title: "RIGHT", color: theme::RIGHT };

pub fn mode_name(share: f64, autos_present: usize) -> String {
    match share {
        s if s <= 0.0 && autos_present == 0 => "Human mode".into(),
        s if s <= 0.0 => format!("Human mode + {autos_present} auto{}", if autos_present == 1 { "" } else { "s" }),
        s if s >= 1.0 => "Auto mode".into(),
        s => format!("Mixed · {:.0}% autos", s * 100.0),
    }
}

pub fn header(ui: &mut egui::Ui, style: &PaneStyle, snap: &WorldSnap, queued: usize) {
    ui.horizontal_wrapped(|ui| {
        ui.label(RichText::new(style.title).strong().color(style.color));
        let autos = snap.vehicles.iter().filter(|d| d.flags & traffic_core::snapshot::FLAG_AUTO != 0).count();
        ui.label(RichText::new(mode_name(snap.auto_share, autos)).strong());
        ui.separator();
        stat(ui, "vehicles", snap.vehicles.len().to_string());
        stat(ui, "mean", format!("{:.0} km/h", snap.mean_speed * 3.6));
        stat(ui, "queued", queued.to_string());
        let c = snap.stats.collisions;
        let crash = if c > 0 { RichText::new(format!("{c}")).color(theme::CRASH).strong() } else { RichText::new("0") };
        ui.label(RichText::new("crashes").weak());
        ui.label(crash);
    });
}

fn stat(ui: &mut egui::Ui, label: &str, value: String) {
    ui.label(RichText::new(label).weak());
    ui.label(RichText::new(value).monospace());
}

/// Road views for one world: strip + detail on the corridor, ring view on
/// validation rings. Returns a mile clicked on the strip.
pub fn road(ui: &mut egui::Ui, road: &Road, snap: &WorldSnap, cam: &mut Camera, by: ColorBy, strip_h: f32) -> Option<f64> {
    if !road.is_corridor() {
        view_road::ring_view(ui, snap, road, by);
        return None;
    }
    let clicked = view_strip::strip(ui, road, snap, cam, by, strip_h);
    ui.add_space(4.0);
    view_detail::detail(ui, road, snap, cam, by);
    clicked
}
