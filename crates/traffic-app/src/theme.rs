//! Colours shared by all views.

use egui::Color32;

pub const ASPHALT: Color32 = Color32::from_rgb(58, 62, 70);
pub const EDGE_MARK: Color32 = Color32::from_rgb(235, 235, 225);
pub const LANE_MARK: Color32 = Color32::from_rgb(200, 200, 190);
pub const BLINKER: Color32 = Color32::from_rgb(255, 176, 0);
pub const BRAKE: Color32 = Color32::from_rgb(255, 40, 40);
pub const CLOSURE: Color32 = Color32::from_rgb(255, 140, 0);
pub const CRASH: Color32 = Color32::from_rgb(255, 0, 200);
pub const HUMAN: Color32 = Color32::from_rgb(236, 240, 241);
pub const TRUCK: Color32 = Color32::from_rgb(149, 165, 166);
pub const AUTO: Color32 = Color32::from_rgb(0, 184, 255);

/// Series colours for the two worlds (left pane, right pane).
pub const LEFT: Color32 = Color32::from_rgb(255, 159, 67);
pub const RIGHT: Color32 = Color32::from_rgb(72, 219, 251);

pub fn bg_road(ui: &egui::Ui) -> Color32 {
    if ui.visuals().dark_mode { Color32::from_rgb(28, 31, 36) } else { Color32::from_rgb(214, 222, 214) }
}
