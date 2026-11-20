//! Colours shared by all views.

use egui::Color32;

pub const ASPHALT: Color32 = Color32::from_rgb(58, 62, 70);
pub const LANE_MARK: Color32 = Color32::from_rgb(200, 200, 190);
pub const BRAKE: Color32 = Color32::from_rgb(255, 40, 40);
pub const CRASH: Color32 = Color32::from_rgb(255, 0, 200);
pub const HUMAN: Color32 = Color32::from_rgb(236, 240, 241);
pub const TRUCK: Color32 = Color32::from_rgb(149, 165, 166);
pub const AUTO: Color32 = Color32::from_rgb(0, 184, 255);

pub fn bg_road(ui: &egui::Ui) -> Color32 {
    if ui.visuals().dark_mode { Color32::from_rgb(28, 31, 36) } else { Color32::from_rgb(214, 222, 214) }
}
