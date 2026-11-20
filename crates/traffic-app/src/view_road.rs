//! Road drawing: ring view (validation scenarios), corridor strip, zoomed
//! detail view and the ring minimap. Vehicles are batched into one mesh per
//! view, so tens of thousands of quads cost one draw call.

use egui::{Color32, Mesh, Pos2, Sense, Shape, Stroke, Vec2, vec2};
use traffic_core::Road;
use traffic_core::snapshot::{FLAG_AUTO, FLAG_BRAKING, FLAG_CRASHED, FLAG_TRUCK, VehDraw, WorldSnap};

use crate::theme;

/// How vehicles are coloured.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ColorBy {
    Speed,
    Kind,
}

/// Speed ramp: red (stopped) -> amber -> green (at the reference speed).
pub fn speed_color(v: f32, v_ref: f32) -> Color32 {
    let x = (v / v_ref.max(1.0)).clamp(0.0, 1.0);
    let stops = [(0.0, [214, 48, 49]), (0.35, [232, 126, 4]), (0.65, [241, 196, 15]), (1.0, [39, 174, 96])];
    for w in stops.windows(2) {
        let ((x0, c0), (x1, c1)) = (w[0], w[1]);
        if x <= x1 {
            let t = (x - x0) / (x1 - x0);
            let l = |a: u8, b: u8| (a as f32 + (b as f32 - a as f32) * t) as u8;
            return Color32::from_rgb(l(c0[0], c1[0]), l(c0[1], c1[1]), l(c0[2], c1[2]));
        }
    }
    Color32::from_rgb(39, 174, 96)
}

pub fn vehicle_color(d: &VehDraw, by: ColorBy, v_ref: f32) -> Color32 {
    if d.flags & FLAG_CRASHED != 0 {
        return theme::CRASH;
    }
    match by {
        ColorBy::Speed => speed_color(d.v, v_ref),
        ColorBy::Kind => {
            if d.flags & FLAG_AUTO != 0 {
                theme::AUTO
            } else if d.flags & FLAG_TRUCK != 0 {
                theme::TRUCK
            } else {
                theme::HUMAN
            }
        }
    }
}

/// Add an oriented rectangle: centre `c`, unit direction `dir`.
pub fn quad(mesh: &mut Mesh, c: Pos2, dir: Vec2, half_len: f32, half_w: f32, color: Color32) {
    let n = vec2(-dir.y, dir.x);
    let base = mesh.vertices.len() as u32;
    for p in [
        c - dir * half_len - n * half_w,
        c + dir * half_len - n * half_w,
        c + dir * half_len + n * half_w,
        c - dir * half_len + n * half_w,
    ] {
        mesh.colored_vertex(p, color);
    }
    mesh.add_triangle(base, base + 1, base + 2);
    mesh.add_triangle(base, base + 2, base + 3);
}

/// Validation ring: lanes drawn as concentric circles, vehicles as quads.
pub fn ring_view(ui: &mut egui::Ui, snap: &WorldSnap, road: &Road, by: ColorBy) {
    let size = ui.available_size();
    let (resp, painter) = ui.allocate_painter(size, Sense::hover());
    let rect = resp.rect;
    painter.rect_filled(rect, 6.0, theme::bg_road(ui));
    let center = rect.center();
    let lanes = road.lanes as f32;
    let lane_px = (rect.width().min(rect.height()) * 0.035).clamp(8.0, 22.0);
    let radius = rect.width().min(rect.height()) * 0.5 - lane_px * (lanes + 1.0);
    let road_w = lane_px * lanes;
    painter.circle_stroke(center, radius + road_w * 0.5, Stroke::new(road_w + 6.0, theme::ASPHALT));
    for k in 1..road.lanes {
        let r = radius + lane_px * k as f32;
        painter.circle_stroke(center, r, Stroke::new(1.0, theme::LANE_MARK));
    }
    let scale = std::f32::consts::TAU * radius / road.length as f32; // px per m
    let v_ref = road.limit as f32;
    let mut mesh = Mesh::default();
    for d in &snap.vehicles {
        let lane_pos = d.lane as f32 - 1.0 + 0.5 - d.lat;
        let r = radius + lane_px * lane_pos;
        let mid = d.s - d.len * 0.5;
        let th = mid / road.length as f32 * std::f32::consts::TAU - std::f32::consts::FRAC_PI_2;
        let c = center + vec2(th.cos(), th.sin()) * r;
        let dir = vec2(-th.sin(), th.cos());
        let half_len = (d.len * 0.5 * scale * r / radius).max(2.0);
        quad(&mut mesh, c, dir, half_len, lane_px * 0.32, vehicle_color(d, by, v_ref));
        if d.flags & FLAG_BRAKING != 0 {
            quad(&mut mesh, c - dir * (half_len - 1.0), dir, 1.2, lane_px * 0.32, theme::BRAKE);
        }
    }
    painter.add(Shape::mesh(mesh));
    painter.text(
        center,
        egui::Align2::CENTER_CENTER,
        format!("{:.0} m ring\n{} vehicles\nmean {:.1} km/h", road.length, snap.vehicles.len(), snap.mean_speed * 3.6),
        egui::FontId::proportional(15.0),
        ui.visuals().text_color(),
    );
}
