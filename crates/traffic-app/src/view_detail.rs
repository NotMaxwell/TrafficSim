//! Detail view: a zoomed window of the corridor showing both directions
//! with real lane geometry, vehicle bodies, brake lights and lane-change
//! motion. Scroll to zoom, drag to pan; the minimap shows where it is.

use egui::{Align2, Color32, FontId, Mesh, Rect, Sense, Shape, Stroke, pos2, vec2};
use traffic_core::Road;
use traffic_core::road::{Dir, MILE};
use traffic_core::snapshot::{
    FLAG_AUTO, FLAG_BRAKING, FLAG_CACC, FLAG_CRASHED, FLAG_DISTRACTED, FLAG_SIGNAL_LEFT, FLAG_SIGNAL_RIGHT, WorldSnap,
};

use crate::theme;
use crate::view_road::{ColorBy, quad, vehicle_color};
use crate::view_strip::{Band, Camera, bands, draw_road, draw_zones, minimap};

pub fn detail(ui: &mut egui::Ui, road: &Road, snap: &WorldSnap, cam: &mut Camera, by: ColorBy) {
    let size = ui.available_size();
    let (resp, painter) = ui.allocate_painter(size, Sense::click_and_drag());
    let rect = resp.rect;
    painter.rect_filled(rect, 4.0, theme::bg_road(ui));
    let m_max = road.corridor / MILE;

    // Follow camera: centre on the followed vehicle.
    if let Some(id) = cam.follow {
        match snap.vehicles.iter().find(|d| d.id == id).and_then(|d| road.to_corridor(d.s as f64)) {
            Some((_, mile)) => cam.mile = mile,
            None => cam.follow = None,
        }
    }
    // Interaction: scroll zooms, drag pans.
    if resp.hovered() {
        let scroll = ui.input(|i| i.smooth_scroll_delta.y);
        if scroll != 0.0 {
            cam.half_m = (cam.half_m * (1.0 - scroll as f64 * 0.002)).clamp(60.0, 8000.0);
        }
    }
    if resp.dragged() {
        let dx = resp.drag_delta().x as f64;
        cam.mile -= dx / rect.width() as f64 * 2.0 * cam.half_m / MILE;
        cam.follow = None;
    }
    let half = cam.half_m / MILE;
    cam.mile = cam.mile.clamp(half.min(m_max / 2.0), (m_max - half).max(m_max / 2.0));
    let (m0, m1) = (cam.mile - half, cam.mile + half);

    // Lanes are drawn ~2.5x wider than true scale so vehicles stay legible.
    let px_per_m = (rect.width() - 16.0) / (2.0 * cam.half_m) as f32;
    let lane_px = (3.7 * px_per_m * 2.5).clamp(9.0, 34.0);
    let band_h = lane_px * (road.lanes as f32 + 1.0);
    let road_h = (band_h * 2.0 + lane_px * 1.2).min(rect.height() - 60.0);
    let inner = Rect::from_center_size(rect.center(), vec2(rect.width() - 16.0, road_h));
    let (eb, wb) = bands(inner, road.lanes, lane_px * 1.2);
    for b in [&eb, &wb] {
        draw_road(&painter, road, b, m0, m1, true);
        draw_zones(&painter, road, b, m0, m1, snap);
    }
    let mut mesh = Mesh::default();
    let mut clicked_vehicle = None;
    let click = resp.clicked().then(|| resp.interact_pointer_pos()).flatten();
    for b in [&eb, &wb] {
        vehicles(&mut mesh, road, snap, b, m0, m1, by, cam.follow, click, &mut clicked_vehicle);
    }
    painter.add(Shape::mesh(mesh));
    if let Some(id) = clicked_vehicle {
        cam.follow = if cam.follow == Some(id) { None } else { Some(id) };
    }

    // Scale bar and labels.
    let text = ui.visuals().text_color();
    let bar_m = [10.0, 25.0, 50.0, 100.0, 250.0, 500.0, 1000.0, 2000.0].into_iter().find(|m| m * px_per_m > 60.0).unwrap_or(5000.0);
    let y = rect.bottom() - 14.0;
    let x0 = rect.left() + 14.0;
    painter.line_segment([pos2(x0, y), pos2(x0 + bar_m * px_per_m, y)], Stroke::new(2.0, text));
    painter.text(pos2(x0, y - 4.0), Align2::LEFT_BOTTOM, format!("{bar_m:.0} m"), FontId::proportional(11.0), text);
    painter.text(pos2(inner.left(), eb.rect.top() - 4.0), Align2::LEFT_BOTTOM, format!("Eastbound  >>  {}", road.city_b), FontId::proportional(12.0), text);
    painter.text(pos2(inner.right(), wb.rect.bottom() + 4.0), Align2::RIGHT_TOP, format!("{}  <<  Westbound", road.city_a), FontId::proportional(12.0), text);
    let follow = cam.follow.map_or(String::new(), |id| format!(" · following #{id}"));
    painter.text(
        pos2(rect.center().x, rect.top() + 6.0),
        Align2::CENTER_TOP,
        format!("mile {:.2} – {:.2}{follow}", m0.max(0.0), m1.min(m_max)),
        FontId::proportional(12.0),
        text,
    );
    let r = (rect.height().min(rect.width()) * 0.09).clamp(18.0, 40.0);
    let cam_s = road.from_corridor(Dir::East, cam.mile);
    minimap(&painter, pos2(rect.right() - r - 14.0, rect.top() + r + 14.0), r, road, snap, cam_s);
    resp.on_hover_text("Scroll to zoom · drag to pan · click a vehicle to follow it");
}

#[allow(clippy::too_many_arguments)]
fn vehicles(
    mesh: &mut Mesh,
    road: &Road,
    snap: &WorldSnap,
    band: &Band,
    m0: f64,
    m1: f64,
    by: ColorBy,
    follow: Option<u32>,
    click: Option<egui::Pos2>,
    clicked: &mut Option<u32>,
) {
    let v_ref = road.limit as f32;
    let px_per_m = band.rect.width() / ((m1 - m0) * MILE) as f32;
    let lp = band.lane_px();
    let fwd = if band.dir == Dir::East { 1.0 } else { -1.0 };
    for d in &snap.vehicles {
        let Some((dir, mile)) = road.to_corridor(d.s as f64) else { continue };
        if dir != band.dir || mile < m0 - 0.02 || mile > m1 + 0.02 {
            continue;
        }
        let half = (d.len * 0.5 * px_per_m).max(1.5);
        let cx = band.x(mile, m0, m1) - fwd * d.len * 0.5 * px_per_m;
        let c = pos2(cx, band.lane_y(d.lane, d.lat));
        let hw = (lp * 0.34).max(1.5);
        if follow == Some(d.id) {
            quad(mesh, c, vec2(1.0, 0.0), half + 3.0, hw + 3.0, Color32::WHITE);
        }
        quad(mesh, c, vec2(1.0, 0.0), half, hw, vehicle_color(d, by, v_ref));
        if half > 4.0 {
            let rear = c - vec2(fwd * (half - 1.5), 0.0);
            if d.flags & FLAG_BRAKING != 0 {
                quad(mesh, rear, vec2(1.0, 0.0), 1.5, hw, theme::BRAKE);
            }
            // Autos get a cyan roof stripe; CACC-linked ones a bright one.
            if d.flags & FLAG_AUTO != 0 && by == ColorBy::Speed {
                let col = if d.flags & FLAG_CACC != 0 { theme::AUTO } else { theme::AUTO.gamma_multiply(0.5) };
                quad(mesh, c, vec2(1.0, 0.0), half * 0.5, hw * 0.35, col);
            }
            // Blinkers (1 Hz): "left" is toward the next higher lane.
            let sig = if d.flags & FLAG_SIGNAL_LEFT != 0 { 1.0 } else if d.flags & FLAG_SIGNAL_RIGHT != 0 { -1.0 } else { 0.0 };
            if sig != 0.0 && snap.time.fract() < 0.5 {
                let up = (band.lane_y(2, 0.0) - band.lane_y(1, 0.0)).signum() * sig;
                let corner = c + vec2(fwd * (half - 1.5), up * (hw - 1.0));
                quad(mesh, corner, vec2(1.0, 0.0), 1.8, 1.8, theme::BLINKER);
            }
            if d.flags & FLAG_DISTRACTED != 0 {
                quad(mesh, c + vec2(fwd * half * 0.4, 0.0), vec2(1.0, 0.0), 2.0, 2.0, Color32::from_rgb(160, 60, 255));
            }
            if d.flags & FLAG_CRASHED != 0 {
                quad(mesh, c, vec2(0.7, 0.7), half.min(hw) * 1.4, 1.2, Color32::BLACK);
            }
        }
        if let Some(p) = click
            && (p.x - c.x).abs() <= half + 3.0
            && (p.y - c.y).abs() <= hw + 3.0
        {
            *clicked = Some(d.id);
        }
    }
}
