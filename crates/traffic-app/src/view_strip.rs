//! Corridor strip: the whole 20-mile highway as a straight strip, eastbound
//! on top and westbound below, City A on the left. Click to move the
//! detail camera. Also the small ring minimap.

use egui::{Align2, Color32, FontId, Mesh, Pos2, Rect, Sense, Shape, Stroke, pos2, vec2};
use traffic_core::Road;
use traffic_core::road::{Dir, MILE, RampKind};
use traffic_core::snapshot::{FLAG_CRASHED, WorldSnap};

use crate::theme;
use crate::view_road::{ColorBy, quad, vehicle_color};

/// Camera shared by the strip and the detail view.
#[derive(Clone, Debug)]
pub struct Camera {
    /// Centre of the detail window, miles from City A.
    pub mile: f64,
    /// Half width of the detail window, m.
    pub half_m: f64,
    /// Follow this vehicle id, if set.
    pub follow: Option<u32>,
}

impl Default for Camera {
    fn default() -> Self {
        Camera { mile: 7.0, half_m: 250.0, follow: None }
    }
}

/// Vertical layout of one direction band.
pub struct Band {
    pub rect: Rect,
    pub dir: Dir,
    pub lanes: u8,
}

impl Band {
    pub fn lane_px(&self) -> f32 {
        self.rect.height() / (self.lanes as f32 + 1.0)
    }

    /// Centre y of `lane` (lane 0 = aux lane on the outer edge), with a
    /// fractional lateral offset `lat` toward the previous lane.
    pub fn lane_y(&self, lane: u8, lat: f32) -> f32 {
        let pos = lane as f32 - lat + 0.5;
        match self.dir {
            Dir::East => self.rect.top() + pos * self.lane_px(),
            Dir::West => self.rect.bottom() - pos * self.lane_px(),
        }
    }

    pub fn x(&self, mile: f64, m0: f64, m1: f64) -> f32 {
        self.rect.left() + ((mile - m0) / (m1 - m0)) as f32 * self.rect.width()
    }
}

/// Split a rect into the eastbound band (top) and westbound band (bottom).
pub fn bands(rect: Rect, lanes: u8, median: f32) -> (Band, Band) {
    let h = (rect.height() - median) * 0.5;
    let eb = Rect::from_min_size(rect.min, vec2(rect.width(), h));
    let wb = Rect::from_min_size(pos2(rect.left(), rect.top() + h + median), vec2(rect.width(), h));
    (Band { rect: eb, dir: Dir::East, lanes }, Band { rect: wb, dir: Dir::West, lanes })
}

/// Draw the asphalt, ramp stubs and markings for a mile window.
pub fn draw_road(painter: &egui::Painter, road: &Road, band: &Band, m0: f64, m1: f64, detail: bool) {
    let lp = band.lane_px();
    // Sample which mainline lanes exist across the window (lane drops,
    // work zones), every couple of pixels.
    let step = 2.0_f32;
    let cols = ((band.rect.width() / step).ceil() as usize).max(1);
    let mile_at = |k: usize| m0 + (m1 - m0) * ((k as f64 + 0.5) / cols as f64);
    let exists: Vec<Vec<bool>> = (0..=road.lanes)
        .map(|l| (0..cols).map(|k| l > 0 && road.lane_exists(road.from_corridor(band.dir, mile_at(k)), l)).collect())
        .collect();
    let x_of = |k: usize| band.rect.left() + k as f32 * step;
    // Runs of true values -> [x0, x1) pixel ranges.
    let runs = |v: &dyn Fn(usize) -> bool| {
        let mut out = Vec::new();
        let mut start = None;
        for k in 0..=cols {
            let on = k < cols && v(k);
            match (on, start) {
                (true, None) => start = Some(k),
                (false, Some(k0)) => {
                    out.push((x_of(k0), x_of(k).min(band.rect.right())));
                    start = None;
                }
                _ => {}
            }
        }
        out
    };
    for lane in 1..=road.lanes {
        let y = band.lane_y(lane, 0.0);
        for (x0, x1) in runs(&|k| exists[lane as usize][k]) {
            painter.rect_filled(Rect::from_x_y_ranges(x0..=x1, y - lp * 0.5..=y + lp * 0.5), 0.0, theme::ASPHALT);
        }
    }
    // Auxiliary lanes at ramps.
    for r in road.ramps.iter().filter(|r| r.dir == band.dir && !r.all_lanes) {
        let (a, b) = (road.to_corridor(r.zone_start), road.to_corridor(r.zone_end - 0.01));
        let (Some((_, ma)), Some((_, mb))) = (a, b) else { continue };
        let (lo, hi) = (ma.min(mb), ma.max(mb));
        if hi < m0 || lo > m1 {
            continue;
        }
        let y = band.lane_y(0, 0.0);
        let rr = Rect::from_x_y_ranges(band.x(lo, m0, m1)..=band.x(hi, m0, m1).max(band.x(lo, m0, m1) + 2.0), y - lp * 0.5..=y + lp * 0.5);
        painter.rect_filled(rr, 0.0, theme::ASPHALT);
        if detail && rr.width() > 90.0 && band.rect.contains_rect(rr.shrink2(egui::vec2(0.0, 1.0))) {
            let col = if r.kind == RampKind::On { theme::AUTO.gamma_multiply(0.5) } else { theme::LEFT.gamma_multiply(0.5) };
            painter.text(rr.center(), Align2::CENTER_CENTER, &r.name, FontId::proportional(11.0), col);
        }
    }
    if !detail {
        return;
    }
    // Edge lines: outside of lane 1, and wherever a lane has no neighbour
    // on its left. Dashed lines between two lanes that both exist.
    let edge = Stroke::new(1.5, theme::EDGE_MARK);
    let side = |lane: u8, toward: f32| band.lane_y(lane, 0.0) + (band.lane_y(lane + 1, 0.0) - band.lane_y(lane, 0.0)).signum() * toward * lp * 0.5;
    for (x0, x1) in runs(&|k| exists[1][k]) {
        let y = side(1, -1.0);
        painter.line_segment([pos2(x0, y), pos2(x1, y)], edge);
    }
    let px_per_m = band.rect.width() / ((m1 - m0) * MILE) as f32;
    let period = (12.0 * px_per_m).max(6.0);
    for lane in 1..=road.lanes {
        let l = lane as usize;
        let next = |k: usize| l < road.lanes as usize && exists[l + 1][k];
        let y = side(lane, 1.0);
        for (x0, x1) in runs(&|k| exists[l][k] && !next(k)) {
            painter.line_segment([pos2(x0, y), pos2(x1, y)], edge);
        }
        for (x0, x1) in runs(&|k| exists[l][k] && next(k)) {
            let mut x = x0;
            while x < x1 {
                painter.line_segment([pos2(x, y), pos2((x + period * 0.3).min(x1), y)], Stroke::new(1.0, theme::LANE_MARK));
                x += period;
            }
        }
    }
}

/// Mile range of a ring span [s0, s1) within one direction, if any.
fn span_miles(road: &Road, dir: Dir, s0: f64, s1: f64) -> Option<(f64, f64)> {
    let a = road.to_corridor(s0).filter(|x| x.0 == dir).map(|x| x.1);
    let b = road.to_corridor((s1 - 0.5).max(0.0)).filter(|x| x.0 == dir).map(|x| x.1);
    match (a, b) {
        (Some(a), Some(b)) => Some((a.min(b), a.max(b))),
        (Some(a), None) | (None, Some(a)) => Some((a, a)),
        _ => None,
    }
}

/// Closures (orange; crashes magenta), weather (blue) and comms outages
/// (purple) on one direction band.
pub fn draw_zones(painter: &egui::Painter, road: &Road, band: &Band, m0: f64, m1: f64, snap: &WorldSnap) {
    let m = &snap.metrics;
    let lp = band.lane_px();
    let xr = |lo: f64, hi: f64| {
        let (a, b) = (band.x(lo.max(m0), m0, m1), band.x(hi.min(m1), m0, m1));
        a.min(b)..=(a.max(b)).max(a.min(b) + 3.0)
    };
    for (s0, s1) in &m.weather {
        if let Some((lo, hi)) = span_miles(road, band.dir, *s0, *s1)
            && hi >= m0
            && lo <= m1
        {
            painter.rect_filled(Rect::from_x_y_ranges(xr(lo, hi), band.rect.y_range()), 0.0, Color32::from_rgba_unmultiplied(80, 140, 255, 40));
        }
    }
    for (s0, s1) in &m.outages {
        let full = road.dist(*s0, *s1) > road.length * 0.99 || (*s0 == 0.0 && *s1 >= road.length);
        let span = if full { Some((m0, m1)) } else { span_miles(road, band.dir, *s0, *s1) };
        if let Some((lo, hi)) = span {
            let y = match band.dir {
                Dir::East => band.rect.top() + 1.0,
                Dir::West => band.rect.bottom() - 1.0,
            };
            painter.line_segment([pos2(*xr(lo, hi).start(), y), pos2(*xr(lo, hi).end(), y)], Stroke::new(2.5, Color32::from_rgb(170, 90, 255)));
        }
    }
    for (s0, s1, lanes, crash) in &m.closures {
        if let Some((lo, hi)) = span_miles(road, band.dir, *s0, *s1)
            && hi >= m0
            && lo <= m1
        {
            let col = if *crash { theme::CRASH } else { theme::CLOSURE };
            for &l in lanes {
                let y = band.lane_y(l, 0.0);
                painter.rect_filled(Rect::from_x_y_ranges(xr(lo, hi), y - lp * 0.45..=y + lp * 0.45), 1.0, col.gamma_multiply(0.75));
            }
        }
    }
}

/// Draw vehicles of one direction into `mesh`.
#[allow(clippy::too_many_arguments)]
pub fn draw_vehicles(mesh: &mut Mesh, road: &Road, snap: &WorldSnap, band: &Band, m0: f64, m1: f64, by: ColorBy, min_len_px: f32) {
    let v_ref = road.limit as f32;
    let px_per_m = band.rect.width() / ((m1 - m0) * MILE) as f32;
    let lp = band.lane_px();
    let sign = if band.dir == Dir::East { 1.0 } else { -1.0 };
    for d in &snap.vehicles {
        let Some((dir, mile)) = road.to_corridor(d.s as f64) else { continue };
        if dir != band.dir || mile < m0 - 0.01 || mile > m1 + 0.01 {
            continue;
        }
        let half = (d.len * 0.5 * px_per_m).max(min_len_px);
        let x = band.x(mile, m0, m1) - sign * d.len * 0.5 * px_per_m;
        let y = band.lane_y(d.lane, d.lat);
        let c = vehicle_color(d, by, v_ref);
        quad(mesh, pos2(x, y), vec2(1.0, 0.0), half, (lp * 0.36).max(1.0), c);
    }
}

/// The overview strip. Returns a clicked mile (to move the camera).
pub fn strip(ui: &mut egui::Ui, road: &Road, snap: &WorldSnap, cam: &Camera, by: ColorBy, height: f32) -> Option<f64> {
    let (resp, painter) = ui.allocate_painter(vec2(ui.available_width(), height), Sense::click_and_drag());
    let rect = resp.rect;
    painter.rect_filled(rect, 4.0, theme::bg_road(ui));
    let label_w = 40.0;
    let inner = Rect::from_min_max(pos2(rect.left() + label_w, rect.top() + 26.0), pos2(rect.right() - 8.0, rect.bottom() - 4.0));
    let m1 = road.corridor / MILE;
    let (eb, wb) = bands(inner, road.lanes, 6.0);
    let mut mesh = Mesh::default();
    for b in [&eb, &wb] {
        draw_road(&painter, road, b, 0.0, m1, false);
        draw_vehicles(&mut mesh, road, snap, b, 0.0, m1, by, 1.0);
    }
    painter.add(Shape::mesh(mesh));
    for b in [&eb, &wb] {
        draw_zones(&painter, road, b, 0.0, m1, snap);
    }
    // Crash markers.
    for d in snap.vehicles.iter().filter(|d| d.flags & FLAG_CRASHED != 0) {
        if let Some((dir, mile)) = road.to_corridor(d.s as f64) {
            let b = if dir == Dir::East { &eb } else { &wb };
            painter.circle_stroke(pos2(b.x(mile, 0.0, m1), b.rect.center().y), 5.0, Stroke::new(1.5, theme::CRASH));
        }
    }
    let text = ui.visuals().text_color();
    let weak = ui.visuals().weak_text_color();
    for m in 0..=(m1 as i32) {
        let x = eb.x(m as f64, 0.0, m1);
        let major = m % 5 == 0;
        painter.line_segment([pos2(x, rect.top() + 22.0), pos2(x, rect.top() + if major { 26.0 } else { 24.0 })], Stroke::new(1.0, weak));
        if major {
            painter.text(pos2(x, rect.top() + 13.0), Align2::CENTER_TOP, format!("mi {m}"), FontId::proportional(10.0), weak);
        }
    }
    painter.text(pos2(rect.left() + 6.0, eb.rect.center().y), Align2::LEFT_CENTER, "EB >", FontId::proportional(11.0), text);
    painter.text(pos2(rect.left() + 6.0, wb.rect.center().y), Align2::LEFT_CENTER, "< WB", FontId::proportional(11.0), text);
    painter.text(pos2(rect.left() + 6.0, rect.top() + 1.0), Align2::LEFT_TOP, &road.city_a, FontId::proportional(11.0), text);
    painter.text(pos2(rect.right() - 6.0, rect.top() + 1.0), Align2::RIGHT_TOP, &road.city_b, FontId::proportional(11.0), text);
    // Camera window.
    let half_mi = cam.half_m / MILE;
    let (x0, x1) = (eb.x(cam.mile - half_mi, 0.0, m1), eb.x(cam.mile + half_mi, 0.0, m1));
    let win = Rect::from_x_y_ranges(x0.min(x1 - 3.0)..=x1.max(x0 + 3.0), inner.y_range());
    painter.rect_stroke(win, 2.0, Stroke::new(1.5, Color32::WHITE), egui::StrokeKind::Outside);
    if (resp.clicked() || resp.dragged())
        && let Some(p) = resp.interact_pointer_pos()
    {
        let mile = ((p.x - inner.left()) / inner.width()) as f64 * m1;
        return Some(mile.clamp(0.0, m1));
    }
    resp.on_hover_text("Click or drag to move the detail camera");
    None
}

/// Ring minimap: the whole 40-mile loop as a circle, coloured by speed.
pub fn minimap(painter: &egui::Painter, center: Pos2, radius: f32, road: &Road, snap: &WorldSnap, cam_s: f64) {
    painter.circle_filled(center, radius + 6.0, Color32::from_black_alpha(140));
    painter.circle_stroke(center, radius, Stroke::new(5.0, theme::ASPHALT));
    let v_ref = road.limit as f32;
    let mut mesh = Mesh::default();
    for d in &snap.vehicles {
        let th = d.s / road.length as f32 * std::f32::consts::TAU - std::f32::consts::FRAC_PI_2;
        let p = center + vec2(th.cos(), th.sin()) * radius;
        quad(&mut mesh, p, vec2(1.0, 0.0), 1.0, 1.0, crate::view_road::speed_color(d.v, v_ref));
    }
    painter.add(Shape::mesh(mesh));
    let th = cam_s as f32 / road.length as f32 * std::f32::consts::TAU - std::f32::consts::FRAC_PI_2;
    painter.circle_stroke(center + vec2(th.cos(), th.sin()) * radius, 4.0, Stroke::new(1.5, Color32::WHITE));
    let f = FontId::proportional(9.0);
    let w = Color32::from_gray(200);
    painter.text(center + vec2(0.0, -radius - 4.0), Align2::CENTER_BOTTOM, "A", f.clone(), w);
    painter.text(center + vec2(0.0, radius + 4.0), Align2::CENTER_TOP, "B", f, w);
}
