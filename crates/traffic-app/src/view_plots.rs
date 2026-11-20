//! Analysis panel: time-space heatmaps, fundamental diagram, throughput,
//! travel time, safety and fuel, plus a row of headline numbers. Both
//! worlds are overlaid in their pane colours.

use egui::{Color32, ColorImage, Rect, RichText, Sense, TextureHandle, TextureOptions, pos2, vec2};
use egui_plot::{Bar, BarChart, Legend, Line, Plot, PlotPoints, Points};
use traffic_core::Road;
use traffic_core::metrics::{CELL_M, CELL_S, Row};
use traffic_core::road::{Dir, MILE};
use traffic_core::snapshot::WorldSnap;

use crate::runner::Analysis;
use crate::theme;
use crate::view_road::speed_color;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Tab {
    TimeSpace,
    Fundamental,
    Throughput,
    TravelTime,
    Safety,
}

pub struct Plots {
    pub tab: Tab,
    pub dir: Dir,
    tex: [Option<TextureHandle>; 2],
    tex_key: [(u64, Dir); 2],
}

impl Default for Plots {
    fn default() -> Self {
        Plots { tab: Tab::TimeSpace, dir: Dir::East, tex: [None, None], tex_key: [(u64::MAX, Dir::East); 2] }
    }
}

const NAMES: [&str; 2] = ["Left", "Right"];
const COLORS: [Color32; 2] = [theme::LEFT, theme::RIGHT];

impl Plots {
    pub fn ui(&mut self, ui: &mut egui::Ui, road: &Road, worlds: &[WorldSnap], analysis: &[Analysis; 2]) {
        headline(ui, road, worlds, analysis);
        ui.separator();
        ui.horizontal(|ui| {
            for (t, label) in [
                (Tab::TimeSpace, "Time-space"),
                (Tab::Fundamental, "Flow vs density"),
                (Tab::Throughput, "Throughput"),
                (Tab::TravelTime, "Travel time"),
                (Tab::Safety, "Safety & fuel"),
            ] {
                ui.selectable_value(&mut self.tab, t, label);
            }
            if self.tab == Tab::TimeSpace && road.is_corridor() {
                ui.separator();
                ui.selectable_value(&mut self.dir, Dir::East, "Eastbound");
                ui.selectable_value(&mut self.dir, Dir::West, "Westbound");
            }
        });
        let rows = |k: usize| worlds.get(k).map_or(&[][..], |w| &w.metrics.rows[..]);
        match self.tab {
            Tab::TimeSpace => self.time_space(ui, road, worlds),
            Tab::Fundamental => fundamental(ui, worlds),
            Tab::Throughput => series(ui, "throughput", "veh/h leaving", [rows(0), rows(1)], |r| Some(r.throughput_vph), true),
            Tab::TravelTime => travel(ui, [rows(0), rows(1)]),
            Tab::Safety => safety(ui, [rows(0), rows(1)]),
        }
    }

    fn time_space(&mut self, ui: &mut egui::Ui, road: &Road, worlds: &[WorldSnap]) {
        let h = ui.available_height().max(60.0);
        ui.columns(2, |cols| {
            for (k, w) in worlds.iter().enumerate().take(2) {
                let ui = &mut cols[k];
                let key = (w.metrics.grid_version, self.dir);
                if self.tex_key[k] != key || self.tex[k].is_none() {
                    let img = heat_image(road, w, self.dir);
                    match &mut self.tex[k] {
                        Some(t) => t.set(img, TextureOptions::LINEAR),
                        None => self.tex[k] = Some(ui.ctx().load_texture(format!("heat{k}"), img, TextureOptions::LINEAR)),
                    }
                    self.tex_key[k] = key;
                }
                let (rect, _) = ui.allocate_exact_size(vec2(ui.available_width(), h), Sense::hover());
                let p = ui.painter_at(rect);
                p.rect_filled(rect, 2.0, Color32::from_gray(18));
                let plot = Rect::from_min_max(rect.min + vec2(40.0, 4.0), rect.max - vec2(4.0, 16.0));
                if let Some(t) = &self.tex[k]
                    && !w.metrics.grid.is_empty()
                {
                    p.image(t.id(), plot, Rect::from_min_max(pos2(0.0, 0.0), pos2(1.0, 1.0)), Color32::WHITE);
                }
                axes(&p, plot, road, w, self.dir, ui.visuals().weak_text_color());
                p.text(rect.left_top() + vec2(44.0, 6.0), egui::Align2::LEFT_TOP, NAMES[k], egui::FontId::proportional(11.0), COLORS[k]);
            }
        });
    }
}

/// Cell range [c0, c1) for a direction, ordered so that row 0 is the top
/// (highest mile on the corridor, highest s on a ring).
fn cell_rows(road: &Road, dir: Dir, cells: usize) -> Vec<usize> {
    if !road.is_corridor() {
        return (0..cells).rev().collect();
    }
    let c = (road.corridor / CELL_M) as usize;
    let wb0 = ((road.corridor + road.loop_len) / CELL_M) as usize;
    match dir {
        Dir::East => (0..c).rev().collect(),
        Dir::West => (wb0..(wb0 + c).min(cells)).collect(),
    }
}

fn heat_image(road: &Road, w: &WorldSnap, dir: Dir) -> ColorImage {
    let g = &w.metrics.grid;
    let cells = g.first().map_or(1, |c| c.len());
    let rows = cell_rows(road, dir, cells);
    let (wd, ht) = (g.len().max(1), rows.len().max(1));
    let empty = Color32::from_gray(30);
    let v_ref = road.limit as f32;
    let mut px = vec![empty; wd * ht];
    for (x, col) in g.iter().enumerate() {
        for (y, &c) in rows.iter().enumerate() {
            if let Some(&v) = col.get(c)
                && v.is_finite()
            {
                px[y * wd + x] = speed_color(v, v_ref);
            }
        }
    }
    ColorImage::new([wd, ht], px)
}

fn axes(p: &egui::Painter, plot: Rect, road: &Road, w: &WorldSnap, dir: Dir, col: Color32) {
    let font = egui::FontId::proportional(10.0);
    let n = w.metrics.grid.len();
    let (t0, t1) = (w.metrics.grid_t0 - CELL_S, w.metrics.grid_t0 + CELL_S * (n as f64 - 1.0));
    if n > 1 {
        let span = t1 - t0;
        let step = [60.0, 300.0, 600.0, 900.0, 1800.0].into_iter().find(|s| span / s <= 6.0).unwrap_or(3600.0);
        let mut t = (t0 / step).ceil() * step;
        while t <= t1 {
            let x = plot.left() + ((t - t0) / span) as f32 * plot.width();
            p.text(pos2(x, plot.bottom() + 2.0), egui::Align2::CENTER_TOP, format!("{:.0} min", t / 60.0), font.clone(), col);
            t += step;
        }
    }
    let (label_top, label_bot, ticks): (String, String, Vec<(f32, String)>) = if road.is_corridor() {
        let m = road.corridor / MILE;
        let ticks = (0..=(m as i32)).step_by(5).map(|k| (1.0 - k as f32 / m as f32, format!("mi {k}"))).collect();
        let arrow = if dir == Dir::East { "EB, travel ^" } else { "WB, travel v" };
        (arrow.to_string(), String::new(), ticks)
    } else {
        (format!("{:.0} m", road.length), "0 m".into(), Vec::new())
    };
    for (f, t) in ticks {
        p.text(pos2(plot.left() - 3.0, plot.top() + f * plot.height()), egui::Align2::RIGHT_CENTER, t, font.clone(), col);
    }
    p.text(pos2(plot.right() - 2.0, plot.top() + 2.0), egui::Align2::RIGHT_TOP, label_top, font.clone(), col);
    if !label_bot.is_empty() {
        p.text(pos2(plot.left() - 3.0, plot.bottom()), egui::Align2::RIGHT_BOTTOM, label_bot, font, col);
    }
    // Crash markers.
    let cells_rows = cell_rows(road, dir, w.metrics.grid.first().map_or(1, |c| c.len()));
    if n > 1 {
        for &(t, s) in &w.metrics.crashes {
            let c = (s / CELL_M) as usize;
            if let Some(y) = cells_rows.iter().position(|&r| r == c) {
                let x = plot.left() + ((t - t0) / (t1 - t0)) as f32 * plot.width();
                let y = plot.top() + (y as f32 + 0.5) / cells_rows.len() as f32 * plot.height();
                if plot.contains(pos2(x, y)) {
                    p.circle_stroke(pos2(x, y), 4.0, egui::Stroke::new(1.5, theme::CRASH));
                }
            }
        }
    }
}

fn headline(ui: &mut egui::Ui, road: &Road, worlds: &[WorldSnap], analysis: &[Analysis; 2]) {
    let fmt_shock = |v: Option<f64>| v.map_or("–".to_string(), |x| format!("{:.0} km/h {}", x.abs(), if x < 0.0 { "upstream" } else { "downstream" }));
    egui::Grid::new("headline").num_columns(8).spacing([14.0, 2.0]).show(ui, |ui| {
        for h in ["", "throughput (10 min)", "end-to-end travel (median / p90)", "mean speed", "hard brakes /h", "crashes", "fuel proxy"] {
            ui.label(RichText::new(h).small().weak());
        }
        ui.label(RichText::new(if road.is_corridor() { "shockwave EB / WB · amplification" } else { "shockwave · amplification" }).small().weak());
        ui.end_row();
        let fuel0 = worlds.first().and_then(|w| w.metrics.rows.last()).map_or(0.0, |r| r.fuel_proxy);
        for (k, w) in worlds.iter().enumerate().take(2) {
            let rows = &w.metrics.rows;
            let recent: Vec<&Row> = rows.iter().rev().take(10).collect();
            let avg = |f: fn(&Row) -> f64| {
                let v: Vec<f64> = recent.iter().map(|r| f(r)).filter(|x| x.is_finite()).collect();
                if v.is_empty() { f64::NAN } else { v.iter().sum::<f64>() / v.len() as f64 }
            };
            let last = rows.last().copied().unwrap_or_default();
            ui.label(RichText::new(NAMES[k]).color(COLORS[k]).strong());
            ui.monospace(format!("{:.0} veh/h", avg(|r| r.throughput_vph)));
            ui.monospace(if last.travel_med_min.is_finite() { format!("{:.1} / {:.1} min", last.travel_med_min, last.travel_p90_min) } else { "–".into() });
            ui.monospace(format!("{:.0} km/h", w.mean_speed * 3.6));
            ui.monospace(format!("{:.0}", avg(|r| r.hard_brakes) * 60.0));
            ui.monospace(format!("{}", w.stats.collisions));
            let rel = if k == 1 && fuel0 > 0.0 { format!(" ({:+.0}%)", (last.fuel_proxy / fuel0 - 1.0) * 100.0) } else { String::new() };
            ui.monospace(format!("{:.0}{rel}", last.fuel_proxy));
            let a = &analysis[k];
            let amp = a.amplification.map_or("–".into(), |x| format!("{x:.2}"));
            let shock = if road.is_corridor() { format!("{} / {}", fmt_shock(a.shock_eb), fmt_shock(a.shock_wb)) } else { fmt_shock(a.shock_eb) };
            ui.monospace(format!("{shock} · {amp}"));
            ui.end_row();
        }
    });
}

fn fundamental(ui: &mut egui::Ui, worlds: &[WorldSnap]) {
    Plot::new("fd")
        .legend(Legend::default())
        .x_axis_label("density (veh/km/lane)")
        .y_axis_label("flow (veh/h/lane)")
        .include_x(0.0)
        .include_y(0.0)
        .show(ui, |p| {
            for (k, w) in worlds.iter().enumerate().take(2) {
                let pts: Vec<[f64; 2]> = w.metrics.fd.iter().map(|q| [q[0] as f64, q[1] as f64]).collect();
                p.points(Points::new(NAMES[k], PlotPoints::from(pts)).color(COLORS[k].gamma_multiply(0.7)).radius(2.0));
            }
        });
}

fn series(ui: &mut egui::Ui, id: &str, y_label: &str, rows: [&[Row]; 2], f: impl Fn(&Row) -> Option<f64>, smooth: bool) {
    Plot::new(id).legend(Legend::default()).x_axis_label("time (min)").y_axis_label(y_label).include_y(0.0).show(ui, |p| {
        for (k, rs) in rows.iter().enumerate() {
            let mut pts: Vec<[f64; 2]> = rs.iter().filter_map(|r| f(r).filter(|v| v.is_finite()).map(|v| [r.t / 60.0, v])).collect();
            if smooth && pts.len() > 5 {
                let raw = pts.clone();
                for (i, pt) in pts.iter_mut().enumerate() {
                    let lo = i.saturating_sub(2);
                    let hi = (i + 3).min(raw.len());
                    pt[1] = raw[lo..hi].iter().map(|q| q[1]).sum::<f64>() / (hi - lo) as f64;
                }
            }
            p.line(Line::new(NAMES[k], PlotPoints::from(pts)).color(COLORS[k]).width(2.0));
        }
    });
}

fn travel(ui: &mut egui::Ui, rows: [&[Row]; 2]) {
    Plot::new("travel").legend(Legend::default()).x_axis_label("time (min)").y_axis_label("end-to-end travel time (min)").include_y(0.0).show(ui, |p| {
        for (k, rs) in rows.iter().enumerate() {
            let med: Vec<[f64; 2]> = rs.iter().filter(|r| r.travel_med_min.is_finite()).map(|r| [r.t / 60.0, r.travel_med_min]).collect();
            let p90: Vec<[f64; 2]> = rs.iter().filter(|r| r.travel_p90_min.is_finite()).map(|r| [r.t / 60.0, r.travel_p90_min]).collect();
            p.line(Line::new(format!("{} median", NAMES[k]), PlotPoints::from(med)).color(COLORS[k]).width(2.0));
            p.line(Line::new(format!("{} 90th pct", NAMES[k]), PlotPoints::from(p90)).color(COLORS[k].gamma_multiply(0.5)).width(1.5));
        }
    });
}

fn safety(ui: &mut egui::Ui, rows: [&[Row]; 2]) {
    ui.columns(2, |cols| {
        Plot::new("brakes").legend(Legend::default()).x_axis_label("time (min)").y_axis_label("hard brakes per 5 min").include_y(0.0).show(&mut cols[0], |p| {
            for (k, rs) in rows.iter().enumerate() {
                let bars: Vec<Bar> = rs
                    .chunks(5)
                    .map(|c| Bar::new(c[0].t / 60.0 + 2.5 + (k as f64 - 0.5) * 2.0, c.iter().map(|r| r.hard_brakes).sum()).width(2.0))
                    .collect();
                p.bar_chart(BarChart::new(NAMES[k], bars).color(COLORS[k]));
            }
        });
        Plot::new("crashes").legend(Legend::default()).x_axis_label("time (min)").y_axis_label("crashes (cumulative) · fuel proxy / 100").include_y(0.0).show(&mut cols[1], |p| {
            for (k, rs) in rows.iter().enumerate() {
                let c: Vec<[f64; 2]> = rs.iter().map(|r| [r.t / 60.0, r.collisions]).collect();
                let f: Vec<[f64; 2]> = rs.iter().map(|r| [r.t / 60.0, r.fuel_proxy / 100.0]).collect();
                p.line(Line::new(format!("{} crashes", NAMES[k]), PlotPoints::from(c)).color(COLORS[k]).width(2.0));
                p.line(Line::new(format!("{} fuel", NAMES[k]), PlotPoints::from(f)).color(COLORS[k].gamma_multiply(0.5)).width(1.5));
            }
        });
    });
}
