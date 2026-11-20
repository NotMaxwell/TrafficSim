//! Self-screenshots for docs/progress: `--screenshot <file.ppm> --after <s>`
//! captures the app's own framebuffer (never the rest of the screen),
//! writes a binary PPM and closes the window.

use std::time::Instant;

pub struct Capture {
    path: String,
    after: f64,
    started: Instant,
    requested: bool,
}

impl Capture {
    pub fn from_args(args: &[String]) -> Option<Capture> {
        let i = args.iter().position(|a| a == "--screenshot")?;
        let path = args.get(i + 1)?.clone();
        let after = args
            .iter()
            .position(|a| a == "--after")
            .and_then(|j| args.get(j + 1))
            .and_then(|s| s.parse().ok())
            .unwrap_or(10.0);
        Some(Capture { path, after, started: Instant::now(), requested: false })
    }

    /// Call every frame.
    pub fn update(&mut self, ctx: &egui::Context) {
        if !self.requested && self.started.elapsed().as_secs_f64() >= self.after {
            self.requested = true;
            ctx.send_viewport_cmd(egui::ViewportCommand::Screenshot(egui::UserData::default()));
        }
        let shot = ctx.input(|i| {
            i.raw.events.iter().find_map(|e| match e {
                egui::Event::Screenshot { image, .. } => Some(image.clone()),
                _ => None,
            })
        });
        if let Some(img) = shot {
            let [w, h] = img.size;
            let mut out = format!("P6\n{w} {h}\n255\n").into_bytes();
            out.reserve(w * h * 3);
            for p in &img.pixels {
                out.extend_from_slice(&[p.r(), p.g(), p.b()]);
            }
            if let Err(e) = std::fs::write(&self.path, out) {
                eprintln!("screenshot: {e}");
            }
            ctx.send_viewport_cmd(egui::ViewportCommand::Close);
        }
        ctx.request_repaint();
    }
}
