//! Painting: animated engine cross-section, gauges and plots. Pure drawing from a
//! `Snapshot`; checked by looking at the running app (see PR verification), not by
//! line coverage.

use crate::dyno::{DynoPoint, peaks};
use crate::sim::Snapshot;
use eframe::egui::{self, Align2, Color32, FontId, Pos2, Stroke, Vec2, pos2, vec2};
use engine_core::units::{ATM_PA, m3_to_cc, pa_to_bar, w_to_hp};

const STEEL: Color32 = Color32::from_rgb(150, 155, 165);
const DARK: Color32 = Color32::from_rgb(60, 62, 70);
const INTAKE: Color32 = Color32::from_rgb(80, 150, 255);
const EXHAUST: Color32 = Color32::from_rgb(255, 110, 80);

/// Gas colour from temperature: cool blue → hot orange → white-yellow.
fn gas_color(t_k: f64, burning: bool) -> Color32 {
    if burning {
        return Color32::from_rgb(255, 190, 60);
    }
    let x = ((t_k - 300.0) / 1700.0).clamp(0.0, 1.0) as f32;
    let lerp = |a: f32, b: f32| (a + (b - a) * x) as u8;
    Color32::from_rgb(lerp(70.0, 255.0), lerp(110.0, 120.0), lerp(200.0, 40.0))
}

fn stroke_name(cycle_deg: f64) -> &'static str {
    match cycle_deg {
        d if d < 180.0 => "power",
        d if d < 360.0 => "exhaust",
        d if d < 540.0 => "intake",
        _ => "compression",
    }
}

/// Animated cross-section of every cylinder, side by side.
/// A cylinder's own frame: u across the bore, v up the bore axis (metres), origin at the
/// crank, the axis tilted `tilt` rad from vertical (V and flat banks), `scale` px per metre.
struct Frame {
    crank: Pos2,
    cos: f32,
    sin: f32,
    scale: f32,
}

impl Frame {
    fn at(&self, u: f64, v: f64) -> Pos2 {
        let (u, v) = (u as f32 * self.scale, v as f32 * self.scale);
        self.crank + vec2(u * self.cos + v * self.sin, -(v * self.cos - u * self.sin))
    }
    /// Metres for `px` screen pixels (outlines and offsets keep their pixel size).
    fn px(&self, px: f32) -> f64 {
        (px / self.scale) as f64
    }
    fn quad(&self, u0: f64, u1: f64, v0: f64, v1: f64) -> Vec<Pos2> {
        vec![
            self.at(u0, v0),
            self.at(u1, v0),
            self.at(u1, v1),
            self.at(u0, v1),
        ]
    }
}

pub fn engine(ui: &mut egui::Ui, s: &Snapshot) {
    let (rect, _) = ui.allocate_exact_size(ui.available_size(), egui::Sense::hover());
    let p = ui.painter_at(rect);
    p.rect_filled(rect, 0.0, Color32::from_rgb(24, 26, 30));
    let g = s.def.geometry;
    let n = s.cylinders.len().max(1);
    let (r, l, b) = (g.crank_radius(), g.rod_m, g.bore_m);
    let clear_h = g.clearance_volume() / g.piston_area();
    let total_h = r + l + r + 0.6 * b + clear_h + 0.35 * b + 0.6 * b;
    // Two banks side by side per crank throw (V, flat), one column per throw.
    let layout = &s.def.layout;
    let place = layout.place(n);
    let cols = place.iter().map(|q| q.1 + 1).max().unwrap_or(1);
    let vee = layout.bank_angle_deg > 0.0;
    let half_angle = (0.5 * layout.bank_angle_deg.clamp(0.0, 180.0)).to_radians();
    let below = r + 0.4 * b;
    let up = total_h - below;
    let (sh, ch) = (half_angle.sin(), half_angle.cos());
    let half_w = up * sh + 0.95 * b * ch;
    let above = up * ch + 0.95 * b * sh;
    let height = above + below;
    let cell_w = rect.width() / cols as f32;
    let scale = (0.85 * rect.height() / height as f32).min(0.9 * cell_w / (2.0 * half_w as f32));
    let crank_y = rect.center().y + 0.5 * (above - below) as f32 * scale;

    for (i, c) in s.cylinders.iter().enumerate() {
        let (bank, pos) = place[i];
        let tilt = if !vee {
            0.0
        } else if bank == 0 {
            -half_angle
        } else {
            half_angle
        } as f32;
        let f = Frame {
            crank: pos2(rect.left() + cell_w * (pos as f32 + 0.5), crank_y),
            cos: tilt.cos(),
            sin: tilt.sin(),
            scale,
        };
        cylinder(&p, &f, c, i, s, cell_w);
    }
}

fn cylinder(
    p: &egui::Painter,
    f: &Frame,
    c: &engine_core::engine::CylinderView,
    i: usize,
    s: &Snapshot,
    cell_w: f32,
) {
    let g = s.def.geometry;
    let (r, l, b) = (g.crank_radius(), g.rod_m, g.bore_m);
    let clear_h = g.clearance_volume() / g.piston_area();
    let piston_h = 0.6 * b;
    let head_h = 0.35 * b;
    let th = c.cycle_deg.to_radians();
    let pin = (r * th.sin(), r * th.cos());
    let wrist_v = r * th.cos() + (l * l - (r * th.sin()).powi(2)).sqrt();
    let crown_v = wrist_v + 0.35 * b;
    let deck_v = r + l + 0.35 * b + clear_h;
    let skirt_v = l - r + 0.35 * b - piston_h;
    let half = 0.5 * b;

    // Chamber gas, walls, head.
    p.add(egui::Shape::convex_polygon(
        f.quad(-half, half, crown_v, deck_v),
        gas_color(c.temperature_k, c.burning),
        Stroke::NONE,
    ));
    let wall = Stroke::new(3.0, STEEL);
    for side in [-1.0, 1.0] {
        let u = side * (half + f.px(2.0));
        p.line_segment([f.at(u, deck_v), f.at(u, skirt_v)], wall);
    }
    let lip = f.px(6.0);
    p.add(egui::Shape::convex_polygon(
        f.quad(-half - lip, half + lip, deck_v, deck_v + head_h),
        DARK,
        Stroke::NONE,
    ));

    // Valves: stem + disc, pushed down by lift (drawn 1:1).
    for (du, lift, col) in [
        (-0.25, c.intake_lift_m, INTAKE),
        (0.25, c.exhaust_lift_m, EXHAUST),
    ] {
        let u = du * b;
        let v = deck_v - lift;
        p.line_segment(
            [f.at(u, deck_v + head_h + f.px(8.0)), f.at(u, v)],
            Stroke::new(2.0, col),
        );
        let (vr, t) = (0.16 * b, f.px(1.5));
        p.add(egui::Shape::convex_polygon(
            f.quad(u - vr, u + vr, v - t, v + t),
            col,
            Stroke::NONE,
        ));
    }

    // Piston, rod, crank.
    p.add(egui::Shape::convex_polygon(
        f.quad(-half, half, crown_v - piston_h, crown_v),
        STEEL,
        Stroke::new(1.0, DARK),
    ));
    let scale = f.scale;
    p.circle_filled(f.crank, (r * 1.35) as f32 * scale, DARK);
    p.line_segment(
        [f.at(pin.0, pin.1), f.at(0.0, wrist_v)],
        Stroke::new(
            ((0.12 * b) as f32 * scale).max(3.0),
            Color32::from_rgb(190, 190, 200),
        ),
    );
    p.circle_filled(f.at(0.0, wrist_v), 3.0, DARK);
    p.circle_filled(
        f.at(pin.0, pin.1),
        ((0.1 * b) as f32 * scale).max(3.0),
        Color32::WHITE,
    );
    p.circle_filled(f.crank, 3.0, STEEL);

    let (num, stroke, bar) = (i + 1, stroke_name(c.cycle_deg), pa_to_bar(c.pressure_pa));
    if f.sin == 0.0 {
        let font = FontId::proportional((cell_w / 7.0).clamp(9.0, 14.0));
        p.text(
            f.at(0.0, deck_v + head_h + f.px(14.0)),
            Align2::CENTER_BOTTOM,
            format!("#{num}\n{stroke}\n{bar:.1} bar"),
            font,
            Color32::LIGHT_GRAY,
        );
    } else {
        // V and flat: the two banks share a column, so one line each under the crank, the
        // left bank's first.
        let size = (cell_w / 12.0).clamp(8.0, 13.0);
        let line = if f.sin < 0.0 { 0.0 } else { 1.0 };
        p.text(
            f.crank + vec2(0.0, (r * 1.35) as f32 * f.scale + 4.0 + line * (size + 3.0)),
            Align2::CENTER_TOP,
            format!("#{num} {stroke} {bar:.1} bar"),
            FontId::proportional(size),
            Color32::LIGHT_GRAY,
        );
    }
}

pub fn gauges(ui: &mut egui::Ui, s: &Snapshot, audio: &Result<u32, String>) {
    ui.heading(format!("{:.0} rpm", s.rpm));
    ui.add(
        egui::ProgressBar::new((s.rpm / s.def.redline_rpm).clamp(0.0, 1.0) as f32)
            .text(format!("redline {:.0}", s.def.redline_rpm)),
    );
    ui.heading(format!(
        "{:.0} km/h  ·  gear {}",
        s.speed_m_s * 3.6,
        if s.gear == 0 {
            "N".to_string()
        } else {
            s.gear.to_string()
        }
    ));
    ui.label(format!("Clutch  {:.0} % engaged", s.clutch * 100.0));
    ui.label(format!("Torque  {:.1} N·m", s.torque_nm));
    ui.label(format!("Power   {:.1} hp", w_to_hp(s.power_w).max(0.0)));
    ui.label(format!("Crank   {:.0}° / 720°", s.crank_deg));
    ui.label(format!("Manifold {:.2} bar", pa_to_bar(s.plenum_pa)));
    ui.add(
        egui::ProgressBar::new((s.plenum_pa / ATM_PA).clamp(0.0, 1.0) as f32)
            .text("manifold / atm"),
    );
    ui.separator();
    ui.label(format!(
        "{}  ·  {} cyl  ·  {:.0} cc",
        s.def.name,
        s.def.cylinders,
        m3_to_cc(s.def.displacement_m3())
    ));
    ui.label(match audio {
        Ok(sr) => format!("Audio: {sr} Hz"),
        Err(e) => format!("No audio: {e}"),
    });
}

/// Line plot of `ys` in a box, scaled to [lo, hi].
fn polyline(
    ui: &mut egui::Ui,
    title: &str,
    ys: &[f32],
    lo: f32,
    hi: f32,
    color: Color32,
    height: f32,
) {
    ui.label(title);
    let (rect, _) = ui.allocate_exact_size(
        Vec2::new(ui.available_width(), height),
        egui::Sense::hover(),
    );
    let p = ui.painter_at(rect);
    p.rect_filled(rect, 2.0, Color32::from_rgb(20, 22, 26));
    if ys.len() < 2 {
        return;
    }
    let span = (hi - lo).max(1e-9);
    let pts: Vec<Pos2> = ys
        .iter()
        .enumerate()
        .map(|(i, y)| {
            let x = rect.left() + rect.width() * i as f32 / (ys.len() - 1) as f32;
            pos2(
                x,
                rect.bottom() - rect.height() * ((y - lo) / span).clamp(0.0, 1.0),
            )
        })
        .collect();
    p.add(egui::Shape::line(pts, Stroke::new(1.5, color)));
}

pub fn plots(ui: &mut egui::Ui, s: &Snapshot, dyno: &[DynoPoint]) {
    let bar: Vec<f32> = s.pressure_trace.iter().map(|p| p / 1e5).collect();
    let max = bar.iter().cloned().fold(1.0f32, f32::max);
    ui.columns(3, |cols| {
        polyline(
            &mut cols[0],
            &format!("Cylinder 1 pressure over the cycle (0–720°, peak {max:.0} bar)"),
            &bar,
            0.0,
            max * 1.05,
            Color32::from_rgb(255, 170, 60),
            110.0,
        );
        polyline(
            &mut cols[1],
            "Sound (speaker signal)",
            &s.scope,
            -1.0,
            1.0,
            Color32::from_rgb(120, 220, 140),
            110.0,
        );
        dyno_plot(&mut cols[2], dyno);
    });
}

/// Torque and power curves of the last dyno sweep, each scaled to its own peak.
fn dyno_plot(ui: &mut egui::Ui, points: &[DynoPoint]) {
    let Some((t, p)) = peaks(points) else {
        ui.label("Dyno: press \"Run dyno sweep\" for the torque and power curves");
        return;
    };
    let torque: Vec<f32> = points.iter().map(|d| d.torque_nm as f32).collect();
    let power: Vec<f32> = points.iter().map(|d| d.power_hp() as f32).collect();
    let (first, last) = (points[0].rpm, points[points.len() - 1].rpm);
    polyline(
        ui,
        &format!(
            "Dyno {first:.0}-{last:.0} rpm: torque, peak {:.0} N·m at {:.0}",
            t.torque_nm, t.rpm
        ),
        &torque,
        0.0,
        t.torque_nm as f32 * 1.05,
        Color32::from_rgb(255, 170, 60),
        48.0,
    );
    polyline(
        ui,
        &format!("power, peak {:.0} hp at {:.0}", p.power_hp(), p.rpm),
        &power,
        0.0,
        p.power_hp() as f32 * 1.05,
        Color32::from_rgb(110, 170, 255),
        48.0,
    );
}

/// The engine's pipes as a schematic ([`crate::piping`]): every part at its real length and bore,
/// fitted to the panel, coloured by what flows through it.
pub fn piping(ui: &mut egui::Ui, def: &engine_core::engine::EngineDef) {
    use crate::piping::{Part, schematic};
    let (rect, _) = ui.allocate_exact_size(ui.available_size(), egui::Sense::hover());
    let p = ui.painter_at(rect);
    p.rect_filled(rect, 0.0, Color32::from_rgb(24, 26, 30));
    let items = schematic(def);
    if items.is_empty() {
        return;
    }
    let (mut x0, mut x1, mut y0, mut y1) = (f64::MAX, f64::MIN, f64::MAX, f64::MIN);
    for i in &items {
        x0 = x0.min(i.x0);
        x1 = x1.max(i.x1);
        y0 = y0.min(i.y0);
        y1 = y1.max(i.y1);
    }
    let inner = rect.shrink(30.0);
    let scale = (inner.width() as f64 / (x1 - x0)).min(inner.height() as f64 / (y1 - y0)) as f32;
    let at = |x: f64, y: f64| {
        inner.left_top() + vec2(((x - x0) as f32) * scale, ((y - y0) as f32) * scale)
    };
    let font = FontId::proportional(11.0);
    for i in &items {
        let colour = match i.part {
            Part::Air | Part::Inlet | Part::Tank | Part::Plenum | Part::Runner => INTAKE,
            Part::Throttle => STEEL,
            Part::Cylinder => DARK,
            Part::Catalyst => Color32::from_rgb(200, 170, 90),
            Part::Muffler | Part::Resonator => Color32::from_rgb(170, 120, 110),
            _ => EXHAUST,
        };
        let r = egui::Rect::from_min_max(at(i.x0, i.y0), at(i.x1, i.y1));
        match i.part {
            Part::Air | Part::Outlet => {
                p.arrow(
                    pos2(r.left(), r.center().y),
                    vec2(r.width().max(12.0), 0.0),
                    Stroke::new(2.0, colour),
                );
            }
            Part::Cylinder => {
                p.rect(
                    r,
                    2.0,
                    colour,
                    Stroke::new(1.0, STEEL),
                    egui::StrokeKind::Inside,
                );
            }
            Part::Merge => {
                p.line_segment([at(i.x0, i.y0), at(i.x1, i.y1)], Stroke::new(2.0, colour));
            }
            _ => {
                p.rect_filled(r, 1.0, colour.gamma_multiply(0.85));
            }
        }
        if !i.label.is_empty() && (r.width() > 60.0 || matches!(i.part, Part::Cylinder)) {
            let pos = if matches!(i.part, Part::Cylinder) {
                r.center()
            } else {
                pos2(r.center().x, r.top() - 2.0)
            };
            let align = if matches!(i.part, Part::Cylinder) {
                Align2::CENTER_CENTER
            } else {
                Align2::CENTER_BOTTOM
            };
            p.text(pos, align, &i.label, font.clone(), Color32::from_gray(210));
        }
    }
    p.text(
        rect.left_bottom() + vec2(8.0, -8.0),
        Align2::LEFT_BOTTOM,
        "to scale · air -> intake -> cylinders -> exhaust -> outlets",
        FontId::proportional(11.0),
        Color32::from_gray(150),
    );
}
