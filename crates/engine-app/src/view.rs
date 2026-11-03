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
    if let Some(rotor) = g.rotor {
        rotary(&p, rect, s, &rotor);
        return;
    }
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

/// Point of a Wankel housing's epitrochoid at parameter `alpha` rad, for eccentricity `e` and
/// generating radius `r`: (e·cos 3α + R·cos α, e·sin 3α + R·sin α) (Yamamoto, *Rotary Engine*,
/// 1981, §2; Heywood 2nd ed., §1.8). Major axis along x, minor (R − e) along y.
pub(crate) fn housing_point(alpha: f64, e: f64, r: f64) -> (f64, f64) {
    (
        e * (3.0 * alpha).cos() + r * alpha.cos(),
        e * (3.0 * alpha).sin() + r * alpha.sin(),
    )
}

/// Rotor centre and its three apexes for a chamber at cycle angle `psi_deg` (ψ = ⅔ of the
/// shaft angle). The rotor turns at a third of the shaft (ψ/2 per ψ); the drawing puts that
/// chamber's face mid-direction at 90° + ψ/2, so it is at its smallest at ψ = 0 with the face
/// up against the housing's waist (the spark plugs), largest at ψ = 180°. Apex k sits at
/// rotor angle θ + 120°·k with θ = 30° + ψ/2, the centre on the eccentric at 3θ, so every apex
/// rides on the housing: housing_point(θ + 120°·k).
pub(crate) fn rotor_pose(psi_deg: f64, e: f64, r: f64) -> ((f64, f64), [(f64, f64); 3]) {
    let theta = (30.0 + 0.5 * psi_deg).to_radians();
    let c = (e * (3.0 * theta).cos(), e * (3.0 * theta).sin());
    let apex = |k: f64| {
        let a = theta + k * 2.0 * std::f64::consts::FRAC_PI_3;
        (c.0 + r * a.cos(), c.1 + r * a.sin())
    };
    (c, [apex(0.0), apex(1.0), apex(2.0)])
}

/// Animated cross-section of a Wankel engine: one housing per rotor, the rotor turning on its
/// eccentric, each chamber filled with its gas colour, spark plugs at the waist (top), the
/// exhaust and intake ports below lit by their opening.
fn rotary(p: &egui::Painter, rect: egui::Rect, s: &Snapshot, rotor: &engine_core::geometry::Rotor) {
    use std::f64::consts::{FRAC_PI_3, TAU};
    let (e, r) = (rotor.eccentricity_m, rotor.generating_radius_m);
    let rotors = s.cylinders.len().div_ceil(3).max(1);
    let cell_w = rect.width() / rotors as f32;
    let scale =
        (0.8 * rect.height() / (2.0 * (r - e)) as f32).min(0.85 * cell_w / (2.0 * (r + e)) as f32);
    let font = FontId::proportional((cell_w / 22.0).clamp(9.0, 13.0));
    for k in 0..rotors {
        let centre = pos2(
            rect.left() + cell_w * (k as f32 + 0.5),
            rect.center().y - 0.06 * rect.height(),
        );
        let at = |(x, y): (f64, f64)| centre + vec2(x as f32 * scale, -(y as f32) * scale);
        let chambers = &s.cylinders[3 * k..(3 * k + 3).min(s.cylinders.len())];
        // Housing.
        let n = 96;
        let outline: Vec<Pos2> = (0..n)
            .map(|i| at(housing_point(TAU * i as f64 / n as f64, e, r)))
            .collect();
        p.add(egui::Shape::closed_line(outline, Stroke::new(4.0, STEEL)));
        // Chambers: a strip from the housing arc between two apexes to the rotor flank.
        let Some(first) = chambers.first() else {
            continue;
        };
        let (c, _) = rotor_pose(first.cycle_deg, e, r);
        for ch in chambers {
            let mid = (90.0 + 0.5 * ch.cycle_deg).to_radians();
            let (a0, a1) = (mid - FRAC_PI_3, mid + FRAC_PI_3);
            let (p0, p1) = (
                (c.0 + r * a0.cos(), c.1 + r * a0.sin()),
                (c.0 + r * a1.cos(), c.1 + r * a1.sin()),
            );
            // The flank bulges out of the apex chord by most of the gap left at the waist.
            let bulge = 0.6 * (0.5 * r - 2.0 * e).max(0.0);
            let col = gas_color(ch.temperature_k, ch.burning);
            let mut mesh = egui::Mesh::default();
            let m = 24;
            for i in 0..=m {
                let t = i as f64 / m as f64;
                let h = housing_point(a0 + t * (a1 - a0), e, r);
                let out = bulge * 4.0 * t * (1.0 - t);
                let f = (
                    p0.0 + t * (p1.0 - p0.0) + out * mid.cos(),
                    p0.1 + t * (p1.1 - p0.1) + out * mid.sin(),
                );
                mesh.colored_vertex(at(h), col);
                mesh.colored_vertex(at(f), col);
                if i > 0 {
                    let j = 2 * i as u32;
                    mesh.add_triangle(j - 2, j - 1, j);
                    mesh.add_triangle(j - 1, j + 1, j);
                }
            }
            p.add(mesh);
        }
        // Rotor: three flanks between the apexes, on top of the chamber strips.
        let mut flank = Vec::new();
        for ch in chambers {
            let mid = (90.0 + 0.5 * ch.cycle_deg).to_radians();
            let bulge = 0.6 * (0.5 * r - 2.0 * e).max(0.0);
            let a0 = mid - FRAC_PI_3;
            let a1 = mid + FRAC_PI_3;
            let m = 12;
            for i in 0..m {
                let t = i as f64 / m as f64;
                let out = bulge * 4.0 * t * (1.0 - t);
                flank.push((
                    c.0 + r * ((1.0 - t) * a0.cos() + t * a1.cos()) + out * mid.cos(),
                    c.1 + r * ((1.0 - t) * a0.sin() + t * a1.sin()) + out * mid.sin(),
                ));
            }
        }
        // Flanks in rotor order so the outline closes: sort by angle about the centre.
        flank.sort_by(|a, b| {
            let ang = |q: &(f64, f64)| (q.1 - c.1).atan2(q.0 - c.0);
            ang(a).total_cmp(&ang(b))
        });
        let body: Vec<Pos2> = flank.iter().map(|&q| at(q)).collect();
        p.add(egui::Shape::convex_polygon(
            body,
            STEEL,
            Stroke::new(1.5, DARK),
        ));
        // Eccentric lobe, rotor gear, shaft.
        p.circle_filled(at(c), (0.45 * r) as f32 * scale, DARK);
        p.circle_stroke(at(c), (0.3 * r) as f32 * scale, Stroke::new(2.0, STEEL));
        p.circle_filled(
            at((0.0, 0.0)),
            (0.17 * r) as f32 * scale,
            Color32::from_rgb(190, 190, 200),
        );
        p.circle_filled(at(c), 3.0, Color32::WHITE);
        // Spark plugs at the waist; ports below, lit while any chamber has them open.
        let burning = chambers.iter().any(|ch| ch.burning);
        for dx in [-0.12, 0.12] {
            let tip = at((dx * r, r - e));
            p.circle_filled(
                tip,
                4.0,
                if burning {
                    Color32::from_rgb(255, 230, 120)
                } else {
                    DARK
                },
            );
            p.line_segment([tip, tip + vec2(0.0, -14.0)], Stroke::new(3.0, STEEL));
        }
        let open = |f: fn(&engine_core::engine::CylinderView) -> f64| {
            chambers.iter().map(f).fold(0.0, f64::max) > 0.0
        };
        for (deg, col, lit) in [
            (225.0f64, EXHAUST, open(|ch| ch.exhaust_lift_m)),
            (315.0, INTAKE, open(|ch| ch.intake_lift_m)),
        ] {
            let q = housing_point(deg.to_radians(), e, r);
            let dir = vec2(
                deg.to_radians().cos() as f32,
                -(deg.to_radians().sin() as f32),
            );
            let base = at(q);
            let colour = if lit { col } else { col.gamma_multiply(0.3) };
            p.line_segment([base, base + 22.0 * dir], Stroke::new(8.0, colour));
        }
        // One line per chamber under the housing.
        let lines: Vec<String> = chambers
            .iter()
            .enumerate()
            .map(|(j, ch)| {
                format!(
                    "#{} {} {:.1} bar",
                    3 * k + j + 1,
                    stroke_name(ch.cycle_deg),
                    pa_to_bar(ch.pressure_pa)
                )
            })
            .collect();
        p.text(
            at((0.0, -(r - e))) + vec2(0.0, 30.0),
            Align2::CENTER_TOP,
            format!("Rotor {}\n{}", k + 1, lines.join("\n")),
            font.clone(),
            Color32::LIGHT_GRAY,
        );
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
    let line = audio_line(audio, s.dropouts);
    if s.dropouts > 0 {
        ui.colored_label(ui.visuals().warn_fg_color, line);
    } else {
        ui.label(line);
    }
}

/// The audio status: the device's rate, and how many samples the device found missing (the
/// simulation fell behind: heard as crackle), or why there is no sound.
pub fn audio_line(audio: &Result<u32, String>, dropouts: u64) -> String {
    match audio {
        Ok(sr) if dropouts > 0 => {
            format!("Audio: {sr} Hz · {dropouts} samples dropped (the simulation fell behind)")
        }
        Ok(sr) => format!("Audio: {sr} Hz"),
        Err(e) => format!("No audio: {e}"),
    }
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

#[cfg(test)]
mod tests {
    use super::*;

    /// Every apex of the rotor rides on the housing at any chamber angle: an apex at rotor
    /// angle a, centre at e·(cos 3θ, sin 3θ), lies at e·(cos 3a, sin 3a) + R·(cos a, sin a)
    /// because 3·(θ + 120°·k) ≡ 3θ (Yamamoto §2) — checked here on the drawn pose.
    #[test]
    fn the_drawn_rotor_apexes_ride_on_the_housing() {
        let (e, r) = (0.015, 0.105);
        for psi in [0.0, 37.0, 180.0, 359.0, 611.0] {
            let (_, apexes) = rotor_pose(psi, e, r);
            for (k, a) in apexes.iter().enumerate() {
                let alpha = (30.0 + 0.5 * psi + 120.0 * k as f64).to_radians();
                let h = housing_point(alpha, e, r);
                assert!((a.0 - h.0).hypot(a.1 - h.1) < 1e-12, "psi {psi} apex {k}");
            }
        }
    }

    /// The chamber is smallest at ψ = 0 (its face pressed to the waist, centre at (0, e)) and
    /// largest at ψ = 180° (centre at (e, 0), away from the face at 180°).
    #[test]
    fn the_drawn_chamber_is_smallest_at_its_top_centre() {
        let (e, r) = (0.015, 0.105);
        let (c, _) = rotor_pose(0.0, e, r);
        assert!(c.0.abs() < 1e-12 && (c.1 - e).abs() < 1e-12);
        let (c, _) = rotor_pose(180.0, e, r);
        assert!((c.0 - e).abs() < 1e-12 && c.1.abs() < 1e-12);
        // The housing's minor half-axis is R − e = 90 mm (Heywood Fig. 1.23 RENESIS).
        let w = housing_point(std::f64::consts::FRAC_PI_2, e, r);
        assert!((w.1 - 0.090).abs() < 1e-12);
    }

    #[test]
    fn the_audio_line_tells_when_the_simulation_fell_behind() {
        assert_eq!(audio_line(&Ok(48_000), 0), "Audio: 48000 Hz");
        let l = audio_line(&Ok(48_000), 1234);
        assert!(l.contains("1234 samples dropped"), "{l}");
        assert_eq!(audio_line(&Err("none".into()), 5), "No audio: none");
    }
}
