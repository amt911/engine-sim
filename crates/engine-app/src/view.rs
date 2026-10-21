//! Painting: animated engine cross-section, gauges and plots. Pure drawing from a
//! `Snapshot`; checked by looking at the running app (see PR verification), not by
//! line coverage.

use crate::dyno::{DynoPoint, peaks};
use crate::sim::Snapshot;
use eframe::egui::{
    self, Align2, Color32, FontId, Pos2, Rect, Stroke, StrokeKind, Vec2, pos2, vec2,
};
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
pub fn engine(ui: &mut egui::Ui, s: &Snapshot) {
    let (rect, _) = ui.allocate_exact_size(ui.available_size(), egui::Sense::hover());
    let p = ui.painter_at(rect);
    p.rect_filled(rect, 0.0, Color32::from_rgb(24, 26, 30));
    let g = s.def.geometry;
    let n = s.cylinders.len().max(1);
    let (r, l, b) = (g.crank_radius(), g.rod_m, g.bore_m);
    let clear_h = g.clearance_volume() / g.piston_area();
    let piston_h = 0.6 * b;
    let head_h = 0.35 * b;
    let total_h = r + l + r + piston_h + clear_h + head_h + 0.6 * b;
    let cell_w = rect.width() / n as f32;
    let scale = (0.85 * rect.height() / total_h as f32).min(0.8 * cell_w / (1.9 * b as f32));
    let m = |x: f64| x as f32 * scale;

    for (i, c) in s.cylinders.iter().enumerate() {
        let cx = rect.left() + cell_w * (i as f32 + 0.5);
        let bottom = rect.center().y + m(total_h) * 0.5;
        let crank = pos2(cx, bottom - m(r + 0.4 * b));
        let th = c.cycle_deg.to_radians();
        let pin = crank + vec2(m(r * th.sin()), -m(r * th.cos()));
        let wrist_y = crank.y - m(r * th.cos() + (l * l - (r * th.sin()).powi(2)).sqrt());
        let crown_y = wrist_y - m(0.35 * b);
        let tdc_crown = crank.y - m(r + l) - m(0.35 * b);
        let deck = tdc_crown - m(clear_h);
        let bdc_skirt = crank.y - m(l - r) - m(0.35 * b) + m(piston_h);
        let half = m(b) * 0.5;

        // Chamber gas, walls, head.
        let gas = Rect::from_min_max(pos2(cx - half, deck), pos2(cx + half, crown_y));
        p.rect_filled(gas, 0.0, gas_color(c.temperature_k, c.burning));
        let wall = Stroke::new(3.0, STEEL);
        p.line_segment(
            [
                pos2(cx - half - 2.0, deck),
                pos2(cx - half - 2.0, bdc_skirt),
            ],
            wall,
        );
        p.line_segment(
            [
                pos2(cx + half + 2.0, deck),
                pos2(cx + half + 2.0, bdc_skirt),
            ],
            wall,
        );
        let head = Rect::from_min_max(
            pos2(cx - half - 6.0, deck - m(head_h)),
            pos2(cx + half + 6.0, deck),
        );
        p.rect_filled(head, 2.0, DARK);

        // Valves: stem + disc, pushed down by lift (drawn 1:1).
        for (dx, lift, col) in [
            (-0.25, c.intake_lift_m, INTAKE),
            (0.25, c.exhaust_lift_m, EXHAUST),
        ] {
            let vx = cx + m(dx * b);
            let vy = deck + m(lift);
            let vr = m(0.16 * b);
            p.line_segment(
                [pos2(vx, deck - m(head_h) - 8.0), pos2(vx, vy)],
                Stroke::new(2.0, col),
            );
            p.rect_filled(
                Rect::from_center_size(pos2(vx, vy), vec2(2.0 * vr, 3.0)),
                1.0,
                col,
            );
        }

        // Piston, rod, crank.
        let piston = Rect::from_min_max(
            pos2(cx - half, crown_y),
            pos2(cx + half, crown_y + m(piston_h)),
        );
        p.rect_filled(piston, 3.0, STEEL);
        p.rect_stroke(piston, 3.0, Stroke::new(1.0, DARK), StrokeKind::Inside);
        p.circle_filled(crank, m(r * 1.35), DARK);
        p.line_segment(
            [pin, pos2(cx, wrist_y)],
            Stroke::new(m(0.12 * b).max(3.0), Color32::from_rgb(190, 190, 200)),
        );
        p.circle_filled(pos2(cx, wrist_y), 3.0, DARK);
        p.circle_filled(pin, m(0.1 * b).max(3.0), Color32::WHITE);
        p.circle_filled(crank, 3.0, STEEL);

        let label = format!(
            "#{}\n{}\n{:.1} bar",
            i + 1,
            stroke_name(c.cycle_deg),
            pa_to_bar(c.pressure_pa)
        );
        let font = FontId::proportional((cell_w / 7.0).clamp(9.0, 14.0));
        p.text(
            pos2(cx, deck - m(head_h) - 14.0),
            Align2::CENTER_BOTTOM,
            label,
            font,
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
