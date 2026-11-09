//! The turbo panel of the 2D view: both wheels seen along the shaft in their volutes, turning
//! with the shaft (slowed to be seen), the gas coloured by its temperature, the wastegate's flap
//! and the readouts (spec `2026-11-08-turbo-views-design.md`).

use crate::sim::Snapshot;
use eframe::egui::{self, Align2, Color32, FontId, Pos2, Stroke, pos2, vec2};
use engine_core::units::{ATM_PA, pa_to_bar};
use std::f64::consts::TAU;

/// The wheels are drawn at this fraction of the shaft's speed: a real shaft's 100–200 krpm
/// (up to 3300 turns a second) would strobe at a screen's 60 frames into a still or backward
/// wheel, while 100–200 rpm reads plainly as faster or slower. The true speed is in the readout.
pub const SHOWN_FRACTION: f64 = 1e-3;
/// The longest frame step the wheels follow, s: a UI stall does not fling them round.
const MAX_STEP_S: f64 = 0.1;
/// The flap's swing fully open, degrees (ESTIMATE: a swing-valve wastegate's flapper opens
/// some 30–45° off its seat; drawn at 40).
pub const FLAP_FULL_DEG: f64 = 40.0;
/// The turbine wheel's blades (ESTIMATE: automotive radial turbines carry 9–12; the definition
/// gives the compressor's only).
const TURBINE_BLADES: usize = 11;

/// The wheel's angle after `dt` seconds at `shaft_rpm`, turning at [`SHOWN_FRACTION`] of it,
/// kept in [0, 2π).
pub fn advance_wheel(angle: f64, shaft_rpm: f64, dt: f64) -> f64 {
    let turns = shaft_rpm.max(0.0) * SHOWN_FRACTION / 60.0 * dt.clamp(0.0, MAX_STEP_S);
    (angle + TAU * turns).rem_euclid(TAU)
}

/// The wastegate flap's angle off its seat, rad, for the gate's opening 0–1.
pub fn flap_angle_rad(open: f64) -> f64 {
    open.clamp(0.0, 1.0) * FLAP_FULL_DEG.to_radians()
}

/// `n` blades evenly round the wheel, the first at `angle`, rad.
pub fn blade_angles(n: usize, angle: f64) -> Vec<f64> {
    (0..n).map(|k| angle + k as f64 * TAU / n as f64).collect()
}

/// A volute's outline round `c`: its radius growing a third over the turn from `r0`, the
/// tongue at `start`, rad (screen y down).
fn volute(c: Pos2, r0: f32, start: f32) -> Vec<Pos2> {
    (0..=48)
        .map(|k| {
            let u = k as f32 / 48.0;
            let a = start + u * std::f32::consts::TAU;
            let r = r0 * (1.0 + 0.35 * u);
            c + vec2(a.cos(), a.sin()) * r
        })
        .collect()
}

/// One wheel in its volute: the gas filling the scroll, the wheel's disc, its blades and hub.
#[allow(clippy::too_many_arguments)]
fn wheel(
    p: &egui::Painter,
    c: Pos2,
    r: f32,
    gas: Color32,
    metal: Color32,
    blades: usize,
    splitters: bool,
    angle: f64,
) {
    let outline = volute(c, 1.25 * r, -0.5);
    p.circle_filled(c, 1.3 * r, gas.gamma_multiply(0.9));
    p.add(egui::Shape::line(outline, Stroke::new(2.0, metal)));
    p.circle_filled(c, r, Color32::from_rgb(28, 30, 36));
    let blade = |a: f64, from: f32, w: f32| {
        // backswept: the tip trails the root by a fifth of a radian
        let (a0, a1) = (a as f32, a as f32 - 0.2);
        p.line_segment(
            [
                c + vec2(a0.cos(), a0.sin()) * from,
                c + vec2(a1.cos(), a1.sin()) * 0.95 * r,
            ],
            Stroke::new(w, metal),
        );
    };
    for a in blade_angles(blades, angle) {
        blade(a, 0.25 * r, 2.0);
    }
    if splitters {
        for a in blade_angles(blades, angle + std::f64::consts::PI / blades.max(1) as f64) {
            blade(a, 0.55 * r, 1.2);
        }
    }
    p.circle_filled(c, 0.22 * r, metal);
}

/// The turbo panel in `rect`; `angle` is the wheels' angle, rad ([`advance_wheel`]).
pub fn panel(p: &egui::Painter, rect: egui::Rect, s: &Snapshot, angle: f64) {
    let (Some(def), Some(t)) = (&s.def.turbo, &s.turbo) else {
        return;
    };
    p.rect_filled(rect, 4.0, Color32::from_rgb(32, 34, 40));
    let font = FontId::proportional((rect.width() / 14.0).clamp(10.0, 14.0));
    let text = Color32::from_gray(220);
    p.text(
        rect.center_top() + vec2(0.0, 6.0),
        Align2::CENTER_TOP,
        "Turbo",
        font.clone(),
        text,
    );
    let r = (0.17 * rect.height()).min(0.28 * rect.width());
    let cx = rect.center().x;
    let comp_c = pos2(cx, rect.top() + 0.30 * rect.height());
    let turb_c = pos2(cx, rect.top() + 0.66 * rect.height());
    // the shaft joining them, behind both
    p.line_segment(
        [comp_c, turb_c],
        Stroke::new(0.18 * r, Color32::from_gray(120)),
    );
    let steel = Color32::from_rgb(200, 202, 208);
    let cast = Color32::from_rgb(150, 110, 90);
    wheel(
        p,
        comp_c,
        r,
        crate::view::gas_color(t.boost_t_k, false),
        steel,
        def.blades as usize,
        true,
        angle,
    );
    let exhaust_k = t.turbine_inlet_t_k;
    wheel(
        p,
        turb_c,
        0.9 * r,
        crate::view::gas_color(exhaust_k, false),
        cast,
        TURBINE_BLADES,
        false,
        angle,
    );
    let small = FontId::proportional(font.size * 0.8);
    p.text(
        comp_c + vec2(-1.35 * r, -1.25 * r),
        Align2::LEFT_BOTTOM,
        "compressor",
        small.clone(),
        text,
    );
    p.text(
        turb_c + vec2(-1.35 * r, -1.2 * r),
        Align2::LEFT_BOTTOM,
        "turbine",
        small.clone(),
        text,
    );
    // the wastegate: a flap hinged at the turbine's outlet, swung open by the gate
    if def.wastegate_area_m2 > 0.0 {
        let hinge = turb_c + vec2(1.35 * r, 0.9 * r);
        let a = flap_angle_rad(t.wastegate_open) as f32;
        let len = 0.6 * r;
        p.line_segment(
            [hinge, hinge + vec2(0.0, -len)],
            Stroke::new(1.0, Color32::from_gray(90)),
        );
        p.line_segment(
            [hinge, hinge + vec2(-a.sin(), -a.cos()) * len],
            Stroke::new(3.0, Color32::from_rgb(230, 180, 60)),
        );
        p.circle_filled(hinge, 2.5, Color32::from_gray(160));
    }
    let gate = if def.vnt_open_area_m2 > 0.0 {
        format!("vanes {:.0} % open", 100.0 * t.vanes_open)
    } else if def.wastegate_area_m2 > 0.0 {
        format!("wastegate {:.0} %", 100.0 * t.wastegate_open)
    } else {
        String::new()
    };
    let lines = [
        format!("{:.0} krpm", t.shaft_rpm / 1000.0),
        format!("boost {:+.2} bar", pa_to_bar(t.boost_pa - ATM_PA)),
        gate,
    ];
    let mut y = rect.bottom() - 6.0;
    for l in lines.iter().rev().filter(|l| !l.is_empty()) {
        p.text(pos2(cx, y), Align2::CENTER_BOTTOM, l, font.clone(), text);
        y -= font.size + 3.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    #[test]
    fn the_wheels_turn_at_a_thousandth_of_the_shaft() {
        // 150 krpm shows as 150 rpm: 2.5 turns a second, so a second of frames from 0 lands
        // half a turn on
        let a = (0..10).fold(0.0, |a, _| advance_wheel(a, 150_000.0, 0.1));
        assert!((a - PI).abs() < 1e-9, "{a}");
        // a shaft at rest leaves the wheel where it is; the angle stays in [0, 2π)
        assert_eq!(advance_wheel(1.0, 0.0, 0.5), 1.0);
        // 60 krpm shows as one turn a second: a tenth of a second past 6 rad wraps round
        let a = advance_wheel(6.0, 60_000.0, 0.1);
        assert!((0.0..2.0 * PI).contains(&a), "{a}");
        assert!((a - (6.0 + 0.2 * PI - 2.0 * PI)).abs() < 1e-9);
        // a long stall of the UI (a dragged window) does not fling it: the step is capped
        assert_eq!(
            advance_wheel(0.0, 150_000.0, 10.0),
            advance_wheel(0.0, 150_000.0, 0.1)
        );
    }

    #[test]
    fn the_flap_swings_with_the_gate_up_to_its_stop() {
        assert_eq!(flap_angle_rad(0.0), 0.0);
        assert!((flap_angle_rad(1.0) - FLAP_FULL_DEG.to_radians()).abs() < 1e-12);
        assert!((flap_angle_rad(0.5) - 0.5 * FLAP_FULL_DEG.to_radians()).abs() < 1e-12);
        // out-of-range input (a filter's overshoot) stays on the stops
        assert_eq!(flap_angle_rad(1.5), flap_angle_rad(1.0));
        assert_eq!(flap_angle_rad(-0.2), 0.0);
    }

    #[test]
    fn the_blades_are_spread_evenly_and_turn_together() {
        let b = blade_angles(6, 0.3);
        assert_eq!(b.len(), 6);
        for (k, a) in b.iter().enumerate() {
            assert!((a - (0.3 + k as f64 * PI / 3.0)).abs() < 1e-12);
        }
        assert!(blade_angles(0, 0.0).is_empty());
    }
}
