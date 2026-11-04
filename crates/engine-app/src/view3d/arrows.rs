//! The handles drawn as arrows: each editable measure of the clicked part is a double-headed
//! arrow along the direction it moves on screen, named with its value, grabbed anywhere along
//! its length (a dot was hard to find and to hit).

use eframe::egui::{self, Pos2, Vec2};

/// Half an arrow's length on screen, px.
pub const HALF_PX: f32 = 34.0;
/// How far from an arrow's line a press still takes it, px.
pub const GRAB_PX: f32 = 9.0;

/// Whether `pos` is on the arrow centred at `at` along the unit `dir`, `half` long each way,
/// within `tol` of its line.
pub fn hit(pos: Pos2, at: Pos2, dir: Vec2, half: f32, tol: f32) -> bool {
    let d = pos - at;
    let along = d.dot(dir).clamp(-half, half);
    (d - dir * along).length() <= tol
}

/// The arrow's colours: amber, brighter and thicker when lit (under the pointer, or held).
const AMBER: egui::Color32 = egui::Color32::from_rgb(255, 196, 40);
const LIT: egui::Color32 = egui::Color32::from_rgb(255, 240, 150);

/// Paint an arrow centred at `at` along `dir`, a head at each end, and its `label` past the head
/// its value grows towards.
pub fn paint(p: &egui::Painter, at: Pos2, dir: Vec2, lit: bool, label: &str) {
    let (color, width) = if lit { (LIT, 4.0) } else { (AMBER, 2.5) };
    let outline = egui::Stroke::new(width + 2.0, egui::Color32::from_black_alpha(160));
    let stroke = egui::Stroke::new(width, color);
    let (a, b) = (at - dir * HALF_PX, at + dir * HALF_PX);
    p.line_segment([a, b], outline);
    p.line_segment([a, b], stroke);
    let side = dir.rot90();
    for (tip, back) in [(b, -dir), (a, dir)] {
        let head = vec![
            tip,
            tip + back * 11.0 + side * 6.0,
            tip + back * 11.0 - side * 6.0,
        ];
        p.add(egui::Shape::convex_polygon(
            head,
            color,
            egui::Stroke::new(1.0, egui::Color32::BLACK),
        ));
    }
    p.circle_filled(at, if lit { 5.0 } else { 3.5 }, color);
    let anchor = b + dir * 8.0;
    let align = if dir.x >= 0.0 {
        egui::Align2::LEFT_CENTER
    } else {
        egui::Align2::RIGHT_CENTER
    };
    let font = egui::FontId::proportional(if lit { 14.0 } else { 12.0 });
    let galley = p.layout_no_wrap(label.to_owned(), font.clone(), color);
    let r = align.anchor_size(anchor, galley.size()).expand(3.0);
    p.rect_filled(r, 3.0, egui::Color32::from_black_alpha(170));
    p.text(anchor, align, label, font, color);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn an_arrow_is_grabbed_along_its_length_only() {
        let (at, dir) = (egui::pos2(100.0, 100.0), egui::vec2(0.6, 0.8));
        let side = egui::vec2(-0.8, 0.6);
        let on = |along: f32, off: f32| hit(at + dir * along + side * off, at, dir, 30.0, 9.0);
        assert!(on(0.0, 0.0), "its middle");
        assert!(on(29.0, 5.0) && on(-29.0, -5.0), "near either head");
        assert!(!on(0.0, 12.0), "beside it");
        assert!(!on(45.0, 0.0) && !on(-45.0, 0.0), "past its heads");
        // 30 along and 9 off is the corner: in; 31 along is past a head (with nothing off): out
        assert!(on(30.0, 8.9));
        assert!(!on(31.0 + 9.0, 0.0));
    }
}
