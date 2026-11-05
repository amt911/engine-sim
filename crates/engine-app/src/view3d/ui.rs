//! The 3D view in the central panel: the 2D/3D switch, the detail choice, the camera's mouse
//! input, and the frame handed to the renderer.

use super::camera::OrbitCamera;
use super::detail::{Detail, DetailGovernor};
use super::render::{Frame3d, MeshKey, Shared, target_size};
use super::scene::{self, Instance, Scene};
use crate::sim::Snapshot;
use eframe::egui;
use eframe::egui_wgpu;
use engine_core::engine::EngineDef;
use std::sync::{Arc, Mutex};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ViewMode {
    TwoD,
    ThreeD,
}

/// The layers drawn over the engine (the user's toggles).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Layers {
    /// The exhaust pipes coloured by their temperature (wall, else gas), with a legend.
    pub temperature: bool,
    /// Marks running along the exhaust pipes at their gas's speed (×[`FLOW_SCALE`]).
    pub flow: bool,
    /// Each chamber's charge, coloured by its temperature, glowing while it burns.
    pub combustion: bool,
    /// The engine opened along (inline) or across (V, rotary) its cylinders.
    pub cutaway: bool,
}

impl Default for Layers {
    fn default() -> Self {
        Self {
            temperature: true,
            flow: true,
            combustion: true,
            cutaway: false,
        }
    }
}

/// A handle in the hand: its value as it moves, from its value when it was taken.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Drag {
    pub handle: super::handles::Handle,
    pub start: f64,
    pub value: f64,
    /// Pixels the pointer has moved along the handle's on-screen axis.
    pub along_px: f32,
    /// The handle's axis on screen (unit) and how many pixels a metre along it covers.
    pub dir: egui::Vec2,
    pub px_per_m: f32,
}

/// Where world point `p` shows in `rect` for `view_proj` (None behind the camera).
fn to_screen(view_proj: glam::Mat4, rect: egui::Rect, p: glam::Vec3) -> Option<egui::Pos2> {
    let c = view_proj * p.extend(1.0);
    if c.w <= 1e-6 {
        return None;
    }
    let (x, y) = (c.x / c.w, c.y / c.w);
    Some(egui::pos2(
        rect.left() + 0.5 * (x + 1.0) * rect.width(),
        rect.top() + 0.5 * (1.0 - y) * rect.height(),
    ))
}

/// The flow marks run at this share of the gas's speed, so they can be followed by eye.
pub const FLOW_SCALE: f32 = 0.05;

/// Flow marks per pipe at each detail.
fn marks(d: Detail) -> usize {
    match d {
        Detail::Low => 0,
        Detail::Medium => 4,
        Detail::High => 8,
    }
}

/// How long the edited parts stay tinted after the last change, s.
pub const TINT_SECS: f64 = 0.5;

/// The measure last edited and when its tint ends (egui's clock, s).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Tint {
    pub field: super::handles::Field,
    pub until: f64,
}

/// A part's colour pulled to the highlight (the arrows' amber).
fn tinted(c: [f32; 4]) -> [f32; 4] {
    const AMBER: [f32; 3] = [1.0, 0.77, 0.16];
    [
        0.35 * c[0] + 0.65 * AMBER[0],
        0.35 * c[1] + 0.65 * AMBER[1],
        0.35 * c[2] + 0.65 * AMBER[2],
        c[3],
    ]
}

#[derive(Debug, Clone)]
pub struct View3dState {
    /// The measure being edited, tinted on the parts it changes until [`TINT_SECS`] after its
    /// last change.
    pub tint: Option<Tint>,
    /// The measure whose parts were drawn tinted last frame.
    pub tinted: Option<super::handles::Field>,
    /// Where the clicked part's handles stood on screen last frame (what each edits).
    pub handle_spots: Vec<(super::handles::Field, egui::Pos2)>,
    /// A handle being dragged.
    pub drag: Option<Drag>,
    /// The clicked part (its index among the frame's instances) and where its card stands.
    pub selected: Option<(usize, egui::Pos2)>,
    /// How far round its pipe each pipe's flow marks have run (0..1).
    phases: Vec<f32>,
    pub layers: Layers,
    pub mode: ViewMode,
    /// Whether the renderer is installed (a wgpu device exists); without it the view is 2D.
    pub available: bool,
    pub camera: Option<OrbitCamera>,
    pub governor: DetailGovernor,
    /// Bumped whenever the drawn engine changes, so the meshes and the still parts are rebuilt.
    pub def_version: u64,
    drawn: Option<EngineDef>,
    scene: Option<Scene>,
    moving: Vec<Instance>,
    pub(crate) shared: Arc<Mutex<Shared>>,
}

impl Default for View3dState {
    fn default() -> Self {
        Self {
            tint: None,
            tinted: None,
            handle_spots: Vec::new(),
            drag: None,
            selected: None,
            phases: Vec::new(),
            layers: Layers::default(),
            mode: ViewMode::ThreeD,
            available: false,
            camera: None,
            governor: DetailGovernor::default(),
            def_version: 0,
            drawn: None,
            scene: None,
            moving: Vec::new(),
            shared: Arc::default(),
        }
    }
}

impl View3dState {
    /// The index of the first drawn `part` in the frame (tests pick parts by kind).
    #[cfg(test)]
    pub fn first(&self, part: scene::Part) -> Option<usize> {
        self.shared
            .lock()
            .ok()?
            .instances
            .iter()
            .position(|i| i.part == part)
    }

    /// `field` was changed at `now`: its parts are tinted for [`TINT_SECS`] from now.
    pub fn touch(&mut self, field: super::handles::Field, now: f64) {
        self.tint = Some(Tint {
            field,
            until: now + TINT_SECS,
        });
    }

    /// The measure tinted at `now`, if its time has not run out.
    pub fn tint_field(&self, now: f64) -> Option<super::handles::Field> {
        self.tint.filter(|t| now < t.until).map(|t| t.field)
    }

    /// Tint, in the frame just filled, the parts the measure being edited changes.
    fn tint_frame(&mut self, def: &EngineDef, now: f64) {
        self.tinted = self.tint_field(now);
        let Some(field) = self.tinted else { return };
        if let Ok(mut sh) = self.shared.lock() {
            let mut parts = std::mem::take(&mut sh.instances);
            let hit: Vec<usize> = (0..parts.len())
                .filter(|&k| field.changes(def, &parts, k))
                .collect();
            for k in hit {
                parts[k].color = tinted(parts[k].color);
            }
            sh.instances = parts;
        }
    }

    /// The view to draw: 3D only when asked for and a renderer exists.
    pub fn shows_3d(&self) -> bool {
        self.available && self.mode == ViewMode::ThreeD
    }

    /// Rebuild the still parts when the engine changed (compared by value: an edit, a load).
    fn sync(&mut self, def: &EngineDef) {
        if self.drawn.as_ref() != Some(def) {
            self.def_version += 1;
            let s = scene::build(def);
            // an edit keeps the user's view; another engine (layout, cylinders, or a size a
            // quarter off) is framed afresh
            let same_size = self.scene.as_ref().is_some_and(|old| {
                let k = s.focus.1 / old.focus.1.max(1e-6);
                (0.8..=1.25).contains(&k)
            });
            let same_engine = same_size
                && self.drawn.as_ref().is_some_and(|d| {
                    d.cylinders == def.cylinders
                        && d.layout == def.layout
                        && d.geometry.rotor.is_some() == def.geometry.rotor.is_some()
                });
            if !same_engine || self.camera.is_none() {
                self.camera = Some(OrbitCamera::framing(s.focus.0, s.focus.1, start_yaw(def)));
            }
            self.scene = Some(s);
            self.drawn = Some(def.clone());
        }
    }

    /// Fill the shared frame (still parts then moving ones) for the cylinders of `snap`.
    fn fill(&mut self, snap: &Snapshot, aspect: f32, dt: f32) {
        let Some(s) = &self.scene else { return };
        scene::moving(&snap.def, &snap.cylinders, &mut self.moving);
        // Low draws no valve train: its parts would be invisible but still clickable
        if !self.governor.detail.realistic() {
            self.moving.retain(|i| !i.part.is_valve_train());
        }
        self.phases.resize(s.pipes.len(), 0.0);
        let n_marks = if self.layers.flow && !snap.pipes.is_empty() {
            marks(self.governor.detail)
        } else {
            0
        };
        if let Ok(mut sh) = self.shared.lock() {
            sh.instances.clear();
            sh.instances.extend_from_slice(&s.static_parts);
            // the pipes in their temperature (the wall's when it is modelled, else the gas's)
            if self.layers.temperature {
                for p in &s.pipes {
                    if let Some(g) = snap.pipes.get(p.pipe) {
                        let c = super::colormap::temperature(g.wall_k.unwrap_or(g.temperature_k));
                        for i in &mut sh.instances[p.instances.clone()] {
                            i.color = c;
                        }
                    }
                }
            }
            let combustion = self.layers.combustion;
            sh.instances.extend(
                self.moving
                    .iter()
                    .filter(|i| combustion || i.part != scene::Part::Charge),
            );
            // the flow marks: each pipe's run on at its gas's speed, coloured by the gas
            for (p, phase) in s.pipes.iter().zip(&mut self.phases) {
                let Some(g) = snap.pipes.get(p.pipe) else {
                    continue;
                };
                let len: f32 = p.runs.iter().map(super::routing::Run::length).sum();
                if len > 0.0 {
                    *phase =
                        (*phase + g.velocity_m_s as f32 * FLOW_SCALE * dt / len).rem_euclid(1.0);
                }
                let r = p.runs.first().map_or(0.0, |_| {
                    s.static_parts[p.instances.start]
                        .model
                        .x_axis
                        .truncate()
                        .length()
                });
                let mut c = super::colormap::temperature(g.temperature_k);
                c[3] = 0.3;
                for k in 0..n_marks {
                    if let Some(at) =
                        super::routing::point_along(&p.runs, *phase + k as f32 / n_marks as f32)
                    {
                        sh.instances.push(scene::Instance {
                            part: scene::Part::Joint,
                            model: glam::Mat4::from_scale_rotation_translation(
                                glam::Vec3::splat(1.15 * r),
                                glam::Quat::IDENTITY,
                                at,
                            ),
                            color: c,
                        });
                    }
                }
            }
            // the clicked part a little brighter
            if let Some(i) = self.selected.and_then(|(k, _)| sh.instances.get_mut(k)) {
                for c in &mut i.color[..3] {
                    *c = (*c * 1.35 + 0.08).min(1.0);
                }
            }
            sh.clip = self.layers.cutaway.then(|| scene::cut_plane(&snap.def));
            if let Some(c) = &self.camera {
                sh.view_proj = c.view_proj(aspect);
            }
        }
    }
}

/// Where the camera starts: a V or flat engine from its front three-quarter, so the banks and
/// the crank between them show; an inline engine or a rotary from its side.
fn start_yaw(def: &EngineDef) -> f32 {
    if def.geometry.rotor.is_none() && def.layout.bank_angle_deg > 0.0 {
        1.15
    } else {
        0.6
    }
}

/// The temperature layer's legend in the view's lower left corner: the colour map from 300 to
/// 1300 K, or a note when the exhaust has no gas data.
fn legend(ui: &egui::Ui, rect: egui::Rect, no_data: bool) {
    use super::colormap::{COLD_K, HOT_K, temperature};
    let p = ui.painter_at(rect);
    let origin = rect.left_bottom() + egui::vec2(12.0, -28.0);
    let font = egui::FontId::proportional(11.0);
    let text = egui::Color32::from_gray(200);
    if no_data {
        p.text(
            origin,
            egui::Align2::LEFT_CENTER,
            "Temperature: no exhaust gas data",
            font,
            text,
        );
        return;
    }
    let (w, h, steps) = (160.0, 10.0, 32);
    for k in 0..steps {
        let t = COLD_K + (HOT_K - COLD_K) * (k as f64 + 0.5) / steps as f64;
        let [r, g, b, _] = temperature(t).map(|c| (c * 255.0) as u8);
        let x0 = origin.x + w * k as f32 / steps as f32;
        p.rect_filled(
            egui::Rect::from_min_max(
                egui::pos2(x0, origin.y - h / 2.0),
                egui::pos2(x0 + w / steps as f32 + 0.5, origin.y + h / 2.0),
            ),
            0.0,
            egui::Color32::from_rgb(r, g, b),
        );
    }
    p.text(
        origin + egui::vec2(0.0, 12.0),
        egui::Align2::LEFT_TOP,
        "300 K",
        font.clone(),
        text,
    );
    p.text(
        origin + egui::vec2(w, 12.0),
        egui::Align2::RIGHT_TOP,
        "1300 K",
        font.clone(),
        text,
    );
    p.text(
        origin + egui::vec2(w + 8.0, 0.0),
        egui::Align2::LEFT_CENTER,
        format!("gas flow marks at ×{FLOW_SCALE} speed"),
        font,
        text,
    );
}

/// The bar over the engine view: 2D / 3D and the detail.
pub fn bar(ui: &mut egui::Ui, st: &mut View3dState) {
    ui.horizontal(|ui| {
        if !st.available {
            ui.add_enabled(false, egui::Button::new("3D"))
                .on_disabled_hover_text("No GPU device for the 3D view: showing 2D");
            return;
        }
        ui.selectable_value(&mut st.mode, ViewMode::TwoD, "2D");
        ui.selectable_value(&mut st.mode, ViewMode::ThreeD, "3D");
        if st.mode == ViewMode::ThreeD {
            let label = |d: Option<Detail>| match d {
                None => "Auto",
                Some(Detail::Low) => "Low",
                Some(Detail::Medium) => "Medium",
                Some(Detail::High) => "High",
            };
            let mut choice = st.governor.manual();
            egui::ComboBox::new("detail3d", "Detail")
                .selected_text(label(choice))
                .show_ui(ui, |ui| {
                    for d in [
                        None,
                        Some(Detail::Low),
                        Some(Detail::Medium),
                        Some(Detail::High),
                    ] {
                        ui.selectable_value(&mut choice, d, label(d));
                    }
                });
            if choice != st.governor.manual() {
                st.governor.set_manual(choice);
            }
            ui.menu_button("Layers", |ui| {
                ui.checkbox(&mut st.layers.temperature, "Temperature");
                ui.checkbox(&mut st.layers.flow, "Gas flow");
                ui.checkbox(&mut st.layers.combustion, "Combustion");
                ui.checkbox(&mut st.layers.cutaway, "Cut-away");
            });
            ui.small("drag: turn · wheel: zoom · double-click: reframe");
        }
    });
}

/// The 3D engine in the rest of the panel.
pub fn show(
    ui: &mut egui::Ui,
    st: &mut View3dState,
    snap: &Snapshot,
    edit: &mut EngineDef,
    dirty: &mut bool,
) {
    st.sync(&snap.def);
    let now = ui.input(|i| i.time);
    let (rect, resp) = ui.allocate_exact_size(ui.available_size(), egui::Sense::click_and_drag());
    // the handles of the clicked part, on screen: where each stands, the screen direction its
    // value grows along and how many pixels a metre of it takes there
    let handles: Vec<(super::handles::Handle, egui::Pos2, egui::Vec2, f32)> =
        match (st.selected, &st.scene) {
            (Some((k, _)), Some(scene)) => st
                .shared
                .lock()
                .ok()
                .map(|sh| {
                    super::handles::handles(&snap.def, scene, &sh.instances, k)
                        .into_iter()
                        .filter_map(|h| {
                            let at = to_screen(sh.view_proj, rect, h.at)?;
                            let tip = to_screen(sh.view_proj, rect, h.at + 0.01 * h.axis)?;
                            let v = (tip - at) / 0.01;
                            (v.length() > 1e-3).then(|| (h, at, v.normalized(), v.length()))
                        })
                        .collect()
                })
                .unwrap_or_default(),
            _ => Vec::new(),
        };
    st.handle_spots.clear();
    st.handle_spots
        .extend(handles.iter().map(|(h, p, _, _)| (h.field, *p)));
    // the arrow under the pointer (the nearest, if they cross)
    let grab = |pos: egui::Pos2| {
        handles
            .iter()
            .filter(|(_, at, dir, _)| {
                super::arrows::hit(
                    pos,
                    *at,
                    *dir,
                    super::arrows::HALF_PX,
                    super::arrows::GRAB_PX,
                )
            })
            .min_by(|a, b| a.1.distance(pos).total_cmp(&b.1.distance(pos)))
            .copied()
    };
    let hovered = resp.hover_pos().and_then(grab).map(|(h, ..)| h.field);
    // where the press was, not where the pointer is once the move counts as a drag
    let pressed_at = ui.input(|i| i.pointer.press_origin());
    if resp.drag_started()
        && let Some((h, _, dir, px_per_m)) = pressed_at.and_then(grab)
    {
        let start = h.field.get(&snap.def);
        st.drag = Some(Drag {
            handle: h,
            start,
            value: start,
            along_px: 0.0,
            dir,
            px_per_m,
        });
    }
    if let Some(d) = st.drag.as_mut() {
        d.along_px += resp.drag_delta().dot(d.dir);
        let value = super::handles::dragged(&d.handle, d.start, d.along_px, d.px_per_m);
        // the engine follows as the arrow moves (the sim paces the rebuilds); the part stays
        // selected, its card showing the new numbers
        if value != d.value || resp.drag_stopped() {
            d.value = value;
            *dirty |= super::handles::apply(edit, d.handle.field, d.value);
        }
        // held: its parts stay tinted, and half a second after the last change
        st.tint = Some(Tint {
            field: d.handle.field,
            until: now + TINT_SECS,
        });
        if resp.drag_stopped() {
            st.drag = None;
        }
    } else if let Some(c) = st.camera.as_mut() {
        let d = resp.drag_delta();
        c.drag(-d.x, d.y);
        if resp.hovered() {
            c.zoom(ui.input(|i| i.smooth_scroll_delta.y));
        }
    }
    if resp.double_clicked()
        && let Some(s) = &st.scene
    {
        st.camera = Some(OrbitCamera::framing(
            s.focus.0,
            s.focus.1,
            st.drawn.as_ref().map_or(0.6, start_yaw),
        ));
    }
    st.governor.observe(1000.0 * ui.input(|i| i.unstable_dt));
    let aspect = rect.width() / rect.height().max(1.0);
    let dt = ui.input(|i| i.unstable_dt).min(0.1);
    st.fill(snap, aspect, dt);
    let e_over_r = snap
        .def
        .geometry
        .rotor
        .map_or(0.0, |r| (r.eccentricity_m / r.generating_radius_m) as f32);
    let frame = Frame3d {
        shared: st.shared.clone(),
        size_px: target_size([rect.width(), rect.height()], ui.ctx().pixels_per_point()),
        key: MeshKey {
            def_version: st.def_version,
            segments: st.governor.detail.segments(),
            e_over_r,
            realistic: st.governor.detail.realistic(),
            rod_over_bore: rod_over_bore(&snap.def),
            throw_over_bore: throw_over_bore(&snap.def),
            intake_lobe_over_bore: lobe_over_bore(&snap.def, true),
            exhaust_lobe_over_bore: lobe_over_bore(&snap.def, false),
        },
    };
    ui.painter()
        .add(egui_wgpu::Callback::new_paint_callback(rect, frame));
    // a click picks the part under the pointer (another click on nothing, or Escape, lets go)
    // a click on an arrow is not a click on the scene: the part stays selected
    if resp.clicked()
        && let Some(pos) = resp.interact_pointer_pos()
        && grab(pos).is_none()
    {
        let ndc = (
            2.0 * (pos.x - rect.left()) / rect.width().max(1.0) - 1.0,
            1.0 - 2.0 * (pos.y - rect.top()) / rect.height().max(1.0),
        );
        st.selected = st.shared.lock().ok().and_then(|sh| {
            let (o, d) = super::pick::ray(sh.view_proj, ndc)?;
            super::pick::pick(&sh.instances, o, d).map(|(k, _)| (k, pos))
        });
    }
    if ui.input(|i| i.key_pressed(egui::Key::Escape)) {
        st.selected = None;
    }
    let mut tint_touched = None;
    if let (Some((k, pos)), Some(scene)) = (st.selected, &st.scene) {
        let card = st.shared.lock().ok().and_then(|sh| {
            let c = super::card::card(&snap.def, scene, &sh.instances, k)?;
            Some((c, super::handles::card_fields(&snap.def, &sh.instances, k)))
        });
        if let Some((c, fields)) = card {
            // right of the pointer and of every arrow with its label: the card sits above the
            // scene, so an arrow under it could not be grabbed
            let right = handles
                .iter()
                .map(|(h, at, dir, _)| {
                    // the label's width at the lit size (14 px: about 8 px a character)
                    let label = h.field.label().len() + h.field.show(h.field.get(&snap.def)).len();
                    at.x + dir.x.abs() * super::arrows::HALF_PX + 24.0 + 8.0 * label as f32
                })
                .fold(pos.x, f32::max);
            egui::Area::new(egui::Id::unique("part_card"))
                .fixed_pos(egui::pos2(right + 14.0, pos.y + 8.0))
                .show(ui.ctx(), |ui| {
                    egui::Frame::popup(ui.style()).show(ui, |ui| {
                        ui.strong(&c.title);
                        egui::Grid::new("part_card_rows").show(ui, |ui| {
                            for (label, value) in &c.rows {
                                ui.label(label);
                                ui.label(value);
                                ui.end_row();
                            }
                        });
                        // what the part is made of, typed or dragged here
                        if !fields.is_empty() {
                            ui.separator();
                            egui::Grid::new("part_card_edits").show(ui, |ui| {
                                for &f in &fields {
                                    let (k, unit, dp) = f.units();
                                    let (lo, hi) = f.range();
                                    let mut shown = k * f.get(edit);
                                    let name = ui.label(f.label());
                                    let r = ui
                                        .add(
                                            egui::DragValue::new(&mut shown)
                                                .range(k * lo..=k * hi)
                                                .max_decimals(dp)
                                                .suffix(unit),
                                        )
                                        .labelled_by(name.id);
                                    if r.changed() {
                                        *dirty |= super::handles::apply(edit, f, shown / k);
                                        tint_touched = Some(f);
                                    }
                                    ui.end_row();
                                }
                            });
                        }
                    });
                });
        }
    }
    if let Some(f) = tint_touched {
        st.touch(f, now);
    }
    // the edited parts tinted while they change; the frame is plain again once the time is up
    st.tint_frame(&snap.def, now);
    if let Some(t) = st.tint.filter(|t| now < t.until) {
        ui.ctx()
            .request_repaint_after(std::time::Duration::from_secs_f64(
                (t.until - now).min(TINT_SECS),
            ));
    }
    // the handles over the scene as arrows, named with their values; the one under the pointer
    // or in the hand lit
    {
        let p = ui.painter_at(rect);
        let held = st.drag.map(|d| d.handle.field);
        for (h, at, dir, _) in &handles {
            let lit = held == Some(h.field) || (held.is_none() && hovered == Some(h.field));
            let value = match st.drag {
                Some(d) if d.handle.field == h.field => d.value,
                _ => h.field.get(&snap.def),
            };
            super::arrows::paint(
                &p,
                *at,
                *dir,
                lit,
                &format!("{} {}", h.field.label(), h.field.show(value)),
            );
        }
        if let Some(d) = st.drag {
            p.text(
                rect.left_top() + egui::vec2(12.0, 12.0),
                egui::Align2::LEFT_TOP,
                format!(
                    "{}: {}",
                    d.handle.field.label(),
                    d.handle.field.show(d.value)
                ),
                egui::FontId::proportional(14.0),
                egui::Color32::from_rgb(255, 214, 64),
            );
        }
    }
    // over the scene: painted after its callback
    if st.layers.temperature {
        legend(ui, rect, snap.pipes.is_empty());
    }
}

/// Half the stroke over the bore: what the realistic throw's mesh depends on.
fn throw_over_bore(def: &EngineDef) -> f32 {
    (0.5 * def.geometry.stroke_m / def.geometry.bore_m.max(1e-6)) as f32
}

/// The tallest lift of the intake (or exhaust) lobe over the bore: what the realistic cam's
/// mesh depends on.
fn lobe_over_bore(def: &EngineDef, intake: bool) -> f32 {
    let v = if intake {
        &def.intake_valve
    } else {
        &def.exhaust_valve
    };
    (scene::lobe_lift_m(v) / def.geometry.bore_m.max(1e-6)) as f32
}

/// The rod's length over the bore: what the realistic rod's mesh depends on.
fn rod_over_bore(def: &EngineDef) -> f32 {
    (def.geometry.rod_m / def.geometry.bore_m.max(1e-6)) as f32
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn no_render_state_means_2d() {
        let st = View3dState::default();
        assert_eq!(st.mode, ViewMode::ThreeD);
        assert!(!st.shows_3d());
        let st = View3dState {
            available: true,
            ..View3dState::default()
        };
        assert!(st.shows_3d());
    }

    #[test]
    fn scene_rebuilds_when_the_definition_changes() {
        let mut st = View3dState::default();
        let mut d = EngineDef::inline4();
        st.sync(&d);
        assert_eq!(st.def_version, 1);
        st.sync(&d);
        assert_eq!(st.def_version, 1, "same engine: nothing rebuilt");
        d.cylinders = 6;
        st.sync(&d);
        assert_eq!(st.def_version, 2);
    }

    #[test]
    fn an_edit_keeps_the_camera_and_another_engine_reframes_it() {
        let mut st = View3dState::default();
        let mut d = EngineDef::inline4();
        st.sync(&d);
        if let Some(c) = st.camera.as_mut() {
            c.distance = 0.123;
        }
        // a bigger bore: the same engine, edited, seen from where the user put the camera
        d.geometry.bore_m *= 1.05;
        st.sync(&d);
        assert_eq!(st.def_version, 2, "rebuilt");
        assert_eq!(st.camera.map(|c| c.distance), Some(0.123), "camera kept");
        // another layout (a V8): framed afresh
        st.sync(&EngineDef::crossplane_v8());
        assert_ne!(st.camera.map(|c| c.distance), Some(0.123), "reframed");
        // another engine of the same layout but another size (a much bigger inline four): framed
        // afresh too
        let mut d = EngineDef::inline4();
        st.sync(&d);
        if let Some(c) = st.camera.as_mut() {
            c.distance = 0.123;
        }
        d.geometry.bore_m *= 1.6;
        d.geometry.stroke_m *= 1.6;
        d.geometry.rod_m *= 1.6;
        st.sync(&d);
        assert_ne!(
            st.camera.map(|c| c.distance),
            Some(0.123),
            "a bigger engine reframed"
        );
    }

    #[test]
    fn view_toggle_switches_to_2d_and_back() {
        use egui_kittest::{Harness, kittest::Queryable};
        let st = View3dState {
            available: true,
            ..View3dState::default()
        };
        let mut h = Harness::new_ui_state(|ui, s: &mut View3dState| bar(ui, s), st);
        h.get_by_label("2D").click();
        h.run();
        assert_eq!(h.state().mode, ViewMode::TwoD);
        assert!(
            h.query_by_label("Detail").is_none(),
            "no detail choice in 2D"
        );
        h.get_by_label("3D").click();
        h.run();
        assert_eq!(h.state().mode, ViewMode::ThreeD);
        assert!(h.query_by_label("Detail").is_some());
    }

    #[test]
    fn without_a_gpu_the_3d_button_is_off() {
        use egui_kittest::{Harness, kittest::Queryable};
        let mut h =
            Harness::new_ui_state(|ui, s: &mut View3dState| bar(ui, s), View3dState::default());
        h.run();
        assert!(h.query_by_label("2D").is_none());
        h.get_by_label("3D").click();
        h.run();
        assert!(
            !h.state().shows_3d(),
            "a click on the disabled 3D does nothing"
        );
    }

    /// Frame cost of the 3D view on this machine: the LFA V10 (the most parts) at 1920 × 1080,
    /// 600 frames per detail level. CPU = refilling the instances + `prepare`; GPU = submit to
    /// done. The budget (spec): CPU ≤ 2 ms p99, GPU ≤ 4 ms at High. Needs a GPU (or lavapipe):
    /// `cargo test --release -p engine-app bench_3d_frame -- --ignored --nocapture`.
    #[test]
    #[ignore]
    fn bench_3d_frame() {
        use super::super::render;
        use eframe::egui_wgpu::{CallbackTrait, ScreenDescriptor, wgpu};
        use egui_kittest::wgpu as kw;
        use std::time::Instant;
        let rs = kw::create_render_state(
            kw::default_wgpu_setup(),
            eframe::egui_wgpu::RendererOptions::PREDICTABLE,
        );
        render::install(&rs);
        let def = EngineDef::lfa_v10();
        let mut snap = crate::sim::Simulation::with_quality(
            def.clone(),
            def.vehicle.clone(),
            48_000.0,
            1,
            crate::quality::Quality::Low,
        )
        .snapshot();
        let screen = ScreenDescriptor {
            size_in_pixels: [1920, 1080],
            pixels_per_point: 1.0,
        };
        println!("adapter: {:?}", rs.adapter.get_info().name);
        for detail in [Detail::Low, Detail::Medium, Detail::High] {
            let mut st = View3dState {
                available: true,
                ..View3dState::default()
            };
            st.governor.set_manual(Some(detail));
            st.sync(&def);
            let (mut cpu, mut gpu) = (Vec::new(), Vec::new());
            for f in 0..600 {
                for (i, c) in snap.cylinders.iter_mut().enumerate() {
                    c.cycle_deg = (f as f64 * 7.0 + 72.0 * i as f64) % 720.0;
                    c.piston_m = def.geometry.piston_position(c.cycle_deg.to_radians());
                }
                let t0 = Instant::now();
                st.fill(&snap, 1920.0 / 1080.0, 1.0 / 60.0);
                let frame = render::Frame3d {
                    shared: st.shared.clone(),
                    size_px: [1920, 1080],
                    key: render::MeshKey {
                        def_version: st.def_version,
                        segments: detail.segments(),
                        e_over_r: 0.0,
                        realistic: detail.realistic(),
                        rod_over_bore: rod_over_bore(&def),
                        throw_over_bore: throw_over_bore(&def),
                        intake_lobe_over_bore: lobe_over_bore(&def, true),
                        exhaust_lobe_over_bore: lobe_over_bore(&def, false),
                    },
                };
                let mut enc = rs.device.create_command_encoder(&Default::default());
                {
                    let mut r = rs.renderer.write();
                    frame.prepare(
                        &rs.device,
                        &rs.queue,
                        &screen,
                        &mut enc,
                        &mut r.callback_resources,
                    );
                }
                let t1 = Instant::now();
                rs.queue.submit([enc.finish()]);
                rs.device.poll(wgpu::PollType::wait_indefinitely()).unwrap();
                let t2 = Instant::now();
                if f >= 60 {
                    cpu.push((t1 - t0).as_secs_f64() * 1e3);
                    gpu.push((t2 - t1).as_secs_f64() * 1e3);
                }
            }
            let stat = |v: &mut Vec<f64>| {
                v.sort_by(f64::total_cmp);
                (
                    v.iter().sum::<f64>() / v.len() as f64,
                    v[v.len() * 99 / 100],
                )
            };
            let (c, g) = (stat(&mut cpu), stat(&mut gpu));
            println!(
                "{detail:?}: cpu mean {:.3} ms p99 {:.3} ms · gpu mean {:.3} ms p99 {:.3} ms",
                c.0, c.1, g.0, g.1
            );
        }
    }

    #[test]
    fn the_layers_menu_toggles_combustion_and_the_cut() {
        use egui_kittest::{Harness, kittest::Queryable};
        let st = View3dState {
            available: true,
            ..View3dState::default()
        };
        let mut h = Harness::new_ui_state(|ui, s: &mut View3dState| bar(ui, s), st);
        h.get_by_label("Layers").click();
        h.run();
        h.get_by_label("Cut-away").click();
        h.run();
        assert!(h.state().layers.cutaway);
        if h.query_by_label("Combustion").is_none() {
            h.get_by_label("Layers").click();
            h.run();
        }
        h.get_by_label("Combustion").click();
        h.run();
        assert!(!h.state().layers.combustion);
    }

    #[test]
    fn layers_shape_the_frame() {
        let def = EngineDef::inline4();
        let mut st = View3dState::default();
        st.sync(&def);
        let snap = crate::sim::Simulation::new(def, 48_000.0, 1).snapshot();
        st.fill(&snap, 1.5, 0.0);
        let count = |st: &View3dState| {
            let sh = st.shared.lock().unwrap();
            (
                sh.instances
                    .iter()
                    .filter(|i| i.part == scene::Part::Charge)
                    .count(),
                sh.clip,
            )
        };
        assert_eq!(count(&st), (4, None));
        st.layers = Layers {
            combustion: false,
            cutaway: true,
            ..Layers::default()
        };
        st.fill(&snap, 1.5, 0.0);
        assert_eq!(count(&st), (0, Some((glam::Vec3::Z, 0.0))));
    }

    #[test]
    fn the_pipes_take_their_temperature_and_the_marks_run_with_the_gas() {
        use engine_core::exhaust::PipeState;
        let def = EngineDef::inline4();
        let mut st = View3dState::default();
        st.governor.set_manual(Some(Detail::Medium));
        st.sync(&def);
        let mut snap = crate::sim::Simulation::new(def, 48_000.0, 1).snapshot();
        let n = st.scene.as_ref().unwrap().pipes.len();
        snap.pipes = (0..n)
            .map(|_| PipeState {
                temperature_k: 900.0,
                velocity_m_s: 20.0,
                wall_k: None,
            })
            .collect();
        st.fill(&snap, 1.5, 0.0);
        let path = st.scene.as_ref().unwrap().pipes[0].clone();
        let len: f32 = path
            .runs
            .iter()
            .map(super::super::routing::Run::length)
            .sum();
        {
            let sh = st.shared.lock().unwrap();
            let hot = super::super::colormap::temperature(900.0);
            assert!(
                sh.instances[path.instances.clone()]
                    .iter()
                    .all(|i| i.color == hot)
            );
            // 4 marks per pipe at Medium
            let glowing = sh.instances.iter().filter(|i| i.color[3] == 0.3).count();
            assert_eq!(glowing, 4 * n);
        }
        // after 0.1 s the marks have run 20 · 0.05 · 0.1 = 0.1 m along their pipe
        st.fill(&snap, 1.5, 0.1);
        assert!(
            (st.phases[0] - 0.1 / len).abs() < 1e-5,
            "{} vs {}",
            st.phases[0],
            0.1 / len
        );
        // no gas data: no marks, the pipes keep their colour
        snap.pipes.clear();
        st.fill(&snap, 1.5, 0.1);
        let sh = st.shared.lock().unwrap();
        assert!(!sh.instances.iter().any(|i| i.color[3] == 0.3));
    }

    #[test]
    fn an_edit_tints_its_parts_for_half_a_second() {
        use super::super::handles::Field;
        let mut t = View3dState::default();
        assert_eq!(t.tint_field(0.0), None);
        t.touch(Field::IntakeOpens, 10.0);
        assert_eq!(t.tint_field(10.0), Some(Field::IntakeOpens));
        assert_eq!(t.tint_field(10.49), Some(Field::IntakeOpens));
        assert_eq!(t.tint_field(10.5), None, "off at the timeout");
        // another touch while it is on takes it over and runs the clock again
        t.touch(Field::Bore, 10.4);
        t.touch(Field::Rod, 10.8);
        assert_eq!(t.tint_field(11.2), Some(Field::Rod));
        assert_eq!(t.tint_field(11.3), None);
    }

    #[test]
    fn the_tint_colours_the_edited_parts_only() {
        use super::super::handles::Field;
        let def = EngineDef::inline4();
        let mut st = View3dState::default();
        st.governor.set_manual(Some(Detail::High));
        st.sync(&def);
        let snap = crate::sim::Simulation::new(def.clone(), 48_000.0, 1).snapshot();
        st.fill(&snap, 1.5, 0.0);
        let colours = |st: &View3dState| -> Vec<(scene::Part, [f32; 4])> {
            let sh = st.shared.lock().unwrap();
            sh.instances.iter().map(|i| (i.part, i.color)).collect()
        };
        let before = colours(&st);
        // nothing touched: nothing tinted
        st.tint_frame(&snap.def, 5.0);
        assert_eq!(st.tinted, None);
        assert_eq!(colours(&st), before);
        st.touch(Field::IntakeOpens, 5.0);
        st.tint_frame(&snap.def, 5.1);
        assert_eq!(st.tinted, Some(Field::IntakeOpens));
        let after = colours(&st);
        let mut hit = 0;
        for (b, a) in before.iter().zip(&after) {
            if a.1 != b.1 {
                hit += 1;
                assert!(
                    matches!(
                        b.0,
                        scene::Part::IntakeValve | scene::Part::IntakeStem | scene::Part::IntakeCam
                    ),
                    "{:?} tinted",
                    b.0
                );
                // towards amber: red and green up, blue down
                assert!(
                    a.1[0] >= b.1[0] && a.1[1] >= b.1[1] && a.1[2] <= b.1[2],
                    "{a:?} {b:?}"
                );
            }
        }
        assert!(
            hit >= 3 * 4,
            "{hit} parts tinted (4 cylinders × valve, stem, cam)"
        );
        // past the timeout the next frame is plain again
        st.fill(&snap, 1.5, 0.0);
        st.tint_frame(&snap.def, 5.6);
        assert_eq!(st.tinted, None);
        assert_eq!(colours(&st), before);
    }
}
