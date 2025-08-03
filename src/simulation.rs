use egui::{Button, Context, Slider, Ui, vec2};
use macroquad::{
    camera::Camera2D,
    color::{Color, colors},
    input::{self, KeyCode, MouseButton},
    math::Vec2,
    window,
};
use slotmap::{HopSlotMap, new_key_type};

use crate::{
    constraint::{Constraint, PointHandle},
    particle::Particle,
    soft_body::{
        Actor, AttatchmentPointHandle, BoundingBox, ConnectionState, JoiningSpring, Keybind,
        LinearSpring, Point, SoftBody,
    },
    utils,
};

#[derive(Clone, Debug)]
pub struct Simulation {
    pub soft_bodies: HopSlotMap<SoftBodyKey, SoftBody>,
    pub keys: Vec<SoftBodyKey>,

    pub particles: Vec<Particle>,

    pub constraints: HopSlotMap<ConstraintKey, Constraint>,

    pub input_state: InputState,
}

new_key_type! {
    pub struct SoftBodyKey;
    pub struct ConstraintKey;
}

#[derive(Clone, Debug)]
pub struct InputState {
    pub selected_attatchment_point: Option<(AttatchmentPointHandle, f32)>,
    pub target_attatchment_point: Option<AttatchmentPointHandle>,
    pub can_connect: bool,

    pub grabbing: bool,
    pub clicking: bool,

    pub ui_hovered: bool,

    pub mouse: Point,

    pub editing: bool,
    pub selected_soft_body: Option<SoftBodyKey>,

    pub keybind_focus: Option<KeybindFocus>,
}

impl Default for InputState {
    fn default() -> Self {
        Self {
            selected_attatchment_point: None,
            target_attatchment_point: None,
            can_connect: false,

            grabbing: false,
            clicking: false,

            ui_hovered: false,

            mouse: Point::default(),

            editing: true,
            selected_soft_body: None,

            keybind_focus: None,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum KeybindFocus {
    Activate(usize),
    NewActivate,
    Disable(usize),
    NewDisable,
}

impl Simulation {
    pub const GRAB_SPRING: JoiningSpring = JoiningSpring {
        force_constant: 10.0,
        normal_damping: 2.5,
        perpendicular_damping: 5.0,
        compression: true,
        tension: true,
        maximum_force: 0.75,
        maximum_normal_damping: 50.0,
        maximum_perpendicular_damping: 50.0,
    };

    pub const PULL_SPRING: JoiningSpring = JoiningSpring {
        force_constant: 10.0,
        normal_damping: 2.5,
        perpendicular_damping: 10.0,
        compression: true,
        tension: true,
        maximum_force: 0.75,
        maximum_normal_damping: 50.0,
        maximum_perpendicular_damping: 50.0,
    };

    pub const ALIGN_SPRING: LinearSpring = LinearSpring {
        target_distance: 0.0,
        force_constant: 100.0,
        damping: 50.0,
        compression: true,
        tension: true,
        maximum_force: 50.0,
        maximum_damping: 50.0,
        destroy_on_maximum: false,
    };

    pub const MAXIMUM_ATTATCHMENT_DISTANCE: f32 = 0.5;

    pub fn new() -> Self {
        Self {
            soft_bodies: HopSlotMap::default(),
            keys: Vec::new(),

            particles: Vec::new(),

            constraints: HopSlotMap::default(),

            input_state: InputState::default(),
        }
    }

    pub fn draw(&self, debug: bool, bounding_box: BoundingBox) {
        for particle in &self.particles {
            if bounding_box.is_point_within_distance(particle.position, particle.size()) {
                particle.draw();
            }
        }

        for (_, soft_body) in &self.soft_bodies {
            if bounding_box.is_other_within_distance(&soft_body.bounding_box, 0.2) {
                soft_body.draw_actors_back();
            }
        }

        for (_, soft_body) in &self.soft_bodies {
            if bounding_box.is_other_within_distance(&soft_body.bounding_box, 0.2) {
                soft_body.draw();
            }
        }

        if let Some(selected) = self.input_state.selected_soft_body {
            if let Some(soft_body) = self.soft_bodies.get(selected) {
                soft_body.outline_color(0.05, colors::BLUE);
            }
        }

        for (_, soft_body) in &self.soft_bodies {
            if bounding_box.is_other_within_distance(&soft_body.bounding_box, 0.2) {
                soft_body.draw_actors_front();
            }
        }

        for (_, soft_body) in &self.soft_bodies {
            if bounding_box.is_other_within_distance(&soft_body.bounding_box, 0.2) {
                soft_body.draw_attatchment_points();
            }
        }

        let color = if self.input_state.target_attatchment_point.is_some()
            && self.input_state.can_connect
        {
            Some(colors::BLUE)
        } else {
            None
        };

        if let Some((attatchment_point, _)) = self.input_state.selected_attatchment_point {
            if let Some(soft_body) = self.soft_bodies.get(attatchment_point.soft_body) {
                soft_body.draw_attatchment_point(attatchment_point.index, true, color);
            }
        }

        if let Some(attatchment_point) = self.input_state.target_attatchment_point {
            if let Some(soft_body) = self.soft_bodies.get(attatchment_point.soft_body) {
                soft_body.draw_attatchment_point(attatchment_point.index, true, color);
            }
        }

        if debug {
            for (_, soft_body) in &self.soft_bodies {
                if soft_body.pressure > f32::EPSILON {
                    soft_body.fill_color(Color {
                        a: (soft_body.pressure / soft_body.gas_force).clamp(0.0, 1.0) / 2.0,
                        ..utils::generate_color_for_spring(soft_body.pressure / 3.0, 0.0)
                    });
                }
            }

            for (_, soft_body) in &self.soft_bodies {
                if bounding_box.is_other_within_distance(&soft_body.bounding_box, 0.2) {
                    soft_body.draw_springs();
                }
            }
        }

        egui_macroquad::draw();
    }

    pub fn update_keys(&mut self) {
        self.keys = self.soft_bodies.keys().collect();
    }

    pub fn tick_simulation(&mut self, dt: f32) -> Option<Vec2> {
        for particle in &mut self.particles {
            particle.tick(dt);
        }

        let mut i = 0;

        while i < self.particles.len() {
            let particle = &self.particles[i];
            if particle.age > particle.end_age {
                self.particles.remove(i);
                continue;
            }

            i += 1;
        }

        let mut camera_position = None;

        self.update_grabbing(dt);

        let mut unstable_soft_bodies = Vec::new();

        for (i, &key) in self.keys.iter().enumerate() {
            let soft_body = &mut self.soft_bodies[key];

            let (new_camera_position, mut new_particles, unstable) =
                soft_body.apply_impulse_and_velocity(dt);

            self.particles.append(&mut new_particles);

            if camera_position.is_none() {
                camera_position = new_camera_position;
            }

            if unstable && !soft_body.is_debris() {
                unstable_soft_bodies.push((i, key));
            }
        }

        let mut empty_constraints = Vec::new();

        for (key, constraint) in &mut self.constraints {
            constraint.apply_to_soft_bodies(&mut self.soft_bodies);

            if constraint.is_empty() {
                empty_constraints.push(key);
            }
        }

        for key in empty_constraints {
            self.remove_constraint(key, None);
        }

        for (i, &first_key) in self.keys.iter().enumerate().skip(1) {
            for &second_key in self.keys.iter().take(i) {
                let [first, second] = self
                    .soft_bodies
                    .get_disjoint_mut([first_key, second_key])
                    .unwrap();

                if first.bounding_box.intersects_other(&second.bounding_box) {
                    first.check_points_against_other_one_sided(second);
                    second.check_points_against_other_one_sided(first);
                }
            }
        }

        for (i, key) in unstable_soft_bodies {
            self.destroy_soft_body(key, Some(i));
        }

        let mut i = 0;

        while i < self.keys.len() {
            let key = self.keys[i];

            let soft_body = &mut self.soft_bodies[key];

            if let Some(debris_age) = soft_body.debris_age {
                if debris_age >= SoftBody::DEBRIS_DECAY_TIME || soft_body.area() < 0.0 {
                    self.soft_bodies.remove(key);
                    self.keys.swap_remove(i);

                    continue;
                }
            } else {
                if soft_body.is_self_intersecting() {
                    self.destroy_soft_body(key, Some(i));

                    continue;
                }
            }

            soft_body.update_triangulation_indecies();

            i += 1;
        }

        self.input_state.clicking = false;

        camera_position
    }

    pub fn update_input(&mut self, camera: &Camera2D, dt: f32) {
        const SELECTION_RANGE: f32 = 0.25;

        self.update_gui();

        let mouse_position = utils::mouse_position(camera);

        self.input_state.mouse.velocity = (mouse_position - self.input_state.mouse.position) / dt;
        self.input_state.mouse.position = mouse_position;
        self.input_state.mouse.mass = 10000.0;

        if self.input_state.grabbing || !self.input_state.ui_hovered {
            self.input_state.clicking |= input::is_mouse_button_pressed(MouseButton::Left);
            self.input_state.grabbing =
                self.input_state.clicking || input::is_mouse_button_down(MouseButton::Left);
        } else {
            self.input_state.clicking = false;
            self.input_state.grabbing = false;
        }

        let mut selected_attatchment_point = None;
        let mut selected_distance_squared = f32::INFINITY;

        if !self.input_state.ui_hovered {
            let key_to_skip = (self.input_state.grabbing)
                .then(|| {
                    self.input_state
                        .selected_attatchment_point
                        .map(|(AttatchmentPointHandle { soft_body, .. }, _)| soft_body)
                })
                .flatten();

            let required_length = (self.input_state.grabbing)
                .then(|| {
                    self.input_state.selected_attatchment_point.map(
                        |(AttatchmentPointHandle { soft_body, index }, _)| {
                            Some(self.soft_bodies.get(soft_body)?.attatchment_points[index].length)
                        },
                    )
                })
                .flatten()
                .flatten();

            for (key, soft_body) in &self.soft_bodies {
                if !soft_body
                    .bounding_box
                    .is_point_within_distance(mouse_position, 0.25)
                    || Some(key) == key_to_skip
                {
                    continue;
                }

                for (index, attatchment_point) in soft_body.attatchment_points.iter().enumerate() {
                    if attatchment_point.connection.is_some() && self.input_state.grabbing {
                        continue;
                    }

                    if let Some(required_length) = required_length {
                        if required_length != attatchment_point.length {
                            continue;
                        }
                    }

                    let mut i = attatchment_point.start_point;
                    let mut point_index = 0;

                    if attatchment_point.length < 2 {
                        let (Point { position, .. }, _) = soft_body.shape[i];

                        let distance_squared = position.distance_squared(mouse_position);

                        if distance_squared < SELECTION_RANGE.powi(2)
                            && distance_squared < selected_distance_squared
                        {
                            selected_attatchment_point = Some((
                                AttatchmentPointHandle {
                                    soft_body: key,
                                    index,
                                },
                                0.0,
                            ));

                            selected_distance_squared = distance_squared;
                        }

                        continue;
                    }

                    let (&Point { position: a, .. }, _, &Point { position: b, .. }) =
                        soft_body.get_line(i).unwrap();
                    let (closest_point, mut line_progress_of_minimum) =
                        utils::closest_point_on_line(a, b, mouse_position);

                    let mut minimum_distance_squared =
                        closest_point.distance_squared(mouse_position);

                    i = soft_body.next_point(i);
                    point_index += 1;

                    for _ in 2..attatchment_point.length {
                        let (&Point { position: a, .. }, _, &Point { position: b, .. }) =
                            soft_body.get_line(i).unwrap();
                        let (closest_point, line_progress) =
                            utils::closest_point_on_line(a, b, mouse_position);

                        let distance_squared = closest_point.distance_squared(mouse_position);

                        if distance_squared < minimum_distance_squared {
                            minimum_distance_squared = distance_squared;
                            line_progress_of_minimum = line_progress + point_index as f32;
                        }

                        i = soft_body.next_point(i);
                        point_index += 1;
                    }

                    if minimum_distance_squared < SELECTION_RANGE.powi(2)
                        && minimum_distance_squared < selected_distance_squared
                    {
                        selected_attatchment_point = Some((
                            AttatchmentPointHandle {
                                soft_body: key,
                                index,
                            },
                            line_progress_of_minimum,
                        ));

                        selected_distance_squared = minimum_distance_squared;
                    }
                }
            }
        }

        self.input_state.can_connect = false;
        if let Some(target) = self.input_state.target_attatchment_point {
            if let Some((selected, _)) = self.input_state.selected_attatchment_point {
                self.input_state.can_connect = self
                    .are_attatchment_points_within_range(
                        [selected, target],
                        Self::MAXIMUM_ATTATCHMENT_DISTANCE,
                    )
                    .unwrap_or(false);
            }
        }

        if self.input_state.grabbing {
            if self.input_state.selected_attatchment_point.is_some() {
                self.input_state.target_attatchment_point =
                    selected_attatchment_point.map(|(handle, _)| handle);
            }
        } else {
            if let Some(target) = self.input_state.target_attatchment_point {
                if let Some((selected, _)) = self.input_state.selected_attatchment_point {
                    if self
                        .are_attatchment_points_within_range(
                            [selected, target],
                            Self::MAXIMUM_ATTATCHMENT_DISTANCE,
                        )
                        .unwrap()
                    {
                        self.connect_attatchment_points([selected, target]).unwrap();
                    }
                }

                self.input_state.target_attatchment_point = None;
            }

            self.input_state.selected_attatchment_point = selected_attatchment_point;
        }

        if !self.input_state.ui_hovered {
            if input::is_mouse_button_pressed(MouseButton::Left) {
                self.input_state.editing = false;
            }

            if input::is_mouse_button_pressed(MouseButton::Right) {
                'outer: {
                    for (key, soft_body) in &self.soft_bodies {
                        if soft_body.uses_keybinds() && soft_body.contains_point(mouse_position) {
                            self.input_state.editing = true;
                            self.input_state.selected_soft_body = Some(key);
                            break 'outer;
                        }
                    }

                    self.input_state.editing = false;
                }
            }
        }

        if !self.input_state.editing {
            self.input_state.selected_soft_body = None;
        }

        if input::is_key_pressed(KeyCode::F1) {
            if self.input_state.editing && self.input_state.selected_soft_body.is_none() {
                self.input_state.editing = false;
            } else {
                self.input_state.editing = true;
                self.input_state.selected_soft_body = None;
            }
        }
    }

    pub fn update_gui(&mut self) {
        egui_macroquad::ui(|egui| {
            self.update_keybind_editor(egui);

            self.input_state.ui_hovered = egui.is_pointer_over_area();
        });
    }

    pub fn update_keybind_editor(&mut self, egui: &Context) {
        egui.set_zoom_factor(window::screen_dpi_scale());

        let window = egui::Window::new("Info")
            .resizable(false)
            .movable(false)
            .collapsible(false)
            .open(&mut self.input_state.editing);

        window.show(egui, |ui| {
            let Some(soft_body_key) = self.input_state.selected_soft_body else {
                ui.label("This is a physics sandbox for building spaceships.");
                ui.label("The orb with a white circle inside of it is your habitat bubble. If it is destroyed, \
                    you will need to restart the program to regain control of the camera and interactables.");
                ui.label("Click and drag on a white line to connect it to another or move it around. After \
                    being connected, click on it again to disconnect.");
                ui.label("Right click on an interactible to view and edit its keybinds. It can be used when \
                    connected to your habitat bubble.");
                ui.label("Press F1 to toggle this menu.");

                return;
            };

            let Some(soft_body) = self.soft_bodies.get_mut(soft_body_key) else {
                return;
            };

            if input::is_mouse_button_pressed(MouseButton::Left) {
                self.input_state.keybind_focus = None;
            }

            let mut show_keybind = |name: &str, keybind: &mut Keybind, ui: &mut Ui| {
                let mut show_key = |focus: KeybindFocus, key: Option<&KeyCode>, ui: &mut Ui| {
                    ui.horizontal(|ui| {
                        let focused = self.input_state.keybind_focus == Some(focus);

                        let size = if key.is_some() {
                            vec2(150.0, 0.0)
                        } else {
                            vec2(20.0, 20.0)
                        };

                        let button = if focused {
                            Button::new("press a key")
                        } else {
                            Button::new(if let Some(key) = key {
                                format!("{key:?}")
                            } else {
                                "+".to_owned()
                            })
                        }
                        .min_size(size);

                        if ui
                            .add(button)
                            .on_hover_text("Backspace or Delete to remove")
                            .clicked()
                        {
                            self.input_state.keybind_focus = Some(focus);
                        }
                    });
                };

                ui.heading(name);

                ui.label("Any of:");
                for (i, key) in keybind.activate.iter().enumerate() {
                    show_key(KeybindFocus::Activate(i), Some(key), ui);
                }
                ui.add_space(2.5);
                show_key(KeybindFocus::NewActivate, None, ui);

                ui.add_space(5.0);

                ui.label("None of:");
                for (i, key) in keybind.disable.iter().enumerate() {
                    show_key(KeybindFocus::Disable(i), Some(key), ui);
                }
                ui.add_space(2.5);
                show_key(KeybindFocus::NewDisable, None, ui);

                if let (Some(keybind_focus), Some(key_code)) = (
                    self.input_state.keybind_focus,
                    input::get_last_key_pressed(),
                ) {
                    if key_code == KeyCode::Escape
                        || Some(key_code) == keybind.get(keybind_focus)
                        || (KeyCode::F1 as u16..=KeyCode::F25 as u16).contains(&(key_code as u16))
                    {
                    } else if key_code == KeyCode::Delete || key_code == KeyCode::Backspace {
                        match keybind_focus {
                            KeybindFocus::Activate(i) => {
                                keybind.activate.remove(i);
                            }
                            KeybindFocus::Disable(i) => {
                                keybind.disable.remove(i);
                            }
                            _ => (),
                        }
                    } else {
                        keybind.remove(key_code);

                        match keybind_focus {
                            KeybindFocus::Activate(i) => keybind.activate[i] = key_code,
                            KeybindFocus::NewActivate => keybind.activate.push(key_code),
                            KeybindFocus::Disable(i) => keybind.disable[i] = key_code,
                            KeybindFocus::NewDisable => keybind.disable.push(key_code),
                        }
                    }

                    self.input_state.keybind_focus = None;
                }
            };

            for (i, actor) in soft_body.actors.iter_mut().enumerate() {
                if i != 0 {
                    ui.add_space(5.0);
                }

                match actor {
                    Actor::RocketMotor {
                        force,
                        enable,
                        max_particle_time,
                        ..
                    } => {
                        show_keybind("Enable Thrust", enable, ui);
                        ui.add_space(5.0);

                        ui.label("Force");
                        let length = force.length();
                        let mut new_length = length;
                        ui.add(Slider::new(&mut new_length, 25.0..=200.0));

                        if length != new_length {
                            *force = force.normalize_or_zero() * new_length;
                            *max_particle_time = 0.5 / new_length;
                        }
                    }
                    Actor::HabitatBubble { .. } => (),
                    Actor::Piston { lengths, enable } => {
                        show_keybind("Extend", enable, ui);

                        ui.add_space(5.0);

                        ui.label("Length");
                        let (_, off_length, on_length) = lengths.first().unwrap();
                        let length = on_length / off_length;
                        let mut new_length = length;
                        ui.add(Slider::new(&mut new_length, 1.0..=6.0));

                        if length != new_length {
                            for (_, off_length, on_length) in lengths {
                                *on_length = *off_length * new_length;
                            }
                        }
                    }
                }
            }
        });
    }

    pub fn update_grabbing(&mut self, dt: f32) {
        if let Some((handle, progress)) = self.input_state.selected_attatchment_point {
            if !self.soft_bodies.contains_key(handle.soft_body) {
                self.input_state.selected_attatchment_point = None;
            } else if self.input_state.grabbing {
                if self.input_state.clicking
                    && self.soft_bodies[handle.soft_body].attatchment_points[handle.index]
                        .connection
                        .is_some()
                {
                    self.disconnect_attatchment_point(handle).unwrap();
                    self.input_state.selected_attatchment_point = None;
                    self.input_state.target_attatchment_point = None;
                    return;
                }

                if let Some(target) = self.input_state.target_attatchment_point {
                    self.push_together([handle, target], dt);
                } else {
                    self.push_towards_mouse(handle, progress, dt);
                }
            }
        }
    }

    pub fn push_together(&mut self, [handle_a, handle_b]: [AttatchmentPointHandle; 2], dt: f32) {
        let [soft_body_a, soft_body_b] = self
            .soft_bodies
            .get_disjoint_mut([handle_a.soft_body, handle_b.soft_body])
            .unwrap();

        let length_a = soft_body_a.shape.len();
        let length_b = soft_body_b.shape.len();

        let attatchment_point_a = soft_body_a.attatchment_points[handle_a.index];
        let attatchment_point_b = soft_body_b.attatchment_points[handle_b.index];

        let mut mass_moment_a = Vec2::ZERO;
        let mut mass_moment_b = Vec2::ZERO;

        let mut momentum_a = Vec2::ZERO;
        let mut momentum_b = Vec2::ZERO;

        let mut total_mass_a = 0.0;
        let mut total_mass_b = 0.0;

        let mut point_a = attatchment_point_a.start_point;
        let mut point_b =
            (attatchment_point_b.start_point + attatchment_point_b.length - 1) % length_b;

        for _ in 0..attatchment_point_a.length {
            {
                let (point_a, _) = &mut soft_body_a.shape[point_a];
                let (point_b, _) = &mut soft_body_b.shape[point_b];

                mass_moment_a += point_a.position * point_a.mass;
                mass_moment_b += point_b.position * point_b.mass;

                momentum_a += point_a.velocity * point_a.mass;
                momentum_b += point_b.velocity * point_b.mass;

                total_mass_a += point_a.mass;
                total_mass_b += point_b.mass;

                let (_, _, impulse, _) = Self::PULL_SPRING.get_force(point_a, point_b);

                point_a.impulse += impulse / 2.0 * dt * point_a.mass;
            }

            if point_a < length_a - 1 {
                point_a += 1;
            } else {
                point_a = 0;
            }

            if point_b > 0 {
                point_b -= 1;
            } else {
                point_b = length_b - 1;
            }
        }

        let center_of_mass_a = mass_moment_a / total_mass_a;
        let center_of_mass_b = mass_moment_b / total_mass_b;
        let position_offset = center_of_mass_a - center_of_mass_b;

        let velocity_a = momentum_a / total_mass_a;
        let velocity_b = momentum_b / total_mass_b;

        let velocity_offset = velocity_a - velocity_b;

        let mut point_a = attatchment_point_a.start_point;
        let mut point_b =
            (attatchment_point_b.start_point + attatchment_point_b.length - 1) % length_b;

        for _ in 0..attatchment_point_a.length {
            {
                let (point_a, _) = &mut soft_body_a.shape[point_a];
                let (point_b, _) = &mut soft_body_b.shape[point_b];

                let mut moved_point_b = Point {
                    position: point_b.position + position_offset,
                    velocity: point_b.velocity + velocity_offset,
                    ..*point_b
                };

                let (_, _, impulse, _) = Self::ALIGN_SPRING.get_force(point_a, &mut moved_point_b);

                point_a.impulse += impulse / 2.0 * dt * point_a.mass;
                point_b.impulse -= impulse / 2.0 * dt * point_b.mass;
            }

            if point_a < length_a - 1 {
                point_a += 1;
            } else {
                point_a = 0;
            }

            if point_b > 0 {
                point_b -= 1;
            } else {
                point_b = length_b - 1;
            }
        }
    }

    pub fn push_towards_mouse(&mut self, handle: AttatchmentPointHandle, progress: f32, dt: f32) {
        let line_offset = progress.floor() as usize;
        let interpolation = progress.rem_euclid(1.0);

        let soft_body = &mut self.soft_bodies[handle.soft_body];
        let length = soft_body.shape.len();

        let attatchment_point = soft_body.attatchment_points[handle.index];

        let (point_a, _, point_b) = soft_body
            .get_line_mut((attatchment_point.start_point + line_offset) % length)
            .unwrap();

        let interpolation_scale = utils::interpolation_scale(interpolation);

        let mut composite_point = Point {
            position: point_a.position.lerp(point_b.position, interpolation),
            velocity: point_a.velocity.lerp(point_b.velocity, interpolation),
            mass: utils::lerp(point_a.mass, point_b.mass, interpolation) * interpolation_scale,
            ..Default::default()
        };

        let mut mouse = Point {
            velocity: composite_point.velocity.lerp(
                self.input_state.mouse.velocity,
                1.0 / (self.input_state.mouse.position)
                    .distance_squared(composite_point.position)
                    .max(1.0),
            ),
            ..self.input_state.mouse
        };

        Self::GRAB_SPRING.apply_force(&mut mouse, &mut composite_point, dt);

        let impulse = composite_point.impulse;

        let mut i = attatchment_point.start_point;

        for _ in 0..attatchment_point.length {
            let (point, _) = &mut soft_body.shape[i];

            point.impulse += impulse * point.mass;

            i = soft_body.next_point(i);
        }
    }

    pub fn destroy_soft_body(&mut self, key: SoftBodyKey, key_index: Option<usize>) {
        for (index, attatchment_point) in self.soft_bodies[key]
            .attatchment_points
            .clone()
            .into_iter()
            .enumerate()
        {
            if attatchment_point.connection.is_some() {
                self.disconnect_attatchment_point(AttatchmentPointHandle {
                    soft_body: key,
                    index,
                })
                .unwrap();
            }
        }

        let soft_body = self.soft_bodies.remove(key).unwrap();
        if let Some(i) = key_index {
            self.keys.swap_remove(i);
        }

        for triangle in soft_body.decompose_into_triangles() {
            let key = self.soft_bodies.insert(triangle);
            if key_index.is_some() {
                self.keys.push(key);
            }
        }

        if key_index.is_none() {
            self.update_keys();
        }
    }

    pub fn insert_constraint(&mut self, mut constraint: Constraint) -> ConstraintKey {
        let mut keys_to_replace = Vec::new();

        let key = self.constraints.insert_with_key(|key| {
            constraint.insert(key, &mut self.soft_bodies, &mut keys_to_replace);
            constraint
        });

        for key_to_replace in keys_to_replace {
            self.remove_constraint(key_to_replace, Some(key));
        }

        key
    }

    pub fn remove_constraint(
        &mut self,
        key: ConstraintKey,
        replacement: Option<ConstraintKey>,
    ) -> Option<Constraint> {
        let constraint = self.constraints.remove(key);

        if let Some(constraint) = constraint {
            let mut points_to_replace = Vec::new();
            constraint.remove(
                key,
                replacement,
                &mut self.soft_bodies,
                &mut points_to_replace,
            );

            if let Some(replacement) = replacement {
                let constraint = &mut self.constraints[replacement];

                for point in points_to_replace {
                    match constraint {
                        Constraint::HoldTogether { points } => points.push(point),
                    }
                }
            }

            Some(constraint)
        } else {
            None
        }
    }

    pub fn clear_connections_from(&mut self, soft_body_key: SoftBodyKey) -> Option<SoftBodyKey> {
        let soft_body = &mut self.soft_bodies[soft_body_key];

        let mut source = None;

        if soft_body.connection_state.is_connected() {
            if soft_body.connection_state == ConnectionState::Connected {
                soft_body.connection_state = ConnectionState::Disconnected;
            }

            if soft_body.connection_state == ConnectionState::Source {
                source = Some(soft_body_key);
            }

            for attatchment_point in soft_body.attatchment_points.clone() {
                let Some(AttatchmentPointHandle {
                    soft_body: connection,
                    ..
                }) = attatchment_point.connection
                else {
                    continue;
                };

                let result = self.clear_connections_from(connection);

                if result.is_some() {
                    source = result;
                }
            }
        }

        source
    }

    pub fn connect_attatched_soft_bodies(&mut self, soft_body_key: SoftBodyKey) {
        let soft_body = &mut self.soft_bodies[soft_body_key];

        if matches!(
            soft_body.connection_state,
            ConnectionState::Disconnected | ConnectionState::Source,
        ) {
            if soft_body.connection_state == ConnectionState::Disconnected {
                soft_body.connection_state = ConnectionState::Connected;
            }

            for attatchment_point in soft_body.attatchment_points.clone() {
                let Some(AttatchmentPointHandle {
                    soft_body: connection,
                    ..
                }) = attatchment_point.connection
                else {
                    continue;
                };

                self.connect_attatched_soft_bodies(connection);
            }
        }
    }

    #[must_use]
    pub fn are_attatchment_points_within_range(
        &self,
        [handle_a, handle_b]: [AttatchmentPointHandle; 2],
        maximum_distance: f32,
    ) -> Option<bool> {
        let soft_body_a = self.soft_bodies.get(handle_a.soft_body)?;
        let soft_body_b = self.soft_bodies.get(handle_b.soft_body)?;

        let attatchment_point_a = soft_body_a.attatchment_points.get(handle_a.index)?;
        let attatchment_point_b = soft_body_b.attatchment_points.get(handle_b.index)?;

        if attatchment_point_a.length != attatchment_point_b.length
            || attatchment_point_a.connection.is_some()
            || attatchment_point_b.connection.is_some()
        {
            return None;
        }

        if maximum_distance.is_finite() {
            let mut point_a = attatchment_point_a.start_point;
            let mut point_b = (attatchment_point_b.start_point + attatchment_point_b.length - 1)
                % soft_body_b.shape.len();

            let maximum_distance_squared = maximum_distance.powi(2);

            for _ in 0..attatchment_point_a.length {
                if (soft_body_a.shape[point_a].0.position)
                    .distance_squared(soft_body_b.shape[point_b].0.position)
                    > maximum_distance_squared
                {
                    return Some(false);
                }

                if point_a < soft_body_a.shape.len() - 1 {
                    point_a += 1;
                } else {
                    point_a = 0;
                }

                if point_b > 0 {
                    point_b -= 1;
                } else {
                    point_b = soft_body_b.shape.len() - 1;
                }
            }
        }

        Some(true)
    }

    /// Returns `None` if both handles point to the same soft body, if either is invalid, or if
    /// they don't have the same length.
    #[must_use]
    pub fn connect_attatchment_points(
        &mut self,
        [handle_a, handle_b]: [AttatchmentPointHandle; 2],
    ) -> Option<()> {
        if (self.soft_bodies[handle_a.soft_body].connection_state).is_connected() {
            self.connect_attatched_soft_bodies(handle_b.soft_body);
        } else if (self.soft_bodies[handle_b.soft_body].connection_state).is_connected() {
            self.connect_attatched_soft_bodies(handle_a.soft_body);
        }

        let [soft_body_a, soft_body_b] = self
            .soft_bodies
            .get_disjoint_mut([handle_a.soft_body, handle_b.soft_body])?;

        let length_a = soft_body_a.shape.len();
        let length_b = soft_body_b.shape.len();

        let attatchment_point_a = soft_body_a.attatchment_points.get_mut(handle_a.index)?;
        let attatchment_point_b = soft_body_b.attatchment_points.get_mut(handle_b.index)?;

        if attatchment_point_a.length != attatchment_point_b.length
            || attatchment_point_a.connection.is_some()
            || attatchment_point_b.connection.is_some()
        {
            return None;
        }

        // Connect points
        attatchment_point_a.connection = Some(handle_b);
        attatchment_point_b.connection = Some(handle_a);

        let mut point_a = attatchment_point_a.start_point;
        let mut point_b =
            (attatchment_point_b.start_point + attatchment_point_b.length - 1) % length_b;

        let mut new_constraints = Vec::new();

        for _ in 0..attatchment_point_a.length {
            soft_body_a.shape[point_a].0.num_connections += 1;
            soft_body_b.shape[point_b].0.num_connections += 1;

            new_constraints.push(Constraint::HoldTogether {
                points: vec![
                    PointHandle {
                        soft_body: handle_a.soft_body,
                        index: point_a,
                    },
                    PointHandle {
                        soft_body: handle_b.soft_body,
                        index: point_b,
                    },
                ],
            });

            if point_a < length_a - 1 {
                point_a += 1;
            } else {
                point_a = 0;
            }

            if point_b > 0 {
                point_b -= 1;
            } else {
                point_b = length_b - 1;
            }
        }

        for constraint in new_constraints {
            self.insert_constraint(constraint);
        }

        Some(())
    }

    #[must_use]
    pub fn disconnect_attatchment_point(&mut self, handle_a: AttatchmentPointHandle) -> Option<()> {
        let source = self.clear_connections_from(handle_a.soft_body);

        let handle_b = &self
            .soft_bodies
            .get(handle_a.soft_body)?
            .attatchment_points
            .get(handle_a.index)?
            .connection?;

        let other_connection = self
            .soft_bodies
            .get_mut(handle_b.soft_body)?
            .attatchment_points
            .get_mut(handle_b.index)?
            .connection;

        if other_connection != Some(handle_a) {
            return None;
        }

        let [soft_body_a, soft_body_b] = self
            .soft_bodies
            .get_disjoint_mut([handle_a.soft_body, handle_b.soft_body])?;

        let length_a = soft_body_a.shape.len();
        let length_b = soft_body_b.shape.len();

        let attatchment_point_a = soft_body_a.attatchment_points.get_mut(handle_a.index)?;
        let attatchment_point_b = soft_body_b.attatchment_points.get_mut(handle_b.index)?;

        attatchment_point_a.connection = None;
        attatchment_point_b.connection = None;

        let mut point_a = attatchment_point_a.start_point;
        let mut point_b =
            (attatchment_point_b.start_point + attatchment_point_b.length - 1) % length_b;

        for _ in 0..attatchment_point_a.length {
            soft_body_a.shape[point_a].0.num_connections -= 1;
            soft_body_b.shape[point_b].0.num_connections -= 1;

            if soft_body_a.shape[point_a].0.num_connections == 0 {
                soft_body_a.shape[point_a].0.constraint = None;
            }

            if soft_body_b.shape[point_b].0.num_connections == 0 {
                soft_body_b.shape[point_b].0.constraint = None;
            }

            if point_a < length_a - 1 {
                point_a += 1;
            } else {
                point_a = 0;
            }

            if point_b > 0 {
                point_b -= 1;
            } else {
                point_b = length_b - 1;
            }
        }

        if let Some(source) = source {
            self.connect_attatched_soft_bodies(source);
        }

        Some(())
    }
}
