use macroquad::{
    camera::Camera2D,
    color::colors,
    input::{self, MouseButton},
};
use slotmap::{HopSlotMap, new_key_type};

use crate::{
    constraint::{Constraint, PointHandle},
    soft_body::{AttatchmentPointHandle, LinearSpring, Point, SoftBody},
    utils,
};

#[derive(Clone, Debug)]
pub struct Simulation {
    pub soft_bodies: HopSlotMap<SoftBodyKey, SoftBody>,
    pub keys: Vec<SoftBodyKey>,

    pub constraints: HopSlotMap<ConstraintKey, Constraint>,

    pub input_state: InputState,
}

new_key_type! {
    pub struct SoftBodyKey;
    pub struct ConstraintKey;
}

#[derive(Clone, Debug, Default)]
pub struct InputState {
    pub selected_attatchment_point: Option<(AttatchmentPointHandle, f32)>,
    pub target_attatchment_point: Option<AttatchmentPointHandle>,
    pub can_connect: bool,

    pub grabbing: bool,
    pub clicking: bool,

    pub mouse: Point,
}

impl Simulation {
    pub fn new() -> Self {
        Self {
            soft_bodies: HopSlotMap::default(),
            keys: Vec::new(),

            constraints: HopSlotMap::default(),

            input_state: InputState::default(),
        }
    }

    pub fn draw(&self, debug: bool) {
        for (_, soft_body) in &self.soft_bodies {
            soft_body.draw();
        }

        if debug {
            for (_, soft_body) in &self.soft_bodies {
                soft_body.draw_springs();
            }
        }

        for (_, soft_body) in &self.soft_bodies {
            soft_body.draw_attatchment_points();
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
    }

    pub fn update_keys(&mut self) {
        self.keys = self.soft_bodies.keys().collect();
    }

    pub fn tick_simulation(&mut self, dt: f32) {
        self.update_grabbing(dt);

        for (_, soft_body) in &mut self.soft_bodies {
            soft_body.apply_impulse_and_velocity(dt);
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
    }

    pub fn update_input(&mut self, camera: &Camera2D, dt: f32) {
        const SELECTION_RANGE: f32 = 0.25;

        let mouse_position = utils::mouse_position(camera);

        self.input_state.mouse.velocity = (mouse_position - self.input_state.mouse.position) * dt;
        self.input_state.mouse.position = mouse_position;
        self.input_state.mouse.mass = 10000.0;

        self.input_state.clicking |= input::is_mouse_button_pressed(MouseButton::Left);
        self.input_state.grabbing =
            self.input_state.clicking || input::is_mouse_button_down(MouseButton::Left);

        let mut selected_attatchment_point = None;
        let mut selected_distance_squared = f32::INFINITY;

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

                let mut minimum_distance_squared = closest_point.distance_squared(mouse_position);

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

        self.input_state.can_connect = false;
        if let Some(target) = self.input_state.target_attatchment_point {
            if let Some((selected, _)) = self.input_state.selected_attatchment_point {
                self.input_state.can_connect = self
                    .are_attatchment_points_within_range([selected, target], 1.0)
                    .unwrap();
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
                        .are_attatchment_points_within_range([selected, target], 1.0)
                        .unwrap()
                    {
                        self.connect_attatchment_points([selected, target]).unwrap();
                    }
                }

                self.input_state.target_attatchment_point = None;
            }

            self.input_state.selected_attatchment_point = selected_attatchment_point;
        }
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
        const PULL_SPRING: LinearSpring = LinearSpring {
            target_distance: 0.0,
            force_constant: 10.0,
            damping: 20.0,
            compression: true,
            tension: true,
            maximum_force: 0.5,
        };

        let [soft_body_a, soft_body_b] = self
            .soft_bodies
            .get_disjoint_mut([handle_a.soft_body, handle_b.soft_body])
            .unwrap();

        let length_a = soft_body_a.shape.len();
        let length_b = soft_body_b.shape.len();

        let attatchment_point_a = soft_body_a.attatchment_points[handle_a.index];
        let attatchment_point_b = soft_body_b.attatchment_points[handle_b.index];

        let mut point_a = attatchment_point_a.start_point;
        let mut point_b =
            (attatchment_point_b.start_point + attatchment_point_b.length - 1) % length_b;

        for _ in 0..attatchment_point_a.length {
            {
                let (point_a, _) = &mut soft_body_a.shape[point_a];
                let (point_b, _) = &mut soft_body_b.shape[point_b];

                let spring = LinearSpring {
                    maximum_force: PULL_SPRING.maximum_force
                        / point_a.position.distance(point_b.position).max(0.25),
                    ..PULL_SPRING
                };

                let impulse = spring.get_force(point_a, point_b);

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
        const GRAB_SPRING: LinearSpring = LinearSpring {
            target_distance: 0.0,
            force_constant: 10.0,
            damping: 20.0,
            compression: true,
            tension: true,
            maximum_force: 0.5,
        };

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

        let spring = LinearSpring {
            maximum_force: GRAB_SPRING.maximum_force
                / (self.input_state.mouse.position)
                    .distance(composite_point.position)
                    .max(1.0),
            ..GRAB_SPRING
        };

        spring.apply_force(
            &mut self.input_state.mouse.clone(),
            &mut composite_point,
            dt,
        );

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

        Some(())
    }
}
