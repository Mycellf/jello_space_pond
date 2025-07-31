use std::sync::{LazyLock, Mutex};

use earcut::Earcut;
use macroquad::{
    color::{Color, colors},
    math::{Vec2, vec2},
    models::{self, Mesh},
    shapes,
    ui::Vertex,
};

use crate::utils;

/// Points should always be oriented counter clockwise
#[non_exhaustive]
#[derive(Clone, Debug)]
pub struct SoftBody {
    pub shape: Vec<(Point, Line)>,
    pub internal_springs: Vec<([usize; 2], LinearSpring)>,
    pub bounding_box: BoundingBox,
    pub gas_force: f32,
    pub pressure: f32,
}

impl SoftBody {
    pub fn new(
        shape: Vec<(Point, Line)>,
        internal_springs: Vec<([usize; 2], LinearSpring)>,
        gas_force: f32,
    ) -> Self {
        Self {
            shape,
            internal_springs,
            bounding_box: BoundingBox::default(),
            gas_force,
            pressure: 0.0,
        }
    }

    /// CREDIT: tirithen <https://github.com/not-fl3/macroquad/issues/174#issuecomment-817203498>
    /// (made to work with convex polygons via earcut)
    pub fn fill_color(&self, color: Color) {
        static EARCUT: LazyLock<Mutex<Earcut<f32>>> = LazyLock::new(|| Mutex::new(Earcut::new()));

        let mut vertices = Vec::with_capacity(self.shape.len() + 2);
        let mut indices = Vec::with_capacity(self.shape.len() * 3);

        for (Point { position, .. }, _) in self.shape.iter() {
            let vertex = Vertex::new(position.x, position.y, 0.0, 0.0, 0.0, color);

            vertices.push(vertex);
        }

        EARCUT.lock().unwrap().earcut(
            self.shape.iter().map(|(point, _)| point.position.into()),
            &[],
            &mut indices,
        );

        let mesh = Mesh {
            vertices,
            indices,
            texture: None,
        };

        models::draw_mesh(&mesh);
    }

    pub fn draw_springs(&self) {
        if self.shape.len() > 1 {
            for i in 0..self.shape.len() {
                let (point_a, line, point_b) = self.get_line(i).unwrap();

                line.spring.draw_line(point_a, point_b);
            }

            for &(indecies, ref spring) in &self.internal_springs {
                let (point_a, _) = &self.shape[indecies[0]];
                let (point_b, _) = &self.shape[indecies[1]];

                spring.draw_line(point_a, point_b);
            }

            for i in 0..self.shape.len() {
                let [point_a, point_b, point_c] = self.get_angle(i).unwrap();

                if let Some(spring) = point_b.spring {
                    spring.draw_circle(point_a, point_b, point_c);
                }
            }
        }
    }

    pub fn apply_impulse_and_velocity(&mut self, dt: f32) {
        self.add_pressure_impulse(dt);

        if self.shape.len() > 1 {
            // Angular Springs
            for i in 0..self.shape.len() {
                let [point_a, point_b, point_c] = self.get_angle_mut(i).unwrap();

                if let Some(spring) = point_b.spring {
                    spring.apply_forces(point_a, point_b, point_c, dt);
                }
            }

            // Linear Springs
            for i in 0..self.shape.len() {
                let (point_a, line, point_b) = self.get_line_mut(i).unwrap();

                line.spring.apply_force(point_a, point_b, dt);
            }

            // Internal Linear Springs
            for &(indecies, ref spring) in &self.internal_springs {
                let [(point_a, _), (point_b, _)] = self.shape.get_disjoint_mut(indecies).unwrap();

                spring.apply_force(point_a, point_b, dt);
            }
        }

        for (point, _) in &mut self.shape {
            point.apply_impulse_and_velocity(dt);
        }

        self.update_bounding_box();
    }

    pub fn add_pressure_impulse(&mut self, dt: f32) {
        if self.gas_force.abs() <= f32::EPSILON {
            self.pressure = 0.0;

            return;
        }

        let pressure = self.gas_force / self.area();

        self.pressure = pressure;

        for i in 0..self.shape.len() {
            let (point_a, _, point_b) = self.get_line_mut(i).unwrap();

            // Magnitude is proportional to the length of the edge
            let force_direction = (point_a.position - point_b.position).perp();

            let pressure_force = force_direction * pressure;

            point_a.impulse += pressure_force * dt / 2.0;
            point_b.impulse += pressure_force * dt / 2.0;
        }
    }

    pub fn update_bounding_box(&mut self) {
        if self.shape.is_empty() {
            return;
        }

        let mut min = self.shape.first().unwrap().0.position;
        let mut max = min;

        for (Point { position, .. }, _) in self.shape.iter().skip(1) {
            if position.x < min.x {
                min.x = position.x;
            }

            if position.y < min.y {
                min.y = position.y;
            }

            if position.x > max.x {
                max.x = position.x;
            }

            if position.y > max.y {
                max.y = position.y;
            }
        }

        let size = max - min;

        self.bounding_box = BoundingBox {
            min_corner: min,
            size,
        };
    }

    pub fn get_line(&self, i: usize) -> Option<(&Point, &Line, &Point)> {
        let (point_a, line) = self.shape.get(i)?;
        let (point_b, _) = &self.shape[if i < self.shape.len() - 1 { i + 1 } else { 0 }];

        Some((point_a, line, point_b))
    }

    pub fn get_line_mut(&mut self, i: usize) -> Option<(&mut Point, &mut Line, &mut Point)> {
        let length = self.shape.len();

        if i >= length {
            return None;
        }

        let [(point_a, line), (point_b, _)] = self
            .shape
            .get_disjoint_mut([i, if i < length - 1 { i + 1 } else { 0 }])
            .unwrap();

        Some((point_a, line, point_b))
    }

    pub fn get_angle(&self, i: usize) -> Option<[&Point; 3]> {
        let (point_b, _) = self.shape.get(i)?;
        let (point_c, _) = &self.shape[if i < self.shape.len() - 1 { i + 1 } else { 0 }];
        let (point_a, _) = &self.shape[if i > 0 { i - 1 } else { self.shape.len() - 1 }];

        Some([point_a, point_b, point_c])
    }

    pub fn get_angle_mut(&mut self, i: usize) -> Option<[&mut Point; 3]> {
        let length = self.shape.len();

        if i >= length {
            return None;
        }

        let [(point_a, _), (point_b, _), (point_c, _)] = self
            .shape
            .get_disjoint_mut([
                if i > 0 { i - 1 } else { length - 1 },
                i,
                if i < length - 1 { i + 1 } else { 0 },
            ])
            .unwrap();

        Some([point_a, point_b, point_c])
    }

    pub fn get_adjacent_lines_to_point(&self, i: usize) -> Option<[&Line; 2]> {
        let (_, line_b) = self.shape.get(i)?;
        let (_, line_a) = &self.shape[if i > 0 { i - 1 } else { self.shape.len() - 1 }];

        Some([line_a, line_b])
    }

    pub fn get_friction_of_point(&self, i: usize) -> Option<f32> {
        let [line_a, line_b] = self.get_adjacent_lines_to_point(i)?;

        Some((line_a.friction + line_b.friction) / 2.0)
    }

    pub fn contains_point(&self, point: Vec2) -> bool {
        if self.bounding_box.contains_point(point) {
            // Cast a horizontal line to the right of the point
            let mut num_intersections = 0;

            for i in 0..self.shape.len() {
                let (point_a, _, point_b) = self.get_line(i).unwrap();

                let point_a = point_a.position;
                let point_b = point_b.position;

                if point_a.x <= point.x && point_b.x <= point.x {
                    // Both points are to the left of line
                    continue;
                }

                if point_a.y >= point.y && point_b.y >= point.y
                    || point_a.y <= point.y && point_b.y <= point.y
                {
                    // Both points are above or below
                    continue;
                }

                if point_a.x > point.x && point_b.x > point.x {
                    // Both points are to the right of the line
                    let max_y = point_a.y.max(point_b.y);
                    let min_y = point_a.y.min(point_b.y);

                    if max_y > point.y && min_y <= point.y {
                        // The lines intersect as one point is above, another is below
                        num_intersections += 1;
                        continue;
                    }
                }

                // One is to the left, one is to the right
                let (left_point, right_point) = if point_a.x < point_b.x {
                    (point_a, point_b)
                } else {
                    (point_b, point_a)
                };

                let scaled_sin_angle = (right_point - left_point).perp_dot(point - left_point);

                let intersection = if left_point.y > right_point.y {
                    scaled_sin_angle < 0.0
                } else {
                    scaled_sin_angle > 0.0
                };

                if intersection {
                    num_intersections += 1;
                }
            }

            num_intersections % 2 == 1
        } else {
            false
        }
    }

    /// Returns `(line index, closest point, distance squared, progress between points)`
    pub fn closest_line_to_point(&self, point: Vec2) -> (usize, Vec2, f32, f32) {
        let mut closest_line = 0;
        let (mut closest_point, mut closest_progress) = self.closest_point_on_line(0, point);
        let mut closest_distance_squared = closest_point.distance_squared(point);

        for i in 1..self.shape.len() {
            let (closest_point_on_line, progress) = self.closest_point_on_line(i, point);
            let distance_squared = closest_point_on_line.distance_squared(point);

            if distance_squared < closest_distance_squared {
                closest_line = i;
                closest_point = closest_point_on_line;
                closest_progress = progress;
                closest_distance_squared = distance_squared;
            }
        }

        (
            closest_line,
            closest_point,
            closest_distance_squared,
            closest_progress,
        )
    }

    pub fn closest_point_on_line(&self, line: usize, point: Vec2) -> (Vec2, f32) {
        let (start, _, end) = self.get_line(line).unwrap();

        utils::closest_point_on_line(start.position, end.position, point)
    }

    pub fn check_points_against_other_one_sided(&mut self, other: &mut SoftBody) -> bool {
        let mut collided = false;

        for i in 0..self.shape.len() {
            let point_friction = self.get_friction_of_point(i).unwrap();
            let point = &mut self.shape[i].0;

            if !other.contains_point(point.position) {
                continue;
            }

            let (line, closest_point, _, interpolation) =
                other.closest_line_to_point(point.position);

            other.check_other_point_against_line(
                point,
                point_friction,
                line,
                closest_point,
                interpolation,
            );

            if interpolation <= f32::EPSILON {
                // Wedged into corner
                other.check_other_point_against_line(
                    point,
                    point_friction,
                    if line == 0 {
                        other.shape.len() - 1
                    } else {
                        line - 1
                    },
                    closest_point,
                    1.0,
                )
            } else if interpolation >= 1.0 - f32::EPSILON {
                // Wedged into corner
                other.check_other_point_against_line(
                    point,
                    point_friction,
                    if line >= other.shape.len() - 1 {
                        0
                    } else {
                        line + 1
                    },
                    closest_point,
                    0.0,
                )
            }

            collided = true;
        }

        collided
    }

    pub fn check_other_point_against_line(
        &mut self,
        point: &mut Point,
        point_friction: f32,
        line: usize,
        closest_point: Vec2,
        interpolation: f32,
    ) {
        let (point_a, Line { friction, .. }, point_b) = self.get_line_mut(line).unwrap();

        Self::check_point_against_line(
            point_a,
            point_b,
            point,
            utils::combine_friction(point_friction, *friction),
            closest_point,
            interpolation,
        );
    }

    pub fn check_own_point_against_line(
        &mut self,
        point: usize,
        line: usize,
        closest_point: Vec2,
        interpolation: f32,
    ) {
        let point_friction = self.get_friction_of_point(point).unwrap();

        let length = self.shape.len();

        let [(point, _), (point_a, Line { friction, .. }), (point_b, _)] = self
            .shape
            .get_disjoint_mut([point, line, if line < length - 1 { line + 1 } else { 0 }])
            .unwrap();

        Self::check_point_against_line(
            point_a,
            point_b,
            point,
            utils::combine_friction(point_friction, *friction),
            closest_point,
            interpolation,
        );
    }

    pub fn check_point_against_line(
        point_a: &mut Point,
        point_b: &mut Point,
        point: &mut Point,
        friction: f32,
        closest_point: Vec2,
        interpolation: f32,
    ) {
        // Will move the points just the right distance so the line intersects the new position
        let interpolation_scale = 1.0 / (2.0 * interpolation.powi(2) - 2.0 * interpolation + 1.0);

        let composite_velocity = point_a.velocity.lerp(point_b.velocity, interpolation);
        let composite_mass =
            utils::lerp(point_a.mass, point_b.mass, interpolation) * interpolation_scale;

        point.position = point
            .position
            .lerp(closest_point, point.mass / (point.mass + composite_mass));

        let composite_position_nudge = point.position - closest_point;

        let tangent = (point_a.position - point_b.position).normalize_or_zero();
        let normal = tangent.perp();

        let composite_tangent_velocity = composite_velocity.project_onto_normalized(tangent);
        let composite_normal_velocity = composite_velocity.project_onto_normalized(normal);

        let point_tangent_velocity = point.velocity.project_onto_normalized(tangent);
        let point_normal_velocity = point.velocity.project_onto_normalized(normal);

        let friction_velocity_nudge = if friction.abs() <= f32::EPSILON {
            Vec2::ZERO
        } else {
            let relative_tangent_velocity = point_tangent_velocity - composite_tangent_velocity;
            let relative_normal_velocity = point_normal_velocity - composite_normal_velocity;

            let friction_velocity_nudge = -relative_tangent_velocity.normalize_or_zero()
                * relative_normal_velocity.length()
                * friction;

            if relative_tangent_velocity.dot(relative_tangent_velocity + friction_velocity_nudge)
                < 0.0
            {
                Vec2::ZERO
            } else {
                friction_velocity_nudge
            }
        };

        let weighted_normal_velocity = (point_normal_velocity * point.mass
            + composite_normal_velocity * composite_mass)
            / (point.mass + composite_mass);

        point.velocity +=
            weighted_normal_velocity - point_normal_velocity + friction_velocity_nudge / 2.0;
        let composite_velocity_nudge =
            weighted_normal_velocity - composite_normal_velocity - friction_velocity_nudge / 2.0;

        point_a.velocity += composite_velocity_nudge * (1.0 - interpolation) * interpolation_scale;
        point_b.velocity += composite_velocity_nudge * interpolation * interpolation_scale;

        point_a.position += composite_position_nudge * (1.0 - interpolation) * interpolation_scale;
        point_b.position += composite_position_nudge * interpolation * interpolation_scale;
    }

    /// CREDIT: chmike: <https://stackoverflow.com/a/717367>
    pub fn area(&self) -> f32 {
        let mut double_area = 0.0;

        for i in 1..self.shape.len() - 1 {
            double_area += self.shape[i].0.position.x
                * (self.shape[i + 1].0.position.y - self.shape[i - 1].0.position.y);
        }

        // i == self.shape.len() - 1
        double_area += self.shape[self.shape.len() - 1].0.position.x
            * (self.shape[0].0.position.y - self.shape[self.shape.len() - 2].0.position.y);

        // i == self.shape.len()
        double_area += self.shape[0].0.position.x
            * (self.shape[1].0.position.y - self.shape[self.shape.len() - 1].0.position.y);

        double_area / 2.0
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Point {
    pub position: Vec2,
    pub velocity: Vec2,
    pub impulse: Vec2,
    pub mass: f32,
    pub spring: Option<AngularSpring>,
}

impl Point {
    pub fn apply_impulse_and_velocity(&mut self, dt: f32) {
        self.position += self.velocity / 2.0 * dt;

        self.velocity += self.impulse / self.mass;
        self.impulse = Vec2::ZERO;

        self.position += self.velocity / 2.0 * dt;
    }

    pub fn momentum(&self) -> Vec2 {
        self.velocity * self.mass
    }
}

impl Default for Point {
    fn default() -> Self {
        Self {
            position: Vec2::ZERO,
            velocity: Vec2::ZERO,
            impulse: Vec2::ZERO,
            mass: 1.0,
            spring: Some(AngularSpring::default()),
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Line {
    pub spring: LinearSpring,
    pub friction: f32,
}

impl Default for Line {
    fn default() -> Self {
        Self {
            spring: LinearSpring::default(),
            friction: 0.25,
        }
    }
}

/// If the points are at the exact same position, no force is applied
#[derive(Clone, Copy, Debug)]
pub struct LinearSpring {
    pub target_distance: f32,
    pub force_constant: f32,
    pub damping: f32,
    pub compression: bool,
    pub tension: bool,
}

impl LinearSpring {
    pub fn draw_line(&self, point_a: &Point, point_b: &Point) {
        const WEAK_COLOR: Color = colors::GREEN;
        const STRONG_COLOR: Color = colors::RED;

        let force = self.get_force(point_a, point_b);

        let color = utils::color_lerp(
            WEAK_COLOR,
            STRONG_COLOR,
            (force.length() / 2.0).clamp(0.0, 1.0),
        );

        utils::draw_line(point_a.position, point_b.position, 0.05, color);
    }

    pub fn apply_force(&self, point_a: &mut Point, point_b: &mut Point, dt: f32) {
        let impulse = self.get_force(point_a, point_b);

        point_a.impulse += impulse / 2.0 * dt;
        point_b.impulse -= impulse / 2.0 * dt;
    }

    pub fn get_force(&self, point_a: &Point, point_b: &Point) -> Vec2 {
        let displacement = point_a.position - point_b.position;
        let distance = displacement.length();

        if distance <= f32::EPSILON {
            return Vec2::ZERO;
        }

        let normalized_displacement = displacement / distance;

        let relative_velocity = point_a.velocity - point_b.velocity;
        let normal_velocity = relative_velocity.dot(normalized_displacement);

        let force = self.force_constant * (self.target_distance - distance);
        let damping = -normal_velocity * self.damping;

        let mut total_force = force + damping;

        if !self.compression && total_force > 0.0 || !self.tension && total_force < 0.0 {
            total_force = 0.0;
        }

        normalized_displacement * total_force
    }
}

impl Default for LinearSpring {
    fn default() -> Self {
        Self {
            target_distance: 1.0,
            force_constant: 50.0,
            damping: 10.0,
            compression: true,
            tension: true,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct AngularSpring {
    pub target_angle: f32,
    pub force_constant: f32,
    pub damping: f32,
    pub inwards: bool,
    pub outwards: bool,
}

impl AngularSpring {
    pub fn draw_circle(&self, point_a: &Point, point_b: &Point, point_c: &Point) {
        const WEAK_COLOR: Color = colors::GREEN;
        const STRONG_COLOR: Color = colors::RED;

        let forces = self.get_forces(point_a, point_b, point_c);

        let color = utils::color_lerp(
            WEAK_COLOR,
            STRONG_COLOR,
            (forces.iter().copied().map(Vec2::length).sum::<f32>() / 6.0).clamp(0.0, 1.0),
        );

        shapes::draw_circle(point_b.position.x, point_b.position.y, 0.1, color);
    }

    pub fn apply_forces(
        &self,
        point_a: &mut Point,
        point_b: &mut Point,
        point_c: &mut Point,
        dt: f32,
    ) {
        let [impulse_a, impulse_b, impulse_c] = self.get_forces(point_a, point_b, point_c);

        point_a.impulse += impulse_a * dt;
        point_b.impulse += impulse_b * dt;
        point_c.impulse += impulse_c * dt;
    }

    pub fn get_forces(&self, point_a: &Point, point_b: &Point, point_c: &Point) -> [Vec2; 3] {
        let base_direction = point_b.position - point_a.position;
        let measure_direction = point_c.position - point_b.position;

        if base_direction == Vec2::ZERO || measure_direction == Vec2::ZERO {
            return [Vec2::ZERO; 3];
        }

        let angle = base_direction.angle_between(measure_direction);

        let point_a_normal = base_direction.normalize_or_zero().perp();
        let point_c_normal = measure_direction.normalize_or_zero().perp();

        let angular_velocity_a =
            (point_a.velocity - point_b.velocity).dot(point_a_normal) / base_direction.length();
        let angular_velocity_c =
            (point_c.velocity - point_b.velocity).dot(point_c_normal) / measure_direction.length();

        let relative_angular_velocity = angular_velocity_c + angular_velocity_a;

        let force = self.force_constant * (self.target_angle - angle);
        let damping = -relative_angular_velocity * self.damping;

        let mut total_force = force + damping;

        if !self.inwards && total_force > 0.0 || !self.outwards && total_force < 0.0 {
            total_force = 0.0;
        }

        total_force = total_force.clamp(-self.force_constant * 10.0, self.force_constant * 10.0);

        let point_a_force = point_a_normal * total_force / base_direction.length();
        let point_c_force = point_c_normal * total_force / measure_direction.length();

        [
            point_a_force,
            -(point_a_force + point_c_force),
            point_c_force,
        ]
    }
}

impl Default for AngularSpring {
    fn default() -> Self {
        Self {
            target_angle: 0.0,
            force_constant: 1.0,
            damping: 1.0,
            inwards: true,
            outwards: true,
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct BoundingBox {
    pub min_corner: Vec2,
    pub size: Vec2,
}

impl BoundingBox {
    pub fn draw(&self) {
        shapes::draw_rectangle_lines(
            self.min_corner.x - 0.05,
            self.min_corner.y - 0.05,
            self.size.x + 0.1,
            self.size.y + 0.1,
            0.1,
            colors::DARKBLUE,
        );
    }

    pub fn max_corner(&self) -> Vec2 {
        self.min_corner + self.size
    }

    pub fn contains_point(&self, point: Vec2) -> bool {
        point.x > self.min_corner.x
            && point.y > self.min_corner.y
            && point.x < self.max_corner().x
            && point.y < self.max_corner().y
    }

    pub fn intersects_other(&self, other: &BoundingBox) -> bool {
        other.max_corner().x > self.min_corner.x
            && other.max_corner().y > self.min_corner.y
            && other.min_corner.x < self.max_corner().x
            && other.min_corner.y < self.max_corner().y
    }
}

#[derive(Clone, Debug)]
pub struct SoftBodyBuilder {
    pub soft_body: SoftBody,
    pub internal_springs: Vec<InternalSpringBuilder>,

    pub base_point: Point,
    pub base_line: Line,

    pub subdivisions: usize,

    pub last_spring_specified: bool,
    pub spring_scale: f32,
}

impl Default for SoftBodyBuilder {
    fn default() -> Self {
        Self {
            soft_body: SoftBody {
                shape: Vec::new(),
                internal_springs: Vec::new(),
                bounding_box: BoundingBox::default(),
                gas_force: 0.0,
                pressure: 0.0,
            },
            internal_springs: Vec::new(),

            base_point: Point::default(),
            base_line: Line::default(),

            subdivisions: 0,

            last_spring_specified: false,
            spring_scale: 1.0,
        }
    }
}

impl SoftBodyBuilder {
    pub fn build(mut self) -> SoftBody {
        assert!(self.soft_body.shape.len() >= 3, "Not enough points");

        let first_position = self.soft_body.shape.first().unwrap().0.position;
        self.add_subdivisions(first_position);
        self.fix_last_spring(first_position);

        for (id, internal_spring) in self.internal_springs.into_iter().enumerate() {
            match internal_spring {
                InternalSpringBuilder::Incomplete(_) => panic!("Spring {id} is incomplete"),
                InternalSpringBuilder::Unused | InternalSpringBuilder::Complete => (),
            }
        }

        for i in 0..self.soft_body.shape.len() {
            let [point_a, point_b, point_c] = self.soft_body.get_angle_mut(i).unwrap();

            if let Some(spring) = &mut point_b.spring {
                let angle = (point_b.position - point_a.position)
                    .angle_between(point_c.position - point_b.position);

                spring.target_angle = angle;
            }
        }

        self.soft_body
    }

    pub fn point(self, x: f32, y: f32) -> Self {
        self.point_ex(vec2(x, y))
    }

    pub fn point_ex(mut self, point: Vec2) -> Self {
        self.add_subdivisions(point + self.base_point.position);

        self.point_inner(point + self.base_point.position);
        self
    }

    fn point_inner(&mut self, point: Vec2) {
        self.fix_last_spring(point);

        self.soft_body.shape.push((
            Point {
                position: point,
                ..self.base_point
            },
            self.base_line,
        ));
        self.last_spring_specified = false;
    }

    fn add_subdivisions(&mut self, point: Vec2) {
        if self.subdivisions > 0 {
            if let Some(&(Point { position, .. }, _)) = self.soft_body.shape.last() {
                let segments = self.subdivisions + 1;

                for i in 1..segments {
                    self.point_inner(position.lerp(point, i as f32 / segments as f32));
                }
            }
        }
    }

    fn fix_last_spring(&mut self, point: Vec2) {
        if !self.last_spring_specified {
            if let Some(&mut (Point { position, .. }, Line { ref mut spring, .. })) =
                self.soft_body.shape.last_mut()
            {
                spring.target_distance = position.distance(point) * self.spring_scale;
            }
        }
    }

    pub fn with_spring(mut self, spring: LinearSpring) -> Self {
        self.soft_body.shape.last_mut().unwrap().1.spring = spring;
        self.last_spring_specified = true;
        self
    }

    pub fn with_spring_length(mut self, length: f32) -> Self {
        self.soft_body
            .shape
            .last_mut()
            .unwrap()
            .1
            .spring
            .target_distance = length;
        self.last_spring_specified = true;
        self
    }

    pub fn base_spring(mut self, spring: LinearSpring) -> Self {
        self.base_line.spring = spring;
        self
    }

    pub fn with_internal_spring_start(mut self, id: usize) -> Self {
        while self.internal_springs.len() <= id {
            self.internal_springs.push(InternalSpringBuilder::Unused);
        }

        match self.internal_springs[id] {
            InternalSpringBuilder::Unused => {
                self.internal_springs[id] =
                    InternalSpringBuilder::Incomplete(self.soft_body.shape.len() - 1);
            }
            InternalSpringBuilder::Incomplete(_) | InternalSpringBuilder::Complete => {
                panic!("The spring {id} already exists");
            }
        }

        self
    }

    pub fn with_internal_spring_end(mut self, id: usize, spring: LinearSpring) -> Self {
        match self.internal_springs.get(id) {
            Some(&InternalSpringBuilder::Incomplete(start_index)) => {
                self.internal_springs[id] = InternalSpringBuilder::Complete;
                let end_index = self.soft_body.shape.len() - 1;

                self.soft_body
                    .internal_springs
                    .push(([start_index, end_index], spring));
            }
            Some(InternalSpringBuilder::Complete) => {
                panic!("The spring {id} is already finished");
            }
            Some(InternalSpringBuilder::Unused) | None => {
                panic!("The spring {id} does not exist");
            }
        }

        self
    }

    pub fn base_point(mut self, point: Point) -> Self {
        self.base_point = point;
        self
    }

    pub fn base_angular_spring(mut self, spring: Option<AngularSpring>) -> Self {
        self.base_point.spring = spring;
        self
    }

    pub fn mass(mut self, mass: f32) -> Self {
        self.base_point.mass = mass;
        self
    }

    pub fn velocity(self, x: f32, y: f32) -> Self {
        self.velocity_ex(vec2(x, y))
    }

    pub fn velocity_ex(mut self, velocity: Vec2) -> Self {
        self.base_point.velocity = velocity;
        self
    }

    pub fn offset(self, x: f32, y: f32) -> Self {
        self.offset_ex(vec2(x, y))
    }

    pub fn offset_ex(mut self, offset: Vec2) -> Self {
        self.base_point.position = offset;
        self
    }

    pub fn subdivisions(mut self, subdivisions: usize) -> Self {
        self.subdivisions = subdivisions;
        self
    }

    pub fn gas_force(mut self, gas_force: f32) -> Self {
        self.soft_body.gas_force = gas_force;
        self
    }

    pub fn spring_scale(mut self, spring_scale: f32) -> Self {
        self.spring_scale = spring_scale;
        self
    }

    pub fn friction(mut self, friction: f32) -> Self {
        self.base_line.friction = friction;
        self
    }
}

#[derive(Clone, Copy, Debug)]
pub enum InternalSpringBuilder {
    Unused,
    Incomplete(usize),
    Complete,
}
