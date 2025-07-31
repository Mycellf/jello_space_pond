use macroquad::{
    color::{Color, colors},
    math::Vec2,
    shapes,
};

use crate::utils;

/// Points should always be oriented counter clockwise
#[non_exhaustive]
#[derive(Clone, Debug)]
pub struct SoftBody {
    pub shape: Vec<(Point, Line)>,
    pub internal_springs: Vec<([usize; 2], Spring)>,
    pub bounding_box: BoundingBox,
    pub gas_force: f32,
    pub pressure: f32,
}

impl SoftBody {
    pub fn new(
        shape: Vec<(Point, Line)>,
        internal_springs: Vec<([usize; 2], Spring)>,
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

    pub fn draw(&self) {
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
        }
    }

    pub fn apply_impulse_and_velocity(&mut self, dt: f32) {
        self.add_pressure_impulse(dt);

        if self.shape.len() > 1 {
            for i in 0..self.shape.len() {
                let (point_a, line, point_b) = self.get_line_mut(i).unwrap();

                line.spring.apply_force(point_a, point_b, dt);
            }

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
        if self.gas_force <= f32::EPSILON {
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

    pub fn contains_point(&self, point: Vec2) -> bool {
        if self.bounding_box.contains_point(point) {
            // Cast a horizontal line to the right of the point
            let mut num_intersections = 0;

            for i in 0..self.shape.len() {
                let (point_a, _, point_b) = self.get_line(i).unwrap();

                let point_a = point_a.position;
                let point_b = point_b.position;

                if point_a.x < point.x && point_b.x < point.x {
                    // Both points are to the left of line
                    continue;
                }

                if (point_a.y > point.y) == (point_b.y > point.y) {
                    // Both points are above or below
                    continue;
                }

                if point_a.x >= point.x && point_b.x >= point.x {
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
                    scaled_sin_angle <= 0.0
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

        for (point, _) in &mut self.shape {
            if !other.contains_point(point.position) {
                continue;
            }

            let (line_index, closest_point, _, point_interpolation) =
                other.closest_line_to_point(point.position);

            let (point_a, _, point_b) = other.get_line_mut(line_index).unwrap();

            // Will move the points just the right distance so the line intersects the new position
            let interpolation_scale =
                1.0 / (2.0 * point_interpolation.powi(2) - 2.0 * point_interpolation + 1.0);

            let composite_point = Point {
                position: closest_point,
                velocity: point_a.velocity.lerp(point_b.velocity, point_interpolation)
                    * interpolation_scale,
                impulse: Vec2::ZERO,
                mass: point_a.mass + point_b.mass,
            };

            point.position = point.position.lerp(
                composite_point.position,
                point.mass / (point.mass + composite_point.mass),
            );

            let composite_position_nudge = point.position - composite_point.position;

            let collision_direction = (point_a.position - point_b.position)
                .normalize_or_zero()
                .perp();

            let composite_velocity = composite_point
                .velocity
                .project_onto_normalized(collision_direction);

            let point_velocity = point.velocity.project_onto_normalized(collision_direction);

            let weighted_velocity = (point_velocity * point.mass
                + composite_velocity * composite_point.mass)
                / (composite_point.mass + point.mass);

            point.velocity += weighted_velocity - point_velocity;
            let composite_velocity_nudge = weighted_velocity - composite_velocity;

            point_a.velocity +=
                composite_velocity_nudge * (1.0 - point_interpolation) * interpolation_scale;
            point_b.velocity +=
                composite_velocity_nudge * point_interpolation * interpolation_scale;

            point_a.position +=
                composite_position_nudge * (1.0 - point_interpolation) * interpolation_scale;
            point_b.position +=
                composite_position_nudge * point_interpolation * interpolation_scale;

            collided = true;
        }

        collided
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

#[derive(Clone, Copy, Debug, Default)]
pub struct Point {
    pub position: Vec2,
    pub velocity: Vec2,
    pub impulse: Vec2,
    pub mass: f32,
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

#[derive(Clone, Copy, Debug, Default)]
pub struct Line {
    pub spring: Spring,
}

/// If the points are at the exact same position, no force is applied
#[derive(Clone, Copy, Debug)]
pub struct Spring {
    pub target_distance: f32,
    pub force_constant: f32,
    pub damping: f32,
}

impl Spring {
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

        normalized_displacement * (force + damping)
    }
}

impl Default for Spring {
    fn default() -> Self {
        Self {
            target_distance: 1.0,
            force_constant: 50.0,
            damping: 10.0,
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
        point.x >= self.min_corner.x
            && point.y >= self.min_corner.y
            && point.x <= self.max_corner().x
            && point.y <= self.max_corner().y
    }

    pub fn intersects_other(&self, other: &BoundingBox) -> bool {
        other.max_corner().x >= self.min_corner.x
            && other.max_corner().y >= self.min_corner.y
            && other.min_corner.x <= self.max_corner().x
            && other.min_corner.y <= self.max_corner().y
    }
}
