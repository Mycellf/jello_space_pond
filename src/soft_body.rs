use macroquad::{
    color::{Color, colors},
    math::Vec2,
    shapes,
};

use crate::utils;

/// Points should always be oriented counter clockwise
#[derive(Clone, Debug)]
pub struct SoftBody {
    pub shape: Vec<(Point, Line)>,
    pub internal_springs: Vec<([usize; 2], Spring)>,
    pub bounding_box: BoundingBox,
}

impl SoftBody {
    pub fn draw(&self) {
        if self.shape.len() > 1 {
            let length = self.shape.len();

            for i in 0..self.shape.len() {
                let (point_a, line) = &self.shape[i];
                let (point_b, _) = &self.shape[if i < length - 1 { i + 1 } else { 0 }];

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
        if self.shape.len() > 1 {
            let length = self.shape.len();

            for i in 0..self.shape.len() {
                let [(point_a, line), (point_b, _)] = self
                    .shape
                    .get_disjoint_mut([i, if i < length - 1 { i + 1 } else { 0 }])
                    .unwrap();

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

        shapes::draw_line(
            point_a.position.x,
            point_a.position.y,
            point_b.position.x,
            point_b.position.y,
            0.05,
            color,
        );
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
            colors::BLUE,
        );
    }

    pub fn contains_point(&self, point: Vec2) -> bool {
        point.x >= self.min_corner.x
            && point.y >= self.min_corner.y
            && point.x <= self.min_corner.x + self.size.x
            && point.y <= self.min_corner.y + self.size.y
    }
}
