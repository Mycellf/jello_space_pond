use macroquad::{
    color::Color,
    math::{FloatExt, Vec2, vec2},
    shapes::{self, DrawRectangleParams},
};

use crate::utils;

#[derive(Clone, Copy, Debug)]
pub struct Particle {
    pub position: Vec2,
    pub shape: Shape,

    pub age: f32,
    pub end_age: f32,

    pub start_velocity: Vec2,
    pub end_velocity: Vec2,

    pub start_color: Color,
    pub end_color: Color,

    pub start_rotation: f32,
    pub end_rotation: f32,

    pub start_size: f32,
    pub end_size: f32,
}

impl Particle {
    pub fn draw(&self) {
        let t = self.progress();

        self.shape.draw(
            self.position,
            self.start_rotation.lerp(self.end_rotation, t),
            self.start_size.lerp(self.end_size, t),
            utils::color_lerp(self.start_color, self.end_color, t),
        );
    }

    pub fn tick(&mut self, dt: f32) {
        self.position += self.start_velocity.lerp(self.end_velocity, self.progress()) / 2.0 * dt;

        self.age += dt;

        self.position += self.start_velocity.lerp(self.end_velocity, self.progress()) / 2.0 * dt;
    }

    pub fn progress(&self) -> f32 {
        self.age / self.end_age
    }
}

#[derive(Clone, Copy, Debug)]
pub enum Shape {
    Circle,
    Rectangle { aspect: f32 },
}

impl Shape {
    pub fn draw(&self, position: Vec2, rotation: f32, size: f32, color: Color) {
        match self {
            Shape::Circle => shapes::draw_circle(position.x, position.y, size / 2.0, color),
            Shape::Rectangle { aspect } => shapes::draw_rectangle_ex(
                position.x,
                position.y,
                size * *aspect,
                size,
                DrawRectangleParams {
                    offset: vec2(0.5, 0.5),
                    rotation,
                    color,
                },
            ),
        }
    }
}
