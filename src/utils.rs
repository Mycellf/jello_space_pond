use macroquad::{
    camera::Camera2D,
    color::Color,
    input,
    math::{Vec2, vec2},
    shapes, window,
};
use ndarray::{Array2, Dimension};

use crate::soft_body::BoundingBox;

#[must_use]
pub fn exp_decay_cutoff(a: f32, b: f32, decay: f32, dt: f32, cutoff: f32) -> (f32, bool) {
    if (a - b).abs() < cutoff {
        (b, true)
    } else {
        (exp_decay(a, b, decay, dt), false)
    }
}

/// CREDIT: Freya Holmér: <https://www.youtube.com/watch?v=LSNQuFEDOyQ>
#[must_use]
pub fn exp_decay(a: f32, b: f32, decay: f32, dt: f32) -> f32 {
    b + (a - b) * (-decay * dt).exp()
}

#[must_use]
pub const fn lerp(a: f32, b: f32, t: f32) -> f32 {
    a + (b - a) * t
}

#[must_use]
pub const fn color_lerp(a: Color, b: Color, t: f32) -> Color {
    Color {
        r: lerp(a.r, b.r, t),
        g: lerp(a.g, b.g, t),
        b: lerp(a.b, b.b, t),
        a: lerp(a.a, b.a, t),
    }
}

pub fn update_camera_aspect_ratio(camera: &mut Camera2D) {
    camera.zoom.x = camera.zoom.y.abs() * window::screen_height() / window::screen_width();
}

pub fn mouse_position(camera: &Camera2D) -> Vec2 {
    camera.screen_to_world(input::mouse_position().into())
}

pub fn interpolation_scale(interpolation: f32) -> f32 {
    1.0 / (2.0 * interpolation.powi(2) - 2.0 * interpolation + 1.0)
}

pub fn draw_line(start: Vec2, end: Vec2, thickness: f32, color: Color) {
    shapes::draw_line(start.x, start.y, end.x, end.y, thickness, color);
}

pub fn combine_friction(a: f32, b: f32) -> f32 {
    a * b
}

pub fn combine_friction_to_point(a: f32, b: f32) -> f32 {
    a.max(b)
}

pub fn generate_color_for_spring(force: f32, damping: f32) -> Color {
    fn scale(force: f32) -> f32 {
        (force / 2.0).clamp(0.0, 1.0)
    }

    let force = force.abs();
    let damping = damping.abs();

    Color {
        r: scale(force),
        g: 1.0 - scale(force.max(damping)),
        b: scale(damping),
        a: 1.0,
    }
}

pub fn clamp_sign(value: f32, allow_positive: bool, allow_negative: bool) -> f32 {
    if !allow_positive && value > 0.0 || !allow_negative && value < 0.0 {
        0.0
    } else {
        value
    }
}

/// CREDIT: Grumdrig: <https://stackoverflow.com/a/1501725>
pub fn closest_point_on_line(start: Vec2, end: Vec2, point: Vec2) -> (Vec2, f32) {
    let length_squared = (start - end).length_squared();

    if length_squared == 0.0 {
        (start, 0.0)
    } else {
        let line_progress = (point - start).dot(end - start) / length_squared;
        let line_progress = line_progress.clamp(0.0, 1.0);

        (start.lerp(end, line_progress), line_progress)
    }
}

pub fn are_line_segments_intersecting([a1, b1]: [Vec2; 2], [a2, b2]: [Vec2; 2]) -> bool {
    line_segment_intersection([a1, b1], [a2, b2]).is_some()
}

pub fn intersection_point_of_line_segments(
    [a1, b1]: [Vec2; 2],
    [a2, b2]: [Vec2; 2],
) -> Option<(Vec2, [f32; 2])> {
    let ([t_times_divisor, u_times_divisor], divisor) =
        line_segment_intersection([a1, b1], [a2, b2])?;

    let t = t_times_divisor / divisor;
    let u = u_times_divisor / divisor;

    Some((a1 + t * (b1 - a1), [t, u]))
}

/// CREDIT: Wikipedia: <https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection>
///
/// If the lines intersect, returns `([progress along line segment times coeficient; 2], coeficient)`
pub fn line_segment_intersection(
    [a1, b1]: [Vec2; 2],
    [a2, b2]: [Vec2; 2],
) -> Option<([f32; 2], f32)> {
    let first_bounding_box = BoundingBox::fit_points(a1, b1);
    let second_bounding_box = BoundingBox::fit_points(a2, b2);

    if !first_bounding_box.intersects_other(&second_bounding_box) {
        return None;
    }

    let divisor = vec2(a1.x - b1.x, a2.x - b2.x).perp_dot(vec2(a1.y - b1.y, a2.y - b2.y));

    let t_times_divisor =
        vec2(a1.x - a2.x, a2.x - b2.x).perp_dot(vec2(a1.y - a2.y, a2.y - b2.y)) * divisor.signum();
    let u_times_divisor =
        -vec2(a1.x - b1.x, a1.x - a2.x).perp_dot(vec2(a1.y - b1.y, a1.y - a2.y)) * divisor.signum();

    let divisor = divisor.abs();

    if divisor <= f32::EPSILON {
        return None;
    }

    (t_times_divisor >= 0.0
        && t_times_divisor <= divisor
        && u_times_divisor >= 0.0
        && u_times_divisor <= divisor)
        .then_some(([t_times_divisor, u_times_divisor], divisor))
}

pub trait RotateCounterClockwise {
    fn rotate_counter_clockwise(&self) -> Self;
}

impl<T: RotateCounterClockwise> RotateCounterClockwise for Array2<T> {
    fn rotate_counter_clockwise(&self) -> Self {
        let (height, width) = self.raw_dim().into_pattern();

        Array2::from_shape_fn([width, height], |(x, y)| {
            self[[width - y - 1, x]].rotate_counter_clockwise()
        })
    }
}

impl<T: RotateCounterClockwise> RotateCounterClockwise for Option<T> {
    fn rotate_counter_clockwise(&self) -> Self {
        if let Some(inner) = self {
            Some(inner.rotate_counter_clockwise())
        } else {
            None
        }
    }
}

pub trait RotateClockwise {
    fn rotate_clockwise(&self) -> Self;
}

impl<T: RotateClockwise> RotateClockwise for Array2<T> {
    fn rotate_clockwise(&self) -> Self {
        let (height, width) = self.raw_dim().into_pattern();

        Array2::from_shape_fn([width, height], |(x, y)| {
            self[[y, width - x - 1]].rotate_clockwise()
        })
    }
}

impl<T: RotateClockwise> RotateClockwise for Option<T> {
    fn rotate_clockwise(&self) -> Self {
        if let Some(inner) = self {
            Some(inner.rotate_clockwise())
        } else {
            None
        }
    }
}
