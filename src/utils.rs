use macroquad::{
    camera::Camera2D,
    color::Color,
    input,
    math::{Vec2, vec2},
    shapes, window,
};

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

pub fn draw_line(start: Vec2, end: Vec2, thickness: f32, color: Color) {
    shapes::draw_line(start.x, start.y, end.x, end.y, thickness, color);
}

pub fn combine_friction(a: f32, b: f32) -> f32 {
    a * b
}

pub fn combine_friction_to_point(a: f32, b: f32) -> f32 {
    a.max(b)
}

/// CREDIT: Wikipedia: <https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection>
pub fn are_lines_intersecting([a1, b1]: [Vec2; 2], [a2, b2]: [Vec2; 2]) -> bool {
    let first_bounding_box = BoundingBox::fit_points(a1, b1);
    let second_bounding_box = BoundingBox::fit_points(a2, b2);

    if !first_bounding_box.intersects_other(&second_bounding_box) {
        return false;
    }

    let first_determinant = a1.perp_dot(b1);
    let second_determinant = a2.perp_dot(b2);

    let first_x_determinant = vec2(a1.x, 1.0).perp_dot(vec2(b1.x, 1.0));
    let second_x_determinant = vec2(a2.x, 1.0).perp_dot(vec2(b2.x, 1.0));

    let first_y_determinant = vec2(a1.y, 1.0).perp_dot(vec2(b1.y, 1.0));
    let second_y_determinant = vec2(a2.y, 1.0).perp_dot(vec2(b2.y, 1.0));

    let intersection = vec2(
        vec2(first_determinant, first_x_determinant)
            .perp_dot(vec2(second_determinant, second_x_determinant)),
        vec2(first_determinant, first_y_determinant)
            .perp_dot(vec2(second_determinant, second_y_determinant)),
    ) / vec2(first_x_determinant, first_y_determinant)
        .perp_dot(vec2(second_x_determinant, second_y_determinant));

    first_bounding_box.contains_point(intersection)
        && second_bounding_box.contains_point(intersection)
}
