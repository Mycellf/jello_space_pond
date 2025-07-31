pub mod simulation;
pub mod soft_body;
pub mod utils;

use std::f32::consts::SQRT_2;

use macroquad::{
    camera::{self, Camera2D},
    color::colors,
    input::{self, KeyCode},
    math::{Vec2, vec2},
    shapes,
    window::{self, Conf},
};

use crate::{
    simulation::Simulation,
    soft_body::{Line, Point, SoftBody, Spring},
};

const START_IN_FULLSCREEN: bool = true;

fn config() -> Conf {
    Conf {
        window_title: "Jello Space Pond".to_owned(),
        fullscreen: START_IN_FULLSCREEN,
        ..Default::default()
    }
}

#[macroquad::main(config)]
async fn main() {
    let mut simulation = Simulation {
        soft_bodies: vec![
            SoftBody {
                shape: vec![
                    (
                        Point {
                            position: vec2(0.0, -1.0),
                            mass: 1.0,
                            ..Default::default()
                        },
                        Line::default(),
                    ),
                    (
                        Point {
                            position: vec2(-0.5, -1.0),
                            mass: 1.0,
                            ..Default::default()
                        },
                        Line::default(),
                    ),
                    (
                        Point {
                            position: vec2(-0.5, -1.5),
                            mass: 1.0,
                            ..Default::default()
                        },
                        Line::default(),
                    ),
                    (
                        Point {
                            position: vec2(0.0, -1.5),
                            mass: 1.0,
                            ..Default::default()
                        },
                        Line::default(),
                    ),
                    (
                        Point {
                            position: vec2(0.5, -1.5),
                            mass: 1.0,
                            ..Default::default()
                        },
                        Line::default(),
                    ),
                    (
                        Point {
                            position: vec2(1.0, -1.5),
                            mass: 1.0,
                            ..Default::default()
                        },
                        Line::default(),
                    ),
                    (
                        Point {
                            position: vec2(1.0, -1.0),
                            mass: 1.0,
                            ..Default::default()
                        },
                        Line::default(),
                    ),
                    (
                        Point {
                            position: vec2(1.0, -0.5),
                            mass: 1.0,
                            ..Default::default()
                        },
                        Line::default(),
                    ),
                    (
                        Point {
                            position: vec2(0.5, -0.5),
                            mass: 1.0,
                            ..Default::default()
                        },
                        Line::default(),
                    ),
                    (
                        Point {
                            position: vec2(0.5, -1.0),
                            mass: 1.0,
                            ..Default::default()
                        },
                        Line::default(),
                    ),
                ],
                internal_springs: vec![
                    (
                        [0, 2],
                        Spring {
                            target_distance: SQRT_2,
                            ..Default::default()
                        },
                    ),
                    (
                        [1, 3],
                        Spring {
                            target_distance: SQRT_2,
                            ..Default::default()
                        },
                    ),
                    (
                        [9, 3],
                        Spring {
                            target_distance: SQRT_2,
                            ..Default::default()
                        },
                    ),
                    (
                        [0, 4],
                        Spring {
                            target_distance: SQRT_2,
                            ..Default::default()
                        },
                    ),
                    (
                        [4, 6],
                        Spring {
                            target_distance: SQRT_2,
                            ..Default::default()
                        },
                    ),
                    (
                        [5, 9],
                        Spring {
                            target_distance: SQRT_2,
                            ..Default::default()
                        },
                    ),
                    (
                        [9, 7],
                        Spring {
                            target_distance: SQRT_2,
                            ..Default::default()
                        },
                    ),
                    (
                        [8, 6],
                        Spring {
                            target_distance: SQRT_2,
                            ..Default::default()
                        },
                    ),
                    ([0, 3], Spring::default()),
                    ([9, 4], Spring::default()),
                    ([9, 6], Spring::default()),
                ],
                bounding_box: Default::default(),
            },
            SoftBody {
                shape: vec![
                    (
                        Point {
                            position: vec2(0.0, -0.5),
                            mass: 5.0,
                            ..Default::default()
                        },
                        Line {
                            spring: Spring::default(),
                        },
                    ),
                    (
                        Point {
                            position: vec2(0.1, -0.5),
                            mass: 5.0,
                            ..Default::default()
                        },
                        Line {
                            spring: Spring::default(),
                        },
                    ),
                    (
                        Point {
                            position: vec2(1.0, 0.5),
                            mass: 5.0,
                            ..Default::default()
                        },
                        Line {
                            spring: Spring::default(),
                        },
                    ),
                    (
                        Point {
                            position: vec2(0.0, 0.5),
                            mass: 5.0,
                            ..Default::default()
                        },
                        Line {
                            spring: Spring::default(),
                        },
                    ),
                ],
                internal_springs: vec![
                    (
                        [0, 2],
                        Spring {
                            target_distance: SQRT_2,
                            ..Default::default()
                        },
                    ),
                    (
                        [1, 3],
                        Spring {
                            target_distance: SQRT_2,
                            ..Default::default()
                        },
                    ),
                ],
                bounding_box: Default::default(),
            },
            SoftBody {
                shape: vec![
                    (
                        Point {
                            position: vec2(2.0, -1.0),
                            mass: 1.0,
                            ..Default::default()
                        },
                        Line {
                            spring: Spring::default(),
                        },
                    ),
                    (
                        Point {
                            position: vec2(3.0, -1.0),
                            mass: 1.0,
                            ..Default::default()
                        },
                        Line {
                            spring: Spring::default(),
                        },
                    ),
                    (
                        Point {
                            position: vec2(3.0, 0.0),
                            mass: 1.0,
                            ..Default::default()
                        },
                        Line {
                            spring: Spring::default(),
                        },
                    ),
                    (
                        Point {
                            position: vec2(2.0, 0.0),
                            mass: 1.0,
                            ..Default::default()
                        },
                        Line {
                            spring: Spring::default(),
                        },
                    ),
                ],
                internal_springs: vec![
                    (
                        [0, 2],
                        Spring {
                            target_distance: SQRT_2,
                            ..Default::default()
                        },
                    ),
                    (
                        [1, 3],
                        Spring {
                            target_distance: SQRT_2,
                            ..Default::default()
                        },
                    ),
                ],
                bounding_box: Default::default(),
            },
        ],
    };

    let mut camera = Camera2D {
        zoom: -2.0 / Vec2::splat(10.0),
        ..Default::default()
    };

    let mut fullscreen = START_IN_FULLSCREEN;

    loop {
        if input::is_key_pressed(KeyCode::F11) {
            fullscreen ^= true;
            macroquad::window::set_fullscreen(fullscreen);
        }

        simulation.update(1.0 / 120.0);

        utils::update_camera_aspect_ratio(&mut camera);
        camera::set_camera(&camera);

        simulation.draw();

        let mouse_position = utils::mouse_position(&camera);

        for soft_body in &simulation.soft_bodies {
            if soft_body.contains_point(mouse_position) {
                soft_body.bounding_box.draw();

                let (closest_line, closest_point, _, _) =
                    soft_body.closest_line_to_point(mouse_position);

                let (point_a, _, point_b) = soft_body.get_line(closest_line).unwrap();

                utils::draw_line(point_a.position, point_b.position, 0.05, colors::BLUE);

                shapes::draw_circle(closest_point.x, closest_point.y, 0.05, colors::BLUE);
            }
        }

        window::next_frame().await;
    }
}
