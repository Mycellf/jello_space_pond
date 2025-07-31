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
    let mut simulation = assemble_simulation();

    let mut camera = Camera2D {
        zoom: -2.0 / Vec2::splat(10.0),
        ..Default::default()
    };

    let mut fullscreen = START_IN_FULLSCREEN;

    let ticks_per_second = 120.0;

    let minimum_fast_framerate: f32 = 15.0;
    let maximum_ticks_per_frame = (ticks_per_second / minimum_fast_framerate).ceil() as usize;

    let mut tick_time = 0.0;

    loop {
        if input::is_key_pressed(KeyCode::F11) {
            fullscreen ^= true;
            macroquad::window::set_fullscreen(fullscreen);
        }

        tick_time += macroquad::time::get_frame_time() * ticks_per_second;

        for _ in 0..maximum_ticks_per_frame.min(tick_time.floor() as usize) {
            simulation.update(1.0 / ticks_per_second);

            tick_time -= 1.0;
        }

        tick_time = tick_time.min(1.0);

        utils::update_camera_aspect_ratio(&mut camera);
        camera::set_camera(&camera);

        simulation.draw();

        let mouse_position = utils::mouse_position(&camera);

        for (_, soft_body) in &simulation.soft_bodies {
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

fn assemble_simulation() -> Simulation {
    let mut simulation = Simulation::new();

    simulation.soft_bodies.insert(SoftBody::new(
        vec![
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
        vec![
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
        10.0,
    ));

    simulation.soft_bodies.insert(SoftBody::new(
        vec![
            (
                Point {
                    position: vec2(0.0, 0.5),
                    mass: 5.0,
                    ..Default::default()
                },
                Line::default(),
            ),
            (
                Point {
                    position: vec2(0.1, 0.5),
                    mass: 5.0,
                    ..Default::default()
                },
                Line::default(),
            ),
            (
                Point {
                    position: vec2(1.0, 1.5),
                    mass: 5.0,
                    ..Default::default()
                },
                Line::default(),
            ),
            (
                Point {
                    position: vec2(0.0, 1.5),
                    mass: 5.0,
                    ..Default::default()
                },
                Line::default(),
            ),
        ],
        vec![
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
        0.0,
    ));

    simulation.soft_bodies.insert(SoftBody::new(
        vec![
            (
                Point {
                    position: vec2(2.0, -1.0),
                    mass: 1.0,
                    ..Default::default()
                },
                Line::default(),
            ),
            (
                Point {
                    position: vec2(3.0, -1.0),
                    mass: 1.0,
                    ..Default::default()
                },
                Line::default(),
            ),
            (
                Point {
                    position: vec2(3.0, 0.0),
                    mass: 1.0,
                    ..Default::default()
                },
                Line::default(),
            ),
            (
                Point {
                    position: vec2(2.0, 0.0),
                    mass: 1.0,
                    ..Default::default()
                },
                Line::default(),
            ),
        ],
        vec![
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
        0.0,
    ));

    simulation.soft_bodies.insert(SoftBody::new(
        vec![
            (
                Point {
                    position: vec2(-5.0, 3.0),
                    velocity: vec2(1.0, -0.75),
                    mass: 10.0,
                    ..Default::default()
                },
                Line {
                    spring: Spring {
                        force_constant: 100.0,
                        target_distance: 2.0,
                        ..Default::default()
                    },
                },
            ),
            (
                Point {
                    position: vec2(-4.0, 3.0),
                    velocity: vec2(1.0, -0.75),
                    mass: 10.0,
                    ..Default::default()
                },
                Line {
                    spring: Spring {
                        force_constant: 100.0,
                        target_distance: 2.0,
                        ..Default::default()
                    },
                },
            ),
            (
                Point {
                    position: vec2(-4.0, 4.0),
                    velocity: vec2(1.0, -0.75),
                    mass: 10.0,
                    ..Default::default()
                },
                Line {
                    spring: Spring {
                        force_constant: 100.0,
                        target_distance: 0.25,
                        ..Default::default()
                    },
                },
            ),
            (
                Point {
                    position: vec2(-5.0, 4.0),
                    velocity: vec2(1.0, -0.75),
                    mass: 10.0,
                    ..Default::default()
                },
                Line {
                    spring: Spring {
                        force_constant: 100.0,
                        target_distance: 0.25,
                        ..Default::default()
                    },
                },
            ),
        ],
        vec![
            (
                [0, 2],
                Spring {
                    target_distance: 1.0,
                    force_constant: 200.0,
                    damping: 20.0,
                },
            ),
            (
                [1, 3],
                Spring {
                    target_distance: 4.0,
                    force_constant: 200.0,
                    ..Default::default()
                },
            ),
        ],
        50.0,
    ));

    simulation.update_keys();

    simulation
}
