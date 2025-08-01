pub mod constraint;
pub mod simulation;
pub mod soft_body;
pub mod utils;

use std::f32::consts::{SQRT_2, TAU};

use macroquad::{
    camera::{self, Camera2D},
    color::colors,
    input::{self, KeyCode},
    math::{Vec2, vec2},
    shapes,
    window::{self, Conf},
};

use crate::{
    constraint::{Constraint, PointHandle},
    simulation::Simulation,
    soft_body::{AngularSpring, LinearSpring, SoftBodyBuilder},
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
    let mut debug = false;
    let mut running = true;

    let ticks_per_second = 120.0;

    let minimum_fast_framerate: f32 = 15.0;
    let maximum_ticks_per_frame = (ticks_per_second / minimum_fast_framerate).ceil() as usize;

    let mut tick_time = 0.0;

    loop {
        if input::is_key_pressed(KeyCode::F11) {
            fullscreen ^= true;
            macroquad::window::set_fullscreen(fullscreen);
        }

        if input::is_key_pressed(KeyCode::F3) {
            debug ^= true;
        }

        if input::is_key_pressed(KeyCode::Space) {
            running ^= true;
        }

        if running {
            tick_time += macroquad::time::get_frame_time() * ticks_per_second;

            for _ in 0..maximum_ticks_per_frame.min(tick_time.floor() as usize) {
                simulation.update(1.0 / ticks_per_second);

                tick_time -= 1.0;
            }

            tick_time = tick_time.min(1.0);
        }

        let mut input = vec2(0.0, 0.0);

        input.x += input::is_key_down(KeyCode::D) as u8 as f32;
        input.x -= input::is_key_down(KeyCode::A) as u8 as f32;
        input.y += input::is_key_down(KeyCode::W) as u8 as f32;
        input.y -= input::is_key_down(KeyCode::S) as u8 as f32;

        camera.target += input * macroquad::time::get_frame_time() * 5.0;

        utils::update_camera_aspect_ratio(&mut camera);
        camera::set_camera(&camera);

        simulation.draw(debug);

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

    let diagonal_spring = LinearSpring {
        target_distance: SQRT_2,
        ..Default::default()
    };

    let orthogonal_spring = LinearSpring::default();

    simulation.soft_bodies.insert(
        SoftBodyBuilder::default()
            .gas_force(30.0)
            .offset(-3.0, -1.5)
            .spring_scale(2.0)
            .subdivisions(2)
            .point(0.5, 0.5)
            .with_internal_spring_start(0)
            .with_internal_spring_start(3)
            .with_internal_spring_start(8)
            .point(0.0, 0.5)
            .with_internal_spring_start(1)
            .point(0.0, 0.0)
            .with_internal_spring_end(0, diagonal_spring)
            .point(0.5, 0.0)
            .with_internal_spring_end(1, diagonal_spring)
            .with_internal_spring_end(8, orthogonal_spring)
            .with_internal_spring_start(2)
            .point(1.0, 0.0)
            .with_internal_spring_end(3, diagonal_spring)
            .with_internal_spring_start(4)
            .with_internal_spring_start(9)
            .point(1.5, 0.0)
            .with_internal_spring_start(5)
            .with_internal_spring_start(10)
            .point(1.5, 0.5)
            .with_internal_spring_end(4, diagonal_spring)
            .with_internal_spring_start(7)
            .with_internal_spring_start(11)
            .point(1.5, 1.0)
            .with_internal_spring_start(6)
            .point(1.0, 1.0)
            .with_internal_spring_end(7, diagonal_spring)
            .point(1.0, 0.5)
            .with_internal_spring_end(2, diagonal_spring)
            .with_internal_spring_end(5, diagonal_spring)
            .with_internal_spring_end(6, diagonal_spring)
            .with_internal_spring_end(9, orthogonal_spring)
            .with_internal_spring_end(10, diagonal_spring)
            .with_internal_spring_end(11, orthogonal_spring)
            .build(),
    );

    let diagonal_spring = LinearSpring {
        target_distance: SQRT_2,
        ..Default::default()
    };

    simulation.soft_bodies.insert({
        let mut soft_body = SoftBodyBuilder::default()
            .offset(10.0, -10.0)
            .subdivisions(2)
            .point(0.0, 0.0)
            .with_internal_spring_start(0)
            .point(1.0, 0.0)
            .with_internal_spring_start(1)
            .point(1.0, 1.0)
            .with_internal_spring_end(0, diagonal_spring)
            .point(0.0, 1.0)
            .with_internal_spring_end(1, diagonal_spring)
            .build();

        soft_body.shape[1].0.position.x -= 0.3;
        soft_body.shape[2].0.position.x -= 0.6;
        soft_body.shape[3].0.position.x -= 0.9;

        soft_body
    });

    simulation.soft_bodies.insert(
        SoftBodyBuilder::default()
            .offset(3.0, -5.0)
            .subdivisions(2)
            .point(0.0, 0.0)
            .with_internal_spring_start(0)
            .point(1.0, 0.0)
            .with_internal_spring_start(1)
            .point(1.0, 1.0)
            .with_internal_spring_end(0, diagonal_spring)
            .point(0.0, 1.0)
            .with_internal_spring_end(1, diagonal_spring)
            .build(),
    );

    simulation.soft_bodies.insert(
        SoftBodyBuilder::default()
            .gas_force(50.0)
            .mass(10.0)
            .velocity(3.0, -3.0)
            .offset(-12.0, 7.75)
            .point(0.0, 0.0)
            .with_internal_spring_start(0)
            .with_spring(LinearSpring {
                force_constant: 100.0,
                target_distance: 2.0,
                ..Default::default()
            })
            .point(1.0, 0.0)
            .with_internal_spring_start(1)
            .with_spring(LinearSpring {
                force_constant: 100.0,
                target_distance: 2.0,
                ..Default::default()
            })
            .point(1.0, 1.0)
            .with_internal_spring_end(
                0,
                LinearSpring {
                    target_distance: 1.0,
                    force_constant: 200.0,
                    damping: 20.0,
                    ..Default::default()
                },
            )
            .with_spring(LinearSpring {
                force_constant: 100.0,
                target_distance: 0.25,
                ..Default::default()
            })
            .point(0.0, 1.0)
            .with_internal_spring_end(
                1,
                LinearSpring {
                    target_distance: 4.0,
                    force_constant: 200.0,
                    ..Default::default()
                },
            )
            .with_spring(LinearSpring {
                force_constant: 100.0,
                target_distance: 0.25,
                ..Default::default()
            })
            .build(),
    );

    let mut keys = Vec::new();

    for x in 3..19 {
        for y in -21..-5 {
            let mut builder = SoftBodyBuilder::default()
                .gas_force(5.0)
                .friction(1.0)
                .mass(0.5)
                .base_angular_spring(Some(AngularSpring {
                    force_constant: 0.5,
                    damping: 0.1,
                    outwards: false,
                    ..Default::default()
                }))
                .base_spring({
                    LinearSpring {
                        force_constant: 25.0,
                        ..Default::default()
                    }
                })
                .spring_scale(0.5)
                .offset(x as f32 * 2.0, y as f32 * 2.0);

            for i in 0..12 {
                let angle = i as f32 / 12.0 * TAU;

                builder = builder.point(angle.cos(), angle.sin())
            }

            let key = simulation.soft_bodies.insert(builder.build());

            keys.push(key);
        }
    }

    simulation.insert_constraint(Constraint::HoldTogether {
        points: vec![
            PointHandle {
                soft_body: keys[14],
                index: 2,
            },
            PointHandle {
                soft_body: keys[15],
                index: 10,
            },
        ],
    });

    simulation.insert_constraint(Constraint::HoldTogether {
        points: vec![
            PointHandle {
                soft_body: keys[14],
                index: 3,
            },
            PointHandle {
                soft_body: keys[15],
                index: 9,
            },
        ],
    });

    simulation.insert_constraint(Constraint::HoldTogether {
        points: vec![
            PointHandle {
                soft_body: keys[14],
                index: 4,
            },
            PointHandle {
                soft_body: keys[15],
                index: 8,
            },
        ],
    });

    simulation.update_keys();

    simulation
}
