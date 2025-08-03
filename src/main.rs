pub mod constraint;
pub mod loop_crafting;
pub mod particle;
pub mod simulation;
pub mod soft_body;
pub mod utils;

use std::f32::consts::{SQRT_2, TAU};

use macroquad::{
    camera::{self, Camera2D},
    input::{self, KeyCode},
    math::{Vec2, vec2},
    window::{self, Conf},
};

use crate::{
    simulation::Simulation,
    soft_body::{
        Actor, AngularSpring, AttatchmentPointHandle, ConnectionState, Keybind, LinearSpring,
        SoftBodyBuilder,
    },
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

    let zoom_speed = 1.1f32;
    let mut screen_height = 10.0;
    let mut camera = Camera2D {
        zoom: -2.0 / Vec2::splat(screen_height),
        ..Default::default()
    };

    let mut fullscreen = START_IN_FULLSCREEN;
    let mut debug = false;
    let mut running = true;

    let ticks_per_second = 120.0;

    let minimum_fast_framerate: f32 = 30.0;
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

        let input = input::mouse_wheel().1.clamp(-1.0, 1.0);

        if input.abs() > f32::EPSILON {
            screen_height *= zoom_speed.powf(-input);
            screen_height = screen_height.clamp(10.0, 100.0);

            camera.zoom = -2.0 / Vec2::splat(screen_height);
        }

        // let mut input = vec2(0.0, 0.0);
        //
        // input.x += input::is_key_down(KeyCode::D) as u8 as f32;
        // input.x -= input::is_key_down(KeyCode::A) as u8 as f32;
        // input.y += input::is_key_down(KeyCode::W) as u8 as f32;
        // input.y -= input::is_key_down(KeyCode::S) as u8 as f32;
        //
        // camera.target += input * macroquad::time::get_frame_time() * 5.0;

        utils::update_camera_aspect_ratio(&mut camera);

        simulation.update_input(&camera, macroquad::time::get_frame_time());

        if running {
            tick_time += macroquad::time::get_frame_time() * ticks_per_second;

            for _ in 0..maximum_ticks_per_frame.min(tick_time.floor() as usize) {
                let new_camera_position = simulation.tick_simulation(1.0 / ticks_per_second);

                if let Some(new_camera_position) = new_camera_position {
                    camera.target = new_camera_position;
                }

                tick_time -= 1.0;
            }

            tick_time = tick_time.min(1.0);
        }

        camera::set_camera(&camera);

        simulation.draw(debug);

        window::next_frame().await;
    }
}

fn assemble_simulation() -> Simulation {
    let mut simulation = Simulation::new();

    let mut builder = SoftBodyBuilder::default()
        .connection_state(ConnectionState::Source)
        .gas_force(10.0)
        .mass(0.5)
        .base_angular_spring(Some(AngularSpring {
            force_constant: 50.0,
            damping: 5.0,
            ..Default::default()
        }))
        .spring_scale(0.75)
        .with_actor(Actor::HabitatBubble {
            minimum_pressure: 0.5,
        })
        .offset(-5.0, 0.0);

    for i in 0..12 {
        let angle = (i as f32 + 0.5) / 12.0 * TAU;

        builder = builder.point(angle.cos(), angle.sin());

        if i % 3 == 1 {
            builder = builder.with_attatchment_point(4);
        }
    }

    simulation.soft_bodies.insert(builder.build());

    let diagonal_spring = LinearSpring {
        target_distance: SQRT_2,
        ..Default::default()
    };

    let slight_diagonal_spring = LinearSpring {
        target_distance: vec2(1.0, 1.0 / 3.0).length(),
        ..Default::default()
    };

    let orthogonal_spring = LinearSpring {
        target_distance: 1.0 / 3.0,
        ..Default::default()
    };

    let corner_spring = LinearSpring {
        target_distance: SQRT_2 / 3.0,
        damping: 100.0,
        tension: false,
        ..Default::default()
    };

    let keybinds = [
        Keybind {
            activate: vec![KeyCode::W],
            disable: vec![KeyCode::S],
        },
        Keybind {
            activate: vec![KeyCode::S],
            disable: vec![KeyCode::W],
        },
        Keybind {
            activate: vec![KeyCode::W, KeyCode::D],
            disable: vec![KeyCode::S, KeyCode::A],
        },
        Keybind {
            activate: vec![KeyCode::S, KeyCode::A],
            disable: vec![KeyCode::W, KeyCode::D],
        },
        Keybind {
            activate: vec![KeyCode::W, KeyCode::A],
            disable: vec![KeyCode::S, KeyCode::D],
        },
        Keybind {
            activate: vec![KeyCode::S, KeyCode::D],
            disable: vec![KeyCode::W, KeyCode::A],
        },
    ];

    for x in 0..8 {
        for y in 0..6 {
            simulation.soft_bodies.insert(
                SoftBodyBuilder::default()
                    .offset(x as f32, -(y as f32) * 2.0)
                    .point(0.0, 0.0)
                    .with_attatchment_point(2)
                    .with_internal_spring_start(0)
                    .with_internal_spring_start(4)
                    .point(1.0 / 3.0, 0.0)
                    .with_internal_spring_start(1)
                    .with_internal_spring_start(5)
                    .with_attatchment_point(4)
                    .point(1.0 / 3.0, 1.0 / 3.0)
                    .with_internal_spring_start(2)
                    .with_internal_spring_end(4, corner_spring)
                    .point(1.0 / 3.0, 2.0 / 3.0)
                    .with_internal_spring_start(3)
                    .with_internal_spring_start(6)
                    .point(1.0 / 3.0, 1.0)
                    .with_attatchment_point(2)
                    .with_internal_spring_start(7)
                    .with_internal_spring_end(0, slight_diagonal_spring)
                    .point(0.0, 1.0)
                    .with_internal_spring_end(1, slight_diagonal_spring)
                    .with_internal_spring_end(6, corner_spring)
                    .point(0.0, 2.0 / 3.0)
                    .with_internal_spring_end(3, orthogonal_spring)
                    .with_internal_spring_end(7, corner_spring)
                    .with_actor(Actor::RocketMotor {
                        line: 0,
                        force: vec2(100.0, 0.0),
                        enable: keybinds[y].clone(),
                        particle_time: 0.0,
                        max_particle_time: 0.005,
                    })
                    .point(0.0, 1.0 / 3.0)
                    .with_internal_spring_end(2, orthogonal_spring)
                    .with_internal_spring_end(5, corner_spring)
                    .build(),
            );
        }
    }

    for x in 0..12 {
        for y in 2..12 {
            simulation.soft_bodies.insert(
                SoftBodyBuilder::default()
                    .offset(x as f32 * 4.0, y as f32 * 2.0)
                    .point(0.0, 0.0)
                    .with_attatchment_point(4)
                    .with_internal_spring_start(0)
                    .point(1.0 / 3.0, 0.0)
                    .with_internal_spring_start(2)
                    .point(2.0 / 3.0, 0.0)
                    .with_internal_spring_start(3)
                    .point(1.0, 0.0)
                    .with_attatchment_point(4)
                    .with_internal_spring_start(1)
                    .point(1.0, 1.0 / 3.0)
                    .with_internal_spring_end(3, corner_spring)
                    .point(1.0, 2.0 / 3.0)
                    .with_internal_spring_start(4)
                    .point(1.0, 1.0)
                    .with_attatchment_point(4)
                    .with_internal_spring_end(0, diagonal_spring)
                    .point(2.0 / 3.0, 1.0)
                    .with_internal_spring_end(4, corner_spring)
                    .point(1.0 / 3.0, 1.0)
                    .with_internal_spring_start(5)
                    .point(0.0, 1.0)
                    .with_attatchment_point(4)
                    .with_internal_spring_end(1, diagonal_spring)
                    .point(0.0, 2.0 / 3.0)
                    .with_internal_spring_end(5, corner_spring)
                    .point(0.0, 1.0 / 3.0)
                    .with_internal_spring_end(2, corner_spring)
                    .build(),
            );

            simulation.soft_bodies.insert(
                SoftBodyBuilder::default()
                    .offset(x as f32 * 4.0 - 1.0, y as f32 * 2.0)
                    .point(0.0, 0.0)
                    .with_attatchment_point(2)
                    .with_internal_spring_start(0)
                    .point(1.0 / 3.0, 0.0)
                    .with_internal_spring_start(1)
                    .with_attatchment_point(4)
                    .point(1.0 / 3.0, 1.0 / 3.0)
                    .with_internal_spring_start(2)
                    .point(1.0 / 3.0, 2.0 / 3.0)
                    .with_internal_spring_start(3)
                    .point(1.0 / 3.0, 1.0)
                    .with_attatchment_point(2)
                    .with_internal_spring_end(0, slight_diagonal_spring)
                    .point(0.0, 1.0)
                    .with_attatchment_point(4)
                    .with_internal_spring_end(1, slight_diagonal_spring)
                    .point(0.0, 2.0 / 3.0)
                    .with_internal_spring_end(3, orthogonal_spring)
                    .point(0.0, 1.0 / 3.0)
                    .with_internal_spring_end(2, orthogonal_spring)
                    .build(),
            );
        }
    }

    let mut keys = Vec::new();

    for x in 0..12 {
        for y in 2..12 {
            let mut builder = SoftBodyBuilder::default()
                .gas_force(10.0)
                .friction(1.0)
                .mass(0.5)
                .base_angular_spring(Some(AngularSpring {
                    force_constant: 50.0,
                    damping: 5.0,
                    outwards: false,
                    ..Default::default()
                }))
                .base_spring({
                    LinearSpring {
                        force_constant: 50.0,
                        maximum_force: 1.5,
                        ..Default::default()
                    }
                })
                .spring_scale(0.5)
                .offset(x as f32 * 4.0 + 2.0, y as f32 * 2.0);

            for i in 0..12 {
                let angle = (i as f32 + 0.5) / 12.0 * TAU;

                builder = builder.point(angle.cos(), angle.sin());

                if i % 3 == 1 {
                    builder = builder.with_attatchment_point(4);
                }
            }

            let key = simulation.soft_bodies.insert(builder.build());

            keys.push(key);
        }
    }

    simulation
        .connect_attatchment_points([
            AttatchmentPointHandle {
                soft_body: keys[14],
                index: 0,
            },
            AttatchmentPointHandle {
                soft_body: keys[15],
                index: 2,
            },
        ])
        .unwrap();

    simulation.update_keys();

    simulation
}
