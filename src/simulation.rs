use macroquad::color::colors;
use slotmap::{SlotMap, new_key_type};

use crate::soft_body::SoftBody;

pub struct Simulation {
    pub soft_bodies: SlotMap<SoftBodyKey, SoftBody>,
    pub keys: Vec<SoftBodyKey>,
}

new_key_type! {
    pub struct SoftBodyKey;
}

impl Simulation {
    pub fn new() -> Self {
        Self {
            soft_bodies: SlotMap::default(),
            keys: Vec::new(),
        }
    }

    pub fn draw(&self, debug: bool) {
        for (_, soft_body) in &self.soft_bodies {
            soft_body.fill_color(colors::WHITE);

            if debug {
                soft_body.draw_springs();
            }
        }
    }

    pub fn update_keys(&mut self) {
        self.keys = self.soft_bodies.keys().collect();
    }

    pub fn update(&mut self, dt: f32) {
        for (_, soft_body) in &mut self.soft_bodies {
            soft_body.apply_impulse_and_velocity(dt);
        }

        for (i, &first_key) in self.keys.iter().enumerate().skip(1) {
            for &second_key in self.keys.iter().take(i) {
                let [first, second] = self
                    .soft_bodies
                    .get_disjoint_mut([first_key, second_key])
                    .unwrap();

                if first.bounding_box.intersects_other(&second.bounding_box) {
                    first.check_points_against_other_one_sided(second);
                    second.check_points_against_other_one_sided(first);
                }
            }
        }
    }
}
