use macroquad::color::colors;
use slotmap::{HopSlotMap, new_key_type};

use crate::{
    constraint::{Constraint, PointHandle},
    soft_body::SoftBody,
};

pub struct Simulation {
    pub soft_bodies: HopSlotMap<SoftBodyKey, SoftBody>,
    pub keys: Vec<SoftBodyKey>,

    pub constraints: HopSlotMap<ConstraintKey, Constraint>,
}

new_key_type! {
    pub struct SoftBodyKey;
    pub struct ConstraintKey;
}

impl Simulation {
    pub fn new() -> Self {
        Self {
            soft_bodies: HopSlotMap::default(),
            keys: Vec::new(),

            constraints: HopSlotMap::default(),
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

        for (_, constraint) in &mut self.constraints {
            constraint.apply_to_soft_bodies(&mut self.soft_bodies);
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

    pub fn insert_constraint(&mut self, constraint: Constraint) -> ConstraintKey {
        let mut keys_to_remove = Vec::new();

        let key = self.constraints.insert_with_key(|key| {
            match &constraint {
                Constraint::HoldTogether { points } => {
                    for &PointHandle { soft_body, index } in points {
                        let point = &mut self.soft_bodies[soft_body].shape[index].0;

                        if let Some(key) = point.constraint {
                            keys_to_remove.push(key);
                        }

                        point.constraint = Some(key);
                    }
                }
            }

            constraint
        });

        for key in keys_to_remove {
            self.remove_constraint(key);
        }

        key
    }

    pub fn remove_constraint(&mut self, key: ConstraintKey) -> Option<Constraint> {
        let constraint = self.constraints.remove(key);

        if let Some(constraint) = constraint {
            match &constraint {
                Constraint::HoldTogether { points } => {
                    for &PointHandle { soft_body, index } in points {
                        let point = &mut self.soft_bodies[soft_body].shape[index].0;

                        if point.constraint == Some(key) {
                            point.constraint = None;
                        }
                    }
                }
            }

            Some(constraint)
        } else {
            None
        }
    }
}
