use slotmap::{HopSlotMap, new_key_type};

use crate::{constraint::Constraint, soft_body::SoftBody};

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
            soft_body.draw();

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

        let mut empty_constraints = Vec::new();

        for (key, constraint) in &mut self.constraints {
            constraint.apply_to_soft_bodies(&mut self.soft_bodies);

            if constraint.is_empty() {
                empty_constraints.push(key);
            }
        }

        for key in empty_constraints {
            self.remove_constraint(key, None);
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

        let mut i = 0;

        while i < self.keys.len() {
            let key = self.keys[i];

            let soft_body = &mut self.soft_bodies[key];

            if let Some(debris_age) = soft_body.debris_age {
                if debris_age >= SoftBody::DEBRIS_DECAY_TIME || soft_body.area() < 0.0 {
                    self.soft_bodies.remove(key);
                    self.keys.swap_remove(i);

                    continue;
                }
            } else {
                if soft_body.is_self_intersecting() {
                    self.destroy_soft_body(key, Some(i));

                    continue;
                }
            }

            soft_body.update_triangulation_indecies();

            i += 1;
        }
    }

    pub fn destroy_soft_body(&mut self, key: SoftBodyKey, key_index: Option<usize>) {
        let soft_body = self.soft_bodies.remove(key).unwrap();
        if let Some(i) = key_index {
            self.keys.swap_remove(i);
        }

        for triangle in soft_body.decompose_into_triangles() {
            let key = self.soft_bodies.insert(triangle);
            if key_index.is_some() {
                self.keys.push(key);
            }
        }

        if key_index.is_none() {
            self.update_keys();
        }
    }

    pub fn insert_constraint(&mut self, mut constraint: Constraint) -> ConstraintKey {
        let mut keys_to_replace = Vec::new();

        let key = self.constraints.insert_with_key(|key| {
            constraint.insert(key, &mut self.soft_bodies, &mut keys_to_replace);
            constraint
        });

        for key_to_replace in keys_to_replace {
            self.remove_constraint(key_to_replace, Some(key));
        }

        key
    }

    pub fn remove_constraint(
        &mut self,
        key: ConstraintKey,
        replacement: Option<ConstraintKey>,
    ) -> Option<Constraint> {
        let constraint = self.constraints.remove(key);

        if let Some(constraint) = constraint {
            let mut points_to_replace = Vec::new();
            constraint.remove(
                key,
                replacement,
                &mut self.soft_bodies,
                &mut points_to_replace,
            );

            if let Some(replacement) = replacement {
                let constraint = &mut self.constraints[replacement];

                for point in points_to_replace {
                    match constraint {
                        Constraint::HoldTogether { points } => points.push(point),
                    }
                }
            }

            Some(constraint)
        } else {
            None
        }
    }
}
