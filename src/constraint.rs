use macroquad::math::Vec2;
use slotmap::HopSlotMap;

use crate::{
    simulation::{ConstraintKey, SoftBodyKey},
    soft_body::{Point, SoftBody},
};

#[derive(Clone, Debug)]
pub enum Constraint {
    HoldTogether { points: Vec<PointHandle> },
}

impl Constraint {
    pub fn apply_to_soft_bodies(&mut self, soft_bodies: &mut HopSlotMap<SoftBodyKey, SoftBody>) {
        match self {
            Constraint::HoldTogether { points } => {
                let mut total_mass = 0.0;
                let mut total_momentum = Vec2::ZERO;
                let mut total_mass_moment = Vec2::ZERO;

                let mut i = 0;
                while i < points.len() {
                    let Some(point) = points[i].get(soft_bodies) else {
                        points.remove(i);
                        continue;
                    };

                    if point.constraint.is_none() {
                        points.remove(i);
                        continue;
                    }

                    total_mass += point.mass;
                    total_momentum += point.velocity * point.mass;
                    total_mass_moment += point.position * point.mass;

                    i += 1;
                }

                let average_velocity = total_momentum / total_mass;
                let average_position = total_mass_moment / total_mass;

                for handle in points {
                    let point = handle.get_mut(soft_bodies).unwrap();

                    point.position = average_position;
                    point.velocity = average_velocity;
                }
            }
        }
    }

    pub fn insert(
        &mut self,
        key: ConstraintKey,
        soft_bodies: &mut HopSlotMap<SoftBodyKey, SoftBody>,
        keys_to_replace: &mut Vec<ConstraintKey>,
    ) {
        match self {
            Constraint::HoldTogether { points } => {
                let mut i = 0;
                while i < points.len() {
                    let Some(point) = points[i].get_mut(soft_bodies) else {
                        points.remove(i);
                        continue;
                    };

                    if let Some(key) = point.constraint {
                        keys_to_replace.push(key);
                    }

                    point.constraint = Some(key);

                    i += 1;
                }
            }
        }
    }

    pub fn remove(
        &self,
        key: ConstraintKey,
        replacement: Option<ConstraintKey>,
        soft_bodies: &mut HopSlotMap<SoftBodyKey, SoftBody>,
        points_regrouped: &mut Vec<PointHandle>,
    ) {
        match self {
            Constraint::HoldTogether { points } => {
                for point_handle in points {
                    let Some(point) = point_handle.get_mut(soft_bodies) else {
                        continue;
                    };

                    if point.constraint == Some(key) {
                        if replacement.is_some() {
                            points_regrouped.push(*point_handle);
                        }
                        point.constraint = replacement;
                    }
                }
            }
        }
    }

    pub fn is_empty(&self) -> bool {
        match self {
            Constraint::HoldTogether { points } => points.len() <= 1,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PointHandle {
    pub soft_body: SoftBodyKey,
    pub index: usize,
}

impl PointHandle {
    pub fn get(self, soft_bodies: &HopSlotMap<SoftBodyKey, SoftBody>) -> Option<&Point> {
        Some(&soft_bodies.get(self.soft_body)?.shape.get(self.index)?.0)
    }

    pub fn get_mut(
        self,
        soft_bodies: &mut HopSlotMap<SoftBodyKey, SoftBody>,
    ) -> Option<&mut Point> {
        Some(
            &mut soft_bodies
                .get_mut(self.soft_body)?
                .shape
                .get_mut(self.index)?
                .0,
        )
    }
}
