use macroquad::math::Vec2;
use slotmap::HopSlotMap;

use crate::{simulation::SoftBodyKey, soft_body::SoftBody};

#[derive(Clone, Debug)]
pub enum Constraint {
    HoldTogether { points: Vec<PointHandle> },
}

impl Constraint {
    pub fn apply_to_soft_bodies(&self, soft_bodies: &mut HopSlotMap<SoftBodyKey, SoftBody>) {
        match self {
            Constraint::HoldTogether { points } => {
                let mut total_mass = 0.0;
                let mut total_momentum = Vec2::ZERO;
                let mut total_mass_moment = Vec2::ZERO;

                for &PointHandle { soft_body, index } in points {
                    let point = &soft_bodies[soft_body].shape[index].0;

                    total_mass += point.mass;
                    total_momentum += point.velocity * point.mass;
                    total_mass_moment += point.position * point.mass;
                }

                let average_velocity = total_momentum / total_mass;
                let average_position = total_mass_moment / total_mass;

                for &PointHandle { soft_body, index } in points {
                    let point = &mut soft_bodies[soft_body].shape[index].0;

                    point.position = average_position;
                    point.velocity = average_velocity;
                }
            }
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct PointHandle {
    pub soft_body: SoftBodyKey,
    pub index: usize,
}
