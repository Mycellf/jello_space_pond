use crate::soft_body::SoftBody;

pub struct Simulation {
    pub soft_bodies: Vec<SoftBody>,
}

impl Simulation {
    pub fn draw(&self) {
        for soft_body in &self.soft_bodies {
            soft_body.draw();
        }
    }

    pub fn update(&mut self, dt: f32) {
        for soft_body in &mut self.soft_bodies {
            soft_body.apply_impulse_and_velocity(dt);

            soft_body.update_bounding_box();
        }
    }
}
