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
        }

        for i in 1..self.soft_bodies.len() {
            for j in 0..i {
                let [first, second] = self.soft_bodies.get_disjoint_mut([i, j]).unwrap();

                if !first.bounding_box.intersects_other(&second.bounding_box) {
                    // The objects do not intersect
                    continue;
                }

                first.check_points_against_other_one_sided(second);
                second.check_points_against_other_one_sided(first);
            }
        }
    }
}
