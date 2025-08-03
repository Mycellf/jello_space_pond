use std::ops::{Index, IndexMut, Neg};

use macroquad::math::Vec2;

#[derive(Clone, Debug, Default)]
pub struct LoopCrafting {
    pub points: [[Option<Direction>; Self::HEIGHT]; Self::WIDTH],
    pub start: Option<[usize; 2]>,
    pub end: Option<[usize; 2]>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Direction {
    Right,
    Up,
    Left,
    Down,
}

impl LoopCrafting {
    pub const SIZE: [usize; 2] = [5, 5];
    pub const WIDTH: usize = Self::SIZE[0];
    pub const HEIGHT: usize = Self::SIZE[1];

    pub fn clear(&mut self) {
        for column in &mut self.points {
            for point in column {
                *point = None;
            }
        }

        self.start = None;
        self.end = None;
    }

    pub fn track_mouse(&mut self, offset: Vec2) {
        let index = <[f32; 2]>::from(offset).map(|x| x.floor() as usize);
        self.track_index(index);
    }

    pub fn track_index(&mut self, index: [usize; 2]) {
        if index[0] >= Self::WIDTH || index[1] >= Self::HEIGHT {
            return;
        }

        if let (Some(start), Some(end)) = (self.start, self.end) {
            for direction in Direction::ALL {
                if direction.apply_offset(end) == Some(index) {
                    if let Some(replaced_direction) = self[index] {
                        if replaced_direction == -direction {
                            self[index] = None;
                            self.end = Some(index);
                        }
                    } else {
                        self[end] = Some(direction);
                        self.end = Some(index);
                    }

                    break;
                }
            }

            if self.start == self.end && self[start].is_some() {
                todo!();
            }
        } else {
            self.start = Some(index);
            self.end = Some(index);
        }
    }
}

impl Index<[usize; 2]> for LoopCrafting {
    type Output = Option<Direction>;

    fn index(&self, index: [usize; 2]) -> &Self::Output {
        &self.points[index[0]][index[1]]
    }
}

impl IndexMut<[usize; 2]> for LoopCrafting {
    fn index_mut(&mut self, index: [usize; 2]) -> &mut Self::Output {
        &mut self.points[index[0]][index[1]]
    }
}

impl Direction {
    pub const ALL: [Self; 4] = [Self::Right, Self::Up, Self::Left, Self::Down];

    pub fn offset(self) -> [isize; 2] {
        match self {
            Direction::Right => [1, 0],
            Direction::Up => [0, 1],
            Direction::Left => [-1, 0],
            Direction::Down => [0, -1],
        }
    }

    pub fn apply_offset(self, index: [usize; 2]) -> Option<[usize; 2]> {
        match self {
            Direction::Right => {
                (index[0] < LoopCrafting::WIDTH - 1).then(|| [index[0] + 1, index[1]])
            }
            Direction::Up => {
                (index[1] < LoopCrafting::HEIGHT - 1).then(|| [index[0], index[1] + 1])
            }
            Direction::Left => (index[0] > 0).then(|| [index[0] - 1, index[1]]),
            Direction::Down => (index[1] > 0).then(|| [index[0], index[1] - 1]),
        }
    }
}

impl Neg for Direction {
    type Output = Direction;

    fn neg(self) -> Self::Output {
        match self {
            Direction::Right => Direction::Left,
            Direction::Up => Direction::Down,
            Direction::Left => Direction::Right,
            Direction::Down => Direction::Up,
        }
    }
}
