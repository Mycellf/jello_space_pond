use std::ops::{Index, IndexMut, Neg};

use macroquad::math::Vec2;
use ndarray::Array2;

use crate::utils::{RotateClockwise, RotateCounterClockwise};

#[derive(Clone, Debug, Default)]
pub struct LoopCrafting {
    pub points: [[Option<Direction>; Self::HEIGHT]; Self::WIDTH],
    pub start: Option<[usize; 2]>,
    pub end: Option<[usize; 2]>,

    pub recipe: Option<Recipe>,
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

        self.recipe = None;
    }

    pub fn track_mouse(&mut self, offset: Vec2) {
        let index = <[f32; 2]>::from(offset).map(|x| x.floor() as usize);
        self.track_index(index);
    }

    pub fn track_index(&mut self, index: [usize; 2]) {
        if self.recipe.is_some() || index[0] >= Self::WIDTH || index[1] >= Self::HEIGHT {
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
                self.recipe = Some((&*self).into());
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

impl RotateCounterClockwise for Direction {
    fn rotate_counter_clockwise(&self) -> Self {
        match self {
            Direction::Right => Direction::Up,
            Direction::Up => Direction::Left,
            Direction::Left => Direction::Down,
            Direction::Down => Direction::Right,
        }
    }
}

impl RotateClockwise for Direction {
    fn rotate_clockwise(&self) -> Self {
        -self.rotate_counter_clockwise()
    }
}

#[derive(Clone, Debug)]
pub struct Recipe {
    pub contents: Array2<Option<Direction>>,
}

impl From<&LoopCrafting> for Recipe {
    fn from(crafting: &LoopCrafting) -> Self {
        if let Some(recipe) = &crafting.recipe {
            return recipe.clone();
        }

        let mut min_x = usize::MAX;
        let mut min_y = usize::MAX;
        let mut max_x = 0;
        let mut max_y = 0;

        for (x, column) in crafting.points.iter().enumerate() {
            for (y, point) in column.iter().enumerate() {
                if point.is_some() {
                    min_x = x.min(min_x);
                    min_y = y.min(min_y);
                    max_x = x.max(max_x);
                    max_y = y.max(max_y);
                }
            }
        }

        if min_x > max_x || min_y > max_y {
            unreachable!();
        }

        let size = [max_x - min_x + 1, max_y - min_y + 1];

        let mut contents = Array2::from_elem(size, None);

        for x in min_x..max_x + 1 {
            for y in min_y..max_y + 1 {
                contents[[x, y]] = crafting[[x, y]];
            }
        }

        Recipe { contents }
    }
}

impl PartialEq for Recipe {
    fn eq(&self, other: &Self) -> bool {
        if self.contents == other.contents {
            return true;
        }

        let mut other_contents = other.contents.clone();

        for _ in 0..3 {
            other_contents = other_contents.rotate_counter_clockwise();

            if self.contents == other_contents {
                return true;
            }
        }

        false
    }
}
