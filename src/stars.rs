use std::{
    array,
    f32::consts::{SQRT_2, TAU},
};

use macroquad::{
    color::colors,
    rand,
    shapes::{self, DrawRectangleParams},
};
use nalgebra::{Isometry2, Point2, Vector2};
use ndarray::Array2;

pub const STAR_MAP_SIZE: f32 = 1000.0;

pub const STAR_MAP_BUCKET_SIZE: f32 = 10.0;

pub const STAR_DENSITY: f32 = 0.025;
pub const NUM_STARS: u64 = (STAR_MAP_SIZE * STAR_MAP_SIZE * STAR_DENSITY) as u64;

#[derive(Clone, Copy, Debug)]
pub struct Star {
    pub position: Isometry2<f32>,
}

pub fn from_seed(seed: u64) -> PointSet<Star> {
    rand::srand(seed);

    let mut stars = PointSet::new(
        [(STAR_MAP_SIZE / STAR_MAP_BUCKET_SIZE).ceil() as usize; 2],
        STAR_MAP_BUCKET_SIZE,
        [-STAR_MAP_SIZE / 2.0; 2].into(),
    );

    for _ in 0..NUM_STARS {
        stars.insert(Star::random()).unwrap();
    }

    stars
}

pub fn draw_stars_in_area(stars: &PointSet<Star>, area: [Point2<f32>; 2]) {
    let modular_area = area.map(|point| point.map(to_star_space));

    let [min_corner, max_corner] = modular_area.map(|corner| stars.index_of(corner).unwrap());

    let mut x = min_corner[0];

    loop {
        let mut y = min_corner[1];

        loop {
            for star in &stars.points[[x, y]] {
                star.draw(area);
            }

            if y == max_corner[1] {
                break;
            }

            y += 1;
            y %= stars.buckets()[1];
        }

        if x == max_corner[0] {
            break;
        }

        x += 1;
        x %= stars.buckets()[0];
    }
}

pub fn to_star_space(value: f32) -> f32 {
    (value + STAR_MAP_SIZE / 2.0).rem_euclid(STAR_MAP_SIZE) - STAR_MAP_SIZE / 2.0
}

impl Star {
    pub const SIZE: f32 = 1.0 / 12.0;

    pub fn random() -> Self {
        Self {
            position: Isometry2::new(
                array::from_fn(|_| rand::gen_range(-STAR_MAP_SIZE / 2.0, STAR_MAP_SIZE / 2.0))
                    .into(),
                rand::gen_range(0.0, TAU),
            ),
        }
    }

    pub fn draw(self, area: [Point2<f32>; 2]) {
        let position = Point2::from(self);

        let offset = position - area[0];

        let offset = offset.map(to_star_space);

        let position = area[0] + offset;

        if position.x + Self::SIZE / SQRT_2 < area[0].x
            || position.y + Self::SIZE / SQRT_2 < area[0].y
            || position.x - Self::SIZE / SQRT_2 > area[1].x
            || position.y - Self::SIZE / SQRT_2 > area[1].y
        {
            return;
        }

        #[allow(clippy::unnecessary_cast)]
        shapes::draw_rectangle_ex(
            position.x as f32,
            position.y as f32,
            Self::SIZE as f32,
            Self::SIZE as f32,
            DrawRectangleParams {
                offset: [0.5, 0.5].into(),
                rotation: self.position.rotation.angle() as f32,
                color: colors::WHITE,
            },
        );
    }
}

impl From<Isometry2<f32>> for Star {
    fn from(position: Isometry2<f32>) -> Self {
        Self { position }
    }
}

impl From<Star> for Point2<f32> {
    fn from(star: Star) -> Self {
        star.position.translation.vector.into()
    }
}

impl From<Star> for Isometry2<f32> {
    fn from(star: Star) -> Self {
        star.position
    }
}

#[derive(Clone, Debug)]
pub struct PointSet<T> {
    pub points: Array2<Vec<T>>,
    pub bucket_size: f32,
    pub offset: Vector2<f32>,
}

impl<T> PointSet<T> {
    pub const EDGE_MARGIN: f32 = 1e-6;

    #[must_use]
    pub fn new(buckets: [usize; 2], bucket_size: f32, offset: Vector2<f32>) -> Self {
        Self {
            points: Array2::from_shape_fn(buckets, |_| Vec::new()),
            bucket_size,
            offset,
        }
    }

    #[must_use]
    pub fn placeholder() -> Self {
        Self::new([0, 0], 0.0, Vector2::from([0.0, 0.0]))
    }

    #[must_use]
    pub fn insert(&mut self, point: T) -> Option<[usize; 2]>
    where
        T: Into<Point2<f32>> + Clone,
    {
        let index = self.index_of(point.clone().into())?;

        self.points[index].push(point);

        Some(index)
    }

    #[must_use]
    pub fn index_of(&self, position: Point2<f32>) -> Option<[usize; 2]> {
        if !self.check_bounds_of(position) {
            return None;
        }

        let position = position - self.offset;

        let index = position.map(|x| (x.max(0.0) / self.bucket_size) as usize);
        let index = array::from_fn(|i| index[i].min(self.buckets()[i] - 1));

        Some(index)
    }

    /// WARN: Will not be reliable if `radius` is bigger than `self.width()` or `self.height()`
    #[must_use]
    pub fn indecies_near_to(
        &self,
        position: Point2<f32>,
        radius: f32,
    ) -> Option<impl Iterator<Item = [usize; 2]>> {
        if !self.is_within_radius(position, radius) {
            return None;
        }

        let offset = Vector2::from([radius; 2]);
        let offset_perp = Vector2::from([-radius, radius]);

        let corners = [
            self.index_of(position - offset),
            self.index_of(position + offset),
            self.index_of(position - offset_perp),
            self.index_of(position + offset_perp),
        ]
        .into_iter()
        .flatten();

        let corners_x = corners.clone().map(|[x, _]| x);
        let corners_y = corners.clone().map(|[_, y]| y);

        let min_x = corners_x.clone().min();
        let max_x = corners_x.max();
        let min_y = corners_y.clone().min();
        let max_y = corners_y.max();

        let min_x = min_x.unwrap_or(0);
        let max_x = max_x.unwrap_or(self.buckets()[0] - 1);
        let min_y = min_y.unwrap_or(0);
        let max_y = max_y.unwrap_or(self.buckets()[1] - 1);

        #[allow(clippy::range_plus_one)]
        let [x_range, y_range] = [min_x..max_x + 1, min_y..max_y + 1];

        Some(x_range.flat_map(move |x| y_range.clone().map(move |y| [x, y])))
    }

    #[must_use]
    pub fn is_within_radius(&self, position: Point2<f32>, radius: f32) -> bool {
        let position = position - self.offset;

        position.x >= -radius
            && position.y >= -radius
            && position.x <= self.width() + radius
            && position.y <= self.height() + radius
    }

    #[must_use]
    pub fn check_bounds_of(&self, position: Point2<f32>) -> bool {
        self.is_within_radius(position, Self::EDGE_MARGIN)
    }

    pub fn clear(&mut self) {
        for bucket in &mut self.points {
            bucket.clear();
        }
    }

    pub fn iter_all(&self) -> impl Iterator<Item = &T> {
        self.points.iter().flatten()
    }

    pub fn iter_all_mut(&mut self) -> impl Iterator<Item = &mut T> {
        self.points.iter_mut().flatten()
    }

    #[must_use]
    pub fn iter_near(
        &self,
        position: Point2<f32>,
        radius: f32,
    ) -> Option<impl Iterator<Item = &T>> {
        Some(
            self.indecies_near_to(position, radius)?
                .flat_map(|index| &self.points[index]),
        )
    }

    #[must_use]
    pub fn buckets(&self) -> [usize; 2] {
        self.points.dim().into()
    }

    #[must_use]
    pub fn size(&self) -> Vector2<f32> {
        Vector2::from(<[usize; 2]>::from(self.points.dim())).map(|x| x as f32) * self.bucket_size
    }

    #[must_use]
    pub fn width(&self) -> f32 {
        self.size().x
    }

    #[must_use]
    pub fn height(&self) -> f32 {
        self.size().y
    }
}

pub trait AddOffset {
    fn add_offset(&mut self, offset: Vector2<f32>);
}

impl<T> AddOffset for PointSet<T>
where
    T: AddOffset,
{
    fn add_offset(&mut self, offset: Vector2<f32>) {
        self.offset += offset;

        for bucket in &mut self.points {
            for point in bucket {
                point.add_offset(offset);
            }
        }
    }
}
