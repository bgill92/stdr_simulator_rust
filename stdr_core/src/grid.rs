use crate::error::CoreError;
use crate::pose::Pose2D;

/// Cells strictly above this value are occupied.
pub const OCCUPANCY_THRESHOLD: i8 = 70;

/// How unknown cells (`-1`) are treated. Collision passes `Solid`, sensors `Transparent` (C++ parity).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Unknown {
    Solid,
    Transparent,
}

/// Row-major occupancy grid, row 0 at the bottom (ROS `nav_msgs/OccupancyGrid` convention):
/// 0 = free, 100 = occupied, -1 = unknown.
#[derive(Clone, Debug, PartialEq)]
pub struct OccupancyGrid {
    width: u32,
    height: u32,
    resolution: f64,
    origin: Pose2D,
    data: Vec<i8>,
}

impl OccupancyGrid {
    /// The only constructor; invariants checked here so cell lookups need no per-call guards.
    pub fn new(
        width: u32,
        height: u32,
        resolution: f64,
        origin: Pose2D,
        data: Vec<i8>,
    ) -> Result<Self, CoreError> {
        if resolution.is_nan() || resolution <= 0.0 {
            return Err(CoreError::Invalid(format!(
                "grid resolution must be > 0, got {resolution}"
            )));
        }
        if width == 0 || height == 0 {
            return Err(CoreError::Invalid(format!(
                "grid must be non-empty, got {width}x{height}"
            )));
        }
        if data.len() as u64 != u64::from(width) * u64::from(height) {
            return Err(CoreError::Invalid(format!(
                "grid data has {} cells, expected {width}x{height}",
                data.len()
            )));
        }
        Ok(Self {
            width,
            height,
            resolution,
            origin,
            data,
        })
    }

    pub fn width(&self) -> u32 {
        self.width
    }

    pub fn height(&self) -> u32 {
        self.height
    }

    pub fn resolution(&self) -> f64 {
        self.resolution
    }

    pub fn origin(&self) -> Pose2D {
        self.origin
    }

    pub fn data(&self) -> &[i8] {
        &self.data
    }

    /// `as i32` truncates toward zero like the C++ `static_cast<int>`, so -0.5 cells lands in cell 0.
    pub fn world_to_cell(&self, x: f64, y: f64) -> (i32, i32) {
        (
            ((x - self.origin.x) / self.resolution) as i32,
            ((y - self.origin.y) / self.resolution) as i32,
        )
    }

    pub fn in_bounds(&self, (cx, cy): (i32, i32)) -> bool {
        u32::try_from(cx).is_ok_and(|x| x < self.width)
            && u32::try_from(cy).is_ok_and(|y| y < self.height)
    }

    /// `None` when out of bounds.
    pub fn at(&self, c: (i32, i32)) -> Option<i8> {
        self.in_bounds(c)
            .then(|| self.data[c.1 as usize * self.width as usize + c.0 as usize])
    }

    /// Out of bounds and occupied cells block; unknown cells block only under `Unknown::Solid`.
    pub fn is_blocked(&self, c: (i32, i32), unknown: Unknown) -> bool {
        match self.at(c) {
            None => true,
            Some(v) if v < 0 => unknown == Unknown::Solid,
            Some(v) => v > OCCUPANCY_THRESHOLD,
        }
    }
}
