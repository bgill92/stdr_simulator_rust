//! Range sensors over the occupancy grid. Unknown cells are transparent (C++ parity).

use std::f64::consts::PI;

use rand::Rng;
use rand_distr::{Distribution, Normal};

use crate::config::{Sensor, SensorConfig};
use crate::grid::{OccupancyGrid, Unknown};
use crate::pose::Pose2D;

/// REP-117 ranges: +Inf = no return or beyond `range_max`, -Inf = closer than `range_min`.
#[derive(Clone, PartialEq, Debug, Default)]
pub struct LaserScan {
    pub angle_min: f64,
    pub angle_max: f64,
    pub angle_increment: f64,
    pub range_min: f64,
    pub range_max: f64,
    pub ranges: Vec<f32>,
}

/// Minimum range over the cone, REP-117 infinities as for `LaserScan`.
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct SonarScan {
    pub range: f64,
}

#[derive(Clone, PartialEq, Debug)]
pub enum Measurement {
    Laser(LaserScan),
    Sonar(SonarScan),
}

impl Measurement {
    pub fn as_laser(&self) -> Option<&LaserScan> {
        match self {
            Measurement::Laser(s) => Some(s),
            Measurement::Sonar(_) => None,
        }
    }

    pub fn as_sonar(&self) -> Option<&SonarScan> {
        match self {
            Measurement::Sonar(s) => Some(s),
            Measurement::Laser(_) => None,
        }
    }
}

/// One reading of `sensor` mounted at `world_pose`; `None` for a camera, whose image the app
/// renders (core holds no render types).
pub fn simulate(
    sensor: &Sensor,
    world_pose: Pose2D,
    grid: &OccupancyGrid,
    rng: &mut (impl Rng + ?Sized),
) -> Option<Measurement> {
    let res = grid.resolution();
    let origin = grid.grid_coords(world_pose.x, world_pose.y);
    let noise_std = sensor.common.noise_std;
    Some(match sensor.kind {
        SensorConfig::Laser(spec) => {
            let increment = if spec.num_rays > 1 {
                (spec.max_angle - spec.min_angle) / f64::from(spec.num_rays - 1)
            } else {
                0.0
            };
            let max_steps = (spec.max_range / res) as i32;
            let ranges = (0..spec.num_rays.max(0))
                .map(|i| {
                    let angle = world_pose.theta + spec.min_angle + f64::from(i) * increment;
                    let hit = grid.raycast(origin, angle, max_steps, Unknown::Transparent);
                    finish(hit, res, spec.min_range, spec.max_range, noise_std, rng) as f32
                })
                .collect();
            Measurement::Laser(LaserScan {
                angle_min: spec.min_angle,
                angle_max: spec.max_angle,
                angle_increment: increment,
                range_min: spec.min_range,
                range_max: spec.max_range,
                ranges,
            })
        }
        SensorConfig::Sonar(spec) => {
            let max_steps = (spec.max_range / res) as i32;
            let half = spec.cone_angle / 2.0;
            let mut hit: Option<i32> = None;
            // Float-accumulated 1° sweep: the accumulation decides the ray count (C++ parity).
            let mut a = -half;
            while a <= half {
                if let Some(step) = grid.raycast(
                    origin,
                    world_pose.theta + a,
                    max_steps,
                    Unknown::Transparent,
                ) {
                    hit = Some(hit.map_or(step, |h| h.min(step)));
                }
                a += PI / 180.0;
            }
            Measurement::Sonar(SonarScan {
                range: finish(hit, res, spec.min_range, spec.max_range, noise_std, rng),
            })
        }
        SensorConfig::Camera(_) => return None,
    })
}

/// Raycast steps → REP-117 range. No hit stays +Inf untouched by noise (no physical return);
/// otherwise add zero-mean noise, then `< min` → -Inf and `>= max` → +Inf.
fn finish(
    hit: Option<i32>,
    res: f64,
    min: f64,
    max: f64,
    noise_std: f64,
    rng: &mut (impl Rng + ?Sized),
) -> f64 {
    let Some(step) = hit else {
        return f64::INFINITY;
    };
    let mut range = f64::from(step) * res;
    if noise_std > 0.0 {
        range += Normal::new(0.0, noise_std).map_or(0.0, |n| n.sample(rng));
    }
    if range < min {
        f64::NEG_INFINITY
    } else if range >= max {
        f64::INFINITY
    } else {
        range
    }
}
