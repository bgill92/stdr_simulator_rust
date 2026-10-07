//! Bevy-free 2D simulation core: poses, footprints, occupancy grids, map and robot config loading.

mod config;
mod error;
mod footprint;
mod grid;
mod map;
mod pose;

pub use config::{
    Alphas, KinematicConfig, KinematicKind, LaserSpec, OdometryModel, RobotConfig, Sensor,
    SensorCommon, SensorConfig, SonarSpec, load_robot_config,
};
pub use error::CoreError;
pub use footprint::Footprint;
pub use grid::{OCCUPANCY_THRESHOLD, OccupancyGrid, Unknown};
pub use map::load_map;
pub use pose::{Point2D, Pose2D, Twist2D, angle_diff, normalize_angle};
