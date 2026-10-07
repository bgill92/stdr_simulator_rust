//! Bevy-free 2D simulation core: poses, footprints, occupancy grids, map and robot config
//! loading, kinematics, collision, range sensors, rate scheduling and the simulation engine.

mod collision;
mod config;
mod engine;
mod error;
mod footprint;
mod grid;
mod map;
mod motion;
mod pose;
mod scheduler;
mod sensors;

pub use collision::path_collides;
pub use config::{
    AlphaRow, Alphas, CameraSpec, KinematicConfig, KinematicKind, LaserSpec, OdometryModel,
    RobotConfig, Sensor, SensorCommon, SensorConfig, SonarSpec, load_robot_config,
};
pub use engine::{RobotId, RobotRuntime, RobotState, SimulationEngine};
pub use error::CoreError;
pub use footprint::Footprint;
pub use grid::{OCCUPANCY_THRESHOLD, OccupancyGrid, Unknown};
pub use map::load_map;
pub use motion::{OdometryVariance, integrate, odometry_variance, perturb};
pub use pose::{Point2D, Pose2D, Twist2D, angle_diff, normalize_angle};
pub use scheduler::{RateScheduler, SchedulingMode};
pub use sensors::{LaserScan, Measurement, SonarScan, simulate};
