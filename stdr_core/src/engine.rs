//! The simulation: map, robots, sim time and the one RNG.

use std::collections::BTreeMap;
use std::fmt;
use std::str::FromStr;

use rand::SeedableRng;
use rand::rngs::StdRng;

use crate::collision::path_collides;
use crate::config::RobotConfig;
use crate::error::CoreError;
use crate::grid::OccupancyGrid;
use crate::motion::{integrate, perturb};
use crate::pose::{Pose2D, Twist2D};
use crate::scheduler::{RateScheduler, SchedulingMode, check_step_dt};
use crate::sensors::{Measurement, simulate};

/// Spawn-order id; displays as `robot{n}` (C++ robot names), starting at `robot0`.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Debug)]
pub struct RobotId(u32);

impl fmt::Display for RobotId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "robot{}", self.0)
    }
}

impl FromStr for RobotId {
    type Err = CoreError;

    fn from_str(s: &str) -> Result<Self, CoreError> {
        s.strip_prefix("robot")
            .and_then(|n| n.parse().ok())
            .map(RobotId)
            .ok_or_else(|| CoreError::Invalid(format!("not a robot id: '{s}'")))
    }
}

#[derive(Clone, Copy, PartialEq, Debug, Default)]
pub struct RobotState {
    /// Ground truth.
    pub pose: Pose2D,
    /// The robot's dead-reckoned belief: integrates the clean command, never sees walls.
    pub odom_pose: Pose2D,
    pub cmd_vel: Twist2D,
}

#[derive(Clone, Debug)]
pub struct RobotRuntime {
    pub config: RobotConfig,
    /// Spawn pose; `reset` returns here.
    pub initial_pose: Pose2D,
    pub state: RobotState,
    /// Latest measurement per sensor index; `None` until it first fires with a map loaded.
    pub data: Vec<Option<Measurement>>,
    /// Whether the last step's motion was blocked.
    pub collided: bool,
    scheduler: RateScheduler,
}

impl RobotRuntime {
    pub fn sensor_index(&self, frame_id: &str) -> Option<usize> {
        self.config
            .sensors
            .iter()
            .position(|s| s.common.frame_id == frame_id)
    }

    /// Panics if `i` is not a sensor index.
    pub fn sensor_world_pose(&self, i: usize) -> Pose2D {
        self.state.pose * self.config.sensors[i].common.pose
    }
}

pub struct SimulationEngine {
    map: Option<OccupancyGrid>,
    map_revision: u64,
    robots: BTreeMap<RobotId, RobotRuntime>,
    next_id: u32,
    step_dt: f64,
    elapsed: f64,
    ticks: u64,
    mode: SchedulingMode,
    rng: StdRng,
}

impl SimulationEngine {
    /// `seed: None` seeds from OS entropy. The one RNG drives motion and sensor noise.
    pub fn new(step_dt: f64, seed: Option<u64>) -> Result<Self, CoreError> {
        check_step_dt(step_dt)?;
        Ok(Self {
            map: None,
            map_revision: 0,
            robots: BTreeMap::new(),
            next_id: 0,
            step_dt,
            elapsed: 0.0,
            ticks: 0,
            mode: SchedulingMode::default(),
            rng: seed.map_or_else(rand::make_rng, StdRng::seed_from_u64),
        })
    }

    pub fn step_dt(&self) -> f64 {
        self.step_dt
    }

    /// Takes effect from the next `step`; sim time stays continuous.
    pub fn set_step_dt(&mut self, dt: f64) -> Result<(), CoreError> {
        check_step_dt(dt)?;
        for r in self.robots.values_mut() {
            r.scheduler.set_step_dt(dt)?;
        }
        self.step_dt = dt;
        Ok(())
    }

    /// Seconds simulated since construction or the last `reset`.
    pub fn sim_time(&self) -> f64 {
        self.elapsed
    }

    pub fn ticks(&self) -> u64 {
        self.ticks
    }

    /// Applies to robots spawned afterwards (C++ parity).
    pub fn set_scheduling_mode(&mut self, m: SchedulingMode) {
        self.mode = m;
    }

    pub fn set_map(&mut self, g: OccupancyGrid) {
        self.map = Some(g);
        self.map_revision += 1;
    }

    pub fn map(&self) -> Option<&OccupancyGrid> {
        self.map.as_ref()
    }

    /// Bumped by every `set_map`.
    pub fn map_revision(&self) -> u64 {
        self.map_revision
    }

    /// `pose` overrides `cfg.initial_pose` and becomes the reset pose.
    pub fn spawn(&mut self, cfg: RobotConfig, pose: Pose2D) -> RobotId {
        let mut scheduler = RateScheduler::new(self.step_dt).expect("engine step_dt is positive");
        for (i, s) in cfg.sensors.iter().enumerate() {
            scheduler.set_rate(i, s.common.frequency, self.mode);
        }
        let id = RobotId(self.next_id);
        self.next_id += 1;
        self.robots.insert(
            id,
            RobotRuntime {
                data: vec![None; cfg.sensors.len()],
                config: cfg,
                initial_pose: pose,
                state: RobotState {
                    pose,
                    odom_pose: pose,
                    cmd_vel: Twist2D::default(),
                },
                collided: false,
                scheduler,
            },
        );
        id
    }

    pub fn remove(&mut self, id: RobotId) -> bool {
        self.robots.remove(&id).is_some()
    }

    pub fn robot(&self, id: RobotId) -> Option<&RobotRuntime> {
        self.robots.get(&id)
    }

    /// In spawn order.
    pub fn robots(&self) -> impl Iterator<Item = (RobotId, &RobotRuntime)> {
        self.robots.iter().map(|(&id, r)| (id, r))
    }

    pub fn set_cmd_vel(&mut self, id: RobotId, t: Twist2D) -> bool {
        self.robots
            .get_mut(&id)
            .map(|r| r.state.cmd_vel = t)
            .is_some()
    }

    /// Moves truth and collapses odometry onto it (the belief did not observe the jump).
    pub fn teleport(&mut self, id: RobotId, p: Pose2D) -> bool {
        self.robots
            .get_mut(&id)
            .map(|r| {
                r.state.pose = p;
                r.state.odom_pose = p;
            })
            .is_some()
    }

    pub fn effective_rate(&self, id: RobotId, sensor_idx: usize) -> Option<f64> {
        self.robots.get(&id)?.scheduler.effective_rate(sensor_idx)
    }

    /// One tick of `step_dt`: noisy truth and clean odometry integrate the command; a blocked
    /// path holds truth but odometry still advances (wheels turn, encoders cannot see the wall);
    /// due sensors fire from the committed truth pose. Without a map nothing collides and no
    /// sensor fires.
    pub fn step(&mut self) {
        let dt = self.step_dt;
        for r in self.robots.values_mut() {
            let k = &r.config.kinematic;
            let pivot = r.config.center_of_rotation;
            let s = &mut r.state;
            let noisy =
                perturb(s.cmd_vel, k, dt, &mut self.rng).expect("engine step_dt is positive");
            let pose = integrate(k.kind, s.pose, noisy, dt, pivot);
            s.odom_pose = integrate(k.kind, s.odom_pose, s.cmd_vel, dt, pivot);
            r.collided = self
                .map
                .as_ref()
                .is_some_and(|g| path_collides(g, &r.config.footprint, s.pose, pose));
            if !r.collided {
                s.pose = pose;
            }
            for i in r.scheduler.tick() {
                if let Some(g) = &self.map {
                    let sensor = &r.config.sensors[i];
                    r.data[i] = Some(simulate(
                        sensor,
                        s.pose * sensor.common.pose,
                        g,
                        &mut self.rng,
                    ));
                }
            }
        }
        self.elapsed += dt;
        self.ticks += 1;
    }

    /// Every robot back to its spawn pose with zero command, no data and no collision; sim time
    /// to zero. Sensor schedules keep their phase (C++ parity).
    pub fn reset(&mut self) {
        for r in self.robots.values_mut() {
            r.state = RobotState {
                pose: r.initial_pose,
                odom_pose: r.initial_pose,
                cmd_vel: Twist2D::default(),
            };
            r.data.fill(None);
            r.collided = false;
        }
        self.elapsed = 0.0;
        self.ticks = 0;
    }
}
