//! C++ `test_world_model.cpp` (robots/map), `test_simulation_engine.cpp` (minus the Tf/Odom
//! stream cases) and the reset/teleport/odometry cases of `test_standalone_backend.cpp`, all on
//! the merged `SimulationEngine`. C++ `step(dt)` is `step()` at the constructed `step_dt`;
//! `set_robot_pose` is `teleport`; `clear_sensor_data` is part of `reset`; robot names are `RobotId`.
#![allow(non_snake_case)]

use approx::assert_abs_diff_eq;
use std::f64::consts::PI;
use std::path::Path;
use stdr_core::{
    Footprint, KinematicConfig, KinematicKind, LaserSpec, OccupancyGrid, OdometryModel, Point2D,
    Pose2D, RobotConfig, RobotId, SchedulingMode, Sensor, SensorCommon, SensorConfig,
    SimulationEngine, Twist2D, integrate, load_robot_config,
};

const fn pose(x: f64, y: f64, theta: f64) -> Pose2D {
    Pose2D { x, y, theta }
}

const fn twist(linear_x: f64, linear_y: f64, angular_z: f64) -> Twist2D {
    Twist2D {
        linear_x,
        linear_y,
        angular_z,
    }
}

fn engine() -> SimulationEngine {
    SimulationEngine::new(0.1, Some(0)).unwrap()
}

fn minimal_robot() -> RobotConfig {
    RobotConfig {
        footprint: Footprint::Circle { radius: 0.05 },
        ..RobotConfig::default()
    }
}

/// One 5-ray laser, 0.5 m range, firing every tick (frequency 0).
fn robot_with_laser() -> RobotConfig {
    let mut cfg = minimal_robot();
    cfg.sensors.push(Sensor {
        common: SensorCommon::default(),
        kind: SensorConfig::Laser(LaserSpec {
            min_angle: -PI / 2.0,
            max_angle: PI / 2.0,
            min_range: 0.05,
            max_range: 0.5,
            num_rays: 5,
        }),
    });
    cfg
}

fn with_laser_hz(mut cfg: RobotConfig, hz: f64) -> RobotConfig {
    cfg.sensors[0].common.frequency = hz;
    cfg
}

fn velocity_noise(mut cfg: RobotConfig) -> RobotConfig {
    cfg.kinematic = KinematicConfig {
        odometry: OdometryModel::Velocity,
        alphas: stdr_core::Alphas([[0.05, 0.0, 0.0], [0.0; 3], [0.0, 0.0, 0.05], [0.0; 3]]),
        ..cfg.kinematic
    };
    cfg
}

/// `w × h` at 0.1 m, free except the listed occupied columns.
fn map(w: u32, h: u32, wall_cols: &[u32]) -> OccupancyGrid {
    let data = (0..w * h)
        .map(|i| if wall_cols.contains(&(i % w)) { 100 } else { 0 })
        .collect();
    OccupancyGrid::new(w, h, 0.1, Pose2D::default(), data).unwrap()
}

fn free_map() -> OccupancyGrid {
    map(20, 20, &[])
}

fn wall_map() -> OccupancyGrid {
    map(20, 20, &[6])
}

fn resource_robot(name: &str) -> RobotConfig {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("../stdr_resources/resources");
    load_robot_config(dir.join("robots").join(name), &dir)
        .unwrap()
        .0
}

fn state(e: &SimulationEngine, id: RobotId) -> stdr_core::RobotState {
    e.robot(id).unwrap().state
}

/// Times the laser fired over `n` ticks, seen as a changed reading: the robot drives straight at
/// a wall one cell per tick, so every fresh scan differs from the last.
fn laser_fires(cfg: RobotConfig, n: usize) -> usize {
    let mut e = engine();
    e.set_map(map(40, 10, &[39]));
    let mut cfg = cfg;
    cfg.sensors[0].kind = SensorConfig::Laser(LaserSpec {
        min_angle: 0.0,
        max_angle: 0.0,
        min_range: 0.05,
        max_range: 5.0,
        num_rays: 1,
    });
    let id = e.spawn(cfg, pose(0.15, 0.45, 0.0));
    e.set_cmd_vel(id, twist(1.0, 0.0, 0.0));
    let mut last = None;
    let mut fires = 0;
    for _ in 0..n {
        e.step();
        let now = e.robot(id).unwrap().data[0].clone();
        if now.is_some() && now != last {
            fires += 1;
        }
        last = now;
    }
    fires
}

mod WorldModelTest {
    use super::*;

    #[test]
    fn NoMapReturnsNullptr() {
        assert!(engine().map().is_none());
    }

    #[test]
    fn SetAndGetMap() {
        let mut e = engine();
        e.set_map(map(5, 5, &[]));
        assert_eq!(e.map().unwrap().width(), 5);
    }

    #[test]
    fn AddRobotAssignsName() {
        let mut e = engine();
        assert_eq!(
            e.spawn(minimal_robot(), Pose2D::default()).to_string(),
            "robot0"
        );
    }

    #[test]
    fn AddMultipleRobotsSequentialNaming() {
        let mut e = engine();
        let a = e.spawn(minimal_robot(), Pose2D::default());
        let b = e.spawn(minimal_robot(), Pose2D::default());
        assert_eq!(
            (a.to_string().as_str(), b.to_string().as_str()),
            ("robot0", "robot1")
        );
        let ids: Vec<_> = e.robots().map(|(id, _)| id).collect();
        assert_eq!(ids, [a, b]);
    }

    #[test]
    fn RemoveRobot() {
        let mut e = engine();
        let id = e.spawn(minimal_robot(), Pose2D::default());
        assert!(e.robot(id).is_some());
        assert!(e.remove(id));
        assert!(e.robot(id).is_none());
        assert!(!e.remove(id));
    }

    #[test]
    fn SetRobotPose() {
        let mut e = engine();
        let id = e.spawn(minimal_robot(), Pose2D::default());
        assert!(e.teleport(id, pose(3.0, 4.0, 1.5)));
        assert_eq!(state(&e, id).pose, pose(3.0, 4.0, 1.5));
    }

    #[test]
    fn SpawnOverridesConfigInitialPose() {
        let mut e = engine();
        let mut cfg = minimal_robot();
        cfg.initial_pose = pose(9.0, 9.0, 9.0);
        let id = e.spawn(cfg, pose(1.0, 2.0, 0.5));
        let r = e.robot(id).unwrap();
        assert_eq!(r.config.initial_pose, pose(1.0, 2.0, 0.5));
        assert_eq!(r.config.initial_pose, r.initial_pose);
    }

    #[test]
    fn AddRobotInitializesOdomPoseToInitialPose() {
        let mut e = engine();
        let id = e.spawn(minimal_robot(), pose(1.0, 2.0, 0.5));
        assert_eq!(state(&e, id).odom_pose, pose(1.0, 2.0, 0.5));
    }

    /// Odometry is first driven away from truth (collision holds truth), then teleport collapses it.
    #[test]
    fn SetRobotPoseResetsOdomPose() {
        let mut e = engine();
        e.set_map(wall_map());
        let id = e.spawn(minimal_robot(), pose(0.55, 1.0, 0.0));
        e.set_cmd_vel(id, twist(1.0, 0.0, 0.0));
        e.step();
        assert_ne!(state(&e, id).pose, state(&e, id).odom_pose);
        e.teleport(id, pose(3.0, 4.0, 1.5));
        assert_eq!(state(&e, id).odom_pose, pose(3.0, 4.0, 1.5));
    }

    /// C++ committed truth and odometry through `set_robot_poses`; here `step` does, and a
    /// collision is what separates them.
    #[test]
    fn SetRobotPosesCommitsTruthAndOdomIndependently() {
        let mut e = engine();
        e.set_map(wall_map());
        let start = pose(0.55, 1.0, 0.0);
        let id = e.spawn(minimal_robot(), start);
        e.set_cmd_vel(id, twist(1.0, 0.0, 0.0));
        e.step();
        assert_eq!(state(&e, id).pose, start);
        assert_eq!(
            state(&e, id).odom_pose,
            integrate(
                KinematicKind::Ideal,
                start,
                twist(1.0, 0.0, 0.0),
                0.1,
                Point2D::default()
            )
        );
    }

    #[test]
    fn SetRobotCmdVel() {
        let mut e = engine();
        let id = e.spawn(minimal_robot(), Pose2D::default());
        assert!(e.set_cmd_vel(id, twist(1.0, 0.5, 0.2)));
        assert_eq!(state(&e, id).cmd_vel, twist(1.0, 0.5, 0.2));
        let gone: RobotId = "robot9".parse().unwrap();
        assert!(!e.set_cmd_vel(gone, twist(1.0, 0.0, 0.0)));
        assert!(!e.teleport(gone, Pose2D::default()));
    }
}

mod SimulationEngineTest {
    use super::*;

    #[test]
    fn SpawnRobotReturnsName() {
        let mut e = engine();
        let id = e.spawn(minimal_robot(), pose(1.0, 1.0, 0.0));
        assert_eq!(id.to_string(), "robot0");
        assert_eq!("robot0".parse::<RobotId>().unwrap(), id);
        assert!("bot0".parse::<RobotId>().is_err());
    }

    #[test]
    fn DeleteRobot() {
        let mut e = engine();
        let id = e.spawn(minimal_robot(), pose(1.0, 1.0, 0.0));
        e.step();
        assert!(e.robot(id).is_some());
        e.remove(id);
        assert!(e.robot(id).is_none());
    }

    /// No per-robot clear: `reset` on an empty engine is the no-op.
    #[test]
    fn ClearSensorDataUnknownRobotIsNoop() {
        let mut e = engine();
        e.reset();
        assert_eq!(e.robots().count(), 0);
    }

    #[test]
    fn StepUpdatesPose() {
        let mut e = engine();
        e.set_map(free_map());
        let id = e.spawn(minimal_robot(), pose(1.0, 1.0, 0.0));
        e.set_cmd_vel(id, twist(1.0, 0.0, 0.0));
        e.step();
        assert!(state(&e, id).pose.x > 1.0);
        assert_eq!(state(&e, id).pose.y, 1.0);
    }

    #[test]
    fn StepWithMapRunsSensors() {
        let mut e = engine();
        e.set_map(free_map());
        let id = e.spawn(robot_with_laser(), pose(1.0, 1.0, 0.0));
        e.step();
        let data = &e.robot(id).unwrap().data;
        assert_eq!(data.len(), 1);
        assert_eq!(
            data[0].as_ref().unwrap().as_laser().unwrap().ranges.len(),
            5
        );
    }

    #[test]
    fn ClearSensorDataResetsToFreshlySpawnedState() {
        let mut e = engine();
        e.set_map(wall_map());
        let id = e.spawn(robot_with_laser(), pose(0.55, 1.0, 0.0));
        e.set_cmd_vel(id, twist(1.0, 0.0, 0.0));
        e.step();
        let r = e.robot(id).unwrap();
        assert!(r.collided && r.data[0].is_some());
        e.reset();
        let r = e.robot(id).unwrap();
        assert!(!r.collided);
        assert_eq!(r.data, [None]);
    }

    #[test]
    fn StepWithoutMapSkipsRaycastSensors() {
        let mut e = engine();
        let id = e.spawn(robot_with_laser(), pose(0.5, 0.5, 0.0));
        e.step();
        assert_eq!(e.robot(id).unwrap().data, [None]);
    }

    #[test]
    fn DefaultCenterOfRotationPureRotationStaysInPlace() {
        let mut e = engine();
        let id = e.spawn(minimal_robot(), pose(1.0, 2.0, 0.0));
        e.set_cmd_vel(id, twist(0.0, 0.0, 1.0));
        e.step();
        let p = state(&e, id).pose;
        assert_abs_diff_eq!(p.x, 1.0, epsilon = 1e-9);
        assert_abs_diff_eq!(p.y, 2.0, epsilon = 1e-9);
        assert_abs_diff_eq!(p.theta, 0.1, epsilon = 1e-9);
    }

    #[test]
    fn OffsetCenterOfRotationPivotsPoseAboutConfiguredPoint() {
        let mut e = engine();
        let cfg = RobotConfig {
            footprint: Footprint::Circle { radius: 1.0 },
            center_of_rotation: Point2D { x: 1.0, y: 0.0 },
            ..minimal_robot()
        };
        let id = e.spawn(cfg, Pose2D::default());
        e.set_cmd_vel(id, twist(0.0, 0.0, 1.0));
        for _ in 0..10 {
            e.step();
        }
        let p = state(&e, id).pose;
        assert!(p.x.hypot(p.y) > 1e-6);
        assert_abs_diff_eq!((p.x - 1.0).hypot(p.y), 1.0, epsilon = 1e-9);
    }

    #[test]
    fn RateSchedulerDefaultRunsEverySensorEveryTick() {
        assert_eq!(laser_fires(robot_with_laser(), 5), 5);
    }

    #[test]
    fn RateSchedulerLaserAtHalfTickRate() {
        assert_eq!(laser_fires(with_laser_hz(robot_with_laser(), 5.0), 20), 10);
    }

    /// C++ checked the Tf stream; re-keyed to the laser.
    #[test]
    fn RateSchedulerChangeStepDtRecomputes() {
        let mut e = engine();
        let id = e.spawn(with_laser_hz(robot_with_laser(), 50.0), pose(0.5, 0.5, 0.0));
        assert_eq!(e.effective_rate(id, 0), Some(10.0));
        e.set_step_dt(0.02).unwrap();
        assert_eq!(e.effective_rate(id, 0), Some(50.0));
        let id = e.spawn(with_laser_hz(robot_with_laser(), 20.0), pose(0.5, 0.5, 0.0));
        assert_abs_diff_eq!(
            e.effective_rate(id, 0).unwrap(),
            1.0 / (3.0 * 0.02),
            epsilon = 1e-9
        );
    }

    #[test]
    fn RobotRemovalCleansScheduler() {
        let mut e = engine();
        let id = e.spawn(robot_with_laser(), pose(0.5, 0.5, 0.0));
        e.step();
        assert!(e.effective_rate(id, 0).unwrap() > 0.0);
        e.remove(id);
        e.step();
        assert_eq!(e.effective_rate(id, 0), None);
    }

    #[test]
    fn SensorDataPresizedOnSpawn() {
        let mut e = engine();
        let id = e.spawn(robot_with_laser(), pose(1.0, 1.0, 0.0));
        assert_eq!(e.robot(id).unwrap().data, [None]);
    }

    #[test]
    fn StepDtAccessorReflectsValue() {
        let mut e = SimulationEngine::new(0.05, None).unwrap();
        assert_eq!(e.step_dt(), 0.05);
        e.set_step_dt(0.02).unwrap();
        assert_eq!(e.step_dt(), 0.02);
        assert!(e.set_step_dt(0.0).is_err());
        assert_eq!(e.step_dt(), 0.02);
    }

    #[test]
    fn InvalidStepDtThrows() {
        assert!(SimulationEngine::new(0.0, None).is_err());
        assert!(SimulationEngine::new(-0.1, None).is_err());
    }

    #[test]
    fn SchedulingModeAppliesToSpawnedRobots() {
        let mut e = engine();
        e.set_scheduling_mode(SchedulingMode::Accumulator);
        let id = e.spawn(with_laser_hz(robot_with_laser(), 7.0), pose(1.0, 1.0, 0.0));
        assert_abs_diff_eq!(e.effective_rate(id, 0).unwrap(), 7.0, epsilon = 1e-9);
    }

    #[test]
    fn SpawnInitializesOdomPoseToInitialPose() {
        let mut e = engine();
        let id = e.spawn(minimal_robot(), pose(1.0, 2.0, 0.3));
        assert_eq!(state(&e, id).odom_pose, pose(1.0, 2.0, 0.3));
    }

    #[test]
    fn PerfectOdometryMatchesTruePoseExactly() {
        let mut e = engine();
        let id = e.spawn(minimal_robot(), Pose2D::default());
        e.set_cmd_vel(id, twist(0.5, 0.0, 0.3));
        for _ in 0..20 {
            e.step();
        }
        assert_eq!(state(&e, id).pose, state(&e, id).odom_pose);
    }

    #[test]
    fn VelocityOdometryMatchesCleanIntegrationAndPoseDiverges() {
        let mut e = engine();
        let id = e.spawn(velocity_noise(minimal_robot()), Pose2D::default());
        let cmd = twist(1.0, 0.0, 0.3);
        e.set_cmd_vel(id, cmd);
        let mut expected = Pose2D::default();
        let mut diverged = false;
        for _ in 0..30 {
            e.step();
            expected = integrate(KinematicKind::Ideal, expected, cmd, 0.1, Point2D::default());
            let s = state(&e, id);
            diverged |=
                (s.pose.x - s.odom_pose.x).abs() > 1e-9 || (s.pose.y - s.odom_pose.y).abs() > 1e-9;
        }
        assert_eq!(state(&e, id).odom_pose, expected);
        assert!(diverged);
    }

    #[test]
    fn CollisionHoldsTruePoseButOdomKeepsAdvancing() {
        let mut e = engine();
        e.set_map(wall_map());
        let cfg = RobotConfig {
            footprint: Footprint::Circle { radius: 0.03 },
            ..minimal_robot()
        };
        let start = pose(0.55, 1.0, 0.0);
        let id = e.spawn(cfg, start);
        e.set_cmd_vel(id, twist(1.0, 0.0, 0.0));
        e.step();
        assert_eq!(state(&e, id).pose, start);
        assert!(state(&e, id).odom_pose.x > start.x);
        assert!(e.robot(id).unwrap().collided);
    }
}

mod StandaloneBackend {
    use super::*;

    #[test]
    fn ResetRestoresSpawnPoseAndClearsElapsedTime() {
        let mut e = engine();
        e.set_map(map(60, 60, &[]));
        let id = e.spawn(resource_robot("simple_robot.yaml"), pose(1.0, 2.0, 0.5));
        e.teleport(id, pose(3.0, 4.0, 1.0));
        e.set_cmd_vel(id, twist(0.5, 0.2, 0.3));
        for _ in 0..25 {
            e.step();
        }
        assert!(e.sim_time() > 0.0);
        e.reset();
        let s = state(&e, id);
        assert_eq!(s.pose, pose(1.0, 2.0, 0.5));
        assert_eq!(s.odom_pose, pose(1.0, 2.0, 0.5));
        assert_eq!(s.cmd_vel, Twist2D::default());
        assert_eq!(e.sim_time(), 0.0);
        assert!(e.robot(id).unwrap().data.iter().all(Option::is_none));
    }

    #[test]
    fn TeleportResetsOdomPose() {
        let mut e = engine();
        let id = e.spawn(
            resource_robot("simple_robot_noisy.yaml"),
            pose(1.0, 1.0, 0.0),
        );
        e.set_cmd_vel(id, twist(0.5, 0.0, 0.3));
        for _ in 0..10 {
            e.step();
        }
        e.teleport(id, pose(2.0, 3.0, 1.57));
        let s = state(&e, id);
        assert_eq!(s.odom_pose, s.pose);
        assert_eq!(s.pose, pose(2.0, 3.0, 1.57));
    }

    #[test]
    fn OdomPoseEqualsPoseUnderPerfectModel() {
        let mut e = engine();
        let id = e.spawn(resource_robot("simple_robot.yaml"), Pose2D::default());
        e.set_cmd_vel(id, twist(0.5, 0.0, 0.3));
        for _ in 0..25 {
            e.step();
        }
        assert_eq!(state(&e, id).pose, state(&e, id).odom_pose);
    }

    #[test]
    fn OdomPoseDivergesUnderVelocityModel() {
        let mut e = engine();
        let id = e.spawn(resource_robot("simple_robot_noisy.yaml"), Pose2D::default());
        e.set_cmd_vel(id, twist(0.5, 0.0, 0.3));
        for _ in 0..50 {
            e.step();
        }
        assert_ne!(state(&e, id).pose, state(&e, id).odom_pose);
    }
}

mod engine {
    use super::*;

    #[test]
    fn step_uses_stored_step_dt() {
        let mut e = SimulationEngine::new(0.05, None).unwrap();
        let id = e.spawn(minimal_robot(), Pose2D::default());
        e.set_cmd_vel(id, twist(1.0, 0.0, 0.0));
        e.step();
        assert_abs_diff_eq!(state(&e, id).pose.x, 0.05, epsilon = 1e-12);
        e.set_step_dt(0.2).unwrap();
        e.step();
        assert_abs_diff_eq!(state(&e, id).pose.x, 0.25, epsilon = 1e-12);
    }

    /// Motion noise and sensor noise come from the one seeded RNG.
    #[test]
    fn seeded_engine_is_deterministic() {
        let run = |seed| {
            let mut e = SimulationEngine::new(0.1, Some(seed)).unwrap();
            e.set_map(map(60, 60, &[59]));
            let mut cfg = velocity_noise(robot_with_laser());
            cfg.sensors[0].common.noise_std = 0.05;
            if let SensorConfig::Laser(spec) = &mut cfg.sensors[0].kind {
                spec.max_range = 6.0;
            }
            let id = e.spawn(cfg, pose(1.0, 3.0, 0.0));
            e.set_cmd_vel(id, twist(0.5, 0.0, 0.1));
            for _ in 0..30 {
                e.step();
            }
            let r = e.robot(id).unwrap();
            (r.state, r.data.clone())
        };
        assert_eq!(run(7), run(7));
        let (a, b) = (run(7), run(8));
        assert_ne!(a.0, b.0);
        assert_ne!(a.1, b.1);
    }

    #[test]
    fn sim_time_accumulates_across_step_dt_change() {
        let mut e = engine();
        for _ in 0..3 {
            e.step();
        }
        e.set_step_dt(0.05).unwrap();
        for _ in 0..2 {
            e.step();
        }
        assert_abs_diff_eq!(e.sim_time(), 0.4, epsilon = 1e-12);
        assert_eq!(e.ticks(), 5);
    }

    #[test]
    fn sensor_lookup_by_frame_id() {
        let mut e = engine();
        let id = e.spawn(
            resource_robot("simple_robot.yaml"),
            pose(1.0, 2.0, PI / 2.0),
        );
        let r = e.robot(id).unwrap();
        assert_eq!(r.sensor_index("laser_0"), Some(0));
        assert_eq!(r.sensor_index("laser_1"), None);
        // simple_robot.yaml mounts its laser at the body origin, turned backwards.
        let mount = r.config.sensors[0].common.pose;
        assert_eq!((mount.x, mount.y), (0.0, 0.0));
        assert!(mount.theta < -3.0);
        let p = r.sensor_world_pose(0);
        assert_abs_diff_eq!(p.x, 1.0, epsilon = 1e-12);
        assert_abs_diff_eq!(p.theta, PI / 2.0 + mount.theta, epsilon = 1e-12);
    }

    #[test]
    fn reset_zeroes_sim_time() {
        let mut e = engine();
        e.spawn(minimal_robot(), Pose2D::default());
        for _ in 0..4 {
            e.step();
        }
        e.reset();
        assert_eq!((e.sim_time(), e.ticks()), (0.0, 0));
    }
}
