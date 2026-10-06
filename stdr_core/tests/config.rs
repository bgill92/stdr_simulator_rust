//! C++ `test_config_loader.cpp` `LoadRobotConfig.*` plus the config rows of PLAN.md's fix/quirk table.
#![allow(non_snake_case)]

use std::path::{Path, PathBuf};

use stdr_core::{
    Footprint, KinematicKind, OdometryModel, Point2D, Pose2D, RobotConfig, SensorConfig,
    load_robot_config,
};

fn fixtures() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures")
}

fn load(name: &str) -> RobotConfig {
    let (cfg, warnings) =
        load_robot_config(fixtures().join(name), fixtures()).unwrap_or_else(|e| panic!("{e}"));
    assert!(warnings.is_empty(), "{warnings:?}");
    cfg
}

fn load_err(name: &str) -> String {
    load_robot_config(fixtures().join(name), fixtures())
        .unwrap_err()
        .to_string()
}

/// `$STDR_RESOURCES_DIR` (C++ convention: `stdr_resources/resources`), else the copy vendored in this repo.
fn resources() -> PathBuf {
    std::env::var_os("STDR_RESOURCES_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|| {
            Path::new(env!("CARGO_MANIFEST_DIR")).join("../stdr_resources/resources")
        })
}

fn load_shipped(name: &str) -> (RobotConfig, Vec<String>) {
    load_robot_config(resources().join("robots").join(name), resources())
        .unwrap_or_else(|e| panic!("{name}: {e}"))
}

fn laser(cfg: &RobotConfig, i: usize) -> stdr_core::LaserSpec {
    match cfg.sensors[i].kind {
        SensorConfig::Laser(l) => l,
        other => panic!("sensor {i} is {other:?}"),
    }
}

mod LoadRobotConfig {
    use super::*;

    #[test]
    fn SimpleRobotParsesInitialPose() {
        assert_eq!(
            load("simple_robot.yaml").initial_pose,
            Pose2D {
                x: 3.0,
                y: 2.0,
                theta: 1.57
            }
        );
    }

    #[test]
    fn SimpleRobotParsesFootprintRadius() {
        assert_eq!(
            load("simple_robot.yaml").footprint,
            Footprint::Circle { radius: 0.05 }
        );
    }

    #[test]
    fn SimpleRobotLoadsLaserFromFile() {
        let cfg = load("simple_robot.yaml");
        assert_eq!(cfg.sensors.len(), 1);
        let l = laser(&cfg, 0);
        assert_eq!((l.max_range, l.min_range, l.num_rays), (4.09, 0.06, 667));
    }

    #[test]
    #[allow(clippy::approx_constant)] // The fixture's literal value, not pi.
    fn SimpleRobotInlinePoseOverridesFilepose() {
        assert_eq!(
            load("simple_robot.yaml").sensors[0].common.pose.theta,
            -3.1415
        );
    }

    #[test]
    fn SimpleRobotLoadsKinematic() {
        assert_eq!(
            load("simple_robot.yaml").kinematic.kind,
            KinematicKind::Ideal
        );
    }

    #[test]
    fn InlineOnlyLaserAndKinematic() {
        let cfg = load("robot_inline_only.yaml");
        assert_eq!(cfg.sensors.len(), 1);
        assert_eq!(laser(&cfg, 0).max_range, 8.0);
        assert_eq!(cfg.kinematic.kind, KinematicKind::Omni);
    }

    #[test]
    fn KinematicOdometryModelDefaultsToPerfect() {
        assert_eq!(
            load("simple_robot.yaml").kinematic.odometry,
            OdometryModel::Perfect
        );
    }

    #[test]
    fn KinematicOdometryModelParsedInline() {
        assert_eq!(
            load("robot_odometry_inline_velocity.yaml")
                .kinematic
                .odometry,
            OdometryModel::Velocity
        );
    }

    #[test]
    fn KinematicOdometryModelParsedFromFile() {
        assert_eq!(
            load("robot_odometry_file_velocity.yaml").kinematic.odometry,
            OdometryModel::Velocity
        );
    }

    #[test]
    fn InvalidOdometryModelReturnsError() {
        let err = load_err("robot_odometry_invalid.yaml");
        assert!(err.contains("odometry_model"), "{err}");
    }

    // C++ also checked `noise.enabled` and `noise.mean == 0.5`; both fields are dropped (`noise_std > 0` = on).
    #[test]
    fn LaserNoiseLoadedFromFile() {
        assert_eq!(load("simple_robot.yaml").sensors[0].common.noise_std, 0.05);
    }

    #[test]
    fn MissingFileReturnsError() {
        assert!(load_robot_config("/nonexistent/robot.yaml", "/nonexistent").is_err());
    }

    #[test]
    fn MissingRobotSpecificationsReturnsError() {
        let err = load_err("robot_bad.yaml");
        assert!(err.contains("robot_specifications"), "{err}");
    }

    #[test]
    fn BadSensorFilenameReturnsError() {
        let err = load_err("robot_bad_sensor.yaml");
        assert!(err.contains("nonexistent_laser.yaml"), "{err}");
    }

    #[test]
    fn FootprintPointsWrappedKeyParsed() {
        let Footprint::Polygon(pts) = load("robot_polygon.yaml").footprint else {
            panic!("not a polygon")
        };
        assert_eq!(pts.len(), 4);
        assert_eq!(pts[0], Point2D { x: 0.1, y: 0.2 });
        assert_eq!(pts[2], Point2D { x: -0.1, y: -0.2 });
    }

    #[test]
    fn FootprintPointsBareKeyParsed() {
        let Footprint::Polygon(pts) = load("robot_polygon_bare.yaml").footprint else {
            panic!("not a polygon")
        };
        assert_eq!(pts.len(), 3);
        assert_eq!(pts[0], Point2D { x: 0.3, y: 0.4 });
        assert_eq!(pts[1].x, -0.3);
    }

    #[test]
    fn CenterOfRotationDefaultsToOrigin() {
        assert_eq!(
            load("simple_robot.yaml").center_of_rotation,
            Point2D::default()
        );
    }

    #[test]
    fn CenterOfRotationInsidePolygonParsed() {
        assert_eq!(
            load("robot_cor_valid.yaml").center_of_rotation,
            Point2D { x: 0.05, y: 0.0 }
        );
    }

    #[test]
    fn CenterOfRotationOnPolygonEdgeAccepted() {
        assert_eq!(
            load("robot_cor_on_edge.yaml").center_of_rotation,
            Point2D { x: 0.1, y: 0.0 }
        );
    }

    #[test]
    fn CenterOfRotationOutsidePolygonReturnsError() {
        let err = load_err("robot_cor_outside_polygon.yaml");
        assert!(err.contains("center_of_rotation"), "{err}");
    }

    #[test]
    fn CenterOfRotationOutsideCircleReturnsError() {
        let err = load_err("robot_cor_outside_circle.yaml");
        assert!(err.contains("center_of_rotation"), "{err}");
    }

    #[test]
    fn SensorWithoutFrameIdGetsDefault() {
        assert_eq!(
            load("robot_frame_id.yaml").sensors[0].common.frame_id,
            "laser_0"
        );
    }

    #[test]
    fn SensorWithExplicitFrameIdIsPreserved() {
        assert_eq!(
            load("robot_frame_id.yaml").sensors[1].common.frame_id,
            "my_laser"
        );
    }

    #[test]
    fn SonarAutoIndexIsStableAcrossMixedNames() {
        let cfg = load("robot_frame_id.yaml");
        let ids: Vec<_> = cfg
            .sensors
            .iter()
            .map(|s| (s.kind.name(), s.common.frame_id.as_str()))
            .collect();
        assert_eq!(
            ids,
            [
                ("laser", "laser_0"),
                ("laser", "my_laser"),
                ("sonar", "named_sonar"),
                ("sonar", "sonar_1")
            ]
        );
    }
}

mod config {
    use super::*;

    #[test]
    fn noise_filename_include_is_followed() {
        // standard_sonar.yaml and VL53L0X.yaml carry `noise: {filename: ...}`; C++ dropped it (noise off).
        let (omni, _) = load_shipped("omni_robot.yaml");
        let sonars: Vec<_> = omni
            .sensors
            .iter()
            .filter(|s| s.kind.name() == "sonar")
            .collect();
        assert_eq!(sonars.len(), 5);
        assert!(sonars.iter().all(|s| s.common.noise_std == 0.01));
        let (trin, _) = load_shipped("trin_bot.yaml");
        assert!(trin.sensors.iter().all(|s| s.common.noise_std == 0.005));
    }

    #[test]
    fn partial_inline_pose_keeps_file_fields() {
        // offset_laser.yaml pose is (0.2, -0.1, 0.5); inline overrides theta only (C++ zeroed x and y).
        assert_eq!(
            load("robot_overrides.yaml").sensors[0].common.pose,
            Pose2D {
                x: 0.2,
                y: -0.1,
                theta: 1.0
            }
        );
    }

    #[test]
    fn inline_noise_zero_disables() {
        assert_eq!(
            load("robot_overrides.yaml").sensors[1].common.noise_std,
            0.0
        );
    }

    #[test]
    fn noise_mean_ignored() {
        // Inline `noise_mean` alone leaves the file's noise_std (C++ replaced the whole noise block, std 0).
        assert_eq!(
            load("robot_overrides.yaml").sensors[2].common.noise_std,
            0.05
        );
    }

    #[test]
    fn frame_id_parsed_for_all_kinds() {
        // frame_id lives in SensorCommon, so every kind reads it (C++ skipped it for some kinds).
        let cfg = load("robot_frame_id.yaml");
        let named = |i: usize| {
            (
                cfg.sensors[i].kind.name(),
                cfg.sensors[i].common.frame_id.as_str(),
            )
        };
        assert_eq!(named(1), ("laser", "my_laser"));
        assert_eq!(named(2), ("sonar", "named_sonar"));
    }

    #[test]
    fn include_cycle_is_an_error() {
        let err = load_err("robot_include_loop.yaml");
        assert!(err.contains("include chain"), "{err}");
    }

    #[test]
    fn unknown_kinematic_rejected_at_load() {
        let err = load_err("robot_kinematic_invalid.yaml");
        assert!(
            err.contains("kinematic_model") && err.contains("tank"),
            "{err}"
        );
    }

    #[test]
    fn noisy_kinematic_alphas_loaded() {
        let (cfg, _) = load_shipped("simple_robot_noisy.yaml");
        assert_eq!(cfg.kinematic.odometry, OdometryModel::Velocity);
        assert_eq!(
            cfg.kinematic.alphas.0,
            [
                [0.01, 0.0, 0.005],
                [0.0, 0.0, 0.0],
                [0.005, 0.0, 0.02],
                [0.002, 0.0, 0.0]
            ]
        );
    }

    #[test]
    fn every_shipped_robot_loads() {
        let mut yamls: Vec<_> = std::fs::read_dir(resources().join("robots"))
            .unwrap()
            .map(|e| e.unwrap().path())
            .filter(|p| p.extension().is_some_and(|e| e == "yaml"))
            .collect();
        yamls.sort();
        assert_eq!(yamls.len(), 13);
        for yaml in &yamls {
            let name = yaml.file_name().unwrap().to_str().unwrap();
            let (cfg, warnings) = load_shipped(name);
            assert_eq!(
                warnings.is_empty(),
                name != "square_robot_rfid_reader.yaml",
                "{name}: {warnings:?}"
            );
            for s in &cfg.sensors {
                assert!(!s.common.frame_id.is_empty(), "{name}");
            }
        }
    }

    #[test]
    fn rfid_robot_loads_with_warnings() {
        let (cfg, warnings) = load_shipped("square_robot_rfid_reader.yaml");
        assert_eq!(warnings.len(), 2, "{warnings:?}");
        assert!(warnings.iter().any(|w| w.contains("rfid_reader")));
        // radius and points both given: the points win, as in C++.
        assert!(warnings.iter().any(|w| w.contains("radius and points")));
        assert!(matches!(&cfg.footprint, Footprint::Polygon(p) if p.len() == 4));
        assert_eq!(cfg.sensors.len(), 1);
        assert_eq!(cfg.sensors[0].common.frame_id, "laser_0");
    }

    #[test]
    fn default_frame_ids_on_shipped_robot() {
        let (cfg, _) = load_shipped("omni_robot.yaml");
        let ids: Vec<_> = cfg
            .sensors
            .iter()
            .map(|s| s.common.frame_id.as_str())
            .collect();
        assert_eq!(
            ids,
            [
                "laser_0", "sonar_0", "sonar_1", "sonar_2", "sonar_3", "sonar_4"
            ]
        );
        // Inline sonar pose overrides the file's.
        assert_eq!(
            cfg.sensors[2].common.pose,
            Pose2D {
                x: 0.0,
                y: 0.1,
                theta: 1.570795
            }
        );
    }

    #[test]
    fn missing_footprint_is_zero_radius_circle() {
        let (cfg, _) = load_shipped("too_simple_robot.yaml");
        assert_eq!(cfg.footprint, Footprint::Circle { radius: 0.0 });
    }
}
