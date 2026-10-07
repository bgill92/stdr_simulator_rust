//! C++ `test_sensors.cpp` (laser + sonar) and the sensor quirk rows of PLAN.md.
//! `config.noise.mean` is not ported (never read), so the `NoiseIsZeroMeanNotBiased` cases
//! check a tiny `noise_std` against the noise-free range.
#![allow(non_snake_case)]

use approx::assert_abs_diff_eq;
use rand::SeedableRng;
use rand::rngs::StdRng;
use std::f64::consts::PI;
use stdr_core::{
    CameraSpec, LaserScan, LaserSpec, OccupancyGrid, Pose2D, Sensor, SensorCommon, SensorConfig,
    SonarSpec, simulate,
};

/// 10×10 at 0.1 m; `walled` marks rows/cols 0 and 9 occupied.
fn grid(walled: bool) -> OccupancyGrid {
    let data = (0..100)
        .map(|i| {
            let (col, row) = (i % 10, i / 10);
            if walled && (col == 0 || col == 9 || row == 0 || row == 9) {
                100
            } else {
                0
            }
        })
        .collect();
    OccupancyGrid::new(10, 10, 0.1, Pose2D::default(), data).unwrap()
}

const CENTRE: Pose2D = Pose2D {
    x: 0.5,
    y: 0.5,
    theta: 0.0,
};

fn sensor(kind: SensorConfig, noise_std: f64) -> Sensor {
    Sensor {
        common: SensorCommon {
            noise_std,
            ..SensorCommon::default()
        },
        kind,
    }
}

fn laser(
    min_angle: f64,
    max_angle: f64,
    min_range: f64,
    max_range: f64,
    num_rays: i32,
) -> SensorConfig {
    SensorConfig::Laser(LaserSpec {
        min_angle,
        max_angle,
        min_range,
        max_range,
        num_rays,
    })
}

fn sonar(min_range: f64, max_range: f64, cone_angle: f64) -> SensorConfig {
    SensorConfig::Sonar(SonarSpec {
        min_range,
        max_range,
        cone_angle,
    })
}

fn scan(s: &Sensor, at: Pose2D, g: &OccupancyGrid) -> LaserScan {
    simulate(s, at, g, &mut StdRng::seed_from_u64(1))
        .unwrap()
        .as_laser()
        .unwrap()
        .clone()
}

fn sonar_range(s: &Sensor, at: Pose2D, g: &OccupancyGrid) -> f64 {
    simulate(s, at, g, &mut StdRng::seed_from_u64(1))
        .unwrap()
        .as_sonar()
        .unwrap()
        .range
}

mod LaserSimulatorTest {
    use super::*;

    #[test]
    fn RayCountMatchesConfig() {
        let s = sensor(laser(-PI / 2.0, PI / 2.0, 0.1, 5.0, 9), 0.0);
        assert_eq!(scan(&s, CENTRE, &grid(false)).ranges.len(), 9);
    }

    #[test]
    fn EmptyMapAllInfinite() {
        let s = sensor(laser(-PI / 4.0, PI / 4.0, 0.05, 100.0, 5), 0.0);
        assert_eq!(scan(&s, CENTRE, &grid(false)).ranges, [f32::INFINITY; 5]);
    }

    #[test]
    fn RayExitingMapIsPositiveInfinity() {
        let s = sensor(laser(0.0, 0.0, 0.05, 5.0, 1), 0.0);
        let at = Pose2D { x: 0.95, ..CENTRE };
        assert_eq!(scan(&s, at, &grid(false)).ranges, [f32::INFINITY]);
    }

    #[test]
    fn TooCloseObstacleIsNegativeInfinity() {
        let s = sensor(laser(0.0, 0.0, 0.5, 5.0, 1), 0.0);
        assert_eq!(scan(&s, CENTRE, &grid(true)).ranges, [f32::NEG_INFINITY]);
    }

    #[test]
    fn WallInFront() {
        let s = sensor(laser(0.0, 0.0, 0.05, 5.0, 1), 0.0);
        assert!(scan(&s, CENTRE, &grid(true)).ranges[0] < 5.0);
    }

    #[test]
    fn NoiseIsZeroMeanNotBiased() {
        let s = sensor(laser(0.0, 0.0, 0.05, 5.0, 1), 0.001);
        assert_abs_diff_eq!(scan(&s, CENTRE, &grid(true)).ranges[0], 0.4, epsilon = 0.01);
    }

    #[test]
    fn NoiseDoesNotAffectNoHitBeam() {
        let s = sensor(laser(-PI / 4.0, PI / 4.0, 0.05, 5.0, 5), 1.0);
        assert_eq!(scan(&s, CENTRE, &grid(false)).ranges, [f32::INFINITY; 5]);
    }
}

mod SonarSimulatorTest {
    use super::*;

    #[test]
    fn EmptyMapMaxRange() {
        let s = sensor(sonar(0.1, 100.0, PI / 4.0), 0.0);
        assert_eq!(sonar_range(&s, CENTRE, &grid(false)), f64::INFINITY);
    }

    #[test]
    fn ObstacleInCone() {
        let s = sensor(sonar(0.05, 5.0, PI / 4.0), 0.0);
        assert!(sonar_range(&s, CENTRE, &grid(true)) < 5.0);
    }

    #[test]
    fn NoiseIsZeroMeanNotBiased() {
        // Oblique cone rays make the minimum ~0.5 m, not the 0.4 m straight-ahead range.
        let s = sensor(sonar(0.05, 5.0, PI / 4.0), 0.001);
        assert_abs_diff_eq!(sonar_range(&s, CENTRE, &grid(true)), 0.5, epsilon = 0.02);
    }

    /// max_range 0.3 / res 0.1 truncates to 2 steps; the old `max_steps + 1` sentinel case.
    #[test]
    fn NoHitNoiseOffReturnsInfinity() {
        let s = sensor(sonar(0.1, 0.3, PI / 4.0), 0.0);
        assert_eq!(sonar_range(&s, CENTRE, &grid(false)), f64::INFINITY);
    }

    #[test]
    fn NoHitNoiseOnReturnsInfinity() {
        let s = sensor(sonar(0.1, 0.3, PI / 4.0), 0.05);
        assert_eq!(sonar_range(&s, CENTRE, &grid(false)), f64::INFINITY);
    }
}

mod sensors {
    use super::*;

    /// `(0.7 / 0.1) as i32` = 6, so a wall 7 cells out is never reached. Were it reached (7
    /// steps), its 0.7 m return plus noise would come back finite about half the time.
    #[test]
    fn max_steps_truncates() {
        let mut data = vec![0; 100];
        for row in 0..10 {
            data[row * 10 + 7] = 100;
        }
        let g = OccupancyGrid::new(10, 10, 0.1, Pose2D::default(), data).unwrap();
        let s = sensor(laser(0.0, 0.0, 0.05, 0.7, 200), 0.05);
        let at = Pose2D { x: 0.05, ..CENTRE };
        assert!(scan(&s, at, &g).ranges.iter().all(|&r| r == f32::INFINITY));
    }

    /// The sweep is `while a <= cone/2 { a += 1° }` in f64. For a 5° cone the accumulated angle
    /// overshoots +2.5°, so the +edge ray is never cast while the -edge ray is.
    #[test]
    fn sonar_sweep_ray_count_matches_cpp() {
        // 500×500 at 0.01 m; sensor at cell (100, 250). Each target cell lies only on the
        // ±2.5° ray, 200 steps out.
        let probe = |cell: (usize, usize)| {
            let mut data = vec![0; 500 * 500];
            data[cell.1 * 500 + cell.0] = 100;
            let g = OccupancyGrid::new(500, 500, 0.01, Pose2D::default(), data).unwrap();
            let s = sensor(sonar(0.05, 5.0, 5.0 * PI / 180.0), 0.0);
            sonar_range(
                &s,
                Pose2D {
                    x: 1.005,
                    y: 2.505,
                    theta: 0.0,
                },
                &g,
            )
        };
        assert_abs_diff_eq!(probe((300, 241)), 2.0, epsilon = 1e-12);
        assert_eq!(probe((300, 259)), f64::INFINITY);
    }

    /// Core schedules cameras but never renders them.
    #[test]
    fn camera_yields_no_measurement() {
        let cam = sensor(SensorConfig::Camera(CameraSpec::default()), 0.0);
        assert_eq!(
            simulate(&cam, CENTRE, &grid(true), &mut StdRng::seed_from_u64(1)),
            None
        );
    }
}
