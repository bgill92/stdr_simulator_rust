//! C++ `test_motion_models.cpp`. `model.update(..)` = `integrate(kind, pose, perturb(cmd)?, ..)`;
//! `model.integrate(..)` = `integrate(kind, pose, cmd, ..)`.
#![allow(non_snake_case)]

use approx::assert_abs_diff_eq;
use rand::SeedableRng;
use rand::rngs::StdRng;
use std::f64::consts::PI;
use stdr_core::{
    Alphas, KinematicConfig, KinematicKind, OdometryModel, Point2D, Pose2D, Twist2D, integrate,
    odometry_variance, perturb,
};

const DT: f64 = 0.1;
const TOL: f64 = 1e-9;
const NO_PIVOT: Point2D = Point2D { x: 0.0, y: 0.0 };

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

fn assert_pose_eq(a: Pose2D, b: Pose2D) {
    assert_abs_diff_eq!(a.x, b.x, epsilon = TOL);
    assert_abs_diff_eq!(a.y, b.y, epsilon = TOL);
    assert_abs_diff_eq!(a.theta, b.theta, epsilon = TOL);
}

/// Perfect odometry, all alphas zero: noise disabled.
fn zero_noise() -> KinematicConfig {
    KinematicConfig::default()
}

/// Rows Ux, Uy, W, G; columns ux², uy², w².
fn alphas(rows: [[f64; 3]; 4]) -> KinematicConfig {
    KinematicConfig {
        odometry: OdometryModel::Velocity,
        alphas: Alphas(rows),
        ..KinematicConfig::default()
    }
}

/// a_ux_ux = a_w_w = 0.05 under Velocity.
fn velocity_noise() -> KinematicConfig {
    alphas([[0.05, 0.0, 0.0], [0.0; 3], [0.0, 0.0, 0.05], [0.0; 3]])
}

/// C++ `model.update`: noisy command, then the clean kinematics.
fn update(
    kind: KinematicKind,
    p: Pose2D,
    cmd: Twist2D,
    k: &KinematicConfig,
    pivot: Point2D,
    rng: &mut StdRng,
) -> Pose2D {
    integrate(kind, p, perturb(cmd, k, DT, rng).unwrap(), DT, pivot)
}

fn rng(seed: u64) -> StdRng {
    StdRng::seed_from_u64(seed)
}

/// Pure spin about an offset pivot keeps the body origin on a circle of radius |pivot|.
fn assert_sweeps_arc(kind: KinematicKind, pivot: Point2D, pivot_world: Point2D) {
    let start = pose(0.0, 0.0, 0.0);
    let mut p = start;
    for _ in 0..10 {
        p = update(
            kind,
            p,
            twist(0.0, 0.0, 1.0),
            &zero_noise(),
            pivot,
            &mut rng(0),
        );
    }
    assert!((p.x - start.x).hypot(p.y - start.y) > 1e-6);
    assert_abs_diff_eq!(
        (p.x - pivot_world.x).hypot(p.y - pivot_world.y),
        1.0,
        epsilon = TOL
    );
}

fn assert_velocity_diverges(kind: KinematicKind, cmd: Twist2D) {
    let mut r = rng(7);
    let (mut noisy, mut clean) = (Pose2D::default(), Pose2D::default());
    let mut diverged = false;
    for _ in 0..50 {
        noisy = update(kind, noisy, cmd, &velocity_noise(), NO_PIVOT, &mut r);
        clean = integrate(kind, clean, cmd, DT, NO_PIVOT);
        diverged |= (noisy.x - clean.x).abs() > 1e-6
            || (noisy.y - clean.y).abs() > 1e-6
            || (noisy.theta - clean.theta).abs() > 1e-6;
    }
    assert!(
        diverged,
        "velocity-mode noise should separate truth from odometry"
    );
}

mod IdealMotionModelTest {
    use super::*;
    const K: KinematicKind = KinematicKind::Ideal;

    fn step(p: Pose2D, cmd: Twist2D) -> Pose2D {
        update(K, p, cmd, &zero_noise(), NO_PIVOT, &mut rng(0))
    }

    #[test]
    fn StationaryRobotStaysStill() {
        let start = pose(1.0, 2.0, 0.5);
        assert_pose_eq(step(start, twist(0.0, 0.0, 0.0)), start);
    }

    #[test]
    fn StraightLineMotion() {
        assert_pose_eq(
            step(Pose2D::default(), twist(1.0, 0.0, 0.0)),
            pose(0.1, 0.0, 0.0),
        );
    }

    #[test]
    fn PureRotation() {
        assert_pose_eq(
            step(pose(1.0, 1.0, 0.0), twist(0.0, 0.0, 1.0)),
            pose(1.0, 1.0, 0.1),
        );
    }

    #[test]
    fn ArcMotion() {
        let r = step(Pose2D::default(), twist(1.0, 0.0, 1.0));
        assert_abs_diff_eq!(r.x, 0.0, epsilon = 0.2);
        assert_abs_diff_eq!(r.theta, 0.1, epsilon = TOL);
        assert!(r.x > 0.0);
    }

    #[test]
    fn ThetaNormalized() {
        let r = step(pose(0.0, 0.0, PI - 0.01), twist(0.0, 0.0, 10.0));
        assert!((-PI..=PI).contains(&r.theta), "{}", r.theta);
    }
}

mod OmniMotionModelTest {
    use super::*;
    const K: KinematicKind = KinematicKind::Omni;

    fn step(p: Pose2D, cmd: Twist2D) -> Pose2D {
        update(K, p, cmd, &zero_noise(), NO_PIVOT, &mut rng(0))
    }

    #[test]
    fn StationaryRobotStaysStill() {
        let start = pose(3.0, -1.5, 0.8);
        assert_pose_eq(step(start, twist(0.0, 0.0, 0.0)), start);
    }

    #[test]
    fn LateralMotion() {
        let r = step(Pose2D::default(), twist(0.0, 1.0, 0.0));
        assert_abs_diff_eq!(r.x, 0.0, epsilon = TOL);
        assert!(r.y > 0.0);
        assert_abs_diff_eq!(r.theta, 0.0, epsilon = TOL);
    }

    #[test]
    fn CombinedMotion() {
        let r = step(Pose2D::default(), twist(1.0, 1.0, 1.0));
        assert!(r.x > 0.0 && r.y > 0.0 && r.theta > 0.0);
    }
}

mod IdealMotionModelCenterOfRotationTest {
    use super::*;
    const K: KinematicKind = KinematicKind::Ideal;

    #[test]
    fn ZeroPivotMatchesDefaultBehaviour() {
        let (start, cmd) = (pose(1.0, 2.0, 0.3), twist(1.0, 0.0, 0.5));
        let zero = Point2D { x: 0.0, y: 0.0 };
        assert_pose_eq(
            update(K, start, cmd, &zero_noise(), zero, &mut rng(0)),
            integrate(K, start, cmd, DT, Point2D::default()),
        );
    }

    #[test]
    fn StraightLineUnaffectedByPivot() {
        let cmd = twist(1.0, 0.0, 0.0);
        let pivot = Point2D { x: 0.5, y: 0.3 };
        assert_pose_eq(
            update(K, Pose2D::default(), cmd, &zero_noise(), pivot, &mut rng(0)),
            update(
                K,
                Pose2D::default(),
                cmd,
                &zero_noise(),
                NO_PIVOT,
                &mut rng(0),
            ),
        );
    }

    #[test]
    fn PureRotationAboutOffsetPivotSweepsArc() {
        let pivot = Point2D { x: 1.0, y: 0.0 };
        assert_sweeps_arc(K, pivot, pivot);
    }
}

mod OmniMotionModelCenterOfRotationTest {
    use super::*;
    const K: KinematicKind = KinematicKind::Omni;

    #[test]
    fn ZeroPivotMatchesDefaultBehaviour() {
        let (start, cmd) = (pose(3.0, -1.5, 0.8), twist(1.0, 0.5, 0.4));
        let zero = Point2D { x: 0.0, y: 0.0 };
        assert_pose_eq(
            update(K, start, cmd, &zero_noise(), zero, &mut rng(0)),
            integrate(K, start, cmd, DT, Point2D::default()),
        );
    }

    #[test]
    fn StraightLineUnaffectedByPivot() {
        let cmd = twist(1.0, 0.5, 0.0);
        let pivot = Point2D { x: 0.4, y: -0.2 };
        assert_pose_eq(
            update(K, Pose2D::default(), cmd, &zero_noise(), pivot, &mut rng(0)),
            update(
                K,
                Pose2D::default(),
                cmd,
                &zero_noise(),
                NO_PIVOT,
                &mut rng(0),
            ),
        );
    }

    #[test]
    fn PureRotationAboutOffsetPivotSweepsArc() {
        // Start theta is 0, so the world pivot is the body pivot.
        let pivot = Point2D { x: 0.0, y: 1.0 };
        assert_sweeps_arc(K, pivot, pivot);
    }
}

mod IdealMotionModelIntegrateTest {
    use super::*;
    const K: KinematicKind = KinematicKind::Ideal;

    #[test]
    fn PerfectMatchesUpdateExactly() {
        let (start, cmd) = (pose(0.5, -0.5, 0.3), twist(1.0, 0.0, 0.7));
        assert_eq!(
            update(K, start, cmd, &zero_noise(), NO_PIVOT, &mut rng(42)),
            integrate(K, start, cmd, DT, NO_PIVOT)
        );
    }

    #[test]
    fn PerfectIgnoresNonzeroAlphas() {
        let k = KinematicConfig {
            odometry: OdometryModel::Perfect,
            ..velocity_noise()
        };
        let cmd = twist(1.0, 0.0, 0.5);
        assert_eq!(
            update(K, Pose2D::default(), cmd, &k, NO_PIVOT, &mut rng(42)),
            integrate(K, Pose2D::default(), cmd, DT, NO_PIVOT)
        );
    }

    #[test]
    fn VelocityDivergesFromIntegrate() {
        assert_velocity_diverges(K, twist(1.0, 0.0, 0.3));
    }
}

mod OmniMotionModelIntegrateTest {
    use super::*;
    const K: KinematicKind = KinematicKind::Omni;

    #[test]
    fn PerfectMatchesUpdateExactly() {
        let (start, cmd) = (pose(-0.2, 0.4, -0.1), twist(1.0, 0.5, 0.4));
        assert_eq!(
            update(K, start, cmd, &zero_noise(), NO_PIVOT, &mut rng(42)),
            integrate(K, start, cmd, DT, NO_PIVOT)
        );
    }

    #[test]
    fn VelocityDivergesFromIntegrate() {
        assert_velocity_diverges(K, twist(1.0, 0.5, 0.3));
    }
}

mod ApplyNoiseTest {
    use super::*;

    #[test]
    fn ZeroDtThrows() {
        assert!(perturb(twist(1.0, 0.0, 0.0), &velocity_noise(), 0.0, &mut rng(1)).is_err());
    }

    #[test]
    fn NegativeDtThrows() {
        assert!(perturb(twist(1.0, 0.0, 0.0), &velocity_noise(), -0.1, &mut rng(1)).is_err());
    }

    #[test]
    fn ZeroDtThrowsUnderPerfect() {
        assert!(perturb(twist(1.0, 0.0, 0.0), &zero_noise(), 0.0, &mut rng(1)).is_err());
    }

    /// The drift term is folded into `angular_z`, so "zero drift" is an unchanged `angular_z`.
    #[test]
    fn PerfectReturnsCommandUnchangedWithZeroDrift() {
        let k = KinematicConfig {
            odometry: OdometryModel::Perfect,
            ..velocity_noise()
        };
        let cmd = twist(1.0, 0.5, 0.2);
        assert_eq!(perturb(cmd, &k, DT, &mut rng(1)).unwrap(), cmd);
    }

    #[test]
    fn LinearNoiseVarianceScalesInverselyWithDt() {
        const SAMPLES: usize = 20_000;
        const A_UX_UX: f64 = 0.01;
        let k = alphas([[A_UX_UX, 0.0, 0.0], [0.0; 3], [0.0; 3], [0.0; 3]]);
        let cmd = twist(1.0, 0.0, 0.0);
        for dt in [0.01, 0.1] {
            let mut r = rng(123);
            let noise: Vec<f64> = (0..SAMPLES)
                .map(|_| perturb(cmd, &k, dt, &mut r).unwrap().linear_x - cmd.linear_x)
                .collect();
            let mean = noise.iter().sum::<f64>() / SAMPLES as f64;
            let var = noise.iter().map(|n| n * n).sum::<f64>() / SAMPLES as f64 - mean * mean;
            let expected = A_UX_UX / dt;
            assert_abs_diff_eq!(var, expected, epsilon = expected * 0.1);
        }
    }
}

mod OdometryVarianceTest {
    use super::*;

    #[test]
    fn ZeroIntervalThrows() {
        assert!(odometry_variance(twist(1.0, 0.0, 0.0), &velocity_noise(), 0.0).is_err());
    }

    #[test]
    fn NegativeIntervalThrows() {
        assert!(odometry_variance(twist(1.0, 0.0, 0.0), &velocity_noise(), -0.1).is_err());
    }

    #[test]
    fn PerfectReturnsZeroVariance() {
        let k = KinematicConfig {
            odometry: OdometryModel::Perfect,
            ..velocity_noise()
        };
        let v = odometry_variance(twist(1.0, 0.5, 0.2), &k, DT).unwrap();
        assert_eq!((v.translational, v.rotational), (0.0, 0.0));
    }

    #[test]
    fn TranslationalMatchesClosedForm() {
        let k = alphas([[0.02, 0.01, 0.005], [0.0; 3], [0.0; 3], [0.0; 3]]);
        let v = odometry_variance(twist(2.0, 1.0, 0.5), &k, 0.5).unwrap();
        let expected = (0.02 * 2.0 * 2.0 + 0.01 * 1.0 * 1.0 + 0.005 * 0.5 * 0.5) * 0.5;
        assert_abs_diff_eq!(v.translational, expected, epsilon = TOL);
    }

    #[test]
    fn RotationalCombinesAngularAndDriftTerms() {
        let k = alphas([[0.0; 3], [0.0; 3], [0.01, 0.0, 0.02], [0.005, 0.0, 0.015]]);
        let v = odometry_variance(twist(1.0, 0.0, 2.0), &k, 0.2).unwrap();
        let angular = (0.01 * 1.0 * 1.0 + 0.02 * 2.0 * 2.0) * 0.2;
        let drift = (0.005 * 1.0 * 1.0 + 0.015 * 2.0 * 2.0) * 0.2;
        assert_abs_diff_eq!(v.rotational, angular + drift, epsilon = TOL);
    }
}
