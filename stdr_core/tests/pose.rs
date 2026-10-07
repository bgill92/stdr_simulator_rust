//! C++ `test_geometry_utils.cpp` (compose/inverse/pivot) and the pose cases of `test_plot_helpers.cpp`.
#![allow(non_snake_case)]

use approx::assert_abs_diff_eq;
use std::f64::consts::PI;
use stdr_core::{Point2D, Pose2D, angle_diff};

fn assert_pose_eq(a: Pose2D, b: Pose2D, tol: f64) {
    assert_abs_diff_eq!(a.x, b.x, epsilon = tol);
    assert_abs_diff_eq!(a.y, b.y, epsilon = tol);
    assert_abs_diff_eq!(a.theta, b.theta, epsilon = tol);
}

const fn pose(x: f64, y: f64, theta: f64) -> Pose2D {
    Pose2D { x, y, theta }
}

mod ComposeInverseTest {
    use super::*;

    #[test]
    fn ComposeWithInverseIsIdentity() {
        let a = pose(3.0, -1.5, 0.8);
        assert_pose_eq(a * a.inverse(), Pose2D::default(), 1e-12);
    }

    #[test]
    fn CorrectionTransformComposedOntoBeliefRecoversTruth() {
        let truth = pose(2.0, 1.0, 0.3);
        let belief = pose(1.8, 0.9, 0.25);
        let correction = truth * belief.inverse();
        assert_pose_eq(correction * belief, truth, 1e-12);
    }
}

mod BodyToPivotPoseTest {
    use super::*;

    #[test]
    fn ZeroPivotReturnsPoseUnchanged() {
        let body = pose(1.0, 2.0, 0.5);
        assert_pose_eq(body * Pose2D::translation(Point2D::default()), body, 1e-12);
    }
}

mod PivotToBodyPoseTest {
    use super::*;

    #[test]
    fn RoundTripRecoverOriginalPose() {
        let body = pose(3.0, -1.5, 0.8);
        let pivot = Point2D { x: 0.4, y: -0.2 };
        let pivot_pose = body * Pose2D::translation(pivot);
        assert_pose_eq(pivot_pose * Pose2D::translation(-pivot), body, 1e-12);
    }
}

mod PlotHelpers {
    use super::*;

    #[test]
    fn WrappedAngleDiffZeroWhenEqual() {
        assert_abs_diff_eq!(angle_diff(0.5, 0.5), 0.0, epsilon = 1e-10);
    }

    #[test]
    fn WrappedAngleDiffPositiveWrap() {
        assert_abs_diff_eq!(angle_diff(0.9 * PI, -0.9 * PI), -0.2 * PI, epsilon = 1e-10);
    }

    #[test]
    fn WrappedAngleDiffNegativeWrap() {
        assert_abs_diff_eq!(angle_diff(-0.9 * PI, 0.9 * PI), 0.2 * PI, epsilon = 1e-10);
    }

    #[test]
    fn WrappedAngleDiffFullTurnIsZero() {
        assert_abs_diff_eq!(angle_diff(2.0 * PI, 0.0), 0.0, epsilon = 1e-10);
    }

    #[test]
    fn WrappedAngleDiffExactlyPositivePi() {
        assert_abs_diff_eq!(angle_diff(PI, 0.0), PI, epsilon = 1e-10);
    }

    #[test]
    fn WrappedAngleDiffExactlyNegativePi() {
        assert_abs_diff_eq!(angle_diff(-PI, 0.0), -PI, epsilon = 1e-10);
    }

    // C++ map_to_robot(p, robot) == robot.inverse() * p; robot_to_map(p, robot) == robot * p.
    #[test]
    fn MapToRobotIdentityAtOrigin() {
        let p_map = pose(3.0, 4.0, 0.5);
        assert_pose_eq(Pose2D::default().inverse() * p_map, p_map, 1e-10);
    }

    #[test]
    fn MapToRobotKnownPose() {
        let robot = pose(1.0, 2.0, PI / 2.0);
        assert_pose_eq(
            robot.inverse() * pose(2.0, 2.0, 0.0),
            pose(0.0, -1.0, -PI / 2.0),
            1e-10,
        );
    }

    #[test]
    fn RobotToMapInvertsMapToRobot() {
        let robot = pose(3.0, -1.5, 0.7);
        let p_map = pose(5.0, 2.0, 1.1);
        assert_pose_eq(robot * (robot.inverse() * p_map), p_map, 1e-10);
    }
}

mod pose {
    use super::*;

    #[test]
    fn mul_does_not_wrap() {
        let p = pose(0.0, 0.0, 3.0) * pose(0.0, 0.0, 3.0);
        assert_eq!(p.theta, 6.0);
        assert_abs_diff_eq!(p.wrapped().theta, 6.0 - 2.0 * PI, epsilon = 1e-12);
    }

    #[test]
    fn transform_point_rotates_then_translates() {
        let p = pose(1.0, 2.0, PI / 2.0).transform_point(Point2D { x: 1.0, y: 0.0 });
        assert_abs_diff_eq!(p.x, 1.0, epsilon = 1e-12);
        assert_abs_diff_eq!(p.y, 3.0, epsilon = 1e-12);
    }
}
