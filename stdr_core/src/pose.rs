use std::ops::{Mul, Neg};

use serde::Deserialize;

/// SE(2) pose. `Mul` composes (`a * b` = `b` expressed in `a`'s frame, mapped to the world).
#[derive(Clone, Copy, PartialEq, Debug, Default, Deserialize)]
#[serde(default)]
pub struct Pose2D {
    pub x: f64,
    pub y: f64,
    pub theta: f64,
}

#[derive(Clone, Copy, PartialEq, Debug, Default, Deserialize)]
#[serde(default)]
pub struct Point2D {
    pub x: f64,
    pub y: f64,
}

#[derive(Clone, Copy, PartialEq, Debug, Default)]
pub struct Twist2D {
    pub linear_x: f64,
    pub linear_y: f64,
    pub angular_z: f64,
}

impl Mul for Pose2D {
    type Output = Pose2D;

    /// Theta is `a + b`, deliberately not wrapped (C++ `compose` parity); call `wrapped()` when needed.
    #[allow(clippy::suspicious_arithmetic_impl)]
    fn mul(self, b: Pose2D) -> Pose2D {
        let p = self.transform_point(Point2D { x: b.x, y: b.y });
        Pose2D {
            x: p.x,
            y: p.y,
            theta: self.theta + b.theta,
        }
    }
}

impl Pose2D {
    pub fn inverse(self) -> Pose2D {
        let (s, c) = self.theta.sin_cos();
        Pose2D {
            x: -(self.x * c + self.y * s),
            y: -(-self.x * s + self.y * c),
            theta: -self.theta,
        }
    }

    /// `R(theta) * p + t`.
    pub fn transform_point(self, p: Point2D) -> Point2D {
        let (s, c) = self.theta.sin_cos();
        Point2D {
            x: self.x + p.x * c - p.y * s,
            y: self.y + p.x * s + p.y * c,
        }
    }

    pub fn wrapped(self) -> Pose2D {
        Pose2D {
            theta: normalize_angle(self.theta),
            ..self
        }
    }

    pub fn translation(p: Point2D) -> Pose2D {
        Pose2D {
            x: p.x,
            y: p.y,
            theta: 0.0,
        }
    }
}

impl Neg for Point2D {
    type Output = Point2D;

    fn neg(self) -> Point2D {
        Point2D {
            x: -self.x,
            y: -self.y,
        }
    }
}

/// Wraps to [-pi, pi]. The one `atan2(sin, cos)` site in the crate.
pub fn normalize_angle(a: f64) -> f64 {
    a.sin().atan2(a.cos())
}

pub fn angle_diff(a: f64, b: f64) -> f64 {
    normalize_angle(a - b)
}
