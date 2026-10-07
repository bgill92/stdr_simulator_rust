//! Kinematics (ideal = differential drive, omni = holonomic) and the Thrun velocity-model noise.

use rand::Rng;
use rand_distr::{Distribution, Normal};

use crate::config::{AlphaRow, KinematicConfig, KinematicKind, OdometryModel};
use crate::error::CoreError;
use crate::pose::{Point2D, Pose2D, Twist2D, normalize_angle};

/// Advances `pose` by `vel` over `dt`, rotating about `pivot` (body frame). Ideal ignores
/// `linear_y` and uses exact arcs; omni integrates x/y/theta decoupled. Theta is wrapped once.
pub fn integrate(
    kind: KinematicKind,
    pose: Pose2D,
    vel: Twist2D,
    dt: f64,
    pivot: Point2D,
) -> Pose2D {
    let p = pose * Pose2D::translation(pivot);
    let mut next = p;
    let (v, w) = (vel.linear_x, vel.angular_z);
    match kind {
        // Arc radius exceeds 1e9 m below this |w|: effectively a straight line.
        KinematicKind::Ideal if w.abs() < 1e-9 => {
            next.x += v * dt * p.theta.cos();
            next.y += v * dt * p.theta.sin();
        }
        KinematicKind::Ideal => {
            let r = v / w;
            next.x += r * ((p.theta + w * dt).sin() - p.theta.sin());
            next.y -= r * ((p.theta + w * dt).cos() - p.theta.cos());
            next.theta += w * dt;
        }
        KinematicKind::Omni => {
            let (s, c) = p.theta.sin_cos();
            let vy = vel.linear_y;
            next.x += v * dt * c - vy * dt * s;
            next.y += v * dt * s + vy * dt * c;
            next.theta += w * dt;
        }
    }
    next.theta = normalize_angle(next.theta);
    // `next - R(theta)·pivot` rather than `next * translation(-pivot)`: same value, but this
    // rounding order is bit-identical to C++ `pivot_to_body_pose`, which the headless diff needs.
    let off = Pose2D {
        theta: next.theta,
        ..Pose2D::default()
    }
    .transform_point(pivot);
    Pose2D {
        x: next.x - off.x,
        y: next.y - off.y,
        theta: next.theta,
    }
}

/// Velocity-model noise (C++ `apply_noise`). The heading-drift term `g` is folded into
/// `angular_z`, which is all the integrator ever used it for. Per-tick variance is scaled by
/// `1/dt` so drift over a fixed interval is tick-rate independent. `Perfect` returns `cmd`
/// untouched without drawing from `rng`.
pub fn perturb(
    cmd: Twist2D,
    k: &KinematicConfig,
    dt: f64,
    rng: &mut (impl Rng + ?Sized),
) -> Result<Twist2D, CoreError> {
    // Checked before the Perfect short-circuit so a bad dt is never silently accepted.
    if dt.is_nan() || dt <= 0.0 {
        return Err(CoreError::Invalid(format!(
            "perturb: dt must be positive, got {dt}"
        )));
    }
    if k.odometry == OdometryModel::Perfect {
        return Ok(cmd);
    }
    let mut sample = |row| {
        let sigma = (k.alphas.variance(row, cmd) / dt).sqrt();
        // `Normal` is only built for sigma > 0 (zero-alpha channels draw nothing).
        if sigma > 0.0 {
            Normal::new(0.0, sigma).map_or(0.0, |n| n.sample(rng))
        } else {
            0.0
        }
    };
    let linear_x = cmd.linear_x + sample(AlphaRow::Ux);
    let linear_y = cmd.linear_y + sample(AlphaRow::Uy);
    let angular_z = cmd.angular_z + sample(AlphaRow::W);
    let drift = sample(AlphaRow::G);
    Ok(Twist2D {
        linear_x,
        linear_y,
        angular_z: angular_z + drift,
    })
}

/// Expected odometry error variance accumulated over `interval` seconds at command `cmd`.
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct OdometryVariance {
    /// Along-track (`linear_x`) variance, m².
    pub translational: f64,
    /// Heading variance, rad²: the angular-velocity and drift channels add (independent draws).
    pub rotational: f64,
}

/// Closed form of `perturb`'s accumulated variance: per tick `(a·u²/dt)·dt²`, summed over
/// `interval/dt` ticks, is `a·u²·interval`, independent of dt.
pub fn odometry_variance(
    cmd: Twist2D,
    k: &KinematicConfig,
    interval: f64,
) -> Result<OdometryVariance, CoreError> {
    if interval.is_nan() || interval <= 0.0 {
        return Err(CoreError::Invalid(format!(
            "odometry_variance: interval must be positive, got {interval}"
        )));
    }
    if k.odometry == OdometryModel::Perfect {
        return Ok(OdometryVariance {
            translational: 0.0,
            rotational: 0.0,
        });
    }
    let a = &k.alphas;
    Ok(OdometryVariance {
        translational: a.variance(AlphaRow::Ux, cmd) * interval,
        rotational: (a.variance(AlphaRow::W, cmd) + a.variance(AlphaRow::G, cmd)) * interval,
    })
}
