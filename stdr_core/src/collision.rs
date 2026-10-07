//! Swept footprint vs occupancy grid.

use crate::footprint::Footprint;
use crate::grid::{OccupancyGrid, Unknown};
use crate::pose::{Pose2D, angle_diff};

/// Whether moving `from` → `to` sweeps `footprint` through a blocked cell. Walks the cell path
/// one cell at a time (always including `to`, so `from == to` checks a single pose), rotating
/// the footprint by a shortest-arc theta lerp, and tests a 3×3 neighbourhood around every
/// footprint-edge cell. Out of bounds and unknown cells collide.
pub fn path_collides(
    grid: &OccupancyGrid,
    footprint: &Footprint,
    from: Pose2D,
    to: Pose2D,
) -> bool {
    let fp = footprint.vertices();
    let res = grid.resolution();
    let (fx, fy) = grid.world_to_cell(from.x, from.y);
    let (tx, ty) = grid.world_to_cell(to.x, to.y);
    let (dx, dy) = (f64::from(tx - fx), f64::from(ty - fy));
    let (path_angle, path_dist) = (dy.atan2(dx), dx.hypot(dy));
    let dtheta = angle_diff(to.theta, from.theta);

    let mut d = 0.0;
    while d <= path_dist {
        let step = offset((fx, fy), path_angle, d);
        let t = if path_dist > 0.0 { d / path_dist } else { 1.0 };
        let rot = Pose2D {
            theta: from.theta + t * dtheta,
            ..Pose2D::default()
        };
        // Footprint vertex → cell, relative to the path cell; truncation as C++.
        let cell = |i: usize| {
            let p = rot.transform_point(fp[i % fp.len()]);
            (step.0 + (p.x / res) as i32, step.1 + (p.y / res) as i32)
        };
        for i in 0..fp.len() {
            if edge_cells(cell(i), cell(i + 1)).any(|(cx, cy)| {
                (-1..=1).any(|ny| {
                    (-1..=1).any(|nx| grid.is_blocked((cx + nx, cy + ny), Unknown::Solid))
                })
            }) {
                return true;
            }
        }
        d += 1.0;
    }
    false
}

/// `a + round(d · (cos, sin))`; `f64::round` is half-away-from-zero like `std::round`.
fn offset(a: (i32, i32), angle: f64, d: f64) -> (i32, i32) {
    (
        a.0 + (d * angle.cos()).round() as i32,
        a.1 + (d * angle.sin()).round() as i32,
    )
}

/// Cells from `a` toward `b` at unit spacing, then `b` itself.
fn edge_cells(a: (i32, i32), b: (i32, i32)) -> impl Iterator<Item = (i32, i32)> {
    let (dx, dy) = (f64::from(b.0 - a.0), f64::from(b.1 - a.1));
    let (angle, dist) = (dy.atan2(dx), dx.hypot(dy));
    (0..)
        .map(f64::from)
        .take_while(move |&d| d < dist)
        .map(move |d| offset(a, angle, d))
        .chain(std::iter::once(b))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    /// C++ `std::round` is half-away-from-zero; `f64::round` matches (`round_ties_even` would not).
    #[test]
    fn edge_cells_round_half_away() {
        assert_eq!(offset((0, 0), 0.0, 2.5), (3, 0));
        assert_eq!(offset((0, 0), PI, 2.5), (-3, 0));
        assert_eq!(offset((1, 1), 0.0, 0.5), (2, 1));
    }
}
