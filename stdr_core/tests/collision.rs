//! C++ `test_collision_checker.cpp`. Single-pose `check_collision` cases go through
//! `path_collides(from == to)`; `ZeroResolution*` / `EmptyMap*` are `grid::rejects_*` (grid.rs).
#![allow(non_snake_case)]

use std::f64::consts::PI;
use stdr_core::{Footprint, OccupancyGrid, Pose2D, path_collides};

/// 10×10 at 0.1 m, origin (0, 0); listed (col, row) cells occupied.
fn grid(occupied: &[(usize, usize)]) -> OccupancyGrid {
    let mut data = vec![0; 100];
    for &(col, row) in occupied {
        data[row * 10 + col] = 100;
    }
    OccupancyGrid::new(10, 10, 0.1, Pose2D::default(), data).unwrap()
}

/// Smaller than one cell.
const SMALL: Footprint = Footprint::Circle { radius: 0.05 };

const fn pose(x: f64, y: f64, theta: f64) -> Pose2D {
    Pose2D { x, y, theta }
}

fn at(g: &OccupancyGrid, p: Pose2D) -> bool {
    path_collides(g, &SMALL, p, p)
}

mod CollisionCheckerTest {
    use super::*;

    #[test]
    fn FreeCellNoCollision() {
        assert!(!at(&grid(&[]), pose(0.45, 0.45, 0.0)));
    }

    #[test]
    fn OccupiedCellCollision() {
        assert!(at(&grid(&[(5, 5)]), pose(0.55, 0.55, 0.0)));
    }

    #[test]
    fn OutOfBoundsCollision() {
        assert!(at(&grid(&[]), pose(5.0, 5.0, 0.0)));
    }

    #[test]
    fn PathCollisionDetected() {
        let wall: Vec<_> = (0..10).map(|row| (5, row)).collect();
        let g = grid(&wall);
        assert!(path_collides(
            &g,
            &SMALL,
            pose(0.25, 0.45, 0.0),
            pose(0.75, 0.45, 0.0)
        ));
    }

    #[test]
    fn PathNoCollision() {
        let g = grid(&[]);
        assert!(!path_collides(
            &g,
            &SMALL,
            pose(0.15, 0.15, 0.0),
            pose(0.45, 0.45, 0.0)
        ));
    }
}

mod collision {
    use super::*;

    /// A thin bar turning across ±π while moving 3 cells: the C++ raw lerp swept the
    /// intermediate cells through theta ≈ ±1 rad, poking the bar out of the map; the shortest
    /// arc stays near ±π, pointing away from the wall and the edges.
    #[test]
    fn rotation_through_pi_uses_shortest_arc() {
        // 40×10 free grid at 0.1 m with a wall at column 30; robot at x = 2.05 (cell 20).
        let mut data = vec![0; 400];
        for row in 0..10 {
            data[row * 40 + 30] = 100;
        }
        let g = OccupancyGrid::new(40, 10, 0.1, Pose2D::default(), data).unwrap();
        let bar = Footprint::Polygon(
            [(0.0, -0.01), (0.9, -0.01), (0.9, 0.01), (0.0, 0.01)]
                .map(|(x, y)| stdr_core::Point2D { x, y })
                .to_vec(),
        );
        let from = pose(2.05, 0.45, PI - 0.05);
        let to = pose(2.35, 0.45, -PI + 0.05);
        assert!(
            path_collides(&g, &bar, from, pose(2.05, 0.45, 0.0)),
            "bar reaches the wall at theta 0"
        );
        assert!(!path_collides(&g, &bar, from, from));
        assert!(
            !path_collides(&g, &bar, from, to),
            "shortest arc never faces the wall"
        );
    }
}
