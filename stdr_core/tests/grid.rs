//! Grid invariants and cell policy (PLAN.md fix/quirk table, grid rows).

use stdr_core::{OccupancyGrid, Pose2D, Unknown};

fn grid(data: Vec<i8>, w: u32, h: u32) -> OccupancyGrid {
    OccupancyGrid::new(w, h, 0.1, Pose2D::default(), data).unwrap()
}

mod grid {
    use super::*;

    #[test]
    fn rejects_nonpositive_resolution() {
        for res in [0.0, -0.05, f64::NAN] {
            assert!(
                OccupancyGrid::new(1, 1, res, Pose2D::default(), vec![0]).is_err(),
                "resolution {res}"
            );
        }
    }

    #[test]
    fn rejects_empty() {
        assert!(OccupancyGrid::new(0, 1, 0.1, Pose2D::default(), vec![]).is_err());
        assert!(OccupancyGrid::new(1, 0, 0.1, Pose2D::default(), vec![]).is_err());
        assert!(OccupancyGrid::new(2, 2, 0.1, Pose2D::default(), vec![0; 3]).is_err());
    }

    #[test]
    fn unknown_policy_is_explicit() {
        // Row-major, row 0 first: cells (0,0)=-1, (1,0)=0, (0,1)=70, (1,1)=71.
        let g = grid(vec![-1, 0, 70, 71], 2, 2);
        assert!(g.is_blocked((0, 0), Unknown::Solid));
        assert!(!g.is_blocked((0, 0), Unknown::Transparent));
        for policy in [Unknown::Solid, Unknown::Transparent] {
            assert!(!g.is_blocked((1, 0), policy));
            assert!(
                !g.is_blocked((0, 1), policy),
                "70 is not above the threshold"
            );
            assert!(g.is_blocked((1, 1), policy));
            assert!(g.is_blocked((2, 0), policy), "out of bounds blocks");
            assert!(g.is_blocked((0, -1), policy), "out of bounds blocks");
        }
        assert_eq!(g.at((1, 1)), Some(71));
        assert_eq!(g.at((-1, 0)), None);
    }

    #[test]
    fn world_to_cell_truncates_toward_zero() {
        let g = OccupancyGrid::new(
            4,
            4,
            0.5,
            Pose2D {
                x: 1.0,
                y: -1.0,
                theta: 0.0,
            },
            vec![0; 16],
        )
        .unwrap();
        // -0.5 of a cell left of the origin truncates to cell 0, which is in bounds.
        assert_eq!(g.world_to_cell(0.75, -1.25), (0, 0));
        assert!(g.in_bounds((0, 0)));
        assert_eq!(g.world_to_cell(1.99, -0.01), (1, 1));
        assert_eq!(g.world_to_cell(0.0, -2.0), (-2, -2));
    }
}
