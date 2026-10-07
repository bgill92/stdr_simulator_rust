//! C++ `test_geometry_utils.cpp` point-in-footprint cases.
#![allow(non_snake_case)]

use stdr_core::{Footprint, Point2D};

const fn pt(x: f64, y: f64) -> Point2D {
    Point2D { x, y }
}

fn unit_square() -> Footprint {
    Footprint::Polygon(vec![pt(0.0, 0.0), pt(1.0, 0.0), pt(1.0, 1.0), pt(0.0, 1.0)])
}

const UNIT_CIRCLE: Footprint = Footprint::Circle { radius: 1.0 };

mod PointInFootprintTest {
    use super::*;

    #[test]
    fn DegeneratePolygonReturnsFalse() {
        assert!(!Footprint::Polygon(vec![pt(0.0, 0.0), pt(1.0, 0.0)]).contains(pt(0.0, 0.0)));
    }

    #[test]
    fn CircleInteriorPointIsInside() {
        assert!(UNIT_CIRCLE.contains(pt(0.0, 0.0)));
    }

    #[test]
    fn CircleExteriorPointIsOutside() {
        assert!(!UNIT_CIRCLE.contains(pt(2.0, 0.0)));
    }

    #[test]
    fn CircleOnEdgePointIsInside() {
        assert!(UNIT_CIRCLE.contains(pt(1.0, 0.0)));
    }

    #[test]
    fn PolygonInteriorPointIsInside() {
        assert!(unit_square().contains(pt(0.5, 0.5)));
    }

    #[test]
    fn PolygonExteriorPointIsOutside() {
        assert!(!unit_square().contains(pt(2.0, 2.0)));
    }

    #[test]
    fn PolygonOnEdgePointIsInside() {
        assert!(unit_square().contains(pt(0.5, 0.0)));
    }

    #[test]
    fn PolygonOnCornerPointIsInside() {
        assert!(unit_square().contains(pt(0.0, 0.0)));
    }
}

mod footprint {
    use super::*;

    #[test]
    fn concave_polygon_notch_is_outside() {
        // U shape: the notch between the arms is outside, the arms inside.
        let u = Footprint::Polygon(vec![
            pt(0.0, 0.0),
            pt(3.0, 0.0),
            pt(3.0, 3.0),
            pt(2.0, 3.0),
            pt(2.0, 1.0),
            pt(1.0, 1.0),
            pt(1.0, 3.0),
            pt(0.0, 3.0),
        ]);
        assert!(!u.contains(pt(1.5, 2.0)));
        assert!(u.contains(pt(0.5, 2.0)));
        assert!(u.contains(pt(2.5, 2.0)));
    }

    #[test]
    fn circle_vertices_are_one_degree_ring() {
        let v = Footprint::Circle { radius: 2.0 }.vertices();
        assert_eq!(v.len(), 360);
        assert_eq!(v[0], pt(2.0, 0.0));
        assert!(v.iter().all(|p| (p.x.hypot(p.y) - 2.0).abs() < 1e-12));
        assert_eq!(
            &*unit_square().vertices(),
            &[pt(0.0, 0.0), pt(1.0, 0.0), pt(1.0, 1.0), pt(0.0, 1.0)]
        );
    }
}
