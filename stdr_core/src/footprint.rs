use std::borrow::Cow;
use std::f64::consts::PI;

use crate::pose::Point2D;

/// Robot outline in the body frame.
#[derive(Clone, Debug, PartialEq)]
pub enum Footprint {
    Circle { radius: f64 },
    Polygon(Vec<Point2D>),
}

impl Default for Footprint {
    /// A robot yaml without a footprint is a zero-radius circle (C++ parity).
    fn default() -> Self {
        Footprint::Circle { radius: 0.0 }
    }
}

/// On-edge tolerance: well above float noise, negligible for real robot geometry.
const EDGE_EPS: f64 = 1e-9;

impl Footprint {
    /// Points on the boundary count as inside. Polygons with fewer than 3 vertices contain nothing.
    pub fn contains(&self, p: Point2D) -> bool {
        let pts = match self {
            Footprint::Circle { radius } => return p.x.hypot(p.y) <= radius + EDGE_EPS,
            Footprint::Polygon(pts) if pts.len() < 3 => return false,
            Footprint::Polygon(pts) => pts,
        };
        // Crossing-number test with a +x ray; on-edge points short-circuit to inside.
        let mut inside = false;
        for (i, &a) in pts.iter().enumerate() {
            let b = pts[(i + 1) % pts.len()];
            let (dx, dy) = (b.x - a.x, b.y - a.y);
            let len_sq = dx * dx + dy * dy;
            if len_sq > 0.0
                && (dx * (p.y - a.y) - dy * (p.x - a.x)).abs() <= EDGE_EPS * len_sq.sqrt()
            {
                let dot = (p.x - a.x) * dx + (p.y - a.y) * dy;
                if dot >= -EDGE_EPS && dot <= len_sq + EDGE_EPS {
                    return true;
                }
            }
            if (a.y <= p.y) != (b.y <= p.y) && p.x < a.x + (p.y - a.y) * dx / dy {
                inside = !inside;
            }
        }
        inside
    }

    /// Polygon vertices as given; a circle becomes a 360-point ring at 1° steps (C++ `expand_footprint`).
    pub fn vertices(&self) -> Cow<'_, [Point2D]> {
        match self {
            Footprint::Polygon(pts) => Cow::Borrowed(pts),
            Footprint::Circle { radius } => Cow::Owned(
                (0..360)
                    .map(|i| {
                        let a = f64::from(i) * PI / 180.0;
                        Point2D {
                            x: radius * a.cos(),
                            y: radius * a.sin(),
                        }
                    })
                    .collect(),
            ),
        }
    }
}
