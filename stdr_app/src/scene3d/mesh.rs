//! Core shapes as 3D meshes, in the ROS frame: metres, Z up. Only the scene root rotates them
//! into Bevy's Y-up world.

use bevy::asset::RenderAssetUsages;
use bevy::mesh::{Indices, PrimitiveTopology};
use bevy::prelude::*;
use stdr_core::{Footprint, OCCUPANCY_THRESHOLD, OccupancyGrid};

/// Occupied cells as walls of `height`. Each row's run of occupied cells becomes one box (greedy
/// row merge) with a top and four sides but no bottom; unknown cells are not walls.
pub fn extrude_grid(g: &OccupancyGrid, height: f32) -> Mesh {
    let res = g.resolution();
    let origin = g.origin();
    let occupied = |v: i8| v > OCCUPANCY_THRESHOLD;
    let mut faces = Faces::default();
    for (row, cells) in g.data().chunks(g.width() as usize).enumerate() {
        let y0 = origin.y + row as f64 * res;
        let mut col = 0;
        for run in cells.chunk_by(|a, b| occupied(*a) == occupied(*b)) {
            if occupied(run[0]) {
                let x0 = origin.x + col as f64 * res;
                let x1 = x0 + run.len() as f64 * res;
                let ring = [(x0, y0), (x1, y0), (x1, y0 + res), (x0, y0 + res)]
                    .map(|(x, y)| Vec2::new(x as f32, y as f32));
                faces.prism(&ring, height);
            }
            col += run.len();
        }
    }
    faces.into_mesh()
}

/// The robot outline (a circle is its 360-point ring) as a closed-top prism of `height`.
pub fn extrude_footprint(f: &Footprint, height: f32) -> Mesh {
    let mut ring: Vec<Vec2> = f
        .vertices()
        .iter()
        .map(|p| Vec2::new(p.x as f32, p.y as f32))
        .collect();
    if signed_area(&ring) < 0.0 {
        ring.reverse();
    }
    let mut faces = Faces::default();
    faces.prism(&ring, height);
    faces.into_mesh()
}

/// Flat-shaded faces: every face has its own vertices so each carries the face normal.
#[derive(Default)]
struct Faces {
    positions: Vec<[f32; 3]>,
    normals: Vec<[f32; 3]>,
    indices: Vec<u32>,
}

impl Faces {
    /// Triangles `tris` over `corners`, counter-clockwise seen from the side `normal` points to.
    fn face(&mut self, corners: &[Vec3], tris: &[[usize; 3]], normal: Vec3) {
        let base = self.positions.len() as u32;
        self.positions.extend(corners.iter().map(|c| c.to_array()));
        self.normals
            .extend(std::iter::repeat_n(normal.to_array(), corners.len()));
        self.indices
            .extend(tris.iter().flatten().map(|&i| base + i as u32));
    }

    /// Walls of a counter-clockwise `ring` from z = 0 to `height`, plus the top cap.
    fn prism(&mut self, ring: &[Vec2], height: f32) {
        for (i, &a) in ring.iter().enumerate() {
            let b = ring[(i + 1) % ring.len()];
            // Outward normal of a counter-clockwise edge points to its right.
            let normal = Vec2::new(b.y - a.y, a.x - b.x).normalize_or_zero();
            if normal == Vec2::ZERO {
                continue;
            }
            self.face(
                &[
                    a.extend(0.0),
                    b.extend(0.0),
                    b.extend(height),
                    a.extend(height),
                ],
                &[[0, 1, 2], [0, 2, 3]],
                normal.extend(0.0),
            );
        }
        let top: Vec<Vec3> = ring.iter().map(|p| p.extend(height)).collect();
        self.face(&top, &triangulate(ring), Vec3::Z);
    }

    fn into_mesh(self) -> Mesh {
        Mesh::new(
            PrimitiveTopology::TriangleList,
            RenderAssetUsages::default(),
        )
        .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, self.positions)
        .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, self.normals)
        .with_inserted_indices(Indices::U32(self.indices))
    }
}

/// Shoelace; positive for counter-clockwise.
fn signed_area(ring: &[Vec2]) -> f32 {
    ring.iter()
        .zip(ring.iter().cycle().skip(1))
        .map(|(a, b)| a.perp_dot(*b))
        .sum::<f32>()
        / 2.0
}

/// Ear clipping of a simple counter-clockwise ring, so concave footprints cap correctly.
// ponytail: O(n³) worst case, O(n²) for convex rings; fine for footprints (a circle is 360
// points) and 4-point wall boxes. Switch to a monotone-partition triangulator if rings grow.
fn triangulate(ring: &[Vec2]) -> Vec<[usize; 3]> {
    let mut left: Vec<usize> = (0..ring.len()).collect();
    let mut tris = Vec::with_capacity(ring.len().saturating_sub(2));
    while left.len() > 3 {
        let n = left.len();
        let corner = |i: usize| [left[(i + n - 1) % n], left[i], left[(i + 1) % n]];
        let is_ear = |i: usize| {
            let [a, b, c] = corner(i).map(|k| ring[k]);
            let inside = |q: Vec2| {
                (b - a).perp_dot(q - a) >= 0.0
                    && (c - b).perp_dot(q - b) >= 0.0
                    && (a - c).perp_dot(q - c) >= 0.0
            };
            (b - a).perp_dot(c - b) > 0.0
                && !left
                    .iter()
                    .filter(|k| !corner(i).contains(k))
                    .any(|&k| inside(ring[k]))
        };
        // A degenerate ring (collinear or zero-area points) has no ear; clipping any corner
        // still terminates and only adds zero-area triangles.
        let ear = (0..n).find(|&i| is_ear(i)).unwrap_or(0);
        tris.push(corner(ear));
        left.remove(ear);
    }
    if let [a, b, c] = left[..] {
        tris.push([a, b, c]);
    }
    tris
}

#[cfg(test)]
mod tests {
    use super::*;
    use stdr_core::{Point2D, Pose2D};

    fn positions(m: &Mesh) -> Vec<Vec3> {
        m.attribute(Mesh::ATTRIBUTE_POSITION)
            .unwrap()
            .as_float3()
            .unwrap()
            .iter()
            .map(|&p| Vec3::from_array(p))
            .collect()
    }

    fn normals(m: &Mesh) -> Vec<Vec3> {
        m.attribute(Mesh::ATTRIBUTE_NORMAL)
            .unwrap()
            .as_float3()
            .unwrap()
            .iter()
            .map(|&p| Vec3::from_array(p))
            .collect()
    }

    fn index_count(m: &Mesh) -> usize {
        m.indices().unwrap().len()
    }

    fn bounds(m: &Mesh) -> (Vec3, Vec3) {
        positions(m)
            .into_iter()
            .fold((Vec3::MAX, Vec3::MIN), |(lo, hi), p| (lo.min(p), hi.max(p)))
    }

    /// `rows` top to bottom as text: `#` occupied, `?` unknown, `.` free; 0.5 m cells at (1, 2).
    fn grid(rows: &[&str]) -> OccupancyGrid {
        let w = rows[0].len() as u32;
        let data = rows
            .iter()
            .rev()
            .flat_map(|r| {
                r.chars().map(|c| match c {
                    '#' => 100,
                    '?' => -1,
                    _ => 0,
                })
            })
            .collect();
        let origin = Pose2D {
            x: 1.0,
            y: 2.0,
            theta: 0.0,
        };
        OccupancyGrid::new(w, rows.len() as u32, 0.5, origin, data).unwrap()
    }

    fn triangle_area(m: &Mesh, normal: Vec3) -> f32 {
        let (p, n) = (positions(m), normals(m));
        let idx: Vec<usize> = m.indices().unwrap().iter().collect();
        idx.chunks(3)
            .filter(|t| n[t[0]] == normal)
            .map(|t| (p[t[1]] - p[t[0]]).cross(p[t[2]] - p[t[0]]).length() / 2.0)
            .sum()
    }

    #[test]
    fn one_cell_is_a_box_without_bottom() {
        let m = extrude_grid(&grid(&["...", ".#.", "..."]), 2.0);
        // Top + 4 sides, 4 vertices and 2 triangles each.
        assert_eq!(positions(&m).len(), 20);
        assert_eq!(index_count(&m), 30);
        assert_eq!(
            bounds(&m),
            (Vec3::new(1.5, 2.5, 0.0), Vec3::new(2.0, 3.0, 2.0))
        );
    }

    #[test]
    fn row_run_merges_into_one_box() {
        let m = extrude_grid(&grid(&["###."]), 1.0);
        assert_eq!(positions(&m).len(), 20);
        assert_eq!(
            bounds(&m),
            (Vec3::new(1.0, 2.0, 0.0), Vec3::new(2.5, 2.5, 1.0))
        );
    }

    #[test]
    fn separate_runs_and_rows_stay_separate_boxes() {
        let m = extrude_grid(&grid(&["#.#", "##."]), 1.0);
        assert_eq!(positions(&m).len(), 3 * 20);
        assert_eq!(index_count(&m), 3 * 30);
    }

    #[test]
    fn free_and_unknown_cells_emit_nothing() {
        let m = extrude_grid(&grid(&["?.?", "..."]), 1.0);
        assert!(positions(&m).is_empty());
        assert_eq!(index_count(&m), 0);
    }

    #[test]
    fn box_normals_are_unit_and_outward() {
        let m = extrude_grid(&grid(&["##"]), 1.0);
        let centre = Vec3::new(1.5, 2.25, 0.5);
        let (p, n) = (positions(&m), normals(&m));
        for (p, n) in p.iter().zip(&n) {
            assert!((n.length() - 1.0).abs() < 1e-6);
            assert!((*p - centre).dot(*n) > 0.0, "{p} {n}");
        }
        // Winding agrees with the normal: counter-clockwise seen from outside.
        let idx: Vec<usize> = m.indices().unwrap().iter().collect();
        for t in idx.chunks(3) {
            let face = (p[t[1]] - p[t[0]]).cross(p[t[2]] - p[t[0]]);
            assert!(face.dot(n[t[0]]) > 0.0);
        }
    }

    #[test]
    fn circle_footprint_is_a_360_sided_prism() {
        let m = extrude_footprint(&Footprint::Circle { radius: 0.5 }, 0.3);
        // 360 wall quads + a 360-vertex cap of 358 triangles.
        assert_eq!(positions(&m).len(), 360 * 4 + 360);
        assert_eq!(index_count(&m), 360 * 6 + 358 * 3);
        let (lo, hi) = bounds(&m);
        assert!((lo - Vec3::new(-0.5, -0.5, 0.0)).abs().max_element() < 1e-4);
        assert!((hi - Vec3::new(0.5, 0.5, 0.3)).abs().max_element() < 1e-4);
    }

    #[test]
    fn concave_cap_covers_exactly_the_polygon() {
        // random_shape_robot.yaml: two reflex corners.
        let pts = [
            (-0.2, -0.2),
            (0.2, -0.2),
            (0.3, -0.25),
            (0.2, 0.2),
            (0.3, 0.3),
            (-0.2, 0.2),
        ];
        let fp = Footprint::Polygon(pts.iter().map(|&(x, y)| Point2D { x, y }).collect());
        let ring: Vec<Vec2> = pts
            .iter()
            .map(|&(x, y)| Vec2::new(x as f32, y as f32))
            .collect();
        let m = extrude_footprint(&fp, 0.2);
        assert!((triangle_area(&m, Vec3::Z) - signed_area(&ring)).abs() < 1e-6);
        // A clockwise copy is reoriented and caps the same.
        let cw = Footprint::Polygon(pts.iter().rev().map(|&(x, y)| Point2D { x, y }).collect());
        let m = extrude_footprint(&cw, 0.2);
        assert!((triangle_area(&m, Vec3::Z) - signed_area(&ring)).abs() < 1e-6);
    }

    #[test]
    fn zero_radius_footprint_does_not_hang() {
        let m = extrude_footprint(&Footprint::Circle { radius: 0.0 }, 0.3);
        assert!(positions(&m).iter().all(|p| p.x == 0.0 && p.y == 0.0));
    }
}
