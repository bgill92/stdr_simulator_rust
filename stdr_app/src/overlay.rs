//! What a robot looks like, independent of where it is drawn. Everything is in f64 world
//! coordinates with `[u8; 4]` colours, so neither bevy nor egui types appear here; adapters
//! (`GizmoCanvas` for the 2D view, a `RecordingCanvas` in tests) decide how to draw it.

use stdr_core::{
    Footprint, LaserScan, Measurement, Point2D, Pose2D, RobotRuntime, SensorConfig, angle_diff,
};

#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Style {
    pub color: [u8; 4],
    /// Line width in screen pixels.
    pub width: f32,
}

pub const FOOTPRINT: Style = Style {
    color: [0, 200, 0, 255],
    width: 2.0,
};
pub const SELECTED: Style = Style {
    color: [255, 200, 0, 255],
    width: 3.0,
};
pub const COLLIDED: Style = Style {
    color: [230, 30, 30, 255],
    width: 3.0,
};
pub const ODOM_GHOST: Style = Style {
    color: [255, 140, 0, 160],
    width: 1.5,
};
pub const HEADING: Style = Style {
    color: [255, 0, 0, 255],
    width: 2.5,
};
pub const TRAIL: Style = Style {
    color: [0, 120, 255, 200],
    width: 1.5,
};
pub const LASER_RAY: Style = Style {
    color: [255, 50, 50, 80],
    width: 1.0,
};
pub const SONAR_CONE: Style = Style {
    color: [50, 200, 255, 120],
    width: 1.5,
};

/// Radius of the robot centre dot, in screen pixels.
pub const CENTER_DOT_PX: f32 = 3.0;

/// A draw target in world coordinates.
pub trait Canvas {
    fn polyline(&mut self, pts: &[[f64; 2]], style: Style, closed: bool);
    /// `radius` in screen pixels.
    fn points(&mut self, pts: &[[f64; 2]], style: Style, radius: f32);
}

/// Odometry ghost underneath, then the footprint (red if collided, highlighted if selected),
/// heading line and centre dot.
pub fn draw_robot(c: &mut impl Canvas, r: &RobotRuntime, selected: bool) {
    let fp = &r.config.footprint;
    let pose = r.state.pose;
    c.polyline(&footprint_polygon(fp, r.state.odom_pose), ODOM_GHOST, true);
    let style = if r.collided {
        COLLIDED
    } else if selected {
        SELECTED
    } else {
        FOOTPRINT
    };
    c.polyline(&footprint_polygon(fp, pose), style, true);
    c.polyline(&heading_segment(fp, pose), HEADING, false);
    c.points(&[[pose.x, pose.y]], HEADING, CENTER_DOT_PX);
}

/// Latest laser rays (finite returns only) and sonar cones for the sensors `visible` accepts,
/// by sensor index. Sensors that have not fired yet draw nothing.
pub fn draw_sensors(c: &mut impl Canvas, r: &RobotRuntime, visible: impl Fn(usize) -> bool) {
    for (i, sensor) in r.config.sensors.iter().enumerate() {
        if !visible(i) {
            continue;
        }
        let pose = r.sensor_world_pose(i);
        let origin = [pose.x, pose.y];
        match (&sensor.kind, &r.data[i]) {
            (SensorConfig::Laser(_), Some(Measurement::Laser(scan))) => {
                for end in scan_endpoints(scan, pose) {
                    c.polyline(&[origin, end], LASER_RAY, false);
                }
            }
            (SensorConfig::Sonar(spec), Some(Measurement::Sonar(scan))) => {
                if let Some(cone) = sonar_cone(spec.cone_angle, scan.range, pose) {
                    c.polyline(&cone, SONAR_CONE, true);
                }
            }
            _ => {}
        }
    }
}

pub fn draw_trail(c: &mut impl Canvas, xy: &[[f64; 2]], style: Style) {
    if xy.len() >= 2 {
        c.polyline(xy, style, false);
    }
}

fn xy(p: Point2D) -> [f64; 2] {
    [p.x, p.y]
}

/// Footprint outline in world coordinates.
pub fn footprint_polygon(fp: &Footprint, pose: Pose2D) -> Vec<[f64; 2]> {
    fp.vertices()
        .iter()
        .map(|&v| xy(pose.transform_point(v)))
        .collect()
}

/// Farthest footprint vertex from the body origin: the robot's reach, used for the heading
/// line and click picking.
pub fn footprint_extent(fp: &Footprint) -> f64 {
    fp.vertices()
        .iter()
        .map(|v| v.x.hypot(v.y))
        .fold(0.0, f64::max)
}

/// From the robot centre forward to the edge of the footprint.
pub fn heading_segment(fp: &Footprint, pose: Pose2D) -> [[f64; 2]; 2] {
    let tip = pose.transform_point(Point2D {
        x: footprint_extent(fp),
        y: 0.0,
    });
    [[pose.x, pose.y], xy(tip)]
}

/// World endpoint of every finite range within `[range_min, range_max]`; REP-117 infinities (no
/// return, too close) and NaN are skipped.
pub fn scan_endpoints(scan: &LaserScan, sensor_pose: Pose2D) -> Vec<[f64; 2]> {
    scan.ranges
        .iter()
        .enumerate()
        .map(|(i, &r)| (i, f64::from(r)))
        .filter(|&(_, r)| r.is_finite() && (scan.range_min..=scan.range_max).contains(&r))
        .map(|(i, r)| {
            let a = scan.angle_min + i as f64 * scan.angle_increment;
            let (s, c) = a.sin_cos();
            xy(sensor_pose.transform_point(Point2D { x: r * c, y: r * s }))
        })
        .collect()
}

/// Triangle apex → left edge → right edge at `range`; `None` for a non-finite range.
pub fn sonar_cone(cone_angle: f64, range: f64, sensor_pose: Pose2D) -> Option<[[f64; 2]; 3]> {
    if !range.is_finite() {
        return None;
    }
    let edge = |a: f64| {
        let (s, c) = a.sin_cos();
        xy(sensor_pose.transform_point(Point2D {
            x: range * c,
            y: range * s,
        }))
    };
    Some([
        [sensor_pose.x, sensor_pose.y],
        edge(cone_angle / 2.0),
        edge(-cone_angle / 2.0),
    ])
}

/// How far odometry has drifted from the truth.
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct PoseError {
    /// Position error, metres.
    pub xy: f64,
    /// Heading error `truth - odom`, wrapped to (-π, π].
    pub theta: f64,
}

pub fn pose_error(truth: Pose2D, odom: Pose2D) -> PoseError {
    PoseError {
        xy: (truth.x - odom.x).hypot(truth.y - odom.y),
        theta: angle_diff(truth.theta, odom.theta),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use stdr_core::{OccupancyGrid, RobotConfig, SimulationEngine, Twist2D};

    #[derive(Debug, PartialEq)]
    enum Call {
        Polyline(Vec<[f64; 2]>, Style, bool),
        Points(Vec<[f64; 2]>, Style, f32),
    }

    #[derive(Default)]
    struct RecordingCanvas(Vec<Call>);

    impl Canvas for RecordingCanvas {
        fn polyline(&mut self, pts: &[[f64; 2]], style: Style, closed: bool) {
            self.0.push(Call::Polyline(pts.to_vec(), style, closed));
        }
        fn points(&mut self, pts: &[[f64; 2]], style: Style, radius: f32) {
            self.0.push(Call::Points(pts.to_vec(), style, radius));
        }
    }

    fn round(calls: Vec<Call>) -> Vec<Call> {
        let r = |pts: Vec<[f64; 2]>| {
            pts.into_iter()
                .map(|p| p.map(|v| (v * 1e9).round() / 1e9 + 0.0))
                .collect()
        };
        calls
            .into_iter()
            .map(|c| match c {
                Call::Polyline(p, s, closed) => Call::Polyline(r(p), s, closed),
                Call::Points(p, s, rad) => Call::Points(r(p), s, rad),
            })
            .collect()
    }

    #[test]
    fn draw_robot_records_expected_calls() {
        let square = Footprint::Polygon(vec![
            Point2D { x: 1.0, y: 1.0 },
            Point2D { x: -1.0, y: 1.0 },
            Point2D { x: -1.0, y: -1.0 },
            Point2D { x: 1.0, y: -1.0 },
        ]);
        let cfg = RobotConfig {
            footprint: square,
            ..Default::default()
        };
        let mut engine = SimulationEngine::new(0.1, Some(0)).unwrap();
        // Row y = 5 is a wall right above the robot, so the step below is blocked: truth holds,
        // odometry advances 0.1 m along +y.
        let mut cells = vec![0i8; 100];
        cells[50..60].fill(100);
        engine.set_map(OccupancyGrid::new(10, 10, 1.0, Pose2D::default(), cells).unwrap());
        // Facing +y at (2, 3): body (1, 1) lands at world (1, 4).
        let pose = Pose2D {
            x: 2.0,
            y: 3.0,
            theta: std::f64::consts::FRAC_PI_2,
        };
        let id = engine.spawn(cfg, pose);
        engine.set_cmd_vel(
            id,
            Twist2D {
                linear_x: 1.0,
                ..Default::default()
            },
        );
        engine.step();
        let r = engine.robot(id).unwrap();
        assert!(r.collided);

        let mut c = RecordingCanvas::default();
        draw_robot(&mut c, r, true);

        assert_eq!(
            round(c.0),
            vec![
                Call::Polyline(
                    vec![[1.0, 4.1], [1.0, 2.1], [3.0, 2.1], [3.0, 4.1]],
                    ODOM_GHOST,
                    true
                ),
                Call::Polyline(
                    vec![[1.0, 4.0], [1.0, 2.0], [3.0, 2.0], [3.0, 4.0]],
                    COLLIDED,
                    true
                ),
                Call::Polyline(
                    // Heading reaches the farthest vertex: 3 + √2, rounded to 1e-9.
                    vec![[2.0, 3.0], [2.0, 4.414213562]],
                    HEADING,
                    false
                ),
                Call::Points(vec![[2.0, 3.0]], HEADING, CENTER_DOT_PX),
            ]
        );
    }

    #[test]
    fn scan_endpoints_skip_infinite_ranges() {
        let scan = LaserScan {
            angle_min: -std::f64::consts::FRAC_PI_2,
            angle_increment: std::f64::consts::FRAC_PI_2,
            range_max: 5.0,
            ranges: vec![1.0, f32::INFINITY, 2.0, f32::NEG_INFINITY],
            ..Default::default()
        };
        let pose = Pose2D {
            x: 1.0,
            y: 0.0,
            theta: 0.0,
        };
        let pts = scan_endpoints(&scan, pose);
        assert_eq!(pts.len(), 2);
        assert!((pts[0][0] - 1.0).abs() < 1e-12 && (pts[0][1] + 1.0).abs() < 1e-12);
        assert!((pts[1][0] - 1.0).abs() < 1e-12 && (pts[1][1] - 2.0).abs() < 1e-12);
    }

    #[test]
    fn sonar_cone_needs_a_finite_range() {
        assert_eq!(sonar_cone(0.5, f64::INFINITY, Pose2D::default()), None);
        let [apex, left, right] =
            sonar_cone(std::f64::consts::FRAC_PI_2, 2.0, Pose2D::default()).expect("finite range");
        assert_eq!(apex, [0.0, 0.0]);
        assert!(left[1] > 0.0 && right[1] < 0.0);
        assert!((left[0] - 2f64.sqrt()).abs() < 1e-12);
    }
}

// C++ `PlotHelpers.ScanToMapPoints*`, on `scan_endpoints` with the sensor pose composed as
// `robot_in_map * laser_in_robot`.
#[cfg(test)]
#[allow(non_snake_case)]
mod PlotHelpers {
    use super::scan_endpoints;
    use stdr_core::{LaserScan, Pose2D};

    fn one_angle(range_min: f64, range_max: f64, ranges: Vec<f32>) -> LaserScan {
        LaserScan {
            angle_min: 0.0,
            angle_increment: 0.0,
            range_min,
            range_max,
            ranges,
            ..Default::default()
        }
    }

    fn near(p: [f64; 2], x: f64, y: f64) -> bool {
        (p[0] - x).abs() < 1e-9 && (p[1] - y).abs() < 1e-9
    }

    #[test]
    fn ScanToMapPointsEmptyScanIsEmpty() {
        assert!(scan_endpoints(&LaserScan::default(), Pose2D::default()).is_empty());
    }

    #[test]
    fn ScanToMapPointsSkipsOutOfRangeAndNonFiniteRays() {
        let scan = one_angle(0.1, 5.0, vec![0.05, 6.0, f32::NAN, f32::INFINITY, 2.0]);
        let pts = scan_endpoints(&scan, Pose2D::default());
        assert_eq!(pts.len(), 1);
        assert!(near(pts[0], 2.0, 0.0));
    }

    #[test]
    fn ScanToMapPointsSingleRayAtIdentityPosesLandsOnXAxis() {
        let pts = scan_endpoints(&one_angle(0.0, 10.0, vec![3.0]), Pose2D::default());
        assert_eq!(pts.len(), 1);
        assert!(near(pts[0], 3.0, 0.0));
    }

    #[test]
    fn ScanToMapPointsRotatedRobotWithOffsetLaser() {
        let laser_in_robot = Pose2D {
            x: 0.2,
            y: 0.0,
            theta: 0.0,
        };
        let robot_in_map = Pose2D {
            x: 0.0,
            y: 0.0,
            theta: std::f64::consts::FRAC_PI_2,
        };
        let pts = scan_endpoints(
            &one_angle(0.0, 10.0, vec![1.0]),
            robot_in_map * laser_in_robot,
        );
        assert_eq!(pts.len(), 1);
        assert!(near(pts[0], 0.0, 1.2));
    }
}
