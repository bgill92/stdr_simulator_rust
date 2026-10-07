//! Odometry Trace plus the laser scan at a clicked breadcrumb, placed from that breadcrumb's truth
//! or odometry pose. Truth, odometry and scan are stored together per breadcrumb so one scan can
//! be replayed from either pose.

use bevy::prelude::*;
use bevy_egui::{EguiContexts, egui};
use egui_plot::PlotPoint;
use stdr_core::{LaserScan, Point2D, Pose2D, RobotId, RobotRuntime, SensorConfig};

use super::odometry_trace::{error_readout, trail_key};
use crate::overlay::{self, Canvas};
use crate::plot::overlay_plot::{ODOM, PlotCanvas, SCAN, TRUTH, draw_map, map_plot};
use crate::plot::{
    LockView, MARKER_SPACING, MAX_MARKERS, PlotterCtl, Positioned, Push, TeleportDetector, Trail,
    add_plotter, plotter_window,
};
use crate::register_plotter;
use crate::sim::{Selection, SimWorld};
use crate::view2d::MapTexture;

const NAME: &str = "Scan Trace";
/// A click farther than this from every breadcrumb clears the selection.
const PICK_RADIUS_PX: f32 = 8.0;
const BREADCRUMB_PX: f32 = 2.0;
const SELECTED_PX: f32 = 6.0;
const SCAN_POINT_PX: f32 = 1.5;

#[derive(Clone, Debug)]
struct Sample {
    truth: Pose2D,
    odom: Pose2D,
    /// The robot's first laser at that moment; `None` without a laser or before it fired.
    scan: Option<LaserScan>,
}

impl Positioned for Sample {
    fn position(&self) -> Point2D {
        self.truth.position()
    }
}

impl Sample {
    fn pose(&self, from_odom: bool) -> Pose2D {
        if from_odom { self.odom } else { self.truth }
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
struct Selected {
    index: usize,
    from_odom: bool,
}

#[derive(Resource)]
struct ScanTrace {
    robot: Option<RobotId>,
    samples: Trail<Sample>,
    jumps: TeleportDetector,
    selected: Option<Selected>,
}

impl Default for ScanTrace {
    fn default() -> Self {
        Self {
            robot: None,
            samples: Trail::new(MAX_MARKERS, MARKER_SPACING),
            jumps: TeleportDetector::default(),
            selected: None,
        }
    }
}

fn first_laser(r: &RobotRuntime) -> Option<usize> {
    r.config
        .sensors
        .iter()
        .position(|s| matches!(s.kind, SensorConfig::Laser(_)))
}

/// The selection follows its breadcrumb when the oldest one is dropped, and goes with it.
fn after_eviction(sel: Option<Selected>) -> Option<Selected> {
    let s = sel?;
    Some(Selected {
        index: s.index.checked_sub(1)?,
        ..s
    })
}

fn sample(sim: Res<SimWorld>, sel: Res<Selection>, mut st: ResMut<ScanTrace>) {
    if st.robot != sel.robot {
        *st = ScanTrace {
            robot: sel.robot,
            ..default()
        };
    }
    let Some(r) = st.robot.and_then(|id| sim.robot(id)) else {
        return;
    };
    if st.jumps.observe(r.state.pose.position()) {
        st.samples.clear();
        st.selected = None;
    }
    let scan = first_laser(r).and_then(|i| r.data[i].as_ref()?.as_laser().cloned());
    let pushed = st.samples.push_if_moved(Sample {
        truth: r.state.pose,
        odom: r.state.odom_pose,
        scan,
    });
    if pushed
        == (Push::Pushed {
            evicted_front: true,
        })
    {
        st.selected = after_eviction(st.selected);
    }
}

/// The breadcrumb (truth or odometry) nearest `pointer` in screen pixels, within the pick radius.
fn pick(
    samples: &Trail<Sample>,
    pointer: egui::Pos2,
    to_screen: impl Fn(Pose2D) -> egui::Pos2,
) -> Option<Selected> {
    let mut best = None;
    let mut best_d = PICK_RADIUS_PX;
    for (index, s) in samples.iter().enumerate() {
        for from_odom in [false, true] {
            let d = to_screen(s.pose(from_odom)).distance(pointer);
            if d <= best_d {
                best_d = d;
                best = Some(Selected { index, from_odom });
            }
        }
    }
    best
}

fn render(
    mut ctx: EguiContexts,
    sim: Res<SimWorld>,
    tex: Res<MapTexture>,
    mut st: ResMut<ScanTrace>,
    mut ctl: ResMut<PlotterCtl<ScanTrace>>,
    mut lock: Local<LockView>,
) -> Result {
    let robot = st.robot.and_then(|id| sim.robot(id));
    let laser = robot.and_then(|r| Some((r, first_laser(r)?)));
    plotter_window(ctx.ctx_mut()?, &mut ctl, |ui| {
        let st = &mut *st;
        let shown = st
            .selected
            .and_then(|s| Some((s, st.samples.get(s.index)?)));
        let truth = st.samples.xy();
        let odom: Vec<[f64; 2]> = st.samples.iter().map(|s| [s.odom.x, s.odom.y]).collect();
        let resp = map_plot("scan_trace", lock.0).show(ui, |p| {
            draw_map(p, &tex);
            let mut c = PlotCanvas(p);
            for (xy, style) in [(truth, TRUTH), (odom.as_slice(), ODOM)] {
                overlay::draw_trail(&mut c, xy, style);
                c.points(xy, style, BREADCRUMB_PX);
            }
            if let Some(r) = robot {
                overlay::draw_robot(&mut c, r, false);
            }
            // Only the selected scan is drawn, never all of them.
            if let Some((s, sample)) = shown {
                let pose = sample.pose(s.from_odom);
                let style = if s.from_odom { ODOM } else { TRUTH };
                c.points(&[[pose.x, pose.y]], style, SELECTED_PX);
                if let Some(r) = robot {
                    let fp = &r.config.footprint;
                    c.polyline(&overlay::footprint_polygon(fp, pose), style, true);
                    c.polyline(&overlay::heading_segment(fp, pose), style, false);
                }
                if let (Some(scan), Some((r, i))) = (&sample.scan, laser) {
                    let sensor = pose * r.config.sensors[i].common.pose;
                    c.points(&overlay::scan_endpoints(scan, sensor), SCAN, SCAN_POINT_PX);
                }
            }
        });
        let pointer = resp.response.interact_pointer_pos();
        if resp.response.clicked()
            && let Some(at) = pointer
        {
            let t = &resp.transform;
            st.selected = pick(&st.samples, at, |p| {
                t.position_from_point(&PlotPoint::new(p.x, p.y))
            });
        } else if resp.response.secondary_clicked() {
            st.selected = None;
        }

        trail_key(ui);
        ui.checkbox(&mut lock.0, "Lock view to map");
        if robot.is_some() && laser.is_none() {
            ui.weak("Robot has no laser sensor.");
        }
        if let Some(r) = robot {
            error_readout(ui, r);
        }
        match st
            .selected
            .and_then(|s| Some((s, st.samples.get(s.index)?)))
        {
            Some((s, sample)) => {
                ui.horizontal(|ui| {
                    let rays = sample.scan.as_ref().map_or(0, |scan| scan.ranges.len());
                    let from = if s.from_odom { "odometry" } else { "truth" };
                    ui.label(format!(
                        "Selected: {from} breadcrumb #{}  ({rays} rays)",
                        s.index
                    ));
                    if ui.button("Clear selection").clicked() {
                        st.selected = None;
                    }
                });
            }
            None => {
                ui.weak("Click a breadcrumb to show its scan; right-click clears.");
            }
        }
    });
    Ok(())
}

register_plotter!(
    ScanTrace,
    NAME,
    "Odometry Trace plus the laser scan at any clicked breadcrumb, placed from the truth or odometry pose.",
    |app| add_plotter::<ScanTrace, _, _>(app, NAME, sample, render)
);

#[cfg(test)]
mod tests {
    use super::*;

    fn at(x: f64) -> Pose2D {
        Pose2D {
            x,
            ..Default::default()
        }
    }

    fn trail() -> Trail<Sample> {
        let mut t = Trail::new(MAX_MARKERS, MARKER_SPACING);
        for (truth, odom) in [(0.0, 0.0), (1.0, 1.5)] {
            t.push_if_moved(Sample {
                truth: at(truth),
                odom: at(odom),
                scan: None,
            });
        }
        t
    }

    /// 100 px per metre along x.
    fn to_screen(p: Pose2D) -> egui::Pos2 {
        egui::pos2((p.x * 100.0) as f32, 0.0)
    }

    #[test]
    fn pick_takes_the_nearest_breadcrumb_within_radius() {
        let t = trail();
        let s = |index, from_odom| Some(Selected { index, from_odom });
        assert_eq!(pick(&t, egui::pos2(103.0, 0.0), to_screen), s(1, false));
        assert_eq!(pick(&t, egui::pos2(147.0, 0.0), to_screen), s(1, true));
        assert_eq!(pick(&t, egui::pos2(125.0, 0.0), to_screen), None);
    }

    #[test]
    fn selection_shifts_with_eviction_and_drops_with_its_breadcrumb() {
        let s = |index| Selected {
            index,
            from_odom: true,
        };
        assert_eq!(after_eviction(Some(s(3))), Some(s(2)));
        assert_eq!(after_eviction(Some(s(0))), None);
        assert_eq!(after_eviction(None), None);
    }
}
