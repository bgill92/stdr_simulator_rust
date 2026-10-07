//! The map with the selected robot's true path (green) and its odometry belief (orange). Under
//! `odometry_model: perfect` the trails coincide; under `velocity` they diverge as odometry
//! drifts. Does not drive the robot: use teleop or Pose Error.

use bevy::prelude::*;
use bevy_egui::{EguiContexts, egui};
use stdr_core::{Pose2D, RobotId};

use crate::overlay;
use crate::plot::overlay_plot::{ODOM, PlotCanvas, TRUTH, color32, draw_map, map_plot};
use crate::plot::{
    LockView, MARKER_SPACING, MAX_MARKERS, PlotterCtl, Positioned, TeleportDetector, Trail,
    add_plotter, plotter_window,
};
use crate::register_plotter;
use crate::sim::{Selection, SimWorld};
use crate::view2d::MapTexture;

const NAME: &str = "Odometry Trace";

#[derive(Resource)]
struct OdometryTrace {
    robot: Option<RobotId>,
    truth: Trail<Pose2D>,
    odom: Trail<Pose2D>,
    jumps: TeleportDetector,
}

impl Default for OdometryTrace {
    fn default() -> Self {
        Self {
            robot: None,
            truth: Trail::new(MAX_MARKERS, MARKER_SPACING),
            odom: Trail::new(MAX_MARKERS, MARKER_SPACING),
            jumps: TeleportDetector::default(),
        }
    }
}

fn sample(sim: Res<SimWorld>, sel: Res<Selection>, mut st: ResMut<OdometryTrace>) {
    if st.robot != sel.robot {
        *st = OdometryTrace {
            robot: sel.robot,
            ..default()
        };
    }
    let Some(r) = st.robot.and_then(|id| sim.robot(id)) else {
        return;
    };
    let s = &r.state;
    if st.jumps.observe(s.pose.position()) {
        st.truth.clear();
        st.odom.clear();
    }
    // Each trail is spaced on its own last point, so drift changes their densities independently.
    st.truth.push_if_moved(s.pose);
    st.odom.push_if_moved(s.odom_pose);
}

/// The coloured "truth / odometry" key under a trace plot.
pub(super) fn trail_key(ui: &mut egui::Ui) {
    ui.horizontal(|ui| {
        ui.colored_label(color32(TRUTH), "— truth");
        ui.colored_label(color32(ODOM), "— odometry");
    });
}

/// Latest position and yaw error of `r`.
pub(super) fn error_readout(ui: &mut egui::Ui, r: &stdr_core::RobotRuntime) {
    let e = overlay::pose_error(r.state.pose, r.state.odom_pose);
    ui.label(format!(
        "Position error: {:.3} m   Yaw error: {:.3} rad",
        e.xy, e.theta
    ));
}

fn render(
    mut ctx: EguiContexts,
    sim: Res<SimWorld>,
    tex: Res<MapTexture>,
    st: Res<OdometryTrace>,
    mut ctl: ResMut<PlotterCtl<OdometryTrace>>,
    mut lock: Local<LockView>,
) -> Result {
    let robot = st.robot.and_then(|id| sim.robot(id));
    plotter_window(ctx.ctx_mut()?, &mut ctl, |ui| {
        map_plot("odometry_trace", lock.0).show(ui, |p| {
            draw_map(p, &tex);
            let mut c = PlotCanvas(p);
            overlay::draw_trail(&mut c, st.truth.xy(), TRUTH);
            overlay::draw_trail(&mut c, st.odom.xy(), ODOM);
            if let Some(r) = robot {
                overlay::draw_robot(&mut c, r, false);
            }
        });
        trail_key(ui);
        ui.checkbox(&mut lock.0, "Lock view to map");
        if let Some(r) = robot {
            error_readout(ui, r);
        }
    });
    Ok(())
}

register_plotter!(
    OdometryTrace,
    NAME,
    "Map with the true pose (green) and the odometry belief (orange); trails diverge as odometry drifts.",
    |app| add_plotter::<OdometryTrace, _, _>(app, NAME, sample, render)
);
