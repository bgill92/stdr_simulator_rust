//! Drives the selected robot in a circle and plots how far odometry drifts from the truth.

use bevy::prelude::*;
use bevy_egui::EguiContexts;
use egui_plot::{Legend, Plot};
use stdr_core::{RobotId, Twist2D};

use crate::overlay;
use crate::plot::{PlotterCtl, PlotterSample, SampleGate, TimeSeries, add_plotter, plotter_tab};
use crate::register_plotter;
use crate::sim::{Selection, SimCommand, SimWorld};
use crate::ui::dock::Dock;

const NAME: &str = "Pose Error";
const DRIVE: Twist2D = Twist2D {
    linear_x: 0.3,
    linear_y: 0.0,
    angular_z: 0.5,
};
/// Sim seconds between plotted points.
const SAMPLE_PERIOD: f64 = 0.05;

#[derive(Resource)]
struct PoseError {
    /// The robot being driven and plotted; a selection change starts over.
    robot: Option<RobotId>,
    xy: TimeSeries,
    th: TimeSeries,
    gate: SampleGate,
}

impl Default for PoseError {
    fn default() -> Self {
        Self {
            robot: None,
            xy: TimeSeries::default(),
            th: TimeSeries::default(),
            gate: SampleGate::new(SAMPLE_PERIOD),
        }
    }
}

fn stop(id: Option<RobotId>, cmd: &mut MessageWriter<SimCommand>) {
    if let Some(id) = id {
        cmd.write(SimCommand::CmdVel {
            id,
            twist: Twist2D::default(),
        });
    }
}

fn sample(
    sim: Res<SimWorld>,
    sel: Res<Selection>,
    mut st: ResMut<PoseError>,
    mut cmd: MessageWriter<SimCommand>,
) {
    if st.robot != sel.robot {
        // The previously driven robot must not keep circling.
        stop(st.robot, &mut cmd);
        *st = PoseError {
            robot: sel.robot,
            ..default()
        };
    }
    let Some(id) = st.robot else { return };
    cmd.write(SimCommand::CmdVel { id, twist: DRIVE });
    let Some(r) = sim.robot(id) else { return };
    let t = sim.sim_time();
    if !st.gate.due(t) {
        return;
    }
    let e = overlay::pose_error(r.state.pose, r.state.odom_pose);
    st.xy.push(t, e.xy);
    st.th.push(t, e.theta);
}

/// Pausing or closing the plotter stops the robot it was driving (edge-triggered, so teleop
/// works while the plotter is paused).
fn stop_when_inactive(
    ctl: Res<PlotterCtl<PoseError>>,
    st: Res<PoseError>,
    mut cmd: MessageWriter<SimCommand>,
    mut was_active: Local<bool>,
) {
    let active = !ctl.paused && !ctl.removed;
    if *was_active && !active {
        stop(st.robot, &mut cmd);
    }
    *was_active = active;
}

fn render(
    mut ctx: EguiContexts,
    dock: Res<Dock>,
    st: Res<PoseError>,
    mut ctl: ResMut<PlotterCtl<PoseError>>,
) -> Result {
    plotter_tab(ctx.ctx_mut()?, &dock, &mut ctl, |ui| {
        if st.robot.is_none() {
            ui.label("Select a robot to drive.");
        }
        Plot::new("pose_error")
            .view_aspect(1.6)
            .x_axis_label("sim time (s)")
            .y_axis_label("error")
            .legend(Legend::default())
            .show(ui, |p| {
                p.line(st.xy.line("|truth - odom| (m)"));
                p.line(st.th.line("yaw error (rad)"));
            });
    });
    Ok(())
}

register_plotter!(
    PoseError,
    NAME,
    "Drives the selected robot in a circle and plots truth vs odometry.",
    |app| {
        add_plotter::<PoseError, _, _>(app, NAME, sample, render);
        app.add_systems(Update, stop_when_inactive.in_set(PlotterSample));
    }
);
