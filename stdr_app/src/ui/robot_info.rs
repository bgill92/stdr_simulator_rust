use bevy::prelude::*;
use bevy_egui::{EguiContexts, egui};
use stdr_core::{Measurement, Pose2D};

use super::dock::{Dock, show_pane};
use crate::sim::{Selection, SensorVisibility, SimCommand, SimWorld};

fn pose_text(label: &str, p: Pose2D) -> String {
    format!("{label}  x={:.3}  y={:.3}  θ={:.3}", p.x, p.y, p.theta)
}

/// Every robot: pose, odometry, command, and one row per sensor (draw toggle, rate, latest
/// reading). Clicking a robot's name selects it.
pub fn robot_info(
    mut ctx: EguiContexts,
    dock: Res<Dock>,
    sim: Res<SimWorld>,
    mut sel: ResMut<Selection>,
    mut vis: ResMut<SensorVisibility>,
    mut cmd: MessageWriter<SimCommand>,
) -> Result {
    show_pane(ctx.ctx_mut()?, "robot_info", dock.robot_info, |ui| {
        let dt = sim.step_dt();
        ui.label(format!("Sim step: {dt:.3} s ({:.1} Hz)", 1.0 / dt));
        ui.separator();
        if sim.robots().next().is_none() {
            ui.label("No robots in simulation.");
        }
        for (id, r) in sim.robots() {
            if ui
                .selectable_label(sel.robot == Some(id), id.to_string())
                .clicked()
            {
                sel.robot = Some(id);
            }
            ui.indent(id, |ui| {
                let s = &r.state;
                ui.monospace(pose_text("Pose:", s.pose));
                ui.monospace(pose_text("Odom:", s.odom_pose));
                let v = s.cmd_vel;
                ui.monospace(format!(
                    "Vel:   vx={:.3}  vy={:.3}  ω={:.3}",
                    v.linear_x, v.linear_y, v.angular_z
                ));
                if r.collided {
                    ui.colored_label(egui::Color32::RED, "Collided");
                }
                for (i, sensor) in r.config.sensors.iter().enumerate() {
                    ui.horizontal(|ui| {
                        let mut shown = vis.0.contains(&(id, i));
                        let label = format!("{} {}", sensor.kind.name(), sensor.common.frame_id);
                        if ui.checkbox(&mut shown, label).changed() {
                            if shown {
                                vis.0.insert((id, i));
                            } else {
                                vis.0.remove(&(id, i));
                            }
                        }
                        if let Some(hz) = sim.effective_rate(id, i) {
                            ui.label(format!("{hz:.1} Hz"));
                        }
                        match &r.data[i] {
                            Some(Measurement::Laser(scan)) => {
                                ui.label(format!("{} rays", scan.ranges.len()));
                            }
                            Some(Measurement::Sonar(scan)) => {
                                ui.label(format!("{:.3} m", scan.range));
                            }
                            None => {}
                        }
                    });
                }
                if ui.button("Delete").clicked() {
                    cmd.write(SimCommand::DeleteRobot(id));
                }
            });
            ui.separator();
        }
    });
    Ok(())
}
