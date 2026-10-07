//! Menu bar, Start/Pause/Reset control bar with the 2D/3D toggle, status bar and the spawn dialog.

use std::path::PathBuf;

use bevy::prelude::*;
use bevy_egui::{EguiContexts, egui};
use stdr_core::{Pose2D, SimulationEngine};

use super::messages::MessageLog;
use crate::plot::{PlotterNames, ShowPlotter};
use crate::scene3d::ViewMode;
use crate::sim::{SimCommand, SimEvent, SimWorld, load_robot};

pub const SPEEDS: [f64; 4] = [0.5, 1.0, 2.0, 5.0];
pub const TIMESTEPS: [(f64, &str); 5] = [
    (0.01, "0.01 s (100 Hz)"),
    (0.05, "0.05 s (20 Hz)"),
    (0.1, "0.1 s (10 Hz)"),
    (0.2, "0.2 s (5 Hz)"),
    (0.5, "0.5 s (2 Hz)"),
];

/// A robot yaml picked via File → Load Robot, waiting for its spawn pose.
#[derive(Resource, Default)]
pub struct SpawnDialog(Option<(PathBuf, Pose2D)>);

/// `HH:MM:SS.cc`, rounded to centiseconds; negative clamps to zero.
pub fn format_elapsed_time(seconds: f64) -> String {
    let total_cs = (seconds.max(0.0) * 100.0).round() as u64;
    let (cs, s) = (total_cs % 100, total_cs / 100);
    format!("{:02}:{:02}:{:02}.{cs:02}", s / 3600, s / 60 % 60, s % 60)
}

/// Pause and speed come from `Time<Virtual>`, step size and sim time from the engine: no copies.
pub fn status_text(virt: &Time<Virtual>, sim: &SimulationEngine) -> String {
    format!(
        "{}  |  Speed: {:.1}x  |  dt: {:.3}s  |  Time: {}",
        if virt.is_paused() {
            "Paused"
        } else {
            "Running"
        },
        virt.relative_speed_f64(),
        sim.step_dt(),
        format_elapsed_time(sim.sim_time()),
    )
}

fn sim_buttons(ui: &mut egui::Ui, cmd: &mut MessageWriter<SimCommand>) {
    for (label, c) in [
        ("Start", SimCommand::Start),
        ("Pause", SimCommand::Pause),
        ("Reset", SimCommand::Reset),
    ] {
        if ui.button(label).clicked() {
            cmd.write(c);
            ui.close();
        }
    }
}

fn pick_yaml(title: &str) -> Option<PathBuf> {
    rfd::FileDialog::new()
        .set_title(title)
        .add_filter("YAML", &["yaml", "yml"])
        .pick_file()
}

#[allow(clippy::too_many_arguments)]
pub fn toolbar(
    mut ctx: EguiContexts,
    virt: Res<Time<Virtual>>,
    sim: Res<SimWorld>,
    log: Res<MessageLog>,
    mut spawn: ResMut<SpawnDialog>,
    plotters: Res<PlotterNames>,
    mut view: ResMut<ViewMode>,
    mut cmd: MessageWriter<SimCommand>,
    mut events: MessageWriter<SimEvent>,
    mut show: MessageWriter<ShowPlotter>,
    mut exit: MessageWriter<AppExit>,
) -> Result {
    let ctx = ctx.ctx_mut()?;
    let width = ctx.content_rect().width();
    // Areas rather than panels: bevy_egui gives no root `Ui`, and a background-layer root
    // would make egui claim the pointer over the whole map.
    egui::Area::new("toolbar".into())
        .fixed_pos([0.0, 0.0])
        .show(ctx, |ui| {
            egui::Frame::side_top_panel(ui.style()).show(ui, |ui| {
                ui.set_width(width);
                egui::MenuBar::new().ui(ui, |ui| {
                    ui.menu_button("File", |ui| {
                        if ui.button("Load Map...").clicked() {
                            ui.close();
                            if let Some(path) = pick_yaml("Load Map") {
                                cmd.write(SimCommand::LoadMap(path));
                            }
                        }
                        if ui.button("Load Robot...").clicked() {
                            ui.close();
                            if let Some(path) = pick_yaml("Load Robot") {
                                // Read now only to prefill the dialog with the yaml pose.
                                match load_robot(&path) {
                                    Ok((cfg, _)) => spawn.0 = Some((path, cfg.initial_pose)),
                                    Err(e) => {
                                        events.write(SimEvent::Log(format!(
                                            "Failed to load robot: {e}"
                                        )));
                                    }
                                }
                            }
                        }
                        ui.separator();
                        if ui.button("Exit").clicked() {
                            exit.write(AppExit::Success);
                        }
                    });
                    ui.menu_button("Simulation", |ui| {
                        sim_buttons(ui, &mut cmd);
                        ui.separator();
                        for s in SPEEDS {
                            let on = virt.relative_speed_f64() == s;
                            if ui.radio(on, format!("Speed {s}x")).clicked() {
                                cmd.write(SimCommand::SetSpeed(s));
                                ui.close();
                            }
                        }
                        ui.separator();
                        ui.menu_button("Timestep", |ui| {
                            for (dt, label) in TIMESTEPS {
                                let on = (sim.step_dt() - dt).abs() < 1e-9;
                                if ui.radio(on, label).clicked() {
                                    cmd.write(SimCommand::SetStepDt(dt));
                                    ui.close();
                                }
                            }
                        });
                    });
                    // Re-opens a plotter whose window was closed.
                    ui.menu_button("Plotters", |ui| {
                        for &name in &plotters.0 {
                            if ui.button(name).clicked() {
                                show.write(ShowPlotter(name));
                                ui.close();
                            }
                        }
                    });
                });
                ui.horizontal(|ui| {
                    sim_buttons(ui, &mut cmd);
                    ui.separator();
                    ui.selectable_value(&mut *view, ViewMode::TwoD, "2D");
                    ui.selectable_value(&mut *view, ViewMode::ThreeD, "3D");
                });
            });
        });

    egui::Area::new("status_bar".into())
        .anchor(egui::Align2::LEFT_BOTTOM, [0.0, 0.0])
        .show(ctx, |ui| {
            egui::Frame::side_top_panel(ui.style()).show(ui, |ui| {
                ui.set_width(width);
                ui.horizontal(|ui| {
                    ui.label(status_text(&virt, &sim));
                    if let Some(last) = log.0.back() {
                        ui.separator();
                        ui.colored_label(egui::Color32::from_rgb(255, 153, 51), last);
                    }
                });
            });
        });

    if let Some((path, pose)) = &mut spawn.0 {
        let mut open = true;
        let mut done = false;
        egui::Window::new("Spawn Robot")
            .open(&mut open)
            .collapsible(false)
            .resizable(false)
            .anchor(egui::Align2::CENTER_CENTER, [0.0, 0.0])
            .show(ctx, |ui| {
                ui.label(path.display().to_string());
                ui.separator();
                ui.add(
                    egui::DragValue::new(&mut pose.x)
                        .speed(0.1)
                        .prefix("x: ")
                        .suffix(" m"),
                );
                ui.add(
                    egui::DragValue::new(&mut pose.y)
                        .speed(0.1)
                        .prefix("y: ")
                        .suffix(" m"),
                );
                ui.add(
                    egui::Slider::new(
                        &mut pose.theta,
                        -std::f64::consts::PI..=std::f64::consts::PI,
                    )
                    .text("theta (rad)"),
                );
                ui.separator();
                ui.horizontal(|ui| {
                    if ui.button("Spawn").clicked() {
                        cmd.write(SimCommand::SpawnRobot {
                            path: path.clone(),
                            pose: *pose,
                        });
                        done = true;
                    }
                    done |= ui.button("Cancel").clicked();
                });
            });
        if done || !open {
            spawn.0 = None;
        }
    }
    Ok(())
}

// C++ `FormatElapsedTime.*`.
#[cfg(test)]
#[allow(non_snake_case)]
mod FormatElapsedTime {
    use super::format_elapsed_time;

    #[test]
    fn ZeroSeconds() {
        assert_eq!(format_elapsed_time(0.0), "00:00:00.00");
    }

    #[test]
    fn OneAndHalfSeconds() {
        assert_eq!(format_elapsed_time(1.5), "00:00:01.50");
    }

    #[test]
    fn EightyThreePointFortyFiveSeconds() {
        assert_eq!(format_elapsed_time(83.45), "00:01:23.45");
    }

    #[test]
    fn OneHourOneMinuteOneSecond() {
        assert_eq!(format_elapsed_time(3661.0), "01:01:01.00");
    }

    #[test]
    fn NegativeInputClampsToZero() {
        assert_eq!(format_elapsed_time(-5.0), "00:00:00.00");
    }
}
