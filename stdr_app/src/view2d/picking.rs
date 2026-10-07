use bevy::prelude::*;
use bevy::window::PrimaryWindow;
use bevy_egui::input::EguiWantsInput;
use bevy_egui::{EguiContexts, egui};
use stdr_core::{Point2D, Pose2D, RobotId, SimulationEngine};

use super::camera::{MainCamera, ViewLock, cursor_world, world_per_pixel};
use crate::sim::{Selection, SimCommand, SimWorld};

/// Clicks within this many pixels of a robot centre pick it even outside its footprint.
const PICK_RADIUS_PX: f32 = 15.0;

/// Where the map context menu was opened: screen position (logical px) and world point.
#[derive(Resource, Default)]
pub struct ContextMenu(Option<(Vec2, Point2D)>);

/// The robot whose footprint contains `p`, else the nearest centre within `radius`.
pub fn robot_at(sim: &SimulationEngine, p: Point2D, radius: f64) -> Option<RobotId> {
    sim.robots()
        .map(|(id, r)| {
            let pose = r.state.pose;
            let inside = r
                .config
                .footprint
                .contains(pose.inverse().transform_point(p));
            let d = if inside {
                0.0
            } else {
                (p.x - pose.x).hypot(p.y - pose.y)
            };
            (id, d)
        })
        .filter(|&(_, d)| d <= radius)
        .min_by(|a, b| a.1.total_cmp(&b.1))
        .map(|(id, _)| id)
}

/// Left click selects the robot under the cursor (an empty click keeps the selection);
/// right click opens the context menu.
pub fn pick_robot(
    egui_input: Res<EguiWantsInput>,
    buttons: Res<ButtonInput<MouseButton>>,
    window: Single<&Window, With<PrimaryWindow>>,
    cam: Single<(&Camera, &GlobalTransform, &Projection), With<MainCamera>>,
    sim: Res<SimWorld>,
    mut sel: ResMut<Selection>,
    mut menu: ResMut<ContextMenu>,
) {
    if egui_input.wants_pointer_input() {
        return;
    }
    let (camera, at, proj) = *cam;
    let Some(p) = cursor_world(&window, camera, at) else {
        return;
    };
    let p = Point2D {
        x: f64::from(p.x),
        y: f64::from(p.y),
    };
    if buttons.just_pressed(MouseButton::Left) {
        let radius = f64::from(PICK_RADIUS_PX * world_per_pixel(proj));
        if let Some(id) = robot_at(&sim, p, radius) {
            sel.robot = Some(id);
        }
    }
    if buttons.just_pressed(MouseButton::Right) {
        menu.0 = window.cursor_position().map(|screen| (screen, p));
    }
}

pub fn context_menu(
    mut ctx: EguiContexts,
    mut menu: ResMut<ContextMenu>,
    mut lock: ResMut<ViewLock>,
    sel: Res<Selection>,
    sim: Res<SimWorld>,
    mut cmd: MessageWriter<SimCommand>,
) -> Result {
    let Some((screen, p)) = menu.0 else {
        return Ok(());
    };
    let ctx = ctx.ctx_mut()?;
    let selected = sel.robot.and_then(|id| Some((id, sim.robot(id)?)));
    let mut close = false;
    let area = egui::Area::new("map_context_menu".into())
        .order(egui::Order::Foreground)
        .fixed_pos([screen.x, screen.y])
        .show(ctx, |ui| {
            egui::Frame::menu(ui.style()).show(ui, |ui| {
                let enabled = selected.is_some();
                if ui
                    .add_enabled(enabled, egui::Button::new("Teleport here"))
                    .clicked()
                    && let Some((id, r)) = selected
                {
                    // Keeps the heading (C++ parity).
                    let pose = Pose2D {
                        x: p.x,
                        y: p.y,
                        theta: r.state.pose.theta,
                    };
                    cmd.write(SimCommand::Teleport { id, pose });
                    close = true;
                }
                if ui
                    .add_enabled(enabled, egui::Button::new("Delete robot"))
                    .clicked()
                    && let Some((id, _)) = selected
                {
                    cmd.write(SimCommand::DeleteRobot(id));
                    close = true;
                }
                ui.separator();
                close |= ui.checkbox(&mut lock.0, "Lock view").clicked();
            });
        });
    let pressed_outside = ctx.input(|i| {
        i.pointer.primary_pressed()
            && i.pointer
                .interact_pos()
                .is_some_and(|pos| !area.response.rect.contains(pos))
    });
    if close || pressed_outside || ctx.input(|i| i.key_pressed(egui::Key::Escape)) {
        menu.0 = None;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use stdr_core::{Footprint, RobotConfig};

    #[test]
    fn robot_at_prefers_footprint_then_nearest_centre() {
        let mut sim = SimulationEngine::new(0.1, Some(0)).unwrap();
        let cfg = RobotConfig {
            footprint: Footprint::Circle { radius: 0.5 },
            ..Default::default()
        };
        let a = sim.spawn(cfg.clone(), Pose2D::default());
        let b = sim.spawn(
            cfg,
            Pose2D {
                x: 2.0,
                ..Default::default()
            },
        );
        let at = |x, r| robot_at(&sim, Point2D { x, y: 0.0 }, r);
        assert_eq!(at(0.4, 0.0), Some(a));
        assert_eq!(at(1.7, 0.0), Some(b));
        assert_eq!(at(1.0, 0.0), None);
        assert_eq!(at(0.8, 1.0), Some(a));
    }
}
