use std::collections::HashMap;

use bevy::prelude::*;
use stdr_core::{Pose2D, RobotId};

use super::camera::{MainCamera, world_per_pixel};
use crate::overlay::{self, Canvas, Style};
use crate::plot::Trail;
use crate::sim::{Selection, SensorVisibility, SimCommand, SimWorld};

/// Truth-pose samples kept per robot.
const TRAIL_CAP: usize = 2000;
/// Metres the robot must move before the next sample.
const TRAIL_SPACING: f64 = 0.05;

/// Each robot's truth-pose path on the map.
#[derive(Resource, Default)]
pub struct Trails(pub HashMap<RobotId, Trail<Pose2D>>);

/// `Canvas` over Bevy gizmos: the only place overlay f64 world coordinates become `Vec2`.
/// Gizmo line width is global, so `Style::width` is ignored here.
pub struct GizmoCanvas<'a, 'w, 's> {
    pub gizmos: &'a mut Gizmos<'w, 's>,
    /// World units per screen pixel, for pixel-sized points.
    pub world_per_px: f32,
}

fn v2(p: &[f64; 2]) -> Vec2 {
    Vec2::new(p[0] as f32, p[1] as f32)
}

fn color(s: Style) -> Color {
    let [r, g, b, a] = s.color;
    Color::srgba_u8(r, g, b, a)
}

impl Canvas for GizmoCanvas<'_, '_, '_> {
    fn polyline(&mut self, pts: &[[f64; 2]], style: Style, closed: bool) {
        let close = pts.first().filter(|_| closed);
        self.gizmos
            .linestrip_2d(pts.iter().chain(close).map(v2), color(style));
    }

    fn points(&mut self, pts: &[[f64; 2]], style: Style, radius: f32) {
        for p in pts {
            self.gizmos
                .circle_2d(v2(p), radius * self.world_per_px, color(style));
        }
    }
}

/// After `apply_sim_commands`: a reset or teleport starts a fresh trail so no line joins the
/// old and new poses; deleted robots lose theirs.
pub fn sample_trails(
    mut commands: MessageReader<SimCommand>,
    sim: Res<SimWorld>,
    mut trails: ResMut<Trails>,
) {
    for c in commands.read() {
        match c {
            SimCommand::Reset => trails.0.clear(),
            SimCommand::Teleport { id, .. } => {
                trails.0.remove(id);
            }
            _ => {}
        }
    }
    trails.0.retain(|id, _| sim.robot(*id).is_some());
    for (id, r) in sim.robots() {
        trails
            .0
            .entry(id)
            .or_insert_with(|| Trail::new(TRAIL_CAP, TRAIL_SPACING))
            .push_if_moved(r.state.pose);
    }
}

pub fn draw_overlay(
    mut gizmos: Gizmos,
    sim: Res<SimWorld>,
    trails: Res<Trails>,
    sel: Res<Selection>,
    vis: Res<SensorVisibility>,
    proj: Single<&Projection, With<MainCamera>>,
) {
    let mut c = GizmoCanvas {
        gizmos: &mut gizmos,
        world_per_px: world_per_pixel(&proj),
    };
    for (id, r) in sim.robots() {
        if let Some(trail) = trails.0.get(&id) {
            overlay::draw_trail(&mut c, trail.xy(), overlay::TRAIL);
        }
        overlay::draw_sensors(&mut c, r, |i| vis.0.contains(&(id, i)));
        overlay::draw_robot(&mut c, r, sel.robot == Some(id));
    }
}
