//! The map with the selected robot and its breadcrumb trail, coloured by age (oldest blue, latest
//! red).

use bevy::prelude::*;
use bevy_egui::EguiContexts;
use stdr_core::{Pose2D, RobotId};

use crate::overlay::{self, Canvas, Style};
use crate::plot::overlay_plot::{PlotCanvas, draw_map, map_plot};
use crate::plot::{
    LockView, MARKER_SPACING, MAX_MARKERS, PlotterCtl, Positioned, TeleportDetector, Trail,
    add_plotter, plotter_window,
};
use crate::register_plotter;
use crate::sim::{Selection, SimWorld};
use crate::view2d::MapTexture;

const NAME: &str = "Map Trace";
const PATH: Style = Style {
    color: [153, 153, 153, 180],
    width: 1.5,
};
/// Breadcrumb dot radius, screen pixels.
const MARKER_PX: f32 = 2.5;

#[derive(Resource)]
struct MapTrace {
    robot: Option<RobotId>,
    trail: Trail<Pose2D>,
    jumps: TeleportDetector,
}

impl Default for MapTrace {
    fn default() -> Self {
        Self {
            robot: None,
            trail: Trail::new(MAX_MARKERS, MARKER_SPACING),
            jumps: TeleportDetector::default(),
        }
    }
}

fn sample(sim: Res<SimWorld>, sel: Res<Selection>, mut st: ResMut<MapTrace>) {
    if st.robot != sel.robot {
        *st = MapTrace {
            robot: sel.robot,
            ..default()
        };
    }
    let Some(r) = st.robot.and_then(|id| sim.robot(id)) else {
        return;
    };
    let pose = r.state.pose;
    if st.jumps.observe(pose.position()) {
        st.trail.clear();
    }
    st.trail.push_if_moved(pose);
}

/// Blue for the oldest of `n` breadcrumbs through to red for the latest.
fn age_color(i: usize, n: usize) -> [u8; 4] {
    let t = if n > 1 {
        i as f64 / (n - 1) as f64
    } else {
        1.0
    };
    let r = (255.0 * t).round() as u8;
    [r, 0, 255 - r, 255]
}

fn render(
    mut ctx: EguiContexts,
    sim: Res<SimWorld>,
    tex: Res<MapTexture>,
    st: Res<MapTrace>,
    mut ctl: ResMut<PlotterCtl<MapTrace>>,
    mut lock: Local<LockView>,
) -> Result {
    let robot = st.robot.and_then(|id| sim.robot(id));
    plotter_window(ctx.ctx_mut()?, &mut ctl, |ui| {
        map_plot("map_trace", lock.0).show(ui, |p| {
            draw_map(p, &tex);
            let mut c = PlotCanvas(p);
            let xy = st.trail.xy();
            overlay::draw_trail(&mut c, xy, PATH);
            for (i, pt) in xy.iter().enumerate() {
                let style = Style {
                    color: age_color(i, xy.len()),
                    width: 1.0,
                };
                c.points(&[*pt], style, MARKER_PX);
            }
            if let Some(r) = robot {
                overlay::draw_robot(&mut c, r, false);
            }
        });
        ui.checkbox(&mut lock.0, "Lock view to map");
    });
    Ok(())
}

register_plotter!(
    MapTrace,
    NAME,
    "Map with the selected robot and its breadcrumb trail (latest = red, oldest = blue).",
    |app| add_plotter::<MapTrace, _, _>(app, NAME, sample, render)
);

#[cfg(test)]
mod tests {
    use super::age_color;

    #[test]
    fn age_color_runs_blue_to_red() {
        assert_eq!(age_color(0, 3), [0, 0, 255, 255]);
        assert_eq!(age_color(2, 3), [255, 0, 0, 255]);
        assert_eq!(age_color(0, 1), [255, 0, 0, 255]);
    }
}
