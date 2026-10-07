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
/// Colour steps of the age gradient; each step is drawn as one plot item.
const AGE_BINS: usize = 16;

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

/// `xy` split oldest-first into at most `AGE_BINS` consecutive runs, each with its age colour.
fn age_bins(xy: &[[f64; 2]]) -> Vec<([u8; 4], Vec<[f64; 2]>)> {
    let n = AGE_BINS.min(xy.len());
    let mut bins = vec![Vec::new(); n];
    for (i, pt) in xy.iter().enumerate() {
        bins[i * n / xy.len()].push(*pt);
    }
    bins.into_iter()
        .enumerate()
        .map(|(b, pts)| (age_color(b, n), pts))
        .collect()
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
            for (color, pts) in age_bins(xy) {
                let style = Style { color, width: 1.0 };
                c.points(&pts, style, MARKER_PX);
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
    use super::{AGE_BINS, age_bins, age_color};

    #[test]
    fn age_color_runs_blue_to_red() {
        assert_eq!(age_color(0, 3), [0, 0, 255, 255]);
        assert_eq!(age_color(2, 3), [255, 0, 0, 255]);
        assert_eq!(age_color(0, 1), [255, 0, 0, 255]);
    }

    #[test]
    fn age_bins_keep_every_point_in_order_and_run_blue_to_red() {
        let xy: Vec<[f64; 2]> = (0..500).map(|i| [f64::from(i), 0.0]).collect();
        let bins = age_bins(&xy);
        assert_eq!(bins.len(), AGE_BINS);
        assert_eq!(bins[0].0, [0, 0, 255, 255]);
        assert_eq!(bins[AGE_BINS - 1].0, [255, 0, 0, 255]);
        let flat: Vec<[f64; 2]> = bins.into_iter().flat_map(|(_, pts)| pts).collect();
        assert_eq!(flat, xy);
    }

    #[test]
    fn age_bins_colour_each_point_of_a_short_trail() {
        let xy = [[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]];
        let colors: Vec<_> = age_bins(&xy).into_iter().map(|(c, _)| c).collect();
        assert_eq!(colors, (0..3).map(|i| age_color(i, 3)).collect::<Vec<_>>());
        assert!(age_bins(&[]).is_empty());
    }
}
