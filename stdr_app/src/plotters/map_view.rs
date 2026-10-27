//! The map with the selected robot at its current pose, and nothing else.
//!
//! This is the smallest complete plotter, kept as a starting point for new ones: copy this file,
//! rename the state type, `NAME` and the plot id, add the module to `plotters/mod.rs`, then grow
//! the state, sample and render pieces below. Every plotter is the same four pieces:
//!
//! 1. a state `Resource` (here [`MapView`]) holding whatever the plotter has collected;
//! 2. a sample system ([`sample`]) that reads the simulation into that state each frame;
//! 3. a render system ([`render`]) that draws the state in the plotter's Plots tab;
//! 4. a `register_plotter!` entry so `--plotter` and the Plotters menu know about it.
//!
//! The framework in `crate::plot` (`add_plotter`) wires these together and owns the lifecycle, so
//! a plotter never handles it itself:
//! - Pause (the tab's checkbox) skips the sample system; the state keeps what it has and the
//!   render system keeps drawing it.
//! - Closing the tab removes the plotter: neither system runs until the Plotters menu shows it
//!   again, and then the state starts over from `Default`.
//! - A simulation reset puts the state back to `Default` too.

use bevy::prelude::*;
use bevy_egui::EguiContexts;
use stdr_core::RobotId;

use crate::overlay;
use crate::plot::overlay_plot::{PlotCanvas, draw_map, map_plot};
use crate::plot::{LockView, PlotterCtl, add_plotter, plotter_tab};
use crate::register_plotter;
use crate::sim::{Selection, SimWorld};
use crate::ui::dock::Dock;
use crate::view2d::MapTexture;

/// The tab title and Plotters menu entry. The framework also uses it to address this plotter's tab,
/// so it must be unique.
const NAME: &str = "Map View";

/// The plotter's state. `add_plotter` creates it with `Default` and resets it to `Default` on a
/// simulation reset or when a closed tab is shown again, so `Default` must be the empty state.
///
/// A new plotter adds its collected data here: Map Trace keeps a `Trail` of poses, the time-series
/// plotters keep `TimeSeries` lines. Map View needs only which robot it is following.
#[derive(Resource, Default)]
struct MapView {
    robot: Option<RobotId>,
}

/// Runs every `Update` while the plotter is neither paused nor removed (the framework's run
/// condition), so it only ever sees live frames.
///
/// Read the simulation through `Res<SimWorld>` and store what the render system needs in the
/// state. Following `Selection` here, rather than in `render`, is what lets Pause hold the view
/// on the robot it was showing.
fn sample(sel: Res<Selection>, mut st: ResMut<MapView>) {
    if st.robot != sel.robot {
        // A plotter with collected data starts over when the robot changes, as Map Trace does:
        // `*st = MapView { robot: sel.robot, ..default() };`
        st.robot = sel.robot;
    }
}

/// Runs in the egui pass while the plotter is not removed, paused or not. It should only draw: all
/// state changes belong in `sample`, so pausing freezes what `sample` stored (here, which robot is
/// followed). Anything read straight from `SimWorld` stays live, so the robot keeps moving while
/// paused; a plotter that should freeze its whole display stores that data in its state instead.
///
/// `plotter_tab` draws the tab (with its Pause checkbox) only while it is the selected Plots tab,
/// so the body closure costs nothing when hidden. `PlotterCtl<MapView>` carries the pause flag the
/// checkbox edits.
fn render(
    mut ctx: EguiContexts,
    dock: Res<Dock>,
    sim: Res<SimWorld>,
    tex: Res<MapTexture>,
    st: Res<MapView>,
    mut ctl: ResMut<PlotterCtl<MapView>>,
    // A UI preference, not plot data: a `Local` survives resets, which the state does not.
    mut lock: Local<LockView>,
) -> Result {
    let robot = st.robot.and_then(|id| sim.robot(id));
    plotter_tab(ctx.ctx_mut()?, &dock, &mut ctl, |ui| {
        // Plot coordinates are world metres. The id must be unique among plots.
        map_plot("map_view", lock.0).show(ui, |p| {
            draw_map(p, &tex);
            // `PlotCanvas` lets the 2D view's `overlay::draw_*` routines draw into this plot, so
            // trails, scans and sensors can be added here the way the other map plotters do.
            let mut c = PlotCanvas(p);
            if let Some(r) = robot {
                overlay::draw_robot(&mut c, r, false);
            }
        });
        // Extra controls go below the plot, like this one.
        ui.checkbox(&mut lock.0, "Lock view to map");
    });
    Ok(())
}

// The key (`MapView`) is what `--plotter` matches; keep it the state type's name. The build
// closure runs only when `--plotter` selects this plotter (or no filter is given).
register_plotter!(
    MapView,
    NAME,
    "Map with the selected robot at its current pose; a template for new plotters.",
    |app| add_plotter::<MapView, _, _>(app, NAME, sample, render)
);
