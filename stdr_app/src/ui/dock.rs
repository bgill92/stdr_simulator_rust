//! The docked layout under the menu bar (C++ parity): Map on the left with Robot Info and Teleop
//! docked below it, Plots on the right with one tab per open plotter. The toolbar system lays the
//! panes out once per frame into [`Dock`]; each pane's own system then draws into its rect, and
//! the map cameras render only inside the Map pane.

use bevy::camera::Viewport;
use bevy::prelude::*;
use bevy::window::PrimaryWindow;
use bevy_egui::egui;

use crate::plot::ClosePlotter;
use crate::scene3d::OrbitCamera;
use crate::view2d::MainCamera;

/// The camera images' tab, after the plotters'. Not closable: it shows while a camera exists.
pub const CAMERAS_TAB: &str = "Cameras";
/// Share of the window width the Plots column starts with (C++ `kHorizontalSplitFraction`).
const PLOTS_SHARE: f32 = 0.5;
/// Share of the left column the Map starts with (C++ `kTopRowFraction`).
const MAP_SHARE: f32 = 0.7;
/// Width the Teleop pane starts with, inside the bottom-left dock.
const TELEOP_WIDTH: f32 = 280.0;
/// Narrowest either column can be dragged.
const MIN_COLUMN: f32 = 200.0;

/// This frame's pane rects (egui points = window logical pixels) and the selected Plots tab.
#[derive(Resource, Default)]
pub struct Dock {
    pub map: Option<egui::Rect>,
    pub robot_info: Option<egui::Rect>,
    pub teleop: Option<egui::Rect>,
    /// The selected tab's body, under the tab strip.
    pub plots: Option<egui::Rect>,
    pub active_tab: Option<&'static str>,
}

impl Dock {
    /// The body rect if `tab` is the selected Plots tab.
    pub fn tab_body(&self, tab: &str) -> Option<egui::Rect> {
        self.plots.filter(|_| self.active_tab == Some(tab))
    }
}

/// Plots tabs in order: open plotters in build order, then Cameras.
pub fn plot_tabs(
    names: &[&'static str],
    open: impl Fn(&str) -> bool,
    cameras: bool,
) -> Vec<&'static str> {
    let mut tabs: Vec<_> = names.iter().copied().filter(|n| open(n)).collect();
    if cameras {
        tabs.push(CAMERAS_TAB);
    }
    tabs
}

/// Keeps the selected tab while it exists, else falls back to the first.
pub fn active_tab(tabs: &[&'static str], current: Option<&str>) -> Option<&'static str> {
    current
        .and_then(|c| tabs.iter().copied().find(|&t| t == c))
        .or_else(|| tabs.first().copied())
}

/// The Map pane as a camera viewport: logical rect × `scale` to physical pixels, clamped to the
/// window. None while the pane is empty.
pub fn map_viewport(rect: egui::Rect, scale: f32, window: UVec2) -> Option<Viewport> {
    let min = (Vec2::new(rect.min.x, rect.min.y) * scale)
        .round()
        .max(Vec2::ZERO);
    let max = (Vec2::new(rect.max.x, rect.max.y) * scale)
        .round()
        .min(window.as_vec2());
    let size = (max - min).max(Vec2::ZERO).as_uvec2();
    (size.x > 0 && size.y > 0).then(|| Viewport {
        physical_position: min.as_uvec2(),
        physical_size: size,
        ..default()
    })
}

/// Records the rest of `ui` as a pane body and takes it, so resizable panels keep their size.
fn take_body(ui: &mut egui::Ui) -> egui::Rect {
    let rect = ui.available_rect_before_wrap();
    ui.take_available_space();
    rect
}

/// Lays out the panes in what `root` has left under the menu and status bars.
pub fn layout(
    root: &mut egui::Ui,
    dock: &mut Dock,
    tabs: &[&'static str],
    close: &mut MessageWriter<ClosePlotter>,
) {
    let width = root.available_width();
    egui::Panel::right("plots")
        .default_size(width * PLOTS_SHARE)
        .size_range(MIN_COLUMN..=(width - MIN_COLUMN).max(MIN_COLUMN))
        .show(root, |ui| {
            ui.strong("Plots");
            dock.active_tab = active_tab(tabs, dock.active_tab);
            ui.horizontal_wrapped(|ui| {
                for &tab in tabs {
                    ui.selectable_value(&mut dock.active_tab, Some(tab), tab);
                    if tab != CAMERAS_TAB
                        && ui
                            .small_button("×")
                            .on_hover_text("Close plotter")
                            .clicked()
                    {
                        close.write(ClosePlotter(tab));
                    }
                }
            });
            ui.separator();
            if tabs.is_empty() {
                ui.weak("No plots open; the Plotters menu reopens them.");
            }
            dock.plots = Some(take_body(ui));
        });
    let height = root.available_height();
    egui::Panel::bottom("robot_dock")
        .resizable(true)
        .default_size(height * (1.0 - MAP_SHARE))
        .show(root, |ui| {
            egui::Panel::right("teleop")
                .default_size(TELEOP_WIDTH)
                .show(ui, |ui| {
                    ui.strong("Teleop");
                    ui.separator();
                    dock.teleop = Some(take_body(ui));
                });
            ui.strong("Robot Info");
            ui.separator();
            dock.robot_info = Some(take_body(ui));
        });
    egui::Panel::top("map_title").show(root, |ui| ui.strong("Map"));
    // Not taken: nothing in egui covers the map, so the pointer reaches it.
    dock.map = Some(root.available_rect_before_wrap());
}

/// Draws a pane's contents into its rect, scrolling when they do not fit.
pub fn show_pane(
    ctx: &egui::Context,
    id: &str,
    rect: Option<egui::Rect>,
    add: impl FnOnce(&mut egui::Ui),
) {
    let Some(rect) = rect else { return };
    let mut ui = egui::Ui::new(
        ctx.clone(),
        egui::Id::new(id),
        egui::UiBuilder::new().max_rect(rect),
    );
    egui::ScrollArea::both()
        .id_salt(id)
        .auto_shrink(false)
        .show(&mut ui, add);
}

/// The 2D and 3D views; not the egui host or the sensor cameras.
type MapCameras = Or<(With<MainCamera>, With<OrbitCamera>)>;

/// Points the 2D and 3D map cameras at the Map pane.
pub fn apply_map_viewport(
    dock: Res<Dock>,
    window: Single<&Window, With<PrimaryWindow>>,
    mut cams: Query<&mut Camera, MapCameras>,
) {
    let Some(rect) = dock.map else { return };
    let vp = map_viewport(rect, window.scale_factor(), window.physical_size());
    // `Viewport` has no `PartialEq`; only touch the camera when the pane moved.
    let key = |v: &Option<Viewport>| v.as_ref().map(|v| (v.physical_position, v.physical_size));
    for mut cam in &mut cams {
        if key(&cam.viewport) != key(&vp) {
            cam.viewport = vp.clone();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const NAMES: [&str; 3] = ["Map Trace", "Pose Error", "Scan Trace"];

    #[test]
    fn tabs_follow_open_plotters_in_build_order_then_cameras() {
        let open = |n: &str| n != "Pose Error";
        assert_eq!(plot_tabs(&NAMES, open, false), ["Map Trace", "Scan Trace"]);
        assert_eq!(
            plot_tabs(&NAMES, open, true),
            ["Map Trace", "Scan Trace", CAMERAS_TAB]
        );
        assert!(plot_tabs(&NAMES, |_| false, false).is_empty());
    }

    #[test]
    fn active_tab_survives_until_its_tab_closes() {
        assert_eq!(active_tab(&NAMES, None), Some("Map Trace"));
        assert_eq!(active_tab(&NAMES, Some("Scan Trace")), Some("Scan Trace"));
        assert_eq!(
            active_tab(&["Map Trace"], Some("Scan Trace")),
            Some("Map Trace")
        );
        assert_eq!(active_tab(&[], Some("Scan Trace")), None);
    }

    #[test]
    fn map_viewport_scales_and_clamps_to_the_window() {
        let rect = egui::Rect::from_min_max(egui::pos2(0.0, 50.0), egui::pos2(400.0, 300.0));
        let vp = map_viewport(rect, 2.0, UVec2::new(700, 1000)).unwrap();
        assert_eq!(vp.physical_position, UVec2::new(0, 100));
        assert_eq!(vp.physical_size, UVec2::new(700, 500));
        let empty = egui::Rect::from_min_max(egui::pos2(10.0, 10.0), egui::pos2(10.0, 90.0));
        assert!(map_viewport(empty, 1.0, UVec2::new(100, 100)).is_none());
    }
}
