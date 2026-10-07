//! The overlay inside `egui_plot`: plot coordinates are world metres, so robots, trails and
//! scans go through the same `overlay::draw_*` routines as the 2D view.

use bevy_egui::egui;
use egui_plot::{Line, Plot, PlotImage, PlotPoint, PlotUi, Points};

use crate::overlay::{Canvas, Style};
use crate::view2d::MapTexture;

pub const TRUTH: Style = Style {
    color: [0, 204, 0, 255],
    width: 2.0,
};
pub const ODOM: Style = Style {
    color: [255, 140, 0, 255],
    width: 2.0,
};
pub const SCAN: Style = Style {
    color: [51, 204, 255, 255],
    width: 1.0,
};

/// `Canvas` over an `egui_plot` plot. Items are unnamed, so they stay out of the legend.
pub struct PlotCanvas<'a, 'p>(pub &'a mut PlotUi<'p>);

pub fn color32(s: Style) -> egui::Color32 {
    let [r, g, b, a] = s.color;
    egui::Color32::from_rgba_unmultiplied(r, g, b, a)
}

impl Canvas for PlotCanvas<'_, '_> {
    fn polyline(&mut self, pts: &[[f64; 2]], style: Style, closed: bool) {
        let close = pts.first().filter(|_| closed);
        let pts: Vec<[f64; 2]> = pts.iter().chain(close).copied().collect();
        self.0.line(
            Line::new("", pts)
                .color(color32(style))
                .width(style.width)
                .allow_hover(false),
        );
    }

    fn points(&mut self, pts: &[[f64; 2]], style: Style, radius: f32) {
        self.0.points(
            Points::new("", pts.to_vec())
                .color(color32(style))
                .radius(radius)
                .filled(true)
                .allow_hover(false),
        );
    }
}

/// A square, equal-aspect map plot. Locked: fitted to its contents with pan/zoom off (clicks
/// still reach the response).
pub fn map_plot(id: &str, lock_view: bool) -> Plot<'static> {
    let plot = Plot::new(id)
        .data_aspect(1.0)
        .view_aspect(1.0)
        .x_axis_label("x (m)")
        .y_axis_label("y (m)");
    if lock_view {
        plot.allow_drag(false)
            .allow_zoom(false)
            .allow_scroll(false)
            .allow_boxed_zoom(false)
            .allow_double_click_reset(false)
            .auto_bounds(true)
    } else {
        plot
    }
}

/// The map image as the plot background; nothing until a map is loaded and registered with egui.
pub fn draw_map(plot_ui: &mut PlotUi<'_>, tex: &MapTexture) {
    let Some(id) = tex.egui_id else { return };
    let c = tex.rect.center();
    let size = tex.rect.size();
    plot_ui.image(
        PlotImage::new(
            "",
            id,
            PlotPoint::new(f64::from(c.x), f64::from(c.y)),
            egui::vec2(size.x, size.y),
        )
        .allow_hover(false),
    );
}
