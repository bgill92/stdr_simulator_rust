use bevy::camera::visibility::RenderLayers;
use bevy::input::mouse::{AccumulatedMouseScroll, MouseScrollUnit};
use bevy::prelude::*;
use bevy::window::PrimaryWindow;
use bevy_egui::PrimaryEguiContext;
use bevy_egui::input::EguiWantsInput;

use super::map_texture::{MapTexture, map_rect};
use crate::sim::SimWorld;

#[derive(Component)]
pub struct MainCamera;

/// Locked: the map is kept fitted to the Map pane and pan/zoom are off.
#[derive(Resource, Default)]
pub struct ViewLock(pub bool);

/// Zoom factor per mouse-wheel notch.
const ZOOM_STEP: f32 = 1.1;
/// Pixel-unit scroll (touchpads) per notch-equivalent.
const PIXELS_PER_NOTCH: f32 = 40.0;
/// Margin around the map after a fit.
const FIT_MARGIN: f32 = 1.05;
/// The overlay gizmos' own layer: only the 2D camera sees it, so 3D cameras never draw them.
const OVERLAY_LAYER: usize = 1;

/// The map camera, whose viewport is the Map pane, and a full-window camera that only hosts egui:
/// it renders last, clearing nothing and seeing no layer, so the panes draw over either map view.
pub fn spawn_camera(mut commands: Commands, mut gizmos: ResMut<GizmoConfigStore>) {
    gizmos
        .config_mut::<DefaultGizmoConfigGroup>()
        .0
        .render_layers = RenderLayers::layer(OVERLAY_LAYER);
    commands.spawn((
        Camera2d,
        MainCamera,
        RenderLayers::from_layers(&[0, OVERLAY_LAYER]),
    ));
    commands.spawn((
        Camera2d,
        Camera {
            order: 1,
            clear_color: ClearColorConfig::None,
            ..default()
        },
        PrimaryEguiContext,
        RenderLayers::none(),
    ));
}

/// World units per logical pixel.
pub fn world_per_pixel(proj: &Projection) -> f32 {
    match proj {
        Projection::Orthographic(o) => o.scale,
        _ => 1.0,
    }
}

/// The cursor in window coordinates, if it is over the camera's viewport (the Map pane).
pub fn cursor_in_viewport(window: &Window, camera: &Camera) -> Option<Vec2> {
    let p = window.cursor_position()?;
    camera.logical_viewport_rect()?.contains(p).then_some(p)
}

/// The cursor in world coordinates, if it is over the camera's viewport.
pub fn cursor_world(window: &Window, camera: &Camera, at: &GlobalTransform) -> Option<Vec2> {
    camera
        .viewport_to_world_2d(at, cursor_in_viewport(window, camera)?)
        .ok()
}

/// This frame's wheel movement in notches, whatever unit the device reports.
pub fn scroll_notches(scroll: &AccumulatedMouseScroll) -> f32 {
    match scroll.unit {
        MouseScrollUnit::Line => scroll.delta.y,
        MouseScrollUnit::Pixel => scroll.delta.y / PIXELS_PER_NOTCH,
    }
}

/// Centre the map and fit it to the Map pane.
pub fn fit_to_map(
    sim: Res<SimWorld>,
    cam: Single<(&Camera, &mut Transform, &mut Projection), With<MainCamera>>,
) {
    let Some(grid) = sim.map() else { return };
    let rect = map_rect(grid);
    let (camera, mut t, mut proj) = cam.into_inner();
    if let Projection::Orthographic(o) = &mut *proj {
        let pane = camera
            .logical_viewport_size()
            .unwrap_or(Vec2::ONE)
            .max(Vec2::ONE);
        o.scale = (rect.size() / pane).max_element() * FIT_MARGIN;
    }
    t.translation = rect.center().extend(t.translation.z);
}

/// On every new map texture, every Map pane resize, and every frame while locked.
pub fn fit_needed(
    tex: Res<MapTexture>,
    lock: Res<ViewLock>,
    cam: Single<&Camera, With<MainCamera>>,
    mut last_size: Local<Option<Vec2>>,
) -> bool {
    let size = cam.logical_viewport_size();
    let resized = size != *last_size;
    *last_size = size;
    tex.is_changed() || lock.0 || resized
}

/// Middle-drag keeps the world point grabbed under the cursor; the wheel zooms about the cursor.
pub fn camera_pan_zoom(
    lock: Res<ViewLock>,
    egui_input: Res<EguiWantsInput>,
    buttons: Res<ButtonInput<MouseButton>>,
    scroll: Res<AccumulatedMouseScroll>,
    window: Single<&Window, With<PrimaryWindow>>,
    cam: Single<(&Camera, &GlobalTransform, &mut Transform, &mut Projection), With<MainCamera>>,
    mut grab: Local<Option<Vec2>>,
) {
    if lock.0 || egui_input.wants_pointer_input() {
        *grab = None;
        return;
    }
    let (camera, at, mut t, mut proj) = cam.into_inner();
    let Projection::Orthographic(o) = &mut *proj else {
        return;
    };
    let mut cursor = cursor_world(&window, camera, at);
    if cursor.is_none() {
        // Over another pane or outside the window: the map takes no input.
        *grab = None;
        return;
    }
    if !buttons.pressed(MouseButton::Middle) {
        *grab = None;
    } else if let Some(p) = cursor {
        let g = *grab.get_or_insert(p);
        t.translation += (g - p).extend(0.0);
        cursor = Some(g);
    }
    let notches = scroll_notches(&scroll);
    if notches != 0.0 {
        let factor = ZOOM_STEP.powf(-notches);
        o.scale *= factor;
        // Keep the world point under the cursor fixed: c' = p - (p - c) * factor.
        if let Some(p) = cursor {
            let c = t.translation.truncate();
            t.translation = (p - (p - c) * factor).extend(t.translation.z);
        }
    }
}
