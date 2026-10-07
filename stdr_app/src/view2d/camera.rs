use bevy::input::mouse::{AccumulatedMouseScroll, MouseScrollUnit};
use bevy::prelude::*;
use bevy::window::PrimaryWindow;
use bevy_egui::input::EguiWantsInput;

use super::map_texture::{MapTexture, map_rect};
use crate::sim::SimWorld;

#[derive(Component)]
pub struct MainCamera;

/// Locked: the map is kept fitted to the window and pan/zoom are off.
#[derive(Resource, Default)]
pub struct ViewLock(pub bool);

/// Zoom factor per mouse-wheel notch.
const ZOOM_STEP: f32 = 1.1;
/// Pixel-unit scroll (touchpads) per notch-equivalent.
const PIXELS_PER_NOTCH: f32 = 40.0;
/// Margin around the map after a fit.
const FIT_MARGIN: f32 = 1.05;

pub fn spawn_camera(mut commands: Commands) {
    commands.spawn((Camera2d, MainCamera));
}

/// World units per logical pixel.
pub fn world_per_pixel(proj: &Projection) -> f32 {
    match proj {
        Projection::Orthographic(o) => o.scale,
        _ => 1.0,
    }
}

/// The cursor in world coordinates, if it is inside the window.
pub fn cursor_world(window: &Window, camera: &Camera, at: &GlobalTransform) -> Option<Vec2> {
    camera
        .viewport_to_world_2d(at, window.cursor_position()?)
        .ok()
}

/// On every new map texture (and every frame while locked): centre the map and fit it to the window.
pub fn fit_to_map(
    sim: Res<SimWorld>,
    window: Single<&Window, With<PrimaryWindow>>,
    cam: Single<(&mut Transform, &mut Projection), With<MainCamera>>,
) {
    let Some(grid) = sim.map() else { return };
    let rect = map_rect(grid);
    let (mut t, mut proj) = cam.into_inner();
    if let Projection::Orthographic(o) = &mut *proj {
        let win = window.size().max(Vec2::ONE);
        o.scale = (rect.size() / win).max_element() * FIT_MARGIN;
    }
    t.translation = rect.center().extend(t.translation.z);
}

pub fn fit_needed(tex: Res<MapTexture>, lock: Res<ViewLock>) -> bool {
    tex.is_changed() || lock.0
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
    if !buttons.pressed(MouseButton::Middle) {
        *grab = None;
    } else if let Some(p) = cursor {
        let g = *grab.get_or_insert(p);
        t.translation += (g - p).extend(0.0);
        cursor = Some(g);
    }
    let notches = match scroll.unit {
        MouseScrollUnit::Line => scroll.delta.y,
        MouseScrollUnit::Pixel => scroll.delta.y / PIXELS_PER_NOTCH,
    };
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
