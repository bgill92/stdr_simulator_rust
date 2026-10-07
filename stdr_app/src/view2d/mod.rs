//! The 2D map view: map sprite, robot/sensor overlay, camera, and mouse picking. Its input and
//! overlay systems pause while the 3D view is shown.

mod camera;
mod map_texture;
mod picking;
mod robots;

use bevy::prelude::*;
use bevy_egui::{EguiGlobalSettings, EguiPrimaryContextPass};

pub use camera::{MainCamera, cursor_in_viewport, scroll_notches};
pub use map_texture::{MapSprite, MapTexture, map_rect, sync_map_texture};
pub use robots::{Trails, sample_trails};

use crate::scene3d::{ViewMode, in_2d};
use crate::sim::apply_sim_commands;

pub struct View2dPlugin;

impl Plugin for View2dPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<MapTexture>()
            .init_resource::<Trails>()
            .init_resource::<camera::ViewLock>()
            .init_resource::<picking::ContextMenu>()
            .init_resource::<ViewMode>()
            // The 2D camera hosts egui explicitly; auto-creation could pick a 3D camera instead.
            .insert_resource(EguiGlobalSettings {
                auto_create_primary_context: false,
                ..default()
            })
            .add_systems(Startup, camera::spawn_camera)
            .add_systems(
                PreUpdate,
                (sync_map_texture, sample_trails).after(apply_sim_commands),
            )
            .add_systems(
                Update,
                (
                    camera::fit_to_map.run_if(camera::fit_needed),
                    (
                        camera::camera_pan_zoom,
                        picking::pick_robot,
                        robots::draw_overlay,
                    )
                        .chain()
                        .run_if(in_2d),
                )
                    .chain(),
            )
            .add_systems(EguiPrimaryContextPass, picking::context_menu);
    }
}
