//! The 2D map view: map sprite, robot/sensor overlay, camera, and mouse picking.

mod camera;
mod map_texture;
mod picking;
mod robots;

use bevy::prelude::*;
use bevy_egui::EguiPrimaryContextPass;

pub use map_texture::{MapTexture, sync_map_texture};
pub use robots::{Trails, sample_trails};

use crate::sim::apply_sim_commands;

pub struct View2dPlugin;

impl Plugin for View2dPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<MapTexture>()
            .init_resource::<Trails>()
            .init_resource::<camera::ViewLock>()
            .init_resource::<picking::ContextMenu>()
            .add_systems(Startup, camera::spawn_camera)
            .add_systems(
                PreUpdate,
                (sync_map_texture, sample_trails).after(apply_sim_commands),
            )
            .add_systems(
                Update,
                (
                    camera::fit_to_map.run_if(camera::fit_needed),
                    camera::camera_pan_zoom,
                    picking::pick_robot,
                    robots::draw_overlay,
                )
                    .chain(),
            )
            .add_systems(EguiPrimaryContextPass, picking::context_menu);
    }
}
