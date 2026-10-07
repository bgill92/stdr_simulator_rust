//! egui panes: toolbar + status bar, the dock layout, robot info, messages, teleop.

pub mod dock;
pub mod messages;
pub mod robot_info;
pub mod teleop;
pub mod toolbar;

use bevy::prelude::*;
use bevy_egui::EguiPrimaryContextPass;

pub struct UiPlugin;

impl Plugin for UiPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<messages::MessageLog>()
            .init_resource::<toolbar::SpawnDialog>()
            .init_resource::<dock::Dock>()
            .add_plugins(teleop::TeleopPlugin)
            .add_systems(Update, messages::log_sim_events)
            .add_systems(
                EguiPrimaryContextPass,
                (
                    toolbar::toolbar,
                    (robot_info::robot_info, dock::apply_map_viewport).after(toolbar::toolbar),
                ),
            );
    }
}
