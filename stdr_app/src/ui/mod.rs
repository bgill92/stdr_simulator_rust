//! egui panels: toolbar + status bar, robot info, messages, teleop.

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
            .add_plugins(teleop::TeleopPlugin)
            .add_systems(Update, messages::log_sim_events)
            .add_systems(
                EguiPrimaryContextPass,
                (
                    toolbar::toolbar,
                    robot_info::robot_info,
                    messages::messages_window,
                ),
            );
    }
}
