use std::process::ExitCode;

use bevy::prelude::*;
use bevy_egui::EguiPlugin;
use clap::Parser;
use stdr_app::cli::Cli;
use stdr_app::sim::{SimEvent, SimPlugin, SimWorld, load_robot};
use stdr_app::ui::UiPlugin;
use stdr_app::view2d::View2dPlugin;
use stdr_core::load_map;

fn main() -> ExitCode {
    let cli = Cli::parse();

    // Startup files load before the window opens and abort on error (C++ parity); the events
    // they would have produced through `SimCommand` are queued for the first frame.
    let mut sim = SimWorld::default();
    let mut events = Vec::new();
    if let Some(path) = &cli.map {
        match load_map(path) {
            Ok(grid) => {
                sim.set_map(grid);
                events.push(SimEvent::MapLoaded(path.clone()));
            }
            Err(e) => {
                eprintln!("Failed to load map: {e}");
                return ExitCode::FAILURE;
            }
        }
    }
    if let Some(path) = &cli.robot {
        match load_robot(path) {
            Ok((cfg, warnings)) => {
                events.extend(warnings.into_iter().map(SimEvent::Log));
                let pose = cli.spawn_pose(cfg.initial_pose);
                events.push(SimEvent::RobotSpawned(sim.spawn(cfg, pose)));
            }
            Err(e) => {
                eprintln!("Failed to spawn robot: {e}");
                return ExitCode::FAILURE;
            }
        }
    }

    let mut app = App::new();
    app.add_plugins((
        DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "STDR Simulator".into(),
                ..default()
            }),
            ..default()
        }),
        EguiPlugin::default(),
        SimPlugin,
        View2dPlugin,
        UiPlugin,
    ))
    .insert_resource(sim);
    for e in events {
        app.world_mut().write_message(e);
    }
    match app.run() {
        AppExit::Success => ExitCode::SUCCESS,
        AppExit::Error(_) => ExitCode::FAILURE,
    }
}
