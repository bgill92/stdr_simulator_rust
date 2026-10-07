use std::path::{Path, PathBuf};

use bevy::prelude::*;
use stdr_core::{CoreError, Pose2D, RobotConfig, RobotId, Twist2D, load_map, load_robot_config};

use super::{CatchUp, STEP_DT_RANGE, SimWorld};

/// Every change to the simulation goes through here, applied in PreUpdate.
#[derive(Message, Clone, Debug, PartialEq)]
pub enum SimCommand {
    LoadMap(PathBuf),
    SpawnRobot {
        path: PathBuf,
        pose: Pose2D,
    },
    DeleteRobot(RobotId),
    /// Resume.
    Start,
    Pause,
    /// Pause, every robot back to its spawn pose, sim time to zero.
    Reset,
    /// Sim seconds per real second; must be > 0.
    SetSpeed(f64),
    /// Seconds per tick, clamped to `STEP_DT_RANGE`.
    SetStepDt(f64),
    Teleport {
        id: RobotId,
        pose: Pose2D,
    },
    CmdVel {
        id: RobotId,
        twist: Twist2D,
    },
}

#[derive(Message, Clone, Debug, PartialEq)]
pub enum SimEvent {
    MapLoaded(PathBuf),
    RobotSpawned(RobotId),
    RobotDeleted(RobotId),
    Reset,
    Paused,
    Resumed,
    /// A frame wanted more sim time than the catch-up cap (seconds) allowed; the rest was dropped.
    FellBehind(f64),
    /// Load warnings and command errors.
    Log(String),
}

/// Includes resolve against `$STDR_RESOURCES_DIR`, else the robot yaml's directory (C++ parity).
pub fn load_robot(path: &Path) -> Result<(RobotConfig, Vec<String>), CoreError> {
    let base_dir = std::env::var_os("STDR_RESOURCES_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|| path.parent().unwrap_or(Path::new("")).to_path_buf());
    load_robot_config(path, base_dir)
}

pub fn apply_sim_commands(
    mut commands: MessageReader<SimCommand>,
    mut events: MessageWriter<SimEvent>,
    mut sim: ResMut<SimWorld>,
    catch_up: Res<CatchUp>,
    mut virt: ResMut<Time<Virtual>>,
    mut fixed: ResMut<Time<Fixed>>,
) {
    for c in commands.read() {
        match c {
            SimCommand::LoadMap(path) => match load_map(path) {
                Ok(grid) => {
                    sim.set_map(grid);
                    events.write(SimEvent::MapLoaded(path.clone()));
                }
                Err(e) => {
                    events.write(SimEvent::Log(format!("Failed to load map: {e}")));
                }
            },
            SimCommand::SpawnRobot { path, pose } => match load_robot(path) {
                Ok((cfg, warnings)) => {
                    events.write_batch(warnings.into_iter().map(SimEvent::Log));
                    events.write(SimEvent::RobotSpawned(sim.spawn(cfg, *pose)));
                }
                Err(e) => {
                    events.write(SimEvent::Log(format!("Failed to spawn robot: {e}")));
                }
            },
            SimCommand::DeleteRobot(id) => {
                if sim.remove(*id) {
                    events.write(SimEvent::RobotDeleted(*id));
                }
            }
            SimCommand::Start => {
                virt.unpause();
                events.write(SimEvent::Resumed);
            }
            SimCommand::Pause => {
                virt.pause();
                events.write(SimEvent::Paused);
            }
            SimCommand::Reset => {
                virt.pause();
                sim.reset();
                events.write(SimEvent::Reset);
            }
            SimCommand::SetSpeed(speed) => {
                if *speed > 0.0 && speed.is_finite() {
                    virt.set_relative_speed_f64(*speed);
                    catch_up.apply(&mut virt);
                } else {
                    events.write(SimEvent::Log(format!("Invalid speed {speed}")));
                }
            }
            SimCommand::SetStepDt(dt) => {
                if !dt.is_finite() {
                    events.write(SimEvent::Log(format!("Invalid timestep {dt}")));
                    continue;
                }
                let dt = dt.clamp(STEP_DT_RANGE.0, STEP_DT_RANGE.1);
                sim.set_step_dt(dt).expect("clamped step_dt is positive");
                fixed.set_timestep_seconds(dt);
            }
            SimCommand::Teleport { id, pose } => {
                sim.teleport(*id, *pose);
            }
            SimCommand::CmdVel { id, twist } => {
                sim.set_cmd_vel(*id, *twist);
            }
        }
    }
}
