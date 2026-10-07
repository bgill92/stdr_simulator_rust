//! The simulation inside Bevy. `SimWorld` (the engine) is the source of truth for robots, map,
//! sim time and step size; pause and speed live in `Time<Virtual>`. State is mutated only by
//! `apply_sim_commands` (PreUpdate) and `sim_step` (FixedUpdate, one `engine.step()` per tick).

mod commands;

use std::collections::HashSet;
use std::time::Duration;

use bevy::prelude::*;
use stdr_core::{RobotId, SimulationEngine};

pub use commands::{SimCommand, SimEvent, apply_sim_commands, load_robot};

/// Startup step size (C++ standalone default).
pub const DEFAULT_STEP_DT: f64 = 0.01;
/// `SetStepDt` clamp range.
pub const STEP_DT_RANGE: (f64, f64) = (0.001, 1.0);

#[derive(Resource, Deref, DerefMut)]
pub struct SimWorld(pub SimulationEngine);

impl Default for SimWorld {
    fn default() -> Self {
        Self(SimulationEngine::new(DEFAULT_STEP_DT, None).expect("default step_dt is positive"))
    }
}

/// Bounds how much sim time one frame may run after a stall; the backlog beyond it is dropped.
#[derive(Resource)]
pub struct CatchUp {
    /// Sim seconds per frame.
    pub cap: f64,
    pub fell_behind_warned: bool,
}

impl Default for CatchUp {
    fn default() -> Self {
        Self {
            cap: 0.25,
            fell_behind_warned: false,
        }
    }
}

impl CatchUp {
    /// Bevy clamps the *real* delta before applying speed, so the clamp is `cap / speed`.
    fn apply(&self, virt: &mut Time<Virtual>) {
        virt.set_max_delta(Duration::from_secs_f64(
            self.cap / virt.relative_speed_f64(),
        ));
    }
}

/// The one selected robot (map pick, robot info, teleop). Takes the first robot spawned while
/// nothing is selected; cleared when the selected robot is deleted.
#[derive(Resource, Default)]
pub struct Selection {
    pub robot: Option<RobotId>,
}

/// `(robot, sensor index)` pairs whose latest readings are drawn on the map. Empty by default.
#[derive(Resource, Default)]
pub struct SensorVisibility(pub HashSet<(RobotId, usize)>);

pub struct SimPlugin;

impl Plugin for SimPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<SimWorld>()
            .init_resource::<CatchUp>()
            .init_resource::<Selection>()
            .init_resource::<SensorVisibility>()
            .add_message::<SimCommand>()
            .add_message::<SimEvent>()
            .add_systems(Startup, init_time)
            .add_systems(
                PreUpdate,
                (apply_sim_commands, detect_fell_behind, sync_selection).chain(),
            )
            // A pause or reset applied this frame must not let the frame's already-accumulated
            // fixed ticks run.
            .add_systems(
                FixedUpdate,
                sim_step.run_if(|t: Res<Time<Virtual>>| !t.is_paused()),
            );
    }
}

/// The sim starts paused (C++ parity), with the fixed timestep taken from the engine.
fn init_time(
    sim: Res<SimWorld>,
    catch_up: Res<CatchUp>,
    mut virt: ResMut<Time<Virtual>>,
    mut fixed: ResMut<Time<Fixed>>,
) {
    fixed.set_timestep_seconds(sim.step_dt());
    virt.pause();
    catch_up.apply(&mut virt);
}

pub fn sim_step(mut sim: ResMut<SimWorld>) {
    sim.step();
}

/// Edge-triggered: one `FellBehind` per stretch of frames whose real delta exceeds the clamp.
fn detect_fell_behind(
    real: Res<Time<Real>>,
    virt: Res<Time<Virtual>>,
    mut catch_up: ResMut<CatchUp>,
    mut events: MessageWriter<SimEvent>,
) {
    let behind = !virt.is_paused() && real.delta() > virt.max_delta();
    if behind && !catch_up.fell_behind_warned {
        events.write(SimEvent::FellBehind(catch_up.cap));
    }
    catch_up.fell_behind_warned = behind;
}

fn sync_selection(mut events: MessageReader<SimEvent>, mut sel: ResMut<Selection>) {
    for e in events.read() {
        match *e {
            SimEvent::RobotSpawned(id) if sel.robot.is_none() => sel.robot = Some(id),
            SimEvent::RobotDeleted(id) if sel.robot == Some(id) => sel.robot = None,
            _ => {}
        }
    }
}
