//! The app's sim control without a window: `MinimalPlugins` + `SimPlugin`, real time driven by
//! `TimeUpdateStrategy::ManualDuration`.

use std::path::PathBuf;
use std::time::Duration;

use bevy::prelude::*;
use bevy::time::TimeUpdateStrategy;
use stdr_app::sim::{Selection, SimCommand, SimEvent, SimPlugin, SimWorld};
use stdr_app::ui::teleop::{TeleopPlugin, TeleopState};
use stdr_app::ui::toolbar::status_text;
use stdr_app::view2d::{MapTexture, Trails, sample_trails, sync_map_texture};
use stdr_core::{
    LaserSpec, OccupancyGrid, Pose2D, RobotConfig, RobotId, Sensor, SensorCommon, SensorConfig,
    SimulationEngine, Twist2D,
};

const STEP_DT: f64 = 0.01;
/// One frame of real time: exactly 5 ticks at `STEP_DT`.
const FRAME: Duration = Duration::from_millis(50);
const TICKS_PER_FRAME: u64 = 5;

/// Every `SimEvent`, in order.
#[derive(Resource, Default)]
struct Seen(Vec<SimEvent>);

fn record(mut events: MessageReader<SimEvent>, mut seen: ResMut<Seen>) {
    seen.0.extend(events.read().cloned());
}

/// After the first update: Startup has run (sim paused, fixed timestep = `STEP_DT`).
fn app() -> App {
    let mut app = App::new();
    app.add_plugins((MinimalPlugins, SimPlugin))
        .insert_resource(SimWorld(SimulationEngine::new(STEP_DT, Some(7)).unwrap()))
        .insert_resource(TimeUpdateStrategy::ManualDuration(FRAME))
        .init_resource::<Seen>()
        .add_systems(Last, record);
    app.update();
    app
}

/// Started, past the frame whose virtual delta was computed while still paused.
fn running_app() -> App {
    let mut app = app();
    send(&mut app, SimCommand::Start);
    app.update();
    assert_eq!(
        sim(&app).ticks(),
        0,
        "the resume frame has no virtual time yet"
    );
    app
}

fn send(app: &mut App, c: SimCommand) {
    app.world_mut().write_message(c);
}

fn sim(app: &App) -> &SimulationEngine {
    &app.world().resource::<SimWorld>().0
}

fn sim_mut(app: &mut App) -> Mut<'_, SimWorld> {
    app.world_mut().resource_mut::<SimWorld>()
}

fn keys(app: &mut App) -> Mut<'_, ButtonInput<KeyCode>> {
    app.world_mut().resource_mut::<ButtonInput<KeyCode>>()
}

fn virt(app: &App) -> &Time<Virtual> {
    app.world().resource::<Time<Virtual>>()
}

/// Ticks run by one more update.
fn ticks_of_next_update(app: &mut App) -> u64 {
    let before = sim(app).ticks();
    app.update();
    sim(app).ticks() - before
}

fn seen(app: &App) -> &[SimEvent] {
    &app.world().resource::<Seen>().0
}

fn free_map() -> OccupancyGrid {
    OccupancyGrid::new(100, 100, 0.1, Pose2D::default(), vec![0; 100 * 100]).unwrap()
}

fn laser_robot() -> RobotConfig {
    RobotConfig {
        sensors: vec![Sensor {
            common: SensorCommon::default(),
            kind: SensorConfig::Laser(LaserSpec {
                min_angle: -1.0,
                max_angle: 1.0,
                min_range: 0.1,
                max_range: 5.0,
                num_rays: 10,
            }),
        }],
        ..Default::default()
    }
}

fn fixture(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../stdr_core/tests/fixtures")
        .join(name)
}

#[test]
fn starts_paused() {
    let mut app = app();
    assert!(virt(&app).is_paused());
    for _ in 0..5 {
        app.update();
    }
    assert_eq!(sim(&app).ticks(), 0);
}

#[test]
fn n_updates_advance_sim_time_by_ticks_times_step_dt() {
    let mut app = running_app();
    for n in 1..=10 {
        app.update();
        assert_eq!(sim(&app).ticks(), n * TICKS_PER_FRAME);
    }
    let s = sim(&app);
    assert!((s.sim_time() - s.ticks() as f64 * STEP_DT).abs() < 1e-9);
}

#[test]
fn pause_freezes() {
    let mut app = running_app();
    app.update();
    send(&mut app, SimCommand::Pause);
    // The pause lands in PreUpdate, before this frame's fixed ticks: none of them run.
    assert_eq!(ticks_of_next_update(&mut app), 0);
    let (ticks, t) = (sim(&app).ticks(), sim(&app).sim_time());
    for _ in 0..10 {
        app.update();
    }
    assert_eq!((sim(&app).ticks(), sim(&app).sim_time()), (ticks, t));
}

#[test]
fn resume_does_not_replay() {
    let mut app = running_app();
    app.update();
    send(&mut app, SimCommand::Pause);
    for _ in 0..40 {
        app.update(); // 2 s paused
    }
    send(&mut app, SimCommand::Start);
    assert_eq!(ticks_of_next_update(&mut app), 0);
    for _ in 0..5 {
        assert_eq!(ticks_of_next_update(&mut app), TICKS_PER_FRAME);
    }
}

#[test]
fn double_speed_doubles_ticks_per_frame() {
    let mut app = running_app();
    assert_eq!(ticks_of_next_update(&mut app), TICKS_PER_FRAME);
    send(&mut app, SimCommand::SetSpeed(2.0));
    app.update(); // virtual time for this frame was already taken at 1x
    for _ in 0..3 {
        assert_eq!(ticks_of_next_update(&mut app), 2 * TICKS_PER_FRAME);
    }
}

#[test]
fn set_step_dt_changes_timestep() {
    let mut app = running_app();
    app.update();
    let t0 = sim(&app).sim_time();
    send(&mut app, SimCommand::SetStepDt(0.05));
    // Applied before this frame's fixed ticks: one 50 ms tick per 50 ms frame from now on.
    for _ in 0..3 {
        assert_eq!(ticks_of_next_update(&mut app), 1);
    }
    assert_eq!(sim(&app).step_dt(), 0.05);
    assert_eq!(
        app.world().resource::<Time<Fixed>>().timestep(),
        Duration::from_millis(50)
    );
    assert!((sim(&app).sim_time() - (t0 + 0.15)).abs() < 1e-9);

    send(&mut app, SimCommand::SetStepDt(10.0));
    app.update();
    assert_eq!(sim(&app).step_dt(), 1.0, "clamped to the upper bound");
}

#[test]
fn two_second_manual_delta_is_capped_to_a_quarter_second_of_sim() {
    let mut app = running_app();
    app.insert_resource(TimeUpdateStrategy::ManualDuration(Duration::from_secs(2)));
    let t0 = sim(&app).sim_time();
    assert_eq!(ticks_of_next_update(&mut app), 25);
    assert!((sim(&app).sim_time() - t0 - 0.25).abs() < 1e-9);

    // The cap is in sim seconds, whatever the speed.
    send(&mut app, SimCommand::SetSpeed(5.0));
    app.update();
    assert_eq!(ticks_of_next_update(&mut app), 25);
}

#[test]
fn fell_behind_fires_once() {
    let mut app = running_app();
    app.insert_resource(TimeUpdateStrategy::ManualDuration(Duration::from_secs(2)));
    for _ in 0..5 {
        app.update();
    }
    let fell_behind = |app: &App| {
        seen(app)
            .iter()
            .filter(|e| matches!(e, SimEvent::FellBehind(_)))
            .count()
    };
    assert_eq!(fell_behind(&app), 1);

    // Edge-triggered: catching up re-arms it.
    app.insert_resource(TimeUpdateStrategy::ManualDuration(FRAME));
    app.update();
    app.insert_resource(TimeUpdateStrategy::ManualDuration(Duration::from_secs(2)));
    app.update();
    app.update();
    assert_eq!(fell_behind(&app), 2);
}

#[test]
fn reset_restores_spawn_state() {
    let mut app = app();
    let spawn = Pose2D {
        x: 5.0,
        y: 5.0,
        theta: 0.3,
    };
    let id = {
        let mut s = sim_mut(&mut app);
        s.set_map(free_map());
        let id = s.spawn(laser_robot(), spawn);
        s.set_cmd_vel(
            id,
            Twist2D {
                linear_x: 0.5,
                ..Default::default()
            },
        );
        id
    };
    send(&mut app, SimCommand::Start);
    for _ in 0..5 {
        app.update();
    }
    let r = sim(&app).robot(id).unwrap();
    assert_ne!(r.state.pose, spawn);
    assert!(r.data[0].is_some());

    send(&mut app, SimCommand::Reset);
    app.update();
    let check = |app: &App| {
        let s = sim(app);
        let r = s.robot(id).unwrap();
        assert_eq!(r.state.pose, spawn);
        assert_eq!(r.state.odom_pose, spawn);
        assert_eq!(r.state.cmd_vel, Twist2D::default());
        assert!(r.data.iter().all(Option::is_none));
        assert_eq!((s.sim_time(), s.ticks()), (0.0, 0));
    };
    check(&app);
    assert!(virt(&app).is_paused());
    assert!(seen(&app).contains(&SimEvent::Reset));
    for _ in 0..3 {
        app.update();
    }
    check(&app);
}

#[test]
fn driving_grows_trail_and_reset_or_teleport_clears_it() {
    let mut app = app();
    app.init_resource::<Trails>().add_systems(
        PreUpdate,
        sample_trails.after(stdr_app::sim::apply_sim_commands),
    );
    let spawn = Pose2D::default();
    let id = {
        let mut s = sim_mut(&mut app);
        let id = s.spawn(RobotConfig::default(), spawn);
        s.set_cmd_vel(
            id,
            Twist2D {
                linear_x: 1.0,
                ..Default::default()
            },
        );
        id
    };
    let trail = |app: &App| app.world().resource::<Trails>().0[&id].xy().to_vec();
    send(&mut app, SimCommand::Start);
    for _ in 0..5 {
        app.update();
    }
    assert!(trail(&app).len() >= 3);

    let far = Pose2D {
        x: 5.0,
        y: 5.0,
        theta: 0.0,
    };
    send(&mut app, SimCommand::Pause);
    send(&mut app, SimCommand::Teleport { id, pose: far });
    app.update();
    assert_eq!(trail(&app), [[5.0, 5.0]]);

    send(&mut app, SimCommand::Reset);
    app.update();
    assert_eq!(trail(&app), [[spawn.x, spawn.y]]);
}

#[test]
fn pause_resume_via_command() {
    let mut app = running_app();
    assert!(!virt(&app).is_paused());
    send(&mut app, SimCommand::Pause);
    app.update();
    assert!(virt(&app).is_paused());
    send(&mut app, SimCommand::Start);
    app.update();
    assert!(!virt(&app).is_paused());
    assert_eq!(ticks_of_next_update(&mut app), TICKS_PER_FRAME);
    assert_eq!(
        seen(&app),
        [SimEvent::Resumed, SimEvent::Paused, SimEvent::Resumed]
    );
}

#[test]
fn teleop_stop_targets_previously_driven_robot() {
    let mut app = app();
    app.add_plugins(TeleopPlugin)
        .init_resource::<ButtonInput<KeyCode>>();
    let (a, b) = {
        let mut s = sim_mut(&mut app);
        (
            s.spawn(RobotConfig::default(), Pose2D::default()),
            s.spawn(RobotConfig::default(), Pose2D::default()),
        )
    };
    let cmd_vel = |app: &App, id: RobotId| sim(app).robot(id).unwrap().state.cmd_vel;
    let select = |app: &mut App, id| app.world_mut().resource_mut::<Selection>().robot = Some(id);

    select(&mut app, a);
    keys(&mut app).press(KeyCode::KeyW);
    app.update(); // teleop writes CmdVel(a) in Update ...
    app.update(); // ... applied next PreUpdate
    assert_eq!(cmd_vel(&app, a).linear_x, 0.5);
    assert_eq!(app.world().resource::<TeleopState>().driving, Some(a));

    // Selection moves to b in the same frame W is released: the stop must reach a (C++ sent
    // it to the newly selected robot and a kept driving).
    select(&mut app, b);
    keys(&mut app).release(KeyCode::KeyW);
    app.update();
    app.update();
    assert_eq!(cmd_vel(&app, a), Twist2D::default());
    assert_eq!(cmd_vel(&app, b), Twist2D::default());
    assert_eq!(app.world().resource::<TeleopState>().driving, None);
}

#[test]
fn teleop_sends_exactly_one_stop_on_release() {
    #[derive(Resource, Default)]
    struct CmdVels(usize);
    let mut app = app();
    app.add_plugins(TeleopPlugin)
        .init_resource::<ButtonInput<KeyCode>>()
        .init_resource::<CmdVels>()
        .add_systems(
            Last,
            |mut c: MessageReader<SimCommand>, mut n: ResMut<CmdVels>| {
                n.0 += c
                    .read()
                    .filter(|c| matches!(c, SimCommand::CmdVel { .. }))
                    .count();
            },
        );
    let id = sim_mut(&mut app).spawn(RobotConfig::default(), Pose2D::default());
    app.world_mut().resource_mut::<Selection>().robot = Some(id);
    keys(&mut app).press(KeyCode::KeyQ);
    app.update();
    app.update();
    keys(&mut app).release(KeyCode::KeyQ);
    for _ in 0..5 {
        app.update();
    }
    // Two frames driving, one stop, then silence.
    assert_eq!(app.world().resource::<CmdVels>().0, 3);
}

#[test]
fn map_texture_rebuilds_only_on_revision_change() {
    let mut app = app();
    app.add_plugins(AssetPlugin::default())
        .init_asset::<Image>()
        .init_resource::<MapTexture>()
        .add_systems(
            PreUpdate,
            sync_map_texture.after(stdr_app::sim::apply_sim_commands),
        );
    let texture = |app: &App| {
        let t = app.world().resource::<MapTexture>();
        (t.image.id(), t.revision)
    };
    app.update();
    assert_eq!(texture(&app).1, 0, "no map, no texture");

    // 2x1 grid: free on the left, occupied on the right.
    let grid = OccupancyGrid::new(2, 1, 0.5, Pose2D::default(), vec![0, 100]).unwrap();
    sim_mut(&mut app).set_map(grid.clone());
    app.update();
    let first = texture(&app);
    assert_eq!(first.1, 1);
    let image = app
        .world()
        .resource::<Assets<Image>>()
        .get(first.0)
        .unwrap();
    assert_eq!(image.size(), UVec2::new(2, 1));
    assert_eq!(
        image.data.as_deref(),
        Some(&[255, 255, 255, 255, 0, 0, 0, 255][..])
    );

    for _ in 0..5 {
        app.update();
    }
    assert_eq!(texture(&app), first);
    assert_eq!(app.world().resource::<Assets<Image>>().len(), 1);

    sim_mut(&mut app).set_map(grid);
    app.update();
    let second = texture(&app);
    assert_eq!(second.1, 2);
    assert_ne!(second.0, first.0);
}

#[test]
fn selection_falls_back_to_first_robot() {
    let mut app = app();
    let selected = |app: &App| app.world().resource::<Selection>().robot;
    let spawn = |app: &mut App| {
        send(
            app,
            SimCommand::SpawnRobot {
                path: fixture("robot_inline_only.yaml"),
                pose: Pose2D::default(),
            },
        )
    };
    spawn(&mut app);
    spawn(&mut app);
    app.update();
    let ids: Vec<RobotId> = sim(&app).robots().map(|(id, _)| id).collect();
    assert_eq!(ids.len(), 2);
    assert_eq!(selected(&app), Some(ids[0]));

    // Deleting another robot keeps the selection; deleting the selected one clears it.
    send(&mut app, SimCommand::DeleteRobot(ids[1]));
    app.update();
    assert_eq!(selected(&app), Some(ids[0]));
    send(&mut app, SimCommand::DeleteRobot(ids[0]));
    app.update();
    assert_eq!(selected(&app), None);

    spawn(&mut app);
    app.update();
    assert_eq!(selected(&app), sim(&app).robots().next().map(|(id, _)| id));
}

#[test]
fn toolbar_reads_speed_from_virtual_time() {
    let mut app = app();
    assert!(status_text(virt(&app), sim(&app)).starts_with("Paused  |  Speed: 1.0x"));
    send(&mut app, SimCommand::SetSpeed(2.0));
    send(&mut app, SimCommand::Start);
    app.update();
    assert!(status_text(virt(&app), sim(&app)).starts_with("Running  |  Speed: 2.0x"));

    // No shadow copy: whatever `Time<Virtual>` says is what the toolbar shows.
    app.world_mut()
        .resource_mut::<Time<Virtual>>()
        .set_relative_speed(5.0);
    assert!(status_text(virt(&app), sim(&app)).contains("Speed: 5.0x"));
}

#[test]
fn spawn_from_bad_path_logs_instead_of_failing() {
    let mut app = app();
    send(
        &mut app,
        SimCommand::SpawnRobot {
            path: fixture("does_not_exist.yaml"),
            pose: Pose2D::default(),
        },
    );
    app.update();
    assert_eq!(sim(&app).robots().count(), 0);
    assert!(matches!(seen(&app), [SimEvent::Log(m)] if m.starts_with("Failed to spawn robot")));
}
