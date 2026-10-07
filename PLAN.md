# STDR Simulator — Rust/Bevy Port Plan

## Context

`stdr_standalone` (C++20, ImGui + ImPlot, no ROS) is the in-process 2D robot simulator: occupancy-grid map,
robots with ideal/omni kinematics, laser + sonar, odometry noise, a fixed-timestep sim loop, and a plotter
plugin framework (`Plotter` base class, `REGISTER_PLOTTER`, `--plotter` CLI filter). This plan ports that
standalone feature set to Rust on Bevy, keeps the same YAML inputs so both sims run the same maps/robots,
and adds a 3D scene + camera sensor on top (M4).

This is **not** a 1:1 port. An audit of the C++ found heavy duplication that a literal port would carry over:
six-way per-sensor-kind fan-out in ~8 places, laser/sonar ray march byte-identical, the 2D rigid transform
copied 5+ times, angle wrap in 3 places, per-plotter copies of trail/draw_pose/teleport-detector/error math,
three independent "selected robot" states, a 21-method `SimView` with 8 unused methods, and a sensor ring
path used by no production plotter. It also found real bugs (listed in §Bug fixes vs kept quirks). The Rust
design consolidates behind deep modules: small interfaces, lots of behaviour hidden, the interface is the
test surface. C++ **test names** are kept so parity is auditable; C++ **type/module names** are not.

Reference C++ (keep open while porting), all under `/home/bilal/Projects/robotics/stdr_simulator/`:
- `stdr_simulation/include/stdr_simulation/types.hpp`, `simulation_engine.hpp`, `rate_scheduler.hpp`
- `stdr_simulation/src/{motion,collision,sensors}/*.cpp`, `config_loader.cpp`
- `stdr_standalone/src/standalone_backend.cpp` (sim loop, catch-up cap, reset), `map_loader.cpp`
- `stdr_gui/include/stdr_gui/plot/plotter.hpp`, `simulator_backend.hpp`, `stdr_gui/ARCHITECTURE.md`
- `stdr_gui/src/plotters/{pose_error,map_trace,odometry_trace,scan_trace}_plotter.cpp`
- `stdr_gui/src/teleop_controller.cpp`

## Decisions (made with user)

| Topic | Decision |
|---|---|
| Design principle | Deep modules: small interface, lots of behaviour behind it; the interface is the test surface. No trait with a single impl. |
| Fidelity | Fix C++ logic/config bugs (§Bug fixes vs kept quirks). Keep numeric quirks. Headless diff vs C++ is noise-off only. |
| Physics | No physics crate. Port C++ kinematics + occupancy-grid collision. Bevy ships no physics; avian/rapier would change behaviour. |
| Sensors | One `Vec<Sensor>` per robot; `SensorConfig` / `Measurement` enums. Laser + Sonar in M1. rfid/co2/thermal/sound are future variants; their YAML entries are skipped with a warning so every robot yaml still loads. |
| Engine | `WorldModel` merged into `SimulationEngine`. One `BTreeMap<RobotId, RobotRuntime>` replaces the C++'s four parallel name-keyed maps. One seedable `StdRng`. Engine owns sim time (`elapsed`, `ticks`). No Tf/Odom streams: dead in standalone (engine ignores them, panel never shows them). |
| Overlay | One `draw_robot` / `draw_sensors` / `draw_trail` routine over `trait Canvas`. Adapters: `GizmoCanvas` (view2d), `PlotCanvas` (egui_plot), `RecordingCanvas` (tests). Styling and per-sensor drawing live in one place. |
| UI | `bevy_egui` + `egui_plot`. Bevy renders map/robots/rays; egui for toolbar, robot info, teleop, plotter windows. |
| Plotters | Compiled-in. Each plotter = state `Resource` + sample/render systems, registered via `inventory`. No plotter trait, no `SimView`, no `PlotSink`. `--plotter Name` filter kept. |
| Config | C++ YAML schema unchanged. Loader = `serde_yaml_ng::Value` include-expansion + deep-merge, then serde-derive into `#[serde(default)]` structs. Fixes nested partial overrides and `noise: {filename:}` includes. |
| 3D/camera | Separate milestone (M4, §M4). Core stays render-agnostic: `SensorConfig::Camera` is scheduled by core and rendered by the app. |
| Threading | No sim thread. Single-threaded `FixedUpdate`. One tick is tens of µs; catch-up cap bounds worst case. Deterministic, lock-free, trivially headless-testable. |

## Workspace layout

```
stdr_simulator_rust/
  Cargo.toml                  # [workspace] members = ["stdr_core", "stdr_app"]
  PLAN.md
  stdr_core/                  # lib. NO bevy dep. Pure sim, f64.
    src/error.rs              # CoreError (thiserror)
    src/pose.rs               # Pose2D (Mul = SE(2) compose, inverse, transform_point, wrapped), Point2D, Twist2D,
                              # normalize_angle(), angle_diff()   -- the ONE atan2(sin,cos) site
    src/footprint.rs          # enum Footprint { Circle{radius}, Polygon(Vec<Point2D>) }; contains(), vertices()
    src/grid.rs               # OccupancyGrid::new(..) -> Result (invariants); { world_to_cell, grid_coords, in_bounds, at,
                              # is_blocked(policy), raycast }; enum Unknown { Solid, Transparent }; OCCUPANCY_THRESHOLD = 70
    src/map.rs                # load_map(yaml) -> Result<OccupancyGrid> (png/pgm, vertical flip, thresholds, negate); metadata private
    src/config.rs             # RobotConfig, Sensor{common, kind}, SensorCommon, LaserSpec, SonarSpec, KinematicConfig,
                              # KinematicKind, OdometryModel, Alphas([[f64;3];4]), load_robot_config(path, base_dir)
    src/config/yaml.rs        # (private) resolve_includes, deep_merge, load_with_include<T>
    src/motion.rs             # integrate(kind, pose, vel, dt, pivot), perturb(cmd, &KinematicConfig, dt, rng), odometry_variance
    src/collision.rs          # path_collides(grid, footprint, from, to) -> bool
    src/sensors.rs            # simulate(&Sensor, world_pose, &grid, rng) -> Option<Measurement> (None: camera); private finish() (noise + REP-117)
    src/scheduler.rs          # RateScheduler keyed by sensor index (usize), SchedulingMode, private enum Schedule
    src/engine.rs             # SimulationEngine (owns map, robots, sim time, rng), RobotRuntime, RobotState, RobotId
    tests/                    # golden tests transcribed from C++ gtests (names kept)
    examples/headless.rs      # --seed; step N ticks noise-off, print poses (diff vs C++ run)
  stdr_app/                   # bin. bevy + egui + plotters.
    src/main.rs               # loads --map/--robot (fail fast), adds the plugins; --plotter (repeatable) lands in M3
    src/lib.rs                # the plugins as a library, so tests/headless.rs can drive them
    src/cli.rs                # clap: --map --robot --x --y --theta
    src/sim/{mod,commands}.rs # SimPlugin + resources + sim_step; SimCommand/SimEvent + apply_sim_commands
    src/view2d/{map_texture,robots,camera,picking}.rs   # robots.rs = GizmoCanvas + per-frame draw_robot/draw_sensors/draw_trail calls
    src/ui/{toolbar,robot_info,messages,teleop}.rs      # teleop.rs = teleop_twist(keys, KinematicKind, speeds) pure fn + unit tests
    src/overlay.rs            # trait Canvas, Style, draw_robot, draw_sensors, draw_trail, geometry helpers (no bevy/egui types)
    src/plot/mod.rs           # PlotterCtl<P>, add_plotter, plotter_window, PlotterEntry registry,
                              # Trail<T>, Positioned, TeleportDetector, TimeSeries, SampleGate
    src/plot/overlay_plot.rs  # PlotCanvas (Canvas adapter over egui_plot::PlotUi), map_plot, draw_map
    src/plotters/{pose_error,map_trace,odometry_trace,scan_trace}.rs
    src/scene3d/{mod,mesh,camera_sensor}.rs   # M4: extruded map/robots, orbit camera, camera sensors
    tests/headless.rs         # MinimalPlugins app tests
```

Why two crates: the only split that pays is bevy-free vs bevy-dependent. `stdr_core` compiles in seconds, needs
no GPU for `cargo test`, and stays bevy-free through M4 (the camera only adds plain data and scheduling). Plotters stay inside `stdr_app` because `inventory`
entries in a separate rlib can be dropped by the linker (same problem as C++ `WHOLE_ARCHIVE`).

`stdr_core` keeps C++ *test* names; types and modules are consolidated per §Core types. Engine owns map + robots.
`Result<T, CoreError>` (`thiserror`) replaces `tl::expected`. RNG: one `rand::StdRng` in the engine, `new(step_dt, seed: Option<u64>)`.
Not bit-compatible with `mt19937`, so golden tests cover noise-off paths and statistical properties only.

## Core types (stdr_core)

Each entry names the C++ it collapses and the gtests that become its tests.

**Pose** (`pose.rs`) — collapses `geometry_utils.cpp` (`compose`, `inverse`, `compute_sensor_world_pose`,
`body_to_pivot_pose`, `pivot_to_body_pose`), `angle_utils.hpp`, GUI `helpers::{map_to_robot, robot_to_map,
wrapped_angle_diff}`, `pose_utils::transform_to_world`, and two inline rotate+translate blocks in `collision_checker.cpp`.
```rust
#[derive(Clone, Copy, PartialEq, Debug, Default, Deserialize)] #[serde(default)]
pub struct Pose2D { pub x: f64, pub y: f64, pub theta: f64 }
pub struct Point2D { pub x: f64, pub y: f64 }
pub struct Twist2D { pub linear_x: f64, pub linear_y: f64, pub angular_z: f64 }
impl Mul for Pose2D { .. }                      // SE(2) compose; theta = a + b, NOT wrapped (C++ parity)
impl Pose2D {
    pub fn inverse(self) -> Pose2D;
    pub fn transform_point(self, p: Point2D) -> Point2D;   // R(theta)·p + t
    pub fn wrapped(self) -> Pose2D;                        // theta := normalize_angle(theta)
    pub fn translation(p: Point2D) -> Pose2D;
}
pub fn normalize_angle(a: f64) -> f64 { a.sin().atan2(a.cos()) }   // the one atan2(sin,cos) site
pub fn angle_diff(a: f64, b: f64) -> f64 { normalize_angle(a - b) }
```
Sensor world pose = `robot.pose * sensor.common.pose`. `body_to_pivot = pose * Pose2D::translation(pivot)`;
`pivot_to_body = pose * Pose2D::translation(-pivot)`. Tests: `ComposeInverseTest.*`, `BodyToPivotPoseTest.*`,
`PivotToBodyPoseTest.*`, `PlotHelpers.{MapToRobot*, RobotToMapInvertsMapToRobot, WrappedAngleDiff*}`.

**Footprint** (`footprint.rs`)
```rust
pub enum Footprint { Circle { radius: f64 }, Polygon(Vec<Point2D>) }
impl Footprint {
    pub fn contains(&self, p: Point2D) -> bool;     // ray-crossing + edge eps 1e-9; n < 3 → false
    pub fn vertices(&self) -> Cow<'_, [Point2D]>;   // Polygon → borrow; Circle → 1° ring (C++ expand_footprint)
}
```
Loader: `radius` only → Circle; `points` → Polygon; both → error. Replaces the `points.empty()` encoding.
Tests: `PointInFootprintTest.*`, `LoadRobotConfig.{SimpleRobotParsesFootprintRadius, FootprintPoints*}`.

**Kinematics** (`config.rs`)
```rust
pub struct Alphas(pub [[f64; 3]; 4]);                // rows Ux, Uy, W, G; cols ux², uy², w²
#[repr(usize)] pub enum AlphaRow { Ux, Uy, W, G }
impl Alphas { pub fn variance(&self, row: AlphaRow, u: Twist2D) -> f64 }   // quadratic form, once (C++: 7×)
#[derive(Deserialize, Default)] #[serde(rename_all = "lowercase")] pub enum KinematicKind { #[default] Ideal, Omni }
pub enum OdometryModel { #[default] Perfect, Velocity }
pub struct KinematicConfig { pub kind: KinematicKind, pub odometry: OdometryModel, pub alphas: Alphas }
```
`Alphas` deserialises via a private 12-field `AlphasYaml` (`a_ux_ux` …) and one `TryFrom` impl (rejects negative or non-finite alphas at load) — the only place the
YAML names exist. `""` → Ideal (C++ parity); unknown strings rejected **at load** (C++ accepted at load, threw at spawn).
Tests: `ApplyNoiseTest.*`, `OdometryVarianceTest.*`, `LoadRobotConfig.{SimpleRobotLoadsKinematic, KinematicOdometryModel*, InvalidOdometryModelReturnsError}`.

**Sensors** (`config.rs` + `sensors.rs`)
```rust
pub struct SensorCommon { pub pose: Pose2D, pub frequency: f64, pub frame_id: String, pub noise_std: f64 }  // 0 = off
pub struct LaserSpec { pub min_angle: f64, pub max_angle: f64, pub min_range: f64, pub max_range: f64, pub num_rays: i32 }
pub struct SonarSpec { pub min_range: f64, pub max_range: f64, pub cone_angle: f64 }
pub enum SensorConfig { Laser(LaserSpec), Sonar(SonarSpec), Camera(CameraSpec) }   // Rfid/Co2/Thermal/Sound: future variants
pub struct Sensor { pub common: SensorCommon, pub kind: SensorConfig }
pub struct LaserScan { angle_min, angle_max, angle_increment, range_min, range_max, ranges: Vec<f32> }
pub struct SonarScan { pub range: f64 }
pub enum Measurement { Laser(LaserScan), Sonar(SonarScan) }
impl Measurement { pub fn as_laser(&self) -> Option<&LaserScan>; pub fn as_sonar(&self) -> Option<&SonarScan> }
pub struct RobotConfig { initial_pose, footprint, center_of_rotation, kinematic, sensors: Vec<Sensor> }
```
`NoiseConfig.mean` / `.enabled` dropped (C++ never reads `mean`; `enabled` ≡ `std_dev > 0`). One `frame_id`
default loop with per-kind counters (`laser_{i}` / `sonar_{i}`; counter increments even for named sensors).
`SensorCommon` and `LaserSpec` / `SonarSpec` both derive `Deserialize` and are read from the *same* merged `Value`
(two `from_value` calls); `SensorCommon.noise_std` uses `deserialize_with` over `noise.noise_specifications.noise_std`.
No per-kind `XYaml` + `From` pair: adding a kind = one spec struct + one enum variant + one loader-table entry + one `simulate` arm.
Tests: `LoadRobotConfig.{SimpleRobotLoadsLaserFromFile, LaserNoiseLoadedFromFile, SensorWith*FrameId*, SonarAutoIndexIsStableAcrossMixedNames, InlineOnlyLaserAndKinematic}`.

**OccupancyGrid** (`grid.rs`) — collapses five world→cell sites, three `kOccupancyThreshold` declarations, and
both ray-march loops (`laser_simulator.cpp:56-81` ≡ `sonar_simulator.cpp:58-80`).
```rust
pub const OCCUPANCY_THRESHOLD: i8 = 70;
pub enum Unknown { Solid, Transparent }      // the one explicit policy knob (C++: two divergent comparisons)
pub struct OccupancyGrid { width: u32, height: u32, resolution: f64, origin: Pose2D, data: Vec<i8> }   // getters; `new` is the only constructor
impl OccupancyGrid {
    pub fn new(width: u32, height: u32, resolution: f64, origin: Pose2D, data: Vec<i8>) -> Result<Self, CoreError>;
        // invariants enforced once, at the seam: resolution > 0, width/height > 0, data.len() == w*h.
        // Sensors and collision drop their per-call guards (C++ guarded resolution <= 0 in three places; sonar returned a finite max_range).
    pub fn world_to_cell(&self, x: f64, y: f64) -> (i32, i32);   // ((x - ox) / res) as i32 — truncation quirk kept
    pub fn grid_coords(&self, x: f64, y: f64) -> (f64, f64);     // continuous, for ray origins
    pub fn in_bounds(&self, c: (i32, i32)) -> bool;
    pub fn at(&self, c: (i32, i32)) -> Option<i8>;               // None = out of bounds
    pub fn is_blocked(&self, c: (i32, i32), unknown: Unknown) -> bool;   // OOB → true; > 70 → true; -1 → policy
    pub fn raycast(&self, origin: (f64, f64), angle: f64, max_steps: i32, unknown: Unknown) -> Option<i32>;
        // steps 1..=max; cell = (ox + cos·step) as i32; Some(step) on first blocked in-bounds cell; None if exits map or clear
}
```
Collision passes `Unknown::Solid`, sensors `Unknown::Transparent` — same observable behaviour as C++, policy now
a named parameter at two call sites. `origin.theta != 0` is rejected at map load (C++ parsed and ignored it;
all 8 shipped maps are 0). Tests: `CollisionCheckerTest.{OutOfBoundsCollision, OccupiedCellCollision}`,
`LaserSimulatorTest.RayExitingMapIsPositiveInfinity`, new `grid::unknown_policy_is_explicit`,
`grid::rejects_nonpositive_resolution`, `grid::rejects_empty` (replace `CollisionCheckerTest.{ZeroResolutionReturnsCollision, EmptyMapReturnsCollision}`).

**Scheduler** (`scheduler.rs`) — keyed by **sensor index** (position in `config.sensors`). The C++ `StreamKind::{Tf, Odom}`
streams are not ported: the standalone engine ignores their events (`simulation_engine.cpp:350-354`) and the panel rows
are gated on `publishes_ros_topics()`, which is always false. A ROS transport that needs publish cadence adds its own entries.
```rust
pub enum SchedulingMode { SnapToMultiple, Accumulator }      // FromStr / Display
enum Schedule { Snap { period_ticks: u64 }, Accum { period_s: f64, acc: f64 } }   // period 0 → every tick (C++ sentinel kept)
struct Entry { freq_hz: f64, ticks: u64, schedule: Schedule }                     // ticks preserved across set_rate (C++)
pub struct RateScheduler { step_dt: f64, entries: BTreeMap<usize, Entry> }
impl RateScheduler {
    pub fn new(step_dt) -> Result<Self>; pub fn set_step_dt(dt) -> Result<()>;    // recomputes every period
    pub fn set_rate(idx, hz, mode); pub fn tick(&mut self) -> Vec<usize>; pub fn clear();
    pub fn effective_rate(idx) -> Option<f64>; pub fn period_ticks(idx) -> Option<u64>; pub fn mode(idx) -> Option<SchedulingMode>;
}
```
`BTreeMap` replaces the `kind << 32 | idx` key and the post-sort. Tests: `RateSchedulerTest.*`, `EffectiveRateTest.*`,
`AccumulatorModeTest.*`, `SchedulingModeStringTest.*` (all re-keyed to sensor indices).

**Engine** (`engine.rs`) — `WorldModel` deleted; the C++'s four parallel maps (`robots_`, `sensor_data_`,
`schedulers_`, `last_events_`) existed only because of the world/engine split.
```rust
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)] pub struct RobotId(u32);   // Display "robot{n}", FromStr
pub struct RobotState { pub pose: Pose2D, pub odom_pose: Pose2D, pub cmd_vel: Twist2D }
pub struct RobotRuntime {
    pub config: RobotConfig, pub initial_pose: Pose2D, pub state: RobotState,
    pub data: Vec<Option<Measurement>>,   // index = sensor index; None until first fire (e.g. no map). Stale data retained.
    pub collided: bool, scheduler: RateScheduler,
    pub fired: Vec<usize>,   // M4: indices the last step fired, the app's "capture this frame" signal for cameras
}
impl RobotRuntime { pub fn sensor_index(&self, frame_id: &str) -> Option<usize>; pub fn sensor_world_pose(&self, i: usize) -> Pose2D }
pub struct SimulationEngine { map: Option<OccupancyGrid>, map_revision: u64, robots: BTreeMap<RobotId, RobotRuntime>,
                              next_id: u32, step_dt: f64, elapsed: f64, ticks: u64, mode: SchedulingMode, rng: StdRng }
impl SimulationEngine {
    pub fn new(step_dt: f64, seed: Option<u64>) -> Result<Self>;       // ONE rng; None → entropy
    pub fn set_step_dt(&mut self, dt) -> Result<()>; pub fn step_dt(&self) -> f64;
    pub fn sim_time(&self) -> f64; pub fn ticks(&self) -> u64;         // engine owns sim time (C++ kept elapsed_time_ in the backend)
    pub fn set_scheduling_mode(&mut self, m);
    pub fn set_map(&mut self, g: OccupancyGrid); pub fn map(&self) -> Option<&OccupancyGrid>; pub fn map_revision(&self) -> u64;
    pub fn spawn(&mut self, cfg: RobotConfig, pose: Pose2D) -> RobotId; pub fn remove(&mut self, id) -> bool;
    pub fn robot(&self, id) -> Option<&RobotRuntime>; pub fn robots(&self) -> impl Iterator<Item = (RobotId, &RobotRuntime)>;
    pub fn set_cmd_vel(&mut self, id, t: Twist2D) -> bool; pub fn teleport(&mut self, id, p: Pose2D) -> bool;   // collapses odom
    pub fn effective_rate(&self, id, sensor_idx: usize) -> Option<f64>;   // replaces 7 C++ getters
    pub fn step(&mut self);   // uses self.step_dt — single time source; elapsed += step_dt; ticks += 1
    pub fn reset(&mut self);  // all robots → initial_pose, zero cmd_vel, data = None, collided = false; elapsed = 0; ticks = 0
}
```
`step` iterates `robots.values_mut()` in place (no per-tick `RobotState` copies): `perturb → integrate(truth) /
integrate(odom) → path_collides → commit (collision keeps truth, advances odom) → for i in scheduler.tick():
fired.push(i); data[i] = simulate(...)` (camera → stays `None`); then `elapsed += step_dt; ticks += 1`. `elapsed` accumulates per step (not `ticks * step_dt`)
so a `set_step_dt` mid-run keeps time continuous. Tests: `WorldModelTest.*` (robots/map), `SimulationEngineTest.*`
(minus `RateSchedulerOdomFiresEveryTickAtMatchedRate`, `RateSchedulerTfClampedAtSimRate`: assertions already exist for sensor streams),
`StandaloneBackend.{ResetRestoresSpawnPose*, TeleportResetsOdomPose, OdomPose*Model}` on `reset`/`teleport`,
new `engine::step_uses_stored_step_dt`, `engine::seeded_engine_is_deterministic`, `engine::sim_time_accumulates_across_step_dt_change`,
`engine::reset_zeroes_sim_time`.

**Motion** (`motion.rs`) — `ideal_*` + `omni_*` + `noise_model` → one `integrate` with `match kind` only on the
3–6 differing lines; pivot shift + wrap shared. `perturb` = C++ `apply_noise` (dt ≤ 0 → Err; Perfect short-circuit).
Tests: `IdealMotionModel*`, `OmniMotionModel*`, `{Ideal,Omni}MotionModelCenterOfRotationTest.*`, `{Ideal,Omni}MotionModelIntegrateTest.*`.

**Sensors** (`sensors.rs`) — laser = per-ray `grid.raycast` + `finish`; sonar = `min` over the 1° float-accumulated
sweep of the same. `finish(hit, res, min, max, noise_std, rng) -> f64`: no-hit → +Inf untouched; add noise;
`< min` → −Inf; `>= max` → +Inf. Tests: `LaserSimulatorTest.*`, `SonarSimulatorTest.*`.

**Collision** (`collision.rs`) — `path_collides` uses `grid.is_blocked(.., Solid)`, `Pose2D::transform_point`,
3×3 neighbourhood, OOB = collision, **shortest-arc** theta lerp (C++ lerped raw theta: crossing ±π swept a full turn).
The C++ single-pose `check_collision` is not ported: standalone has no caller (only the ROS robot node used it).
Tests: `CollisionCheckerTest.{FreeCellNoCollision, OccupiedCellCollision, OutOfBoundsCollision}` via `path_collides(from == to)`,
`CollisionCheckerTest.{PathCollisionDetected, PathNoCollision}`, new `collision::rotation_through_pi_uses_shortest_arc`.
(`ZeroResolution*` / `EmptyMap*` moved to `grid::new` tests.)

**Teleop** — lives in `stdr_app/src/ui/teleop.rs`, not core (C++ keeps it in the GUI library too; it needs only `Twist2D` +
`KinematicKind`). `teleop_twist(keys: TeleopKeys, kind: KinematicKind, speeds) -> Twist2D`: W/Up S/Down A/Left D/Right Q/E;
opposing keys cancel; strafe only for Omni; defaults 0.5 m/s, 1.0 rad/s. The C++ `TeleopController` class and
`kinematic_type_from_string` are not ported. Tests: `TeleopController.*` minus `KinematicTypeFromString*`, as plain unit tests (no Bevy).

## Config loading

Mechanism: expand includes and deep-merge at `serde_yaml_ng::Value` level, then `from_value` into
`#[serde(default)]` structs. Why: the C++ semantics ("file = base, inline overrides field-wise, recursively") **is**
a mapping deep-merge. Doing it on `Value` handles nested partial `pose:` override, nested `noise: {filename:}`
includes, and inline `noise_std: 0` disabling, for every kind with zero per-type code. `Partial<T>` overlay
structs would re-create the C++ per-type duplication (12 `Option<f64>` for alphas alone).

```rust
/// {filename: F, <k>_specifications: {...}}  →  <k>_specifications = deep_merge(F[k][<k>_specifications], inline).
/// Recurses into the merged mapping so a laser file's `noise: {filename: ...}` is expanded too.
fn resolve_includes(node: &mut Value, kind: &str, base_dir: &Path) -> Result<(), CoreError>;
fn resolve_includes_in(v: &mut Value, base_dir: &Path) -> Result<(), CoreError>;   // every child mapping with filename/<key>_specifications
fn deep_merge(base: &mut Value, overlay: Value);        // Mapping ∧ Mapping → recurse per key; otherwise overlay wins
pub(crate) fn load_with_include<T: DeserializeOwned>(node: Value, kind: &str, base_dir: &Path) -> Result<T, CoreError>;
```
Per-kind YAML structs (`LaserYaml { max_angle, …, frequency, frame_id, pose: Pose2D, noise: NoiseYaml }`,
`NoiseYaml { noise_specifications: { noise_std } }`) are `#[serde(default)]` with one `From` each into `Sensor`.
`load_robot_config` walks `robot.robot_specifications` (sequence of single-key maps) and dispatches on the key
through a table `[("laser", ..), ("sonar", ..)]` plus `footprint` / `initial_pose` / `center_of_rotation` /
`kinematic`; this table is the only per-kind fan-out left in the loader. Unsupported kinds (`rfid_reader`,
`co2_sensor`, `thermal_sensor`, `sound_sensor`) → warning, skipped. `base_dir` = `$STDR_RESOURCES_DIR` else the
robot yaml's directory. Post-parse validation: `footprint.contains(center_of_rotation)`. Footprint `point:`
wrapper and bare `{x, y}` both accepted (C++ parity). Map yaml: `image` relative to the yaml dir, `resolution`
required, `origin: [x, y, theta]` optional, `occupied_thresh` 0.65, `free_thresh` 0.196, `negate` int. The metadata
struct is private to `map.rs`; `load_map(yaml) -> Result<OccupancyGrid>` is the whole interface (C++ exported
`MapMetadata` only because two packages shared the YAML parser). `LoadMapMetadata.*` tests go through `load_map`
with 2×2 PGM fixtures: thresholds → cell values, negate, origin, missing `image`/`resolution` → `Err`.

## ECS mapping (stdr_app)

Source of truth = `SimulationEngine` in a Resource, read directly by systems (`Res<SimWorld>`). No snapshot copy:
the C++ snapshot existed for a sim thread we do not have, and Bevy parallelises readers. Mutation only in
`apply_sim_commands` (PreUpdate) and `sim_step` (FixedUpdate). Bevy entities are render mirrors synced once per frame.

**Resources** — every value has exactly one owner. Sim time, step_dt → engine. Pause, speed → `Time<Virtual>`.
No mirrors (the C++ toolbar shadowed `speed_multiplier_` because the backend had no getter).
- `SimWorld(SimulationEngine)` — also the source of `sim_time()`, `ticks()`, `step_dt()`. Never use Bevy time as sim time.
- `CatchUp { cap: 0.25, fell_behind_warned: bool }` — the only app-level sim-control state.
- `Selection { robot: Option<RobotId> }` — the **one** selected-robot state (map pick, robot info, teleop, all plotters).
  Falls back to first robot on spawn if `None`; cleared on delete. (C++ had three independent notions.)
- `TeleopState { driving: Option<RobotId>, speeds: TeleopSpeeds }`
- `RobotEntities(HashMap<RobotId, Entity>)`, `MessageLog`, `SensorVisibility(HashSet<(RobotId, usize)>)`
- `MapTexture { image: Handle<Image>, egui_id, size_m, origin, revision: u64 }` — one texture shared by sprite and
  `egui_plot::PlotImage`; `sync_map_texture` rebuilds only when `revision != world.map_revision()` (replaces both
  C++ texture-update paths, which used pointer compare and a full-grid compare per frame).
- `PlotterFilter(Vec<String>)` from CLI

**Components** (render only): `RobotMarker { id }` + `Transform`, `OdomGhost { pose }`, `Trail`, `MapSprite`, `MainCamera`.

**Messages** (Bevy 0.17+ renamed buffered events to `Message`/`MessageWriter`/`MessageReader`)
- `SimCommand`: `LoadMap`, `SpawnRobot{path, pose}`, `DeleteRobot(RobotId)`, `Start`, `Pause`, `Reset`, `SetSpeed`,
  `SetStepDt`, `Teleport{id, pose}`, `CmdVel{id, twist}`. One enum replaces the C++ mutating backend API and
  `CommandQueue`. Pause/Resume via queue (C++ `FIXME: NOT IMPLEMENTED`) is free.
- `SimEvent`: `MapLoaded`, `RobotSpawned(RobotId)`, `RobotDeleted(RobotId)`, `Reset`, `Paused`, `Resumed`, `FellBehind(f64)`, `Log(String)`.

**Schedule**
- `PreUpdate`: `apply_sim_commands` — drain `SimCommand`, mutate `SimWorld` / `CatchUp`, emit `SimEvent`, set
  `Time<Virtual>` pause/speed and `Time<Fixed>` timestep. Then `sync_map_texture`.
- `FixedUpdate`: `sim_step` — `engine.step()` only.
- `Update`: `sync_robot_mirrors`, `camera_pan_zoom`, `pick_robot`, `right_click_teleport`, `teleop_input`,
  `draw_overlay` (`GizmoCanvas` + `overlay::{draw_robot, draw_sensors, draw_trail}`), plotter `sample` systems (after sync).
- `EguiPrimaryContextPass`: `toolbar`, `robot_info`, `messages`, `teleop_window`, plotter `render` systems.
  Toolbar/status read `Time<Virtual>::{is_paused, relative_speed}`, `engine.step_dt()`, `engine.sim_time()` directly.

**Time mapping**
- `step_dt` → `Time::<Fixed>::set_timestep`, clamp [0.001, 1.0], default 0.01. Menu offers 0.01/0.05/0.1/0.2/0.5.
- `speed` → `Time::<Virtual>::set_relative_speed`. Menu 0.5x/1x/2x/5x.
- pause → `Time::<Virtual>::pause()/unpause()`. Bevy stops fixed overstep while paused and re-anchors on unpause = C++ "never replay paused time".
- catch-up cap → `Time::<Virtual>::set_max_delta(cap / speed)`, re-applied on every `SetSpeed`. Bevy clamps *real*
  delta before multiplying by speed, hence the division. Emit `FellBehind` once (edge-triggered) when `Time<Real>::delta() > max_delta`.
- Reset: pause, `engine.reset()` (zeroes sim time too), emit `SimEvent::Reset` (plotters reset to `Default`).

**Teleop** — `teleop_input` (gated on `!wants_keyboard_input()`): compute twist for `Selection.robot`. Non-zero →
`CmdVel(sel)`, `driving = Some(sel)`. Zero, or selection changed, or input captured, or no selection →
`if let Some(prev) = driving.take() { CmdVel(prev, zero) }`. Fixes the C++ bug where the stop went to the
*currently* selected robot and the previous one kept driving. Exactly one zero on release.

**Toolbar / robot info** — `const SPEEDS: [f64; 4]`, `const TIMESTEPS: [(f64, &str); 5]`, one loop each (C++: if-chains).
`sim_buttons(ui, &mut writer)` shared by menu and control bar. Robot info: one loop over `config.sensors`,
`match kind` only for the label (C++: six copied button loops); rate via `engine.effective_rate(id, i)`.

**Overlay** (`overlay.rs`) — one routine decides *what* a robot looks like; adapters decide *where* it is drawn.
C++ had the robot overlay (footprint, heading, collided tint, odom ghost, trail, scan rays, sonar cone) in the map
panel and again in three plotters.
```rust
pub struct Style { pub color: [u8; 4], pub width: f32 }
pub trait Canvas {
    fn polyline(&mut self, pts: &[[f64; 2]], style: Style, closed: bool);
    fn points(&mut self, pts: &[[f64; 2]], style: Style, radius: f32);
}
pub fn draw_robot(c: &mut impl Canvas, r: &RobotRuntime, selected: bool);   // footprint (red if collided), heading, odom ghost
pub fn draw_sensors(c: &mut impl Canvas, r: &RobotRuntime, id: RobotId, visible: &SensorVisibility);  // match kind: laser rays / sonar cone
pub fn draw_trail(c: &mut impl Canvas, xy: &[[f64; 2]], style: Style);
// pure geometry helpers, also used by plotters: footprint_polygon, heading_segment, scan_endpoints, sonar_cone, pose_error
```
Adapters: `GizmoCanvas<'a>(&'a mut Gizmos)` in `view2d/robots.rs` (f64 → `Vec2` here and nowhere else),
`PlotCanvas<'a>(&'a mut egui_plot::PlotUi)` in `plot/overlay_plot.rs`, `RecordingCanvas(Vec<Call>)` in tests.
Three adapters, one of them test-only = a real seam. `Canvas` works in f64 world coords; colours as `[u8; 4]` so
neither bevy nor egui types appear in `overlay.rs`. Test: `overlay::draw_robot_records_expected_calls`.

## Plotters

No trait. Each plotter = one `Resource` state type `P: Default` + a sample system + a render system. The host provides:

```rust
// plot/mod.rs
#[derive(Resource)] pub struct PlotterCtl<P> { pub paused: bool, pub removed: bool, _p: PhantomData<P> }
pub fn plotter_active<P: 'static>(c: Res<PlotterCtl<P>>) -> bool { !c.paused && !c.removed }
pub fn plotter_window<P>(ctx: &egui::Context, ctl: &mut PlotterCtl<P>, name: &str, body: impl FnOnce(&mut egui::Ui));
    // egui::Window(name) with Pause checkbox + Remove button, then body
pub fn add_plotter<P: Resource + Default>(app: &mut App, sample: impl IntoSystemConfigs<M1>, render: impl IntoSystemConfigs<M2>) {
    app.init_resource::<P>().init_resource::<PlotterCtl<P>>()
       .add_systems(Update, sample.after(sync_robot_mirrors).run_if(plotter_active::<P>))
       .add_systems(EguiPrimaryContextPass, render.run_if(|c: Res<PlotterCtl<P>>| !c.removed))
       .add_systems(PreUpdate, reset_on_event::<P>.after(apply_sim_commands));   // SimEvent::Reset → *P = P::default()
}
pub struct PlotterEntry { pub key: &'static str, pub name: &'static str, pub description: &'static str, pub build: fn(&mut App) }
inventory::collect!(PlotterEntry);
macro_rules! register_plotter { ($key:ident, $name:literal, $desc:literal, $build:expr) => { inventory::submit! { PlotterEntry { key: stringify!($key), .. } } } }

pub trait Positioned { fn position(&self) -> Point2D }       // impls: Pose2D, scan_trace::Sample — two impls = real seam
pub struct Trail<T> { buf: VecDeque<T>, xy: Vec<[f64; 2]>, cap: usize, spacing: f64 }
impl<T: Positioned> Trail<T> { pub fn push_if_moved(&mut self, t: T) -> Push /* Skipped | Pushed { evicted_front: bool } */;
                               pub fn clear(&mut self); pub fn xy(&self) -> &[[f64; 2]]; pub fn get(&self, i: usize) -> Option<&T>; pub fn len(&self) -> usize }
pub struct TeleportDetector { last: Option<Point2D>, threshold: f64 }   // observe(p) -> bool jumped; reset()
pub struct TimeSeries { pts: Vec<[f64; 2]>, cap: usize }                 // push(t, v); last(); line(name) -> egui_plot::Line; clear()
pub struct SampleGate { period: f64, last: Option<f64> }                 // due(now_sim_time) -> bool; fires on backwards time
```

Pause/remove are plain fields. A plotter needing a pause side-effect adds one system with
`run_if(resource_changed::<PlotterCtl<P>>)`. Reset = `Default`. Registry: `main` dedups `--plotter` keys, warns on
unknown, calls `build` for matches (empty filter = all). A complete plotter:

```rust
#[derive(Resource, Default)] struct PoseError { xy: TimeSeries, th: TimeSeries, gate: SampleGate /* 50 ms */ }
fn sample(sim: Res<SimWorld>, sel: Res<Selection>, mut st: ResMut<PoseError>, mut cmd: MessageWriter<SimCommand>) {
    let Some(id) = sel.robot else { return };
    cmd.write(SimCommand::CmdVel { id, twist: Twist2D { linear_x: 0.3, linear_y: 0.0, angular_z: 0.5 } });
    let t = sim.sim_time();
    if !st.gate.due(t) { return }
    let Some(r) = sim.robot(id) else { return };
    let e = overlay::pose_error(r.state.pose, r.state.odom_pose);
    st.xy.push(t, e.xy); st.th.push(t, e.theta);
}
fn render(mut ctx: EguiContexts, st: Res<PoseError>, mut ctl: ResMut<PlotterCtl<PoseError>>) {
    plotter_window(ctx.ctx_mut(), &mut ctl, "Pose Error", |ui| {
        Plot::new("pose_error").x_axis_label("sim time (s)").show(ui, |p| { p.line(st.xy.line("|truth - odom| (m)")); p.line(st.th.line("yaw error (rad)")); });
    });
}
fn stop_on_pause(ctl: Res<PlotterCtl<PoseError>>, sel: Res<Selection>, mut cmd: MessageWriter<SimCommand>) {
    if ctl.paused { if let Some(id) = sel.robot { cmd.write(SimCommand::CmdVel { id, twist: Twist2D::default() }); } }
}
register_plotter!(PoseError, "Pose Error", "Drives the selected robot in a circle and plots truth vs odometry.", |app| {
    add_plotter::<PoseError>(app, sample, render);
    app.add_systems(Update, stop_on_pause.run_if(resource_changed::<PlotterCtl<PoseError>>));
});
```

- **Map Trace** = `Trail<Pose2D>` + `TeleportDetector` + `draw_map` + `overlay::{draw_trail, draw_robot}` via `PlotCanvas`, "Lock view to map" checkbox.
- **Odometry Trace** = two trails + `TimeSeries` ×2 + latest-error readout; truth/odom drawn by `overlay::draw_robot`.
- **Scan Trace** = `Trail<Sample { truth, odom, scan: Option<LaserScan> }>` cap 500, spacing 0.1 m; `Option<Selected { index, from_odom }>`
  adjusted on `Pushed { evicted_front: true }`; click pick via `plot_ui.pointer_coordinate()` + `transform().position_from_point()` ≤ 8 px;
  right-click clears; only the selected scan rendered (never all 500).
- The C++ "lazy first robot" and "fetch footprint once" blocks vanish: read `Selection` + `sim.robot(id).config` each sample (a map lookup).
- No egui_dock; plain `egui::Window`s. Add docking only if layout becomes a problem.

**`plot/overlay_plot.rs`**: `PlotCanvas` (the `Canvas` adapter over `PlotUi`), `map_plot(id, lock_view) -> Plot`
(equal aspect, inputs off when locked), `draw_map(plot_ui, &MapTexture)`, colour consts `TRUTH`, `ODOM`, `SCAN`.
Everything robot-shaped goes through `overlay::draw_*` (see §ECS mapping → Overlay). The geometry helpers in
`overlay.rs` replace 5 C++ transform copies + 4 scan-endpoint copies.

**Data flow**
```
FixedUpdate sim_step:  engine.step() -> engine.robot(id).data[i] (latest Measurement per sensor), engine.sim_time()
Update      sample:    Res<SimWorld> + Res<Selection> + ResMut<P>   (once per frame; SampleGate on sim_time for scalar plots)
EguiPass    render:    Res<P> -> PlotCanvas / egui_plot::{Line, PlotImage}
```

## 2D rendering + input

- `OccupancyGrid -> Image` (`Rgba8UnormSrgb`): 0→white, 100→black, −1→grey. Grid row 0 = bottom; image row 0 = top; flip once on build. Rebuild on `map_revision` change.
- Bevy 2D is X-right/Y-up = ROS map frame. Poses map to `Transform` directly, no flip. Map sprite `custom_size = (w·res, h·res)`, `translation = origin + size/2`.
- Robots and sensors via `GizmoCanvas` + `overlay::{draw_robot, draw_sensors, draw_trail}`: footprint, heading, red tint when
  `collided`, orange odom ghost, trail, laser rays, sonar cone. Per-sensor toggles via `SensorVisibility`.
- `Camera2d`: wheel zoom about cursor (`OrthographicProjection::scale`), middle-drag pan, fit-to-view on map load. All pointer input gated on `!egui_ctx.wants_pointer_input()`.
- Left-click selects nearest robot (`viewport_to_world_2d` + `Footprint::contains`, 15 px fallback) → `Selection`. Right-click egui context menu: "Teleport here", "Delete robot", "Lock view".
- Toolbar (egui top panel): File → Load Map / Load Robot (`rfd` file dialog), Simulation → Start/Pause/Reset, Speed, Timestep; status bar with elapsed sim time + messages. Spawn dialog (x/y/theta) after Load Robot, prefilled from YAML `initial_pose`.
- CLI `--x/--y/--theta` override YAML `initial_pose`; when absent, YAML is honoured (C++ ignored YAML: documented difference).

## M4: 3D scene + camera sensor

The 3D view is a second way to look at the same engine. It reads existing core types, adds no
physics and keeps `stdr_core` bevy-free. The camera is the one new sensor kind: core schedules it,
the app renders it.

**Camera sensor (core).** The C++ STDR has no camera, so the yaml schema is new. It follows the
other kinds: a `camera` entry with optional `filename` include and inline `camera_specifications`
deep-merged on top.
```yaml
- camera:
    camera_specifications:
      pose: {x: 0.1, y: 0, theta: 0}   # mount pose in the body frame (SensorCommon)
      frequency: 10                    # Hz, scheduled like any sensor (0 = every tick)
      frame_id: front_camera           # default camera_{n}
      width: 320                       # image pixels
      height: 240
      fov: 1.0472                      # horizontal field of view, rad, in (0, π)
      near: 0.05                       # clip planes, m, 0 < near < far
      far: 50.0
```
```rust
pub struct CameraSpec { pub width: u32, pub height: u32, pub fov: f64, pub near: f64, pub far: f64 }  // Default 320×240, 60°, 0.05, 50
pub enum SensorConfig { Laser(LaserSpec), Sonar(SonarSpec), Camera(CameraSpec) }
pub fn simulate(..) -> Option<Measurement>;   // None for Camera: core never renders, Measurement holds no image
pub struct RobotRuntime { .., pub fired: Vec<usize> }   // sensor indices the last step's scheduler fired
```
Invalid specs (zero size, fov outside (0, π), `near <= 0` or `far <= near`) are rejected at load:
the renderer would panic on them. `fired` is cleared at the start of every `step` and by `reset`,
and lists every fired index, map or no map. A camera's `data[i]` stays `None`. Every existing robot
yaml loads unchanged; `stdr_resources/resources/robots/camera_robot.yaml` is the example.

**Meshes** (`stdr_app/src/scene3d/mesh.rs`, pure functions, ROS frame: metres, Z up).
```rust
pub fn extrude_grid(g: &OccupancyGrid, height: f32) -> Mesh;     // occupied (> OCCUPANCY_THRESHOLD) cells
pub fn extrude_footprint(f: &Footprint, height: f32) -> Mesh;
```
`extrude_grid` merges each row's run of occupied cells into one box (greedy row merge) and emits its
top and four sides; there is no bottom face (the floor covers it). One quad = 4 vertices with a
flat normal + 2 triangles, so a run costs 20 vertices / 30 indices. Unknown cells are not walls
(sensor policy). `frieburg.png` has 16 578 occupied cells in 3 758 runs, about 75k vertices.
`extrude_footprint` = side walls over `Footprint::vertices()` (a circle is the 360-point ring) plus
an ear-clipped top cap, so concave footprints such as `random_shape_robot.yaml` cap correctly.

**Scene** (`stdr_app/src/scene3d/mod.rs`, `Scene3dPlugin`).
- `SceneRoot` entity with `Transform::from_rotation(Quat::from_rotation_x(-FRAC_PI_2))`: ROS
  `(x, y, z)` → Bevy `(x, z, −y)`. Everything below it is in ROS coordinates.
- Floor: a map-sized quad at z = 0 textured with `MapTexture.image` (unlit). Walls: one
  `extrude_grid(grid, WALL_HEIGHT)` entity, rebuilt when `MapTexture` changes (i.e. on map revision).
- `sync_robot_mirrors` (Update): diffs `engine.robots()` against `RobotEntities(HashMap<RobotId,
  Entity>)`; spawns `(RobotMarker { id }, Mesh3d(extrude_footprint), ChildOf(root))`, sets
  `Transform` to `(x, y, 0)` + `rot_z(theta)` every frame, despawns deleted robots (children too).
  Mirrors and cameras exist in both view modes, so camera images keep updating in 2D.
- `ViewMode { TwoD (default), ThreeD }` toggled from the toolbar. 2D: unchanged. 3D: the
  orbit `Camera3d` (order −1) renders the scene; the 2D camera stays active only to host egui
  (`PrimaryEguiContext`, clear colour `None`), the map sprite is hidden, and the gizmo overlay,
  2D picking and 2D pan/zoom are off. Orbit camera: left-drag orbits, middle-drag pans, wheel
  zooms, fitted to the map on load; pointer input gated on egui.

**Camera rendering** (`stdr_app/src/scene3d/camera_sensor.rs`).
- When a mirror spawns, each camera sensor gets a `Camera3d` child (`is_active: false`,
  `RenderTarget::Image`) at `(x, y, CAMERA_Z)` looking along the sensor heading with ROS +Z up;
  Bevy's vertical fov = `2·atan(tan(fov/2)·height/width)`. The app owns
  `CameraFrames(HashMap<(RobotId, usize), Handle<Image>>)`; entries go away with the robot.
- Capture: a FixedUpdate system after `sim_step` collects `fired` camera indices into
  `PendingCaptures`; each Update a camera is active iff it is pending, then the set is cleared.
  So a camera renders on frames where at least one of its ticks fired, and a paused sim keeps the
  last picture.
- A "Cameras" egui window shows every `CameraFrames` image (`EguiUserTextures`, no GPU readback).
  No new plotter; readback waits until a plotter needs pixels.

Tests: `scene3d::mesh` unit tests (one-cell grid: 20 vertices, 30 indices, bounds = the cell ×
height; a row run merges to one box; two rows stay two boxes; unknown/free cells emit nothing;
circle footprint side count and bounds; concave cap area = polygon area; normals unit-length and
outward; Z-up→Y-up: a mesh vertex `(x, y, h)` under the root rotation lands at `(x, h, −y)`),
`config::{camera_parsed_with_include_and_defaults, invalid_camera_rejected_at_load,
shipped_camera_robot_loads, every_shipped_robot_loads}`, `engine::camera_fires_by_index_data_stays_none`,
`engine::fired_lists_last_step_indices_without_map`, `engine::reset_clears_fired`, sensors `simulate` camera → `None`.
Perf check: frame time with `frieburg.yaml` in the 3D view, numbers in the PR.

## Bug fixes vs kept quirks

| Kind | Item | Test |
|---|---|---|
| Fix | `noise: {filename: …}` include followed (C++ silently dropped it: `standard_sonar.yaml`, `VL53L0X.yaml` had no noise) | `config::noise_filename_include_is_followed` |
| Fix | Partial inline `pose:` keeps absent file fields (C++ zeroed them) | `config::partial_inline_pose_keeps_file_fields` |
| Fix | Inline `noise_std: 0` disables file noise (C++ could not turn noise off inline) | `config::inline_noise_zero_disables` |
| Fix | `frame_id` parsed for every sensor kind | `config::frame_id_parsed_for_all_kinds` |
| Fix | Unknown `kinematic_model` rejected at load, not at spawn | `config::unknown_kinematic_rejected_at_load` |
| Fix | Negative or non-finite odometry alphas rejected at load (C++ made `perturb` sigma NaN, silently zeroing that noise channel) | `config::negative_alpha_rejected_at_load`, `config::non_finite_alpha_rejected_at_load` |
| Fix | `map.origin[2] != 0` → error instead of silently ignored | `map::rotated_origin_rejected` |
| Fix | Path-collision theta lerp uses shortest arc | `collision::rotation_through_pi_uses_shortest_arc` |
| Fix | Unknown-cell policy explicit (`Unknown`), one `is_blocked` | `grid::unknown_policy_is_explicit` |
| Fix | One time source: `step()` uses stored `step_dt` | `engine::step_uses_stored_step_dt` |
| Fix | One seedable RNG for motion + sensors (C++ sensors unseedable) | `engine::seeded_engine_is_deterministic` |
| Fix | Pause/Resume via `SimCommand` implemented; Reset clears sensor data and plotter state | `headless::pause_resume_via_command`, `headless::reset_defaults_plotter_state` |
| Fix | Teleop stop goes to the robot that was driving | `headless::teleop_stop_targets_previously_driven_robot` |
| Fix | YAML `initial_pose` honoured when `--x/--y/--theta` absent | `cli::yaml_pose_used_when_flags_absent` |
| Fix | `NoiseConfig.mean` / `.enabled` dropped (never read) | `config::noise_mean_ignored` |
| Fix | Sim time owned by the engine; `reset()` zeroes it in the same place as robot state | `engine::sim_time_accumulates_across_step_dt_change`, `engine::reset_zeroes_sim_time` |
| Fix | Grid invariants at construction (`resolution > 0`, dims > 0, `data.len() == w*h`); C++ sonar returned a finite `max_range` on `resolution <= 0`, laser +Inf | `grid::rejects_nonpositive_resolution`, `grid::rejects_empty` |
| Fix | No shadow copies of pause / speed / step_dt in the app (C++ toolbar shadowed speed) | `headless::toolbar_reads_speed_from_virtual_time` |
| Quirk | `as i32` truncation in `world_to_cell` (−0.5 → cell 0, in-bounds) | `grid::world_to_cell_truncates_toward_zero` |
| Quirk | `max_steps = (max_range / res) as i32` | `sensors::max_steps_truncates` |
| Quirk | Sonar sweep `while a <= cone/2 { a += 1° }` float accumulation decides ray count | `sensors::sonar_sweep_ray_count_matches_cpp` |
| Quirk | REP-117: no-hit → +Inf untouched by noise; `< min` → −Inf; `>= max` → +Inf | `LaserSimulatorTest.{NoiseDoesNotAffectNoHitBeam, TooCloseObstacleIsNegativeInfinity}`, `SonarSimulatorTest.NoHitNoise{On,Off}ReturnsInfinity` |
| Quirk | `f64::round` half-away-from-zero in scheduler period and collision edge cells | `RateSchedulerTest.RoundsToNearestPeriod`, `collision::edge_cells_round_half_away` |
| Quirk | `Mul` does not wrap theta; motion wraps once after integration | `IdealMotionModelTest.ThetaNormalized`, `pose::mul_does_not_wrap` |
| Quirk | Collision keeps truth, advances odom; OOB footprint = collision; 3×3 neighbourhood | `SimulationEngineTest.CollisionHoldsTruePoseButOdomKeepsAdvancing`, `CollisionCheckerTest.OutOfBoundsCollision` |
| Quirk | `freq <= 0` fires every tick in both modes; rate > sim rate clamps | `RateSchedulerTest.{DefaultZeroFreqFiresEveryTick, FreqExceedsSimRateClamped}`, `AccumulatorModeTest.*` |
| Quirk | Sensors treat unknown cells as free, collision as occupied | `grid::unknown_policy_is_explicit` |

## Milestones

Each = own spec → plan → implement cycle; independently testable.

**M0 — workspace, pose/footprint/grid, config, map loading**
`error.rs`, `pose.rs`, `footprint.rs`, `grid.rs` (incl. `OccupancyGrid::new` + invariant tests), `map.rs` (metadata private,
`LoadMapMetadata.*` through `load_map`), `config.rs`, `config/yaml.rs`.
Deliverable: `cargo test -p stdr_core` loads every yaml in `stdr_resources/resources/robots` (include resolution via
`STDR_RESOURCES_DIR`, nested `noise: {filename:}`, default `frame_id`s, center-of-rotation-in-footprint validation;
`square_robot_rfid_reader.yaml` loads with a warning) and every `stdr_resources/maps/*.yaml` (dims, vertical flip,
thresholds, negate, PGM P2/P5).
Tests: `LoadRobotConfig.*`, `LoadMapMetadata.*`, `MapLoader.*`, `PointInFootprintTest.*`, `ComposeInverseTest.*`,
`BodyToPivotPoseTest.*`, `PivotToBodyPoseTest.*`, plus the config/map/grid/pose rows of the fix/quirk table.

**M1 — core sim**
`motion.rs`, `collision.rs`, `sensors.rs`, `scheduler.rs`, `engine.rs`, `examples/headless.rs --seed`.
Tests: `IdealMotionModel*`, `OmniMotionModel*`, `ApplyNoiseTest.*`, `OdometryVarianceTest.*`, `CollisionCheckerTest.*`
(single-pose cases via `path_collides(from == to)`; `ZeroResolution*`/`EmptyMap*` already in M0 `grid::new`),
`LaserSimulatorTest.*`, `SonarSimulatorTest.*`, `RateSchedulerTest.*`, `EffectiveRateTest.*`, `AccumulatorModeTest.*`,
`SchedulingModeStringTest.*`, `WorldModelTest.*` (robots/map only), `SimulationEngineTest.*` (minus the two Tf/Odom cases),
`StandaloneBackend.{Reset*, Teleport*, OdomPose*}` on `engine.reset/teleport`, `engine::sim_time_accumulates_across_step_dt_change`,
`engine::reset_zeroes_sim_time`, plus the motion/collision/sensor/scheduler/engine rows of the fix/quirk table.
Headless output diffable against a C++ noise-off run.

**M2 — Bevy app, no plotters**
`SimPlugin` (resources/commands/events/fixed step; `CatchUp` only), map sprite + revision-keyed texture, `overlay.rs`
(`Canvas`, `draw_*`, geometry) + `GizmoCanvas`, camera, `Selection`, teleport, `TeleopState`, `ui/teleop.rs` (+ `TeleopController.*`
unit tests), egui toolbar/robot info/messages, CLI.
Headless tests (`MinimalPlugins` + `TimeUpdateStrategy::ManualDuration`): N updates advance `engine.sim_time()` by
`ticks·step_dt`; pause freezes; resume does not replay; 2× speed doubles ticks/frame; `SetStepDt` changes timestep;
2 s manual delta capped to 0.25 s sim; reset restores spawn pose / zero twist / `data = None` / `sim_time == 0`; `FellBehind` fires once;
`pause_resume_via_command`; `teleop_stop_targets_previously_driven_robot`; `map_texture_rebuilds_only_on_revision_change`;
`selection_falls_back_to_first_robot`; `yaml_pose_used_when_flags_absent`; `toolbar_reads_speed_from_virtual_time`;
`overlay::draw_robot_records_expected_calls` (RecordingCanvas, no Bevy).

**M3 — plotter framework + 4 plotters**
`plot/mod.rs`, `plot/overlay_plot.rs` (`PlotCanvas`, `map_plot`, `draw_map`), `inventory` registry, `--plotter` filter,
Pose Error / Map Trace / Odometry Trace / Scan Trace.
Tests: `Trail.*`, `PlotHelpers.{ScanToMapPoints*, ShouldSample*}` (→ `overlay` / `SampleGate`),
`PlotPanel.{Filter*, DuplicateFilterNameYieldsOneSlot, EmptyFilterInstantiatesAll, UnknownFilterNameYieldsNoSlots}` (registry),
lifecycle as headless fixture-plotter tests (paused → sample system does not run; reset → `P::default()`;
`reset_defaults_plotter_state`), `teleport_detector_clears_on_jump`, `trail_eviction_reports_front`.
Retired C++ tests (no counterpart by design): `PlotSink.*`, `PlotView.*`, `SpscRing.*`, `LatestSnapshot.*`, `CommandQueue.*`,
`SimIntrospection.*`, `SimView.*` (covered by engine tests), `MapTransformTest.*` (camera does it), `PlotterRegistry.WholeArchive*`
(inventory in the bin crate), `PlotPanel.{Budget*, Exception*}`.

**M4 — 3D scene + camera sensor**
Per §M4: `CameraSpec` + `fired` in core, `scene3d` (meshes, scene root, mirrors, orbit camera, 2D/3D toggle),
camera sensors rendering to images shown in an egui window, `camera_robot.yaml`. Tests: the §M4 list. Perf check on `frieburg.png`.

## Crates (verify exact versions at `cargo init`)

| Crate | Purpose | Known-good at plan time (2026-10) |
|---|---|---|
| `bevy` | engine; 2D render + winit/X11 features (no audio, gamepad or native Wayland: those need ALSA/libudev/libwayland dev packages) | 0.19.1 stable (0.20 in rc) |
| `bevy_egui` | egui integration | 0.40–0.42 pair with Bevy 0.19 |
| `egui`, `egui_plot` | UI + plots; **egui version must match what bevy_egui pins** | egui_plot 0.37 latest |
| `inventory` | plotter self-registration | |
| `clap` (derive) | CLI | |
| `rfd` | native file dialogs | |
| `serde` (derive) | config structs | |
| `serde_yaml_ng` | YAML `Value` for include/merge + `from_value` (`serde_yaml` unmaintained) | |
| `image` (png, pnm) | map images | |
| `rand`, `rand_distr` | noise; `Normal` only constructed when `noise_std > 0` | |
| `thiserror` | core errors | |
| dev: `approx` | float asserts | |

## Risks / gotchas

- `Time<Virtual>::max_delta` clamps before speed multiplier → set `cap / speed`. Sim time is `engine.sim_time()`, never Bevy time.
- `Canvas` is f64 world coords; `GizmoCanvas` converts to f32 at the adapter, nowhere else.
- `step_dt = 0.001` at 5× = thousands of `FixedUpdate`s per frame; cap bounds it, expect `FellBehind`.
- Numeric parity: `as i32` truncates like `static_cast<int>`; `f64::round` = half-away-from-zero like `std::round`; keep sonar float-accumulation loop. Do not "fix" these (table above). f64 in core, f32 only at Bevy/egui boundary; `LaserScan.ranges: Vec<f32>` as C++.
- RNG differs from `mt19937` → golden tests noise-off or statistical.
- `Value`-level deep-merge means unknown YAML keys are silently ignored (as C++). Optionally `#[serde(deny_unknown_fields)]` on spec structs in M0 to catch typos.
- `Unknown::Transparent` for sensors / `Solid` for collision reproduces C++; changing either is a behaviour change, test-gated.
- egui_plot: unbounded scalar series at 60 Hz ≈ 216k points/hour; `SampleGate` (e.g. 50 ms) + `TimeSeries` cap for scalar plotters. Only render selected scan.
- `inventory`: entries must be `'static` fn items, plotters inside the bin crate.
- bevy_egui: UI systems only in its context-pass schedule; gate world input on `wants_pointer_input()` / `wants_keyboard_input()`; share textures via `EguiUserTextures`.
- Robot mirrors keyed by `RobotId`; diff `engine.robots()` vs `RobotEntities` each frame; never store `Entity` in core.
- bevy_egui's auto primary context goes to whichever new camera it sees first, which can be a 3D camera: the
  2D camera carries `PrimaryEguiContext` explicitly and auto-creation is off.
- A `Screenshot` of an image render target only captures on frames a camera renders it; sensor cameras render
  only on fired frames, so offscreen checks must screenshot on such a frame (the texture itself persists).

## Glossary

- **sim time** — engine-owned `elapsed` in seconds; advances by `step_dt` per tick; zeroed by `reset()`. Never Bevy time.
- **tick** — one `engine.step()`.
- **sensor index** — position in `config.sensors`; the scheduler key and the `data[i]` slot. Frame ids map to it via `sensor_index(frame_id)`.
- **map revision** — counter bumped on `set_map`; the texture cache key.
- **overlay** — robot/sensor drawing on top of the map, in world coordinates (`overlay.rs`).
- **Canvas** — the overlay's draw-target seam; adapters `GizmoCanvas` (2D view), `PlotCanvas` (egui_plot), `RecordingCanvas` (tests).
- **mirror** — a Bevy entity that reflects one engine robot for rendering; never the source of truth.

## Verification

Each milestone's test list above is its gate. From M2 on, also run:

```
cargo run -p stdr_app -- --map <stdr_resources/maps/sparse_obstacles.yaml> \
  --robot <stdr_resources/resources/robots/simple_robot_realistic.yaml> --x 3 --y 2 --theta 1.57
```

and drive with WASD/QE; laser rays, odom ghost drift, and plotter windows should match the C++ standalone.
