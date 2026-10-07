# stdr_simulator_rust

A Rust port of the [STDR Simulator](https://github.com/stdr-simulator-ros-pkg/stdr_simulator),
a 2D multi-robot simulator. The port targets a [Bevy](https://bevyengine.org/) front end;
the design and milestone plan live in [PLAN.md](PLAN.md).

## Status

- **M0 (done):** `stdr_core` crate with the workspace, `Pose2D`, `Footprint`, `OccupancyGrid`,
  map loading (YAML + PNG/PGM), and robot config loading (YAML with includes and noise files).
- **M1 (done):** core simulation in `stdr_core`: kinematics and odometry noise, swept-footprint
  collision, laser and sonar, per-sensor rate scheduling, and `SimulationEngine` (robots, map,
  sim time, one seedable RNG), plus the `headless` example.
- **M2 (done):** `stdr_app`, the Bevy app: 2D map view with pan/zoom, robot, sensor and trail overlay,
  click-to-select, right-click teleport/delete, WASD/QE keyboard teleop, and egui toolbar,
  robot info and message panels. The simulation runs in Bevy's fixed timestep, starts paused,
  and caps catch-up at 0.25 s of sim time per frame.
- **M3 (done):** plotter windows over the map and sim time: Pose Error, Map Trace, Odometry
  Trace and Scan Trace, compiled in and selected with `--plotter`.
- **M4 (done):** 3D view of the extruded map and robots (toolbar 2D/3D toggle; left-drag orbits,
  middle-drag pans, wheel zooms) and a camera sensor: the core schedules it, the app renders each
  camera into an image shown in the Cameras window.

## Build and test

```sh
cargo build --workspace
cargo test --workspace
```

Run the simulation without a UI, printing every tick's poses and sensor readings:

```sh
STDR_RESOURCES_DIR=stdr_resources/resources cargo run -p stdr_core --example headless -- \
  --map stdr_resources/maps/sparse_obstacles.yaml \
  --robot stdr_resources/resources/robots/simple_robot.yaml --x 3 --y 2 --theta 1.57 --noise-off
```

With `--noise-off` the output is meant to be diffed line for line against the C++ engine run
the same way; `--seed N` makes noisy runs reproducible. See the example's header for all flags.

Run the app (opens a window; press Start, then drive the selected robot with W/S, A/D (omni
strafe), Q/E):

```sh
STDR_RESOURCES_DIR=stdr_resources/resources cargo run -p stdr_app -- \
  --map stdr_resources/maps/sparse_obstacles.yaml \
  --robot stdr_resources/resources/robots/simple_robot_realistic.yaml --x 3 --y 2 --theta 1.57
```

Every plotter opens by default; `--plotter KEY` (repeatable: `PoseError`, `MapTrace`,
`OdometryTrace`, `ScanTrace`) opens only those, and the Plotters menu re-opens a closed one. Pose
Error drives the selected robot in a circle while it is open; held teleop keys override it, and
pausing or closing it stops the robot.

`--x/--y/--theta` override the robot yaml's `initial_pose` field by field; without them the yaml
pose is used. Robots loaded through File → Load Robot resolve includes the same way, so start the
app with `STDR_RESOURCES_DIR` set when loading the shipped robots. The app talks X11 (Wayland
desktops run it through XWayland), so building it needs no system dev packages.

CI runs `cargo fmt --check`, `cargo clippy`, and `cargo test` on pushes to `main` and on pull requests.

## Camera sensor

The original STDR has no camera, so its yaml schema is new to this port. It follows the other
sensor kinds (optional `filename` include, inline `camera_specifications` merged on top):

```yaml
- camera:
    camera_specifications:
      pose: {x: 0.15, y: 0, theta: 0}   # mount pose in the robot frame
      frequency: 10                     # Hz; 0 = every sim tick
      frame_id: front_camera            # default camera_<n>
      width: 320                        # image size in pixels
      height: 240
      fov: 1.0472                       # horizontal field of view, rad, in (0, pi)
      near: 0.05                        # clip planes, m, 0 < near < far
      far: 50
```

Omitted fields default to the values above (frame id `camera_<n>`, frequency 0). The lens sits
just above the 0.2 m robot body; walls are 1 m tall. Cameras render only on frames where they
fired, so a paused simulation keeps the last image. Try it with
`--robot stdr_resources/resources/robots/camera_robot.yaml`.

## Test resources

The tests load the original STDR robot and map definitions. By default they use the copy
vendored in [`stdr_resources/`](stdr_resources/) (`resources/` for robots, sensors, kinematic
models and noise files; `maps/` for maps). Set `STDR_RESOURCES_DIR` to another
`stdr_resources/resources` directory (e.g. from a checkout of the C++ project) to use that
instead; maps are then read from its sibling `maps/` directory.

## License

GPL-3.0-only, matching the original STDR Simulator. See [LICENSE](LICENSE).
