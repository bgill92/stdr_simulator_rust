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
- **M2–M4 (not started):** Bevy app, plotters, 3D scene + camera sensor. See PLAN.md for details.

`stdr_app` is currently an empty placeholder crate.

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

CI runs `cargo fmt --check`, `cargo clippy`, and `cargo test` on pushes to `main` and on pull requests.

## Test resources

The tests load the original STDR robot and map definitions. By default they use the copy
vendored in [`stdr_resources/`](stdr_resources/) (`resources/` for robots, sensors, kinematic
models and noise files; `maps/` for maps). Set `STDR_RESOURCES_DIR` to another
`stdr_resources/resources` directory (e.g. from a checkout of the C++ project) to use that
instead; maps are then read from its sibling `maps/` directory.

## License

GPL-3.0-only, matching the original STDR Simulator. See [LICENSE](LICENSE).
