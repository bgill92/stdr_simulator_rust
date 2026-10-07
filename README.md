# stdr_simulator_rust

A Rust port of the [STDR Simulator](https://github.com/stdr-simulator-ros-pkg/stdr_simulator),
a 2D multi-robot simulator. The port targets a [Bevy](https://bevyengine.org/) front end;
the design and milestone plan live in [PLAN.md](PLAN.md).

## Status

- **M0 (done):** `stdr_core` crate with the workspace, `Pose`, footprint, `OccupancyGrid`,
  map loading (YAML + PGM), and robot config loading (YAML with includes and noise files).
- **M1–M4 (not started):** core simulation (motion, collision, sensors, scheduler), Bevy app,
  plotters, 3D scene + camera sensor. See PLAN.md for details.

`stdr_app` is currently an empty placeholder crate.

## Build and test

```sh
cargo build --workspace
cargo test --workspace
```

CI runs `cargo fmt --check`, `cargo clippy`, and `cargo test` on pushes to `main` and on pull requests.

## Test resources

The tests load the original STDR robot and map definitions. By default they use the copy
vendored in [`stdr_resources/`](stdr_resources/) (`resources/` for robots, sensors, kinematic
models and noise files; `maps/` for maps). Set `STDR_RESOURCES_DIR` to another
`stdr_resources/resources` directory (e.g. from a checkout of the C++ project) to use that
instead; maps are then read from its sibling `maps/` directory.

## License

GPL-3.0-only, matching the original STDR Simulator. See [LICENSE](LICENSE).
