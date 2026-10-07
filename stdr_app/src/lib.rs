//! Bevy front end for `stdr_core`: 2D map view, 3D scene with camera sensors, egui panels,
//! keyboard teleop, plotters. The binary
//! in `main.rs` wires these plugins together; the library exists so `tests/` can drive them
//! headless.

pub mod cli;
pub mod overlay;
pub mod plot;
pub mod plotters;
pub mod scene3d;
pub mod sim;
pub mod ui;
pub mod view2d;
