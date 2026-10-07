//! Bevy front end for `stdr_core`: 2D map view, egui panels, keyboard teleop. The binary in
//! `main.rs` wires these plugins together; the library exists so `tests/` can drive them headless.

pub mod cli;
pub mod overlay;
pub mod sim;
pub mod ui;
pub mod view2d;
