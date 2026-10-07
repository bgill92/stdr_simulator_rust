//! The compiled-in plotters. Each registers itself with `register_plotter!`; `--plotter` picks
//! which ones `plot::add_plotters` builds.

mod map_trace;
mod odometry_trace;
mod pose_error;
mod scan_trace;
