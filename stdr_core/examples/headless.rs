//! Runs one robot for N ticks at a constant command and prints its state every tick, in a plain
//! text format meant for `diff` against the C++ engine driven the same way.
//!
//! ```text
//! cargo run -p stdr_core --example headless -- --map M.yaml --robot R.yaml [--x X --y Y --theta T]
//!     [--ticks 1000] [--step-dt 0.01] [--vx 0.3] [--vy 0] [--wz 0.2] [--seed S] [--noise-off]
//! ```
//!
//! Per tick: `tick sim_time x y theta odom_x odom_y odom_theta collided`, then one indented line
//! per sensor, sorted by frame id: `frame_id none` before its first reading, else its ranges.
//! `--noise-off` forces perfect odometry and zero sensor noise; only that mode is comparable with
//! C++ (the RNGs differ). Includes resolve against `$STDR_RESOURCES_DIR`, else the robot's dir.

use std::collections::HashMap;
use std::path::Path;
use std::process::ExitCode;

use stdr_core::{
    Measurement, OdometryModel, Pose2D, SimulationEngine, Twist2D, load_map, load_robot_config,
};

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("headless: {e}");
            ExitCode::FAILURE
        }
    }
}

fn run() -> Result<(), String> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let noise_off = args.iter().any(|a| a == "--noise-off");
    let mut opts = HashMap::new();
    let mut it = args.iter().filter(|a| *a != "--noise-off");
    while let Some(key) = it.next() {
        let value = it.next().ok_or(format!("{key} needs a value"))?;
        opts.insert(key.trim_start_matches("--"), value.as_str());
    }
    let num = |key: &str, default: f64| -> Result<f64, String> {
        opts.get(key).map_or(Ok(default), |v| {
            v.parse().map_err(|e| format!("--{key}: {e}"))
        })
    };
    let path = |key: &str| opts.get(key).ok_or(format!("--{key} is required"));

    let robot = Path::new(path("robot")?);
    let base_dir: std::path::PathBuf = std::env::var_os("STDR_RESOURCES_DIR").map_or_else(
        || robot.parent().unwrap_or(Path::new(".")).into(),
        Into::into,
    );
    let (mut cfg, warnings) = load_robot_config(robot, &base_dir).map_err(|e| e.to_string())?;
    for w in warnings {
        eprintln!("warning: {w}");
    }
    if noise_off {
        cfg.kinematic.odometry = OdometryModel::Perfect;
        for s in &mut cfg.sensors {
            s.common.noise_std = 0.0;
        }
    }
    let pose = Pose2D {
        x: num("x", cfg.initial_pose.x)?,
        y: num("y", cfg.initial_pose.y)?,
        theta: num("theta", cfg.initial_pose.theta)?,
    };
    let seed = opts
        .get("seed")
        .map(|s| s.parse().map_err(|e| format!("--seed: {e}")))
        .transpose()?;

    let mut engine =
        SimulationEngine::new(num("step-dt", 0.01)?, seed).map_err(|e| e.to_string())?;
    engine.set_map(load_map(path("map")?).map_err(|e| e.to_string())?);
    let id = engine.spawn(cfg, pose);
    engine.set_cmd_vel(
        id,
        Twist2D {
            linear_x: num("vx", 0.3)?,
            linear_y: num("vy", 0.0)?,
            angular_z: num("wz", 0.2)?,
        },
    );

    for _ in 0..num("ticks", 1000.0)? as u64 {
        engine.step();
        let r = engine.robot(id).expect("spawned above");
        let (p, o) = (r.state.pose, r.state.odom_pose);
        println!(
            "{} {:.6} {:.9} {:.9} {:.9} {:.9} {:.9} {:.9} {}",
            engine.ticks(),
            engine.sim_time(),
            p.x,
            p.y,
            p.theta,
            o.x,
            o.y,
            o.theta,
            u8::from(r.collided)
        );
        let mut lines: Vec<String> = r
            .config
            .sensors
            .iter()
            .zip(&r.data)
            .map(|(s, d)| {
                let values = match d {
                    None => "none".to_owned(),
                    Some(Measurement::Laser(scan)) => scan
                        .ranges
                        .iter()
                        .map(|v| format!("{v:.6}"))
                        .collect::<Vec<_>>()
                        .join(" "),
                    Some(Measurement::Sonar(scan)) => format!("{:.9}", scan.range),
                };
                format!("  {} {values}", s.common.frame_id)
            })
            .collect();
        lines.sort();
        for line in lines {
            println!("{line}");
        }
    }
    Ok(())
}
