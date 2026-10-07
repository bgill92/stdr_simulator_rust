use std::path::PathBuf;

use clap::Parser;
use stdr_core::Pose2D;

/// STDR 2D robot simulator.
#[derive(Parser, Debug)]
#[command(version, allow_negative_numbers = true)]
pub struct Cli {
    /// Map yaml to load on startup.
    #[arg(long)]
    pub map: Option<PathBuf>,
    /// Robot yaml to spawn on startup.
    #[arg(long)]
    pub robot: Option<PathBuf>,
    /// Spawn x in metres [default: the robot yaml's initial_pose].
    #[arg(long)]
    pub x: Option<f64>,
    /// Spawn y in metres [default: the robot yaml's initial_pose].
    #[arg(long)]
    pub y: Option<f64>,
    /// Spawn heading in radians [default: the robot yaml's initial_pose].
    #[arg(long)]
    pub theta: Option<f64>,
    /// Open only this plotter, by key (repeatable) [default: all]. Keys: PoseError, MapTrace,
    /// OdometryTrace, ScanTrace.
    #[arg(long = "plotter", value_name = "KEY")]
    pub plotters: Vec<String>,
}

impl Cli {
    /// Each flag overrides its own field of the yaml `initial_pose` (C++ ignored the yaml pose).
    pub fn spawn_pose(&self, yaml: Pose2D) -> Pose2D {
        Pose2D {
            x: self.x.unwrap_or(yaml.x),
            y: self.y.unwrap_or(yaml.y),
            theta: self.theta.unwrap_or(yaml.theta),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const YAML: Pose2D = Pose2D {
        x: 1.0,
        y: 2.0,
        theta: 0.5,
    };

    #[test]
    fn yaml_pose_used_when_flags_absent() {
        let cli = Cli::parse_from(["stdr_app", "--robot", "r.yaml"]);
        assert_eq!(cli.spawn_pose(YAML), YAML);
    }

    #[test]
    fn flags_override_yaml_pose_per_field() {
        let cli = Cli::parse_from(["stdr_app", "--x", "-3", "--theta", "1.57"]);
        assert_eq!(
            cli.spawn_pose(YAML),
            Pose2D {
                x: -3.0,
                y: 2.0,
                theta: 1.57
            }
        );
    }
}
