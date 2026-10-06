//! Robot yaml (C++ STDR schema) → `RobotConfig`. Includes are expanded and deep-merged at the
//! `Value` level (`yaml.rs`), then serde fills `#[serde(default)]` structs.

mod yaml;

use std::collections::HashMap;
use std::path::Path;

use serde::de::DeserializeOwned;
use serde::{Deserialize, Deserializer};
use serde_yaml_ng::Value;

use crate::error::CoreError;
use crate::footprint::Footprint;
use crate::pose::{Point2D, Pose2D};

#[derive(Clone, Debug, Default, PartialEq)]
pub struct RobotConfig {
    pub initial_pose: Pose2D,
    pub footprint: Footprint,
    /// Pivot in the body frame; must lie inside `footprint`.
    pub center_of_rotation: Point2D,
    pub kinematic: KinematicConfig,
    /// In yaml order; a sensor's position here is its sensor index.
    pub sensors: Vec<Sensor>,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug, Default, Deserialize)]
#[serde(try_from = "String")]
pub enum KinematicKind {
    #[default]
    Ideal,
    Omni,
}

impl TryFrom<String> for KinematicKind {
    type Error = String;

    /// `""` means ideal (C++ parity); anything unknown is rejected here, at load.
    fn try_from(s: String) -> Result<Self, String> {
        match s.as_str() {
            "" | "ideal" => Ok(Self::Ideal),
            "omni" => Ok(Self::Omni),
            _ => Err(format!(
                "unknown kinematic_model '{s}'; allowed values: ideal, omni"
            )),
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug, Default, Deserialize)]
#[serde(try_from = "String")]
pub enum OdometryModel {
    /// Odometry equals truth; alphas ignored.
    #[default]
    Perfect,
    /// Alpha noise perturbs truth; odometry integrates the clean command.
    Velocity,
}

impl TryFrom<String> for OdometryModel {
    type Error = String;

    fn try_from(s: String) -> Result<Self, String> {
        match s.as_str() {
            "perfect" => Ok(Self::Perfect),
            "velocity" => Ok(Self::Velocity),
            _ => Err(format!(
                "unknown odometry_model '{s}'; allowed values: perfect, velocity"
            )),
        }
    }
}

/// Velocity-model noise coefficients: rows Ux, Uy, W, G; columns ux², uy², w².
#[derive(Clone, Copy, PartialEq, Debug, Default, Deserialize)]
#[serde(from = "AlphasYaml")]
pub struct Alphas(pub [[f64; 3]; 4]);

/// The only place the yaml alpha names exist.
#[derive(Deserialize, Default)]
#[serde(default)]
struct AlphasYaml {
    a_ux_ux: f64,
    a_ux_uy: f64,
    a_ux_w: f64,
    a_uy_ux: f64,
    a_uy_uy: f64,
    a_uy_w: f64,
    a_w_ux: f64,
    a_w_uy: f64,
    a_w_w: f64,
    a_g_ux: f64,
    a_g_uy: f64,
    a_g_w: f64,
}

impl From<AlphasYaml> for Alphas {
    fn from(a: AlphasYaml) -> Self {
        Alphas([
            [a.a_ux_ux, a.a_ux_uy, a.a_ux_w],
            [a.a_uy_ux, a.a_uy_uy, a.a_uy_w],
            [a.a_w_ux, a.a_w_uy, a.a_w_w],
            [a.a_g_ux, a.a_g_uy, a.a_g_w],
        ])
    }
}

#[derive(Clone, Copy, PartialEq, Debug, Default, Deserialize)]
#[serde(default)]
pub struct KinematicConfig {
    #[serde(rename = "kinematic_model")]
    pub kind: KinematicKind,
    #[serde(rename = "odometry_model")]
    pub odometry: OdometryModel,
    #[serde(rename = "kinematic_parameters")]
    pub alphas: Alphas,
}

/// Fields every sensor kind shares, read from the same yaml mapping as its kind-specific spec.
#[derive(Clone, PartialEq, Debug, Default, Deserialize)]
#[serde(default)]
pub struct SensorCommon {
    /// Mounting pose in the robot body frame.
    pub pose: Pose2D,
    pub frequency: f64,
    /// Defaults to `<kind>_<n>`, n counting every sensor of that kind (named or not) in yaml order.
    pub frame_id: String,
    /// Gaussian range noise std-dev from `noise.noise_specifications.noise_std`; 0 = off.
    /// `noise_mean` is not read (C++ never used it).
    #[serde(rename = "noise", deserialize_with = "noise_std")]
    pub noise_std: f64,
}

fn noise_std<'de, D: Deserializer<'de>>(d: D) -> Result<f64, D::Error> {
    #[derive(Deserialize, Default)]
    #[serde(default)]
    struct Noise {
        noise_specifications: Specs,
    }
    #[derive(Deserialize, Default)]
    #[serde(default)]
    struct Specs {
        noise_std: f64,
    }
    Ok(Noise::deserialize(d)?.noise_specifications.noise_std)
}

#[derive(Clone, Copy, PartialEq, Debug, Default, Deserialize)]
#[serde(default)]
pub struct LaserSpec {
    pub min_angle: f64,
    pub max_angle: f64,
    pub min_range: f64,
    pub max_range: f64,
    pub num_rays: i32,
}

#[derive(Clone, Copy, PartialEq, Debug, Default, Deserialize)]
#[serde(default)]
pub struct SonarSpec {
    pub min_range: f64,
    pub max_range: f64,
    pub cone_angle: f64,
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum SensorConfig {
    Laser(LaserSpec),
    Sonar(SonarSpec),
}

impl SensorConfig {
    /// The yaml key, also the default `frame_id` prefix.
    pub fn name(&self) -> &'static str {
        match self {
            SensorConfig::Laser(_) => "laser",
            SensorConfig::Sonar(_) => "sonar",
        }
    }
}

#[derive(Clone, PartialEq, Debug)]
pub struct Sensor {
    pub common: SensorCommon,
    pub kind: SensorConfig,
}

type SpecParser = fn(Value) -> Result<SensorConfig, serde_yaml_ng::Error>;

/// The loader's only per-kind fan-out: a new sensor kind is one entry here plus its spec struct.
const SENSOR_KINDS: [(&str, SpecParser); 2] = [
    ("laser", |v| {
        serde_yaml_ng::from_value(v).map(SensorConfig::Laser)
    }),
    ("sonar", |v| {
        serde_yaml_ng::from_value(v).map(SensorConfig::Sonar)
    }),
];

/// Valid STDR kinds with no simulator yet: skipped with a warning so every shipped robot still loads.
const UNSUPPORTED_KINDS: [&str; 4] = [
    "rfid_reader",
    "co2_sensor",
    "thermal_sensor",
    "sound_sensor",
];

/// Loads a robot yaml. Every include `filename` resolves against `base_dir` (callers pass
/// `$STDR_RESOURCES_DIR`, else the robot yaml's directory). Returns the config plus load warnings.
pub fn load_robot_config(
    path: impl AsRef<Path>,
    base_dir: impl AsRef<Path>,
) -> Result<(RobotConfig, Vec<String>), CoreError> {
    let (path, base_dir) = (path.as_ref(), base_dir.as_ref());
    let text = std::fs::read_to_string(path).map_err(|source| CoreError::Io {
        path: path.into(),
        source,
    })?;
    let root: Value = serde_yaml_ng::from_str(&text).map_err(|source| CoreError::Yaml {
        path: path.into(),
        source,
    })?;
    let Some(Value::Sequence(items)) = root
        .get("robot")
        .and_then(|r| r.get("robot_specifications"))
    else {
        return Err(CoreError::Invalid(format!(
            "{}: missing robot.robot_specifications sequence",
            path.display()
        )));
    };

    let mut cfg = RobotConfig::default();
    let mut warnings = Vec::new();
    for (key, node) in items.iter().filter_map(Value::as_mapping).flatten() {
        let key = key.as_str().unwrap_or_default();
        let node = node.clone();
        match key {
            "initial_pose" => cfg.initial_pose = parse(node, path)?,
            "center_of_rotation" => cfg.center_of_rotation = parse(node, path)?,
            "kinematic" => cfg.kinematic = parse(yaml::specifications(node, key, base_dir)?, path)?,
            "footprint" => {
                let fp: FootprintYaml = parse(yaml::specifications(node, key, base_dir)?, path)?;
                cfg.footprint = fp.into_footprint(path, &mut warnings);
            }
            _ => {
                if let Some((_, spec_parser)) = SENSOR_KINDS.iter().find(|(k, _)| *k == key) {
                    let spec = yaml::specifications(node, key, base_dir)?;
                    let common = parse(spec.clone(), path)?;
                    let kind = spec_parser(spec).map_err(|source| CoreError::Yaml {
                        path: path.into(),
                        source,
                    })?;
                    cfg.sensors.push(Sensor { common, kind });
                } else if UNSUPPORTED_KINDS.contains(&key) {
                    warnings.push(format!(
                        "{}: '{key}' sensors are not simulated yet; skipped",
                        path.display()
                    ));
                } else {
                    warnings.push(format!(
                        "{}: unknown robot_specifications entry '{key}' ignored",
                        path.display()
                    ));
                }
            }
        }
    }

    if !cfg.footprint.contains(cfg.center_of_rotation) {
        let Point2D { x, y } = cfg.center_of_rotation;
        return Err(CoreError::Invalid(format!(
            "{}: center_of_rotation ({x}, {y}) is outside the robot footprint",
            path.display()
        )));
    }

    let mut counters: HashMap<&str, usize> = HashMap::new();
    for sensor in &mut cfg.sensors {
        let name = sensor.kind.name();
        let n = counters.entry(name).or_default();
        if sensor.common.frame_id.is_empty() {
            sensor.common.frame_id = format!("{name}_{n}");
        }
        *n += 1;
    }

    Ok((cfg, warnings))
}

fn parse<T: DeserializeOwned>(v: Value, path: &Path) -> Result<T, CoreError> {
    serde_yaml_ng::from_value(v).map_err(|source| CoreError::Yaml {
        path: path.into(),
        source,
    })
}

#[derive(Deserialize, Default)]
#[serde(default)]
struct FootprintYaml {
    radius: Option<f64>,
    points: Vec<PointEntry>,
}

/// Both `- point: {x, y}` and bare `- {x, y}` are accepted (C++ parity).
#[derive(Deserialize)]
#[serde(untagged)]
enum PointEntry {
    Wrapped { point: Point2D },
    Bare(Point2D),
}

impl FootprintYaml {
    fn into_footprint(self, path: &Path, warnings: &mut Vec<String>) -> Footprint {
        if self.points.is_empty() {
            return Footprint::Circle {
                radius: self.radius.unwrap_or(0.0),
            };
        }
        if self.radius.is_some() {
            // C++ used the points whenever any were given; kept so e.g. square_robot_rfid_reader.yaml loads.
            warnings.push(format!(
                "{}: footprint has both radius and points; using points",
                path.display()
            ));
        }
        Footprint::Polygon(
            self.points
                .into_iter()
                .map(|p| match p {
                    PointEntry::Wrapped { point } | PointEntry::Bare(point) => point,
                })
                .collect(),
        )
    }
}
