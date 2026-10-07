use std::path::Path;

use serde::Deserialize;

use crate::error::CoreError;
use crate::grid::OccupancyGrid;
use crate::pose::Pose2D;

/// ROS map_server yaml. Private: `load_map` is the whole interface.
#[derive(Deserialize)]
struct MapMetadata {
    image: String,
    resolution: f64,
    #[serde(default)]
    origin: Vec<f64>,
    #[serde(default = "default_occupied_thresh")]
    occupied_thresh: f64,
    #[serde(default = "default_free_thresh")]
    free_thresh: f64,
    #[serde(default)]
    negate: i64,
}

fn default_occupied_thresh() -> f64 {
    0.65
}

fn default_free_thresh() -> f64 {
    0.196
}

/// Loads a map yaml and its PNG/PGM image (`image` is relative to the yaml's directory).
/// Image row 0 is the top, grid row 0 the bottom, so rows are flipped. White = free, black = occupied
/// (inverted by `negate`); between the thresholds = unknown (-1).
pub fn load_map(yaml: impl AsRef<Path>) -> Result<OccupancyGrid, CoreError> {
    let yaml = yaml.as_ref();
    let text = std::fs::read_to_string(yaml).map_err(|source| CoreError::Io {
        path: yaml.into(),
        source,
    })?;
    let meta: MapMetadata = serde_yaml_ng::from_str(&text).map_err(|source| CoreError::Yaml {
        path: yaml.into(),
        source,
    })?;

    // C++ only reads origin when it has at least x and y.
    let origin = match meta.origin[..] {
        [x, y] => Pose2D { x, y, theta: 0.0 },
        [x, y, theta, ..] => Pose2D { x, y, theta },
        _ => Pose2D::default(),
    };
    if origin.theta != 0.0 {
        return Err(CoreError::Invalid(format!(
            "{}: rotated map origin (theta = {}) is not supported",
            yaml.display(),
            origin.theta
        )));
    }

    let image_path = yaml.parent().unwrap_or(Path::new("")).join(&meta.image);
    let image = image::open(&image_path)
        .map_err(|source| CoreError::Image {
            path: image_path.clone(),
            source,
        })?
        .into_luma8();
    let (width, height) = image.dimensions();

    let data = image
        .rows()
        .rev()
        .flatten()
        .map(|px| {
            let mut occ = (255.0 - f64::from(px.0[0])) / 255.0;
            if meta.negate != 0 {
                occ = 1.0 - occ;
            }
            if occ > meta.occupied_thresh {
                100
            } else if occ < meta.free_thresh {
                0
            } else {
                -1
            }
        })
        .collect();
    OccupancyGrid::new(width, height, meta.resolution, origin, data)
        .map_err(|e| CoreError::Invalid(format!("{}: {e}", yaml.display())))
}
