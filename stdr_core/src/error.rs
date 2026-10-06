use std::path::PathBuf;

#[derive(Debug, thiserror::Error)]
pub enum CoreError {
    #[error("{}: {source}", path.display())]
    Io {
        path: PathBuf,
        source: std::io::Error,
    },
    #[error("{}: {source}", path.display())]
    Yaml {
        path: PathBuf,
        source: serde_yaml_ng::Error,
    },
    #[error("{}: {source}", path.display())]
    Image {
        path: PathBuf,
        source: image::ImageError,
    },
    /// Input parsed but violates an invariant (bad footprint, rotated map origin, empty grid, ...).
    #[error("{0}")]
    Invalid(String),
}
