//! C++ `LoadMapMetadata.*` (through `load_map`) and standalone `MapLoader.*`.
#![allow(non_snake_case)]

use std::path::{Path, PathBuf};

use stdr_core::{OccupancyGrid, Pose2D, load_map};

fn fixture(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures/maps")
        .join(name)
}

/// `$STDR_RESOURCES_DIR/../maps` (C++ layout), else the copy vendored in this repo.
fn shipped_maps_dir() -> PathBuf {
    std::env::var_os("STDR_RESOURCES_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|| {
            Path::new(env!("CARGO_MANIFEST_DIR")).join("../stdr_resources/resources")
        })
        .join("../maps")
}

fn load(name: &str) -> OccupancyGrid {
    load_map(fixture(name)).unwrap_or_else(|e| panic!("{name}: {e}"))
}

// tiny.pgm (2x2) top row: 255, 0; bottom row: 90 (occ 0.647), 190 (occ 0.255). Grid row 0 = image bottom.
mod LoadMapMetadata {
    use super::*;

    #[test]
    fn ValidMapReturnsMetadata() {
        let g = load("tiny.yaml");
        assert_eq!((g.width(), g.height()), (2, 2));
        assert_eq!(g.resolution(), 0.02);
        assert_eq!(
            g.origin(),
            Pose2D {
                x: 1.5,
                y: -2.0,
                theta: 0.0
            }
        );
        // occupied_thresh 0.6 / free_thresh 0.3: 0.647 → occupied, 0.255 → free.
        assert_eq!(g.data(), &[100, 0, 0, 100]);
    }

    #[test]
    fn ImagePathIsResolved() {
        // `image` is relative to the yaml's directory, not the working directory.
        assert_ne!(std::env::current_dir().unwrap(), fixture(""));
        load("tiny.yaml");
    }

    #[test]
    fn MissingFileReturnsError() {
        assert!(load_map("/nonexistent/path/map.yaml").is_err());
    }

    #[test]
    fn MissingImageKeyReturnsError() {
        let err = load_map(fixture("no_image.yaml")).unwrap_err().to_string();
        assert!(err.contains("image"), "{err}");
    }

    #[test]
    fn MissingResolutionKeyReturnsError() {
        let err = load_map(fixture("no_resolution.yaml"))
            .unwrap_err()
            .to_string();
        assert!(err.contains("resolution"), "{err}");
    }

    #[test]
    fn NegateOneIsTrue() {
        // Negated occupancy: 255 → 1.0, 0 → 0.0, 90 → 0.353, 190 → 0.745 (default thresholds).
        assert_eq!(load("tiny_negate.yaml").data(), &[-1, 100, 100, 0]);
    }

    #[test]
    fn DefaultThresholdsWhenAbsent() {
        // 0.65 / 0.196: both bottom-row pixels fall between the thresholds.
        assert_eq!(load("tiny_defaults.yaml").data(), &[-1, -1, 0, 100]);
    }
}

mod MapLoader {
    use super::*;

    fn maze1() -> OccupancyGrid {
        load_map(shipped_maps_dir().join("maze1.yaml")).unwrap()
    }

    #[test]
    fn FailsOnMissingFile() {
        assert!(load_map("/nonexistent/map.yaml").is_err());
    }

    #[test]
    fn LoadsMaze1Successfully() {
        let g = maze1();
        assert!(g.width() > 0 && g.height() > 0);
        assert_eq!(g.resolution(), 0.003);
        assert_eq!(g.data().len(), (g.width() * g.height()) as usize);
    }

    #[test]
    fn OccupancyValuesAreInRange() {
        assert!(maze1().data().iter().all(|v| [0, 100, -1].contains(v)));
    }

    #[test]
    fn OriginMatchesYaml() {
        assert_eq!(maze1().origin(), Pose2D::default());
    }

    // p5_4x4.pgm rows top to bottom: white, black, gray (128), white.
    #[test]
    fn LoadsPgmMap() {
        let g = load("p5_4x4.yaml");
        assert_eq!((g.width(), g.height()), (4, 4));
        assert_eq!(g.data().len(), 16);
    }

    #[test]
    fn PixelConversionWhiteToFreeBlackToOccupied() {
        let g = load("p5_4x4.yaml");
        assert_eq!(g.at((0, 0)), Some(0));
        assert_eq!(g.at((0, 1)), Some(-1));
        assert_eq!(g.at((0, 2)), Some(100));
        assert_eq!(g.at((0, 3)), Some(0));
    }

    #[test]
    fn NegateFlipsMapping() {
        let g = load("p5_4x4_negate.yaml");
        assert_eq!(g.at((0, 0)), Some(100));
        assert_eq!(g.at((0, 2)), Some(0));
    }
}

mod map {
    use super::*;

    #[test]
    fn rotated_origin_rejected() {
        let err = load_map(fixture("tiny_rotated.yaml"))
            .unwrap_err()
            .to_string();
        assert!(err.contains("origin"), "{err}");
    }

    #[test]
    fn pgm_maxval_is_rescaled() {
        // P2 with maxval 15: 15 is white (free), 0 black (occupied), 7 mid-gray (unknown).
        assert_eq!(load("maxval15.yaml").data(), &[0, 100, -1]);
    }

    /// Per shipped map: (name, free, occupied, unknown, sum of index * value). Generated from the C++
    /// loader's pipeline (libpng gray transform, flip, thresholds), so this pins cell-for-cell parity.
    const CPP_GRIDS: [(&str, usize, usize, usize, i64); 8] = [
        ("frieburg", 271585, 16578, 609837, 555581018502),
        ("hospital_section", 334302, 74481, 72315, 2239113485685),
        ("maze1", 647087, 35189, 0, 1246399161300),
        ("mines", 120713, 47712, 569575, 1491315763162),
        ("robocup", 217892, 47482, 295227, 1334878992853),
        ("simple_rooms", 83184, 36816, 0, 219062534100),
        ("simple_rooms_no_walls", 106084, 13912, 4, 83614011312),
        ("sparse_obstacles", 436909, 80090, 61151, 2324239860511),
    ];

    #[test]
    fn every_shipped_map_loads() {
        let mut yamls: Vec<_> = std::fs::read_dir(shipped_maps_dir())
            .unwrap()
            .map(|e| e.unwrap().path())
            .filter(|p| p.extension().is_some_and(|e| e == "yaml"))
            .collect();
        yamls.sort();
        assert_eq!(yamls.len(), CPP_GRIDS.len());
        for (yaml, (name, free, occupied, unknown, weighted)) in yamls.iter().zip(CPP_GRIDS) {
            assert_eq!(yaml.file_stem().unwrap(), name);
            let g = load_map(yaml).unwrap_or_else(|e| panic!("{e}"));
            assert_eq!(
                (g.width(), g.height()),
                image::image_dimensions(yaml.with_extension("png")).unwrap(),
                "{name}"
            );
            let count = |v: i8| g.data().iter().filter(|&&c| c == v).count();
            assert_eq!(
                (count(0), count(100), count(-1)),
                (free, occupied, unknown),
                "{name}"
            );
            let sum: i64 = g
                .data()
                .iter()
                .enumerate()
                .map(|(i, &v)| i as i64 * i64::from(v))
                .sum();
            assert_eq!(sum, weighted, "{name}");
        }
    }
}
