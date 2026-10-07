//! Sample buffers shared by the 2D view and the plotters. No bevy/egui types.

use std::collections::VecDeque;

use stdr_core::{Point2D, Pose2D};

/// Where a trail sample sits on the map.
pub trait Positioned {
    fn position(&self) -> Point2D;
}

impl Positioned for Pose2D {
    fn position(&self) -> Point2D {
        Point2D {
            x: self.x,
            y: self.y,
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Push {
    Skipped,
    Pushed { evicted_front: bool },
}

/// The last `cap` samples at least `spacing` apart, with their positions kept contiguous for
/// drawing.
#[derive(Clone, Debug)]
pub struct Trail<T> {
    buf: VecDeque<T>,
    xy: Vec<[f64; 2]>,
    cap: usize,
    spacing: f64,
}

impl<T: Positioned> Trail<T> {
    pub fn new(cap: usize, spacing: f64) -> Self {
        Self {
            buf: VecDeque::with_capacity(cap),
            xy: Vec::with_capacity(cap),
            cap,
            spacing,
        }
    }

    /// Appends `t` unless it is within `spacing` of the last sample; drops the oldest sample
    /// past `cap`.
    pub fn push_if_moved(&mut self, t: T) -> Push {
        let p = t.position();
        if let Some(&[x, y]) = self.xy.last()
            && (p.x - x).hypot(p.y - y) < self.spacing
        {
            return Push::Skipped;
        }
        self.buf.push_back(t);
        self.xy.push([p.x, p.y]);
        let evicted_front = self.buf.len() > self.cap;
        if evicted_front {
            self.buf.pop_front();
            self.xy.remove(0);
        }
        Push::Pushed { evicted_front }
    }

    pub fn clear(&mut self) {
        self.buf.clear();
        self.xy.clear();
    }

    pub fn xy(&self) -> &[[f64; 2]] {
        &self.xy
    }

    pub fn get(&self, i: usize) -> Option<&T> {
        self.buf.get(i)
    }

    pub fn len(&self) -> usize {
        self.buf.len()
    }

    pub fn is_empty(&self) -> bool {
        self.buf.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn at(x: f64) -> Pose2D {
        Pose2D {
            x,
            ..Default::default()
        }
    }

    #[test]
    fn trail_eviction_reports_front() {
        let mut t = Trail::new(2, 0.1);
        assert_eq!(
            t.push_if_moved(at(0.0)),
            Push::Pushed {
                evicted_front: false
            }
        );
        assert_eq!(t.push_if_moved(at(0.05)), Push::Skipped);
        assert_eq!(
            t.push_if_moved(at(0.1)),
            Push::Pushed {
                evicted_front: false
            }
        );
        assert_eq!(
            t.push_if_moved(at(0.2)),
            Push::Pushed {
                evicted_front: true
            }
        );
        assert_eq!(t.xy(), [[0.1, 0.0], [0.2, 0.0]]);
        assert_eq!(t.get(0), Some(&at(0.1)));
        t.clear();
        assert!(t.is_empty() && t.xy().is_empty());
    }
}
