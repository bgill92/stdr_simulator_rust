//! The plotter host and the sample buffers plotters (and the 2D view) share.
//!
//! A plotter is a `Resource` state type `P: Default` plus a sample system (Update, skipped while
//! paused or removed) and a render system (egui pass, skipped while removed), wired by
//! [`add_plotter`] and self-registered with [`register_plotter!`]. A `SimEvent::Reset` puts the
//! state back to `P::default()`.

pub mod overlay_plot;

use std::collections::VecDeque;
use std::marker::PhantomData;

use bevy::ecs::component::Mutable;
use bevy::ecs::system::ScheduleSystem;
use bevy::prelude::*;
use bevy_egui::{EguiPrimaryContextPass, egui};
use stdr_core::{Point2D, Pose2D};

use crate::sim::{SimEvent, apply_sim_commands};

/// Pause keeps the state but skips sampling; a removed plotter neither samples nor renders until
/// it is shown again, and then starts from `P::default()`.
#[derive(Resource)]
pub struct PlotterCtl<P> {
    pub name: &'static str,
    pub paused: bool,
    pub removed: bool,
    /// Build order, for cascading the windows' first positions.
    slot: usize,
    _p: PhantomData<fn() -> P>,
}

impl<P> PlotterCtl<P> {
    pub fn new(name: &'static str, slot: usize) -> Self {
        Self {
            name,
            paused: false,
            removed: false,
            slot,
            _p: PhantomData,
        }
    }
}

pub fn plotter_active<P: Send + Sync + 'static>(c: Res<PlotterCtl<P>>) -> bool {
    !c.paused && !c.removed
}

/// The plotter's window: a Pause checkbox above `body`; closing the window removes the plotter.
pub fn plotter_window<P>(
    ctx: &egui::Context,
    ctl: &mut PlotterCtl<P>,
    body: impl FnOnce(&mut egui::Ui),
) {
    let mut open = true;
    egui::Window::new(ctl.name)
        .open(&mut open)
        .default_pos([
            360.0 + 40.0 * ctl.slot as f32,
            60.0 + 40.0 * ctl.slot as f32,
        ])
        .default_width(420.0)
        .show(ctx, |ui| {
            ui.checkbox(&mut ctl.paused, "Pause");
            body(ui);
        });
    ctl.removed = !open;
}

/// The "Lock view to map" checkbox. A UI preference, so it lives in a render system's `Local`
/// and survives resets.
pub struct LockView(pub bool);

impl Default for LockView {
    fn default() -> Self {
        Self(true)
    }
}

/// Display names of every plotter added to the app, in build order (the toolbar's Plotters menu).
#[derive(Resource, Default)]
pub struct PlotterNames(pub Vec<&'static str>);

/// Re-open the named plotter after its window was closed.
#[derive(Message, Clone, Debug, PartialEq)]
pub struct ShowPlotter(pub &'static str);

/// Wires one plotter into the app.
pub fn add_plotter<P: Resource<Mutability = Mutable> + Default, M1, M2>(
    app: &mut App,
    name: &'static str,
    sample: impl IntoScheduleConfigs<ScheduleSystem, M1>,
    render: impl IntoScheduleConfigs<ScheduleSystem, M2>,
) {
    app.init_resource::<PlotterNames>();
    let slot = app.world().resource::<PlotterNames>().0.len();
    app.init_resource::<P>()
        .insert_resource(PlotterCtl::<P>::new(name, slot))
        .add_message::<ShowPlotter>()
        .add_systems(Update, sample.run_if(plotter_active::<P>))
        .add_systems(
            EguiPrimaryContextPass,
            render.run_if(|c: Res<PlotterCtl<P>>| !c.removed),
        )
        .add_systems(
            PreUpdate,
            (reset_on_event::<P>, show_on_request::<P>).after(apply_sim_commands),
        );
    app.world_mut().resource_mut::<PlotterNames>().0.push(name);
}

fn reset_on_event<P: Resource<Mutability = Mutable> + Default>(
    mut events: MessageReader<SimEvent>,
    mut p: ResMut<P>,
) {
    if events.read().any(|e| *e == SimEvent::Reset) {
        *p = P::default();
    }
}

fn show_on_request<P: Resource<Mutability = Mutable> + Default>(
    mut requests: MessageReader<ShowPlotter>,
    mut ctl: ResMut<PlotterCtl<P>>,
    mut p: ResMut<P>,
) {
    if requests.read().any(|r| r.0 == ctl.name) && ctl.removed {
        ctl.removed = false;
        *p = P::default();
    }
}

/// One compiled-in plotter. `key` is what `--plotter` matches.
pub struct PlotterEntry {
    pub key: &'static str,
    pub name: &'static str,
    pub description: &'static str,
    pub build: fn(&mut App),
}

inventory::collect!(PlotterEntry);

/// Registers a plotter: `register_plotter!(Key, NAME, "Description", |app| ...)`.
#[macro_export]
macro_rules! register_plotter {
    ($key:ident, $name:expr, $desc:literal, $build:expr) => {
        inventory::submit! {
            $crate::plot::PlotterEntry {
                key: stringify!($key),
                name: $name,
                description: $desc,
                build: $build,
            }
        }
    };
}

/// Every registered plotter, sorted by key so the build order does not depend on link order.
pub fn registry() -> Vec<&'static PlotterEntry> {
    let mut v: Vec<_> = inventory::iter::<PlotterEntry>.into_iter().collect();
    v.sort_by_key(|e| e.key);
    v
}

/// The entries `filter` names (each once, in filter order), or all of them for an empty filter;
/// plus the filter names that matched nothing.
pub fn select<'a>(
    entries: &[&'a PlotterEntry],
    filter: &[String],
) -> (Vec<&'a PlotterEntry>, Vec<String>) {
    if filter.is_empty() {
        return (entries.to_vec(), Vec::new());
    }
    let mut picked: Vec<&PlotterEntry> = Vec::new();
    let mut unknown = Vec::new();
    for name in filter {
        match entries.iter().find(|e| e.key == name) {
            Some(e) if !picked.iter().any(|p| p.key == e.key) => picked.push(e),
            Some(_) => {}
            None => unknown.push(name.clone()),
        }
    }
    (picked, unknown)
}

/// Builds the plotters `filter` selects (empty = all) and warns about unknown names.
pub fn add_plotters(app: &mut App, filter: &[String]) {
    // The toolbar's Plotters menu needs these even when nothing is selected.
    app.init_resource::<PlotterNames>()
        .add_message::<ShowPlotter>();
    let entries = registry();
    let (picked, unknown) = select(&entries, filter);
    for name in unknown {
        let keys: Vec<_> = entries.iter().map(|e| e.key).collect();
        warn!(
            "Unknown plotter \"{name}\" requested; available plotters: {}",
            keys.join(", ")
        );
    }
    for e in picked {
        (e.build)(app);
    }
}

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

/// Breadcrumb spacing for the plotter trails, metres (C++ `kMarkerSpacingMeters`).
pub const MARKER_SPACING: f64 = 0.1;
/// Breadcrumbs kept per plotter trail (C++ `kMaxMarkers`).
pub const MAX_MARKERS: usize = 500;

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

    pub fn iter(&self) -> impl Iterator<Item = &T> {
        self.buf.iter()
    }

    pub fn len(&self) -> usize {
        self.buf.len()
    }

    pub fn is_empty(&self) -> bool {
        self.buf.is_empty()
    }
}

/// Flags a jump between consecutive observations (a teleport or reset, not motion) so trails can
/// restart instead of drawing a line across the map.
#[derive(Clone, Debug)]
pub struct TeleportDetector {
    last: Option<Point2D>,
    threshold: f64,
}

impl Default for TeleportDetector {
    /// 1 m between frames is far beyond any drivable speed (C++ `kTeleportThresholdMeters`).
    fn default() -> Self {
        Self {
            last: None,
            threshold: 1.0,
        }
    }
}

impl TeleportDetector {
    /// True when `p` is more than the threshold away from the previous observation.
    pub fn observe(&mut self, p: Point2D) -> bool {
        let jumped = self
            .last
            .is_some_and(|l| (p.x - l.x).hypot(p.y - l.y) > self.threshold);
        self.last = Some(p);
        jumped
    }

    pub fn reset(&mut self) {
        self.last = None;
    }
}

/// `(t, v)` points for one plot line; the oldest drop past `cap`.
#[derive(Clone, Debug)]
pub struct TimeSeries {
    pts: VecDeque<[f64; 2]>,
    cap: usize,
}

impl Default for TimeSeries {
    /// 10 000 points: over 8 minutes of sim time at a 50 ms sample period.
    fn default() -> Self {
        Self::new(10_000)
    }
}

impl TimeSeries {
    pub fn new(cap: usize) -> Self {
        Self {
            pts: VecDeque::new(),
            cap,
        }
    }

    pub fn push(&mut self, t: f64, v: f64) {
        if self.pts.len() == self.cap {
            self.pts.pop_front();
        }
        self.pts.push_back([t, v]);
    }

    pub fn last(&self) -> Option<[f64; 2]> {
        self.pts.back().copied()
    }

    pub fn len(&self) -> usize {
        self.pts.len()
    }

    pub fn is_empty(&self) -> bool {
        self.pts.is_empty()
    }

    pub fn line(&self, name: &str) -> egui_plot::Line<'static> {
        egui_plot::Line::new(
            name,
            self.pts.iter().copied().collect::<egui_plot::PlotPoints>(),
        )
    }

    pub fn clear(&mut self) {
        self.pts.clear();
    }
}

/// Throttles sampling to one per `period` of sim time (C++ `should_sample`).
#[derive(Clone, Debug)]
pub struct SampleGate {
    period: f64,
    last: Option<f64>,
}

impl SampleGate {
    pub fn new(period: f64) -> Self {
        Self { period, last: None }
    }

    /// True (and rebased to `now`) on the first call, once `period` has passed, or when sim time
    /// went backwards (a reset must not stall sampling until the clock catches up).
    pub fn due(&mut self, now: f64) -> bool {
        let due = self
            .last
            .is_none_or(|last| now < last || now - last >= self.period);
        if due {
            self.last = Some(now);
        }
        due
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
    fn teleport_detector_clears_on_jump() {
        let mut d = TeleportDetector::default();
        let p = |x| Point2D { x, y: 0.0 };
        assert!(!d.observe(p(0.0)), "nothing to jump from");
        assert!(!d.observe(p(0.9)));
        assert!(d.observe(p(2.0)));
        assert!(!d.observe(p(2.5)), "rebased on the jump");
        d.reset();
        assert!(!d.observe(p(10.0)));
    }

    #[test]
    fn time_series_drops_oldest_past_cap() {
        let mut s = TimeSeries::new(2);
        for t in 0..3 {
            s.push(f64::from(t), 1.0);
        }
        assert_eq!(s.len(), 2);
        assert_eq!(s.last(), Some([2.0, 1.0]));
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

// C++ `Trail.*`, on the plotter trail parameters. Wrapped so the suite name does not clash with
// the type.
#[cfg(test)]
mod cpp {
    #[allow(non_snake_case)]
    mod Trail {
        use super::super::{MARKER_SPACING, MAX_MARKERS, Trail as T};
        use stdr_core::Pose2D;

        fn at(x: f64) -> Pose2D {
            Pose2D {
                x,
                ..Default::default()
            }
        }

        fn trail() -> T<Pose2D> {
            T::new(MAX_MARKERS, MARKER_SPACING)
        }

        #[test]
        fn FirstPointIsAlwaysAccepted() {
            let mut t = trail();
            t.push_if_moved(at(0.0));
            assert_eq!(t.len(), 1);
        }

        #[test]
        fn PointBelowSpacingIsDropped() {
            let mut t = trail();
            t.push_if_moved(at(0.0));
            t.push_if_moved(at(MARKER_SPACING / 2.0));
            assert_eq!(t.len(), 1);
        }

        #[test]
        fn PointAtOrAboveSpacingIsAccepted() {
            let mut t = trail();
            t.push_if_moved(at(0.0));
            t.push_if_moved(at(MARKER_SPACING));
            assert_eq!(t.len(), 2);
            assert_eq!(t.xy()[1], [MARKER_SPACING, 0.0]);
        }

        #[test]
        fn CapIsRespected() {
            let mut t = trail();
            for i in 0..MAX_MARKERS + 10 {
                t.push_if_moved(at(i as f64 * MARKER_SPACING * 2.0));
            }
            assert_eq!(t.len(), MAX_MARKERS);
            assert_eq!(t.xy().len(), MAX_MARKERS);
            let last = (MAX_MARKERS + 9) as f64 * MARKER_SPACING * 2.0;
            assert_eq!(t.xy().last(), Some(&[last, 0.0]));
            assert_eq!(t.get(MAX_MARKERS - 1), Some(&at(last)));
        }

        #[test]
        fn ClearEmptiesTheTrail() {
            let mut t = trail();
            t.push_if_moved(at(0.0));
            t.clear();
            assert!(t.is_empty());
        }
    }
}

// C++ `PlotHelpers.ShouldSample*`, on `SampleGate`.
#[cfg(test)]
#[allow(non_snake_case)]
mod PlotHelpers {
    use super::SampleGate;

    #[test]
    fn ShouldSampleFiresAtPeriod() {
        let mut g = SampleGate::new(0.1);
        assert!(g.due(0.0), "the first call fires");
        assert!(!g.due(0.05));
        assert!(g.due(0.1));
        assert!(!g.due(0.15), "rebased to 0.1 on the last fire");
    }

    #[test]
    fn ShouldSampleFiresWhenTimeGoesBackwards() {
        let mut g = SampleGate::new(1.0);
        assert!(g.due(10.0));
        assert!(g.due(0.5));
        assert!(!g.due(1.0), "rebased to 0.5");
    }
}

// C++ `PlotPanel.{Filter*, DuplicateFilterNameYieldsOneSlot, EmptyFilterInstantiatesAll,
// UnknownFilterNameYieldsNoSlots}`, on `select`.
#[cfg(test)]
#[allow(non_snake_case)]
mod PlotPanel {
    use super::{PlotterEntry, select};

    static A: PlotterEntry = PlotterEntry {
        key: "FakePlotter0",
        name: "Fake 0",
        description: "",
        build: |_| {},
    };
    static B: PlotterEntry = PlotterEntry {
        key: "FakePlotter1",
        name: "Fake 1",
        description: "",
        build: |_| {},
    };

    fn keys(filter: &[&str]) -> (Vec<&'static str>, Vec<String>) {
        let filter: Vec<String> = filter.iter().map(|s| s.to_string()).collect();
        let (picked, unknown) = select(&[&A, &B], &filter);
        (picked.iter().map(|e| e.key).collect(), unknown)
    }

    #[test]
    fn EmptyFilterInstantiatesAll() {
        assert_eq!(keys(&[]).0, ["FakePlotter0", "FakePlotter1"]);
    }

    #[test]
    fn FilterSelectsOnlyNamedPlotters() {
        assert_eq!(keys(&["FakePlotter1"]).0, ["FakePlotter1"]);
    }

    #[test]
    fn UnknownFilterNameYieldsNoSlots() {
        let (picked, unknown) = keys(&["NoSuchPlotter"]);
        assert!(picked.is_empty());
        assert_eq!(unknown, ["NoSuchPlotter"]);
    }

    #[test]
    fn DuplicateFilterNameYieldsOneSlot() {
        assert_eq!(keys(&["FakePlotter0", "FakePlotter0"]).0, ["FakePlotter0"]);
    }

    #[test]
    fn FilterWithUnknownAndKnownNameKeepsKnown() {
        let (picked, unknown) = keys(&["NoSuch", "FakePlotter0"]);
        assert_eq!(picked, ["FakePlotter0"]);
        assert_eq!(unknown, ["NoSuch"]);
    }
}
