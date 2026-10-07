//! Per-sensor fire cadence on a fixed sim tick.

use std::collections::BTreeMap;
use std::fmt;
use std::str::FromStr;

use crate::error::CoreError;

#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum SchedulingMode {
    /// Fire every `round(1 / (hz · step_dt))` ticks (at least 1).
    #[default]
    SnapToMultiple,
    /// Accumulate sim time and fire whenever it reaches `1 / hz`: true average rate.
    Accumulator,
}

impl fmt::Display for SchedulingMode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            SchedulingMode::SnapToMultiple => "snap_to_multiple",
            SchedulingMode::Accumulator => "accumulator",
        })
    }
}

impl FromStr for SchedulingMode {
    type Err = CoreError;

    fn from_str(s: &str) -> Result<Self, CoreError> {
        match s {
            "snap_to_multiple" => Ok(SchedulingMode::SnapToMultiple),
            "accumulator" => Ok(SchedulingMode::Accumulator),
            _ => Err(CoreError::Invalid(format!(
                "unknown scheduling_mode '{s}'; expected 'snap_to_multiple' or 'accumulator'"
            ))),
        }
    }
}

// Periods are derived from (freq_hz, step_dt) on demand instead of cached, so `set_step_dt`
// has nothing to recompute.
#[derive(Clone, Debug)]
struct Entry {
    freq_hz: f64,
    mode: SchedulingMode,
    /// Ticks seen; preserved across `set_rate` so an in-flight change keeps its cadence (C++).
    ticks: u64,
    /// Accumulator residual in seconds, in `[0, 1/freq_hz)`.
    acc: f64,
}

/// Keyed by sensor index (position in `RobotConfig::sensors`).
#[derive(Clone, Debug)]
pub struct RateScheduler {
    step_dt: f64,
    entries: BTreeMap<usize, Entry>,
}

pub(crate) fn check_step_dt(step_dt: f64) -> Result<(), CoreError> {
    if step_dt.is_nan() || step_dt <= 0.0 {
        return Err(CoreError::Invalid(format!(
            "step_dt must be positive, got {step_dt}"
        )));
    }
    Ok(())
}

impl RateScheduler {
    pub fn new(step_dt: f64) -> Result<Self, CoreError> {
        check_step_dt(step_dt)?;
        Ok(Self {
            step_dt,
            entries: BTreeMap::new(),
        })
    }

    pub fn step_dt(&self) -> f64 {
        self.step_dt
    }

    /// Periods follow the new dt; tick counts and accumulator residuals are kept.
    pub fn set_step_dt(&mut self, step_dt: f64) -> Result<(), CoreError> {
        check_step_dt(step_dt)?;
        self.step_dt = step_dt;
        Ok(())
    }

    /// `hz <= 0` fires every tick in both modes; a rate above the sim rate is clamped to it.
    pub fn set_rate(&mut self, idx: usize, hz: f64, mode: SchedulingMode) {
        let e = self.entries.entry(idx).or_insert(Entry {
            freq_hz: 0.0,
            mode,
            ticks: 0,
            acc: 0.0,
        });
        e.freq_hz = hz;
        e.mode = mode;
    }

    pub fn clear(&mut self) {
        self.entries.clear();
    }

    /// Advances one tick; returns the sensor indices that fire, ascending.
    pub fn tick(&mut self) -> Vec<usize> {
        let dt = self.step_dt;
        self.entries
            .iter_mut()
            .filter_map(|(&idx, e)| {
                e.ticks += 1;
                let fire = match e.mode {
                    SchedulingMode::SnapToMultiple => e.ticks % period_ticks(e.freq_hz, dt) == 0,
                    SchedulingMode::Accumulator if e.freq_hz <= 0.0 => true,
                    SchedulingMode::Accumulator => {
                        let period = 1.0 / e.freq_hz;
                        e.acc += dt;
                        let fire = e.acc >= period;
                        if fire {
                            // Wrap so the residual stays bounded when period < dt.
                            e.acc %= period;
                        }
                        fire
                    }
                };
                fire.then_some(idx)
            })
            .collect()
    }

    /// Snap: `1 / (period_ticks · dt)`. Accumulator: the target, clamped to the sim rate.
    pub fn effective_rate(&self, idx: usize) -> Option<f64> {
        let e = self.entries.get(&idx)?;
        let sim_rate = 1.0 / self.step_dt;
        Some(match e.mode {
            SchedulingMode::Accumulator if e.freq_hz > 0.0 && e.freq_hz <= sim_rate => e.freq_hz,
            SchedulingMode::Accumulator => sim_rate,
            SchedulingMode::SnapToMultiple => {
                1.0 / (period_ticks(e.freq_hz, self.step_dt) as f64 * self.step_dt)
            }
        })
    }

    pub fn period_ticks(&self, idx: usize) -> Option<u64> {
        let e = self.entries.get(&idx)?;
        Some(period_ticks(e.freq_hz, self.step_dt))
    }

    pub fn mode(&self, idx: usize) -> Option<SchedulingMode> {
        self.entries.get(&idx).map(|e| e.mode)
    }
}

/// `round(1 / (hz · dt))`, at least 1; `hz <= 0` → 1. `f64::round` is half-away-from-zero
/// like `std::round`.
fn period_ticks(hz: f64, dt: f64) -> u64 {
    if hz <= 0.0 {
        return 1;
    }
    ((1.0 / (hz * dt)).round() as u64).max(1)
}
