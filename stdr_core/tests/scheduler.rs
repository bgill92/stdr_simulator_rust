//! C++ `test_rate_scheduler.cpp`, re-keyed from `(StreamKind, index)` to sensor indices.
//! Unregistered indices return `None` where C++ returned 0 / `SnapToMultiple`.
#![allow(non_snake_case)]

use approx::assert_abs_diff_eq;
use stdr_core::{RateScheduler, SchedulingMode};

use SchedulingMode::{Accumulator, SnapToMultiple};

fn sched(dt: f64) -> RateScheduler {
    RateScheduler::new(dt).unwrap()
}

/// Ticks `n` times; how often `idx` fired.
fn fires(s: &mut RateScheduler, idx: usize, n: usize) -> usize {
    (0..n).filter(|_| s.tick().contains(&idx)).count()
}

/// Per-tick fire pattern of `idx` over `n` ticks.
fn pattern(s: &mut RateScheduler, idx: usize, n: usize) -> Vec<bool> {
    (0..n).map(|_| s.tick().contains(&idx)).collect()
}

mod RateSchedulerTest {
    use super::*;

    #[test]
    fn ConstructorRejectsNonPositiveStepDt() {
        assert!(RateScheduler::new(0.0).is_err());
        assert!(RateScheduler::new(-1.0).is_err());
    }

    #[test]
    fn SetStepDtRejectsNonPositive() {
        let mut s = sched(0.1);
        assert!(s.set_step_dt(0.0).is_err());
        assert!(s.set_step_dt(-0.5).is_err());
        assert_eq!(s.step_dt(), 0.1);
    }

    #[test]
    fn UnregisteredStreamReturnsZero() {
        let s = sched(0.1);
        assert_eq!(s.effective_rate(0), None);
        assert_eq!(s.period_ticks(0), None);
    }

    #[test]
    fn DefaultZeroFreqFiresEveryTick() {
        let mut s = sched(0.1);
        s.set_rate(0, 0.0, SnapToMultiple);
        assert_eq!(s.period_ticks(0), Some(1));
        assert_eq!(fires(&mut s, 0, 5), 5);
    }

    #[test]
    fn NegativeFreqTreatedAsZero() {
        let mut s = sched(0.1);
        s.set_rate(0, -5.0, SnapToMultiple);
        assert_eq!(s.period_ticks(0), Some(1));
        assert_eq!(fires(&mut s, 0, 5), 5);
    }

    #[test]
    fn FreqExactlyMatchingSimRate() {
        let mut s = sched(0.1);
        s.set_rate(0, 10.0, SnapToMultiple);
        assert_eq!(s.period_ticks(0), Some(1));
        assert_eq!(s.effective_rate(0), Some(10.0));
    }

    #[test]
    fn FreqHalfSimRate() {
        let mut s = sched(0.1);
        s.set_rate(0, 5.0, SnapToMultiple);
        assert_eq!(s.period_ticks(0), Some(2));
        assert_eq!(
            pattern(&mut s, 0, 6),
            [false, true, false, true, false, true]
        );
    }

    #[test]
    fn VeryLowFreq() {
        let mut s = sched(0.1);
        s.set_rate(0, 0.5, SnapToMultiple);
        assert_eq!(s.period_ticks(0), Some(20));
    }

    #[test]
    fn FreqExceedsSimRateClamped() {
        let mut s = sched(0.1);
        s.set_rate(0, 100.0, SnapToMultiple);
        assert_eq!(s.period_ticks(0), Some(1));
        assert_eq!(s.effective_rate(0), Some(10.0));
    }

    /// 7 Hz at 10 Hz sim: raw period 1.43 ticks rounds to 1, not up to 2.
    #[test]
    fn RoundsToNearestPeriod() {
        let mut s = sched(0.1);
        s.set_rate(0, 7.0, SnapToMultiple);
        assert_eq!(s.period_ticks(0), Some(1));
        assert_eq!(s.effective_rate(0), Some(10.0));
    }

    #[test]
    fn ChangingStepDtRecomputesPeriods() {
        let mut s = sched(0.1);
        s.set_rate(0, 5.0, SnapToMultiple);
        s.set_rate(1, 10.0, SnapToMultiple);
        assert_eq!((s.period_ticks(0), s.period_ticks(1)), (Some(2), Some(1)));
        s.set_step_dt(0.05).unwrap();
        assert_eq!((s.period_ticks(0), s.period_ticks(1)), (Some(4), Some(2)));
    }

    #[test]
    fn MultipleStreamsIndependent() {
        let mut s = sched(0.1);
        s.set_rate(0, 5.0, SnapToMultiple);
        s.set_rate(1, 10.0, SnapToMultiple);
        let fired: Vec<_> = (0..4).map(|_| s.tick()).collect();
        assert_eq!(fired.iter().filter(|f| f.contains(&0)).count(), 2);
        assert_eq!(fired.iter().filter(|f| f.contains(&1)).count(), 4);
    }

    #[test]
    fn ClearRemovesAllEntries() {
        let mut s = sched(0.1);
        for i in 0..3 {
            s.set_rate(i, 10.0, SnapToMultiple);
        }
        s.clear();
        assert!(s.tick().is_empty());
        assert_eq!(s.period_ticks(0), None);
        assert_eq!(s.effective_rate(1), None);
    }

    #[test]
    fn TickReturnsStreamsInStableOrder() {
        let mut s = sched(0.1);
        for i in [4, 2, 0, 3, 1] {
            s.set_rate(i, 10.0, SnapToMultiple);
        }
        assert_eq!(s.tick(), [0, 1, 2, 3, 4]);
    }
}

mod EffectiveRateTest {
    use super::*;

    /// C++ `MatchesInverseOfPeriodTimesDt` over `FreqDtCombinations`.
    #[test]
    fn MatchesInverseOfPeriodTimesDt() {
        for (hz, dt) in [
            (10.0, 0.1),
            (5.0, 0.1),
            (20.0, 0.05),
            (1.0, 0.1),
            (0.5, 0.1),
        ] {
            let mut s = sched(dt);
            s.set_rate(0, hz, SnapToMultiple);
            let period = s.period_ticks(0).unwrap() as f64;
            assert_abs_diff_eq!(
                s.effective_rate(0).unwrap(),
                1.0 / (period * dt),
                epsilon = 1e-12
            );
        }
    }
}

mod AccumulatorModeTest {
    use super::*;

    #[test]
    fn DefaultModeIsSnapToMultiple() {
        assert_eq!(SchedulingMode::default(), SnapToMultiple);
    }

    #[test]
    fn UnregisteredStreamReturnsSnapMode() {
        assert_eq!(sched(0.1).mode(42), None);
    }

    #[test]
    fn AccumulatorZeroFreqFiresEveryTick() {
        let mut s = sched(0.1);
        s.set_rate(0, 0.0, Accumulator);
        assert_eq!(fires(&mut s, 0, 5), 5);
    }

    #[test]
    fn AccumulatorNegativeFreqFiresEveryTick() {
        let mut s = sched(0.1);
        s.set_rate(0, -5.0, Accumulator);
        assert_eq!(fires(&mut s, 0, 5), 5);
    }

    #[test]
    fn AccumulatorExactMatchFiresEveryTick() {
        let mut s = sched(0.1);
        s.set_rate(0, 10.0, Accumulator);
        assert_eq!(fires(&mut s, 0, 5), 5);
    }

    #[test]
    fn AccumulatorHalfSimRateFiresAlternate() {
        let mut s = sched(0.1);
        s.set_rate(0, 5.0, Accumulator);
        assert_eq!(
            pattern(&mut s, 0, 6),
            [false, true, false, true, false, true]
        );
    }

    #[test]
    fn AccumulatorClampedAtSimRate() {
        let mut s = sched(0.1);
        s.set_rate(0, 100.0, Accumulator);
        assert_eq!(fires(&mut s, 0, 5), 5);
    }

    #[test]
    fn AccumulatorResidualBoundedAtHighFreq() {
        let mut s = sched(0.1);
        s.set_rate(0, 100.0, Accumulator);
        assert_eq!(fires(&mut s, 0, 1000), 1000);
    }

    #[test]
    fn AccumulatorEffectiveRateMatchesTargetForCleanRatio() {
        let mut s = sched(0.1);
        s.set_rate(0, 5.0, Accumulator);
        assert_eq!(s.effective_rate(0), Some(5.0));
        assert_eq!(s.mode(0), Some(Accumulator));
    }

    #[test]
    fn AccumulatorEffectiveRateClampedAtSimRate() {
        let mut s = sched(0.1);
        s.set_rate(0, 100.0, Accumulator);
        assert_eq!(s.effective_rate(0), Some(10.0));
    }

    #[test]
    fn AccumulatorMatchesTargetRateOverTime() {
        let mut s = sched(0.1);
        s.set_rate(0, 7.0, Accumulator);
        assert!(fires(&mut s, 0, 100).abs_diff(70) <= 1);
    }

    #[test]
    fn AccumulatorNonIntegerRatioMatchesTarget() {
        let mut s = sched(0.1);
        s.set_rate(0, 3.0, Accumulator);
        assert!(fires(&mut s, 0, 100).abs_diff(30) <= 1);
    }

    #[test]
    fn AccumulatorIndependentOfSnap() {
        let mut s = sched(0.1);
        s.set_rate(0, 5.0, SnapToMultiple);
        s.set_rate(1, 7.0, Accumulator);
        let fired: Vec<_> = (0..100).map(|_| s.tick()).collect();
        assert_eq!(fired.iter().filter(|f| f.contains(&0)).count(), 50);
        assert!(fired.iter().filter(|f| f.contains(&1)).count().abs_diff(70) <= 1);
    }

    /// 4 Hz: two 0.1 s ticks leave 0.2 s accumulated; one 0.2 s tick reaches the 0.25 s period.
    #[test]
    fn SetStepDtPreservesAccumulatorState() {
        let mut s = sched(0.1);
        s.set_rate(0, 4.0, Accumulator);
        assert_eq!(pattern(&mut s, 0, 2), [false, false]);
        s.set_step_dt(0.2).unwrap();
        assert!(s.tick().contains(&0));
    }
}

mod SchedulingModeStringTest {
    use super::*;

    #[test]
    fn ToStringRoundTripsBothModes() {
        for m in [SnapToMultiple, Accumulator] {
            assert_eq!(m.to_string().parse::<SchedulingMode>().unwrap(), m);
        }
        assert_eq!(SnapToMultiple.to_string(), "snap_to_multiple");
        assert_eq!(Accumulator.to_string(), "accumulator");
    }

    #[test]
    fn FromStringRejectsUnknown() {
        let err = "nope".parse::<SchedulingMode>().unwrap_err().to_string();
        for needle in ["nope", "snap_to_multiple", "accumulator"] {
            assert!(err.contains(needle), "{err}");
        }
    }

    #[test]
    fn FromStringParsesValid() {
        assert_eq!(
            "snap_to_multiple".parse::<SchedulingMode>().unwrap(),
            SnapToMultiple
        );
        assert_eq!(
            "accumulator".parse::<SchedulingMode>().unwrap(),
            Accumulator
        );
    }
}
