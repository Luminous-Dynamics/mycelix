// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Navigation runtime: measurement routing, health monitoring, failover.

use crate::measurements::{Measurement, MeasurementModality};
use serde::{Deserialize, Serialize};

/// Navigation health status.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum NavigationHealth {
    Good,
    Degraded,
    Lost,
}

/// Failover mode when primary positioning fails.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum NavigationFailoverMode {
    DeadReckoning,
    PeerOnly,
    None,
}

/// Policy for routing measurements to positioning algorithms.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum MeasurementRoutingPolicy {
    AllSources,
    LocalOnly,
    /// Legacy modality allowlist. The name does not imply statistically
    /// qualified confidence; migrate authority-facing callers to policy based
    /// on explicit uncertainty, freshness, provenance, and source health.
    HighConfidenceOnly,
}

/// Statistics for measurement routing.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct MeasurementRoutingStats {
    pub total_received: u64,
    pub total_accepted: u64,
    pub total_rejected: u64,
}

/// Routes measurements to the appropriate positioning algorithm.
pub struct MeasurementRouter {
    policy: MeasurementRoutingPolicy,
    stats: MeasurementRoutingStats,
    accepted_modalities: Vec<MeasurementModality>,
}

impl MeasurementRouter {
    pub fn new(policy: MeasurementRoutingPolicy) -> Self {
        Self {
            policy,
            stats: MeasurementRoutingStats::default(),
            accepted_modalities: vec![
                MeasurementModality::UwbToF,
                MeasurementModality::WifiRtt,
                MeasurementModality::Gps,
            ],
        }
    }

    pub fn route(&mut self, measurement: &Measurement) -> bool {
        self.stats.total_received += 1;
        let accept = match self.policy {
            MeasurementRoutingPolicy::AllSources => true,
            MeasurementRoutingPolicy::LocalOnly => {
                measurement.provenance == crate::measurements::MeasurementProvenance::Local
            }
            MeasurementRoutingPolicy::HighConfidenceOnly => {
                self.accepted_modalities.contains(&measurement.modality)
            }
        };
        if accept {
            self.stats.total_accepted += 1;
        } else {
            self.stats.total_rejected += 1;
        }
        accept
    }

    pub fn stats(&self) -> &MeasurementRoutingStats {
        &self.stats
    }
}

/// Clock-order errors while determining observation/fix freshness.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FixAgeError {
    /// The supplied current time precedes the fix timestamp.
    ClockReversal { fix_us: u64, current_us: u64 },
}

impl std::fmt::Display for FixAgeError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::ClockReversal { fix_us, current_us } => write!(
                f,
                "current timestamp {current_us}us precedes fix timestamp {fix_us}us"
            ),
        }
    }
}

impl std::error::Error for FixAgeError {}

/// Domain-specific navigation coordinator.
pub struct DomainNavigator {
    health: NavigationHealth,
    failover: NavigationFailoverMode,
    last_fix_us: u64,
    router: MeasurementRouter,
}

impl DomainNavigator {
    pub fn new() -> Self {
        Self {
            health: NavigationHealth::Good,
            failover: NavigationFailoverMode::DeadReckoning,
            last_fix_us: 0,
            router: MeasurementRouter::new(MeasurementRoutingPolicy::AllSources),
        }
    }
    pub fn health(&self) -> NavigationHealth {
        self.health
    }
    pub fn update_fix(&mut self, timestamp_us: u64) {
        self.last_fix_us = timestamp_us;
        self.health = NavigationHealth::Good;
    }
    pub fn check_health(&mut self, current_us: u64) {
        let Ok(age) = fix_age_s_checked(self.last_fix_us, current_us) else {
            // A future-dated fix or clock reversal is not evidence of freshness.
            // Fail closed until a stronger trusted-time model can classify it.
            self.health = NavigationHealth::Lost;
            return;
        };
        if age > 30.0 {
            self.health = NavigationHealth::Lost;
        } else if age > 10.0 {
            self.health = NavigationHealth::Degraded;
        }
    }
    pub fn router_mut(&mut self) -> &mut MeasurementRouter {
        &mut self.router
    }
}

impl Default for DomainNavigator {
    fn default() -> Self {
        Self::new()
    }
}

/// Compute fix age in seconds from microsecond timestamps with explicit
/// clock-order validation.
pub fn fix_age_s_checked(fix_us: u64, current_us: u64) -> Result<f64, FixAgeError> {
    if current_us < fix_us {
        return Err(FixAgeError::ClockReversal { fix_us, current_us });
    }
    Ok((current_us - fix_us) as f64 / 1_000_000.0)
}

/// Backward-compatible fix-age helper.
///
/// Clock reversal now returns positive infinity rather than zero, so legacy
/// freshness comparisons fail conservatively instead of treating a
/// future-dated fix as maximally fresh. New authority-facing callers should
/// use [`fix_age_s_checked`] to preserve the rejection reason.
pub fn fix_age_s(fix_us: u64, current_us: u64) -> f64 {
    fix_age_s_checked(fix_us, current_us).unwrap_or(f64::INFINITY)
}

/// Convert sigma (standard deviation) to a presentation confidence score [0, 100].
///
/// This exponential mapping is a UI heuristic, not a calibrated probability
/// and not authority-bearing measurement confidence.
pub fn confidence_from_sigma(sigma_m: f64) -> f64 {
    if sigma_m <= 0.0 {
        return 100.0;
    }
    (100.0 * (-sigma_m / 10.0).exp()).clamp(0.0, 100.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_fix_age() {
        assert!((fix_age_s_checked(1_000_000, 2_000_000).unwrap() - 1.0).abs() < 0.001);
        assert_eq!(fix_age_s_checked(1_000_000, 1_000_000).unwrap(), 0.0);
    }

    #[test]
    fn checked_fix_age_rejects_clock_reversal() {
        let err = fix_age_s_checked(2_000_000, 1_000_000).expect_err("future fix must reject");
        assert_eq!(
            err,
            FixAgeError::ClockReversal {
                fix_us: 2_000_000,
                current_us: 1_000_000,
            }
        );
    }

    #[test]
    fn legacy_fix_age_fails_conservatively_on_clock_reversal() {
        assert!(fix_age_s(2_000_000, 1_000_000).is_infinite());
    }

    #[test]
    fn navigator_marks_future_dated_fix_lost() {
        let mut navigator = DomainNavigator::new();
        navigator.update_fix(2_000_000);
        navigator.check_health(1_000_000);
        assert_eq!(navigator.health(), NavigationHealth::Lost);
    }

    #[test]
    fn navigator_health_tracks_fix_age() {
        let mut navigator = DomainNavigator::new();
        navigator.update_fix(1_000_000);
        navigator.check_health(6_000_000);
        assert_eq!(navigator.health(), NavigationHealth::Good);
        navigator.check_health(12_000_001);
        assert_eq!(navigator.health(), NavigationHealth::Degraded);
        navigator.check_health(32_000_001);
        assert_eq!(navigator.health(), NavigationHealth::Lost);
    }

    #[test]
    fn test_confidence() {
        assert!(confidence_from_sigma(0.0) > 99.0);
        assert!(confidence_from_sigma(50.0) < 1.0);
    }
}
