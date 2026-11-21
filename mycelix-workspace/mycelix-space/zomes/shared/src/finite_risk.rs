// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Finite-horizon conjunction risk bounds with explicit dependence semantics.
//!
//! A complete event-universe receipt remains mandatory before an aggregate is emitted.
//! Version 0.1 is a bounded baseline; joint dependence models are a later stage.
//! This module aggregates supplied per-event marginal probabilities without
//! assuming independence. It returns mathematically valid union bounds for the
//! supplied event set and a separately named independence-only comparison
//! baseline. It does not calculate event Pc, determine event-universe
//! completeness, validate covariance realism, or establish physical truth.

use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;

/// A per-event marginal probability produced by a separately qualified Pc path.
///
/// Exactly one canonical event revision must be provided for each event ID.
/// Repeated updates to one event must first be reconciled under an explicit
/// event-version policy; they are not independent collision opportunities.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct EventMarginalProbability {
    pub event_id: String,
    /// Event TCA in microseconds since 1970-01-01T00:00:00 TAI.
    pub tca_tai_micros: i64,
    /// Marginal probability conditional on the declared per-event model.
    pub pc: f64,
    pub pc_method_id: String,
    pub covariance_profile_id: String,
}

/// Evidence that the event universe supplied to the aggregator was complete.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
pub enum EventUniverseCoverage {
    /// A separate screening/candidate-generation contract says this finite
    /// event universe is complete for the declared horizon and capture profile.
    Complete {
        coverage_receipt_sha256: String,
        capture_profile_id: String,
    },
    /// Some candidates or time intervals were not resolved. No finite-horizon
    /// total is returned; the caller must retain an indeterminate disposition.
    Incomplete { reason: String },
    /// Completeness has not been established.
    Unknown,
}

/// Input contract for aggregating event marginals over a fixed TAI horizon.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct FiniteHorizonRiskInput {
    pub horizon_id: String,
    pub horizon_start_tai_micros: i64,
    pub horizon_end_tai_micros: i64,
    /// SHA-256 of the canonical, ordered event-set manifest.
    pub event_set_sha256: String,
    /// Exact event-level Pc method/profile family used to produce the marginals.
    pub pc_model_profile_id: String,
    pub coverage: EventUniverseCoverage,
    pub events: Vec<EventMarginalProbability>,
}

/// Dependence-agnostic bounds for the probability of at least one supplied event.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct FiniteHorizonUnionBounds {
    pub horizon_id: String,
    pub horizon_start_tai_micros: i64,
    pub horizon_end_tai_micros: i64,
    pub event_set_sha256: String,
    pub coverage_receipt_sha256: String,
    pub capture_profile_id: String,
    pub pc_model_profile_id: String,
    pub event_count: usize,
    /// Sum of the supplied marginal probabilities; this may exceed one.
    pub marginal_pc_sum: f64,
    /// Distribution-free lower bound: max_i P(C_i).
    pub union_probability_lower_bound: f64,
    /// Distribution-free upper bound: min(1, sum_i P(C_i)).
    pub union_probability_upper_bound: f64,
    /// Comparison only. Valid as the union probability only under mutual
    /// independence of the supplied collision events.
    pub independence_assumption_reference: f64,
}

/// Why an aggregation request cannot produce a complete finite-horizon result.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
pub enum FiniteHorizonRiskError {
    EmptyHorizonId,
    InvalidHorizon,
    InvalidEventSetDigest,
    EmptyPcModelProfileId,
    EmptyCoverageReceiptDigest,
    InvalidCoverageReceiptDigest,
    EmptyCaptureProfileId,
    IncompleteEventUniverse { reason: String },
    UnknownEventUniverseCoverage,
    TooManyEvents { count: usize, limit: usize },
    EmptyEventId,
    DuplicateEventId { event_id: String },
    EventOutsideHorizon { event_id: String },
    InvalidPc { event_id: String },
    EmptyPcMethodId { event_id: String },
    EmptyCovarianceProfileId { event_id: String },
}

const MAX_FINITE_HORIZON_EVENTS: usize = 1_000_000;

fn is_sha256_hex(value: &str) -> bool {
    value.len() == 64 && value.bytes().all(|byte| byte.is_ascii_hexdigit())
}

/// Aggregate a complete event set without inventing a dependence model.
///
/// For collision events C_i with marginal probabilities p_i, the union bounds
/// are valid under arbitrary dependence:
///
/// - lower = max_i p_i
/// - upper = min(1, sum_i p_i)
///
/// The returned independence reference is computed only as a comparator:
/// 1 - product_i (1 - p_i). It is not a qualified total probability unless
/// mutual independence is supported by the declared joint model.
///
/// A coverage receipt is mandatory. This function validates its shape and
/// binds it to the result, but the caller remains responsible for verifying
/// the receipt and the upstream candidate-capture/event-search contract.
/// A complete empty event set has an exact zero union probability; callers must
/// not construct that empty set from missing, filtered, or unresolved candidates.
/// An aggregate is defined only over the exact event universe proven complete by its receipt.
/// Event dependence is deliberately unknown in this baseline; only bounds are unconditional.

pub fn aggregate_finite_horizon_union_bounds(
    input: &FiniteHorizonRiskInput,
) -> Result<FiniteHorizonUnionBounds, FiniteHorizonRiskError> {
    if input.horizon_id.trim().is_empty() {
        return Err(FiniteHorizonRiskError::EmptyHorizonId);
    }
    if input.horizon_end_tai_micros <= input.horizon_start_tai_micros {
        return Err(FiniteHorizonRiskError::InvalidHorizon);
    }
    if !is_sha256_hex(&input.event_set_sha256) {
        return Err(FiniteHorizonRiskError::InvalidEventSetDigest);
    }
    if input.pc_model_profile_id.trim().is_empty() {
        return Err(FiniteHorizonRiskError::EmptyPcModelProfileId);
    }

    let (coverage_receipt_sha256, capture_profile_id) = match &input.coverage {
        EventUniverseCoverage::Complete {
            coverage_receipt_sha256,
            capture_profile_id,
        } => {
            if !is_sha256_hex(coverage_receipt_sha256) {
                return Err(FiniteHorizonRiskError::InvalidCoverageReceiptDigest);
            }
            if capture_profile_id.trim().is_empty() {
                return Err(FiniteHorizonRiskError::EmptyCaptureProfileId);
            }
            (coverage_receipt_sha256.clone(), capture_profile_id.clone())
        }
        EventUniverseCoverage::Incomplete { reason } => {
            return Err(FiniteHorizonRiskError::IncompleteEventUniverse {
                reason: reason.clone(),
            });
        }
        EventUniverseCoverage::Unknown => {
            return Err(FiniteHorizonRiskError::UnknownEventUniverseCoverage);
        }
    };

    if input.events.len() > MAX_FINITE_HORIZON_EVENTS {
        return Err(FiniteHorizonRiskError::TooManyEvents {
            count: input.events.len(),
            limit: MAX_FINITE_HORIZON_EVENTS,
        });
    }

    let mut event_ids = BTreeSet::new();
    let mut marginal_sum = 0.0_f64;
    let mut lower_bound = 0.0_f64;
    let mut log_survival = 0.0_f64;
    let mut any_certain_event = false;

    for event in &input.events {
        if event.event_id.trim().is_empty() {
            return Err(FiniteHorizonRiskError::EmptyEventId);
        }
        if !event_ids.insert(event.event_id.as_str()) {
            return Err(FiniteHorizonRiskError::DuplicateEventId {
                event_id: event.event_id.clone(),
            });
        }
        if event.tca_tai_micros < input.horizon_start_tai_micros
            || event.tca_tai_micros > input.horizon_end_tai_micros
        {
            return Err(FiniteHorizonRiskError::EventOutsideHorizon {
                event_id: event.event_id.clone(),
            });
        }
        if !event.pc.is_finite() || !(0.0..=1.0).contains(&event.pc) {
            return Err(FiniteHorizonRiskError::InvalidPc {
                event_id: event.event_id.clone(),
            });
        }
        if event.pc_method_id.trim().is_empty() {
            return Err(FiniteHorizonRiskError::EmptyPcMethodId {
                event_id: event.event_id.clone(),
            });
        }
        if event.covariance_profile_id.trim().is_empty() {
            return Err(FiniteHorizonRiskError::EmptyCovarianceProfileId {
                event_id: event.event_id.clone(),
            });
        }

        marginal_sum += event.pc;
        lower_bound = lower_bound.max(event.pc);
        if event.pc >= 1.0 {
            any_certain_event = true;
        } else {
            // ln_1p preserves precision for very small marginal Pc values.
            log_survival += (-event.pc).ln_1p();
        }
    }

    let upper_bound = marginal_sum.min(1.0);
    let independence_reference = if any_certain_event {
        1.0
    } else {
        // -expm1(sum(log(1-p_i))) is more stable than 1 - product(1-p_i)
        // for small Pc values.
        -log_survival.exp_m1()
    };

    Ok(FiniteHorizonUnionBounds {
        horizon_id: input.horizon_id.clone(),
        horizon_start_tai_micros: input.horizon_start_tai_micros,
        horizon_end_tai_micros: input.horizon_end_tai_micros,
        event_set_sha256: input.event_set_sha256.clone(),
        coverage_receipt_sha256,
        capture_profile_id,
        pc_model_profile_id: input.pc_model_profile_id.clone(),
        event_count: input.events.len(),
        marginal_pc_sum: marginal_sum,
        union_probability_lower_bound: lower_bound,
        union_probability_upper_bound: upper_bound,
        independence_assumption_reference: independence_reference,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    const VALID_DIGEST_A: &str = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";
    const VALID_DIGEST_B: &str = "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb";

    fn event(event_id: &str, tca: i64, pc: f64) -> EventMarginalProbability {
        EventMarginalProbability {
            event_id: event_id.to_string(),
            tca_tai_micros: tca,
            pc,
            pc_method_id: "cara-3d-reference-v1".to_string(),
            covariance_profile_id: "teme-covariance-profile-v1".to_string(),
        }
    }

    fn input(events: Vec<EventMarginalProbability>) -> FiniteHorizonRiskInput {
        FiniteHorizonRiskInput {
            horizon_id: "protected-object-25544-window-1".to_string(),
            horizon_start_tai_micros: 1_000,
            horizon_end_tai_micros: 10_000,
            event_set_sha256: VALID_DIGEST_A.to_string(),
            pc_model_profile_id: "pc-profile-v1".to_string(),
            coverage: EventUniverseCoverage::Complete {
                coverage_receipt_sha256: VALID_DIGEST_B.to_string(),
                capture_profile_id: "screen-volume-profile-v1".to_string(),
            },
            events,
        }
    }

    #[test]
    fn union_bounds_do_not_assume_independence() {
        let result = aggregate_finite_horizon_union_bounds(&input(vec![
            event("event-a", 2_000, 0.1),
            event("event-b", 3_000, 0.2),
        ]))
        .unwrap();

        assert!((result.union_probability_lower_bound - 0.2).abs() < 1e-12);
        assert!((result.union_probability_upper_bound - 0.3).abs() < 1e-12);
        // This is a separate comparator, not the default aggregate truth.
        assert!((result.independence_assumption_reference - 0.28).abs() < 1e-12);
        assert_eq!(result.event_count, 2);
        assert_eq!(result.coverage_receipt_sha256, VALID_DIGEST_B);
    }

    #[test]
    fn one_event_has_exact_union_probability() {
        let result =
            aggregate_finite_horizon_union_bounds(&input(vec![event("event-a", 2_000, 0.17)]))
                .unwrap();

        assert_eq!(result.union_probability_lower_bound, 0.17);
        assert_eq!(result.union_probability_upper_bound, 0.17);
        assert_eq!(result.independence_assumption_reference, 0.17);
    }

    #[test]
    fn empty_complete_event_set_has_zero_union_probability() {
        let result = aggregate_finite_horizon_union_bounds(&input(Vec::new())).unwrap();

        assert_eq!(result.event_count, 0);
        assert_eq!(result.union_probability_lower_bound, 0.0);
        assert_eq!(result.union_probability_upper_bound, 0.0);
        assert_eq!(result.independence_assumption_reference, 0.0);
    }

    #[test]
    fn incomplete_or_unknown_event_universe_is_not_aggregated() {
        let mut incomplete = input(vec![event("event-a", 2_000, 0.1)]);
        incomplete.coverage = EventUniverseCoverage::Incomplete {
            reason: "unresolved trajectory interval".to_string(),
        };
        assert_eq!(
            aggregate_finite_horizon_union_bounds(&incomplete),
            Err(FiniteHorizonRiskError::IncompleteEventUniverse {
                reason: "unresolved trajectory interval".to_string(),
            })
        );

        let mut unknown = input(Vec::new());
        unknown.coverage = EventUniverseCoverage::Unknown;
        assert_eq!(
            aggregate_finite_horizon_union_bounds(&unknown),
            Err(FiniteHorizonRiskError::UnknownEventUniverseCoverage)
        );
    }

    #[test]
    fn invalid_probabilities_are_rejected_not_clamped() {
        for (id, pc) in [
            ("nan", f64::NAN),
            ("infinity", f64::INFINITY),
            ("negative", -0.01),
            ("above-one", 1.01),
        ] {
            assert_eq!(
                aggregate_finite_horizon_union_bounds(&input(vec![event(id, 2_000, pc)])),
                Err(FiniteHorizonRiskError::InvalidPc {
                    event_id: id.to_string(),
                })
            );
        }
    }

    #[test]
    fn duplicate_event_ids_are_rejected() {
        assert_eq!(
            aggregate_finite_horizon_union_bounds(&input(vec![
                event("duplicate", 2_000, 0.1),
                event("duplicate", 2_100, 0.2),
            ])),
            Err(FiniteHorizonRiskError::DuplicateEventId {
                event_id: "duplicate".to_string(),
            })
        );
    }

    #[test]
    fn events_outside_the_declared_horizon_are_rejected() {
        assert_eq!(
            aggregate_finite_horizon_union_bounds(&input(vec![event("too-late", 10_001, 0.1)])),
            Err(FiniteHorizonRiskError::EventOutsideHorizon {
                event_id: "too-late".to_string(),
            })
        );
    }

    #[test]
    fn invalid_receipt_and_event_set_digests_are_rejected() {
        let mut bad_event_set = input(Vec::new());
        bad_event_set.event_set_sha256 = "not-a-digest".to_string();
        assert_eq!(
            aggregate_finite_horizon_union_bounds(&bad_event_set),
            Err(FiniteHorizonRiskError::InvalidEventSetDigest)
        );

        let mut bad_receipt = input(Vec::new());
        bad_receipt.coverage = EventUniverseCoverage::Complete {
            coverage_receipt_sha256: "bad".to_string(),
            capture_profile_id: "profile".to_string(),
        };
        assert_eq!(
            aggregate_finite_horizon_union_bounds(&bad_receipt),
            Err(FiniteHorizonRiskError::InvalidCoverageReceiptDigest)
        );
    }

    #[test]
    fn malformed_event_semantics_are_rejected() {
        let mut missing_method = input(vec![event("event-a", 2_000, 0.1)]);
        missing_method.events[0].pc_method_id.clear();
        assert_eq!(
            aggregate_finite_horizon_union_bounds(&missing_method),
            Err(FiniteHorizonRiskError::EmptyPcMethodId {
                event_id: "event-a".to_string(),
            })
        );

        let mut missing_covariance = input(vec![event("event-a", 2_000, 0.1)]);
        missing_covariance.events[0].covariance_profile_id.clear();
        assert_eq!(
            aggregate_finite_horizon_union_bounds(&missing_covariance),
            Err(FiniteHorizonRiskError::EmptyCovarianceProfileId {
                event_id: "event-a".to_string(),
            })
        );
    }

    #[test]
    fn zero_and_certain_event_bounds_are_exact() {
        let zero = aggregate_finite_horizon_union_bounds(&input(vec![
            event("zero-a", 2_000, 0.0),
            event("zero-b", 3_000, 0.0),
        ]))
        .unwrap();
        assert_eq!(zero.union_probability_lower_bound, 0.0);
        assert_eq!(zero.union_probability_upper_bound, 0.0);
        assert_eq!(zero.independence_assumption_reference, 0.0);

        let certain = aggregate_finite_horizon_union_bounds(&input(vec![
            event("certain", 2_000, 1.0),
            event("other", 3_000, 0.2),
        ]))
        .unwrap();
        assert_eq!(certain.union_probability_lower_bound, 1.0);
        assert_eq!(certain.union_probability_upper_bound, 1.0);
        assert_eq!(certain.independence_assumption_reference, 1.0);
    }
}
