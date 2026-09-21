// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Deterministic Bayesian Knowledge Tracing as an advisory evidence projection.
//!
//! This module intentionally does not expose a `mastered`, `passed`, or
//! credential-authority result. It projects explicitly admitted, provenance-complete
//! attempt observations into one dimension-specific estimate.

use crate::attempt_evidence::{
    AttemptEvidence, AttemptEvidenceCompleteness, AttemptEvidenceContractError, AttemptOutcome,
};
use crate::crypto::hash_to_string;
use crate::learning_evidence::{
    AdmissionOutcome, AdvisoryCapabilityEstimate, CapabilityDimension, CapabilityId,
    DimensionEstimate, EstimatorProvenance, EvidenceAdmissionDecision, EvidenceEventId,
    LearnerId,
};
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;

pub const BKT_ESTIMATOR_ID: &str = "praxis:bkt-projection";
pub const BKT_ESTIMATOR_VERSION: &str = "integer-v1";

/// Parameters for the integer-deterministic BKT projection.
///
/// All probabilities use permille (0..=1000). `support_confidence_step_permille`
/// is deliberately named as a support heuristic rather than statistical certainty:
/// each admitted observation adds this much displayed confidence, capped at 1000.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct BktParameters {
    pub prior_permille: u16,
    pub learn_permille: u16,
    pub guess_permille: u16,
    pub slip_permille: u16,
    pub support_confidence_step_permille: u16,
}

impl Default for BktParameters {
    fn default() -> Self {
        Self {
            prior_permille: 100,
            learn_permille: 100,
            guess_permille: 200,
            slip_permille: 100,
            support_confidence_step_permille: 50,
        }
    }
}

impl BktParameters {
    pub fn validate(&self) -> Result<(), BktProjectionError> {
        for (field, value) in [
            ("prior_permille", self.prior_permille),
            ("learn_permille", self.learn_permille),
            ("guess_permille", self.guess_permille),
            ("slip_permille", self.slip_permille),
            (
                "support_confidence_step_permille",
                self.support_confidence_step_permille,
            ),
        ] {
            if value > 1000 {
                return Err(BktProjectionError::ParameterOutOfRange {
                    field: field.to_string(),
                    value_permille: value,
                });
            }
        }
        Ok(())
    }
}

/// Explicit encoding policy for turning an attempt outcome into the binary
/// observation required by classical BKT.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum BktObservationEncoding {
    /// Accept only source observations that were already binary.
    BinaryOnly,
    /// Convert a scored observation to binary using this explicit threshold.
    /// Binary source observations remain binary and are not re-thresholded.
    ScoredThreshold {
        correct_at_or_above_permille: u16,
    },
}

impl BktObservationEncoding {
    pub fn validate(&self) -> Result<(), BktProjectionError> {
        if let Self::ScoredThreshold {
            correct_at_or_above_permille,
        } = self
        {
            if *correct_at_or_above_permille > 1000 {
                return Err(BktProjectionError::EncodingThresholdOutOfRange {
                    value_permille: *correct_at_or_above_permille,
                });
            }
        }
        Ok(())
    }

    fn encode(&self, outcome: &AttemptOutcome) -> Result<bool, BktProjectionError> {
        match (self, outcome) {
            (_, AttemptOutcome::Binary { correct }) => Ok(*correct),
            (Self::BinaryOnly, AttemptOutcome::Scored { .. }) => {
                Err(BktProjectionError::ScoredObservationRequiresThreshold)
            }
            (
                Self::ScoredThreshold {
                    correct_at_or_above_permille,
                },
                AttemptOutcome::Scored { score_permille },
            ) => {
                if *score_permille > 1000 {
                    return Err(BktProjectionError::AttemptContract(
                        AttemptEvidenceContractError::OutcomeOutOfRange {
                            score_permille: *score_permille,
                        },
                    ));
                }
                Ok(*score_permille >= *correct_at_or_above_permille)
            }
        }
    }

    fn canonical_fragment(&self) -> String {
        match self {
            Self::BinaryOnly => "encoding=binary-only".to_string(),
            Self::ScoredThreshold {
                correct_at_or_above_permille,
            } => format!("encoding=scored-threshold:{correct_at_or_above_permille}"),
        }
    }
}

/// One attempt plus the explicit policy-relative admission decision that permits
/// it to enter this projection.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct AdmittedBktAttempt {
    pub attempt: AttemptEvidence,
    pub admission: EvidenceAdmissionDecision,
}

/// Complete input required to recompute a projection.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct BktProjectionInput {
    pub learner_id: LearnerId,
    pub capability_id: CapabilityId,
    pub dimension: CapabilityDimension,
    pub admission_profile_id: String,
    pub admission_profile_version: String,
    pub parameters: BktParameters,
    pub encoding: BktObservationEncoding,
    pub observations: Vec<AdmittedBktAttempt>,
    pub generated_at: i64,
}

/// Recomputable receipt for one BKT projection.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct BktProjectionReceipt {
    pub estimate: AdvisoryCapabilityEstimate,
    /// Canonical chronological ordering actually consumed by the estimator.
    pub ordered_event_ids: Vec<EvidenceEventId>,
    pub parameters: BktParameters,
    pub encoding: BktObservationEncoding,
}

impl BktProjectionReceipt {
    pub const fn grants_credential_authority(&self) -> bool {
        false
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum BktProjectionError {
    EmptyAdmissionProfile,
    NoObservations,
    ParameterOutOfRange {
        field: String,
        value_permille: u16,
    },
    EncodingThresholdOutOfRange {
        value_permille: u16,
    },
    AttemptContract(AttemptEvidenceContractError),
    IncompleteAttempt(EvidenceEventId),
    LearnerMismatch(EvidenceEventId),
    CapabilityMismatch(EvidenceEventId),
    DimensionMismatch(EvidenceEventId),
    AdmissionEventMismatch(EvidenceEventId),
    AdmissionProfileMismatch(EvidenceEventId),
    ObservationNotAdmitted(EvidenceEventId),
    DuplicateEvent(EvidenceEventId),
    ScoredObservationRequiresThreshold,
    ImpossibleObservationLikelihood(EvidenceEventId),
}

/// Project explicitly admitted attempt evidence into a dimension-specific advisory
/// BKT estimate.
///
/// Input order is not trusted. Observations are sorted canonically by
/// `(observed_at, event_id)` before the sequence-dependent BKT update is applied.
pub fn project_bkt(
    mut input: BktProjectionInput,
) -> Result<BktProjectionReceipt, BktProjectionError> {
    if input.admission_profile_id.trim().is_empty()
        || input.admission_profile_version.trim().is_empty()
    {
        return Err(BktProjectionError::EmptyAdmissionProfile);
    }
    if input.observations.is_empty() {
        return Err(BktProjectionError::NoObservations);
    }
    input.parameters.validate()?;
    input.encoding.validate()?;

    let mut seen = BTreeSet::new();
    for admitted in &input.observations {
        let event_id = admitted.attempt.event_id.clone();
        admitted
            .attempt
            .validate()
            .map_err(BktProjectionError::AttemptContract)?;

        if !matches!(
            &admitted.attempt.completeness,
            AttemptEvidenceCompleteness::Complete
        ) {
            return Err(BktProjectionError::IncompleteAttempt(event_id));
        }
        if admitted.attempt.learner_id != input.learner_id {
            return Err(BktProjectionError::LearnerMismatch(event_id));
        }
        if admitted.attempt.capability_id != input.capability_id {
            return Err(BktProjectionError::CapabilityMismatch(event_id));
        }
        if admitted.attempt.dimension.as_ref() != Some(&input.dimension) {
            return Err(BktProjectionError::DimensionMismatch(event_id));
        }
        if admitted.admission.validate().is_err() {
            return Err(BktProjectionError::AdmissionProfileMismatch(event_id));
        }
        if admitted.admission.event_id != admitted.attempt.event_id {
            return Err(BktProjectionError::AdmissionEventMismatch(event_id));
        }
        if admitted.admission.profile_id != input.admission_profile_id
            || admitted.admission.profile_version != input.admission_profile_version
        {
            return Err(BktProjectionError::AdmissionProfileMismatch(event_id));
        }
        if !matches!(&admitted.admission.outcome, AdmissionOutcome::Admitted) {
            return Err(BktProjectionError::ObservationNotAdmitted(event_id));
        }
        if !seen.insert(admitted.attempt.event_id.clone()) {
            return Err(BktProjectionError::DuplicateEvent(event_id));
        }
    }

    input.observations.sort_by(|left, right| {
        left.attempt
            .observed_at
            .cmp(&right.attempt.observed_at)
            .then_with(|| left.attempt.event_id.0.cmp(&right.attempt.event_id.0))
    });

    let parameter_digest = parameter_digest(&input.parameters, &input.encoding);
    let mut state = input.parameters.prior_permille;
    let mut ordered_event_ids = Vec::with_capacity(input.observations.len());

    for admitted in &input.observations {
        let correct = input.encoding.encode(&admitted.attempt.outcome)?;
        state = bkt_update_integer(
            state,
            correct,
            &input.parameters,
            &admitted.attempt.event_id,
        )?;
        ordered_event_ids.push(admitted.attempt.event_id.clone());
    }

    let support_confidence = (input.observations.len() as u64)
        .saturating_mul(input.parameters.support_confidence_step_permille as u64)
        .min(1000) as u16;

    let estimate = AdvisoryCapabilityEstimate {
        learner_id: input.learner_id,
        capability_id: input.capability_id,
        estimator: EstimatorProvenance {
            estimator_id: BKT_ESTIMATOR_ID.to_string(),
            estimator_version: BKT_ESTIMATOR_VERSION.to_string(),
            parameters_digest: Some(parameter_digest),
        },
        admission_profile_id: input.admission_profile_id,
        admission_profile_version: input.admission_profile_version,
        input_event_ids: ordered_event_ids.clone(),
        dimensions: vec![DimensionEstimate {
            dimension: input.dimension,
            estimate_permille: state,
            confidence_permille: support_confidence,
        }],
        generated_at: input.generated_at,
    };

    Ok(BktProjectionReceipt {
        estimate,
        ordered_event_ids,
        parameters: input.parameters,
        encoding: input.encoding,
    })
}

/// Integer-only BKT update. This avoids native/WASM floating-point drift and uses
/// round-half-up at each probability normalization boundary.
fn bkt_update_integer(
    prior_permille: u16,
    correct: bool,
    parameters: &BktParameters,
    event_id: &EvidenceEventId,
) -> Result<u16, BktProjectionError> {
    let prior = prior_permille as u64;
    let unknown = 1000u64 - prior;

    let known_likelihood = if correct {
        1000u64 - parameters.slip_permille as u64
    } else {
        parameters.slip_permille as u64
    };
    let unknown_likelihood = if correct {
        parameters.guess_permille as u64
    } else {
        1000u64 - parameters.guess_permille as u64
    };

    let known_weight = known_likelihood * prior;
    let unknown_weight = unknown_likelihood * unknown;
    let denominator = known_weight + unknown_weight;
    if denominator == 0 {
        return Err(BktProjectionError::ImpossibleObservationLikelihood(
            event_id.clone(),
        ));
    }

    let posterior = div_round_half_up(known_weight * 1000, denominator);
    let learning_gain = div_round_half_up(
        (1000 - posterior) * parameters.learn_permille as u64,
        1000,
    );
    let updated = (posterior + learning_gain).min(1000);
    Ok(updated as u16)
}

fn div_round_half_up(numerator: u64, denominator: u64) -> u64 {
    (numerator + denominator / 2) / denominator
}

fn parameter_digest(parameters: &BktParameters, encoding: &BktObservationEncoding) -> String {
    let canonical = format!(
        "praxis-bkt-integer-v1|prior={}|learn={}|guess={}|slip={}|support-confidence-step={}|{}",
        parameters.prior_permille,
        parameters.learn_permille,
        parameters.guess_permille,
        parameters.slip_permille,
        parameters.support_confidence_step_permille,
        encoding.canonical_fragment(),
    );
    format!("blake3:{}", hash_to_string(canonical.as_bytes()))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::learning_evidence::{AssistanceKind, EvidenceProvenance, EvidenceSource};

    fn attempt(
        event_id: &str,
        observed_at: i64,
        outcome: AttemptOutcome,
    ) -> AdmittedBktAttempt {
        let attempt = AttemptEvidence {
            event_id: EvidenceEventId(event_id.into()),
            learner_id: LearnerId("learner-1".into()),
            capability_id: CapabilityId("rust:ownership".into()),
            source_activity_id: Some(format!("task:{event_id}")),
            source: EvidenceSource::Assessment,
            dimension: Some(CapabilityDimension::Application),
            outcome,
            response_time_ms: Some(1_000),
            assistance: AssistanceKind::Unassisted,
            provenance: EvidenceProvenance {
                producer_id: "praxis-test".into(),
                producer_version: Some("1".into()),
                source_record_id: Some(format!("record:{event_id}")),
                artifact_digest: None,
            },
            completeness: AttemptEvidenceCompleteness::Complete,
            observed_at,
        };
        let admission = EvidenceAdmissionDecision {
            event_id: attempt.event_id.clone(),
            profile_id: "practice-bkt".into(),
            profile_version: "1".into(),
            outcome: AdmissionOutcome::Admitted,
            decided_at: observed_at + 1,
        };
        AdmittedBktAttempt { attempt, admission }
    }

    fn input(observations: Vec<AdmittedBktAttempt>) -> BktProjectionInput {
        BktProjectionInput {
            learner_id: LearnerId("learner-1".into()),
            capability_id: CapabilityId("rust:ownership".into()),
            dimension: CapabilityDimension::Application,
            admission_profile_id: "practice-bkt".into(),
            admission_profile_version: "1".into(),
            parameters: BktParameters {
                prior_permille: 500,
                learn_permille: 100,
                guess_permille: 200,
                slip_permille: 100,
                support_confidence_step_permille: 50,
            },
            encoding: BktObservationEncoding::BinaryOnly,
            observations,
            generated_at: 1_700_001_000,
        }
    }

    #[test]
    fn integer_bkt_has_frozen_regression_vectors() {
        let parameters = BktParameters {
            prior_permille: 500,
            learn_permille: 100,
            guess_permille: 200,
            slip_permille: 100,
            support_confidence_step_permille: 50,
        };
        assert_eq!(
            bkt_update_integer(
                500,
                true,
                &parameters,
                &EvidenceEventId("correct".into())
            ),
            Ok(836)
        );
        assert_eq!(
            bkt_update_integer(
                500,
                false,
                &parameters,
                &EvidenceEventId("incorrect".into())
            ),
            Ok(200)
        );
    }

    #[test]
    fn input_order_does_not_change_projection() {
        let early = attempt("event-a", 10, AttemptOutcome::Binary { correct: true });
        let late = attempt("event-b", 20, AttemptOutcome::Binary { correct: false });

        let forward = project_bkt(input(vec![early.clone(), late.clone()])).unwrap();
        let reversed = project_bkt(input(vec![late, early])).unwrap();

        assert_eq!(forward.estimate, reversed.estimate);
        assert_eq!(forward.ordered_event_ids, reversed.ordered_event_ids);
        assert_eq!(
            forward.ordered_event_ids,
            vec![
                EvidenceEventId("event-a".into()),
                EvidenceEventId("event-b".into())
            ]
        );
    }

    #[test]
    fn tie_break_order_is_event_id_not_input_order() {
        let a = attempt("a", 10, AttemptOutcome::Binary { correct: true });
        let b = attempt("b", 10, AttemptOutcome::Binary { correct: false });
        let receipt = project_bkt(input(vec![b, a])).unwrap();
        assert_eq!(
            receipt.ordered_event_ids,
            vec![EvidenceEventId("a".into()), EvidenceEventId("b".into())]
        );
    }

    #[test]
    fn rejected_observation_cannot_enter_projection() {
        let mut observation = attempt("event-a", 10, AttemptOutcome::Binary { correct: true });
        observation.admission.outcome = AdmissionOutcome::Rejected {
            reason: "not admissible".into(),
        };
        assert_eq!(
            project_bkt(input(vec![observation])),
            Err(BktProjectionError::ObservationNotAdmitted(EvidenceEventId(
                "event-a".into()
            )))
        );
    }

    #[test]
    fn incomplete_legacy_attempt_cannot_enter_projection() {
        let legacy = AttemptEvidence::from_legacy_record_attempt(
            EvidenceEventId("legacy".into()),
            LearnerId("learner-1".into()),
            CapabilityId("rust:ownership".into()),
            true,
            700,
            10,
            "1",
        );
        let admitted = AdmittedBktAttempt {
            admission: EvidenceAdmissionDecision {
                event_id: legacy.event_id.clone(),
                profile_id: "practice-bkt".into(),
                profile_version: "1".into(),
                outcome: AdmissionOutcome::Admitted,
                decided_at: 11,
            },
            attempt: legacy,
        };
        assert_eq!(
            project_bkt(input(vec![admitted])),
            Err(BktProjectionError::IncompleteAttempt(EvidenceEventId(
                "legacy".into()
            )))
        );
    }

    #[test]
    fn scored_observation_requires_explicit_threshold() {
        let scored = attempt(
            "score",
            10,
            AttemptOutcome::Scored {
                score_permille: 800,
            },
        );
        assert_eq!(
            project_bkt(input(vec![scored])),
            Err(BktProjectionError::ScoredObservationRequiresThreshold)
        );
    }

    #[test]
    fn explicit_scored_threshold_is_bound_into_parameters_digest() {
        let scored = attempt(
            "score",
            10,
            AttemptOutcome::Scored {
                score_permille: 800,
            },
        );
        let mut threshold_input = input(vec![scored]);
        threshold_input.encoding = BktObservationEncoding::ScoredThreshold {
            correct_at_or_above_permille: 750,
        };
        let threshold_receipt = project_bkt(threshold_input).unwrap();

        let binary_receipt = project_bkt(input(vec![attempt(
            "binary",
            10,
            AttemptOutcome::Binary { correct: true },
        )]))
        .unwrap();

        assert_ne!(
            threshold_receipt.estimate.estimator.parameters_digest,
            binary_receipt.estimate.estimator.parameters_digest
        );
    }

    #[test]
    fn duplicate_event_is_rejected() {
        let first = attempt("same", 10, AttemptOutcome::Binary { correct: true });
        let second = attempt("same", 20, AttemptOutcome::Binary { correct: false });
        assert_eq!(
            project_bkt(input(vec![first, second])),
            Err(BktProjectionError::DuplicateEvent(EvidenceEventId(
                "same".into()
            )))
        );
    }

    #[test]
    fn projection_is_dimension_specific() {
        let mut observation = attempt("event-a", 10, AttemptOutcome::Binary { correct: true });
        observation.attempt.dimension = Some(CapabilityDimension::Transfer);
        assert_eq!(
            project_bkt(input(vec![observation])),
            Err(BktProjectionError::DimensionMismatch(EvidenceEventId(
                "event-a".into()
            )))
        );
    }

    #[test]
    fn projection_remains_advisory() {
        let receipt = project_bkt(input(vec![attempt(
            "event-a",
            10,
            AttemptOutcome::Binary { correct: true },
        )]))
        .unwrap();
        assert!(!receipt.grants_credential_authority());
        assert!(!receipt.estimate.grants_credential_authority());
        assert_eq!(receipt.estimate.dimensions.len(), 1);
        assert_eq!(
            receipt.estimate.estimator.estimator_version,
            BKT_ESTIMATOR_VERSION
        );
    }
}
