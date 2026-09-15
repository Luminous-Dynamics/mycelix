// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! AC-002 capture observation and signal contract.
//!
//! This module defines a deliberately non-adjudicative epistemic pipeline for
//! institutional capture analysis. It measures institutions and processes, not
//! human worth. Observations and signals can justify review; they cannot, by
//! themselves, authorize a rights-affecting consequence.

use serde::{Deserialize, Serialize};

use crate::anti_capture::ConsequenceBasis;

/// Institutional/system subject that can be evaluated for capture risk.
///
/// There is intentionally no `Person` variant. AC-002 measures processes,
/// institutions, markets, contracting procedures, authority graphs, and
/// aggregates rather than assigning individuals a capture or trust score.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum CaptureSubject {
    Institution(String),
    Process(String),
    Market(String),
    ContractingProcedure(String),
    AuthorityGraph(String),
    Aggregate(String),
}

impl CaptureSubject {
    fn label(&self) -> &str {
        match self {
            Self::Institution(value)
            | Self::Process(value)
            | Self::Market(value)
            | Self::ContractingProcedure(value)
            | Self::AuthorityGraph(value)
            | Self::Aggregate(value) => value,
        }
    }
}

/// Initial family of interpretable anti-capture metrics.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum CaptureMetric {
    AuthorityConcentration,
    DelegationConcentration,
    InfluenceOpacity,
    OwnershipConcentration,
    ProcurementSupplierConcentration,
    ProcurementSingleBidShare,
    EvidenceDeficit,
    ContestabilityDeficit,
    ExitDeficit,
    Custom(String),
}

/// Deterministic fixed-ratio measurement value.
///
/// Avoiding floating point here keeps serialized evidence reproducible. For
/// example, 2350 / 10000 with unit `share` represents 23.50%.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct MetricValue {
    pub numerator: i64,
    pub denominator: u64,
    pub unit: String,
}

/// Measurement together with the exact method/algorithm reference used.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct Measurement {
    pub metric: CaptureMetric,
    pub value: MetricValue,
    pub method_ref: String,
}

/// Source provenance for an observation.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct ProvenanceRef {
    pub source_ref: String,
    pub content_hash: Option<String>,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum ConfidenceLevel {
    Low,
    Moderate,
    High,
}

/// Confidence semantics are explicit so a qualitative judgment cannot silently
/// masquerade as a calibrated probability.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum ConfidenceAssessment {
    /// Empirical confidence expressed in basis points, 0..=10_000.
    EmpiricalBps { bps: u16, basis: String },
    Qualitative {
        level: ConfidenceLevel,
        basis: String,
    },
    Unknown { reason: String },
}

/// Every observation declares uncertainty and limitations.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct Uncertainty {
    pub confidence: ConfidenceAssessment,
    pub limitations: Vec<String>,
}

/// A directly measured observation. It is not a finding of wrongdoing.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct CaptureObservation {
    pub id: String,
    pub subject: CaptureSubject,
    pub measurement: Measurement,
    pub provenance: Vec<ProvenanceRef>,
    pub uncertainty: Uncertainty,
    pub observed_at: u64,
}

impl CaptureObservation {
    /// AC-001 classifies an observation as non-adjudicative by construction.
    pub fn constitutional_basis(&self) -> ConsequenceBasis {
        ConsequenceBasis::Observation
    }
}

/// A plausible non-capture explanation that must remain visible during review.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct AlternativeExplanation {
    pub statement: String,
    pub evidence_needed: Vec<String>,
}

/// An explanatory hypothesis grounded in observations.
///
/// Hypotheses must include at least one alternative explanation. AC-002 therefore
/// cannot encode `high concentration => corruption` as an unquestioned inference.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct CaptureHypothesis {
    pub id: String,
    pub subject: CaptureSubject,
    pub statement: String,
    pub supporting_observation_refs: Vec<String>,
    pub alternative_explanations: Vec<AlternativeExplanation>,
}

/// Human-review lifecycle for a signal. None of these states is an adjudication.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum ReviewState {
    Unreviewed,
    UnderReview,
    SupportedForFurtherInquiry,
    NeedsMoreEvidence,
    Rejected,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct SignalReview {
    pub state: ReviewState,
    pub reviewer_refs: Vec<String>,
    pub rationale: Option<String>,
}

/// A review-priority signal produced from measurements/hypotheses.
///
/// `authorizes_consequence` is required to remain false. It is explicit rather
/// than implicit so serialized integrations cannot quietly reinterpret a signal
/// as a finding with enforcement authority.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct CaptureSignal {
    pub id: String,
    pub subject: CaptureSubject,
    pub indicator_ref: String,
    pub observation_refs: Vec<String>,
    pub hypothesis_refs: Vec<String>,
    pub review: SignalReview,
    pub authorizes_consequence: bool,
}

impl CaptureSignal {
    /// AC-001 classifies every AC-002 signal as non-adjudicative.
    pub fn constitutional_basis(&self) -> ConsequenceBasis {
        ConsequenceBasis::CaptureSignal
    }
}

/// Fail-closed AC-002 contract violations.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum CaptureContractViolation {
    MissingId,
    MissingSubject,
    MissingMetricName,
    InvalidMeasurementDenominator,
    MissingMeasurementUnit,
    MissingMethodReference,
    MissingProvenance,
    InvalidProvenanceReference,
    InvalidContentHash,
    ConfidenceOutOfRange,
    MissingConfidenceBasis,
    MissingUncertaintyLimitations,
    InvalidUncertaintyLimitation,
    MissingHypothesisStatement,
    MissingSupportingObservations,
    MissingAlternativeExplanation,
    InvalidAlternativeExplanation,
    MissingAlternativeEvidenceNeed,
    MissingIndicatorReference,
    MissingSignalObservations,
    SignalCannotAuthorizeConsequence,
    ReviewMissingReviewer,
    ReviewMissingRationale,
    InvalidReference,
}

/// Stateless validator for the AC-002 epistemic boundary.
#[derive(Debug, Default, Clone, Copy)]
pub struct CaptureContract;

impl CaptureContract {
    pub fn validate_observation(
        observation: &CaptureObservation,
    ) -> Result<(), Vec<CaptureContractViolation>> {
        let mut violations = Vec::new();

        validate_id(&observation.id, &mut violations);
        validate_subject(&observation.subject, &mut violations);
        validate_measurement(&observation.measurement, &mut violations);

        if observation.provenance.is_empty() {
            violations.push(CaptureContractViolation::MissingProvenance);
        }
        for provenance in &observation.provenance {
            if provenance.source_ref.trim().is_empty() {
                violations.push(CaptureContractViolation::InvalidProvenanceReference);
            }
            if provenance
                .content_hash
                .as_ref()
                .is_some_and(|hash| hash.trim().is_empty())
            {
                violations.push(CaptureContractViolation::InvalidContentHash);
            }
        }

        validate_uncertainty(&observation.uncertainty, &mut violations);
        finish(violations)
    }

    pub fn validate_hypothesis(
        hypothesis: &CaptureHypothesis,
    ) -> Result<(), Vec<CaptureContractViolation>> {
        let mut violations = Vec::new();

        validate_id(&hypothesis.id, &mut violations);
        validate_subject(&hypothesis.subject, &mut violations);
        if hypothesis.statement.trim().is_empty() {
            violations.push(CaptureContractViolation::MissingHypothesisStatement);
        }
        if hypothesis.supporting_observation_refs.is_empty() {
            violations.push(CaptureContractViolation::MissingSupportingObservations);
        }
        validate_refs(&hypothesis.supporting_observation_refs, &mut violations);

        if hypothesis.alternative_explanations.is_empty() {
            violations.push(CaptureContractViolation::MissingAlternativeExplanation);
        }
        for alternative in &hypothesis.alternative_explanations {
            if alternative.statement.trim().is_empty() {
                violations.push(CaptureContractViolation::InvalidAlternativeExplanation);
            }
            if alternative.evidence_needed.is_empty() {
                violations.push(CaptureContractViolation::MissingAlternativeEvidenceNeed);
            }
            validate_refs(&alternative.evidence_needed, &mut violations);
        }

        finish(violations)
    }

    pub fn validate_signal(
        signal: &CaptureSignal,
    ) -> Result<(), Vec<CaptureContractViolation>> {
        let mut violations = Vec::new();

        validate_id(&signal.id, &mut violations);
        validate_subject(&signal.subject, &mut violations);
        if signal.indicator_ref.trim().is_empty() {
            violations.push(CaptureContractViolation::MissingIndicatorReference);
        }
        if signal.observation_refs.is_empty() {
            violations.push(CaptureContractViolation::MissingSignalObservations);
        }
        validate_refs(&signal.observation_refs, &mut violations);
        validate_refs(&signal.hypothesis_refs, &mut violations);

        if signal.authorizes_consequence {
            violations.push(CaptureContractViolation::SignalCannotAuthorizeConsequence);
        }

        validate_review(&signal.review, &mut violations);
        finish(violations)
    }
}

fn validate_id(id: &str, violations: &mut Vec<CaptureContractViolation>) {
    if id.trim().is_empty() {
        violations.push(CaptureContractViolation::MissingId);
    }
}

fn validate_subject(subject: &CaptureSubject, violations: &mut Vec<CaptureContractViolation>) {
    if subject.label().trim().is_empty() {
        violations.push(CaptureContractViolation::MissingSubject);
    }
}

fn validate_measurement(
    measurement: &Measurement,
    violations: &mut Vec<CaptureContractViolation>,
) {
    if matches!(&measurement.metric, CaptureMetric::Custom(name) if name.trim().is_empty()) {
        violations.push(CaptureContractViolation::MissingMetricName);
    }
    if measurement.value.denominator == 0 {
        violations.push(CaptureContractViolation::InvalidMeasurementDenominator);
    }
    if measurement.value.unit.trim().is_empty() {
        violations.push(CaptureContractViolation::MissingMeasurementUnit);
    }
    if measurement.method_ref.trim().is_empty() {
        violations.push(CaptureContractViolation::MissingMethodReference);
    }
}

fn validate_uncertainty(
    uncertainty: &Uncertainty,
    violations: &mut Vec<CaptureContractViolation>,
) {
    match &uncertainty.confidence {
        ConfidenceAssessment::EmpiricalBps { bps, basis } => {
            if *bps > 10_000 {
                violations.push(CaptureContractViolation::ConfidenceOutOfRange);
            }
            if basis.trim().is_empty() {
                violations.push(CaptureContractViolation::MissingConfidenceBasis);
            }
        }
        ConfidenceAssessment::Qualitative { basis, .. } => {
            if basis.trim().is_empty() {
                violations.push(CaptureContractViolation::MissingConfidenceBasis);
            }
        }
        ConfidenceAssessment::Unknown { reason } => {
            if reason.trim().is_empty() {
                violations.push(CaptureContractViolation::MissingConfidenceBasis);
            }
        }
    }

    if uncertainty.limitations.is_empty() {
        violations.push(CaptureContractViolation::MissingUncertaintyLimitations);
    }
    if uncertainty
        .limitations
        .iter()
        .any(|limitation| limitation.trim().is_empty())
    {
        violations.push(CaptureContractViolation::InvalidUncertaintyLimitation);
    }
}

fn validate_review(review: &SignalReview, violations: &mut Vec<CaptureContractViolation>) {
    let reviewed = review.state != ReviewState::Unreviewed;
    if reviewed && review.reviewer_refs.is_empty() {
        violations.push(CaptureContractViolation::ReviewMissingReviewer);
    }
    validate_refs(&review.reviewer_refs, violations);

    let rationale_required = matches!(
        review.state,
        ReviewState::SupportedForFurtherInquiry
            | ReviewState::NeedsMoreEvidence
            | ReviewState::Rejected
    );
    if rationale_required
        && review
            .rationale
            .as_ref()
            .is_none_or(|rationale| rationale.trim().is_empty())
    {
        violations.push(CaptureContractViolation::ReviewMissingRationale);
    }
}

fn validate_refs(refs: &[String], violations: &mut Vec<CaptureContractViolation>) {
    if refs.iter().any(|reference| reference.trim().is_empty()) {
        violations.push(CaptureContractViolation::InvalidReference);
    }
}

fn finish(
    violations: Vec<CaptureContractViolation>,
) -> Result<(), Vec<CaptureContractViolation>> {
    if violations.is_empty() {
        Ok(())
    } else {
        Err(violations)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn observation() -> CaptureObservation {
        CaptureObservation {
            id: "obs:single-bid-share:2026-q3".into(),
            subject: CaptureSubject::Process("municipal-procurement".into()),
            measurement: Measurement {
                metric: CaptureMetric::ProcurementSingleBidShare,
                value: MetricValue {
                    numerator: 2_350,
                    denominator: 10_000,
                    unit: "share".into(),
                },
                method_ref: "ocp:red-flag:single-bid:v1".into(),
            },
            provenance: vec![ProvenanceRef {
                source_ref: "ocds:release-package:2026-q3".into(),
                content_hash: Some("sha256:abc123".into()),
            }],
            uncertainty: Uncertainty {
                confidence: ConfidenceAssessment::Qualitative {
                    level: ConfidenceLevel::Moderate,
                    basis: "coverage audit found missing bid data in two procedures".into(),
                },
                limitations: vec!["emergency procedures are included in the denominator".into()],
            },
            observed_at: 1_789_000_000,
        }
    }

    fn hypothesis() -> CaptureHypothesis {
        CaptureHypothesis {
            id: "hyp:competition-risk:q3".into(),
            subject: CaptureSubject::Process("municipal-procurement".into()),
            statement: "competition may be weaker than the historical baseline".into(),
            supporting_observation_refs: vec!["obs:single-bid-share:2026-q3".into()],
            alternative_explanations: vec![AlternativeExplanation {
                statement: "the quarter contained unusually specialized emergency purchases".into(),
                evidence_needed: vec!["dataset:procedure-classification:2026-q3".into()],
            }],
        }
    }

    fn signal() -> CaptureSignal {
        CaptureSignal {
            id: "signal:competition-risk:q3".into(),
            subject: CaptureSubject::Process("municipal-procurement".into()),
            indicator_ref: "indicator:single-bid-share".into(),
            observation_refs: vec!["obs:single-bid-share:2026-q3".into()],
            hypothesis_refs: vec!["hyp:competition-risk:q3".into()],
            review: SignalReview {
                state: ReviewState::Unreviewed,
                reviewer_refs: vec![],
                rationale: None,
            },
            authorizes_consequence: false,
        }
    }

    #[test]
    fn valid_observation_is_accepted_and_remains_non_adjudicative() {
        let observation = observation();
        assert_eq!(CaptureContract::validate_observation(&observation), Ok(()));
        assert_eq!(
            observation.constitutional_basis(),
            ConsequenceBasis::Observation
        );
    }

    #[test]
    fn observation_requires_reproducible_measurement_and_provenance() {
        let mut observation = observation();
        observation.measurement.value.denominator = 0;
        observation.measurement.method_ref.clear();
        observation.provenance.clear();

        let errors = CaptureContract::validate_observation(&observation)
            .expect_err("unreproducible observation must fail closed");
        assert!(errors.contains(&CaptureContractViolation::InvalidMeasurementDenominator));
        assert!(errors.contains(&CaptureContractViolation::MissingMethodReference));
        assert!(errors.contains(&CaptureContractViolation::MissingProvenance));
    }

    #[test]
    fn observation_requires_declared_uncertainty() {
        let mut observation = observation();
        observation.uncertainty.limitations.clear();
        observation.uncertainty.confidence = ConfidenceAssessment::EmpiricalBps {
            bps: 10_001,
            basis: String::new(),
        };

        let errors = CaptureContract::validate_observation(&observation)
            .expect_err("false precision must fail closed");
        assert!(errors.contains(&CaptureContractViolation::ConfidenceOutOfRange));
        assert!(errors.contains(&CaptureContractViolation::MissingConfidenceBasis));
        assert!(errors.contains(&CaptureContractViolation::MissingUncertaintyLimitations));
    }

    #[test]
    fn hypothesis_requires_observations_and_alternative_explanations() {
        assert_eq!(CaptureContract::validate_hypothesis(&hypothesis()), Ok(()));

        let mut hypothesis = hypothesis();
        hypothesis.supporting_observation_refs.clear();
        hypothesis.alternative_explanations.clear();
        let errors = CaptureContract::validate_hypothesis(&hypothesis)
            .expect_err("one-way accusation must fail closed");
        assert!(errors.contains(&CaptureContractViolation::MissingSupportingObservations));
        assert!(errors.contains(&CaptureContractViolation::MissingAlternativeExplanation));
    }

    #[test]
    fn capture_signal_can_never_authorize_a_consequence() {
        let mut signal = signal();
        signal.authorizes_consequence = true;
        assert_eq!(
            CaptureContract::validate_signal(&signal),
            Err(vec![CaptureContractViolation::SignalCannotAuthorizeConsequence])
        );
        assert_eq!(signal.constitutional_basis(), ConsequenceBasis::CaptureSignal);
    }

    #[test]
    fn completed_review_requires_reviewer_and_rationale() {
        let mut signal = signal();
        signal.review.state = ReviewState::SupportedForFurtherInquiry;
        let errors = CaptureContract::validate_signal(&signal)
            .expect_err("anonymous unexplained review must fail closed");
        assert!(errors.contains(&CaptureContractViolation::ReviewMissingReviewer));
        assert!(errors.contains(&CaptureContractViolation::ReviewMissingRationale));

        signal.review.reviewer_refs = vec!["reviewer:independent-audit-7".into()];
        signal.review.rationale = Some("pattern merits procurement-audit review".into());
        assert_eq!(CaptureContract::validate_signal(&signal), Ok(()));
    }

    #[test]
    fn signal_requires_observations_even_when_a_hypothesis_exists() {
        let mut signal = signal();
        signal.observation_refs.clear();
        let errors = CaptureContract::validate_signal(&signal)
            .expect_err("signal without measured observation must fail closed");
        assert!(errors.contains(&CaptureContractViolation::MissingSignalObservations));
    }

    #[test]
    fn custom_metrics_require_names() {
        let mut observation = observation();
        observation.measurement.metric = CaptureMetric::Custom("  ".into());
        assert!(
            CaptureContract::validate_observation(&observation)
                .expect_err("anonymous metric must fail closed")
                .contains(&CaptureContractViolation::MissingMetricName)
        );
    }
}
