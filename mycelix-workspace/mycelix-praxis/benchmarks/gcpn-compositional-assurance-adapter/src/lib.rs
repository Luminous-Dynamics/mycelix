//! Executable synthetic adapter for the GCPN compositional-assurance engine.
//!
//! This crate deliberately distinguishes faults represented by the six-stage
//! engine from dimensions that its current contract cannot express. Unsupported
//! faults fail closed rather than being approximated by a semantically different
//! field (for example, a decision record is not an action receipt).

use std::collections::BTreeSet;

use gcpn_compositional_assurance::{
    AuditReport, CapReceiptLink, CausalClaim, ClaimDisposition, ClaimRequirement, EvidenceValidity,
    ForecastTiming, PairedComparison, PopulationAccounting, PostProcessingFacts, Scenario,
    ScenarioFacts, Stage, StageEvidence, VerificationAccounting, ViolationCode,
};

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum Fault {
    ForecastLate,
    ForecastStale,
    ObservationMissing,
    SharedSourceLineage,
    AuthorityMismatch,
    CapMessageIdentityMismatch,
    ReceiverDenominatorMissing,
    DeliveryReceiptIncomplete,
    AccessibilityUnverified,
    ActionReceiptMissing,
    OutcomeUnascertained,
    PostCutoffEvidence,
    HindsightLeakage,
    DuplicateCaseKey,
    ValidContradictoryEvidence,
    ClaimOutOfScope,
    SourceUnavailable,
}

impl Fault {
    pub const ALL: [Self; 17] = [
        Self::ForecastLate,
        Self::ForecastStale,
        Self::ObservationMissing,
        Self::SharedSourceLineage,
        Self::AuthorityMismatch,
        Self::CapMessageIdentityMismatch,
        Self::ReceiverDenominatorMissing,
        Self::DeliveryReceiptIncomplete,
        Self::AccessibilityUnverified,
        Self::ActionReceiptMissing,
        Self::OutcomeUnascertained,
        Self::PostCutoffEvidence,
        Self::HindsightLeakage,
        Self::DuplicateCaseKey,
        Self::ValidContradictoryEvidence,
        Self::ClaimOutOfScope,
        Self::SourceUnavailable,
    ];

    pub const SUPPORTED: [Self; 12] = [
        Self::ForecastLate,
        Self::ForecastStale,
        Self::ObservationMissing,
        Self::SharedSourceLineage,
        Self::CapMessageIdentityMismatch,
        Self::ReceiverDenominatorMissing,
        Self::OutcomeUnascertained,
        Self::PostCutoffEvidence,
        Self::HindsightLeakage,
        Self::ValidContradictoryEvidence,
        Self::ClaimOutOfScope,
        Self::SourceUnavailable,
    ];

    pub const NOT_REPRESENTED: [Self; 5] = [
        Self::AuthorityMismatch,
        Self::DeliveryReceiptIncomplete,
        Self::AccessibilityUnverified,
        Self::ActionReceiptMissing,
        Self::DuplicateCaseKey,
    ];

    pub const fn label(self) -> &'static str {
        match self {
            Self::ForecastLate => "forecast_late",
            Self::ForecastStale => "forecast_stale",
            Self::ObservationMissing => "observation_missing",
            Self::SharedSourceLineage => "shared_source_lineage",
            Self::AuthorityMismatch => "authority_mismatch",
            Self::CapMessageIdentityMismatch => "cap_message_identity_mismatch",
            Self::ReceiverDenominatorMissing => "receiver_denominator_missing",
            Self::DeliveryReceiptIncomplete => "delivery_receipt_incomplete",
            Self::AccessibilityUnverified => "accessibility_unverified",
            Self::ActionReceiptMissing => "action_receipt_missing",
            Self::OutcomeUnascertained => "outcome_unascertained",
            Self::PostCutoffEvidence => "post_cutoff_evidence",
            Self::HindsightLeakage => "hindsight_leakage",
            Self::DuplicateCaseKey => "duplicate_case_key",
            Self::ValidContradictoryEvidence => "valid_contradictory_evidence",
            Self::ClaimOutOfScope => "claim_out_of_scope",
            Self::SourceUnavailable => "source_unavailable",
        }
    }

    pub const fn supported(self) -> bool {
        matches!(
            self,
            Self::ForecastLate
                | Self::ForecastStale
                | Self::ObservationMissing
                | Self::SharedSourceLineage
                | Self::CapMessageIdentityMismatch
                | Self::ReceiverDenominatorMissing
                | Self::OutcomeUnascertained
                | Self::PostCutoffEvidence
                | Self::HindsightLeakage
                | Self::ValidContradictoryEvidence
                | Self::ClaimOutOfScope
                | Self::SourceUnavailable
        )
    }

    pub const fn affected_claim(self) -> Option<(Stage, &'static str)> {
        match self {
            Self::ForecastLate | Self::HindsightLeakage => {
                Some((Stage::Decision, "decision-valid"))
            }
            Self::ForecastStale | Self::PostCutoffEvidence => {
                Some((Stage::Forecast, "forecast-valid"))
            }
            Self::ObservationMissing
            | Self::SharedSourceLineage
            | Self::ValidContradictoryEvidence => {
                Some((Stage::Observation, "reference-valid"))
            }
            Self::CapMessageIdentityMismatch => Some((Stage::CapLifecycle, "receipt-valid")),
            Self::ReceiverDenominatorMissing => Some((Stage::PopulationReach, "reach-valid")),
            Self::OutcomeUnascertained
            | Self::ClaimOutOfScope
            | Self::SourceUnavailable => {
                Some((Stage::AfterEventReview, "review-valid"))
            }
            Self::AuthorityMismatch
            | Self::DeliveryReceiptIncomplete
            | Self::AccessibilityUnverified
            | Self::ActionReceiptMissing
            | Self::DuplicateCaseKey => None,
        }
    }

    pub const fn expected_disposition(self) -> Option<ClaimDisposition> {
        match self {
            Self::ForecastLate
            | Self::ForecastStale
            | Self::SharedSourceLineage
            | Self::CapMessageIdentityMismatch
            | Self::ReceiverDenominatorMissing
            | Self::PostCutoffEvidence
            | Self::HindsightLeakage => Some(ClaimDisposition::Indeterminate),
            Self::ObservationMissing | Self::OutcomeUnascertained | Self::SourceUnavailable => {
                Some(ClaimDisposition::Unavailable)
            }
            Self::ValidContradictoryEvidence => Some(ClaimDisposition::Refutes),
            Self::ClaimOutOfScope => Some(ClaimDisposition::OutOfScope),
            Self::AuthorityMismatch
            | Self::DeliveryReceiptIncomplete
            | Self::AccessibilityUnverified
            | Self::ActionReceiptMissing
            | Self::DuplicateCaseKey => None,
        }
    }

    pub const fn expected_violation(self) -> Option<ViolationCode> {
        match self {
            Self::ForecastLate => Some(ViolationCode::ForecastIssuedAfterDecisionCutoff),
            Self::ForecastStale => Some(ViolationCode::StaleEvidence),
            Self::ObservationMissing | Self::OutcomeUnascertained => {
                Some(ViolationCode::ClaimEvidenceMissing)
            }
            Self::SharedSourceLineage => Some(ViolationCode::CommonCauseLineage),
            Self::CapMessageIdentityMismatch => {
                Some(ViolationCode::CapReceiptMessageMismatch)
            }
            Self::ReceiverDenominatorMissing => {
                Some(ViolationCode::PopulationDenominatorUnknown)
            }
            Self::PostCutoffEvidence => Some(ViolationCode::EvidenceAfterAsOf),
            Self::HindsightLeakage => Some(ViolationCode::PostprocessingOutcomeLeakage),
            Self::ValidContradictoryEvidence
            | Self::ClaimOutOfScope
            | Self::SourceUnavailable
            | Self::AuthorityMismatch
            | Self::DeliveryReceiptIncomplete
            | Self::AccessibilityUnverified
            | Self::ActionReceiptMissing
            | Self::DuplicateCaseKey => None,
        }
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum AdapterError {
    UnsupportedFault(Fault),
    DuplicateFault(Fault),
    MissingClaim { stage: Stage, claim_id: &'static str },
    AuditFailed(String),
}

impl std::fmt::Display for AdapterError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::UnsupportedFault(fault) => {
                write!(formatter, "fault is not represented by current engine: {}", fault.label())
            }
            Self::DuplicateFault(fault) => {
                write!(formatter, "duplicate fault requested: {}", fault.label())
            }
            Self::MissingClaim { stage, claim_id } => {
                write!(formatter, "audit omitted expected claim {}:{}", stage.label(), claim_id)
            }
            Self::AuditFailed(message) => write!(formatter, "compositional audit failed: {message}"),
        }
    }
}

impl std::error::Error for AdapterError {}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SingleObservation {
    pub fault: Fault,
    pub stage: Stage,
    pub claim_id: &'static str,
    pub expected: ClaimDisposition,
    pub observed: ClaimDisposition,
    pub expected_violation: Option<ViolationCode>,
    pub violation_detected: bool,
    pub report_fingerprint: String,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct PairObservation {
    pub faults: [Fault; 2],
    pub affected_claim_dispositions: Vec<ClaimDisposition>,
    pub no_false_support: bool,
}

pub fn baseline_scenario() -> Scenario {
    let requirements = vec![
        requirement(Stage::Forecast, "forecast-valid", "forecast-source", "run-1", 1),
        requirement(Stage::Observation, "reference-valid", "reference-source", "obs-v1", 2),
        requirement(Stage::CapLifecycle, "receipt-valid", "cap-source", "cap-v1", 1),
        requirement(Stage::PopulationReach, "reach-valid", "population-source", "pop-v1", 1),
        requirement(Stage::Decision, "decision-valid", "decision-source", "decision-v1", 1),
        requirement(Stage::AfterEventReview, "review-valid", "review-source", "review-v1", 1),
    ];

    let evidence = vec![
        record("forecast-e1", Stage::Forecast, "forecast-valid", "forecast-source", "run-1", "forecast-root"),
        record("observation-e1", Stage::Observation, "reference-valid", "reference-source", "obs-v1", "sensor-root-a"),
        record("observation-e2", Stage::Observation, "reference-valid", "reference-source", "obs-v1", "sensor-root-b"),
        record("cap-e1", Stage::CapLifecycle, "receipt-valid", "cap-source", "cap-v1", "cap-root"),
        record("reach-e1", Stage::PopulationReach, "reach-valid", "population-source", "pop-v1", "population-root"),
        record("decision-e1", Stage::Decision, "decision-valid", "decision-source", "decision-v1", "decision-root"),
        record("review-e1", Stage::AfterEventReview, "review-valid", "review-source", "review-v1", "review-root"),
    ];

    Scenario {
        scenario_id: "adapter-clean-baseline".into(),
        specification_digest: "gcpn-compositional-assurance-adapter-v0-1".into(),
        as_of_cutoff: 100,
        requirements,
        evidence,
        facts: ScenarioFacts {
            forecast_timing: Some(ForecastTiming {
                issued_at: 10,
                decision_cutoff: 20,
                event_valid_at: 100,
                minimum_action_lead: 30,
            }),
            cap_receipt: Some(CapReceiptLink {
                current_message_id: "cap-alert-42".into(),
                current_version: "2".into(),
                receipt_message_id: "cap-alert-42".into(),
                receipt_version: "2".into(),
            }),
            population: Some(PopulationAccounting {
                eligible_denominator: Some(1000),
                observed_cases: 900,
                typed_missing_cases: 100,
                unique_reach: 700,
            }),
            verification: Some(VerificationAccounting {
                eligible_cases: 100,
                observed_cases: 90,
                typed_missing_cases: 10,
            }),
            paired_comparison: Some(PairedComparison {
                candidate_case_frame_digest: "frame-1".into(),
                reference_case_frame_digest: "frame-1".into(),
                candidate_source: "forecast-model".into(),
                reference_source: "forecast-model".into(),
                candidate_version: "model-v1".into(),
                reference_version: "model-v1".into(),
            }),
            causal_claim: Some(CausalClaim {
                claim_id: "review-valid".into(),
                claim_made: false,
                identification_method_id: None,
                design_id: None,
                supporting_evidence_ids: Vec::new(),
            }),
            post_processing: Some(PostProcessingFacts {
                intrinsic_skill_claim: true,
                calibration_uses_evaluation_outcomes: false,
                calibration_trained_through: Some(5),
                evaluation_period_start: Some(20),
            }),
        },
    }
}

fn requirement(
    stage: Stage,
    claim_id: &str,
    source: &str,
    version: &str,
    minimum_lineages: usize,
) -> ClaimRequirement {
    ClaimRequirement {
        stage,
        claim_id: claim_id.into(),
        subject_digest: "subject-1".into(),
        expected_source: Some(source.into()),
        expected_version: Some(version.into()),
        minimum_independent_supporting_lineages: minimum_lineages,
    }
}

fn record(
    evidence_id: &str,
    stage: Stage,
    claim_id: &str,
    source: &str,
    version: &str,
    lineage_root: &str,
) -> StageEvidence {
    StageEvidence {
        evidence_id: evidence_id.into(),
        stage,
        claim_id: claim_id.into(),
        subject_digest: "subject-1".into(),
        source: source.into(),
        version: version.into(),
        event_time: 10,
        recorded_at: 20,
        validity: EvidenceValidity::Current,
        disposition: ClaimDisposition::Supports,
        lineage_root: lineage_root.into(),
        evaluator_id: format!("evaluator-{evidence_id}"),
    }
}

fn find_claim_evidence_mut(
    scenario: &mut Scenario,
    stage: Stage,
    claim_id: &str,
) -> impl Iterator<Item = &mut StageEvidence> {
    scenario
        .evidence
        .iter_mut()
        .filter(move |evidence| evidence.stage == stage && evidence.claim_id == claim_id)
}

fn apply_one(scenario: &mut Scenario, fault: Fault) {
    match fault {
        Fault::ForecastLate => {
            if let Some(timing) = scenario.facts.forecast_timing.as_mut() {
                timing.issued_at = timing.decision_cutoff + 1;
            }
        }
        Fault::ForecastStale => {
            if let Some(evidence) =
                find_claim_evidence_mut(scenario, Stage::Forecast, "forecast-valid").next()
            {
                evidence.validity = EvidenceValidity::Stale;
            }
        }
        Fault::ObservationMissing => {
            scenario.evidence.retain(|evidence| {
                !(evidence.stage == Stage::Observation && evidence.claim_id == "reference-valid")
            });
        }
        Fault::SharedSourceLineage => {
            for evidence in find_claim_evidence_mut(scenario, Stage::Observation, "reference-valid") {
                evidence.lineage_root = "sensor-root-shared".into();
            }
        }
        Fault::CapMessageIdentityMismatch => {
            if let Some(receipt) = scenario.facts.cap_receipt.as_mut() {
                receipt.receipt_message_id = "cap-alert-wrong".into();
            }
        }
        Fault::ReceiverDenominatorMissing => {
            if let Some(population) = scenario.facts.population.as_mut() {
                population.eligible_denominator = None;
            }
        }
        Fault::OutcomeUnascertained => {
            scenario.evidence.retain(|evidence| {
                !(evidence.stage == Stage::AfterEventReview && evidence.claim_id == "review-valid")
            });
        }
        Fault::PostCutoffEvidence => {
            if let Some(evidence) =
                find_claim_evidence_mut(scenario, Stage::Forecast, "forecast-valid").next()
            {
                evidence.recorded_at = scenario.as_of_cutoff + 1;
            }
        }
        Fault::HindsightLeakage => {
            if let Some(post_processing) = scenario.facts.post_processing.as_mut() {
                post_processing.calibration_uses_evaluation_outcomes = true;
            }
        }
        Fault::ValidContradictoryEvidence => {
            for evidence in find_claim_evidence_mut(scenario, Stage::Observation, "reference-valid") {
                evidence.disposition = ClaimDisposition::Refutes;
            }
        }
        Fault::ClaimOutOfScope => {
            for evidence in find_claim_evidence_mut(scenario, Stage::AfterEventReview, "review-valid") {
                evidence.disposition = ClaimDisposition::OutOfScope;
            }
        }
        Fault::SourceUnavailable => {
            for evidence in find_claim_evidence_mut(scenario, Stage::AfterEventReview, "review-valid") {
                evidence.disposition = ClaimDisposition::Unavailable;
            }
        }
        Fault::AuthorityMismatch
        | Fault::DeliveryReceiptIncomplete
        | Fault::AccessibilityUnverified
        | Fault::ActionReceiptMissing
        | Fault::DuplicateCaseKey => {}
    }
}

pub fn apply_faults(faults: &[Fault]) -> Result<Scenario, AdapterError> {
    let mut seen = BTreeSet::new();
    for fault in faults {
        if !seen.insert(*fault) {
            return Err(AdapterError::DuplicateFault(*fault));
        }
        if !fault.supported() {
            return Err(AdapterError::UnsupportedFault(*fault));
        }
    }

    let mut scenario = baseline_scenario();
    for fault in faults {
        apply_one(&mut scenario, *fault);
    }
    Ok(scenario)
}

pub fn evaluate_single(fault: Fault) -> Result<SingleObservation, AdapterError> {
    let scenario = apply_faults(&[fault])?;
    let report: AuditReport = scenario.audit().map_err(AdapterError::AuditFailed)?;
    let (stage, claim_id) = fault
        .affected_claim()
        .ok_or(AdapterError::UnsupportedFault(fault))?;
    let claim = report
        .claims
        .iter()
        .find(|claim| claim.stage == stage && claim.claim_id == claim_id)
        .ok_or(AdapterError::MissingClaim { stage, claim_id })?;
    let expected = fault
        .expected_disposition()
        .ok_or(AdapterError::UnsupportedFault(fault))?;
    let expected_violation = fault.expected_violation();

    Ok(SingleObservation {
        fault,
        stage,
        claim_id,
        expected,
        observed: claim.disposition,
        expected_violation,
        violation_detected: expected_violation
            .is_none_or(|violation| claim.violations.contains(&violation)),
        report_fingerprint: report.reproducibility_fingerprint,
    })
}

pub fn supported_pairs() -> Vec<[Fault; 2]> {
    let mut pairs = Vec::with_capacity(66);
    for (left_index, left) in Fault::SUPPORTED.iter().enumerate() {
        for right in Fault::SUPPORTED.iter().skip(left_index + 1) {
            pairs.push([*left, *right]);
        }
    }
    pairs
}

pub fn evaluate_pair(faults: [Fault; 2]) -> Result<PairObservation, AdapterError> {
    let scenario = apply_faults(&faults)?;
    let report = scenario.audit().map_err(AdapterError::AuditFailed)?;
    let mut affected = BTreeSet::new();

    for fault in faults {
        if let Some(claim) = fault.affected_claim() {
            affected.insert(claim);
        }
    }

    let mut dispositions = Vec::with_capacity(affected.len());
    for (stage, claim_id) in affected {
        let claim = report
            .claims
            .iter()
            .find(|claim| claim.stage == stage && claim.claim_id == claim_id)
            .ok_or(AdapterError::MissingClaim { stage, claim_id })?;
        dispositions.push(claim.disposition);
    }

    let no_false_support = !dispositions.is_empty()
        && dispositions
            .iter()
            .all(|disposition| *disposition != ClaimDisposition::Supports);

    Ok(PairObservation {
        faults,
        affected_claim_dispositions: dispositions,
        no_false_support,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn clean_baseline_supports_all_six_claims_without_violations() {
        let report = baseline_scenario().audit().expect("baseline must audit");
        assert_eq!(report.claims.len(), 6);
        assert!(
            report
                .claims
                .iter()
                .all(|claim| claim.disposition == ClaimDisposition::Supports)
        );
        assert!(report.violations.is_empty());
    }

    #[test]
    fn all_twelve_supported_single_faults_match_expected_stage_dispositions() {
        for fault in Fault::SUPPORTED {
            let observation = evaluate_single(fault).expect("supported fault must execute");
            assert_eq!(
                observation.observed, observation.expected,
                "wrong disposition for {}",
                fault.label()
            );
            assert!(
                observation.violation_detected,
                "expected violation missing for {}",
                fault.label()
            );
        }
    }

    #[test]
    fn all_sixty_six_supported_pairwise_interactions_are_executed() {
        let pairs = supported_pairs();
        assert_eq!(pairs.len(), 66);

        let observations: Vec<_> = pairs
            .into_iter()
            .map(|pair| evaluate_pair(pair).expect("pairwise fault case must audit"))
            .collect();

        assert_eq!(observations.len(), 66);
        assert!(observations.iter().all(|observation| {
            observation.no_false_support
                && !observation.affected_claim_dispositions.is_empty()
        }));
    }

    #[test]
    fn five_unrepresented_dimensions_fail_closed() {
        assert_eq!(Fault::NOT_REPRESENTED.len(), 5);
        for fault in Fault::NOT_REPRESENTED {
            assert!(matches!(
                apply_faults(&[fault]),
                Err(AdapterError::UnsupportedFault(observed)) if observed == fault
            ));
        }
    }

    #[test]
    fn duplicate_fault_atoms_are_rejected_before_mutation() {
        assert!(matches!(
            apply_faults(&[Fault::ForecastLate, Fault::ForecastLate]),
            Err(AdapterError::DuplicateFault(Fault::ForecastLate))
        ));
    }

    #[test]
    fn source_lineage_fault_is_a_detected_common_cause_violation() {
        let observation = evaluate_single(Fault::SharedSourceLineage)
            .expect("lineage fault must execute");
        assert_eq!(observation.observed, ClaimDisposition::Indeterminate);
        assert_eq!(
            observation.expected_violation,
            Some(ViolationCode::CommonCauseLineage)
        );
        assert!(observation.violation_detected);
    }

    #[test]
    fn missing_outcome_is_unavailable_not_refuted() {
        let observation = evaluate_single(Fault::OutcomeUnascertained)
            .expect("missing outcome fault must execute");
        assert_eq!(observation.observed, ClaimDisposition::Unavailable);
        assert_ne!(observation.observed, ClaimDisposition::Refutes);
    }

    #[test]
    fn cap_identity_and_population_denominator_faults_are_stage_local() {
        let cap = evaluate_single(Fault::CapMessageIdentityMismatch)
            .expect("CAP fault must execute");
        let reach = evaluate_single(Fault::ReceiverDenominatorMissing)
            .expect("denominator fault must execute");

        assert_eq!(cap.stage, Stage::CapLifecycle);
        assert_eq!(
            cap.expected_violation,
            Some(ViolationCode::CapReceiptMessageMismatch)
        );
        assert_eq!(reach.stage, Stage::PopulationReach);
        assert_eq!(
            reach.expected_violation,
            Some(ViolationCode::PopulationDenominatorUnknown)
        );
    }

    #[test]
    fn pairwise_catalogue_is_deterministic() {
        assert_eq!(supported_pairs(), supported_pairs());
    }
}
