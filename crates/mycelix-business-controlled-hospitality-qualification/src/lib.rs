// Copyright (C) 2024-2026 Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Immutable control-evidence envelope for read-only hospitality qualification.
//!
//! This crate never rewrites the underlying hospitality report. It verifies that full-interval
//! control coverage describes the exact same pilot semantics, then emits a new digest-bound
//! limitation state while preserving the original model verdict and report digest.

use std::collections::{BTreeMap, BTreeSet};

use mycelix_business_control_coverage::{
    ControlCoverageEvidence, ControlCoveragePlan, CoverageError, QualificationLimitationTransition,
};
use mycelix_business_control_reconciliation::{
    aggregation_semantic_authority_unverified_ref, control_metric_semantic_authority_unverified_ref,
    control_source_authenticity_unverified_ref, control_source_external_reality_unverified_ref,
    upstream_export_completeness_unverified_ref,
};
use mycelix_business_core::{Digest32, ReferenceId};
use mycelix_business_derived_forecast_actual::{ActualProjectionSpec, AggregationKind};
use mycelix_business_forecast_plan::ForecastTargetPlan;
use mycelix_business_hospitality_transaction_field::TransactionHospitalityReport;
use mycelix_business_pilot_hospitality::{HospitalityForecastPilotRegistration, sales_input_ref};
use sha2::{Digest, Sha256};

pub const CONTROLLED_QUALIFICATION_IS_READ_ONLY: bool = true;
pub const ORIGINAL_REPORT_IS_IMMUTABLE: bool = true;

fn zero_digest(value: &Digest32) -> bool {
    value == &Digest32([0; 32])
}

fn hash_str(hasher: &mut Sha256, value: &str) {
    hasher.update((value.len() as u64).to_be_bytes());
    hasher.update(value.as_bytes());
}

fn finish_digest(hasher: Sha256) -> Digest32 {
    Digest32(hasher.finalize().into())
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ControlledHospitalityQualification {
    pub original_report_digest: Digest32,
    pub registration_digest: Digest32,
    pub target_plan_digest: Digest32,
    pub projection_spec_digest: Digest32,
    pub control_coverage_plan_digest: Digest32,
    pub control_coverage_digest: Digest32,
    pub qualification_transition_digests: Vec<Digest32>,
    pub prior_limitations: BTreeSet<ReferenceId>,
    pub narrowed_limitations: BTreeSet<ReferenceId>,
    pub envelope_digest: Digest32,
}

#[derive(Debug)]
pub enum ControlledQualificationError {
    InvalidRegistration,
    InvalidTargetPlan,
    InvalidProjectionSpec,
    InvalidOriginalReportDigest,
    OriginalReportBindingMismatch,
    OriginalLimitationStateMismatch,
    ControlCoverage(CoverageError),
    ControlPlanRegisteredAfterPilot,
    ConnectorMismatch,
    SourceInputMismatch,
    ScopeMismatch,
    QualificationWindowMismatch,
    TargetSemanticsMismatch { target: ReferenceId },
    ProjectionAggregationMismatch,
    TransitionCountMismatch,
    InvalidQualificationTransition,
    UnexpectedTransitionSet,
    MissingBroadLimitation { limitation: ReferenceId },
    ZeroDigest,
    EnvelopeMismatch,
}

pub fn compose_controlled_hospitality_qualification(
    registration: &HospitalityForecastPilotRegistration,
    target_plan: &ForecastTargetPlan,
    projection_spec: &ActualProjectionSpec,
    report: &TransactionHospitalityReport,
    control_plan: &ControlCoveragePlan,
    coverage: &ControlCoverageEvidence,
) -> Result<ControlledHospitalityQualification, ControlledQualificationError> {
    registration
        .validate()
        .map_err(|_| ControlledQualificationError::InvalidRegistration)?;
    target_plan
        .validate_against(&registration.protocol)
        .map_err(|_| ControlledQualificationError::InvalidTargetPlan)?;
    projection_spec
        .validate_against(&registration.protocol, target_plan)
        .map_err(|_| ControlledQualificationError::InvalidProjectionSpec)?;
    if !report.validate_digest() || zero_digest(&report.report_digest) {
        return Err(ControlledQualificationError::InvalidOriginalReportDigest);
    }
    validate_original_report_bindings(registration, target_plan, projection_spec, report)?;
    coverage
        .validate_against(control_plan)
        .map_err(ControlledQualificationError::ControlCoverage)?;
    validate_control_semantics(registration, target_plan, projection_spec, report, control_plan, coverage)?;

    let transitions = coverage
        .qualification_limitation_transitions(control_plan)
        .map_err(ControlledQualificationError::ControlCoverage)?;
    validate_transition_set(&transitions, control_plan, coverage)?;

    let prior_limitations = report.unresolved_limitations.clone();
    let narrowed_limitations = apply_expected_replacements(&prior_limitations)?;
    let qualification_transition_digests = transitions
        .iter()
        .map(|transition| transition.transition_digest)
        .collect::<Vec<_>>();

    let mut envelope = ControlledHospitalityQualification {
        original_report_digest: report.report_digest,
        registration_digest: registration.registration_digest,
        target_plan_digest: target_plan.plan_digest,
        projection_spec_digest: projection_spec.spec_digest,
        control_coverage_plan_digest: control_plan.plan_digest,
        control_coverage_digest: coverage.coverage_digest,
        qualification_transition_digests,
        prior_limitations,
        narrowed_limitations,
        envelope_digest: Digest32([0; 32]),
    };
    envelope.envelope_digest = envelope_digest(&envelope);
    Ok(envelope)
}

impl ControlledHospitalityQualification {
    pub fn validate_against(
        &self,
        registration: &HospitalityForecastPilotRegistration,
        target_plan: &ForecastTargetPlan,
        projection_spec: &ActualProjectionSpec,
        report: &TransactionHospitalityReport,
        control_plan: &ControlCoveragePlan,
        coverage: &ControlCoverageEvidence,
    ) -> Result<(), ControlledQualificationError> {
        if zero_digest(&self.envelope_digest) {
            return Err(ControlledQualificationError::ZeroDigest);
        }
        let rebuilt = compose_controlled_hospitality_qualification(
            registration,
            target_plan,
            projection_spec,
            report,
            control_plan,
            coverage,
        )?;
        if &rebuilt != self || self.envelope_digest != envelope_digest(self) {
            return Err(ControlledQualificationError::EnvelopeMismatch);
        }
        Ok(())
    }
}

fn validate_original_report_bindings(
    registration: &HospitalityForecastPilotRegistration,
    target_plan: &ForecastTargetPlan,
    projection_spec: &ActualProjectionSpec,
    report: &TransactionHospitalityReport,
) -> Result<(), ControlledQualificationError> {
    if report.registration_digest != registration.registration_digest
        || report.target_plan_digest != target_plan.plan_digest
        || report.projection_spec_digest != projection_spec.spec_digest
        || report.field_evidence.plan_digest != registration.field_plan.plan_digest
        || report.field_evidence.profile != registration.field_plan.profile
        || report.field_evidence.capability != registration.field_plan.capability
        || report.field_evidence.scope != registration.field_plan.scope
        || report.field_evidence.shadow_protocol_digest
            != registration.field_plan.shadow_protocol_digest
    {
        return Err(ControlledQualificationError::OriginalReportBindingMismatch);
    }
    if report.field_evidence.known_limitations != report.unresolved_limitations {
        return Err(ControlledQualificationError::OriginalLimitationStateMismatch);
    }
    Ok(())
}

fn validate_control_semantics(
    registration: &HospitalityForecastPilotRegistration,
    target_plan: &ForecastTargetPlan,
    projection_spec: &ActualProjectionSpec,
    report: &TransactionHospitalityReport,
    control_plan: &ControlCoveragePlan,
    coverage: &ControlCoverageEvidence,
) -> Result<(), ControlledQualificationError> {
    if control_plan.registered_at_unix_ms > registration.protocol.registered_at_unix_ms {
        return Err(ControlledQualificationError::ControlPlanRegisteredAfterPilot);
    }
    if control_plan.connector != registration.ingress_connector
        || control_plan.connector != report.campaign_replay.connector
    {
        return Err(ControlledQualificationError::ConnectorMismatch);
    }
    if control_plan.source_input != projection_spec.source_input
        || control_plan.source_input != sales_input_ref()
    {
        return Err(ControlledQualificationError::SourceInputMismatch);
    }
    if control_plan.scope != registration.field_plan.scope {
        return Err(ControlledQualificationError::ScopeMismatch);
    }
    if control_plan.qualification_start_unix_ms != registration.protocol.evaluation_start_unix_ms
        || control_plan.qualification_end_unix_ms != registration.protocol.evaluation_end_unix_ms
        || control_plan.qualification_start_unix_ms
            != registration.field_plan.evaluation_start_unix_ms
        || control_plan.qualification_end_unix_ms != registration.field_plan.evaluation_end_unix_ms
    {
        return Err(ControlledQualificationError::QualificationWindowMismatch);
    }
    if control_plan.campaign_binding_mismatch(report, coverage) {
        return Err(ControlledQualificationError::OriginalReportBindingMismatch);
    }
    if !matches!(projection_spec.aggregation, AggregationKind::Sum) {
        return Err(ControlledQualificationError::ProjectionAggregationMismatch);
    }
    for planned in &target_plan.targets {
        let target = &planned.target;
        if target.metric != control_plan.metric
            || target.scope != control_plan.scope
            || target.unit != control_plan.unit
            || target.scale != control_plan.scale
        {
            return Err(ControlledQualificationError::TargetSemanticsMismatch {
                target: planned.target_id.clone(),
            });
        }
    }
    Ok(())
}

trait ControlCampaignBinding {
    fn campaign_binding_mismatch(
        &self,
        report: &TransactionHospitalityReport,
        coverage: &ControlCoverageEvidence,
    ) -> bool;
}

impl ControlCampaignBinding for ControlCoveragePlan {
    fn campaign_binding_mismatch(
        &self,
        report: &TransactionHospitalityReport,
        coverage: &ControlCoverageEvidence,
    ) -> bool {
        coverage.campaign_digest != report.campaign_digest
            || coverage.replay_digest != report.campaign_replay.evidence_digest
    }
}

fn validate_transition_set(
    transitions: &[QualificationLimitationTransition],
    plan: &ControlCoveragePlan,
    coverage: &ControlCoverageEvidence,
) -> Result<(), ControlledQualificationError> {
    if transitions.len() != 2 {
        return Err(ControlledQualificationError::TransitionCountMismatch);
    }
    if transitions
        .iter()
        .any(|transition| !transition.validate_against(plan, coverage))
    {
        return Err(ControlledQualificationError::InvalidQualificationTransition);
    }
    let actual = transitions
        .iter()
        .map(|transition| (transition.from.clone(), transition.to.clone()))
        .collect::<BTreeMap<_, _>>();
    let expected = BTreeMap::from([
        (
            upstream_export_completeness_unverified_ref(),
            control_source_external_reality_unverified_ref(),
        ),
        (
            aggregation_semantic_authority_unverified_ref(),
            control_metric_semantic_authority_unverified_ref(),
        ),
    ]);
    if actual != expected {
        return Err(ControlledQualificationError::UnexpectedTransitionSet);
    }
    Ok(())
}

fn apply_expected_replacements(
    prior: &BTreeSet<ReferenceId>,
) -> Result<BTreeSet<ReferenceId>, ControlledQualificationError> {
    let export = upstream_export_completeness_unverified_ref();
    let aggregation = aggregation_semantic_authority_unverified_ref();
    for required in [&export, &aggregation] {
        if !prior.contains(required) {
            return Err(ControlledQualificationError::MissingBroadLimitation {
                limitation: required.clone(),
            });
        }
    }
    let mut after = prior.clone();
    after.remove(&export);
    after.remove(&aggregation);
    after.insert(control_source_external_reality_unverified_ref());
    after.insert(control_metric_semantic_authority_unverified_ref());
    after.insert(control_source_authenticity_unverified_ref());
    Ok(after)
}

fn envelope_digest(value: &ControlledHospitalityQualification) -> Digest32 {
    let mut hasher = Sha256::new();
    hash_str(&mut hasher, "mycelix:controlled-hospitality-qualification:v1");
    hasher.update(value.original_report_digest.0);
    hasher.update(value.registration_digest.0);
    hasher.update(value.target_plan_digest.0);
    hasher.update(value.projection_spec_digest.0);
    hasher.update(value.control_coverage_plan_digest.0);
    hasher.update(value.control_coverage_digest.0);
    hasher.update((value.qualification_transition_digests.len() as u64).to_be_bytes());
    for digest in &value.qualification_transition_digests {
        hasher.update(digest.0);
    }
    hash_limitations(&mut hasher, &value.prior_limitations);
    hash_limitations(&mut hasher, &value.narrowed_limitations);
    finish_digest(hasher)
}

fn hash_limitations(hasher: &mut Sha256, limitations: &BTreeSet<ReferenceId>) {
    hasher.update((limitations.len() as u64).to_be_bytes());
    for limitation in limitations {
        hash_str(hasher, limitation.as_str());
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn id(value: &str) -> ReferenceId {
        ReferenceId::new(value).unwrap()
    }

    #[test]
    fn expected_replacements_preserve_unrelated_limitations() {
        let mut before = BTreeSet::from([
            upstream_export_completeness_unverified_ref(),
            aggregation_semantic_authority_unverified_ref(),
            id("limitation:time-rule-source-authority-unverified:v1"),
        ]);
        let after = apply_expected_replacements(&before).unwrap();
        assert!(!after.contains(&upstream_export_completeness_unverified_ref()));
        assert!(!after.contains(&aggregation_semantic_authority_unverified_ref()));
        assert!(after.contains(&control_source_external_reality_unverified_ref()));
        assert!(after.contains(&control_metric_semantic_authority_unverified_ref()));
        assert!(after.contains(&control_source_authenticity_unverified_ref()));
        assert!(after.contains(&id("limitation:time-rule-source-authority-unverified:v1")));

        before.remove(&upstream_export_completeness_unverified_ref());
        assert!(matches!(
            apply_expected_replacements(&before),
            Err(ControlledQualificationError::MissingBroadLimitation { .. })
        ));
    }

    #[test]
    fn envelope_digest_binds_before_and_after_states() {
        let before = BTreeSet::from([
            upstream_export_completeness_unverified_ref(),
            aggregation_semantic_authority_unverified_ref(),
        ]);
        let after = apply_expected_replacements(&before).unwrap();
        let mut envelope = ControlledHospitalityQualification {
            original_report_digest: Digest32::repeat(1),
            registration_digest: Digest32::repeat(2),
            target_plan_digest: Digest32::repeat(3),
            projection_spec_digest: Digest32::repeat(4),
            control_coverage_plan_digest: Digest32::repeat(5),
            control_coverage_digest: Digest32::repeat(6),
            qualification_transition_digests: vec![Digest32::repeat(7), Digest32::repeat(8)],
            prior_limitations: before,
            narrowed_limitations: after,
            envelope_digest: Digest32([0; 32]),
        };
        envelope.envelope_digest = envelope_digest(&envelope);
        let original = envelope.envelope_digest;
        envelope
            .narrowed_limitations
            .insert(id("limitation:unexpected:v1"));
        assert_ne!(original, envelope_digest(&envelope));
    }
}
