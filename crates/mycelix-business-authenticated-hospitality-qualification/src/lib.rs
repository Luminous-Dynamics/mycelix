// Copyright (C) 2024-2026 Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Additive authenticity-strengthened hospitality qualification.
//!
//! This crate preserves both the original transaction report and the controlled qualification
//! envelope. It revalidates both underlying theorems and the complete external-authenticity theorem
//! before narrowing only the remaining control-source authenticity limitation.

use std::collections::BTreeSet;

use mycelix_business_control_authenticity_coverage::{
    AuthenticityCoverageError, AuthenticityQualificationTransition, ControlAuthenticityCoverageEvidence,
    ControlAuthenticityCoveragePlan, ControlAuthenticityEntry,
    control_source_issuer_authority_unverified_ref,
};
use mycelix_business_control_coverage::{ControlCoverageEntry, ControlCoverageEvidence, ControlCoveragePlan};
use mycelix_business_control_reconciliation::control_source_authenticity_unverified_ref;
use mycelix_business_controlled_hospitality_qualification::{
    ControlledHospitalityQualification, ControlledQualificationError,
};
use mycelix_business_core::{Digest32, ReferenceId};
use mycelix_business_derived_forecast_actual::ActualProjectionSpec;
use mycelix_business_forecast_plan::ForecastTargetPlan;
use mycelix_business_hospitality_transaction_field::TransactionHospitalityReport;
use mycelix_business_pilot_hospitality::HospitalityForecastPilotRegistration;
use sha2::{Digest, Sha256};

pub const AUTHENTICATED_QUALIFICATION_IS_READ_ONLY: bool = true;
pub const PRIOR_EVIDENCE_ENVELOPES_ARE_IMMUTABLE: bool = true;

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
pub struct AuthenticatedHospitalityQualification {
    pub original_report_digest: Digest32,
    pub controlled_qualification_digest: Digest32,
    pub authenticity_plan_digest: Digest32,
    pub authenticity_coverage_digest: Digest32,
    pub authenticity_transition_digest: Digest32,
    pub prior_limitations: BTreeSet<ReferenceId>,
    pub narrowed_limitations: BTreeSet<ReferenceId>,
    pub envelope_digest: Digest32,
}

#[derive(Debug)]
pub enum AuthenticatedQualificationError {
    Controlled(ControlledQualificationError),
    Authenticity(AuthenticityCoverageError),
    ControlledEnvelopeBindingMismatch,
    AuthenticityTransitionMismatch,
    MissingAuthenticityLimitation,
    ZeroDigest,
    EnvelopeMismatch,
}

#[allow(clippy::too_many_arguments)]
pub fn compose_authenticated_hospitality_qualification(
    registration: &HospitalityForecastPilotRegistration,
    target_plan: &ForecastTargetPlan,
    projection_spec: &ActualProjectionSpec,
    report: &TransactionHospitalityReport,
    control_plan: &ControlCoveragePlan,
    control_coverage: &ControlCoverageEvidence,
    control_entries: &[ControlCoverageEntry],
    controlled: &ControlledHospitalityQualification,
    authenticity_plan: &ControlAuthenticityCoveragePlan,
    authenticity_coverage: &ControlAuthenticityCoverageEvidence,
    authenticity_entries: &[ControlAuthenticityEntry],
) -> Result<AuthenticatedHospitalityQualification, AuthenticatedQualificationError> {
    controlled
        .validate_against(
            registration,
            target_plan,
            projection_spec,
            report,
            control_plan,
            control_coverage,
        )
        .map_err(AuthenticatedQualificationError::Controlled)?;

    if controlled.original_report_digest != report.report_digest
        || controlled.control_coverage_plan_digest != control_plan.plan_digest
        || controlled.control_coverage_digest != control_coverage.coverage_digest
    {
        return Err(AuthenticatedQualificationError::ControlledEnvelopeBindingMismatch);
    }

    let transition = authenticity_coverage
        .qualification_limitation_transition(
            authenticity_plan,
            control_plan,
            control_coverage,
            control_entries,
            authenticity_entries,
        )
        .map_err(AuthenticatedQualificationError::Authenticity)?;
    validate_authenticity_transition(&transition, authenticity_plan, authenticity_coverage)?;

    let prior_limitations = controlled.narrowed_limitations.clone();
    let narrowed_limitations = apply_authenticity_transition(&prior_limitations)?;
    let mut envelope = AuthenticatedHospitalityQualification {
        original_report_digest: report.report_digest,
        controlled_qualification_digest: controlled.envelope_digest,
        authenticity_plan_digest: authenticity_plan.plan_digest,
        authenticity_coverage_digest: authenticity_coverage.coverage_digest,
        authenticity_transition_digest: transition.transition_digest,
        prior_limitations,
        narrowed_limitations,
        envelope_digest: Digest32([0; 32]),
    };
    envelope.envelope_digest = envelope_digest(&envelope);
    Ok(envelope)
}

impl AuthenticatedHospitalityQualification {
    #[allow(clippy::too_many_arguments)]
    pub fn validate_against(
        &self,
        registration: &HospitalityForecastPilotRegistration,
        target_plan: &ForecastTargetPlan,
        projection_spec: &ActualProjectionSpec,
        report: &TransactionHospitalityReport,
        control_plan: &ControlCoveragePlan,
        control_coverage: &ControlCoverageEvidence,
        control_entries: &[ControlCoverageEntry],
        controlled: &ControlledHospitalityQualification,
        authenticity_plan: &ControlAuthenticityCoveragePlan,
        authenticity_coverage: &ControlAuthenticityCoverageEvidence,
        authenticity_entries: &[ControlAuthenticityEntry],
    ) -> Result<(), AuthenticatedQualificationError> {
        if zero_digest(&self.envelope_digest) {
            return Err(AuthenticatedQualificationError::ZeroDigest);
        }
        let rebuilt = compose_authenticated_hospitality_qualification(
            registration,
            target_plan,
            projection_spec,
            report,
            control_plan,
            control_coverage,
            control_entries,
            controlled,
            authenticity_plan,
            authenticity_coverage,
            authenticity_entries,
        )?;
        if &rebuilt != self || self.envelope_digest != envelope_digest(self) {
            return Err(AuthenticatedQualificationError::EnvelopeMismatch);
        }
        Ok(())
    }
}

fn validate_authenticity_transition(
    transition: &AuthenticityQualificationTransition,
    plan: &ControlAuthenticityCoveragePlan,
    evidence: &ControlAuthenticityCoverageEvidence,
) -> Result<(), AuthenticatedQualificationError> {
    if transition.from != control_source_authenticity_unverified_ref()
        || transition.to != control_source_issuer_authority_unverified_ref()
        || !transition.validate_against(plan, evidence)
    {
        return Err(AuthenticatedQualificationError::AuthenticityTransitionMismatch);
    }
    Ok(())
}

fn apply_authenticity_transition(
    prior: &BTreeSet<ReferenceId>,
) -> Result<BTreeSet<ReferenceId>, AuthenticatedQualificationError> {
    let from = control_source_authenticity_unverified_ref();
    if !prior.contains(&from) {
        return Err(AuthenticatedQualificationError::MissingAuthenticityLimitation);
    }
    let mut after = prior.clone();
    after.remove(&from);
    after.insert(control_source_issuer_authority_unverified_ref());
    Ok(after)
}

fn envelope_digest(value: &AuthenticatedHospitalityQualification) -> Digest32 {
    let mut hasher = Sha256::new();
    hash_str(&mut hasher, "mycelix:authenticated-hospitality-qualification:v1");
    hasher.update(value.original_report_digest.0);
    hasher.update(value.controlled_qualification_digest.0);
    hasher.update(value.authenticity_plan_digest.0);
    hasher.update(value.authenticity_coverage_digest.0);
    hasher.update(value.authenticity_transition_digest.0);
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
    fn authenticity_transition_preserves_every_other_limitation() {
        let prior = BTreeSet::from([
            control_source_authenticity_unverified_ref(),
            id("limitation:control-source-external-reality-unverified:v1"),
            id("limitation:control-metric-semantic-authority-unverified:v1"),
            id("limitation:time-rule-source-authority-unverified:v1"),
        ]);
        let after = apply_authenticity_transition(&prior).unwrap();
        assert!(!after.contains(&control_source_authenticity_unverified_ref()));
        assert!(after.contains(&control_source_issuer_authority_unverified_ref()));
        assert!(after.contains(&id("limitation:control-source-external-reality-unverified:v1")));
        assert!(after.contains(&id("limitation:control-metric-semantic-authority-unverified:v1")));
        assert!(after.contains(&id("limitation:time-rule-source-authority-unverified:v1")));
    }

    #[test]
    fn missing_authenticity_limitation_fails_closed() {
        let prior = BTreeSet::from([id("limitation:other:v1")]);
        assert!(matches!(
            apply_authenticity_transition(&prior),
            Err(AuthenticatedQualificationError::MissingAuthenticityLimitation)
        ));
    }

    #[test]
    fn envelope_digest_binds_prior_and_narrowed_states() {
        let prior = BTreeSet::from([control_source_authenticity_unverified_ref()]);
        let after = apply_authenticity_transition(&prior).unwrap();
        let mut value = AuthenticatedHospitalityQualification {
            original_report_digest: Digest32::repeat(1),
            controlled_qualification_digest: Digest32::repeat(2),
            authenticity_plan_digest: Digest32::repeat(3),
            authenticity_coverage_digest: Digest32::repeat(4),
            authenticity_transition_digest: Digest32::repeat(5),
            prior_limitations: prior,
            narrowed_limitations: after,
            envelope_digest: Digest32([0; 32]),
        };
        value.envelope_digest = envelope_digest(&value);
        let original = value.envelope_digest;
        value.narrowed_limitations.insert(id("limitation:tampered:v1"));
        assert_ne!(original, envelope_digest(&value));
    }
}
