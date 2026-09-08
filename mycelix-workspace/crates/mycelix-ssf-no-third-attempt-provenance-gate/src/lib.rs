// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Structural provenance gate for replay evidence.
//!
//! v0.1 permits replay consideration only for an audited `Initial` execution
//! lineage. Any legacy or canonical replay provenance is already a second
//! attempt and is therefore structurally unable to become a third-attempt
//! candidate. This crate creates no replay or effect authority.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use mycelix_ssf_actuator_effect_protocol::{
    recovery_policy_for, ActuatorEffectSubjectV1, ActuatorInvocationAttemptId,
    ActuatorStableEffectIdentityV1, EffectRecoveryPolicyV1,
};
use mycelix_ssf_canonical_actuator_execution::CanonicalQualifiedEffectDispositionV1;
use mycelix_ssf_canonical_completed_effect_evidence::CanonicalCompletedInvocationRecordV1;
use mycelix_ssf_canonical_completed_outcome_history_audit::AuditedCanonicalCompletedOutcomeHistoryHeadV1;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum InitialReplayEvidenceBasisV1 {
    ProvenNotApplied,
    IdempotentOutcomeUnknown,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum NoThirdAttemptProvenanceGateErrorV1 {
    HistoryEmpty,
    ReplayProvenanceAlreadyConsumed,
    ConfirmedEffectCannotReplay,
    TransactionalUnknownRequiresReconciliation,
    NonIdempotentUnknownCannotReplay,
    RecoveryPolicyMismatch,
}

pub struct NoThirdAttemptProvenanceGateFailureV1<S, TQ> {
    history: AuditedCanonicalCompletedOutcomeHistoryHeadV1<S, TQ>,
    error: NoThirdAttemptProvenanceGateErrorV1,
}

impl<S, TQ> NoThirdAttemptProvenanceGateFailureV1<S, TQ> {
    pub const fn error(&self) -> NoThirdAttemptProvenanceGateErrorV1 {
        self.error
    }

    pub fn into_history(self) -> AuditedCanonicalCompletedOutcomeHistoryHeadV1<S, TQ> {
        self.history
    }
}

pub struct InitialReplayEvidenceCandidateV1<S, TQ> {
    history: AuditedCanonicalCompletedOutcomeHistoryHeadV1<S, TQ>,
    prior_attempt_id: ActuatorInvocationAttemptId,
    prior_effect_subject: ActuatorEffectSubjectV1,
    stable_effect_identity: ActuatorStableEffectIdentityV1,
    basis: InitialReplayEvidenceBasisV1,
    valid_until: u64,
}

impl<S, TQ> InitialReplayEvidenceCandidateV1<S, TQ> {
    pub const fn history(&self) -> &AuditedCanonicalCompletedOutcomeHistoryHeadV1<S, TQ> {
        &self.history
    }

    pub const fn prior_attempt_id(&self) -> ActuatorInvocationAttemptId {
        self.prior_attempt_id
    }

    pub const fn prior_effect_subject(&self) -> ActuatorEffectSubjectV1 {
        self.prior_effect_subject
    }

    pub const fn stable_effect_identity(&self) -> ActuatorStableEffectIdentityV1 {
        self.stable_effect_identity
    }

    pub const fn basis(&self) -> InitialReplayEvidenceBasisV1 {
        self.basis
    }

    pub const fn valid_until(&self) -> u64 {
        self.valid_until
    }

    pub const fn provenance_is_initial(&self) -> bool {
        true
    }

    pub const fn contains_replay_authority(&self) -> bool {
        false
    }

    pub const fn contains_effect_authority(&self) -> bool {
        false
    }

    pub const fn permits_third_attempt(&self) -> bool {
        false
    }
}

fn fail<S, TQ>(
    history: AuditedCanonicalCompletedOutcomeHistoryHeadV1<S, TQ>,
    error: NoThirdAttemptProvenanceGateErrorV1,
) -> Result<InitialReplayEvidenceCandidateV1<S, TQ>, NoThirdAttemptProvenanceGateFailureV1<S, TQ>> {
    Err(NoThirdAttemptProvenanceGateFailureV1 { history, error })
}

pub fn derive_initial_replay_evidence_candidate<S, TQ>(
    history: AuditedCanonicalCompletedOutcomeHistoryHeadV1<S, TQ>,
) -> Result<InitialReplayEvidenceCandidateV1<S, TQ>, NoThirdAttemptProvenanceGateFailureV1<S, TQ>> {
    let invocation_record = history.invocation_record();
    if !matches!(invocation_record, CanonicalCompletedInvocationRecordV1::Initial(_)) {
        return fail(
            history,
            NoThirdAttemptProvenanceGateErrorV1::ReplayProvenanceAlreadyConsumed,
        );
    }

    let entry = match history.latest_entry() {
        Some(entry) => entry,
        None => {
            return fail(
                history,
                NoThirdAttemptProvenanceGateErrorV1::HistoryEmpty,
            );
        }
    };

    let evidence = entry.manifest.subject.evidence;
    let attempt = evidence.attempt();
    let canonical = evidence.canonical_disposition();

    let basis = match canonical {
        CanonicalQualifiedEffectDispositionV1::Confirmed { .. } => {
            return fail(
                history,
                NoThirdAttemptProvenanceGateErrorV1::ConfirmedEffectCannotReplay,
            );
        }
        CanonicalQualifiedEffectDispositionV1::ProvenNotApplied { .. } => {
            InitialReplayEvidenceBasisV1::ProvenNotApplied
        }
        CanonicalQualifiedEffectDispositionV1::OutcomeUnknown {
            recovery_policy, ..
        } => {
            let expected_recovery = recovery_policy_for(attempt.subject.recovery_mode);
            if recovery_policy != expected_recovery {
                return fail(
                    history,
                    NoThirdAttemptProvenanceGateErrorV1::RecoveryPolicyMismatch,
                );
            }

            match recovery_policy {
                EffectRecoveryPolicyV1::ReconcileOnly => {
                    return fail(
                        history,
                        NoThirdAttemptProvenanceGateErrorV1::TransactionalUnknownRequiresReconciliation,
                    );
                }
                EffectRecoveryPolicyV1::ExactSameClaimReplayMayBePermitted => {
                    InitialReplayEvidenceBasisV1::IdempotentOutcomeUnknown
                }
                EffectRecoveryPolicyV1::NeverAutomaticRetry => {
                    return fail(
                        history,
                        NoThirdAttemptProvenanceGateErrorV1::NonIdempotentUnknownCannotReplay,
                    );
                }
            }
        }
    };

    Ok(InitialReplayEvidenceCandidateV1 {
        valid_until: history.audit_valid_until(),
        prior_attempt_id: attempt.attempt_id,
        prior_effect_subject: attempt.subject,
        stable_effect_identity: attempt.subject.stable_identity(),
        basis,
        history,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn replay_provenance_and_confirmed_are_distinct_structural_stops() {
        assert_ne!(
            NoThirdAttemptProvenanceGateErrorV1::ReplayProvenanceAlreadyConsumed,
            NoThirdAttemptProvenanceGateErrorV1::ConfirmedEffectCannotReplay
        );
    }

    #[test]
    fn candidate_basis_does_not_encode_authority() {
        assert_ne!(
            InitialReplayEvidenceBasisV1::ProvenNotApplied,
            InitialReplayEvidenceBasisV1::IdempotentOutcomeUnknown
        );
    }
}
