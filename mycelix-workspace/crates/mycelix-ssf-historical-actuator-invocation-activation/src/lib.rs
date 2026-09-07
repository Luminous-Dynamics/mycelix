// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Fresh activation of historically reconciled actuator invocation journals.
//! Historical reconciliation refreshes knowledge of the old journal event; it
//! never refreshes the original effect authority. This crate requires a newer
//! trusted-time observation and preserves the old authority ceiling.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use mycelix_ssf_actuator_effect_protocol::ActuatorInvocationAttemptManifestV1;
use mycelix_ssf_contracts::SSF_SCHEMA_V1;
use mycelix_ssf_current_authority_revalidation::QualifiedCurrentTimeV1;
use mycelix_ssf_historical_actuator_invocation_journal::{
    HistoricalInvocationJournalEvidenceV1, HistoricalInvocationJournalReconciliationSubjectV1,
    HistoricallyReconciledInitialInvocationJournalV1,
    HistoricallyReconciledReplayInvocationJournalV1,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum HistoricalInvocationActivationKindV1 {
    Initial,
    Replay,
}

mod sealed {
    pub trait Sealed {}
}

pub trait ExactHistoricallyReconciledInvocationJournalV1: sealed::Sealed {
    fn exact_attempt_manifest(&self) -> ActuatorInvocationAttemptManifestV1;
    fn reconciliation_latest_possible_unix_ms(&self) -> u64;
    fn invocation_eligibility_valid_until(&self) -> u64;
    fn exact_historical_binding_is_consistent(&self) -> bool;
    fn activation_kind(&self) -> HistoricalInvocationActivationKindV1;
}

impl<R, RTQ> sealed::Sealed for HistoricallyReconciledInitialInvocationJournalV1<R, RTQ> {}

impl<R, RTQ> ExactHistoricallyReconciledInvocationJournalV1
    for HistoricallyReconciledInitialInvocationJournalV1<R, RTQ>
{
    fn exact_attempt_manifest(&self) -> ActuatorInvocationAttemptManifestV1 {
        self.journal_receipt().manifest.attempt
    }

    fn reconciliation_latest_possible_unix_ms(&self) -> u64 {
        self.current_time().latest_possible_unix_ms()
    }

    fn invocation_eligibility_valid_until(&self) -> u64 {
        self.invocation_eligibility_valid_until()
    }

    fn exact_historical_binding_is_consistent(&self) -> bool {
        let journal = self.journal_receipt();
        let reconciliation = self.reconciliation_receipt();
        match (reconciliation.subject, reconciliation.evidence) {
            (
                HistoricalInvocationJournalReconciliationSubjectV1::Initial {
                    original_store,
                    claim_record,
                    current_time_receipt,
                    latest_possible_unix_ms,
                },
                HistoricalInvocationJournalEvidenceV1::Initial(evidence),
            ) => {
                reconciliation.schema_version == SSF_SCHEMA_V1
                    && evidence == journal
                    && original_store == journal.manifest.expected_store
                    && claim_record == journal.manifest.claim_record()
                    && current_time_receipt == self.current_time().receipt_commitment()
                    && latest_possible_unix_ms == self.current_time().latest_possible_unix_ms()
                    && reconciliation.valid_until >= latest_possible_unix_ms
            }
            _ => false,
        }
    }

    fn activation_kind(&self) -> HistoricalInvocationActivationKindV1 {
        HistoricalInvocationActivationKindV1::Initial
    }
}

impl<R, RTQ> sealed::Sealed for HistoricallyReconciledReplayInvocationJournalV1<R, RTQ> {}

impl<R, RTQ> ExactHistoricallyReconciledInvocationJournalV1
    for HistoricallyReconciledReplayInvocationJournalV1<R, RTQ>
{
    fn exact_attempt_manifest(&self) -> ActuatorInvocationAttemptManifestV1 {
        self.journal_receipt().manifest.attempt
    }

    fn reconciliation_latest_possible_unix_ms(&self) -> u64 {
        self.current_time().latest_possible_unix_ms()
    }

    fn invocation_eligibility_valid_until(&self) -> u64 {
        self.invocation_eligibility_valid_until()
    }

    fn exact_historical_binding_is_consistent(&self) -> bool {
        let journal = self.journal_receipt();
        let reconciliation = self.reconciliation_receipt();
        match (reconciliation.subject, reconciliation.evidence) {
            (
                HistoricalInvocationJournalReconciliationSubjectV1::Replay {
                    original_store,
                    prior_attempt_id,
                    current_time_receipt,
                    latest_possible_unix_ms,
                },
                HistoricalInvocationJournalEvidenceV1::Replay(evidence),
            ) => {
                reconciliation.schema_version == SSF_SCHEMA_V1
                    && evidence == journal
                    && original_store == journal.manifest.expected_store
                    && prior_attempt_id == journal.manifest.prior_attempt_id()
                    && current_time_receipt == self.current_time().receipt_commitment()
                    && latest_possible_unix_ms == self.current_time().latest_possible_unix_ms()
                    && reconciliation.valid_until >= latest_possible_unix_ms
                    && journal.manifest.replay.subject.prior_effect_subject.stable_identity()
                        == journal.manifest.replay.subject.stable_effect_identity
                    && journal.manifest.attempt.subject.stable_identity()
                        == journal.manifest.replay.subject.stable_effect_identity
            }
            _ => false,
        }
    }

    fn activation_kind(&self) -> HistoricalInvocationActivationKindV1 {
        HistoricalInvocationActivationKindV1::Replay
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HistoricalInvocationActivationErrorV1 {
    UnsupportedAttemptSchema,
    HistoricalBindingMismatch,
    FreshTimeRegressedBeforeAttempt,
    FreshTimeRegressedBeforeReconciliation,
    FreshCurrentTimeAlreadyExpired,
    HistoricalInvocationAuthorityAlreadyExpired,
    AttemptAlreadyExpired,
    ActuatorGenerationAlreadyExpired,
}

pub struct HistoricalInvocationActivationFailureV1<H, TQ> {
    historical: H,
    current_time: QualifiedCurrentTimeV1<TQ>,
    error: HistoricalInvocationActivationErrorV1,
}

impl<H, TQ> HistoricalInvocationActivationFailureV1<H, TQ> {
    pub const fn error(&self) -> HistoricalInvocationActivationErrorV1 {
        self.error
    }

    pub fn into_parts(self) -> (H, QualifiedCurrentTimeV1<TQ>) {
        (self.historical, self.current_time)
    }
}

pub struct HistoricallyActivatedActuatorInvocationV1<H, TQ> {
    historical: H,
    current_time: QualifiedCurrentTimeV1<TQ>,
    attempt: ActuatorInvocationAttemptManifestV1,
    kind: HistoricalInvocationActivationKindV1,
    valid_until: u64,
}

impl<H, TQ> HistoricallyActivatedActuatorInvocationV1<H, TQ> {
    pub const fn historical(&self) -> &H {
        &self.historical
    }

    pub const fn current_time(&self) -> &QualifiedCurrentTimeV1<TQ> {
        &self.current_time
    }

    pub const fn attempt(&self) -> ActuatorInvocationAttemptManifestV1 {
        self.attempt
    }

    pub const fn kind(&self) -> HistoricalInvocationActivationKindV1 {
        self.kind
    }

    pub const fn valid_until(&self) -> u64 {
        self.valid_until
    }

    pub const fn eligible_for_actuator_reservation(&self) -> bool {
        true
    }

    pub const fn external_effect_attempted(&self) -> bool {
        false
    }
}

pub fn activate_historically_reconciled_invocation<H, TQ>(
    historical: H,
    current_time: QualifiedCurrentTimeV1<TQ>,
) -> Result<
    HistoricallyActivatedActuatorInvocationV1<H, TQ>,
    HistoricalInvocationActivationFailureV1<H, TQ>,
>
where
    H: ExactHistoricallyReconciledInvocationJournalV1,
{
    let attempt = historical.exact_attempt_manifest();
    let fresh_latest = current_time.latest_possible_unix_ms();
    let reconciliation_latest = historical.reconciliation_latest_possible_unix_ms();

    let error = if attempt.schema_version != SSF_SCHEMA_V1 {
        Some(HistoricalInvocationActivationErrorV1::UnsupportedAttemptSchema)
    } else if !historical.exact_historical_binding_is_consistent() {
        Some(HistoricalInvocationActivationErrorV1::HistoricalBindingMismatch)
    } else if fresh_latest < attempt.latest_possible_unix_ms {
        Some(HistoricalInvocationActivationErrorV1::FreshTimeRegressedBeforeAttempt)
    } else if fresh_latest < reconciliation_latest {
        Some(HistoricalInvocationActivationErrorV1::FreshTimeRegressedBeforeReconciliation)
    } else if current_time.valid_until() < fresh_latest {
        Some(HistoricalInvocationActivationErrorV1::FreshCurrentTimeAlreadyExpired)
    } else if historical.invocation_eligibility_valid_until() < fresh_latest {
        Some(HistoricalInvocationActivationErrorV1::HistoricalInvocationAuthorityAlreadyExpired)
    } else if attempt.attempt_valid_until < fresh_latest {
        Some(HistoricalInvocationActivationErrorV1::AttemptAlreadyExpired)
    } else if attempt.subject.actuator.valid_until < fresh_latest {
        Some(HistoricalInvocationActivationErrorV1::ActuatorGenerationAlreadyExpired)
    } else {
        None
    };

    if let Some(error) = error {
        return Err(HistoricalInvocationActivationFailureV1 {
            historical,
            current_time,
            error,
        });
    }

    let valid_until = historical
        .invocation_eligibility_valid_until()
        .min(attempt.attempt_valid_until)
        .min(attempt.subject.actuator.valid_until)
        .min(current_time.valid_until());

    Ok(HistoricallyActivatedActuatorInvocationV1 {
        kind: historical.activation_kind(),
        historical,
        current_time,
        attempt,
        valid_until,
    })
}

#[cfg(test)]
mod tests {
    #[test]
    fn fresh_time_must_not_regress() {
        let original = 100_u64;
        let reconciliation = 120_u64;
        let fresh = 119_u64;
        assert!(fresh >= original);
        assert!(fresh < reconciliation);
    }

    #[test]
    fn final_ceiling_only_shrinks() {
        let historical = 100_u64;
        let attempt = 95_u64;
        let actuator = 90_u64;
        let time = 85_u64;
        assert_eq!(historical.min(attempt).min(actuator).min(time), 85);
    }
}
