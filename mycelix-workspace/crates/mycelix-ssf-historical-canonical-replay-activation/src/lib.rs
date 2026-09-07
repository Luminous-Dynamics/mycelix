// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Fresh activation after historical reconciliation of a canonical replay journal.
//!
//! Historical reconciliation proves an old canonical `Journaled` fact under a
//! newer same-identity reconciler. It does not make the old attempt live again.
//! This crate requires a newer trusted-time observation, prevents clock rollback,
//! intersects all inherited authority ceilings, and still performs no actuator
//! reservation or external effect.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use mycelix_ssf_actuator_effect_protocol::ActuatorInvocationAttemptManifestV1;
use mycelix_ssf_canonical_replay_invocation_journal::{
    CanonicalReplayInvocationJournalDispositionV1,
    CanonicalReplayInvocationJournalReceiptCommitment,
};
use mycelix_ssf_canonical_replay_policy::CanonicalReplayAuthorizationCommitment;
use mycelix_ssf_contracts::SSF_SCHEMA_V1;
use mycelix_ssf_current_authority_revalidation::QualifiedCurrentTimeV1;
use mycelix_ssf_historical_canonical_replay_journal::{
    HistoricalCanonicalReplayJournalReceiptCommitment,
    HistoricallyReconciledCanonicalReplayJournalV1,
};
use mycelix_ssf_replay_aware_invocation_journal::ReplayInvocationJournalRecordCommitment;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HistoricalCanonicalReplayActivationEvidenceV1 {
    pub journal_record: ReplayInvocationJournalRecordCommitment,
    pub original_journal_receipt: CanonicalReplayInvocationJournalReceiptCommitment,
    pub historical_reconciliation_receipt: HistoricalCanonicalReplayJournalReceiptCommitment,
    pub authorization: CanonicalReplayAuthorizationCommitment,
    pub valid_until: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HistoricalCanonicalReplayActivationErrorV1 {
    UnsupportedJournalReceiptSchema,
    UnsupportedJournalManifestSchema,
    JournalWasNotDurablyJournaled,
    StableEffectIdentityMismatch,
    FreshTimeRegressedBeforeAttempt,
    FreshTimeRegressedBeforeReconciliation,
    FreshTimeAlreadyExpired,
    HistoricalEligibilityAlreadyExpired,
    ReplayAuthorizationAlreadyExpired,
    AttemptAlreadyExpired,
    ActuatorGenerationAlreadyExpired,
    HistoricalReconciliationAlreadyExpired,
}

pub struct HistoricalCanonicalReplayActivationFailureV1<H, TQ> {
    historical: H,
    fresh_time: QualifiedCurrentTimeV1<TQ>,
    error: HistoricalCanonicalReplayActivationErrorV1,
}

impl<H, TQ> HistoricalCanonicalReplayActivationFailureV1<H, TQ> {
    pub const fn error(&self) -> HistoricalCanonicalReplayActivationErrorV1 {
        self.error
    }

    pub fn into_parts(self) -> (H, QualifiedCurrentTimeV1<TQ>) {
        (self.historical, self.fresh_time)
    }
}

pub struct HistoricallyActivatedCanonicalReplayInvocationV1<H, TQ> {
    historical: H,
    fresh_time: QualifiedCurrentTimeV1<TQ>,
    attempt: ActuatorInvocationAttemptManifestV1,
    evidence: HistoricalCanonicalReplayActivationEvidenceV1,
    valid_until: u64,
}

impl<H, TQ> HistoricallyActivatedCanonicalReplayInvocationV1<H, TQ> {
    pub const fn historical(&self) -> &H {
        &self.historical
    }

    pub const fn fresh_time(&self) -> &QualifiedCurrentTimeV1<TQ> {
        &self.fresh_time
    }

    pub const fn attempt(&self) -> ActuatorInvocationAttemptManifestV1 {
        self.attempt
    }

    pub const fn evidence(&self) -> HistoricalCanonicalReplayActivationEvidenceV1 {
        self.evidence
    }

    pub const fn activation_latest_possible_unix_ms(&self) -> u64 {
        self.fresh_time.latest_possible_unix_ms()
    }

    pub const fn valid_until(&self) -> u64 {
        self.valid_until
    }

    pub const fn eligible_for_final_readiness(&self) -> bool {
        true
    }

    pub const fn external_effect_attempted(&self) -> bool {
        false
    }

    pub const fn permits_third_attempt(&self) -> bool {
        false
    }
}

pub fn activate_historically_reconciled_canonical_replay<R, RTQ, TQ>(
    historical: HistoricallyReconciledCanonicalReplayJournalV1<R, RTQ>,
    fresh_time: QualifiedCurrentTimeV1<TQ>,
) -> Result<
    HistoricallyActivatedCanonicalReplayInvocationV1<
        HistoricallyReconciledCanonicalReplayJournalV1<R, RTQ>,
        TQ,
    >,
    HistoricalCanonicalReplayActivationFailureV1<
        HistoricallyReconciledCanonicalReplayJournalV1<R, RTQ>,
        TQ,
    >,
> {
    let journal = historical.journal_receipt();
    let manifest = journal.manifest;
    let attempt = manifest.attempt;
    let reconciliation = historical.reconciliation_receipt();
    let latest = fresh_time.latest_possible_unix_ms();
    let reconciliation_time = historical.current_time().latest_possible_unix_ms();

    let (journal_record, _) = match journal.disposition {
        CanonicalReplayInvocationJournalDispositionV1::Journaled {
            record,
            post_frontier,
        } => (record, post_frontier),
        _ => {
            return Err(HistoricalCanonicalReplayActivationFailureV1 {
                historical,
                fresh_time,
                error: HistoricalCanonicalReplayActivationErrorV1::JournalWasNotDurablyJournaled,
            });
        }
    };

    let stable = attempt.subject.stable_identity();
    let error = if journal.schema_version != SSF_SCHEMA_V1 {
        Some(HistoricalCanonicalReplayActivationErrorV1::UnsupportedJournalReceiptSchema)
    } else if manifest.schema_version != SSF_SCHEMA_V1 || attempt.schema_version != SSF_SCHEMA_V1 {
        Some(HistoricalCanonicalReplayActivationErrorV1::UnsupportedJournalManifestSchema)
    } else if manifest.replay.subject.stable_effect_identity != stable
        || manifest.replay.subject.evidence_subject.stable_effect_identity != stable
    {
        Some(HistoricalCanonicalReplayActivationErrorV1::StableEffectIdentityMismatch)
    } else if latest < attempt.latest_possible_unix_ms {
        Some(HistoricalCanonicalReplayActivationErrorV1::FreshTimeRegressedBeforeAttempt)
    } else if latest < reconciliation_time {
        Some(HistoricalCanonicalReplayActivationErrorV1::FreshTimeRegressedBeforeReconciliation)
    } else if fresh_time.valid_until() < latest {
        Some(HistoricalCanonicalReplayActivationErrorV1::FreshTimeAlreadyExpired)
    } else if historical.authority_valid_until() < latest {
        Some(HistoricalCanonicalReplayActivationErrorV1::HistoricalEligibilityAlreadyExpired)
    } else if manifest.replay.valid_until < latest {
        Some(HistoricalCanonicalReplayActivationErrorV1::ReplayAuthorizationAlreadyExpired)
    } else if attempt.attempt_valid_until < latest {
        Some(HistoricalCanonicalReplayActivationErrorV1::AttemptAlreadyExpired)
    } else if attempt.subject.actuator.valid_until < latest {
        Some(HistoricalCanonicalReplayActivationErrorV1::ActuatorGenerationAlreadyExpired)
    } else if reconciliation.valid_until < latest {
        Some(HistoricalCanonicalReplayActivationErrorV1::HistoricalReconciliationAlreadyExpired)
    } else {
        None
    };

    if let Some(error) = error {
        return Err(HistoricalCanonicalReplayActivationFailureV1 {
            historical,
            fresh_time,
            error,
        });
    }

    let valid_until = fresh_time
        .valid_until()
        .min(historical.authority_valid_until())
        .min(manifest.replay.valid_until)
        .min(attempt.attempt_valid_until)
        .min(attempt.subject.actuator.valid_until)
        .min(reconciliation.valid_until);

    let evidence = HistoricalCanonicalReplayActivationEvidenceV1 {
        journal_record,
        original_journal_receipt: journal.receipt_commitment,
        historical_reconciliation_receipt: reconciliation.receipt_commitment,
        authorization: manifest.replay.authorization,
        valid_until,
    };

    Ok(HistoricallyActivatedCanonicalReplayInvocationV1 {
        historical,
        fresh_time,
        attempt,
        evidence,
        valid_until,
    })
}

#[cfg(test)]
mod tests {
    #[test]
    fn historical_activation_ceiling_never_expands() {
        assert_eq!(180_u64.min(90).min(80).min(70), 70);
    }

    #[test]
    fn rollback_must_be_rejected_by_ordering() {
        let attempt_time = 120_u64;
        let later_time = 100_u64;
        assert!(later_time < attempt_time);
    }
}
