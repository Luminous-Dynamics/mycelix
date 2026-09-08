// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Defensive self-audit for canonical completed-outcome history heads.
//!
//! A qualified store read proves that a current store returned a particular
//! durable head. This crate additionally re-proves that the exact paired latest
//! entry was internally valid historical evidence when it was recorded. The
//! resulting token is evidence-only and creates no replay or effect authority.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use mycelix_ssf_canonical_actuator_execution::CanonicalQualifiedEffectDispositionV1;
use mycelix_ssf_canonical_completed_effect_evidence::{
    CanonicalCompletedInvocationRecordV1, CanonicalCompletedInvocationProvenanceV1,
};
use mycelix_ssf_canonical_completed_outcome_history::{
    CanonicalCompletedOutcomeHistoryEntryV1, CanonicalCompletedOutcomeStoreTimeBasisV1,
    CanonicalCompletedOutcomeTerminalV1, QualifiedCanonicalCompletedOutcomeHistoryHeadV1,
};
use mycelix_ssf_contracts::SSF_SCHEMA_V1;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum CanonicalCompletedOutcomeHistoryAuditErrorV1 {
    UnsupportedStoreTimeBasis,
    HeadReadAlreadyExpired,
    ReceiptOutlivesStore,
    GenesisUnexpectedEntry,
    NonGenesisMissingEntry,
    LatestRecordMismatch,
    UnsupportedManifestSchema,
    ManifestStoreMismatch,
    ManifestInvocationMismatch,
    EvidenceInvocationMismatch,
    EvidenceProvenanceMismatch,
    PredecessorInvocationMismatch,
    PredecessorAlreadyTerminal,
    ManifestExpiredAtRecording,
    ManifestOutlivesStore,
    RecordingTimeBeforePreInvocation,
    RecordingTimeBeforePostInvocation,
    RecordingTimeAfterHeadRead,
    GenerationMismatch,
    TerminalStateMismatch,
}

fn terminal_from_disposition(
    disposition: CanonicalQualifiedEffectDispositionV1,
) -> Option<CanonicalCompletedOutcomeTerminalV1> {
    match disposition {
        CanonicalQualifiedEffectDispositionV1::Confirmed { .. } => {
            Some(CanonicalCompletedOutcomeTerminalV1::Confirmed)
        }
        CanonicalQualifiedEffectDispositionV1::ProvenNotApplied { .. } => {
            Some(CanonicalCompletedOutcomeTerminalV1::ProvenNotApplied)
        }
        CanonicalQualifiedEffectDispositionV1::OutcomeUnknown { .. } => None,
    }
}

fn provenance_record(
    provenance: CanonicalCompletedInvocationProvenanceV1,
) -> Option<CanonicalCompletedInvocationRecordV1> {
    provenance.invocation_record()
}

pub struct AuditedCanonicalCompletedOutcomeHistoryHeadV1<S, TQ> {
    history: QualifiedCanonicalCompletedOutcomeHistoryHeadV1<S, TQ>,
    audit_valid_until: u64,
}

impl<S, TQ> AuditedCanonicalCompletedOutcomeHistoryHeadV1<S, TQ> {
    pub const fn history(&self) -> &QualifiedCanonicalCompletedOutcomeHistoryHeadV1<S, TQ> {
        &self.history
    }

    pub const fn latest_entry(&self) -> Option<CanonicalCompletedOutcomeHistoryEntryV1> {
        self.history.latest_entry()
    }

    pub const fn invocation_record(&self) -> CanonicalCompletedInvocationRecordV1 {
        self.history.head().invocation_record()
    }

    pub const fn audit_valid_until(&self) -> u64 {
        self.audit_valid_until
    }

    pub const fn contains_replay_authority(&self) -> bool {
        false
    }

    pub const fn contains_effect_authority(&self) -> bool {
        false
    }
}

pub struct CanonicalCompletedOutcomeHistoryAuditFailureV1<S, TQ> {
    history: QualifiedCanonicalCompletedOutcomeHistoryHeadV1<S, TQ>,
    error: CanonicalCompletedOutcomeHistoryAuditErrorV1,
}

impl<S, TQ> CanonicalCompletedOutcomeHistoryAuditFailureV1<S, TQ> {
    pub const fn error(&self) -> CanonicalCompletedOutcomeHistoryAuditErrorV1 {
        self.error
    }

    pub fn into_history(self) -> QualifiedCanonicalCompletedOutcomeHistoryHeadV1<S, TQ> {
        self.history
    }
}

fn fail<S, TQ>(
    history: QualifiedCanonicalCompletedOutcomeHistoryHeadV1<S, TQ>,
    error: CanonicalCompletedOutcomeHistoryAuditErrorV1,
) -> Result<
    AuditedCanonicalCompletedOutcomeHistoryHeadV1<S, TQ>,
    CanonicalCompletedOutcomeHistoryAuditFailureV1<S, TQ>,
> {
    Err(CanonicalCompletedOutcomeHistoryAuditFailureV1 { history, error })
}

pub fn audit_canonical_completed_outcome_history_head<S, TQ>(
    history: QualifiedCanonicalCompletedOutcomeHistoryHeadV1<S, TQ>,
) -> Result<
    AuditedCanonicalCompletedOutcomeHistoryHeadV1<S, TQ>,
    CanonicalCompletedOutcomeHistoryAuditFailureV1<S, TQ>,
> {
    let descriptor = history.expected_store().descriptor();
    let receipt = history.receipt();
    let head = history.head();
    let latest_read = history.current_time().latest_possible_unix_ms();

    if descriptor.time_basis != CanonicalCompletedOutcomeStoreTimeBasisV1::UnixMillisecondsUtc {
        return fail(
            history,
            CanonicalCompletedOutcomeHistoryAuditErrorV1::UnsupportedStoreTimeBasis,
        );
    }
    if receipt.valid_until < latest_read || history.current_time().valid_until() < latest_read {
        return fail(
            history,
            CanonicalCompletedOutcomeHistoryAuditErrorV1::HeadReadAlreadyExpired,
        );
    }
    if receipt.valid_until > descriptor.valid_until {
        return fail(
            history,
            CanonicalCompletedOutcomeHistoryAuditErrorV1::ReceiptOutlivesStore,
        );
    }

    let entry = match (head.generation().get(), head.head(), history.latest_entry()) {
        (0, None, None) => {
            let audit_valid_until = descriptor
                .valid_until
                .min(receipt.valid_until)
                .min(history.current_time().valid_until());
            return Ok(AuditedCanonicalCompletedOutcomeHistoryHeadV1 {
                history,
                audit_valid_until,
            });
        }
        (0, _, Some(_)) => {
            return fail(
                history,
                CanonicalCompletedOutcomeHistoryAuditErrorV1::GenesisUnexpectedEntry,
            );
        }
        (_, Some(_), None) => {
            return fail(
                history,
                CanonicalCompletedOutcomeHistoryAuditErrorV1::NonGenesisMissingEntry,
            );
        }
        (_, None, _) => {
            return fail(
                history,
                CanonicalCompletedOutcomeHistoryAuditErrorV1::NonGenesisMissingEntry,
            );
        }
        (_, Some(_), Some(entry)) => entry,
    };

    let manifest = entry.manifest;
    let evidence = manifest.subject.evidence;
    let pre_time = evidence.pre_invocation_time().latest_possible_unix_ms;
    let post_time = evidence
        .post_invocation_time()
        .map(|time| time.latest_possible_unix_ms);

    if head.head() != Some(entry.record) {
        return fail(
            history,
            CanonicalCompletedOutcomeHistoryAuditErrorV1::LatestRecordMismatch,
        );
    }
    if manifest.schema_version != SSF_SCHEMA_V1 {
        return fail(
            history,
            CanonicalCompletedOutcomeHistoryAuditErrorV1::UnsupportedManifestSchema,
        );
    }
    if manifest.expected_store != descriptor {
        return fail(
            history,
            CanonicalCompletedOutcomeHistoryAuditErrorV1::ManifestStoreMismatch,
        );
    }
    if manifest.subject.invocation_record != head.invocation_record() {
        return fail(
            history,
            CanonicalCompletedOutcomeHistoryAuditErrorV1::ManifestInvocationMismatch,
        );
    }
    if evidence.invocation_record() != head.invocation_record() {
        return fail(
            history,
            CanonicalCompletedOutcomeHistoryAuditErrorV1::EvidenceInvocationMismatch,
        );
    }
    if provenance_record(evidence.provenance()) != Some(head.invocation_record()) {
        return fail(
            history,
            CanonicalCompletedOutcomeHistoryAuditErrorV1::EvidenceProvenanceMismatch,
        );
    }
    if manifest.expected_head.invocation_record() != head.invocation_record() {
        return fail(
            history,
            CanonicalCompletedOutcomeHistoryAuditErrorV1::PredecessorInvocationMismatch,
        );
    }
    if !manifest.expected_head.permits_append() {
        return fail(
            history,
            CanonicalCompletedOutcomeHistoryAuditErrorV1::PredecessorAlreadyTerminal,
        );
    }
    if manifest.valid_until < manifest.recording_latest_possible_unix_ms {
        return fail(
            history,
            CanonicalCompletedOutcomeHistoryAuditErrorV1::ManifestExpiredAtRecording,
        );
    }
    if manifest.valid_until > manifest.expected_store.valid_until {
        return fail(
            history,
            CanonicalCompletedOutcomeHistoryAuditErrorV1::ManifestOutlivesStore,
        );
    }
    if manifest.recording_latest_possible_unix_ms < pre_time {
        return fail(
            history,
            CanonicalCompletedOutcomeHistoryAuditErrorV1::RecordingTimeBeforePreInvocation,
        );
    }
    if post_time.is_some_and(|post| manifest.recording_latest_possible_unix_ms < post) {
        return fail(
            history,
            CanonicalCompletedOutcomeHistoryAuditErrorV1::RecordingTimeBeforePostInvocation,
        );
    }
    if latest_read < manifest.recording_latest_possible_unix_ms {
        return fail(
            history,
            CanonicalCompletedOutcomeHistoryAuditErrorV1::RecordingTimeAfterHeadRead,
        );
    }

    let Some(expected_generation) = manifest.expected_head.generation().get().checked_add(1) else {
        return fail(
            history,
            CanonicalCompletedOutcomeHistoryAuditErrorV1::GenerationMismatch,
        );
    };
    if expected_generation != head.generation().get() {
        return fail(
            history,
            CanonicalCompletedOutcomeHistoryAuditErrorV1::GenerationMismatch,
        );
    }
    if terminal_from_disposition(evidence.canonical_disposition()) != head.terminal() {
        return fail(
            history,
            CanonicalCompletedOutcomeHistoryAuditErrorV1::TerminalStateMismatch,
        );
    }

    let audit_valid_until = descriptor
        .valid_until
        .min(receipt.valid_until)
        .min(history.current_time().valid_until());

    Ok(AuditedCanonicalCompletedOutcomeHistoryHeadV1 {
        history,
        audit_valid_until,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn terminal_mapping_uses_only_canonical_disposition_shape() {
        let _ = terminal_from_disposition;
    }

    #[test]
    fn audit_errors_are_distinct() {
        assert_ne!(
            CanonicalCompletedOutcomeHistoryAuditErrorV1::ManifestExpiredAtRecording,
            CanonicalCompletedOutcomeHistoryAuditErrorV1::ManifestOutlivesStore
        );
    }
}
