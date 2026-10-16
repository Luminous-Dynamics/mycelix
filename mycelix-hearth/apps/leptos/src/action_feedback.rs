// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Resident-facing action outcome copy and independent publication evidence.
//!
//! These are intentionally separate dimensions. A successful Holochain zome
//! response confirms a source-chain result; publication evidence describes
//! what has separately been observed about DHT operation receipts or a scoped
//! read. One must never be inferred from the other.

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum ActionFeedbackKind {
    PreviewOnly,
    SourceChainCommitted,
    OutcomeUnknown,
    NotSent { requirement: &'static str },
}

pub(crate) fn feedback_copy(kind: ActionFeedbackKind, subject: &str) -> String {
    match kind {
        ActionFeedbackKind::PreviewOnly => {
            format!("Preview only: {subject} changed local demo state; nothing was synced.")
        }
        ActionFeedbackKind::SourceChainCommitted => format!(
            "{subject} saved. Sharing with other peers may still be in progress."
        ),
        ActionFeedbackKind::OutcomeUnknown => format!(
            "We couldn't confirm whether {subject} succeeded. Check its current state before retrying."
        ),
        ActionFeedbackKind::NotSent { requirement } => format!(
            "Not sent: {subject}. {requirement} is required; nothing was sent."
        ),
    }
}

/// A snapshot of independently queried publication evidence. These states never
/// authorize actions and must not be inferred from a successful mutation result.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum PublicationEvidence {
    Checking,
    Unavailable,
    NotAvailableToCaller,
    NoOperationData {
        checked_at_unix_seconds: u64,
    },
    ReceiptsObserved {
        operation_count: u32,
        receipt_count: u32,
        valid_receipt_count: u32,
        rejected_receipt_count: u32,
        abandoned_receipt_count: u32,
        complete_operation_count: u32,
        rejected_operation_count: u32,
        abandoned_operation_count: u32,
        checked_at_unix_seconds: u64,
    },
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct PublicationEvidenceNotice {
    pub subject: String,
    /// Short display-only prefix of the action hash; never used for dispatch.
    pub action_hash_prefix: String,
    pub evidence: PublicationEvidence,
}

fn age_description(checked_at: u64, now: u64) -> String {
    if checked_at > now {
        return "The observation timestamp is invalid, so freshness is unknown.".into();
    }
    "This evidence is a snapshot and may become stale.".into()
}

/// Resident-friendly copy from a timestamped publication evidence snapshot.
pub(crate) fn publication_evidence_copy(
    evidence: PublicationEvidence,
    now_unix_seconds: u64,
) -> String {
    match evidence {
        PublicationEvidence::Checking => {
            "Checking the authoring conductor for publication evidence…".into()
        }
        PublicationEvidence::Unavailable => {
            "Publication evidence is unavailable. The action's saved result is unchanged; peer availability is unknown.".into()
        }
        PublicationEvidence::NotAvailableToCaller => {
            "Publication evidence is not available to this caller. No peer-sharing conclusion can be drawn.".into()
        }
        PublicationEvidence::NoOperationData { checked_at_unix_seconds } => format!(
            "The conductor returned no DHT operation receipt data. Peer availability is unknown. {}",
            age_description(checked_at_unix_seconds, now_unix_seconds)
        ),
        PublicationEvidence::ReceiptsObserved {
            operation_count: 0,
            checked_at_unix_seconds,
            ..
        } => format!(
            "No publication-operation evidence was available. Peer availability is unknown. {}",
            age_description(checked_at_unix_seconds, now_unix_seconds)
        ),
        PublicationEvidence::ReceiptsObserved {
            operation_count,
            receipt_count,
            valid_receipt_count,
            rejected_receipt_count,
            abandoned_receipt_count,
            complete_operation_count,
            rejected_operation_count,
            abandoned_operation_count,
            checked_at_unix_seconds,
        } => {
            let progress = if rejected_operation_count > 0 || abandoned_operation_count > 0 {
                format!(
                    "{complete_operation_count} of {operation_count} DHT operations reached their receipt threshold; rejection evidence affects {rejected_operation_count} operation(s), and abandoned evidence affects {abandoned_operation_count} operation(s)."
                )
            } else if complete_operation_count == operation_count {
                format!(
                    "The conductor reports the required receipt threshold for all {operation_count} DHT operation(s) at the last check."
                )
            } else {
                format!(
                    "{complete_operation_count} of {operation_count} DHT operations reached their receipt threshold at the last check."
                )
            };
            format!(
                "{progress} Observed {receipt_count} receipt record(s): {valid_receipt_count} valid, {rejected_receipt_count} rejected, {abandoned_receipt_count} abandoned. This is evidence from the authoring conductor, not proof every peer currently has the update. {}",
                age_description(checked_at_unix_seconds, now_unix_seconds)
            )
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{
        feedback_copy, publication_evidence_copy, ActionFeedbackKind, PublicationEvidence,
    };

    #[test]
    fn preview_copy_cannot_be_mistaken_for_synced_success() {
        let copy = feedback_copy(ActionFeedbackKind::PreviewOnly, "the care task");
        assert!(copy.starts_with("Preview only:"));
        assert!(copy.contains("local demo state"));
        assert!(copy.contains("nothing was synced"));
    }

    #[test]
    fn source_chain_commit_does_not_claim_peer_propagation() {
        let copy = feedback_copy(
            ActionFeedbackKind::SourceChainCommitted,
            "Care task completion",
        );
        assert!(copy.contains("Care task completion saved."));
        assert!(copy.contains("Sharing with other peers may still be in progress"));
        assert!(!copy.to_lowercase().contains("globally synchronized"));
    }

    #[test]
    fn ambiguous_error_never_claims_failure_or_success() {
        let copy = feedback_copy(ActionFeedbackKind::OutcomeUnknown, "the presence update");
        assert!(copy.contains("couldn't confirm whether"));
        assert!(copy.contains("check its current state before retrying"));
        assert!(!copy.contains("failed"));
        assert!(!copy.contains("confirmed."));
    }

    #[test]
    fn missing_identity_is_stated_as_not_sent() {
        let copy = feedback_copy(
            ActionFeedbackKind::NotSent {
                requirement: "a verified recipient identity",
            },
            "this invitation",
        );
        assert!(copy.starts_with("Not sent:"));
        assert!(copy.contains("a verified recipient identity is required"));
        assert!(copy.contains("nothing was sent"));
    }

    #[test]
    fn all_action_subjects_get_every_action_outcome_variant() {
        let subjects = [
            "bond update",
            "gratitude expression",
            "vote submission",
            "presence update",
            "care task completion",
            "invitation",
        ];
        let variants = [
            (ActionFeedbackKind::PreviewOnly, "Preview only:"),
            (ActionFeedbackKind::SourceChainCommitted, "saved."),
            (ActionFeedbackKind::OutcomeUnknown, "couldn't confirm whether"),
            (
                ActionFeedbackKind::NotSent {
                    requirement: "sufficient information",
                },
                "Not sent:",
            ),
        ];

        for subject in subjects {
            for (kind, required_text) in variants {
                let copy = feedback_copy(kind, subject);
                assert!(!copy.trim().is_empty(), "empty feedback for {subject}");
                assert!(
                    copy.contains(required_text),
                    "unexpected copy for {subject} / {kind:?}: {copy}"
                );
            }
        }
    }

    #[test]
    fn checking_and_unavailable_are_distinct_from_receipt_results() {
        assert!(publication_evidence_copy(PublicationEvidence::Checking, 200).contains("Checking"));
        let unavailable = publication_evidence_copy(PublicationEvidence::Unavailable, 200);
        assert!(unavailable.contains("unavailable"));
        assert!(unavailable.contains("peer availability is unknown"));
        assert!(!unavailable.contains("failed"));
        assert!(publication_evidence_copy(PublicationEvidence::NotAvailableToCaller, 200)
            .contains("not available to this caller"));
    }

    #[test]
    fn checked_but_empty_is_explicitly_unknown_not_complete() {
        let copy = publication_evidence_copy(
            PublicationEvidence::NoOperationData {
                checked_at_unix_seconds: 100,
            },
            104,
        );
        assert!(copy.contains("no DHT operation receipt data"));
        assert!(copy.contains("Peer availability is unknown"));
        assert!(copy.contains("snapshot and may become stale"));
        assert!(!copy.contains("threshold for all"));
    }

    #[test]
    fn single_receipt_does_not_claim_global_synchronization() {
        let copy = publication_evidence_copy(
            PublicationEvidence::ReceiptsObserved {
                operation_count: 3,
                receipt_count: 1,
                valid_receipt_count: 1,
                rejected_receipt_count: 0,
                abandoned_receipt_count: 0,
                complete_operation_count: 0,
                rejected_operation_count: 0,
                abandoned_operation_count: 0,
                checked_at_unix_seconds: 100,
            },
            103,
        );
        assert!(copy.contains("0 of 3 DHT operations"));
        assert!(copy.contains("Observed 1 receipt record"));
        assert!(copy.contains("not proof every peer currently has the update"));
    }

    #[test]
    fn all_thresholds_complete_still_has_global_scope_caveat() {
        let copy = publication_evidence_copy(
            PublicationEvidence::ReceiptsObserved {
                operation_count: 4,
                receipt_count: 20,
                valid_receipt_count: 20,
                rejected_receipt_count: 0,
                abandoned_receipt_count: 0,
                complete_operation_count: 4,
                rejected_operation_count: 0,
                abandoned_operation_count: 0,
                checked_at_unix_seconds: 100,
            },
            160,
        );
        assert!(copy.contains("receipt threshold for all 4 DHT operation"));
        assert!(copy.contains("not proof every peer currently has the update"));
        assert!(copy.contains("snapshot and may become stale"));
    }

    #[test]
    fn rejection_and_abandonment_remain_visible() {
        let copy = publication_evidence_copy(
            PublicationEvidence::ReceiptsObserved {
                operation_count: 3,
                receipt_count: 7,
                valid_receipt_count: 4,
                rejected_receipt_count: 2,
                abandoned_receipt_count: 1,
                complete_operation_count: 1,
                rejected_operation_count: 1,
                abandoned_operation_count: 1,
                checked_at_unix_seconds: 100,
            },
            100,
        );
        assert!(copy.contains("rejection evidence affects 1 operation"));
        assert!(copy.contains("abandoned evidence affects 1 operation"));
        assert!(copy.contains("4 valid, 2 rejected, 1 abandoned"));
    }

    #[test]
    fn invalid_future_timestamp_never_appears_fresh() {
        let copy = publication_evidence_copy(
            PublicationEvidence::NoOperationData {
                checked_at_unix_seconds: 200,
            },
            199,
        );
        assert!(copy.contains("timestamp is invalid"));
        assert!(copy.contains("freshness is unknown"));
    }
}
