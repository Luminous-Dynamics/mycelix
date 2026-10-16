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

/// Evidence about propagation/current-view state, separate from operation outcome.
///
/// Current runtime action handlers do not yet fetch this evidence. In
/// particular, a successful zome response must not construct ReceiptsObserved.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum PublicationEvidence {
    NotChecked,
    Unavailable,
    ReceiptsObserved {
        operation_count: u32,
        receipt_count: u32,
        complete_operation_count: u32,
        rejected_operation_count: u32,
        checked_at_unix_seconds: u64,
    },
    CurrentStateObserved {
        source: &'static str,
        checked_at_unix_seconds: u64,
    },
}

fn age_description(checked_at: u64, now: u64) -> String {
    if checked_at > now {
        return "The observation timestamp is invalid, so freshness is unknown.".into();
    }
    let age = now - checked_at;
    let unit = if age == 1 { "second" } else { "seconds" };
    format!("Checked {age} {unit} ago.")
}

/// Copy for a separately observed publication/read state. The caller supplies
/// an observed clock, allowing tests and UI freshness displays to be deterministic.
pub(crate) fn publication_evidence_copy(
    evidence: PublicationEvidence,
    now_unix_seconds: u64,
) -> String {
    match evidence {
        PublicationEvidence::NotChecked => {
            "Peer sharing status has not been checked.".into()
        }
        PublicationEvidence::Unavailable => {
            "Peer sharing status is unavailable here; remote availability is unknown.".into()
        }
        PublicationEvidence::ReceiptsObserved {
            operation_count: 0,
            ..
        } => {
            "No publication-operation evidence was returned; peer availability is unknown.".into()
        }
        PublicationEvidence::ReceiptsObserved {
            operation_count,
            receipt_count,
            complete_operation_count,
            rejected_operation_count,
            checked_at_unix_seconds,
        } => {
            let state = if rejected_operation_count > 0 {
                format!(
                    "Holochain reports rejection evidence for {rejected_operation_count} DHT operation(s). This is not proof that every operation was rejected."
                )
            } else if complete_operation_count == operation_count {
                format!(
                    "Holochain reported the required validation receipts for all {operation_count} DHT operation(s) at the last check."
                )
            } else {
                format!(
                    "{receipt_count} validation receipt(s) were observed across {operation_count} DHT operation(s); {complete_operation_count} operation(s) reached their receipt threshold at the last check."
                )
            };
            format!(
                "{state} This does not prove every peer currently has the update. {}",
                age_description(checked_at_unix_seconds, now_unix_seconds)
            )
        }
        PublicationEvidence::CurrentStateObserved {
            source,
            checked_at_unix_seconds,
        } => format!(
            "The current state was observed by {source}. This confirms that query view only; other peers may have a different view. {}",
            age_description(checked_at_unix_seconds, now_unix_seconds)
        ),
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
        assert!(!copy.ends_with("confirmed."));
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
        assert!(!copy.contains("every peer has the update"));
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
            (
                ActionFeedbackKind::OutcomeUnknown,
                "couldn't confirm whether",
            ),
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
    fn one_peer_receipt_never_claims_global_synchronization() {
        let copy = publication_evidence_copy(
            PublicationEvidence::ReceiptsObserved {
                operation_count: 3,
                receipt_count: 1,
                complete_operation_count: 0,
                rejected_operation_count: 0,
                checked_at_unix_seconds: 100,
            },
            103,
        );
        assert!(copy.contains("1 validation receipt"));
        assert!(copy.contains("does not prove every peer currently has the update"));
        assert!(copy.contains("Checked 3 seconds ago."));
        assert!(!copy.to_lowercase().contains("globally synchronized"));
    }

    #[test]
    fn all_operations_receipt_complete_still_does_not_claim_global_availability() {
        let copy = publication_evidence_copy(
            PublicationEvidence::ReceiptsObserved {
                operation_count: 4,
                receipt_count: 20,
                complete_operation_count: 4,
                rejected_operation_count: 0,
                checked_at_unix_seconds: 100,
            },
            160,
        );
        assert!(copy.contains("required validation receipts for all 4 DHT operation(s)"));
        assert!(copy.contains("does not prove every peer currently has the update"));
        assert!(copy.contains("Checked 60 seconds ago."));
    }

    #[test]
    fn no_operation_data_or_zero_receipts_are_not_inferred_as_success() {
        let no_ops = publication_evidence_copy(
            PublicationEvidence::ReceiptsObserved {
                operation_count: 0,
                receipt_count: 0,
                complete_operation_count: 0,
                rejected_operation_count: 0,
                checked_at_unix_seconds: 100,
            },
            100,
        );
        assert!(no_ops.contains("peer availability is unknown"));

        let pending = publication_evidence_copy(
            PublicationEvidence::ReceiptsObserved {
                operation_count: 2,
                receipt_count: 0,
                complete_operation_count: 0,
                rejected_operation_count: 0,
                checked_at_unix_seconds: 100,
            },
            100,
        );
        assert!(pending.contains("0 validation receipt(s)"));
        assert!(pending.contains("does not prove every peer currently has the update"));
    }

    #[test]
    fn rejected_operation_receipt_does_not_overstate_action_level_rejection() {
        let copy = publication_evidence_copy(
            PublicationEvidence::ReceiptsObserved {
                operation_count: 2,
                receipt_count: 5,
                complete_operation_count: 1,
                rejected_operation_count: 1,
                checked_at_unix_seconds: 100,
            },
            100,
        );
        assert!(copy.contains("rejection evidence for 1 DHT operation(s)"));
        assert!(copy.contains("not proof that every operation was rejected"));
    }

    #[test]
    fn current_state_read_is_scoped_and_timestamped() {
        let copy = publication_evidence_copy(
            PublicationEvidence::CurrentStateObserved {
                source: "the selected-Hearth read",
                checked_at_unix_seconds: 90,
            },
            92,
        );
        assert!(copy.contains("observed by the selected-Hearth read"));
        assert!(copy.contains("query view only"));
        assert!(copy.contains("other peers may have a different view"));
        assert!(copy.contains("Checked 2 seconds ago."));
    }

    #[test]
    fn invalid_future_timestamp_never_appears_fresh() {
        let copy = publication_evidence_copy(
            PublicationEvidence::CurrentStateObserved {
                source: "the selected-Hearth read",
                checked_at_unix_seconds: 200,
            },
            199,
        );
        assert!(copy.contains("timestamp is invalid"));
        assert!(copy.contains("freshness is unknown"));
    }

    #[test]
    fn absent_or_unavailable_receipt_sources_do_not_imply_remote_failure() {
        assert!(publication_evidence_copy(PublicationEvidence::NotChecked, 200).contains("not been checked"));
        let unavailable = publication_evidence_copy(PublicationEvidence::Unavailable, 200);
        assert!(unavailable.contains("is unavailable here"));
        assert!(unavailable.contains("remote availability is unknown"));
        assert!(!unavailable.contains("failed"));
    }
}
