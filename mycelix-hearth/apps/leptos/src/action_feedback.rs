// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Resident-facing action result copy.
//!
//! Keep the words tied to evidence. A successful Holochain zome response
//! confirms the source-chain operation returned successfully; it does not prove
//! remote peers have validated or integrated the operation.

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum ActionFeedbackKind {
    PreviewOnly,
    SourceChainCommitted,

    OutcomeUnknown,
    NotSent { requirement: &'static str },
    /// At least this many peer validation receipts were actually observed.
    /// A receipt count is propagation evidence, not global synchronization.
    PeerValidationObserved { receipt_count: u32 },
    /// A named, correctly scoped read observed the current state. This is not a
    /// guarantee that every other peer has the same view.
    CurrentStateObserved { source: &'static str },
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
        ActionFeedbackKind::PeerValidationObserved { receipt_count: 0 } => (
            "No peer validation receipts are confirmed here; peer availability is not established.".into()
        ),
        ActionFeedbackKind::PeerValidationObserved { receipt_count } => {
            let noun = if receipt_count == 1 { "peer" } else { "peers" };
            format!(
                "Validation receipts from {receipt_count} {noun} were observed. This does not prove every peer has the update."
            )
        }
        ActionFeedbackKind::CurrentStateObserved { source } => format!(
            "The current state was observed by {source}. This confirms this view only; other peers may have a different view."
        ),
    }
}

#[cfg(test)]
mod tests {
    use super::{feedback_copy, ActionFeedbackKind};

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
        assert!(copy.contains("saved."));
        assert!(copy.contains("Sharing with other peers may still be in progress"));
        assert!(!copy.to_lowercase().contains("globally synchronized"));
        assert!(!copy.contains("every peer has the update"));
    }

    #[test]
    fn one_validation_receipt_does_not_claim_global_synchronization() {
        let copy = feedback_copy(
            ActionFeedbackKind::PeerValidationObserved { receipt_count: 1 },
            "presence update",
        );
        assert!(copy.contains("receipts from 1 peer"));
        assert!(copy.contains("does not prove every peer has the update"));
        assert!(!copy.to_lowercase().contains("globally synchronized"));
    }

    #[test]
    fn zero_validation_receipts_are_not_reported_as_peer_confirmation() {
        let copy = feedback_copy(
            ActionFeedbackKind::PeerValidationObserved { receipt_count: 0 },
            "presence update",
        );
        assert!(copy.contains("No peer validation receipts"));
        assert!(copy.contains("not established"));
    }

    #[test]
    fn scoped_read_is_not_reported_as_global_peer_consistency() {
        let copy = feedback_copy(
            ActionFeedbackKind::CurrentStateObserved { source: "the selected-Hearth read" },
            "care task",
        );
        assert!(copy.contains("observed by the selected-Hearth read"));
        assert!(copy.contains("other peers may have a different view"));
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
            ActionFeedbackKind::NotSent { requirement: "a verified recipient identity" },
            "this invitation",
        );
        assert!(copy.starts_with("Not sent:"));
        assert!(copy.contains("a verified recipient identity is required"));
        assert!(copy.contains("nothing was sent"));
    }
    #[test]
    fn every_action_subject_gets_all_four_truthful_outcome_variants() {
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
            (ActionFeedbackKind::SourceChainCommitted, "saved to your Holochain source chain"),
            (ActionFeedbackKind::OutcomeUnknown, "couldn't confirm whether"),
            (ActionFeedbackKind::NotSent { requirement: "sufficient information" }, "Not sent:"),
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

}
