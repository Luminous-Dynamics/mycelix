// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Resident-facing action result copy.
//!
//! Keep the words tied to evidence: local demo mutations are not synchronized,
//! errors without a typed rejection proof have unknown outcome, and an action is
//! confirmed only after its call returns success.

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum ActionFeedbackKind {
    PreviewOnly,
    Confirmed,
    OutcomeUnknown,
    NotSent,
}

pub(crate) fn feedback_copy(kind: ActionFeedbackKind, subject: &str) -> String {
    match kind {
        ActionFeedbackKind::PreviewOnly => {
            format!("Preview only: {subject} changed local demo state; nothing was synced.")
        }
        ActionFeedbackKind::Confirmed => format!("{subject} confirmed."),
        ActionFeedbackKind::OutcomeUnknown => format!(
            "We couldn't confirm whether {subject} succeeded. Check its current state before retrying."
        ),
        ActionFeedbackKind::NotSent => format!(
            "Not sent: {subject}. Choose a verified recipient identity before sending."
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
    fn confirmed_copy_is_reserved_for_successful_response() {
        assert_eq!(
            feedback_copy(ActionFeedbackKind::Confirmed, "Care task completion"),
            "Care task completion confirmed."
        );
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
        let copy = feedback_copy(ActionFeedbackKind::NotSent, "this invitation");
        assert!(copy.starts_with("Not sent:"));
        assert!(copy.contains("verified recipient identity"));
    }
}
