// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Typed provider/item envelope for the task-first Mycelix Inbox.
//!
//! These types model user-facing presentation state and safe handoff only.
//! They do not acknowledge provider records, resolve domain work, authorize
//! capabilities, deliver messages, or execute consequential actions.

/// Local presentation state for an Inbox item.
///
/// `Seen` means only that the shell has locally presented/marked the item as
/// seen. It is not provider acknowledgement, acceptance, or resolution.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum InboxSeenState {
    Unread,
    Seen,
}

impl InboxSeenState {
    pub fn label(self) -> &'static str {
        match self {
            Self::Unread => "Unread",
            Self::Seen => "Seen",
        }
    }
}

/// Provider-owned cue describing whether a user response is useful.
///
/// This is a presentation cue, not a universal domain lifecycle or severity.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum InboxResponseCue {
    NoneRequired,
    ReviewRecommended,
    ResponseRequested,
}

impl InboxResponseCue {
    pub fn label(self) -> &'static str {
        match self {
            Self::NoneRequired => "No response required",
            Self::ReviewRecommended => "Review recommended",
            Self::ResponseRequested => "Response requested",
        }
    }
}

/// Safe handoff produced by selecting an Inbox item.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum InboxTarget {
    Navigate { href: String },
    Preview {
        preview_kind: String,
        record_ref: String,
    },
    ResumeDraft {
        draft_kind: String,
        draft_ref: String,
    },
}

/// One provider-owned record projected into the shared Inbox.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct InboxItem {
    pub id: String,
    /// Source provenance for the envelope, not a trust/authority score.
    pub provider_id: String,
    pub title: String,
    pub detail: Option<String>,
    pub local_seen: InboxSeenState,
    pub response: InboxResponseCue,
    /// Provider-owned human-readable state copy. The shell may present it but
    /// must not reinterpret it into a stronger shared lifecycle.
    pub provider_state_label: Option<String>,
    /// Presentation timestamps only; neither establishes urgency or authority.
    pub occurred_at_unix_ms: Option<u64>,
    pub updated_at_unix_ms: Option<u64>,
    pub target: InboxTarget,
}

impl InboxItem {
    pub fn navigation(
        provider_id: impl Into<String>,
        id: impl Into<String>,
        title: impl Into<String>,
        href: impl Into<String>,
    ) -> Self {
        Self {
            id: id.into(),
            provider_id: provider_id.into(),
            title: title.into(),
            detail: None,
            local_seen: InboxSeenState::Unread,
            response: InboxResponseCue::NoneRequired,
            provider_state_label: None,
            occurred_at_unix_ms: None,
            updated_at_unix_ms: None,
            target: InboxTarget::Navigate { href: href.into() },
        }
    }

    /// Return a locally presentation-updated copy without changing any
    /// provider-owned lifecycle fields.
    pub fn with_seen(mut self, state: InboxSeenState) -> Self {
        self.local_seen = state;
        self
    }
}

/// What the shell can currently establish about one Inbox provider.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum InboxProviderState {
    Ready,
    Unavailable,
    Unknown,
}

impl InboxProviderState {
    pub fn label(self) -> &'static str {
        match self {
            Self::Ready => "Ready",
            Self::Unavailable => "Unavailable",
            Self::Unknown => "Unknown",
        }
    }
}

/// One provider's Inbox contribution plus explicit provider state.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct InboxBatch {
    pub provider_id: String,
    pub state: InboxProviderState,
    pub items: Vec<InboxItem>,
}

impl InboxBatch {
    pub fn ready(provider_id: impl Into<String>, items: Vec<InboxItem>) -> Self {
        Self {
            provider_id: provider_id.into(),
            state: InboxProviderState::Ready,
            items,
        }
    }

    pub fn unavailable(provider_id: impl Into<String>) -> Self {
        Self {
            provider_id: provider_id.into(),
            state: InboxProviderState::Unavailable,
            items: Vec::new(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{
        InboxBatch, InboxItem, InboxProviderState, InboxResponseCue, InboxSeenState, InboxTarget,
    };

    #[test]
    fn ready_empty_is_distinct_from_unavailable() {
        let empty = InboxBatch::ready("messages", vec![]);
        let unavailable = InboxBatch::unavailable("messages");

        assert_eq!(empty.state, InboxProviderState::Ready);
        assert_eq!(unavailable.state, InboxProviderState::Unavailable);
        assert_ne!(empty.state, unavailable.state);
    }

    #[test]
    fn marking_seen_changes_only_local_presentation_state() {
        let original = InboxItem::navigation(
            "governance",
            "proposal-42",
            "Proposal 42 updated",
            "/proposals/42",
        );
        let seen = original.clone().with_seen(InboxSeenState::Seen);

        assert_eq!(original.local_seen, InboxSeenState::Unread);
        assert_eq!(seen.local_seen, InboxSeenState::Seen);
        assert_eq!(seen.provider_id, original.provider_id);
        assert_eq!(seen.target, original.target);
        assert_eq!(seen.provider_state_label, original.provider_state_label);
    }

    #[test]
    fn response_cue_is_not_execution() {
        let mut item = InboxItem::navigation(
            "hearth",
            "care-7",
            "Care request needs review",
            "/care/7",
        );
        item.response = InboxResponseCue::ResponseRequested;

        assert_eq!(item.response.label(), "Response requested");
        assert!(matches!(item.target, InboxTarget::Navigate { .. }));
    }

    #[test]
    fn shared_target_set_contains_no_direct_execution_variant() {
        let target = InboxTarget::Preview {
            preview_kind: "governance.vote".into(),
            record_ref: "proposal-42".into(),
        };

        assert!(matches!(target, InboxTarget::Preview { .. }));
    }
}
