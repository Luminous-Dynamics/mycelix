// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Typed presentation contract for the Mycelix Global Finder.
//!
//! These types carry discoverable presentation data only. They do not perform
//! search, authorize actions, verify identity/evidence, or execute commands.

/// Shared presentation taxonomy for Finder results.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FinderKind {
    Person,
    Space,
    Conversation,
    Document,
    Project,
    Learning,
    App,
    Action,
}

impl FinderKind {
    pub fn label(self) -> &'static str {
        match self {
            Self::Person => "Person",
            Self::Space => "Space",
            Self::Conversation => "Conversation",
            Self::Document => "Document",
            Self::Project => "Project",
            Self::Learning => "Learning",
            Self::App => "App",
            Self::Action => "Action",
        }
    }
}

/// What selecting a Finder result is allowed to propose to the shell.
///
/// A proposed action remains a proposal. The Finder has no direct execution
/// variant: consequential actions must proceed through preview, user
/// authorization, current authority/capability checks, and domain execution.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum FinderIntent {
    Navigate {
        href: String,
    },
    ProposeAction {
        action_kind: String,
        action_ref: String,
    },
}

/// Presentation-safe result contributed by a domain/provider.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct FinderEntry {
    /// Stable identifier within the owning provider.
    pub id: String,
    /// Provider/domain identity. The shell treats this as provenance for the
    /// result, not as an authority or trust score.
    pub provider_id: String,
    pub kind: FinderKind,
    pub title: String,
    pub subtitle: Option<String>,
    /// Provider-approved terms that may participate in local matching.
    pub keywords: Vec<String>,
    pub intent: FinderIntent,
}

impl FinderEntry {
    pub fn navigation(
        provider_id: impl Into<String>,
        id: impl Into<String>,
        kind: FinderKind,
        title: impl Into<String>,
        href: impl Into<String>,
    ) -> Self {
        Self {
            id: id.into(),
            provider_id: provider_id.into(),
            kind,
            title: title.into(),
            subtitle: None,
            keywords: Vec::new(),
            intent: FinderIntent::Navigate { href: href.into() },
        }
    }

    pub fn proposed_action(
        provider_id: impl Into<String>,
        id: impl Into<String>,
        title: impl Into<String>,
        action_kind: impl Into<String>,
        action_ref: impl Into<String>,
    ) -> Self {
        Self {
            id: id.into(),
            provider_id: provider_id.into(),
            kind: FinderKind::Action,
            title: title.into(),
            subtitle: None,
            keywords: Vec::new(),
            intent: FinderIntent::ProposeAction {
                action_kind: action_kind.into(),
                action_ref: action_ref.into(),
            },
        }
    }
}

/// What the shell can currently establish about one Finder provider.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FinderProviderState {
    /// Provider completed the requested lookup. `entries` may legitimately be
    /// empty without implying anything about other providers.
    Ready,
    /// Provider lookup is in progress.
    Searching,
    /// Provider cannot presently be queried/read.
    Unavailable,
    /// Provider status cannot presently be established.
    Unknown,
}

impl FinderProviderState {
    pub fn label(self) -> &'static str {
        match self {
            Self::Ready => "Ready",
            Self::Searching => "Searching",
            Self::Unavailable => "Unavailable",
            Self::Unknown => "Unknown",
        }
    }
}

/// One provider's result batch plus explicit provider state.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct FinderBatch {
    pub provider_id: String,
    pub state: FinderProviderState,
    pub entries: Vec<FinderEntry>,
}

impl FinderBatch {
    pub fn ready(provider_id: impl Into<String>, entries: Vec<FinderEntry>) -> Self {
        Self {
            provider_id: provider_id.into(),
            state: FinderProviderState::Ready,
            entries,
        }
    }

    pub fn unavailable(provider_id: impl Into<String>) -> Self {
        Self {
            provider_id: provider_id.into(),
            state: FinderProviderState::Unavailable,
            entries: Vec::new(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{FinderBatch, FinderEntry, FinderIntent, FinderKind, FinderProviderState};

    #[test]
    fn empty_ready_provider_is_not_the_same_as_unavailable_provider() {
        let empty = FinderBatch::ready("knowledge", vec![]);
        let unavailable = FinderBatch::unavailable("knowledge");

        assert_eq!(empty.state, FinderProviderState::Ready);
        assert_eq!(unavailable.state, FinderProviderState::Unavailable);
        assert_ne!(empty.state, unavailable.state);
    }

    #[test]
    fn action_results_are_proposals_not_execution_callbacks() {
        let entry = FinderEntry::proposed_action(
            "governance",
            "proposal-42",
            "Vote on proposal 42",
            "governance.vote",
            "proposal-42",
        );

        assert_eq!(entry.kind, FinderKind::Action);
        assert!(matches!(entry.intent, FinderIntent::ProposeAction { .. }));
    }

    #[test]
    fn navigation_result_preserves_provider_provenance() {
        let entry = FinderEntry::navigation(
            "knowledge",
            "doc-7",
            FinderKind::Document,
            "Local-first notes",
            "/documents/doc-7",
        );

        assert_eq!(entry.provider_id, "knowledge");
        assert_eq!(entry.kind.label(), "Document");
    }
}
