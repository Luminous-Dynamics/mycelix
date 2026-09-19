// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Typed registry contract for Mycelix's task-first Create surface.
//!
//! The shared shell may discover creation opportunities, but it does not own
//! domain validation, authority, submission, or execution. Selecting an entry
//! can only open a domain composer or produce a typed draft proposal.

/// What selecting a Create entry may request from the shell.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum CreateTarget {
    /// Navigate to a domain-owned composer/editor where current validation and
    /// authority checks remain authoritative.
    OpenComposer { href: String },
    /// Hand a typed draft reference to a domain-owned preview flow.
    /// This is still a draft/proposal, not execution.
    ProposeDraft {
        draft_kind: String,
        draft_ref: String,
    },
}

/// One human-facing creation opportunity contributed by a domain/provider.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CreateEntry {
    pub id: String,
    /// Provenance for the entry, not a trust or authority score.
    pub provider_id: String,
    pub label: String,
    pub description: Option<String>,
    pub keywords: Vec<String>,
    pub target: CreateTarget,
}

impl CreateEntry {
    pub fn composer(
        provider_id: impl Into<String>,
        id: impl Into<String>,
        label: impl Into<String>,
        href: impl Into<String>,
    ) -> Self {
        Self {
            id: id.into(),
            provider_id: provider_id.into(),
            label: label.into(),
            description: None,
            keywords: Vec::new(),
            target: CreateTarget::OpenComposer { href: href.into() },
        }
    }

    pub fn proposed_draft(
        provider_id: impl Into<String>,
        id: impl Into<String>,
        label: impl Into<String>,
        draft_kind: impl Into<String>,
        draft_ref: impl Into<String>,
    ) -> Self {
        Self {
            id: id.into(),
            provider_id: provider_id.into(),
            label: label.into(),
            description: None,
            keywords: Vec::new(),
            target: CreateTarget::ProposeDraft {
                draft_kind: draft_kind.into(),
                draft_ref: draft_ref.into(),
            },
        }
    }
}

/// What the shell can currently establish about a Create provider.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CreateProviderState {
    /// Provider successfully supplied its current creation opportunities.
    /// `entries` may legitimately be empty.
    Ready,
    /// Provider cannot presently supply its creation opportunities.
    Unavailable,
    /// Provider status cannot presently be established.
    Unknown,
}

impl CreateProviderState {
    pub fn label(self) -> &'static str {
        match self {
            Self::Ready => "Ready",
            Self::Unavailable => "Unavailable",
            Self::Unknown => "Unknown",
        }
    }
}

/// One provider's Create contribution plus explicit provider state.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CreateBatch {
    pub provider_id: String,
    pub state: CreateProviderState,
    pub entries: Vec<CreateEntry>,
}

impl CreateBatch {
    pub fn ready(provider_id: impl Into<String>, entries: Vec<CreateEntry>) -> Self {
        Self {
            provider_id: provider_id.into(),
            state: CreateProviderState::Ready,
            entries,
        }
    }

    pub fn unavailable(provider_id: impl Into<String>) -> Self {
        Self {
            provider_id: provider_id.into(),
            state: CreateProviderState::Unavailable,
            entries: Vec::new(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{CreateBatch, CreateEntry, CreateProviderState, CreateTarget};

    #[test]
    fn empty_ready_provider_is_not_the_same_as_unavailable_provider() {
        let empty = CreateBatch::ready("knowledge", vec![]);
        let unavailable = CreateBatch::unavailable("knowledge");

        assert_eq!(empty.state, CreateProviderState::Ready);
        assert_eq!(unavailable.state, CreateProviderState::Unavailable);
        assert_ne!(empty.state, unavailable.state);
    }

    #[test]
    fn composer_entries_only_open_domain_owned_composers() {
        let entry = CreateEntry::composer(
            "knowledge",
            "document",
            "Create document",
            "/documents/new",
        );

        assert!(matches!(entry.target, CreateTarget::OpenComposer { .. }));
        assert_eq!(entry.provider_id, "knowledge");
    }

    #[test]
    fn generated_drafts_remain_proposals_not_execution() {
        let entry = CreateEntry::proposed_draft(
            "governance",
            "proposal-draft",
            "Draft proposal",
            "governance.proposal",
            "draft-42",
        );

        assert!(matches!(entry.target, CreateTarget::ProposeDraft { .. }));
    }
}
