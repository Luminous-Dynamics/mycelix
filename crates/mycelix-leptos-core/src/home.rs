// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Typed provider/item contract for the task-first Mycelix Home surface.
//!
//! Home is a presentation projection over provider-owned state. These types do
//! not authorize actions, infer priority, mark work complete, synchronize data,
//! or execute consequential operations.

/// Stable Home sections from the v1 experience contract.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HomeSection {
    Continue,
    NeedsAttention,
    LocalWork,
    Pinned,
}

impl HomeSection {
    pub fn label(self) -> &'static str {
        match self {
            Self::Continue => "Continue",
            Self::NeedsAttention => "Needs attention",
            Self::LocalWork => "Local work",
            Self::Pinned => "Pinned",
        }
    }
}

/// Safe handoff produced by selecting a Home item.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum HomeTarget {
    Navigate { href: String },
    ResumeDraft {
        draft_kind: String,
        draft_ref: String,
    },
}

/// One provider-owned item projected onto Home.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct HomeItem {
    pub id: String,
    /// Provenance for the item, not a trust/authority score.
    pub provider_id: String,
    pub section: HomeSection,
    pub title: String,
    pub detail: Option<String>,
    /// Provider-owned human-readable lifecycle/state copy. The shell may show
    /// it but must not reinterpret it into a stronger shared lifecycle.
    pub state_label: Option<String>,
    /// Optional provider-supplied timestamp for presentation recency only.
    /// It must not be interpreted as authority, correctness, or urgency.
    pub updated_at_unix_ms: Option<u64>,
    pub target: HomeTarget,
}

impl HomeItem {
    pub fn navigation(
        provider_id: impl Into<String>,
        id: impl Into<String>,
        section: HomeSection,
        title: impl Into<String>,
        href: impl Into<String>,
    ) -> Self {
        Self {
            id: id.into(),
            provider_id: provider_id.into(),
            section,
            title: title.into(),
            detail: None,
            state_label: None,
            updated_at_unix_ms: None,
            target: HomeTarget::Navigate { href: href.into() },
        }
    }

    pub fn resumable_draft(
        provider_id: impl Into<String>,
        id: impl Into<String>,
        title: impl Into<String>,
        draft_kind: impl Into<String>,
        draft_ref: impl Into<String>,
    ) -> Self {
        Self {
            id: id.into(),
            provider_id: provider_id.into(),
            section: HomeSection::Continue,
            title: title.into(),
            detail: None,
            state_label: None,
            updated_at_unix_ms: None,
            target: HomeTarget::ResumeDraft {
                draft_kind: draft_kind.into(),
                draft_ref: draft_ref.into(),
            },
        }
    }
}

/// What Home can currently establish about one provider contribution.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HomeProviderState {
    Ready,
    Unavailable,
    Unknown,
}

impl HomeProviderState {
    pub fn label(self) -> &'static str {
        match self {
            Self::Ready => "Ready",
            Self::Unavailable => "Unavailable",
            Self::Unknown => "Unknown",
        }
    }
}

/// One provider's Home projection plus explicit provider state.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct HomeBatch {
    pub provider_id: String,
    pub state: HomeProviderState,
    pub items: Vec<HomeItem>,
}

impl HomeBatch {
    pub fn ready(provider_id: impl Into<String>, items: Vec<HomeItem>) -> Self {
        Self {
            provider_id: provider_id.into(),
            state: HomeProviderState::Ready,
            items,
        }
    }

    pub fn unavailable(provider_id: impl Into<String>) -> Self {
        Self {
            provider_id: provider_id.into(),
            state: HomeProviderState::Unavailable,
            items: Vec::new(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{HomeBatch, HomeItem, HomeProviderState, HomeSection, HomeTarget};

    #[test]
    fn ready_empty_is_distinct_from_unavailable() {
        let empty = HomeBatch::ready("hearth", vec![]);
        let unavailable = HomeBatch::unavailable("hearth");

        assert_eq!(empty.state, HomeProviderState::Ready);
        assert_eq!(unavailable.state, HomeProviderState::Unavailable);
        assert_ne!(empty.state, unavailable.state);
    }

    #[test]
    fn home_targets_only_navigate_or_resume() {
        let item = HomeItem::navigation(
            "governance",
            "proposal-42",
            HomeSection::NeedsAttention,
            "Review proposal 42",
            "/proposals/42",
        );

        assert!(matches!(item.target, HomeTarget::Navigate { .. }));
    }

    #[test]
    fn resumable_draft_is_not_marked_complete_or_executed() {
        let item = HomeItem::resumable_draft(
            "knowledge",
            "draft-7",
            "Continue local-first notes",
            "knowledge.document",
            "draft-7",
        );

        assert_eq!(item.section, HomeSection::Continue);
        assert!(matches!(item.target, HomeTarget::ResumeDraft { .. }));
        assert!(item.state_label.is_none());
    }

    #[test]
    fn shared_model_has_no_cross_domain_priority_number() {
        let item = HomeItem::navigation(
            "hearth",
            "care-1",
            HomeSection::NeedsAttention,
            "Review care request",
            "/care/1",
        );

        assert_eq!(item.provider_id, "hearth");
        assert_eq!(item.section.label(), "Needs attention");
    }
}
