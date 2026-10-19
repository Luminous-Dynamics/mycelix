// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Freshness and staleness indicators for live-backed UI surfaces.
//!
//! Optional display detail may make a badge more useful to a person (for
//! example, "14 minutes old"), but it must never replace the underlying
//! freshness state for assistive technology. A stale value with custom detail
//! remains explicitly stale; an unknown value remains explicitly unknown.

use leptos::prelude::*;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FreshnessLevel {
    Fresh,
    Aging,
    Stale,
    Unknown,
}

impl FreshnessLevel {
    pub fn label(self) -> &'static str {
        match self {
            Self::Fresh => "Fresh",
            Self::Aging => "Aging",
            Self::Stale => "Stale",
            Self::Unknown => "Unknown",
        }
    }

    pub fn css_class(self) -> &'static str {
        match self {
            Self::Fresh => "freshness-fresh",
            Self::Aging => "freshness-aging",
            Self::Stale => "freshness-stale",
            Self::Unknown => "freshness-unknown",
        }
    }
}

fn accessibility_label(level: FreshnessLevel, detail: Option<&str>) -> String {
    match detail.map(str::trim).filter(|detail| !detail.is_empty()) {
        Some(detail) if detail != level.label() => {
            format!("Freshness: {}. {detail}", level.label())
        }
        _ => format!("Freshness: {}", level.label()),
    }
}

#[component]
pub fn FreshnessBadge(
    level: FreshnessLevel,
    #[prop(optional, into)] detail: Option<String>,
) -> impl IntoView {
    let aria_label = accessibility_label(level, detail.as_deref());
    let visible_label = detail.unwrap_or_else(|| level.label().into());

    view! {
        <span
            class=format!("badge freshness-badge {}", level.css_class())
            aria-label=aria_label
        >
            {visible_label}
        </span>
    }
}

#[cfg(test)]
mod tests {
    use super::{FreshnessLevel, accessibility_label};

    #[test]
    fn custom_detail_cannot_hide_stale_state() {
        assert_eq!(
            accessibility_label(FreshnessLevel::Stale, Some("14 minutes old")),
            "Freshness: Stale. 14 minutes old"
        );
    }

    #[test]
    fn unknown_remains_explicit_when_detail_is_absent() {
        assert_eq!(
            accessibility_label(FreshnessLevel::Unknown, None),
            "Freshness: Unknown"
        );
    }

    #[test]
    fn empty_or_duplicate_detail_does_not_duplicate_the_semantic_label() {
        assert_eq!(
            accessibility_label(FreshnessLevel::Fresh, Some("")),
            "Freshness: Fresh"
        );
        assert_eq!(
            accessibility_label(FreshnessLevel::Fresh, Some("Fresh")),
            "Freshness: Fresh"
        );
    }
}
