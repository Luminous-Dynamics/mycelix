// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Generic availability-state rendering helpers for Mycelix frontends.
//!
//! Availability is deliberately not collapsed into a binary online/offline
//! signal. In particular, `Unknown` means the application cannot currently
//! establish the state; it must not be presented as `Empty`, `Unavailable`, or
//! `Live` merely to produce a more definite-looking interface.

use leptos::prelude::*;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AvailabilityStateKind {
    Live,
    Mock,
    Empty,
    Locked,
    Degraded,
    Unknown,
    Unavailable,
}

impl AvailabilityStateKind {
    pub fn label(self) -> &'static str {
        match self {
            Self::Live => "Live",
            Self::Mock => "Mock",
            Self::Empty => "Empty",
            Self::Locked => "Locked",
            Self::Degraded => "Degraded",
            Self::Unknown => "Unknown",
            Self::Unavailable => "Unavailable",
        }
    }

    pub fn css_class(self) -> &'static str {
        match self {
            Self::Live => "availability-live",
            Self::Mock => "availability-mock",
            Self::Empty => "availability-empty",
            Self::Locked => "availability-locked",
            Self::Degraded => "availability-degraded",
            Self::Unknown => "availability-unknown",
            Self::Unavailable => "availability-unavailable",
        }
    }

    pub fn icon(self) -> &'static str {
        match self {
            Self::Live => "●",
            Self::Mock => "◌",
            Self::Empty => "○",
            Self::Locked => "◈",
            Self::Degraded => "△",
            Self::Unknown => "?",
            Self::Unavailable => "×",
        }
    }
}

#[component]
pub fn AvailabilityState(
    kind: AvailabilityStateKind,
    #[prop(into)] title: String,
    #[prop(into)] description: String,
    action: Option<AnyView>,
) -> impl IntoView {
    let availability_label = format!("Availability: {}", kind.label());

    view! {
        <div class=format!("availability-state {}", kind.css_class())>
            <div class="availability-state-header">
                <span class="availability-state-icon" aria-hidden="true">{kind.icon()}</span>
                <div class="availability-state-copy">
                    <div class="availability-state-meta">
                        <span class="availability-state-title">{title}</span>
                        <span
                            class=format!("status-pill {}", kind.css_class())
                            aria-label=availability_label
                        >
                            {kind.label()}
                        </span>
                    </div>
                    <p class="availability-state-description">{description}</p>
                </div>
            </div>
            {action}
        </div>
    }
}

#[cfg(test)]
mod tests {
    use super::AvailabilityStateKind;

    #[test]
    fn unknown_is_distinct_from_empty_and_unavailable() {
        assert_ne!(AvailabilityStateKind::Unknown, AvailabilityStateKind::Empty);
        assert_ne!(AvailabilityStateKind::Unknown, AvailabilityStateKind::Unavailable);
        assert_eq!(AvailabilityStateKind::Unknown.label(), "Unknown");
        assert_eq!(AvailabilityStateKind::Unknown.css_class(), "availability-unknown");
    }
}
