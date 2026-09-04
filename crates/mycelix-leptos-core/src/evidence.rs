// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Evidence availability disclosure for Mycelix frontends.
//!
//! This module intentionally models **availability**, not proof validity.
//! `Present` means an evidence reference or payload is available to the caller;
//! it does not mean the evidence is authentic, authorized, cryptographically
//! verified, semantically valid, or sufficient for any decision.
//!
//! Verification belongs in a separate typed layer that is populated from an
//! actual backend verification result. Keeping these concerns separate prevents
//! presentation from silently strengthening an evidence claim.

use leptos::prelude::*;

use crate::freshness::{FreshnessBadge, FreshnessLevel};

/// What the caller can currently establish about evidence availability.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EvidenceAvailability {
    /// Evidence exists and is available to the caller. No validity claim.
    Present {
        /// Optional freshness information about the available evidence.
        ///
        /// `None` means no freshness assertion is being made. Use
        /// `Some(FreshnessLevel::Unknown)` when freshness is relevant but the
        /// caller cannot establish it. Keeping those states distinct avoids
        /// manufacturing uncertainty where no freshness claim was requested.
        freshness: Option<FreshnessLevel>,
    },
    /// The caller has established that the expected evidence is absent.
    Missing,
    /// The evidence source cannot currently be read or reached.
    Unavailable,
    /// The caller cannot currently establish whether evidence exists.
    Unknown,
}

impl EvidenceAvailability {
    pub fn label(self) -> &'static str {
        match self {
            Self::Present { .. } => "Present",
            Self::Missing => "Missing",
            Self::Unavailable => "Unavailable",
            Self::Unknown => "Unknown",
        }
    }

    pub fn css_class(self) -> &'static str {
        match self {
            Self::Present { .. } => "evidence-present",
            Self::Missing => "evidence-missing",
            Self::Unavailable => "evidence-unavailable",
            Self::Unknown => "evidence-unknown",
        }
    }

    pub fn icon(self) -> &'static str {
        match self {
            Self::Present { .. } => "●",
            Self::Missing => "○",
            Self::Unavailable => "×",
            Self::Unknown => "?",
        }
    }

    pub fn freshness(self) -> Option<FreshnessLevel> {
        match self {
            Self::Present { freshness } => freshness,
            Self::Missing | Self::Unavailable | Self::Unknown => None,
        }
    }

    fn status_style(self) -> &'static str {
        match self {
            // Present is informational rather than success-colored: existence
            // alone is not a verification result.
            Self::Present { .. } => {
                "background: var(--bg-surface, transparent); color: var(--semantic-info, var(--text-secondary, currentColor)); border-color: var(--border-visible, currentColor);"
            }
            Self::Missing => {
                "background: var(--bg-surface, transparent); color: var(--semantic-warning, var(--text-secondary, currentColor)); border-color: var(--border-visible, currentColor);"
            }
            Self::Unavailable => {
                "background: var(--bg-surface, transparent); color: var(--semantic-warning, var(--text-secondary, currentColor)); border-color: var(--border-visible, currentColor);"
            }
            Self::Unknown => {
                "background: var(--bg-surface, transparent); color: var(--text-muted, currentColor); border-color: var(--border-visible, currentColor);"
            }
        }
    }
}

fn accessibility_label(availability: EvidenceAvailability) -> String {
    match availability.freshness() {
        Some(freshness) => format!(
            "Evidence: {}. Freshness: {}",
            availability.label(),
            freshness.label()
        ),
        None => format!("Evidence: {}", availability.label()),
    }
}

/// A compact evidence disclosure with optional detail and action.
///
/// This component only presents the state supplied by its caller. It performs
/// no retrieval, signature verification, authority check, provenance traversal,
/// or cryptographic validation itself.
#[component]
pub fn EvidenceDisclosure(
    availability: EvidenceAvailability,
    #[prop(into)] title: String,
    #[prop(optional, into)] detail: Option<String>,
    action: Option<AnyView>,
) -> impl IntoView {
    let evidence_label = accessibility_label(availability);
    let freshness = availability.freshness();

    view! {
        <div class=format!("evidence-disclosure {}", availability.css_class())>
            <div
                class="evidence-disclosure-meta"
                style="display: flex; align-items: center; gap: var(--space-sm, 0.5rem); flex-wrap: wrap;"
            >
                <span class="evidence-disclosure-icon" aria-hidden="true">
                    {availability.icon()}
                </span>
                <span
                    class="evidence-disclosure-title"
                    style="font-weight: 600; color: var(--text-primary, currentColor);"
                >
                    {title}
                </span>
                <span
                    class=format!("status-pill {}", availability.css_class())
                    style=availability.status_style()
                    aria-label=evidence_label
                >
                    {availability.label()}
                </span>
                {freshness.map(|level| view! { <FreshnessBadge level=level /> })}
            </div>

            {detail.map(|detail| view! {
                <p
                    class="evidence-disclosure-detail"
                    style="margin: var(--space-xs, 0.25rem) 0 0; color: var(--text-secondary, currentColor); font-size: var(--text-sm, 0.8125rem);"
                >
                    {detail}
                </p>
            })}

            {action}
        </div>
    }
}

#[cfg(test)]
mod tests {
    use super::{EvidenceAvailability, accessibility_label};
    use crate::freshness::FreshnessLevel;

    #[test]
    fn present_does_not_claim_verification() {
        let state = EvidenceAvailability::Present { freshness: None };
        assert_eq!(state.label(), "Present");
        assert_eq!(accessibility_label(state), "Evidence: Present");
        assert!(!accessibility_label(state).contains("Verified"));
        assert!(!accessibility_label(state).contains("Valid"));
    }

    #[test]
    fn missing_unavailable_and_unknown_are_distinct() {
        assert_ne!(EvidenceAvailability::Missing, EvidenceAvailability::Unavailable);
        assert_ne!(EvidenceAvailability::Missing, EvidenceAvailability::Unknown);
        assert_ne!(EvidenceAvailability::Unavailable, EvidenceAvailability::Unknown);
    }

    #[test]
    fn present_evidence_can_disclose_staleness_without_becoming_missing() {
        let state = EvidenceAvailability::Present {
            freshness: Some(FreshnessLevel::Stale),
        };
        assert_eq!(state.label(), "Present");
        assert_eq!(state.freshness(), Some(FreshnessLevel::Stale));
        assert_eq!(
            accessibility_label(state),
            "Evidence: Present. Freshness: Stale"
        );
    }

    #[test]
    fn no_freshness_assertion_differs_from_unknown_freshness() {
        let unasserted = EvidenceAvailability::Present { freshness: None };
        let unknown = EvidenceAvailability::Present {
            freshness: Some(FreshnessLevel::Unknown),
        };

        assert_eq!(accessibility_label(unasserted), "Evidence: Present");
        assert_eq!(
            accessibility_label(unknown),
            "Evidence: Present. Freshness: Unknown"
        );
        assert_ne!(unasserted, unknown);
    }
}
