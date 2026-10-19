// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Trust tier badge component.
//!
//! Displays a user's consciousness-gated trust tier. This component presents
//! an application-level tier; it does not claim cryptographic verification,
//! identity proof, capability authority, or evidence validity.

use leptos::prelude::*;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct TierPresentation {
    css_class: &'static str,
    label: &'static str,
}

fn tier_presentation(tier: &str) -> TierPresentation {
    match tier.trim().to_ascii_lowercase().as_str() {
        "observer" => TierPresentation {
            css_class: "tier-observer",
            label: "Observer",
        },
        // The shared stylesheet retains the historical basic/standard/elevated
        // class names while the product language uses Participant/Citizen/
        // Steward. Keep that compatibility mapping explicit here rather than
        // duplicating colors in Rust.
        "participant" => TierPresentation {
            css_class: "tier-basic",
            label: "Participant",
        },
        "citizen" => TierPresentation {
            css_class: "tier-standard",
            label: "Citizen",
        },
        "steward" => TierPresentation {
            css_class: "tier-elevated",
            label: "Steward",
        },
        "guardian" => TierPresentation {
            css_class: "tier-guardian",
            label: "Guardian",
        },
        _ => TierPresentation {
            css_class: "trust-tier-unknown",
            label: "Unknown",
        },
    }
}

/// Displays a trust tier using the shared Mycelix design-system tier classes.
///
/// Unknown input remains explicitly unknown rather than being visually mapped
/// to the lowest known tier. That distinction matters: an unrecognized or
/// unavailable tier value is not evidence that the subject is an Observer.
///
/// # Props
///
/// * `tier` — Trust tier name (e.g. "Observer", "guardian", "Citizen").
///
/// # Example
///
/// ```rust,no_run
/// use mycelix_leptos_core::TrustBadge;
/// // view! { <TrustBadge tier="Guardian".to_string() /> }
/// ```
#[component]
pub fn TrustBadge(tier: String) -> impl IntoView {
    let presentation = tier_presentation(&tier);
    let aria_label = format!("Trust tier: {}", presentation.label);
    let class = format!(
        "tier-badge trust-badge {}",
        presentation.css_class
    );
    let unknown_style = if presentation.label == "Unknown" {
        "background: var(--bg-surface, transparent); color: var(--text-muted, currentColor); border: 1px solid var(--border-visible, currentColor);"
    } else {
        ""
    };

    view! {
        <span class=class style=unknown_style aria-label=aria_label>
            {presentation.label}
        </span>
    }
}

#[cfg(test)]
mod tests {
    use super::tier_presentation;

    #[test]
    fn product_tier_names_map_to_existing_shared_theme_classes() {
        assert_eq!(tier_presentation("observer").css_class, "tier-observer");
        assert_eq!(tier_presentation("Participant").css_class, "tier-basic");
        assert_eq!(tier_presentation("CITIZEN").css_class, "tier-standard");
        assert_eq!(tier_presentation(" steward ").css_class, "tier-elevated");
        assert_eq!(tier_presentation("guardian").css_class, "tier-guardian");
    }

    #[test]
    fn unknown_tier_is_not_downgraded_to_observer() {
        let unknown = tier_presentation("not-a-tier");
        assert_eq!(unknown.label, "Unknown");
        assert_eq!(unknown.css_class, "trust-tier-unknown");
        assert_ne!(unknown.css_class, "tier-observer");
    }
}
