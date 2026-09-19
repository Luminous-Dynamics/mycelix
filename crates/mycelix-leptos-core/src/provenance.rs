// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Ordered provenance disclosure for consequential Mycelix UI surfaces.
//!
//! A provenance trail is explanatory structure, not a verifier. Each step
//! carries the evidence availability state supplied by its caller. Missing,
//! unavailable, and unknown evidence remain visible in their original position
//! instead of being filtered out to make the chain look more complete.

use leptos::prelude::*;

use crate::evidence::{EvidenceAvailability, EvidenceDisclosure};

/// One ordered step in a provenance trail.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ProvenanceStep {
    /// Stable key for reactive list rendering.
    pub id: String,
    /// Human-readable event or decision label.
    pub label: String,
    /// Optional explanatory copy. This must not be used to strengthen the
    /// evidence state encoded by `evidence`.
    pub detail: Option<String>,
    /// What the caller can establish about evidence for this step.
    pub evidence: EvidenceAvailability,
    /// Optional destination for inspecting the underlying record/reference.
    /// The shared UI labels this neutrally as "Inspect", not "View proof".
    pub inspect_href: Option<String>,
}

impl ProvenanceStep {
    /// Whether evidence availability for this step is not presently established
    /// as `Present`.
    ///
    /// This deliberately says nothing about freshness or validity. A present
    /// but stale record is not an availability gap.
    pub fn has_availability_gap(&self) -> bool {
        !matches!(self.evidence, EvidenceAvailability::Present { .. })
    }
}

fn availability_gap_count(steps: &[ProvenanceStep]) -> usize {
    steps
        .iter()
        .filter(|step| step.has_availability_gap())
        .count()
}

/// Ordered provenance trail.
///
/// The component preserves every supplied step, including availability gaps.
/// It does not infer missing steps, validate chain continuity, establish
/// authority, or perform cryptographic verification.
#[component]
pub fn ProvenanceTrail(steps: Vec<ProvenanceStep>) -> impl IntoView {
    let count = steps.len();
    let availability_gaps = availability_gap_count(&steps);
    let aria_label = if availability_gaps == 0 {
        format!("Provenance trail, {count} steps")
    } else {
        format!(
            "Provenance trail, {count} steps, {availability_gaps} evidence availability gaps"
        )
    };
    let steps_for_list = steps.clone();

    view! {
        <section class="provenance-trail" aria-label=aria_label>
            {if steps.is_empty() {
                Some(view! {
                    <p
                        class="provenance-trail-empty"
                        style="margin: 0; color: var(--text-muted, currentColor); font-size: var(--text-sm, 0.8125rem);"
                    >
                        "No provenance steps are available."
                    </p>
                })
            } else {
                None
            }}

            <ol
                class="provenance-trail-list"
                style="display: flex; flex-direction: column; gap: var(--space-md, 0.75rem); margin: 0; padding-left: var(--space-xl, 1.5rem);"
            >
                <For
                    each=move || steps_for_list.clone()
                    key=|step| step.id.clone()
                    children=move |step| view! { <ProvenanceStepView step=step /> }
                />
            </ol>
        </section>
    }
}

#[component]
fn ProvenanceStepView(step: ProvenanceStep) -> impl IntoView {
    let ProvenanceStep {
        id,
        label,
        detail,
        evidence,
        inspect_href,
    } = step;

    let inspect_context = label.clone();
    let action = inspect_href.map(|href| {
        let aria_label = format!("Inspect evidence for {inspect_context}");
        view! {
            <a
                class="provenance-inspect-link"
                href=href
                aria-label=aria_label
                style="display: inline-flex; margin-top: var(--space-xs, 0.25rem); font-size: var(--text-sm, 0.8125rem);"
            >
                "Inspect"
            </a>
        }
        .into_any()
    });

    let disclosure = match detail {
        Some(detail) => view! {
            <EvidenceDisclosure
                availability=evidence
                title=label
                detail=detail
                action=action
            />
        }
        .into_any(),
        None => view! {
            <EvidenceDisclosure
                availability=evidence
                title=label
                action=action
            />
        }
        .into_any(),
    };

    view! {
        <li class="provenance-step" data-provenance-step=id>
            {disclosure}
        </li>
    }
}

#[cfg(test)]
mod tests {
    use super::{ProvenanceStep, availability_gap_count};
    use crate::evidence::EvidenceAvailability;
    use crate::freshness::FreshnessLevel;

    fn step(id: &str, evidence: EvidenceAvailability) -> ProvenanceStep {
        ProvenanceStep {
            id: id.into(),
            label: id.into(),
            detail: None,
            evidence,
            inspect_href: None,
        }
    }

    #[test]
    fn missing_unavailable_and_unknown_steps_count_as_availability_gaps() {
        let steps = vec![
            step("proposal", EvidenceAvailability::Present { freshness: None }),
            step("decision", EvidenceAvailability::Missing),
            step("timelock", EvidenceAvailability::Unavailable),
            step("execution", EvidenceAvailability::Unknown),
        ];

        assert_eq!(steps.len(), 4);
        assert_eq!(availability_gap_count(&steps), 3);
        assert_eq!(steps[1].evidence.label(), "Missing");
        assert_eq!(steps[2].evidence.label(), "Unavailable");
        assert_eq!(steps[3].evidence.label(), "Unknown");
    }

    #[test]
    fn present_but_stale_evidence_is_not_an_availability_gap() {
        let stale = step(
            "decision",
            EvidenceAvailability::Present {
                freshness: Some(FreshnessLevel::Stale),
            },
        );

        assert!(!stale.has_availability_gap());
        assert_eq!(stale.evidence.freshness(), Some(FreshnessLevel::Stale));
    }

    #[test]
    fn all_present_means_available_not_verified() {
        let steps = vec![
            step("proposal", EvidenceAvailability::Present { freshness: None }),
            step("decision", EvidenceAvailability::Present { freshness: None }),
        ];

        assert_eq!(availability_gap_count(&steps), 0);
        assert!(steps.iter().all(|step| step.evidence.label() == "Present"));
    }
}
