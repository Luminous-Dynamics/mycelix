// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Route-level availability gates for Hearth domain pages.
//!
//! A domain page may render its own empty state only when the source was
//! actually established. `Unknown` and `Unavailable` hide the domain page so
//! legacy "nothing here" copy cannot strengthen an unperformed/failed query
//! into an authoritative empty result.

use crate::hearth_truth::{HearthAvailability, use_hearth_truth};
use leptos::prelude::*;
use mycelix_leptos_core::AvailabilityStateKind;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HearthDataDomain {
    Home,
    Kinship,
    Care,
    Decisions,
    Gratitude,
    Stories,
    Milestones,
    Rhythms,
    Emergency,
    Resources,
    Autonomy,
}

pub fn domain_state(
    availability: &HearthAvailability,
    domain: HearthDataDomain,
) -> AvailabilityStateKind {
    match domain {
        HearthDataDomain::Home => combine_states(&[
            availability.current_hearth,
            availability.members,
            availability.bonds,
            availability.gratitude,
            availability.presence,
            availability.care_schedules,
            availability.decisions,
        ]),
        HearthDataDomain::Kinship => {
            combine_states(&[availability.members, availability.bonds])
        }
        HearthDataDomain::Care => {
            combine_states(&[availability.members, availability.care_schedules])
        }
        HearthDataDomain::Decisions => combine_states(&[
            availability.members,
            availability.decisions,
            availability.votes,
        ]),
        HearthDataDomain::Gratitude => {
            combine_states(&[availability.members, availability.gratitude])
        }
        HearthDataDomain::Stories => {
            combine_states(&[availability.members, availability.stories])
        }
        HearthDataDomain::Milestones => {
            combine_states(&[availability.members, availability.milestones])
        }
        HearthDataDomain::Rhythms => combine_states(&[
            availability.members,
            availability.rhythms,
            availability.presence,
        ]),
        HearthDataDomain::Emergency => {
            combine_states(&[availability.members, availability.emergency_alerts])
        }
        HearthDataDomain::Resources => combine_states(&[availability.resources]),
        HearthDataDomain::Autonomy => {
            combine_states(&[availability.members, availability.autonomy_profiles])
        }
    }
}

pub fn can_render_domain(kind: AvailabilityStateKind) -> bool {
    matches!(
        kind,
        AvailabilityStateKind::Live
            | AvailabilityStateKind::Mock
            | AvailabilityStateKind::Empty
            | AvailabilityStateKind::Degraded
    )
}

fn combine_states(states: &[AvailabilityStateKind]) -> AvailabilityStateKind {
    if states.contains(&AvailabilityStateKind::Unavailable) {
        return AvailabilityStateKind::Unavailable;
    }
    if states.contains(&AvailabilityStateKind::Unknown) {
        return AvailabilityStateKind::Unknown;
    }
    if states.contains(&AvailabilityStateKind::Locked) {
        return AvailabilityStateKind::Locked;
    }
    if states.contains(&AvailabilityStateKind::Degraded) {
        return AvailabilityStateKind::Degraded;
    }
    if states.contains(&AvailabilityStateKind::Mock) {
        return AvailabilityStateKind::Mock;
    }
    if states.iter().all(|state| *state == AvailabilityStateKind::Empty) {
        AvailabilityStateKind::Empty
    } else {
        AvailabilityStateKind::Live
    }
}

#[component]
pub fn HearthDataBoundary(
    domain: HearthDataDomain,
    #[prop(default = "Hearth area")] title: &'static str,
    children: Children,
) -> impl IntoView {
    let truth = use_hearth_truth();
    let child_view = children();

    let truth_for_content = truth.clone();
    let content_style = move || {
        let kind = domain_state(&truth_for_content.availability.get(), domain);
        if can_render_domain(kind) {
            "display: block"
        } else {
            "display: none"
        }
    };

    let truth_for_blocked = truth.clone();
    let blocked_style = move || {
        let kind = domain_state(&truth_for_blocked.availability.get(), domain);
        if can_render_domain(kind) {
            "display: none"
        } else {
            "display: block"
        }
    };

    let truth_for_kind = truth.clone();
    let truth_for_description = truth.clone();
    let truth_for_degraded = truth.clone();

    view! {
        <div class="hearth-domain-content" style=content_style>
            {child_view}
        </div>

        <div class="availability-state hearth-domain-unavailable" style=blocked_style role="status">
            <div class="availability-state-copy">
                <div class="availability-state-meta">
                    <span class="availability-state-title">{title}</span>
                    <span class="status-pill availability-unavailable">
                        {move || domain_state(&truth_for_kind.availability.get(), domain).label()}
                    </span>
                </div>
                <p class="availability-state-description">
                    {move || {
                        let kind = domain_state(&truth_for_description.availability.get(), domain);
                        match kind {
                            AvailabilityStateKind::Unknown => {
                                "This data source has not been established yet. No empty-result claim is being made."
                            }
                            AvailabilityStateKind::Unavailable => {
                                "This Hearth area is not live-backed in the current frontend. Demo records are hidden in live mode."
                            }
                            AvailabilityStateKind::Locked => {
                                "This Hearth area is currently locked."
                            }
                            _ => "This Hearth area is not currently available.",
                        }
                    }}
                </p>
            </div>
        </div>

        <div
            class="hearth-domain-degraded"
            role="status"
            style=move || {
                if domain_state(&truth_for_degraded.availability.get(), domain)
                    == AvailabilityStateKind::Degraded
                {
                    "display: block"
                } else {
                    "display: none"
                }
            }
        >
            "Some records could not be established from the live source. The visible data is partial and must not be treated as complete."
        </div>
    }
}

#[cfg(test)]
mod tests {
    use super::{HearthDataDomain, domain_state};
    use crate::hearth_truth::HearthAvailability;
    use mycelix_leptos_core::AvailabilityStateKind;

    #[test]
    fn care_is_unavailable_when_live_care_bridge_is_unwired() {
        let availability = HearthAvailability::live_pending();
        assert_eq!(
            domain_state(&availability, HearthDataDomain::Care),
            AvailabilityStateKind::Unavailable
        );
    }

    #[test]
    fn supported_kinship_can_be_live_with_empty_bonds() {
        let mut availability = HearthAvailability::live_pending();
        availability.members = AvailabilityStateKind::Live;
        availability.bonds = AvailabilityStateKind::Empty;
        assert_eq!(
            domain_state(&availability, HearthDataDomain::Kinship),
            AvailabilityStateKind::Live
        );
    }

    #[test]
    fn mock_mode_remains_explicitly_mock() {
        let availability = HearthAvailability::mock_mode();
        assert_eq!(
            domain_state(&availability, HearthDataDomain::Home),
            AvailabilityStateKind::Mock
        );
    }
}
