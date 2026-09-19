// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! First task-first Hearth landing surfaces.
//!
//! These pages improve orientation without inventing cross-domain authority or
//! lifecycle state. Domain pages remain authoritative for their own actions.

use crate::pending_votes::UnvotedOpenDecisionsInbox;
use leptos::prelude::*;
use leptos_router::components::A;

const FIND_DESTINATIONS: [(&str, &str, &str); 10] = [
    ("/kinship", "Bonds", "People and relationships in this Hearth"),
    ("/care", "Care", "Care schedules and shared responsibilities"),
    ("/gratitude", "Gratitude", "Expressions of appreciation"),
    ("/stories", "Stories", "Family memories, traditions, recipes, and wisdom"),
    ("/milestones", "Milestones", "Shared moments worth remembering"),
    ("/rhythms", "Rhythms", "Presence and household rhythms"),
    ("/decisions", "Decisions", "Open and historical household decisions"),
    ("/resources", "Resources", "Shared household resources"),
    ("/autonomy", "Autonomy", "Personal and household autonomy controls"),
    ("/personal/profile", "Me", "Your personal Hearth profile"),
];

#[component]
pub fn FindPage() -> impl IntoView {
    view! {
        <div class="page task-surface-page find-page">
            <h1 class="page-title">"find"</h1>
            <p class="page-subtitle">
                "Find people, shared work, memories, and household areas without needing to know Hearth’s internal structure."
            </p>

            <div class="task-directory" role="list" aria-label="Hearth destinations">
                {FIND_DESTINATIONS
                    .into_iter()
                    .map(|(href, label, description)| {
                        view! {
                            <A href=href attr:class="task-directory-item" attr:role="listitem">
                                <strong>{label}</strong>
                                <span>{description}</span>
                            </A>
                        }
                    })
                    .collect_view()}
            </div>

            <p class="task-surface-note">
                "Cross-domain text search is not claimed here yet. This first step is a deterministic local directory; the shared Finder will replace it after its provider integration is qualified."
            </p>
        </div>
    }
}

#[component]
pub fn CreatePage() -> impl IntoView {
    view! {
        <div class="page task-surface-page create-page">
            <h1 class="page-title">"create"</h1>
            <p class="page-subtitle">"Start something meaningful, then finish it in the Hearth area that owns it."</p>

            <section aria-labelledby="create-available-heading">
                <h2 id="create-available-heading">"Live-capable now"</h2>
                <div class="task-directory" role="list">
                    <A href="/gratitude" attr:class="task-directory-item" attr:role="listitem">
                        <strong>"Express gratitude"</strong>
                        <span>"Open the Gratitude area. The route boundary will verify whether its live source is available before showing the composer."</span>
                    </A>
                </div>
            </section>

            <section aria-labelledby="create-not-ready-heading">
                <h2 id="create-not-ready-heading">"Demo-only or not live-wired yet"</h2>
                <div class="task-directory" role="list">
                    <div class="task-directory-item task-directory-unavailable" role="listitem">
                        <strong>"Share a story"</strong>
                        <span>"Story composition currently mutates only the frontend signal; live story loading/submission is not established yet."</span>
                    </div>
                    <div class="task-directory-item task-directory-unavailable" role="listitem">
                        <strong>"New care task"</strong>
                        <span>"The care zome has a creation API, but the frontend composer/action bridge is not wired yet."</span>
                    </div>
                    <div class="task-directory-item task-directory-unavailable" role="listitem">
                        <strong>"New decision"</strong>
                        <span>"The decisions zome has a creation API, but the frontend composer/action bridge is not wired yet."</span>
                    </div>
                </div>
            </section>

            <p class="task-surface-note">
                "Create only hands you into domain-owned workflows. This page does not grant permission, submit records, or execute actions."
            </p>
        </div>
    }
}

#[component]
pub fn InboxPage() -> impl IntoView {
    view! {
        <div class="page task-surface-page inbox-page">
            <h1 class="page-title">"inbox"</h1>
            <p class="page-subtitle">"Things that arrive for you to inspect, revisit, or respond to."</p>

            <UnvotedOpenDecisionsInbox />

            <section aria-labelledby="inbox-other-heading">
                <h2 id="inbox-other-heading">"Other attention areas"</h2>
                <div class="task-directory" role="list" aria-label="Current attention areas">
                    <A href="/care" attr:class="task-directory-item" attr:role="listitem">
                        <strong>"Care"</strong>
                        <span>"Inspect Care directly; its route will disclose if live care data is unavailable."</span>
                    </A>
                    <A href="/decisions" attr:class="task-directory-item" attr:role="listitem">
                        <strong>"Decisions"</strong>
                        <span>"Inspect the full Decision lifecycle, recorded outcomes, and vote-history provenance."</span>
                    </A>
                </div>
            </section>

            <p class="task-surface-note">
                "Opening Inbox does not mark any provider record seen, acknowledged, accepted, voted, or resolved."
            </p>
        </div>
    }
}

#[cfg(test)]
mod tests {
    use super::FIND_DESTINATIONS;

    #[test]
    fn local_find_directory_contains_only_explicit_routes() {
        assert!(!FIND_DESTINATIONS.is_empty());
        assert!(
            FIND_DESTINATIONS
                .iter()
                .all(|(href, _, _)| href.starts_with('/'))
        );
        assert!(
            FIND_DESTINATIONS
                .iter()
                .any(|(href, _, _)| *href == "/care")
        );
        assert!(
            FIND_DESTINATIONS
                .iter()
                .any(|(href, _, _)| *href == "/personal/profile")
        );
    }
}
