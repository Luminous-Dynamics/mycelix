// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! First task-first Hearth landing surfaces.
//!
//! These pages improve orientation without inventing cross-domain authority or
//! lifecycle state. Domain pages remain authoritative for their own actions.

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
                {FIND_DESTINATIONS.into_iter().map(|(href, label, description)| view! {
                    <A href=href attr:class="task-directory-item" attr:role="listitem">
                        <strong>{label}</strong>
                        <span>{description}</span>
                    </A>
                }).collect_view()}
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
                <h2 id="create-available-heading">"Available now"</h2>
                <div class="task-directory" role="list">
                    <A href="/gratitude" attr:class="task-directory-item" attr:role="listitem">
                        <strong>"Express gratitude"</strong>
                        <span>"Open the Gratitude area and compose an appreciation."</span>
                    </A>
                    <A href="/stories" attr:class="task-directory-item" attr:role="listitem">
                        <strong>"Share a story"</strong>
                        <span>"Open Stories and add a memory, tradition, recipe, wisdom, or origin story."</span>
                    </A>
                </div>
            </section>

            <section aria-labelledby="create-not-ready-heading">
                <h2 id="create-not-ready-heading">"Not wired yet"</h2>
                <div class="task-directory" role="list">
                    <div class="task-directory-item task-directory-unavailable" role="listitem">
                        <strong>"New care task"</strong>
                        <span>"The Care page shows this affordance today, but its creation action is not connected yet."</span>
                    </div>
                    <div class="task-directory-item task-directory-unavailable" role="listitem">
                        <strong>"New decision"</strong>
                        <span>"The Decisions page shows this affordance today, but its creation action is not connected yet."</span>
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

            <div class="empty-state">
                <p>"Hearth does not yet expose unified Inbox envelopes."</p>
                <p>"For now, care work and household decisions remain in their own areas rather than being guessed into one notification stream."</p>
            </div>

            <div class="task-directory" role="list" aria-label="Current attention areas">
                <A href="/care" attr:class="task-directory-item" attr:role="listitem">
                    <strong>"Care"</strong>
                    <span>"Review active care schedules directly."</span>
                </A>
                <A href="/decisions" attr:class="task-directory-item" attr:role="listitem">
                    <strong>"Decisions"</strong>
                    <span>"Review open household decisions directly."</span>
                </A>
            </div>

            <p class="task-surface-note">
                "Opening Inbox does not mark any provider record seen, acknowledged, accepted, or resolved."
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
        assert!(FIND_DESTINATIONS.iter().all(|(href, _, _)| href.starts_with('/')));
        assert!(FIND_DESTINATIONS.iter().any(|(href, _, _)| *href == "/care"));
        assert!(FIND_DESTINATIONS.iter().any(|(href, _, _)| *href == "/personal/profile"));
    }
}
