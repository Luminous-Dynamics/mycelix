// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Local action history, scoped to the selected verified Hearth.
//!
//! This is recovery visibility, not a command surface: unknown actions cannot be
//! replayed here, and local rows do not prove remote success or peer propagation.

use leptos::prelude::*;

use crate::action_journal_store::{ActionKind, AttemptStatus, JournalEntryV1};
use crate::hearth_context::use_hearth;

const MAX_VISIBLE_ENTRIES: usize = 20;

fn action_label(action: ActionKind) -> &'static str {
    match action {
        ActionKind::BondUpdate => "Bond update",
        ActionKind::Gratitude => "Gratitude",
        ActionKind::Vote => "Vote",
        ActionKind::Presence => "Presence update",
        ActionKind::CareCompletion => "Care completion",
        ActionKind::Invitation => "Invitation",
    }
}

fn status_label(status: AttemptStatus) -> &'static str {
    match status {
        AttemptStatus::Prepared => "Prepared locally",
        AttemptStatus::Submitted => "Submitted · outcome not confirmed",
        AttemptStatus::SourceChainCommitted => "Saved response received",
        AttemptStatus::ExplicitlyRejected => "Explicitly rejected",
        AttemptStatus::OutcomeUnknown => "Outcome unknown",
        AttemptStatus::ReconciledApplied => "Reconciled · applied",
        AttemptStatus::ReconciledNotApplied => "Reconciled · not applied",
        AttemptStatus::NotSent => "Not sent",
    }
}

fn status_copy(status: AttemptStatus) -> &'static str {
    match status {
        AttemptStatus::Prepared => {
            "Prepared locally. The dispatch gate has not authorized a request to be sent."
        }
        AttemptStatus::Submitted => {
            "A request may have been sent. Its outcome is not confirmed; do not repeat it."
        }
        AttemptStatus::SourceChainCommitted => {
            "The conductor returned a successful response. Peer availability is a separate observation."
        }
        AttemptStatus::ExplicitlyRejected => {
            "A typed domain response established that this attempt was not applied."
        }
        AttemptStatus::OutcomeUnknown => {
            "The request may or may not have applied. Do not repeat it until reconciled."
        }
        AttemptStatus::ReconciledApplied => {
            "An authoritative, correctly scoped read established that the intended effect exists."
        }
        AttemptStatus::ReconciledNotApplied => {
            "An authoritative, correctly scoped read established that the intended effect does not exist. Any new attempt requires a new user decision."
        }
        AttemptStatus::NotSent => {
            "No request was sent for this attempt."
        }
    }
}

fn visible_entries(
    entries: &[JournalEntryV1],
    scope_fingerprint: Option<&str>,
) -> Vec<JournalEntryV1> {
    let Some(scope_fingerprint) = scope_fingerprint else {
        return Vec::new();
    };

    let mut visible = entries
        .iter()
        .filter(|entry| entry.hearth_scope_fingerprint() == scope_fingerprint)
        .cloned()
        .collect::<Vec<_>>();
    visible.sort_by(|a, b| {
        b.updated_at_unix_ms()
            .cmp(&a.updated_at_unix_ms())
            .then_with(|| b.operation_id().cmp(a.operation_id()))
    });
    visible.truncate(MAX_VISIBLE_ENTRIES);
    visible
}

fn iso_timestamp(unix_ms: u64) -> String {
    let date = js_sys::Date::new(&wasm_bindgen::JsValue::from_f64(unix_ms as f64));
    date.to_iso_string()
        .as_string()
        .unwrap_or_else(|| "timestamp unavailable".into())
}

fn short_operation_id(operation_id: &str) -> String {
    operation_id.chars().take(8).collect()
}

#[component]
pub fn ActionJournalHistory() -> impl IntoView {
    let hearth = use_hearth();
    let entries = hearth.action_journal_entries;
    let scope = hearth.action_journal_scope_fingerprint;

    view! {
        <Show
            when=move || scope.get().is_some()
            fallback=|| ()
        >
            <section class="action-journal-history" aria-labelledby="action-journal-history-title">
                <details>
                    <summary id="action-journal-history-title">
                        "Recent actions in this Hearth"
                    </summary>
                    <p class="action-journal-history__scope-note">
                        "This is local history from this browser tab. Reload to refresh changes made by other tabs. Storage can be cleared or evicted, and this history does not prove that other peers have the latest data. Unknown outcomes must not be repeated until reconciled."
                    </p>
                    {move || {
                        let selected_scope = scope.get();
                        let current = visible_entries(&entries.get(), selected_scope.as_deref());
                        if current.is_empty() {
                            view! {
                                <p class="action-journal-history__empty">
                                    "No locally recorded actions are available for this Hearth in this browser."
                                </p>
                            }.into_any()
                        } else {
                            current
                                .into_iter()
                                .map(|entry| {
                                    let action = action_label(entry.action());
                                    let status = status_label(entry.status());
                                    let copy = status_copy(entry.status());
                                    let operation = short_operation_id(entry.operation_id());
                                    let created = iso_timestamp(entry.created_at_unix_ms());
                                    let updated = iso_timestamp(entry.updated_at_unix_ms());
                                    let class = match entry.status() {
                                        AttemptStatus::OutcomeUnknown | AttemptStatus::Submitted => {
                                            "action-journal-entry action-journal-entry--unknown"
                                        }
                                        AttemptStatus::ExplicitlyRejected => {
                                            "action-journal-entry action-journal-entry--rejected"
                                        }
                                        AttemptStatus::NotSent => {
                                            "action-journal-entry action-journal-entry--not-sent"
                                        }
                                        _ => "action-journal-entry",
                                    };
                                    view! {
                                        <article class=class>
                                            <div class="action-journal-entry__heading">
                                                <strong>{action}</strong>
                                                <span class="action-journal-entry__status">{status}</span>
                                            </div>
                                            <p>{copy}</p>
                                            <small>{format!("Attempt {operation}… · Created {created} · Updated {updated}")}</small>
                                        </article>
                                    }
                                })
                                .collect_view()
                                .into_any()
                        }
                    }}
                </details>
            </section>
        </Show>
    }
}

#[cfg(test)]
mod tests {
    use super::{action_label, short_operation_id, status_copy, status_label, visible_entries};
    use crate::action_journal_store::{ActionKind, AttemptStatus, JournalEntryV1};

    const SCOPE_A: &str =
        "sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";
    const SCOPE_B: &str =
        "sha256:bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb";
    const TARGET: &str =
        "sha256:cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc";

    fn entry(operation_id: &str, scope: &str, created: u64) -> JournalEntryV1 {
        JournalEntryV1::new(
            operation_id,
            ActionKind::CareCompletion,
            scope,
            TARGET,
            created,
        ).unwrap()
    }

    #[test]
    fn history_is_hidden_when_scope_is_unknown_and_never_crosses_hearths() {
        let entries = vec![
            entry("aaaaaaaa-aaaa-4aaa-8aaa-aaaaaaaaaaaa", SCOPE_A, 100),
            entry("bbbbbbbb-bbbb-4bbb-8bbb-bbbbbbbbbbbb", SCOPE_B, 100),
        ];
        assert!(visible_entries(&entries, None).is_empty());
        let scoped = visible_entries(&entries, Some(SCOPE_A));
        assert_eq!(scoped.len(), 1);
        assert_eq!(scoped[0].hearth_scope_fingerprint(), SCOPE_A);
    }

    #[test]
    fn newest_updates_are_first_and_history_is_bounded() {
        let entries = (0..25)
            .map(|index| {
                entry(
                    &format!("c0ffee{index:02}-0000-4000-8000-000000000001"),
                    SCOPE_A,
                    100 + index,
                )
            })
            .collect::<Vec<_>>();
        let visible = visible_entries(&entries, Some(SCOPE_A));
        assert_eq!(visible.len(), 20);
        assert!(visible[0].updated_at_unix_ms() >= visible[19].updated_at_unix_ms());
    }

    #[test]
    fn status_copy_does_not_treat_unknown_as_failure_or_retry_permission() {
        let copy = status_copy(AttemptStatus::OutcomeUnknown).to_lowercase();
        assert!(copy.contains("may or may not"));
        assert!(copy.contains("do not repeat"));
        assert!(!status_label(AttemptStatus::OutcomeUnknown).contains("failed"));
        assert!(!status_copy(AttemptStatus::SourceChainCommitted).contains("every peer"));
    }

    #[test]
    fn labels_are_allowlisted_and_operation_id_is_shortened() {
        assert_eq!(action_label(ActionKind::Gratitude), "Gratitude");
        assert_eq!(action_label(ActionKind::Invitation), "Invitation");
        assert_eq!(
            short_operation_id("c0ffee00-0000-4000-8000-000000000001"),
            "c0ffee00"
        );
    }
}
