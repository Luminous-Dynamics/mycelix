// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Hearth presentation settings.
//!
//! Settings on this page are browser-local convenience state. They are not
//! Hearth records and cannot change membership, roles, governance, or any other
//! source-backed authority.

use crate::hearth_prefs::{MAX_MOTTO_CHARS, use_hearth_prefs};
use leptos::prelude::*;

#[component]
pub fn SettingsPage() -> impl IntoView {
    let prefs = use_hearth_prefs();
    let prefs_for_value = prefs.clone();
    let prefs_for_input = prefs.clone();
    let prefs_for_disabled = prefs.clone();
    let prefs_for_clear = prefs.clone();
    let motto_for_count = prefs.motto;

    view! {
        <div class="page settings-page">
            <h1 class="page-title">"Settings"</h1>
            <p class="page-subtitle">
                "Personalize how Hearth feels in this browser. These preferences are local presentation state, not shared Hearth authority."
            </p>

            <section aria-labelledby="hearth-motto-heading">
                <h2 id="hearth-motto-heading">"home motto"</h2>
                <p class="field-help">
                    "An optional line shown beneath the Hearth name on Home. It is stored only in this browser and does not change the source-backed Hearth record."
                </p>

                <label for="hearth-motto">"motto"</label>
                <input
                    id="hearth-motto"
                    name="hearth-motto"
                    type="text"
                    maxlength=MAX_MOTTO_CHARS.to_string()
                    prop:value=move || prefs_for_value.motto.get()
                    on:input=move |event| {
                        prefs_for_input.set_motto(event_target_value(&event));
                    }
                />
                <div class="field-meta" aria-live="polite">
                    {move || format!(
                        "{} / {} characters",
                        motto_for_count.get().chars().count(),
                        MAX_MOTTO_CHARS,
                    )}
                </div>

                <button
                    type="button"
                    class="secondary-action"
                    disabled=move || prefs_for_disabled.motto.get().is_empty()
                    on:click=move |_| prefs_for_clear.clear_motto()
                >
                    "Clear motto"
                </button>
            </section>

            <section aria-labelledby="settings-boundary-heading">
                <h2 id="settings-boundary-heading">"what these settings can do"</h2>
                <p>
                    "Browser preferences may change presentation only. Membership, roles, Care, Decisions, bonds, and other Hearth state remain determined by their source-backed records and zome rules."
                </p>
            </section>
        </div>
    }
}
