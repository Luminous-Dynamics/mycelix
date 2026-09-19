// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Shared search input primitive.

use leptos::prelude::*;

/// Search input with a required accessible name and reactive callback.
#[component]
pub fn SearchBar(
    query: ReadSignal<String>,
    on_change: Callback<String>,
    #[prop(optional, default = "Search...")] placeholder: &'static str,
    #[prop(optional, default = "Search")] aria_label: &'static str,
) -> impl IntoView {
    view! {
        <div class="search-bar">
            <input
                class="form-input search-input"
                type="search"
                placeholder=placeholder
                aria-label=aria_label
                prop:value=move || query.get()
                on:input=move |ev| {
                    on_change.run(event_target_value(&ev));
                }
            />
        </div>
    }
}
