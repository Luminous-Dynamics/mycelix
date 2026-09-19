// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Presentation for committed Personal mutations that remain unobserved in the
//! current read model.

use leptos::prelude::*;
use mycelix_leptos_core::{AvailabilityState, AvailabilityStateKind};

use crate::mutation_ledger::use_mutation_ledger;
use crate::mutation_state::PersonalMutationTarget;

#[component]
pub fn MutationPendingNotice(target: PersonalMutationTarget) -> impl IntoView {
    let ledger = use_mutation_ledger();
    let target_for_items = target.clone();

    view! {
        <div class="mutation-evidence-list">
            <For
                each=move || ledger.for_target(&target_for_items)
                key=|receipt| receipt.action_hash.clone()
                children=move |receipt| {
                    let title = receipt.title().to_string();
                    let description = receipt.description();
                    let action_hash = receipt.action_hash;
                    view! {
                        <AvailabilityState
                            kind=AvailabilityStateKind::Degraded
                            title=title
                            description=description
                            action={Some(view! {
                                <code class="hash-line">{action_hash}</code>
                            }.into_any())}
                        />
                    }
                }
            />
        </div>
    }
}
