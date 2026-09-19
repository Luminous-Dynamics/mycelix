// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Read-only Personal page surfaces.
//!
//! These pages deliberately contain no Holochain provider access, mutation
//! calls, optimistic publication, rollback behavior, capability checks, or
//! authorization decisions. They only render already-established Personal
//! context state.

use leptos::prelude::*;

use crate::components::{ActivityItemCard, CredentialCard, PageHeader, SectionTitle};
use crate::context::use_personal;

#[component]
pub fn WalletPage() -> impl IntoView {
    let ctx = use_personal();

    view! {
        <div class="stack-page">
            <PageHeader
                eyebrow="Credential Wallet"
                title="Portable proof inventory"
                summary="Stored credentials, proof posture, and trust-bearing materials live here.".to_string()
            />

            <section class="vault-card">
                <SectionTitle title="Credentials" />
                <div class="credential-grid">
                    <For
                        each=move || ctx.credentials.get()
                        key=|cred| cred.hash.clone()
                        children=move |cred| view! { <CredentialCard credential=cred /> }
                    />
                </div>
            </section>
        </div>
    }
}

#[component]
pub fn ActivityPage() -> impl IntoView {
    let ctx = use_personal();

    view! {
        <div class="stack-page">
            <PageHeader
                eyebrow="Activity"
                title="What left the vault, and why"
                summary="This page will eventually reflect bridge queries, events, and disclosures directly.".to_string()
            />
            <section class="vault-card">
                <SectionTitle title="Disclosure and handoff log" />
                <ul class="activity-list">
                    <For
                        each=move || ctx.activity.get()
                        key=|entry| entry.id.clone()
                        children=move |entry| view! { <ActivityItemCard item=entry wide=true /> }
                    />
                </ul>
            </section>
        </div>
    }
}

#[component]
pub fn UnlockPage() -> impl IntoView {
    view! {
        <div class="stack-page">
            <PageHeader
                eyebrow="Unlock"
                title="Vault entry surface"
                summary="Biometric and passphrase unlock belongs here once Personal runtime security flows are wired.".to_string()
            />
            <section class="vault-card narrow-card">
                <p class="supporting-copy">
                    "The route now exists so portal and mobile wrappers have a stable unlock target. "
                    "Secure unlock implementation should follow after the conductor-facing Personal view layer."
                </p>
            </section>
        </div>
    }
}
