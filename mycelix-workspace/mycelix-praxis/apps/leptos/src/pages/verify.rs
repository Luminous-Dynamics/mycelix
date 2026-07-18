// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Fail-closed credential verification gateway.
//!
//! Praxis issues credentials with proof fields, but this build does not yet
//! verify the issuer signature, expiry, or revocation status. A reference must
//! never become "verified" because it is syntactically plausible.

use leptos::prelude::*;

#[component]
pub fn VerificationPage() -> impl IntoView {
    let (input_reference, set_input_reference) = signal(String::new());
    let (inspected_reference, set_inspected_reference) = signal::<Option<String>>(None);

    let inspect_reference = move |_| {
        let reference = input_reference.get().trim().to_string();
        set_inspected_reference.set((!reference.is_empty()).then_some(reference));
    };

    view! {
        <div class="verify-page">
            <header class="verify-header">
                <h2>"Credential Verification"</h2>
                <p>"Verification is unavailable in this build and fails closed."</p>
            </header>

            <div class="verify-container">
                <div class="verify-input-box">
                    <label>"Credential action hash or identifier:"</label>
                    <input
                        type="text"
                        placeholder="Paste a credential reference"
                        prop:value=input_reference
                        on:input=move |event| {
                            set_input_reference.set(event_target_value(&event));
                            set_inspected_reference.set(None);
                        }
                    />
                    <button
                        class="btn-primary"
                        style="width: 100%; margin-top: 1rem"
                        on:click=inspect_reference
                    >
                        "Inspect verification status"
                    </button>
                </div>

                {move || view! {
                    <VerificationUnavailable reference=inspected_reference.get() />
                }}
            </div>

            <VerificationRequirements />
        </div>
    }
}

/// Shareable credential reference. The route preserves the supplied identifier
/// but does not resolve or verify it.
#[component]
pub fn VerificationPortal() -> impl IntoView {
    let params = leptos_router::hooks::use_params_map();
    let reference = move || params.read().get("id").unwrap_or_default();

    view! {
        <div class="verify-page">
            <header class="verify-header">
                <h2>"Credential Verification"</h2>
                <p>"This reference has not been verified."</p>
            </header>

            <div class="verify-container">
                {move || view! {
                    <VerificationUnavailable reference=Some(reference()) />
                }}
            </div>

            <VerificationRequirements />
        </div>
    }
}

#[component]
fn VerificationUnavailable(reference: Option<String>) -> impl IntoView {
    view! {
        <div class="verify-result-area">
            <div class="verify-fail">
                <strong>"Not verified"</strong>
                {reference
                    .filter(|value| !value.is_empty())
                    .map(|value| view! {
                        <p>
                            "Reference: "
                            <span class="mono">{value}</span>
                        </p>
                    })}
                <p>
                    "No signature, expiry, ownership, or revocation check has been run. The reference is displayed only as supplied."
                </p>
            </div>
        </div>
    }
}

#[component]
fn VerificationRequirements() -> impl IntoView {
    view! {
        <section class="verify-footer">
            <p>
                "A future positive result must reconstruct the canonical credential payload, verify the issuer signature, evaluate expiry, and consult an authenticated revocation source."
            </p>
            <p>
                "Proof fields, a Holochain action hash, or standards-compatible formatting are not by themselves verification."
            </p>
        </section>
    }
}
