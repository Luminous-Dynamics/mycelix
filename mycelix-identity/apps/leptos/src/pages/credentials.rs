// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Verifiable credentials held and issued by the current sovereign identity.
//!
//! Live mode is driven exclusively by the typed credential projection from the
//! Identity DNA. Raw Holochain Records and proof envelopes never reach the UI.

use leptos::prelude::*;
use crate::identity_context::use_identity;
use identity_leptos_types::*;

fn credential_status(credential: &CredentialView) -> &'static str {
    if credential.revoked {
        return "Revoked";
    }

    if let Some(valid_until) = credential.valid_until.as_deref() {
        let expiry_ms = js_sys::Date::parse(valid_until);
        if expiry_ms.is_finite() {
            let now_ms = js_sys::Date::now();
            if now_ms > expiry_ms {
                return "Expired";
            }
        }
    }

    "Active"
}

fn short_did(did: &str) -> String {
    if did.len() > 34 {
        format!("{}...{}", &did[..20], &did[did.len() - 10..])
    } else {
        did.to_string()
    }
}

fn render_credential(credential: &CredentialView) -> AnyView {
    let status = credential_status(credential);
    let status_class = match status {
        "Active" => "credential-active",
        "Expired" => "credential-expired",
        _ => "credential-revoked",
    };
    let title = credential.primary_type().to_string();
    let issuer = short_did(&credential.issuer_did);
    let subject = short_did(&credential.subject_did);
    let valid_from = credential.valid_from.clone();
    let valid_until = credential
        .valid_until
        .clone()
        .unwrap_or_else(|| "No expiry declared".into());
    let claims = serde_json::to_string_pretty(&credential.claims)
        .unwrap_or_else(|_| "{}".into());
    let schema = credential
        .schema_id
        .clone()
        .unwrap_or_else(|| "No schema".into());

    view! {
        <article class="stat-card credential-card">
            <div class="credential-header">
                <div>
                    <strong>{title}</strong>
                    <div style="margin-top: 4px; font-family: var(--font-mono); font-size: var(--text-xs); opacity: 0.65;">
                        {short_did(&credential.id)}
                    </div>
                </div>
                <span class={format!("credential-status {}", status_class)}>{status}</span>
            </div>

            <div class="credential-meta-grid">
                <div class="meta-item">
                    <span class="meta-label">"Issuer"</span>
                    <code class="meta-value">{issuer}</code>
                </div>
                <div class="meta-item">
                    <span class="meta-label">"Subject"</span>
                    <code class="meta-value">{subject}</code>
                </div>
                <div class="meta-item">
                    <span class="meta-label">"Valid from"</span>
                    <span class="meta-value">{valid_from}</span>
                </div>
                <div class="meta-item">
                    <span class="meta-label">"Valid until"</span>
                    <span class="meta-value">{valid_until}</span>
                </div>
            </div>

            <div style="margin-top: var(--space-md);">
                <span class="meta-label">"Schema"</span>
                <code class="meta-value" style="display: block; margin-top: 4px;">{schema}</code>
            </div>

            <details style="margin-top: var(--space-md);">
                <summary style="cursor: pointer; font-weight: 600;">"View claims"</summary>
                <pre style="margin-top: var(--space-sm); overflow: auto; font-size: var(--text-xs);">{claims}</pre>
            </details>
        </article>
    }.into_any()
}

#[component]
pub fn CredentialsPage() -> impl IntoView {
    let ctx = use_identity();
    let held = move || ctx.credentials_held.get();
    let issued = move || ctx.credentials_issued.get();

    view! {
        <div class="page page-credentials">
            <h1>"Credentials"</h1>
            <p class="page-subtitle">
                "Verifiable credentials held and issued by your sovereign identity"
            </p>

            <section class="trust-section">
                <div style="display: flex; justify-content: space-between; align-items: baseline; gap: var(--space-md);">
                    <div>
                        <h2>"Held Credentials"</h2>
                        <p class="section-desc">"Credentials where your DID is the subject"</p>
                    </div>
                    <span class="stat-label">{move || format!("{} credential(s)", held().len())}</span>
                </div>

                {move || {
                    let creds = held();
                    if creds.is_empty() {
                        return view! {
                            <div class="empty-state">
                                <p>"No credentials held by this identity."</p>
                                <p class="empty-hint">"Issued credentials will appear here after they are published to the Identity DHT."</p>
                            </div>
                        }.into_any();
                    }

                    view! {
                        <div style="display: grid; gap: var(--space-md);">
                            {creds.iter().map(render_credential).collect::<Vec<_>>()}
                        </div>
                    }.into_any()
                }}
            </section>

            <section class="trust-section">
                <div style="display: flex; justify-content: space-between; align-items: baseline; gap: var(--space-md);">
                    <div>
                        <h2>"Issued Credentials"</h2>
                        <p class="section-desc">"Credentials issued by your DID"</p>
                    </div>
                    <span class="stat-label">{move || format!("{} credential(s)", issued().len())}</span>
                </div>

                {move || {
                    let creds = issued();
                    if creds.is_empty() {
                        return view! {
                            <div class="empty-state">
                                <p>"No credentials issued by this identity."</p>
                            </div>
                        }.into_any();
                    }

                    view! {
                        <div style="display: grid; gap: var(--space-md);">
                            {creds.iter().map(render_credential).collect::<Vec<_>>()}
                        </div>
                    }.into_any()
                }}
            </section>
        </div>
    }
}
