// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

use leptos::prelude::*;
use crate::identity_context::use_identity;
use identity_leptos_types::*;

#[derive(Clone, Copy, PartialEq)]
enum CredTab { Held, Issued }

#[component]
pub fn CredentialsPage() -> impl IntoView {
    let ctx = use_identity();
    let (active_tab, set_active_tab) = signal(CredTab::Held);
    let now_secs = move || (js_sys::Date::now() / 1000.0) as i64;

    let held = move || ctx.credentials_held.get();
    let issued = move || ctx.credentials_issued.get();

    view! {
        <div class="page page-credentials">
            <h1>"Credentials"</h1>
            <p class="page-subtitle">"Verifiable claims about your identity \u{2014} held, issued, and verified on the DHT"</p>

            // ── Tab Bar ──
            <div class="cred-tabs" role="tablist">
                <button
                    class=move || if active_tab.get() == CredTab::Held { "cred-tab active" } else { "cred-tab" }
                    role="tab"
                    aria-selected=move || (active_tab.get() == CredTab::Held).to_string()
                    on:click=move |_| set_active_tab.set(CredTab::Held)
                >
                    {move || format!("Held ({})", held().len())}
                </button>
                <button
                    class=move || if active_tab.get() == CredTab::Issued { "cred-tab active" } else { "cred-tab" }
                    role="tab"
                    aria-selected=move || (active_tab.get() == CredTab::Issued).to_string()
                    on:click=move |_| set_active_tab.set(CredTab::Issued)
                >
                    {move || format!("Issued ({})", issued().len())}
                </button>
            </div>

            // ── Held Credentials ──
            {move || {
                if active_tab.get() != CredTab::Held { return view! { <div /> }.into_any(); }
                let creds = held();
                let now = now_secs();
                if creds.is_empty() {
                    return view! {
                        <div class="empty-state">
                            <p>"\u{1F4DC} No credentials held yet"</p>
                            <p class="empty-hint">"Request credentials from issuers in your community"</p>
                        </div>
                    }.into_any();
                }
                view! {
                    <div class="cred-grid">
                        {creds.iter().map(|cred| {
                            let status = cred.status_label(now);
                            let status_class = match status {
                                "Active" => "cred-status-active",
                                "Expired" => "cred-status-expired",
                                "Revoked" => "cred-status-revoked",
                                _ => "cred-status-unknown",
                            };
                            let primary_type = cred.primary_type().to_string();
                            let issuer_short = if cred.issuer_did.len() > 30 {
                                format!("{}...", &cred.issuer_did[..30])
                            } else {
                                cred.issuer_did.clone()
                            };
                            let claims_preview = format_claims(&cred.claims);
                            let expiry_text = cred.expires_at
                                .map(|exp| format_relative_time(exp, now))
                                .unwrap_or_else(|| "No expiry".into());

                            view! {
                                <div class="cred-card">
                                    <div class="cred-card-header">
                                        <span class="cred-type">{primary_type}</span>
                                        <span class={format!("cred-status {status_class}")}>{status}</span>
                                    </div>
                                    <div class="cred-claims">{claims_preview}</div>
                                    <div class="cred-meta">
                                        <div class="cred-meta-item">
                                            <span class="cred-meta-label">"Issuer"</span>
                                            <span class="cred-meta-value">{issuer_short}</span>
                                        </div>
                                        <div class="cred-meta-item">
                                            <span class="cred-meta-label">"Expires"</span>
                                            <span class="cred-meta-value">{expiry_text}</span>
                                        </div>
                                    </div>
                                    {cred.schema_id.as_ref().map(|schema| view! {
                                        <div class="cred-schema">
                                            <span class="cred-meta-label">"Schema"</span>
                                            <code class="cred-schema-id">{schema.clone()}</code>
                                        </div>
                                    })}
                                </div>
                            }
                        }).collect::<Vec<_>>()}
                    </div>
                }.into_any()
            }}

            // ── Issued Credentials ──
            {move || {
                if active_tab.get() != CredTab::Issued { return view! { <div /> }.into_any(); }
                let creds = issued();
                let now = now_secs();
                if creds.is_empty() {
                    return view! {
                        <div class="empty-state">
                            <p>"\u{270D} No credentials issued yet"</p>
                            <p class="empty-hint">"Issue credentials to attest skills, memberships, or achievements"</p>
                        </div>
                    }.into_any();
                }
                view! {
                    <div class="cred-grid">
                        {creds.iter().map(|cred| {
                            let primary_type = cred.primary_type().to_string();
                            let subject_short = if cred.subject_did.len() > 30 {
                                format!("{}...", &cred.subject_did[..30])
                            } else {
                                cred.subject_did.clone()
                            };
                            let claims_preview = format_claims(&cred.claims);
                            let status = cred.status_label(now);

                            view! {
                                <div class="cred-card cred-card-issued">
                                    <div class="cred-card-header">
                                        <span class="cred-type">{primary_type}</span>
                                        <span class="cred-issued-badge">"Issued by you"</span>
                                    </div>
                                    <div class="cred-claims">{claims_preview}</div>
                                    <div class="cred-meta">
                                        <div class="cred-meta-item">
                                            <span class="cred-meta-label">"Subject"</span>
                                            <span class="cred-meta-value">{subject_short}</span>
                                        </div>
                                        <div class="cred-meta-item">
                                            <span class="cred-meta-label">"Status"</span>
                                            <span class="cred-meta-value">{status}</span>
                                        </div>
                                    </div>
                                </div>
                            }
                        }).collect::<Vec<_>>()}
                    </div>
                }.into_any()
            }}
        </div>
    }
}

fn format_claims(claims: &serde_json::Value) -> String {
    if let Some(obj) = claims.as_object() {
        obj.iter()
            .take(3)
            .map(|(k, v)| {
                let val = match v {
                    serde_json::Value::String(s) => s.clone(),
                    serde_json::Value::Number(n) => n.to_string(),
                    serde_json::Value::Bool(b) => b.to_string(),
                    _ => v.to_string(),
                };
                format!("{}: {}", k, val)
            })
            .collect::<Vec<_>>()
            .join(" \u{00B7} ")
    } else {
        String::new()
    }
}

fn format_relative_time(timestamp: i64, now: i64) -> String {
    let diff = timestamp - now;
    if diff < 0 {
        let days = (-diff) / 86400;
        if days > 365 { format!("Expired {}y ago", days / 365) }
        else if days > 30 { format!("Expired {}mo ago", days / 30) }
        else { format!("Expired {}d ago", days) }
    } else {
        let days = diff / 86400;
        if days > 365 { format!("in {}y", days / 365) }
        else if days > 30 { format!("in {}mo", days / 30) }
        else { format!("in {}d", days) }
    }
}
