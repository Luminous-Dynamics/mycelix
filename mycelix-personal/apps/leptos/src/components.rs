// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Presentation-only helpers for the Personal frontend.
//!
//! This module intentionally owns no Holochain calls, mutation/rollback logic,
//! runtime-mode decisions, capability checks, route declarations, or domain
//! authority. It only renders values already supplied by Personal state.

use leptos::prelude::*;
use mycelix_leptos_core::FreshnessLevel;
use personal_leptos_types::{
    ActivityItemView, ConsentGrantView, CredentialType, MasterKeyView, StoredCredentialView,
};

#[component]
pub fn PageHeader(
    eyebrow: &'static str,
    #[prop(into)] title: TextProp,
    #[prop(into)] summary: TextProp,
) -> impl IntoView {
    view! {
        <header class="page-header">
            <span class="page-eyebrow">{eyebrow}</span>
            <h1>{move || title.get()}</h1>
            <p>{move || summary.get()}</p>
        </header>
    }
}

#[component]
pub fn SectionTitle(title: &'static str) -> impl IntoView {
    view! { <h2 class="section-title">{title}</h2> }
}

#[component]
pub fn VaultStat<F>(#[prop(into)] label: TextProp, value: F) -> impl IntoView
where
    F: Fn() -> String + 'static,
{
    view! {
        <div class="vault-stat">
            <span class="vault-stat-label">{move || label.get()}</span>
            <strong class="vault-stat-value">{value()}</strong>
        </div>
    }
}

#[component]
pub fn KeyCard(key_data: MasterKeyView) -> impl IntoView {
    view! {
        <div class="mini-card">
            <div class="mini-card-header">
                <strong>{key_data.label}</strong>
                <span class="status-pill" class:status-pill-active=key_data.active>
                    {if key_data.active { "Active" } else { "Inactive" }}
                </span>
            </div>
            <p class="mini-card-meta">{format!("Purpose: {}", key_data.purpose)}</p>
            <code class="hash-line">{key_data.public_key_hex}</code>
        </div>
    }
}

#[component]
pub fn CredentialCard(credential: StoredCredentialView) -> impl IntoView {
    let kind_label = credential.credential_type.label().to_string();
    let tone = match credential.credential_type {
        CredentialType::Identity => "tone-identity",
        CredentialType::Health => "tone-health",
        CredentialType::FederatedLearning => "tone-learning",
        CredentialType::Governance => "tone-governance",
        CredentialType::Domain(_) => "tone-domain",
    };

    view! {
        <article class=format!("credential-card {}", tone)>
            <div class="credential-topline">
                <span class="credential-kind">{kind_label}</span>
                <span class="status-pill" class:status-pill-active=!credential.revoked>
                    {if credential.revoked { "Revoked" } else { "Active" }}
                </span>
            </div>
            <strong class="credential-issuer">{credential.issuer}</strong>
            <p class="supporting-copy">
                {match credential.expires_at {
                    Some(_) => "Portable credential with an explicit expiry window.",
                    None => "Portable credential with no current expiry recorded.",
                }}
            </p>
            <code class="hash-line">{credential.hash}</code>
        </article>
    }
}

#[component]
pub fn ConsentCard(grant: ConsentGrantView) -> impl IntoView {
    view! {
        <article class="mini-card">
            <div class="mini-card-header">
                <strong>{grant.grantee}</strong>
                <span class="status-pill" class:status-pill-active=grant.active>
                    {if grant.active { "Active" } else { "Inactive" }}
                </span>
            </div>
            <p class="mini-card-meta">{format!("Types: {}", grant.record_types.join(", "))}</p>
        </article>
    }
}

#[component]
pub fn ActivityItemCard(item: ActivityItemView, wide: bool) -> impl IntoView {
    let class_name = if wide {
        "activity-item activity-item-wide"
    } else {
        "activity-item"
    };

    view! {
        <li class=class_name>
            <div class="mini-card-header">
                <strong>{format!("{} · {}", item.domain, item.title)}</strong>
                {item.success.map(|ok| {
                    view! {
                        <span class="status-pill" class:status-pill-active=ok>
                            {if ok { "Success" } else { "Blocked" }}
                        </span>
                    }
                })}
            </div>
            <p class="mini-card-meta">{item.detail}</p>
        </li>
    }
}

pub fn freshness_from_micros(timestamp_micros: i64) -> FreshnessLevel {
    let now_micros = (js_sys::Date::now() * 1000.0) as i64;
    let age_minutes = now_micros.saturating_sub(timestamp_micros) / 60_000_000;
    if age_minutes <= 5 {
        FreshnessLevel::Fresh
    } else if age_minutes <= 60 {
        FreshnessLevel::Aging
    } else {
        FreshnessLevel::Stale
    }
}

pub fn format_relative_micros(timestamp_micros: i64) -> String {
    let date = js_sys::Date::new(&wasm_bindgen::JsValue::from_f64(
        (timestamp_micros / 1000) as f64,
    ));
    date.to_locale_string("en-US", &wasm_bindgen::JsValue::UNDEFINED)
        .as_string()
        .unwrap_or_else(|| "recently".into())
}
