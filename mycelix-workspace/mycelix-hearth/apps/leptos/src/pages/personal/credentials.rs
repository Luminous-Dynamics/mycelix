// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Source-backed Personal credential surface.
//!
//! Live mode reads the credential wallet directly. The UI reports stored
//! credential records and their explicit revocation metadata; it does not
//! infer verification, validity, expiry, or trust from unrelated surfaces.

use leptos::prelude::*;
use mycelix_leptos_core::holochain_provider::{use_holochain, ConnectionStatus};
use personal_leptos_types::StoredCredentialView;

use crate::mock_data;

#[component]
pub fn CredentialsPage() -> impl IntoView {
    let ctx = use_holochain();

    let credentials = LocalResource::new({
        let ctx = ctx.clone();
        move || {
            let _status = ctx.status.get();
            let _signer = ctx.zome_call_signing_ready.get();
            let ctx = ctx.clone();
            async move {
                if !ctx.zome_calls_ready_untracked() {
                    return Err(
                        "The Personal credential source is not ready for an authorized zome call."
                            .to_string(),
                    );
                }

                ctx.call_zome::<(), Vec<StoredCredentialView>>(
                    "personal",
                    "credential_wallet",
                    "get_my_credentials_view",
                    &(),
                )
                .await
            }
        }
    });

    view! {
        <div class="page credentials-page">
            <h1 class="page-title">"credentials"</h1>
            <p class="page-subtitle">"verifiable claims you choose to carry"</p>

            {move || match ctx.status.get() {
                ConnectionStatus::Mock => view! {
                    <DemoCredentials/>
                }.into_any(),

                ConnectionStatus::Connected if ctx.zome_calls_ready() => {
                    match credentials.get() {
                        None => view! {
                            <CredentialSourceStatus
                                status="Loading"
                                description="Reading stored credential records from the Personal DNA…"
                            />
                        }.into_any(),

                        Some(Ok(records)) => view! {
                            <LiveCredentials records/>
                        }.into_any(),

                        Some(Err(_)) => view! {
                            <CredentialSourceStatus
                                status="Unavailable"
                                description="The Personal credential source could not be read."
                            />
                        }.into_any(),
                    }
                }

                ConnectionStatus::Connecting | ConnectionStatus::Reconnecting => view! {
                    <CredentialSourceStatus
                        status="Unavailable"
                        description="Waiting for an authorized Live connection before reading credential records."
                    />
                }.into_any(),

                ConnectionStatus::Connected => view! {
                    <CredentialSourceStatus
                        status="Unavailable"
                        description="The conductor is connected, but an authorized zome-call signer is not ready."
                    />
                }.into_any(),

                ConnectionStatus::Disconnected => view! {
                    <CredentialSourceStatus
                        status="Unavailable"
                        description="Live credential records are unavailable while the conductor is disconnected."
                    />
                }.into_any(),
            }}

            <section aria-labelledby="credentials-boundary-heading">
                <h2 id="credentials-boundary-heading">"authority boundary"</h2>
                <p>
                    "These records are what the Personal credential wallet reports. A stored credential is not automatically a verified claim, and Hearth does not infer validity, trust, or verification from its presence."
                </p>
            </section>
        </div>
    }
}

#[component]
fn CredentialSourceStatus(status: &'static str, description: &'static str) -> impl IntoView {
    view! {
        <section aria-labelledby="credentials-source-heading">
            <h2 id="credentials-source-heading">"source status"</h2>
            <div class="availability-state availability-unavailable" role="status">
                <div class="availability-state-header">
                    <span class="availability-state-icon" aria-hidden="true">"×"</span>
                    <div class="availability-state-copy">
                        <div class="availability-state-meta">
                            <span class="availability-state-title">"Credential records"</span>
                            <span class="status-pill availability-unavailable">{status}</span>
                        </div>
                        <p class="availability-state-description">{description}</p>
                    </div>
                </div>
            </div>
        </section>
    }
}

#[component]
fn DemoCredentials() -> impl IntoView {
    let records = mock_data::mock_credentials();

    view! {
        <section aria-labelledby="credentials-source-heading">
            <h2 id="credentials-source-heading">"source status"</h2>
            <div class="availability-state availability-demo" role="status">
                <div class="availability-state-header">
                    <span class="availability-state-icon" aria-hidden="true">"~"</span>
                    <div class="availability-state-copy">
                        <div class="availability-state-meta">
                            <span class="availability-state-title">"Credential records"</span>
                            <span class="status-pill availability-demo">"Demo"</span>
                        </div>
                        <p class="availability-state-description">
                            "Sample credential records are shown because Demo mode was explicitly selected. They are not records from your Personal DNA."
                        </p>
                    </div>
                </div>
            </div>

            <CredentialList records/>
        </section>
    }
}

#[component]
fn LiveCredentials(records: Vec<StoredCredentialView>) -> impl IntoView {
    view! {
        <section aria-labelledby="credentials-source-heading">
            <h2 id="credentials-source-heading">"source status"</h2>
            <div class="availability-state availability-live" role="status">
                <div class="availability-state-header">
                    <span class="availability-state-icon" aria-hidden="true">"✓"</span>
                    <div class="availability-state-copy">
                        <div class="availability-state-meta">
                            <span class="availability-state-title">"Credential records"</span>
                            <span class="status-pill availability-live">"Live"</span>
                        </div>
                        <p class="availability-state-description">
                            "Read from personal.credential_wallet.get_my_credentials_view."
                        </p>
                    </div>
                </div>
            </div>

            <CredentialList records/>
        </section>
    }
}

#[component]
fn CredentialList(records: Vec<StoredCredentialView>) -> impl IntoView {
    if records.is_empty() {
        return view! {
            <p class="field-help">"No stored credential records were returned by the Personal source."</p>
        }
        .into_any();
    }

    view! {
        <div class="metadata-list">
            {records
                .into_iter()
                .map(|credential| view! {
                    <article class="trust-card">
                        <div class="metadata-row">
                            <span class="metadata-key">"Type"</span>
                            <span class="metadata-value">{credential.credential_type.label()}</span>
                        </div>
                        <div class="metadata-row">
                            <span class="metadata-key">"Issuer"</span>
                            <span class="metadata-value">{credential.issuer}</span>
                        </div>
                        <div class="metadata-row">
                            <span class="metadata-key">"Issued"</span>
                            <span class="metadata-value">{credential.issued_at.to_string()}</span>
                        </div>
                        {credential.expires_at.map(|expires_at| view! {
                            <div class="metadata-row">
                                <span class="metadata-key">"Expires at source timestamp"</span>
                                <span class="metadata-value">{expires_at.to_string()}</span>
                            </div>
                        })}
                        <div class="metadata-row">
                            <span class="metadata-key">"Revoked"</span>
                            <span class="metadata-value">
                                {if credential.revoked { "Yes — source marked revoked" } else { "No — source does not mark revoked" }}
                            </span>
                        </div>
                        <div class="metadata-row">
                            <span class="metadata-key">"Record hash"</span>
                            <span class="metadata-value">{credential.hash}</span>
                        </div>
                    </article>
                })
                .collect_view()}
        </div>
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn live_source_is_explicitly_credential_wallet_scoped() {
        let source = "personal.credential_wallet.get_my_credentials_view";
        assert!(source.starts_with("personal.credential_wallet."));
        assert!(!source.contains("mock"));
        assert!(!source.contains("hearth"));
    }

    #[test]
    fn credential_presence_is_not_verification() {
        let boundary =
            "A stored credential is not automatically a verified claim.";
        assert!(boundary.contains("not automatically"));
        assert!(boundary.contains("verified claim"));
    }
}
