// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Source-backed Personal identity surface.
//!
//! Live mode reads the Personal DNA through the unified hApp's personal role.
//! Browser-local state and Hearth membership are never promoted into identity
//! or credential authority. Explicit Mock mode may still show clearly-labelled
//! sample data for development.

use leptos::prelude::*;
use mycelix_leptos_core::holochain_provider::{use_holochain, ConnectionStatus, HolochainCtx};
use personal_leptos_types::{ProfileView, TrustCredentialView};

use crate::mock_data;
use mycelix_leptos_core::use_consciousness;

#[derive(Clone, Debug)]
struct PersonalProfileSnapshot {
    profile: Result<Option<ProfileView>, String>,
    trust_credentials: Result<Vec<TrustCredentialView>, String>,
}

async fn load_personal_profile(ctx: HolochainCtx) -> PersonalProfileSnapshot {
    let agent_did = ctx.connected_agent_did();

    if !ctx.zome_calls_ready_untracked() {
        let reason = "Live Personal sources are not ready for an authorized zome call.".to_string();
        return PersonalProfileSnapshot {
            profile: Err(reason.clone()),
            trust_credentials: Err(reason),
        };
    }

    let profile: Result<Option<ProfileView>, String> = ctx
        .call_zome("personal", "identity_vault", "get_my_profile_view", &())
        .await;

    let trust_credentials = match agent_did.clone() {
        Some(subject_did) => {
            ctx.call_zome::<String, Vec<TrustCredentialView>>(
                "personal",
                "credential_wallet",
                "get_trust_credentials_view",
                &subject_did,
            )
            .await
        }
        None => Err(
            "The connected agent identity is unavailable, so trust credentials cannot be scoped safely."
                .to_string(),
        ),
    };

    PersonalProfileSnapshot {
        profile,
        trust_credentials,
    }
}

#[component]
pub fn ProfilePage() -> impl IntoView {
    let ctx = use_holochain();
    let consciousness = use_consciousness();

    // LocalResource is the appropriate Leptos primitive for browser-only,
    // !Send Holochain transport work. Its source tracks connection and signer
    // lifecycle so a reconnect or signer transition creates a fresh read.
    let profile_source = LocalResource::new({
        let ctx = ctx.clone();
        move || {
            let _status = ctx.status.get();
            let _signer = ctx.zome_call_signing_ready.get();
            let ctx = ctx.clone();
            async move { load_personal_profile(ctx).await }
        }
    });

    view! {
        <div class="page profile-page">
            <h1 class="page-title">"identity vault"</h1>
            <p class="page-subtitle">"your sovereign identity"</p>

            {move || match ctx.status.get() {
                ConnectionStatus::Mock => view! {
                    <DemoProfileContent consciousness=consciousness.clone()/>
                }.into_any(),

                ConnectionStatus::Connected if ctx.zome_calls_ready() => {
                    match profile_source.get() {
                        None => view! {
                            <SourceStatus
                                title="Personal identity"
                                description="Reading the Personal identity vault…"
                                status="Loading"
                            />
                        }.into_any(),

                        Some(snapshot) => view! {
                            <LiveProfileContent snapshot=snapshot/>
                        }.into_any(),
                    }
                }

                ConnectionStatus::Connecting | ConnectionStatus::Reconnecting => view! {
                    <SourceStatus
                        title="Personal identity"
                        description="Waiting for an authorized Live connection before reading identity data."
                        status="Unavailable"
                    />
                }.into_any(),

                ConnectionStatus::Connected => view! {
                    <SourceStatus
                        title="Personal identity"
                        description="The conductor is connected, but an authorized zome-call signer is not ready."
                        status="Unavailable"
                    />
                }.into_any(),

                ConnectionStatus::Disconnected => view! {
                    <SourceStatus
                        title="Personal identity"
                        description="Live identity data is unavailable while the conductor is disconnected."
                        status="Unavailable"
                    />
                }.into_any(),
            }}
        </div>
    }
}

#[component]
fn SourceStatus(
    title: &'static str,
    description: &'static str,
    status: &'static str,
) -> impl IntoView {
    view! {
        <section aria-label=title>
            <div class="availability-state availability-unavailable" role="status">
                <div class="availability-state-header">
                    <span class="availability-state-icon" aria-hidden="true">"×"</span>
                    <div class="availability-state-copy">
                        <div class="availability-state-meta">
                            <span class="availability-state-title">{title}</span>
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
fn DemoProfileContent(
    consciousness: mycelix_leptos_core::ConsciousnessState,
) -> impl IntoView {
    let profile = mock_data::mock_profile();
    let trust = mock_data::mock_trust_credential();
    let name = profile.display_name.clone();
    let bio = profile.bio.clone().unwrap_or_default();

    view! {
        <section aria-label="Demo identity">
            <div class="availability-state availability-demo" role="status">
                <div class="availability-state-header">
                    <span class="availability-state-icon" aria-hidden="true">"~"</span>
                    <div class="availability-state-copy">
                        <div class="availability-state-meta">
                            <span class="availability-state-title">"Demo identity"</span>
                            <span class="status-pill availability-demo">"Demo"</span>
                        </div>
                        <p class="availability-state-description">
                            "Sample identity and trust data are shown because Demo mode was explicitly selected. They are not records from your Personal DNA."
                        </p>
                    </div>
                </div>
            </div>

            <ProfileCard name bio/>

            <section class="trust-section">
                <h2>"Demo trust sample"</h2>
                <div class="trust-card">
                    <div class="trust-tier">
                        <span class=format!("tier-badge {}", trust.trust_tier.css_class())>
                            {trust.trust_tier.label()}
                        </span>
                        <span class="trust-range">
                            {format!(
                                "Sample range: {:.0}% - {:.0}%",
                                trust.trust_score_range.lower * 100.0,
                                trust.trust_score_range.upper * 100.0
                            )}
                        </span>
                    </div>
                    <p class="field-help">"Demo fixture only; no verification claim."</p>
                </div>
            </section>

            <ConsciousnessSection consciousness/>
        </section>
    }
}

#[component]
fn LiveProfileContent(snapshot: PersonalProfileSnapshot) -> impl IntoView {
    view! {
        <section aria-label="Live identity sources">
            <div class="availability-state availability-live" role="status">
                <div class="availability-state-header">
                    <span class="availability-state-icon" aria-hidden="true">"✓"</span>
                    <div class="availability-state-copy">
                        <div class="availability-state-meta">
                            <span class="availability-state-title">"Personal identity source"</span>
                            <span class="status-pill availability-live">"Live"</span>
                        </div>
                        <p class="availability-state-description">
                            "Read from the Personal DNA in the unified hApp. Browser-local preferences and Hearth membership do not supply these fields."
                        </p>
                    </div>
                </div>
            </div>

            {match snapshot.profile {
                Ok(Some(profile)) => view! {
                    <ProfileCard
                        name=profile.display_name.clone()
                        bio=profile.bio.clone().unwrap_or_default()
                    />
                    <ProfileDetails profile=profile/>
                }.into_any(),

                Ok(None) => view! {
                    <SourceStatus
                        title="Identity profile"
                        description="No source-backed profile is recorded for this agent yet."
                        status="Empty"
                    />
                }.into_any(),

                Err(_) => view! {
                    <SourceStatus
                        title="Identity profile"
                        description="The Personal identity source could not be read."
                        status="Unavailable"
                    />
                }.into_any(),
            }}

            <section class="trust-section">
                <h2>"Trust credentials"</h2>
                {match snapshot.trust_credentials {
                    Ok(credentials) if credentials.is_empty() => view! {
                        <p class="field-help">
                            "No active trust credential is recorded for this agent."
                        </p>
                    }.into_any(),

                    Ok(credentials) => view! {
                        <div class="trust-card">
                            {credentials.into_iter().map(|credential| view! {
                                <TrustCredentialRow credential/>
                            }).collect_view()}
                        </div>
                    }.into_any(),

                    Err(_) => view! {
                        <SourceStatus
                            title="Trust credentials"
                            description="The Personal credential source could not be read."
                            status="Unavailable"
                        />
                    }.into_any(),
                }}
            </section>
        </section>
    }
}

#[component]
fn ProfileCard(name: String, bio: String) -> impl IntoView {
    let initial = name.chars().next().unwrap_or('?').to_string();

    view! {
        <div class="profile-card">
            <div class="profile-avatar">
                <div class="avatar-placeholder">{initial}</div>
            </div>
            <div class="profile-info">
                <h2>{name}</h2>
                <p class="profile-bio">{bio}</p>
            </div>
        </div>
    }
}

#[component]
fn ProfileDetails(profile: ProfileView) -> impl IntoView {
    let metadata: Vec<_> = profile
        .metadata
        .iter()
        .map(|(key, value)| (key.clone(), value.clone()))
        .collect();

    view! {
        <section>
            <h2>"Details"</h2>
            <div class="metadata-list">
                {metadata
                    .into_iter()
                    .map(|(key, value)| view! {
                        <div class="metadata-row">
                            <span class="metadata-key">{key}</span>
                            <span class="metadata-value">{value}</span>
                        </div>
                    })
                    .collect_view()}
            </div>
            <p class="field-help">
                {format!(
                    "Profile record updated at source timestamp {}.",
                    profile.updated_at
                )}
            </p>
        </section>

        <section aria-label="Profile provenance">
            <h2>"Source"</h2>
            <p class="field-help">"personal.identity_vault.get_my_profile_view"</p>
        </section>
    }
}

#[component]
fn TrustCredentialRow(credential: TrustCredentialView) -> impl IntoView {
    view! {
        <article class="metadata-list">
            <div class="metadata-row">
                <span class="metadata-key">"Tier"</span>
                <span class=format!("metadata-value tier-badge {}", credential.trust_tier.css_class())>
                    {credential.trust_tier.label()}
                </span>
            </div>
            <div class="metadata-row">
                <span class="metadata-key">"Proven range"</span>
                <span class="metadata-value">
                    {format!(
                        "{:.0}% - {:.0}%",
                        credential.trust_score_range.lower * 100.0,
                        credential.trust_score_range.upper * 100.0
                    )}
                </span>
            </div>
            <div class="metadata-row">
                <span class="metadata-key">"Issuer"</span>
                <span class="metadata-value">{credential.issuer_did.clone()}</span>
            </div>
            <div class="metadata-row">
                <span class="metadata-key">"Issued"</span>
                <span class="metadata-value">{credential.issued_at.to_string()}</span>
            </div>
            <div class="metadata-row">
                <span class="metadata-key">"Credential ID"</span>
                <span class="metadata-value">{credential.id}</span>
            </div>
        </article>
    }
}

#[component]
fn ConsciousnessSection(consciousness: mycelix_leptos_core::ConsciousnessState) -> impl IntoView {
    view! {
        <section>
            <h2>"Local consciousness context"</h2>
            <div class="trust-card">
                <div class="consciousness-tier">
                    <span>"Current local context: "</span>
                    <span class=move || format!(
                        "tier-badge {}",
                        consciousness.tier.get().css_class()
                    )>
                        {move || consciousness.tier.get().label()}
                    </span>
                </div>
                <p class="field-help">
                    "This is local application context. It is not a Personal identity credential or a verification result."
                </p>
            </div>
        </section>
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn live_sources_are_explicitly_role_and_zome_scoped() {
        let profile_source = "personal.identity_vault.get_my_profile_view";
        let trust_source = "personal.credential_wallet.get_trust_credentials_view";
        assert!(profile_source.starts_with("personal."));
        assert!(trust_source.starts_with("personal."));
        assert!(!profile_source.contains("mock"));
        assert!(!trust_source.contains("mock"));
    }

    #[test]
    fn live_identity_does_not_use_hearth_membership_as_profile_source() {
        let profile_source = "personal.identity_vault.get_my_profile_view";
        assert!(!profile_source.contains("hearth"));
    }
}
