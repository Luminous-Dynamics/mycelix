// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

use leptos::ev::SubmitEvent;
use leptos::prelude::*;
use leptos_router::components::{A, Route, Router, Routes};
use leptos_router::hooks::use_location;
use leptos_router::path;
use wasm_bindgen_futures::spawn_local;

use mycelix_leptos_core::consciousness::refresh_consciousness_from_conductor;
use mycelix_leptos_core::holochain_provider::ConnectionStatus;
use mycelix_leptos_core::{
    init_consciousness_ui, provide_consciousness_context, provide_homeostasis_context,
    provide_local_identity, provide_theme_context, provide_thermodynamic_context,
    provide_toast_context, use_toasts, ActivityFeed, ActivityFeedItem, AppShell, AvailabilityState,
    AvailabilityStateKind, EmptyState, FreshnessBadge, FreshnessLevel, HolochainProviderAuto,
    HolochainProviderConfig, NavLink, NavTab, ToastContainer, ToastKind,
};
use personal_leptos_types::{
    ConditionalMutationResultView, ConditionalProfileMutationInputView, MutationReceiptView,
};

use crate::components::{
    format_relative_micros, freshness_from_micros, ConsentCard, KeyCard, PageHeader, SectionTitle,
    VaultStat,
};
use crate::context::{
    provide_cultural_context, provide_personal_context, use_cultural, use_personal,
    PersonalSourceState, SymbolRegistry,
};
use crate::mutation_ledger::{provide_mutation_ledger, use_mutation_ledger};
use crate::mutation_refresh::{
    refresh_health_after_mutation, refresh_identity_after_mutation,
    refresh_preferences_after_mutation,
};
use crate::mutation_state::{MutationRefreshOutcome, PersonalMutationTarget};
use crate::mutation_truth::MutationPendingNotice;
use crate::pages::{ActivityPage, UnlockPage, WalletPage};
use crate::runtime_mode::{detect_runtime_mode, provide_runtime_mode, PersonalRuntimeMode};
use crate::telemetry::ConstellationTelemetry;

#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum PersonalTheme {
    Vault,
    Dawn,
}

impl mycelix_leptos_core::AppTheme for PersonalTheme {
    fn label(&self) -> &'static str {
        match self {
            Self::Vault => "vault",
            Self::Dawn => "dawn",
        }
    }

    fn all() -> &'static [Self] {
        &[Self::Vault, Self::Dawn]
    }

    fn next(&self) -> Self {
        match self {
            Self::Vault => Self::Dawn,
            Self::Dawn => Self::Vault,
        }
    }

    fn is_light(&self) -> bool {
        matches!(self, Self::Dawn)
    }
}

#[component]
pub fn App() -> impl IntoView {
    let runtime_mode = detect_runtime_mode();
    let config = HolochainProviderConfig {
        app_id: "mycelix-unified".into(),
        default_role: Some("personal".into()),
        log_prefix: "[Personal]",
        connect_strategy: runtime_mode.connect_strategy(),
        status_labels: None,
    };

    view! {
        <HolochainProviderAuto config=config>
            <AppInner runtime_mode />
        </HolochainProviderAuto>
    }
}

#[component]
fn AppInner(runtime_mode: PersonalRuntimeMode) -> impl IntoView {
    provide_runtime_mode(runtime_mode);
    provide_theme_context("personal-theme", PersonalTheme::Vault);
    provide_thermodynamic_context();
    let consciousness = provide_consciousness_context();
    provide_toast_context();
    provide_homeostasis_context(1, "--personal-homeostasis");
    provide_local_identity();
    provide_personal_context(runtime_mode);
    provide_mutation_ledger();
    provide_cultural_context();
    init_consciousness_ui();

    let hc = mycelix_leptos_core::holochain_provider::use_holochain();
    refresh_consciousness_from_conductor(&consciousness, &hc);

    let nav_links = vec![
        NavLink {
            href: "/",
            label: "Vault",
            icon: Some("◉"),
        },
        NavLink {
            href: "/identity",
            label: "Identity",
            icon: Some("ID"),
        },
        NavLink {
            href: "/wallet",
            label: "Wallet",
            icon: Some("VC"),
        },
        NavLink {
            href: "/health",
            label: "Health",
            icon: Some("HX"),
        },
        NavLink {
            href: "/preferences",
            label: "Preferences",
            icon: Some("PX"),
        },
        NavLink {
            href: "/activity",
            label: "Activity",
            icon: Some("AX"),
        },
        NavLink {
            href: "/profile",
            label: "Profile",
            icon: Some("ME"),
        },
        // Constellation Group (Satellite hApps)
        NavLink {
            href: "/civic",
            label: "Civic",
            icon: Some("⚔"),
        },
        NavLink {
            href: "/knowledge",
            label: "Knowledge",
            icon: Some("📖"),
        },
        NavLink {
            href: "/finance",
            label: "Finance",
            icon: Some("💰"),
        },
    ];

    let mobile_tabs = vec![
        NavTab {
            href: "/",
            icon: "◉",
            label: "Vault",
        },
        NavTab {
            href: "/wallet",
            icon: "VC",
            label: "Wallet",
        },
        NavTab {
            href: "/health",
            icon: "HX",
            label: "Health",
        },
        NavTab {
            href: "/preferences",
            icon: "PX",
            label: "Prefs",
        },
    ];

    view! {
        <Router>
            <AppShell
                brand_name="Personal"
                brand_icon="◉"
                nav_links=nav_links
                mobile_tabs=mobile_tabs
            >
                <Routes fallback=|| view! {
                    <EmptyState icon="?" title="Vault route not found" />
                }>
                    <Route path=path!("/") view=VaultPage />
                    <Route path=path!("/identity") view=IdentityPage />
                    <Route path=path!("/wallet") view=WalletPage />
                    <Route path=path!("/health") view=HealthPage />
                    <Route path=path!("/preferences") view=PreferencesPage />
                    <Route path=path!("/activity") view=ActivityPage />
                    <Route path=path!("/profile") view=IdentityPage />
                    <Route path=path!("/unlock") view=UnlockPage />
                    // Constellation Satellite Routes
                    <Route path=path!("/civic") view=|| view! { <EmbeddedSatellite name="Civic" port=5174 /> } />
                    <Route path=path!("/knowledge") view=|| view! { <EmbeddedSatellite name="Knowledge" port=5175 /> } />
                    <Route path=path!("/finance") view=|| view! { <EmbeddedSatellite name="Finance" port=5176 /> } />
                </Routes>
            </AppShell>
            <ToastContainer />
        </Router>
    }
}

#[component]
fn CulturalReskinSelector() -> impl IntoView {
    let cultural = use_cultural();
    let symbols = cultural.symbols;

    let set_context = move |id: &str| {
        let new_symbols = match id {
            "indigenous" => SymbolRegistry {
                hearth_alias: "WELL".into(),
                mycel_alias: "SPARK".into(),
                genesis_alias: "The Sun-Rise".into(),
                orientation: "Indigenous Stewardship metaphors".into(),
            },
            "community" => SymbolRegistry {
                hearth_alias: "CAMPFIRE".into(),
                mycel_alias: "EMBER".into(),
                genesis_alias: "The Ignition".into(),
                orientation: "Urban Mutual-Aid metaphors".into(),
            },
            _ => SymbolRegistry::default(),
        };
        symbols.set(new_symbols);
    };

    view! {
        <div style="display: flex; gap: 0.8rem; margin-bottom: 2rem; padding: 0.8rem; background: rgba(255,255,255,0.03); border-radius: 8px; border: 1px solid var(--md-divider);">
            <span style="font-size: 0.75rem; color: var(--md-fg-muted); align-self: center; margin-right: 0.5rem;">
                "CULTURAL HUD:"
            </span>
            <button class="btn-vault" on:click=move |_| set_context("canonical")> "Canonical" </button>
            <button class="btn-vault" on:click=move |_| set_context("indigenous")> "Indigenous" </button>
            <button class="btn-vault" on:click=move |_| set_context("community")> "Community" </button>
        </div>
    }
}

#[component]
fn VaultPage() -> impl IntoView {
    let ctx = use_personal();
    let cultural = use_cultural();
    let symbols = cultural.symbols;

    let hc = mycelix_leptos_core::holochain_provider::use_holochain();
    let runtime_mode = ctx.runtime_mode;
    let loading = ctx.loading;
    let identity_state = ctx.identity_state;
    let wallet_state = ctx.wallet_state;
    let health_state = ctx.health_state;
    let preferences_state = ctx.preferences_state;
    let activity_state = ctx.activity_state;
    let connection_status = hc.status;
    let signer_ready = hc.zome_call_signing_ready;

    let active_consents = Memo::new(move |_| {
        ctx.consents
            .get()
            .into_iter()
            .filter(|grant| grant.active)
            .count()
    });
    let latest_activity_at = Memo::new(move |_| {
        ctx.activity
            .get()
            .into_iter()
            .map(|item| item.created_at)
            .max()
    });
    let activity_feed = move || {
        ctx.activity
            .get()
            .into_iter()
            .map(|item| ActivityFeedItem {
                id: item.id,
                domain_label: item.domain,
                description: format!("{}: {}", item.title, item.detail),
                emphasis_class: item.success.map(|ok| {
                    if ok {
                        "activity-feed-success".to_string()
                    } else {
                        "activity-feed-warning".to_string()
                    }
                }),
            })
            .collect::<Vec<_>>()
    };

    view! {
        <div class="vault-page">
            <PageHeader
                eyebrow="Sovereign Vault"
                title=move || format!("Private posture, proof posture, and {} posture in one place.", symbols.get().hearth_alias.to_lowercase())
                summary=move || format!("{}. {}", ctx.status_note.get(), symbols.get().orientation)
            />

            <CulturalReskinSelector />

            <div style="display: flex; gap: 0.75rem; align-items: center; flex-wrap: wrap; margin-bottom: 1rem;">
                {move || {
                    latest_activity_at.get().map(|timestamp| {
                        let level = freshness_from_micros(timestamp);
                        let detail = format!("Activity {}", format_relative_micros(timestamp));
                        view! { <FreshnessBadge level detail /> }.into_any()
                    }).unwrap_or_else(|| {
                        view! { <FreshnessBadge level=FreshnessLevel::Unknown detail="No live activity yet" /> }.into_any()
                    })
                }}
            </div>

            {move || {
                if runtime_mode.is_demo() {
                    return view! {
                        <AvailabilityState
                            kind=AvailabilityStateKind::Mock
                            title="Explicit Demo Vault"
                            description="Personal is intentionally running with illustrative fixtures. These records are not conductor-backed evidence."
                            action={None}
                        />
                    }.into_any();
                }

                if loading.get() {
                    return view! {
                        <AvailabilityState
                            kind=AvailabilityStateKind::Degraded
                            title="Reconciling Live Personal State"
                            description="Personal is loading typed view endpoints. Demo records cannot enter this Live session."
                            action={None}
                        />
                    }.into_any();
                }

                if connection_status.get() != ConnectionStatus::Connected {
                    return view! {
                        <AvailabilityState
                            kind=AvailabilityStateKind::Unavailable
                            title="Live Conductor Unavailable"
                            description="Personal remains in Live mode. Connection failure does not fall back to demo data."
                            action={None}
                        />
                    }.into_any();
                }

                if !signer_ready.get() {
                    return view! {
                        <AvailabilityState
                            kind=AvailabilityStateKind::Locked
                            title="Signer Required"
                            description="The conductor is connected, but Personal cannot make authorized zome calls until a browser signer is available."
                            action={None}
                        />
                    }.into_any();
                }

                let states = [
                    identity_state.get(),
                    wallet_state.get(),
                    health_state.get(),
                    preferences_state.get(),
                    activity_state.get(),
                ];

                if states.iter().any(|state| {
                    matches!(
                        state,
                        PersonalSourceState::Degraded | PersonalSourceState::Unavailable
                    )
                }) {
                    view! {
                        <AvailabilityState
                            kind=AvailabilityStateKind::Degraded
                            title="Partial Live Personal State"
                            description="One or more Personal sources could not be established. Available source data remains visible without substituting fixtures for missing sources."
                            action={None}
                        />
                    }.into_any()
                } else if states.iter().any(|state| {
                    matches!(
                        state,
                        PersonalSourceState::AwaitingLive | PersonalSourceState::LoadingLive
                    )
                }) {
                    view! {
                        <AvailabilityState
                            kind=AvailabilityStateKind::Degraded
                            title="Awaiting Live Personal Sources"
                            description="The runtime is ready, but one or more Personal source queries have not completed yet."
                            action={None}
                        />
                    }.into_any()
                } else if states
                    .iter()
                    .all(|state| *state == PersonalSourceState::Empty)
                {
                    view! {
                        <AvailabilityState
                            kind=AvailabilityStateKind::Empty
                            title="Live Personal Vault Is Empty"
                            description="Authoritative Personal view queries completed without records. No illustrative records are inserted into this Live session."
                            action={Some(view! {
                                <A href="/identity" attr:class="btn btn-primary">"Create Profile"</A>
                            }.into_any())}
                        />
                    }.into_any()
                } else {
                    view! { <></> }.into_any()
                }
            }}

            <div class="hero-strip">
                <div class="hero-panel">
                    <span class="hero-kicker">"Vault state"</span>
                    <h2>"Source-backed when Live, illustrative only when Demo"</h2>
                    <p>
                        "Personal keeps runtime provenance explicit. Empty and unavailable source states remain visible instead of being replaced by example records."
                    </p>
                    <div class="hero-actions">
                        <A href="/wallet" attr:class="btn btn-primary">"Open wallet"</A>
                        <A href="/preferences" attr:class="btn">"Review preferences"</A>
                    </div>
                </div>
                <div class="hero-panel hero-panel-accent">
                    <span class="hero-kicker">"Mutation truth"</span>
                    <p>
                        "A source-chain write receipt remains distinct from whether its source refresh published into the current Personal reconciliation epoch."
                    </p>
                </div>
            </div>

            <ConstellationTelemetry />

            <div class="stats-grid">
                <VaultStat label="Credentials" value=move || ctx.credentials.get().len().to_string() />
                <VaultStat label=move || format!("Active {} consents", symbols.get().hearth_alias.to_lowercase()) value=move || active_consents.get().to_string() />
                <VaultStat label="Health records" value=move || ctx.health_record_count.get().to_string() />
                <VaultStat label="Runtime" value=move || runtime_mode.label().to_string() />
            </div>

            <div class="vault-columns">
                <section class="vault-card">
                    <SectionTitle title="Recent activity" />
                    <ActivityFeed items=activity_feed() />
                </section>

                <section class="vault-card">
                    <SectionTitle title="Launch paths" />
                    <div class="link-stack">
                        <a class="launch-link" href="/identity">"Identity posture"</a>
                        <a class="launch-link" href="/wallet">"Credential wallet"</a>
                        <a class="launch-link" href="/health">"Health summary"</a>
                        <a class="launch-link" href="/preferences">"Sharing controls"</a>
                    </div>
                </section>
            </div>
        </div>
    }
}

#[component]
fn IdentityPage() -> impl IntoView {
    let ctx = use_personal();
    let ledger = use_mutation_ledger();
    let location = use_location();
    let hc = mycelix_leptos_core::holochain_provider::use_holochain();
    let toasts = use_toasts();
    let ctx_for_save = ctx.clone();

    let save_profile = move |ev: SubmitEvent| {
        ev.prevent_default();
        let input = ConditionalProfileMutationInputView {
            expected_action_hash: ctx_for_save.profile_action_hash.get_untracked(),
            profile: ctx_for_save.draft_profile.get(),
        };

        let hc = hc.clone();
        let toasts = toasts.clone();
        let ctx = ctx_for_save.clone();
        let ledger = ledger;
        spawn_local(async move {
            match hc
                .call_zome_default::<_, ConditionalMutationResultView>(
                    "identity_vault",
                    "set_profile_view_if_current",
                    &input,
                )
                .await
            {
                Ok(ConditionalMutationResultView::Committed { receipt }) => {
                    let action_hash = receipt.action_hash;
                    ledger.record_committed(PersonalMutationTarget::Profile, action_hash.clone());
                    let outcome = refresh_identity_after_mutation(ctx.clone(), hc.clone()).await;
                    ledger.record_refresh_outcome(&action_hash, outcome);
                    if let MutationRefreshOutcome::Published { epoch } = outcome {
                        toasts.push(
                            format!(
                                "Profile write committed; Identity refresh published in Personal epoch {epoch}, but action observation remains separately evidenced."
                            ),
                            ToastKind::Success,
                        );
                    }
                }
                Ok(ConditionalMutationResultView::Conflict { current_action_hash }) => {
                    let current = current_action_hash
                        .map(|hash| format!("current action {hash}"))
                        .unwrap_or_else(|| "current authoritative Profile is empty".into());
                    toasts.push(
                        format!(
                            "Profile was not written because the displayed baseline is no longer current ({current}). Your local draft was retained; reconcile Personal state before retrying."
                        ),
                        ToastKind::Error,
                    );
                }
                Err(err) => {
                    toasts.push(
                        format!("Profile write failed; local draft retained: {err}"),
                        ToastKind::Error,
                    );
                }
            }
        });
    };

    view! {
        <div class="stack-page">
            <PageHeader
                eyebrow="Identity Vault"
                title="Profile posture and key posture"
                summary={
                    if location.pathname.get().contains("/profile") {
                        "Profile route aliases into the identity vault until deeper Personal profile pages are split."
                            .to_string()
                    } else {
                        "Profile drafts stay local until a conditional source-chain write succeeds against the exact Profile action that produced the displayed baseline."
                            .to_string()
                    }
                }
            />

            <div class="two-up">
                <section class="vault-card">
                    <SectionTitle title="Profile" />
                    <form on:submit=save_profile class="profile-form">
                        <label class="field-block">
                            <span>"Display name"</span>
                            <input
                                class="form-input"
                                prop:value=move || ctx.draft_profile.get().display_name
                                on:input=move |ev| {
                                    let value = event_target_value(&ev);
                                    ctx.draft_profile.update(|profile| profile.display_name = value);
                                }
                            />
                        </label>
                        <label class="field-block">
                            <span>"Bio"</span>
                            <textarea
                                class="form-textarea"
                                prop:value=move || ctx.draft_profile.get().bio.unwrap_or_default()
                                on:input=move |ev| {
                                    let value = event_target_value(&ev);
                                    ctx.draft_profile.update(|profile| {
                                        profile.bio = if value.trim().is_empty() { None } else { Some(value) };
                                    });
                                }
                            />
                        </label>
                        <div class="form-actions">
                            <button class="btn btn-primary" type="submit">"Commit profile draft"</button>
                            <a class="btn" href="/wallet">"Open wallet"</a>
                        </div>
                    </form>
                    <MutationPendingNotice target=PersonalMutationTarget::Profile />
                </section>

                <section class="vault-card">
                    <SectionTitle title="Key posture" />
                    <div class="key-list">
                        <For
                            each=move || ctx.keys.get()
                            key=|key| format!("{}-{}", key.label, key.purpose)
                            children=move |key| view! { <KeyCard key_data=key /> }
                        />
                    </div>
                </section>
            </div>
        </div>
    }
}

#[component]
fn HealthPage() -> impl IntoView {
    let ctx = use_personal();
    let ledger = use_mutation_ledger();
    let hc = mycelix_leptos_core::holochain_provider::use_holochain();
    let toasts = use_toasts();
    let consent_grantee = RwSignal::new(String::new());
    let consent_types = RwSignal::new("allergy, medication".to_string());
    let ctx_for_consent = ctx.clone();

    let create_consent = move |ev: SubmitEvent| {
        ev.prevent_default();
        let grantee = consent_grantee.get();
        let record_types: Vec<String> = consent_types
            .get()
            .split(',')
            .map(str::trim)
            .filter(|s| !s.is_empty())
            .map(ToString::to_string)
            .collect();

        let input = personal_leptos_types::ConsentGrantInputView {
            grantee,
            record_types,
            expires_at: None,
            active: true,
        };

        let hc = hc.clone();
        let toasts = toasts.clone();
        let ctx = ctx_for_consent.clone();
        let ledger = ledger;
        let consent_grantee_signal = consent_grantee;
        let consent_types_signal = consent_types;
        spawn_local(async move {
            match hc
                .call_zome_default::<_, MutationReceiptView>(
                    "health_vault",
                    "grant_consent_view",
                    &input,
                )
                .await
            {
                Ok(receipt) => {
                    let action_hash = receipt.action_hash;
                    ledger.record_committed(
                        PersonalMutationTarget::HealthConsent,
                        action_hash.clone(),
                    );
                    consent_grantee_signal.set(String::new());
                    consent_types_signal.set("allergy, medication".to_string());
                    let outcome = refresh_health_after_mutation(ctx.clone(), hc.clone()).await;
                    ledger.record_refresh_outcome(&action_hash, outcome);

                    if let MutationRefreshOutcome::Published { epoch } = outcome {
                        let observed = ctx
                            .consents
                            .get_untracked()
                            .iter()
                            .any(|consent| consent.hash == action_hash);
                        if observed {
                            ledger.mark_observed(&action_hash);
                            toasts.push(
                                format!(
                                    "Consent write committed and observed in the Health read model for Personal epoch {epoch}."
                                ),
                                ToastKind::Success,
                            );
                        }
                    }
                }
                Err(err) => toasts.push(format!("Consent grant failed: {err}"), ToastKind::Error),
            }
        });
    };

    view! {
        <div class="stack-page">
            <PageHeader
                eyebrow="Health Vault"
                title="Private summary without replacing the Health domain app"
                summary="Personal owns posture and disclosure. Health owns deeper medical workflows.".to_string()
            />

            <div class="two-up">
                <section class="vault-card">
                    <SectionTitle title="Recent biometrics" />
                    <div class="biometric-list">
                        <For
                            each=move || ctx.biometrics.get()
                            key=|item| item.hash.clone()
                            children=move |item| view! {
                                <div class="metric-row">
                                    <span class="metric-name">{item.metric_type}</span>
                                    <span class="metric-value">{format!("{} {}", item.value, item.unit)}</span>
                                </div>
                            }
                        />
                    </div>
                </section>

                <section class="vault-card">
                    <SectionTitle title="Consent grants" />
                    <form class="profile-form" on:submit=create_consent>
                        <label class="field-block">
                            <span>"Grantee key (base64 raw39)"</span>
                            <input
                                class="form-input"
                                prop:value=move || consent_grantee.get()
                                on:input=move |ev| consent_grantee.set(event_target_value(&ev))
                            />
                        </label>
                        <label class="field-block">
                            <span>"Record types"</span>
                            <input
                                class="form-input"
                                prop:value=move || consent_types.get()
                                on:input=move |ev| consent_types.set(event_target_value(&ev))
                            />
                        </label>
                        <div class="form-actions">
                            <button class="btn btn-primary" type="submit">"Create Consent"</button>
                        </div>
                    </form>
                    <MutationPendingNotice target=PersonalMutationTarget::HealthConsent />
                    <div class="consent-list">
                        <For
                            each=move || ctx.consents.get()
                            key=|grant| grant.hash.clone()
                            children=move |grant| view! { <ConsentCard grant=grant /> }
                        />
                    </div>
                </section>
            </div>
        </div>
    }
}

#[component]
fn PreferencesPage() -> impl IntoView {
    let ctx = use_personal();

    view! {
        <div class="stack-page">
            <PageHeader
                eyebrow="Preferences"
                title="Sharing posture and disclosure policy"
                summary=ctx.status_note.get_untracked()
            />

            <div class="two-up">
                <section class="vault-card">
                    <SectionTitle title="Domain posture" />
                    <div class="preference-list">
                        <For
                            each=move || ctx.preferences.get()
                            key=|pref| format!("{}-{}", pref.source_cluster, pref.target_cluster)
                            children=move |pref| view! { <PreferenceCard pref=pref /> }
                        />
                    </div>
                </section>

                <section class="vault-card">
                    <SectionTitle title="Change log" />
                    <div class="preference-list">
                        <For
                            each=move || ctx.preference_log.get()
                            key=|log| format!("{}-{}-{}", log.source_cluster, log.target_cluster, log.changed_at)
                            children=move |log| view! {
                                <div class="mini-card">
                                    <div class="mini-card-header">
                                        <strong>{format!("{} -> {}", log.source_cluster, log.target_cluster)}</strong>
                                        <span class="status-pill" class:status-pill-active=log.now_allowed>
                                            {if log.now_allowed { "Allowed" } else { "Blocked" }}
                                        </span>
                                    </div>
                                    <p class="mini-card-meta">
                                        {format!("Was allowed: {}", if log.was_allowed { "yes" } else { "no" })}
                                    </p>
                                </div>
                            }
                        />
                    </div>
                </section>
            </div>
        </div>
    }
}

#[component]
fn PreferenceCard(pref: personal_leptos_types::DataSharingPreferenceView) -> impl IntoView {
    let ctx = use_personal();
    let ledger = use_mutation_ledger();
    let hc = mycelix_leptos_core::holochain_provider::use_holochain();
    let toasts = use_toasts();
    let local_pref = RwSignal::new(pref);
    let blocked_zomes_text = RwSignal::new(local_pref.get_untracked().blocked_zomes.join(", "));
    let mutation_target = PersonalMutationTarget::preference(
        local_pref.get_untracked().source_cluster.clone(),
        local_pref.get_untracked().target_cluster.clone(),
    );
    let toggle_target = mutation_target.clone();
    let save_target = mutation_target.clone();
    let view_target = mutation_target;
    let toggle_ctx = ctx.clone();
    let toggle_hc = hc.clone();
    let toggle_toasts = toasts.clone();
    let save_ctx = ctx.clone();
    let save_hc = hc.clone();
    let save_toasts = toasts.clone();

    let toggle = move |_| {
        let previous = local_pref.get();
        let mut next = local_pref.get();
        next.allowed = !next.allowed;
        local_pref.set(next.clone());
        blocked_zomes_text.set(next.blocked_zomes.join(", "));

        let hc = toggle_hc.clone();
        let toasts = toggle_toasts.clone();
        let ctx = toggle_ctx.clone();
        let ledger = ledger;
        let target = toggle_target.clone();
        let local_pref_signal = local_pref;
        let blocked_zomes_signal = blocked_zomes_text;
        spawn_local(async move {
            match hc
                .call_zome_default::<_, MutationReceiptView>(
                    "data_preferences",
                    "set_preference_view",
                    &next,
                )
                .await
            {
                Ok(receipt) => {
                    let action_hash = receipt.action_hash;
                    ledger.record_committed(target, action_hash.clone());
                    let outcome = refresh_preferences_after_mutation(ctx.clone(), hc.clone()).await;
                    ledger.record_refresh_outcome(&action_hash, outcome);
                    if let MutationRefreshOutcome::Published { epoch } = outcome {
                        let state = if next.allowed { "allowed" } else { "blocked" };
                        toasts.push(
                            format!(
                                "{} -> {} committed as {}; Preferences refresh published in Personal epoch {epoch}, but action observation remains unconfirmed.",
                                next.source_cluster, next.target_cluster, state
                            ),
                            ToastKind::Success,
                        );
                    }
                }
                Err(err) => {
                    local_pref_signal.set(previous.clone());
                    blocked_zomes_signal.set(previous.blocked_zomes.join(", "));
                    toasts.push(format!("Preference update failed: {err}"), ToastKind::Error);
                }
            }
        });
    };

    let save_details = move |_| {
        let previous = local_pref.get();
        let mut next = local_pref.get();
        next.blocked_zomes = blocked_zomes_text
            .get()
            .split(',')
            .map(str::trim)
            .filter(|s| !s.is_empty())
            .map(ToString::to_string)
            .collect();
        local_pref.set(next.clone());

        let hc = save_hc.clone();
        let toasts = save_toasts.clone();
        let ctx = save_ctx.clone();
        let ledger = ledger;
        let target = save_target.clone();
        let local_pref_signal = local_pref;
        let blocked_zomes_signal = blocked_zomes_text;
        spawn_local(async move {
            match hc
                .call_zome_default::<_, MutationReceiptView>(
                    "data_preferences",
                    "set_preference_view",
                    &next,
                )
                .await
            {
                Ok(receipt) => {
                    let action_hash = receipt.action_hash;
                    ledger.record_committed(target, action_hash.clone());
                    let outcome = refresh_preferences_after_mutation(ctx.clone(), hc.clone()).await;
                    ledger.record_refresh_outcome(&action_hash, outcome);
                    if let MutationRefreshOutcome::Published { epoch } = outcome {
                        let state = if next.allowed { "allowed" } else { "blocked" };
                        toasts.push(
                            format!(
                                "{} -> {} committed as {}; Preferences refresh published in Personal epoch {epoch}, but action observation remains unconfirmed.",
                                next.source_cluster, next.target_cluster, state
                            ),
                            ToastKind::Success,
                        );
                    }
                }
                Err(err) => {
                    local_pref_signal.set(previous.clone());
                    blocked_zomes_signal.set(previous.blocked_zomes.join(", "));
                    toasts.push(format!("Preference update failed: {err}"), ToastKind::Error);
                }
            }
        });
    };

    view! {
        <article class="mini-card">
            <div class="mini-card-header">
                <strong>{move || format!(
                    "{} -> {}",
                    local_pref.get().source_cluster,
                    local_pref.get().target_cluster
                )}</strong>
                <span class="status-pill" class:status-pill-active=move || local_pref.get().allowed>
                    {move || if local_pref.get().allowed { "Allowed" } else { "Blocked" }}
                </span>
            </div>
            <label class="field-block preference-field">
                <span>"Reason"</span>
                <textarea
                    class="form-textarea preference-textarea"
                    prop:value=move || local_pref.get().reason
                    on:input=move |ev| {
                        let value = event_target_value(&ev);
                        local_pref.update(|pref| pref.reason = value);
                    }
                />
            </label>
            <label class="field-block preference-field">
                <span>"Blocked zomes"</span>
                <input
                    class="form-input"
                    prop:value=move || blocked_zomes_text.get()
                    placeholder="lab_result, claims, records"
                    on:input=move |ev| {
                        blocked_zomes_text.set(event_target_value(&ev));
                    }
                />
            </label>
            <div class="form-actions">
                <button class="btn" on:click=toggle>
                    {move || if local_pref.get().allowed { "Block Flow" } else { "Allow Flow" }}
                </button>
                <button class="btn btn-primary" on:click=save_details>
                    "Save Details"
                </button>
            </div>
            <MutationPendingNotice target=view_target />
        </article>
    }
}

#[component]
fn EmbeddedSatellite(name: &'static str, port: u16) -> impl IntoView {
    let url = format!("http://localhost:{}", port);

    view! {
        <div class="embedded-satellite-container">
            <div class="embedded-header">
                <span class="embedded-title">{format!("{} Satellite", name)}</span>
                <span class="telemetry-badge">"STANDALONE"</span>
            </div>
            <iframe
                src=url
                title=name
                class="embedded-satellite-frame"
                style="width: 100%; height: calc(100vh - 120px); border: none; background: var(--bg-surface);"
            ></iframe>
        </div>
    }
}
