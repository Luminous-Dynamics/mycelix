// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Identity context — provides reactive identity data via Leptos signals.
//!
//! Each data domain loads independently via its own `spawn_local` (no waterfall).
//! Demo mode may render fixtures; live mode remains empty until canonical conductor
//! state is successfully loaded. Version signals enable resource invalidation after mutations.

use leptos::prelude::*;
use wasm_bindgen_futures::spawn_local;
use identity_leptos_types::*;

use mycelix_leptos_core::holochain_provider::{use_holochain, HolochainCtx};
use crate::mock_data;

/// Version signals — bumped by actions to trigger data reload.
#[derive(Clone, Copy)]
pub struct IdentityVersions {
    pub did: RwSignal<u32>,
    pub mfa: RwSignal<u32>,
    pub credentials: RwSignal<u32>,
    pub recovery: RwSignal<u32>,
    pub trust: RwSignal<u32>,
}

#[derive(Clone)]
pub struct IdentityCtx {
    pub versions: IdentityVersions,
    pub did_document: RwSignal<Option<DidDocumentView>>,
    pub mfa_state: RwSignal<Option<MfaStateView>>,
    pub recovery_config: RwSignal<Option<RecoveryConfigView>>,
    pub credentials_held: RwSignal<Vec<CredentialView>>,
    pub credentials_issued: RwSignal<Vec<CredentialView>>,
    pub trust_credentials: RwSignal<Vec<TrustCredentialView>>,
    pub reputation: RwSignal<Option<ReputationView>>,
    pub my_name: RwSignal<Option<NameRegistryView>>,
    pub loading: RwSignal<bool>,
    pub last_error: RwSignal<Option<String>>,
}

pub fn provide_identity_context() {
    let versions = IdentityVersions {
        did: RwSignal::new(0),
        mfa: RwSignal::new(0),
        credentials: RwSignal::new(0),
        recovery: RwSignal::new(0),
        trust: RwSignal::new(0),
    };

    // Demo mode may use fixture data; live mode must start empty so test fixtures
    // can never be mistaken for a real sovereign identity.
    let hc = use_holochain();
    let demo = hc.is_mock();
    let ctx = IdentityCtx {
        versions,
        did_document: RwSignal::new(demo.then(mock_data::mock_did_document)),
        mfa_state: RwSignal::new(demo.then(mock_data::mock_mfa_state)),
        recovery_config: RwSignal::new(demo.then(mock_data::mock_recovery_config)),
        credentials_held: RwSignal::new(if demo { mock_data::mock_credentials_held() } else { Vec::new() }),
        credentials_issued: RwSignal::new(if demo { mock_data::mock_credentials_issued() } else { Vec::new() }),
        trust_credentials: RwSignal::new(if demo { mock_data::mock_trust_credentials() } else { Vec::new() }),
        reputation: RwSignal::new(demo.then(mock_data::mock_reputation)),
        my_name: RwSignal::new(demo.then(mock_data::mock_name)),
        loading: RwSignal::new(!demo),
        last_error: RwSignal::new(None),
    };

    provide_context(ctx.clone());

    // Launch independent async loads — each domain fetches concurrently (no waterfall).
    if demo {
        return;
    }
    let ctx_did = ctx.clone();
    spawn_local(async move {
        gloo_timers::future::sleep(std::time::Duration::from_millis(500)).await;
        load_did(ctx_did).await;
    });

    let ctx_mfa = ctx.clone();
    spawn_local(async move {
        gloo_timers::future::sleep(std::time::Duration::from_millis(500)).await;
        load_mfa(ctx_mfa).await;
    });

    let ctx_cred = ctx.clone();
    spawn_local(async move {
        gloo_timers::future::sleep(std::time::Duration::from_millis(500)).await;
        load_credentials(ctx_cred).await;
    });

    let ctx_rep = ctx.clone();
    spawn_local(async move {
        gloo_timers::future::sleep(std::time::Duration::from_millis(500)).await;
        load_reputation(ctx_rep).await;
    });

    let ctx_loading = ctx.clone();
    let hc_loading = hc.clone();
    spawn_local(async move {
        // Never leave the identity UI in an indefinite spinner. The conductor
        // provider has its own bounded connection/health-check lifecycle.
        for _ in 0..60 {
            if hc_loading.status.get_untracked() != mycelix_leptos_core::holochain_provider::ConnectionStatus::Connecting {
                break;
            }
            gloo_timers::future::sleep(std::time::Duration::from_millis(250)).await;
        }
        ctx_loading.loading.set(false);
    });
}

async fn load_did(ctx: IdentityCtx) {
    let hc = use_holochain();
    if hc.is_mock() { return; }

    match hc.call_zome_default::<(), Option<DidDocumentView>>(
        "did_registry",
        "get_my_did_view",
        &(),
    ).await {
        Ok(Some(did)) => {
            web_sys::console::log_1(&"[Identity] Loaded canonical DID view from conductor".into());
            ctx.did_document.set(Some(did));
        }
        Ok(None) => {
            // No DID is a valid first-run state, not a transport failure.
            ctx.did_document.set(None);
            web_sys::console::log_1(&"[Identity] No DID exists for this agent yet".into());
        }
        Err(e) => {
            let message = format!("DID load failed: {e}");
            web_sys::console::warn_1(&message.clone().into());
            ctx.last_error.set(Some(message));
        }
    }
}

async fn load_mfa(ctx: IdentityCtx) {
    let hc = use_holochain();
    if hc.is_mock() { return; }

    match hc.call_zome_default::<String, serde_json::Value>(
        "mfa", "get_mfa_state", &"self".to_string()
    ).await {
        Ok(record) => {
            match serde_json::from_value::<MfaStateView>(record) {
                Ok(mfa) => {
                    web_sys::console::log_1(
                        &format!("[Identity] Loaded MFA: {} factors", mfa.factors.len()).into()
                    );
                    ctx.mfa_state.set(Some(mfa));
                }
                Err(e) => {
                    web_sys::console::warn_1(
                        &format!("[Identity] Failed to parse MFA state: {e}").into()
                    );
                }
            }
        }
        Err(e) => {
            web_sys::console::warn_1(&format!("[Identity] get_mfa_state failed: {e}").into());
        }
    }

    // Also load recovery config (same spawn — minimal latency)
    match hc.call_zome_default::<String, serde_json::Value>(
        "recovery", "get_recovery_config", &"self".to_string()
    ).await {
        Ok(record) => {
            if let Ok(config) = serde_json::from_value::<RecoveryConfigView>(record) {
                ctx.recovery_config.set(Some(config));
            }
        }
        Err(_) => {}
    }
}

async fn load_credentials(ctx: IdentityCtx) {
    let hc = use_holochain();
    if hc.is_mock() { return; }

    match hc.call_zome_default::<(), Vec<serde_json::Value>>(
        "verifiable_credential", "get_my_credentials", &()
    ).await {
        Ok(records) => {
            let creds: Vec<CredentialView> = records.iter().filter_map(|r| {
                match serde_json::from_value::<CredentialView>(r.clone()) {
                    Ok(c) => Some(c),
                    Err(e) => {
                        web_sys::console::warn_1(
                            &format!("[Identity] Failed to parse credential: {e}").into()
                        );
                        None
                    }
                }
            }).collect();
            if !creds.is_empty() {
                ctx.credentials_held.set(creds);
            }
        }
        Err(e) => {
            web_sys::console::warn_1(
                &format!("[Identity] get_my_credentials failed: {e}").into()
            );
        }
    }
}

async fn load_reputation(ctx: IdentityCtx) {
    let hc = use_holochain();
    if hc.is_mock() { return; }

    match hc.call_zome_default::<String, serde_json::Value>(
        "reputation_aggregator", "get_composite_reputation", &"self".to_string()
    ).await {
        Ok(record) => {
            if let Ok(rep) = serde_json::from_value::<ReputationView>(record) {
                web_sys::console::log_1(
                    &format!("[Identity] Loaded reputation: {:.2}", rep.composite_score).into()
                );
                ctx.reputation.set(Some(rep));
            }
        }
        Err(_) => {}
    }
}

pub fn use_identity() -> IdentityCtx {
    expect_context::<IdentityCtx>()
}


/// Create the caller's first sovereign identity on the live identity DNA.
///
/// The zome generates the DID from the conductor-owned agent key; the browser
/// never supplies or invents the authoritative identifier. After creation we
/// re-read the canonical DID document so the UI is driven by DHT state.
pub async fn create_my_did(ctx: IdentityCtx, hc: HolochainCtx) -> Result<(), String> {
    if hc.is_mock() {
        return Err("Demo mode cannot create a live DID".into());
    }
    if hc.status.get_untracked() != mycelix_leptos_core::holochain_provider::ConnectionStatus::Connected {
        return Err("Holochain identity runtime is not connected".into());
    }

    let did = hc
        .call_zome_default::<(), DidDocumentView>("did_registry", "create_did_view", &())
        .await
        .map_err(|e| e.to_string())?;
    ctx.did_document.set(Some(did));
    Ok(())
}
