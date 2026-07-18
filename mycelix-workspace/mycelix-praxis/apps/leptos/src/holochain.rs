// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Holochain conductor context for Praxis Leptos CSR.
//!
//! Wraps [`mycelix_leptos_core::holochain_provider`] with Praxis-specific
//! defaults (app_id, log prefix, default role). All pages use `use_holochain()`
//! to call zome functions.
//!
//! Gains from using the shared provider:
//! - Auto-reconnect with exponential backoff
//! - Cross-role dispatch (call Craft zomes via "craft" role)
//! - Request timeout (30s default)
//! - 8D Sovereign Profile integration

use leptos::prelude::*;

pub use mycelix_leptos_core::holochain_provider::{
    ConnectStrategy, ConnectionBadge, ConnectionStatus, HolochainCtx, HolochainProviderConfig,
    StatusLabels, use_holochain,
};

use crate::mode::{AppMode, use_app_mode};
use crate::persistence;

/// Reactive decision used by data resources before they create their future.
///
/// Reading this inside a `LocalResource` source closure subscribes that
/// resource to mode, transport, and signer changes. In particular, a resource
/// created while Live mode is connecting will run again once zome calls become
/// authorized.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DataSource {
    Demo,
    Local,
    LiveWaiting,
    LiveReady,
}

/// Result of loading a mode-aware resource.
///
/// `WaitingForLive` and `LiveError` are deliberately distinct from a valid
/// empty collection so the UI never presents connection failure as "no data".
#[derive(Clone, Debug)]
pub enum ResourceState<T> {
    WaitingForLive,
    Ready(T),
    LiveError,
}

pub fn tracked_data_source(mode: ReadSignal<AppMode>, hc: &HolochainCtx) -> DataSource {
    match mode.get() {
        AppMode::Demo => DataSource::Demo,
        AppMode::Local => DataSource::Local,
        AppMode::Live if hc.zome_calls_ready() => DataSource::LiveReady,
        AppMode::Live => DataSource::LiveWaiting,
    }
}

#[component]
pub fn LiveResourceStatus(failed: bool) -> impl IntoView {
    let message = if failed {
        "Live data could not be loaded. Review the Live status above for details."
    } else {
        "Waiting for a connected conductor and authorized zome-call signer…"
    };

    view! {
        <div class="data-empty-state" role=if failed { "alert" } else { "status" }>
            {message}
        </div>
    }
}

/// localStorage key for a user-set conductor URL override.
const CONDUCTOR_URL_OVERRIDE_KEY: &str = "praxis_conductor_url_override";

/// Default conductor App WebSocket URL when no override or `index.html`
/// injection is present.
const DEFAULT_CONDUCTOR_URL: &str = "ws://localhost:8888";

/// Apply a saved conductor URL override (if any) to `window.__HC_CONDUCTOR_URL`.
///
/// Must run **before** [`HolochainProvider`] mounts — `HolochainProviderAuto`
/// reads `window.__HC_CONDUCTOR_URL` exactly once, at the start of its
/// connect task. Call this at the top of `main()`, before `mount_to_body`.
///
/// Without this, the only way to point Praxis at a non-default conductor was
/// editing `window.__HC_CONDUCTOR_URL` / `window.__HC_AUTH_TOKEN` directly in
/// `index.html` — there was no in-app way to set or even see it.
pub fn apply_conductor_url_override() {
    let Some(url) = persistence::load::<String>(CONDUCTOR_URL_OVERRIDE_KEY) else {
        return;
    };
    if url.trim().is_empty() {
        return;
    }
    if let Some(window) = web_sys::window() {
        let _ = js_sys::Reflect::set(
            &window,
            &wasm_bindgen::JsValue::from_str("__HC_CONDUCTOR_URL"),
            &wasm_bindgen::JsValue::from_str(&url),
        );
    }
}

fn active_conductor_url() -> String {
    web_sys::window()
        .and_then(|w| {
            js_sys::Reflect::get(&w, &wasm_bindgen::JsValue::from_str("__HC_CONDUCTOR_URL")).ok()
        })
        .and_then(|v| v.as_string())
        .unwrap_or_else(|| DEFAULT_CONDUCTOR_URL.to_string())
}

/// Wraps children with Praxis-configured HolochainProvider.
///
/// Demo and Local modes do not open a transport. Live mode requires the shared
/// ecosystem conductor with app_id "praxis" and never falls back to mock.
#[component]
pub fn HolochainProvider(mode: ReadSignal<AppMode>, children: Children) -> impl IntoView {
    let selected_mode = mode.get_untracked();
    let (connect_strategy, status_labels) = match selected_mode {
        AppMode::Demo => (
            ConnectStrategy::MockOnly,
            StatusLabels {
                disconnected: "Demo unavailable",
                connecting: "Starting demo…",
                connected: "Demo",
                mock: "Demo",
            },
        ),
        AppMode::Local => (
            ConnectStrategy::MockOnly,
            StatusLabels {
                disconnected: "Local unavailable",
                connecting: "Starting local…",
                connected: "Local",
                mock: "Local",
            },
        ),
        AppMode::Live => (
            ConnectStrategy::WebSocketRequired,
            StatusLabels {
                disconnected: "Live unavailable",
                connecting: "Connecting…",
                connected: "Live",
                mock: "Live unavailable",
            },
        ),
    };

    let config = HolochainProviderConfig {
        app_id: "praxis".to_string(),
        default_role: Some("praxis".to_string()),
        log_prefix: "[Praxis]",
        connect_strategy,
        status_labels: Some(status_labels),
    };

    view! {
        <mycelix_leptos_core::holochain_provider::HolochainProviderAuto config=config>
            {children()}
        </mycelix_leptos_core::holochain_provider::HolochainProviderAuto>
    }
}

/// Persistent, mode-aware disclosure of demo data and live failures.
#[component]
pub fn ModeStatusBanner() -> impl IntoView {
    let mode = use_app_mode();
    let hc = use_holochain();
    let hc_status = hc.clone();
    let hc_error = hc.clone();
    let hc_signing = hc.clone();
    let hc_refresh = hc.clone();

    view! {
        {move || match mode.get() {
            AppMode::Demo => view! {
                <div class="mode-banner mode-banner-demo" role="status">
                    <strong>"Demo data"</strong>
                    <span>"Representative examples are shown and are not learner or network records."</span>
                </div>
            }.into_any(),
            AppMode::Local => view! {
                <div class="mode-banner mode-banner-local" role="status">
                    <strong>"Local mode"</strong>
                    <span>"Progress stays in this browser. No conductor connection is attempted."</span>
                </div>
            }.into_any(),
            AppMode::Live => {
                let status = hc_status.status.get();
                let last_error = hc_error.last_error.get();
                let signing_ready = hc_signing.zome_call_signing_ready.get();
                if status == ConnectionStatus::Connected
                    && signing_ready
                    && last_error.is_none()
                {
                    view! {
                        <div class="mode-banner mode-banner-live" role="status">
                            <strong>"Live mode ready"</strong>
                            <span>"Connected to the Praxis conductor with authorized zome-call signing."</span>
                        </div>
                    }.into_any()
                } else if status == ConnectionStatus::Connected && !signing_ready {
                    let detail = last_error.unwrap_or_else(|| {
                        "The conductor transport is connected, but no authorized browser signer is available. Launch Praxis from a compatible Holochain host or install the host signer, then re-check.".to_string()
                    });
                    let hc_refresh = hc_refresh.clone();
                    view! {
                        <div class="mode-banner mode-banner-error" role="alert">
                            <strong>"Live signer unavailable"</strong>
                            <span>{detail}</span>
                            <button
                                class="btn-sm btn-outline"
                                on:click=move |_| {
                                    if hc_refresh.refresh_zome_call_signing_ready() {
                                        hc_refresh.clear_last_error();
                                    }
                                }
                            >
                                "Re-check signer"
                            </button>
                            <ConductorSettingsPanel />
                        </div>
                    }.into_any()
                } else {
                    let detail = last_error.unwrap_or_else(|| match status {
                        ConnectionStatus::Connecting | ConnectionStatus::Reconnecting => {
                            "Connecting to the Praxis conductor…".to_string()
                        }
                        _ => "The Praxis conductor is unavailable.".to_string(),
                    });
                    view! {
                        <div class="mode-banner mode-banner-error" role="alert">
                            <strong>"Live data unavailable"</strong>
                            <span>{detail}</span>
                            <ConductorSettingsPanel />
                        </div>
                    }.into_any()
                }
            }
        }}
    }
}

/// In-app conductor connection settings.
///
/// Shows the live connection status (via [`use_holochain`]) alongside the
/// conductor WebSocket URL currently in effect, and lets the learner
/// override it — persisted to localStorage, applied on next load via
/// [`apply_conductor_url_override`]. This is the first in-app UI for
/// confirming/setting the conductor connection; previously this was only
/// configurable by editing `window.__HC_CONDUCTOR_URL` in `index.html`.
#[component]
pub fn ConductorSettingsPanel() -> impl IntoView {
    let hc = use_holochain();
    let hc_label = hc.clone();
    let hc_signing = hc.clone();

    let (draft_url, set_draft_url) = signal(
        persistence::load::<String>(CONDUCTOR_URL_OVERRIDE_KEY)
            .unwrap_or_else(active_conductor_url),
    );
    let (saved, set_saved) = signal(false);

    let on_save = move |_| {
        let url = draft_url.get();
        if url.trim().is_empty() {
            return;
        }
        persistence::save(CONDUCTOR_URL_OVERRIDE_KEY, &url);
        set_saved.set(true);
    };

    let on_reset = move |_| {
        persistence::remove(CONDUCTOR_URL_OVERRIDE_KEY);
        set_draft_url.set(DEFAULT_CONDUCTOR_URL.to_string());
        set_saved.set(true);
    };

    view! {
        <div
            class="conductor-settings"
            style="margin-top: 0.75rem; padding: 0.75rem; background: var(--surface-low); border: 1px solid var(--border); border-radius: 8px; font-size: 0.85rem"
        >
            <div style="display: flex; justify-content: space-between; align-items: center; margin-bottom: 0.5rem">
                <strong>"Conductor Connection"</strong>
                <span>{move || hc_label.status_label()}</span>
            </div>
            <div style="display: flex; justify-content: space-between; margin-bottom: 0.5rem; color: var(--text-secondary)">
                <span>"Zome-call authorization"</span>
                <span>{move || if hc_signing.zome_call_signing_ready.get() { "Ready" } else { "Signer required" }}</span>
            </div>
            <div style="display: flex; gap: 0.5rem">
                <input
                    type="text"
                    style="flex: 1; padding: 0.4rem; border: 1px solid var(--border); border-radius: 4px; background: var(--surface); color: var(--text)"
                    prop:value=move || draft_url.get()
                    on:input=move |ev| set_draft_url.set(event_target_value(&ev))
                    placeholder=DEFAULT_CONDUCTOR_URL
                />
                <button class="btn-sm btn-primary" on:click=on_save>"Save"</button>
                <button class="btn-sm btn-outline" on:click=on_reset>"Reset"</button>
            </div>
            {move || if saved.get() {
                view! {
                    <p style="margin: 0.5rem 0 0; color: var(--text-secondary)">
                        "Saved. Reload the app to connect using the new conductor URL."
                    </p>
                }.into_any()
            } else {
                view! { <span></span> }.into_any()
            }}
        </div>
    }
}
