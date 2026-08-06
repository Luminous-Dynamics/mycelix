// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Auto-connecting Holochain provider with configurable failure semantics.
//!
//! Replaces per-app `holochain.rs` files. Configurable via
//! [`HolochainProviderConfig`] for single-role, multi-role, and
//! JS-status-only connection strategies.

use leptos::prelude::*;
use send_wrapper::SendWrapper;
use serde::{Serialize, de::DeserializeOwned};
use std::cell::RefCell;
use std::rc::Rc;
use wasm_bindgen::JsCast;
use wasm_bindgen::JsValue;
use wasm_bindgen_futures::spawn_local;

use mycelix_leptos_client::{
    BrowserWsTransport, ConnectConfig, ConnectionStatus as TransportConnectionStatus,
    HolochainTransport, HostZomeCallSigner, ReconnectConfig, decode, encode,
};

// ---------------------------------------------------------------------------
// Connection status
// ---------------------------------------------------------------------------

/// Connection status for UI display.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ConnectionStatus {
    Disconnected,
    Connecting,
    Connected,
    Reconnecting,
    Mock,
}

impl ConnectionStatus {
    pub fn css_class(&self) -> &'static str {
        match self {
            Self::Disconnected => "status-disconnected",
            Self::Connecting => "status-connecting",
            Self::Connected => "status-connected",
            Self::Reconnecting => "status-reconnecting",
            Self::Mock => "status-mock",
        }
    }

    pub fn label(&self) -> &'static str {
        match self {
            Self::Disconnected => "Disconnected",
            Self::Connecting => "Connecting\u{2026}",
            Self::Connected => "Connected",
            Self::Reconnecting => "Reconnecting\u{2026}",
            Self::Mock => "Mock",
        }
    }
}

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// How the provider connects to the conductor.
#[derive(Clone, Debug)]
pub enum ConnectStrategy {
    /// Auto-connect via BrowserWsTransport, fall back to mock.
    WebSocket,
    /// Auto-connect via BrowserWsTransport and remain disconnected on failure.
    ///
    /// Use this for an explicitly selected Live mode so a failed connection is
    /// never represented as a successful mock-data session.
    WebSocketRequired,
    /// Read `__HC_STATUS` from JS window, no real transport.
    JsStatusOnly,
    /// Always mock, no connection attempt.
    MockOnly,
}

/// Configuration for the auto-connecting Holochain provider.
#[derive(Clone, Debug)]
pub struct HolochainProviderConfig {
    /// hApp identifier (e.g. "praxis", "hearth", "mycelix-unified").
    pub app_id: String,
    /// Default role for single-role apps (e.g. Some("hearth")).
    /// Multi-role apps (governance) set this to None and pass role explicitly.
    pub default_role: Option<String>,
    /// Log prefix for console messages (e.g. "[Hearth]").
    pub log_prefix: &'static str,
    /// Connection strategy.
    pub connect_strategy: ConnectStrategy,
    /// Custom status labels (e.g. Health uses "Local Demo" instead of "Mock").
    pub status_labels: Option<StatusLabels>,
}

/// Custom labels for connection status display.
#[derive(Clone, Debug)]
pub struct StatusLabels {
    pub disconnected: &'static str,
    pub connecting: &'static str,
    pub connected: &'static str,
    pub mock: &'static str,
}

// ---------------------------------------------------------------------------
// Holochain context
// ---------------------------------------------------------------------------

type TransportCell = SendWrapper<Rc<RefCell<Option<BrowserWsTransport>>>>;

/// The Holochain client context shared across the app via Leptos context.
#[derive(Clone)]
pub struct HolochainCtx {
    pub status: ReadSignal<ConnectionStatus>,
    /// Whether the connected browser transport can authorize zome calls.
    ///
    /// This is deliberately separate from `status`: an authenticated
    /// WebSocket and app-info response do not provide signing credentials.
    pub zome_call_signing_ready: ReadSignal<bool>,
    /// Most recent transport, serialization, or zome-call failure.
    pub last_error: ReadSignal<Option<String>>,
    set_status: WriteSignal<ConnectionStatus>,
    set_zome_call_signing_ready: WriteSignal<bool>,
    set_last_error: WriteSignal<Option<String>>,
    transport: TransportCell,
    default_role: Option<String>,
    status_labels: Option<StatusLabels>,
}

impl HolochainCtx {
    fn record_error(&self, message: String) -> String {
        self.set_last_error.set(Some(message.clone()));
        message
    }

    /// Call a zome function on the specified role.
    pub async fn call_zome<I: Serialize, O: DeserializeOwned>(
        &self,
        role: &str,
        zome: &str,
        fn_name: &str,
        input: &I,
    ) -> Result<O, String> {
        let transport = {
            let slot = self.transport.borrow();
            match slot.as_ref() {
                Some(transport) => transport.clone(),
                None => {
                    return Err(self
                        .record_error(format!("No conductor transport: {role}.{zome}.{fn_name}")));
                }
            }
        };

        if !transport.zome_call_signer_available() {
            self.set_zome_call_signing_ready.set(false);
            return Err(self.record_error(format!(
                "Zome call {role}.{zome}.{fn_name} blocked: no authorized browser signer; provide window.{} or install a Rust signer",
                HostZomeCallSigner::GLOBAL_NAME
            )));
        }
        self.set_zome_call_signing_ready.set(true);

        let payload =
            encode(input).map_err(|error| self.record_error(format!("Encode error: {error}")))?;

        let response_bytes = transport
            .call_zome(role, zome, fn_name, payload)
            .await
            .map_err(|error| {
                self.record_error(format!("Zome call {role}.{zome}.{fn_name} failed: {error}"))
            })?;

        let decoded = decode(&response_bytes)
            .map_err(|error| self.record_error(format!("Decode error: {error}")))?;
        self.set_last_error.set(None);
        Ok(decoded)
    }

    /// Call a zome function using the default role.
    ///
    /// # Panics
    /// Panics if no `default_role` was configured.
    pub async fn call_zome_default<I: Serialize, O: DeserializeOwned>(
        &self,
        zome: &str,
        fn_name: &str,
        input: &I,
    ) -> Result<O, String> {
        let role = self
            .default_role
            .as_deref()
            .expect("call_zome_default requires a default_role in HolochainProviderConfig");
        self.call_zome(role, zome, fn_name, input).await
    }

    pub fn is_mock(&self) -> bool {
        self.status.get_untracked() == ConnectionStatus::Mock
    }

    /// Reactive readiness check for resources and view closures.
    pub fn zome_calls_ready(&self) -> bool {
        self.status.get() == ConnectionStatus::Connected && self.zome_call_signing_ready.get()
    }

    /// Non-reactive readiness snapshot for event handlers.
    pub fn zome_calls_ready_untracked(&self) -> bool {
        self.status.get_untracked() == ConnectionStatus::Connected
            && self.zome_call_signing_ready.get_untracked()
    }

    pub fn clear_last_error(&self) {
        self.set_last_error.set(None);
    }

    /// Re-read signer availability from the current browser transport.
    ///
    /// Hosts that inject a signer after the provider connects can call this to
    /// update reactive UI state before enabling a write action.
    pub fn refresh_zome_call_signing_ready(&self) -> bool {
        let ready = self
            .transport
            .borrow()
            .as_ref()
            .is_some_and(BrowserWsTransport::zome_call_signer_available);
        self.set_zome_call_signing_ready.set(ready);
        ready
    }

    /// Return the connected agent pubkey in Holochain's `u...` display form.
    pub fn connected_agent_pub_key_b64(&self) -> Option<String> {
        self.transport
            .borrow()
            .as_ref()
            .and_then(|transport| transport.connected_agent_pub_key_b64())
    }

    /// Return the connected agent DID derived from the conductor cell.
    pub fn connected_agent_did(&self) -> Option<String> {
        self.transport
            .borrow()
            .as_ref()
            .and_then(|transport| transport.connected_agent_did())
    }

    /// Get the display label for the current status.
    pub fn status_label(&self) -> &'static str {
        if self.status.get() == ConnectionStatus::Connected
            && self.transport.borrow().is_some()
            && !self.zome_call_signing_ready.get()
        {
            return "Signer required";
        }

        if let Some(ref labels) = self.status_labels {
            match self.status.get() {
                ConnectionStatus::Disconnected => labels.disconnected,
                ConnectionStatus::Connecting => labels.connecting,
                ConnectionStatus::Connected => labels.connected,
                ConnectionStatus::Reconnecting => labels.connecting,
                ConnectionStatus::Mock => labels.mock,
            }
        } else {
            self.status.get().label()
        }
    }

    /// CSS class that keeps a connected-but-unsigned browser session out of
    /// the success color used for fully ready transports.
    pub fn status_css_class(&self) -> &'static str {
        if self.status.get() == ConnectionStatus::Connected
            && self.transport.borrow().is_some()
            && !self.zome_call_signing_ready.get()
        {
            "status-disconnected"
        } else {
            self.status.get().css_class()
        }
    }
}

// ---------------------------------------------------------------------------
// Provider component
// ---------------------------------------------------------------------------

const DEFAULT_CONDUCTOR_URL: &str = "ws://localhost:8888";

fn conductor_url() -> String {
    let Some(window) = web_sys::window() else {
        return DEFAULT_CONDUCTOR_URL.to_string();
    };

    if let Ok(value) = js_sys::Reflect::get(&window, &"__HC_CONDUCTOR_URL".into()) {
        if let Some(url) = value.as_string().filter(|url| !url.trim().is_empty()) {
            return url;
        }
    }

    launcher_environment(&window)
        .and_then(|environment| {
            js_sys::Reflect::get(&environment, &"APP_INTERFACE_PORT".into())
                .ok()
                .and_then(|port| port.as_f64())
                .filter(|port| port.fract() == 0.0 && (1.0..=65_535.0).contains(port))
        })
        .map(|port| format!("ws://localhost:{}", port as u16))
        .unwrap_or_else(|| DEFAULT_CONDUCTOR_URL.to_string())
}

fn launcher_environment(window: &web_sys::Window) -> Option<JsValue> {
    js_sys::Reflect::get(window, &"__HC_LAUNCHER_ENV__".into())
        .ok()
        .filter(|value| !value.is_null() && !value.is_undefined())
}

fn js_uint8_array(value: &JsValue) -> Option<Vec<u8>> {
    if !value.is_instance_of::<js_sys::Uint8Array>() {
        return None;
    }
    let array = js_sys::Uint8Array::new(value);
    let mut bytes = vec![0; array.length() as usize];
    array.copy_to(&mut bytes);
    Some(bytes)
}

fn auth_token() -> Option<Vec<u8>> {
    let window = web_sys::window()?;

    if let Ok(value) = js_sys::Reflect::get(&window, &"__HC_AUTH_TOKEN".into()) {
        if let Some(token) = js_uint8_array(&value) {
            return Some(token);
        }
    }

    launcher_environment(&window).and_then(|environment| {
        js_sys::Reflect::get(&environment, &"APP_INTERFACE_TOKEN".into())
            .ok()
            .and_then(|value| js_uint8_array(&value))
    })
}

fn read_js_conductor_status() -> ConnectionStatus {
    let Some(window) = web_sys::window() else {
        return ConnectionStatus::Mock;
    };
    let val = js_sys::Reflect::get(&window, &JsValue::from_str("__HC_STATUS"))
        .ok()
        .and_then(|v| v.as_string())
        .unwrap_or_else(|| "mock".to_string());
    match val.as_str() {
        "connected" => ConnectionStatus::Connected,
        "connecting" => ConnectionStatus::Connecting,
        "disconnected" => ConnectionStatus::Disconnected,
        _ => ConnectionStatus::Mock,
    }
}

/// Auto-connecting Holochain provider.
///
/// Provides [`HolochainCtx`] via Leptos context. On mount, connects to the
/// conductor using the configured strategy. Mock fallback occurs only for
/// [`ConnectStrategy::WebSocket`]; required connections remain disconnected.
#[component]
pub fn HolochainProviderAuto(config: HolochainProviderConfig, children: Children) -> impl IntoView {
    let initial_status = match &config.connect_strategy {
        ConnectStrategy::WebSocket | ConnectStrategy::WebSocketRequired => {
            ConnectionStatus::Connecting
        }
        ConnectStrategy::JsStatusOnly => read_js_conductor_status(),
        ConnectStrategy::MockOnly => ConnectionStatus::Mock,
    };

    let (status, set_status) = signal(initial_status);
    let (zome_call_signing_ready, set_zome_call_signing_ready) = signal(false);
    let (last_error, set_last_error) = signal(None::<String>);
    // Distinguishes an ACCIDENTAL degrade (a Live strategy that failed to connect
    // and fell back to mock data) from a DELIBERATE demo (ConnectStrategy::MockOnly).
    // Only the former warrants shouting at the user -- see the banner below.
    let (degraded_to_mock, set_degraded_to_mock) = signal(false);
    let transport: TransportCell = SendWrapper::new(Rc::new(RefCell::new(None)));

    let ctx = HolochainCtx {
        status,
        zome_call_signing_ready,
        last_error,
        set_status,
        set_zome_call_signing_ready,
        set_last_error,
        transport: transport.clone(),
        default_role: config.default_role.clone(),
        status_labels: config.status_labels.clone(),
    };

    provide_context(ctx);

    match config.connect_strategy {
        strategy @ (ConnectStrategy::WebSocket | ConnectStrategy::WebSocketRequired) => {
            let transport_for_connect = transport.clone();
            let log_prefix = config.log_prefix;
            let app_id = config.app_id.clone();
            let fallback_to_mock = matches!(strategy, ConnectStrategy::WebSocket);

            spawn_local(async move {
                let url = conductor_url();
                let token = auth_token();
                web_sys::console::log_1(
                    &format!("{log_prefix} Connecting to conductor at {url}\u{2026}").into(),
                );

                let ws_transport = BrowserWsTransport::new();
                *transport_for_connect.borrow_mut() = Some(ws_transport.clone());
                ws_transport.set_status_handler(move |transport_status| {
                    match transport_status {
                        TransportConnectionStatus::Disconnected => {
                            set_zome_call_signing_ready.set(false);
                            set_status.set(ConnectionStatus::Disconnected);
                        }
                        TransportConnectionStatus::Connecting => {
                            set_zome_call_signing_ready.set(false);
                            set_status.set(ConnectionStatus::Connecting);
                        }
                        TransportConnectionStatus::Connected => {
                            let signer_ready = HostZomeCallSigner::is_available();
                            set_zome_call_signing_ready.set(signer_ready);
                            if signer_ready {
                                set_last_error.set(None);
                            } else {
                                set_last_error.set(Some(format!(
                                    "{log_prefix} Conductor transport connected, but zome-call signing is unavailable. Provide window.{} before using Live actions.",
                                    HostZomeCallSigner::GLOBAL_NAME
                                )));
                            }
                            set_status.set(ConnectionStatus::Connected);
                        }
                        TransportConnectionStatus::Reconnecting {
                            attempt,
                            max_attempts,
                        } => {
                            set_zome_call_signing_ready.set(false);
                            set_last_error.set(Some(format!(
                                "{log_prefix} Conductor connection interrupted; reconnecting ({attempt}/{max_attempts})."
                            )));
                            set_status.set(ConnectionStatus::Reconnecting);
                        }
                        TransportConnectionStatus::Error(error) => {
                            set_zome_call_signing_ready.set(false);
                            set_last_error.set(Some(format!(
                                "{log_prefix} Conductor transport error: {error}"
                            )));
                            set_status.set(ConnectionStatus::Disconnected);
                        }
                    }
                });
                let connect_config = ConnectConfig {
                    url,
                    app_id,
                    auth_token: token,
                    reconnect: Some(ReconnectConfig::default()),
                    request_timeout_ms: Some(30_000),
                };

                match ws_transport.connect(connect_config).await {
                    Ok(()) => {
                        let signer_ready = ws_transport.zome_call_signer_available();
                        set_zome_call_signing_ready.set(signer_ready);
                        if signer_ready {
                            web_sys::console::log_1(
                                &format!("{log_prefix} Connected with authorized signing.").into(),
                            );
                            set_last_error.set(None);
                        } else {
                            let message = format!(
                                "{log_prefix} Conductor transport connected, but zome-call signing is unavailable. Provide window.{} before using Live actions.",
                                HostZomeCallSigner::GLOBAL_NAME
                            );
                            web_sys::console::warn_1(&JsValue::from_str(&message));
                            set_last_error.set(Some(message));
                        }
                        set_status.set(ConnectionStatus::Connected);
                    }
                    Err(e) => {
                        let transport_status = ws_transport.status();
                        let retrying = matches!(
                            transport_status,
                            TransportConnectionStatus::Reconnecting { .. }
                        );
                        let message = if fallback_to_mock {
                            ws_transport.disconnect();
                            *transport_for_connect.borrow_mut() = None;
                            format!("{log_prefix} Could not connect: {e}. Running in mock mode.")
                        } else if retrying {
                            format!(
                                "{log_prefix} Initial connection failed: {e}. The bounded Live retry policy remains active."
                            )
                        } else {
                            *transport_for_connect.borrow_mut() = None;
                            format!(
                                "{log_prefix} Could not connect: {e}. Live mode remains disconnected."
                            )
                        };
                        web_sys::console::log_1(&JsValue::from_str(&message));
                        set_zome_call_signing_ready.set(false);
                        set_last_error.set(Some(message));
                        if fallback_to_mock {
                            set_degraded_to_mock.set(true);
                        }
                        set_status.set(if fallback_to_mock {
                            ConnectionStatus::Mock
                        } else if retrying {
                            ConnectionStatus::Reconnecting
                        } else {
                            ConnectionStatus::Disconnected
                        });
                    }
                }
            });
        }
        ConnectStrategy::JsStatusOnly => {
            // For JsStatusOnly with Connecting status, poll after a delay
            if initial_status == ConnectionStatus::Connecting {
                let log_prefix = config.log_prefix;
                leptos::prelude::set_timeout(
                    move || {
                        let resolved = read_js_conductor_status();
                        set_status.set(resolved);
                        web_sys::console::log_1(
                            &format!("{log_prefix} Conductor status: {resolved:?}").into(),
                        );
                    },
                    std::time::Duration::from_millis(3500),
                );
            }
        }
        ConnectStrategy::MockOnly => {
            // Nothing to do
        }
    }

    // Unmissable degraded-mode banner.
    //
    // WHY THIS IS HERE AND NOT LEFT TO EACH APP: the provider already recorded
    // both `ConnectionStatus::Mock` and `last_error` on a failed connection, and
    // it always has. But of the 12 apps using a Live-with-fallback strategy, NINE
    // render neither -- no ConnectionBadge, no last_error -- so a user saw
    // fabricated data presented exactly like real data, with the only trace being
    // a console.log. In a browser tab that is bad; in a packaged desktop app that
    // someone installed, it is lying to them.
    //
    // Making this opt-in would reproduce the bug, since the nine apps that ignore
    // the existing signals would ignore a new one too. It is deliberately NOT
    // configurable: an app that genuinely wants a quiet demo should say so with
    // ConnectStrategy::MockOnly, which never trips this because it never degrades.
    view! {
        <Show when=move || degraded_to_mock.get()>
            <div
                role="alert"
                style="position:fixed;top:0;left:0;right:0;z-index:2147483647;\
                       background:#7f1d1d;color:#fff;padding:0.6rem 1rem;\
                       font:600 14px/1.4 system-ui,sans-serif;text-align:center;\
                       box-shadow:0 2px 8px rgba(0,0,0,.4)"
            >
                "⚠ Not connected to a conductor — showing sample data, not real data."
                {move || last_error.get().map(|e| view! {
                    <div style="font-weight:400;font-size:12px;opacity:.85;margin-top:.25rem">
                        {e}
                    </div>
                })}
            </div>
        </Show>
        {children()}
    }
}

/// Retrieve the [`HolochainCtx`] from the nearest [`HolochainProviderAuto`].
pub fn use_holochain() -> HolochainCtx {
    expect_context::<HolochainCtx>()
}

/// Connection status badge component (CSS-class-based).
#[component]
pub fn ConnectionBadge() -> impl IntoView {
    let ctx = use_holochain();
    let ctx2 = ctx.clone();

    view! {
        <span class=move || format!("connection-badge {}", ctx.status_css_class())>
            <span class="status-dot"></span>
            {move || ctx2.status_label()}
        </span>
    }
}
