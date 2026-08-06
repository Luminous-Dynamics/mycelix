// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Holochain conductor connection for Mycelix Pulse.
//!
//! Connects via WebSocket and fails closed to Unavailable; Demo is explicit.
//! Runtime identifiers still target the legacy `mycelix_mail` role and app id
//! until a compatibility migration is designed and executed.

use leptos::prelude::*;
use send_wrapper::SendWrapper;
use serde::{Serialize, de::DeserializeOwned};
use std::cell::RefCell;
use std::rc::Rc;
use wasm_bindgen::JsValue;
use wasm_bindgen_futures::spawn_local;

use mycelix_leptos_client::{
    AdminGrantedSigner, BrowserWsTransport, ConnectConfig, HolochainTransport, SigningCredentials,
    decode, encode,
};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ConnectionStatus {
    Disconnected,
    Connecting,
    Connected,
    Demo,
    Unavailable,
}

type TransportCell = SendWrapper<Rc<RefCell<Option<BrowserWsTransport>>>>;

#[derive(Clone)]
pub struct HolochainCtx {
    pub status: ReadSignal<ConnectionStatus>,
    pub last_error: ReadSignal<Option<String>>,
    set_status: WriteSignal<ConnectionStatus>,
    set_last_error: WriteSignal<Option<String>>,
    transport: TransportCell,
}

impl HolochainCtx {
    pub fn is_mock(&self) -> bool {
        self.status.get_untracked() == ConnectionStatus::Demo
    }

    /// Enter demo mode only after an explicit user action.
    pub fn enter_demo(&self) {
        self.transport.borrow_mut().take();
        self.set_last_error.set(None);
        self.set_status.set(ConnectionStatus::Demo);
    }

    /// Register a callback for real-time signals from the conductor.
    pub fn set_signal_handler<F: Fn(Vec<u8>) + 'static>(&self, handler: F) {
        let transport_ref = self.transport.borrow();
        if let Some(transport) = transport_ref.as_ref() {
            transport.set_signal_handler(handler);
        }
    }

    pub async fn call_zome<I: Serialize, O: DeserializeOwned>(
        &self,
        zome: &str,
        fn_name: &str,
        input: &I,
    ) -> Result<O, String> {
        self.call_zome_on_role("main", zome, fn_name, input).await
    }

    /// Call a zome function on a specific role (e.g. "identity" for DID operations).
    pub async fn call_zome_on_role<I: Serialize, O: DeserializeOwned>(
        &self,
        role: &str,
        zome: &str,
        fn_name: &str,
        input: &I,
    ) -> Result<O, String> {
        if self.status.get_untracked() != ConnectionStatus::Connected {
            return Err("Live runtime is not ready for zome calls".to_string());
        }
        if !crate::alpha_scope::role_available(role) {
            return Err(format!(
                "Role {role:?} is not available in the restricted alpha hApp"
            ));
        }
        if role == "main" && !crate::alpha_scope::zome_available(zome) {
            return Err(format!(
                "Zome {zome:?} is not available in the restricted alpha DNA"
            ));
        }

        // Use BrowserWsTransport directly (Rust-native, like Prism)
        let transport_ref = self.transport.borrow();
        let transport = transport_ref
            .as_ref()
            .ok_or_else(|| "Not connected to conductor".to_string())?;

        let payload = encode(input).map_err(|e| format!("Encode error: {e}"))?;
        let response = transport
            .call_zome(role, zome, fn_name, payload)
            .await
            .map_err(|e| format!("Zome call {zome}.{fn_name} failed: {e}"))?;
        decode(&response).map_err(|e| format!("Decode error for {zome}.{fn_name}: {e}"))
    }
}

const LOCAL_CONDUCTOR_URL: &str = "ws://localhost:8888";
const TUNNEL_CONDUCTOR_URL: &str = "wss://mail-conductor.luminousdynamics.io";

/// Fetch auth token from the SPA server's /api/token endpoint.
/// Requests are explicitly marked no-store; credential responses must never enter browser caches.
async fn fetch_auth_token() -> Option<Vec<u8>> {
    // Fetch auth token from SPA server using gloo-net (works reliably in WASM)
    let resp = match gloo_net::http::Request::get("/api/token")
        .header("Cache-Control", "no-store")
        .header("Pragma", "no-cache")
        .send()
        .await
    {
        Ok(r) => r,
        Err(e) => {
            web_sys::console::warn_1(&format!("[Mail] Token fetch error: {e}").into());
            return None;
        }
    };

    if !resp.ok() {
        web_sys::console::warn_1(
            &format!("[Mail] Token gate returned HTTP {}", resp.status()).into(),
        );
        return None;
    }

    let json: serde_json::Value = match resp.json().await {
        Ok(j) => j,
        Err(e) => {
            web_sys::console::warn_1(&format!("[Mail] Token JSON parse error: {e}").into());
            return None;
        }
    };

    let token_b64 = json.get("token")?.as_str()?;
    if token_b64.is_empty() {
        web_sys::console::warn_1(&"[Mail] Token: empty string".into());
        return None;
    }

    // Decode base64 to bytes
    let decoded = web_sys::window()?.atob(token_b64).ok()?;
    let bytes: Vec<u8> = decoded.chars().map(|c| c as u8).collect();
    web_sys::console::log_1(&format!("[Mail] Auth token: {} bytes", bytes.len()).into());
    Some(bytes)
}

fn atob_bytes(window: &web_sys::Window, b64: &str) -> Option<Vec<u8>> {
    let decoded = window.atob(b64).ok()?;
    Some(decoded.chars().map(|c| c as u8).collect())
}

/// Fetch zome-call signing credentials from the SPA server's
/// `/api/signing-credentials` endpoint — a throwaway Ed25519 keypair
/// granted, via `AdminRequest::GrantZomeCallCapability`, permission to sign
/// zome calls on behalf of this connection's cell. See
/// `mycelix_leptos_client::admin_granted_signer` for the full rationale
/// (Holochain 0.6 requires every zome call to carry a real Ed25519
/// signature; a bare browser has no access to the cell's own lair-held key,
/// so a trusted broker issues a scoped, revocable one instead). Returns
/// `None` on any failure — the caller falls back to leaving zome calls
/// unavailable rather than sending an unsigned call.
async fn fetch_signing_credentials() -> Option<SigningCredentials> {
    let resp = match gloo_net::http::Request::get("/api/signing-credentials")
        .header("Cache-Control", "no-store")
        .header("Pragma", "no-cache")
        .send()
        .await
    {
        Ok(r) => r,
        Err(e) => {
            web_sys::console::warn_1(
                &format!("[Mail] Signing-credentials fetch error: {e}").into(),
            );
            return None;
        }
    };

    if !resp.ok() {
        web_sys::console::warn_1(
            &format!(
                "[Mail] Signing-credentials gate returned HTTP {}",
                resp.status()
            )
            .into(),
        );
        return None;
    }

    let json: serde_json::Value = match resp.json().await {
        Ok(j) => j,
        Err(e) => {
            web_sys::console::warn_1(
                &format!("[Mail] Signing-credentials JSON parse error: {e}").into(),
            );
            return None;
        }
    };

    let window = web_sys::window()?;
    let signing_agent_key = atob_bytes(&window, json.get("signingAgentKey")?.as_str()?)?;
    let private_key_vec = atob_bytes(&window, json.get("privateKey")?.as_str()?)?;
    let cap_secret = atob_bytes(&window, json.get("capSecret")?.as_str()?)?;

    let private_key: [u8; 32] = match private_key_vec.try_into() {
        Ok(bytes) => bytes,
        Err(bytes) => {
            web_sys::console::warn_1(
                &format!(
                    "[Mail] Signing-credentials: private key is {} bytes, expected 32",
                    bytes.len()
                )
                .into(),
            );
            return None;
        }
    };

    web_sys::console::log_1(
        &format!(
            "[Mail] Signing credentials acquired ({} byte granted key)",
            signing_agent_key.len()
        )
        .into(),
    );

    Some(SigningCredentials {
        signing_agent_key,
        private_key,
        cap_secret,
    })
}

fn validated_conductor_override(candidate: &str) -> Option<String> {
    let parsed = web_sys::Url::new(candidate).ok()?;
    if !parsed.username().is_empty()
        || !parsed.password().is_empty()
        || !parsed.search().is_empty()
        || !parsed.hash().is_empty()
        || !matches!(parsed.pathname().as_str(), "" | "/")
        || parsed.port().is_empty()
    {
        return None;
    }
    let host = parsed.hostname().to_ascii_lowercase();
    let loopback = matches!(host.as_str(), "localhost" | "127.0.0.1" | "[::1]" | "::1");
    match parsed.protocol().as_str() {
        "ws:" if loopback => Some(candidate.to_string()),
        "wss:" => Some(candidate.to_string()),
        _ => None,
    }
}

fn conductor_url() -> String {
    // Explicit overrides must still obey the secure transport boundary.
    if let Some(url) = web_sys::window().and_then(|w| {
        js_sys::Reflect::get(&w, &JsValue::from_str("__HC_CONDUCTOR_URL"))
            .ok()
            .and_then(|v| v.as_string())
            .and_then(|value| validated_conductor_override(&value))
    }) {
        return url;
    }

    let hostname = web_sys::window()
        .and_then(|w| w.location().hostname().ok())
        .unwrap_or_default();

    if hostname == "localhost" || hostname == "127.0.0.1" || hostname == "::1" {
        return LOCAL_CONDUCTOR_URL.to_string();
    }

    // Remote browsers use the authenticated TLS tunnel. Plaintext LAN
    // WebSockets are intentionally unsupported for the security alpha.
    TUNNEL_CONDUCTOR_URL.to_string()
}

/// Wait for the dynamic token promise (from /api/token), then read the value.
#[component]
pub fn HolochainProvider(children: Children) -> impl IntoView {
    let designated_demo = web_sys::window()
        .and_then(|w| w.location().hostname().ok())
        .is_some_and(|host| crate::mail_context::is_demo_hostname(&host));
    let initial_status = if designated_demo {
        ConnectionStatus::Demo
    } else {
        ConnectionStatus::Connecting
    };
    let (status, set_status) = signal(initial_status);
    let (last_error, set_last_error) = signal(None::<String>);
    let transport: TransportCell = SendWrapper::new(Rc::new(RefCell::new(None)));

    let ctx = HolochainCtx {
        status,
        last_error,
        set_status,
        set_last_error,
        transport: transport.clone(),
    };

    provide_context(ctx);

    let transport_for_connect = transport.clone();
    if designated_demo {
        return children();
    }

    spawn_local(async move {
        let url = conductor_url();
        let token = fetch_auth_token().await;
        let has_token = token.is_some();

        // Remote production conductors require the SPA token gate. A missing
        // token is a failed authentication stage, not a partially-live mode.
        if url.starts_with("wss://") && !has_token {
            let reason = "Authentication token unavailable".to_string();
            web_sys::console::warn_1(&format!("[Mail] {reason}").into());
            set_last_error.set(Some(reason));
            set_status.set(ConnectionStatus::Unavailable);
            return;
        }

        web_sys::console::log_1(
            &format!("[Mail] Connecting to {url} (token={has_token})...").into(),
        );

        let ws_transport = BrowserWsTransport::new();
        let config = ConnectConfig {
            url: url.clone(),
            app_id: "mycelix_mail".to_string(),
            auth_token: token,
            reconnect: None,
            request_timeout_ms: Some(30_000),
        };

        if let Err(error) = ws_transport.connect(config).await {
            let reason = format!("Conductor connection failed: {error:?}");
            web_sys::console::warn_1(&format!("[Mail] {reason}").into());
            set_last_error.set(Some(reason));
            set_status.set(ConnectionStatus::Unavailable);
            return;
        }
        web_sys::console::log_1(&"[Mail] Transport connected; acquiring signer...".into());

        let Some(credentials) = fetch_signing_credentials().await else {
            let reason = "Zome-call signer unavailable".to_string();
            web_sys::console::warn_1(&format!("[Mail] {reason}").into());
            set_last_error.set(Some(reason));
            set_status.set(ConnectionStatus::Unavailable);
            return;
        };
        ws_transport.set_zome_call_signer(AdminGrantedSigner::new(credentials));

        // Prove the full live chain before publishing Connected: transport,
        // app/cell resolution, signer, capability grant, zome dispatch, and
        // response decoding. A missing profile legitimately decodes as null.
        let health_payload = match encode(&()) {
            Ok(payload) => payload,
            Err(error) => {
                let reason = format!("Health-check encoding failed: {error}");
                set_last_error.set(Some(reason));
                set_status.set(ConnectionStatus::Unavailable);
                return;
            }
        };
        let health_response = match ws_transport
            .call_zome("main", "mail_profiles", "get_my_profile", health_payload)
            .await
        {
            Ok(response) => response,
            Err(error) => {
                let reason = format!("Signed health check failed: {error}");
                web_sys::console::warn_1(&format!("[Mail] {reason}").into());
                set_last_error.set(Some(reason));
                set_status.set(ConnectionStatus::Unavailable);
                return;
            }
        };
        if let Err(error) = decode::<serde_json::Value>(&health_response) {
            let reason = format!("Health-check response could not be decoded: {error}");
            web_sys::console::warn_1(&format!("[Mail] {reason}").into());
            set_last_error.set(Some(reason));
            set_status.set(ConnectionStatus::Unavailable);
            return;
        }

        *transport_for_connect.borrow_mut() = Some(ws_transport);
        set_last_error.set(None);
        set_status.set(ConnectionStatus::Connected);
        web_sys::console::log_1(&"[Mail] Live runtime ready.".into());
    });

    children()
}

pub fn use_holochain() -> HolochainCtx {
    expect_context::<HolochainCtx>()
}
