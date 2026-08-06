// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Browser WebSocket transport for Holochain conductor communication.
//!
//! Uses `web-sys::WebSocket` to send binary (MessagePack) frames to the
//! Holochain conductor. Request/response correlation is handled via a
//! monotonic request ID and an in-memory pending-request map.
//!
//! # Wire protocol
//!
//! The conductor App API expects:
//! 1. **Authentication** (required by Holochain 0.6) — send the conductor's
//!    fire-and-forget `WireAuthenticate` frame with the token issued by the
//!    admin API.
//! 2. **App info discovery** — send `AppRequest::AppInfo` to get the
//!    `role_name → CellId` mapping.
//! 3. **Zome calls** — send `AppRequest::CallZome` with the resolved `CellId`,
//!    nonce, expiry, and a signature from the configured
//!    [`ZomeCallSigner`](crate::types::ZomeCallSigner) (Rust-side or the
//!    launcher host signer). There is no unsigned fallback.

use crate::error::ClientError;
use crate::transport::HolochainTransport;
use crate::types::{
    AppRequest, AppResponse, AuthenticateRequest, CellId, CellInfoVariant, ConnectConfig,
    ConnectionStatus, IncomingWireMessage, SignedZomeCall, WireAuthenticate, WireRequest,
    ZomeCallSigner, ZomeCallToSign, decode_tagged,
};
use base64::Engine as _;
use base64::engine::general_purpose::URL_SAFE_NO_PAD;

use std::cell::RefCell;
use std::collections::HashMap;
use std::future::Future;
use std::pin::Pin;
use std::rc::Rc;

use js_sys::{ArrayBuffer, Uint8Array};
use wasm_bindgen::JsCast;
use wasm_bindgen::prelude::*;
use wasm_bindgen_futures::JsFuture;
use web_sys::{MessageEvent, WebSocket};

// ---------------------------------------------------------------------------
// Pending request tracking
// ---------------------------------------------------------------------------

type PendingMap = HashMap<u64, PendingRequest>;
type StatusHandler = Rc<dyn Fn(ConnectionStatus)>;

struct PendingRequest {
    /// Waker to resolve the future when the response arrives.
    resolve: Box<dyn FnOnce(Result<Vec<u8>, ClientError>)>,
}

/// Adapter for the official Holochain launcher host-signer convention.
///
/// A launcher or native shell may inject
/// `window.__HC_ZOME_CALL_SIGNER__.signZomeCall(request)`. The callback must
/// return a promise resolving to `{ bytes: Uint8Array, signature: Uint8Array }`.
#[derive(Clone, Copy, Debug, Default)]
pub struct HostZomeCallSigner;

impl HostZomeCallSigner {
    pub const GLOBAL_NAME: &'static str = "__HC_ZOME_CALL_SIGNER__";

    pub fn is_available() -> bool {
        let Some(window) = web_sys::window() else {
            return false;
        };
        let Ok(signer) = js_sys::Reflect::get(&window, &Self::GLOBAL_NAME.into()) else {
            return false;
        };
        if signer.is_null() || signer.is_undefined() {
            return false;
        }
        js_sys::Reflect::get(&signer, &"signZomeCall".into())
            .ok()
            .and_then(|value| value.dyn_into::<js_sys::Function>().ok())
            .is_some()
    }
}

impl ZomeCallSigner for HostZomeCallSigner {
    fn sign_zome_call(
        &self,
        request: ZomeCallToSign,
    ) -> Pin<Box<dyn Future<Output = Result<SignedZomeCall, ClientError>>>> {
        Box::pin(async move {
            let window = web_sys::window()
                .ok_or_else(|| ClientError::SigningUnavailable("window is not available".into()))?;
            let signer =
                js_sys::Reflect::get(&window, &Self::GLOBAL_NAME.into()).map_err(|error| {
                    ClientError::SigningUnavailable(format!(
                        "host signer lookup failed: {}",
                        js_error_message(&error)
                    ))
                })?;
            let sign = js_sys::Reflect::get(&signer, &"signZomeCall".into())
                .map_err(|error| {
                    ClientError::SigningUnavailable(format!(
                        "host signZomeCall lookup failed: {}",
                        js_error_message(&error)
                    ))
                })?
                .dyn_into::<js_sys::Function>()
                .map_err(|_| {
                    ClientError::SigningUnavailable(
                        "window.__HC_ZOME_CALL_SIGNER__.signZomeCall is not callable".into(),
                    )
                })?;

            let request_js = zome_call_to_js(request)?;
            let returned = sign.call1(&signer, &request_js).map_err(|error| {
                ClientError::SigningUnavailable(format!(
                    "host signer rejected the call: {}",
                    js_error_message(&error)
                ))
            })?;
            let signed = JsFuture::from(js_sys::Promise::resolve(&returned))
                .await
                .map_err(|error| {
                    ClientError::SigningUnavailable(format!(
                        "host signer promise rejected: {}",
                        js_error_message(&error)
                    ))
                })?;

            let bytes = required_uint8_array(&signed, "bytes")?;
            let signature = required_uint8_array(&signed, "signature")?;
            SignedZomeCall::new(bytes, signature)
        })
    }
}

// ---------------------------------------------------------------------------
// Internal shared state
// ---------------------------------------------------------------------------

struct Inner {
    ws: Option<WebSocket>,
    status: ConnectionStatus,
    next_id: u64,
    pending: PendingMap,
    /// Role name → CellId mapping, populated by app_info after connect.
    cell_map: HashMap<String, CellId>,
    /// Agent public key from app info, used only for identity display.
    agent_pub_key: Option<Vec<u8>>,
    /// Optional Rust-side signer. If absent, the official host signer is used.
    zome_call_signer: Option<Rc<dyn ZomeCallSigner>>,
    /// Closures that must be kept alive for the WebSocket callbacks.
    /// Stored as JsValue to avoid type-parameter complexity.
    _callbacks: Vec<JsValue>,
    /// Signal handler — called when the conductor sends a signal (not a response).
    signal_handler: Option<Box<dyn Fn(Vec<u8>)>>,
    /// Observer for transport lifecycle changes.
    status_handler: Option<StatusHandler>,
    /// Stored connect config for reconnection attempts.
    connect_config: Option<ConnectConfig>,
    /// Current reconnect attempt counter (0 = not reconnecting).
    reconnect_attempt: u32,
    /// Prevent duplicate close/error paths from scheduling parallel retries.
    reconnect_scheduled: bool,
    /// Invalidates delayed retry callbacks after an explicit disconnect.
    reconnect_epoch: u64,
}

impl Default for Inner {
    fn default() -> Self {
        Self {
            ws: None,
            status: ConnectionStatus::Disconnected,
            next_id: 1,
            pending: HashMap::new(),
            cell_map: HashMap::new(),
            agent_pub_key: None,
            zome_call_signer: None,
            _callbacks: Vec::new(),
            signal_handler: None,
            status_handler: None,
            connect_config: None,
            reconnect_attempt: 0,
            reconnect_scheduled: false,
            reconnect_epoch: 0,
        }
    }
}

// ---------------------------------------------------------------------------
// BrowserWsTransport
// ---------------------------------------------------------------------------

/// WebSocket-based transport for browser WASM targets.
///
/// This transport opens a binary WebSocket to the Holochain conductor,
/// authenticates with a required app token, discovers the role→cell_id
/// mapping via `app_info`, and then sends properly-formed zome call
/// requests with the correct `CellId`.
///
/// # Thread safety
///
/// Browser WASM is single-threaded, so this uses `Rc<RefCell<_>>` instead
/// of `Arc<Mutex<_>>`. This type is `!Send` and `!Sync`, which is correct
/// for the browser environment.
///
/// # Reconnection
///
/// If `ConnectConfig.reconnect` is set to `Some(ReconnectConfig)`, the transport
/// will automatically attempt to reconnect when the WebSocket closes. It uses
/// exponential backoff (base_delay_ms doubled each attempt, capped at max_delay_ms).
/// Pending requests at the time of disconnect are failed immediately.
/// If reconnect is `None`, the transport enters `Disconnected` and subsequent
/// `call_zome` calls return [`ClientError::NotConnected`].
#[derive(Clone)]
pub struct BrowserWsTransport {
    inner: Rc<RefCell<Inner>>,
}

impl BrowserWsTransport {
    /// Create a new transport instance. Does not connect immediately.
    pub fn new() -> Self {
        Self {
            inner: Rc::new(RefCell::new(Inner::default())),
        }
    }

    /// Register a callback for Holochain signals (app signals broadcast by the conductor).
    ///
    /// Signals are fire-and-forget messages from the conductor that don't correspond
    /// to any request. They carry MessagePack-encoded payloads from zome signal handlers.
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// transport.set_signal_handler(|payload: Vec<u8>| {
    ///     // Decode and route signal to the appropriate domain
    ///     web_sys::console::log_1(&format!("Signal: {} bytes", payload.len()).into());
    /// });
    /// ```
    pub fn set_signal_handler<F: Fn(Vec<u8>) + 'static>(&self, handler: F) {
        self.inner.borrow_mut().signal_handler = Some(Box::new(handler));
    }

    /// Observe future connection lifecycle changes, including reconnects.
    ///
    /// The callback is invoked after the internal `RefCell` borrow has been
    /// released, so it may safely query other application state. Registration
    /// does not emit the current status; call [`HolochainTransport::status`] if
    /// an initial snapshot is needed.
    pub fn set_status_handler<F: Fn(ConnectionStatus) + 'static>(&self, handler: F) {
        self.inner.borrow_mut().status_handler = Some(Rc::new(handler));
    }

    pub fn clear_status_handler(&self) {
        self.inner.borrow_mut().status_handler = None;
    }

    /// Install a Rust-side authorized signer for every zome call.
    ///
    /// If none is installed, the transport checks the official launcher host
    /// signer at `window.__HC_ZOME_CALL_SIGNER__`. There is no unsigned path.
    pub fn set_zome_call_signer<S: ZomeCallSigner + 'static>(&self, signer: S) {
        self.inner.borrow_mut().zome_call_signer = Some(Rc::new(signer));
    }

    pub fn clear_zome_call_signer(&self) {
        self.inner.borrow_mut().zome_call_signer = None;
    }

    /// Whether a Rust-side or launcher host signer is currently available.
    pub fn zome_call_signer_available(&self) -> bool {
        self.inner.borrow().zome_call_signer.is_some() || HostZomeCallSigner::is_available()
    }

    /// Return the connected agent pubkey bytes discovered from app info.
    pub fn connected_agent_pub_key(&self) -> Option<Vec<u8>> {
        self.inner.borrow().agent_pub_key.clone()
    }

    /// Return the connected agent pubkey in the same `u...` base64url form
    /// used by Holochain hash display formatting.
    pub fn connected_agent_pub_key_b64(&self) -> Option<String> {
        self.connected_agent_pub_key()
            .map(|bytes| format!("u{}", URL_SAFE_NO_PAD.encode(bytes)))
    }

    /// Return the connected Mycelix DID derived from the conductor agent key.
    pub fn connected_agent_did(&self) -> Option<String> {
        self.connected_agent_pub_key_b64()
            .map(|key| format!("did:mycelix:{key}"))
    }

    /// Store and publish a lifecycle transition without holding the internal
    /// state borrow while application code runs.
    fn publish_status(inner_rc: &Rc<RefCell<Inner>>, status: ConnectionStatus) {
        let handler = {
            let mut state = inner_rc.borrow_mut();
            state.status = status.clone();
            state.status_handler.clone()
        };
        if let Some(handler) = handler {
            handler(status);
        }
    }

    /// Fail in-flight work and schedule at most one reconnect attempt.
    fn handle_disconnect(inner_rc: &Rc<RefCell<Inner>>, reason: String) {
        let mut state = inner_rc.borrow_mut();
        state.ws = None;
        let pending: Vec<_> = state.pending.drain().collect();

        if state.reconnect_scheduled {
            drop(state);
            for (_, request) in pending {
                (request.resolve)(Err(ClientError::ConnectionFailed(reason.clone())));
            }
            return;
        }

        let reconnect_config = state
            .connect_config
            .as_ref()
            .and_then(|config| config.reconnect.clone());
        if let Some(reconnect) = reconnect_config {
            let attempt = state.reconnect_attempt + 1;
            if attempt <= reconnect.max_attempts {
                state.reconnect_attempt = attempt;
                state.reconnect_scheduled = true;
                let delay = (reconnect.base_delay_ms * 2u32.saturating_pow(attempt - 1))
                    .min(reconnect.max_delay_ms);
                let config = state
                    .connect_config
                    .clone()
                    .expect("reconnect config disappeared while borrowed");
                let reconnect_status = ConnectionStatus::Reconnecting {
                    attempt,
                    max_attempts: reconnect.max_attempts,
                };
                let reconnect_epoch = state.reconnect_epoch;
                let inner_for_reconnect = Rc::clone(inner_rc);
                drop(state);

                Self::publish_status(inner_rc, reconnect_status);
                for (_, request) in pending {
                    (request.resolve)(Err(ClientError::ConnectionFailed(reason.clone())));
                }
                web_sys::console::log_1(
                    &format!(
                        "[HC] Reconnect attempt {attempt}/{} in {delay}ms",
                        reconnect.max_attempts
                    )
                    .into(),
                );

                let closure = Closure::once(move || {
                    let should_reconnect = {
                        let mut state = inner_for_reconnect.borrow_mut();
                        if state.reconnect_epoch != reconnect_epoch
                            || state.connect_config.is_none()
                        {
                            false
                        } else {
                            state.reconnect_scheduled = false;
                            true
                        }
                    };
                    if !should_reconnect {
                        return;
                    }
                    let transport = BrowserWsTransport {
                        inner: Rc::clone(&inner_for_reconnect),
                    };
                    wasm_bindgen_futures::spawn_local(async move {
                        match transport.connect(config).await {
                            Ok(()) => {
                                web_sys::console::log_1(&"[HC] Reconnected successfully".into());
                                let mut state = transport.inner.borrow_mut();
                                state.reconnect_attempt = 0;
                                state.reconnect_scheduled = false;
                            }
                            Err(error) => {
                                web_sys::console::warn_1(
                                    &format!("[HC] Reconnect failed: {error}").into(),
                                );
                                BrowserWsTransport::handle_disconnect(
                                    &transport.inner,
                                    format!("Reconnect failed: {error}"),
                                );
                            }
                        }
                    });
                });

                if let Some(window) = web_sys::window() {
                    match window.set_timeout_with_callback_and_timeout_and_arguments_0(
                        closure.as_ref().unchecked_ref(),
                        delay as i32,
                    ) {
                        Ok(_) => closure.forget(),
                        Err(error) => {
                            inner_rc.borrow_mut().reconnect_scheduled = false;
                            Self::publish_status(
                                inner_rc,
                                ConnectionStatus::Error(format!(
                                    "failed to schedule reconnect: {error:?}"
                                )),
                            );
                        }
                    }
                } else {
                    inner_rc.borrow_mut().reconnect_scheduled = false;
                    Self::publish_status(
                        inner_rc,
                        ConnectionStatus::Error(
                            "cannot schedule reconnect without a browser window".into(),
                        ),
                    );
                }
                return;
            }
        }

        state.reconnect_scheduled = false;
        drop(state);
        Self::publish_status(inner_rc, ConnectionStatus::Disconnected);
        for (_, request) in pending {
            (request.resolve)(Err(ClientError::ConnectionFailed(reason.clone())));
        }
    }

    /// Allocate the next request ID.
    fn next_id(inner: &mut Inner) -> u64 {
        let id = inner.next_id;
        inner.next_id = inner.next_id.wrapping_add(1);
        id
    }

    /// Send a request envelope over the WebSocket and await the response bytes.
    ///
    /// This is the core request/response primitive used by app_info and
    /// call_zome. If a `request_timeout_ms` is configured, the
    /// request will be failed with `ClientError::Timeout` after that duration.
    fn send_request(
        inner_rc: &Rc<RefCell<Inner>>,
        request_type: &str,
        data: Vec<u8>,
    ) -> Pin<Box<dyn Future<Output = Result<Vec<u8>, ClientError>>>> {
        let inner = Rc::clone(inner_rc);
        let request_type = request_type.to_string();

        Box::pin(async move {
            let (id, wire_bytes, timeout_ms) = {
                let mut state = inner.borrow_mut();

                let ws = state.ws.as_ref().ok_or(ClientError::NotConnected)?;
                if ws.ready_state() != WebSocket::OPEN {
                    return Err(ClientError::NotConnected);
                }

                let id = Self::next_id(&mut state);
                let envelope = WireRequest {
                    id,
                    request_type,
                    data,
                };

                let wire_bytes = rmp_serde::to_vec_named(&envelope)
                    .map_err(|e| ClientError::SerializationError(e.to_string()))?;

                let timeout_ms = state
                    .connect_config
                    .as_ref()
                    .and_then(|c| c.request_timeout_ms)
                    .unwrap_or(30_000);

                (id, wire_bytes, timeout_ms)
            };

            // Create a oneshot channel for the response
            let (tx, rx) = futures::channel::oneshot::channel();

            {
                let mut state = inner.borrow_mut();
                state.pending.insert(
                    id,
                    PendingRequest {
                        resolve: Box::new(move |result| {
                            let _ = tx.send(result);
                        }),
                    },
                );

                let ws = state.ws.as_ref().ok_or(ClientError::NotConnected)?;
                ws.send_with_u8_array(&wire_bytes)
                    .map_err(|e| ClientError::WebSocketError(format!("{e:?}")))?;
            }

            // Schedule a timeout that removes the pending request
            let inner_for_timeout = Rc::clone(&inner);
            let timeout_closure = Closure::once(move || {
                let mut state = inner_for_timeout.borrow_mut();
                if let Some(pending) = state.pending.remove(&id) {
                    (pending.resolve)(Err(ClientError::Timeout(timeout_ms)));
                }
            });
            if let Some(window) = web_sys::window() {
                let _ = window.set_timeout_with_callback_and_timeout_and_arguments_0(
                    timeout_closure.as_ref().unchecked_ref(),
                    timeout_ms as i32,
                );
                timeout_closure.forget();
            }

            rx.await
                .map_err(|_| ClientError::WebSocketError("Response channel dropped".to_string()))?
        })
    }

    /// Set up WebSocket event callbacks (onmessage, onerror, onclose).
    fn attach_callbacks(inner_rc: &Rc<RefCell<Inner>>, ws: &WebSocket) {
        let mut callbacks = Vec::new();

        // -- onmessage: decode response and resolve pending future --
        {
            let inner = Rc::clone(inner_rc);
            let onmessage = Closure::<dyn FnMut(MessageEvent)>::new(move |event: MessageEvent| {
                let data = event.data();

                // Binary frame -> ArrayBuffer
                let bytes = if let Ok(buf) = data.dyn_into::<ArrayBuffer>() {
                    let arr = Uint8Array::new(&buf);
                    let mut vec = vec![0u8; arr.length() as usize];
                    arr.copy_to(&mut vec);
                    vec
                } else {
                    // Not a binary frame -- ignore (could be text heartbeat)
                    return;
                };

                // Decode the outer conductor message. Signals intentionally
                // have no request ID and must not be forced into a response.
                let message: IncomingWireMessage = match rmp_serde::from_slice(&bytes) {
                    Ok(message) => message,
                    Err(e) => {
                        web_sys::console::warn_1(
                            &format!("Failed to decode conductor message: {e}").into(),
                        );
                        return;
                    }
                };

                match message {
                    IncomingWireMessage::Signal { data } => {
                        let state = inner.borrow();
                        if let Some(ref handler) = state.signal_handler {
                            handler(data);
                        }
                    }
                    IncomingWireMessage::Response { id, data } => {
                        let mut state = inner.borrow_mut();
                        if let Some(pending) = state.pending.remove(&id) {
                            match data {
                                Some(data) => (pending.resolve)(Ok(data)),
                                None => (pending.resolve)(Err(ClientError::InvalidResponse(
                                    "conductor canceled the response".into(),
                                ))),
                            }
                        } else {
                            web_sys::console::warn_1(
                                &format!("Response for unknown request ID: {id}").into(),
                            );
                        }
                    }
                }
            });
            ws.set_onmessage(Some(onmessage.as_ref().unchecked_ref()));
            callbacks.push(onmessage.into_js_value());
        }

        // -- onerror --
        {
            let inner = Rc::clone(inner_rc);
            let onerror = Closure::<dyn FnMut(web_sys::ErrorEvent)>::new(
                move |event: web_sys::ErrorEvent| {
                    let msg = event.message();
                    let pending: Vec<_> = inner.borrow_mut().pending.drain().collect();
                    Self::publish_status(&inner, ConnectionStatus::Error(msg.clone()));
                    for (_, req) in pending {
                        (req.resolve)(Err(ClientError::WebSocketError(msg.clone())));
                    }
                    Self::handle_disconnect(&inner, format!("WebSocket error: {msg}"));
                },
            );
            ws.set_onerror(Some(onerror.as_ref().unchecked_ref()));
            callbacks.push(onerror.into_js_value());
        }

        // -- onclose --
        {
            let inner = Rc::clone(inner_rc);
            let onclose = Closure::<dyn FnMut(web_sys::CloseEvent)>::new(
                move |event: web_sys::CloseEvent| {
                    let reason = if event.reason().is_empty() {
                        format!("WebSocket closed (code {})", event.code())
                    } else {
                        event.reason()
                    };

                    Self::handle_disconnect(&inner, reason);
                },
            );
            ws.set_onclose(Some(onclose.as_ref().unchecked_ref()));
            callbacks.push(onclose.into_js_value());
        }

        // Store callbacks to prevent GC
        inner_rc.borrow_mut()._callbacks = callbacks;
    }
}

impl Default for BrowserWsTransport {
    fn default() -> Self {
        Self::new()
    }
}

impl HolochainTransport for BrowserWsTransport {
    fn connect(
        &self,
        config: ConnectConfig,
    ) -> Pin<Box<dyn Future<Output = Result<(), ClientError>>>> {
        let inner = Rc::clone(&self.inner);

        Box::pin(async move {
            let token = config.auth_token.clone().ok_or_else(|| {
                ClientError::AuthenticationFailed(
                    "Holochain 0.6 app WebSockets require an app authentication token".into(),
                )
            })?;

            // If already connected, no-op
            {
                let state = inner.borrow();
                if state.ws.is_some() && state.status == ConnectionStatus::Connected {
                    return Ok(());
                }
            }

            {
                let mut state = inner.borrow_mut();
                state.cell_map.clear();
                state.agent_pub_key = None;
                state.connect_config = Some(config.clone());
            }
            Self::publish_status(&inner, ConnectionStatus::Connecting);

            // Create WebSocket
            let ws = match WebSocket::new(&config.url) {
                Ok(ws) => ws,
                Err(error) => {
                    let error = ClientError::ConnectionFailed(format!("{error:?}"));
                    Self::publish_status(&inner, ConnectionStatus::Error(error.to_string()));
                    Self::handle_disconnect(&inner, error.to_string());
                    return Err(error);
                }
            };

            // Set binary type to arraybuffer for MessagePack
            ws.set_binary_type(web_sys::BinaryType::Arraybuffer);

            // Set up callbacks before waiting for open
            Self::attach_callbacks(&inner, &ws);

            // Wait for the WebSocket to open
            let open_promise = js_sys::Promise::new(&mut |resolve, reject| {
                let onopen = Closure::once(move |_: JsValue| {
                    resolve.call0(&JsValue::NULL).unwrap_or(JsValue::UNDEFINED);
                });
                ws.set_onopen(Some(onopen.as_ref().unchecked_ref()));
                // Prevent GC -- the closure is consumed after one call
                onopen.forget();

                // Also wire up a rejection on the pre-open error case
                let onerror_reject = Closure::once(move |_: web_sys::ErrorEvent| {
                    reject
                        .call1(&JsValue::NULL, &"Connection failed".into())
                        .unwrap_or(JsValue::UNDEFINED);
                });
                // Note: this overwrites the onerror we set in attach_callbacks,
                // but only until onopen fires. After connection, attach_callbacks
                // re-sets it. We accept this brief window.
                ws.set_onerror(Some(onerror_reject.as_ref().unchecked_ref()));
                onerror_reject.forget();
            });

            if let Err(error) = JsFuture::from(open_promise).await {
                let error = ClientError::ConnectionFailed(format!("{error:?}"));
                Self::publish_status(&inner, ConnectionStatus::Error(error.to_string()));
                let _ = ws.close();
                Self::handle_disconnect(&inner, error.to_string());
                return Err(error);
            }

            // Re-attach callbacks after the open event (onerror was overwritten)
            Self::attach_callbacks(&inner, &ws);

            // Store the connected WebSocket. The config was retained before
            // opening so startup failures can use the same retry policy.
            {
                let mut state = inner.borrow_mut();
                state.ws = Some(ws);
            }

            let initialize: Result<(), ClientError> = async {
                // Authentication is a fire-and-forget WireAuthenticate frame.
                // The next app_info response confirms that the token was
                // accepted and identifies the app it was issued for.
                let auth_inner = rmp_serde::to_vec_named(&AuthenticateRequest { token })
                    .map_err(|e| ClientError::SerializationError(e.to_string()))?;
                let frame = rmp_serde::to_vec_named(&WireAuthenticate {
                    msg_type: "authenticate".to_string(),
                    data: auth_inner,
                })
                .map_err(|e| ClientError::SerializationError(e.to_string()))?;

                {
                    let state = inner.borrow();
                    let ws = state.ws.as_ref().ok_or(ClientError::NotConnected)?;
                    ws.send_with_u8_array(&frame)
                        .map_err(|e| ClientError::WebSocketError(format!("{e:?}")))?;
                    // Fire-and-forget: do NOT register a pending request or await.
                }

                // --- Fix 2: AppInfo discovery ---
                // Unit variant: the conductor scopes app_info by the
                // connection's authenticated app_id, not a request field.
                let app_info_bytes = rmp_serde::to_vec_named(&AppRequest::AppInfo)
                    .map_err(|e| ClientError::SerializationError(e.to_string()))?;

                let response_bytes = Self::send_request(&inner, "request", app_info_bytes).await?;

                // Parse the response to build role→cell_id mapping
                let app_info: AppResponse = decode_tagged(&response_bytes).map_err(|e| {
                    ClientError::InvalidResponse(format!(
                        "Failed to parse app_info response: {e:?}"
                    ))
                })?;

                match app_info {
                    AppResponse::AppInfo(Some(info)) => {
                        if info.installed_app_id != config.app_id {
                            return Err(ClientError::AuthenticationFailed(format!(
                                "token resolved app '{}' instead of requested '{}'",
                                info.installed_app_id, config.app_id
                            )));
                        }

                        let mut state = inner.borrow_mut();
                        let mut first_agent: Option<Vec<u8>> = None;

                        for (role_name, cells) in &info.cell_info {
                            for cell_variant in cells {
                                if let CellInfoVariant::Provisioned(cell) = cell_variant {
                                    state
                                        .cell_map
                                        .insert(role_name.clone(), cell.cell_id.clone());
                                    // Capture agent pub key from the first cell
                                    if first_agent.is_none() {
                                        first_agent = Some(cell.cell_id.1.clone());
                                    }
                                    break; // Use first provisioned cell per role
                                }
                            }
                        }

                        state.agent_pub_key = first_agent;

                        if state.cell_map.is_empty() {
                            web_sys::console::warn_1(
                                &format!(
                                    "app_info for '{}' returned no provisioned cells",
                                    config.app_id
                                )
                                .into(),
                            );
                        }
                    }
                    AppResponse::AppInfo(None) => {
                        return Err(ClientError::ConnectionFailed(format!(
                            "no installed app matches app_id '{}' for this connection's token",
                            config.app_id
                        )));
                    }
                    AppResponse::Error(e) => {
                        return Err(ClientError::ConnectionFailed(format!(
                            "app_info failed: {}",
                            e.message()
                        )));
                    }
                    _ => {
                        return Err(ClientError::InvalidResponse(
                            "Unexpected response type for app_info".into(),
                        ));
                    }
                }

                Ok(())
            }
            .await;

            match initialize {
                Ok(()) => {
                    Self::publish_status(&inner, ConnectionStatus::Connected);
                    Ok(())
                }
                Err(error) => {
                    let mut state = inner.borrow_mut();
                    let ws = state.ws.take();
                    state.cell_map.clear();
                    state.agent_pub_key = None;
                    drop(state);
                    Self::publish_status(&inner, ConnectionStatus::Error(error.to_string()));
                    if let Some(ws) = ws {
                        let _ = ws.close();
                    }
                    Self::handle_disconnect(&inner, error.to_string());
                    Err(error)
                }
            }
        })
    }

    fn call_zome(
        &self,
        role_name: &str,
        zome_name: &str,
        fn_name: &str,
        payload: Vec<u8>,
    ) -> Pin<Box<dyn Future<Output = Result<Vec<u8>, ClientError>>>> {
        let inner = Rc::clone(&self.inner);
        let role_name = role_name.to_string();
        let zome_name = zome_name.to_string();
        let fn_name = fn_name.to_string();

        Box::pin(async move {
            // --- Fix 3: Look up cell_id from stored mapping ---
            let (cell_id, rust_signer) = {
                let state = inner.borrow();

                let cell_id = state
                    .cell_map
                    .get(&role_name)
                    .ok_or_else(|| ClientError::UnknownRole(role_name.clone()))?
                    .clone();

                (cell_id, state.zome_call_signer.clone())
            };

            // --- Fix 7: require a real signer; no unsigned fallback ---
            // `set_zome_call_signer`'s own doc comment already promised "no
            // unsigned path", but until now this function ignored both the
            // Rust-side signer and the host launcher signer and always sent
            // a zeroed signature — the signing infrastructure below existed
            // but was never wired into the actual call path.
            let nonce = generate_nonce()?;
            let expires_at =
                // 60s, not the previous 5s: a 5-second nonce window left
                // essentially no margin for real WebSocket round-trip +
                // conductor processing latency and was confirmed, in the
                // Pulse Sweettest suite (same BadNonce("Expired") failure
                // class, longer 5-minute window there), to be genuinely
                // exceedable even without any external contention. Still
                // well inside Holochain's own conductor-side upper bound
                // (holochain_nonce::FRESH_NONCE_EXPIRES_AFTER = 5 minutes).
                now_micros() + 60_000_000;

            let to_sign = ZomeCallToSign {
                cell_id,
                zome_name,
                fn_name,
                payload,
                nonce,
                expires_at,
            };

            // --- Fix 8: use the signer's SignedZomeCall directly instead of
            // discarding it ---
            // The signer already builds and signs the real ZomeCallParams
            // shape (see admin_granted_signer::ZomeCallParamsWire) — its
            // `bytes()` IS the map-encoded ZomeCallParams the conductor
            // hashes and verifies `signature()` against. Re-serializing a
            // different, flat CallZomeRequestWire here (as this code
            // previously did, extracting only `.signature()` and
            // discarding `.bytes()`) sent a structurally wrong request
            // whose signature didn't match what was actually hashed —
            // confirmed live via the Pulse browser proof: every zome call
            // failed with "Failed to deserialize request" even with a real
            // signer installed and a correctly-computed signature.
            let signed = if let Some(signer) = rust_signer {
                signer.sign_zome_call(to_sign).await?
            } else if HostZomeCallSigner::is_available() {
                HostZomeCallSigner.sign_zome_call(to_sign).await?
            } else {
                return Err(ClientError::SigningUnavailable(
                    "no authorized zome-call signer is configured (neither a Rust-side signer \
                     nor window.__HC_ZOME_CALL_SIGNER__); unsigned zome calls are not sent"
                        .into(),
                ));
            };

            let call_request = AppRequest::CallZome(signed);
            let call_bytes = rmp_serde::to_vec_named(&call_request)
                .map_err(|e| ClientError::SerializationError(e.to_string()))?;

            let response_bytes = Self::send_request(&inner, "request", call_bytes).await?;

            // Try to decode as AppResponse for proper error handling
            match decode_tagged::<AppResponse>(&response_bytes) {
                Ok(AppResponse::ZomeCalled(data)) => Ok(data),
                Ok(AppResponse::Error(e)) => {
                    Err(ClientError::ZomeCallFailed(e.message().to_string()))
                }
                Ok(other) => Err(ClientError::InvalidResponse(format!(
                    "Unexpected response type for call_zome: {other:?}"
                ))),
                // A decode failure must not be treated as a successful call
                // with the raw undecoded bytes as its result — the caller
                // would try to further decode `response_bytes` as their
                // expected ExternIO type and get a confusing failure later
                // instead of an immediate, clear error here.
                Err(error) => Err(ClientError::InvalidResponse(format!(
                    "failed to decode call_zome response: {error}"
                ))),
            }
        })
    }

    fn status(&self) -> ConnectionStatus {
        self.inner.borrow().status.clone()
    }

    fn disconnect(&self) {
        let (ws, pending) = {
            let mut state = self.inner.borrow_mut();
            let ws = state.ws.take();
            state.cell_map.clear();
            state.agent_pub_key = None;
            state.connect_config = None;
            state.reconnect_attempt = 0;
            state.reconnect_scheduled = false;
            state.reconnect_epoch = state.reconnect_epoch.wrapping_add(1);
            state._callbacks.clear();
            let pending: Vec<_> = state.pending.drain().collect();
            (ws, pending)
        };
        Self::publish_status(&self.inner, ConnectionStatus::Disconnected);
        if let Some(ws) = ws {
            let _ = ws.close();
        }
        for (_, req) in pending {
            (req.resolve)(Err(ClientError::NotConnected));
        }
    }
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// Generate a random 32-byte nonce using Web Crypto. There is deliberately no
/// `Math.random` fallback for a replay-protection value.
fn generate_nonce() -> Result<Vec<u8>, ClientError> {
    let crypto = js_sys::Reflect::get(&js_sys::global(), &"crypto".into()).map_err(|error| {
        ClientError::SecureRandomUnavailable(format!("global crypto lookup failed: {error:?}"))
    })?;
    if crypto.is_undefined() || crypto.is_null() {
        return Err(ClientError::SecureRandomUnavailable(
            "Web Crypto is not available".into(),
        ));
    }

    let get_random_values = js_sys::Reflect::get(&crypto, &"getRandomValues".into())
        .map_err(|error| {
            ClientError::SecureRandomUnavailable(format!(
                "crypto.getRandomValues lookup failed: {error:?}"
            ))
        })?
        .dyn_into::<js_sys::Function>()
        .map_err(|_| {
            ClientError::SecureRandomUnavailable("crypto.getRandomValues is not callable".into())
        })?;

    let array = js_sys::Uint8Array::new_with_length(32);
    get_random_values.call1(&crypto, &array).map_err(|error| {
        ClientError::SecureRandomUnavailable(format!("crypto.getRandomValues failed: {error:?}"))
    })?;

    let mut nonce = vec![0u8; 32];
    array.copy_to(&mut nonce);
    Ok(nonce)
}

/// Current time in microseconds since epoch, using `Date.now()`.
fn now_micros() -> u64 {
    (js_sys::Date::now() * 1000.0) as u64
}

fn zome_call_to_js(request: ZomeCallToSign) -> Result<JsValue, ClientError> {
    let object = js_sys::Object::new();
    let cell_id = js_sys::Array::new();
    cell_id.push(&uint8_array(&request.cell_id.0));
    cell_id.push(&uint8_array(&request.cell_id.1));

    set_js_property(&object, "cell_id", cell_id.as_ref())?;
    set_js_property(&object, "zome_name", &request.zome_name.into())?;
    set_js_property(&object, "fn_name", &request.fn_name.into())?;

    // The official host hook accepts an unsigned CallZomeRequest and performs
    // the ExternIO encoding itself. Convert our already-encoded transport
    // payload back to a JS value so the host does not MessagePack-encode a
    // Uint8Array wrapper a second time.
    let payload: serde_json::Value = rmp_serde::from_slice(&request.payload).map_err(|error| {
        ClientError::SerializationError(format!(
            "host signer payload is not a supported MessagePack value: {error}"
        ))
    })?;
    let payload = serde_wasm_bindgen::to_value(&payload).map_err(|error| {
        ClientError::SerializationError(format!(
            "failed to convert host signer payload to JavaScript: {error}"
        ))
    })?;
    set_js_property(&object, "payload", &payload)?;

    // AppWebsocket populates provenance before invoking the host signer. For
    // a role-resolved request, it is the agent key from the CellId.
    set_js_property(&object, "provenance", &uint8_array(&request.cell_id.1))?;
    Ok(object.into())
}

fn uint8_array(bytes: &[u8]) -> JsValue {
    let value = Uint8Array::new_with_length(bytes.len() as u32);
    value.copy_from(bytes);
    value.into()
}

fn set_js_property(
    object: &js_sys::Object,
    name: &str,
    value: &JsValue,
) -> Result<(), ClientError> {
    js_sys::Reflect::set(object, &name.into(), value)
        .map(|_| ())
        .map_err(|error| {
            ClientError::SerializationError(format!(
                "failed to construct host signer request field '{name}': {}",
                js_error_message(&error)
            ))
        })
}

fn required_uint8_array(value: &JsValue, field: &str) -> Result<Vec<u8>, ClientError> {
    let field_value = js_sys::Reflect::get(value, &field.into()).map_err(|error| {
        ClientError::InvalidSignature(format!(
            "host signer result field '{field}' could not be read: {}",
            js_error_message(&error)
        ))
    })?;
    if !field_value.is_instance_of::<Uint8Array>() {
        return Err(ClientError::InvalidSignature(format!(
            "host signer result field '{field}' must be a Uint8Array"
        )));
    }

    let array = Uint8Array::new(&field_value);
    let mut bytes = vec![0u8; array.length() as usize];
    array.copy_to(&mut bytes);
    Ok(bytes)
}

fn js_error_message(value: &JsValue) -> String {
    value
        .as_string()
        .or_else(|| {
            js_sys::Reflect::get(value, &"message".into())
                .ok()
                .and_then(|message| message.as_string())
        })
        .unwrap_or_else(|| format!("{value:?}"))
}
