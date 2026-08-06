// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Native WebSocket transport for testing against a real Holochain conductor.
//!
//! Same wire protocol as `BrowserWsTransport` but uses `tokio-tungstenite`
//! instead of `web_sys::WebSocket`. This allows testing from native Rust
//! without a browser.
//!
//! # Usage
//!
//! ```rust,ignore
//! let transport = NativeWsTransport::new();
//! transport.connect(ConnectConfig {
//!     url: "ws://localhost:8888".into(),
//!     app_id: "mycelix-commons".into(),
//!     auth_token: None,
//! }).await?;
//!
//! let result = transport.call_zome("commons", "proposals", "list", encode(&())?).await?;
//! ```

use futures_util::{SinkExt, StreamExt};
use std::collections::HashMap;
use std::sync::Arc;
use tokio::sync::{Mutex, oneshot};
use tokio_tungstenite::tungstenite::Message;

use crate::error::ClientError;
use crate::transport::HolochainTransport;
use crate::types::*;

type CellId = (Vec<u8>, Vec<u8>);
type PendingMap = HashMap<u64, oneshot::Sender<Result<Vec<u8>, ClientError>>>;

struct Inner {
    status: ConnectionStatus,
    next_id: u64,
    cell_map: HashMap<String, CellId>,
    agent_pub_key: Option<Vec<u8>>,
    pending: PendingMap,
}

/// Native WebSocket transport for testing.
#[derive(Clone)]
pub struct NativeWsTransport {
    inner: Arc<Mutex<Inner>>,
    write_tx: Arc<
        Mutex<
            Option<
                futures_util::stream::SplitSink<
                    tokio_tungstenite::WebSocketStream<
                        tokio_tungstenite::MaybeTlsStream<tokio::net::TcpStream>,
                    >,
                    Message,
                >,
            >,
        >,
    >,
}

impl NativeWsTransport {
    pub fn new() -> Self {
        Self {
            inner: Arc::new(Mutex::new(Inner {
                status: ConnectionStatus::Disconnected,
                next_id: 1,
                cell_map: HashMap::new(),
                agent_pub_key: None,
                pending: HashMap::new(),
            })),
            write_tx: Arc::new(Mutex::new(None)),
        }
    }

    async fn send_request(
        &self,
        request_type: &str,
        data: Vec<u8>,
    ) -> Result<Vec<u8>, ClientError> {
        let (tx, rx) = oneshot::channel();

        let id = {
            let mut inner = self.inner.lock().await;
            let id = inner.next_id;
            inner.next_id += 1;
            inner.pending.insert(id, tx);
            id
        };

        let wire = WireRequest {
            id,
            request_type: request_type.to_string(),
            data,
        };
        let bytes = rmp_serde::to_vec_named(&wire)
            .map_err(|e| ClientError::SerializationError(e.to_string()))?;

        let mut write_guard = self.write_tx.lock().await;
        let write = write_guard.as_mut().ok_or(ClientError::NotConnected)?;
        write
            .send(Message::Binary(bytes.into()))
            .await
            .map_err(|e| ClientError::WebSocketError(e.to_string()))?;
        drop(write_guard);

        rx.await
            .map_err(|_| ClientError::WebSocketError("channel closed".into()))?
    }

    /// This transport has no authorized `ZomeCallSigner` adapter (unlike
    /// `BrowserWsTransport`, which can use a Rust-side signer or the
    /// launcher's `window.__HC_ZOME_CALL_SIGNER__`). It previously sent
    /// every zome call with a hardcoded zeroed signature — refuse instead
    /// of sending an unsigned call.
    async fn call_zome_impl(
        &self,
        _role_name: &str,
        _zome_name: &str,
        _fn_name: &str,
        _payload: Vec<u8>,
    ) -> Result<Vec<u8>, ClientError> {
        Err(ClientError::SigningUnavailable(
            "NativeWsTransport has no authorized zome-call signer configured; unsigned \
             zome calls are not sent"
                .into(),
        ))
    }

    async fn connect_impl(&self, config: ConnectConfig) -> Result<(), ClientError> {
        eprintln!("[NativeWs] Connecting to {}...", config.url);

        // Holochain conductor requires Origin header in WebSocket upgrade
        use tokio_tungstenite::tungstenite::client::IntoClientRequest;
        let mut request = config
            .url
            .clone()
            .into_client_request()
            .map_err(|e| ClientError::ConnectionFailed(e.to_string()))?;
        request
            .headers_mut()
            .insert("Origin", "http://localhost".parse().unwrap());

        let (ws_stream, _) = tokio_tungstenite::connect_async(request)
            .await
            .map_err(|e| ClientError::ConnectionFailed(e.to_string()))?;

        let (write, mut read) = ws_stream.split();
        *self.write_tx.lock().await = Some(write);

        // Spawn reader task
        let inner = self.inner.clone();
        tokio::spawn(async move {
            while let Some(msg) = read.next().await {
                if let Ok(Message::Binary(data)) = msg {
                    if let Ok(message) = rmp_serde::from_slice::<IncomingWireMessage>(&data) {
                        if let IncomingWireMessage::Response { id, data } = message {
                            let mut state = inner.lock().await;
                            if let Some(tx) = state.pending.remove(&id) {
                                let result = data.ok_or_else(|| {
                                    ClientError::InvalidResponse(
                                        "conductor canceled the response".into(),
                                    )
                                });
                                let _ = tx.send(result);
                            }
                        }
                    }
                }
            }
        });

        // Authenticate if token provided
        if let Some(token) = config.auth_token {
            let auth_req = AppRequest::Authenticate { token };
            let data = rmp_serde::to_vec_named(&auth_req)
                .map_err(|e| ClientError::SerializationError(e.to_string()))?;
            let _ = self.send_request("authenticate", data).await;
        }

        // App Info discovery. Unit variant: the conductor scopes
        // app_info by the connection's authenticated app_id, not a
        // request field (same fix as browser.rs).
        let data = rmp_serde::to_vec_named(&AppRequest::AppInfo)
            .map_err(|e| ClientError::SerializationError(e.to_string()))?;

        let info_bytes = self.send_request("request", data).await?;

        // Must unwrap the AppResponse envelope first — this used to
        // deserialize the raw bytes directly as AppInfoResponse, which
        // never matched the real `{"type": ..., "value": ...}` shape.
        // decode_tagged (not plain rmp_serde::from_slice) is required:
        // the conductor map-encodes tag/content enums, which the default
        // array-mode deserializer can't parse (see decode_tagged's doc).
        let response: AppResponse = decode_tagged(&info_bytes)
            .map_err(|e| ClientError::InvalidResponse(format!("app_info decode: {e:?}")))?;
        let info = match response {
            AppResponse::AppInfo(Some(info)) => info,
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
        };

        if info.installed_app_id != config.app_id {
            return Err(ClientError::AuthenticationFailed(format!(
                "token resolved app '{}' instead of requested '{}'",
                info.installed_app_id, config.app_id
            )));
        }

        eprintln!(
            "[NativeWs] App: {}, {} roles",
            info.installed_app_id,
            info.cell_info.len()
        );

        let mut inner = self.inner.lock().await;
        for (role_name, cells) in &info.cell_info {
            for cell in cells {
                if let CellInfoVariant::Provisioned(p) = cell {
                    inner.cell_map.insert(role_name.clone(), p.cell_id.clone());
                    if inner.agent_pub_key.is_none() {
                        inner.agent_pub_key = Some(p.cell_id.1.clone());
                    }
                    eprintln!("[NativeWs] Role '{role_name}' -> cell discovered");
                }
            }
        }
        inner.status = ConnectionStatus::Connected;

        eprintln!(
            "[NativeWs] Connected! {} roles mapped",
            inner.cell_map.len()
        );
        Ok(())
    }

    /// Send-safe variants of the `HolochainTransport` methods below, for
    /// callers (e.g. Tauri's `#[tauri::command]` async fns) that need
    /// `Future: Send`. The trait methods return `Pin<Box<dyn Future<Output
    /// = ...>>>` with no `+ Send` bound, because `BrowserWsTransport` also
    /// implements this trait and WASM futures are never `Send`. This
    /// concrete type's state (`tokio::sync::Mutex`, channels) is Send-safe,
    /// so these inherent methods just re-expose `connect_impl`/
    /// `call_zome_impl` as ordinary (non-erased) `async fn`s, which the
    /// compiler can verify are `Send`.
    pub async fn connect_send(&self, config: ConnectConfig) -> Result<(), ClientError> {
        self.connect_impl(config).await
    }

    pub async fn call_zome_send(
        &self,
        role_name: &str,
        zome_name: &str,
        fn_name: &str,
        payload: Vec<u8>,
    ) -> Result<Vec<u8>, ClientError> {
        self.call_zome_impl(role_name, zome_name, fn_name, payload)
            .await
    }
}

impl HolochainTransport for NativeWsTransport {
    fn call_zome(
        &self,
        role_name: &str,
        zome_name: &str,
        fn_name: &str,
        payload: Vec<u8>,
    ) -> std::pin::Pin<Box<dyn std::future::Future<Output = Result<Vec<u8>, ClientError>>>> {
        let this = self.clone();
        let role = role_name.to_string();
        let zome = zome_name.to_string();
        let fname = fn_name.to_string();
        Box::pin(async move { this.call_zome_impl(&role, &zome, &fname, payload).await })
    }

    fn status(&self) -> ConnectionStatus {
        // Can't await in a sync fn — return last known status
        ConnectionStatus::Disconnected // Will be overridden after connect
    }

    fn connect(
        &self,
        config: ConnectConfig,
    ) -> std::pin::Pin<Box<dyn std::future::Future<Output = Result<(), ClientError>>>> {
        let this = self.clone();
        Box::pin(async move { this.connect_impl(config).await })
    }

    fn disconnect(&self) {
        // Drop the write half — reader will close naturally
        let write_tx = self.write_tx.clone();
        tokio::spawn(async move {
            *write_tx.lock().await = None;
        });
    }
}
