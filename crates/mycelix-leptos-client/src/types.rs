// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Core types for the Holochain browser client.
//!
//! These types mirror the Holochain conductor wire protocol at a level
//! sufficient for zome calls without depending on `holochain_types` or
//! `holochain_conductor_api` (which pull in tokio and cannot compile to
//! `wasm32-unknown-unknown`).

use serde::{Deserialize, Serialize};
use std::future::Future;
use std::pin::Pin;

use crate::error::ClientError;

/// Serde adapter that preserves byte vectors as MessagePack `bin` values.
///
/// Holochain hashes, authentication tokens, payloads, and signatures are
/// binary values on the conductor wire. A plain `Vec<u8>` is otherwise encoded
/// by `rmp-serde` as an integer array, which is a different protocol shape.
pub(crate) mod binary_bytes {
    use serde::de::{SeqAccess, Visitor};
    use serde::{Deserializer, Serializer};
    use std::fmt;

    pub fn serialize<S>(bytes: &[u8], serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        serializer.serialize_bytes(bytes)
    }

    pub fn deserialize<'de, D>(deserializer: D) -> Result<Vec<u8>, D::Error>
    where
        D: Deserializer<'de>,
    {
        struct BytesVisitor;

        impl<'de> Visitor<'de> for BytesVisitor {
            type Value = Vec<u8>;

            fn expecting(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
                formatter.write_str("a MessagePack binary value")
            }

            fn visit_bytes<E>(self, value: &[u8]) -> Result<Self::Value, E> {
                Ok(value.to_vec())
            }

            fn visit_byte_buf<E>(self, value: Vec<u8>) -> Result<Self::Value, E> {
                Ok(value)
            }

            // Accept legacy array-encoded bytes on reads during migration.
            fn visit_seq<A>(self, mut sequence: A) -> Result<Self::Value, A::Error>
            where
                A: SeqAccess<'de>,
            {
                let mut bytes = Vec::with_capacity(sequence.size_hint().unwrap_or(0));
                while let Some(byte) = sequence.next_element()? {
                    bytes.push(byte);
                }
                Ok(bytes)
            }
        }

        deserializer.deserialize_any(BytesVisitor)
    }
}

/// Serde adapter for a `(DnaHash, AgentPubKey)` pair, encoding each half as
/// MessagePack `bin` via [`binary_bytes`] rather than the default per-tuple
/// element array encoding.
mod cell_id_bytes {
    use super::binary_bytes;
    use serde::{Deserialize, Deserializer, Serialize, Serializer};

    pub fn serialize<S>(cell_id: &(Vec<u8>, Vec<u8>), serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        #[derive(Serialize)]
        struct Wire<'a>(
            #[serde(serialize_with = "binary_bytes::serialize")] &'a [u8],
            #[serde(serialize_with = "binary_bytes::serialize")] &'a [u8],
        );
        Wire(&cell_id.0, &cell_id.1).serialize(serializer)
    }

    pub fn deserialize<'de, D>(deserializer: D) -> Result<(Vec<u8>, Vec<u8>), D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct Wire(
            #[serde(deserialize_with = "binary_bytes::deserialize")] Vec<u8>,
            #[serde(deserialize_with = "binary_bytes::deserialize")] Vec<u8>,
        );
        let wire = Wire::deserialize(deserializer)?;
        Ok((wire.0, wire.1))
    }
}

// ---------------------------------------------------------------------------
// Connection status
// ---------------------------------------------------------------------------

/// Current state of the WebSocket connection to the conductor.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ConnectionStatus {
    /// No connection attempt has been made, or the connection was closed.
    Disconnected,
    /// A connection attempt is in progress.
    Connecting,
    /// WebSocket authentication and app-info discovery succeeded.
    ///
    /// Browser zome calls additionally require an available authorized signer.
    Connected,
    /// Attempting to reconnect after a disconnect (attempt N of max).
    Reconnecting { attempt: u32, max_attempts: u32 },
    /// The connection is in an error state.
    Error(String),
}

// ---------------------------------------------------------------------------
// Connection configuration
// ---------------------------------------------------------------------------

/// Configuration for connecting to a Holochain conductor.
#[derive(Debug, Clone)]
pub struct ConnectConfig {
    /// WebSocket URL (e.g. "ws://localhost:8888").
    pub url: String,
    /// The installed app ID to discover cell mappings for.
    pub app_id: String,
    /// Authentication token issued by the conductor's admin API.
    ///
    /// The field remains optional so transports can define their own policy,
    /// but the Holochain 0.6 browser and native WebSocket transports reject
    /// `None` before opening an application session.
    pub auth_token: Option<Vec<u8>>,
    /// Optional auto-reconnect configuration. If `Some`, the transport will
    /// attempt to reconnect when the WebSocket closes unexpectedly.
    pub reconnect: Option<ReconnectConfig>,
    /// Timeout in milliseconds for individual zome call requests.
    /// Defaults to 30_000ms (30 seconds) if `None`.
    pub request_timeout_ms: Option<u32>,
}

/// Configuration for automatic WebSocket reconnection.
#[derive(Debug, Clone)]
pub struct ReconnectConfig {
    /// Maximum number of reconnection attempts before giving up.
    pub max_attempts: u32,
    /// Initial delay between reconnection attempts in milliseconds.
    /// Doubled after each failed attempt (exponential backoff).
    pub base_delay_ms: u32,
    /// Maximum delay between reconnection attempts in milliseconds.
    pub max_delay_ms: u32,
}

impl Default for ReconnectConfig {
    fn default() -> Self {
        Self {
            max_attempts: 5,
            base_delay_ms: 1_000,
            max_delay_ms: 30_000,
        }
    }
}

// ---------------------------------------------------------------------------
// Zome call request/response (our internal representation)
// ---------------------------------------------------------------------------

/// A request to call a zome function on the conductor.
///
/// The `payload` field is already MessagePack-encoded (the zome function's
/// input type). The transport layer wraps this in the conductor's wire
/// protocol envelope before sending.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ZomeCallRequest {
    /// Role name from the hApp manifest (e.g. "governance", "commons").
    pub role_name: String,
    /// Zome name within the DNA (e.g. "agora", "food-production").
    pub zome_name: String,
    /// Function name exported by the zome (e.g. "create_proposal").
    pub fn_name: String,
    /// MessagePack-encoded input payload for the zome function.
    pub payload: Vec<u8>,
}

/// A response from a zome call.
///
/// The `payload` field contains the MessagePack-encoded return value from
/// the zome function. Use [`decode`] to deserialize it into the expected type.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ZomeCallResponse {
    /// MessagePack-encoded output from the zome function.
    pub payload: Vec<u8>,
}

/// Complete low-level call context handed to a Rust-side authorized signer.
///
/// `payload` is already ExternIO/MessagePack encoded. The signer owns
/// capability-secret and provenance selection, builds the unsigned conductor
/// parameters with the supplied nonce and expiration, and returns their final
/// encoded bytes plus Ed25519 signature. The browser host adapter translates
/// this into the official JavaScript `CallZomeRequest` shape.
#[derive(Debug, Clone)]
pub struct ZomeCallToSign {
    pub cell_id: (Vec<u8>, Vec<u8>),
    pub zome_name: String,
    pub fn_name: String,
    pub payload: Vec<u8>,
    pub nonce: Vec<u8>,
    pub expires_at: u64,
}

/// Conductor wire representation of an authorized zome call.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SignedZomeCall {
    #[serde(with = "binary_bytes")]
    bytes: Vec<u8>,
    #[serde(with = "binary_bytes")]
    signature: Vec<u8>,
}

impl SignedZomeCall {
    pub fn new(bytes: Vec<u8>, signature: Vec<u8>) -> Result<Self, ClientError> {
        if bytes.is_empty() {
            return Err(ClientError::InvalidSignature(
                "signed zome-call bytes must not be empty".into(),
            ));
        }
        if signature.len() != 64 {
            return Err(ClientError::InvalidSignature(format!(
                "expected a 64-byte Ed25519 signature, got {} bytes",
                signature.len()
            )));
        }
        if signature.iter().all(|byte| *byte == 0) {
            return Err(ClientError::InvalidSignature(
                "zero signatures are never authorized".into(),
            ));
        }
        Ok(Self { bytes, signature })
    }

    pub fn bytes(&self) -> &[u8] {
        &self.bytes
    }

    pub fn signature(&self) -> &[u8] {
        &self.signature
    }
}

/// Application boundary for authorized zome-call signing.
pub trait ZomeCallSigner {
    fn sign_zome_call(
        &self,
        request: ZomeCallToSign,
    ) -> Pin<Box<dyn Future<Output = Result<SignedZomeCall, ClientError>>>>;
}

// ---------------------------------------------------------------------------
// Holochain wire protocol types
// ---------------------------------------------------------------------------

/// Envelope for requests sent to the conductor over WebSocket.
///
/// The Holochain conductor expects:
/// - `type`: always `"request"` (NOT "call_zome" or "app_info")
/// - `data`: MessagePack-encoded inner request (double-encoded)
/// - `id`: numeric correlation ID
///
/// The actual request type ("app_info", "call_zome") goes INSIDE the
/// `data` field as a tagged enum: `{type: "app_info", value: null}`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub(crate) struct WireRequest {
    /// Unique request identifier for response correlation.
    pub id: u64,
    /// Always "request" for app API, "authenticate" for auth.
    #[serde(rename = "type")]
    pub request_type: String,
    /// Double-encoded: msgpack(AppRequest) inside this bytes field.
    #[serde(with = "binary_bytes")]
    pub data: Vec<u8>,
}

/// Incoming outer conductor message.
///
/// Signals do not have request IDs, so they cannot be decoded as responses.
#[derive(Debug, Clone, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub(crate) enum IncomingWireMessage {
    Response {
        id: u64,
        #[serde(with = "optional_binary_bytes")]
        data: Option<Vec<u8>>,
    },
    Signal {
        #[serde(with = "binary_bytes")]
        data: Vec<u8>,
    },
}

mod optional_binary_bytes {
    use super::binary_bytes;
    use serde::{Deserialize, Deserializer, Serializer};

    pub fn serialize<S>(bytes: &Option<Vec<u8>>, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        match bytes {
            Some(bytes) => binary_bytes::serialize(bytes, serializer),
            None => serializer.serialize_none(),
        }
    }

    pub fn deserialize<'de, D>(deserializer: D) -> Result<Option<Vec<u8>>, D::Error>
    where
        D: Deserializer<'de>,
    {
        let value = Option::<serde_bytes_compat::BinaryBytes>::deserialize(deserializer)?;
        Ok(value.map(|bytes| bytes.0))
    }

    mod serde_bytes_compat {
        use super::binary_bytes;
        use serde::{Deserialize, Deserializer};

        pub struct BinaryBytes(pub Vec<u8>);

        impl<'de> Deserialize<'de> for BinaryBytes {
            fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
            where
                D: Deserializer<'de>,
            {
                binary_bytes::deserialize(deserializer).map(Self)
            }
        }
    }
}

/// Holochain 0.6 `WireMessage::Authenticate { data }` — an un-id'd frame that
/// the conductor never responds to (holochain_websocket 0.6). `data` is
/// msgpack([`AuthenticateRequest`]). Distinct from [`WireRequest`], which
/// carries an `id` and expects a response.
#[derive(Debug, Clone, Serialize)]
pub(crate) struct WireAuthenticate {
    /// Always "authenticate".
    #[serde(rename = "type")]
    pub msg_type: String,
    /// msgpack(AppAuthenticationRequest) bytes.
    #[serde(with = "binary_bytes")]
    pub data: Vec<u8>,
}

/// `AppAuthenticationRequest { token }` (holochain_conductor_api 0.6) — the
/// inner payload of a [`WireAuthenticate`] frame. A bare `{ token }`, NOT the
/// externally-tagged [`AppRequest`] enum.
#[derive(Debug, Clone, Serialize)]
pub(crate) struct AuthenticateRequest {
    /// The auth token issued by the conductor's admin API.
    #[serde(with = "binary_bytes")]
    pub token: Vec<u8>,
}

// ---------------------------------------------------------------------------
// AppRequest / AppResponse — conductor wire protocol enums
// ---------------------------------------------------------------------------

/// Requests the conductor App API understands.
///
/// Serialized as externally tagged: `{"type": "call_zome", "value": {...}}`
/// (matches `holochain_conductor_api::app_interface::AppRequest`'s real
/// `#[serde(tag = "type", content = "value")]` — a prior `content = "data"`
/// here was a genuine bug, found live via the Pulse browser proof: it
/// deserialized fine for nothing because no real conductor was ever
/// exercised through this path before).
#[derive(Debug, Clone, Serialize)]
#[serde(tag = "type", content = "value")]
#[cfg_attr(not(any(feature = "native", test)), allow(dead_code))]
pub(crate) enum AppRequest {
    /// Authenticate with the conductor using an issued token.
    #[serde(rename = "authenticate")]
    Authenticate { token: Vec<u8> },

    /// Request app info (installed cells, role→cell_id map). Unit variant —
    /// no `installed_app_id` field: `holochain_conductor_api`'s real
    /// `AppRequest::AppInfo` takes none, since the app is already scoped by
    /// the connection's authenticated app-interface token. A prior
    /// `{ installed_app_id: String }` shape here always failed server-side
    /// deserialization (found live via the Pulse browser proof).
    #[serde(rename = "app_info")]
    AppInfo,

    /// Call a zome function. `SignedZomeCall` (`{bytes, signature}`), NOT a
    /// flat struct — matches the conductor's real
    /// `AppRequest::CallZome(Box<ZomeCallParamsSigned>)`, where `bytes` is
    /// the map-encoded `ZomeCallParams` (cell_id/zome_name/fn_name/payload/
    /// cap_secret/provenance/nonce/expires_at — no `signature` field; that
    /// wraps `bytes` separately) and `signature` is a real Ed25519
    /// signature over `sha512(bytes)`. A prior flat 9-field
    /// `CallZomeRequestWire` here — sent even after a real `ZomeCallSigner`
    /// was wired in — discarded the signer's correctly-shaped `bytes` and
    /// resent a differently-shaped struct instead, so the signature never
    /// matched what the conductor actually hashed; confirmed live via the
    /// Pulse browser proof: every zome call failed with "Failed to
    /// deserialize request" even with a real signer installed and a
    /// correctly-computed signature. `admin_granted_signer::ZomeCallParams`
    /// builds the real inner shape `bytes` must contain.
    #[serde(rename = "call_zome")]
    CallZome(SignedZomeCall),
}

/// No longer sent over the wire (see `AppRequest::CallZome`'s doc comment)
/// — kept only because `call_zome_request_wire_roundtrip` and
/// `call_zome_request_wire_fields_use_binary_not_array_encoding` are
/// otherwise-still-valid regression coverage for the `binary_bytes`/
/// `cell_id_bytes`/`optional_binary_bytes` helper modules, which real wire
/// types (`WireRequest`, `SignedZomeCall`, ...) still use.
#[allow(dead_code)]
#[derive(Debug, Clone, Serialize, Deserialize)]
pub(crate) struct CallZomeRequestWire {
    #[serde(with = "cell_id_bytes")]
    pub cell_id: (Vec<u8>, Vec<u8>),
    pub zome_name: String,
    pub fn_name: String,
    #[serde(with = "binary_bytes")]
    pub payload: Vec<u8>,
    #[serde(with = "optional_binary_bytes")]
    pub cap_secret: Option<Vec<u8>>,
    #[serde(with = "binary_bytes")]
    pub provenance: Vec<u8>,
    #[serde(with = "binary_bytes")]
    pub signature: Vec<u8>,
    #[serde(with = "binary_bytes")]
    pub nonce: Vec<u8>,
    pub expires_at: u64,
}

/// Responses from the conductor App API.
#[derive(Debug, Clone, Deserialize)]
#[serde(tag = "type", content = "value")]
pub(crate) enum AppResponse {
    /// App info listing installed cells. `None` means no installed app
    /// matches the connection's authenticated app_id (matches
    /// `holochain_conductor_api`'s real `Option<AppInfo>`).
    #[serde(rename = "app_info")]
    AppInfo(Option<AppInfoResponse>),

    /// Successful zome call result (ExternIO bytes).
    #[serde(rename = "zome_called")]
    ZomeCalled(Vec<u8>),

    /// Error from the conductor.
    #[serde(rename = "error")]
    Error(AppError),
}

/// App info response from the conductor. Only the two fields we actually
/// use are declared — unknown fields (`status`, `agent_pub_key`,
/// `manifest`, `installed_at`, ...) are ignored by serde's default struct
/// deserialization, no `#[serde(default)]`/catch-all needed for those.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub(crate) struct AppInfoResponse {
    /// The installed app ID.
    pub installed_app_id: String,
    /// Cell info keyed by role name — a genuine MAP (`{"main": [...]}`),
    /// NOT an array of `{role_name, cells}` entries. A prior `Vec<{role_name,
    /// cells}>` shape here was a real, previously-uncaught bug (every past
    /// "verification" of this type only round-tripped self-serialized data
    /// using the same wrong shape on both ends) — found live via the Pulse
    /// browser proof: `holochain_conductor_api::AppInfo::cell_info` really
    /// is `HashMap<RoleName, Vec<CellInfo>>`, and decoding a real response
    /// against the array-shaped struct failed with "invalid type: map,
    /// expected a sequence".
    #[serde(default)]
    pub cell_info: std::collections::HashMap<String, Vec<CellInfoVariant>>,
}

/// Cell info variant — we only care about Provisioned cells.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", content = "value")]
pub(crate) enum CellInfoVariant {
    /// A provisioned (running) cell.
    #[serde(rename = "provisioned")]
    Provisioned(ProvisionedCell),
    /// Cloned cells and stem cells — we pass through but don't use them for lookup.
    #[serde(other)]
    Other,
}

/// A provisioned cell with its cell_id.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub(crate) struct ProvisionedCell {
    /// `(DnaHash, AgentPubKey)` as raw bytes.
    #[serde(with = "cell_id_bytes")]
    pub cell_id: (Vec<u8>, Vec<u8>),
}

/// Error payload from the conductor, matching
/// `holochain_conductor_api::app_interface::ExternalApiWireError` exactly —
/// a prior `{message: String}` struct here silently deserialized to an
/// empty default on every real error (found live via the Pulse browser
/// proof: `AppResponse::Error` parsed "successfully" into an empty message
/// every time, because the actual wire shape never matched at all).
#[derive(Debug, Clone, Deserialize)]
#[serde(tag = "type", content = "value", rename_all = "snake_case")]
pub(crate) enum AppError {
    InternalError(String),
    Deserialization(String),
    DnaReadError(String),
    RibosomeError(String),
    ZomeCallAuthenticationFailed(String),
    ZomeCallUnauthorized(String),
    CountersigningSessionError(String),
}

impl AppError {
    pub fn message(&self) -> &str {
        match self {
            AppError::InternalError(m)
            | AppError::Deserialization(m)
            | AppError::DnaReadError(m)
            | AppError::RibosomeError(m)
            | AppError::ZomeCallAuthenticationFailed(m)
            | AppError::ZomeCallUnauthorized(m)
            | AppError::CountersigningSessionError(m) => m,
        }
    }
}

/// A `CellId` is `(DnaHash, AgentPubKey)` as raw byte vectors.
pub(crate) type CellId = (Vec<u8>, Vec<u8>);

// ---------------------------------------------------------------------------
// Convenience encode/decode
// ---------------------------------------------------------------------------

/// Encode a value as MessagePack bytes.
///
/// This matches what Holochain's `ExternIO::encode` does internally.
///
/// # Errors
///
/// Returns [`ClientError::SerializationError`] if serialization fails.
pub fn encode<T: Serialize>(value: &T) -> Result<Vec<u8>, ClientError> {
    rmp_serde::to_vec_named(value).map_err(|e| ClientError::SerializationError(e.to_string()))
}

/// Decode MessagePack bytes into a typed value.
///
/// This matches what Holochain's `ExternIO::decode` does internally.
///
/// # Errors
///
/// Returns [`ClientError::SerializationError`] if deserialization fails.
pub fn decode<T: for<'de> Deserialize<'de>>(bytes: &[u8]) -> Result<T, ClientError> {
    rmp_serde::from_slice(bytes).map_err(|e| ClientError::SerializationError(e.to_string()))
}

/// Decode a MessagePack-encoded `#[serde(tag = "type", content = "value")]`
/// enum (`AppResponse`, and any other Holochain wire-protocol enum with the
/// same shape).
///
/// This is a thin wrapper around [`decode`] — kept as a separate name/doc
/// comment because the fact that a *direct* `rmp_serde::from_slice` decode
/// works for adjacently-tagged enums (including ones with nested raw-byte
/// fields, e.g. `AppInfoResponse`'s `AgentPubKey`/`DnaHash` bytes) is not
/// obvious and was live-verified rather than assumed. Two prior bugs were
/// misdiagnosed as "adjacently-tagged enums can't decode via rmp_serde at
/// all" and routed through generic-value intermediates (first
/// `serde_json::Value`, then `rmpv::Value`) to work around it — both were
/// unnecessary detours:
/// - The original `"invalid type: map, expected a sequence"` error (found
///   live via the Pulse browser proof) was actually caused by a mismatched
///   `content = "data"` vs. the real conductor's `content = "value"` — a
///   plain wire-shape bug, not a decoder limitation.
/// - `serde_json::Value` as an intermediate then failed separately on any
///   payload containing real byte-array data (`AgentPubKey`/`DnaHash`),
///   since JSON has no bytes type — but this was only ever a problem
///   *because* of the generic-value detour; direct `rmp_serde::from_slice`
///   decodes `Vec<u8>` fields fine regardless of whether the wire encoded
///   them as MessagePack Bin or Array (verified via isolated repro:
///   `rmp_serde`'s `Deserializer::deserialize_seq` has its own Bin-as-seq
///   compatibility handling, independent of the target field's type).
/// - `rmpv::Value` was tried next and materializes bytes fine, but its
///   `Deserializer::deserialize_enum` impl assumes serde's *default*
///   (externally-tagged or int-tagged) enum representation and is
///   incompatible with `tag`/`content` adjacent tagging — confirmed via
///   isolated repro, failing with `"invalid type: string ..., expected
///   array, map or int"` on the tag value itself.
///
/// With the `content = "value"` bug fixed, none of this is needed: `serde`'s
/// adjacently-tagged codegen talks to `rmp_serde`'s `Deserializer` directly
/// and handles the map shape (and any nested bytes) correctly on its own.
pub(crate) fn decode_tagged<T: for<'de> Deserialize<'de>>(bytes: &[u8]) -> Result<T, ClientError> {
    decode(bytes)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roundtrip_encode_decode() {
        #[derive(Debug, PartialEq, Serialize, Deserialize)]
        struct TestPayload {
            name: String,
            value: u64,
        }

        let original = TestPayload {
            name: "test".into(),
            value: 42,
        };
        let encoded = encode(&original).unwrap();
        let decoded: TestPayload = decode(&encoded).unwrap();
        assert_eq!(original, decoded);
    }

    #[test]
    fn encode_unit_produces_bytes() {
        let encoded = encode(&()).unwrap();
        assert!(!encoded.is_empty());
    }

    #[test]
    fn decode_bad_bytes_errors() {
        let result = decode::<String>(&[0xFF, 0xFF, 0xFF]);
        assert!(result.is_err());
        let err = result.unwrap_err();
        assert!(matches!(err, ClientError::SerializationError(_)));
    }

    #[test]
    fn connection_status_equality() {
        assert_eq!(ConnectionStatus::Connected, ConnectionStatus::Connected);
        assert_ne!(ConnectionStatus::Connected, ConnectionStatus::Disconnected);
        assert_eq!(
            ConnectionStatus::Error("x".into()),
            ConnectionStatus::Error("x".into())
        );
    }

    #[test]
    fn connect_config_creation() {
        let config = ConnectConfig {
            url: "ws://localhost:8888".into(),
            app_id: "mycelix-unified".into(),
            auth_token: Some(vec![1, 2, 3]),
            reconnect: None,
            request_timeout_ms: None,
        };
        assert_eq!(config.url, "ws://localhost:8888");
        assert_eq!(config.app_id, "mycelix-unified");
        assert!(config.auth_token.is_some());
    }

    #[test]
    fn app_request_serialization() {
        // Matches the real conductor shape: AppRequest::CallZome wraps a
        // SignedZomeCall{bytes, signature}, not a flat struct — see
        // AppRequest::CallZome's doc comment. Decoded via decode_tagged
        // (not serde_json::Value — SignedZomeCall's fields are genuine
        // MessagePack Bin, which serde_json::Value can't represent; see
        // decode_tagged's own doc comment for why this specific mistake is
        // documented rather than just avoided).
        let signed = SignedZomeCall::new(vec![1, 2, 3], vec![9u8; 64]).unwrap();
        let req = AppRequest::CallZome(signed);
        let bytes = rmp_serde::to_vec_named(&req).unwrap();
        assert!(!bytes.is_empty());

        #[derive(Deserialize)]
        #[serde(tag = "type", content = "value")]
        enum Decoded {
            #[serde(rename = "call_zome")]
            CallZome {
                bytes: serde_bytes::ByteBuf,
                signature: serde_bytes::ByteBuf,
            },
        }
        let decoded: Decoded = decode_tagged(&bytes).unwrap();
        let Decoded::CallZome { bytes, signature } = decoded;
        assert_eq!(bytes.into_vec(), vec![1, 2, 3]);
        assert_eq!(signature.len(), 64);
    }

    // ── Wire protocol roundtrip tests ──

    #[test]
    fn wire_request_roundtrip() {
        let req = WireRequest {
            id: 42,
            request_type: "call_zome".into(),
            data: encode(&"hello").unwrap(),
        };
        let bytes = rmp_serde::to_vec_named(&req).unwrap();
        let decoded: WireRequest = rmp_serde::from_slice(&bytes).unwrap();
        assert_eq!(decoded.id, 42);
        assert_eq!(decoded.request_type, "call_zome");
    }

    #[test]
    fn incoming_wire_message_response_roundtrip() {
        let resp = IncomingWireMessageFixture::Response {
            id: 7,
            data: Some(serde_bytes::ByteBuf::from(vec![1, 2, 3])),
        };
        let bytes = rmp_serde::to_vec_named(&resp).unwrap();
        let decoded: IncomingWireMessage = rmp_serde::from_slice(&bytes).unwrap();
        assert!(matches!(
            decoded,
            IncomingWireMessage::Response { id: 7, data: Some(data) } if data == vec![1, 2, 3]
        ));
    }

    #[test]
    fn incoming_wire_message_response_with_error_payload() {
        // Conductor errors travel inside the response's `data` as an
        // encoded `AppResponse::Error`, not as a separate wire-envelope
        // field — the envelope itself has no error concept.
        let error_payload = rmp_serde::to_vec_named(&AppResponseFixture::Error(
            AppErrorFixture::RibosomeError("zome function not found".into()),
        ))
        .unwrap();
        let resp = IncomingWireMessageFixture::Response {
            id: 1,
            data: Some(serde_bytes::ByteBuf::from(error_payload)),
        };
        let bytes = rmp_serde::to_vec_named(&resp).unwrap();
        let decoded: IncomingWireMessage = rmp_serde::from_slice(&bytes).unwrap();
        let IncomingWireMessage::Response {
            data: Some(data), ..
        } = decoded
        else {
            panic!("expected a Response with data");
        };
        let app_response: AppResponse = decode_tagged(&data).unwrap();
        assert!(matches!(
            app_response,
            AppResponse::Error(e) if e.message() == "zome function not found"
        ));
    }

    #[derive(Serialize)]
    #[serde(tag = "type", rename_all = "snake_case")]
    enum IncomingWireMessageFixture {
        Response {
            id: u64,
            data: Option<serde_bytes::ByteBuf>,
        },
    }

    #[derive(Serialize)]
    #[serde(tag = "type", content = "value")]
    enum AppResponseFixture {
        #[serde(rename = "error")]
        Error(AppErrorFixture),
    }

    #[derive(Serialize)]
    #[serde(tag = "type", content = "value", rename_all = "snake_case")]
    enum AppErrorFixture {
        RibosomeError(String),
    }

    #[test]
    fn call_zome_request_wire_roundtrip() {
        let req = CallZomeRequestWire {
            cell_id: (vec![0xAB; 39], vec![0xCD; 39]),
            zome_name: "proposals".into(),
            fn_name: "list_active_proposals".into(),
            payload: encode(&()).unwrap(),
            cap_secret: None,
            provenance: vec![0xCD; 39],
            signature: vec![0; 64],
            nonce: vec![0xFF; 32],
            expires_at: 1711900800_000_000, // microseconds
        };
        let bytes = rmp_serde::to_vec_named(&req).unwrap();
        let decoded: CallZomeRequestWire = rmp_serde::from_slice(&bytes).unwrap();
        assert_eq!(decoded.zome_name, "proposals");
        assert_eq!(decoded.fn_name, "list_active_proposals");
        assert_eq!(decoded.cell_id.0.len(), 39);
        assert_eq!(decoded.signature.len(), 64);
        assert_eq!(decoded.nonce.len(), 32);
    }

    #[test]
    fn call_zome_request_wire_fields_use_binary_not_array_encoding() {
        // A round-trip test alone can't catch this: `binary_bytes::deserialize`
        // deliberately also accepts legacy array-encoded input (for reads
        // during migration), so encode-as-array/decode-as-array would still
        // round-trip "successfully" while sending the wrong wire shape to a
        // real conductor. Assert the actual encoded bytes instead.
        let req = CallZomeRequestWire {
            cell_id: (vec![0xAB; 39], vec![0xCD; 39]),
            zome_name: "z".into(),
            fn_name: "f".into(),
            payload: vec![1, 2, 3],
            cap_secret: None,
            provenance: vec![0xCD; 39],
            signature: vec![7; 64],
            nonce: vec![0xFF; 32],
            expires_at: 0,
        };
        let bytes = rmp_serde::to_vec_named(&req).unwrap();
        // A 39-byte binary value is encoded as bin8 (0xc4, len, ...bytes);
        // a 39-element array would start with 0xdc/0xc9-style array markers
        // instead. Every raw-byte field must appear as a genuine `bin` run
        // somewhere in the payload, not as 39/64/32 individual fixints.
        let bin8 = |len: u8| [0xc4u8, len];
        assert!(
            bytes.windows(2).any(|w| w == bin8(39)),
            "expected at least one bin8-encoded 39-byte value (cell_id/provenance)"
        );
        assert!(
            bytes.windows(2).any(|w| w == bin8(64)),
            "expected a bin8-encoded 64-byte signature"
        );
        assert!(
            bytes.windows(2).any(|w| w == bin8(32)),
            "expected a bin8-encoded 32-byte nonce"
        );
    }

    #[test]
    fn app_request_authenticate() {
        let req = AppRequest::Authenticate {
            token: vec![1, 2, 3, 4],
        };
        let bytes = rmp_serde::to_vec_named(&req).unwrap();
        assert!(!bytes.is_empty());
    }

    #[test]
    fn app_request_app_info_is_a_unit_variant_on_the_wire() {
        // Real regression coverage, not just "doesn't panic": AppInfo must
        // serialize with no `installed_app_id` field at all, matching
        // holochain_conductor_api's real unit variant. A prior struct-variant
        // shape here always failed server-side deserialization.
        let req = AppRequest::AppInfo;
        let bytes = rmp_serde::to_vec_named(&req).unwrap();
        let value: serde_json::Value = rmp_serde::from_slice(&bytes).unwrap();
        assert_eq!(value.get("type").and_then(|v| v.as_str()), Some("app_info"));
        assert!(
            value.get("value").is_none_or(|v| v.is_null()),
            "app_info must not carry a value payload, got: {value:?}"
        );
    }

    #[test]
    fn app_response_zome_called_deserialize() {
        // Simulate a real conductor response: {"type": "zome_called", "value": [msgpack bytes]}.
        // A prior "data" key here (matching the wrong tag on AppResponse
        // itself) let this test pass while masking a real deserialization
        // failure against an actual conductor.
        let response_data = encode(&"success").unwrap();
        let resp_obj = serde_json::json!({
            "type": "zome_called",
            "value": response_data,
        });
        let bytes = rmp_serde::to_vec_named(&resp_obj).unwrap();
        let decoded: AppResponse =
            rmp_serde::from_slice(&bytes).expect("must deserialize a real zome_called response");
        assert!(matches!(decoded, AppResponse::ZomeCalled(_)));
    }

    #[test]
    fn app_info_response_deserialize() {
        let mut cell_info = std::collections::HashMap::new();
        cell_info.insert(
            "governance".to_string(),
            vec![CellInfoVariant::Provisioned(ProvisionedCell {
                cell_id: (vec![0u8; 39], vec![1u8; 39]),
            })],
        );
        let resp = AppInfoResponse {
            installed_app_id: "test-app".into(),
            cell_info,
        };
        let bytes = rmp_serde::to_vec_named(&resp).unwrap();
        let decoded: AppInfoResponse = rmp_serde::from_slice(&bytes).unwrap();
        assert_eq!(decoded.installed_app_id, "test-app");
        assert_eq!(decoded.cell_info.len(), 1);
        assert!(decoded.cell_info.contains_key("governance"));
    }

    #[test]
    fn decode_tagged_survives_real_bin_encoded_agent_pub_key_bytes() {
        // Regression test for a real bug found live via the Pulse browser
        // proof: decode_tagged's now-removed serde_json::Value intermediate
        // step had no Binary variant, so it failed outright — with
        // "invalid type: byte array, expected any valid JSON value" — on the
        // very first real app_info response, which always carries raw
        // AgentPubKey/DnaHash bytes as genuine MessagePack Bin (not the
        // Array-of-ints encoding `rmp_serde::to_vec_named` produces for a
        // plain `Vec<u8>` field — that distinction is exactly why this test
        // has to force real Bin encoding via `serde_bytes::ByteBuf` in the
        // fixture, not just reuse ProvisionedCell's own Serialize impl).
        //
        // See decode_tagged's doc comment: the actual fix was removing the
        // generic-value detour entirely, not swapping to a different one.
        #[derive(Serialize)]
        #[serde(tag = "type", content = "value")]
        enum TaggedFixture {
            #[serde(rename = "app_info")]
            AppInfo(Option<Payload>),
        }
        #[derive(Serialize)]
        struct Payload {
            installed_app_id: String,
            // A real map, matching the actual conductor shape — see
            // AppInfoResponse::cell_info's doc comment.
            cell_info: std::collections::HashMap<String, Vec<PayloadCellVariant>>,
        }
        #[derive(Serialize)]
        #[serde(tag = "type", content = "value")]
        enum PayloadCellVariant {
            #[serde(rename = "provisioned")]
            Provisioned(PayloadProvisionedCell),
        }
        #[derive(Serialize)]
        struct PayloadProvisionedCell {
            // ByteBuf forces genuine MessagePack Bin encoding, matching what
            // a real conductor sends for AgentPubKey/DnaHash.
            cell_id: (serde_bytes::ByteBuf, serde_bytes::ByteBuf),
        }

        let mut cell_info = std::collections::HashMap::new();
        cell_info.insert(
            "mail".to_string(),
            vec![PayloadCellVariant::Provisioned(PayloadProvisionedCell {
                cell_id: (
                    serde_bytes::ByteBuf::from(vec![0xAB; 39]),
                    serde_bytes::ByteBuf::from(vec![0xCD; 39]),
                ),
            })],
        );
        let fixture = TaggedFixture::AppInfo(Some(Payload {
            installed_app_id: "mycelix_mail".into(),
            cell_info,
        }));

        let bytes = rmp_serde::to_vec_named(&fixture).unwrap();
        // Decode into the REAL production type (plain Vec<u8> fields) to
        // prove the compat path: rmp_serde's Deserializer accepts Bin-marker
        // data for a plain Vec<u8> field transparently.
        let decoded: AppResponse =
            decode_tagged(&bytes).expect("must survive real Bin-encoded agent/DNA bytes");
        match decoded {
            AppResponse::AppInfo(Some(info)) => {
                assert_eq!(info.installed_app_id, "mycelix_mail");
                let cells = info.cell_info.get("mail").expect("mail role present");
                if let CellInfoVariant::Provisioned(cell) = &cells[0] {
                    assert_eq!(cell.cell_id.0, vec![0xABu8; 39]);
                    assert_eq!(cell.cell_id.1, vec![0xCDu8; 39]);
                } else {
                    panic!("expected Provisioned cell");
                }
            }
            other => panic!("expected AppInfo(Some(_)), got {other:?}"),
        }
    }

    // ── Domain type roundtrip tests (verify portal types match zome expectations) ──

    #[test]
    fn encode_complex_struct() {
        #[derive(Debug, PartialEq, Serialize, Deserialize)]
        struct Proposal {
            id: String,
            title: String,
            status: String,
            votes_for: u32,
            votes_against: u32,
        }

        let proposal = Proposal {
            id: "MIP-042".into(),
            title: "Community solar garden".into(),
            status: "Active".into(),
            votes_for: 34,
            votes_against: 8,
        };

        let encoded = encode(&proposal).unwrap();
        let decoded: Proposal = decode(&encoded).unwrap();
        assert_eq!(decoded.id, "MIP-042");
        assert_eq!(decoded.votes_for, 34);
    }

    #[test]
    fn encode_nested_enum() {
        #[derive(Debug, PartialEq, Serialize, Deserialize)]
        enum Status {
            Active,
            Draft,
            Executed,
        }

        #[derive(Debug, PartialEq, Serialize, Deserialize)]
        struct Item {
            status: Status,
        }

        let item = Item {
            status: Status::Active,
        };
        let encoded = encode(&item).unwrap();
        let decoded: Item = decode(&encoded).unwrap();
        assert_eq!(decoded.status, Status::Active);
    }

    #[test]
    fn encode_vec_of_structs() {
        #[derive(Debug, PartialEq, Serialize, Deserialize)]
        struct Vote {
            voter: String,
            choice: String,
            weight: f64,
        }

        let votes = vec![
            Vote {
                voter: "alice".into(),
                choice: "for".into(),
                weight: 1.5,
            },
            Vote {
                voter: "bob".into(),
                choice: "against".into(),
                weight: 0.8,
            },
        ];

        let encoded = encode(&votes).unwrap();
        let decoded: Vec<Vote> = decode(&encoded).unwrap();
        assert_eq!(decoded.len(), 2);
        assert_eq!(decoded[0].voter, "alice");
        assert!((decoded[1].weight - 0.8).abs() < f64::EPSILON);
    }

    #[test]
    fn encode_optional_fields() {
        #[derive(Debug, PartialEq, Serialize, Deserialize)]
        struct Record {
            id: String,
            parent: Option<String>,
            tags: Vec<String>,
        }

        let with_parent = Record {
            id: "1".into(),
            parent: Some("0".into()),
            tags: vec!["a".into()],
        };
        let without = Record {
            id: "2".into(),
            parent: None,
            tags: vec![],
        };

        let e1 = encode(&with_parent).unwrap();
        let e2 = encode(&without).unwrap();
        let d1: Record = decode(&e1).unwrap();
        let d2: Record = decode(&e2).unwrap();

        assert_eq!(d1.parent, Some("0".into()));
        assert_eq!(d2.parent, None);
        assert!(d2.tags.is_empty());
    }

    #[test]
    fn mock_transport_returns_not_connected() {
        use crate::MockTransport;
        use crate::transport::HolochainTransport;

        let mock = MockTransport::new();
        assert_eq!(mock.status(), ConnectionStatus::Disconnected);
    }
}
