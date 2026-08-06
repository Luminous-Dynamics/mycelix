// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Error types for the Holochain browser client.

/// Errors that can occur during Holochain client operations.
#[derive(Debug, thiserror::Error)]
pub enum ClientError {
    /// No active connection to the conductor.
    #[error("Not connected to conductor")]
    NotConnected,

    /// WebSocket connection could not be established.
    #[error("Connection failed: {0}")]
    ConnectionFailed(String),

    /// MessagePack serialization or deserialization failed.
    #[error("Serialization error: {0}")]
    SerializationError(String),

    /// The conductor returned an error for the zome call.
    #[error("Zome call failed: {0}")]
    ZomeCallFailed(String),

    /// The zome call did not complete within the timeout period.
    #[error("Timeout after {0}ms")]
    Timeout(u32),

    /// A WebSocket-level error occurred.
    #[error("WebSocket error: {0}")]
    WebSocketError(String),

    /// A response was received for an unknown request ID.
    #[error("Unknown request ID: {0}")]
    UnknownRequestId(u64),

    /// The conductor sent a response that could not be parsed.
    #[error("Invalid response: {0}")]
    InvalidResponse(String),

    /// Authentication with the conductor failed.
    #[error("Authentication failed: {0}")]
    AuthenticationFailed(String),

    /// No authorized signer has been configured for zome calls.
    #[error("Zome call signing is unavailable: {0}")]
    SigningUnavailable(String),

    /// A signer returned a malformed or explicitly unsigned call.
    #[error("Invalid zome call signature: {0}")]
    InvalidSignature(String),

    /// The runtime could not provide cryptographically secure random bytes.
    #[error("Secure randomness unavailable: {0}")]
    SecureRandomUnavailable(String),

    /// The requested role name was not found in the app info.
    #[error("Unknown role: {0}")]
    UnknownRole(String),
}
