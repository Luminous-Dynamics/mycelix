// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Error types for the Holochain browser client.

use crate::conductor_error::{ConductorError, ConductorErrorKind};

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

    /// Structured error returned by the conductor application API.
    #[error(transparent)]
    Conductor(#[from] ConductorError),

    /// Legacy unstructured zome-call error retained while transports migrate
    /// to [`ClientError::Conductor`].
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

impl ClientError {
    /// Return the structured conductor error when this client failure came
    /// directly from the conductor application API.
    pub fn conductor_error(&self) -> Option<&ConductorError> {
        match self {
            Self::Conductor(error) => Some(error),
            _ => None,
        }
    }

    /// Return only the broad machine-readable conductor error class.
    ///
    /// This never derives a more specific class from the human-readable error
    /// payload.
    pub fn conductor_kind(&self) -> Option<ConductorErrorKind> {
        self.conductor_error().map(|error| error.kind)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn structured_conductor_error_roundtrips_through_client_error() {
        let conductor = ConductorError {
            kind: ConductorErrorKind::ZomeCallUnauthorized,
            message: "cap grant rejected".into(),
        };
        let error: ClientError = conductor.clone().into();

        assert_eq!(
            error.conductor_kind(),
            Some(ConductorErrorKind::ZomeCallUnauthorized)
        );
        assert_eq!(error.conductor_error(), Some(&conductor));
        assert_eq!(
            error.to_string(),
            "Conductor ZomeCallUnauthorized error: cap grant rejected"
        );
    }

    #[test]
    fn unstructured_errors_do_not_claim_a_conductor_class() {
        let legacy = ClientError::ZomeCallFailed("legacy text".into());
        let transport = ClientError::WebSocketError("closed".into());

        assert_eq!(legacy.conductor_kind(), None);
        assert_eq!(transport.conductor_kind(), None);
    }

    #[test]
    fn ribosome_class_does_not_strengthen_message_into_head_moved() {
        let error: ClientError = ConductorError {
            kind: ConductorErrorKind::Ribosome,
            message: "source chain head moved".into(),
        }
        .into();

        assert_eq!(error.conductor_kind(), Some(ConductorErrorKind::Ribosome));
    }
}
