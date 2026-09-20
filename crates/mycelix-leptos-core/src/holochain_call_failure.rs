// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Lossless failure-kind projection for typed Holochain call errors.
//!
//! This module classifies only information already represented by typed
//! [`ClientError`] variants. It deliberately does not infer retryability,
//! transaction outcome, or stronger Holochain subtypes from display strings.

use mycelix_leptos_client::{ClientError, ConductorErrorKind};

use crate::holochain_call_error::HolochainCallError;

/// Machine-readable failure class derived only from existing typed evidence.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HolochainCallFailureKind {
    ConnectionUnavailable,
    Timeout,
    SigningUnavailable,
    InvalidSignature,
    AuthenticationFailed,
    Serialization,
    TransportProtocol,
    SecureRandomnessUnavailable,
    UnknownRole,
    LegacyUnstructuredZomeCall,
    Conductor(ConductorErrorKind),
}

impl HolochainCallFailureKind {
    pub const fn label(self) -> &'static str {
        match self {
            Self::ConnectionUnavailable => "connection-unavailable",
            Self::Timeout => "timeout",
            Self::SigningUnavailable => "signing-unavailable",
            Self::InvalidSignature => "invalid-signature",
            Self::AuthenticationFailed => "authentication-failed",
            Self::Serialization => "serialization",
            Self::TransportProtocol => "transport-protocol",
            Self::SecureRandomnessUnavailable => "secure-randomness-unavailable",
            Self::UnknownRole => "unknown-role",
            Self::LegacyUnstructuredZomeCall => "legacy-unstructured-zome-call",
            Self::Conductor(_) => "conductor",
        }
    }
}

impl HolochainCallError {
    /// Classify the typed source without inspecting its human-readable payload.
    pub fn failure_kind(&self) -> HolochainCallFailureKind {
        match self.client_error() {
            ClientError::NotConnected | ClientError::ConnectionFailed(_) => {
                HolochainCallFailureKind::ConnectionUnavailable
            }
            ClientError::Timeout(_) => HolochainCallFailureKind::Timeout,
            ClientError::SigningUnavailable(_) => HolochainCallFailureKind::SigningUnavailable,
            ClientError::InvalidSignature(_) => HolochainCallFailureKind::InvalidSignature,
            ClientError::AuthenticationFailed(_) => {
                HolochainCallFailureKind::AuthenticationFailed
            }
            ClientError::SerializationError(_) => HolochainCallFailureKind::Serialization,
            ClientError::WebSocketError(_)
            | ClientError::UnknownRequestId(_)
            | ClientError::InvalidResponse(_) => HolochainCallFailureKind::TransportProtocol,
            ClientError::SecureRandomUnavailable(_) => {
                HolochainCallFailureKind::SecureRandomnessUnavailable
            }
            ClientError::UnknownRole(_) => HolochainCallFailureKind::UnknownRole,
            ClientError::ZomeCallFailed(_) => {
                HolochainCallFailureKind::LegacyUnstructuredZomeCall
            }
            ClientError::Conductor(error) => HolochainCallFailureKind::Conductor(error.kind),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{HolochainCallPhase, HolochainCallError};
    use mycelix_leptos_client::ConductorError;

    fn wrapped(source: ClientError) -> HolochainCallError {
        HolochainCallError::new(
            HolochainCallPhase::Transport,
            "personal",
            "identity_vault",
            "set_profile_view_if_current",
            source,
        )
    }

    #[test]
    fn every_client_error_variant_maps_without_message_parsing() {
        let cases = [
            (
                ClientError::NotConnected,
                HolochainCallFailureKind::ConnectionUnavailable,
            ),
            (
                ClientError::ConnectionFailed("offline".into()),
                HolochainCallFailureKind::ConnectionUnavailable,
            ),
            (ClientError::Timeout(30_000), HolochainCallFailureKind::Timeout),
            (
                ClientError::SigningUnavailable("missing signer".into()),
                HolochainCallFailureKind::SigningUnavailable,
            ),
            (
                ClientError::InvalidSignature("bad signature".into()),
                HolochainCallFailureKind::InvalidSignature,
            ),
            (
                ClientError::AuthenticationFailed("bad token".into()),
                HolochainCallFailureKind::AuthenticationFailed,
            ),
            (
                ClientError::SerializationError("codec".into()),
                HolochainCallFailureKind::Serialization,
            ),
            (
                ClientError::WebSocketError("closed".into()),
                HolochainCallFailureKind::TransportProtocol,
            ),
            (
                ClientError::UnknownRequestId(7),
                HolochainCallFailureKind::TransportProtocol,
            ),
            (
                ClientError::InvalidResponse("bad response".into()),
                HolochainCallFailureKind::TransportProtocol,
            ),
            (
                ClientError::SecureRandomUnavailable("no crypto".into()),
                HolochainCallFailureKind::SecureRandomnessUnavailable,
            ),
            (
                ClientError::UnknownRole("personal".into()),
                HolochainCallFailureKind::UnknownRole,
            ),
            (
                ClientError::ZomeCallFailed("legacy".into()),
                HolochainCallFailureKind::LegacyUnstructuredZomeCall,
            ),
        ];

        for (source, expected) in cases {
            assert_eq!(wrapped(source).failure_kind(), expected);
        }
    }

    #[test]
    fn conductor_class_is_preserved_exactly() {
        let error = wrapped(ClientError::Conductor(ConductorError {
            kind: ConductorErrorKind::ZomeCallUnauthorized,
            message: "denied".into(),
        }));

        assert_eq!(
            error.failure_kind(),
            HolochainCallFailureKind::Conductor(
                ConductorErrorKind::ZomeCallUnauthorized
            )
        );
    }

    #[test]
    fn ribosome_message_cannot_strengthen_failure_kind() {
        let error = wrapped(ClientError::Conductor(ConductorError {
            kind: ConductorErrorKind::Ribosome,
            message: "source chain head moved".into(),
        }));

        assert_eq!(
            error.failure_kind(),
            HolochainCallFailureKind::Conductor(ConductorErrorKind::Ribosome)
        );
    }

    #[test]
    fn labels_do_not_claim_retryability() {
        assert_eq!(
            HolochainCallFailureKind::ConnectionUnavailable.label(),
            "connection-unavailable"
        );
        assert_eq!(
            HolochainCallFailureKind::Conductor(ConductorErrorKind::Ribosome).label(),
            "conductor"
        );
    }
}
