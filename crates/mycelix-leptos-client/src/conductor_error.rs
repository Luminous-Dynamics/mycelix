// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Structured conductor error classes preserved from the Holochain app wire.
//!
//! Holochain 0.6 exposes [`ExternalApiWireError`](https://docs.rs/holochain_conductor_api/0.6)
//! as a small tagged enum whose payloads are human-readable strings.  This
//! module preserves the enum tag as machine-readable evidence without claiming
//! that details hidden inside those strings (for example a specific source-chain
//! head movement) have a structured protocol representation.

use crate::types::AppError;

/// Stable, machine-readable class of an error returned by the conductor app API.
///
/// These variants intentionally mirror the broad classes on Holochain 0.6's
/// `ExternalApiWireError`.  They are not a taxonomy of every underlying
/// Holochain error and must not be strengthened into one by inspecting the
/// human-readable message payload.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ConductorErrorKind {
    Internal,
    Deserialization,
    DnaRead,
    Ribosome,
    ZomeCallAuthenticationFailed,
    ZomeCallUnauthorized,
    CountersigningSession,
}

impl ConductorErrorKind {
    pub const fn label(self) -> &'static str {
        match self {
            Self::Internal => "internal",
            Self::Deserialization => "deserialization",
            Self::DnaRead => "dna-read",
            Self::Ribosome => "ribosome",
            Self::ZomeCallAuthenticationFailed => "zome-call-authentication-failed",
            Self::ZomeCallUnauthorized => "zome-call-unauthorized",
            Self::CountersigningSession => "countersigning-session",
        }
    }
}

/// Structured conductor failure retaining both the wire class and its display
/// message.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ConductorError {
    pub kind: ConductorErrorKind,
    pub message: String,
}

impl ConductorError {
    pub(crate) fn from_app_error(error: &AppError) -> Self {
        let (kind, message) = match error {
            AppError::InternalError(message) => (ConductorErrorKind::Internal, message),
            AppError::Deserialization(message) => {
                (ConductorErrorKind::Deserialization, message)
            }
            AppError::DnaReadError(message) => (ConductorErrorKind::DnaRead, message),
            AppError::RibosomeError(message) => (ConductorErrorKind::Ribosome, message),
            AppError::ZomeCallAuthenticationFailed(message) => (
                ConductorErrorKind::ZomeCallAuthenticationFailed,
                message,
            ),
            AppError::ZomeCallUnauthorized(message) => {
                (ConductorErrorKind::ZomeCallUnauthorized, message)
            }
            AppError::CountersigningSessionError(message) => {
                (ConductorErrorKind::CountersigningSession, message)
            }
        };

        Self {
            kind,
            message: message.clone(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_wire_variant_preserves_its_class() {
        let cases = [
            (
                AppError::InternalError("internal".into()),
                ConductorErrorKind::Internal,
            ),
            (
                AppError::Deserialization("decode".into()),
                ConductorErrorKind::Deserialization,
            ),
            (
                AppError::DnaReadError("dna".into()),
                ConductorErrorKind::DnaRead,
            ),
            (
                AppError::RibosomeError("ribosome".into()),
                ConductorErrorKind::Ribosome,
            ),
            (
                AppError::ZomeCallAuthenticationFailed("auth".into()),
                ConductorErrorKind::ZomeCallAuthenticationFailed,
            ),
            (
                AppError::ZomeCallUnauthorized("unauthorized".into()),
                ConductorErrorKind::ZomeCallUnauthorized,
            ),
            (
                AppError::CountersigningSessionError("countersigning".into()),
                ConductorErrorKind::CountersigningSession,
            ),
        ];

        for (wire, expected) in cases {
            let structured = ConductorError::from_app_error(&wire);
            assert_eq!(structured.kind, expected);
        }
    }

    #[test]
    fn display_message_is_retained_without_becoming_protocol_authority() {
        let wire = AppError::RibosomeError("source chain head moved".into());
        let structured = ConductorError::from_app_error(&wire);

        assert_eq!(structured.kind, ConductorErrorKind::Ribosome);
        assert_eq!(structured.message, "source chain head moved");
    }

    #[test]
    fn labels_are_stable_and_not_message_derived() {
        assert_eq!(ConductorErrorKind::Internal.label(), "internal");
        assert_eq!(ConductorErrorKind::Ribosome.label(), "ribosome");
        assert_eq!(
            ConductorErrorKind::ZomeCallUnauthorized.label(),
            "zome-call-unauthorized"
        );
    }
}
