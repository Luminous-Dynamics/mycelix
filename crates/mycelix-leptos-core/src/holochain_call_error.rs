// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Typed zome-call failure context for shared Leptos providers.
//!
//! This type preserves the low-level [`ClientError`], the exact
//! role/zome/function identity of the attempted call, and the provider phase
//! that admitted the failure. It deliberately does not infer stronger protocol
//! semantics from human-readable error text.

use std::error::Error;
use std::fmt;

use mycelix_leptos_client::{ClientError, ConductorError, ConductorErrorKind};

/// Exact provider phase in which a zome call failed.
///
/// The phase is authoritative because the provider itself assigns it at the
/// operation boundary; unlike an error message, it is not inferred from text.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HolochainCallPhase {
    /// Provider admission before payload encoding or transport dispatch.
    Admission,
    /// Serialization of the input into ExternIO/MessagePack.
    Encode,
    /// Signed transport dispatch and conductor response acquisition.
    Transport,
    /// Deserialization of a successful response payload into the requested type.
    Decode,
}

impl HolochainCallPhase {
    pub const fn label(self) -> &'static str {
        match self {
            Self::Admission => "admission",
            Self::Encode => "encode",
            Self::Transport => "transport",
            Self::Decode => "decode",
        }
    }
}

/// A failed Holochain zome call with the exact call target and provider phase
/// retained alongside the structured shared-client error.
#[derive(Debug)]
pub struct HolochainCallError {
    phase: HolochainCallPhase,
    role: String,
    zome: String,
    function: String,
    source: ClientError,
}

impl HolochainCallError {
    pub fn new(
        phase: HolochainCallPhase,
        role: impl Into<String>,
        zome: impl Into<String>,
        function: impl Into<String>,
        source: ClientError,
    ) -> Self {
        Self {
            phase,
            role: role.into(),
            zome: zome.into(),
            function: function.into(),
            source,
        }
    }

    pub fn phase(&self) -> HolochainCallPhase {
        self.phase
    }

    pub fn role(&self) -> &str {
        &self.role
    }

    pub fn zome(&self) -> &str {
        &self.zome
    }

    pub fn function(&self) -> &str {
        &self.function
    }

    pub fn client_error(&self) -> &ClientError {
        &self.source
    }

    pub fn into_client_error(self) -> ClientError {
        self.source
    }

    pub fn conductor_error(&self) -> Option<&ConductorError> {
        self.source.conductor_error()
    }

    pub fn conductor_kind(&self) -> Option<ConductorErrorKind> {
        self.source.conductor_kind()
    }
}

impl fmt::Display for HolochainCallError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            formatter,
            "Zome call {}.{}.{} failed during {}: {}",
            self.role,
            self.zome,
            self.function,
            self.phase.label(),
            self.source
        )
    }
}

impl Error for HolochainCallError {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        Some(&self.source)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn call_identity_and_phase_are_retained_exactly() {
        let error = HolochainCallError::new(
            HolochainCallPhase::Transport,
            "personal",
            "identity_vault",
            "set_profile_view_if_current",
            ClientError::Timeout(30_000),
        );

        assert_eq!(error.phase(), HolochainCallPhase::Transport);
        assert_eq!(error.role(), "personal");
        assert_eq!(error.zome(), "identity_vault");
        assert_eq!(error.function(), "set_profile_view_if_current");
        assert!(matches!(error.client_error(), ClientError::Timeout(30_000)));
    }

    #[test]
    fn same_serialization_class_does_not_erase_provider_phase() {
        let encode = HolochainCallError::new(
            HolochainCallPhase::Encode,
            "personal",
            "identity_vault",
            "set_profile_view_if_current",
            ClientError::SerializationError("codec".into()),
        );
        let decode = HolochainCallError::new(
            HolochainCallPhase::Decode,
            "personal",
            "identity_vault",
            "set_profile_view_if_current",
            ClientError::SerializationError("codec".into()),
        );

        assert_eq!(encode.phase(), HolochainCallPhase::Encode);
        assert_eq!(decode.phase(), HolochainCallPhase::Decode);
        assert_ne!(encode.phase(), decode.phase());
    }

    #[test]
    fn structured_conductor_kind_survives_provider_context() {
        let source = ClientError::Conductor(ConductorError {
            kind: ConductorErrorKind::ZomeCallUnauthorized,
            message: "capability denied".into(),
        });
        let error = HolochainCallError::new(
            HolochainCallPhase::Transport,
            "personal",
            "data_preferences",
            "set_preference_view_if_current",
            source,
        );

        assert_eq!(
            error.conductor_kind(),
            Some(ConductorErrorKind::ZomeCallUnauthorized)
        );
        assert_eq!(
            error.conductor_error().map(|error| error.message.as_str()),
            Some("capability denied")
        );
    }

    #[test]
    fn ribosome_message_does_not_become_a_stronger_provider_class() {
        let source = ClientError::Conductor(ConductorError {
            kind: ConductorErrorKind::Ribosome,
            message: "source chain head moved".into(),
        });
        let error = HolochainCallError::new(
            HolochainCallPhase::Transport,
            "personal",
            "identity_vault",
            "set_profile_view_if_current",
            source,
        );

        assert_eq!(error.conductor_kind(), Some(ConductorErrorKind::Ribosome));
        assert_eq!(
            error.conductor_error().map(|error| error.message.as_str()),
            Some("source chain head moved")
        );
    }

    #[test]
    fn display_includes_phase_and_call_identity_without_erasing_source() {
        let error = HolochainCallError::new(
            HolochainCallPhase::Admission,
            "personal",
            "identity_vault",
            "get_my_profile_view",
            ClientError::NotConnected,
        );

        assert_eq!(
            error.to_string(),
            "Zome call personal.identity_vault.get_my_profile_view failed during admission: Not connected to conductor"
        );
        assert!(error.source().is_some());
    }

    #[test]
    fn phase_labels_are_stable() {
        assert_eq!(HolochainCallPhase::Admission.label(), "admission");
        assert_eq!(HolochainCallPhase::Encode.label(), "encode");
        assert_eq!(HolochainCallPhase::Transport.label(), "transport");
        assert_eq!(HolochainCallPhase::Decode.label(), "decode");
    }
}
