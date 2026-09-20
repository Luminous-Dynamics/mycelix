// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Typed zome-call failure context for shared Leptos providers.
//!
//! This type preserves both the low-level [`ClientError`] and the exact
//! role/zome/function identity of the attempted call. It deliberately does not
//! infer stronger protocol semantics from human-readable error text.

use std::error::Error;
use std::fmt;

use mycelix_leptos_client::{ClientError, ConductorError, ConductorErrorKind};

/// A failed Holochain zome call with the exact call target retained alongside
/// the structured shared-client error.
#[derive(Debug)]
pub struct HolochainCallError {
    role: String,
    zome: String,
    function: String,
    source: ClientError,
}

impl HolochainCallError {
    pub fn new(
        role: impl Into<String>,
        zome: impl Into<String>,
        function: impl Into<String>,
        source: ClientError,
    ) -> Self {
        Self {
            role: role.into(),
            zome: zome.into(),
            function: function.into(),
            source,
        }
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
            "Zome call {}.{}.{} failed: {}",
            self.role, self.zome, self.function, self.source
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
    fn call_identity_is_retained_exactly() {
        let error = HolochainCallError::new(
            "personal",
            "identity_vault",
            "set_profile_view_if_current",
            ClientError::Timeout(30_000),
        );

        assert_eq!(error.role(), "personal");
        assert_eq!(error.zome(), "identity_vault");
        assert_eq!(error.function(), "set_profile_view_if_current");
        assert!(matches!(error.client_error(), ClientError::Timeout(30_000)));
    }

    #[test]
    fn structured_conductor_kind_survives_provider_context() {
        let source = ClientError::Conductor(ConductorError {
            kind: ConductorErrorKind::ZomeCallUnauthorized,
            message: "capability denied".into(),
        });
        let error = HolochainCallError::new(
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
    fn display_includes_call_identity_without_erasing_source() {
        let error = HolochainCallError::new(
            "personal",
            "identity_vault",
            "get_my_profile_view",
            ClientError::NotConnected,
        );

        assert_eq!(
            error.to_string(),
            "Zome call personal.identity_vault.get_my_profile_view failed: Not connected to conductor"
        );
        assert!(error.source().is_some());
    }
}
