// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Typed inspection envelope for the task-first Mycelix `Me` surface.
//!
//! These types preserve provider-owned facts and safe handoff targets. They do
//! not prove identity, calculate security/trust scores, authorize capabilities,
//! mutate recovery configuration, or execute destructive controls.

use crate::freshness::FreshnessLevel;

/// Stable user-centric sections from the Me v1 experience contract.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MeSection {
    Identity,
    SecurityRecovery,
    Devices,
    Permissions,
    PreferencesAccessibility,
    DataSync,
    TrustReputation,
}

impl MeSection {
    pub fn label(self) -> &'static str {
        match self {
            Self::Identity => "Identity",
            Self::SecurityRecovery => "Security & recovery",
            Self::Devices => "Devices",
            Self::Permissions => "Permissions",
            Self::PreferencesAccessibility => "Preferences & accessibility",
            Self::DataSync => "Data & sync",
            Self::TrustReputation => "Trust & reputation",
        }
    }
}

/// What the provider can currently establish about one displayed fact.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum MeFactState {
    /// A provider-supplied value is known. `Known` does not by itself mean
    /// verified, valid, authorized, secure, current, or sufficient.
    Known { value: String },
    /// The relevant fact cannot currently be established.
    Unknown,
    /// The source needed to establish the fact cannot presently be accessed.
    Unavailable,
}

impl MeFactState {
    pub fn label(&self) -> &str {
        match self {
            Self::Known { value } => value,
            Self::Unknown => "Unknown",
            Self::Unavailable => "Unavailable",
        }
    }
}

/// Safe handoff exposed by a Me item.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum MeTarget {
    Navigate { href: String },
    /// Open a provider-owned preview for a consequential control. The shared
    /// Me surface does not execute the control directly.
    PreviewControl {
        control_kind: String,
        control_ref: String,
    },
}

/// One provider-owned fact/control projected into Me.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct MeItem {
    pub id: String,
    /// Source provenance, not a trust/authority score.
    pub provider_id: String,
    pub section: MeSection,
    pub label: String,
    pub state: MeFactState,
    pub detail: Option<String>,
    pub freshness: Option<FreshnessLevel>,
    pub target: Option<MeTarget>,
}

impl MeItem {
    pub fn known(
        provider_id: impl Into<String>,
        id: impl Into<String>,
        section: MeSection,
        label: impl Into<String>,
        value: impl Into<String>,
    ) -> Self {
        Self {
            id: id.into(),
            provider_id: provider_id.into(),
            section,
            label: label.into(),
            state: MeFactState::Known {
                value: value.into(),
            },
            detail: None,
            freshness: None,
            target: None,
        }
    }

    pub fn unknown(
        provider_id: impl Into<String>,
        id: impl Into<String>,
        section: MeSection,
        label: impl Into<String>,
    ) -> Self {
        Self {
            id: id.into(),
            provider_id: provider_id.into(),
            section,
            label: label.into(),
            state: MeFactState::Unknown,
            detail: None,
            freshness: None,
            target: None,
        }
    }
}

/// What Me can currently establish about one provider contribution.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MeProviderState {
    Ready,
    Unavailable,
    Unknown,
}

impl MeProviderState {
    pub fn label(self) -> &'static str {
        match self {
            Self::Ready => "Ready",
            Self::Unavailable => "Unavailable",
            Self::Unknown => "Unknown",
        }
    }
}

/// One provider's Me projection plus explicit provider state.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct MeBatch {
    pub provider_id: String,
    pub state: MeProviderState,
    pub items: Vec<MeItem>,
}

impl MeBatch {
    pub fn ready(provider_id: impl Into<String>, items: Vec<MeItem>) -> Self {
        Self {
            provider_id: provider_id.into(),
            state: MeProviderState::Ready,
            items,
        }
    }

    pub fn unavailable(provider_id: impl Into<String>) -> Self {
        Self {
            provider_id: provider_id.into(),
            state: MeProviderState::Unavailable,
            items: Vec::new(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{MeBatch, MeFactState, MeItem, MeProviderState, MeSection, MeTarget};

    #[test]
    fn ready_empty_is_distinct_from_unavailable() {
        let empty = MeBatch::ready("identity", vec![]);
        let unavailable = MeBatch::unavailable("identity");

        assert_eq!(empty.state, MeProviderState::Ready);
        assert_eq!(unavailable.state, MeProviderState::Unavailable);
        assert_ne!(empty.state, unavailable.state);
    }

    #[test]
    fn known_identity_fact_does_not_claim_verification_or_authority() {
        let item = MeItem::known(
            "local-identity",
            "local-did",
            MeSection::Identity,
            "Identity on this device",
            "did:mycelix:example",
        );

        assert!(matches!(item.state, MeFactState::Known { .. }));
        assert!(item.target.is_none());
    }

    #[test]
    fn unknown_remote_fact_remains_unknown() {
        let item = MeItem::unknown(
            "did-registry",
            "live-browser-key-status",
            MeSection::Identity,
            "Current remote key association",
        );

        assert_eq!(item.state, MeFactState::Unknown);
    }

    #[test]
    fn consequential_controls_are_preview_only_in_shared_contract() {
        let target = MeTarget::PreviewControl {
            control_kind: "device.revoke".into(),
            control_ref: "device-7".into(),
        };

        assert!(matches!(target, MeTarget::PreviewControl { .. }));
    }

    #[test]
    fn model_contains_sections_not_a_universal_security_score() {
        assert_eq!(MeSection::SecurityRecovery.label(), "Security & recovery");
        assert_eq!(MeSection::TrustReputation.label(), "Trust & reputation");
    }
}
