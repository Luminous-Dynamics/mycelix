// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! PSI-002B — authority-neutral contact-discovery composition semantics.
//!
//! This crate deliberately implements no Privacy Pass, ARC, OHTTP, registry
//! signature, Holochain, Xenia, VOPRF, or PSI protocol. It only freezes the
//! composition dimensions that must be explicit before PSI-002A can be used as
//! part of a private-contact-discovery design.
//!
//! Governing boundary:
//!
//! ```text
//! query budget declared
//! + transport profile declared
//! + registry snapshot declared
//! + VOPRF key epoch declared
//! + retention declared
//!     != abuse resistance
//!     != transport unlinkability
//!     != registry authenticity/currentness
//!     != composition qualification
//!     != production admission
//! ```

use privacy_computation_core::{LeakageDeclaration, SemanticAuthority};
use serde::{Deserialize, Serialize};

pub const PSI_002A_SUBJECT: &str = "4cd3bbc47c27a3f11df7b3308ea0888adbf2322e";
pub const POLICY_PROFILE: &str = "psi-contact-discovery-composition-v1";

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ExternalProfileIdentity {
    pub protocol: String,
    pub version: String,
    pub profile: String,
}

impl ExternalProfileIdentity {
    fn valid(&self) -> bool {
        nonempty(&self.protocol) && nonempty(&self.version) && nonempty(&self.profile)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum AuthorizationPresentation {
    StableIdentity,
    PseudonymousToken,
    DeclaredUnlinkableToken,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ReplayProtection {
    None,
    SingleUseToken,
    ScopedNullifier { domain: String },
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct QueryBudgetPolicy {
    pub authorization_profile: ExternalProfileIdentity,
    pub service_domain: String,
    pub budget_epoch: String,
    pub presentation: AuthorizationPresentation,
    pub max_identifiers_per_request: u32,
    pub max_redemptions_per_epoch: u32,
    pub replay_protection: ReplayProtection,
    pub redemption_linkability: LeakageDeclaration,
    pub issuer_verifier_collusion: LeakageDeclaration,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum TransportModel {
    Direct,
    ObliviousRelay {
        profile: ExternalProfileIdentity,
        gateway_config_identity: String,
        relay_trust_domain: String,
        gateway_client_network_identity: LeakageDeclaration,
        traffic_analysis: LeakageDeclaration,
    },
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct VoprfKeyScope {
    pub service_domain: String,
    pub key_epoch: String,
    pub key_identity_sha256: String,
    pub backend_profile: String,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum EvidenceDeclaration {
    Unspecified,
    DeclaredPresent,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct RegistrySnapshotProfile {
    pub snapshot_sha256: String,
    pub registry_epoch: String,
    pub sequence: u64,
    pub admitted_key_epoch: String,
    pub authenticity: EvidenceDeclaration,
    pub currentness: EvidenceDeclaration,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum RetentionClass {
    Ephemeral,
    UntilEpochEnd,
    ExplicitSeconds(u64),
    ForbiddenPersistence,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct RetentionPolicy {
    pub raw_canonical_identifiers: RetentionClass,
    pub blinded_elements: RetentionClass,
    pub derived_client_tags: RetentionClass,
    pub server_registry_tags: RetentionClass,
    pub intersection_results: RetentionClass,
    pub authorization_evidence: RetentionClass,
    pub network_logs: RetentionClass,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CompositionRequirements {
    pub require_replay_protection: bool,
    pub require_declared_unlinkable_redemption: bool,
    pub require_gateway_network_identity_hiding: bool,
    pub require_authenticated_registry: bool,
    pub require_current_registry: bool,
    pub forbid_persistent_derived_tags: bool,
}

impl Default for CompositionRequirements {
    fn default() -> Self {
        Self {
            require_replay_protection: true,
            require_declared_unlinkable_redemption: true,
            require_gateway_network_identity_hiding: true,
            require_authenticated_registry: true,
            require_current_registry: true,
            forbid_persistent_derived_tags: true,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ContactDiscoveryCompositionProfile {
    pub psi_002a_subject: String,
    pub policy_profile: String,
    pub query_budget: QueryBudgetPolicy,
    pub transport: TransportModel,
    pub voprf_key: VoprfKeyScope,
    pub registry: RegistrySnapshotProfile,
    pub retention: RetentionPolicy,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum CompositionFailure {
    WrongPsi002ASubject,
    WrongPolicyProfile,
    InvalidAuthorizationProfile,
    EmptyServiceDomain,
    EmptyBudgetEpoch,
    ZeroIdentifierBudget,
    ZeroRedemptionBudget,
    ReplayProtectionRequired,
    EmptyNullifierDomain,
    UnlinkablePresentationRequired,
    UnlinkableRedemptionNotDeclared,
    InvalidTransportProfile,
    ObliviousTransportRequired,
    GatewayNetworkIdentityHidingNotDeclared,
    TrafficAnalysisOverclaim,
    KeyServiceDomainMismatch,
    EmptyKeyEpoch,
    InvalidKeyIdentityDigest,
    EmptyBackendProfile,
    InvalidRegistrySnapshotDigest,
    EmptyRegistryEpoch,
    RegistryKeyEpochMismatch,
    RegistryAuthenticityRequired,
    RegistryCurrentnessRequired,
    PersistentDerivedTagsForbidden,
    ZeroExplicitRetention,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum CompositionDisposition {
    StructurallyCompatible,
    Incompatible(CompositionFailure),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CompositionEvaluation {
    pub disposition: CompositionDisposition,
    pub authority: SemanticAuthority,
}

impl CompositionEvaluation {
    fn compatible() -> Self {
        Self {
            disposition: CompositionDisposition::StructurallyCompatible,
            authority: SemanticAuthority::StructuralOnly,
        }
    }

    fn incompatible(reason: CompositionFailure) -> Self {
        Self {
            disposition: CompositionDisposition::Incompatible(reason),
            authority: SemanticAuthority::StructuralOnly,
        }
    }

    pub const fn abuse_resistance_established(&self) -> bool {
        false
    }

    pub const fn transport_unlinkability_established(&self) -> bool {
        false
    }

    pub const fn registry_authenticity_established(&self) -> bool {
        false
    }

    pub const fn registry_currentness_established(&self) -> bool {
        false
    }

    pub const fn composition_qualified(&self) -> bool {
        false
    }

    pub const fn production_admission_granted(&self) -> bool {
        false
    }

    pub const fn application_authority_granted(&self) -> bool {
        false
    }
}

pub fn evaluate_composition(
    requirements: CompositionRequirements,
    profile: &ContactDiscoveryCompositionProfile,
) -> CompositionEvaluation {
    if profile.psi_002a_subject != PSI_002A_SUBJECT {
        return CompositionEvaluation::incompatible(CompositionFailure::WrongPsi002ASubject);
    }
    if profile.policy_profile != POLICY_PROFILE {
        return CompositionEvaluation::incompatible(CompositionFailure::WrongPolicyProfile);
    }

    let budget = &profile.query_budget;
    if !budget.authorization_profile.valid() {
        return CompositionEvaluation::incompatible(CompositionFailure::InvalidAuthorizationProfile);
    }
    if !nonempty(&budget.service_domain) {
        return CompositionEvaluation::incompatible(CompositionFailure::EmptyServiceDomain);
    }
    if !nonempty(&budget.budget_epoch) {
        return CompositionEvaluation::incompatible(CompositionFailure::EmptyBudgetEpoch);
    }
    if budget.max_identifiers_per_request == 0 {
        return CompositionEvaluation::incompatible(CompositionFailure::ZeroIdentifierBudget);
    }
    if budget.max_redemptions_per_epoch == 0 {
        return CompositionEvaluation::incompatible(CompositionFailure::ZeroRedemptionBudget);
    }
    match &budget.replay_protection {
        ReplayProtection::None if requirements.require_replay_protection => {
            return CompositionEvaluation::incompatible(CompositionFailure::ReplayProtectionRequired);
        }
        ReplayProtection::ScopedNullifier { domain } if !nonempty(domain) => {
            return CompositionEvaluation::incompatible(CompositionFailure::EmptyNullifierDomain);
        }
        _ => {}
    }
    if requirements.require_declared_unlinkable_redemption {
        if budget.presentation != AuthorizationPresentation::DeclaredUnlinkableToken {
            return CompositionEvaluation::incompatible(
                CompositionFailure::UnlinkablePresentationRequired,
            );
        }
        if budget.redemption_linkability != LeakageDeclaration::DeclaredHidden {
            return CompositionEvaluation::incompatible(
                CompositionFailure::UnlinkableRedemptionNotDeclared,
            );
        }
    }

    match &profile.transport {
        TransportModel::Direct if requirements.require_gateway_network_identity_hiding => {
            return CompositionEvaluation::incompatible(CompositionFailure::ObliviousTransportRequired);
        }
        TransportModel::Direct => {}
        TransportModel::ObliviousRelay {
            profile,
            gateway_config_identity,
            relay_trust_domain,
            gateway_client_network_identity,
            traffic_analysis,
        } => {
            if !profile.valid()
                || !nonempty(gateway_config_identity)
                || !nonempty(relay_trust_domain)
            {
                return CompositionEvaluation::incompatible(CompositionFailure::InvalidTransportProfile);
            }
            if requirements.require_gateway_network_identity_hiding
                && *gateway_client_network_identity != LeakageDeclaration::DeclaredHidden
            {
                return CompositionEvaluation::incompatible(
                    CompositionFailure::GatewayNetworkIdentityHidingNotDeclared,
                );
            }
            // RFC 9458 explicitly leaves traffic analysis outside its main
            // privacy theorem. A generic oblivious-relay declaration therefore
            // cannot claim that dimension hidden without a separate profile.
            if *traffic_analysis == LeakageDeclaration::DeclaredHidden {
                return CompositionEvaluation::incompatible(CompositionFailure::TrafficAnalysisOverclaim);
            }
        }
    }

    let key = &profile.voprf_key;
    if key.service_domain != budget.service_domain {
        return CompositionEvaluation::incompatible(CompositionFailure::KeyServiceDomainMismatch);
    }
    if !nonempty(&key.key_epoch) {
        return CompositionEvaluation::incompatible(CompositionFailure::EmptyKeyEpoch);
    }
    if !is_sha256_hex(&key.key_identity_sha256) {
        return CompositionEvaluation::incompatible(CompositionFailure::InvalidKeyIdentityDigest);
    }
    if !nonempty(&key.backend_profile) {
        return CompositionEvaluation::incompatible(CompositionFailure::EmptyBackendProfile);
    }

    let registry = &profile.registry;
    if !is_sha256_hex(&registry.snapshot_sha256) {
        return CompositionEvaluation::incompatible(CompositionFailure::InvalidRegistrySnapshotDigest);
    }
    if !nonempty(&registry.registry_epoch) {
        return CompositionEvaluation::incompatible(CompositionFailure::EmptyRegistryEpoch);
    }
    if registry.admitted_key_epoch != key.key_epoch {
        return CompositionEvaluation::incompatible(CompositionFailure::RegistryKeyEpochMismatch);
    }
    if requirements.require_authenticated_registry
        && registry.authenticity != EvidenceDeclaration::DeclaredPresent
    {
        return CompositionEvaluation::incompatible(CompositionFailure::RegistryAuthenticityRequired);
    }
    if requirements.require_current_registry
        && registry.currentness != EvidenceDeclaration::DeclaredPresent
    {
        return CompositionEvaluation::incompatible(CompositionFailure::RegistryCurrentnessRequired);
    }

    if let Some(reason) = retention_failure(&profile.retention, requirements) {
        return CompositionEvaluation::incompatible(reason);
    }

    CompositionEvaluation::compatible()
}

fn retention_failure(
    retention: &RetentionPolicy,
    requirements: CompositionRequirements,
) -> Option<CompositionFailure> {
    let classes = [
        retention.raw_canonical_identifiers,
        retention.blinded_elements,
        retention.derived_client_tags,
        retention.server_registry_tags,
        retention.intersection_results,
        retention.authorization_evidence,
        retention.network_logs,
    ];
    if classes
        .into_iter()
        .any(|class| matches!(class, RetentionClass::ExplicitSeconds(0)))
    {
        return Some(CompositionFailure::ZeroExplicitRetention);
    }
    if requirements.forbid_persistent_derived_tags
        && retention.derived_client_tags != RetentionClass::ForbiddenPersistence
    {
        return Some(CompositionFailure::PersistentDerivedTagsForbidden);
    }
    None
}

fn nonempty(value: &str) -> bool {
    !value.trim().is_empty()
}

fn is_sha256_hex(value: &str) -> bool {
    value.len() == 64 && value.bytes().all(|byte| byte.is_ascii_hexdigit())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn external(protocol: &str) -> ExternalProfileIdentity {
        ExternalProfileIdentity {
            protocol: protocol.into(),
            version: "v1".into(),
            profile: "test-profile".into(),
        }
    }

    fn base() -> ContactDiscoveryCompositionProfile {
        ContactDiscoveryCompositionProfile {
            psi_002a_subject: PSI_002A_SUBJECT.into(),
            policy_profile: POLICY_PROFILE.into(),
            query_budget: QueryBudgetPolicy {
                authorization_profile: external("anonymous-token"),
                service_domain: "contacts.mycelix.test".into(),
                budget_epoch: "2026w38".into(),
                presentation: AuthorizationPresentation::DeclaredUnlinkableToken,
                max_identifiers_per_request: 256,
                max_redemptions_per_epoch: 4,
                replay_protection: ReplayProtection::ScopedNullifier {
                    domain: "contacts.mycelix.test/2026w38".into(),
                },
                redemption_linkability: LeakageDeclaration::DeclaredHidden,
                issuer_verifier_collusion: LeakageDeclaration::MayReveal,
            },
            transport: TransportModel::ObliviousRelay {
                profile: external("oblivious-http"),
                gateway_config_identity: "gateway-config-v1".into(),
                relay_trust_domain: "relay.example".into(),
                gateway_client_network_identity: LeakageDeclaration::DeclaredHidden,
                traffic_analysis: LeakageDeclaration::MayReveal,
            },
            voprf_key: VoprfKeyScope {
                service_domain: "contacts.mycelix.test".into(),
                key_epoch: "key-epoch-7".into(),
                key_identity_sha256: "11".repeat(32),
                backend_profile: "rfc9497-ristretto255-sha512-voprf-tagged-set-v1".into(),
            },
            registry: RegistrySnapshotProfile {
                snapshot_sha256: "22".repeat(32),
                registry_epoch: "registry-epoch-19".into(),
                sequence: 19,
                admitted_key_epoch: "key-epoch-7".into(),
                authenticity: EvidenceDeclaration::DeclaredPresent,
                currentness: EvidenceDeclaration::DeclaredPresent,
            },
            retention: RetentionPolicy {
                raw_canonical_identifiers: RetentionClass::Ephemeral,
                blinded_elements: RetentionClass::Ephemeral,
                derived_client_tags: RetentionClass::ForbiddenPersistence,
                server_registry_tags: RetentionClass::UntilEpochEnd,
                intersection_results: RetentionClass::ExplicitSeconds(300),
                authorization_evidence: RetentionClass::UntilEpochEnd,
                network_logs: RetentionClass::ExplicitSeconds(60),
            },
        }
    }

    #[test]
    fn compatible_is_structural_only() {
        let evaluation = evaluate_composition(CompositionRequirements::default(), &base());
        assert_eq!(
            evaluation.disposition,
            CompositionDisposition::StructurallyCompatible
        );
        assert_eq!(evaluation.authority, SemanticAuthority::StructuralOnly);
        assert!(!evaluation.abuse_resistance_established());
        assert!(!evaluation.transport_unlinkability_established());
        assert!(!evaluation.registry_authenticity_established());
        assert!(!evaluation.registry_currentness_established());
        assert!(!evaluation.composition_qualified());
        assert!(!evaluation.production_admission_granted());
        assert!(!evaluation.application_authority_granted());
    }

    #[test]
    fn exact_psi_subject_is_required() {
        let mut profile = base();
        profile.psi_002a_subject = "other".into();
        assert_eq!(
            evaluate_composition(CompositionRequirements::default(), &profile).disposition,
            CompositionDisposition::Incompatible(CompositionFailure::WrongPsi002ASubject)
        );
    }

    #[test]
    fn zero_budget_fails_closed() {
        let mut profile = base();
        profile.query_budget.max_identifiers_per_request = 0;
        assert_eq!(
            evaluate_composition(CompositionRequirements::default(), &profile).disposition,
            CompositionDisposition::Incompatible(CompositionFailure::ZeroIdentifierBudget)
        );
    }

    #[test]
    fn replay_protection_is_independent() {
        let mut profile = base();
        profile.query_budget.replay_protection = ReplayProtection::None;
        assert_eq!(
            evaluate_composition(CompositionRequirements::default(), &profile).disposition,
            CompositionDisposition::Incompatible(CompositionFailure::ReplayProtectionRequired)
        );
    }

    #[test]
    fn stable_identity_does_not_satisfy_unlinkable_redemption_requirement() {
        let mut profile = base();
        profile.query_budget.presentation = AuthorizationPresentation::StableIdentity;
        assert_eq!(
            evaluate_composition(CompositionRequirements::default(), &profile).disposition,
            CompositionDisposition::Incompatible(
                CompositionFailure::UnlinkablePresentationRequired
            )
        );
    }

    #[test]
    fn direct_transport_does_not_hide_client_network_identity_from_gateway() {
        let mut profile = base();
        profile.transport = TransportModel::Direct;
        assert_eq!(
            evaluate_composition(CompositionRequirements::default(), &profile).disposition,
            CompositionDisposition::Incompatible(CompositionFailure::ObliviousTransportRequired)
        );
    }

    #[test]
    fn oblivious_transport_cannot_overclaim_traffic_analysis_privacy() {
        let mut profile = base();
        if let TransportModel::ObliviousRelay { traffic_analysis, .. } = &mut profile.transport {
            *traffic_analysis = LeakageDeclaration::DeclaredHidden;
        }
        assert_eq!(
            evaluate_composition(CompositionRequirements::default(), &profile).disposition,
            CompositionDisposition::Incompatible(CompositionFailure::TrafficAnalysisOverclaim)
        );
    }

    #[test]
    fn key_service_scope_must_match_budget_service() {
        let mut profile = base();
        profile.voprf_key.service_domain = "other-service".into();
        assert_eq!(
            evaluate_composition(CompositionRequirements::default(), &profile).disposition,
            CompositionDisposition::Incompatible(CompositionFailure::KeyServiceDomainMismatch)
        );
    }

    #[test]
    fn registry_and_key_epochs_do_not_alias() {
        let mut profile = base();
        profile.registry.admitted_key_epoch = "old-key-epoch".into();
        assert_eq!(
            evaluate_composition(CompositionRequirements::default(), &profile).disposition,
            CompositionDisposition::Incompatible(CompositionFailure::RegistryKeyEpochMismatch)
        );
    }

    #[test]
    fn snapshot_hash_does_not_imply_authenticity_or_currentness() {
        let mut profile = base();
        profile.registry.authenticity = EvidenceDeclaration::Unspecified;
        assert_eq!(
            evaluate_composition(CompositionRequirements::default(), &profile).disposition,
            CompositionDisposition::Incompatible(CompositionFailure::RegistryAuthenticityRequired)
        );
        profile.registry.authenticity = EvidenceDeclaration::DeclaredPresent;
        profile.registry.currentness = EvidenceDeclaration::Unspecified;
        assert_eq!(
            evaluate_composition(CompositionRequirements::default(), &profile).disposition,
            CompositionDisposition::Incompatible(CompositionFailure::RegistryCurrentnessRequired)
        );
    }

    #[test]
    fn derived_tags_must_not_be_retained_when_policy_forbids_it() {
        let mut profile = base();
        profile.retention.derived_client_tags = RetentionClass::UntilEpochEnd;
        assert_eq!(
            evaluate_composition(CompositionRequirements::default(), &profile).disposition,
            CompositionDisposition::Incompatible(
                CompositionFailure::PersistentDerivedTagsForbidden
            )
        );
    }

    #[test]
    fn zero_second_retention_is_rejected_instead_of_reinterpreted() {
        let mut profile = base();
        profile.retention.network_logs = RetentionClass::ExplicitSeconds(0);
        assert_eq!(
            evaluate_composition(CompositionRequirements::default(), &profile).disposition,
            CompositionDisposition::Incompatible(CompositionFailure::ZeroExplicitRetention)
        );
    }

    #[test]
    fn wire_names_keep_privacy_pass_and_ohttp_as_adapter_profiles_not_semantic_roots() {
        let serialized = serde_json::to_string(&base()).unwrap();
        assert!(serialized.contains("anonymous-token"));
        assert!(serialized.contains("oblivious-http"));
        assert!(!serialized.contains("PrivacyPassRfc9577"));
        assert!(!serialized.contains("OhttpRfc9458"));
    }
}
