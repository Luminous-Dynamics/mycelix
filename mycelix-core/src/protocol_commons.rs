// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! PROTO-000..003: protocol-commons profile and negotiation primitives.
//! Protocol compatibility is evidence, never an authority source.

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct ProtocolVersionV1 {
    pub major: u16,
    pub minor: u16,
}

impl ProtocolVersionV1 {
    pub const fn new(major: u16, minor: u16) -> Self {
        Self { major, minor }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum SemanticClassV1 {
    Advisory,
    Feature,
    Evidence,
    Privacy,
    Rights,
    Authority,
    Safety,
    Effect,
}

impl SemanticClassV1 {
    pub const fn is_critical(self) -> bool {
        matches!(
            self,
            Self::Privacy | Self::Rights | Self::Authority | Self::Safety | Self::Effect
        )
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum UnknownSemanticDispositionV1 {
    Ignore,
    FeatureUnavailable,
    ReduceClaimCeiling,
    NeedsManualReview,
    Deny,
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct ProtocolCapabilityV1 {
    pub id: String,
    pub version: u32,
}

impl ProtocolCapabilityV1 {
    pub fn new(id: impl Into<String>, version: u32) -> Self {
        Self { id: id.into(), version }
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ExtensionSpecV1 {
    pub id: String,
    pub version: u32,
    pub semantic_class: SemanticClassV1,
    pub unknown_disposition: UnknownSemanticDispositionV1,
    pub required: bool,
}

impl ExtensionSpecV1 {
    pub fn new(
        id: impl Into<String>,
        version: u32,
        semantic_class: SemanticClassV1,
        unknown_disposition: UnknownSemanticDispositionV1,
        required: bool,
    ) -> Self {
        Self { id: id.into(), version, semantic_class, unknown_disposition, required }
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ProtocolProfileV1 {
    pub profile_id: String,
    pub revision: u32,
    pub core_version: ProtocolVersionV1,
    /// Peer preference only; not a truth, safety, recency, or legitimacy score.
    pub preference_rank: u32,
    pub required_capabilities: BTreeSet<ProtocolCapabilityV1>,
    pub optional_capabilities: BTreeSet<ProtocolCapabilityV1>,
    pub extensions: BTreeMap<String, ExtensionSpecV1>,
    pub semantic_modules: BTreeSet<String>,
    pub authority_vocabulary: String,
    pub privacy_vocabulary: String,
    pub evidence_vocabulary: String,
    pub canonicalization_profile: String,
    pub security_suite: String,
    pub conformance_profile: String,
    pub deprecation_horizon_unix_ms: Option<u64>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ProfileValidationErrorV1 {
    EmptyProfileId,
    CapabilityBothRequiredAndOptional(ProtocolCapabilityV1),
    ExtensionKeyMismatch { key: String, extension_id: String },
    CriticalSemanticCannotBeIgnored { extension_id: String, class: SemanticClassV1 },
    RequiredExtensionCannotDegradeSilently(String),
}

impl ProtocolProfileV1 {
    pub fn validate(&self) -> Result<(), ProfileValidationErrorV1> {
        if self.profile_id.trim().is_empty() {
            return Err(ProfileValidationErrorV1::EmptyProfileId);
        }
        for capability in &self.required_capabilities {
            if self.optional_capabilities.contains(capability) {
                return Err(ProfileValidationErrorV1::CapabilityBothRequiredAndOptional(
                    capability.clone(),
                ));
            }
        }
        for (key, extension) in &self.extensions {
            if key != &extension.id {
                return Err(ProfileValidationErrorV1::ExtensionKeyMismatch {
                    key: key.clone(),
                    extension_id: extension.id.clone(),
                });
            }
            if extension.semantic_class.is_critical()
                && extension.unknown_disposition == UnknownSemanticDispositionV1::Ignore
            {
                return Err(ProfileValidationErrorV1::CriticalSemanticCannotBeIgnored {
                    extension_id: extension.id.clone(),
                    class: extension.semantic_class,
                });
            }
            if extension.required
                && matches!(
                    extension.unknown_disposition,
                    UnknownSemanticDispositionV1::Ignore
                        | UnknownSemanticDispositionV1::FeatureUnavailable
                )
            {
                return Err(ProfileValidationErrorV1::RequiredExtensionCannotDegradeSilently(
                    extension.id.clone(),
                ));
            }
        }
        Ok(())
    }

    pub fn content_digest_sha256(&self) -> Result<String, serde_json::Error> {
        let digest = Sha256::digest(serde_json::to_vec(self)?);
        Ok(hex::encode(digest))
    }

    pub fn exact_ref(&self) -> Result<ExactProfileRefV1, serde_json::Error> {
        Ok(ExactProfileRefV1 {
            profile_id: self.profile_id.clone(),
            revision: self.revision,
            content_digest_sha256: self.content_digest_sha256()?,
        })
    }

    fn all_capabilities(&self) -> BTreeSet<ProtocolCapabilityV1> {
        self.required_capabilities
            .union(&self.optional_capabilities)
            .cloned()
            .collect()
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ExactProfileRefV1 {
    pub profile_id: String,
    pub revision: u32,
    pub content_digest_sha256: String,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ProtocolOfferV1 {
    pub peer_id: String,
    pub profiles: Vec<ProtocolProfileV1>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct DowngradePermitV1 {
    /// Must refer to an authority decision established outside this module.
    pub authority_decision_ref: String,
    pub reason: String,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize, Default)]
pub struct NegotiationPolicyV1 {
    pub minimum_local_preference_rank: u32,
    pub downgrade_permit: Option<DowngradePermitV1>,
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum NegotiationLossV1 {
    CapabilityUnavailable(ProtocolCapabilityV1),
    ExtensionUnavailable { id: String, version: u32 },
    ClaimCeilingReduced { id: String, version: u32 },
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct DowngradeRecordV1 {
    pub preferred_local_profile: ExactProfileRefV1,
    pub selected_local_profile: ExactProfileRefV1,
    pub authority_decision_ref: String,
    pub reason: String,
    pub lost_semantics: Vec<NegotiationLossV1>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct NegotiatedProtocolReceiptV1 {
    pub local_peer_id: String,
    pub remote_peer_id: String,
    pub local_profile: ExactProfileRefV1,
    pub remote_profile: ExactProfileRefV1,
    pub core_version: ProtocolVersionV1,
    pub losses: Vec<NegotiationLossV1>,
    pub downgrade: Option<DowngradeRecordV1>,
}

impl NegotiatedProtocolReceiptV1 {
    pub const fn grants_authority(&self) -> bool { false }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum NegotiationFailureV1 {
    EmptyOffer(String),
    InvalidProfile { peer_id: String, profile_id: String, error: ProfileValidationErrorV1 },
    NoCompatibleProfile,
    DowngradeAuthorizationRequired { selected_rank: u32, minimum_rank: u32 },
    DigestFailure(String),
}

#[derive(Clone, Debug)]
struct Candidate<'a> {
    local: &'a ProtocolProfileV1,
    remote: &'a ProtocolProfileV1,
    losses: Vec<NegotiationLossV1>,
}

pub fn negotiate_protocol_v1(
    local: &ProtocolOfferV1,
    remote: &ProtocolOfferV1,
    policy: &NegotiationPolicyV1,
) -> Result<NegotiatedProtocolReceiptV1, NegotiationFailureV1> {
    validate_offer(local)?;
    validate_offer(remote)?;

    let mut candidates = Vec::new();
    for local_profile in &local.profiles {
        for remote_profile in &remote.profiles {
            if let Some(losses) = compatible_pair(local_profile, remote_profile) {
                candidates.push(Candidate { local: local_profile, remote: remote_profile, losses });
            }
        }
    }
    if candidates.is_empty() {
        return Err(NegotiationFailureV1::NoCompatibleProfile);
    }

    candidates.sort_by(|a, b| {
        let ar = a.local.preference_rank.min(a.remote.preference_rank);
        let br = b.local.preference_rank.min(b.remote.preference_rank);
        br.cmp(&ar)
            .then_with(|| {
                (b.local.preference_rank + b.remote.preference_rank)
                    .cmp(&(a.local.preference_rank + a.remote.preference_rank))
            })
            .then_with(|| a.local.profile_id.cmp(&b.local.profile_id))
            .then_with(|| b.local.revision.cmp(&a.local.revision))
    });
    let selected = &candidates[0];

    if selected.local.preference_rank < policy.minimum_local_preference_rank
        && policy.downgrade_permit.is_none()
    {
        return Err(NegotiationFailureV1::DowngradeAuthorizationRequired {
            selected_rank: selected.local.preference_rank,
            minimum_rank: policy.minimum_local_preference_rank,
        });
    }

    let local_ref = selected.local.exact_ref().map_err(|e| NegotiationFailureV1::DigestFailure(e.to_string()))?;
    let remote_ref = selected.remote.exact_ref().map_err(|e| NegotiationFailureV1::DigestFailure(e.to_string()))?;
    let downgrade = if selected.local.preference_rank < policy.minimum_local_preference_rank {
        let permit = policy.downgrade_permit.as_ref().expect("checked above");
        let preferred = local.profiles.iter().max_by_key(|p| p.preference_rank).expect("validated non-empty");
        Some(DowngradeRecordV1 {
            preferred_local_profile: preferred.exact_ref().map_err(|e| NegotiationFailureV1::DigestFailure(e.to_string()))?,
            selected_local_profile: local_ref.clone(),
            authority_decision_ref: permit.authority_decision_ref.clone(),
            reason: permit.reason.clone(),
            lost_semantics: semantic_loss(preferred, selected.local),
        })
    } else {
        None
    };

    Ok(NegotiatedProtocolReceiptV1 {
        local_peer_id: local.peer_id.clone(),
        remote_peer_id: remote.peer_id.clone(),
        local_profile: local_ref,
        remote_profile: remote_ref,
        core_version: selected.local.core_version,
        losses: selected.losses.clone(),
        downgrade,
    })
}

fn validate_offer(offer: &ProtocolOfferV1) -> Result<(), NegotiationFailureV1> {
    if offer.profiles.is_empty() {
        return Err(NegotiationFailureV1::EmptyOffer(offer.peer_id.clone()));
    }
    for profile in &offer.profiles {
        profile.validate().map_err(|error| NegotiationFailureV1::InvalidProfile {
            peer_id: offer.peer_id.clone(),
            profile_id: profile.profile_id.clone(),
            error,
        })?;
    }
    Ok(())
}

fn compatible_pair(
    local: &ProtocolProfileV1,
    remote: &ProtocolProfileV1,
) -> Option<Vec<NegotiationLossV1>> {
    if local.core_version != remote.core_version
        || local.authority_vocabulary != remote.authority_vocabulary
        || local.privacy_vocabulary != remote.privacy_vocabulary
        || local.evidence_vocabulary != remote.evidence_vocabulary
        || local.canonicalization_profile != remote.canonicalization_profile
        || local.security_suite != remote.security_suite
    {
        return None;
    }

    let local_all = local.all_capabilities();
    let remote_all = remote.all_capabilities();
    if !local.required_capabilities.iter().all(|c| remote_all.contains(c))
        || !remote.required_capabilities.iter().all(|c| local_all.contains(c))
    {
        return None;
    }

    let mut losses = Vec::new();
    if !extensions_compatible(local, remote, &mut losses)
        || !extensions_compatible(remote, local, &mut losses)
    {
        return None;
    }
    losses.sort();
    losses.dedup();
    Some(losses)
}

fn extensions_compatible(
    sender: &ProtocolProfileV1,
    receiver: &ProtocolProfileV1,
    losses: &mut Vec<NegotiationLossV1>,
) -> bool {
    for extension in sender.extensions.values() {
        if let Some(peer) = receiver.extensions.get(&extension.id) {
            if peer.version == extension.version && peer.semantic_class == extension.semantic_class {
                continue;
            }
            if extension.required || peer.required
                || extension.semantic_class.is_critical()
                || peer.semantic_class.is_critical()
            {
                return false;
            }
            losses.push(NegotiationLossV1::ExtensionUnavailable {
                id: extension.id.clone(),
                version: extension.version,
            });
            continue;
        }

        match extension.unknown_disposition {
            UnknownSemanticDispositionV1::Ignore => {
                if extension.required || extension.semantic_class.is_critical() { return false; }
            }
            UnknownSemanticDispositionV1::FeatureUnavailable => {
                if extension.required || extension.semantic_class.is_critical() { return false; }
                losses.push(NegotiationLossV1::ExtensionUnavailable {
                    id: extension.id.clone(),
                    version: extension.version,
                });
            }
            UnknownSemanticDispositionV1::ReduceClaimCeiling => {
                if extension.semantic_class.is_critical() { return false; }
                losses.push(NegotiationLossV1::ClaimCeilingReduced {
                    id: extension.id.clone(),
                    version: extension.version,
                });
            }
            UnknownSemanticDispositionV1::NeedsManualReview | UnknownSemanticDispositionV1::Deny => {
                return false;
            }
        }
    }
    true
}

fn semantic_loss(preferred: &ProtocolProfileV1, selected: &ProtocolProfileV1) -> Vec<NegotiationLossV1> {
    let selected_caps = selected.all_capabilities();
    let mut losses = Vec::new();
    for capability in preferred.all_capabilities() {
        if !selected_caps.contains(&capability) {
            losses.push(NegotiationLossV1::CapabilityUnavailable(capability));
        }
    }
    for extension in preferred.extensions.values() {
        let retained = selected.extensions.get(&extension.id).is_some_and(|candidate| {
            candidate.version == extension.version && candidate.semantic_class == extension.semantic_class
        });
        if !retained {
            losses.push(NegotiationLossV1::ExtensionUnavailable {
                id: extension.id.clone(),
                version: extension.version,
            });
        }
    }
    losses.sort();
    losses.dedup();
    losses
}

#[cfg(test)]
mod tests {
    use super::*;

    fn profile(id: &str, rank: u32) -> ProtocolProfileV1 {
        ProtocolProfileV1 {
            profile_id: id.into(), revision: 1, core_version: ProtocolVersionV1::new(1, 0),
            preference_rank: rank,
            required_capabilities: BTreeSet::from([ProtocolCapabilityV1::new("core", 1)]),
            optional_capabilities: BTreeSet::new(), extensions: BTreeMap::new(),
            semantic_modules: BTreeSet::from(["mycelix.core.v1".into()]),
            authority_vocabulary: "mycelix.authority.v1".into(),
            privacy_vocabulary: "mycelix.privacy.v1".into(),
            evidence_vocabulary: "mycelix.evidence.v1".into(),
            canonicalization_profile: "mycelix.c14n.v1".into(),
            security_suite: "test-suite-v1".into(), conformance_profile: "proto-000-003".into(),
            deprecation_horizon_unix_ms: None,
        }
    }

    fn offer(peer: &str, profiles: Vec<ProtocolProfileV1>) -> ProtocolOfferV1 {
        ProtocolOfferV1 { peer_id: peer.into(), profiles }
    }

    #[test]
    fn critical_unknown_semantic_cannot_be_ignored() {
        let mut p = profile("invalid", 100);
        p.extensions.insert("privacy.retention".into(), ExtensionSpecV1::new(
            "privacy.retention", 1, SemanticClassV1::Privacy, UnknownSemanticDispositionV1::Ignore, false,
        ));
        assert!(matches!(p.validate(), Err(ProfileValidationErrorV1::CriticalSemanticCannotBeIgnored { .. })));
    }

    #[test]
    fn optional_advisory_unknown_can_be_ignored() {
        let local = profile("local", 100);
        let mut remote = profile("remote", 100);
        remote.extensions.insert("example.advisory".into(), ExtensionSpecV1::new(
            "example.advisory", 1, SemanticClassV1::Advisory, UnknownSemanticDispositionV1::Ignore, false,
        ));
        assert!(negotiate_protocol_v1(
            &offer("a", vec![local]), &offer("b", vec![remote]), &NegotiationPolicyV1::default()
        ).is_ok());
    }

    #[test]
    fn required_unknown_fails_closed() {
        let local = profile("local", 100);
        let mut remote = profile("remote", 100);
        remote.extensions.insert("authority.scope".into(), ExtensionSpecV1::new(
            "authority.scope", 1, SemanticClassV1::Authority, UnknownSemanticDispositionV1::Deny, true,
        ));
        assert_eq!(
            negotiate_protocol_v1(
                &offer("a", vec![local]), &offer("b", vec![remote]), &NegotiationPolicyV1::default()
            ),
            Err(NegotiationFailureV1::NoCompatibleProfile)
        );
    }

    #[test]
    fn missing_required_capability_fails_closed() {
        let local = profile("local", 100);
        let mut remote = profile("remote", 100);
        remote.required_capabilities.insert(ProtocolCapabilityV1::new("water.currentness", 1));
        assert_eq!(
            negotiate_protocol_v1(
                &offer("a", vec![local]), &offer("b", vec![remote]), &NegotiationPolicyV1::default()
            ),
            Err(NegotiationFailureV1::NoCompatibleProfile)
        );
    }

    #[test]
    fn downgrade_requires_external_authority_reference() {
        let mut preferred = profile("preferred", 100);
        preferred.security_suite = "new-suite".into();
        preferred.optional_capabilities.insert(ProtocolCapabilityV1::new("privacy.v2", 1));
        let selected = profile("selected", 10);
        let remote = selected.clone();
        let no_permit = NegotiationPolicyV1 { minimum_local_preference_rank: 50, downgrade_permit: None };
        assert_eq!(
            negotiate_protocol_v1(
                &offer("a", vec![preferred.clone(), selected.clone()]), &offer("b", vec![remote.clone()]), &no_permit
            ),
            Err(NegotiationFailureV1::DowngradeAuthorizationRequired { selected_rank: 10, minimum_rank: 50 })
        );

        let permitted = NegotiationPolicyV1 {
            minimum_local_preference_rank: 50,
            downgrade_permit: Some(DowngradePermitV1 {
                authority_decision_ref: "authority:decision:example".into(), reason: "degraded mode".into(),
            }),
        };
        let receipt = negotiate_protocol_v1(
            &offer("a", vec![preferred, selected]), &offer("b", vec![remote]), &permitted
        ).expect("explicitly permitted downgrade should negotiate");
        assert!(!receipt.grants_authority());
        assert_eq!(receipt.downgrade.unwrap().authority_decision_ref, "authority:decision:example");
    }

    #[test]
    fn profile_digest_is_content_bound() {
        let first = profile("digest", 100);
        let mut second = first.clone();
        assert_eq!(first.content_digest_sha256().unwrap(), second.content_digest_sha256().unwrap());
        second.preference_rank = 99;
        assert_ne!(first.content_digest_sha256().unwrap(), second.content_digest_sha256().unwrap());
    }
}
