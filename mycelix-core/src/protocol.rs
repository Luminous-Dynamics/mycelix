// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Constitutional interoperability primitives for the Mycelix protocol commons.
//!
//! This module intentionally does **not** define governance authority, registry
//! legitimacy, certification, or end-to-end security. It provides only a thin
//! semantic waist for exact protocol-profile identity and compatibility.

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};

const PROFILE_DOMAIN_SEPARATOR: &[u8] = b"mycelix-protocol-profile-v1\0";

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub struct ProtocolVersionV1 {
    pub major: u16,
    pub minor: u16,
    pub patch: u16,
}

impl ProtocolVersionV1 {
    pub const fn new(major: u16, minor: u16, patch: u16) -> Self {
        Self { major, minor, patch }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SemanticClassV1 {
    Presentation,
    Advisory,
    Evidence,
    PrivacyConstraint,
    AuthorityConstraint,
    SafetyConstraint,
    LifecycleConstraint,
    EffectConstraint,
}

impl SemanticClassV1 {
    fn tag(self) -> u8 {
        match self {
            Self::Presentation => 0,
            Self::Advisory => 1,
            Self::Evidence => 2,
            Self::PrivacyConstraint => 3,
            Self::AuthorityConstraint => 4,
            Self::SafetyConstraint => 5,
            Self::LifecycleConstraint => 6,
            Self::EffectConstraint => 7,
        }
    }

    fn is_protected_by_default(self) -> bool {
        matches!(
            self,
            Self::PrivacyConstraint
                | Self::AuthorityConstraint
                | Self::SafetyConstraint
                | Self::LifecycleConstraint
                | Self::EffectConstraint
        )
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum UnknownSemanticBehaviorV1 {
    Ignore,
    FeatureUnavailable,
    ReduceClaimCeiling,
    DenyOperation,
    RequireReview,
}

impl UnknownSemanticBehaviorV1 {
    fn tag(self) -> u8 {
        match self {
            Self::Ignore => 0,
            Self::FeatureUnavailable => 1,
            Self::ReduceClaimCeiling => 2,
            Self::DenyOperation => 3,
            Self::RequireReview => 4,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ProtocolSemanticV1 {
    pub id: String,
    pub class: SemanticClassV1,
    pub critical: bool,
    pub unknown_behavior: UnknownSemanticBehaviorV1,
}

impl ProtocolSemanticV1 {
    pub fn validate(&self) -> Result<(), ProtocolProfileErrorV1> {
        if self.id.trim().is_empty() {
            return Err(ProtocolProfileErrorV1::EmptySemanticId);
        }
        if self.critical && self.unknown_behavior == UnknownSemanticBehaviorV1::Ignore {
            return Err(ProtocolProfileErrorV1::CriticalSemanticCannotBeIgnored(
                self.id.clone(),
            ));
        }
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ProtocolProfileV1 {
    pub family: String,
    pub version: ProtocolVersionV1,
    #[serde(default)]
    pub capabilities: Vec<String>,
    #[serde(default)]
    pub semantics: Vec<ProtocolSemanticV1>,
    #[serde(default)]
    pub security_suites: Vec<String>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ProtocolProfileErrorV1 {
    EmptyFamily,
    EmptyCapability,
    EmptySecuritySuite,
    EmptySemanticId,
    DuplicateSemanticId(String),
    CriticalSemanticCannotBeIgnored(String),
}

impl ProtocolProfileV1 {
    pub fn validate(&self) -> Result<(), ProtocolProfileErrorV1> {
        if self.family.trim().is_empty() {
            return Err(ProtocolProfileErrorV1::EmptyFamily);
        }
        if self.capabilities.iter().any(|value| value.trim().is_empty()) {
            return Err(ProtocolProfileErrorV1::EmptyCapability);
        }
        if self
            .security_suites
            .iter()
            .any(|value| value.trim().is_empty())
        {
            return Err(ProtocolProfileErrorV1::EmptySecuritySuite);
        }

        let mut ids = BTreeSet::new();
        for semantic in &self.semantics {
            semantic.validate()?;
            if !ids.insert(semantic.id.as_str()) {
                return Err(ProtocolProfileErrorV1::DuplicateSemanticId(
                    semantic.id.clone(),
                ));
            }
        }
        Ok(())
    }

    /// Content-addressed identity over the exact semantic profile.
    ///
    /// Ordering of set-like fields is normalized before hashing. The identity
    /// is deliberately distinct from the human-readable family/version tuple.
    pub fn profile_id(&self) -> Result<String, ProtocolProfileErrorV1> {
        self.validate()?;

        let mut hasher = Sha256::new();
        hasher.update(PROFILE_DOMAIN_SEPARATOR);
        hash_str(&mut hasher, &self.family);
        hasher.update(self.version.major.to_be_bytes());
        hasher.update(self.version.minor.to_be_bytes());
        hasher.update(self.version.patch.to_be_bytes());

        let mut capabilities = self.capabilities.clone();
        capabilities.sort();
        capabilities.dedup();
        hash_string_set(&mut hasher, &capabilities);

        let mut semantics = self.semantics.clone();
        semantics.sort_by(|left, right| left.id.cmp(&right.id));
        hasher.update((semantics.len() as u64).to_be_bytes());
        for semantic in semantics {
            hash_str(&mut hasher, &semantic.id);
            hasher.update([semantic.class.tag()]);
            hasher.update([u8::from(semantic.critical)]);
            hasher.update([semantic.unknown_behavior.tag()]);
        }

        let mut security_suites = self.security_suites.clone();
        security_suites.sort();
        security_suites.dedup();
        hash_string_set(&mut hasher, &security_suites);

        Ok(hex::encode(hasher.finalize()))
    }

    fn semantic_map(&self) -> BTreeMap<&str, &ProtocolSemanticV1> {
        self.semantics
            .iter()
            .map(|semantic| (semantic.id.as_str(), semantic))
            .collect()
    }
}

fn hash_str(hasher: &mut Sha256, value: &str) {
    hasher.update((value.len() as u64).to_be_bytes());
    hasher.update(value.as_bytes());
}

fn hash_string_set(hasher: &mut Sha256, values: &[String]) {
    hasher.update((values.len() as u64).to_be_bytes());
    for value in values {
        hash_str(hasher, value);
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DirectionalCompatibilityOutcomeV1 {
    Qualified,
    Degraded,
    Refused,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct SemanticConflictV1 {
    pub semantic_id: String,
    pub sender: ProtocolSemanticV1,
    pub receiver: ProtocolSemanticV1,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ClaimCeilingV1 {
    /// Exact semantics whose descriptors are understood identically by both sides.
    pub semantic_ids: Vec<String>,
    /// Semantic classes represented by that exact intersection.
    pub classes: Vec<SemanticClassV1>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct DirectionalCompatibilityV1 {
    pub sender_profile_id: String,
    pub receiver_profile_id: String,
    pub outcome: DirectionalCompatibilityOutcomeV1,
    pub understood_semantics: Vec<String>,
    pub missing_noncritical_semantics: Vec<String>,
    pub missing_critical_semantics: Vec<String>,
    pub semantic_conflicts: Vec<SemanticConflictV1>,
    pub claim_ceiling: ClaimCeilingV1,
}

/// Assess whether `receiver` can safely interpret claims emitted by `sender`.
///
/// Compatibility is directional. Missing critical semantics or conflicting
/// descriptors refuse the direction. Missing non-critical semantics explicitly
/// degrade the claim ceiling rather than being treated as understood.
pub fn assess_directional_compatibility(
    sender: &ProtocolProfileV1,
    receiver: &ProtocolProfileV1,
) -> Result<DirectionalCompatibilityV1, ProtocolProfileErrorV1> {
    sender.validate()?;
    receiver.validate()?;

    let sender_profile_id = sender.profile_id()?;
    let receiver_profile_id = receiver.profile_id()?;
    let receiver_semantics = receiver.semantic_map();

    let mut understood = Vec::new();
    let mut missing_noncritical = Vec::new();
    let mut missing_critical = Vec::new();
    let mut conflicts = Vec::new();
    let mut classes = BTreeSet::new();

    let mut sender_semantics = sender.semantics.iter().collect::<Vec<_>>();
    sender_semantics.sort_by(|left, right| left.id.cmp(&right.id));

    for semantic in sender_semantics {
        match receiver_semantics.get(semantic.id.as_str()) {
            Some(receiver_semantic) if **receiver_semantic == *semantic => {
                understood.push(semantic.id.clone());
                classes.insert(semantic.class);
            }
            Some(receiver_semantic) => conflicts.push(SemanticConflictV1 {
                semantic_id: semantic.id.clone(),
                sender: semantic.clone(),
                receiver: (**receiver_semantic).clone(),
            }),
            None if semantic.critical => missing_critical.push(semantic.id.clone()),
            None => missing_noncritical.push(semantic.id.clone()),
        }
    }

    let outcome = if !missing_critical.is_empty() || !conflicts.is_empty() {
        DirectionalCompatibilityOutcomeV1::Refused
    } else if !missing_noncritical.is_empty() {
        DirectionalCompatibilityOutcomeV1::Degraded
    } else {
        DirectionalCompatibilityOutcomeV1::Qualified
    };

    Ok(DirectionalCompatibilityV1 {
        sender_profile_id,
        receiver_profile_id,
        outcome,
        understood_semantics: understood.clone(),
        missing_noncritical_semantics: missing_noncritical,
        missing_critical_semantics: missing_critical,
        semantic_conflicts: conflicts,
        claim_ceiling: ClaimCeilingV1 {
            semantic_ids: understood,
            classes: classes.into_iter().collect(),
        },
    })
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ProtocolOfferV1 {
    pub preferred: ProtocolProfileV1,
    #[serde(default)]
    pub alternatives: Vec<ProtocolProfileV1>,
}

impl ProtocolOfferV1 {
    fn profiles(&self) -> Result<Vec<&ProtocolProfileV1>, ProtocolProfileErrorV1> {
        self.preferred.validate()?;
        let mut profiles = vec![&self.preferred];
        for profile in &self.alternatives {
            profile.validate()?;
            profiles.push(profile);
        }
        Ok(profiles)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum NegotiationDecisionV1 {
    Qualified,
    Degraded,
    Refused,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ProtocolNegotiationReceiptV1 {
    pub decision: NegotiationDecisionV1,
    pub selected_local_profile_id: Option<String>,
    pub selected_peer_profile_id: Option<String>,
    pub selected_local_version: Option<ProtocolVersionV1>,
    pub selected_peer_version: Option<ProtocolVersionV1>,
    pub local_to_peer: Option<DirectionalCompatibilityV1>,
    pub peer_to_local: Option<DirectionalCompatibilityV1>,
    pub common_capabilities: Vec<String>,
    pub common_security_suites: Vec<String>,
    pub downgrade: bool,
    /// Critical protected semantics from a preferred profile that a selected
    /// downgrade would have dropped. Any such loss makes negotiation refuse.
    pub protected_semantics_lost: Vec<String>,
    pub refusal_reasons: Vec<String>,
}

#[derive(Clone)]
struct Candidate<'a> {
    local: &'a ProtocolProfileV1,
    peer: &'a ProtocolProfileV1,
    local_to_peer: DirectionalCompatibilityV1,
    peer_to_local: DirectionalCompatibilityV1,
    common_capabilities: Vec<String>,
    common_security_suites: Vec<String>,
    downgrade: bool,
    score: (ProtocolVersionV1, ProtocolVersionV1, bool),
}

/// Negotiate the highest mutually compatible profile pair.
///
/// The handshake is deliberately not a conformance proof. Peer-advertised
/// profiles are claims. PROTO-006 may later bind them to independent conformance
/// evidence. This function only computes exact declared semantic overlap.
pub fn negotiate_protocol(
    local: &ProtocolOfferV1,
    peer: &ProtocolOfferV1,
) -> Result<ProtocolNegotiationReceiptV1, ProtocolProfileErrorV1> {
    let local_profiles = local.profiles()?;
    let peer_profiles = peer.profiles()?;
    let local_preferred_id = local.preferred.profile_id()?;
    let peer_preferred_id = peer.preferred.profile_id()?;

    let protected_local = protected_semantics(&local.preferred);
    let protected_peer = protected_semantics(&peer.preferred);

    let mut candidates = Vec::new();
    let mut refusal_reasons = Vec::new();
    let mut all_protected_losses = BTreeSet::new();

    for local_profile in &local_profiles {
        for peer_profile in &peer_profiles {
            if local_profile.family != peer_profile.family {
                continue;
            }
            if local_profile.version.major != peer_profile.version.major {
                continue;
            }

            let local_to_peer = assess_directional_compatibility(local_profile, peer_profile)?;
            let peer_to_local = assess_directional_compatibility(peer_profile, local_profile)?;

            if local_to_peer.outcome == DirectionalCompatibilityOutcomeV1::Refused
                || peer_to_local.outcome == DirectionalCompatibilityOutcomeV1::Refused
            {
                refusal_reasons.push(format!(
                    "semantic incompatibility between {} and {}",
                    local_profile.profile_id()?,
                    peer_profile.profile_id()?
                ));
                continue;
            }

            let local_id = local_profile.profile_id()?;
            let peer_id = peer_profile.profile_id()?;
            let downgrade = local_id != local_preferred_id
                || peer_id != peer_preferred_id
                || local_profile.version != peer_profile.version;

            let mut protected_losses = BTreeSet::new();
            if local_id != local_preferred_id {
                protected_losses.extend(missing_semantics(&protected_local, local_profile));
            }
            if peer_id != peer_preferred_id {
                protected_losses.extend(missing_semantics(&protected_peer, peer_profile));
            }

            if !protected_losses.is_empty() {
                all_protected_losses.extend(protected_losses.clone());
                refusal_reasons.push(format!(
                    "downgrade would drop protected semantics: {}",
                    protected_losses.into_iter().collect::<Vec<_>>().join(",")
                ));
                continue;
            }

            candidates.push(Candidate {
                local: local_profile,
                peer: peer_profile,
                common_capabilities: intersection_strings(
                    &local_profile.capabilities,
                    &peer_profile.capabilities,
                ),
                common_security_suites: intersection_strings(
                    &local_profile.security_suites,
                    &peer_profile.security_suites,
                ),
                local_to_peer,
                peer_to_local,
                downgrade,
                score: (
                    std::cmp::min(local_profile.version, peer_profile.version),
                    std::cmp::max(local_profile.version, peer_profile.version),
                    !downgrade,
                ),
            });
        }
    }

    candidates.sort_by(|left, right| right.score.cmp(&left.score));

    let Some(candidate) = candidates.into_iter().next() else {
        refusal_reasons.sort();
        refusal_reasons.dedup();
        return Ok(ProtocolNegotiationReceiptV1 {
            decision: NegotiationDecisionV1::Refused,
            selected_local_profile_id: None,
            selected_peer_profile_id: None,
            selected_local_version: None,
            selected_peer_version: None,
            local_to_peer: None,
            peer_to_local: None,
            common_capabilities: Vec::new(),
            common_security_suites: Vec::new(),
            downgrade: false,
            protected_semantics_lost: all_protected_losses.into_iter().collect(),
            refusal_reasons,
        });
    };

    let decision = if candidate.downgrade
        || candidate.local_to_peer.outcome == DirectionalCompatibilityOutcomeV1::Degraded
        || candidate.peer_to_local.outcome == DirectionalCompatibilityOutcomeV1::Degraded
    {
        NegotiationDecisionV1::Degraded
    } else {
        NegotiationDecisionV1::Qualified
    };

    Ok(ProtocolNegotiationReceiptV1 {
        decision,
        selected_local_profile_id: Some(candidate.local.profile_id()?),
        selected_peer_profile_id: Some(candidate.peer.profile_id()?),
        selected_local_version: Some(candidate.local.version),
        selected_peer_version: Some(candidate.peer.version),
        local_to_peer: Some(candidate.local_to_peer),
        peer_to_local: Some(candidate.peer_to_local),
        common_capabilities: candidate.common_capabilities,
        common_security_suites: candidate.common_security_suites,
        downgrade: candidate.downgrade,
        protected_semantics_lost: Vec::new(),
        refusal_reasons: Vec::new(),
    })
}

fn protected_semantics(profile: &ProtocolProfileV1) -> BTreeSet<String> {
    profile
        .semantics
        .iter()
        .filter(|semantic| semantic.critical && semantic.class.is_protected_by_default())
        .map(|semantic| semantic.id.clone())
        .collect()
}

fn missing_semantics(
    required: &BTreeSet<String>,
    selected: &ProtocolProfileV1,
) -> BTreeSet<String> {
    let selected_ids = selected
        .semantics
        .iter()
        .map(|semantic| semantic.id.clone())
        .collect::<BTreeSet<_>>();
    required.difference(&selected_ids).cloned().collect()
}

fn intersection_strings(left: &[String], right: &[String]) -> Vec<String> {
    let left = left.iter().cloned().collect::<BTreeSet<_>>();
    let right = right.iter().cloned().collect::<BTreeSet<_>>();
    left.intersection(&right).cloned().collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn semantic(
        id: &str,
        class: SemanticClassV1,
        critical: bool,
        unknown_behavior: UnknownSemanticBehaviorV1,
    ) -> ProtocolSemanticV1 {
        ProtocolSemanticV1 {
            id: id.to_string(),
            class,
            critical,
            unknown_behavior,
        }
    }

    fn profile(
        minor: u16,
        semantics: Vec<ProtocolSemanticV1>,
        capabilities: &[&str],
    ) -> ProtocolProfileV1 {
        ProtocolProfileV1 {
            family: "mycelix-core".to_string(),
            version: ProtocolVersionV1::new(1, minor, 0),
            capabilities: capabilities.iter().map(|value| (*value).to_string()).collect(),
            semantics,
            security_suites: vec!["suite-a".to_string()],
        }
    }

    #[test]
    fn profile_identity_is_order_independent_for_set_like_fields() {
        let first = profile(
            2,
            vec![
                semantic(
                    "privacy.expiry",
                    SemanticClassV1::PrivacyConstraint,
                    true,
                    UnknownSemanticBehaviorV1::DenyOperation,
                ),
                semantic(
                    "ui.label",
                    SemanticClassV1::Presentation,
                    false,
                    UnknownSemanticBehaviorV1::Ignore,
                ),
            ],
            &["alpha", "beta"],
        );
        let mut second = first.clone();
        second.semantics.reverse();
        second.capabilities.reverse();
        second.security_suites.reverse();

        assert_eq!(first.profile_id().unwrap(), second.profile_id().unwrap());
    }

    #[test]
    fn critical_semantics_cannot_be_marked_ignore() {
        let invalid = profile(
            1,
            vec![semantic(
                "authority.scope",
                SemanticClassV1::AuthorityConstraint,
                true,
                UnknownSemanticBehaviorV1::Ignore,
            )],
            &[],
        );

        assert_eq!(
            invalid.validate(),
            Err(ProtocolProfileErrorV1::CriticalSemanticCannotBeIgnored(
                "authority.scope".to_string()
            ))
        );
    }

    #[test]
    fn compatibility_is_directional() {
        let stronger = profile(
            2,
            vec![semantic(
                "privacy.expiry",
                SemanticClassV1::PrivacyConstraint,
                true,
                UnknownSemanticBehaviorV1::DenyOperation,
            )],
            &[],
        );
        let weaker = profile(1, vec![], &[]);

        let strong_to_weak = assess_directional_compatibility(&stronger, &weaker).unwrap();
        let weak_to_strong = assess_directional_compatibility(&weaker, &stronger).unwrap();

        assert_eq!(
            strong_to_weak.outcome,
            DirectionalCompatibilityOutcomeV1::Refused
        );
        assert_eq!(
            strong_to_weak.missing_critical_semantics,
            vec!["privacy.expiry".to_string()]
        );
        assert_eq!(
            weak_to_strong.outcome,
            DirectionalCompatibilityOutcomeV1::Qualified
        );
    }

    #[test]
    fn unknown_noncritical_semantic_degrades_and_lowers_claim_ceiling() {
        let sender = profile(
            2,
            vec![
                semantic(
                    "evidence.subject",
                    SemanticClassV1::Evidence,
                    true,
                    UnknownSemanticBehaviorV1::DenyOperation,
                ),
                semantic(
                    "ui.label",
                    SemanticClassV1::Presentation,
                    false,
                    UnknownSemanticBehaviorV1::Ignore,
                ),
            ],
            &[],
        );
        let receiver = profile(
            2,
            vec![semantic(
                "evidence.subject",
                SemanticClassV1::Evidence,
                true,
                UnknownSemanticBehaviorV1::DenyOperation,
            )],
            &[],
        );

        let result = assess_directional_compatibility(&sender, &receiver).unwrap();

        assert_eq!(
            result.outcome,
            DirectionalCompatibilityOutcomeV1::Degraded
        );
        assert_eq!(
            result.missing_noncritical_semantics,
            vec!["ui.label".to_string()]
        );
        assert_eq!(
            result.claim_ceiling.semantic_ids,
            vec!["evidence.subject".to_string()]
        );
    }

    #[test]
    fn conflicting_descriptor_for_same_semantic_refuses() {
        let sender = profile(
            2,
            vec![semantic(
                "authority.scope",
                SemanticClassV1::AuthorityConstraint,
                true,
                UnknownSemanticBehaviorV1::DenyOperation,
            )],
            &[],
        );
        let receiver = profile(
            2,
            vec![semantic(
                "authority.scope",
                SemanticClassV1::Advisory,
                true,
                UnknownSemanticBehaviorV1::RequireReview,
            )],
            &[],
        );

        let result = assess_directional_compatibility(&sender, &receiver).unwrap();
        assert_eq!(result.outcome, DirectionalCompatibilityOutcomeV1::Refused);
        assert_eq!(result.semantic_conflicts.len(), 1);
    }

    #[test]
    fn downgrade_cannot_drop_protected_critical_semantics() {
        let privacy = semantic(
            "privacy.expiry",
            SemanticClassV1::PrivacyConstraint,
            true,
            UnknownSemanticBehaviorV1::DenyOperation,
        );
        let local = ProtocolOfferV1 {
            preferred: profile(2, vec![privacy], &["base"]),
            alternatives: vec![profile(1, vec![], &["base"])],
        };
        let peer = ProtocolOfferV1 {
            preferred: profile(1, vec![], &["base"]),
            alternatives: vec![],
        };

        let receipt = negotiate_protocol(&local, &peer).unwrap();

        assert_eq!(receipt.decision, NegotiationDecisionV1::Refused);
        assert_eq!(
            receipt.protected_semantics_lost,
            vec!["privacy.expiry".to_string()]
        );
    }

    #[test]
    fn noncritical_version_mismatch_is_explicitly_degraded() {
        let optional_label = semantic(
            "ui.label",
            SemanticClassV1::Presentation,
            false,
            UnknownSemanticBehaviorV1::Ignore,
        );
        let local = ProtocolOfferV1 {
            preferred: profile(2, vec![optional_label], &["base", "pretty-ui"]),
            alternatives: vec![profile(1, vec![], &["base"])],
        };
        let peer = ProtocolOfferV1 {
            preferred: profile(1, vec![], &["base"]),
            alternatives: vec![],
        };

        let receipt = negotiate_protocol(&local, &peer).unwrap();

        assert_eq!(receipt.decision, NegotiationDecisionV1::Degraded);
        assert!(receipt.downgrade);
        assert_eq!(receipt.common_capabilities, vec!["base".to_string()]);
        assert!(receipt.protected_semantics_lost.is_empty());
    }
}
