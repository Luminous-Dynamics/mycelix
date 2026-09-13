// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Canonical semantic identity for ADMIN-003 administrative review policies.
//!
//! The ADMIN-003 semantic kernel remains crypto-neutral. This sibling crate
//! registers one deterministic content-identity profile so adoption,
//! currentness, review-registry, audit, and finality layers can bind the exact
//! review semantics without inventing incompatible hashing rules.
//!
//! Four facts remain deliberately separate:
//!
//! 1. review-policy semantic content;
//! 2. a verified canonical identity for that content;
//! 3. institutional adoption/currentness of that exact identity; and
//! 4. authority to review, stay, remedy, or create an external effect.
//!
//! This crate proves only (1) -> (2).

use mycelix_administrative_procedure::{
    ADMIN_003_PROTOCOL_VERSION, AdministrativeReviewError, AdministrativeReviewPolicy,
    ProcedureProfileId,
};
use mycelix_institutional_core::{Digest32, RulebookRef};
use std::collections::BTreeSet;
use std::fmt;

pub const ADMINISTRATIVE_REVIEW_POLICY_IDENTITY_PROFILE: &str =
    "mycelix-administrative-review-policy-v1-blake3-framed-semantic";
const DOMAIN_ADMINISTRATIVE_REVIEW_POLICY: &[u8] = b"mycelix/administrative-review/policy/v1";

/// Stable semantic content identity. This is neither adoption nor authority.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CanonicalAdministrativeReviewPolicyIdentity {
    pub digest: Digest32,
    pub profile: String,
}

impl CanonicalAdministrativeReviewPolicyIdentity {
    pub fn grants_authority(&self) -> bool {
        false
    }

    pub fn grants_review_authority(&self) -> bool {
        false
    }

    pub fn grants_external_effect_authority(&self) -> bool {
        false
    }
}

/// Opaque proof that one supplied ADMIN-003 review policy carries the registered
/// digest/profile for its own semantic obligations.
///
/// Deliberately not Clone/Serialize/Deserialize. Persisted policy records must
/// be requalified from their exact semantic content instead of deserializing a
/// positive identity token.
#[derive(Debug, PartialEq, Eq)]
pub struct QualifiedAdministrativeReviewPolicyIdentity {
    identity: CanonicalAdministrativeReviewPolicyIdentity,
    policy_ref: String,
    procedure_profile: ProcedureProfileId,
}

impl QualifiedAdministrativeReviewPolicyIdentity {
    pub fn identity(&self) -> &CanonicalAdministrativeReviewPolicyIdentity {
        &self.identity
    }

    pub fn policy_ref(&self) -> &str {
        &self.policy_ref
    }

    pub fn procedure_profile(&self) -> &ProcedureProfileId {
        &self.procedure_profile
    }

    pub fn grants_authority(&self) -> bool {
        false
    }

    pub fn grants_review_authority(&self) -> bool {
        false
    }

    pub fn grants_external_effect_authority(&self) -> bool {
        false
    }
}

/// Compute the canonical identity of an ADMIN-003 policy's review semantics.
///
/// `policy_ref` is provenance/locator metadata and is intentionally excluded.
/// `policy_digest` and `policy_digest_profile` are the claim being verified and
/// therefore cannot be inputs to their own digest.
pub fn administrative_review_policy_semantic_identity(
    policy: &AdministrativeReviewPolicy,
) -> Result<CanonicalAdministrativeReviewPolicyIdentity, AdministrativeReviewPolicyIdentityError> {
    // Reuse ADMIN-003 structural validation without circularly trusting the
    // caller's identity claim. These sentinel values are validation-only and
    // are never included in the canonical digest.
    let mut structural = policy.clone();
    structural.policy_digest = Digest32([1; 32]);
    structural.policy_digest_profile = ADMINISTRATIVE_REVIEW_POLICY_IDENTITY_PROFILE.into();
    structural
        .validate()
        .map_err(AdministrativeReviewPolicyIdentityError::ReviewPolicy)?;

    if policy.protocol_version != ADMIN_003_PROTOCOL_VERSION {
        return Err(AdministrativeReviewPolicyIdentityError::ReviewPolicy(
            AdministrativeReviewError::WrongProtocolVersion,
        ));
    }

    let review_roles = canonical_strings(
        policy
            .accepted_review_roles
            .iter()
            .map(|role| role.as_str()),
    );
    let remedy_types = canonical_strings(policy.allowed_remedy_types.iter().map(String::as_str));

    let mut hasher = blake3::Hasher::new();
    hasher.update(DOMAIN_ADMINISTRATIVE_REVIEW_POLICY);

    frame(
        &mut hasher,
        ADMINISTRATIVE_REVIEW_POLICY_IDENTITY_PROFILE.as_bytes(),
    );
    frame(&mut hasher, policy.protocol_version.as_bytes());
    frame(&mut hasher, policy.procedure_profile.as_str().as_bytes());

    frame(&mut hasher, policy.source_institution.as_str().as_bytes());
    frame_optional_id(
        &mut hasher,
        policy.source_jurisdiction.as_ref().map(|value| value.as_str()),
    );
    frame_rulebook(&mut hasher, &policy.source_rulebook);

    frame(&mut hasher, policy.review_forum.as_str().as_bytes());
    frame_optional_id(
        &mut hasher,
        policy.review_jurisdiction.as_ref().map(|value| value.as_str()),
    );
    frame_rulebook(&mut hasher, &policy.review_rulebook);

    frame(&mut hasher, policy.review_capability.as_str().as_bytes());
    frame(&mut hasher, policy.stay_capability.as_str().as_bytes());
    frame(&mut hasher, policy.remedy_capability.as_str().as_bytes());

    frame_count(&mut hasher, review_roles.len());
    for role in review_roles {
        frame(&mut hasher, role.as_bytes());
    }

    frame(&mut hasher, &policy.challenge_window_ms.to_le_bytes());
    frame(&mut hasher, &policy.appeal_window_ms.to_le_bytes());
    frame(&mut hasher, &policy.finality_delay_ms.to_le_bytes());
    frame(
        &mut hasher,
        &[u8::from(policy.require_independent_reviewer)],
    );

    frame_count(&mut hasher, remedy_types.len());
    for remedy_type in remedy_types {
        frame(&mut hasher, remedy_type.as_bytes());
    }

    Ok(CanonicalAdministrativeReviewPolicyIdentity {
        digest: Digest32(*hasher.finalize().as_bytes()),
        profile: ADMINISTRATIVE_REVIEW_POLICY_IDENTITY_PROFILE.into(),
    })
}

/// Verify the caller's digest/profile claim against the exact review semantics.
pub fn qualify_administrative_review_policy_identity(
    policy: &AdministrativeReviewPolicy,
) -> Result<QualifiedAdministrativeReviewPolicyIdentity, AdministrativeReviewPolicyIdentityError> {
    if policy.policy_digest_profile != ADMINISTRATIVE_REVIEW_POLICY_IDENTITY_PROFILE {
        return Err(AdministrativeReviewPolicyIdentityError::UnsupportedIdentityProfile);
    }

    let identity = administrative_review_policy_semantic_identity(policy)?;
    if policy.policy_digest != identity.digest {
        return Err(AdministrativeReviewPolicyIdentityError::SemanticDigestMismatch);
    }

    Ok(QualifiedAdministrativeReviewPolicyIdentity {
        identity,
        policy_ref: policy.policy_ref.clone(),
        procedure_profile: policy.procedure_profile.clone(),
    })
}

fn canonical_strings<'a>(values: impl Iterator<Item = &'a str>) -> Vec<&'a str> {
    values.collect::<BTreeSet<_>>().into_iter().collect()
}

fn frame_rulebook(hasher: &mut blake3::Hasher, rulebook: &RulebookRef) {
    frame(hasher, rulebook.id.as_str().as_bytes());
    frame(hasher, rulebook.version.as_bytes());
    frame(hasher, &rulebook.digest.0);
}

fn frame_optional_id(hasher: &mut blake3::Hasher, value: Option<&str>) {
    match value {
        None => frame(hasher, &[0]),
        Some(value) => {
            frame(hasher, &[1]);
            frame(hasher, value.as_bytes());
        }
    }
}

fn frame_count(hasher: &mut blake3::Hasher, count: usize) {
    frame(hasher, &(count as u64).to_le_bytes());
}

fn frame(hasher: &mut blake3::Hasher, bytes: &[u8]) {
    hasher.update(&(bytes.len() as u64).to_le_bytes());
    hasher.update(bytes);
}

#[derive(Debug, PartialEq, Eq)]
pub enum AdministrativeReviewPolicyIdentityError {
    ReviewPolicy(AdministrativeReviewError),
    UnsupportedIdentityProfile,
    SemanticDigestMismatch,
}

impl From<AdministrativeReviewError> for AdministrativeReviewPolicyIdentityError {
    fn from(value: AdministrativeReviewError) -> Self {
        Self::ReviewPolicy(value)
    }
}

impl fmt::Display for AdministrativeReviewPolicyIdentityError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::ReviewPolicy(error) => write!(f, "invalid administrative review policy: {error}"),
            Self::UnsupportedIdentityProfile => {
                write!(f, "unsupported administrative review policy identity profile")
            }
            Self::SemanticDigestMismatch => {
                write!(f, "administrative review policy semantic digest mismatch")
            }
        }
    }
}

impl std::error::Error for AdministrativeReviewPolicyIdentityError {}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_administrative_procedure::ProcedureProfileId;
    use mycelix_institutional_core::{
        CapabilityId, InstitutionId, JurisdictionId, RoleId, RulebookId,
    };

    fn digest(byte: u8) -> Digest32 {
        Digest32([byte; 32])
    }

    fn source_rulebook() -> RulebookRef {
        RulebookRef {
            id: RulebookId::new("rulebook:administration:v1").unwrap(),
            version: "1.0.0".into(),
            digest: digest(11),
        }
    }

    fn review_rulebook() -> RulebookRef {
        RulebookRef {
            id: RulebookId::new("rulebook:administrative-review:v1").unwrap(),
            version: "1.0.0".into(),
            digest: digest(12),
        }
    }

    fn base_policy() -> AdministrativeReviewPolicy {
        AdministrativeReviewPolicy {
            protocol_version: ADMIN_003_PROTOCOL_VERSION.into(),
            procedure_profile: ProcedureProfileId::new("procedure:permit:v1").unwrap(),
            policy_ref: "registry:review-policy:permit:v1".into(),
            policy_digest: digest(1),
            policy_digest_profile: ADMINISTRATIVE_REVIEW_POLICY_IDENTITY_PROFILE.into(),
            source_institution: InstitutionId::new("institution:permit-office").unwrap(),
            source_jurisdiction: Some(JurisdictionId::new("jurisdiction:city").unwrap()),
            source_rulebook: source_rulebook(),
            review_forum: InstitutionId::new("institution:appeals-board").unwrap(),
            review_jurisdiction: Some(JurisdictionId::new("jurisdiction:city").unwrap()),
            review_rulebook: review_rulebook(),
            review_capability: CapabilityId::new("administration.review.decide").unwrap(),
            stay_capability: CapabilityId::new("administration.review.stay").unwrap(),
            remedy_capability: CapabilityId::new("administration.review.remedy").unwrap(),
            accepted_review_roles: vec![
                RoleId::new("role:appeals-chair").unwrap(),
                RoleId::new("role:appeals-member").unwrap(),
            ],
            challenge_window_ms: 86_400_000,
            appeal_window_ms: 86_400_000,
            finality_delay_ms: 3_600_000,
            require_independent_reviewer: true,
            allowed_remedy_types: vec!["reconsider".into(), "correct-record".into()],
        }
    }

    fn canonical_policy() -> AdministrativeReviewPolicy {
        let mut policy = base_policy();
        policy.policy_digest = administrative_review_policy_semantic_identity(&policy)
            .unwrap()
            .digest;
        policy
    }

    fn identity(policy: &AdministrativeReviewPolicy) -> Digest32 {
        administrative_review_policy_semantic_identity(policy)
            .unwrap()
            .digest
    }

    #[test]
    fn exact_policy_identity_qualifies_without_granting_authority() {
        let policy = canonical_policy();
        let qualified = qualify_administrative_review_policy_identity(&policy).unwrap();
        assert_eq!(qualified.identity().digest, policy.policy_digest);
        assert_eq!(qualified.policy_ref(), policy.policy_ref);
        assert!(!qualified.grants_authority());
        assert!(!qualified.grants_review_authority());
        assert!(!qualified.grants_external_effect_authority());
    }

    #[test]
    fn role_and_remedy_order_are_non_semantic() {
        let policy = base_policy();
        let expected = identity(&policy);

        let mut reordered_roles = policy.clone();
        reordered_roles.accepted_review_roles.reverse();
        assert_eq!(identity(&reordered_roles), expected);

        let mut reordered_remedies = policy;
        reordered_remedies.allowed_remedy_types.reverse();
        assert_eq!(identity(&reordered_remedies), expected);
    }

    #[test]
    fn provenance_locator_is_not_semantic_identity() {
        let policy = base_policy();
        let expected = identity(&policy);
        let mut mirrored = policy;
        mirrored.policy_ref = "mirror:review-policy:permit:v1".into();
        assert_eq!(identity(&mirrored), expected);
    }

    #[test]
    fn procedure_and_source_scope_are_identity_bearing() {
        let policy = base_policy();
        let expected = identity(&policy);

        let mut changed = policy.clone();
        changed.procedure_profile = ProcedureProfileId::new("procedure:permit:v2").unwrap();
        assert_ne!(identity(&changed), expected);

        let mut changed = policy.clone();
        changed.source_institution = InstitutionId::new("institution:licensing-office").unwrap();
        assert_ne!(identity(&changed), expected);

        let mut changed = policy.clone();
        changed.source_jurisdiction = None;
        assert_ne!(identity(&changed), expected);

        let mut changed = policy;
        changed.source_rulebook.digest = digest(77);
        assert_ne!(identity(&changed), expected);
    }

    #[test]
    fn review_forum_and_rulebook_are_identity_bearing() {
        let policy = base_policy();
        let expected = identity(&policy);

        let mut changed = policy.clone();
        changed.review_forum = InstitutionId::new("institution:review-tribunal").unwrap();
        assert_ne!(identity(&changed), expected);

        let mut changed = policy.clone();
        changed.review_jurisdiction = None;
        assert_ne!(identity(&changed), expected);

        let mut changed = policy;
        changed.review_rulebook.version = "2.0.0".into();
        assert_ne!(identity(&changed), expected);
    }

    #[test]
    fn review_stay_and_remedy_capabilities_are_identity_bearing() {
        let policy = base_policy();
        let expected = identity(&policy);

        let mut changed = policy.clone();
        changed.review_capability = CapabilityId::new("administration.review.alt").unwrap();
        assert_ne!(identity(&changed), expected);

        let mut changed = policy.clone();
        changed.stay_capability = CapabilityId::new("administration.review.stay.alt").unwrap();
        assert_ne!(identity(&changed), expected);

        let mut changed = policy;
        changed.remedy_capability = CapabilityId::new("administration.review.remedy.alt").unwrap();
        assert_ne!(identity(&changed), expected);
    }

    #[test]
    fn windows_and_independence_rule_are_identity_bearing() {
        let policy = base_policy();
        let expected = identity(&policy);

        let mut changed = policy.clone();
        changed.challenge_window_ms += 1;
        assert_ne!(identity(&changed), expected);

        let mut changed = policy.clone();
        changed.appeal_window_ms += 1;
        assert_ne!(identity(&changed), expected);

        let mut changed = policy.clone();
        changed.finality_delay_ms += 1;
        assert_ne!(identity(&changed), expected);

        let mut changed = policy;
        changed.require_independent_reviewer = false;
        assert_ne!(identity(&changed), expected);
    }

    #[test]
    fn accepted_roles_and_remedy_set_are_identity_bearing() {
        let policy = base_policy();
        let expected = identity(&policy);

        let mut changed = policy.clone();
        changed
            .accepted_review_roles
            .push(RoleId::new("role:review-auditor").unwrap());
        assert_ne!(identity(&changed), expected);

        let mut changed = policy;
        changed.allowed_remedy_types.push("refund-fee".into());
        assert_ne!(identity(&changed), expected);
    }

    #[test]
    fn stale_digest_cannot_be_copied_onto_changed_semantics() {
        let policy = canonical_policy();
        let old_digest = policy.policy_digest;
        let mut altered = policy;
        altered.challenge_window_ms += 1;
        altered.policy_digest = old_digest;
        assert_eq!(
            qualify_administrative_review_policy_identity(&altered).unwrap_err(),
            AdministrativeReviewPolicyIdentityError::SemanticDigestMismatch
        );
    }

    #[test]
    fn unsupported_profile_fails_closed() {
        let mut policy = canonical_policy();
        policy.policy_digest_profile = "mycelix-administrative-review-policy-v999".into();
        assert_eq!(
            qualify_administrative_review_policy_identity(&policy).unwrap_err(),
            AdministrativeReviewPolicyIdentityError::UnsupportedIdentityProfile
        );
    }

    #[test]
    fn duplicate_roles_remain_admin003_errors_instead_of_alternate_encodings() {
        let mut policy = base_policy();
        policy
            .accepted_review_roles
            .push(policy.accepted_review_roles[0].clone());
        assert_eq!(
            administrative_review_policy_semantic_identity(&policy).unwrap_err(),
            AdministrativeReviewPolicyIdentityError::ReviewPolicy(
                AdministrativeReviewError::DuplicateReviewRole
            )
        );
    }

    #[test]
    fn duplicate_remedies_remain_admin003_errors_instead_of_alternate_encodings() {
        let mut policy = base_policy();
        policy
            .allowed_remedy_types
            .push(policy.allowed_remedy_types[0].clone());
        assert_eq!(
            administrative_review_policy_semantic_identity(&policy).unwrap_err(),
            AdministrativeReviewPolicyIdentityError::ReviewPolicy(
                AdministrativeReviewError::DuplicateRemedyType
            )
        );
    }
}
