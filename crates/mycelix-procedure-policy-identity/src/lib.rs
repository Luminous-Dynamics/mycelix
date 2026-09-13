// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Canonical semantic identities for ADMIN-002 procedure policies.
//!
//! The administrative-procedure crate stays transport- and crypto-neutral. This
//! sibling crate registers one deterministic content-identity profile so policy
//! providers, currentness adapters, audits, and later administrative layers do
//! not invent incompatible hashing rules.
//!
//! Three facts remain deliberately separate:
//!
//! 1. policy semantic content;
//! 2. a claim that semantic content has one exact digest/profile; and
//! 3. authoritative currentness of that exact policy.
//!
//! This crate proves (1) and (2). It does not prove (3).

use mycelix_administrative_procedure::{
    ProceduralCompletenessError, ProceduralCompletenessPolicy, ProcedureProfileId,
    ReasonsRequirement, ResponseModeRequirement,
};
use mycelix_institutional_core::{Digest32, PrincipalId};
use std::collections::BTreeSet;
use std::fmt;

pub const PROCEDURAL_POLICY_IDENTITY_PROFILE: &str =
    "mycelix-administrative-procedure-policy-v1-blake3-framed-semantic";
const DOMAIN_PROCEDURAL_POLICY: &[u8] = b"mycelix/administrative-procedure/policy/v1";

/// Stable semantic content identity. This is not authority or currentness.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CanonicalProceduralPolicyIdentity {
    pub digest: Digest32,
    pub profile: String,
}

impl CanonicalProceduralPolicyIdentity {
    pub fn grants_authority(&self) -> bool {
        false
    }

    pub fn grants_external_effect_authority(&self) -> bool {
        false
    }
}

/// Opaque proof that one exact policy's claimed digest/profile matches its
/// semantic obligations.
///
/// Deliberately not Clone/Serialize/Deserialize. Persisted claims must be
/// reverified from the policy object rather than deserializing a positive token.
#[derive(Debug, PartialEq, Eq)]
pub struct QualifiedProceduralPolicyIdentity {
    identity: CanonicalProceduralPolicyIdentity,
    policy_ref: String,
    procedure_profile: ProcedureProfileId,
}

impl QualifiedProceduralPolicyIdentity {
    pub fn identity(&self) -> &CanonicalProceduralPolicyIdentity {
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

    pub fn grants_external_effect_authority(&self) -> bool {
        false
    }
}

/// Compute the canonical identity of the policy's semantic obligations.
///
/// `policy_ref` is provenance/locator metadata, so mirror or registry location
/// changes do not rewrite semantic identity. `policy_digest` and
/// `policy_digest_profile` are the claim being checked and therefore cannot be
/// inputs to their own digest.
pub fn procedural_policy_semantic_identity(
    policy: &ProceduralCompletenessPolicy,
) -> Result<CanonicalProceduralPolicyIdentity, ProceduralPolicyIdentityError> {
    // Reuse ADMIN-002 structural validation without trusting the caller's
    // identity claim. These sentinel values are validation-only and are never
    // included in the canonical digest.
    let mut structural = policy.clone();
    structural.policy_digest = Digest32([1; 32]);
    structural.policy_digest_profile = PROCEDURAL_POLICY_IDENTITY_PROFILE.into();
    structural
        .validate()
        .map_err(ProceduralPolicyIdentityError::Procedural)?;

    let notices = canonical_principals(&policy.required_notice_recipients);
    let responses = canonical_principals(&policy.required_response_recipients);

    let mut hasher = blake3::Hasher::new();
    hasher.update(DOMAIN_PROCEDURAL_POLICY);
    frame(&mut hasher, PROCEDURAL_POLICY_IDENTITY_PROFILE.as_bytes());
    frame(&mut hasher, policy.protocol_version.as_bytes());
    frame(&mut hasher, policy.procedure_profile.as_str().as_bytes());

    frame(&mut hasher, &(notices.len() as u64).to_le_bytes());
    for principal in notices {
        frame(&mut hasher, principal.as_bytes());
    }

    frame(&mut hasher, &(responses.len() as u64).to_le_bytes());
    for principal in responses {
        frame(&mut hasher, principal.as_bytes());
    }

    frame(&mut hasher, &[response_mode_code(policy.response_mode)]);
    frame(&mut hasher, &policy.min_response_window_ms.to_le_bytes());
    frame(&mut hasher, &[reasons_code(policy.reasons)]);

    Ok(CanonicalProceduralPolicyIdentity {
        digest: Digest32(*hasher.finalize().as_bytes()),
        profile: PROCEDURAL_POLICY_IDENTITY_PROFILE.into(),
    })
}

/// Verify that the caller's exact digest/profile claim matches semantic content.
///
/// Success still says nothing about whether an institution has selected this
/// policy as current. That requires a separate authoritative provider proof.
pub fn qualify_procedural_policy_identity(
    policy: &ProceduralCompletenessPolicy,
) -> Result<QualifiedProceduralPolicyIdentity, ProceduralPolicyIdentityError> {
    if policy.policy_digest_profile != PROCEDURAL_POLICY_IDENTITY_PROFILE {
        return Err(ProceduralPolicyIdentityError::UnsupportedIdentityProfile);
    }

    let identity = procedural_policy_semantic_identity(policy)?;
    if policy.policy_digest != identity.digest {
        return Err(ProceduralPolicyIdentityError::SemanticDigestMismatch);
    }

    Ok(QualifiedProceduralPolicyIdentity {
        identity,
        policy_ref: policy.policy_ref.clone(),
        procedure_profile: policy.procedure_profile.clone(),
    })
}

fn canonical_principals(principals: &[PrincipalId]) -> Vec<&str> {
    principals
        .iter()
        .map(PrincipalId::as_str)
        .collect::<BTreeSet<_>>()
        .into_iter()
        .collect()
}

fn response_mode_code(mode: ResponseModeRequirement) -> u8 {
    match mode {
        ResponseModeRequirement::None => 0,
        ResponseModeRequirement::Written => 1,
        ResponseModeRequirement::Hearing => 2,
        ResponseModeRequirement::WrittenOrHearing => 3,
    }
}

fn reasons_code(reasons: ReasonsRequirement) -> u8 {
    match reasons {
        ReasonsRequirement::NotRequiredByProfile => 0,
        ReasonsRequirement::AtLeastOne => 1,
    }
}

fn frame(hasher: &mut blake3::Hasher, bytes: &[u8]) {
    hasher.update(&(bytes.len() as u64).to_le_bytes());
    hasher.update(bytes);
}

#[derive(Debug, PartialEq, Eq)]
pub enum ProceduralPolicyIdentityError {
    Procedural(ProceduralCompletenessError),
    UnsupportedIdentityProfile,
    SemanticDigestMismatch,
}

impl fmt::Display for ProceduralPolicyIdentityError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Procedural(error) => write!(f, "{error}"),
            Self::UnsupportedIdentityProfile => {
                write!(f, "unsupported procedural policy identity profile")
            }
            Self::SemanticDigestMismatch => {
                write!(f, "procedural policy semantic digest does not match claim")
            }
        }
    }
}

impl std::error::Error for ProceduralPolicyIdentityError {}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_administrative_procedure::ADMIN_002_PROTOCOL_VERSION;

    fn p(value: &str) -> PrincipalId {
        PrincipalId::new(value).unwrap()
    }

    fn policy() -> ProceduralCompletenessPolicy {
        ProceduralCompletenessPolicy {
            protocol_version: ADMIN_002_PROTOCOL_VERSION.into(),
            procedure_profile: ProcedureProfileId::new("procedure:permit:v1").unwrap(),
            policy_ref: "registry:procedure-policy:permit:v1".into(),
            policy_digest: Digest32([9; 32]),
            policy_digest_profile: "unverified-input".into(),
            required_notice_recipients: vec![p("did:example:applicant"), p("did:example:owner")],
            required_response_recipients: vec![
                p("did:example:applicant"),
                p("did:example:owner"),
            ],
            response_mode: ResponseModeRequirement::WrittenOrHearing,
            min_response_window_ms: 86_400_000,
            reasons: ReasonsRequirement::AtLeastOne,
        }
    }

    fn canonically_claimed_policy() -> ProceduralCompletenessPolicy {
        let mut policy = policy();
        let identity = procedural_policy_semantic_identity(&policy).unwrap();
        policy.policy_digest = identity.digest;
        policy.policy_digest_profile = identity.profile;
        policy
    }

    #[test]
    fn exact_semantics_verify_against_exact_claim() {
        let policy = canonically_claimed_policy();
        let qualified = qualify_procedural_policy_identity(&policy).unwrap();
        assert_eq!(qualified.identity().digest, policy.policy_digest);
        assert_eq!(
            qualified.identity().profile,
            PROCEDURAL_POLICY_IDENTITY_PROFILE
        );
        assert_eq!(qualified.policy_ref(), policy.policy_ref);
        assert_eq!(qualified.procedure_profile(), &policy.procedure_profile);
        assert!(!qualified.grants_authority());
        assert!(!qualified.grants_external_effect_authority());
    }

    #[test]
    fn set_order_does_not_change_semantic_identity() {
        let first = policy();
        let mut second = first.clone();
        second.required_notice_recipients.reverse();
        second.required_response_recipients.reverse();
        assert_eq!(
            procedural_policy_semantic_identity(&first).unwrap(),
            procedural_policy_semantic_identity(&second).unwrap()
        );
    }

    #[test]
    fn provenance_locator_does_not_change_semantic_identity() {
        let first = policy();
        let mut second = first.clone();
        second.policy_ref = "registry:mirror:procedure-policy:permit:v1".into();
        assert_eq!(
            procedural_policy_semantic_identity(&first).unwrap(),
            procedural_policy_semantic_identity(&second).unwrap()
        );
    }

    #[test]
    fn each_semantic_dimension_changes_identity() {
        let first = policy();
        let baseline = procedural_policy_semantic_identity(&first).unwrap().digest;

        let mut changed_profile = first.clone();
        changed_profile.procedure_profile =
            ProcedureProfileId::new("procedure:permit:v2").unwrap();
        assert_ne!(
            baseline,
            procedural_policy_semantic_identity(&changed_profile)
                .unwrap()
                .digest
        );

        let mut changed_notice = first.clone();
        changed_notice
            .required_notice_recipients
            .push(p("did:example:tenant"));
        assert_ne!(
            baseline,
            procedural_policy_semantic_identity(&changed_notice)
                .unwrap()
                .digest
        );

        let mut changed_response = first.clone();
        changed_response.required_response_recipients = vec![p("did:example:applicant")];
        assert_ne!(
            baseline,
            procedural_policy_semantic_identity(&changed_response)
                .unwrap()
                .digest
        );

        let mut changed_mode = first.clone();
        changed_mode.response_mode = ResponseModeRequirement::Hearing;
        assert_ne!(
            baseline,
            procedural_policy_semantic_identity(&changed_mode)
                .unwrap()
                .digest
        );

        let mut changed_window = first.clone();
        changed_window.min_response_window_ms += 1;
        assert_ne!(
            baseline,
            procedural_policy_semantic_identity(&changed_window)
                .unwrap()
                .digest
        );

        let mut changed_reasons = first;
        changed_reasons.reasons = ReasonsRequirement::NotRequiredByProfile;
        assert_ne!(
            baseline,
            procedural_policy_semantic_identity(&changed_reasons)
                .unwrap()
                .digest
        );
    }

    #[test]
    fn same_claim_cannot_be_reused_for_changed_semantics() {
        let claimed = canonically_claimed_policy();
        let mut substituted = claimed.clone();
        substituted.min_response_window_ms += 1;
        assert_eq!(
            qualify_procedural_policy_identity(&substituted).unwrap_err(),
            ProceduralPolicyIdentityError::SemanticDigestMismatch
        );
    }

    #[test]
    fn unsupported_profile_fails_before_digest_acceptance() {
        let mut claimed = canonically_claimed_policy();
        claimed.policy_digest_profile = "other-profile".into();
        assert_eq!(
            qualify_procedural_policy_identity(&claimed).unwrap_err(),
            ProceduralPolicyIdentityError::UnsupportedIdentityProfile
        );
    }

    #[test]
    fn malformed_or_duplicate_semantics_remain_admin_002_errors() {
        let mut duplicate = policy();
        duplicate
            .required_notice_recipients
            .push(p("did:example:applicant"));
        assert!(matches!(
            procedural_policy_semantic_identity(&duplicate),
            Err(ProceduralPolicyIdentityError::Procedural(
                ProceduralCompletenessError::DuplicatePolicyRecipient
            ))
        ));
    }
}
