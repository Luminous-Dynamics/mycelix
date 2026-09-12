//! Generation-bound currentness for institution-adopted provider-profile roots.
//!
//! This crate deliberately reuses `AuthoritySubjectKind::SigningPolicy`: the
//! subject is exactly a signing policy, while namespace + policy id + exact
//! provider-trust identity keep it disjoint from other signing-policy domains.

use mycelix_authority_freshness::{
    qualify_current_freshness, AuthoritySubjectKind, AuthoritySubjectRef, FreshnessError,
    ProfiledDigest, VerifiedAuthorityFreshness,
};
use mycelix_institutional_core::Digest32;
use mycelix_integration_execution_binding::ProviderProfileTrustRoot;
use mycelix_integration_provider_trust_policy::{
    ProviderTrustPolicyError, QualifiedAdoptedProviderTrustPolicy, FRESHNESS_NAMESPACE_SUFFIX,
    POLICY_IDENTITY_PROFILE,
};
use thiserror::Error;

pub const CURRENT_TRUST_PROFILE: &str =
    "mycelix-integration-provider-trust-current-v1-blake3-framed";
const DOMAIN_CURRENT_TRUST: &[u8] = b"mycelix/integration/provider-trust-current/v1";

/// Process-local proof that one institution-adopted provider-profile signing root
/// is the exact Active generation under the shared closed-set freshness theorem.
#[derive(Clone, Debug)]
pub struct QualifiedCurrentProviderTrustPolicy {
    adopted_policy: QualifiedAdoptedProviderTrustPolicy,
    subject: AuthoritySubjectRef,
    current_root: ProviderProfileTrustRoot,
    freshness_digest: Digest32,
    freshness_profile: String,
    qualification_digest: Digest32,
    verified_at_ms: u64,
    valid_until_ms: u64,
}

impl QualifiedCurrentProviderTrustPolicy {
    pub fn adopted_policy(&self) -> &QualifiedAdoptedProviderTrustPolicy {
        &self.adopted_policy
    }

    pub fn subject(&self) -> &AuthoritySubjectRef {
        &self.subject
    }

    pub fn trust_root(&self) -> &ProviderProfileTrustRoot {
        &self.current_root
    }

    pub fn freshness_digest(&self) -> Digest32 {
        self.freshness_digest
    }

    pub fn freshness_profile(&self) -> &str {
        &self.freshness_profile
    }

    pub fn qualification_digest(&self) -> Digest32 {
        self.qualification_digest
    }

    pub fn qualification_profile(&self) -> &'static str {
        CURRENT_TRUST_PROFILE
    }

    pub fn verified_at_ms(&self) -> u64 {
        self.verified_at_ms
    }

    pub fn valid_until_ms(&self) -> u64 {
        self.valid_until_ms
    }

    pub const fn institutional_provider_root_adopted_here(&self) -> bool {
        true
    }

    pub const fn generation_currentness_verified_here(&self) -> bool {
        true
    }

    pub const fn raw_caller_root_accepted_here(&self) -> bool {
        false
    }

    pub const fn freshness_origin_verified_here(&self) -> bool {
        false
    }

    pub const fn provider_profile_verified_here(&self) -> bool {
        false
    }

    pub const fn grants_execution_authority(&self) -> bool {
        false
    }
}

pub fn qualify_current_provider_trust_policy(
    adopted_policy: &QualifiedAdoptedProviderTrustPolicy,
    freshness: &VerifiedAuthorityFreshness,
    now_ms: u64,
) -> Result<QualifiedCurrentProviderTrustPolicy, ProviderTrustCurrentnessError> {
    if now_ms == 0
        || adopted_policy.verified_at_ms() > now_ms
        || adopted_policy.valid_until_ms() <= now_ms
    {
        return Err(ProviderTrustCurrentnessError::AdoptedPolicyNotLive);
    }

    let policy = adopted_policy.policy();
    if freshness.snapshot.generation != policy.generation {
        return Err(ProviderTrustCurrentnessError::GenerationMismatch {
            policy: policy.generation,
            freshness: freshness.snapshot.generation,
        });
    }
    if freshness.snapshot.effective_at_ms < policy.valid_from_ms {
        return Err(ProviderTrustCurrentnessError::FreshnessPredatesPolicy);
    }

    let subject = AuthoritySubjectRef {
        kind: AuthoritySubjectKind::SigningPolicy,
        namespace: freshness_namespace(policy.institution.as_str()),
        subject_id: policy.policy_id.clone(),
        identity: ProfiledDigest {
            digest: adopted_policy.policy_digest(),
            profile: POLICY_IDENTITY_PROFILE.to_owned(),
        },
    };

    let current = qualify_current_freshness(
        std::slice::from_ref(&subject),
        std::slice::from_ref(freshness),
        now_ms,
    )?;

    let verified_at_ms = adopted_policy.verified_at_ms().max(current.verified_at_ms);
    let valid_until_ms = adopted_policy.valid_until_ms().min(current.lease_until_ms);
    if verified_at_ms > now_ms || valid_until_ms <= now_ms {
        return Err(ProviderTrustCurrentnessError::NoUsableCurrentWindow);
    }

    let current_root = adopted_policy.candidate_root().clone();
    if current_root.current_generation() != policy.generation {
        return Err(ProviderTrustCurrentnessError::RootGenerationMismatch);
    }

    let qualification_digest = current_trust_digest(
        adopted_policy.qualification_digest(),
        current.freshness_digest,
        current_root.root_commitment().digest,
    );

    Ok(QualifiedCurrentProviderTrustPolicy {
        adopted_policy: adopted_policy.clone(),
        subject,
        current_root,
        freshness_digest: current.freshness_digest,
        freshness_profile: current.freshness_profile,
        qualification_digest,
        verified_at_ms,
        valid_until_ms,
    })
}

fn freshness_namespace(institution: &str) -> String {
    format!("{institution}/{FRESHNESS_NAMESPACE_SUFFIX}")
}

fn current_trust_digest(
    adopted_qualification: Digest32,
    freshness: Digest32,
    root_digest: [u8; 32],
) -> Digest32 {
    let mut h = blake3::Hasher::new();
    h.update(DOMAIN_CURRENT_TRUST);
    frame(&mut h, CURRENT_TRUST_PROFILE.as_bytes());
    frame(&mut h, &adopted_qualification.0);
    frame(&mut h, &freshness.0);
    frame(&mut h, &root_digest);
    Digest32(*h.finalize().as_bytes())
}

fn frame(h: &mut blake3::Hasher, bytes: &[u8]) {
    h.update(&(bytes.len() as u64).to_le_bytes());
    h.update(bytes);
}

#[derive(Debug, Error)]
pub enum ProviderTrustCurrentnessError {
    #[error("adopted provider trust policy is not live")]
    AdoptedPolicyNotLive,
    #[error("provider trust policy generation {policy} differs from freshness generation {freshness}")]
    GenerationMismatch { policy: u64, freshness: u64 },
    #[error("provider trust freshness predates the adopted policy")]
    FreshnessPredatesPolicy,
    #[error("provider trust currentness leaves no usable window")]
    NoUsableCurrentWindow,
    #[error("retained provider root generation differs from policy generation")]
    RootGenerationMismatch,
    #[error(transparent)]
    Freshness(#[from] FreshnessError),
    #[error(transparent)]
    Policy(#[from] ProviderTrustPolicyError),
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn provider_signing_namespace_is_explicit_and_disjoint() {
        assert_eq!(
            freshness_namespace("institution-1"),
            "institution-1/integration-provider-profile-signing"
        );
    }

    #[test]
    fn current_digest_changes_with_root() {
        let a = current_trust_digest(Digest32([1; 32]), Digest32([2; 32]), [3; 32]);
        let b = current_trust_digest(Digest32([1; 32]), Digest32([2; 32]), [4; 32]);
        assert_ne!(a, b);
    }
}
