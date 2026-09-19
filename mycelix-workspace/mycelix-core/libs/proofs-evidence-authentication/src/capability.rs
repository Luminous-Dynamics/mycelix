use serde::{Deserialize, Serialize};

use crate::{
    AuthenticationFreshnessPolicyV1, AuthenticationPolicyErrorV1, GitObjectIdV1,
    QualificationReceiptDigestV1, ReceiptAuthenticationPolicyV1, Sha256DigestV1,
    TransparencyPolicyV1, WorkflowRevisionPolicyV1,
};

/// Facts established by a concrete cryptographic verifier before policy binding.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct VerifiedSignerIdentityV1 {
    pub oidc_issuer: String,
    pub source_repository: String,
    pub source_repository_owner: String,
    pub signer_workflow: String,
    pub signer_workflow_revision: GitObjectIdV1,
    pub source_revision: GitObjectIdV1,
    pub source_ref: Option<String>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct VerifiedQualificationPredicateV1 {
    pub predicate_type: String,
    pub predicate_schema: String,
    pub receipt_digest: QualificationReceiptDigestV1,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct VerifiedTransparencyV1 {
    pub public_transparency_verified: bool,
    pub timestamp_verified: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct VerifiedFreshnessV1 {
    pub current_at_verification: bool,
    pub age_seconds: Option<u64>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct VerifiedAuthenticationContextV1 {
    pub trusted_root_profile: String,
    pub trusted_root_digest: Sha256DigestV1,
    pub transparency: VerifiedTransparencyV1,
    pub freshness: VerifiedFreshnessV1,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum AuthenticatedReceiptAuthorityV1 {
    ReceiptAuthenticationOnly,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum AuthenticatedCapabilityConstructionErrorV1 {
    InvalidPolicy(AuthenticationPolicyErrorV1),
    ReceiptDigestMismatch,
    OidcIssuerMismatch,
    SourceRepositoryMismatch,
    SourceRepositoryOwnerMismatch,
    SignerWorkflowMismatch,
    SignerWorkflowRevisionMismatch,
    SourceRevisionMismatch,
    SourceRefMismatch,
    PredicateTypeMismatch,
    PredicateSchemaMismatch,
    TrustedRootProfileMismatch,
    TrustedRootDigestMismatch,
    TransparencyPolicyNotMet,
    FreshnessPolicyNotMet,
}

/// Serializable observability data. Deserializing this type never creates authority.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct AuthenticationEvidenceSummaryV1 {
    pub verifier_profile_id: String,
    pub authentication_policy_id: String,
    pub receipt_digest: QualificationReceiptDigestV1,
    pub source_repository: String,
    pub source_repository_owner: String,
    pub signer_workflow: String,
    pub signer_workflow_revision: GitObjectIdV1,
    pub source_revision: GitObjectIdV1,
    pub source_ref: Option<String>,
    pub oidc_issuer: String,
    pub predicate_type: String,
    pub predicate_schema: String,
    pub trusted_root_profile: String,
    pub trusted_root_digest: Sha256DigestV1,
    pub public_transparency_verified: bool,
    pub timestamp_verified: bool,
    pub current_at_verification: bool,
    pub age_seconds: Option<u64>,
    pub authentication_evidence_digest: Sha256DigestV1,
}

/// Opaque proof that one exact canonical qualification receipt passed one exact
/// authentication policy under a concrete verifier backend.
///
/// There is intentionally no `Deserialize`, `Serialize`, `Default`, or public constructor.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AuthenticatedQualificationReceiptV1 {
    receipt_digest: QualificationReceiptDigestV1,
    authentication_policy: ReceiptAuthenticationPolicyV1,
    verified_identity: VerifiedSignerIdentityV1,
    verified_predicate: VerifiedQualificationPredicateV1,
    verified_context: VerifiedAuthenticationContextV1,
    authentication_evidence_digest: Sha256DigestV1,
    authority: AuthenticatedReceiptAuthorityV1,
}

impl AuthenticatedQualificationReceiptV1 {
    pub fn receipt_digest(&self) -> QualificationReceiptDigestV1 {
        self.receipt_digest
    }

    pub fn authentication_policy(&self) -> &ReceiptAuthenticationPolicyV1 {
        &self.authentication_policy
    }

    pub fn authentication_policy_id(&self) -> &str {
        &self.authentication_policy.profile_id
    }

    pub fn verifier_profile_id(&self) -> &str {
        &self.authentication_policy.verifier_profile.profile_id
    }

    pub fn verified_identity(&self) -> &VerifiedSignerIdentityV1 {
        &self.verified_identity
    }

    pub fn verified_predicate(&self) -> &VerifiedQualificationPredicateV1 {
        &self.verified_predicate
    }

    pub fn verified_context(&self) -> &VerifiedAuthenticationContextV1 {
        &self.verified_context
    }

    pub fn authentication_evidence_digest(&self) -> Sha256DigestV1 {
        self.authentication_evidence_digest
    }

    pub const fn authority_scope(&self) -> AuthenticatedReceiptAuthorityV1 {
        self.authority
    }

    pub const fn grants_production_authority(&self) -> bool {
        false
    }

    pub const fn grants_application_authority(&self) -> bool {
        false
    }

    pub fn evidence_summary(&self) -> AuthenticationEvidenceSummaryV1 {
        AuthenticationEvidenceSummaryV1 {
            verifier_profile_id: self.authentication_policy.verifier_profile.profile_id.clone(),
            authentication_policy_id: self.authentication_policy.profile_id.clone(),
            receipt_digest: self.receipt_digest,
            source_repository: self.verified_identity.source_repository.clone(),
            source_repository_owner: self.verified_identity.source_repository_owner.clone(),
            signer_workflow: self.verified_identity.signer_workflow.clone(),
            signer_workflow_revision: self.verified_identity.signer_workflow_revision,
            source_revision: self.verified_identity.source_revision,
            source_ref: self.verified_identity.source_ref.clone(),
            oidc_issuer: self.verified_identity.oidc_issuer.clone(),
            predicate_type: self.verified_predicate.predicate_type.clone(),
            predicate_schema: self.verified_predicate.predicate_schema.clone(),
            trusted_root_profile: self.verified_context.trusted_root_profile.clone(),
            trusted_root_digest: self.verified_context.trusted_root_digest,
            public_transparency_verified: self
                .verified_context
                .transparency
                .public_transparency_verified,
            timestamp_verified: self.verified_context.transparency.timestamp_verified,
            current_at_verification: self.verified_context.freshness.current_at_verification,
            age_seconds: self.verified_context.freshness.age_seconds,
            authentication_evidence_digest: self.authentication_evidence_digest,
        }
    }

    /// Crate-internal minting boundary used only by concrete qualified backends.
    ///
    /// It re-checks all policy-bindable facts after cryptographic verification so a
    /// backend cannot accidentally mint a capability for a different policy/receipt.
    pub(crate) fn from_verified_parts(
        policy: &ReceiptAuthenticationPolicyV1,
        receipt_digest: QualificationReceiptDigestV1,
        verified_identity: VerifiedSignerIdentityV1,
        verified_predicate: VerifiedQualificationPredicateV1,
        verified_context: VerifiedAuthenticationContextV1,
        authentication_evidence_digest: Sha256DigestV1,
    ) -> Result<Self, AuthenticatedCapabilityConstructionErrorV1> {
        policy
            .validate()
            .map_err(AuthenticatedCapabilityConstructionErrorV1::InvalidPolicy)?;

        if verified_predicate.receipt_digest != receipt_digest {
            return Err(AuthenticatedCapabilityConstructionErrorV1::ReceiptDigestMismatch);
        }
        if verified_identity.oidc_issuer != policy.oidc_issuer {
            return Err(AuthenticatedCapabilityConstructionErrorV1::OidcIssuerMismatch);
        }
        if verified_identity.source_repository != policy.source_repository {
            return Err(AuthenticatedCapabilityConstructionErrorV1::SourceRepositoryMismatch);
        }
        if verified_identity.source_repository_owner != policy.source_repository_owner {
            return Err(
                AuthenticatedCapabilityConstructionErrorV1::SourceRepositoryOwnerMismatch,
            );
        }
        if verified_identity.signer_workflow != policy.signer_workflow {
            return Err(AuthenticatedCapabilityConstructionErrorV1::SignerWorkflowMismatch);
        }
        if !workflow_revision_allowed(
            &policy.signer_workflow_revision,
            verified_identity.signer_workflow_revision,
        ) {
            return Err(
                AuthenticatedCapabilityConstructionErrorV1::SignerWorkflowRevisionMismatch,
            );
        }
        if verified_identity.source_revision != policy.source_revision.exact_commit {
            return Err(AuthenticatedCapabilityConstructionErrorV1::SourceRevisionMismatch);
        }
        if let Some(expected_ref) = policy.source_revision.exact_git_ref.as_deref() {
            if verified_identity.source_ref.as_deref() != Some(expected_ref) {
                return Err(AuthenticatedCapabilityConstructionErrorV1::SourceRefMismatch);
            }
        }
        if verified_predicate.predicate_type != policy.expected_predicate_type {
            return Err(AuthenticatedCapabilityConstructionErrorV1::PredicateTypeMismatch);
        }
        if verified_predicate.predicate_schema != policy.expected_predicate_schema {
            return Err(AuthenticatedCapabilityConstructionErrorV1::PredicateSchemaMismatch);
        }
        if verified_context.trusted_root_profile != policy.trusted_root_profile {
            return Err(AuthenticatedCapabilityConstructionErrorV1::TrustedRootProfileMismatch);
        }
        if verified_context.trusted_root_digest != policy.trusted_root_digest {
            return Err(AuthenticatedCapabilityConstructionErrorV1::TrustedRootDigestMismatch);
        }
        if !transparency_policy_met(policy, verified_context.transparency) {
            return Err(AuthenticatedCapabilityConstructionErrorV1::TransparencyPolicyNotMet);
        }
        if !freshness_policy_met(policy, verified_context.freshness) {
            return Err(AuthenticatedCapabilityConstructionErrorV1::FreshnessPolicyNotMet);
        }

        Ok(Self {
            receipt_digest,
            authentication_policy: policy.clone(),
            verified_identity,
            verified_predicate,
            verified_context,
            authentication_evidence_digest,
            authority: AuthenticatedReceiptAuthorityV1::ReceiptAuthenticationOnly,
        })
    }
}

fn workflow_revision_allowed(policy: &WorkflowRevisionPolicyV1, actual: GitObjectIdV1) -> bool {
    match policy {
        WorkflowRevisionPolicyV1::Exact(expected) => *expected == actual,
        WorkflowRevisionPolicyV1::Allowed(allowed) => allowed.contains(&actual),
    }
}

fn transparency_policy_met(
    policy: &ReceiptAuthenticationPolicyV1,
    verified: VerifiedTransparencyV1,
) -> bool {
    match policy.transparency_policy {
        TransparencyPolicyV1::PublicTransparencyRequired => verified.public_transparency_verified,
        TransparencyPolicyV1::TimestampRequired => verified.timestamp_verified,
        TransparencyPolicyV1::PublicTransparencyAndTimestampRequired => {
            verified.public_transparency_verified && verified.timestamp_verified
        }
    }
}

fn freshness_policy_met(
    policy: &ReceiptAuthenticationPolicyV1,
    verified: VerifiedFreshnessV1,
) -> bool {
    match policy.freshness_policy {
        AuthenticationFreshnessPolicyV1::CurrentAtVerification => verified.current_at_verification,
        AuthenticationFreshnessPolicyV1::MaximumAgeSeconds(maximum) => {
            verified.current_at_verification
                && verified.age_seconds.is_some_and(|age| age <= maximum)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        QualificationReceiptCanonicalizationV1, SourceRevisionPolicyV1, VerifierProfileV1,
    };

    fn digest(byte: u8) -> Sha256DigestV1 {
        Sha256DigestV1::from_bytes([byte; 32])
    }

    fn policy() -> ReceiptAuthenticationPolicyV1 {
        ReceiptAuthenticationPolicyV1 {
            profile_id: "github-public-v1".into(),
            verifier_profile: VerifierProfileV1 {
                profile_id: "github-attestation-verifier-v1".into(),
                backend_family: "github-artifact-attestation".into(),
                backend_version: "unqualified-placeholder".into(),
                verification_profile: "public-sigstore-qualification-v1".into(),
            },
            expected_predicate_type: "https://mycelix.org/attestations/qualification/v1".into(),
            expected_predicate_schema: "mycelix-qualification-v1".into(),
            trusted_root_profile: "sigstore-public-good-v1".into(),
            trusted_root_digest: digest(9),
            oidc_issuer: "https://token.actions.githubusercontent.com".into(),
            source_repository: "Luminous-Dynamics/mycelix".into(),
            source_repository_owner: "Luminous-Dynamics".into(),
            signer_workflow: ".github/workflows/proofs.yml".into(),
            signer_workflow_revision: WorkflowRevisionPolicyV1::Exact(GitObjectIdV1::sha1([
                0xbb; 20
            ])),
            source_revision: SourceRevisionPolicyV1 {
                exact_commit: GitObjectIdV1::sha1([0xaa; 20]),
                exact_git_ref: Some("refs/heads/main".into()),
            },
            transparency_policy: TransparencyPolicyV1::PublicTransparencyRequired,
            freshness_policy: AuthenticationFreshnessPolicyV1::CurrentAtVerification,
        }
    }

    fn identity() -> VerifiedSignerIdentityV1 {
        VerifiedSignerIdentityV1 {
            oidc_issuer: "https://token.actions.githubusercontent.com".into(),
            source_repository: "Luminous-Dynamics/mycelix".into(),
            source_repository_owner: "Luminous-Dynamics".into(),
            signer_workflow: ".github/workflows/proofs.yml".into(),
            signer_workflow_revision: GitObjectIdV1::sha1([0xbb; 20]),
            source_revision: GitObjectIdV1::sha1([0xaa; 20]),
            source_ref: Some("refs/heads/main".into()),
        }
    }

    fn receipt_digest(byte: u8) -> QualificationReceiptDigestV1 {
        QualificationReceiptDigestV1 {
            canonicalization: QualificationReceiptCanonicalizationV1::BinaryV1,
            sha256: digest(byte),
        }
    }

    fn predicate(receipt_digest: QualificationReceiptDigestV1) -> VerifiedQualificationPredicateV1 {
        VerifiedQualificationPredicateV1 {
            predicate_type: "https://mycelix.org/attestations/qualification/v1".into(),
            predicate_schema: "mycelix-qualification-v1".into(),
            receipt_digest,
        }
    }

    fn context() -> VerifiedAuthenticationContextV1 {
        VerifiedAuthenticationContextV1 {
            trusted_root_profile: "sigstore-public-good-v1".into(),
            trusted_root_digest: digest(9),
            transparency: VerifiedTransparencyV1 {
                public_transparency_verified: true,
                timestamp_verified: false,
            },
            freshness: VerifiedFreshnessV1 {
                current_at_verification: true,
                age_seconds: Some(10),
            },
        }
    }

    #[test]
    fn matching_verified_facts_can_mint_only_receipt_authentication_capability() {
        let receipt_digest = receipt_digest(1);
        let capability = AuthenticatedQualificationReceiptV1::from_verified_parts(
            &policy(),
            receipt_digest,
            identity(),
            predicate(receipt_digest),
            context(),
            digest(7),
        )
        .unwrap();

        assert_eq!(
            capability.authority_scope(),
            AuthenticatedReceiptAuthorityV1::ReceiptAuthenticationOnly
        );
        assert!(!capability.grants_production_authority());
        assert!(!capability.grants_application_authority());
        assert_eq!(capability.receipt_digest(), receipt_digest);
        assert_eq!(capability.authentication_policy(), &policy());
    }

    #[test]
    fn receipt_and_identity_substitutions_fail() {
        let exact_receipt = receipt_digest(1);
        let wrong_receipt = receipt_digest(2);
        assert_eq!(
            AuthenticatedQualificationReceiptV1::from_verified_parts(
                &policy(),
                exact_receipt,
                identity(),
                predicate(wrong_receipt),
                context(),
                digest(7),
            ),
            Err(AuthenticatedCapabilityConstructionErrorV1::ReceiptDigestMismatch)
        );

        let mut wrong = identity();
        wrong.source_repository = "attacker/repo".into();
        assert_eq!(
            AuthenticatedQualificationReceiptV1::from_verified_parts(
                &policy(),
                exact_receipt,
                wrong,
                predicate(exact_receipt),
                context(),
                digest(7),
            ),
            Err(AuthenticatedCapabilityConstructionErrorV1::SourceRepositoryMismatch)
        );
    }

    #[test]
    fn trusted_root_transparency_and_freshness_are_policy_bound() {
        let exact_receipt = receipt_digest(1);

        let mut wrong_root = context();
        wrong_root.trusted_root_digest = digest(8);
        assert_eq!(
            AuthenticatedQualificationReceiptV1::from_verified_parts(
                &policy(),
                exact_receipt,
                identity(),
                predicate(exact_receipt),
                wrong_root,
                digest(7),
            ),
            Err(AuthenticatedCapabilityConstructionErrorV1::TrustedRootDigestMismatch)
        );

        let mut no_log = context();
        no_log.transparency.public_transparency_verified = false;
        assert_eq!(
            AuthenticatedQualificationReceiptV1::from_verified_parts(
                &policy(),
                exact_receipt,
                identity(),
                predicate(exact_receipt),
                no_log,
                digest(7),
            ),
            Err(AuthenticatedCapabilityConstructionErrorV1::TransparencyPolicyNotMet)
        );

        let mut stale = context();
        stale.freshness.current_at_verification = false;
        assert_eq!(
            AuthenticatedQualificationReceiptV1::from_verified_parts(
                &policy(),
                exact_receipt,
                identity(),
                predicate(exact_receipt),
                stale,
                digest(7),
            ),
            Err(AuthenticatedCapabilityConstructionErrorV1::FreshnessPolicyNotMet)
        );
    }
}
