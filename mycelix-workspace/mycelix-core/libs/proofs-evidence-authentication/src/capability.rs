use serde::{Deserialize, Serialize};

use crate::{
    AuthenticationFreshnessPolicyV1, AuthenticationPolicyErrorV1, GitObjectIdV1,
    QualificationReceiptDigestV1, QualificationReceiptV1, QualificationResultV1,
    ReceiptAuthenticationPolicyV1, ReceiptCanonicalizationErrorV1, Sha256DigestV1,
    TransparencyPolicyV1, WorkflowRevisionPolicyV1,
};

/// Facts established by a concrete cryptographic verifier before policy binding.
///
/// Fields are crate-private so external callers cannot manufacture values whose type
/// name says `Verified`. Public access remains read-only through accessors.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct VerifiedSignerIdentityV1 {
    pub(crate) oidc_issuer: String,
    pub(crate) source_repository: String,
    pub(crate) source_repository_owner: String,
    pub(crate) signer_workflow: String,
    pub(crate) signer_workflow_revision: GitObjectIdV1,
    pub(crate) source_revision: GitObjectIdV1,
    pub(crate) source_ref: Option<String>,
}

impl VerifiedSignerIdentityV1 {
    pub fn oidc_issuer(&self) -> &str {
        &self.oidc_issuer
    }

    pub fn source_repository(&self) -> &str {
        &self.source_repository
    }

    pub fn source_repository_owner(&self) -> &str {
        &self.source_repository_owner
    }

    pub fn signer_workflow(&self) -> &str {
        &self.signer_workflow
    }

    pub fn signer_workflow_revision(&self) -> GitObjectIdV1 {
        self.signer_workflow_revision
    }

    pub fn source_revision(&self) -> GitObjectIdV1 {
        self.source_revision
    }

    pub fn source_ref(&self) -> Option<&str> {
        self.source_ref.as_deref()
    }
}

/// Cryptographically verified in-toto subject + Mycelix predicate facts.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct VerifiedQualificationPredicateV1 {
    pub(crate) predicate_type: String,
    pub(crate) predicate_schema: String,
    pub(crate) attestation_subject_name: String,
    /// SHA-256 digest carried by the outer in-toto subject.
    pub(crate) attestation_subject_sha256: Sha256DigestV1,
    /// Canonical receipt digest carried inside the Mycelix predicate.
    pub(crate) receipt_digest: QualificationReceiptDigestV1,
    pub(crate) qualification_profile: String,
    pub(crate) subject: GitObjectIdV1,
    pub(crate) coherence_result_digest: Sha256DigestV1,
    pub(crate) result: QualificationResultV1,
}

impl VerifiedQualificationPredicateV1 {
    pub fn predicate_type(&self) -> &str {
        &self.predicate_type
    }

    pub fn predicate_schema(&self) -> &str {
        &self.predicate_schema
    }

    pub fn attestation_subject_name(&self) -> &str {
        &self.attestation_subject_name
    }

    pub fn attestation_subject_sha256(&self) -> Sha256DigestV1 {
        self.attestation_subject_sha256
    }

    pub fn receipt_digest(&self) -> QualificationReceiptDigestV1 {
        self.receipt_digest
    }

    pub fn qualification_profile(&self) -> &str {
        &self.qualification_profile
    }

    pub fn subject(&self) -> GitObjectIdV1 {
        self.subject
    }

    pub fn coherence_result_digest(&self) -> Sha256DigestV1 {
        self.coherence_result_digest
    }

    pub fn result(&self) -> QualificationResultV1 {
        self.result
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct VerifiedTransparencyV1 {
    pub(crate) public_transparency_verified: bool,
    pub(crate) timestamp_verified: bool,
}

impl VerifiedTransparencyV1 {
    pub const fn public_transparency_verified(&self) -> bool {
        self.public_transparency_verified
    }

    pub const fn timestamp_verified(&self) -> bool {
        self.timestamp_verified
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct VerifiedFreshnessV1 {
    pub(crate) current_at_verification: bool,
    pub(crate) age_seconds: Option<u64>,
}

impl VerifiedFreshnessV1 {
    pub const fn current_at_verification(&self) -> bool {
        self.current_at_verification
    }

    pub const fn age_seconds(&self) -> Option<u64> {
        self.age_seconds
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct VerifiedAuthenticationContextV1 {
    pub(crate) trusted_root_profile: String,
    pub(crate) trusted_root_digest: Sha256DigestV1,
    pub(crate) transparency: VerifiedTransparencyV1,
    pub(crate) freshness: VerifiedFreshnessV1,
}

impl VerifiedAuthenticationContextV1 {
    pub fn trusted_root_profile(&self) -> &str {
        &self.trusted_root_profile
    }

    pub fn trusted_root_digest(&self) -> Sha256DigestV1 {
        self.trusted_root_digest
    }

    pub const fn transparency(&self) -> VerifiedTransparencyV1 {
        self.transparency
    }

    pub const fn freshness(&self) -> VerifiedFreshnessV1 {
        self.freshness
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum AuthenticatedReceiptAuthorityV1 {
    ReceiptAuthenticationOnly,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum AuthenticatedCapabilityConstructionErrorV1 {
    InvalidPolicy(AuthenticationPolicyErrorV1),
    InvalidReceipt(ReceiptCanonicalizationErrorV1),
    AttestationSubjectNameMismatch,
    AttestationSubjectDigestMismatch,
    ReceiptDigestMismatch,
    QualificationProfileMismatch,
    ReceiptSubjectMismatch,
    CoherenceResultDigestMismatch,
    QualificationResultMismatch,
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
    pub authority_scope: AuthenticatedReceiptAuthorityV1,
    pub verifier_profile_id: String,
    pub authentication_policy_id: String,
    pub attestation_subject_name: String,
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

impl AuthenticationEvidenceSummaryV1 {
    pub const fn grants_production_authority(&self) -> bool {
        false
    }

    pub const fn grants_application_authority(&self) -> bool {
        false
    }
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
            authority_scope: self.authority,
            verifier_profile_id: self.authentication_policy.verifier_profile.profile_id.clone(),
            authentication_policy_id: self.authentication_policy.profile_id.clone(),
            attestation_subject_name: self.verified_predicate.attestation_subject_name.clone(),
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
    /// The canonical receipt digest is recomputed here. The outer in-toto subject and
    /// every redundant theorem-bearing predicate field must agree with the canonical
    /// receipt before policy facts are checked. Thus a backend cannot accidentally mint
    /// a capability for a detached predicate, detached digest, or different policy.
    #[cfg_attr(not(test), allow(dead_code))]
    pub(crate) fn from_verified_parts(
        policy: &ReceiptAuthenticationPolicyV1,
        receipt: &QualificationReceiptV1,
        verified_identity: VerifiedSignerIdentityV1,
        verified_predicate: VerifiedQualificationPredicateV1,
        verified_context: VerifiedAuthenticationContextV1,
        authentication_evidence_digest: Sha256DigestV1,
    ) -> Result<Self, AuthenticatedCapabilityConstructionErrorV1> {
        policy
            .validate()
            .map_err(AuthenticatedCapabilityConstructionErrorV1::InvalidPolicy)?;

        let receipt_digest = receipt
            .digest()
            .map_err(AuthenticatedCapabilityConstructionErrorV1::InvalidReceipt)?;

        if verified_predicate.attestation_subject_name != policy.expected_attestation_subject_name {
            return Err(AuthenticatedCapabilityConstructionErrorV1::AttestationSubjectNameMismatch);
        }
        if verified_predicate.attestation_subject_sha256 != receipt_digest.sha256 {
            return Err(
                AuthenticatedCapabilityConstructionErrorV1::AttestationSubjectDigestMismatch,
            );
        }
        if verified_predicate.receipt_digest != receipt_digest {
            return Err(AuthenticatedCapabilityConstructionErrorV1::ReceiptDigestMismatch);
        }
        if verified_predicate.qualification_profile != receipt.qualification_profile {
            return Err(AuthenticatedCapabilityConstructionErrorV1::QualificationProfileMismatch);
        }
        if verified_predicate.subject != receipt.subject {
            return Err(AuthenticatedCapabilityConstructionErrorV1::ReceiptSubjectMismatch);
        }
        if verified_predicate.coherence_result_digest != receipt.coherence_result_digest {
            return Err(
                AuthenticatedCapabilityConstructionErrorV1::CoherenceResultDigestMismatch,
            );
        }
        if verified_predicate.result != receipt.result {
            return Err(AuthenticatedCapabilityConstructionErrorV1::QualificationResultMismatch);
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

    fn receipt() -> QualificationReceiptV1 {
        QualificationReceiptV1 {
            receipt_version: 1,
            qualification_profile: "myc-zkp-range-001aq".into(),
            statement_profile: "range-membership-v1".into(),
            theorem_profile: "range-membership-v1-air".into(),
            subject: GitObjectIdV1::sha1([0xaa; 20]),
            dependency_graph_digest: digest(1),
            measured_security_receipt_digest: digest(2),
            coherence_policy_id: "coherence-v1".into(),
            coherence_result_digest: digest(3),
            qualification_corpus_digest: digest(4),
            execution_capsule_digest: digest(5),
            result: QualificationResultV1::Pass,
            nonclaims: vec!["application authority not granted".into()],
        }
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
            expected_attestation_subject_name: "mycelix-qualification-receipt".into(),
            expected_predicate_type: "https://mycelix.org/attestations/qualification/v1".into(),
            expected_predicate_schema: "mycelix-qualification-attestation-predicate-v1".into(),
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
                exact_commit: GitObjectIdV1::sha1([0xcc; 20]),
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
            source_revision: GitObjectIdV1::sha1([0xcc; 20]),
            source_ref: Some("refs/heads/main".into()),
        }
    }

    fn predicate(receipt: &QualificationReceiptV1) -> VerifiedQualificationPredicateV1 {
        let receipt_digest = receipt.digest().unwrap();
        VerifiedQualificationPredicateV1 {
            predicate_type: "https://mycelix.org/attestations/qualification/v1".into(),
            predicate_schema: "mycelix-qualification-attestation-predicate-v1".into(),
            attestation_subject_name: "mycelix-qualification-receipt".into(),
            attestation_subject_sha256: receipt_digest.sha256,
            receipt_digest,
            qualification_profile: receipt.qualification_profile.clone(),
            subject: receipt.subject,
            coherence_result_digest: receipt.coherence_result_digest,
            result: receipt.result,
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
        let receipt = receipt();
        let capability = AuthenticatedQualificationReceiptV1::from_verified_parts(
            &policy(),
            &receipt,
            identity(),
            predicate(&receipt),
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
        assert_eq!(capability.receipt_digest(), receipt.digest().unwrap());
        assert_eq!(capability.authentication_policy(), &policy());

        let summary = capability.evidence_summary();
        assert_eq!(summary.authority_scope, AuthenticatedReceiptAuthorityV1::ReceiptAuthenticationOnly);
        assert!(!summary.grants_production_authority());
        assert!(!summary.grants_application_authority());
    }

    #[test]
    fn outer_subject_name_digest_and_predicate_digest_substitution_fail() {
        let receipt = receipt();

        let mut wrong = predicate(&receipt);
        wrong.attestation_subject_name = "other-subject".into();
        assert_eq!(
            AuthenticatedQualificationReceiptV1::from_verified_parts(
                &policy(),
                &receipt,
                identity(),
                wrong,
                context(),
                digest(7),
            ),
            Err(AuthenticatedCapabilityConstructionErrorV1::AttestationSubjectNameMismatch)
        );

        let mut wrong = predicate(&receipt);
        wrong.attestation_subject_sha256 = digest(8);
        assert_eq!(
            AuthenticatedQualificationReceiptV1::from_verified_parts(
                &policy(),
                &receipt,
                identity(),
                wrong,
                context(),
                digest(7),
            ),
            Err(
                AuthenticatedCapabilityConstructionErrorV1::AttestationSubjectDigestMismatch
            )
        );

        let mut wrong = predicate(&receipt);
        wrong.receipt_digest = QualificationReceiptDigestV1 {
            canonicalization: QualificationReceiptCanonicalizationV1::BinaryV1,
            sha256: digest(8),
        };
        assert_eq!(
            AuthenticatedQualificationReceiptV1::from_verified_parts(
                &policy(),
                &receipt,
                identity(),
                wrong,
                context(),
                digest(7),
            ),
            Err(AuthenticatedCapabilityConstructionErrorV1::ReceiptDigestMismatch)
        );
    }

    #[test]
    fn redundant_predicate_metadata_is_bound_to_canonical_receipt() {
        let receipt = receipt();

        let mut wrong = predicate(&receipt);
        wrong.qualification_profile = "other-profile".into();
        assert_eq!(
            AuthenticatedQualificationReceiptV1::from_verified_parts(
                &policy(),
                &receipt,
                identity(),
                wrong,
                context(),
                digest(7),
            ),
            Err(AuthenticatedCapabilityConstructionErrorV1::QualificationProfileMismatch)
        );

        let mut wrong = predicate(&receipt);
        wrong.subject = GitObjectIdV1::sha1([0xdd; 20]);
        assert_eq!(
            AuthenticatedQualificationReceiptV1::from_verified_parts(
                &policy(),
                &receipt,
                identity(),
                wrong,
                context(),
                digest(7),
            ),
            Err(AuthenticatedCapabilityConstructionErrorV1::ReceiptSubjectMismatch)
        );

        let mut wrong = predicate(&receipt);
        wrong.coherence_result_digest = digest(8);
        assert_eq!(
            AuthenticatedQualificationReceiptV1::from_verified_parts(
                &policy(),
                &receipt,
                identity(),
                wrong,
                context(),
                digest(7),
            ),
            Err(AuthenticatedCapabilityConstructionErrorV1::CoherenceResultDigestMismatch)
        );

        let mut wrong = predicate(&receipt);
        wrong.result = QualificationResultV1::RecordedOnly;
        assert_eq!(
            AuthenticatedQualificationReceiptV1::from_verified_parts(
                &policy(),
                &receipt,
                identity(),
                wrong,
                context(),
                digest(7),
            ),
            Err(AuthenticatedCapabilityConstructionErrorV1::QualificationResultMismatch)
        );
    }

    #[test]
    fn identity_substitution_fails() {
        let receipt = receipt();
        let mut wrong = identity();
        wrong.source_repository = "attacker/repo".into();
        assert_eq!(
            AuthenticatedQualificationReceiptV1::from_verified_parts(
                &policy(),
                &receipt,
                wrong,
                predicate(&receipt),
                context(),
                digest(7),
            ),
            Err(AuthenticatedCapabilityConstructionErrorV1::SourceRepositoryMismatch)
        );
    }

    #[test]
    fn trusted_root_transparency_and_freshness_are_policy_bound() {
        let receipt = receipt();

        let mut wrong_root = context();
        wrong_root.trusted_root_digest = digest(8);
        assert_eq!(
            AuthenticatedQualificationReceiptV1::from_verified_parts(
                &policy(),
                &receipt,
                identity(),
                predicate(&receipt),
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
                &receipt,
                identity(),
                predicate(&receipt),
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
                &receipt,
                identity(),
                predicate(&receipt),
                stale,
                digest(7),
            ),
            Err(AuthenticatedCapabilityConstructionErrorV1::FreshnessPolicyNotMet)
        );
    }
}
