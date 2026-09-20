// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Authentication and project-policy qualification for lineage-aware review
//! revisions.
//!
//! Review-revision lineage metadata is security-significant. It is therefore
//! not enough to authenticate a decision and attach `sequence` / `previous`
//! afterward. This crate requires the exact [`ReviewRevisionId`] itself to be
//! the Forge principal-authentication action subject, then carries that exact
//! revision through provider provenance, structural eligibility, and project
//! provider-trust policy.
//!
//! ```text
//! ReviewRevisionV1
//!     ↓ exact ReviewRevisionId
//! PrincipalAuthenticationRequest.action_subject
//!     ↓
//! FORGE-005A evidence binding
//!     ↓
//! structural ReviewSource eligibility
//!     ↓
//! Xenia provider verification
//!     ↓
//! exact project-policy verifier trust
//!     ↓
//! ProjectPolicyTrustedReviewRevisionV1
//! ```
//!
//! None of these steps proves the revision is the globally current review head.

#![forbid(unsafe_code)]
#![warn(missing_docs)]

use mycelix_forge_authentication::{AuthenticationError, EvidenceBoundPrincipalAuthentication};
use mycelix_forge_authority::{AuthorityEpoch, AuthorityError, Capability};
use mycelix_forge_core::{Digest, ProjectIdentity, ProtocolVersion};
use mycelix_forge_project_policy::{
    AuthenticationProviderTrustPolicyV1, ProjectPolicyError, ProjectPolicyStateV1,
};
use mycelix_forge_proposal::{ChangeProposal, ChangeProposalId, ProposalError};
use mycelix_forge_review_state::{ReviewRevisionId, ReviewRevisionV1, ReviewStateError};
use mycelix_forge_xenia::xenia_provider_namespace;
use mycelix_forge_xenia_provider::ProviderVerifiedXeniaAuthenticationV1;
use serde::Serialize;
use thiserror::Error;

const REVISION_AUTH_DOMAIN_V1: &[u8] = b"mycelix-forge/evidence-bound-review-revision/v1\0";
const REVISION_ELIGIBILITY_DOMAIN_V1: &[u8] =
    b"mycelix-forge/eligible-review-revision/v1\0";
const REVISION_PROVIDER_DOMAIN_V1: &[u8] =
    b"mycelix-forge/provider-verified-review-revision/v1\0";
const REVISION_POLICY_DOMAIN_V1: &[u8] =
    b"mycelix-forge/project-policy-trusted-review-revision/v1\0";

/// Positive result proving FORGE-005A evidence names one exact review revision.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct EvidenceBoundReviewRevisionV1 {
    project: ProjectIdentity,
    authority_epoch: Digest,
    proposal: ChangeProposalId,
    revision: ReviewRevisionV1,
    authentication_evidence: Digest,
    evidence_commitment: Digest,
}

impl EvidenceBoundReviewRevisionV1 {
    /// Project containing the exact proposal/revision.
    pub fn project(&self) -> &ProjectIdentity {
        &self.project
    }

    /// Exact authority epoch named by the authentication request.
    pub fn authority_epoch(&self) -> &Digest {
        &self.authority_epoch
    }

    /// Exact immutable proposal being reviewed.
    pub fn proposal(&self) -> &ChangeProposalId {
        &self.proposal
    }

    /// Exact review revision whose id was authenticated.
    pub fn revision(&self) -> &ReviewRevisionV1 {
        &self.revision
    }

    /// Exact FORGE-005A evidence commitment bound to this revision.
    pub fn authentication_evidence(&self) -> &Digest {
        &self.authentication_evidence
    }

    /// Aggregate review-revision authentication evidence commitment.
    pub fn evidence_commitment(&self) -> &Digest {
        &self.evidence_commitment
    }
}

/// Bind one exact review revision to an exact FORGE-005A principal
/// authentication object.
pub fn bind_review_revision_authentication_v1(
    proposal: &ChangeProposal,
    revision: ReviewRevisionV1,
    authentication: &EvidenceBoundPrincipalAuthentication,
    authority_epoch: &AuthorityEpoch,
) -> Result<EvidenceBoundReviewRevisionV1, ReviewRevisionAuthError> {
    let expected_proposal = proposal.proposal_id(revision.proposal().commitment().algorithm())?;
    if revision.proposal() != &expected_proposal {
        return Err(ReviewRevisionAuthError::ProposalMismatch);
    }
    if authority_epoch.project() != proposal.project() {
        return Err(ReviewRevisionAuthError::ProjectMismatch);
    }
    let expected_epoch = authority_epoch.digest(proposal.authority_epoch().algorithm())?;
    if proposal.authority_epoch() != &expected_epoch {
        return Err(ReviewRevisionAuthError::AuthorityEpochMismatch);
    }

    let request = authentication.request();
    if request.project() != proposal.project() {
        return Err(ReviewRevisionAuthError::ProjectMismatch);
    }
    if request.authority_epoch() != &expected_epoch {
        return Err(ReviewRevisionAuthError::AuthenticationAuthorityMismatch);
    }
    if request.principal() != revision.reviewer()
        || authentication.binding().forge_principal() != revision.reviewer()
    {
        return Err(ReviewRevisionAuthError::ReviewerMismatch);
    }
    if request.capability() != Capability::ReviewSource {
        return Err(ReviewRevisionAuthError::WrongAuthenticationCapability);
    }

    let expected_revision = revision.revision_id(request.action_subject().algorithm())?;
    if request.action_subject() != expected_revision.commitment() {
        return Err(ReviewRevisionAuthError::AuthenticationSubjectMismatch);
    }

    let algorithm = authentication.evidence_commitment().algorithm();
    let mut out = Vec::new();
    out.extend_from_slice(REVISION_AUTH_DOMAIN_V1);
    out.extend_from_slice(&ProtocolVersion::CURRENT.get().to_be_bytes());
    push_project_identity(&mut out, proposal.project())?;
    push_digest(&mut out, expected_proposal.commitment())?;
    push_digest(&mut out, &expected_epoch)?;
    push_digest(&mut out, expected_revision.commitment())?;
    push_digest(&mut out, authentication.evidence_commitment())?;
    let evidence_commitment = Digest::of_bytes(algorithm, &out);

    Ok(EvidenceBoundReviewRevisionV1 {
        project: proposal.project().clone(),
        authority_epoch: expected_epoch,
        proposal: expected_proposal,
        revision,
        authentication_evidence: authentication.evidence_commitment().clone(),
        evidence_commitment,
    })
}

/// Positive result proving the exact revision reviewer is structurally eligible
/// for `ReviewSource` at one caller-supplied observation time.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct StructurallyEligibleReviewRevisionV1 {
    revision: EvidenceBoundReviewRevisionV1,
    observed_at_unix_ms: u64,
    evidence_commitment: Digest,
}

impl StructurallyEligibleReviewRevisionV1 {
    /// Exact authenticated revision whose reviewer passed structural eligibility.
    pub fn revision(&self) -> &EvidenceBoundReviewRevisionV1 {
        &self.revision
    }

    /// Caller-supplied observation time used for authority evaluation.
    pub const fn observed_at_unix_ms(&self) -> u64 {
        self.observed_at_unix_ms
    }

    /// Aggregate structural-eligibility evidence commitment.
    pub fn evidence_commitment(&self) -> &Digest {
        &self.evidence_commitment
    }
}

/// Evaluate structural `ReviewSource` eligibility for one authenticated revision.
pub fn qualify_review_revision_eligibility_v1(
    revision: EvidenceBoundReviewRevisionV1,
    authority_epoch: &AuthorityEpoch,
    observed_at_unix_ms: u64,
) -> Result<StructurallyEligibleReviewRevisionV1, ReviewRevisionAuthError> {
    if authority_epoch.project() != revision.project() {
        return Err(ReviewRevisionAuthError::ProjectMismatch);
    }
    let expected_epoch = authority_epoch.digest(revision.authority_epoch().algorithm())?;
    if revision.authority_epoch() != &expected_epoch {
        return Err(ReviewRevisionAuthError::AuthorityEpochMismatch);
    }
    if !authority_epoch.is_principal_eligible(
        revision.revision().reviewer(),
        Capability::ReviewSource,
        observed_at_unix_ms,
    ) {
        return Err(ReviewRevisionAuthError::ReviewerNotEligible);
    }

    let algorithm = revision.evidence_commitment().algorithm();
    let mut out = Vec::new();
    out.extend_from_slice(REVISION_ELIGIBILITY_DOMAIN_V1);
    out.extend_from_slice(&ProtocolVersion::CURRENT.get().to_be_bytes());
    push_digest(&mut out, revision.evidence_commitment())?;
    push_digest(&mut out, &expected_epoch)?;
    out.extend_from_slice(&observed_at_unix_ms.to_be_bytes());
    let evidence_commitment = Digest::of_bytes(algorithm, &out);

    Ok(StructurallyEligibleReviewRevisionV1 {
        revision,
        observed_at_unix_ms,
        evidence_commitment,
    })
}

/// Positive result proving that the exact authentication observation used by a
/// structurally eligible revision was cryptographically attributed to one
/// Xenia verifier identity.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct ProviderVerifiedReviewRevisionV1 {
    revision: StructurallyEligibleReviewRevisionV1,
    authentication_evidence: Digest,
    provider_receipt: Digest,
    verifier_identity: Digest,
    evidence_commitment: Digest,
}

impl ProviderVerifiedReviewRevisionV1 {
    /// Structurally eligible exact revision whose provider provenance passed.
    pub fn revision(&self) -> &StructurallyEligibleReviewRevisionV1 {
        &self.revision
    }

    /// Exact 005A authentication evidence joined to the revision.
    pub fn authentication_evidence(&self) -> &Digest {
        &self.authentication_evidence
    }

    /// Exact Xenia receipt whose provider provenance passed.
    pub fn provider_receipt(&self) -> &Digest {
        &self.provider_receipt
    }

    /// Exact Xenia verifier identity that authenticated the receipt.
    pub fn verifier_identity(&self) -> &Digest {
        &self.verifier_identity
    }

    /// Aggregate provider-verified revision evidence commitment.
    pub fn evidence_commitment(&self) -> &Digest {
        &self.evidence_commitment
    }
}

/// Join provider-verified Xenia authentication to the exact authenticated
/// review revision that consumed its FORGE-005A observation.
pub fn bind_xenia_provider_verified_review_revision_v1(
    revision: StructurallyEligibleReviewRevisionV1,
    authentication: &EvidenceBoundPrincipalAuthentication,
    provider_verified: &ProviderVerifiedXeniaAuthenticationV1,
) -> Result<ProviderVerifiedReviewRevisionV1, ReviewRevisionAuthError> {
    if revision.revision().authentication_evidence() != authentication.evidence_commitment() {
        return Err(ReviewRevisionAuthError::AuthenticationEvidenceMismatch);
    }
    if provider_verified.observation() != authentication.observation() {
        return Err(ReviewRevisionAuthError::ProviderObservationMismatch);
    }

    let request = authentication.request();
    let exact_revision = revision.revision().revision();
    let expected_revision = exact_revision.revision_id(request.action_subject().algorithm())?;
    if request.action_subject() != expected_revision.commitment() {
        return Err(ReviewRevisionAuthError::AuthenticationSubjectMismatch);
    }

    let algorithm = revision.evidence_commitment().algorithm();
    let observation = provider_verified.observation().digest(algorithm)?;
    let mut out = Vec::new();
    out.extend_from_slice(REVISION_PROVIDER_DOMAIN_V1);
    out.extend_from_slice(&ProtocolVersion::CURRENT.get().to_be_bytes());
    push_digest(&mut out, revision.evidence_commitment())?;
    push_digest(&mut out, authentication.evidence_commitment())?;
    push_digest(&mut out, &observation)?;
    push_digest(&mut out, provider_verified.receipt_digest())?;
    push_digest(&mut out, provider_verified.verifier_identity_commitment())?;
    let evidence_commitment = Digest::of_bytes(algorithm, &out);

    Ok(ProviderVerifiedReviewRevisionV1 {
        revision,
        authentication_evidence: authentication.evidence_commitment().clone(),
        provider_receipt: provider_verified.receipt_digest().clone(),
        verifier_identity: provider_verified.verifier_identity_commitment().clone(),
        evidence_commitment,
    })
}

/// Positive result proving that the exact Xenia verifier used by one exact
/// authenticated review revision is trusted by the exact project policy
/// committed into the proposal.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct ProjectPolicyTrustedReviewRevisionV1 {
    revision: ProviderVerifiedReviewRevisionV1,
    project_policy: Digest,
    provider_trust_policy: Digest,
    provider_namespace: Digest,
    verifier_identity: Digest,
    evidence_commitment: Digest,
}

impl ProjectPolicyTrustedReviewRevisionV1 {
    /// Provider-verified exact review revision whose verifier is project-trusted.
    pub fn revision(&self) -> &ProviderVerifiedReviewRevisionV1 {
        &self.revision
    }

    /// Exact proposal project-policy-state commitment.
    pub fn project_policy(&self) -> &Digest {
        &self.project_policy
    }

    /// Exact provider-trust-policy commitment bound by the project policy.
    pub fn provider_trust_policy(&self) -> &Digest {
        &self.provider_trust_policy
    }

    /// Exact provider namespace under which the verifier identity is trusted.
    pub fn provider_namespace(&self) -> &Digest {
        &self.provider_namespace
    }

    /// Exact project-trusted verifier identity.
    pub fn verifier_identity(&self) -> &Digest {
        &self.verifier_identity
    }

    /// Aggregate project-policy-trusted revision evidence commitment.
    pub fn evidence_commitment(&self) -> &Digest {
        &self.evidence_commitment
    }
}

/// Bind one provider-verified review revision to the exact verifier-trust policy
/// committed by its proposal.
pub fn bind_project_policy_trusted_review_revision_v1(
    proposal: &ChangeProposal,
    project_policy: &ProjectPolicyStateV1,
    provider_trust: &AuthenticationProviderTrustPolicyV1,
    revision: ProviderVerifiedReviewRevisionV1,
) -> Result<ProjectPolicyTrustedReviewRevisionV1, ReviewRevisionAuthError> {
    if project_policy.project() != proposal.project() || provider_trust.project() != proposal.project() {
        return Err(ReviewRevisionAuthError::ProjectMismatch);
    }

    let evidence_bound = revision.revision().revision();
    if evidence_bound.project() != proposal.project() {
        return Err(ReviewRevisionAuthError::ProjectMismatch);
    }
    let exact_revision = evidence_bound.revision();
    let expected_proposal = proposal.proposal_id(exact_revision.proposal().commitment().algorithm())?;
    if exact_revision.proposal() != &expected_proposal {
        return Err(ReviewRevisionAuthError::ProposalMismatch);
    }

    let expected_project_policy = project_policy.digest(proposal.project_policy().algorithm())?;
    if proposal.project_policy() != &expected_project_policy {
        return Err(ReviewRevisionAuthError::ProjectPolicyMismatch);
    }
    if !project_policy.binds_provider_trust(provider_trust)? {
        return Err(ReviewRevisionAuthError::ProviderTrustPolicyMismatch);
    }

    let provider_namespace = xenia_provider_namespace();
    if !provider_trust.trusts(&provider_namespace, revision.verifier_identity()) {
        return Err(ReviewRevisionAuthError::VerifierNotTrusted);
    }

    let trust_policy_digest =
        provider_trust.digest(project_policy.authentication_provider_trust().algorithm())?;
    let algorithm = revision.evidence_commitment().algorithm();
    let mut out = Vec::new();
    out.extend_from_slice(REVISION_POLICY_DOMAIN_V1);
    out.extend_from_slice(&ProtocolVersion::CURRENT.get().to_be_bytes());
    push_digest(&mut out, revision.evidence_commitment())?;
    push_digest(&mut out, expected_proposal.commitment())?;
    push_digest(&mut out, &expected_project_policy)?;
    push_digest(&mut out, &trust_policy_digest)?;
    push_digest(&mut out, &provider_namespace)?;
    push_digest(&mut out, revision.verifier_identity())?;
    let evidence_commitment = Digest::of_bytes(algorithm, &out);

    Ok(ProjectPolicyTrustedReviewRevisionV1 {
        verifier_identity: revision.verifier_identity().clone(),
        revision,
        project_policy: expected_project_policy,
        provider_trust_policy: trust_policy_digest,
        provider_namespace,
        evidence_commitment,
    })
}

fn push_project_identity(
    out: &mut Vec<u8>,
    project: &ProjectIdentity,
) -> Result<(), ReviewRevisionAuthError> {
    out.extend_from_slice(&project.version().get().to_be_bytes());
    push_digest(out, project.digest())
}

fn push_digest(out: &mut Vec<u8>, digest: &Digest) -> Result<(), ReviewRevisionAuthError> {
    push_bytes(out, "digest_algorithm", digest.algorithm().id().as_bytes())?;
    push_bytes(out, "digest", digest.as_bytes())
}

fn push_bytes(
    out: &mut Vec<u8>,
    field: &'static str,
    bytes: &[u8],
) -> Result<(), ReviewRevisionAuthError> {
    let len = u32::try_from(bytes.len()).map_err(|_| {
        ReviewRevisionAuthError::CanonicalFieldTooLarge {
            field,
            len: bytes.len(),
            max: u32::MAX as usize,
        }
    })?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(bytes);
    Ok(())
}

/// Review-revision authentication/qualification failure.
#[derive(Debug, Error, PartialEq, Eq)]
pub enum ReviewRevisionAuthError {
    /// Revision names another immutable proposal.
    #[error("review revision does not name the supplied proposal")]
    ProposalMismatch,
    /// Project contexts disagree.
    #[error("review revision authentication project mismatch")]
    ProjectMismatch,
    /// Authority epoch does not equal the proposal/revision context.
    #[error("review revision authority epoch mismatch")]
    AuthorityEpochMismatch,
    /// Authentication request names another authority epoch.
    #[error("review revision authentication request authority mismatch")]
    AuthenticationAuthorityMismatch,
    /// Authentication principal/binding differs from revision reviewer.
    #[error("review revision authentication reviewer mismatch")]
    ReviewerMismatch,
    /// Authentication request is not scoped to `ReviewSource`.
    #[error("review revision authentication is not scoped to ReviewSource")]
    WrongAuthenticationCapability,
    /// Authentication action subject differs from exact revision id.
    #[error("authentication action subject does not match exact review revision")]
    AuthenticationSubjectMismatch,
    /// Reviewer is not structurally eligible at supplied observation time.
    #[error("review revision reviewer is not eligible for ReviewSource")]
    ReviewerNotEligible,
    /// Structural revision used another FORGE-005A authentication object.
    #[error("review revision authentication evidence mismatch")]
    AuthenticationEvidenceMismatch,
    /// Provider verification belongs to another authentication observation.
    #[error("provider-verified observation does not match review revision authentication")]
    ProviderObservationMismatch,
    /// Supplied project-policy state is not the one committed by proposal.
    #[error("review revision project-policy state does not match proposal")]
    ProjectPolicyMismatch,
    /// Supplied provider-trust policy is not bound by project-policy state.
    #[error("review revision provider-trust policy mismatch")]
    ProviderTrustPolicyMismatch,
    /// Xenia verifier identity is not trusted by project policy.
    #[error("review revision Xenia verifier identity is not project-trusted")]
    VerifierNotTrusted,
    /// Canonical field exceeded v1 length bound.
    #[error("canonical field {field} is too large: {len} > {max}")]
    CanonicalFieldTooLarge {
        /// Field name.
        field: &'static str,
        /// Observed size.
        len: usize,
        /// Maximum size.
        max: usize,
    },
    /// Review-state canonicalization failure.
    #[error(transparent)]
    ReviewState(#[from] ReviewStateError),
    /// Authentication canonicalization failure.
    #[error(transparent)]
    Authentication(#[from] AuthenticationError),
    /// Authority canonicalization failure.
    #[error(transparent)]
    Authority(#[from] AuthorityError),
    /// Proposal canonicalization failure.
    #[error(transparent)]
    Proposal(#[from] ProposalError),
    /// Project-policy canonicalization failure.
    #[error(transparent)]
    ProjectPolicy(#[from] ProjectPolicyError),
}

#[cfg(test)]
mod tests {
    use super::*;
    use ed25519_dalek::{Signer as _, SigningKey};
    use ml_dsa::{
        B32, MlDsa65, Signature as MlDsaSignature, SigningKey as MlDsaSigningKey,
        signature::{Keypair as _, Signer as _},
    };
    use mycelix_forge_authentication::{
        PrincipalAuthenticationRequest, PrincipalBinding, bind_principal_authentication,
    };
    use mycelix_forge_authority::{
        AuthorityEpochParts, CapabilityRule, PrincipalGrant, PrincipalId,
    };
    use mycelix_forge_core::{DigestAlgorithm, ProjectIdentitySeed, GENESIS_NONCE_LEN};
    use mycelix_forge_project_policy::TrustedProviderVerifierV1;
    use mycelix_forge_repository::{
        GitObjectAlgorithm, GitObjectId, RepositoryPolicyState, RepositoryRef,
    };
    use mycelix_forge_review_state::ReviewRevisionDecision;
    use mycelix_forge_xenia::{
        XeniaHybridSuite, XeniaVerificationReceiptV1, xenia_challenge_commitment,
        xenia_key_lineage_commitment, xenia_operator_id_commitment,
    };
    use mycelix_forge_xenia_provider::{
        TrustedXeniaVerifierV1, XeniaProviderAttestedReceiptV1,
        provider_attestation_transcript_v1, verify_xenia_provider_authentication_v1,
    };
    use serde::Serialize;

    fn digest(byte: u8) -> Digest {
        Digest::new(DigestAlgorithm::Sha256, vec![byte; 32]).unwrap()
    }

    fn principal(byte: u8) -> PrincipalId {
        PrincipalId::new(digest(byte))
    }

    fn project() -> ProjectIdentity {
        ProjectIdentity::derive(
            &ProjectIdentitySeed::new([0x11; GENESIS_NONCE_LEN], digest(0x12)),
            DigestAlgorithm::Sha256,
        )
        .unwrap()
    }

    struct ProviderIdentity {
        ed: SigningKey,
        ml: MlDsaSigningKey<MlDsa65>,
    }

    impl ProviderIdentity {
        fn from_seeds(ed_seed: [u8; 32], ml_seed: [u8; 32]) -> Self {
            let ml_seed: B32 = ml_seed.into();
            Self {
                ed: SigningKey::from_bytes(&ed_seed),
                ml: MlDsaSigningKey::<MlDsa65>::from_seed(&ml_seed),
            }
        }

        fn trusted(&self) -> TrustedXeniaVerifierV1 {
            TrustedXeniaVerifierV1::new(
                self.ed.verifying_key().to_bytes(),
                self.ml.verifying_key().encode().as_slice().to_vec(),
            )
            .unwrap()
        }

        fn envelope(&self, receipt: XeniaVerificationReceiptV1) -> XeniaProviderAttestedReceiptV1 {
            #[derive(Serialize)]
            struct WireEnvelope {
                receipt: XeniaVerificationReceiptV1,
                verifier_identity_commitment: Digest,
                ed25519_signature: Vec<u8>,
                ml_dsa_65_signature: Vec<u8>,
            }
            let trusted = self.trusted();
            let transcript = provider_attestation_transcript_v1(
                &receipt,
                trusted.identity_commitment(),
            )
            .unwrap();
            let ml_signature: MlDsaSignature<MlDsa65> = self.ml.sign(&transcript);
            let wire = WireEnvelope {
                receipt,
                verifier_identity_commitment: trusted.identity_commitment().clone(),
                ed25519_signature: self.ed.sign(&transcript).to_bytes().to_vec(),
                ml_dsa_65_signature: ml_signature.encode().as_slice().to_vec(),
            };
            serde_json::from_slice(&serde_json::to_vec(&wire).unwrap()).unwrap()
        }
    }

    struct Context {
        provider: ProviderIdentity,
        authority: AuthorityEpoch,
        project_policy: ProjectPolicyStateV1,
        provider_trust: AuthenticationProviderTrustPolicyV1,
        proposal: ChangeProposal,
    }

    fn context(trusted_provider: bool) -> Context {
        let provider = ProviderIdentity::from_seeds([0x81; 32], [0x82; 32]);
        let trusted = provider.trusted();
        let trust_identity = if trusted_provider {
            trusted.identity_commitment().clone()
        } else {
            digest(0xa0)
        };
        let provider_trust = AuthenticationProviderTrustPolicyV1::new(
            project(),
            vec![TrustedProviderVerifierV1::new(
                xenia_provider_namespace(),
                trust_identity,
            )],
        )
        .unwrap();
        let project_policy = ProjectPolicyStateV1::new(
            project(),
            0,
            None,
            &provider_trust,
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        let reviewer = principal(0x20);
        let authority = AuthorityEpoch::new(AuthorityEpochParts {
            project: project(),
            sequence: 0,
            previous: None,
            valid_from_unix_ms: 1_000,
            valid_until_unix_ms: Some(2_000),
            grants: vec![
                PrincipalGrant::new(principal(0x10), [Capability::ManageAuthority]).unwrap(),
                PrincipalGrant::new(reviewer, [Capability::ReviewSource]).unwrap(),
            ],
            thresholds: vec![
                CapabilityRule::new(Capability::ManageAuthority, 1).unwrap(),
                CapabilityRule::new(Capability::ReviewSource, 1).unwrap(),
            ],
            revoked_principals: vec![],
        })
        .unwrap();
        let repository_policy =
            RepositoryPolicyState::new(project(), 0, None, digest(0x31)).unwrap();
        let proposal = ChangeProposal::new(
            principal(0x30),
            &authority,
            &project_policy,
            &repository_policy,
            RepositoryRef::new("refs/heads/main").unwrap(),
            GitObjectId::new(GitObjectAlgorithm::Sha1, vec![0x40; 20]).unwrap(),
            GitObjectId::new(GitObjectAlgorithm::Sha1, vec![0x41; 20]).unwrap(),
            GitObjectId::new(GitObjectAlgorithm::Sha1, vec![0x42; 20]).unwrap(),
            digest(0x50),
            vec![],
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        Context {
            provider,
            authority,
            project_policy,
            provider_trust,
            proposal,
        }
    }

    struct AuthFixture {
        authentication: EvidenceBoundPrincipalAuthentication,
        provider_verified: ProviderVerifiedXeniaAuthenticationV1,
    }

    fn authentication_for_revision(
        context: &Context,
        revision: &ReviewRevisionV1,
        marker: u8,
    ) -> AuthFixture {
        let operator = xenia_operator_id_commitment("operator:alice").unwrap();
        let lineage = xenia_key_lineage_commitment(&[0x21; 32], &[0x22; 64], None).unwrap();
        let binding = PrincipalBinding::new(
            xenia_provider_namespace(),
            operator.clone(),
            revision.reviewer().clone(),
            lineage.clone(),
        );
        let revision_id = revision.revision_id(DigestAlgorithm::Sha256).unwrap();
        let request = PrincipalAuthenticationRequest::new(
            project(),
            context.authority.digest(DigestAlgorithm::Sha256).unwrap(),
            revision.reviewer().clone(),
            binding.digest(DigestAlgorithm::Sha256).unwrap(),
            Capability::ReviewSource,
            revision_id.commitment().clone(),
            [marker; 32],
        );
        let receipt = XeniaVerificationReceiptV1::new(
            request.digest(DigestAlgorithm::Sha256).unwrap(),
            operator,
            lineage,
            xenia_challenge_commitment(request.challenge()),
            XeniaHybridSuite::Ed25519MlDsa65V1,
            digest(marker.wrapping_add(1)),
            digest(marker.wrapping_add(2)),
            digest(marker.wrapping_add(3)),
            1_797_000_000,
        );
        let verifier = context.provider.trusted();
        let envelope = context.provider.envelope(receipt);
        let provider_verified = verify_xenia_provider_authentication_v1(
            &verifier,
            &request,
            &binding,
            &envelope,
        )
        .unwrap();
        let authentication = bind_principal_authentication(
            request,
            binding,
            &context.authority,
            provider_verified.observation().clone(),
        )
        .unwrap();
        AuthFixture {
            authentication,
            provider_verified,
        }
    }

    #[test]
    fn exact_revision_id_crosses_full_auth_and_project_trust_chain() {
        let context = context(true);
        let revision = ReviewRevisionV1::genesis(
            &context.proposal,
            principal(0x20),
            ReviewRevisionDecision::Approve,
            digest(0x60),
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        let auth = authentication_for_revision(&context, &revision, 0x70);
        let bound = bind_review_revision_authentication_v1(
            &context.proposal,
            revision,
            &auth.authentication,
            &context.authority,
        )
        .unwrap();
        let eligible =
            qualify_review_revision_eligibility_v1(bound, &context.authority, 1_500).unwrap();
        let provider = bind_xenia_provider_verified_review_revision_v1(
            eligible,
            &auth.authentication,
            &auth.provider_verified,
        )
        .unwrap();
        let trusted = bind_project_policy_trusted_review_revision_v1(
            &context.proposal,
            &context.project_policy,
            &context.provider_trust,
            provider,
        )
        .unwrap();
        assert_eq!(
            trusted.revision().revision().revision().revision().decision(),
            ReviewRevisionDecision::Approve
        );
    }

    #[test]
    fn authenticated_revision_cannot_be_reused_for_sibling_revision() {
        let context = context(true);
        let genesis = ReviewRevisionV1::genesis(
            &context.proposal,
            principal(0x20),
            ReviewRevisionDecision::RequestChanges,
            digest(0x60),
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        let approve = ReviewRevisionV1::successor(
            &genesis,
            ReviewRevisionDecision::Approve,
            digest(0x61),
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        let withdraw = ReviewRevisionV1::successor(
            &genesis,
            ReviewRevisionDecision::Withdraw,
            digest(0x61),
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        let auth = authentication_for_revision(&context, &approve, 0x71);
        assert_eq!(
            bind_review_revision_authentication_v1(
                &context.proposal,
                withdraw,
                &auth.authentication,
                &context.authority,
            )
            .unwrap_err(),
            ReviewRevisionAuthError::AuthenticationSubjectMismatch
        );
    }

    #[test]
    fn predecessor_metadata_is_inside_authenticated_subject() {
        let context = context(true);
        let a0 = ReviewRevisionV1::genesis(
            &context.proposal,
            principal(0x20),
            ReviewRevisionDecision::RequestChanges,
            digest(0x60),
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        let b0 = ReviewRevisionV1::genesis(
            &context.proposal,
            principal(0x20),
            ReviewRevisionDecision::RequestChanges,
            digest(0x62),
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        let a1 = ReviewRevisionV1::successor(
            &a0,
            ReviewRevisionDecision::Approve,
            digest(0x63),
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        let b1 = ReviewRevisionV1::successor(
            &b0,
            ReviewRevisionDecision::Approve,
            digest(0x63),
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        let auth = authentication_for_revision(&context, &a1, 0x72);
        assert_eq!(
            bind_review_revision_authentication_v1(
                &context.proposal,
                b1,
                &auth.authentication,
                &context.authority,
            )
            .unwrap_err(),
            ReviewRevisionAuthError::AuthenticationSubjectMismatch
        );
    }

    #[test]
    fn provider_verified_observation_cannot_be_borrowed_across_revision_authentication() {
        let context = context(true);
        let revision = ReviewRevisionV1::genesis(
            &context.proposal,
            principal(0x20),
            ReviewRevisionDecision::Approve,
            digest(0x60),
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        let a = authentication_for_revision(&context, &revision, 0x73);
        let b = authentication_for_revision(&context, &revision, 0x74);
        let bound = bind_review_revision_authentication_v1(
            &context.proposal,
            revision,
            &b.authentication,
            &context.authority,
        )
        .unwrap();
        let eligible =
            qualify_review_revision_eligibility_v1(bound, &context.authority, 1_500).unwrap();
        assert_eq!(
            bind_xenia_provider_verified_review_revision_v1(
                eligible,
                &b.authentication,
                &a.provider_verified,
            )
            .unwrap_err(),
            ReviewRevisionAuthError::ProviderObservationMismatch
        );
    }

    #[test]
    fn cryptographically_valid_revision_from_untrusted_verifier_is_not_project_trusted() {
        let context = context(false);
        let revision = ReviewRevisionV1::genesis(
            &context.proposal,
            principal(0x20),
            ReviewRevisionDecision::Approve,
            digest(0x60),
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        let auth = authentication_for_revision(&context, &revision, 0x75);
        let bound = bind_review_revision_authentication_v1(
            &context.proposal,
            revision,
            &auth.authentication,
            &context.authority,
        )
        .unwrap();
        let eligible =
            qualify_review_revision_eligibility_v1(bound, &context.authority, 1_500).unwrap();
        let provider = bind_xenia_provider_verified_review_revision_v1(
            eligible,
            &auth.authentication,
            &auth.provider_verified,
        )
        .unwrap();
        assert_eq!(
            bind_project_policy_trusted_review_revision_v1(
                &context.proposal,
                &context.project_policy,
                &context.provider_trust,
                provider,
            )
            .unwrap_err(),
            ReviewRevisionAuthError::VerifierNotTrusted
        );
    }
}
