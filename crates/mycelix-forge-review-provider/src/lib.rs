// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Join provider-verified authentication to structurally eligible Forge reviews.
//!
//! FORGE-007 deliberately stops at structural review evidence and structural
//! `ReviewSource` eligibility. The Xenia provider adapter separately proves
//! that one exact authentication observation was hybrid-attested by an
//! explicitly trusted Xenia verifier identity. This crate joins those two
//! theorems without silently promoting either one into merge authority.
//!
//! ```text
//! StructurallyEligibleReview
//! + exact EvidenceBoundPrincipalAuthentication used by that review
//! + ProviderVerifiedXeniaAuthenticationV1 for that exact observation
//!     ↓
//! ProviderVerifiedEligibleReviewV1
//! ```
//!
//! The result still does **not** establish trusted time, approval quorum,
//! repository correctness, CI qualification, protected-history compliance, or
//! merge authorization. In particular, a provider-verified `RequestChanges`
//! statement remains `RequestChanges`; provider cryptography is not approval.

#![forbid(unsafe_code)]
#![warn(missing_docs)]

use mycelix_forge_authentication::{AuthenticationError, EvidenceBoundPrincipalAuthentication};
use mycelix_forge_authority::Capability;
use mycelix_forge_core::{Digest, ProtocolVersion};
use mycelix_forge_review::{ReviewError, StructurallyEligibleReview};
use mycelix_forge_xenia_provider::ProviderVerifiedXeniaAuthenticationV1;
use serde::Serialize;
use thiserror::Error;

const PROVIDER_VERIFIED_REVIEW_DOMAIN_V1: &[u8] =
    b"mycelix-forge/provider-verified-eligible-review/v1\0";

/// Positive result proving that one structurally eligible review uses the
/// exact FORGE-005A authentication observation whose Xenia provider provenance
/// was cryptographically verified.
///
/// This type is serializable for evidence export but intentionally not
/// deserializable: callers must construct it by re-running
/// [`bind_xenia_provider_verified_review_v1`].
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct ProviderVerifiedEligibleReviewV1 {
    review: StructurallyEligibleReview,
    authentication_evidence: Digest,
    provider_receipt: Digest,
    verifier_identity: Digest,
    evidence_commitment: Digest,
}

impl ProviderVerifiedEligibleReviewV1 {
    /// Structurally eligible review whose authentication provenance passed.
    pub fn review(&self) -> &StructurallyEligibleReview {
        &self.review
    }

    /// Exact FORGE-005A evidence commitment joined to the review.
    pub fn authentication_evidence(&self) -> &Digest {
        &self.authentication_evidence
    }

    /// Exact Xenia receipt whose trusted-provider provenance passed.
    pub fn provider_receipt(&self) -> &Digest {
        &self.provider_receipt
    }

    /// Exact trusted Xenia verifier identity that authenticated the receipt.
    pub fn verifier_identity(&self) -> &Digest {
        &self.verifier_identity
    }

    /// Aggregate commitment to the structural review, exact 005A evidence,
    /// provider receipt, provider identity, and exact authenticated observation.
    pub fn evidence_commitment(&self) -> &Digest {
        &self.evidence_commitment
    }
}

/// Join one provider-verified Xenia authentication to the exact structurally
/// eligible review that consumed its FORGE-005A evidence.
///
/// Two exact equalities are the critical anti-borrowing boundary:
///
/// 1. the review's stored authentication-evidence commitment must equal the
///    supplied `EvidenceBoundPrincipalAuthentication` commitment; and
/// 2. the provider-verified observation must equal the observation inside that
///    exact authentication object.
///
/// The function then rechecks the review-significant request cross-links so a
/// future refactor cannot accidentally weaken the join by relying on only one
/// opaque commitment comparison.
pub fn bind_xenia_provider_verified_review_v1(
    review: StructurallyEligibleReview,
    authentication: &EvidenceBoundPrincipalAuthentication,
    provider_verified: &ProviderVerifiedXeniaAuthenticationV1,
) -> Result<ProviderVerifiedEligibleReviewV1, ProviderVerifiedReviewError> {
    let evidence_bound_review = review.review();
    if evidence_bound_review.authentication_evidence() != authentication.evidence_commitment() {
        return Err(ProviderVerifiedReviewError::AuthenticationEvidenceMismatch);
    }

    if provider_verified.observation() != authentication.observation() {
        return Err(ProviderVerifiedReviewError::ProviderObservationMismatch);
    }

    let request = authentication.request();
    let statement = evidence_bound_review.statement();

    if request.project() != evidence_bound_review.project() {
        return Err(ProviderVerifiedReviewError::ProjectMismatch);
    }
    if request.authority_epoch() != evidence_bound_review.authority_epoch() {
        return Err(ProviderVerifiedReviewError::AuthorityEpochMismatch);
    }
    if request.principal() != statement.reviewer()
        || authentication.binding().forge_principal() != statement.reviewer()
    {
        return Err(ProviderVerifiedReviewError::ReviewerMismatch);
    }
    if request.capability() != Capability::ReviewSource {
        return Err(ProviderVerifiedReviewError::WrongAuthenticationCapability);
    }

    let expected_statement = statement.statement_id(request.action_subject().algorithm())?;
    if request.action_subject() != expected_statement.commitment() {
        return Err(ProviderVerifiedReviewError::AuthenticationSubjectMismatch);
    }

    let algorithm = review.evidence_commitment().algorithm();
    let observation = provider_verified.observation().digest(algorithm)?;
    let mut out = Vec::new();
    out.extend_from_slice(PROVIDER_VERIFIED_REVIEW_DOMAIN_V1);
    out.extend_from_slice(&ProtocolVersion::CURRENT.get().to_be_bytes());
    push_digest(&mut out, review.evidence_commitment())?;
    push_digest(&mut out, authentication.evidence_commitment())?;
    push_digest(&mut out, &observation)?;
    push_digest(&mut out, provider_verified.receipt_digest())?;
    push_digest(&mut out, provider_verified.verifier_identity_commitment())?;
    let evidence_commitment = Digest::of_bytes(algorithm, &out);

    Ok(ProviderVerifiedEligibleReviewV1 {
        review,
        authentication_evidence: authentication.evidence_commitment().clone(),
        provider_receipt: provider_verified.receipt_digest().clone(),
        verifier_identity: provider_verified.verifier_identity_commitment().clone(),
        evidence_commitment,
    })
}

fn push_digest(
    out: &mut Vec<u8>,
    digest: &Digest,
) -> Result<(), ProviderVerifiedReviewError> {
    push_bytes(out, "digest_algorithm", digest.algorithm().id().as_bytes())?;
    push_bytes(out, "digest", digest.as_bytes())
}

fn push_bytes(
    out: &mut Vec<u8>,
    field: &'static str,
    bytes: &[u8],
) -> Result<(), ProviderVerifiedReviewError> {
    let len = u32::try_from(bytes.len()).map_err(|_| {
        ProviderVerifiedReviewError::CanonicalFieldTooLarge {
            field,
            len: bytes.len(),
            max: u32::MAX as usize,
        }
    })?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(bytes);
    Ok(())
}

/// Provider-verified review join failure.
#[derive(Debug, Error, PartialEq, Eq)]
pub enum ProviderVerifiedReviewError {
    /// The structurally eligible review was built from a different 005A
    /// authentication-evidence object.
    #[error("review authentication evidence does not match supplied authentication")]
    AuthenticationEvidenceMismatch,
    /// Provider provenance was established for another authentication
    /// observation.
    #[error("provider-verified observation does not match review authentication observation")]
    ProviderObservationMismatch,
    /// Authentication request belongs to another project.
    #[error("provider-verified review authentication project mismatch")]
    ProjectMismatch,
    /// Authentication request names another authority epoch.
    #[error("provider-verified review authentication authority epoch mismatch")]
    AuthorityEpochMismatch,
    /// Authentication principal/binding differs from the review statement.
    #[error("provider-verified review reviewer mismatch")]
    ReviewerMismatch,
    /// Authentication request is not scoped to `ReviewSource`.
    #[error("provider-verified review authentication is not scoped to ReviewSource")]
    WrongAuthenticationCapability,
    /// Authentication action subject differs from the exact review statement.
    #[error("provider-verified review action subject mismatch")]
    AuthenticationSubjectMismatch,
    /// Canonical evidence field exceeded the v1 encoding bound.
    #[error("canonical field {field} is too large: {len} > {max}")]
    CanonicalFieldTooLarge {
        /// Field name.
        field: &'static str,
        /// Observed size.
        len: usize,
        /// Maximum size.
        max: usize,
    },
    /// Review statement canonicalization failed.
    #[error(transparent)]
    Review(#[from] ReviewError),
    /// Authentication observation canonicalization failed.
    #[error(transparent)]
    Authentication(#[from] AuthenticationError),
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
        AuthorityEpoch, AuthorityEpochParts, CapabilityRule, PrincipalGrant, PrincipalId,
    };
    use mycelix_forge_core::{
        DigestAlgorithm, ProjectIdentity, ProjectIdentitySeed, GENESIS_NONCE_LEN,
    };
    use mycelix_forge_proposal::ChangeProposal;
    use mycelix_forge_repository::{
        GitObjectAlgorithm, GitObjectId, RepositoryPolicyState, RepositoryRef,
    };
    use mycelix_forge_review::{
        ReviewDecision, ReviewStatement, bind_review_evidence, qualify_review_eligibility,
    };
    use mycelix_forge_xenia::{
        XeniaHybridSuite, XeniaVerificationReceiptV1, xenia_challenge_commitment,
        xenia_key_lineage_commitment, xenia_operator_id_commitment, xenia_provider_namespace,
    };
    use mycelix_forge_xenia_provider::{
        TrustedXeniaVerifierV1, XeniaProviderAttestedReceiptV1,
        provider_attestation_transcript_v1, verify_xenia_provider_authentication_v1,
    };
    use serde::Serialize;

    fn digest(byte: u8) -> Digest {
        Digest::new(DigestAlgorithm::Sha256, vec![byte; 32]).unwrap()
    }

    fn project() -> ProjectIdentity {
        ProjectIdentity::derive(
            &ProjectIdentitySeed::new([0x11; GENESIS_NONCE_LEN], digest(0x12)),
            DigestAlgorithm::Sha256,
        )
        .unwrap()
    }

    fn principal(byte: u8) -> PrincipalId {
        PrincipalId::new(digest(byte))
    }

    fn authority(reviewer: PrincipalId) -> AuthorityEpoch {
        AuthorityEpoch::new(AuthorityEpochParts {
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
        .unwrap()
    }

    fn git(byte: u8) -> GitObjectId {
        GitObjectId::new(GitObjectAlgorithm::Sha1, vec![byte; 20]).unwrap()
    }

    fn proposal(authority: &AuthorityEpoch) -> ChangeProposal {
        let policy = RepositoryPolicyState::new(project(), 0, None, digest(0x31)).unwrap();
        ChangeProposal::new(
            principal(0x30),
            authority,
            digest(0x32),
            &policy,
            RepositoryRef::new("refs/heads/main").unwrap(),
            git(0x40),
            git(0x41),
            git(0x42),
            digest(0x50),
            vec![],
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

    struct FullReviewFixture {
        eligible: StructurallyEligibleReview,
        authentication: EvidenceBoundPrincipalAuthentication,
        provider_verified: ProviderVerifiedXeniaAuthenticationV1,
    }

    fn full_review_fixture(
        provider: &ProviderIdentity,
        decision: ReviewDecision,
        challenge_byte: u8,
    ) -> FullReviewFixture {
        let reviewer = principal(0x20);
        let authority = authority(reviewer.clone());
        let proposal = proposal(&authority);
        let statement = ReviewStatement::new(
            &proposal,
            reviewer.clone(),
            decision,
            digest(0x60),
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        let statement_id = statement.statement_id(DigestAlgorithm::Sha256).unwrap();

        let operator = xenia_operator_id_commitment("operator:alice").unwrap();
        let lineage = xenia_key_lineage_commitment(&[0x21; 32], &[0x22; 64], None).unwrap();
        let binding = PrincipalBinding::new(
            xenia_provider_namespace(),
            operator.clone(),
            reviewer.clone(),
            lineage.clone(),
        );
        let request = PrincipalAuthenticationRequest::new(
            project(),
            authority.digest(DigestAlgorithm::Sha256).unwrap(),
            reviewer,
            binding.digest(DigestAlgorithm::Sha256).unwrap(),
            Capability::ReviewSource,
            statement_id.commitment().clone(),
            [challenge_byte; 32],
        );
        let receipt = XeniaVerificationReceiptV1::new(
            request.digest(DigestAlgorithm::Sha256).unwrap(),
            operator,
            lineage,
            xenia_challenge_commitment(request.challenge()),
            XeniaHybridSuite::Ed25519MlDsa65V1,
            digest(0x71),
            digest(0x72),
            digest(0x73),
            1_797_000_000,
        );
        let envelope = provider.envelope(receipt);
        let trusted = provider.trusted();
        let provider_verified = verify_xenia_provider_authentication_v1(
            &trusted,
            &request,
            &binding,
            &envelope,
        )
        .unwrap();
        let authentication = bind_principal_authentication(
            request,
            binding,
            &authority,
            provider_verified.observation().clone(),
        )
        .unwrap();
        let evidence_bound =
            bind_review_evidence(&proposal, statement, &authentication, &authority).unwrap();
        let eligible = qualify_review_eligibility(evidence_bound, &authority, 1_500).unwrap();

        FullReviewFixture {
            eligible,
            authentication,
            provider_verified,
        }
    }

    #[test]
    fn exact_provider_verified_authentication_upgrades_only_its_own_review() {
        let provider = ProviderIdentity::from_seeds([0x81; 32], [0x82; 32]);
        let fixture = full_review_fixture(&provider, ReviewDecision::Approve, 0x80);
        let positive = bind_xenia_provider_verified_review_v1(
            fixture.eligible,
            &fixture.authentication,
            &fixture.provider_verified,
        )
        .unwrap();

        assert_eq!(
            positive.authentication_evidence(),
            fixture.authentication.evidence_commitment()
        );
        assert_eq!(
            positive.provider_receipt(),
            fixture.provider_verified.receipt_digest()
        );
        assert_eq!(
            positive.verifier_identity(),
            fixture.provider_verified.verifier_identity_commitment()
        );
    }

    #[test]
    fn provider_verified_observation_cannot_be_borrowed_by_another_authentication() {
        let provider = ProviderIdentity::from_seeds([0x83; 32], [0x84; 32]);
        let a = full_review_fixture(&provider, ReviewDecision::Approve, 0x80);
        let b = full_review_fixture(&provider, ReviewDecision::Approve, 0x81);

        assert_eq!(
            bind_xenia_provider_verified_review_v1(
                b.eligible,
                &b.authentication,
                &a.provider_verified,
            )
            .err()
            .unwrap(),
            ProviderVerifiedReviewError::ProviderObservationMismatch
        );
    }

    #[test]
    fn structurally_eligible_review_cannot_swap_its_005a_authentication_object() {
        let provider = ProviderIdentity::from_seeds([0x85; 32], [0x86; 32]);
        let a = full_review_fixture(&provider, ReviewDecision::Approve, 0x82);
        let b = full_review_fixture(&provider, ReviewDecision::Approve, 0x83);

        assert_eq!(
            bind_xenia_provider_verified_review_v1(
                a.eligible,
                &b.authentication,
                &b.provider_verified,
            )
            .err()
            .unwrap(),
            ProviderVerifiedReviewError::AuthenticationEvidenceMismatch
        );
    }

    #[test]
    fn provider_verification_does_not_turn_request_changes_into_approval() {
        let provider = ProviderIdentity::from_seeds([0x87; 32], [0x88; 32]);
        let fixture = full_review_fixture(&provider, ReviewDecision::RequestChanges, 0x84);
        let positive = bind_xenia_provider_verified_review_v1(
            fixture.eligible,
            &fixture.authentication,
            &fixture.provider_verified,
        )
        .unwrap();

        assert_eq!(
            positive.review().review().statement().decision(),
            ReviewDecision::RequestChanges
        );
    }
}
