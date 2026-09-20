// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Project-policy trust for provider-verified Forge authentication.
//!
//! This crate extracts a reusable theorem that capability-specific Forge layers
//! otherwise risk reimplementing differently:
//!
//! ```text
//! EvidenceBoundPrincipalAuthentication
//! + Xenia provider provenance
//! + exact ProjectPolicyStateV1 committed by the proposal
//! + exact authentication-provider trust policy
//! + exact (Xenia namespace, verifier identity) trust membership
//!     ↓
//! ProjectPolicyTrustedXeniaAuthenticationV1
//! ```
//!
//! The result proves provider provenance is trusted by the exact project policy.
//! It does **not** prove that the principal is eligible for the requested
//! capability, satisfy a capability threshold, establish trusted time, or grant
//! merge/release/build authority.

#![forbid(unsafe_code)]
#![warn(missing_docs)]

use mycelix_forge_authentication::{
    AuthenticationError, EvidenceBoundPrincipalAuthentication,
};
use mycelix_forge_core::{Digest, ProtocolVersion};
use mycelix_forge_project_policy::{
    AuthenticationProviderTrustPolicyV1, ProjectPolicyError, ProjectPolicyStateV1,
};
use mycelix_forge_proposal::{ChangeProposal, ProposalError};
use mycelix_forge_xenia::xenia_provider_namespace;
use mycelix_forge_xenia_provider::ProviderVerifiedXeniaAuthenticationV1;
use serde::Serialize;
use thiserror::Error;

const TRUSTED_AUTH_DOMAIN_V1: &[u8] =
    b"mycelix-forge/project-policy-trusted-xenia-authentication/v1\0";

/// Positive result proving that one exact provider-verified Xenia
/// authentication uses a verifier identity trusted by the exact project policy
/// committed into the proposal.
///
/// This type is serializable for evidence export but intentionally not
/// deserializable into positive authority.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct ProjectPolicyTrustedXeniaAuthenticationV1 {
    authentication: EvidenceBoundPrincipalAuthentication,
    provider_receipt: Digest,
    verifier_identity: Digest,
    project_policy: Digest,
    provider_trust_policy: Digest,
    evidence_commitment: Digest,
}

impl ProjectPolicyTrustedXeniaAuthenticationV1 {
    /// Exact FORGE-005A authentication whose provider provenance is trusted.
    pub fn authentication(&self) -> &EvidenceBoundPrincipalAuthentication {
        &self.authentication
    }

    /// Exact Xenia receipt whose provider provenance passed.
    pub fn provider_receipt(&self) -> &Digest {
        &self.provider_receipt
    }

    /// Exact project-trusted Xenia verifier identity.
    pub fn verifier_identity(&self) -> &Digest {
        &self.verifier_identity
    }

    /// Exact project-policy state committed by the proposal.
    pub fn project_policy(&self) -> &Digest {
        &self.project_policy
    }

    /// Exact authentication-provider trust policy bound by project policy.
    pub fn provider_trust_policy(&self) -> &Digest {
        &self.provider_trust_policy
    }

    /// Aggregate evidence commitment for this trust join.
    pub fn evidence_commitment(&self) -> &Digest {
        &self.evidence_commitment
    }
}

/// Bind one provider-verified Xenia authentication to the exact authentication
/// provider trust policy committed by a proposal.
///
/// The function is capability-neutral: `ReviewSource`, `Witness`,
/// `MergeProtected`, `QualifyBuild`, `Release`, and other Forge capabilities
/// retain their exact request scope. Capability eligibility and thresholds are
/// intentionally left to later authority layers.
pub fn bind_project_policy_trusted_xenia_authentication_v1(
    proposal: &ChangeProposal,
    project_policy: &ProjectPolicyStateV1,
    provider_trust: &AuthenticationProviderTrustPolicyV1,
    authentication: &EvidenceBoundPrincipalAuthentication,
    provider_verified: &ProviderVerifiedXeniaAuthenticationV1,
) -> Result<ProjectPolicyTrustedXeniaAuthenticationV1, AuthenticationPolicyError> {
    if project_policy.project() != proposal.project()
        || provider_trust.project() != proposal.project()
        || authentication.request().project() != proposal.project()
    {
        return Err(AuthenticationPolicyError::ProjectMismatch);
    }

    if authentication.request().authority_epoch() != proposal.authority_epoch() {
        return Err(AuthenticationPolicyError::AuthorityEpochMismatch);
    }

    let xenia_namespace = xenia_provider_namespace();
    if authentication.binding().provider_namespace() != &xenia_namespace {
        return Err(AuthenticationPolicyError::WrongProviderNamespace);
    }

    if provider_verified.observation() != authentication.observation() {
        return Err(AuthenticationPolicyError::ProviderObservationMismatch);
    }

    let expected_project_policy = project_policy.digest(proposal.project_policy().algorithm())?;
    if proposal.project_policy() != &expected_project_policy {
        return Err(AuthenticationPolicyError::ProjectPolicyMismatch);
    }
    if !project_policy.binds_provider_trust(provider_trust)? {
        return Err(AuthenticationPolicyError::ProviderTrustPolicyMismatch);
    }
    if !provider_trust.trusts(
        &xenia_namespace,
        provider_verified.verifier_identity_commitment(),
    ) {
        return Err(AuthenticationPolicyError::VerifierNotTrusted);
    }

    let provider_trust_policy =
        provider_trust.digest(project_policy.authentication_provider_trust().algorithm())?;
    let algorithm = authentication.evidence_commitment().algorithm();
    let mut out = Vec::new();
    out.extend_from_slice(TRUSTED_AUTH_DOMAIN_V1);
    out.extend_from_slice(&ProtocolVersion::CURRENT.get().to_be_bytes());
    push_digest(&mut out, authentication.evidence_commitment())?;
    push_digest(&mut out, provider_verified.receipt_digest())?;
    push_digest(
        &mut out,
        provider_verified.verifier_identity_commitment(),
    )?;
    push_digest(&mut out, &expected_project_policy)?;
    push_digest(&mut out, &provider_trust_policy)?;
    push_digest(&mut out, &xenia_namespace)?;
    let evidence_commitment = Digest::of_bytes(algorithm, &out);

    Ok(ProjectPolicyTrustedXeniaAuthenticationV1 {
        authentication: authentication.clone(),
        provider_receipt: provider_verified.receipt_digest().clone(),
        verifier_identity: provider_verified.verifier_identity_commitment().clone(),
        project_policy: expected_project_policy,
        provider_trust_policy,
        evidence_commitment,
    })
}

fn push_digest(out: &mut Vec<u8>, digest: &Digest) -> Result<(), AuthenticationPolicyError> {
    push_bytes(out, "digest_algorithm", digest.algorithm().id().as_bytes())?;
    push_bytes(out, "digest", digest.as_bytes())
}

fn push_bytes(
    out: &mut Vec<u8>,
    field: &'static str,
    bytes: &[u8],
) -> Result<(), AuthenticationPolicyError> {
    let len = u32::try_from(bytes.len()).map_err(|_| {
        AuthenticationPolicyError::CanonicalFieldTooLarge {
            field,
            len: bytes.len(),
            max: u32::MAX as usize,
        }
    })?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(bytes);
    Ok(())
}

/// Project-policy authentication trust-join failure.
#[derive(Debug, Error, PartialEq, Eq)]
pub enum AuthenticationPolicyError {
    /// Proposal, project policy, trust policy, and authentication do not name
    /// one exact project.
    #[error("project-policy trusted authentication project mismatch")]
    ProjectMismatch,
    /// Authentication request names another authority epoch than the proposal.
    #[error("project-policy trusted authentication authority epoch mismatch")]
    AuthorityEpochMismatch,
    /// The principal binding is not from the Xenia Forge provider namespace.
    #[error("project-policy trusted authentication uses the wrong provider namespace")]
    WrongProviderNamespace,
    /// Provider verification belongs to another authentication observation.
    #[error("provider-verified observation does not match authentication observation")]
    ProviderObservationMismatch,
    /// Supplied project policy is not the exact policy committed by proposal.
    #[error("project-policy state does not match proposal commitment")]
    ProjectPolicyMismatch,
    /// Supplied provider trust policy is not bound by project policy.
    #[error("authentication provider trust policy is not bound by project policy")]
    ProviderTrustPolicyMismatch,
    /// The provider verifier identity is not trusted by project policy.
    #[error("Xenia verifier identity is not trusted by project policy")]
    VerifierNotTrusted,
    /// Canonical field exceeded the v1 encoding bound.
    #[error("canonical field {field} is too large: {len} > {max}")]
    CanonicalFieldTooLarge {
        /// Field name.
        field: &'static str,
        /// Observed length.
        len: usize,
        /// Maximum encodable length.
        max: usize,
    },
    /// Project-policy validation/canonicalization failure.
    #[error(transparent)]
    ProjectPolicy(#[from] ProjectPolicyError),
    /// Proposal validation/canonicalization failure.
    #[error(transparent)]
    Proposal(#[from] ProposalError),
    /// Authentication validation/canonicalization failure.
    #[error(transparent)]
    Authentication(#[from] AuthenticationError),
}

#[cfg(test)]
mod tests {
    use super::*;
    use ed25519_dalek::{Signer as _, SigningKey};
    use ml_dsa::{
        signature::{Keypair as _, Signer as _}, B32, MlDsa65,
        Signature as MlDsaSignature, SigningKey as MlDsaSigningKey,
    };
    use mycelix_forge_authentication::{
        bind_principal_authentication, PrincipalAuthenticationRequest, PrincipalBinding,
    };
    use mycelix_forge_authority::{
        AuthorityEpoch, AuthorityEpochParts, Capability, CapabilityRule, PrincipalGrant,
        PrincipalId,
    };
    use mycelix_forge_core::{
        DigestAlgorithm, ProjectIdentity, ProjectIdentitySeed, GENESIS_NONCE_LEN,
    };
    use mycelix_forge_project_policy::TrustedProviderVerifierV1;
    use mycelix_forge_repository::{
        GitObjectAlgorithm, GitObjectId, RepositoryPolicyState, RepositoryRef,
    };
    use mycelix_forge_xenia::{
        xenia_challenge_commitment, xenia_key_lineage_commitment,
        xenia_operator_id_commitment, XeniaHybridSuite, XeniaVerificationReceiptV1,
    };
    use mycelix_forge_xenia_provider::{
        provider_attestation_transcript_v1, verify_xenia_provider_authentication_v1,
        TrustedXeniaVerifierV1, XeniaProviderAttestedReceiptV1,
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
        fn new(marker: u8) -> Self {
            let seed: B32 = [marker.wrapping_add(1); 32].into();
            Self {
                ed: SigningKey::from_bytes(&[marker; 32]),
                ml: MlDsaSigningKey::<MlDsa65>::from_seed(&seed),
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

    fn context(trust_real_provider: bool) -> Context {
        let provider = ProviderIdentity::new(0x81);
        let verifier_identity = if trust_real_provider {
            provider.trusted().identity_commitment().clone()
        } else {
            digest(0xa1)
        };
        let provider_trust = AuthenticationProviderTrustPolicyV1::new(
            project(),
            vec![TrustedProviderVerifierV1::new(
                xenia_provider_namespace(),
                verifier_identity,
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
        let authority = AuthorityEpoch::new(AuthorityEpochParts {
            project: project(),
            sequence: 0,
            previous: None,
            valid_from_unix_ms: 1_000,
            valid_until_unix_ms: Some(10_000),
            grants: vec![PrincipalGrant::new(
                principal(0x10),
                [Capability::ManageAuthority, Capability::Witness],
            )
            .unwrap()],
            thresholds: vec![
                CapabilityRule::new(Capability::ManageAuthority, 1).unwrap(),
                CapabilityRule::new(Capability::Witness, 1).unwrap(),
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

    fn authentication(
        context: &Context,
        capability: Capability,
        marker: u8,
        provider_namespace: Digest,
    ) -> AuthFixture {
        let witness = principal(0x10);
        let operator = xenia_operator_id_commitment("operator:witness").unwrap();
        let lineage = xenia_key_lineage_commitment(&[0x21; 32], &[0x22; 64], None).unwrap();
        let binding = PrincipalBinding::new(
            provider_namespace,
            operator.clone(),
            witness.clone(),
            lineage.clone(),
        );
        let request = PrincipalAuthenticationRequest::new(
            project(),
            context.authority.digest(DigestAlgorithm::Sha256).unwrap(),
            witness,
            binding.digest(DigestAlgorithm::Sha256).unwrap(),
            capability,
            digest(0x60),
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
        let trusted = context.provider.trusted();
        let envelope = context.provider.envelope(receipt);
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
    fn capability_neutral_trust_join_preserves_witness_scope() {
        let context = context(true);
        let auth = authentication(
            &context,
            Capability::Witness,
            0x70,
            xenia_provider_namespace(),
        );
        let trusted = bind_project_policy_trusted_xenia_authentication_v1(
            &context.proposal,
            &context.project_policy,
            &context.provider_trust,
            &auth.authentication,
            &auth.provider_verified,
        )
        .unwrap();
        assert_eq!(
            trusted.authentication().request().capability(),
            Capability::Witness
        );
    }

    #[test]
    fn cryptographically_valid_but_project_untrusted_verifier_is_rejected() {
        let context = context(false);
        let auth = authentication(
            &context,
            Capability::Witness,
            0x71,
            xenia_provider_namespace(),
        );
        assert_eq!(
            bind_project_policy_trusted_xenia_authentication_v1(
                &context.proposal,
                &context.project_policy,
                &context.provider_trust,
                &auth.authentication,
                &auth.provider_verified,
            )
            .unwrap_err(),
            AuthenticationPolicyError::VerifierNotTrusted
        );
    }

    #[test]
    fn provider_observation_cannot_be_borrowed_between_challenges() {
        let context = context(true);
        let a = authentication(
            &context,
            Capability::Witness,
            0x72,
            xenia_provider_namespace(),
        );
        let b = authentication(
            &context,
            Capability::Witness,
            0x73,
            xenia_provider_namespace(),
        );
        assert_eq!(
            bind_project_policy_trusted_xenia_authentication_v1(
                &context.proposal,
                &context.project_policy,
                &context.provider_trust,
                &b.authentication,
                &a.provider_verified,
            )
            .unwrap_err(),
            AuthenticationPolicyError::ProviderObservationMismatch
        );
    }
}
