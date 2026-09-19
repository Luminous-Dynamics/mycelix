// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Provider-neutral principal-authentication binding for Mycelix Forge.
//!
//! This crate is deliberately crypto-free and I/O-free. It defines the exact
//! subjects an authentication provider must bind, and it checks that provider
//! evidence is tied to one project, authority epoch, principal, capability,
//! action subject, key lineage, and single challenge.
//!
//! It does **not** verify signatures, consume challenges, authorize a
//! capability, or satisfy a quorum. A provider adapter (for example Xenia)
//! must establish cryptographic validity and freshness separately; Forge's
//! authority/quorum layer remains responsible for authorization.

#![forbid(unsafe_code)]
#![warn(missing_docs)]

use mycelix_forge_authority::{AuthorityEpoch, AuthorityError, Capability, PrincipalId};
use mycelix_forge_core::{
    Digest, DigestAlgorithm, ProjectIdentity, ProtocolVersion, CURRENT_PROTOCOL_VERSION,
};
use serde::{Deserialize, Serialize};
use thiserror::Error;

const PRINCIPAL_BINDING_DOMAIN_V1: &[u8] = b"mycelix-forge/principal-binding/v1\0";
const AUTH_REQUEST_DOMAIN_V1: &[u8] = b"mycelix-forge/principal-authentication-request/v1\0";
const AUTH_OBSERVATION_DOMAIN_V1: &[u8] = b"mycelix-forge/principal-authentication-observation/v1\0";
const AUTH_EVIDENCE_DOMAIN_V1: &[u8] = b"mycelix-forge/evidence-bound-principal-authentication/v1\0";

/// Exact challenge size for Forge principal-authentication protocol v1.
pub const AUTHENTICATION_CHALLENGE_LEN: usize = 32;

/// Stable provider/logical-identity to Forge-principal binding.
///
/// `provider_namespace` identifies the authentication namespace or provider
/// contract, while `provider_principal` identifies the logical external
/// identity inside that namespace. `key_lineage` is deliberately separate:
/// key rotation/recovery may change the lineage while retaining the same
/// logical Forge principal.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct PrincipalBinding {
    version: ProtocolVersion,
    provider_namespace: Digest,
    provider_principal: Digest,
    forge_principal: PrincipalId,
    key_lineage: Digest,
}

impl PrincipalBinding {
    /// Construct a v1 principal binding.
    pub fn new(
        provider_namespace: Digest,
        provider_principal: Digest,
        forge_principal: PrincipalId,
        key_lineage: Digest,
    ) -> Self {
        Self {
            version: ProtocolVersion::CURRENT,
            provider_namespace,
            provider_principal,
            forge_principal,
            key_lineage,
        }
    }

    /// Protocol version.
    pub const fn version(&self) -> ProtocolVersion {
        self.version
    }

    /// Authentication-provider namespace commitment.
    pub fn provider_namespace(&self) -> &Digest {
        &self.provider_namespace
    }

    /// Provider-local logical identity commitment.
    pub fn provider_principal(&self) -> &Digest {
        &self.provider_principal
    }

    /// Stable Forge principal represented by this binding.
    pub fn forge_principal(&self) -> &PrincipalId {
        &self.forge_principal
    }

    /// Current provider-specific key-lineage commitment.
    pub fn key_lineage(&self) -> &Digest {
        &self.key_lineage
    }

    /// Canonical v1 bytes.
    pub fn canonical_bytes(&self) -> Result<Vec<u8>, AuthenticationError> {
        ensure_v1(self.version)?;
        let mut out = Vec::new();
        out.extend_from_slice(PRINCIPAL_BINDING_DOMAIN_V1);
        out.extend_from_slice(&self.version.get().to_be_bytes());
        push_digest(&mut out, &self.provider_namespace)?;
        push_digest(&mut out, &self.provider_principal)?;
        push_digest(&mut out, self.forge_principal.commitment())?;
        push_digest(&mut out, &self.key_lineage)?;
        Ok(out)
    }

    /// Digest identifying this exact normalized binding.
    pub fn digest(&self, algorithm: DigestAlgorithm) -> Result<Digest, AuthenticationError> {
        Ok(Digest::of_bytes(algorithm, &self.canonical_bytes()?))
    }
}

/// Exact subject a provider must authenticate.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct PrincipalAuthenticationRequest {
    version: ProtocolVersion,
    project: ProjectIdentity,
    authority_epoch: Digest,
    principal: PrincipalId,
    binding_commitment: Digest,
    capability: Capability,
    action_subject: Digest,
    challenge: [u8; AUTHENTICATION_CHALLENGE_LEN],
}

impl PrincipalAuthenticationRequest {
    /// Construct a v1 authentication request.
    pub fn new(
        project: ProjectIdentity,
        authority_epoch: Digest,
        principal: PrincipalId,
        binding_commitment: Digest,
        capability: Capability,
        action_subject: Digest,
        challenge: [u8; AUTHENTICATION_CHALLENGE_LEN],
    ) -> Self {
        Self {
            version: ProtocolVersion::CURRENT,
            project,
            authority_epoch,
            principal,
            binding_commitment,
            capability,
            action_subject,
            challenge,
        }
    }

    /// Protocol version.
    pub const fn version(&self) -> ProtocolVersion {
        self.version
    }

    /// Project whose authority context is being authenticated.
    pub fn project(&self) -> &ProjectIdentity {
        &self.project
    }

    /// Exact authority epoch commitment named by this request.
    pub fn authority_epoch(&self) -> &Digest {
        &self.authority_epoch
    }

    /// Forge principal expected to authenticate.
    pub fn principal(&self) -> &PrincipalId {
        &self.principal
    }

    /// Exact provider/logical-identity/key-lineage binding commitment.
    pub fn binding_commitment(&self) -> &Digest {
        &self.binding_commitment
    }

    /// Capability-scoping tag. Authentication does not imply authorization.
    pub const fn capability(&self) -> Capability {
        self.capability
    }

    /// Exact action/change/release subject being authenticated.
    pub fn action_subject(&self) -> &Digest {
        &self.action_subject
    }

    /// Verifier-issued freshness challenge.
    pub const fn challenge(&self) -> &[u8; AUTHENTICATION_CHALLENGE_LEN] {
        &self.challenge
    }

    /// Canonical v1 bytes providers should sign/verify or otherwise bind.
    pub fn canonical_bytes(&self) -> Result<Vec<u8>, AuthenticationError> {
        ensure_v1(self.version)?;
        let mut out = Vec::new();
        out.extend_from_slice(AUTH_REQUEST_DOMAIN_V1);
        out.extend_from_slice(&self.version.get().to_be_bytes());
        push_project_identity(&mut out, &self.project)?;
        push_digest(&mut out, &self.authority_epoch)?;
        push_digest(&mut out, self.principal.commitment())?;
        push_digest(&mut out, &self.binding_commitment)?;
        out.extend_from_slice(&self.capability.code().to_be_bytes());
        push_digest(&mut out, &self.action_subject)?;
        out.extend_from_slice(&self.challenge);
        Ok(out)
    }

    /// Digest identifying this exact authentication request.
    pub fn digest(&self, algorithm: DigestAlgorithm) -> Result<Digest, AuthenticationError> {
        Ok(Digest::of_bytes(algorithm, &self.canonical_bytes()?))
    }
}

/// Raw provider/verifier observation to be bound by Forge.
///
/// This type does not claim that the provider actually verified cryptography
/// or consumed the challenge. `provider_evidence` and `freshness_evidence`
/// are opaque commitments whose truth must be established by the adapter.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct AuthenticationObservation {
    version: ProtocolVersion,
    request_commitment: Digest,
    provider_evidence: Digest,
    freshness_evidence: Digest,
}

impl AuthenticationObservation {
    /// Construct a v1 raw observation.
    pub fn new(
        request_commitment: Digest,
        provider_evidence: Digest,
        freshness_evidence: Digest,
    ) -> Self {
        Self {
            version: ProtocolVersion::CURRENT,
            request_commitment,
            provider_evidence,
            freshness_evidence,
        }
    }

    /// Exact request the provider claims to have verified.
    pub fn request_commitment(&self) -> &Digest {
        &self.request_commitment
    }

    /// Commitment to provider-specific cryptographic verification evidence.
    pub fn provider_evidence(&self) -> &Digest {
        &self.provider_evidence
    }

    /// Commitment to provider/verifier freshness or challenge-consumption evidence.
    pub fn freshness_evidence(&self) -> &Digest {
        &self.freshness_evidence
    }

    /// Canonical v1 bytes.
    pub fn canonical_bytes(&self) -> Result<Vec<u8>, AuthenticationError> {
        ensure_v1(self.version)?;
        let mut out = Vec::new();
        out.extend_from_slice(AUTH_OBSERVATION_DOMAIN_V1);
        out.extend_from_slice(&self.version.get().to_be_bytes());
        push_digest(&mut out, &self.request_commitment)?;
        push_digest(&mut out, &self.provider_evidence)?;
        push_digest(&mut out, &self.freshness_evidence)?;
        Ok(out)
    }

    /// Digest identifying this exact raw observation.
    pub fn digest(&self, algorithm: DigestAlgorithm) -> Result<Digest, AuthenticationError> {
        Ok(Digest::of_bytes(algorithm, &self.canonical_bytes()?))
    }
}

/// Positive structural result proving that provider evidence is bound to the
/// exact Forge project/authority/principal/action request supplied by the
/// caller.
///
/// This is deliberately named **evidence-bound**, not "authorized" or
/// "cryptographically verified". Provider adapters establish those stronger
/// claims separately.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct EvidenceBoundPrincipalAuthentication {
    request: PrincipalAuthenticationRequest,
    binding: PrincipalBinding,
    observation: AuthenticationObservation,
    evidence_commitment: Digest,
}

impl EvidenceBoundPrincipalAuthentication {
    /// Bound request.
    pub fn request(&self) -> &PrincipalAuthenticationRequest {
        &self.request
    }

    /// Bound external-principal/key-lineage mapping.
    pub fn binding(&self) -> &PrincipalBinding {
        &self.binding
    }

    /// Provider/freshness observation.
    pub fn observation(&self) -> &AuthenticationObservation {
        &self.observation
    }

    /// Aggregate evidence commitment.
    pub fn evidence_commitment(&self) -> &Digest {
        &self.evidence_commitment
    }
}

/// Bind provider authentication evidence to exact Forge authority subjects.
///
/// This function intentionally does **not** check principal eligibility or
/// capability thresholds. Those are authorization decisions owned by
/// `mycelix-forge-authority`.
pub fn bind_principal_authentication(
    request: PrincipalAuthenticationRequest,
    binding: PrincipalBinding,
    authority_epoch: &AuthorityEpoch,
    observation: AuthenticationObservation,
) -> Result<EvidenceBoundPrincipalAuthentication, AuthenticationError> {
    if request.project() != authority_epoch.project() {
        return Err(AuthenticationError::ProjectMismatch);
    }

    let expected_epoch = authority_epoch.digest(request.authority_epoch().algorithm())?;
    if &expected_epoch != request.authority_epoch() {
        return Err(AuthenticationError::AuthorityEpochMismatch);
    }

    if binding.forge_principal() != request.principal() {
        return Err(AuthenticationError::PrincipalMismatch);
    }

    let expected_binding = binding.digest(request.binding_commitment().algorithm())?;
    if &expected_binding != request.binding_commitment() {
        return Err(AuthenticationError::BindingMismatch);
    }

    let expected_request = request.digest(observation.request_commitment().algorithm())?;
    if &expected_request != observation.request_commitment() {
        return Err(AuthenticationError::ObservationRequestMismatch);
    }

    let algorithm = observation.provider_evidence().algorithm();
    let evidence_commitment = evidence_commitment(
        algorithm,
        &request,
        &binding,
        &observation,
        &expected_epoch,
    )?;

    Ok(EvidenceBoundPrincipalAuthentication {
        request,
        binding,
        observation,
        evidence_commitment,
    })
}

/// Authentication-contract failures.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum AuthenticationError {
    /// Unsupported Forge protocol version.
    #[error("unsupported Forge authentication protocol version: {0}")]
    UnsupportedProtocolVersion(u16),
    /// Canonical field exceeded a v1 encoding bound.
    #[error("canonical field {field} is too large: {len} > {max}")]
    CanonicalFieldTooLarge {
        /// Field name.
        field: &'static str,
        /// Observed byte length.
        len: usize,
        /// Maximum encodable length.
        max: usize,
    },
    /// Request project differs from the supplied authority epoch.
    #[error("authentication request project does not match authority epoch")]
    ProjectMismatch,
    /// Request authority digest is not the supplied authority epoch.
    #[error("authentication request authority epoch commitment does not match")]
    AuthorityEpochMismatch,
    /// Principal binding names a different Forge principal.
    #[error("principal binding does not name the requested Forge principal")]
    PrincipalMismatch,
    /// Request binding commitment does not identify the supplied binding.
    #[error("authentication request principal-binding commitment does not match")]
    BindingMismatch,
    /// Provider observation names a different request.
    #[error("authentication observation request commitment does not match")]
    ObservationRequestMismatch,
    /// Authority canonicalization/lineage error.
    #[error(transparent)]
    Authority(#[from] AuthorityError),
}

fn evidence_commitment(
    algorithm: DigestAlgorithm,
    request: &PrincipalAuthenticationRequest,
    binding: &PrincipalBinding,
    observation: &AuthenticationObservation,
    authority_epoch: &Digest,
) -> Result<Digest, AuthenticationError> {
    let request_digest = request.digest(algorithm)?;
    let binding_digest = binding.digest(algorithm)?;
    let observation_digest = observation.digest(algorithm)?;
    let mut out = Vec::new();
    out.extend_from_slice(AUTH_EVIDENCE_DOMAIN_V1);
    out.extend_from_slice(&ProtocolVersion::CURRENT.get().to_be_bytes());
    push_digest(&mut out, &request_digest)?;
    push_digest(&mut out, &binding_digest)?;
    push_digest(&mut out, &observation_digest)?;
    push_digest(&mut out, authority_epoch)?;
    Ok(Digest::of_bytes(algorithm, &out))
}

fn ensure_v1(version: ProtocolVersion) -> Result<(), AuthenticationError> {
    if version.get() == CURRENT_PROTOCOL_VERSION {
        Ok(())
    } else {
        Err(AuthenticationError::UnsupportedProtocolVersion(version.get()))
    }
}

fn push_project_identity(
    out: &mut Vec<u8>,
    project: &ProjectIdentity,
) -> Result<(), AuthenticationError> {
    out.extend_from_slice(&project.version().get().to_be_bytes());
    push_digest(out, project.digest())
}

fn push_digest(out: &mut Vec<u8>, digest: &Digest) -> Result<(), AuthenticationError> {
    let algorithm = digest.algorithm().id().as_bytes();
    let algorithm_len = u16::try_from(algorithm.len()).map_err(|_| {
        AuthenticationError::CanonicalFieldTooLarge {
            field: "digest_algorithm",
            len: algorithm.len(),
            max: u16::MAX as usize,
        }
    })?;
    let digest_len = u32::try_from(digest.as_bytes().len()).map_err(|_| {
        AuthenticationError::CanonicalFieldTooLarge {
            field: "digest",
            len: digest.as_bytes().len(),
            max: u32::MAX as usize,
        }
    })?;
    out.extend_from_slice(&algorithm_len.to_be_bytes());
    out.extend_from_slice(algorithm);
    out.extend_from_slice(&digest_len.to_be_bytes());
    out.extend_from_slice(digest.as_bytes());
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_forge_authority::{AuthorityEpochParts, CapabilityRule, PrincipalGrant};
    use mycelix_forge_core::{ProjectIdentitySeed, GENESIS_NONCE_LEN};

    fn digest(byte: u8) -> Digest {
        Digest::new(DigestAlgorithm::Sha256, vec![byte; 32]).unwrap()
    }

    fn project() -> ProjectIdentity {
        let seed = ProjectIdentitySeed::new([0x11; GENESIS_NONCE_LEN], digest(0x12));
        ProjectIdentity::derive(&seed, DigestAlgorithm::Sha256).unwrap()
    }

    fn principal(byte: u8) -> PrincipalId {
        PrincipalId::new(digest(byte))
    }

    fn epoch(project: ProjectIdentity, granted: PrincipalId) -> AuthorityEpoch {
        AuthorityEpoch::new(AuthorityEpochParts {
            project,
            sequence: 0,
            previous: None,
            valid_from_unix_ms: 100,
            valid_until_unix_ms: None,
            grants: vec![PrincipalGrant::new(
                granted,
                [Capability::ManageAuthority, Capability::ReviewSource],
            )
            .unwrap()],
            thresholds: vec![
                CapabilityRule::new(Capability::ManageAuthority, 1).unwrap(),
                CapabilityRule::new(Capability::ReviewSource, 1).unwrap(),
            ],
            revoked_principals: vec![],
        })
        .unwrap()
    }

    fn binding(principal: PrincipalId, lineage: u8) -> PrincipalBinding {
        PrincipalBinding::new(digest(0x20), digest(0x21), principal, digest(lineage))
    }

    fn request(
        project: ProjectIdentity,
        epoch: &AuthorityEpoch,
        principal: PrincipalId,
        binding: &PrincipalBinding,
    ) -> PrincipalAuthenticationRequest {
        PrincipalAuthenticationRequest::new(
            project,
            epoch.digest(DigestAlgorithm::Sha256).unwrap(),
            principal,
            binding.digest(DigestAlgorithm::Sha256).unwrap(),
            Capability::ReviewSource,
            digest(0x30),
            [0x40; AUTHENTICATION_CHALLENGE_LEN],
        )
    }

    fn observation(request: &PrincipalAuthenticationRequest) -> AuthenticationObservation {
        AuthenticationObservation::new(
            request.digest(DigestAlgorithm::Sha256).unwrap(),
            digest(0x50),
            digest(0x51),
        )
    }

    #[test]
    fn exact_subject_authentication_binds() {
        let project = project();
        let alice = principal(0x60);
        let epoch = epoch(project.clone(), alice.clone());
        let binding = binding(alice.clone(), 0x61);
        let request = request(project, &epoch, alice, &binding);
        let observation = observation(&request);

        let qualified = bind_principal_authentication(request.clone(), binding.clone(), &epoch, observation)
            .unwrap();
        assert_eq!(qualified.request(), &request);
        assert_eq!(qualified.binding(), &binding);
    }

    #[test]
    fn changing_action_subject_invalidates_observation() {
        let project = project();
        let alice = principal(0x60);
        let epoch = epoch(project.clone(), alice.clone());
        let binding = binding(alice.clone(), 0x61);
        let original = request(project.clone(), &epoch, alice.clone(), &binding);
        let observation = observation(&original);
        let mutated = PrincipalAuthenticationRequest::new(
            project,
            epoch.digest(DigestAlgorithm::Sha256).unwrap(),
            alice,
            binding.digest(DigestAlgorithm::Sha256).unwrap(),
            Capability::ReviewSource,
            digest(0x31),
            [0x40; AUTHENTICATION_CHALLENGE_LEN],
        );

        assert_eq!(
            bind_principal_authentication(mutated, binding, &epoch, observation).unwrap_err(),
            AuthenticationError::ObservationRequestMismatch
        );
    }

    #[test]
    fn changing_challenge_invalidates_observation() {
        let project = project();
        let alice = principal(0x60);
        let epoch = epoch(project.clone(), alice.clone());
        let binding = binding(alice.clone(), 0x61);
        let original = request(project.clone(), &epoch, alice.clone(), &binding);
        let observation = observation(&original);
        let mutated = PrincipalAuthenticationRequest::new(
            project,
            epoch.digest(DigestAlgorithm::Sha256).unwrap(),
            alice,
            binding.digest(DigestAlgorithm::Sha256).unwrap(),
            Capability::ReviewSource,
            digest(0x30),
            [0x41; AUTHENTICATION_CHALLENGE_LEN],
        );

        assert_eq!(
            bind_principal_authentication(mutated, binding, &epoch, observation).unwrap_err(),
            AuthenticationError::ObservationRequestMismatch
        );
    }

    #[test]
    fn changing_capability_invalidates_observation() {
        let project = project();
        let alice = principal(0x60);
        let epoch = epoch(project.clone(), alice.clone());
        let binding = binding(alice.clone(), 0x61);
        let original = request(project.clone(), &epoch, alice.clone(), &binding);
        let observation = observation(&original);
        let mutated = PrincipalAuthenticationRequest::new(
            project,
            epoch.digest(DigestAlgorithm::Sha256).unwrap(),
            alice,
            binding.digest(DigestAlgorithm::Sha256).unwrap(),
            Capability::MergeProtected,
            digest(0x30),
            [0x40; AUTHENTICATION_CHALLENGE_LEN],
        );

        assert_eq!(
            bind_principal_authentication(mutated, binding, &epoch, observation).unwrap_err(),
            AuthenticationError::ObservationRequestMismatch
        );
    }

    #[test]
    fn key_rotation_changes_binding_and_invalidates_old_request() {
        let project = project();
        let alice = principal(0x60);
        let epoch = epoch(project.clone(), alice.clone());
        let old_binding = binding(alice.clone(), 0x61);
        let request = request(project, &epoch, alice.clone(), &old_binding);
        let observation = observation(&request);
        let new_binding = binding(alice, 0x62);

        assert_eq!(
            bind_principal_authentication(request, new_binding, &epoch, observation).unwrap_err(),
            AuthenticationError::BindingMismatch
        );
    }

    #[test]
    fn project_and_authority_epoch_are_exact() {
        let project_a = project();
        let seed = ProjectIdentitySeed::new([0x99; GENESIS_NONCE_LEN], digest(0x12));
        let project_b = ProjectIdentity::derive(&seed, DigestAlgorithm::Sha256).unwrap();
        let alice = principal(0x60);
        let epoch_a = epoch(project_a.clone(), alice.clone());
        let binding = binding(alice.clone(), 0x61);
        let request = request(project_a, &epoch_a, alice.clone(), &binding);
        let observation = observation(&request);
        let epoch_b = epoch(project_b, alice);

        assert_eq!(
            bind_principal_authentication(request, binding, &epoch_b, observation).unwrap_err(),
            AuthenticationError::ProjectMismatch
        );
    }

    #[test]
    fn authentication_does_not_imply_authorization() {
        let project = project();
        let alice = principal(0x60);
        let outsider = principal(0x70);
        let epoch = epoch(project.clone(), alice);
        let binding = binding(outsider.clone(), 0x71);
        let request = request(project, &epoch, outsider.clone(), &binding);
        let observation = observation(&request);

        let qualified = bind_principal_authentication(request, binding, &epoch, observation).unwrap();
        assert_eq!(qualified.request().principal(), &outsider);
        assert!(!epoch.is_principal_eligible(&outsider, Capability::ReviewSource, 101));
    }

    #[test]
    fn serde_round_trip_preserves_raw_subjects_not_positive_claims() {
        let project = project();
        let alice = principal(0x60);
        let epoch = epoch(project.clone(), alice.clone());
        let binding = binding(alice.clone(), 0x61);
        let request = request(project, &epoch, alice, &binding);
        let observation = observation(&request);

        let encoded_binding = serde_json::to_string(&binding).unwrap();
        let decoded_binding: PrincipalBinding = serde_json::from_str(&encoded_binding).unwrap();
        assert_eq!(decoded_binding, binding);

        let encoded_request = serde_json::to_string(&request).unwrap();
        let decoded_request: PrincipalAuthenticationRequest =
            serde_json::from_str(&encoded_request).unwrap();
        assert_eq!(decoded_request, request);

        let encoded_observation = serde_json::to_string(&observation).unwrap();
        let decoded_observation: AuthenticationObservation =
            serde_json::from_str(&encoded_observation).unwrap();
        assert_eq!(decoded_observation, observation);
    }
}
