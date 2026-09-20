// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Typed project-policy state for Mycelix Forge.
//!
//! Forge proposals already bind an exact project-policy commitment. This crate
//! makes the authentication-provider trust portion of that commitment typed
//! and portable rather than leaving verifier trust as process-local
//! configuration.
//!
//! The authority split is deliberate:
//!
//! ```text
//! provider signature verifies
//!     != verifier trusted by this project
//!     != reviewer eligible
//!     != review quorum
//!     != merge authorization
//! ```
//!
//! [`AuthenticationProviderTrustPolicyV1`] answers only which exact provider
//! verifier identities the project recognizes. [`ProjectPolicyStateV1`] binds
//! that trust policy into a version-linked project-policy state suitable for
//! commitment by an immutable change proposal.

#![forbid(unsafe_code)]
#![warn(missing_docs)]

use mycelix_forge_core::{
    Digest, DigestAlgorithm, ProjectIdentity, ProtocolVersion, CURRENT_PROTOCOL_VERSION,
};
use serde::{de::Error as _, Deserialize, Deserializer, Serialize};
use thiserror::Error;

const PROVIDER_TRUST_DOMAIN_V1: &[u8] = b"mycelix-forge/authentication-provider-trust/v1\0";
const PROJECT_POLICY_DOMAIN_V1: &[u8] = b"mycelix-forge/project-policy-state/v1\0";
const MAX_TRUSTED_VERIFIERS_V1: usize = 1024;

/// One exact authentication-provider verifier identity recognized by project
/// policy.
///
/// `provider_namespace` identifies the provider contract (for example the
/// Xenia Forge authentication namespace); `verifier_identity` identifies the
/// exact provider signing identity inside that namespace.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct TrustedProviderVerifierV1 {
    provider_namespace: Digest,
    verifier_identity: Digest,
}

impl TrustedProviderVerifierV1 {
    /// Construct one exact provider/verifier trust entry.
    pub fn new(provider_namespace: Digest, verifier_identity: Digest) -> Self {
        Self {
            provider_namespace,
            verifier_identity,
        }
    }

    /// Provider contract namespace commitment.
    pub fn provider_namespace(&self) -> &Digest {
        &self.provider_namespace
    }

    /// Exact verifier identity commitment trusted inside that provider.
    pub fn verifier_identity(&self) -> &Digest {
        &self.verifier_identity
    }
}

/// Typed set of authentication-provider verifier identities recognized by one
/// project.
///
/// Public construction accepts arbitrary input order and canonicalizes it.
/// Wire deserialization is stricter: entries must already be in strictly
/// increasing canonical order, so two encodings cannot silently normalize to
/// the same policy after acceptance.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct AuthenticationProviderTrustPolicyV1 {
    version: ProtocolVersion,
    project: ProjectIdentity,
    trusted_verifiers: Vec<TrustedProviderVerifierV1>,
}

impl AuthenticationProviderTrustPolicyV1 {
    /// Construct and canonicalize a project provider-trust policy.
    pub fn new(
        project: ProjectIdentity,
        mut trusted_verifiers: Vec<TrustedProviderVerifierV1>,
    ) -> Result<Self, ProjectPolicyError> {
        trusted_verifiers.sort();
        Self::from_canonical_parts(project, trusted_verifiers)
    }

    fn from_canonical_parts(
        project: ProjectIdentity,
        trusted_verifiers: Vec<TrustedProviderVerifierV1>,
    ) -> Result<Self, ProjectPolicyError> {
        if trusted_verifiers.is_empty() {
            return Err(ProjectPolicyError::NoTrustedProviderVerifiers);
        }
        if trusted_verifiers.len() > MAX_TRUSTED_VERIFIERS_V1 {
            return Err(ProjectPolicyError::TooManyTrustedProviderVerifiers {
                actual: trusted_verifiers.len(),
                max: MAX_TRUSTED_VERIFIERS_V1,
            });
        }
        if trusted_verifiers
            .windows(2)
            .any(|pair| pair[0] >= pair[1])
        {
            return Err(ProjectPolicyError::NonCanonicalTrustedProviderVerifiers);
        }

        Ok(Self {
            version: ProtocolVersion::CURRENT,
            project,
            trusted_verifiers,
        })
    }

    /// Project whose provider trust this policy defines.
    pub fn project(&self) -> &ProjectIdentity {
        &self.project
    }

    /// Canonically sorted exact provider/verifier trust entries.
    pub fn trusted_verifiers(&self) -> &[TrustedProviderVerifierV1] {
        &self.trusted_verifiers
    }

    /// Whether this exact provider namespace/verifier identity pair is trusted.
    pub fn trusts(&self, provider_namespace: &Digest, verifier_identity: &Digest) -> bool {
        self.trusted_verifiers
            .binary_search_by(|entry| {
                entry
                    .provider_namespace
                    .cmp(provider_namespace)
                    .then_with(|| entry.verifier_identity.cmp(verifier_identity))
            })
            .is_ok()
    }

    /// Canonical v1 policy bytes.
    pub fn canonical_bytes(&self) -> Result<Vec<u8>, ProjectPolicyError> {
        ensure_v1(self.version)?;
        let count = u16::try_from(self.trusted_verifiers.len()).map_err(|_| {
            ProjectPolicyError::CanonicalFieldTooLarge {
                field: "trusted_verifiers",
                len: self.trusted_verifiers.len(),
                max: u16::MAX as usize,
            }
        })?;

        let mut out = Vec::new();
        out.extend_from_slice(PROVIDER_TRUST_DOMAIN_V1);
        out.extend_from_slice(&self.version.get().to_be_bytes());
        push_project_identity(&mut out, &self.project)?;
        out.extend_from_slice(&count.to_be_bytes());
        for entry in &self.trusted_verifiers {
            push_digest(&mut out, &entry.provider_namespace)?;
            push_digest(&mut out, &entry.verifier_identity)?;
        }
        Ok(out)
    }

    /// Stable commitment to this exact provider-trust policy.
    pub fn digest(&self, algorithm: DigestAlgorithm) -> Result<Digest, ProjectPolicyError> {
        Ok(Digest::of_bytes(algorithm, &self.canonical_bytes()?))
    }
}

impl<'de> Deserialize<'de> for AuthenticationProviderTrustPolicyV1 {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct WirePolicy {
            version: ProtocolVersion,
            project: ProjectIdentity,
            trusted_verifiers: Vec<TrustedProviderVerifierV1>,
        }

        let wire = WirePolicy::deserialize(deserializer)?;
        ensure_v1(wire.version).map_err(D::Error::custom)?;
        Self::from_canonical_parts(wire.project, wire.trusted_verifiers).map_err(D::Error::custom)
    }
}

/// Version-linked project-policy state committed by Forge proposals.
///
/// Protocol v1 requires an exact typed authentication-provider trust policy.
/// Additional project-policy dimensions can be introduced by a later protocol
/// version without reinterpreting this v1 commitment.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct ProjectPolicyStateV1 {
    version: ProtocolVersion,
    project: ProjectIdentity,
    sequence: u64,
    previous: Option<Digest>,
    authentication_provider_trust: Digest,
}

impl ProjectPolicyStateV1 {
    /// Construct a typed v1 project-policy state from the exact provider trust
    /// policy it commits.
    pub fn new(
        project: ProjectIdentity,
        sequence: u64,
        previous: Option<Digest>,
        authentication_provider_trust: &AuthenticationProviderTrustPolicyV1,
        commitment_algorithm: DigestAlgorithm,
    ) -> Result<Self, ProjectPolicyError> {
        if authentication_provider_trust.project() != &project {
            return Err(ProjectPolicyError::ProviderTrustProjectMismatch);
        }
        validate_lineage(sequence, previous.as_ref())?;

        Ok(Self {
            version: ProtocolVersion::CURRENT,
            project,
            sequence,
            previous,
            authentication_provider_trust: authentication_provider_trust
                .digest(commitment_algorithm)?,
        })
    }

    fn from_parts(
        project: ProjectIdentity,
        sequence: u64,
        previous: Option<Digest>,
        authentication_provider_trust: Digest,
    ) -> Result<Self, ProjectPolicyError> {
        validate_lineage(sequence, previous.as_ref())?;
        Ok(Self {
            version: ProtocolVersion::CURRENT,
            project,
            sequence,
            previous,
            authentication_provider_trust,
        })
    }

    /// Project governed by this policy state.
    pub fn project(&self) -> &ProjectIdentity {
        &self.project
    }

    /// Monotonic project-policy sequence.
    pub const fn sequence(&self) -> u64 {
        self.sequence
    }

    /// Previous project-policy-state commitment, absent only for sequence 0.
    pub fn previous(&self) -> Option<&Digest> {
        self.previous.as_ref()
    }

    /// Exact typed authentication-provider trust-policy commitment.
    pub fn authentication_provider_trust(&self) -> &Digest {
        &self.authentication_provider_trust
    }

    /// Verify that this project-policy state contains the supplied exact typed
    /// provider-trust policy.
    pub fn binds_provider_trust(
        &self,
        policy: &AuthenticationProviderTrustPolicyV1,
    ) -> Result<bool, ProjectPolicyError> {
        if policy.project() != &self.project {
            return Ok(false);
        }
        Ok(policy.digest(self.authentication_provider_trust.algorithm())?
            == self.authentication_provider_trust)
    }

    /// Canonical v1 project-policy-state bytes.
    pub fn canonical_bytes(&self) -> Result<Vec<u8>, ProjectPolicyError> {
        ensure_v1(self.version)?;
        validate_lineage(self.sequence, self.previous.as_ref())?;

        let mut out = Vec::new();
        out.extend_from_slice(PROJECT_POLICY_DOMAIN_V1);
        out.extend_from_slice(&self.version.get().to_be_bytes());
        push_project_identity(&mut out, &self.project)?;
        out.extend_from_slice(&self.sequence.to_be_bytes());
        push_optional_digest(&mut out, self.previous.as_ref())?;
        push_digest(&mut out, &self.authentication_provider_trust)?;
        Ok(out)
    }

    /// Stable commitment to this exact project-policy state.
    pub fn digest(&self, algorithm: DigestAlgorithm) -> Result<Digest, ProjectPolicyError> {
        Ok(Digest::of_bytes(algorithm, &self.canonical_bytes()?))
    }
}

impl<'de> Deserialize<'de> for ProjectPolicyStateV1 {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct WireState {
            version: ProtocolVersion,
            project: ProjectIdentity,
            sequence: u64,
            previous: Option<Digest>,
            authentication_provider_trust: Digest,
        }

        let wire = WireState::deserialize(deserializer)?;
        ensure_v1(wire.version).map_err(D::Error::custom)?;
        Self::from_parts(
            wire.project,
            wire.sequence,
            wire.previous,
            wire.authentication_provider_trust,
        )
        .map_err(D::Error::custom)
    }
}

fn validate_lineage(sequence: u64, previous: Option<&Digest>) -> Result<(), ProjectPolicyError> {
    match (sequence, previous.is_some()) {
        (0, false) => Ok(()),
        (0, true) => Err(ProjectPolicyError::GenesisHasPrevious),
        (_, false) => Err(ProjectPolicyError::NonGenesisMissingPrevious),
        (_, true) => Ok(()),
    }
}

fn ensure_v1(version: ProtocolVersion) -> Result<(), ProjectPolicyError> {
    if version.get() == CURRENT_PROTOCOL_VERSION {
        Ok(())
    } else {
        Err(ProjectPolicyError::UnsupportedProtocolVersion(version.get()))
    }
}

fn push_project_identity(
    out: &mut Vec<u8>,
    project: &ProjectIdentity,
) -> Result<(), ProjectPolicyError> {
    out.extend_from_slice(&project.version().get().to_be_bytes());
    push_digest(out, project.digest())
}

fn push_optional_digest(
    out: &mut Vec<u8>,
    digest: Option<&Digest>,
) -> Result<(), ProjectPolicyError> {
    match digest {
        None => out.push(0),
        Some(digest) => {
            out.push(1);
            push_digest(out, digest)?;
        }
    }
    Ok(())
}

fn push_digest(out: &mut Vec<u8>, digest: &Digest) -> Result<(), ProjectPolicyError> {
    push_bytes(out, "digest_algorithm", digest.algorithm().id().as_bytes())?;
    push_bytes(out, "digest", digest.as_bytes())
}

fn push_bytes(
    out: &mut Vec<u8>,
    field: &'static str,
    bytes: &[u8],
) -> Result<(), ProjectPolicyError> {
    let len = u32::try_from(bytes.len()).map_err(|_| ProjectPolicyError::CanonicalFieldTooLarge {
        field,
        len: bytes.len(),
        max: u32::MAX as usize,
    })?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(bytes);
    Ok(())
}

/// Typed project-policy validation/canonicalization failure.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum ProjectPolicyError {
    /// Provider trust policy cannot be empty.
    #[error("authentication-provider trust policy has no trusted verifier identities")]
    NoTrustedProviderVerifiers,
    /// Provider trust policy exceeded the v1 entry bound.
    #[error("too many trusted provider verifiers: {actual} > {max}")]
    TooManyTrustedProviderVerifiers {
        /// Supplied number of entries.
        actual: usize,
        /// Maximum v1 entries.
        max: usize,
    },
    /// Serialized provider trust entries were duplicated or not canonically
    /// sorted.
    #[error("trusted provider verifier entries are not in canonical strict order")]
    NonCanonicalTrustedProviderVerifiers,
    /// A typed provider-trust policy belongs to another project.
    #[error("authentication-provider trust policy belongs to another project")]
    ProviderTrustProjectMismatch,
    /// Project-policy sequence zero cannot name a previous state.
    #[error("genesis project-policy state cannot name a previous state")]
    GenesisHasPrevious,
    /// Non-genesis project-policy state must link its predecessor.
    #[error("non-genesis project-policy state is missing its previous commitment")]
    NonGenesisMissingPrevious,
    /// Unsupported Forge protocol version.
    #[error("unsupported Forge project-policy protocol version: {0}")]
    UnsupportedProtocolVersion(u16),
    /// Canonical field exceeded the v1 encoding bound.
    #[error("canonical field {field} is too large: {len} > {max}")]
    CanonicalFieldTooLarge {
        /// Field name.
        field: &'static str,
        /// Observed length.
        len: usize,
        /// Maximum length.
        max: usize,
    },
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_forge_core::{ProjectIdentitySeed, GENESIS_NONCE_LEN};

    fn digest(byte: u8) -> Digest {
        Digest::new(DigestAlgorithm::Sha256, vec![byte; 32]).unwrap()
    }

    fn project(seed: u8) -> ProjectIdentity {
        ProjectIdentity::derive(
            &ProjectIdentitySeed::new([seed; GENESIS_NONCE_LEN], digest(seed.wrapping_add(1))),
            DigestAlgorithm::Sha256,
        )
        .unwrap()
    }

    fn entry(provider: u8, verifier: u8) -> TrustedProviderVerifierV1 {
        TrustedProviderVerifierV1::new(digest(provider), digest(verifier))
    }

    #[test]
    fn public_construction_canonicalizes_trust_order() {
        let project = project(0x10);
        let policy = AuthenticationProviderTrustPolicyV1::new(
            project,
            vec![entry(0x30, 0x40), entry(0x20, 0x50), entry(0x20, 0x40)],
        )
        .unwrap();
        assert_eq!(policy.trusted_verifiers()[0], entry(0x20, 0x40));
        assert_eq!(policy.trusted_verifiers()[1], entry(0x20, 0x50));
        assert_eq!(policy.trusted_verifiers()[2], entry(0x30, 0x40));
    }

    #[test]
    fn duplicate_provider_verifier_pair_is_rejected() {
        let project = project(0x10);
        assert_eq!(
            AuthenticationProviderTrustPolicyV1::new(
                project,
                vec![entry(0x20, 0x40), entry(0x20, 0x40)],
            )
            .unwrap_err(),
            ProjectPolicyError::NonCanonicalTrustedProviderVerifiers
        );
    }

    #[test]
    fn trust_is_exact_in_both_namespace_and_verifier_identity() {
        let project = project(0x10);
        let policy = AuthenticationProviderTrustPolicyV1::new(
            project,
            vec![entry(0x20, 0x40)],
        )
        .unwrap();
        assert!(policy.trusts(&digest(0x20), &digest(0x40)));
        assert!(!policy.trusts(&digest(0x21), &digest(0x40)));
        assert!(!policy.trusts(&digest(0x20), &digest(0x41)));
    }

    #[test]
    fn project_policy_commitment_changes_when_verifier_trust_changes() {
        let project = project(0x10);
        let a = AuthenticationProviderTrustPolicyV1::new(
            project.clone(),
            vec![entry(0x20, 0x40)],
        )
        .unwrap();
        let b = AuthenticationProviderTrustPolicyV1::new(
            project.clone(),
            vec![entry(0x20, 0x41)],
        )
        .unwrap();
        let a_state = ProjectPolicyStateV1::new(
            project.clone(),
            0,
            None,
            &a,
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        let b_state = ProjectPolicyStateV1::new(
            project,
            0,
            None,
            &b,
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        assert_ne!(
            a_state.digest(DigestAlgorithm::Sha256).unwrap(),
            b_state.digest(DigestAlgorithm::Sha256).unwrap()
        );
    }

    #[test]
    fn project_policy_refuses_cross_project_provider_trust() {
        let a = project(0x10);
        let b = project(0x20);
        let trust = AuthenticationProviderTrustPolicyV1::new(
            b,
            vec![entry(0x30, 0x40)],
        )
        .unwrap();
        assert_eq!(
            ProjectPolicyStateV1::new(
                a,
                0,
                None,
                &trust,
                DigestAlgorithm::Sha256,
            )
            .unwrap_err(),
            ProjectPolicyError::ProviderTrustProjectMismatch
        );
    }

    #[test]
    fn non_genesis_policy_state_requires_previous_commitment() {
        let project = project(0x10);
        let trust = AuthenticationProviderTrustPolicyV1::new(
            project.clone(),
            vec![entry(0x30, 0x40)],
        )
        .unwrap();
        assert_eq!(
            ProjectPolicyStateV1::new(
                project,
                1,
                None,
                &trust,
                DigestAlgorithm::Sha256,
            )
            .unwrap_err(),
            ProjectPolicyError::NonGenesisMissingPrevious
        );
    }

    #[test]
    fn serde_round_trip_preserves_exact_policy_identity() {
        let project = project(0x10);
        let trust = AuthenticationProviderTrustPolicyV1::new(
            project.clone(),
            vec![entry(0x20, 0x40), entry(0x20, 0x41)],
        )
        .unwrap();
        let state = ProjectPolicyStateV1::new(
            project,
            0,
            None,
            &trust,
            DigestAlgorithm::Sha256,
        )
        .unwrap();

        let trust_json = serde_json::to_vec(&trust).unwrap();
        let trust_decoded: AuthenticationProviderTrustPolicyV1 =
            serde_json::from_slice(&trust_json).unwrap();
        assert_eq!(trust_decoded, trust);

        let state_json = serde_json::to_vec(&state).unwrap();
        let state_decoded: ProjectPolicyStateV1 = serde_json::from_slice(&state_json).unwrap();
        assert_eq!(state_decoded, state);
        assert_eq!(
            state_decoded.digest(DigestAlgorithm::Sha256).unwrap(),
            state.digest(DigestAlgorithm::Sha256).unwrap()
        );
    }

    #[test]
    fn noncanonical_wire_trust_order_is_rejected() {
        #[derive(Serialize)]
        struct WirePolicy {
            version: ProtocolVersion,
            project: ProjectIdentity,
            trusted_verifiers: Vec<TrustedProviderVerifierV1>,
        }

        let wire = WirePolicy {
            version: ProtocolVersion::CURRENT,
            project: project(0x10),
            trusted_verifiers: vec![entry(0x30, 0x40), entry(0x20, 0x40)],
        };
        let bytes = serde_json::to_vec(&wire).unwrap();
        assert!(serde_json::from_slice::<AuthenticationProviderTrustPolicyV1>(&bytes).is_err());
    }
}
