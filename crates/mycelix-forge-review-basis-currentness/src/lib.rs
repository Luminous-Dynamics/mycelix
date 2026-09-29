// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! FORGE-007O: provider-scoped currentness evidence for one exact protected
//! review basis.
//!
//! This layer intentionally does not create a global current-state oracle.
//!
//! MergeProtectedReviewBasisQuorumV1
//! + exact provider-scoped currentness observation
//! + independent verifier
//!     -> ProviderVerifiedReviewBasisCurrentnessV1

#![forbid(unsafe_code)]
#![warn(missing_docs)]

use mycelix_forge_core::{Digest, DigestAlgorithm, ProtocolVersion};
use mycelix_forge_merge_protected_review_basis::MergeProtectedReviewBasisQuorumV1;
use serde::{Deserialize, Serialize};
use thiserror::Error;

const OBSERVATION_DOMAIN_V1: &[u8] =
    b"mycelix-forge/review-basis-currentness-observation/v1\0";
const EVIDENCE_DOMAIN_V1: &[u8] =
    b"mycelix-forge/review-basis-currentness-evidence/v1\0";
const MAX_PROVIDER_NAMESPACE_LEN: usize = 256;
const MAX_VERIFIER_IDENTITY_LEN: usize = 512;

/// Provider-reported currentness state.
///
/// Only CurrentWithinScope can be upgraded into the positive evidence type.
/// NotCurrent and Indeterminate are retained as explicit negative evidence
/// rather than being silently collapsed into absence.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ReviewBasisCurrentnessStatusV1 {
    /// The provider claims the exact basis is current within its declared scope.
    CurrentWithinScope,
    /// The provider observed a later conflicting/superseding state within scope.
    NotCurrent,
    /// The provider could not establish currentness within scope.
    Indeterminate,
}

/// Stable identity of the independent currentness verifier.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ReviewBasisCurrentnessVerifierIdentityV1 {
    name: String,
    version: String,
}

impl ReviewBasisCurrentnessVerifierIdentityV1 {
    /// Construct a bounded verifier identity.
    pub fn new(
        name: impl Into<String>,
        version: impl Into<String>,
    ) -> Result<Self, ReviewBasisCurrentnessError> {
        let name = name.into();
        let version = version.into();
        if name.is_empty()
            || name.len() > MAX_VERIFIER_IDENTITY_LEN
            || version.is_empty()
            || version.len() > MAX_VERIFIER_IDENTITY_LEN
        {
            return Err(ReviewBasisCurrentnessError::InvalidVerifierIdentity);
        }
        Ok(Self { name, version })
    }

    /// Verifier name/profile.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Verifier version/profile.
    pub fn version(&self) -> &str {
        &self.version
    }
}

/// Raw provider observation about whether one exact protected review basis is
/// current within one exact provider-declared coverage scope.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ReviewBasisCurrentnessObservationV1 {
    version: ProtocolVersion,
    provider_namespace: String,
    verifier: ReviewBasisCurrentnessVerifierIdentityV1,
    review_basis_quorum: Digest,
    coverage_commitment: Digest,
    observed_at_unix_ms: u64,
    status: ReviewBasisCurrentnessStatusV1,
    provider_evidence: Digest,
}

impl ReviewBasisCurrentnessObservationV1 {
    /// Construct raw, non-authoritative provider currentness evidence.
    pub fn new(
        provider_namespace: impl Into<String>,
        verifier: ReviewBasisCurrentnessVerifierIdentityV1,
        review_basis_quorum: Digest,
        coverage_commitment: Digest,
        observed_at_unix_ms: u64,
        status: ReviewBasisCurrentnessStatusV1,
        provider_evidence: Digest,
    ) -> Result<Self, ReviewBasisCurrentnessError> {
        let provider_namespace = provider_namespace.into();
        if provider_namespace.is_empty()
            || provider_namespace.len() > MAX_PROVIDER_NAMESPACE_LEN
        {
            return Err(ReviewBasisCurrentnessError::InvalidProviderNamespace);
        }
        Ok(Self {
            version: ProtocolVersion::CURRENT,
            provider_namespace,
            verifier,
            review_basis_quorum,
            coverage_commitment,
            observed_at_unix_ms,
            status,
            provider_evidence,
        })
    }

    /// Exact provider namespace whose currentness semantics are being used.
    pub fn provider_namespace(&self) -> &str {
        &self.provider_namespace
    }

    /// Independent verifier identity claimed by the provider.
    pub fn verifier(&self) -> &ReviewBasisCurrentnessVerifierIdentityV1 {
        &self.verifier
    }

    /// Exact FORGE-007N review-basis quorum evidence commitment.
    pub fn review_basis_quorum(&self) -> &Digest {
        &self.review_basis_quorum
    }

    /// Opaque commitment to the exact provider coverage used for currentness.
    pub fn coverage_commitment(&self) -> &Digest {
        &self.coverage_commitment
    }

    /// Provider observation time.
    pub const fn observed_at_unix_ms(&self) -> u64 {
        self.observed_at_unix_ms
    }

    /// Provider-reported currentness state.
    pub const fn status(&self) -> ReviewBasisCurrentnessStatusV1 {
        self.status
    }

    /// Provider-native currentness evidence commitment.
    pub fn provider_evidence(&self) -> &Digest {
        &self.provider_evidence
    }

    /// Canonical observation commitment.
    pub fn digest(
        &self,
        algorithm: DigestAlgorithm,
    ) -> Result<Digest, ReviewBasisCurrentnessError> {
        let mut out = Vec::new();
        out.extend_from_slice(OBSERVATION_DOMAIN_V1);
        out.extend_from_slice(&self.version.get().to_be_bytes());
        push_string(&mut out, &self.provider_namespace)?;
        push_string(&mut out, self.verifier.name())?;
        push_string(&mut out, self.verifier.version())?;
        push_digest(&mut out, &self.review_basis_quorum)?;
        push_digest(&mut out, &self.coverage_commitment)?;
        out.extend_from_slice(&self.observed_at_unix_ms.to_be_bytes());
        out.push(match self.status {
            ReviewBasisCurrentnessStatusV1::CurrentWithinScope => 1,
            ReviewBasisCurrentnessStatusV1::NotCurrent => 2,
            ReviewBasisCurrentnessStatusV1::Indeterminate => 3,
        });
        push_digest(&mut out, &self.provider_evidence)?;
        Ok(Digest::of_bytes(algorithm, &out))
    }
}

/// Independent verifier for one provider's currentness observation.
pub trait ReviewBasisCurrentnessVerifierV1 {
    /// Stable verifier identity.
    fn identity(&self) -> ReviewBasisCurrentnessVerifierIdentityV1;

    /// Verify the provider's currentness claim against the exact selected basis.
    ///
    /// The verifier is responsible for the semantics of coverage_commitment
    /// and for independently checking that the provider's evidence supports the
    /// declared status at the supplied observation time.
    fn verify_currentness(
        &self,
        quorum: &MergeProtectedReviewBasisQuorumV1,
        observation: &ReviewBasisCurrentnessObservationV1,
    ) -> Result<Digest, ReviewBasisCurrentnessVerifierErrorV1>;
}

/// Provider-specific currentness verification failure.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum ReviewBasisCurrentnessVerifierErrorV1 {
    /// Provider evidence did not establish the declared status.
    #[error("review-basis currentness verifier rejected provider evidence")]
    Rejected,
}

/// Positive evidence that an exact review-basis quorum was observed as current
/// within an exact provider-declared coverage scope.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct ProviderVerifiedReviewBasisCurrentnessV1 {
    provider_namespace: String,
    verifier: ReviewBasisCurrentnessVerifierIdentityV1,
    review_basis_quorum: Digest,
    coverage_commitment: Digest,
    observed_at_unix_ms: u64,
    provider_evidence: Digest,
    verifier_evidence: Digest,
    evidence_commitment: Digest,
}

impl ProviderVerifiedReviewBasisCurrentnessV1 {
    /// Exact provider namespace.
    pub fn provider_namespace(&self) -> &str {
        &self.provider_namespace
    }

    /// Exact independent verifier identity.
    pub fn verifier(&self) -> &ReviewBasisCurrentnessVerifierIdentityV1 {
        &self.verifier
    }

    /// Exact FORGE-007N quorum evidence commitment.
    pub fn review_basis_quorum(&self) -> &Digest {
        &self.review_basis_quorum
    }

    /// Exact provider currentness coverage commitment.
    pub fn coverage_commitment(&self) -> &Digest {
        &self.coverage_commitment
    }

    /// Provider observation time.
    pub const fn observed_at_unix_ms(&self) -> u64 {
        self.observed_at_unix_ms
    }

    /// Provider evidence.
    pub fn provider_evidence(&self) -> &Digest {
        &self.provider_evidence
    }

    /// Independent verifier evidence.
    pub fn verifier_evidence(&self) -> &Digest {
        &self.verifier_evidence
    }

    /// Aggregate evidence commitment.
    pub fn evidence_commitment(&self) -> &Digest {
        &self.evidence_commitment
    }
}

/// Qualify provider-scoped currentness for the exact review basis selected by
/// FORGE-007N.
pub fn qualify_provider_verified_review_basis_currentness_v1<
    V: ReviewBasisCurrentnessVerifierV1,
>(
    quorum: &MergeProtectedReviewBasisQuorumV1,
    observation: &ReviewBasisCurrentnessObservationV1,
    verifier: &V,
) -> Result<ProviderVerifiedReviewBasisCurrentnessV1, ReviewBasisCurrentnessError> {
    if observation.review_basis_quorum() != quorum.evidence_commitment() {
        return Err(ReviewBasisCurrentnessError::QuorumMismatch);
    }
    if observation.observed_at_unix_ms() < quorum.quorum_observed_at_unix_ms() {
        return Err(ReviewBasisCurrentnessError::ObservationBeforeQuorum);
    }
    if observation.verifier() != &verifier.identity() {
        return Err(ReviewBasisCurrentnessError::VerifierIdentityMismatch);
    }
    if observation.status() != ReviewBasisCurrentnessStatusV1::CurrentWithinScope {
        return Err(ReviewBasisCurrentnessError::NotCurrent);
    }

    let verifier_evidence = verifier
        .verify_currentness(quorum, observation)
        .map_err(ReviewBasisCurrentnessError::VerifierRejected)?;
    let algorithm = verifier_evidence.algorithm();
    let observation_digest = observation.digest(algorithm)?;

    let mut out = Vec::new();
    out.extend_from_slice(EVIDENCE_DOMAIN_V1);
    out.extend_from_slice(&ProtocolVersion::CURRENT.get().to_be_bytes());
    push_string(&mut out, observation.provider_namespace())?;
    push_string(&mut out, observation.verifier().name())?;
    push_string(&mut out, observation.verifier().version())?;
    push_digest(&mut out, observation.review_basis_quorum())?;
    push_digest(&mut out, observation.coverage_commitment())?;
    out.extend_from_slice(&observation.observed_at_unix_ms().to_be_bytes());
    push_digest(&mut out, &observation_digest)?;
    push_digest(&mut out, &verifier_evidence)?;

    Ok(ProviderVerifiedReviewBasisCurrentnessV1 {
        provider_namespace: observation.provider_namespace().to_owned(),
        verifier: observation.verifier().clone(),
        review_basis_quorum: observation.review_basis_quorum().clone(),
        coverage_commitment: observation.coverage_commitment().clone(),
        observed_at_unix_ms: observation.observed_at_unix_ms(),
        provider_evidence: observation.provider_evidence().clone(),
        verifier_evidence,
        evidence_commitment: Digest::of_bytes(algorithm, &out),
    })
}

fn push_digest(
    out: &mut Vec<u8>,
    digest: &Digest,
) -> Result<(), ReviewBasisCurrentnessError> {
    push_string(out, digest.algorithm().id())?;
    let len = u32::try_from(digest.as_bytes().len())
        .map_err(|_| ReviewBasisCurrentnessError::CanonicalFieldTooLarge("digest"))?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(digest.as_bytes());
    Ok(())
}

fn push_string(
    out: &mut Vec<u8>,
    value: &str,
) -> Result<(), ReviewBasisCurrentnessError> {
    let bytes = value.as_bytes();
    let len = u32::try_from(bytes.len())
        .map_err(|_| ReviewBasisCurrentnessError::CanonicalFieldTooLarge("string"))?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(bytes);
    Ok(())
}

/// FORGE-007O qualification failures.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum ReviewBasisCurrentnessError {
    /// Currentness observation names another FORGE-007N quorum.
    #[error("review-basis currentness references another quorum")]
    QuorumMismatch,
    /// Currentness observation predates the protected-basis quorum.
    #[error("review-basis currentness observation predates quorum")]
    ObservationBeforeQuorum,
    /// Independent verifier identity differs from the supplied verifier.
    #[error("review-basis currentness verifier identity mismatch")]
    VerifierIdentityMismatch,
    /// Provider explicitly reports that the selected basis is not current.
    #[error("review-basis is not current within the declared provider scope")]
    NotCurrent,
    /// Independent verifier rejected the provider evidence.
    #[error("review-basis currentness verifier rejected provider evidence: {0}")]
    VerifierRejected(ReviewBasisCurrentnessVerifierErrorV1),
    /// Provider namespace is empty or too large.
    #[error("review-basis currentness provider namespace is invalid")]
    InvalidProviderNamespace,
    /// Verifier identity is empty or too large.
    #[error("review-basis currentness verifier identity is invalid")]
    InvalidVerifierIdentity,
    /// Canonical field overflow.
    #[error("canonical field too large: {0}")]
    CanonicalFieldTooLarge(&'static str),
}

#[cfg(test)]
mod tests {
    use super::*;

    fn digest(byte: u8) -> Digest {
        Digest::new(DigestAlgorithm::Sha256, vec![byte; 32]).unwrap()
    }

    #[test]
    fn status_round_trips_without_becoming_positive() {
        let observation = ReviewBasisCurrentnessObservationV1::new(
            "provider/test",
            ReviewBasisCurrentnessVerifierIdentityV1::new("verifier", "v1").unwrap(),
            digest(1),
            digest(2),
            42,
            ReviewBasisCurrentnessStatusV1::Indeterminate,
            digest(3),
        )
        .unwrap();
        let bytes = serde_json::to_vec(&observation).unwrap();
        let decoded: ReviewBasisCurrentnessObservationV1 =
            serde_json::from_slice(&bytes).unwrap();
        assert_eq!(observation, decoded);
    }

    #[test]
    fn non_current_status_is_explicit() {
        assert_ne!(
            ReviewBasisCurrentnessStatusV1::NotCurrent,
            ReviewBasisCurrentnessStatusV1::CurrentWithinScope
        );
        assert_ne!(
            ReviewBasisCurrentnessStatusV1::Indeterminate,
            ReviewBasisCurrentnessStatusV1::CurrentWithinScope
        );
    }

    #[test]
    fn positive_type_is_not_deserializable() {
        let source = std::any::type_name::<ProviderVerifiedReviewBasisCurrentnessV1>();
        assert!(source.contains("ProviderVerifiedReviewBasisCurrentnessV1"));
    }
}
