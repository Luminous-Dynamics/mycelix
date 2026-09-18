// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Provider-neutral repository verification contracts for Mycelix Forge.
//!
//! This crate defines the exact subjects an external repository-security
//! adapter must verify. It intentionally does not depend on gittuf, GitHub,
//! GitLab, Radicle, Holochain, or a Git implementation. An adapter can map an
//! external policy/RSL system into these subjects without becoming the Forge
//! protocol's root of trust.

use mycelix_forge_core::{
    Digest, DigestAlgorithm, ProjectIdentity, ProtocolVersion, CURRENT_PROTOCOL_VERSION,
};
use serde::{de::Error as _, Deserialize, Deserializer, Serialize};
use std::{collections::BTreeSet, fmt};
use thiserror::Error;

const ADOPTION_DOMAIN_V1: &[u8] = b"mycelix-forge/repository-adoption/v1\0";
const POLICY_STATE_DOMAIN_V1: &[u8] = b"mycelix-forge/repository-policy-state/v1\0";
const VERIFICATION_REQUEST_DOMAIN_V1: &[u8] =
    b"mycelix-forge/repository-verification-request/v1\0";
const MAX_REF_LEN: usize = 1024;
const MAX_ADAPTER_FIELD_LEN: usize = 128;

/// Git object-ID algorithms understood by Forge repository protocol v1.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum GitObjectAlgorithm {
    #[serde(rename = "sha1")]
    Sha1,
    #[serde(rename = "sha256")]
    Sha256,
}

impl GitObjectAlgorithm {
    pub const fn code(self) -> u8 {
        match self {
            Self::Sha1 => 1,
            Self::Sha256 => 2,
        }
    }

    pub const fn digest_len(self) -> usize {
        match self {
            Self::Sha1 => 20,
            Self::Sha256 => 32,
        }
    }
}

/// Validated Git object identifier with an explicit object-format algorithm.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize)]
pub struct GitObjectId {
    algorithm: GitObjectAlgorithm,
    bytes: Vec<u8>,
}

impl GitObjectId {
    pub fn new(
        algorithm: GitObjectAlgorithm,
        bytes: Vec<u8>,
    ) -> Result<Self, RepositoryVerificationError> {
        let expected = algorithm.digest_len();
        let actual = bytes.len();
        if actual != expected {
            return Err(RepositoryVerificationError::InvalidGitObjectLength {
                algorithm,
                expected,
                actual,
            });
        }
        Ok(Self { algorithm, bytes })
    }

    pub const fn algorithm(&self) -> GitObjectAlgorithm {
        self.algorithm
    }

    pub fn as_bytes(&self) -> &[u8] {
        &self.bytes
    }
}

impl<'de> Deserialize<'de> for GitObjectId {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct WireGitObjectId {
            algorithm: GitObjectAlgorithm,
            bytes: Vec<u8>,
        }

        let wire = WireGitObjectId::deserialize(deserializer)?;
        Self::new(wire.algorithm, wire.bytes).map_err(D::Error::custom)
    }
}

/// Conservative fully-qualified Git reference name used as a protocol subject.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize)]
#[serde(transparent)]
pub struct RepositoryRef(String);

impl RepositoryRef {
    pub fn new(value: impl Into<String>) -> Result<Self, RepositoryVerificationError> {
        let value = value.into();
        validate_ref_name(&value)?;
        Ok(Self(value))
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }
}

impl fmt::Display for RepositoryRef {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

impl<'de> Deserialize<'de> for RepositoryRef {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        let value = String::deserialize(deserializer)?;
        Self::new(value).map_err(D::Error::custom)
    }
}

/// Exact state of one Git reference.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct RepositoryTip {
    reference: RepositoryRef,
    object: GitObjectId,
}

impl RepositoryTip {
    pub fn new(reference: RepositoryRef, object: GitObjectId) -> Self {
        Self { reference, object }
    }

    pub fn reference(&self) -> &RepositoryRef {
        &self.reference
    }

    pub fn object(&self) -> &GitObjectId {
        &self.object
    }
}

/// Exact point from which Forge verification begins for an existing repository.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct RepositoryAdoption {
    version: ProtocolVersion,
    project: ProjectIdentity,
    baseline: RepositoryTip,
    authority_epoch: Digest,
    project_policy: Digest,
    repository_policy: Digest,
    adopted_at_unix_ms: u64,
}

impl RepositoryAdoption {
    pub fn new(
        project: ProjectIdentity,
        baseline: RepositoryTip,
        authority_epoch: Digest,
        project_policy: Digest,
        repository_policy: Digest,
        adopted_at_unix_ms: u64,
    ) -> Self {
        Self {
            version: ProtocolVersion::CURRENT,
            project,
            baseline,
            authority_epoch,
            project_policy,
            repository_policy,
            adopted_at_unix_ms,
        }
    }

    pub fn project(&self) -> &ProjectIdentity {
        &self.project
    }

    pub fn baseline(&self) -> &RepositoryTip {
        &self.baseline
    }

    pub fn authority_epoch(&self) -> &Digest {
        &self.authority_epoch
    }

    pub fn project_policy(&self) -> &Digest {
        &self.project_policy
    }

    pub fn repository_policy(&self) -> &Digest {
        &self.repository_policy
    }

    pub const fn adopted_at_unix_ms(&self) -> u64 {
        self.adopted_at_unix_ms
    }

    pub fn canonical_bytes(&self) -> Result<Vec<u8>, RepositoryVerificationError> {
        ensure_current_version(self.version)?;
        let mut out = Vec::new();
        out.extend_from_slice(ADOPTION_DOMAIN_V1);
        out.extend_from_slice(&self.version.get().to_be_bytes());
        push_project_identity(&mut out, &self.project)?;
        push_tip(&mut out, &self.baseline)?;
        push_digest(&mut out, &self.authority_epoch)?;
        push_digest(&mut out, &self.project_policy)?;
        push_digest(&mut out, &self.repository_policy)?;
        out.extend_from_slice(&self.adopted_at_unix_ms.to_be_bytes());
        Ok(out)
    }

    pub fn digest(
        &self,
        algorithm: DigestAlgorithm,
    ) -> Result<Digest, RepositoryVerificationError> {
        Ok(Digest::of_bytes(algorithm, &self.canonical_bytes()?))
    }
}

impl<'de> Deserialize<'de> for RepositoryAdoption {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct WireAdoption {
            version: ProtocolVersion,
            project: ProjectIdentity,
            baseline: RepositoryTip,
            authority_epoch: Digest,
            project_policy: Digest,
            repository_policy: Digest,
            adopted_at_unix_ms: u64,
        }

        let wire = WireAdoption::deserialize(deserializer)?;
        ensure_current_version(wire.version).map_err(D::Error::custom)?;
        Ok(Self::new(
            wire.project,
            wire.baseline,
            wire.authority_epoch,
            wire.project_policy,
            wire.repository_policy,
            wire.adopted_at_unix_ms,
        ))
    }
}

/// Project-bound monotonic state of an external repository policy system.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct RepositoryPolicyState {
    version: ProtocolVersion,
    project: ProjectIdentity,
    sequence: u64,
    previous: Option<Digest>,
    policy_digest: Digest,
}

impl RepositoryPolicyState {
    pub fn new(
        project: ProjectIdentity,
        sequence: u64,
        previous: Option<Digest>,
        policy_digest: Digest,
    ) -> Result<Self, RepositoryVerificationError> {
        match (sequence, previous.is_some()) {
            (0, false) => {}
            (0, true) => return Err(RepositoryVerificationError::GenesisPolicyHasPrevious),
            (_, true) => {}
            (_, false) => return Err(RepositoryVerificationError::PolicyMissingPrevious),
        }

        Ok(Self {
            version: ProtocolVersion::CURRENT,
            project,
            sequence,
            previous,
            policy_digest,
        })
    }

    pub fn project(&self) -> &ProjectIdentity {
        &self.project
    }

    pub const fn sequence(&self) -> u64 {
        self.sequence
    }

    pub fn previous(&self) -> Option<&Digest> {
        self.previous.as_ref()
    }

    pub fn policy_digest(&self) -> &Digest {
        &self.policy_digest
    }

    pub fn canonical_bytes(&self) -> Result<Vec<u8>, RepositoryVerificationError> {
        ensure_current_version(self.version)?;
        let mut out = Vec::new();
        out.extend_from_slice(POLICY_STATE_DOMAIN_V1);
        out.extend_from_slice(&self.version.get().to_be_bytes());
        push_project_identity(&mut out, &self.project)?;
        out.extend_from_slice(&self.sequence.to_be_bytes());
        push_optional_digest(&mut out, self.previous.as_ref())?;
        push_digest(&mut out, &self.policy_digest)?;
        Ok(out)
    }

    pub fn digest(
        &self,
        algorithm: DigestAlgorithm,
    ) -> Result<Digest, RepositoryVerificationError> {
        Ok(Digest::of_bytes(algorithm, &self.canonical_bytes()?))
    }

    pub fn validate_successor(&self, next: &Self) -> Result<(), RepositoryVerificationError> {
        if self.project != next.project {
            return Err(RepositoryVerificationError::PolicyProjectChanged);
        }

        let expected_sequence = self
            .sequence
            .checked_add(1)
            .ok_or(RepositoryVerificationError::PolicySequenceOverflow)?;
        if next.sequence != expected_sequence {
            return Err(RepositoryVerificationError::PolicySequenceNotContiguous {
                expected: expected_sequence,
                actual: next.sequence,
            });
        }

        let previous = next
            .previous
            .as_ref()
            .ok_or(RepositoryVerificationError::PolicyMissingPrevious)?;
        let expected_previous = self.digest(previous.algorithm())?;
        if *previous != expected_previous {
            return Err(RepositoryVerificationError::PolicyPreviousMismatch);
        }

        Ok(())
    }
}

impl<'de> Deserialize<'de> for RepositoryPolicyState {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct WirePolicyState {
            version: ProtocolVersion,
            project: ProjectIdentity,
            sequence: u64,
            previous: Option<Digest>,
            policy_digest: Digest,
        }

        let wire = WirePolicyState::deserialize(deserializer)?;
        ensure_current_version(wire.version).map_err(D::Error::custom)?;
        Self::new(
            wire.project,
            wire.sequence,
            wire.previous,
            wire.policy_digest,
        )
        .map_err(D::Error::custom)
    }
}

/// Exact protected-source transition an adapter is asked to verify.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct RepositoryVerificationRequest {
    version: ProtocolVersion,
    project: ProjectIdentity,
    adoption: Digest,
    reference: RepositoryRef,
    from: GitObjectId,
    to: GitObjectId,
    authority_epoch: Digest,
    project_policy: Digest,
    repository_policy_state: Digest,
    repository_policy_sequence: u64,
}

impl RepositoryVerificationRequest {
    pub fn new(
        adoption: &RepositoryAdoption,
        from: GitObjectId,
        to: GitObjectId,
        authority_epoch: Digest,
        project_policy: Digest,
        repository_policy_state: &RepositoryPolicyState,
        commitment_algorithm: DigestAlgorithm,
    ) -> Result<Self, RepositoryVerificationError> {
        if from == to {
            return Err(RepositoryVerificationError::NoOpTransition);
        }
        if adoption.project != repository_policy_state.project {
            return Err(RepositoryVerificationError::PolicyProjectChanged);
        }

        Ok(Self {
            version: ProtocolVersion::CURRENT,
            project: adoption.project.clone(),
            adoption: adoption.digest(commitment_algorithm)?,
            reference: adoption.baseline.reference.clone(),
            from,
            to,
            authority_epoch,
            project_policy,
            repository_policy_state: repository_policy_state.digest(commitment_algorithm)?,
            repository_policy_sequence: repository_policy_state.sequence,
        })
    }

    pub fn project(&self) -> &ProjectIdentity {
        &self.project
    }

    pub fn reference(&self) -> &RepositoryRef {
        &self.reference
    }

    pub fn from(&self) -> &GitObjectId {
        &self.from
    }

    pub fn to(&self) -> &GitObjectId {
        &self.to
    }

    pub fn repository_policy_state(&self) -> &Digest {
        &self.repository_policy_state
    }

    pub const fn repository_policy_sequence(&self) -> u64 {
        self.repository_policy_sequence
    }

    pub fn canonical_bytes(&self) -> Result<Vec<u8>, RepositoryVerificationError> {
        ensure_current_version(self.version)?;
        let mut out = Vec::new();
        out.extend_from_slice(VERIFICATION_REQUEST_DOMAIN_V1);
        out.extend_from_slice(&self.version.get().to_be_bytes());
        push_project_identity(&mut out, &self.project)?;
        push_digest(&mut out, &self.adoption)?;
        push_ref(&mut out, &self.reference)?;
        push_git_object(&mut out, &self.from)?;
        push_git_object(&mut out, &self.to)?;
        push_digest(&mut out, &self.authority_epoch)?;
        push_digest(&mut out, &self.project_policy)?;
        push_digest(&mut out, &self.repository_policy_state)?;
        out.extend_from_slice(&self.repository_policy_sequence.to_be_bytes());
        Ok(out)
    }

    pub fn digest(
        &self,
        algorithm: DigestAlgorithm,
    ) -> Result<Digest, RepositoryVerificationError> {
        Ok(Digest::of_bytes(algorithm, &self.canonical_bytes()?))
    }
}

/// Semantics an adapter can demonstrably establish for one observation.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum VerificationCapability {
    #[serde(rename = "ref-tip-binding")]
    RefTipBinding,
    #[serde(rename = "full-history")]
    FullHistory,
    #[serde(rename = "protected-rewrite-detection")]
    ProtectedRewriteDetection,
    #[serde(rename = "policy-lineage-monotonic")]
    PolicyLineageMonotonic,
    #[serde(rename = "authorization-attestations")]
    AuthorizationAttestations,
    #[serde(rename = "offline-evidence")]
    OfflineEvidence,
}

impl VerificationCapability {
    pub const fn code(self) -> u16 {
        match self {
            Self::RefTipBinding => 1,
            Self::FullHistory => 2,
            Self::ProtectedRewriteDetection => 3,
            Self::PolicyLineageMonotonic => 4,
            Self::AuthorizationAttestations => 5,
            Self::OfflineEvidence => 6,
        }
    }
}

/// Required adapter semantics for a project-policy verification lane.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct VerificationProfile {
    required: BTreeSet<VerificationCapability>,
}

impl VerificationProfile {
    pub fn new(
        required: impl IntoIterator<Item = VerificationCapability>,
    ) -> Result<Self, RepositoryVerificationError> {
        let required = required.into_iter().collect::<BTreeSet<_>>();
        if required.is_empty() {
            return Err(RepositoryVerificationError::EmptyVerificationProfile);
        }
        Ok(Self { required })
    }

    /// Minimum M0 protected-source semantics. `LatestOnly` verification cannot
    /// satisfy this profile because `FullHistory` is mandatory.
    pub fn m0_protected_source() -> Self {
        Self::new([
            VerificationCapability::RefTipBinding,
            VerificationCapability::FullHistory,
            VerificationCapability::ProtectedRewriteDetection,
            VerificationCapability::PolicyLineageMonotonic,
            VerificationCapability::OfflineEvidence,
        ])
        .expect("static profile is non-empty")
    }

    pub fn required(&self) -> &BTreeSet<VerificationCapability> {
        &self.required
    }
}

/// Validated opaque adapter identity. This is evidence metadata, not authority.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct AdapterIdentity {
    name: String,
    version: String,
}

impl AdapterIdentity {
    pub fn new(
        name: impl Into<String>,
        version: impl Into<String>,
    ) -> Result<Self, RepositoryVerificationError> {
        let name = name.into();
        let version = version.into();
        validate_adapter_name(&name)?;
        validate_adapter_version(&version)?;
        Ok(Self { name, version })
    }

    pub fn name(&self) -> &str {
        &self.name
    }

    pub fn version(&self) -> &str {
        &self.version
    }
}

impl<'de> Deserialize<'de> for AdapterIdentity {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct WireAdapterIdentity {
            name: String,
            version: String,
        }

        let wire = WireAdapterIdentity::deserialize(deserializer)?;
        Self::new(wire.name, wire.version).map_err(D::Error::custom)
    }
}

/// Adapter-reported result. This is an observation until independently checked.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum AdapterOutcome {
    #[serde(rename = "verified")]
    Verified,
    #[serde(rename = "rejected")]
    Rejected { code: String },
}

/// Provider-neutral observation emitted after an adapter evaluates a request.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct AdapterObservation {
    adapter: AdapterIdentity,
    request_digest: Digest,
    observed_tip: GitObjectId,
    repository_policy_state: Digest,
    history_commitment: Option<Digest>,
    evidence_commitment: Option<Digest>,
    capabilities: BTreeSet<VerificationCapability>,
    outcome: AdapterOutcome,
}

impl AdapterObservation {
    pub fn new(
        adapter: AdapterIdentity,
        request_digest: Digest,
        observed_tip: GitObjectId,
        repository_policy_state: Digest,
        history_commitment: Option<Digest>,
        evidence_commitment: Option<Digest>,
        capabilities: impl IntoIterator<Item = VerificationCapability>,
        outcome: AdapterOutcome,
    ) -> Result<Self, RepositoryVerificationError> {
        let capabilities = capabilities.into_iter().collect::<BTreeSet<_>>();
        if capabilities.contains(&VerificationCapability::FullHistory)
            && history_commitment.is_none()
        {
            return Err(RepositoryVerificationError::MissingHistoryCommitment);
        }
        if capabilities.contains(&VerificationCapability::OfflineEvidence)
            && evidence_commitment.is_none()
        {
            return Err(RepositoryVerificationError::MissingEvidenceCommitment);
        }
        if let AdapterOutcome::Rejected { code } = &outcome {
            if code.is_empty() || code.len() > MAX_ADAPTER_FIELD_LEN {
                return Err(RepositoryVerificationError::InvalidAdapterRejectionCode(
                    code.clone(),
                ));
            }
        }

        Ok(Self {
            adapter,
            request_digest,
            observed_tip,
            repository_policy_state,
            history_commitment,
            evidence_commitment,
            capabilities,
            outcome,
        })
    }

    pub fn adapter(&self) -> &AdapterIdentity {
        &self.adapter
    }

    pub fn capabilities(&self) -> &BTreeSet<VerificationCapability> {
        &self.capabilities
    }
}

impl<'de> Deserialize<'de> for AdapterObservation {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct WireObservation {
            adapter: AdapterIdentity,
            request_digest: Digest,
            observed_tip: GitObjectId,
            repository_policy_state: Digest,
            history_commitment: Option<Digest>,
            evidence_commitment: Option<Digest>,
            capabilities: BTreeSet<VerificationCapability>,
            outcome: AdapterOutcome,
        }

        let wire = WireObservation::deserialize(deserializer)?;
        Self::new(
            wire.adapter,
            wire.request_digest,
            wire.observed_tip,
            wire.repository_policy_state,
            wire.history_commitment,
            wire.evidence_commitment,
            wire.capabilities,
            wire.outcome,
        )
        .map_err(D::Error::custom)
    }
}

/// Positive runtime result proving an adapter observation matches the exact
/// request and required semantics. This does not itself authenticate the
/// adapter or external signatures; the adapter implementation must do that.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct StructurallyQualifiedRepositoryVerification {
    request_digest: Digest,
    adapter: AdapterIdentity,
    observed_tip: GitObjectId,
    repository_policy_state: Digest,
    history_commitment: Option<Digest>,
    evidence_commitment: Option<Digest>,
    capabilities: BTreeSet<VerificationCapability>,
}

impl StructurallyQualifiedRepositoryVerification {
    pub fn request_digest(&self) -> &Digest {
        &self.request_digest
    }

    pub fn adapter(&self) -> &AdapterIdentity {
        &self.adapter
    }

    pub fn observed_tip(&self) -> &GitObjectId {
        &self.observed_tip
    }

    pub fn repository_policy_state(&self) -> &Digest {
        &self.repository_policy_state
    }

    pub fn history_commitment(&self) -> Option<&Digest> {
        self.history_commitment.as_ref()
    }

    pub fn evidence_commitment(&self) -> Option<&Digest> {
        self.evidence_commitment.as_ref()
    }

    pub fn capabilities(&self) -> &BTreeSet<VerificationCapability> {
        &self.capabilities
    }
}

pub fn structurally_qualify_observation(
    profile: &VerificationProfile,
    request: &RepositoryVerificationRequest,
    observation: AdapterObservation,
) -> Result<StructurallyQualifiedRepositoryVerification, RepositoryVerificationError> {
    match &observation.outcome {
        AdapterOutcome::Verified => {}
        AdapterOutcome::Rejected { code } => {
            return Err(RepositoryVerificationError::AdapterRejected(code.clone()));
        }
    }

    let expected_request = request.digest(observation.request_digest.algorithm())?;
    if expected_request != observation.request_digest {
        return Err(RepositoryVerificationError::RequestDigestMismatch);
    }
    if request.to != observation.observed_tip {
        return Err(RepositoryVerificationError::ObservedTipMismatch);
    }
    if request.repository_policy_state != observation.repository_policy_state {
        return Err(RepositoryVerificationError::ObservedPolicyStateMismatch);
    }

    let missing = profile
        .required
        .difference(&observation.capabilities)
        .copied()
        .collect::<Vec<_>>();
    if !missing.is_empty() {
        return Err(RepositoryVerificationError::MissingVerificationCapabilities(
            missing,
        ));
    }

    Ok(StructurallyQualifiedRepositoryVerification {
        request_digest: observation.request_digest,
        adapter: observation.adapter,
        observed_tip: observation.observed_tip,
        repository_policy_state: observation.repository_policy_state,
        history_commitment: observation.history_commitment,
        evidence_commitment: observation.evidence_commitment,
        capabilities: observation.capabilities,
    })
}

#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum RepositoryVerificationError {
    #[error("unsupported repository protocol version: {0}")]
    UnsupportedProtocolVersion(u16),
    #[error("invalid Git object length for {algorithm:?}: expected {expected}, got {actual}")]
    InvalidGitObjectLength {
        algorithm: GitObjectAlgorithm,
        expected: usize,
        actual: usize,
    },
    #[error("invalid repository reference: {0}")]
    InvalidRepositoryRef(String),
    #[error("canonical field {field} is too large: {len} > {max}")]
    CanonicalFieldTooLarge {
        field: &'static str,
        len: usize,
        max: usize,
    },
    #[error("repository policy genesis state must not name a predecessor")]
    GenesisPolicyHasPrevious,
    #[error("non-genesis repository policy state must name a predecessor")]
    PolicyMissingPrevious,
    #[error("repository policy state changed projects")]
    PolicyProjectChanged,
    #[error("repository policy sequence overflow")]
    PolicySequenceOverflow,
    #[error("repository policy sequence is not contiguous: expected {expected}, got {actual}")]
    PolicySequenceNotContiguous { expected: u64, actual: u64 },
    #[error("repository policy predecessor digest mismatch")]
    PolicyPreviousMismatch,
    #[error("repository verification request cannot describe a no-op transition")]
    NoOpTransition,
    #[error("verification profile must require at least one capability")]
    EmptyVerificationProfile,
    #[error("invalid adapter name: {0}")]
    InvalidAdapterName(String),
    #[error("invalid adapter version: {0}")]
    InvalidAdapterVersion(String),
    #[error("adapter full-history capability requires a history commitment")]
    MissingHistoryCommitment,
    #[error("adapter offline-evidence capability requires an evidence commitment")]
    MissingEvidenceCommitment,
    #[error("invalid adapter rejection code: {0}")]
    InvalidAdapterRejectionCode(String),
    #[error("adapter rejected verification: {0}")]
    AdapterRejected(String),
    #[error("adapter observation names a different verification request")]
    RequestDigestMismatch,
    #[error("adapter observation names a different reference tip")]
    ObservedTipMismatch,
    #[error("adapter observation names a different repository policy state")]
    ObservedPolicyStateMismatch,
    #[error("adapter observation is missing required capabilities: {0:?}")]
    MissingVerificationCapabilities(Vec<VerificationCapability>),
}

fn ensure_current_version(version: ProtocolVersion) -> Result<(), RepositoryVerificationError> {
    if version.get() == CURRENT_PROTOCOL_VERSION {
        Ok(())
    } else {
        Err(RepositoryVerificationError::UnsupportedProtocolVersion(
            version.get(),
        ))
    }
}

fn validate_ref_name(value: &str) -> Result<(), RepositoryVerificationError> {
    let invalid = value.is_empty()
        || value.len() > MAX_REF_LEN
        || !value.starts_with("refs/")
        || value == "refs/"
        || value.ends_with('/')
        || value.ends_with('.')
        || value.contains("//")
        || value.contains("..")
        || value.contains("@{")
        || value
            .split('/')
            .any(|part| part.is_empty() || part.starts_with('.') || part.ends_with(".lock"))
        || value.bytes().any(|byte| {
            byte <= 0x20
                || byte == 0x7f
                || matches!(byte, b'~' | b'^' | b':' | b'?' | b'*' | b'[' | b'\\')
        });

    if invalid {
        Err(RepositoryVerificationError::InvalidRepositoryRef(
            value.to_owned(),
        ))
    } else {
        Ok(())
    }
}

fn validate_adapter_name(value: &str) -> Result<(), RepositoryVerificationError> {
    let valid = !value.is_empty()
        && value.len() <= MAX_ADAPTER_FIELD_LEN
        && value
            .bytes()
            .all(|byte| byte.is_ascii_lowercase() || byte.is_ascii_digit() || b"-_.".contains(&byte));
    if valid {
        Ok(())
    } else {
        Err(RepositoryVerificationError::InvalidAdapterName(
            value.to_owned(),
        ))
    }
}

fn validate_adapter_version(value: &str) -> Result<(), RepositoryVerificationError> {
    let valid = !value.is_empty()
        && value.len() <= MAX_ADAPTER_FIELD_LEN
        && value.bytes().all(|byte| (0x21..=0x7e).contains(&byte));
    if valid {
        Ok(())
    } else {
        Err(RepositoryVerificationError::InvalidAdapterVersion(
            value.to_owned(),
        ))
    }
}

fn push_project_identity(
    out: &mut Vec<u8>,
    project: &ProjectIdentity,
) -> Result<(), RepositoryVerificationError> {
    out.extend_from_slice(&project.version().get().to_be_bytes());
    push_digest(out, project.digest())
}

fn push_tip(out: &mut Vec<u8>, tip: &RepositoryTip) -> Result<(), RepositoryVerificationError> {
    push_ref(out, &tip.reference)?;
    push_git_object(out, &tip.object)
}

fn push_ref(
    out: &mut Vec<u8>,
    reference: &RepositoryRef,
) -> Result<(), RepositoryVerificationError> {
    let bytes = reference.as_str().as_bytes();
    push_u16_len(out, bytes.len(), "repository_ref")?;
    out.extend_from_slice(bytes);
    Ok(())
}

fn push_git_object(
    out: &mut Vec<u8>,
    object: &GitObjectId,
) -> Result<(), RepositoryVerificationError> {
    out.push(object.algorithm.code());
    push_u16_len(out, object.bytes.len(), "git_object_id")?;
    out.extend_from_slice(&object.bytes);
    Ok(())
}

fn push_optional_digest(
    out: &mut Vec<u8>,
    digest: Option<&Digest>,
) -> Result<(), RepositoryVerificationError> {
    match digest {
        Some(digest) => {
            out.push(1);
            push_digest(out, digest)
        }
        None => {
            out.push(0);
            Ok(())
        }
    }
}

fn push_digest(out: &mut Vec<u8>, digest: &Digest) -> Result<(), RepositoryVerificationError> {
    let algorithm = digest.algorithm().id().as_bytes();
    push_u16_len(out, algorithm.len(), "digest_algorithm")?;
    out.extend_from_slice(algorithm);
    push_u32_len(out, digest.as_bytes().len(), "digest_bytes")?;
    out.extend_from_slice(digest.as_bytes());
    Ok(())
}

fn push_u16_len(
    out: &mut Vec<u8>,
    len: usize,
    field: &'static str,
) -> Result<(), RepositoryVerificationError> {
    let value = u16::try_from(len).map_err(|_| RepositoryVerificationError::CanonicalFieldTooLarge {
        field,
        len,
        max: u16::MAX as usize,
    })?;
    out.extend_from_slice(&value.to_be_bytes());
    Ok(())
}

fn push_u32_len(
    out: &mut Vec<u8>,
    len: usize,
    field: &'static str,
) -> Result<(), RepositoryVerificationError> {
    let value = u32::try_from(len).map_err(|_| RepositoryVerificationError::CanonicalFieldTooLarge {
        field,
        len,
        max: u32::MAX as usize,
    })?;
    out.extend_from_slice(&value.to_be_bytes());
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_forge_core::{ProjectIdentitySeed, GENESIS_NONCE_LEN};

    fn digest(byte: u8) -> Digest {
        Digest::new(DigestAlgorithm::Sha256, vec![byte; 32]).unwrap()
    }

    fn project() -> ProjectIdentity {
        ProjectIdentity::derive(
            &ProjectIdentitySeed::new([0x11; GENESIS_NONCE_LEN], digest(0x22)),
            DigestAlgorithm::Sha256,
        )
        .unwrap()
    }

    fn git_sha1(byte: u8) -> GitObjectId {
        GitObjectId::new(GitObjectAlgorithm::Sha1, vec![byte; 20]).unwrap()
    }

    fn reference() -> RepositoryRef {
        RepositoryRef::new("refs/heads/main").unwrap()
    }

    fn adoption() -> RepositoryAdoption {
        RepositoryAdoption::new(
            project(),
            RepositoryTip::new(reference(), git_sha1(0x33)),
            digest(0x44),
            digest(0x55),
            digest(0x66),
            1_000,
        )
    }

    fn policy_state() -> RepositoryPolicyState {
        RepositoryPolicyState::new(project(), 0, None, digest(0x66)).unwrap()
    }

    fn request() -> RepositoryVerificationRequest {
        RepositoryVerificationRequest::new(
            &adoption(),
            git_sha1(0x33),
            git_sha1(0x77),
            digest(0x44),
            digest(0x55),
            &policy_state(),
            DigestAlgorithm::Sha256,
        )
        .unwrap()
    }

    #[test]
    fn git_object_lengths_fail_closed() {
        let error = GitObjectId::new(GitObjectAlgorithm::Sha1, vec![0; 19]).unwrap_err();
        assert_eq!(
            error,
            RepositoryVerificationError::InvalidGitObjectLength {
                algorithm: GitObjectAlgorithm::Sha1,
                expected: 20,
                actual: 19,
            }
        );
    }

    #[test]
    fn unsafe_or_ambiguous_refs_fail_closed() {
        for invalid in [
            "main",
            "refs/heads/a..b",
            "refs/heads/a//b",
            "refs/heads/.hidden",
            "refs/heads/a.lock",
            "refs/heads/a?b",
        ] {
            assert!(RepositoryRef::new(invalid).is_err(), "{invalid}");
        }
        assert_eq!(reference().as_str(), "refs/heads/main");
    }

    #[test]
    fn adoption_policy_and_request_have_frozen_v1_vectors() {
        let adoption = adoption();
        assert_eq!(
            adoption.digest(DigestAlgorithm::Sha256).unwrap().to_hex(),
            "b999dc307fddc2dc27942a914016ce0dce3daceb205ffd9f34ffdfefdbb4f263"
        );

        let policy = policy_state();
        assert_eq!(
            policy.digest(DigestAlgorithm::Sha256).unwrap().to_hex(),
            "e13cadc61b053d665aa7fcaf12ab26ab63bfa8009fac650ed25f48c288f4e500"
        );

        assert_eq!(
            request().digest(DigestAlgorithm::Sha256).unwrap().to_hex(),
            "d3a9aa5c5b5328712bd04b84608438bcc722380b3d5dbd8f1e052acff232c2fc"
        );
    }

    #[test]
    fn policy_lineage_rejects_rollback_gap_and_wrong_parent() {
        let first = policy_state();
        let second = RepositoryPolicyState::new(
            project(),
            1,
            Some(first.digest(DigestAlgorithm::Sha256).unwrap()),
            digest(0x67),
        )
        .unwrap();
        first.validate_successor(&second).unwrap();

        let rollback = RepositoryPolicyState::new(
            project(),
            1,
            Some(second.digest(DigestAlgorithm::Sha256).unwrap()),
            digest(0x68),
        )
        .unwrap();
        assert!(matches!(
            second.validate_successor(&rollback),
            Err(RepositoryVerificationError::PolicySequenceNotContiguous { .. })
        ));

        let gap = RepositoryPolicyState::new(
            project(),
            3,
            Some(second.digest(DigestAlgorithm::Sha256).unwrap()),
            digest(0x69),
        )
        .unwrap();
        assert!(matches!(
            second.validate_successor(&gap),
            Err(RepositoryVerificationError::PolicySequenceNotContiguous { .. })
        ));

        let wrong_parent =
            RepositoryPolicyState::new(project(), 2, Some(digest(0xaa)), digest(0x69)).unwrap();
        assert_eq!(
            second.validate_successor(&wrong_parent).unwrap_err(),
            RepositoryVerificationError::PolicyPreviousMismatch
        );
    }

    #[test]
    fn policy_state_is_project_bound() {
        let first = policy_state();
        let other_project = ProjectIdentity::derive(
            &ProjectIdentitySeed::new([0x12; GENESIS_NONCE_LEN], digest(0x22)),
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        let next = RepositoryPolicyState::new(
            other_project,
            1,
            Some(first.digest(DigestAlgorithm::Sha256).unwrap()),
            digest(0x67),
        )
        .unwrap();
        assert_eq!(
            first.validate_successor(&next).unwrap_err(),
            RepositoryVerificationError::PolicyProjectChanged
        );
    }

    #[test]
    fn no_op_transitions_fail_closed() {
        let same = git_sha1(0x33);
        let error = RepositoryVerificationRequest::new(
            &adoption(),
            same.clone(),
            same,
            digest(0x44),
            digest(0x55),
            &policy_state(),
            DigestAlgorithm::Sha256,
        )
        .unwrap_err();
        assert_eq!(error, RepositoryVerificationError::NoOpTransition);
    }

    #[test]
    fn m0_profile_requires_full_history_and_rollback_detection() {
        let profile = VerificationProfile::m0_protected_source();
        assert!(profile
            .required()
            .contains(&VerificationCapability::FullHistory));
        assert!(profile
            .required()
            .contains(&VerificationCapability::PolicyLineageMonotonic));
        assert!(profile
            .required()
            .contains(&VerificationCapability::ProtectedRewriteDetection));
        assert!(profile
            .required()
            .contains(&VerificationCapability::OfflineEvidence));
    }

    #[test]
    fn structurally_qualified_observation_binds_exact_request_tip_and_policy() {
        let request = request();
        let request_digest = request.digest(DigestAlgorithm::Sha256).unwrap();
        let observation = AdapterObservation::new(
            AdapterIdentity::new("gittuf", "0.16.0").unwrap(),
            request_digest.clone(),
            request.to().clone(),
            request.repository_policy_state().clone(),
            Some(digest(0x88)),
            Some(digest(0x89)),
            VerificationProfile::m0_protected_source()
                .required()
                .iter()
                .copied(),
            AdapterOutcome::Verified,
        )
        .unwrap();

        let qualified = structurally_qualify_observation(
            &VerificationProfile::m0_protected_source(),
            &request,
            observation,
        )
        .unwrap();
        assert_eq!(qualified.request_digest(), &request_digest);
        assert_eq!(qualified.adapter().name(), "gittuf");
        assert!(qualified.history_commitment().is_some());
        assert!(qualified.evidence_commitment().is_some());
    }

    #[test]
    fn latest_only_semantics_cannot_satisfy_m0_profile() {
        let request = request();
        let observation = AdapterObservation::new(
            AdapterIdentity::new("gittuf", "0.16.0").unwrap(),
            request.digest(DigestAlgorithm::Sha256).unwrap(),
            request.to().clone(),
            request.repository_policy_state().clone(),
            None,
            None,
            [
                VerificationCapability::RefTipBinding,
                VerificationCapability::PolicyLineageMonotonic,
            ],
            AdapterOutcome::Verified,
        )
        .unwrap();

        let error = structurally_qualify_observation(
            &VerificationProfile::m0_protected_source(),
            &request,
            observation,
        )
        .unwrap_err();
        assert!(matches!(
            error,
            RepositoryVerificationError::MissingVerificationCapabilities(_)
        ));
    }

    #[test]
    fn one_byte_target_mutation_invalidates_observation_binding() {
        let request = request();
        let mut observation = AdapterObservation::new(
            AdapterIdentity::new("gittuf", "0.16.0").unwrap(),
            request.digest(DigestAlgorithm::Sha256).unwrap(),
            request.to().clone(),
            request.repository_policy_state().clone(),
            Some(digest(0x88)),
            Some(digest(0x89)),
            VerificationProfile::m0_protected_source()
                .required()
                .iter()
                .copied(),
            AdapterOutcome::Verified,
        )
        .unwrap();
        observation.observed_tip = git_sha1(0x78);

        assert_eq!(
            structurally_qualify_observation(
                &VerificationProfile::m0_protected_source(),
                &request,
                observation,
            )
            .unwrap_err(),
            RepositoryVerificationError::ObservedTipMismatch
        );
    }

    #[test]
    fn serde_cannot_claim_full_history_without_a_history_commitment() {
        let request = request();
        let malformed = serde_json::json!({
            "adapter": {"name": "gittuf", "version": "0.16.0"},
            "request_digest": request.digest(DigestAlgorithm::Sha256).unwrap(),
            "observed_tip": request.to(),
            "repository_policy_state": request.repository_policy_state(),
            "history_commitment": null,
            "evidence_commitment": digest(0x89),
            "capabilities": ["ref-tip-binding", "full-history", "offline-evidence"],
            "outcome": "verified"
        });
        assert!(serde_json::from_value::<AdapterObservation>(malformed).is_err());
    }

    #[test]
    fn serde_revalidates_git_ids_and_refs() {
        let bad_id = r#"{"algorithm":"sha1","bytes":[0,0,0]}"#;
        assert!(serde_json::from_str::<GitObjectId>(bad_id).is_err());
        assert!(serde_json::from_str::<RepositoryRef>(r#""main""#).is_err());
    }
}
