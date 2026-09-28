// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! FORGE-009D: provider-neutral protected-ref transition intent and CAS receipt verification.
//!
//! This crate closes the repository race at the protocol boundary without
//! creating merge authority.
//!
//! ```text
//! SourceQualifiedProtectedMergeRequestV1
//! + exact ProtectedMergeRequestV1
//!     -> ProtectedRefTransitionIntentV1
//!
//! exact transition intent
//! + provider-specific atomic compare-and-swap receipt
//! + independent provider verifier
//!     -> ProviderVerifiedProtectedRefTransitionReceiptV1
//! ```
//!
//! The transition intent is not merge authorization and has no `Deserialize`
//! implementation. It can only be constructed from the positive FORGE-009B
//! source-qualified request plus the exact FORGE-009A request it qualifies.
//!
//! This crate deliberately does **not** expose a repository mutation API. A
//! concrete executor must be gated by the later M0 + authority authorization
//! theorem. The positive receipt here is post-execution evidence conditional on
//! the named provider verifier; it is not permission to execute.

#![forbid(unsafe_code)]
#![warn(missing_docs)]

use mycelix_forge_core::{
    Digest, DigestAlgorithm, ProjectIdentity, ProtocolVersion, CURRENT_PROTOCOL_VERSION,
};
use mycelix_forge_proposal::ChangeProposalId;
use mycelix_forge_protected_merge_request::{
    ProtectedMergeRequestError, ProtectedMergeRequestId, ProtectedMergeRequestV1,
    PROTECTED_MERGE_NONCE_LEN,
};
use mycelix_forge_protected_merge_source::SourceQualifiedProtectedMergeRequestV1;
use mycelix_forge_repository::{GitObjectAlgorithm, GitObjectId, RepositoryRef};
use serde::{de::Error as _, Deserialize, Deserializer, Serialize};
use thiserror::Error;

const TRANSITION_INTENT_DOMAIN_V1: &[u8] = b"mycelix-forge/protected-ref-transition-intent/v1\0";
const CAS_OBSERVATION_DOMAIN_V1: &[u8] = b"mycelix-forge/protected-ref-cas-observation/v1\0";
const VERIFIED_RECEIPT_DOMAIN_V1: &[u8] =
    b"mycelix-forge/provider-verified-protected-ref-transition/v1\0";
const MAX_PROVIDER_FIELD_LEN: usize = 128;

/// Stable provider identity for one protected-ref CAS implementation/profile.
///
/// This is evidence metadata, not an authorization source. Consumers that care
/// which executor/provider is acceptable must bind that trust separately.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct ProtectedRefCasProviderIdentityV1 {
    name: String,
    version: String,
}

impl ProtectedRefCasProviderIdentityV1 {
    /// Construct a validated provider identity.
    pub fn new(
        name: impl Into<String>,
        version: impl Into<String>,
    ) -> Result<Self, ProtectedRefTransitionError> {
        let name = name.into();
        let version = version.into();
        validate_provider_field("name", &name)?;
        validate_provider_field("version", &version)?;
        Ok(Self { name, version })
    }

    /// Provider implementation/profile name.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Provider implementation/profile version.
    pub fn version(&self) -> &str {
        &self.version
    }
}

impl<'de> Deserialize<'de> for ProtectedRefCasProviderIdentityV1 {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct WireIdentity {
            name: String,
            version: String,
        }

        let wire = WireIdentity::deserialize(deserializer)?;
        Self::new(wire.name, wire.version).map_err(D::Error::custom)
    }
}

/// Positive pre-execution transition subject derived from a source-qualified
/// protected merge request.
///
/// This remains explicitly non-authoritative. It proves only that the exact
/// protected-ref compare-and-swap subject is bound to a positive FORGE-009B
/// source qualification.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct ProtectedRefTransitionIntentV1 {
    version: ProtocolVersion,
    project: ProjectIdentity,
    proposal: ChangeProposalId,
    protected_merge_request: ProtectedMergeRequestId,
    source_qualification: Digest,
    target_ref: RepositoryRef,
    expected_base: GitObjectId,
    proposed_revision: GitObjectId,
    merge_nonce: [u8; PROTECTED_MERGE_NONCE_LEN],
}

impl ProtectedRefTransitionIntentV1 {
    /// Construct an exact protected-ref transition intent from the positive
    /// source-qualified request and the exact request it qualifies.
    pub fn new(
        source_qualified: &SourceQualifiedProtectedMergeRequestV1,
        protected_merge_request: &ProtectedMergeRequestV1,
    ) -> Result<Self, ProtectedRefTransitionError> {
        let algorithm = source_qualified
            .protected_merge_request()
            .commitment()
            .algorithm();
        let expected_request = protected_merge_request.request_id(algorithm)?;
        if source_qualified.protected_merge_request() != &expected_request {
            return Err(ProtectedRefTransitionError::ProtectedMergeRequestMismatch);
        }
        if source_qualified.project() != protected_merge_request.project() {
            return Err(ProtectedRefTransitionError::ProjectMismatch);
        }
        if source_qualified.proposal() != protected_merge_request.proposal() {
            return Err(ProtectedRefTransitionError::ProposalMismatch);
        }

        Self::from_parts(
            protected_merge_request.project().clone(),
            protected_merge_request.proposal().clone(),
            expected_request,
            source_qualified.evidence_commitment().clone(),
            protected_merge_request.target_ref().clone(),
            protected_merge_request.expected_base().clone(),
            protected_merge_request.proposed_revision().clone(),
            *protected_merge_request.merge_nonce(),
        )
    }

    #[allow(clippy::too_many_arguments)]
    fn from_parts(
        project: ProjectIdentity,
        proposal: ChangeProposalId,
        protected_merge_request: ProtectedMergeRequestId,
        source_qualification: Digest,
        target_ref: RepositoryRef,
        expected_base: GitObjectId,
        proposed_revision: GitObjectId,
        merge_nonce: [u8; PROTECTED_MERGE_NONCE_LEN],
    ) -> Result<Self, ProtectedRefTransitionError> {
        if expected_base == proposed_revision {
            return Err(ProtectedRefTransitionError::NoOpTransition);
        }
        if expected_base.algorithm() != proposed_revision.algorithm() {
            return Err(ProtectedRefTransitionError::MixedGitObjectAlgorithms {
                expected: expected_base.algorithm(),
                actual: proposed_revision.algorithm(),
            });
        }

        Ok(Self {
            version: ProtocolVersion::CURRENT,
            project,
            proposal,
            protected_merge_request,
            source_qualification,
            target_ref,
            expected_base,
            proposed_revision,
            merge_nonce,
        })
    }

    /// Project containing the protected ref.
    pub fn project(&self) -> &ProjectIdentity {
        &self.project
    }

    /// Exact immutable proposal.
    pub fn proposal(&self) -> &ChangeProposalId {
        &self.proposal
    }

    /// Exact protected merge request being transitioned.
    pub fn protected_merge_request(&self) -> &ProtectedMergeRequestId {
        &self.protected_merge_request
    }

    /// Exact FORGE-009B source-qualification evidence commitment.
    pub fn source_qualification(&self) -> &Digest {
        &self.source_qualification
    }

    /// Protected repository reference to update.
    pub fn target_ref(&self) -> &RepositoryRef {
        &self.target_ref
    }

    /// Ref value that must still be current when the atomic CAS is attempted.
    pub fn expected_base(&self) -> &GitObjectId {
        &self.expected_base
    }

    /// Ref value requested after a successful atomic CAS.
    pub fn proposed_revision(&self) -> &GitObjectId {
        &self.proposed_revision
    }

    /// Merge-attempt nonce inherited from the protected merge request.
    pub const fn merge_nonce(&self) -> &[u8; PROTECTED_MERGE_NONCE_LEN] {
        &self.merge_nonce
    }

    /// Canonical v1 transition-intent bytes.
    pub fn canonical_bytes(&self) -> Result<Vec<u8>, ProtectedRefTransitionError> {
        ensure_v1(self.version)?;
        let mut out = Vec::new();
        out.extend_from_slice(TRANSITION_INTENT_DOMAIN_V1);
        out.extend_from_slice(&self.version.get().to_be_bytes());
        push_project_identity(&mut out, &self.project)?;
        push_digest(&mut out, self.proposal.commitment())?;
        push_digest(&mut out, self.protected_merge_request.commitment())?;
        push_digest(&mut out, &self.source_qualification)?;
        push_bytes(&mut out, "target_ref", self.target_ref.as_str().as_bytes())?;
        push_git_object(&mut out, &self.expected_base)?;
        push_git_object(&mut out, &self.proposed_revision)?;
        out.extend_from_slice(&self.merge_nonce);
        Ok(out)
    }

    /// Stable commitment to this exact compare-and-swap intent.
    pub fn digest(
        &self,
        algorithm: DigestAlgorithm,
    ) -> Result<Digest, ProtectedRefTransitionError> {
        Ok(Digest::of_bytes(algorithm, &self.canonical_bytes()?))
    }
}

/// Provider-reported outcome for one CAS attempt.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ProtectedRefCasOutcomeV1 {
    /// Provider claims the expected-old -> requested-new CAS was applied.
    #[serde(rename = "applied")]
    Applied,
    /// Provider rejected the attempt because the current ref no longer matched
    /// the expected old object.
    #[serde(rename = "rejected-stale")]
    RejectedStale,
    /// Provider rejected the operation for another reason.
    #[serde(rename = "rejected")]
    Rejected,
}

impl ProtectedRefCasOutcomeV1 {
    const fn code(self) -> u8 {
        match self {
            Self::Applied => 1,
            Self::RejectedStale => 2,
            Self::Rejected => 3,
        }
    }
}

/// Raw provider observation for one protected-ref CAS attempt.
///
/// This is deserializable evidence and is therefore non-authoritative until an
/// independently supplied [`ProtectedRefCasReceiptVerifierV1`] accepts it.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct ProtectedRefCasObservationV1 {
    version: ProtocolVersion,
    provider: ProtectedRefCasProviderIdentityV1,
    transition_intent: Digest,
    target_ref: RepositoryRef,
    expected_before: GitObjectId,
    observed_before: GitObjectId,
    requested_after: GitObjectId,
    observed_after: GitObjectId,
    merge_nonce: [u8; PROTECTED_MERGE_NONCE_LEN],
    provider_operation: Digest,
    provider_receipt: Digest,
    outcome: ProtectedRefCasOutcomeV1,
}

impl ProtectedRefCasObservationV1 {
    /// Construct a raw provider observation.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        provider: ProtectedRefCasProviderIdentityV1,
        transition_intent: Digest,
        target_ref: RepositoryRef,
        expected_before: GitObjectId,
        observed_before: GitObjectId,
        requested_after: GitObjectId,
        observed_after: GitObjectId,
        merge_nonce: [u8; PROTECTED_MERGE_NONCE_LEN],
        provider_operation: Digest,
        provider_receipt: Digest,
        outcome: ProtectedRefCasOutcomeV1,
    ) -> Result<Self, ProtectedRefTransitionError> {
        Self::from_parts(
            provider,
            transition_intent,
            target_ref,
            expected_before,
            observed_before,
            requested_after,
            observed_after,
            merge_nonce,
            provider_operation,
            provider_receipt,
            outcome,
        )
    }

    #[allow(clippy::too_many_arguments)]
    fn from_parts(
        provider: ProtectedRefCasProviderIdentityV1,
        transition_intent: Digest,
        target_ref: RepositoryRef,
        expected_before: GitObjectId,
        observed_before: GitObjectId,
        requested_after: GitObjectId,
        observed_after: GitObjectId,
        merge_nonce: [u8; PROTECTED_MERGE_NONCE_LEN],
        provider_operation: Digest,
        provider_receipt: Digest,
        outcome: ProtectedRefCasOutcomeV1,
    ) -> Result<Self, ProtectedRefTransitionError> {
        let expected_algorithm = expected_before.algorithm();
        for (field, object) in [
            ("observed_before", &observed_before),
            ("requested_after", &requested_after),
            ("observed_after", &observed_after),
        ] {
            if object.algorithm() != expected_algorithm {
                return Err(ProtectedRefTransitionError::ObservationGitObjectAlgorithmMismatch {
                    field,
                    expected: expected_algorithm,
                    actual: object.algorithm(),
                });
            }
        }

        Ok(Self {
            version: ProtocolVersion::CURRENT,
            provider,
            transition_intent,
            target_ref,
            expected_before,
            observed_before,
            requested_after,
            observed_after,
            merge_nonce,
            provider_operation,
            provider_receipt,
            outcome,
        })
    }

    /// Provider identity that emitted this observation.
    pub fn provider(&self) -> &ProtectedRefCasProviderIdentityV1 {
        &self.provider
    }

    /// Exact protected-ref transition intent claimed by the provider.
    pub fn transition_intent(&self) -> &Digest {
        &self.transition_intent
    }

    /// Protected ref the provider claims to have updated.
    pub fn target_ref(&self) -> &RepositoryRef {
        &self.target_ref
    }

    /// Old value passed as the atomic CAS precondition.
    pub fn expected_before(&self) -> &GitObjectId {
        &self.expected_before
    }

    /// Old value observed by the provider for this operation.
    pub fn observed_before(&self) -> &GitObjectId {
        &self.observed_before
    }

    /// New value requested by the CAS operation.
    pub fn requested_after(&self) -> &GitObjectId {
        &self.requested_after
    }

    /// Ref value observed after the provider claims the CAS completed.
    pub fn observed_after(&self) -> &GitObjectId {
        &self.observed_after
    }

    /// Merge-attempt nonce carried into the provider operation.
    pub const fn merge_nonce(&self) -> &[u8; PROTECTED_MERGE_NONCE_LEN] {
        &self.merge_nonce
    }

    /// Provider-specific stable operation identifier.
    pub fn provider_operation(&self) -> &Digest {
        &self.provider_operation
    }

    /// Commitment to provider-native receipt/evidence bytes.
    pub fn provider_receipt(&self) -> &Digest {
        &self.provider_receipt
    }

    /// Claimed provider outcome.
    pub const fn outcome(&self) -> ProtectedRefCasOutcomeV1 {
        self.outcome
    }

    /// Canonical v1 observation bytes.
    pub fn canonical_bytes(&self) -> Result<Vec<u8>, ProtectedRefTransitionError> {
        ensure_v1(self.version)?;
        validate_provider_field("name", self.provider.name())?;
        validate_provider_field("version", self.provider.version())?;
        let mut out = Vec::new();
        out.extend_from_slice(CAS_OBSERVATION_DOMAIN_V1);
        out.extend_from_slice(&self.version.get().to_be_bytes());
        push_bytes(&mut out, "provider_name", self.provider.name().as_bytes())?;
        push_bytes(
            &mut out,
            "provider_version",
            self.provider.version().as_bytes(),
        )?;
        push_digest(&mut out, &self.transition_intent)?;
        push_bytes(&mut out, "target_ref", self.target_ref.as_str().as_bytes())?;
        push_git_object(&mut out, &self.expected_before)?;
        push_git_object(&mut out, &self.observed_before)?;
        push_git_object(&mut out, &self.requested_after)?;
        push_git_object(&mut out, &self.observed_after)?;
        out.extend_from_slice(&self.merge_nonce);
        push_digest(&mut out, &self.provider_operation)?;
        push_digest(&mut out, &self.provider_receipt)?;
        out.push(self.outcome.code());
        Ok(out)
    }

    /// Stable observation commitment.
    pub fn digest(
        &self,
        algorithm: DigestAlgorithm,
    ) -> Result<Digest, ProtectedRefTransitionError> {
        Ok(Digest::of_bytes(algorithm, &self.canonical_bytes()?))
    }
}

impl<'de> Deserialize<'de> for ProtectedRefCasObservationV1 {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct WireObservation {
            version: ProtocolVersion,
            provider: ProtectedRefCasProviderIdentityV1,
            transition_intent: Digest,
            target_ref: RepositoryRef,
            expected_before: GitObjectId,
            observed_before: GitObjectId,
            requested_after: GitObjectId,
            observed_after: GitObjectId,
            merge_nonce: [u8; PROTECTED_MERGE_NONCE_LEN],
            provider_operation: Digest,
            provider_receipt: Digest,
            outcome: ProtectedRefCasOutcomeV1,
        }

        let wire = WireObservation::deserialize(deserializer)?;
        ensure_v1(wire.version).map_err(D::Error::custom)?;
        Self::from_parts(
            wire.provider,
            wire.transition_intent,
            wire.target_ref,
            wire.expected_before,
            wire.observed_before,
            wire.requested_after,
            wire.observed_after,
            wire.merge_nonce,
            wire.provider_operation,
            wire.provider_receipt,
            wire.outcome,
        )
        .map_err(D::Error::custom)
    }
}

/// Provider-specific receipt verification failure.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum ProtectedRefCasVerifierErrorV1 {
    /// The provider-specific receipt verifier rejected the observation.
    #[error("protected-ref CAS receipt verifier rejected provider evidence")]
    Rejected,
}

/// Independent verifier for provider-specific atomic-CAS evidence.
pub trait ProtectedRefCasReceiptVerifierV1 {
    /// Stable provider implementation/profile expected by this verifier.
    fn identity(&self) -> ProtectedRefCasProviderIdentityV1;

    /// Verify provider-native evidence that the exact compare-and-swap
    /// observation was produced by an atomic protected-ref transition.
    fn verify_atomic_compare_and_swap(
        &self,
        intent: &ProtectedRefTransitionIntentV1,
        observation: &ProtectedRefCasObservationV1,
    ) -> Result<Digest, ProtectedRefCasVerifierErrorV1>;
}

/// Positive post-execution receipt relative to one named provider verifier.
///
/// This type has no `Deserialize` implementation. It proves that the supplied
/// verifier accepted provider evidence for the exact old->new CAS subject and
/// that all protocol cross-links matched.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct ProviderVerifiedProtectedRefTransitionReceiptV1 {
    project: ProjectIdentity,
    proposal: ChangeProposalId,
    protected_merge_request: ProtectedMergeRequestId,
    source_qualification: Digest,
    transition_intent: Digest,
    provider: ProtectedRefCasProviderIdentityV1,
    provider_operation: Digest,
    provider_receipt: Digest,
    observed_before: GitObjectId,
    observed_after: GitObjectId,
    verifier_evidence: Digest,
    evidence_commitment: Digest,
}

impl ProviderVerifiedProtectedRefTransitionReceiptV1 {
    /// Project whose protected ref was transitioned.
    pub fn project(&self) -> &ProjectIdentity {
        &self.project
    }

    /// Exact proposal associated with the transition.
    pub fn proposal(&self) -> &ChangeProposalId {
        &self.proposal
    }

    /// Exact protected merge request associated with the transition.
    pub fn protected_merge_request(&self) -> &ProtectedMergeRequestId {
        &self.protected_merge_request
    }

    /// Exact FORGE-009B source qualification consumed by the transition intent.
    pub fn source_qualification(&self) -> &Digest {
        &self.source_qualification
    }

    /// Exact transition-intent commitment accepted by the provider verifier.
    pub fn transition_intent(&self) -> &Digest {
        &self.transition_intent
    }

    /// Provider implementation/profile whose evidence was verified.
    pub fn provider(&self) -> &ProtectedRefCasProviderIdentityV1 {
        &self.provider
    }

    /// Provider-specific operation identifier.
    pub fn provider_operation(&self) -> &Digest {
        &self.provider_operation
    }

    /// Provider-native receipt commitment.
    pub fn provider_receipt(&self) -> &Digest {
        &self.provider_receipt
    }

    /// Exact old ref value proven at the applied transition boundary.
    pub fn observed_before(&self) -> &GitObjectId {
        &self.observed_before
    }

    /// Exact new ref value proven after the applied transition.
    pub fn observed_after(&self) -> &GitObjectId {
        &self.observed_after
    }

    /// Provider-verifier evidence commitment.
    pub fn verifier_evidence(&self) -> &Digest {
        &self.verifier_evidence
    }

    /// Aggregate post-execution evidence commitment.
    pub fn evidence_commitment(&self) -> &Digest {
        &self.evidence_commitment
    }
}

/// Verify one provider-specific post-execution CAS receipt against the exact
/// source-qualified transition intent.
pub fn verify_protected_ref_transition_receipt_v1<V: ProtectedRefCasReceiptVerifierV1>(
    intent: &ProtectedRefTransitionIntentV1,
    observation: &ProtectedRefCasObservationV1,
    verifier: &V,
) -> Result<ProviderVerifiedProtectedRefTransitionReceiptV1, ProtectedRefTransitionError> {
    if observation.provider() != &verifier.identity() {
        return Err(ProtectedRefTransitionError::ProviderIdentityMismatch);
    }

    let intent_algorithm = observation.transition_intent().algorithm();
    let expected_intent = intent.digest(intent_algorithm)?;
    if observation.transition_intent() != &expected_intent {
        return Err(ProtectedRefTransitionError::TransitionIntentMismatch);
    }
    if observation.target_ref() != intent.target_ref() {
        return Err(ProtectedRefTransitionError::TargetRefMismatch);
    }
    if observation.expected_before() != intent.expected_base() {
        return Err(ProtectedRefTransitionError::ExpectedBeforeMismatch);
    }
    if observation.observed_before() != intent.expected_base() {
        return Err(ProtectedRefTransitionError::ObservedBeforeMismatch);
    }
    if observation.requested_after() != intent.proposed_revision() {
        return Err(ProtectedRefTransitionError::RequestedAfterMismatch);
    }
    if observation.observed_after() != intent.proposed_revision() {
        return Err(ProtectedRefTransitionError::ObservedAfterMismatch);
    }
    if observation.merge_nonce() != intent.merge_nonce() {
        return Err(ProtectedRefTransitionError::MergeNonceMismatch);
    }
    if observation.outcome() != ProtectedRefCasOutcomeV1::Applied {
        return Err(ProtectedRefTransitionError::TransitionNotApplied(
            observation.outcome(),
        ));
    }

    let verifier_evidence = verifier.verify_atomic_compare_and_swap(intent, observation)?;
    let algorithm = verifier_evidence.algorithm();
    let transition_intent = intent.digest(algorithm)?;
    let observation_digest = observation.digest(algorithm)?;

    let mut out = Vec::new();
    out.extend_from_slice(VERIFIED_RECEIPT_DOMAIN_V1);
    out.extend_from_slice(&ProtocolVersion::CURRENT.get().to_be_bytes());
    push_project_identity(&mut out, intent.project())?;
    push_digest(&mut out, intent.proposal().commitment())?;
    push_digest(&mut out, intent.protected_merge_request().commitment())?;
    push_digest(&mut out, intent.source_qualification())?;
    push_digest(&mut out, &transition_intent)?;
    push_digest(&mut out, &observation_digest)?;
    push_digest(&mut out, observation.provider_operation())?;
    push_digest(&mut out, observation.provider_receipt())?;
    push_digest(&mut out, &verifier_evidence)?;
    let evidence_commitment = Digest::of_bytes(algorithm, &out);

    Ok(ProviderVerifiedProtectedRefTransitionReceiptV1 {
        project: intent.project().clone(),
        proposal: intent.proposal().clone(),
        protected_merge_request: intent.protected_merge_request().clone(),
        source_qualification: intent.source_qualification().clone(),
        transition_intent,
        provider: observation.provider().clone(),
        provider_operation: observation.provider_operation().clone(),
        provider_receipt: observation.provider_receipt().clone(),
        observed_before: observation.observed_before().clone(),
        observed_after: observation.observed_after().clone(),
        verifier_evidence,
        evidence_commitment,
    })
}

fn ensure_v1(version: ProtocolVersion) -> Result<(), ProtectedRefTransitionError> {
    if version.get() == CURRENT_PROTOCOL_VERSION {
        Ok(())
    } else {
        Err(ProtectedRefTransitionError::UnsupportedProtocolVersion(
            version.get(),
        ))
    }
}

fn validate_provider_field(
    field: &'static str,
    value: &str,
) -> Result<(), ProtectedRefTransitionError> {
    let len = value.len();
    if len == 0 || len > MAX_PROVIDER_FIELD_LEN {
        return Err(ProtectedRefTransitionError::InvalidProviderField { field, len });
    }
    Ok(())
}

fn push_project_identity(
    out: &mut Vec<u8>,
    project: &ProjectIdentity,
) -> Result<(), ProtectedRefTransitionError> {
    out.extend_from_slice(&project.version().get().to_be_bytes());
    push_digest(out, project.digest())
}

fn push_git_object(
    out: &mut Vec<u8>,
    object: &GitObjectId,
) -> Result<(), ProtectedRefTransitionError> {
    out.push(object.algorithm().code());
    push_bytes(out, "git_object", object.as_bytes())
}

fn push_digest(
    out: &mut Vec<u8>,
    digest: &Digest,
) -> Result<(), ProtectedRefTransitionError> {
    push_bytes(out, "digest_algorithm", digest.algorithm().id().as_bytes())?;
    push_bytes(out, "digest", digest.as_bytes())
}

fn push_bytes(
    out: &mut Vec<u8>,
    field: &'static str,
    bytes: &[u8],
) -> Result<(), ProtectedRefTransitionError> {
    let len = u32::try_from(bytes.len()).map_err(|_| {
        ProtectedRefTransitionError::CanonicalFieldTooLarge {
            field,
            len: bytes.len(),
            max: u32::MAX as usize,
        }
    })?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(bytes);
    Ok(())
}

/// Protected-ref transition construction/verification failures.
#[derive(Debug, Error, PartialEq, Eq)]
pub enum ProtectedRefTransitionError {
    /// Positive source qualification belongs to another protected merge request.
    #[error("source-qualified request does not match the protected merge request")]
    ProtectedMergeRequestMismatch,
    /// Source qualification and protected merge request belong to different projects.
    #[error("protected-ref transition project mismatch")]
    ProjectMismatch,
    /// Source qualification and protected merge request belong to different proposals.
    #[error("protected-ref transition proposal mismatch")]
    ProposalMismatch,
    /// Transition attempts to replace a ref with its existing value.
    #[error("protected-ref transition is a no-op")]
    NoOpTransition,
    /// Requested new object uses another Git object algorithm.
    #[error("proposed ref value uses {actual:?}, expected {expected:?}")]
    MixedGitObjectAlgorithms {
        /// Expected algorithm from the current base.
        expected: GitObjectAlgorithm,
        /// Actual proposed-object algorithm.
        actual: GitObjectAlgorithm,
    },
    /// One raw observation field uses another Git object algorithm.
    #[error("CAS observation field {field} uses {actual:?}, expected {expected:?}")]
    ObservationGitObjectAlgorithmMismatch {
        /// Field containing the mismatch.
        field: &'static str,
        /// Expected algorithm from `expected_before`.
        expected: GitObjectAlgorithm,
        /// Actual field algorithm.
        actual: GitObjectAlgorithm,
    },
    /// Observation provider differs from the supplied verifier.
    #[error("protected-ref CAS provider identity mismatch")]
    ProviderIdentityMismatch,
    /// Observation names another transition intent.
    #[error("protected-ref CAS transition intent mismatch")]
    TransitionIntentMismatch,
    /// Observation names another protected ref.
    #[error("protected-ref CAS target ref mismatch")]
    TargetRefMismatch,
    /// CAS expected-old argument differs from the transition intent.
    #[error("protected-ref CAS expected-old value mismatch")]
    ExpectedBeforeMismatch,
    /// Provider observed another old ref value.
    #[error("protected-ref CAS observed stale/different old value")]
    ObservedBeforeMismatch,
    /// CAS requested-new argument differs from the transition intent.
    #[error("protected-ref CAS requested-new value mismatch")]
    RequestedAfterMismatch,
    /// Provider observed another post-transition ref value.
    #[error("protected-ref CAS observed-after value mismatch")]
    ObservedAfterMismatch,
    /// Provider observation carries another merge-attempt nonce.
    #[error("protected-ref CAS merge nonce mismatch")]
    MergeNonceMismatch,
    /// Provider did not report an applied transition.
    #[error("protected-ref CAS was not applied: {0:?}")]
    TransitionNotApplied(ProtectedRefCasOutcomeV1),
    /// Provider identity field failed validation.
    #[error("invalid protected-ref provider field {field}: length {len}")]
    InvalidProviderField {
        /// Field name.
        field: &'static str,
        /// Observed byte length.
        len: usize,
    },
    /// Unsupported protocol version.
    #[error("unsupported protected-ref transition protocol version: {0}")]
    UnsupportedProtocolVersion(u16),
    /// Canonical field exceeded v1 bounds.
    #[error("canonical field {field} is too large: {len} > {max}")]
    CanonicalFieldTooLarge {
        /// Field name.
        field: &'static str,
        /// Actual byte length.
        len: usize,
        /// Maximum encodable length.
        max: usize,
    },
    /// Protected merge request canonicalization failed.
    #[error(transparent)]
    ProtectedMergeRequest(#[from] ProtectedMergeRequestError),
    /// Provider-specific receipt verification failed.
    #[error(transparent)]
    Verifier(#[from] ProtectedRefCasVerifierErrorV1),
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_forge_core::{ProjectIdentitySeed, GENESIS_NONCE_LEN};
    use std::cell::Cell;

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

    fn git(byte: u8) -> GitObjectId {
        GitObjectId::new(GitObjectAlgorithm::Sha1, vec![byte; 20]).unwrap()
    }

    fn merge_request_id(byte: u8) -> ProtectedMergeRequestId {
        let wire_compatible = ChangeProposalId::new(digest(byte));
        let bytes = serde_json::to_vec(&wire_compatible).unwrap();
        serde_json::from_slice(&bytes).unwrap()
    }

    fn provider() -> ProtectedRefCasProviderIdentityV1 {
        ProtectedRefCasProviderIdentityV1::new("git-update-ref", "2.51.0").unwrap()
    }

    fn intent() -> ProtectedRefTransitionIntentV1 {
        ProtectedRefTransitionIntentV1::from_parts(
            project(),
            ChangeProposalId::new(digest(0x20)),
            merge_request_id(0x21),
            digest(0x22),
            RepositoryRef::new("refs/heads/main").unwrap(),
            git(0x30),
            git(0x31),
            [0x40; PROTECTED_MERGE_NONCE_LEN],
        )
        .unwrap()
    }

    fn observation(intent: &ProtectedRefTransitionIntentV1) -> ProtectedRefCasObservationV1 {
        ProtectedRefCasObservationV1::new(
            provider(),
            intent.digest(DigestAlgorithm::Sha256).unwrap(),
            intent.target_ref().clone(),
            intent.expected_base().clone(),
            intent.expected_base().clone(),
            intent.proposed_revision().clone(),
            intent.proposed_revision().clone(),
            *intent.merge_nonce(),
            digest(0x50),
            digest(0x51),
            ProtectedRefCasOutcomeV1::Applied,
        )
        .unwrap()
    }

    struct ExactVerifier {
        calls: Cell<u32>,
    }

    impl ProtectedRefCasReceiptVerifierV1 for ExactVerifier {
        fn identity(&self) -> ProtectedRefCasProviderIdentityV1 {
            provider()
        }

        fn verify_atomic_compare_and_swap(
            &self,
            _intent: &ProtectedRefTransitionIntentV1,
            observation: &ProtectedRefCasObservationV1,
        ) -> Result<Digest, ProtectedRefCasVerifierErrorV1> {
            self.calls.set(self.calls.get() + 1);
            if observation.provider_operation() == &digest(0x50)
                && observation.provider_receipt() == &digest(0x51)
            {
                Ok(digest(0x60))
            } else {
                Err(ProtectedRefCasVerifierErrorV1::Rejected)
            }
        }
    }

    #[test]
    fn exact_applied_cas_receipt_upgrades_to_provider_verified_evidence() {
        let intent = intent();
        let observation = observation(&intent);
        let verifier = ExactVerifier { calls: Cell::new(0) };
        let verified =
            verify_protected_ref_transition_receipt_v1(&intent, &observation, &verifier).unwrap();

        assert_eq!(verified.protected_merge_request(), intent.protected_merge_request());
        assert_eq!(verified.observed_before(), intent.expected_base());
        assert_eq!(verified.observed_after(), intent.proposed_revision());
        assert_eq!(verifier.calls.get(), 1);
    }

    #[test]
    fn stale_old_value_fails_before_provider_verifier_is_invoked() {
        let intent = intent();
        let mut observation = observation(&intent);
        observation.observed_before = git(0x32);
        let verifier = ExactVerifier { calls: Cell::new(0) };

        let error =
            verify_protected_ref_transition_receipt_v1(&intent, &observation, &verifier)
                .unwrap_err();
        assert_eq!(error, ProtectedRefTransitionError::ObservedBeforeMismatch);
        assert_eq!(verifier.calls.get(), 0);
    }

    #[test]
    fn nonce_substitution_fails_closed() {
        let intent = intent();
        let mut observation = observation(&intent);
        observation.merge_nonce = [0x41; PROTECTED_MERGE_NONCE_LEN];
        let verifier = ExactVerifier { calls: Cell::new(0) };

        let error =
            verify_protected_ref_transition_receipt_v1(&intent, &observation, &verifier)
                .unwrap_err();
        assert_eq!(error, ProtectedRefTransitionError::MergeNonceMismatch);
        assert_eq!(verifier.calls.get(), 0);
    }

    #[test]
    fn rejected_or_stale_outcome_never_becomes_positive_receipt() {
        let intent = intent();
        let mut observation = observation(&intent);
        observation.outcome = ProtectedRefCasOutcomeV1::RejectedStale;
        let verifier = ExactVerifier { calls: Cell::new(0) };

        let error =
            verify_protected_ref_transition_receipt_v1(&intent, &observation, &verifier)
                .unwrap_err();
        assert_eq!(
            error,
            ProtectedRefTransitionError::TransitionNotApplied(
                ProtectedRefCasOutcomeV1::RejectedStale
            )
        );
        assert_eq!(verifier.calls.get(), 0);
    }

    #[test]
    fn provider_identity_substitution_fails_closed() {
        let intent = intent();
        let mut observation = observation(&intent);
        observation.provider =
            ProtectedRefCasProviderIdentityV1::new("other-provider", "1").unwrap();
        let verifier = ExactVerifier { calls: Cell::new(0) };

        let error =
            verify_protected_ref_transition_receipt_v1(&intent, &observation, &verifier)
                .unwrap_err();
        assert_eq!(error, ProtectedRefTransitionError::ProviderIdentityMismatch);
        assert_eq!(verifier.calls.get(), 0);
    }

    #[test]
    fn raw_observation_round_trip_remains_non_authoritative() {
        let intent = intent();
        let observation = observation(&intent);
        let bytes = serde_json::to_vec(&observation).unwrap();
        let decoded: ProtectedRefCasObservationV1 = serde_json::from_slice(&bytes).unwrap();
        assert_eq!(observation, decoded);
    }

    #[allow(dead_code)]
    fn constructor_requires_source_qualified_request(
        source: &SourceQualifiedProtectedMergeRequestV1,
        request: &ProtectedMergeRequestV1,
    ) {
        let _ = ProtectedRefTransitionIntentV1::new(source, request);
    }
}
