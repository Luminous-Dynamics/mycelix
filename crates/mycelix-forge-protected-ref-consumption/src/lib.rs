// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! FORGE-009E: atomically couple protected-ref CAS with durable request consumption.
//!
//! The key theorem is not merely "old -> new CAS succeeded". The transition is
//! consumed only when the same provider transaction also creates the canonical
//! per-request consumption marker from an absent state.
//!
//! ```text
//! ProtectedRefTransitionIntentV1
//!     -> AtomicProtectedRefConsumptionIntentV1
//!
//! target ref: expected_base -> proposed_revision
//! consumption marker: absent -> proposed_revision
//! same provider transaction
//! + independent provider verifier
//!     -> ProviderVerifiedAtomicProtectedRefConsumptionReceiptV1
//! ```
//!
//! This crate still does not expose a repository mutation API and does not grant
//! merge authority. It only defines the exact atomic transaction subject and
//! verifies provider-specific post-execution evidence.

#![forbid(unsafe_code)]
#![warn(missing_docs)]

use mycelix_forge_core::{
    Digest, DigestAlgorithm, ProjectIdentity, ProtocolVersion, CURRENT_PROTOCOL_VERSION,
};
use mycelix_forge_proposal::ChangeProposalId;
use mycelix_forge_protected_merge_request::{
    ProtectedMergeRequestId, PROTECTED_MERGE_NONCE_LEN,
};
use mycelix_forge_protected_ref_transition::{
    ProtectedRefCasProviderIdentityV1, ProtectedRefTransitionError,
    ProtectedRefTransitionIntentV1,
};
use mycelix_forge_repository::{
    GitObjectAlgorithm, GitObjectId, RepositoryRef, RepositoryVerificationError,
};
use serde::{de::Error as _, Deserialize, Deserializer, Serialize};
use thiserror::Error;

const ATOMIC_CONSUMPTION_INTENT_DOMAIN_V1: &[u8] =
    b"mycelix-forge/atomic-protected-ref-consumption-intent/v1\0";
const ATOMIC_CONSUMPTION_OBSERVATION_DOMAIN_V1: &[u8] =
    b"mycelix-forge/atomic-protected-ref-consumption-observation/v1\0";
const VERIFIED_ATOMIC_CONSUMPTION_DOMAIN_V1: &[u8] =
    b"mycelix-forge/provider-verified-atomic-protected-ref-consumption/v1\0";
const CONSUMPTION_REF_PREFIX: &str = "refs/mycelix/forge/consumed";

/// Positive non-authoritative transaction subject coupling one protected-ref
/// update with one durable per-request consumption marker.
///
/// The type has no `Deserialize` implementation and can only be constructed
/// from a positive FORGE-009D transition intent.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct AtomicProtectedRefConsumptionIntentV1 {
    version: ProtocolVersion,
    project: ProjectIdentity,
    proposal: ChangeProposalId,
    protected_merge_request: ProtectedMergeRequestId,
    source_qualification: Digest,
    transition_intent: Digest,
    target_ref: RepositoryRef,
    expected_base: GitObjectId,
    proposed_revision: GitObjectId,
    consumption_marker_ref: RepositoryRef,
    consumption_marker_value: GitObjectId,
    merge_nonce: [u8; PROTECTED_MERGE_NONCE_LEN],
}

impl AtomicProtectedRefConsumptionIntentV1 {
    /// Derive an exact atomic target-ref + request-consumption transaction from
    /// the positive FORGE-009D transition intent.
    pub fn new(
        transition: &ProtectedRefTransitionIntentV1,
    ) -> Result<Self, ProtectedRefConsumptionError> {
        let algorithm = transition.protected_merge_request().commitment().algorithm();
        let transition_intent = transition.digest(algorithm)?;
        let marker_ref = consumption_marker_ref(transition.protected_merge_request())?;
        if &marker_ref == transition.target_ref() {
            return Err(ProtectedRefConsumptionError::MarkerAliasesTargetRef);
        }

        Self::from_parts(
            transition.project().clone(),
            transition.proposal().clone(),
            transition.protected_merge_request().clone(),
            transition.source_qualification().clone(),
            transition_intent,
            transition.target_ref().clone(),
            transition.expected_base().clone(),
            transition.proposed_revision().clone(),
            marker_ref,
            transition.proposed_revision().clone(),
            *transition.merge_nonce(),
        )
    }

    #[allow(clippy::too_many_arguments)]
    fn from_parts(
        project: ProjectIdentity,
        proposal: ChangeProposalId,
        protected_merge_request: ProtectedMergeRequestId,
        source_qualification: Digest,
        transition_intent: Digest,
        target_ref: RepositoryRef,
        expected_base: GitObjectId,
        proposed_revision: GitObjectId,
        consumption_marker_ref: RepositoryRef,
        consumption_marker_value: GitObjectId,
        merge_nonce: [u8; PROTECTED_MERGE_NONCE_LEN],
    ) -> Result<Self, ProtectedRefConsumptionError> {
        if expected_base == proposed_revision {
            return Err(ProtectedRefConsumptionError::NoOpTransition);
        }
        let expected_algorithm = expected_base.algorithm();
        for (field, object) in [
            ("proposed_revision", &proposed_revision),
            ("consumption_marker_value", &consumption_marker_value),
        ] {
            if object.algorithm() != expected_algorithm {
                return Err(ProtectedRefConsumptionError::GitObjectAlgorithmMismatch {
                    field,
                    expected: expected_algorithm,
                    actual: object.algorithm(),
                });
            }
        }
        if consumption_marker_value != proposed_revision {
            return Err(ProtectedRefConsumptionError::MarkerValueMismatch);
        }
        if consumption_marker_ref == target_ref {
            return Err(ProtectedRefConsumptionError::MarkerAliasesTargetRef);
        }
        let expected_marker = consumption_marker_ref_from_id(&protected_merge_request)?;
        if consumption_marker_ref != expected_marker {
            return Err(ProtectedRefConsumptionError::ConsumptionMarkerRefMismatch);
        }

        Ok(Self {
            version: ProtocolVersion::CURRENT,
            project,
            proposal,
            protected_merge_request,
            source_qualification,
            transition_intent,
            target_ref,
            expected_base,
            proposed_revision,
            consumption_marker_ref,
            consumption_marker_value,
            merge_nonce,
        })
    }

    /// Project containing the protected transition.
    pub fn project(&self) -> &ProjectIdentity {
        &self.project
    }

    /// Exact proposal associated with the transaction.
    pub fn proposal(&self) -> &ChangeProposalId {
        &self.proposal
    }

    /// Exact protected merge request being consumed.
    pub fn protected_merge_request(&self) -> &ProtectedMergeRequestId {
        &self.protected_merge_request
    }

    /// Exact FORGE-009B source qualification inherited through FORGE-009D.
    pub fn source_qualification(&self) -> &Digest {
        &self.source_qualification
    }

    /// Exact FORGE-009D transition intent commitment.
    pub fn transition_intent(&self) -> &Digest {
        &self.transition_intent
    }

    /// Protected ref updated by the transaction.
    pub fn target_ref(&self) -> &RepositoryRef {
        &self.target_ref
    }

    /// Required current value of the protected ref.
    pub fn expected_base(&self) -> &GitObjectId {
        &self.expected_base
    }

    /// New value requested for the protected ref.
    pub fn proposed_revision(&self) -> &GitObjectId {
        &self.proposed_revision
    }

    /// Canonical durable marker ref for this exact protected merge request.
    pub fn consumption_marker_ref(&self) -> &RepositoryRef {
        &self.consumption_marker_ref
    }

    /// Object stored in the consumption marker after successful transaction.
    pub fn consumption_marker_value(&self) -> &GitObjectId {
        &self.consumption_marker_value
    }

    /// Exact merge-attempt nonce inherited from the request.
    pub const fn merge_nonce(&self) -> &[u8; PROTECTED_MERGE_NONCE_LEN] {
        &self.merge_nonce
    }

    /// Canonical v1 bytes for this atomic multi-ref transaction subject.
    pub fn canonical_bytes(&self) -> Result<Vec<u8>, ProtectedRefConsumptionError> {
        ensure_v1(self.version)?;
        let mut out = Vec::new();
        out.extend_from_slice(ATOMIC_CONSUMPTION_INTENT_DOMAIN_V1);
        out.extend_from_slice(&self.version.get().to_be_bytes());
        push_project_identity(&mut out, &self.project)?;
        push_digest(&mut out, self.proposal.commitment())?;
        push_digest(&mut out, self.protected_merge_request.commitment())?;
        push_digest(&mut out, &self.source_qualification)?;
        push_digest(&mut out, &self.transition_intent)?;
        push_ref(&mut out, &self.target_ref)?;
        push_git_object(&mut out, &self.expected_base)?;
        push_git_object(&mut out, &self.proposed_revision)?;
        push_ref(&mut out, &self.consumption_marker_ref)?;
        push_git_object(&mut out, &self.consumption_marker_value)?;
        out.extend_from_slice(&self.merge_nonce);
        Ok(out)
    }

    /// Stable commitment to the exact atomic transaction intent.
    pub fn digest(
        &self,
        algorithm: DigestAlgorithm,
    ) -> Result<Digest, ProtectedRefConsumptionError> {
        Ok(Digest::of_bytes(algorithm, &self.canonical_bytes()?))
    }
}

/// Provider-reported outcome for the atomic target-ref + marker transaction.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum AtomicProtectedRefConsumptionOutcomeV1 {
    /// Provider claims both ref modifications committed in one transaction.
    #[serde(rename = "applied")]
    Applied,
    /// Protected target ref no longer matched the expected base.
    #[serde(rename = "rejected-stale-target")]
    RejectedStaleTarget,
    /// Consumption marker already existed, indicating this request was already
    /// consumed or otherwise reserved.
    #[serde(rename = "rejected-already-consumed")]
    RejectedAlreadyConsumed,
    /// Provider rejected the transaction for another reason.
    #[serde(rename = "rejected")]
    Rejected,
}

impl AtomicProtectedRefConsumptionOutcomeV1 {
    const fn code(self) -> u8 {
        match self {
            Self::Applied => 1,
            Self::RejectedStaleTarget => 2,
            Self::RejectedAlreadyConsumed => 3,
            Self::Rejected => 4,
        }
    }
}

/// Raw provider observation for one atomic target-ref + consumption-marker
/// transaction.
///
/// This type is deserializable and non-authoritative until accepted by an
/// independent [`AtomicProtectedRefTransactionVerifierV1`].
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct AtomicProtectedRefConsumptionObservationV1 {
    version: ProtocolVersion,
    provider: ProtectedRefCasProviderIdentityV1,
    atomic_intent: Digest,
    transaction_id: Digest,
    target_ref: RepositoryRef,
    target_expected_before: GitObjectId,
    target_observed_before: GitObjectId,
    target_requested_after: GitObjectId,
    target_observed_after: GitObjectId,
    consumption_marker_ref: RepositoryRef,
    consumption_marker_existed_before: bool,
    consumption_marker_requested_value: GitObjectId,
    consumption_marker_observed_after: GitObjectId,
    merge_nonce: [u8; PROTECTED_MERGE_NONCE_LEN],
    provider_receipt: Digest,
    outcome: AtomicProtectedRefConsumptionOutcomeV1,
}

impl AtomicProtectedRefConsumptionObservationV1 {
    /// Construct a raw provider observation.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        provider: ProtectedRefCasProviderIdentityV1,
        atomic_intent: Digest,
        transaction_id: Digest,
        target_ref: RepositoryRef,
        target_expected_before: GitObjectId,
        target_observed_before: GitObjectId,
        target_requested_after: GitObjectId,
        target_observed_after: GitObjectId,
        consumption_marker_ref: RepositoryRef,
        consumption_marker_existed_before: bool,
        consumption_marker_requested_value: GitObjectId,
        consumption_marker_observed_after: GitObjectId,
        merge_nonce: [u8; PROTECTED_MERGE_NONCE_LEN],
        provider_receipt: Digest,
        outcome: AtomicProtectedRefConsumptionOutcomeV1,
    ) -> Result<Self, ProtectedRefConsumptionError> {
        Self::from_parts(
            provider,
            atomic_intent,
            transaction_id,
            target_ref,
            target_expected_before,
            target_observed_before,
            target_requested_after,
            target_observed_after,
            consumption_marker_ref,
            consumption_marker_existed_before,
            consumption_marker_requested_value,
            consumption_marker_observed_after,
            merge_nonce,
            provider_receipt,
            outcome,
        )
    }

    #[allow(clippy::too_many_arguments)]
    fn from_parts(
        provider: ProtectedRefCasProviderIdentityV1,
        atomic_intent: Digest,
        transaction_id: Digest,
        target_ref: RepositoryRef,
        target_expected_before: GitObjectId,
        target_observed_before: GitObjectId,
        target_requested_after: GitObjectId,
        target_observed_after: GitObjectId,
        consumption_marker_ref: RepositoryRef,
        consumption_marker_existed_before: bool,
        consumption_marker_requested_value: GitObjectId,
        consumption_marker_observed_after: GitObjectId,
        merge_nonce: [u8; PROTECTED_MERGE_NONCE_LEN],
        provider_receipt: Digest,
        outcome: AtomicProtectedRefConsumptionOutcomeV1,
    ) -> Result<Self, ProtectedRefConsumptionError> {
        let expected_algorithm = target_expected_before.algorithm();
        for (field, object) in [
            ("target_observed_before", &target_observed_before),
            ("target_requested_after", &target_requested_after),
            ("target_observed_after", &target_observed_after),
            (
                "consumption_marker_requested_value",
                &consumption_marker_requested_value,
            ),
            (
                "consumption_marker_observed_after",
                &consumption_marker_observed_after,
            ),
        ] {
            if object.algorithm() != expected_algorithm {
                return Err(ProtectedRefConsumptionError::GitObjectAlgorithmMismatch {
                    field,
                    expected: expected_algorithm,
                    actual: object.algorithm(),
                });
            }
        }

        Ok(Self {
            version: ProtocolVersion::CURRENT,
            provider,
            atomic_intent,
            transaction_id,
            target_ref,
            target_expected_before,
            target_observed_before,
            target_requested_after,
            target_observed_after,
            consumption_marker_ref,
            consumption_marker_existed_before,
            consumption_marker_requested_value,
            consumption_marker_observed_after,
            merge_nonce,
            provider_receipt,
            outcome,
        })
    }

    /// Provider identity that emitted the observation.
    pub fn provider(&self) -> &ProtectedRefCasProviderIdentityV1 {
        &self.provider
    }

    /// Exact atomic transaction intent claimed by the provider.
    pub fn atomic_intent(&self) -> &Digest {
        &self.atomic_intent
    }

    /// Provider-specific transaction identifier.
    pub fn transaction_id(&self) -> &Digest {
        &self.transaction_id
    }

    /// Protected target ref.
    pub fn target_ref(&self) -> &RepositoryRef {
        &self.target_ref
    }

    /// Expected old target value passed to the transaction.
    pub fn target_expected_before(&self) -> &GitObjectId {
        &self.target_expected_before
    }

    /// Target value observed before/at transaction admission.
    pub fn target_observed_before(&self) -> &GitObjectId {
        &self.target_observed_before
    }

    /// Requested new target value.
    pub fn target_requested_after(&self) -> &GitObjectId {
        &self.target_requested_after
    }

    /// Target value observed after transaction completion.
    pub fn target_observed_after(&self) -> &GitObjectId {
        &self.target_observed_after
    }

    /// Exact durable consumption-marker ref.
    pub fn consumption_marker_ref(&self) -> &RepositoryRef {
        &self.consumption_marker_ref
    }

    /// Whether the provider observed the marker already existing before the
    /// transaction. Positive qualification requires `false`.
    pub const fn consumption_marker_existed_before(&self) -> bool {
        self.consumption_marker_existed_before
    }

    /// Marker value requested by the transaction.
    pub fn consumption_marker_requested_value(&self) -> &GitObjectId {
        &self.consumption_marker_requested_value
    }

    /// Marker value observed after transaction completion.
    pub fn consumption_marker_observed_after(&self) -> &GitObjectId {
        &self.consumption_marker_observed_after
    }

    /// Exact merge-attempt nonce carried into the transaction.
    pub const fn merge_nonce(&self) -> &[u8; PROTECTED_MERGE_NONCE_LEN] {
        &self.merge_nonce
    }

    /// Provider-native receipt commitment.
    pub fn provider_receipt(&self) -> &Digest {
        &self.provider_receipt
    }

    /// Claimed provider outcome.
    pub const fn outcome(&self) -> AtomicProtectedRefConsumptionOutcomeV1 {
        self.outcome
    }

    /// Canonical v1 observation bytes.
    pub fn canonical_bytes(&self) -> Result<Vec<u8>, ProtectedRefConsumptionError> {
        ensure_v1(self.version)?;
        let mut out = Vec::new();
        out.extend_from_slice(ATOMIC_CONSUMPTION_OBSERVATION_DOMAIN_V1);
        out.extend_from_slice(&self.version.get().to_be_bytes());
        push_bytes(&mut out, "provider_name", self.provider.name().as_bytes())?;
        push_bytes(
            &mut out,
            "provider_version",
            self.provider.version().as_bytes(),
        )?;
        push_digest(&mut out, &self.atomic_intent)?;
        push_digest(&mut out, &self.transaction_id)?;
        push_ref(&mut out, &self.target_ref)?;
        push_git_object(&mut out, &self.target_expected_before)?;
        push_git_object(&mut out, &self.target_observed_before)?;
        push_git_object(&mut out, &self.target_requested_after)?;
        push_git_object(&mut out, &self.target_observed_after)?;
        push_ref(&mut out, &self.consumption_marker_ref)?;
        out.push(u8::from(self.consumption_marker_existed_before));
        push_git_object(&mut out, &self.consumption_marker_requested_value)?;
        push_git_object(&mut out, &self.consumption_marker_observed_after)?;
        out.extend_from_slice(&self.merge_nonce);
        push_digest(&mut out, &self.provider_receipt)?;
        out.push(self.outcome.code());
        Ok(out)
    }

    /// Stable observation commitment.
    pub fn digest(
        &self,
        algorithm: DigestAlgorithm,
    ) -> Result<Digest, ProtectedRefConsumptionError> {
        Ok(Digest::of_bytes(algorithm, &self.canonical_bytes()?))
    }
}

impl<'de> Deserialize<'de> for AtomicProtectedRefConsumptionObservationV1 {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct WireObservation {
            version: ProtocolVersion,
            provider: ProtectedRefCasProviderIdentityV1,
            atomic_intent: Digest,
            transaction_id: Digest,
            target_ref: RepositoryRef,
            target_expected_before: GitObjectId,
            target_observed_before: GitObjectId,
            target_requested_after: GitObjectId,
            target_observed_after: GitObjectId,
            consumption_marker_ref: RepositoryRef,
            consumption_marker_existed_before: bool,
            consumption_marker_requested_value: GitObjectId,
            consumption_marker_observed_after: GitObjectId,
            merge_nonce: [u8; PROTECTED_MERGE_NONCE_LEN],
            provider_receipt: Digest,
            outcome: AtomicProtectedRefConsumptionOutcomeV1,
        }

        let wire = WireObservation::deserialize(deserializer)?;
        ensure_v1(wire.version).map_err(D::Error::custom)?;
        Self::from_parts(
            wire.provider,
            wire.atomic_intent,
            wire.transaction_id,
            wire.target_ref,
            wire.target_expected_before,
            wire.target_observed_before,
            wire.target_requested_after,
            wire.target_observed_after,
            wire.consumption_marker_ref,
            wire.consumption_marker_existed_before,
            wire.consumption_marker_requested_value,
            wire.consumption_marker_observed_after,
            wire.merge_nonce,
            wire.provider_receipt,
            wire.outcome,
        )
        .map_err(D::Error::custom)
    }
}

/// Provider-specific atomic transaction verification failure.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum AtomicProtectedRefTransactionVerifierErrorV1 {
    /// Provider-native transaction evidence was rejected.
    #[error("atomic protected-ref transaction verifier rejected provider evidence")]
    Rejected,
}

/// Independent verifier for provider-native atomic multi-ref transaction
/// evidence.
pub trait AtomicProtectedRefTransactionVerifierV1 {
    /// Stable provider implementation/profile expected by this verifier.
    fn identity(&self) -> ProtectedRefCasProviderIdentityV1;

    /// Verify that target-ref update and consumption-marker creation belonged
    /// to one atomic provider transaction.
    fn verify_atomic_transaction(
        &self,
        intent: &AtomicProtectedRefConsumptionIntentV1,
        observation: &AtomicProtectedRefConsumptionObservationV1,
    ) -> Result<Digest, AtomicProtectedRefTransactionVerifierErrorV1>;
}

/// Positive post-execution receipt proving, relative to one provider verifier,
/// that target CAS and durable request consumption committed in one transaction.
///
/// This type has no `Deserialize` implementation.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct ProviderVerifiedAtomicProtectedRefConsumptionReceiptV1 {
    project: ProjectIdentity,
    proposal: ChangeProposalId,
    protected_merge_request: ProtectedMergeRequestId,
    source_qualification: Digest,
    transition_intent: Digest,
    atomic_intent: Digest,
    provider: ProtectedRefCasProviderIdentityV1,
    transaction_id: Digest,
    target_ref: RepositoryRef,
    target_before: GitObjectId,
    target_after: GitObjectId,
    consumption_marker_ref: RepositoryRef,
    consumption_marker_value: GitObjectId,
    provider_receipt: Digest,
    verifier_evidence: Digest,
    evidence_commitment: Digest,
}

impl ProviderVerifiedAtomicProtectedRefConsumptionReceiptV1 {
    /// Project containing the consumed transition.
    pub fn project(&self) -> &ProjectIdentity {
        &self.project
    }

    /// Exact proposal associated with the consumed transition.
    pub fn proposal(&self) -> &ChangeProposalId {
        &self.proposal
    }

    /// Exact protected merge request durably consumed by the transaction.
    pub fn protected_merge_request(&self) -> &ProtectedMergeRequestId {
        &self.protected_merge_request
    }

    /// Exact source qualification bound into the transaction.
    pub fn source_qualification(&self) -> &Digest {
        &self.source_qualification
    }

    /// Exact FORGE-009D transition intent commitment.
    pub fn transition_intent(&self) -> &Digest {
        &self.transition_intent
    }

    /// Exact FORGE-009E atomic transaction intent commitment.
    pub fn atomic_intent(&self) -> &Digest {
        &self.atomic_intent
    }

    /// Provider identity whose transaction evidence was verified.
    pub fn provider(&self) -> &ProtectedRefCasProviderIdentityV1 {
        &self.provider
    }

    /// Provider-specific transaction identifier.
    pub fn transaction_id(&self) -> &Digest {
        &self.transaction_id
    }

    /// Protected target ref.
    pub fn target_ref(&self) -> &RepositoryRef {
        &self.target_ref
    }

    /// Exact old protected-ref value.
    pub fn target_before(&self) -> &GitObjectId {
        &self.target_before
    }

    /// Exact new protected-ref value.
    pub fn target_after(&self) -> &GitObjectId {
        &self.target_after
    }

    /// Durable per-request consumption marker.
    pub fn consumption_marker_ref(&self) -> &RepositoryRef {
        &self.consumption_marker_ref
    }

    /// Object recorded in the consumption marker.
    pub fn consumption_marker_value(&self) -> &GitObjectId {
        &self.consumption_marker_value
    }

    /// Provider-native receipt commitment.
    pub fn provider_receipt(&self) -> &Digest {
        &self.provider_receipt
    }

    /// Provider-verifier evidence commitment.
    pub fn verifier_evidence(&self) -> &Digest {
        &self.verifier_evidence
    }

    /// Aggregate positive transaction evidence commitment.
    pub fn evidence_commitment(&self) -> &Digest {
        &self.evidence_commitment
    }
}

/// Verify an atomic protected-ref + durable-consumption provider receipt.
pub fn verify_atomic_protected_ref_consumption_v1<V: AtomicProtectedRefTransactionVerifierV1>(
    intent: &AtomicProtectedRefConsumptionIntentV1,
    observation: &AtomicProtectedRefConsumptionObservationV1,
    verifier: &V,
) -> Result<ProviderVerifiedAtomicProtectedRefConsumptionReceiptV1, ProtectedRefConsumptionError>
{
    if observation.provider() != &verifier.identity() {
        return Err(ProtectedRefConsumptionError::ProviderIdentityMismatch);
    }
    let intent_algorithm = observation.atomic_intent().algorithm();
    let expected_intent = intent.digest(intent_algorithm)?;
    if observation.atomic_intent() != &expected_intent {
        return Err(ProtectedRefConsumptionError::AtomicIntentMismatch);
    }
    if observation.target_ref() != intent.target_ref() {
        return Err(ProtectedRefConsumptionError::TargetRefMismatch);
    }
    if observation.target_expected_before() != intent.expected_base() {
        return Err(ProtectedRefConsumptionError::TargetExpectedBeforeMismatch);
    }
    if observation.target_observed_before() != intent.expected_base() {
        return Err(ProtectedRefConsumptionError::TargetObservedBeforeMismatch);
    }
    if observation.target_requested_after() != intent.proposed_revision() {
        return Err(ProtectedRefConsumptionError::TargetRequestedAfterMismatch);
    }
    if observation.target_observed_after() != intent.proposed_revision() {
        return Err(ProtectedRefConsumptionError::TargetObservedAfterMismatch);
    }
    if observation.consumption_marker_ref() != intent.consumption_marker_ref() {
        return Err(ProtectedRefConsumptionError::ConsumptionMarkerRefMismatch);
    }
    if observation.consumption_marker_existed_before() {
        return Err(ProtectedRefConsumptionError::ConsumptionMarkerAlreadyExisted);
    }
    if observation.consumption_marker_requested_value() != intent.consumption_marker_value() {
        return Err(ProtectedRefConsumptionError::MarkerRequestedValueMismatch);
    }
    if observation.consumption_marker_observed_after() != intent.consumption_marker_value() {
        return Err(ProtectedRefConsumptionError::MarkerObservedValueMismatch);
    }
    if observation.merge_nonce() != intent.merge_nonce() {
        return Err(ProtectedRefConsumptionError::MergeNonceMismatch);
    }
    if observation.outcome() != AtomicProtectedRefConsumptionOutcomeV1::Applied {
        return Err(ProtectedRefConsumptionError::TransactionNotApplied(
            observation.outcome(),
        ));
    }

    let verifier_evidence = verifier.verify_atomic_transaction(intent, observation)?;
    let algorithm = verifier_evidence.algorithm();
    let atomic_intent = intent.digest(algorithm)?;
    let observation_digest = observation.digest(algorithm)?;

    let mut out = Vec::new();
    out.extend_from_slice(VERIFIED_ATOMIC_CONSUMPTION_DOMAIN_V1);
    out.extend_from_slice(&ProtocolVersion::CURRENT.get().to_be_bytes());
    push_project_identity(&mut out, intent.project())?;
    push_digest(&mut out, intent.proposal().commitment())?;
    push_digest(&mut out, intent.protected_merge_request().commitment())?;
    push_digest(&mut out, intent.source_qualification())?;
    push_digest(&mut out, intent.transition_intent())?;
    push_digest(&mut out, &atomic_intent)?;
    push_digest(&mut out, &observation_digest)?;
    push_digest(&mut out, observation.transaction_id())?;
    push_digest(&mut out, observation.provider_receipt())?;
    push_digest(&mut out, &verifier_evidence)?;
    let evidence_commitment = Digest::of_bytes(algorithm, &out);

    Ok(ProviderVerifiedAtomicProtectedRefConsumptionReceiptV1 {
        project: intent.project().clone(),
        proposal: intent.proposal().clone(),
        protected_merge_request: intent.protected_merge_request().clone(),
        source_qualification: intent.source_qualification().clone(),
        transition_intent: intent.transition_intent().clone(),
        atomic_intent,
        provider: observation.provider().clone(),
        transaction_id: observation.transaction_id().clone(),
        target_ref: intent.target_ref().clone(),
        target_before: intent.expected_base().clone(),
        target_after: intent.proposed_revision().clone(),
        consumption_marker_ref: intent.consumption_marker_ref().clone(),
        consumption_marker_value: intent.consumption_marker_value().clone(),
        provider_receipt: observation.provider_receipt().clone(),
        verifier_evidence,
        evidence_commitment,
    })
}

/// Deterministically derive the durable consumption-marker ref for one exact
/// protected merge request.
pub fn consumption_marker_ref(
    protected_merge_request: &ProtectedMergeRequestId,
) -> Result<RepositoryRef, ProtectedRefConsumptionError> {
    consumption_marker_ref_from_id(protected_merge_request)
}

fn consumption_marker_ref_from_id(
    protected_merge_request: &ProtectedMergeRequestId,
) -> Result<RepositoryRef, ProtectedRefConsumptionError> {
    let digest = protected_merge_request.commitment();
    let value = format!(
        "{}/{}/{}",
        CONSUMPTION_REF_PREFIX,
        digest.algorithm().id(),
        hex::encode(digest.as_bytes())
    );
    Ok(RepositoryRef::new(value)?)
}

fn ensure_v1(version: ProtocolVersion) -> Result<(), ProtectedRefConsumptionError> {
    if version.get() == CURRENT_PROTOCOL_VERSION {
        Ok(())
    } else {
        Err(ProtectedRefConsumptionError::UnsupportedProtocolVersion(
            version.get(),
        ))
    }
}

fn push_project_identity(
    out: &mut Vec<u8>,
    project: &ProjectIdentity,
) -> Result<(), ProtectedRefConsumptionError> {
    out.extend_from_slice(&project.version().get().to_be_bytes());
    push_digest(out, project.digest())
}

fn push_ref(out: &mut Vec<u8>, reference: &RepositoryRef) -> Result<(), ProtectedRefConsumptionError> {
    push_bytes(out, "repository_ref", reference.as_str().as_bytes())
}

fn push_git_object(
    out: &mut Vec<u8>,
    object: &GitObjectId,
) -> Result<(), ProtectedRefConsumptionError> {
    out.push(object.algorithm().code());
    push_bytes(out, "git_object", object.as_bytes())
}

fn push_digest(
    out: &mut Vec<u8>,
    digest: &Digest,
) -> Result<(), ProtectedRefConsumptionError> {
    push_bytes(out, "digest_algorithm", digest.algorithm().id().as_bytes())?;
    push_bytes(out, "digest", digest.as_bytes())
}

fn push_bytes(
    out: &mut Vec<u8>,
    field: &'static str,
    bytes: &[u8],
) -> Result<(), ProtectedRefConsumptionError> {
    let len = u32::try_from(bytes.len()).map_err(|_| {
        ProtectedRefConsumptionError::CanonicalFieldTooLarge {
            field,
            len: bytes.len(),
            max: u32::MAX as usize,
        }
    })?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(bytes);
    Ok(())
}

/// Atomic protected-ref consumption construction/verification failures.
#[derive(Debug, Error, PartialEq, Eq)]
pub enum ProtectedRefConsumptionError {
    /// Protected transition unexpectedly describes a no-op.
    #[error("atomic protected-ref transaction is a no-op")]
    NoOpTransition,
    /// A Git object field uses another object algorithm.
    #[error("field {field} uses {actual:?}, expected {expected:?}")]
    GitObjectAlgorithmMismatch {
        /// Field containing the mismatch.
        field: &'static str,
        /// Expected algorithm.
        expected: GitObjectAlgorithm,
        /// Actual algorithm.
        actual: GitObjectAlgorithm,
    },
    /// Consumption marker value must equal the proposed revision.
    #[error("consumption marker value does not match proposed revision")]
    MarkerValueMismatch,
    /// Canonical marker ref aliases the protected target ref.
    #[error("consumption marker ref aliases protected target ref")]
    MarkerAliasesTargetRef,
    /// Marker ref differs from the deterministic per-request marker.
    #[error("consumption marker ref mismatch")]
    ConsumptionMarkerRefMismatch,
    /// Provider identity differs from the supplied verifier.
    #[error("atomic protected-ref provider identity mismatch")]
    ProviderIdentityMismatch,
    /// Observation names another atomic transaction intent.
    #[error("atomic protected-ref intent mismatch")]
    AtomicIntentMismatch,
    /// Observation names another target ref.
    #[error("atomic protected-ref target ref mismatch")]
    TargetRefMismatch,
    /// Target CAS expected-old argument differs from the intent.
    #[error("atomic protected-ref expected old target mismatch")]
    TargetExpectedBeforeMismatch,
    /// Provider observed another target value before transaction.
    #[error("atomic protected-ref observed stale target")]
    TargetObservedBeforeMismatch,
    /// Target requested-new argument differs from the intent.
    #[error("atomic protected-ref requested new target mismatch")]
    TargetRequestedAfterMismatch,
    /// Provider observed another target value after transaction.
    #[error("atomic protected-ref observed new target mismatch")]
    TargetObservedAfterMismatch,
    /// Consumption marker already existed before transaction.
    #[error("protected merge request was already consumed")]
    ConsumptionMarkerAlreadyExisted,
    /// Requested marker value differs from the canonical intent.
    #[error("consumption marker requested value mismatch")]
    MarkerRequestedValueMismatch,
    /// Observed marker value differs after transaction.
    #[error("consumption marker observed value mismatch")]
    MarkerObservedValueMismatch,
    /// Observation carries another merge-attempt nonce.
    #[error("atomic protected-ref merge nonce mismatch")]
    MergeNonceMismatch,
    /// Provider did not report a committed atomic transaction.
    #[error("atomic protected-ref transaction was not applied: {0:?}")]
    TransactionNotApplied(AtomicProtectedRefConsumptionOutcomeV1),
    /// Unsupported protocol version.
    #[error("unsupported atomic protected-ref protocol version: {0}")]
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
    /// FORGE-009D transition construction/canonicalization failure.
    #[error(transparent)]
    Transition(#[from] ProtectedRefTransitionError),
    /// Repository-ref construction failed.
    #[error(transparent)]
    Repository(#[from] RepositoryVerificationError),
    /// Provider-specific atomic transaction verifier failed.
    #[error(transparent)]
    Verifier(#[from] AtomicProtectedRefTransactionVerifierErrorV1),
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

    fn marker_for(request: &ProtectedMergeRequestId) -> RepositoryRef {
        consumption_marker_ref(request).unwrap()
    }

    fn provider() -> ProtectedRefCasProviderIdentityV1 {
        ProtectedRefCasProviderIdentityV1::new("git-update-ref-transaction", "2.53.0").unwrap()
    }

    fn intent() -> AtomicProtectedRefConsumptionIntentV1 {
        let request = merge_request_id(0x20);
        AtomicProtectedRefConsumptionIntentV1::from_parts(
            project(),
            ChangeProposalId::new(digest(0x21)),
            request.clone(),
            digest(0x22),
            digest(0x23),
            RepositoryRef::new("refs/heads/main").unwrap(),
            git(0x30),
            git(0x31),
            marker_for(&request),
            git(0x31),
            [0x40; PROTECTED_MERGE_NONCE_LEN],
        )
        .unwrap()
    }

    fn observation(
        intent: &AtomicProtectedRefConsumptionIntentV1,
    ) -> AtomicProtectedRefConsumptionObservationV1 {
        AtomicProtectedRefConsumptionObservationV1::new(
            provider(),
            intent.digest(DigestAlgorithm::Sha256).unwrap(),
            digest(0x50),
            intent.target_ref().clone(),
            intent.expected_base().clone(),
            intent.expected_base().clone(),
            intent.proposed_revision().clone(),
            intent.proposed_revision().clone(),
            intent.consumption_marker_ref().clone(),
            false,
            intent.consumption_marker_value().clone(),
            intent.consumption_marker_value().clone(),
            *intent.merge_nonce(),
            digest(0x51),
            AtomicProtectedRefConsumptionOutcomeV1::Applied,
        )
        .unwrap()
    }

    struct ExactVerifier {
        calls: Cell<u32>,
    }

    impl AtomicProtectedRefTransactionVerifierV1 for ExactVerifier {
        fn identity(&self) -> ProtectedRefCasProviderIdentityV1 {
            provider()
        }

        fn verify_atomic_transaction(
            &self,
            _intent: &AtomicProtectedRefConsumptionIntentV1,
            observation: &AtomicProtectedRefConsumptionObservationV1,
        ) -> Result<Digest, AtomicProtectedRefTransactionVerifierErrorV1> {
            self.calls.set(self.calls.get() + 1);
            if observation.transaction_id() == &digest(0x50)
                && observation.provider_receipt() == &digest(0x51)
            {
                Ok(digest(0x60))
            } else {
                Err(AtomicProtectedRefTransactionVerifierErrorV1::Rejected)
            }
        }
    }

    #[test]
    fn marker_ref_is_deterministic_and_namespaced_by_digest_algorithm() {
        let request = merge_request_id(0x20);
        let marker = consumption_marker_ref(&request).unwrap();
        assert!(marker.as_str().starts_with("refs/mycelix/forge/consumed/sha256/"));
        assert_eq!(marker, consumption_marker_ref(&request).unwrap());
    }

    #[test]
    fn exact_atomic_target_and_marker_transaction_becomes_positive() {
        let intent = intent();
        let observation = observation(&intent);
        let verifier = ExactVerifier { calls: Cell::new(0) };
        let verified =
            verify_atomic_protected_ref_consumption_v1(&intent, &observation, &verifier).unwrap();

        assert_eq!(verified.target_before(), intent.expected_base());
        assert_eq!(verified.target_after(), intent.proposed_revision());
        assert_eq!(verified.consumption_marker_ref(), intent.consumption_marker_ref());
        assert_eq!(verifier.calls.get(), 1);
    }

    #[test]
    fn existing_consumption_marker_fails_before_provider_verification() {
        let intent = intent();
        let mut observation = observation(&intent);
        observation.consumption_marker_existed_before = true;
        let verifier = ExactVerifier { calls: Cell::new(0) };

        let error = verify_atomic_protected_ref_consumption_v1(
            &intent,
            &observation,
            &verifier,
        )
        .unwrap_err();
        assert_eq!(
            error,
            ProtectedRefConsumptionError::ConsumptionMarkerAlreadyExisted
        );
        assert_eq!(verifier.calls.get(), 0);
    }

    #[test]
    fn stale_target_fails_closed() {
        let intent = intent();
        let mut observation = observation(&intent);
        observation.target_observed_before = git(0x32);
        let verifier = ExactVerifier { calls: Cell::new(0) };

        let error = verify_atomic_protected_ref_consumption_v1(
            &intent,
            &observation,
            &verifier,
        )
        .unwrap_err();
        assert_eq!(
            error,
            ProtectedRefConsumptionError::TargetObservedBeforeMismatch
        );
        assert_eq!(verifier.calls.get(), 0);
    }

    #[test]
    fn marker_ref_substitution_fails_closed() {
        let intent = intent();
        let mut observation = observation(&intent);
        observation.consumption_marker_ref =
            RepositoryRef::new("refs/mycelix/forge/consumed/sha256/other").unwrap();
        let verifier = ExactVerifier { calls: Cell::new(0) };

        let error = verify_atomic_protected_ref_consumption_v1(
            &intent,
            &observation,
            &verifier,
        )
        .unwrap_err();
        assert_eq!(
            error,
            ProtectedRefConsumptionError::ConsumptionMarkerRefMismatch
        );
        assert_eq!(verifier.calls.get(), 0);
    }

    #[test]
    fn raw_atomic_observation_round_trip_remains_non_authoritative() {
        let intent = intent();
        let observation = observation(&intent);
        let bytes = serde_json::to_vec(&observation).unwrap();
        let decoded: AtomicProtectedRefConsumptionObservationV1 =
            serde_json::from_slice(&bytes).unwrap();
        assert_eq!(observation, decoded);
    }

    #[allow(dead_code)]
    fn constructor_requires_forge_009d_positive_intent(
        transition: &ProtectedRefTransitionIntentV1,
    ) {
        let _ = AtomicProtectedRefConsumptionIntentV1::new(transition);
    }
}
