// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Explicit `MergeProtected` approval of one exact review-policy-satisfied
//! checkpoint as the review basis for a later protected source transition.
//!
//! The claim is deliberately narrower than "current reviews" or "finalized
//! truth":
//!
//! ```text
//! ReviewPolicySatisfiedCheckpointV1
//! + exact review-basis subject
//! + project-policy-trusted Xenia authentication(MergeProtected)
//! + MergeProtected eligibility
//! + distinct MergeProtected threshold
//!     -> MergeProtectedReviewBasisQuorumV1
//! ```
//!
//! A positive result proves that protected-merge authority explicitly selected
//! one exact historical checkpoint as its review basis. It does **not** prove
//! that no later review revision exists, that the checkpoint is globally
//! current, that the source transition is valid, or that a merge executed.

#![forbid(unsafe_code)]
#![warn(missing_docs)]

use mycelix_forge_authentication_policy::ProjectPolicyTrustedXeniaAuthenticationV1;
use mycelix_forge_authority::{AuthorityEpoch, AuthorityError, Capability, PrincipalId};
use mycelix_forge_core::{
    Digest, DigestAlgorithm, ProjectIdentity, ProtocolVersion, CURRENT_PROTOCOL_VERSION,
};
use mycelix_forge_proposal::ChangeProposalId;
use mycelix_forge_review_policy_satisfied_checkpoint::ReviewPolicySatisfiedCheckpointV1;
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;
use thiserror::Error;

const REVIEW_BASIS_SUBJECT_DOMAIN_V1: &[u8] = b"mycelix-forge/review-basis-subject/v1\0";
const MERGE_PROTECTED_STATEMENT_DOMAIN_V1: &[u8] =
    b"mycelix-forge/merge-protected-review-basis-statement/v1\0";
const MERGE_PROTECTED_ATTESTATION_DOMAIN_V1: &[u8] =
    b"mycelix-forge/merge-protected-review-basis-attestation/v1\0";
const MERGE_PROTECTED_QUORUM_DOMAIN_V1: &[u8] =
    b"mycelix-forge/merge-protected-review-basis-quorum/v1\0";

/// Portable, non-authoritative projection of one exact policy-satisfied review
/// checkpoint.
///
/// Deserializing this type does not grant authority. Protected authority exists
/// only after the exact subject is rejoined to a live
/// [`ReviewPolicySatisfiedCheckpointV1`] and passes `MergeProtected` quorum.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ReviewBasisSubjectV1 {
    version: ProtocolVersion,
    project: ProjectIdentity,
    proposal: ChangeProposalId,
    authority_epoch: Digest,
    project_policy: Digest,
    repository_policy_state: Digest,
    review_policy: Digest,
    review_policy_context: Digest,
    review_policy_satisfied_checkpoint: Digest,
    checkpoint_observed_at_unix_ms: u64,
}

impl ReviewBasisSubjectV1 {
    /// Project the exact portable subject from a positive policy-satisfied
    /// checkpoint.
    pub fn from_checkpoint(checkpoint: &ReviewPolicySatisfiedCheckpointV1) -> Self {
        Self {
            version: ProtocolVersion::CURRENT,
            project: checkpoint.project().clone(),
            proposal: checkpoint.proposal().clone(),
            authority_epoch: checkpoint.authority_epoch().clone(),
            project_policy: checkpoint.project_policy().clone(),
            repository_policy_state: checkpoint.repository_policy_state().clone(),
            review_policy: checkpoint.review_policy().clone(),
            review_policy_context: checkpoint.review_policy_context().clone(),
            review_policy_satisfied_checkpoint: checkpoint.evidence_commitment().clone(),
            checkpoint_observed_at_unix_ms: checkpoint.checkpoint_observed_at_unix_ms(),
        }
    }

    /// Project owning the review basis.
    pub fn project(&self) -> &ProjectIdentity {
        &self.project
    }

    /// Exact immutable proposal whose review basis is being selected.
    pub fn proposal(&self) -> &ChangeProposalId {
        &self.proposal
    }

    /// Exact authority epoch already bound by the checkpoint.
    pub fn authority_epoch(&self) -> &Digest {
        &self.authority_epoch
    }

    /// Exact project-policy state already bound by the checkpoint.
    pub fn project_policy(&self) -> &Digest {
        &self.project_policy
    }

    /// Exact repository-policy state already bound by the checkpoint.
    pub fn repository_policy_state(&self) -> &Digest {
        &self.repository_policy_state
    }

    /// Exact adopted review-policy commitment.
    pub fn review_policy(&self) -> &Digest {
        &self.review_policy
    }

    /// Exact proposal-review policy-context commitment.
    pub fn review_policy_context(&self) -> &Digest {
        &self.review_policy_context
    }

    /// Exact FORGE-007M positive checkpoint evidence commitment.
    pub fn review_policy_satisfied_checkpoint(&self) -> &Digest {
        &self.review_policy_satisfied_checkpoint
    }

    /// Structural observation time of the witnessed checkpoint.
    pub const fn checkpoint_observed_at_unix_ms(&self) -> u64 {
        self.checkpoint_observed_at_unix_ms
    }

    /// Canonical bytes for this exact portable review-basis subject.
    pub fn canonical_bytes(&self) -> Result<Vec<u8>, MergeProtectedReviewBasisError> {
        ensure_v1(self.version)?;
        let mut out = Vec::new();
        out.extend_from_slice(REVIEW_BASIS_SUBJECT_DOMAIN_V1);
        out.extend_from_slice(&self.version.get().to_be_bytes());
        push_project_identity(&mut out, &self.project)?;
        push_digest(&mut out, self.proposal.commitment())?;
        push_digest(&mut out, &self.authority_epoch)?;
        push_digest(&mut out, &self.project_policy)?;
        push_digest(&mut out, &self.repository_policy_state)?;
        push_digest(&mut out, &self.review_policy)?;
        push_digest(&mut out, &self.review_policy_context)?;
        push_digest(&mut out, &self.review_policy_satisfied_checkpoint)?;
        out.extend_from_slice(&self.checkpoint_observed_at_unix_ms.to_be_bytes());
        Ok(out)
    }

    /// Stable commitment to this exact review-basis subject.
    pub fn digest(
        &self,
        algorithm: DigestAlgorithm,
    ) -> Result<Digest, MergeProtectedReviewBasisError> {
        Ok(Digest::of_bytes(algorithm, &self.canonical_bytes()?))
    }
}

/// Stable identity for one exact `MergeProtected` review-basis statement.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(transparent)]
pub struct MergeProtectedReviewBasisStatementId(Digest);

impl MergeProtectedReviewBasisStatementId {
    /// Algorithm-qualified statement commitment.
    pub fn commitment(&self) -> &Digest {
        &self.0
    }
}

/// Exact statement asking protected-merge authority to select one review basis.
///
/// The opaque `finalization_context` can commit to human rationale, a release
/// train identifier, an operator request, or another immutable governance
/// context. It is evidence identity, not proof of correctness.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct MergeProtectedReviewBasisStatementV1 {
    version: ProtocolVersion,
    project: ProjectIdentity,
    proposal: ChangeProposalId,
    review_basis: Digest,
    finalization_context: Digest,
}

impl MergeProtectedReviewBasisStatementV1 {
    /// Construct a statement for one exact review-basis subject.
    pub fn new(
        subject: &ReviewBasisSubjectV1,
        finalization_context: Digest,
        commitment_algorithm: DigestAlgorithm,
    ) -> Result<Self, MergeProtectedReviewBasisError> {
        Ok(Self {
            version: ProtocolVersion::CURRENT,
            project: subject.project().clone(),
            proposal: subject.proposal().clone(),
            review_basis: subject.digest(commitment_algorithm)?,
            finalization_context,
        })
    }

    /// Exact project.
    pub fn project(&self) -> &ProjectIdentity {
        &self.project
    }

    /// Exact proposal.
    pub fn proposal(&self) -> &ChangeProposalId {
        &self.proposal
    }

    /// Exact review-basis subject commitment.
    pub fn review_basis(&self) -> &Digest {
        &self.review_basis
    }

    /// Opaque immutable finalization/governance context commitment.
    pub fn finalization_context(&self) -> &Digest {
        &self.finalization_context
    }

    /// Canonical statement bytes.
    pub fn canonical_bytes(&self) -> Result<Vec<u8>, MergeProtectedReviewBasisError> {
        ensure_v1(self.version)?;
        let mut out = Vec::new();
        out.extend_from_slice(MERGE_PROTECTED_STATEMENT_DOMAIN_V1);
        out.extend_from_slice(&self.version.get().to_be_bytes());
        push_project_identity(&mut out, &self.project)?;
        push_digest(&mut out, self.proposal.commitment())?;
        push_digest(&mut out, &self.review_basis)?;
        push_digest(&mut out, &self.finalization_context)?;
        Ok(out)
    }

    /// Stable statement id. The review-basis digest suite anchors the statement
    /// id suite so callers cannot silently choose a different identity algorithm.
    pub fn statement_id(
        &self,
    ) -> Result<MergeProtectedReviewBasisStatementId, MergeProtectedReviewBasisError> {
        Ok(MergeProtectedReviewBasisStatementId(Digest::of_bytes(
            self.review_basis.algorithm(),
            &self.canonical_bytes()?,
        )))
    }
}

/// Positive result proving one project-policy-trusted principal authenticated
/// one exact review-basis statement under `Capability::MergeProtected` and was
/// structurally eligible at the supplied observation time.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct MergeProtectedReviewBasisAttestationV1 {
    statement: MergeProtectedReviewBasisStatementId,
    principal: PrincipalId,
    observed_at_unix_ms: u64,
    trusted_authentication_evidence: Digest,
    verifier_identity: Digest,
    evidence_commitment: Digest,
}

impl MergeProtectedReviewBasisAttestationV1 {
    /// Exact review-basis statement authenticated by this principal.
    pub fn statement(&self) -> &MergeProtectedReviewBasisStatementId {
        &self.statement
    }

    /// Distinct `MergeProtected` principal.
    pub fn principal(&self) -> &PrincipalId {
        &self.principal
    }

    /// Caller-supplied structural observation time for this attestation.
    pub const fn observed_at_unix_ms(&self) -> u64 {
        self.observed_at_unix_ms
    }

    /// Exact project-policy-trusted Xenia authentication evidence.
    pub fn trusted_authentication_evidence(&self) -> &Digest {
        &self.trusted_authentication_evidence
    }

    /// Exact project-trusted Xenia verifier identity.
    pub fn verifier_identity(&self) -> &Digest {
        &self.verifier_identity
    }

    /// Aggregate attestation evidence commitment.
    pub fn evidence_commitment(&self) -> &Digest {
        &self.evidence_commitment
    }
}

/// Bind one project-policy-trusted Xenia authentication to the exact protected
/// review-basis statement.
pub fn bind_merge_protected_review_basis_attestation_v1(
    checkpoint: &ReviewPolicySatisfiedCheckpointV1,
    subject: &ReviewBasisSubjectV1,
    statement: &MergeProtectedReviewBasisStatementV1,
    trusted_authentication: &ProjectPolicyTrustedXeniaAuthenticationV1,
    authority_epoch: &AuthorityEpoch,
    observed_at_unix_ms: u64,
) -> Result<MergeProtectedReviewBasisAttestationV1, MergeProtectedReviewBasisError> {
    validate_subject_against_checkpoint(checkpoint, subject)?;
    validate_statement(subject, statement)?;
    validate_authority_epoch(subject, authority_epoch)?;

    if observed_at_unix_ms < subject.checkpoint_observed_at_unix_ms() {
        return Err(MergeProtectedReviewBasisError::AttestationBeforeCheckpoint {
            observed_at_unix_ms,
            checkpoint_observed_at_unix_ms: subject.checkpoint_observed_at_unix_ms(),
        });
    }

    let authentication = trusted_authentication.authentication();
    let request = authentication.request();
    if request.project() != subject.project() {
        return Err(MergeProtectedReviewBasisError::ProjectMismatch);
    }
    if request.authority_epoch() != subject.authority_epoch() {
        return Err(MergeProtectedReviewBasisError::AuthorityEpochMismatch);
    }
    if request.capability() != Capability::MergeProtected {
        return Err(MergeProtectedReviewBasisError::WrongAuthenticationCapability);
    }
    if request.principal() != authentication.binding().forge_principal() {
        return Err(MergeProtectedReviewBasisError::PrincipalMismatch);
    }
    if trusted_authentication.project_policy() != subject.project_policy() {
        return Err(MergeProtectedReviewBasisError::ProjectPolicyMismatch);
    }

    let expected_statement = statement.statement_id()?;
    if request.action_subject() != expected_statement.commitment() {
        return Err(MergeProtectedReviewBasisError::AuthenticationSubjectMismatch);
    }
    if !authority_epoch.is_valid_at(observed_at_unix_ms)
        || !authority_epoch.is_principal_eligible(
            request.principal(),
            Capability::MergeProtected,
            observed_at_unix_ms,
        )
    {
        return Err(MergeProtectedReviewBasisError::PrincipalNotEligible);
    }

    let algorithm = trusted_authentication.evidence_commitment().algorithm();
    let subject_digest = subject.digest(algorithm)?;
    let mut out = Vec::new();
    out.extend_from_slice(MERGE_PROTECTED_ATTESTATION_DOMAIN_V1);
    out.extend_from_slice(&ProtocolVersion::CURRENT.get().to_be_bytes());
    push_digest(&mut out, &subject_digest)?;
    push_digest(&mut out, expected_statement.commitment())?;
    push_digest(&mut out, request.principal().commitment())?;
    out.extend_from_slice(&observed_at_unix_ms.to_be_bytes());
    push_digest(&mut out, trusted_authentication.evidence_commitment())?;
    push_digest(&mut out, trusted_authentication.verifier_identity())?;
    let evidence_commitment = Digest::of_bytes(algorithm, &out);

    Ok(MergeProtectedReviewBasisAttestationV1 {
        statement: expected_statement,
        principal: request.principal().clone(),
        observed_at_unix_ms,
        trusted_authentication_evidence: trusted_authentication.evidence_commitment().clone(),
        verifier_identity: trusted_authentication.verifier_identity().clone(),
        evidence_commitment,
    })
}

/// One distinct `MergeProtected` principal counted by the review-basis quorum.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct CountedMergeProtectedPrincipalV1 {
    principal: PrincipalId,
    observed_at_unix_ms: u64,
    attestation_evidence: Digest,
}

impl CountedMergeProtectedPrincipalV1 {
    /// Counted protected-merge principal.
    pub fn principal(&self) -> &PrincipalId {
        &self.principal
    }

    /// Structural observation time retained from the attestation.
    pub const fn observed_at_unix_ms(&self) -> u64 {
        self.observed_at_unix_ms
    }

    /// Exact attestation evidence counted by the quorum.
    pub fn attestation_evidence(&self) -> &Digest {
        &self.attestation_evidence
    }
}

/// Positive result proving that the exact `MergeProtected` threshold explicitly
/// selected one exact policy-satisfied checkpoint as its review basis.
///
/// This is intentionally **not** named `CurrentReviews`, `FinalizedTruth`, or
/// `MergeAuthorization`.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct MergeProtectedReviewBasisQuorumV1 {
    project: ProjectIdentity,
    proposal: ChangeProposalId,
    authority_epoch: Digest,
    project_policy: Digest,
    repository_policy_state: Digest,
    review_policy: Digest,
    review_policy_context: Digest,
    review_policy_satisfied_checkpoint: Digest,
    review_basis: Digest,
    statement: MergeProtectedReviewBasisStatementId,
    finalization_context: Digest,
    checkpoint_observed_at_unix_ms: u64,
    quorum_observed_at_unix_ms: u64,
    threshold: u16,
    finalizers: Vec<CountedMergeProtectedPrincipalV1>,
    evidence_commitment: Digest,
}

impl MergeProtectedReviewBasisQuorumV1 {
    /// Project containing the proposal.
    pub fn project(&self) -> &ProjectIdentity {
        &self.project
    }

    /// Exact proposal whose review basis was selected.
    pub fn proposal(&self) -> &ChangeProposalId {
        &self.proposal
    }

    /// Exact authority epoch used for protected-merge authority.
    pub fn authority_epoch(&self) -> &Digest {
        &self.authority_epoch
    }

    /// Exact project-policy state governing provider trust.
    pub fn project_policy(&self) -> &Digest {
        &self.project_policy
    }

    /// Exact repository-policy state already bound by review satisfaction.
    pub fn repository_policy_state(&self) -> &Digest {
        &self.repository_policy_state
    }

    /// Exact adopted review-policy commitment.
    pub fn review_policy(&self) -> &Digest {
        &self.review_policy
    }

    /// Exact proposal-review policy-context commitment.
    pub fn review_policy_context(&self) -> &Digest {
        &self.review_policy_context
    }

    /// Exact FORGE-007M checkpoint evidence selected as review basis.
    pub fn review_policy_satisfied_checkpoint(&self) -> &Digest {
        &self.review_policy_satisfied_checkpoint
    }

    /// Exact portable review-basis subject commitment.
    pub fn review_basis(&self) -> &Digest {
        &self.review_basis
    }

    /// Exact statement accepted by protected-merge authority.
    pub fn statement(&self) -> &MergeProtectedReviewBasisStatementId {
        &self.statement
    }

    /// Opaque immutable finalization/governance context commitment.
    pub fn finalization_context(&self) -> &Digest {
        &self.finalization_context
    }

    /// Structural time of the selected review checkpoint.
    pub const fn checkpoint_observed_at_unix_ms(&self) -> u64 {
        self.checkpoint_observed_at_unix_ms
    }

    /// Common caller-supplied time used for `MergeProtected` quorum evaluation.
    pub const fn quorum_observed_at_unix_ms(&self) -> u64 {
        self.quorum_observed_at_unix_ms
    }

    /// Exact `MergeProtected` threshold from the authority epoch.
    pub const fn threshold(&self) -> u16 {
        self.threshold
    }

    /// Canonically sorted distinct principals counted by the quorum.
    pub fn finalizers(&self) -> &[CountedMergeProtectedPrincipalV1] {
        &self.finalizers
    }

    /// Aggregate protected-review-basis evidence commitment.
    pub fn evidence_commitment(&self) -> &Digest {
        &self.evidence_commitment
    }
}

/// Evaluate the exact `MergeProtected` threshold over one exact review-basis
/// statement.
pub fn evaluate_merge_protected_review_basis_quorum_v1(
    checkpoint: &ReviewPolicySatisfiedCheckpointV1,
    subject: &ReviewBasisSubjectV1,
    statement: &MergeProtectedReviewBasisStatementV1,
    authority_epoch: &AuthorityEpoch,
    attestations: &[MergeProtectedReviewBasisAttestationV1],
    quorum_observed_at_unix_ms: u64,
) -> Result<MergeProtectedReviewBasisQuorumV1, MergeProtectedReviewBasisError> {
    validate_subject_against_checkpoint(checkpoint, subject)?;
    evaluate_validated_quorum(
        subject,
        statement,
        authority_epoch,
        attestations,
        quorum_observed_at_unix_ms,
    )
}

fn evaluate_validated_quorum(
    subject: &ReviewBasisSubjectV1,
    statement: &MergeProtectedReviewBasisStatementV1,
    authority_epoch: &AuthorityEpoch,
    attestations: &[MergeProtectedReviewBasisAttestationV1],
    quorum_observed_at_unix_ms: u64,
) -> Result<MergeProtectedReviewBasisQuorumV1, MergeProtectedReviewBasisError> {
    validate_statement(subject, statement)?;
    validate_authority_epoch(subject, authority_epoch)?;

    if quorum_observed_at_unix_ms < subject.checkpoint_observed_at_unix_ms() {
        return Err(MergeProtectedReviewBasisError::QuorumBeforeCheckpoint);
    }
    if !authority_epoch.is_valid_at(quorum_observed_at_unix_ms) {
        return Err(MergeProtectedReviewBasisError::AuthorityEpochNotValidAtQuorumTime);
    }
    let threshold = authority_epoch
        .threshold_for(Capability::MergeProtected)
        .ok_or(MergeProtectedReviewBasisError::MissingMergeProtectedThreshold)?;

    let expected_statement = statement.statement_id()?;
    let mut seen = BTreeSet::new();
    let mut finalizers = Vec::with_capacity(attestations.len());
    for attestation in attestations {
        if attestation.statement() != &expected_statement {
            return Err(MergeProtectedReviewBasisError::AttestationStatementMismatch);
        }
        if attestation.observed_at_unix_ms() < subject.checkpoint_observed_at_unix_ms() {
            return Err(MergeProtectedReviewBasisError::AttestationBeforeCheckpoint {
                observed_at_unix_ms: attestation.observed_at_unix_ms(),
                checkpoint_observed_at_unix_ms: subject.checkpoint_observed_at_unix_ms(),
            });
        }
        if attestation.observed_at_unix_ms() > quorum_observed_at_unix_ms {
            return Err(MergeProtectedReviewBasisError::AttestationAfterQuorumTime(
                attestation.principal().clone(),
            ));
        }
        if !authority_epoch.is_principal_eligible(
            attestation.principal(),
            Capability::MergeProtected,
            quorum_observed_at_unix_ms,
        ) {
            return Err(MergeProtectedReviewBasisError::PrincipalNotEligibleAtQuorumTime(
                attestation.principal().clone(),
            ));
        }
        if !seen.insert(attestation.principal().clone()) {
            return Err(MergeProtectedReviewBasisError::DuplicatePrincipal(
                attestation.principal().clone(),
            ));
        }
        finalizers.push(CountedMergeProtectedPrincipalV1 {
            principal: attestation.principal().clone(),
            observed_at_unix_ms: attestation.observed_at_unix_ms(),
            attestation_evidence: attestation.evidence_commitment().clone(),
        });
    }

    finalizers.sort_by(|a, b| a.principal.cmp(&b.principal));
    if finalizers.len() < usize::from(threshold) {
        return Err(MergeProtectedReviewBasisError::UnderMergeProtectedThreshold {
            required: threshold,
            actual: finalizers.len(),
        });
    }

    let algorithm = statement.review_basis().algorithm();
    let count = u32::try_from(finalizers.len()).map_err(|_| {
        MergeProtectedReviewBasisError::CanonicalFieldTooLarge {
            field: "finalizers",
            len: finalizers.len(),
            max: u32::MAX as usize,
        }
    })?;
    let mut out = Vec::new();
    out.extend_from_slice(MERGE_PROTECTED_QUORUM_DOMAIN_V1);
    out.extend_from_slice(&ProtocolVersion::CURRENT.get().to_be_bytes());
    push_project_identity(&mut out, subject.project())?;
    push_digest(&mut out, subject.proposal().commitment())?;
    push_digest(&mut out, subject.authority_epoch())?;
    push_digest(&mut out, subject.project_policy())?;
    push_digest(&mut out, subject.repository_policy_state())?;
    push_digest(&mut out, subject.review_policy())?;
    push_digest(&mut out, subject.review_policy_context())?;
    push_digest(&mut out, subject.review_policy_satisfied_checkpoint())?;
    push_digest(&mut out, statement.review_basis())?;
    push_digest(&mut out, expected_statement.commitment())?;
    push_digest(&mut out, statement.finalization_context())?;
    out.extend_from_slice(&subject.checkpoint_observed_at_unix_ms().to_be_bytes());
    out.extend_from_slice(&quorum_observed_at_unix_ms.to_be_bytes());
    out.extend_from_slice(&threshold.to_be_bytes());
    out.extend_from_slice(&count.to_be_bytes());
    for finalizer in &finalizers {
        push_digest(&mut out, finalizer.principal.commitment())?;
        out.extend_from_slice(&finalizer.observed_at_unix_ms.to_be_bytes());
        push_digest(&mut out, &finalizer.attestation_evidence)?;
    }
    let evidence_commitment = Digest::of_bytes(algorithm, &out);

    Ok(MergeProtectedReviewBasisQuorumV1 {
        project: subject.project().clone(),
        proposal: subject.proposal().clone(),
        authority_epoch: subject.authority_epoch().clone(),
        project_policy: subject.project_policy().clone(),
        repository_policy_state: subject.repository_policy_state().clone(),
        review_policy: subject.review_policy().clone(),
        review_policy_context: subject.review_policy_context().clone(),
        review_policy_satisfied_checkpoint: subject.review_policy_satisfied_checkpoint().clone(),
        review_basis: statement.review_basis().clone(),
        statement: expected_statement,
        finalization_context: statement.finalization_context().clone(),
        checkpoint_observed_at_unix_ms: subject.checkpoint_observed_at_unix_ms(),
        quorum_observed_at_unix_ms,
        threshold,
        finalizers,
        evidence_commitment,
    })
}

fn validate_subject_against_checkpoint(
    checkpoint: &ReviewPolicySatisfiedCheckpointV1,
    subject: &ReviewBasisSubjectV1,
) -> Result<(), MergeProtectedReviewBasisError> {
    if subject.project() != checkpoint.project() {
        return Err(MergeProtectedReviewBasisError::ProjectMismatch);
    }
    if subject.proposal() != checkpoint.proposal() {
        return Err(MergeProtectedReviewBasisError::ProposalMismatch);
    }
    if subject.authority_epoch() != checkpoint.authority_epoch() {
        return Err(MergeProtectedReviewBasisError::AuthorityEpochMismatch);
    }
    if subject.project_policy() != checkpoint.project_policy() {
        return Err(MergeProtectedReviewBasisError::ProjectPolicyMismatch);
    }
    if subject.repository_policy_state() != checkpoint.repository_policy_state() {
        return Err(MergeProtectedReviewBasisError::RepositoryPolicyMismatch);
    }
    if subject.review_policy() != checkpoint.review_policy() {
        return Err(MergeProtectedReviewBasisError::ReviewPolicyMismatch);
    }
    if subject.review_policy_context() != checkpoint.review_policy_context() {
        return Err(MergeProtectedReviewBasisError::ReviewPolicyContextMismatch);
    }
    if subject.review_policy_satisfied_checkpoint() != checkpoint.evidence_commitment() {
        return Err(MergeProtectedReviewBasisError::CheckpointEvidenceMismatch);
    }
    if subject.checkpoint_observed_at_unix_ms() != checkpoint.checkpoint_observed_at_unix_ms() {
        return Err(MergeProtectedReviewBasisError::CheckpointTimeMismatch);
    }
    Ok(())
}

fn validate_statement(
    subject: &ReviewBasisSubjectV1,
    statement: &MergeProtectedReviewBasisStatementV1,
) -> Result<(), MergeProtectedReviewBasisError> {
    if statement.project() != subject.project() {
        return Err(MergeProtectedReviewBasisError::ProjectMismatch);
    }
    if statement.proposal() != subject.proposal() {
        return Err(MergeProtectedReviewBasisError::ProposalMismatch);
    }
    let expected_basis = subject.digest(statement.review_basis().algorithm())?;
    if statement.review_basis() != &expected_basis {
        return Err(MergeProtectedReviewBasisError::ReviewBasisMismatch);
    }
    Ok(())
}

fn validate_authority_epoch(
    subject: &ReviewBasisSubjectV1,
    authority_epoch: &AuthorityEpoch,
) -> Result<(), MergeProtectedReviewBasisError> {
    if authority_epoch.project() != subject.project() {
        return Err(MergeProtectedReviewBasisError::ProjectMismatch);
    }
    let expected_epoch = authority_epoch.digest(subject.authority_epoch().algorithm())?;
    if subject.authority_epoch() != &expected_epoch {
        return Err(MergeProtectedReviewBasisError::AuthorityEpochMismatch);
    }
    Ok(())
}

fn ensure_v1(version: ProtocolVersion) -> Result<(), MergeProtectedReviewBasisError> {
    if version.get() == CURRENT_PROTOCOL_VERSION {
        Ok(())
    } else {
        Err(MergeProtectedReviewBasisError::UnsupportedProtocolVersion(
            version.get(),
        ))
    }
}

fn push_project_identity(
    out: &mut Vec<u8>,
    project: &ProjectIdentity,
) -> Result<(), MergeProtectedReviewBasisError> {
    out.extend_from_slice(&project.version().get().to_be_bytes());
    push_digest(out, project.digest())
}

fn push_digest(
    out: &mut Vec<u8>,
    digest: &Digest,
) -> Result<(), MergeProtectedReviewBasisError> {
    push_bytes(out, "digest_algorithm", digest.algorithm().id().as_bytes())?;
    push_bytes(out, "digest", digest.as_bytes())
}

fn push_bytes(
    out: &mut Vec<u8>,
    field: &'static str,
    bytes: &[u8],
) -> Result<(), MergeProtectedReviewBasisError> {
    let len = u32::try_from(bytes.len()).map_err(|_| {
        MergeProtectedReviewBasisError::CanonicalFieldTooLarge {
            field,
            len: bytes.len(),
            max: u32::MAX as usize,
        }
    })?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(bytes);
    Ok(())
}

/// Protected review-basis construction/qualification failure.
#[derive(Debug, Error, PartialEq, Eq)]
pub enum MergeProtectedReviewBasisError {
    /// Project identities differ.
    #[error("merge-protected review basis project mismatch")]
    ProjectMismatch,
    /// Proposal identities differ.
    #[error("merge-protected review basis proposal mismatch")]
    ProposalMismatch,
    /// Authority epoch differs from the exact checkpoint subject.
    #[error("merge-protected review basis authority epoch mismatch")]
    AuthorityEpochMismatch,
    /// Project-policy state differs from the exact checkpoint subject.
    #[error("merge-protected review basis project-policy mismatch")]
    ProjectPolicyMismatch,
    /// Repository-policy state differs from the exact checkpoint subject.
    #[error("merge-protected review basis repository-policy mismatch")]
    RepositoryPolicyMismatch,
    /// Review-policy commitment differs from the exact checkpoint subject.
    #[error("merge-protected review basis review-policy mismatch")]
    ReviewPolicyMismatch,
    /// Review-policy context differs from the exact checkpoint subject.
    #[error("merge-protected review basis review-policy-context mismatch")]
    ReviewPolicyContextMismatch,
    /// Subject does not bind the exact policy-satisfied checkpoint evidence.
    #[error("merge-protected review basis checkpoint evidence mismatch")]
    CheckpointEvidenceMismatch,
    /// Subject and positive checkpoint have different structural times.
    #[error("merge-protected review basis checkpoint time mismatch")]
    CheckpointTimeMismatch,
    /// Statement names another review-basis subject commitment.
    #[error("merge-protected review-basis statement names another review basis")]
    ReviewBasisMismatch,
    /// Authentication request is not scoped to `MergeProtected`.
    #[error("review-basis authentication is not scoped to MergeProtected")]
    WrongAuthenticationCapability,
    /// Authentication request principal and provider binding disagree.
    #[error("review-basis authentication principal mismatch")]
    PrincipalMismatch,
    /// Authentication action subject differs from the exact finalization statement.
    #[error("authentication action subject does not match merge-protected review-basis statement")]
    AuthenticationSubjectMismatch,
    /// Principal is not structurally eligible for `MergeProtected`.
    #[error("principal is not eligible for MergeProtected")]
    PrincipalNotEligible,
    /// Attestation structurally predates the checkpoint it selects.
    #[error(
        "MergeProtected attestation predates checkpoint: attestation={observed_at_unix_ms}, checkpoint={checkpoint_observed_at_unix_ms}"
    )]
    AttestationBeforeCheckpoint {
        /// Caller-supplied attestation observation time.
        observed_at_unix_ms: u64,
        /// Structural checkpoint observation time.
        checkpoint_observed_at_unix_ms: u64,
    },
    /// Common protected-merge quorum time predates the selected checkpoint.
    #[error("MergeProtected quorum time predates selected review checkpoint")]
    QuorumBeforeCheckpoint,
    /// Authority epoch is not valid at common quorum time.
    #[error("authority epoch is not valid at MergeProtected quorum time")]
    AuthorityEpochNotValidAtQuorumTime,
    /// Exact authority epoch has no `MergeProtected` threshold.
    #[error("authority epoch has no MergeProtected threshold")]
    MissingMergeProtectedThreshold,
    /// Attestation names another finalization statement.
    #[error("MergeProtected attestation names another review-basis statement")]
    AttestationStatementMismatch,
    /// Same protected-merge principal appeared more than once.
    #[error("duplicate MergeProtected principal: {0}")]
    DuplicatePrincipal(PrincipalId),
    /// Attestation was observed after the claimed common quorum time.
    #[error("MergeProtected principal {0} was observed after quorum time")]
    AttestationAfterQuorumTime(PrincipalId),
    /// Principal is not eligible at the common quorum time.
    #[error("MergeProtected principal {0} is not eligible at quorum time")]
    PrincipalNotEligibleAtQuorumTime(PrincipalId),
    /// Distinct protected-merge principal count is below threshold.
    #[error("MergeProtected quorum below threshold: {actual} < {required}")]
    UnderMergeProtectedThreshold {
        /// Required distinct principals.
        required: u16,
        /// Supplied distinct positive principals.
        actual: usize,
    },
    /// Unsupported Forge protocol version.
    #[error("unsupported merge-protected review-basis protocol version: {0}")]
    UnsupportedProtocolVersion(u16),
    /// Canonical field exceeded the v1 encoding bound.
    #[error("canonical field {field} is too large: {len} > {max}")]
    CanonicalFieldTooLarge {
        /// Field name.
        field: &'static str,
        /// Observed field length.
        len: usize,
        /// Maximum encodable length.
        max: usize,
    },
    /// Authority validation/canonicalization failure.
    #[error(transparent)]
    Authority(#[from] AuthorityError),
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_forge_authority::{AuthorityEpochParts, CapabilityRule, PrincipalGrant};
    use mycelix_forge_core::{ProjectIdentitySeed, GENESIS_NONCE_LEN};

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

    fn subject() -> ReviewBasisSubjectV1 {
        ReviewBasisSubjectV1 {
            version: ProtocolVersion::CURRENT,
            project: project(),
            proposal: ChangeProposalId::new(digest(0x20)),
            authority_epoch: digest(0x21),
            project_policy: digest(0x22),
            repository_policy_state: digest(0x23),
            review_policy: digest(0x24),
            review_policy_context: digest(0x25),
            review_policy_satisfied_checkpoint: digest(0x26),
            checkpoint_observed_at_unix_ms: 2_000,
        }
    }

    fn authority() -> AuthorityEpoch {
        let epoch = AuthorityEpoch::new(AuthorityEpochParts {
            project: project(),
            sequence: 0,
            previous: None,
            valid_from_unix_ms: 1_000,
            valid_until_unix_ms: Some(5_000),
            grants: vec![
                PrincipalGrant::new(principal(0x10), [Capability::ManageAuthority]).unwrap(),
                PrincipalGrant::new(principal(0x30), [Capability::MergeProtected]).unwrap(),
                PrincipalGrant::new(principal(0x31), [Capability::MergeProtected]).unwrap(),
                PrincipalGrant::new(principal(0x32), [Capability::MergeProtected]).unwrap(),
            ],
            thresholds: vec![
                CapabilityRule::new(Capability::ManageAuthority, 1).unwrap(),
                CapabilityRule::new(Capability::MergeProtected, 2).unwrap(),
            ],
            revoked_principals: vec![],
        })
        .unwrap();

        let expected = epoch.digest(DigestAlgorithm::Sha256).unwrap();
        assert_ne!(expected, digest(0x21));
        epoch
    }

    fn subject_for(authority: &AuthorityEpoch) -> ReviewBasisSubjectV1 {
        let mut value = subject();
        value.authority_epoch = authority.digest(DigestAlgorithm::Sha256).unwrap();
        value
    }

    fn attestation(
        statement: &MergeProtectedReviewBasisStatementV1,
        principal: PrincipalId,
        observed_at_unix_ms: u64,
        marker: u8,
    ) -> MergeProtectedReviewBasisAttestationV1 {
        MergeProtectedReviewBasisAttestationV1 {
            statement: statement.statement_id().unwrap(),
            principal,
            observed_at_unix_ms,
            trusted_authentication_evidence: digest(marker),
            verifier_identity: digest(marker.wrapping_add(1)),
            evidence_commitment: digest(marker.wrapping_add(2)),
        }
    }

    #[test]
    fn finalization_context_changes_exact_statement_identity() {
        let subject = subject();
        let a = MergeProtectedReviewBasisStatementV1::new(
            &subject,
            digest(0x70),
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        let b = MergeProtectedReviewBasisStatementV1::new(
            &subject,
            digest(0x71),
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        assert_ne!(a.statement_id().unwrap(), b.statement_id().unwrap());
    }

    #[test]
    fn two_of_three_merge_protected_principals_select_review_basis() {
        let authority = authority();
        let subject = subject_for(&authority);
        let statement = MergeProtectedReviewBasisStatementV1::new(
            &subject,
            digest(0x70),
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        let a = attestation(&statement, principal(0x30), 2_100, 0x80);
        let b = attestation(&statement, principal(0x31), 2_200, 0x90);
        let result = evaluate_validated_quorum(
            &subject,
            &statement,
            &authority,
            &[b, a],
            2_300,
        )
        .unwrap();
        assert_eq!(result.threshold(), 2);
        assert_eq!(result.finalizers().len(), 2);
        assert!(result.finalizers()[0].principal() < result.finalizers()[1].principal());
    }

    #[test]
    fn duplicate_merge_protected_principal_never_counts_twice() {
        let authority = authority();
        let subject = subject_for(&authority);
        let statement = MergeProtectedReviewBasisStatementV1::new(
            &subject,
            digest(0x70),
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        let a = attestation(&statement, principal(0x30), 2_100, 0x80);
        assert_eq!(
            evaluate_validated_quorum(
                &subject,
                &statement,
                &authority,
                &[a.clone(), a],
                2_300,
            )
            .unwrap_err(),
            MergeProtectedReviewBasisError::DuplicatePrincipal(principal(0x30))
        );
    }

    #[test]
    fn attestation_cannot_precede_selected_checkpoint() {
        let authority = authority();
        let subject = subject_for(&authority);
        let statement = MergeProtectedReviewBasisStatementV1::new(
            &subject,
            digest(0x70),
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        let a = attestation(&statement, principal(0x30), 1_999, 0x80);
        let b = attestation(&statement, principal(0x31), 2_100, 0x90);
        assert_eq!(
            evaluate_validated_quorum(&subject, &statement, &authority, &[a, b], 2_300)
                .unwrap_err(),
            MergeProtectedReviewBasisError::AttestationBeforeCheckpoint {
                observed_at_unix_ms: 1_999,
                checkpoint_observed_at_unix_ms: 2_000,
            }
        );
    }

    #[test]
    fn future_attestation_cannot_be_counted_retroactively() {
        let authority = authority();
        let subject = subject_for(&authority);
        let statement = MergeProtectedReviewBasisStatementV1::new(
            &subject,
            digest(0x70),
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        let a = attestation(&statement, principal(0x30), 2_100, 0x80);
        let b = attestation(&statement, principal(0x31), 2_400, 0x90);
        assert_eq!(
            evaluate_validated_quorum(&subject, &statement, &authority, &[a, b], 2_300)
                .unwrap_err(),
            MergeProtectedReviewBasisError::AttestationAfterQuorumTime(principal(0x31))
        );
    }

    #[test]
    fn attestation_for_another_statement_is_rejected() {
        let authority = authority();
        let subject = subject_for(&authority);
        let statement = MergeProtectedReviewBasisStatementV1::new(
            &subject,
            digest(0x70),
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        let other = MergeProtectedReviewBasisStatementV1::new(
            &subject,
            digest(0x71),
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        let a = attestation(&statement, principal(0x30), 2_100, 0x80);
        let b = attestation(&other, principal(0x31), 2_200, 0x90);
        assert_eq!(
            evaluate_validated_quorum(&subject, &statement, &authority, &[a, b], 2_300)
                .unwrap_err(),
            MergeProtectedReviewBasisError::AttestationStatementMismatch
        );
    }

    #[test]
    fn under_threshold_review_basis_cannot_be_promoted() {
        let authority = authority();
        let subject = subject_for(&authority);
        let statement = MergeProtectedReviewBasisStatementV1::new(
            &subject,
            digest(0x70),
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        let a = attestation(&statement, principal(0x30), 2_100, 0x80);
        assert_eq!(
            evaluate_validated_quorum(&subject, &statement, &authority, &[a], 2_300)
                .unwrap_err(),
            MergeProtectedReviewBasisError::UnderMergeProtectedThreshold {
                required: 2,
                actual: 1,
            }
        );
    }
}
