// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Witness-qualified review-head checkpoints for Mycelix Forge.
//!
//! A provider completeness claim is not merge authority. This crate requires
//! project-authorized Forge witnesses to authenticate the exact completeness
//! statement, then evaluates the exact `Capability::Witness` threshold in the
//! proposal's authority epoch.
//!
//! ```text
//! EvidenceBoundReviewHeadSnapshotV1
//! + ClaimsCompleteForProposal
//! + exact witness statement
//! + ProjectPolicyTrustedXeniaAuthenticationV1(Witness, statement id)
//! + Witness eligibility
//! + distinct Witness threshold at one common observation time
//!     ↓
//! WitnessQualifiedReviewHeadSnapshotV1
//! ```
//!
//! The positive result is deliberately **witness-qualified**, not globally
//! current or globally complete. Holochain has no global consensus clock/state,
//! the quorum time remains caller supplied, and a later merge-finalization
//! theorem must decide whether this exact checkpoint is fresh enough to use.

#![forbid(unsafe_code)]
#![warn(missing_docs)]

use mycelix_forge_authentication_policy::{
    AuthenticationPolicyError, ProjectPolicyTrustedXeniaAuthenticationV1,
};
use mycelix_forge_authority::{
    AuthorityEpoch, AuthorityError, Capability, PrincipalId,
};
use mycelix_forge_core::{
    Digest, DigestAlgorithm, ProjectIdentity, ProtocolVersion, CURRENT_PROTOCOL_VERSION,
};
use mycelix_forge_proposal::{ChangeProposal, ChangeProposalId, ProposalError};
use mycelix_forge_review_head_snapshot::{
    EvidenceBoundReviewHeadSnapshotV1, ReviewHeadCoverageClaimV1, ReviewHeadSnapshotError,
};
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;
use thiserror::Error;

const WITNESS_STATEMENT_DOMAIN_V1: &[u8] =
    b"mycelix-forge/review-head-completeness-witness-statement/v1\0";
const WITNESS_ATTESTATION_DOMAIN_V1: &[u8] =
    b"mycelix-forge/review-head-completeness-witness-attestation/v1\0";
const WITNESS_QUORUM_DOMAIN_V1: &[u8] =
    b"mycelix-forge/witness-qualified-review-head-snapshot/v1\0";

/// Stable identity for one exact completeness-witness statement.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(transparent)]
pub struct CompletenessWitnessStatementId(Digest);

impl CompletenessWitnessStatementId {
    /// Construct from an already-derived commitment.
    pub fn new(commitment: Digest) -> Self {
        Self(commitment)
    }

    /// Algorithm-qualified statement commitment.
    pub fn commitment(&self) -> &Digest {
        &self.0
    }
}

/// One exact project witness statement over one exact evidence-bound review-head
/// snapshot.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ReviewHeadCompletenessWitnessStatementV1 {
    version: ProtocolVersion,
    snapshot_evidence: Digest,
    witness: PrincipalId,
    witness_context: Digest,
}

impl ReviewHeadCompletenessWitnessStatementV1 {
    /// Construct a witness statement only for a provider observation that
    /// explicitly claims proposal-wide completeness.
    pub fn new(
        snapshot: &EvidenceBoundReviewHeadSnapshotV1,
        witness: PrincipalId,
        witness_context: Digest,
    ) -> Result<Self, ReviewHeadWitnessError> {
        if snapshot.observation().coverage()
            != ReviewHeadCoverageClaimV1::ClaimsCompleteForProposal
        {
            return Err(ReviewHeadWitnessError::SnapshotDoesNotClaimCompleteness);
        }
        Ok(Self {
            version: ProtocolVersion::CURRENT,
            snapshot_evidence: snapshot.evidence_commitment().clone(),
            witness,
            witness_context,
        })
    }

    /// Exact evidence-bound snapshot being attested.
    pub fn snapshot_evidence(&self) -> &Digest {
        &self.snapshot_evidence
    }

    /// Forge principal making the completeness attestation.
    pub fn witness(&self) -> &PrincipalId {
        &self.witness
    }

    /// Immutable witness-specific audit/context commitment.
    pub fn witness_context(&self) -> &Digest {
        &self.witness_context
    }

    /// Canonical bytes for this exact statement.
    pub fn canonical_bytes(&self) -> Result<Vec<u8>, ReviewHeadWitnessError> {
        ensure_v1(self.version)?;
        let mut out = Vec::new();
        out.extend_from_slice(WITNESS_STATEMENT_DOMAIN_V1);
        out.extend_from_slice(&self.version.get().to_be_bytes());
        push_digest(&mut out, &self.snapshot_evidence)?;
        push_digest(&mut out, self.witness.commitment())?;
        push_digest(&mut out, &self.witness_context)?;
        Ok(out)
    }

    /// Stable identity for the exact witness statement.
    pub fn statement_id(
        &self,
        algorithm: DigestAlgorithm,
    ) -> Result<CompletenessWitnessStatementId, ReviewHeadWitnessError> {
        Ok(CompletenessWitnessStatementId::new(Digest::of_bytes(
            algorithm,
            &self.canonical_bytes()?,
        )))
    }
}

/// Positive result proving one exact project-authorized witness authenticated one
/// exact completeness statement and was structurally eligible for `Witness` at
/// the supplied observation time.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct WitnessedReviewHeadAttestationV1 {
    statement: ReviewHeadCompletenessWitnessStatementV1,
    observed_at_unix_ms: u64,
    trusted_authentication_evidence: Digest,
    verifier_identity: Digest,
    evidence_commitment: Digest,
}

impl WitnessedReviewHeadAttestationV1 {
    /// Exact witness statement.
    pub fn statement(&self) -> &ReviewHeadCompletenessWitnessStatementV1 {
        &self.statement
    }

    /// Caller-supplied time used for structural Witness eligibility.
    pub const fn observed_at_unix_ms(&self) -> u64 {
        self.observed_at_unix_ms
    }

    /// Exact project-policy-trusted authentication evidence.
    pub fn trusted_authentication_evidence(&self) -> &Digest {
        &self.trusted_authentication_evidence
    }

    /// Xenia verifier identity trusted by project policy for this attestation.
    pub fn verifier_identity(&self) -> &Digest {
        &self.verifier_identity
    }

    /// Aggregate witness-attestation evidence commitment.
    pub fn evidence_commitment(&self) -> &Digest {
        &self.evidence_commitment
    }
}

/// Bind one project-policy-trusted Xenia authentication to one exact
/// completeness-witness statement and re-check structural Witness eligibility.
pub fn bind_witnessed_review_head_attestation_v1(
    proposal: &ChangeProposal,
    snapshot: &EvidenceBoundReviewHeadSnapshotV1,
    statement: ReviewHeadCompletenessWitnessStatementV1,
    trusted_authentication: &ProjectPolicyTrustedXeniaAuthenticationV1,
    authority_epoch: &AuthorityEpoch,
    observed_at_unix_ms: u64,
) -> Result<WitnessedReviewHeadAttestationV1, ReviewHeadWitnessError> {
    validate_snapshot_context(proposal, snapshot)?;
    if snapshot.observation().coverage()
        != ReviewHeadCoverageClaimV1::ClaimsCompleteForProposal
    {
        return Err(ReviewHeadWitnessError::SnapshotDoesNotClaimCompleteness);
    }
    if statement.snapshot_evidence() != snapshot.evidence_commitment() {
        return Err(ReviewHeadWitnessError::StatementSnapshotMismatch);
    }

    if authority_epoch.project() != proposal.project() {
        return Err(ReviewHeadWitnessError::ProjectMismatch);
    }
    let expected_epoch = authority_epoch.digest(proposal.authority_epoch().algorithm())?;
    if proposal.authority_epoch() != &expected_epoch {
        return Err(ReviewHeadWitnessError::AuthorityEpochMismatch);
    }

    let authentication = trusted_authentication.authentication();
    let request = authentication.request();
    if request.project() != proposal.project() {
        return Err(ReviewHeadWitnessError::ProjectMismatch);
    }
    if request.authority_epoch() != &expected_epoch {
        return Err(ReviewHeadWitnessError::AuthorityEpochMismatch);
    }
    if request.capability() != Capability::Witness {
        return Err(ReviewHeadWitnessError::WrongAuthenticationCapability);
    }
    if request.principal() != statement.witness()
        || authentication.binding().forge_principal() != statement.witness()
    {
        return Err(ReviewHeadWitnessError::WitnessMismatch);
    }
    if trusted_authentication.project_policy() != proposal.project_policy() {
        return Err(ReviewHeadWitnessError::ProjectPolicyMismatch);
    }

    let expected_statement = statement.statement_id(request.action_subject().algorithm())?;
    if request.action_subject() != expected_statement.commitment() {
        return Err(ReviewHeadWitnessError::AuthenticationSubjectMismatch);
    }
    if !authority_epoch.is_principal_eligible(
        statement.witness(),
        Capability::Witness,
        observed_at_unix_ms,
    ) {
        return Err(ReviewHeadWitnessError::WitnessNotEligible);
    }

    let algorithm = trusted_authentication.evidence_commitment().algorithm();
    let mut out = Vec::new();
    out.extend_from_slice(WITNESS_ATTESTATION_DOMAIN_V1);
    out.extend_from_slice(&ProtocolVersion::CURRENT.get().to_be_bytes());
    push_digest(&mut out, snapshot.evidence_commitment())?;
    push_digest(&mut out, expected_statement.commitment())?;
    push_digest(
        &mut out,
        trusted_authentication.evidence_commitment(),
    )?;
    out.extend_from_slice(&observed_at_unix_ms.to_be_bytes());
    let evidence_commitment = Digest::of_bytes(algorithm, &out);

    Ok(WitnessedReviewHeadAttestationV1 {
        statement,
        observed_at_unix_ms,
        trusted_authentication_evidence: trusted_authentication.evidence_commitment().clone(),
        verifier_identity: trusted_authentication.verifier_identity().clone(),
        evidence_commitment,
    })
}

/// One normalized witness counted in a successful completeness quorum.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct CountedCompletenessWitnessV1 {
    witness: PrincipalId,
    statement: CompletenessWitnessStatementId,
    observed_at_unix_ms: u64,
    attestation_evidence: Digest,
}

impl CountedCompletenessWitnessV1 {
    /// Distinct witness principal counted by the quorum.
    pub fn witness(&self) -> &PrincipalId {
        &self.witness
    }

    /// Exact witness statement counted by the quorum.
    pub fn statement(&self) -> &CompletenessWitnessStatementId {
        &self.statement
    }

    /// Original structural witness-observation time.
    pub const fn observed_at_unix_ms(&self) -> u64 {
        self.observed_at_unix_ms
    }

    /// Exact per-witness positive evidence commitment.
    pub fn attestation_evidence(&self) -> &Digest {
        &self.attestation_evidence
    }
}

/// Positive result proving that a distinct set of project-authorized witnesses
/// satisfied the exact `Capability::Witness` threshold for one exact review-head
/// completeness checkpoint at one common supplied observation time.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct WitnessQualifiedReviewHeadSnapshotV1 {
    project: ProjectIdentity,
    proposal: ChangeProposalId,
    authority_epoch: Digest,
    project_policy: Digest,
    repository_policy_state: Digest,
    snapshot_evidence: Digest,
    provider_namespace: Digest,
    provider_checkpoint: Digest,
    quorum_observed_at_unix_ms: u64,
    threshold: u16,
    witnesses: Vec<CountedCompletenessWitnessV1>,
    evidence_commitment: Digest,
}

impl WitnessQualifiedReviewHeadSnapshotV1 {
    /// Project containing the exact proposal.
    pub fn project(&self) -> &ProjectIdentity {
        &self.project
    }

    /// Exact proposal whose review-head checkpoint was witnessed.
    pub fn proposal(&self) -> &ChangeProposalId {
        &self.proposal
    }

    /// Exact authority epoch used for Witness eligibility and threshold.
    pub fn authority_epoch(&self) -> &Digest {
        &self.authority_epoch
    }

    /// Exact project-policy state committed by the proposal.
    pub fn project_policy(&self) -> &Digest {
        &self.project_policy
    }

    /// Exact repository-policy state committed by the proposal.
    pub fn repository_policy_state(&self) -> &Digest {
        &self.repository_policy_state
    }

    /// Exact evidence-bound review-head snapshot.
    pub fn snapshot_evidence(&self) -> &Digest {
        &self.snapshot_evidence
    }

    /// Concrete collaboration-state provider namespace claimed by the snapshot.
    pub fn provider_namespace(&self) -> &Digest {
        &self.provider_namespace
    }

    /// Exact opaque provider checkpoint committed by the snapshot.
    pub fn provider_checkpoint(&self) -> &Digest {
        &self.provider_checkpoint
    }

    /// Common caller-supplied observation time used for quorum evaluation.
    pub const fn quorum_observed_at_unix_ms(&self) -> u64 {
        self.quorum_observed_at_unix_ms
    }

    /// Exact distinct Witness threshold required by the authority epoch.
    pub const fn threshold(&self) -> u16 {
        self.threshold
    }

    /// Canonically sorted witnesses counted by the quorum.
    pub fn witnesses(&self) -> &[CountedCompletenessWitnessV1] {
        &self.witnesses
    }

    /// Aggregate witness-quorum evidence commitment.
    pub fn evidence_commitment(&self) -> &Digest {
        &self.evidence_commitment
    }
}

/// Evaluate a distinct project Witness threshold over one exact review-head
/// completeness checkpoint.
///
/// Every counted witness must have authenticated its own exact statement before
/// the common quorum observation time and must still be structurally eligible
/// for `Capability::Witness` at that common time.
pub fn evaluate_review_head_witness_quorum_v1(
    proposal: &ChangeProposal,
    snapshot: &EvidenceBoundReviewHeadSnapshotV1,
    authority_epoch: &AuthorityEpoch,
    attestations: &[WitnessedReviewHeadAttestationV1],
    quorum_observed_at_unix_ms: u64,
) -> Result<WitnessQualifiedReviewHeadSnapshotV1, ReviewHeadWitnessError> {
    validate_snapshot_context(proposal, snapshot)?;
    if snapshot.observation().coverage()
        != ReviewHeadCoverageClaimV1::ClaimsCompleteForProposal
    {
        return Err(ReviewHeadWitnessError::SnapshotDoesNotClaimCompleteness);
    }
    if authority_epoch.project() != proposal.project() {
        return Err(ReviewHeadWitnessError::ProjectMismatch);
    }
    let expected_epoch = authority_epoch.digest(proposal.authority_epoch().algorithm())?;
    if proposal.authority_epoch() != &expected_epoch {
        return Err(ReviewHeadWitnessError::AuthorityEpochMismatch);
    }
    if !authority_epoch.is_valid_at(quorum_observed_at_unix_ms) {
        return Err(ReviewHeadWitnessError::AuthorityEpochNotValidAtQuorumTime);
    }
    let threshold = authority_epoch
        .threshold_for(Capability::Witness)
        .ok_or(ReviewHeadWitnessError::MissingWitnessThreshold)?;

    let mut seen = BTreeSet::new();
    let mut witnesses = Vec::with_capacity(attestations.len());
    for attestation in attestations {
        let statement = attestation.statement();
        if statement.snapshot_evidence() != snapshot.evidence_commitment() {
            return Err(ReviewHeadWitnessError::StatementSnapshotMismatch);
        }
        if attestation.observed_at_unix_ms() > quorum_observed_at_unix_ms {
            return Err(ReviewHeadWitnessError::WitnessObservedAfterQuorumTime(
                statement.witness().clone(),
            ));
        }
        if !authority_epoch.is_principal_eligible(
            statement.witness(),
            Capability::Witness,
            quorum_observed_at_unix_ms,
        ) {
            return Err(ReviewHeadWitnessError::WitnessNotEligibleAtQuorumTime(
                statement.witness().clone(),
            ));
        }
        if !seen.insert(statement.witness().clone()) {
            return Err(ReviewHeadWitnessError::DuplicateWitness(
                statement.witness().clone(),
            ));
        }
        witnesses.push(CountedCompletenessWitnessV1 {
            witness: statement.witness().clone(),
            statement: statement.statement_id(attestation.evidence_commitment().algorithm())?,
            observed_at_unix_ms: attestation.observed_at_unix_ms(),
            attestation_evidence: attestation.evidence_commitment().clone(),
        });
    }

    witnesses.sort_by(|a, b| a.witness.cmp(&b.witness));
    if witnesses.len() < usize::from(threshold) {
        return Err(ReviewHeadWitnessError::UnderWitnessThreshold {
            required: threshold,
            actual: witnesses.len(),
        });
    }

    let snapshot_claim = snapshot.snapshot();
    let proposal_id = proposal.proposal_id(snapshot_claim.proposal().commitment().algorithm())?;
    let algorithm = snapshot.evidence_commitment().algorithm();
    let count = u32::try_from(witnesses.len()).map_err(|_| {
        ReviewHeadWitnessError::CanonicalFieldTooLarge {
            field: "witnesses",
            len: witnesses.len(),
            max: u32::MAX as usize,
        }
    })?;
    let mut out = Vec::new();
    out.extend_from_slice(WITNESS_QUORUM_DOMAIN_V1);
    out.extend_from_slice(&ProtocolVersion::CURRENT.get().to_be_bytes());
    push_project_identity(&mut out, proposal.project())?;
    push_digest(&mut out, proposal_id.commitment())?;
    push_digest(&mut out, &expected_epoch)?;
    push_digest(&mut out, proposal.project_policy())?;
    push_digest(&mut out, proposal.repository_policy_state())?;
    push_digest(&mut out, snapshot.evidence_commitment())?;
    push_digest(&mut out, snapshot.observation().provider_namespace())?;
    push_digest(&mut out, snapshot_claim.provider_checkpoint())?;
    out.extend_from_slice(&quorum_observed_at_unix_ms.to_be_bytes());
    out.extend_from_slice(&threshold.to_be_bytes());
    out.extend_from_slice(&count.to_be_bytes());
    for witness in &witnesses {
        push_digest(&mut out, witness.witness.commitment())?;
        push_digest(&mut out, witness.statement.commitment())?;
        out.extend_from_slice(&witness.observed_at_unix_ms.to_be_bytes());
        push_digest(&mut out, &witness.attestation_evidence)?;
    }
    let evidence_commitment = Digest::of_bytes(algorithm, &out);

    Ok(WitnessQualifiedReviewHeadSnapshotV1 {
        project: proposal.project().clone(),
        proposal: proposal_id,
        authority_epoch: expected_epoch,
        project_policy: proposal.project_policy().clone(),
        repository_policy_state: proposal.repository_policy_state().clone(),
        snapshot_evidence: snapshot.evidence_commitment().clone(),
        provider_namespace: snapshot.observation().provider_namespace().clone(),
        provider_checkpoint: snapshot_claim.provider_checkpoint().clone(),
        quorum_observed_at_unix_ms,
        threshold,
        witnesses,
        evidence_commitment,
    })
}

fn validate_snapshot_context(
    proposal: &ChangeProposal,
    snapshot: &EvidenceBoundReviewHeadSnapshotV1,
) -> Result<(), ReviewHeadWitnessError> {
    let claim = snapshot.snapshot();
    if claim.project() != proposal.project() {
        return Err(ReviewHeadWitnessError::ProjectMismatch);
    }
    let expected_proposal = proposal.proposal_id(claim.proposal().commitment().algorithm())?;
    if claim.proposal() != &expected_proposal {
        return Err(ReviewHeadWitnessError::ProposalMismatch);
    }
    if claim.authority_epoch() != proposal.authority_epoch() {
        return Err(ReviewHeadWitnessError::AuthorityEpochMismatch);
    }
    if claim.project_policy() != proposal.project_policy() {
        return Err(ReviewHeadWitnessError::ProjectPolicyMismatch);
    }
    if claim.repository_policy_state() != proposal.repository_policy_state() {
        return Err(ReviewHeadWitnessError::RepositoryPolicyMismatch);
    }
    Ok(())
}

fn ensure_v1(version: ProtocolVersion) -> Result<(), ReviewHeadWitnessError> {
    if version.get() == CURRENT_PROTOCOL_VERSION {
        Ok(())
    } else {
        Err(ReviewHeadWitnessError::UnsupportedProtocolVersion(version.get()))
    }
}

fn push_project_identity(
    out: &mut Vec<u8>,
    project: &ProjectIdentity,
) -> Result<(), ReviewHeadWitnessError> {
    out.extend_from_slice(&project.version().get().to_be_bytes());
    push_digest(out, project.digest())
}

fn push_digest(out: &mut Vec<u8>, digest: &Digest) -> Result<(), ReviewHeadWitnessError> {
    push_bytes(out, "digest_algorithm", digest.algorithm().id().as_bytes())?;
    push_bytes(out, "digest", digest.as_bytes())
}

fn push_bytes(
    out: &mut Vec<u8>,
    field: &'static str,
    bytes: &[u8],
) -> Result<(), ReviewHeadWitnessError> {
    let len = u32::try_from(bytes.len()).map_err(|_| {
        ReviewHeadWitnessError::CanonicalFieldTooLarge {
            field,
            len: bytes.len(),
            max: u32::MAX as usize,
        }
    })?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(bytes);
    Ok(())
}

/// Witness-qualified review-head checkpoint failure.
#[derive(Debug, Error, PartialEq, Eq)]
pub enum ReviewHeadWitnessError {
    /// Unsupported witness protocol version.
    #[error("unsupported Forge review-head witness protocol version: {0}")]
    UnsupportedProtocolVersion(u16),
    /// Snapshot provider made only an observation, not a completeness claim.
    #[error("review-head snapshot does not claim proposal-wide completeness")]
    SnapshotDoesNotClaimCompleteness,
    /// Witness statement names another evidence-bound snapshot.
    #[error("witness statement does not name supplied review-head snapshot")]
    StatementSnapshotMismatch,
    /// Project contexts disagree.
    #[error("review-head witness project mismatch")]
    ProjectMismatch,
    /// Snapshot names another immutable proposal.
    #[error("review-head witness proposal mismatch")]
    ProposalMismatch,
    /// Authority epoch differs from exact proposal context.
    #[error("review-head witness authority epoch mismatch")]
    AuthorityEpochMismatch,
    /// Project policy differs from exact proposal context.
    #[error("review-head witness project policy mismatch")]
    ProjectPolicyMismatch,
    /// Repository policy differs from exact proposal context.
    #[error("review-head witness repository policy mismatch")]
    RepositoryPolicyMismatch,
    /// Authentication request is not scoped to `Capability::Witness`.
    #[error("review-head witness authentication is not scoped to Witness")]
    WrongAuthenticationCapability,
    /// Authentication principal differs from statement witness.
    #[error("review-head witness authentication principal mismatch")]
    WitnessMismatch,
    /// Authentication action subject differs from exact witness statement id.
    #[error("authentication action subject does not match witness statement")]
    AuthenticationSubjectMismatch,
    /// Witness was not structurally eligible at the attestation time.
    #[error("principal is not eligible for Witness at attestation time")]
    WitnessNotEligible,
    /// Authority epoch is not valid at the common quorum time.
    #[error("authority epoch is not valid at witness quorum observation time")]
    AuthorityEpochNotValidAtQuorumTime,
    /// Exact authority epoch has no Witness threshold.
    #[error("authority epoch has no Witness threshold")]
    MissingWitnessThreshold,
    /// Same witness principal appeared more than once.
    #[error("duplicate completeness witness: {0}")]
    DuplicateWitness(PrincipalId),
    /// A witness attestation is from after the claimed common quorum time.
    #[error("witness {0} was observed after quorum observation time")]
    WitnessObservedAfterQuorumTime(PrincipalId),
    /// Witness was not eligible at the common quorum time.
    #[error("witness {0} is not eligible at quorum observation time")]
    WitnessNotEligibleAtQuorumTime(PrincipalId),
    /// Distinct witness count is below the exact threshold.
    #[error("witness quorum below threshold: {actual} < {required}")]
    UnderWitnessThreshold {
        /// Required distinct witnesses.
        required: u16,
        /// Supplied distinct positive witnesses.
        actual: usize,
    },
    /// Canonical field exceeded the v1 length bound.
    #[error("canonical field {field} is too large: {len} > {max}")]
    CanonicalFieldTooLarge {
        /// Field name.
        field: &'static str,
        /// Observed length.
        len: usize,
        /// Maximum encodable length.
        max: usize,
    },
    /// Authority validation/canonicalization failure.
    #[error(transparent)]
    Authority(#[from] AuthorityError),
    /// Proposal identity/canonicalization failure.
    #[error(transparent)]
    Proposal(#[from] ProposalError),
    /// Snapshot validation/canonicalization failure.
    #[error(transparent)]
    Snapshot(#[from] ReviewHeadSnapshotError),
    /// Project-policy trusted authentication failure.
    #[error(transparent)]
    AuthenticationPolicy(#[from] AuthenticationPolicyError),
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
        bind_principal_authentication, EvidenceBoundPrincipalAuthentication,
        PrincipalAuthenticationRequest, PrincipalBinding,
    };
    use mycelix_forge_authentication_policy::bind_project_policy_trusted_xenia_authentication_v1;
    use mycelix_forge_authority::{
        AuthorityEpochParts, CapabilityRule, PrincipalGrant,
    };
    use mycelix_forge_core::{ProjectIdentitySeed, GENESIS_NONCE_LEN};
    use mycelix_forge_project_policy::{
        AuthenticationProviderTrustPolicyV1, ProjectPolicyStateV1,
        TrustedProviderVerifierV1,
    };
    use mycelix_forge_repository::{
        GitObjectAlgorithm, GitObjectId, RepositoryPolicyState, RepositoryRef,
    };
    use mycelix_forge_review_head_snapshot::{
        bind_review_head_snapshot_observation_v1, ReviewHeadCompletenessObservationV1,
        ReviewHeadSnapshotClaimV1,
    };
    use mycelix_forge_xenia::{
        xenia_challenge_commitment, xenia_key_lineage_commitment,
        xenia_operator_id_commitment, xenia_provider_namespace, XeniaHybridSuite,
        XeniaVerificationReceiptV1,
    };
    use mycelix_forge_xenia_provider::{
        provider_attestation_transcript_v1, verify_xenia_provider_authentication_v1,
        ProviderVerifiedXeniaAuthenticationV1, TrustedXeniaVerifierV1,
        XeniaProviderAttestedReceiptV1,
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
        fn new() -> Self {
            let seed: B32 = [0x82; 32].into();
            Self {
                ed: SigningKey::from_bytes(&[0x81; 32]),
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
            serde_json::from_slice(
                &serde_json::to_vec(&WireEnvelope {
                    receipt,
                    verifier_identity_commitment: trusted.identity_commitment().clone(),
                    ed25519_signature: self.ed.sign(&transcript).to_bytes().to_vec(),
                    ml_dsa_65_signature: ml_signature.encode().as_slice().to_vec(),
                })
                .unwrap(),
            )
            .unwrap()
        }
    }

    struct Context {
        provider: ProviderIdentity,
        authority: AuthorityEpoch,
        project_policy: ProjectPolicyStateV1,
        provider_trust: AuthenticationProviderTrustPolicyV1,
        proposal: ChangeProposal,
    }

    fn context() -> Context {
        let provider = ProviderIdentity::new();
        let provider_trust = AuthenticationProviderTrustPolicyV1::new(
            project(),
            vec![TrustedProviderVerifierV1::new(
                xenia_provider_namespace(),
                provider.trusted().identity_commitment().clone(),
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
            valid_until_unix_ms: Some(5_000),
            grants: vec![
                PrincipalGrant::new(principal(0x10), [Capability::ManageAuthority]).unwrap(),
                PrincipalGrant::new(principal(0x20), [Capability::Witness]).unwrap(),
                PrincipalGrant::new(principal(0x21), [Capability::Witness]).unwrap(),
                PrincipalGrant::new(principal(0x22), [Capability::Witness]).unwrap(),
            ],
            thresholds: vec![
                CapabilityRule::new(Capability::ManageAuthority, 1).unwrap(),
                CapabilityRule::new(Capability::Witness, 2).unwrap(),
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

    fn snapshot(
        context: &Context,
        coverage: ReviewHeadCoverageClaimV1,
    ) -> EvidenceBoundReviewHeadSnapshotV1 {
        let claim = ReviewHeadSnapshotClaimV1::new(
            &context.proposal,
            digest(0x60),
            vec![],
            DigestAlgorithm::Sha256,
        )
        .unwrap();
        let observation = ReviewHeadCompletenessObservationV1::new(
            claim.digest(DigestAlgorithm::Sha256).unwrap(),
            digest(0x61),
            coverage,
            digest(0x62),
            digest(0x63),
            digest(0x64),
        );
        bind_review_head_snapshot_observation_v1(claim, observation).unwrap()
    }

    struct RawAuth {
        authentication: EvidenceBoundPrincipalAuthentication,
        provider_verified: ProviderVerifiedXeniaAuthenticationV1,
    }

    fn raw_auth(
        context: &Context,
        witness: PrincipalId,
        action_subject: Digest,
        capability: Capability,
        marker: u8,
    ) -> RawAuth {
        let operator = xenia_operator_id_commitment(&format!("operator:witness-{marker}"))
            .unwrap();
        let lineage = xenia_key_lineage_commitment(
            &[marker.wrapping_add(1); 32],
            &[marker.wrapping_add(2); 64],
            None,
        )
        .unwrap();
        let binding = PrincipalBinding::new(
            xenia_provider_namespace(),
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
            action_subject,
            [marker; 32],
        );
        let receipt = XeniaVerificationReceiptV1::new(
            request.digest(DigestAlgorithm::Sha256).unwrap(),
            operator,
            lineage,
            xenia_challenge_commitment(request.challenge()),
            XeniaHybridSuite::Ed25519MlDsa65V1,
            digest(marker.wrapping_add(3)),
            digest(marker.wrapping_add(4)),
            digest(marker.wrapping_add(5)),
            1_797_000_000,
        );
        let trusted_verifier = context.provider.trusted();
        let envelope = context.provider.envelope(receipt);
        let provider_verified = verify_xenia_provider_authentication_v1(
            &trusted_verifier,
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
        RawAuth {
            authentication,
            provider_verified,
        }
    }

    fn attestation(
        context: &Context,
        snapshot: &EvidenceBoundReviewHeadSnapshotV1,
        witness_byte: u8,
        marker: u8,
        observed_at: u64,
        capability: Capability,
    ) -> Result<WitnessedReviewHeadAttestationV1, ReviewHeadWitnessError> {
        let witness = principal(witness_byte);
        let statement = ReviewHeadCompletenessWitnessStatementV1::new(
            snapshot,
            witness.clone(),
            digest(marker),
        )?;
        let statement_id = statement.statement_id(DigestAlgorithm::Sha256)?;
        let raw = raw_auth(
            context,
            witness,
            statement_id.commitment().clone(),
            capability,
            marker.wrapping_add(10),
        );
        let trusted = bind_project_policy_trusted_xenia_authentication_v1(
            &context.proposal,
            &context.project_policy,
            &context.provider_trust,
            &raw.authentication,
            &raw.provider_verified,
        )?;
        bind_witnessed_review_head_attestation_v1(
            &context.proposal,
            snapshot,
            statement,
            &trusted,
            &context.authority,
            observed_at,
        )
    }

    #[test]
    fn two_of_three_distinct_project_witnesses_qualify_checkpoint() {
        let context = context();
        let snapshot = snapshot(&context, ReviewHeadCoverageClaimV1::ClaimsCompleteForProposal);
        let a = attestation(
            &context,
            &snapshot,
            0x20,
            0x70,
            1_400,
            Capability::Witness,
        )
        .unwrap();
        let b = attestation(
            &context,
            &snapshot,
            0x21,
            0x71,
            1_450,
            Capability::Witness,
        )
        .unwrap();
        let result = evaluate_review_head_witness_quorum_v1(
            &context.proposal,
            &snapshot,
            &context.authority,
            &[b, a],
            1_500,
        )
        .unwrap();
        assert_eq!(result.threshold(), 2);
        assert_eq!(result.witnesses().len(), 2);
        assert!(result.witnesses()[0].witness() < result.witnesses()[1].witness());
    }

    #[test]
    fn observed_only_snapshot_cannot_be_witnessed_as_complete() {
        let context = context();
        let snapshot = snapshot(&context, ReviewHeadCoverageClaimV1::ObservedSetOnly);
        assert_eq!(
            ReviewHeadCompletenessWitnessStatementV1::new(
                &snapshot,
                principal(0x20),
                digest(0x70),
            )
            .unwrap_err(),
            ReviewHeadWitnessError::SnapshotDoesNotClaimCompleteness
        );
    }

    #[test]
    fn wrong_capability_authentication_cannot_become_witness_attestation() {
        let context = context();
        let snapshot = snapshot(&context, ReviewHeadCoverageClaimV1::ClaimsCompleteForProposal);
        assert_eq!(
            attestation(
                &context,
                &snapshot,
                0x20,
                0x72,
                1_400,
                Capability::ReviewSource,
            )
            .unwrap_err(),
            ReviewHeadWitnessError::WrongAuthenticationCapability
        );
    }

    #[test]
    fn duplicate_witness_never_counts_twice() {
        let context = context();
        let snapshot = snapshot(&context, ReviewHeadCoverageClaimV1::ClaimsCompleteForProposal);
        let a = attestation(
            &context,
            &snapshot,
            0x20,
            0x73,
            1_400,
            Capability::Witness,
        )
        .unwrap();
        assert_eq!(
            evaluate_review_head_witness_quorum_v1(
                &context.proposal,
                &snapshot,
                &context.authority,
                &[a.clone(), a],
                1_500,
            )
            .unwrap_err(),
            ReviewHeadWitnessError::DuplicateWitness(principal(0x20))
        );
    }

    #[test]
    fn future_attestation_cannot_be_counted_retroactively() {
        let context = context();
        let snapshot = snapshot(&context, ReviewHeadCoverageClaimV1::ClaimsCompleteForProposal);
        let a = attestation(
            &context,
            &snapshot,
            0x20,
            0x74,
            1_400,
            Capability::Witness,
        )
        .unwrap();
        let b = attestation(
            &context,
            &snapshot,
            0x21,
            0x75,
            1_600,
            Capability::Witness,
        )
        .unwrap();
        assert_eq!(
            evaluate_review_head_witness_quorum_v1(
                &context.proposal,
                &snapshot,
                &context.authority,
                &[a, b],
                1_500,
            )
            .unwrap_err(),
            ReviewHeadWitnessError::WitnessObservedAfterQuorumTime(principal(0x21))
        );
    }
}
