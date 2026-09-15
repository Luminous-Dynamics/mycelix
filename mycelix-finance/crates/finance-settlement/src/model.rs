use std::collections::{BTreeMap, BTreeSet};

use mycelix_business_core::{Digest32, ExecutionAttemptRef, ReferenceId};
use mycelix_finance_exact::AssetAmount;

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FinalityProfileRef {
    pub id: ReferenceId,
    pub revision: u64,
    pub digest: Digest32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReversalModel {
    MayReverse,
    DeclaredTerminalByProfile,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FinalityProfile {
    pub profile_ref: FinalityProfileRef,
    pub rail: ReferenceId,
    pub network: ReferenceId,
    pub required_evidence_kinds: BTreeSet<ReferenceId>,
    pub min_distinct_sources: u16,
    pub max_observation_age_ms: u64,
    pub reversal_model: ReversalModel,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum FinalityProfileError {
    ZeroRevision,
    NoEvidenceRequirements,
    ZeroDistinctSources,
    ZeroObservationAge,
}

impl FinalityProfile {
    pub fn validate(&self) -> Result<(), FinalityProfileError> {
        if self.profile_ref.revision == 0 {
            return Err(FinalityProfileError::ZeroRevision);
        }
        if self.required_evidence_kinds.is_empty() {
            return Err(FinalityProfileError::NoEvidenceRequirements);
        }
        if self.min_distinct_sources == 0 {
            return Err(FinalityProfileError::ZeroDistinctSources);
        }
        if self.max_observation_age_ms == 0 {
            return Err(FinalityProfileError::ZeroObservationAge);
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SettlementSubject {
    pub id: ReferenceId,
    pub attempt: ExecutionAttemptRef,
    pub rail: ReferenceId,
    pub network: ReferenceId,
    pub amount: AssetAmount,
    pub required_profile: FinalityProfileRef,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ObservedSettlementState {
    Applied,
    Pending,
    Rejected,
    Unknown,
    Reversed,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FinalityEvidence {
    pub evidence_id: ReferenceId,
    pub subject: ReferenceId,
    pub operation_id: ReferenceId,
    pub kind: ReferenceId,
    pub source: ReferenceId,
    pub digest: Digest32,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SettlementObservation {
    pub observation_id: ReferenceId,
    pub subject: ReferenceId,
    pub attempt: ExecutionAttemptRef,
    pub rail: ReferenceId,
    pub network: ReferenceId,
    pub operation_id: ReferenceId,
    pub revision: u64,
    pub amount: AssetAmount,
    pub state: ObservedSettlementState,
    pub observed_at_unix_ms: u64,
    pub evidence: Vec<FinalityEvidence>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct QualifiedOperation {
    revision: u64,
    amount: AssetAmount,
    evidence_ids: BTreeSet<ReferenceId>,
}

impl QualifiedOperation {
    pub(crate) fn new(revision: u64, amount: AssetAmount, evidence_ids: BTreeSet<ReferenceId>) -> Self {
        Self { revision, amount, evidence_ids }
    }

    pub fn revision(&self) -> u64 { self.revision }
    pub fn amount(&self) -> &AssetAmount { &self.amount }
    pub fn evidence_ids(&self) -> &BTreeSet<ReferenceId> { &self.evidence_ids }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct QualifiedSettlement {
    subject: ReferenceId,
    attempt: ExecutionAttemptRef,
    rail: ReferenceId,
    network: ReferenceId,
    amount: AssetAmount,
    profile: FinalityProfileRef,
    evidence_frontier: Digest32,
    operations: BTreeMap<ReferenceId, QualifiedOperation>,
    qualified_at_unix_ms: u64,
    reversal_model: ReversalModel,
}

impl QualifiedSettlement {
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn new(
        subject: ReferenceId,
        attempt: ExecutionAttemptRef,
        rail: ReferenceId,
        network: ReferenceId,
        amount: AssetAmount,
        profile: FinalityProfileRef,
        evidence_frontier: Digest32,
        operations: BTreeMap<ReferenceId, QualifiedOperation>,
        qualified_at_unix_ms: u64,
        reversal_model: ReversalModel,
    ) -> Self {
        Self { subject, attempt, rail, network, amount, profile, evidence_frontier, operations, qualified_at_unix_ms, reversal_model }
    }

    pub fn subject(&self) -> &ReferenceId { &self.subject }
    pub fn attempt(&self) -> &ExecutionAttemptRef { &self.attempt }
    pub fn rail(&self) -> &ReferenceId { &self.rail }
    pub fn network(&self) -> &ReferenceId { &self.network }
    pub fn amount(&self) -> &AssetAmount { &self.amount }
    pub fn profile(&self) -> &FinalityProfileRef { &self.profile }
    pub fn evidence_frontier(&self) -> Digest32 { self.evidence_frontier }
    pub fn operations(&self) -> &BTreeMap<ReferenceId, QualifiedOperation> { &self.operations }
    pub fn qualified_at_unix_ms(&self) -> u64 { self.qualified_at_unix_ms }
    pub fn reversal_model(&self) -> ReversalModel { self.reversal_model }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SettlementQualificationInvalidation {
    subject: ReferenceId,
    prior_profile: FinalityProfileRef,
    operation_id: ReferenceId,
    prior_revision: u64,
    invalidating_observation: ReferenceId,
    invalidated_at_unix_ms: u64,
    evidence_ids: BTreeSet<ReferenceId>,
}

impl SettlementQualificationInvalidation {
    pub fn subject(&self) -> &ReferenceId { &self.subject }
    pub fn prior_profile(&self) -> &FinalityProfileRef { &self.prior_profile }
    pub fn operation_id(&self) -> &ReferenceId { &self.operation_id }
    pub fn prior_revision(&self) -> u64 { self.prior_revision }
    pub fn invalidating_observation(&self) -> &ReferenceId { &self.invalidating_observation }
    pub fn invalidated_at_unix_ms(&self) -> u64 { self.invalidated_at_unix_ms }
    pub fn evidence_ids(&self) -> &BTreeSet<ReferenceId> { &self.evidence_ids }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SettlementInvalidationError {
    SubjectMismatch,
    AttemptMismatch,
    RailMismatch,
    NetworkMismatch,
    AssetMismatch,
    OperationNotPreviouslyQualified,
    RevisionDidNotAdvance,
    ObservationDoesNotInvalidate,
    MissingEvidence,
    EvidenceSubjectMismatch,
    EvidenceOperationMismatch,
}

pub fn derive_invalidation(
    prior: &QualifiedSettlement,
    observation: &SettlementObservation,
) -> Result<SettlementQualificationInvalidation, SettlementInvalidationError> {
    if observation.subject != prior.subject { return Err(SettlementInvalidationError::SubjectMismatch); }
    if observation.attempt != prior.attempt { return Err(SettlementInvalidationError::AttemptMismatch); }
    if observation.rail != prior.rail { return Err(SettlementInvalidationError::RailMismatch); }
    if observation.network != prior.network { return Err(SettlementInvalidationError::NetworkMismatch); }
    if observation.amount.asset() != prior.amount.asset() { return Err(SettlementInvalidationError::AssetMismatch); }

    let prior_operation = prior.operations.get(&observation.operation_id)
        .ok_or(SettlementInvalidationError::OperationNotPreviouslyQualified)?;
    if observation.revision <= prior_operation.revision { return Err(SettlementInvalidationError::RevisionDidNotAdvance); }
    if observation.evidence.is_empty() { return Err(SettlementInvalidationError::MissingEvidence); }
    for evidence in &observation.evidence {
        if evidence.subject != prior.subject { return Err(SettlementInvalidationError::EvidenceSubjectMismatch); }
        if evidence.operation_id != observation.operation_id { return Err(SettlementInvalidationError::EvidenceOperationMismatch); }
    }

    let invalidates = observation.state != ObservedSettlementState::Applied || observation.amount != prior_operation.amount;
    if !invalidates { return Err(SettlementInvalidationError::ObservationDoesNotInvalidate); }

    Ok(SettlementQualificationInvalidation {
        subject: prior.subject.clone(),
        prior_profile: prior.profile.clone(),
        operation_id: observation.operation_id.clone(),
        prior_revision: prior_operation.revision,
        invalidating_observation: observation.observation_id.clone(),
        invalidated_at_unix_ms: observation.observed_at_unix_ms,
        evidence_ids: observation.evidence.iter().map(|evidence| evidence.evidence_id.clone()).collect(),
    })
}
