use std::collections::{BTreeMap, BTreeSet};

use mycelix_business_core::{Digest32, ExecutionAttemptRef, ReferenceId};
use mycelix_finance_exact::AssetAmount;

use crate::{
    CanonicalEncodingError, SETTLEMENT_COMMITMENT_PROFILE_REVISION,
    evaluation_context_commitment, finality_profile_commitment,
};

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

impl ReversalModel {
    pub(crate) const fn canonical_tag(self) -> u8 {
        match self {
            Self::MayReverse => 0x00,
            Self::DeclaredTerminalByProfile => 0x01,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum FinalityProfileError {
    UnsupportedCommitmentProfile { expected: u16, actual: u16 },
    ZeroRevision,
    NoEvidenceRequirements,
    ZeroDistinctSources,
    ZeroObservationAge,
    Commitment(CanonicalEncodingError),
    DigestMismatch,
}

impl From<CanonicalEncodingError> for FinalityProfileError {
    fn from(value: CanonicalEncodingError) -> Self {
        Self::Commitment(value)
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FinalityProfile {
    pub(crate) commitment_profile_revision: u16,
    pub(crate) profile_ref: FinalityProfileRef,
    pub(crate) rail: ReferenceId,
    pub(crate) network: ReferenceId,
    pub(crate) required_evidence_kinds: BTreeSet<ReferenceId>,
    pub(crate) min_distinct_sources: u16,
    pub(crate) max_observation_age_ms: u64,
    pub(crate) reversal_model: ReversalModel,
}

impl FinalityProfile {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        id: ReferenceId,
        revision: u64,
        rail: ReferenceId,
        network: ReferenceId,
        required_evidence_kinds: BTreeSet<ReferenceId>,
        min_distinct_sources: u16,
        max_observation_age_ms: u64,
        reversal_model: ReversalModel,
    ) -> Result<Self, FinalityProfileError> {
        let mut profile = Self {
            commitment_profile_revision: SETTLEMENT_COMMITMENT_PROFILE_REVISION,
            profile_ref: FinalityProfileRef {
                id,
                revision,
                digest: Digest32([0_u8; 32]),
            },
            rail,
            network,
            required_evidence_kinds,
            min_distinct_sources,
            max_observation_age_ms,
            reversal_model,
        };
        profile.validate_shape()?;
        profile.profile_ref.digest = finality_profile_commitment(&profile)?;
        Ok(profile)
    }

    fn validate_shape(&self) -> Result<(), FinalityProfileError> {
        if self.commitment_profile_revision != SETTLEMENT_COMMITMENT_PROFILE_REVISION {
            return Err(FinalityProfileError::UnsupportedCommitmentProfile {
                expected: SETTLEMENT_COMMITMENT_PROFILE_REVISION,
                actual: self.commitment_profile_revision,
            });
        }
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

    pub fn validate(&self) -> Result<(), FinalityProfileError> {
        self.validate_shape()?;
        if finality_profile_commitment(self)? != self.profile_ref.digest {
            return Err(FinalityProfileError::DigestMismatch);
        }
        Ok(())
    }

    pub fn profile_ref(&self) -> &FinalityProfileRef {
        &self.profile_ref
    }

    pub fn rail(&self) -> &ReferenceId {
        &self.rail
    }

    pub fn network(&self) -> &ReferenceId {
        &self.network
    }

    pub fn required_evidence_kinds(&self) -> &BTreeSet<ReferenceId> {
        &self.required_evidence_kinds
    }

    pub fn min_distinct_sources(&self) -> u16 {
        self.min_distinct_sources
    }

    pub fn max_observation_age_ms(&self) -> u64 {
        self.max_observation_age_ms
    }

    pub fn reversal_model(&self) -> ReversalModel {
        self.reversal_model
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EvaluationContextClass {
    DeterministicSupplied,
    HistoricalReplay,
}

impl EvaluationContextClass {
    pub(crate) const fn canonical_tag(self) -> u8 {
        match self {
            Self::DeterministicSupplied => 0x00,
            Self::HistoricalReplay => 0x01,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum EvaluationContextError {
    UnsupportedCommitmentProfile { expected: u16, actual: u16 },
    ZeroTemporalProfileRevision,
    Commitment(CanonicalEncodingError),
    DigestMismatch,
}

impl From<CanonicalEncodingError> for EvaluationContextError {
    fn from(value: CanonicalEncodingError) -> Self {
        Self::Commitment(value)
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SettlementEvaluationContext {
    pub(crate) commitment_profile_revision: u16,
    pub(crate) class: EvaluationContextClass,
    pub(crate) evaluation_time_unix_ms: u64,
    pub(crate) temporal_profile_id: ReferenceId,
    pub(crate) temporal_profile_revision: u64,
    pub(crate) temporal_context_digest: Digest32,
    pub(crate) commitment: Digest32,
}

impl SettlementEvaluationContext {
    fn new(
        class: EvaluationContextClass,
        evaluation_time_unix_ms: u64,
        temporal_profile_id: ReferenceId,
        temporal_profile_revision: u64,
        temporal_context_digest: Digest32,
    ) -> Result<Self, EvaluationContextError> {
        if temporal_profile_revision == 0 {
            return Err(EvaluationContextError::ZeroTemporalProfileRevision);
        }
        let mut context = Self {
            commitment_profile_revision: SETTLEMENT_COMMITMENT_PROFILE_REVISION,
            class,
            evaluation_time_unix_ms,
            temporal_profile_id,
            temporal_profile_revision,
            temporal_context_digest,
            commitment: Digest32([0_u8; 32]),
        };
        context.commitment = evaluation_context_commitment(&context)?;
        Ok(context)
    }

    pub fn deterministic_supplied(
        evaluation_time_unix_ms: u64,
        temporal_profile_id: ReferenceId,
        temporal_profile_revision: u64,
        temporal_context_digest: Digest32,
    ) -> Result<Self, EvaluationContextError> {
        Self::new(
            EvaluationContextClass::DeterministicSupplied,
            evaluation_time_unix_ms,
            temporal_profile_id,
            temporal_profile_revision,
            temporal_context_digest,
        )
    }

    pub fn historical_replay(
        evaluation_time_unix_ms: u64,
        temporal_profile_id: ReferenceId,
        temporal_profile_revision: u64,
        temporal_context_digest: Digest32,
    ) -> Result<Self, EvaluationContextError> {
        Self::new(
            EvaluationContextClass::HistoricalReplay,
            evaluation_time_unix_ms,
            temporal_profile_id,
            temporal_profile_revision,
            temporal_context_digest,
        )
    }

    pub fn validate(&self) -> Result<(), EvaluationContextError> {
        if self.commitment_profile_revision != SETTLEMENT_COMMITMENT_PROFILE_REVISION {
            return Err(EvaluationContextError::UnsupportedCommitmentProfile {
                expected: SETTLEMENT_COMMITMENT_PROFILE_REVISION,
                actual: self.commitment_profile_revision,
            });
        }
        if self.temporal_profile_revision == 0 {
            return Err(EvaluationContextError::ZeroTemporalProfileRevision);
        }
        if evaluation_context_commitment(self)? != self.commitment {
            return Err(EvaluationContextError::DigestMismatch);
        }
        Ok(())
    }

    pub fn class(&self) -> EvaluationContextClass {
        self.class
    }

    pub fn evaluation_time_unix_ms(&self) -> u64 {
        self.evaluation_time_unix_ms
    }

    pub fn temporal_profile_id(&self) -> &ReferenceId {
        &self.temporal_profile_id
    }

    pub fn temporal_profile_revision(&self) -> u64 {
        self.temporal_profile_revision
    }

    pub fn temporal_context_digest(&self) -> Digest32 {
        self.temporal_context_digest
    }

    pub fn commitment(&self) -> Digest32 {
        self.commitment
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SettlementSubject {
    pub id: ReferenceId,
    /// Opaque commitment to the exact Finance effect being settled. FIN-ECO-002
    /// checks equality and commits it into evidence; it does not mint reservation
    /// or Business authority from this digest.
    pub financial_effect_commitment: Digest32,
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

impl ObservedSettlementState {
    pub(crate) const fn canonical_tag(self) -> u8 {
        match self {
            Self::Applied => 0x00,
            Self::Pending => 0x01,
            Self::Rejected => 0x02,
            Self::Unknown => 0x03,
            Self::Reversed => 0x04,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FinalityEvidence {
    pub evidence_id: ReferenceId,
    pub subject: ReferenceId,
    pub operation_id: ReferenceId,
    pub operation_revision: u64,
    pub observation_id: ReferenceId,
    pub kind: ReferenceId,
    pub source: ReferenceId,
    pub digest: Digest32,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SettlementObservation {
    pub observation_id: ReferenceId,
    pub subject: ReferenceId,
    pub financial_effect_commitment: Digest32,
    pub attempt: ExecutionAttemptRef,
    pub rail: ReferenceId,
    pub network: ReferenceId,
    pub operation_id: ReferenceId,
    pub revision: u64,
    /// Exact current amount represented by this operation at this revision.
    /// This is not an incremental delta.
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
    observation_commitments: BTreeSet<Digest32>,
}

impl QualifiedOperation {
    pub(crate) fn new(
        revision: u64,
        amount: AssetAmount,
        evidence_ids: BTreeSet<ReferenceId>,
        observation_commitments: BTreeSet<Digest32>,
    ) -> Self {
        Self {
            revision,
            amount,
            evidence_ids,
            observation_commitments,
        }
    }

    pub fn revision(&self) -> u64 {
        self.revision
    }

    pub fn amount(&self) -> &AssetAmount {
        &self.amount
    }

    pub fn evidence_ids(&self) -> &BTreeSet<ReferenceId> {
        &self.evidence_ids
    }

    pub fn observation_commitments(&self) -> &BTreeSet<Digest32> {
        &self.observation_commitments
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct QualifiedSettlement {
    subject: ReferenceId,
    financial_effect_commitment: Digest32,
    attempt: ExecutionAttemptRef,
    rail: ReferenceId,
    network: ReferenceId,
    amount: AssetAmount,
    profile: FinalityProfileRef,
    evidence_frontier: Digest32,
    evaluation_context_commitment: Digest32,
    evaluation_context_class: EvaluationContextClass,
    evaluated_at_unix_ms: u64,
    operations: BTreeMap<ReferenceId, QualifiedOperation>,
    reversal_model: ReversalModel,
}

impl QualifiedSettlement {
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn new(
        subject: ReferenceId,
        financial_effect_commitment: Digest32,
        attempt: ExecutionAttemptRef,
        rail: ReferenceId,
        network: ReferenceId,
        amount: AssetAmount,
        profile: FinalityProfileRef,
        evidence_frontier: Digest32,
        evaluation_context_commitment: Digest32,
        evaluation_context_class: EvaluationContextClass,
        evaluated_at_unix_ms: u64,
        operations: BTreeMap<ReferenceId, QualifiedOperation>,
        reversal_model: ReversalModel,
    ) -> Self {
        Self {
            subject,
            financial_effect_commitment,
            attempt,
            rail,
            network,
            amount,
            profile,
            evidence_frontier,
            evaluation_context_commitment,
            evaluation_context_class,
            evaluated_at_unix_ms,
            operations,
            reversal_model,
        }
    }

    pub fn subject(&self) -> &ReferenceId {
        &self.subject
    }

    pub fn financial_effect_commitment(&self) -> Digest32 {
        self.financial_effect_commitment
    }

    pub fn attempt(&self) -> &ExecutionAttemptRef {
        &self.attempt
    }

    pub fn rail(&self) -> &ReferenceId {
        &self.rail
    }

    pub fn network(&self) -> &ReferenceId {
        &self.network
    }

    pub fn amount(&self) -> &AssetAmount {
        &self.amount
    }

    pub fn profile(&self) -> &FinalityProfileRef {
        &self.profile
    }

    pub fn evidence_frontier(&self) -> Digest32 {
        self.evidence_frontier
    }

    pub fn evaluation_context_commitment(&self) -> Digest32 {
        self.evaluation_context_commitment
    }

    pub fn evaluation_context_class(&self) -> EvaluationContextClass {
        self.evaluation_context_class
    }

    pub fn evaluated_at_unix_ms(&self) -> u64 {
        self.evaluated_at_unix_ms
    }

    /// Compatibility spelling for downstream code that treats the evaluation
    /// instant as the qualification instant. This is not a trusted-current claim.
    pub fn qualified_at_unix_ms(&self) -> u64 {
        self.evaluated_at_unix_ms
    }

    pub fn operations(&self) -> &BTreeMap<ReferenceId, QualifiedOperation> {
        &self.operations
    }

    pub fn reversal_model(&self) -> ReversalModel {
        self.reversal_model
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SettlementQualificationInvalidation {
    subject: ReferenceId,
    financial_effect_commitment: Digest32,
    prior_profile: FinalityProfileRef,
    operation_id: ReferenceId,
    prior_revision: u64,
    invalidating_observation: ReferenceId,
    invalidating_observation_commitment: Digest32,
    invalidated_at_unix_ms: u64,
    evidence_ids: BTreeSet<ReferenceId>,
}

impl SettlementQualificationInvalidation {
    pub(crate) fn new(
        subject: ReferenceId,
        financial_effect_commitment: Digest32,
        prior_profile: FinalityProfileRef,
        operation_id: ReferenceId,
        prior_revision: u64,
        invalidating_observation: ReferenceId,
        invalidating_observation_commitment: Digest32,
        invalidated_at_unix_ms: u64,
        evidence_ids: BTreeSet<ReferenceId>,
    ) -> Self {
        Self {
            subject,
            financial_effect_commitment,
            prior_profile,
            operation_id,
            prior_revision,
            invalidating_observation,
            invalidating_observation_commitment,
            invalidated_at_unix_ms,
            evidence_ids,
        }
    }

    pub fn subject(&self) -> &ReferenceId {
        &self.subject
    }

    pub fn financial_effect_commitment(&self) -> Digest32 {
        self.financial_effect_commitment
    }

    pub fn prior_profile(&self) -> &FinalityProfileRef {
        &self.prior_profile
    }

    pub fn operation_id(&self) -> &ReferenceId {
        &self.operation_id
    }

    pub fn prior_revision(&self) -> u64 {
        self.prior_revision
    }

    pub fn invalidating_observation(&self) -> &ReferenceId {
        &self.invalidating_observation
    }

    pub fn invalidating_observation_commitment(&self) -> Digest32 {
        self.invalidating_observation_commitment
    }

    pub fn invalidated_at_unix_ms(&self) -> u64 {
        self.invalidated_at_unix_ms
    }

    pub fn evidence_ids(&self) -> &BTreeSet<ReferenceId> {
        &self.evidence_ids
    }
}
