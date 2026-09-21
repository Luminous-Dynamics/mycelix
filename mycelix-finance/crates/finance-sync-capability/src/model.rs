use std::fmt;

use mycelix_finance_sync_graph::{BoundedText, Commitment32, SemanticProfileRefV1};
use serde::{Deserialize, Serialize};

pub const MAX_CAPABILITIES_PER_DIMENSION: usize = 16;
pub const MAX_PROFILE_REFS: usize = 32;

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum CapabilityError {
    EmptyCapabilityDimension,
    TooManyCapabilities,
    DuplicateCapability,
    ContradictoryCapabilityDimension,
    TooManyProfileRefs,
    DuplicateProfileRef,
    InvalidPreparedStateCombination,
    InvalidSynchronizationCombination,
    InvalidFinalityCombination,
    InvalidAtomicityCombination,
    InvalidIdempotencyProfile,
    InvalidTimingProfile,
    InvalidResourceProfile,
    InvalidProfileRevision,
    CanonicalLengthOverflow,
}

impl fmt::Display for CapabilityError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let message = match self {
            Self::EmptyCapabilityDimension => "capability dimension must be explicit and non-empty",
            Self::TooManyCapabilities => "capability dimension exceeds the v1 bound",
            Self::DuplicateCapability => "capability dimension contains a duplicate semantic capability",
            Self::ContradictoryCapabilityDimension => {
                "negative capability marker cannot coexist with positive capabilities"
            }
            Self::TooManyProfileRefs => "profile-reference collection exceeds the v1 bound",
            Self::DuplicateProfileRef => "profile-reference collection contains a duplicate",
            Self::InvalidPreparedStateCombination => {
                "prepared-state commit/cancel semantics require a qualified prepare primitive"
            }
            Self::InvalidSynchronizationCombination => {
                "external synchronization capability/profile declarations are inconsistent"
            }
            Self::InvalidFinalityCombination => {
                "finality evidence capability/profile declarations are inconsistent"
            }
            Self::InvalidAtomicityCombination => {
                "atomicity scope is inconsistent with declared commit/synchronization primitives"
            }
            Self::InvalidIdempotencyProfile => "idempotency semantics are internally inconsistent",
            Self::InvalidTimingProfile => "timing profile contains an invalid zero bound",
            Self::InvalidResourceProfile => "resource profile contains an invalid zero bound",
            Self::InvalidProfileRevision => "static rail-capability profile revision must be non-zero",
            Self::CanonicalLengthOverflow => "canonical encoding length exceeds u32",
        };
        f.write_str(message)
    }
}

impl std::error::Error for CapabilityError {}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CapacityLockCapabilityV1 {
    NoCapacityLock,
    ProviderHold,
    ProviderReserve,
    FundsLock,
    AssetLock,
    TransactionScopedLock,
}

impl CapacityLockCapabilityV1 {
    pub(crate) const fn canonical_tag(self) -> u8 {
        match self {
            Self::NoCapacityLock => 1,
            Self::ProviderHold => 2,
            Self::ProviderReserve => 3,
            Self::FundsLock => 4,
            Self::AssetLock => 5,
            Self::TransactionScopedLock => 6,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PrepareCapabilityV1 {
    NoPrepare,
    LocalPreValidationOnly,
    PreparedOperationHandle,
    ConditionalInstruction,
    AtomicTransactionPrepare,
}

impl PrepareCapabilityV1 {
    pub(crate) const fn canonical_tag(self) -> u8 {
        match self {
            Self::NoPrepare => 1,
            Self::LocalPreValidationOnly => 2,
            Self::PreparedOperationHandle => 3,
            Self::ConditionalInstruction => 4,
            Self::AtomicTransactionPrepare => 5,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CommitCapabilityV1 {
    NoCommitPrimitive,
    SubmitOnly,
    CommitAgainstPreparedState,
    AtomicCommitWithinSingleOwner,
    ExternallySynchronizedCommit,
}

impl CommitCapabilityV1 {
    pub(crate) const fn canonical_tag(self) -> u8 {
        match self {
            Self::NoCommitPrimitive => 1,
            Self::SubmitOnly => 2,
            Self::CommitAgainstPreparedState => 3,
            Self::AtomicCommitWithinSingleOwner => 4,
            Self::ExternallySynchronizedCommit => 5,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CancelCapabilityV1 {
    NotCancelable,
    CancelableBeforeSubmission,
    CancelableWhilePrepared,
    BestEffortCancel,
    GuaranteedAbortOfPreparedState,
}

impl CancelCapabilityV1 {
    pub(crate) const fn canonical_tag(self) -> u8 {
        match self {
            Self::NotCancelable => 1,
            Self::CancelableBeforeSubmission => 2,
            Self::CancelableWhilePrepared => 3,
            Self::BestEffortCancel => 4,
            Self::GuaranteedAbortOfPreparedState => 5,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum QueryCapabilityV1 {
    NoQueryPrimitive,
    ExactOperationState,
    OperationBySemanticIdentity,
    DurableCommitClassificationQuery,
    FinalityQuery,
}

impl QueryCapabilityV1 {
    pub(crate) const fn canonical_tag(self) -> u8 {
        match self {
            Self::NoQueryPrimitive => 1,
            Self::ExactOperationState => 2,
            Self::OperationBySemanticIdentity => 3,
            Self::DurableCommitClassificationQuery => 4,
            Self::FinalityQuery => 5,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EvidenceCapabilityV1 {
    NoStructuredEvidence,
    ProviderAcknowledgement,
    DefinitelyNotCommittedClassifier,
    DefinitelyCommittedClassifier,
    FinalityEvidenceProduction,
}

impl EvidenceCapabilityV1 {
    pub(crate) const fn canonical_tag(self) -> u8 {
        match self {
            Self::NoStructuredEvidence => 1,
            Self::ProviderAcknowledgement => 2,
            Self::DefinitelyNotCommittedClassifier => 3,
            Self::DefinitelyCommittedClassifier => 4,
            Self::FinalityEvidenceProduction => 5,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ReversalCapabilityV1 {
    NoProtocolReversal,
    ProviderReturnFlow,
    ChargebackPossible,
    LedgerReorgPossible,
    ExplicitCompensatingTransferOnly,
}

impl ReversalCapabilityV1 {
    pub(crate) const fn canonical_tag(self) -> u8 {
        match self {
            Self::NoProtocolReversal => 1,
            Self::ProviderReturnFlow => 2,
            Self::ChargebackPossible => 3,
            Self::LedgerReorgPossible => 4,
            Self::ExplicitCompensatingTransferOnly => 5,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AtomicityCapabilityV1 {
    NoMultiLegAtomicity,
    MultiInstructionSingleTransactionOwner,
    ExternalSynchronizationProtocol,
}

impl AtomicityCapabilityV1 {
    pub(crate) const fn canonical_tag(self) -> u8 {
        match self {
            Self::NoMultiLegAtomicity => 1,
            Self::MultiInstructionSingleTransactionOwner => 2,
            Self::ExternalSynchronizationProtocol => 3,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum IdempotencyMechanismV1 {
    NoGuarantee,
    ClientKeyBestEffort,
    ProviderKeyDedupWindow,
    ProviderKeyExactlyOnceWithinProfile,
    NativeSemanticOperationIdentity,
}

impl IdempotencyMechanismV1 {
    pub(crate) const fn canonical_tag(self) -> u8 {
        match self {
            Self::NoGuarantee => 1,
            Self::ClientKeyBestEffort => 2,
            Self::ProviderKeyDedupWindow => 3,
            Self::ProviderKeyExactlyOnceWithinProfile => 4,
            Self::NativeSemanticOperationIdentity => 5,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum IdempotencyKeyScopeV1 {
    None,
    Request,
    Account,
    RailNetwork,
    ProviderGlobal,
    NativeOperation,
}

impl IdempotencyKeyScopeV1 {
    pub(crate) const fn canonical_tag(self) -> u8 {
        match self {
            Self::None => 0,
            Self::Request => 1,
            Self::Account => 2,
            Self::RailNetwork => 3,
            Self::ProviderGlobal => 4,
            Self::NativeOperation => 5,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum IdempotencySemanticScopeV1 {
    None,
    OpaqueRequestBytes,
    ExactLeg,
    ExactEconomicEffect,
    NativeOperation,
}

impl IdempotencySemanticScopeV1 {
    pub(crate) const fn canonical_tag(self) -> u8 {
        match self {
            Self::None => 0,
            Self::OpaqueRequestBytes => 1,
            Self::ExactLeg => 2,
            Self::ExactEconomicEffect => 3,
            Self::NativeOperation => 4,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum RetentionHorizonV1 {
    NotGuaranteed,
    BoundedSeconds { seconds: u64 },
    ProfileBounded { profile: SemanticProfileRefV1 },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum UnknownOutcomeRetryV1 {
    NeverBlindRetry,
    QueryBeforeReplay,
    ReplaySameSemanticOperationWithinHorizon,
    ProviderNativeSafeReplay,
}

impl UnknownOutcomeRetryV1 {
    pub(crate) const fn canonical_tag(self) -> u8 {
        match self {
            Self::NeverBlindRetry => 1,
            Self::QueryBeforeReplay => 2,
            Self::ReplaySameSemanticOperationWithinHorizon => 3,
            Self::ProviderNativeSafeReplay => 4,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum IdempotencyCollisionBehaviorV1 {
    Undefined,
    ProviderRejectsSemanticMismatch,
    FailClosedOnSemanticMismatch,
    NativeIdentityCannotBeRepurposed,
}

impl IdempotencyCollisionBehaviorV1 {
    pub(crate) const fn canonical_tag(self) -> u8 {
        match self {
            Self::Undefined => 0,
            Self::ProviderRejectsSemanticMismatch => 1,
            Self::FailClosedOnSemanticMismatch => 2,
            Self::NativeIdentityCannotBeRepurposed => 3,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct IdempotencyProfileV1 {
    pub mechanism: IdempotencyMechanismV1,
    pub key_scope: IdempotencyKeyScopeV1,
    pub semantic_scope: IdempotencySemanticScopeV1,
    pub retention: RetentionHorizonV1,
    pub retry_after_unknown: UnknownOutcomeRetryV1,
    pub collision_behavior: IdempotencyCollisionBehaviorV1,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TimingProfileV1 {
    pub max_provider_deadline_ms: u64,
    pub max_prepare_lifetime_seconds: Option<u64>,
    pub min_query_poll_interval_ms: Option<u64>,
    pub max_query_attempts_per_window: Option<u32>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ResourceProfileV1 {
    pub max_request_bytes: u32,
    pub max_batch_items: u32,
    pub max_inflight_per_subject: u32,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RailCapabilityProfileInputV1 {
    pub adapter_profile: SemanticProfileRefV1,
    pub adapter_build_profile: SemanticProfileRefV1,
    pub provider_profile: SemanticProfileRefV1,
    pub rail: BoundedText,
    pub network: BoundedText,
    pub operation_profile: SemanticProfileRefV1,
    pub capacity_lock_capabilities: Vec<CapacityLockCapabilityV1>,
    pub prepare_capabilities: Vec<PrepareCapabilityV1>,
    pub commit_capabilities: Vec<CommitCapabilityV1>,
    pub cancel_capabilities: Vec<CancelCapabilityV1>,
    pub query_capabilities: Vec<QueryCapabilityV1>,
    pub evidence_capabilities: Vec<EvidenceCapabilityV1>,
    pub reversal_capabilities: Vec<ReversalCapabilityV1>,
    pub atomicity_capabilities: Vec<AtomicityCapabilityV1>,
    pub idempotency: IdempotencyProfileV1,
    pub producible_finality_profiles: Vec<SemanticProfileRefV1>,
    pub synchronization_profiles: Vec<SemanticProfileRefV1>,
    pub disclosure_profile: SemanticProfileRefV1,
    pub timing: TimingProfileV1,
    pub resources: ResourceProfileV1,
    pub profile_revision: u64,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct RailCapabilityProfileV1 {
    pub(crate) adapter_profile: SemanticProfileRefV1,
    pub(crate) adapter_build_profile: SemanticProfileRefV1,
    pub(crate) provider_profile: SemanticProfileRefV1,
    pub(crate) rail: BoundedText,
    pub(crate) network: BoundedText,
    pub(crate) operation_profile: SemanticProfileRefV1,
    pub(crate) capacity_lock_capabilities: Vec<CapacityLockCapabilityV1>,
    pub(crate) prepare_capabilities: Vec<PrepareCapabilityV1>,
    pub(crate) commit_capabilities: Vec<CommitCapabilityV1>,
    pub(crate) cancel_capabilities: Vec<CancelCapabilityV1>,
    pub(crate) query_capabilities: Vec<QueryCapabilityV1>,
    pub(crate) evidence_capabilities: Vec<EvidenceCapabilityV1>,
    pub(crate) reversal_capabilities: Vec<ReversalCapabilityV1>,
    pub(crate) atomicity_capabilities: Vec<AtomicityCapabilityV1>,
    pub(crate) idempotency: IdempotencyProfileV1,
    pub(crate) producible_finality_profiles: Vec<SemanticProfileRefV1>,
    pub(crate) synchronization_profiles: Vec<SemanticProfileRefV1>,
    pub(crate) disclosure_profile: SemanticProfileRefV1,
    pub(crate) timing: TimingProfileV1,
    pub(crate) resources: ResourceProfileV1,
    pub(crate) profile_revision: u64,
    pub(crate) profile_commitment: Commitment32,
}

impl RailCapabilityProfileV1 {
    pub fn profile_commitment(&self) -> Commitment32 {
        self.profile_commitment
    }

    pub fn rail(&self) -> &BoundedText {
        &self.rail
    }

    pub fn network(&self) -> &BoundedText {
        &self.network
    }

    pub fn operation_profile(&self) -> &SemanticProfileRefV1 {
        &self.operation_profile
    }

    pub fn capacity_lock_capabilities(&self) -> &[CapacityLockCapabilityV1] {
        &self.capacity_lock_capabilities
    }

    pub fn prepare_capabilities(&self) -> &[PrepareCapabilityV1] {
        &self.prepare_capabilities
    }

    pub fn commit_capabilities(&self) -> &[CommitCapabilityV1] {
        &self.commit_capabilities
    }

    pub fn cancel_capabilities(&self) -> &[CancelCapabilityV1] {
        &self.cancel_capabilities
    }

    pub fn query_capabilities(&self) -> &[QueryCapabilityV1] {
        &self.query_capabilities
    }

    pub fn evidence_capabilities(&self) -> &[EvidenceCapabilityV1] {
        &self.evidence_capabilities
    }

    pub fn reversal_capabilities(&self) -> &[ReversalCapabilityV1] {
        &self.reversal_capabilities
    }

    pub fn atomicity_capabilities(&self) -> &[AtomicityCapabilityV1] {
        &self.atomicity_capabilities
    }

    pub fn idempotency(&self) -> &IdempotencyProfileV1 {
        &self.idempotency
    }

    pub fn producible_finality_profiles(&self) -> &[SemanticProfileRefV1] {
        &self.producible_finality_profiles
    }

    pub fn synchronization_profiles(&self) -> &[SemanticProfileRefV1] {
        &self.synchronization_profiles
    }

    pub fn timing(&self) -> &TimingProfileV1 {
        &self.timing
    }

    pub fn resources(&self) -> &ResourceProfileV1 {
        &self.resources
    }

    pub fn profile_revision(&self) -> u64 {
        self.profile_revision
    }
}
