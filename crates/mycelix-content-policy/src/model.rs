use std::collections::{BTreeMap, BTreeSet};

use mycelix_content_core::{
    ContentDigestV1, FailureDomainKindV1, JurisdictionV1, ObjectIdV1, PlacementRequirementsV1,
    StorageIntentIdV1,
};
use mycelix_content_state::{ActionRefV1, AgentRefV1, SnapshotServiceCandidateV1, TimestampMicrosV1};
use serde::{Deserialize, Serialize};

#[derive(
    Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize,
)]
pub enum PolicyAssuranceV1 {
    SelfClaimed,
    ProviderSigned,
    IndependentlyAttested,
}

impl PolicyAssuranceV1 {
    pub fn meets(self, minimum: Self) -> bool {
        self >= minimum
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct AssuredJurisdictionV1 {
    pub jurisdiction: JurisdictionV1,
    pub assurance: PolicyAssuranceV1,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct AssuredFailureDomainV1 {
    pub kind: FailureDomainKindV1,
    pub value: String,
    pub assurance: PolicyAssuranceV1,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct RetentionCapabilityEvidenceV1 {
    pub assurance: PolicyAssuranceV1,
    /// If present, the provider is evidenced to retain through at least this Unix millisecond.
    pub guaranteed_until_unix_ms: Option<u64>,
    /// Evidence for an indefinite-retention capability.
    pub supports_indefinite: bool,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ProviderPolicyEvidenceV1 {
    /// The exact CF-05 advertisement this evidence describes.
    pub advertisement: ActionRefV1,
    pub provider: AgentRefV1,
    /// Half-open validity window for this verified policy-evidence bundle.
    pub valid_from_unix_ms: u64,
    pub valid_until_unix_ms: u64,
    pub storage_jurisdictions: Vec<AssuredJurisdictionV1>,
    /// Evidence that provider-managed encryption at rest is enabled/supported for this placement.
    pub provider_at_rest_encryption: Option<PolicyAssuranceV1>,
    pub retention: Option<RetentionCapabilityEvidenceV1>,
    /// Policy facts from an assurance boundary, not raw CF-05 self-claims.
    pub failure_domains: Vec<AssuredFailureDomainV1>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct PlacementTargetV1 {
    pub object_id: ObjectIdV1,
    pub digest: ContentDigestV1,
    pub size_bytes: u64,
    /// True only when the exact bytes identified by `digest` are already client-side encrypted.
    pub client_side_encrypted: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct HardPolicyGateConfigV1 {
    /// Require CF-05A to have queried the relevant indexes in the caller's local DHT view.
    pub require_queried_indexes_complete: bool,
    /// Refuse to qualify any pool while CF-05A reports malformed/conflicting evidence.
    pub require_clean_projection: bool,
    /// Minimum assurance accepted for provider policy facts.
    pub minimum_provider_fact_assurance: PolicyAssuranceV1,
}

impl HardPolicyGateConfigV1 {
    pub const fn strict() -> Self {
        Self {
            require_queried_indexes_complete: true,
            require_clean_projection: true,
            minimum_provider_fact_assurance: PolicyAssuranceV1::IndependentlyAttested,
        }
    }
}

impl Default for HardPolicyGateConfigV1 {
    fn default() -> Self {
        Self::strict()
    }
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum CandidateRejectionReasonV1 {
    CandidateNotTemporallyLive,
    TargetDigestMismatch,
    TargetSizeMismatch,
    MissingProviderPolicyEvidence,
    ConflictingProviderPolicyEvidence,
    ProviderEvidenceIdentityMismatch,
    InvalidProviderPolicyEvidenceWindow,
    ProviderPolicyEvidenceNotCurrent,
    MissingJurisdictionEvidence,
    InsufficientJurisdictionAssurance,
    InvalidJurisdictionEvidence,
    JurisdictionNotAllowed { jurisdiction: JurisdictionV1 },
    ClientSideEncryptionRequired,
    MissingProviderAtRestEncryptionEvidence,
    InsufficientProviderAtRestEncryptionAssurance,
    MissingRetentionEvidence,
    InsufficientRetentionAssurance,
    RetentionCapabilityInsufficient,
    MissingFailureDomainEvidence { kind: FailureDomainKindV1 },
    InsufficientFailureDomainAssurance { kind: FailureDomainKindV1 },
    InvalidFailureDomainValue { kind: FailureDomainKindV1 },
    AmbiguousFailureDomainEvidence { kind: FailureDomainKindV1 },
    AttestedFailureDomainConflictsWithProviderClaim { kind: FailureDomainKindV1 },
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct CandidateRejectionV1 {
    pub availability_action: ActionRefV1,
    pub advertisement_action: ActionRefV1,
    pub provider: AgentRefV1,
    pub reasons: Vec<CandidateRejectionReasonV1>,
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum PoolFailureV1 {
    InvalidStorageIntent,
    InvalidPlacementTarget,
    TargetObjectMismatch,
    InvalidEvaluationTimestamp,
    EvaluationPredatesIntent,
    SnapshotCoverageInsufficient,
    ProjectionContainsIssues,
    RetentionRequirementOverflow,
    InsufficientEligibleReplicas { required: u16, observed: u16 },
    InsufficientFailureDomainDiversity {
        kind: FailureDomainKindV1,
        required: u16,
        observed: u16,
    },
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct PolicyEligibleCandidateV1 {
    pub candidate: SnapshotServiceCandidateV1,
    /// All sufficiently assured storage jurisdictions accepted by hard policy.
    pub accepted_jurisdictions: BTreeSet<JurisdictionV1>,
    /// One unambiguous sufficiently assured value for every required failure-domain dimension.
    pub accepted_failure_domains: BTreeMap<FailureDomainKindV1, String>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct PolicyQualifiedPoolV1 {
    pub storage_intent_id: StorageIntentIdV1,
    pub target: PlacementTargetV1,
    pub evaluated_at: TimestampMicrosV1,
    pub evaluated_at_unix_ms: u64,
    pub requirements: PlacementRequirementsV1,
    pub candidates: Vec<PolicyEligibleCandidateV1>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct HardPolicyEvaluationV1 {
    /// Present only when all global hard constraints are feasible.
    pub qualified_pool: Option<PolicyQualifiedPoolV1>,
    pub rejections: Vec<CandidateRejectionV1>,
    pub failures: Vec<PoolFailureV1>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum SelectionPolicyErrorV1 {
    DuplicateCandidate { availability_action: ActionRefV1 },
    UnknownCandidate { availability_action: ActionRefV1 },
    InsufficientReplicas { required: u16, selected: u16 },
    InsufficientFailureDomainDiversity {
        kind: FailureDomainKindV1,
        required: u16,
        observed: u16,
    },
}
