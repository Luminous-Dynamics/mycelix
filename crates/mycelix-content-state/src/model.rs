use mycelix_content_core::{ContentDigestV1, DigestAlgorithmV1, FailureDomainKindV1};
use serde::{Deserialize, Serialize};
use serde_big_array::BigArray;

#[derive(
    Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize,
)]
pub struct ActionRefV1(#[serde(with = "BigArray")] pub [u8; 39]);

#[derive(
    Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize,
)]
pub struct AgentRefV1(#[serde(with = "BigArray")] pub [u8; 39]);

#[derive(
    Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize,
)]
pub struct TimestampMicrosV1(pub i64);

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SnapshotCoverageV1 {
    /// Some relevant evidence may not have been fetched or integrated yet.
    Partial,
    /// The caller asserts it queried all indexes relevant to the supplied
    /// advertisements at this local DHT view. This is not a global-finality claim.
    QueriedIndexesComplete,
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub struct FailureDomainClaimEvidenceV1 {
    pub kind: FailureDomainKindV1,
    pub value: String,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ProviderAdvertisementEvidenceV1 {
    pub action: ActionRefV1,
    pub authored_at: TimestampMicrosV1,
    pub provider: AgentRefV1,
    pub iroh_endpoint_id: [u8; 32],
    pub supported_algorithms: Vec<DigestAlgorithmV1>,
    pub max_blob_size_bytes: u64,
    pub ttl_seconds: u32,
    pub failure_domains: Vec<FailureDomainClaimEvidenceV1>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct AvailabilityEvidenceV1 {
    pub action: ActionRefV1,
    pub authored_at: TimestampMicrosV1,
    pub provider: AgentRefV1,
    pub advertisement: ActionRefV1,
    pub digest: ContentDigestV1,
    pub size_bytes: u64,
    pub ttl_seconds: u32,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct WithdrawalEvidenceV1 {
    pub action: ActionRefV1,
    pub authored_at: TimestampMicrosV1,
    pub provider: AgentRefV1,
    pub advertisement: ActionRefV1,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum ObservationOutcomeEvidenceV1 {
    VerifiedComplete { size_bytes: u64 },
    UnavailableOrHidden,
    Busy,
    TransferFailed,
    ProviderReportedIntegrityFailure,
    DigestMismatch {
        observed_digest: ContentDigestV1,
        size_bytes: u64,
    },
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ObservationEvidenceV1 {
    pub action: ActionRefV1,
    pub authored_at: TimestampMicrosV1,
    pub observer: AgentRefV1,
    pub provider: AgentRefV1,
    pub advertisement: ActionRefV1,
    pub digest: ContentDigestV1,
    pub outcome: ObservationOutcomeEvidenceV1,
    pub latency_ms: Option<u64>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct EvidenceSnapshotV1 {
    pub coverage: SnapshotCoverageV1,
    pub advertisements: Vec<ProviderAdvertisementEvidenceV1>,
    pub availability: Vec<AvailabilityEvidenceV1>,
    pub withdrawals: Vec<WithdrawalEvidenceV1>,
    pub observations: Vec<ObservationEvidenceV1>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum AdvertisementTemporalStateV1 {
    NotYetAuthored,
    Live { expires_at: TimestampMicrosV1 },
    Expired { expired_at: TimestampMicrosV1 },
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum WithdrawalObservationV1 {
    NoWithdrawalObserved,
    Withdrawn {
        action: ActionRefV1,
        authored_at: TimestampMicrosV1,
    },
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ProjectedAdvertisementV1 {
    pub action: ActionRefV1,
    pub provider: AgentRefV1,
    pub iroh_endpoint_id: [u8; 32],
    pub supported_algorithms: Vec<DigestAlgorithmV1>,
    pub max_blob_size_bytes: u64,
    pub failure_domains: Vec<FailureDomainClaimEvidenceV1>,
    pub authored_at: TimestampMicrosV1,
    pub temporal_state: AdvertisementTemporalStateV1,
    pub withdrawal: WithdrawalObservationV1,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct SnapshotServiceCandidateV1 {
    pub availability_action: ActionRefV1,
    pub advertisement_action: ActionRefV1,
    pub provider: AgentRefV1,
    pub iroh_endpoint_id: [u8; 32],
    pub digest: ContentDigestV1,
    pub size_bytes: u64,
    pub failure_domains: Vec<FailureDomainClaimEvidenceV1>,
    pub claim_authored_at: TimestampMicrosV1,
    /// Exclusive upper bound. The candidate is live for `now < effective_until`.
    pub effective_until: TimestampMicrosV1,
    pub withdrawal: WithdrawalObservationV1,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ProjectedObservationV1 {
    pub action: ActionRefV1,
    pub authored_at: TimestampMicrosV1,
    pub age_micros: u64,
    pub observer: AgentRefV1,
    pub provider: AgentRefV1,
    pub advertisement: ActionRefV1,
    pub digest: ContentDigestV1,
    pub outcome: ObservationOutcomeEvidenceV1,
    pub latency_ms: Option<u64>,
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum ProjectionIssueKindV1 {
    ConflictingDuplicateAction,
    MissingAdvertisement { advertisement: ActionRefV1 },
    ProviderMismatch,
    UnsupportedDigestAlgorithm,
    InvalidSize,
    ZeroTtl,
    TtlExceedsAdvertisement,
    EvidencePredatesAdvertisement,
    TimestampOverflow,
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub struct ProjectionIssueV1 {
    pub action: ActionRefV1,
    pub kind: ProjectionIssueKindV1,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ProjectedContentStateV1 {
    pub evaluated_at: TimestampMicrosV1,
    pub coverage: SnapshotCoverageV1,
    pub advertisements: Vec<ProjectedAdvertisementV1>,
    /// Candidates that are temporally live in this supplied evidence snapshot.
    /// This is not a proof of availability, authorization, or global non-withdrawal.
    pub service_candidates: Vec<SnapshotServiceCandidateV1>,
    pub observations: Vec<ProjectedObservationV1>,
    pub issues: Vec<ProjectionIssueV1>,
}
