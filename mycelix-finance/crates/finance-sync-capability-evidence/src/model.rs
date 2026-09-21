use std::fmt;

use mycelix_finance_sync_graph::{BoundedText, Commitment32, SemanticProfileRefV1};
use serde::{Deserialize, Serialize};

pub const MAX_EVIDENCE_ITEMS: usize = 512;
pub const MAX_EVIDENCE_ITEMS_PER_DIMENSION: usize = 64;
pub const MAX_CONFLICT_IDENTITIES: usize = 128;
pub const MAX_CONTRADICTED_SUBJECTS: usize = 128;

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum EvidenceError {
    StaticProfileCommitmentMismatch,
    TooManyEvidenceItems,
    TooManyEvidenceItemsForDimension,
    TooManyConflictIdentities,
    TooManyContradictedSubjects,
    CanonicalLengthOverflow,
}

impl fmt::Display for EvidenceError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let message = match self {
            Self::StaticProfileCommitmentMismatch => {
                "evidence bundle does not bind the supplied static capability profile"
            }
            Self::TooManyEvidenceItems => "evidence bundle exceeds the v1 item bound",
            Self::TooManyEvidenceItemsForDimension => {
                "evidence bundle exceeds the per-dimension item bound"
            }
            Self::TooManyConflictIdentities => "evidence bundle exceeds the conflict identity bound",
            Self::TooManyContradictedSubjects => {
                "evidence bundle exceeds the contradicted-claim-subject bound"
            }
            Self::CanonicalLengthOverflow => "canonical encoding length exceeds u32",
        };
        f.write_str(message)
    }
}

impl std::error::Error for EvidenceError {}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CapabilityEvidenceDimensionV1 {
    AdapterArtifact,
    AdapterConfiguration,
    ProviderApiProfile,
    CapacityLock,
    Prepare,
    Commit,
    CancelAbort,
    Query,
    EvidenceProduction,
    Reversal,
    Atomicity,
    Idempotency,
    FinalityProfile,
    SynchronizationProfile,
    Disclosure,
    Timing,
    Resources,
}

impl CapabilityEvidenceDimensionV1 {
    pub(crate) const fn canonical_tag(self) -> u8 {
        match self {
            Self::AdapterArtifact => 1,
            Self::AdapterConfiguration => 2,
            Self::ProviderApiProfile => 3,
            Self::CapacityLock => 4,
            Self::Prepare => 5,
            Self::Commit => 6,
            Self::CancelAbort => 7,
            Self::Query => 8,
            Self::EvidenceProduction => 9,
            Self::Reversal => 10,
            Self::Atomicity => 11,
            Self::Idempotency => 12,
            Self::FinalityProfile => 13,
            Self::SynchronizationProfile => 14,
            Self::Disclosure => 15,
            Self::Timing => 16,
            Self::Resources => 17,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum StructuralEvidenceClaimV1 {
    SupportsDeclaredStaticSemantics,
    ContradictsDeclaredStaticSemantics,
    ObservesRelatedFact,
    Indeterminate,
}

impl StructuralEvidenceClaimV1 {
    pub(crate) const fn canonical_tag(self) -> u8 {
        match self {
            Self::SupportsDeclaredStaticSemantics => 1,
            Self::ContradictsDeclaredStaticSemantics => 2,
            Self::ObservesRelatedFact => 3,
            Self::Indeterminate => 4,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SourceChronologyEvidenceV1 {
    pub chronology_profile: SemanticProfileRefV1,
    pub chronology_commitment: Commitment32,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RailCapabilityEvidenceItemInputV1 {
    pub dimension: CapabilityEvidenceDimensionV1,
    pub evidence_id: BoundedText,
    pub evidence_commitment: Commitment32,
    pub source_profile: SemanticProfileRefV1,
    pub claim: StructuralEvidenceClaimV1,
    pub claim_subject_commitment: Commitment32,
    pub source_revision: Option<u64>,
    pub chronology: Option<SourceChronologyEvidenceV1>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RailCapabilityEvidenceBundleInputV1 {
    pub static_profile_commitment: Commitment32,
    pub evidence_profile: SemanticProfileRefV1,
    pub evidence_context_commitment: Commitment32,
    pub items: Vec<RailCapabilityEvidenceItemInputV1>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum EvidenceBundleDispositionV1 {
    NoDetectedConflict,
    IdentityConflicted,
    SemanticallyContradicted,
    IdentityConflictedAndSemanticallyContradicted,
}

impl EvidenceBundleDispositionV1 {
    pub(crate) const fn canonical_tag(self) -> u8 {
        match self {
            Self::NoDetectedConflict => 1,
            Self::IdentityConflicted => 2,
            Self::SemanticallyContradicted => 3,
            Self::IdentityConflictedAndSemanticallyContradicted => 4,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct StructuralEvidenceRecordV1 {
    pub(crate) dimension: CapabilityEvidenceDimensionV1,
    pub(crate) evidence_id: BoundedText,
    pub(crate) item_commitment: Commitment32,
    pub(crate) claim: StructuralEvidenceClaimV1,
    pub(crate) claim_subject_commitment: Commitment32,
}

impl StructuralEvidenceRecordV1 {
    pub fn dimension(&self) -> CapabilityEvidenceDimensionV1 {
        self.dimension
    }

    pub fn evidence_id(&self) -> &BoundedText {
        &self.evidence_id
    }

    pub fn item_commitment(&self) -> Commitment32 {
        self.item_commitment
    }

    pub fn claim(&self) -> StructuralEvidenceClaimV1 {
        self.claim
    }

    pub fn claim_subject_commitment(&self) -> Commitment32 {
        self.claim_subject_commitment
    }
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize)]
pub struct ContradictedClaimSubjectV1 {
    pub(crate) dimension: CapabilityEvidenceDimensionV1,
    pub(crate) claim_subject_commitment: Commitment32,
}

impl ContradictedClaimSubjectV1 {
    pub fn dimension(&self) -> CapabilityEvidenceDimensionV1 {
        self.dimension
    }

    pub fn claim_subject_commitment(&self) -> Commitment32 {
        self.claim_subject_commitment
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct RailCapabilityEvidenceBundleV1 {
    pub(crate) static_profile_commitment: Commitment32,
    pub(crate) evidence_profile: SemanticProfileRefV1,
    pub(crate) evidence_context_commitment: Commitment32,
    pub(crate) evidence_records: Vec<StructuralEvidenceRecordV1>,
    pub(crate) conflicted_evidence_ids: Vec<BoundedText>,
    pub(crate) contradicted_claim_subjects: Vec<ContradictedClaimSubjectV1>,
    pub(crate) disposition: EvidenceBundleDispositionV1,
    pub(crate) bundle_commitment: Commitment32,
}

impl RailCapabilityEvidenceBundleV1 {
    pub fn static_profile_commitment(&self) -> Commitment32 {
        self.static_profile_commitment
    }

    pub fn evidence_profile(&self) -> &SemanticProfileRefV1 {
        &self.evidence_profile
    }

    pub fn evidence_context_commitment(&self) -> Commitment32 {
        self.evidence_context_commitment
    }

    pub fn evidence_records(&self) -> &[StructuralEvidenceRecordV1] {
        &self.evidence_records
    }

    pub fn conflicted_evidence_ids(&self) -> &[BoundedText] {
        &self.conflicted_evidence_ids
    }

    pub fn contradicted_claim_subjects(&self) -> &[ContradictedClaimSubjectV1] {
        &self.contradicted_claim_subjects
    }

    pub fn disposition(&self) -> EvidenceBundleDispositionV1 {
        self.disposition
    }

    pub fn bundle_commitment(&self) -> Commitment32 {
        self.bundle_commitment
    }
}
