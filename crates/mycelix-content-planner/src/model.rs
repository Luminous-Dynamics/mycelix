use mycelix_content_core::StorageIntentIdV1;
use mycelix_content_policy::{PlacementTargetV1, SelectionPolicyErrorV1};
use mycelix_content_state::{ActionRefV1, AgentRefV1};
use serde::{Deserialize, Serialize};

pub const PLANNER_SCHEMA_V1: u16 = 1;
pub const PENALTY_PPM_MAX: u32 = 1_000_000;

const PROFILE_MAGIC: &[u8] = b"MYCELIX-CONTENT-PLANNER-PROFILE\0";

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct PlannerProfileIdV1(pub [u8; 32]);

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct PlannerInputIdV1(pub [u8; 32]);

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct PlacementProposalIdV1(pub [u8; 32]);

/// Explicit normalization ceilings for soft metrics. Values at or above a
/// ceiling receive the maximum penalty. Missing/unusable weighted metrics also
/// receive the maximum penalty, so omission cannot improve rank.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct PlannerNormalizationProfileV1 {
    pub schema_version: u16,
    pub id: PlannerProfileIdV1,
    pub cost_ceiling_microunits_per_gib: u64,
    pub latency_ceiling_ms: u32,
    pub energy_ceiling_millijoules_per_gib: u64,
    pub locality_ceiling_km: u32,
}

impl PlannerNormalizationProfileV1 {
    pub fn new(
        cost_ceiling_microunits_per_gib: u64,
        latency_ceiling_ms: u32,
        energy_ceiling_millijoules_per_gib: u64,
        locality_ceiling_km: u32,
    ) -> Result<Self, PlannerErrorV1> {
        let mut value = Self {
            schema_version: PLANNER_SCHEMA_V1,
            id: PlannerProfileIdV1([0; 32]),
            cost_ceiling_microunits_per_gib,
            latency_ceiling_ms,
            energy_ceiling_millijoules_per_gib,
            locality_ceiling_km,
        };
        value.validate_fields()?;
        value.id = value.recompute_id();
        Ok(value)
    }

    pub fn validate(&self) -> Result<(), PlannerErrorV1> {
        if self.schema_version != PLANNER_SCHEMA_V1 {
            return Err(PlannerErrorV1::UnsupportedSchemaVersion);
        }
        self.validate_fields()?;
        if self.id != self.recompute_id() {
            return Err(PlannerErrorV1::ProfileIdMismatch);
        }
        Ok(())
    }

    pub fn recompute_id(&self) -> PlannerProfileIdV1 {
        let mut hasher = blake3::Hasher::new();
        hasher.update(PROFILE_MAGIC);
        hasher.update(&PLANNER_SCHEMA_V1.to_be_bytes());
        hasher.update(&self.cost_ceiling_microunits_per_gib.to_be_bytes());
        hasher.update(&self.latency_ceiling_ms.to_be_bytes());
        hasher.update(&self.energy_ceiling_millijoules_per_gib.to_be_bytes());
        hasher.update(&self.locality_ceiling_km.to_be_bytes());
        PlannerProfileIdV1(*hasher.finalize().as_bytes())
    }

    fn validate_fields(&self) -> Result<(), PlannerErrorV1> {
        if self.cost_ceiling_microunits_per_gib == 0
            || self.latency_ceiling_ms == 0
            || self.energy_ceiling_millijoules_per_gib == 0
            || self.locality_ceiling_km == 0
        {
            return Err(PlannerErrorV1::InvalidNormalizationProfile);
        }
        Ok(())
    }
}

/// Soft evidence is recommendation input only. The planner verifies identity and
/// freshness but does not grant this evidence hard-policy authority.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub struct CandidateSoftEvidenceV1 {
    pub availability_action: ActionRefV1,
    pub advertisement_action: ActionRefV1,
    pub provider: AgentRefV1,
    pub observed_at_unix_ms: u64,
    /// Half-open freshness bound: evidence is usable while `now < valid_until`.
    pub valid_until_unix_ms: u64,
    pub cost_microunits_per_gib: Option<u64>,
    pub latency_ms: Option<u32>,
    pub energy_millijoules_per_gib: Option<u64>,
    pub locality_distance_km: Option<u32>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum SoftEvidenceStateV1 {
    Fresh,
    Missing,
    Stale,
    Future,
    InvalidWindow,
    IdentityMismatch,
    Conflicting,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum SoftMetricKindV1 {
    Cost,
    Latency,
    Energy,
    Locality,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct CandidateScoreV1 {
    pub availability_action: ActionRefV1,
    pub advertisement_action: ActionRefV1,
    pub provider: AgentRefV1,
    /// Lower is better. This is the un-divided sum of `weight * penalty_ppm`.
    pub weighted_penalty: u64,
    pub cost_penalty_ppm: u32,
    pub latency_penalty_ppm: u32,
    pub energy_penalty_ppm: u32,
    pub locality_penalty_ppm: u32,
    pub evidence_state: SoftEvidenceStateV1,
    /// Weighted dimensions that lacked a usable value and therefore received the
    /// conservative maximum penalty.
    pub missing_weighted_metrics: Vec<SoftMetricKindV1>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum PlannerAuthorityV1 {
    RecommendationOnly,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum PlannerSelectionStrategyV1 {
    /// Begin with the whole qualified pool and remove worst-ranked candidates
    /// only when the remaining subset still passes CF-06A hard policy.
    PolicyPreservingEliminationV1,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct PlacementProposalV1 {
    pub schema_version: u16,
    pub id: PlacementProposalIdV1,
    pub planner_input_id: PlannerInputIdV1,
    pub storage_intent_id: StorageIntentIdV1,
    pub target: PlacementTargetV1,
    pub evaluated_at_unix_ms: u64,
    pub profile_id: PlannerProfileIdV1,
    pub authority: PlannerAuthorityV1,
    pub strategy: PlannerSelectionStrategyV1,
    /// Full deterministic best-to-worst ranking of the qualified pool.
    pub ranking: Vec<CandidateScoreV1>,
    /// Policy-valid selected subset in ranking order.
    pub selected_availability_actions: Vec<ActionRefV1>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum PlannerErrorV1 {
    UnsupportedSchemaVersion,
    InvalidNormalizationProfile,
    ProfileIdMismatch,
    InvalidStorageIntent,
    IntentPoolMismatch,
    RequirementsPoolMismatch,
    TargetPoolMismatch,
    InvalidPoolTimestamp,
    InvalidQualifiedCandidate,
    InvalidQualifiedPoolSelection(SelectionPolicyErrorV1),
    FinalSelectionRejected(SelectionPolicyErrorV1),
}
