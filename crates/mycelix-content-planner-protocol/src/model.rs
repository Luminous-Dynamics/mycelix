use mycelix_content_core::{FailureDomainKindV1, PlacementPreferencesV1, StorageIntentIdV1};
use mycelix_content_planner::{
    CandidateScoreV1, CandidateSoftEvidenceV1, PlacementProposalIdV1, PlannerAuthorityV1,
    PlannerErrorV1, PlannerInputIdV1, PlannerNormalizationProfileV1, PlannerProfileIdV1,
    SoftEvidenceStateV1, SoftMetricKindV1,
};
use mycelix_content_policy::{PlacementTargetV1, SelectionPolicyErrorV1};
use mycelix_content_state::{ActionRefV1, AgentRefV1};
use serde::{Deserialize, Serialize};

pub const EXTERNAL_PLANNER_PROTOCOL_V1: u16 = 1;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct ExternalPlannerRequestIdV1(pub [u8; 32]);

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct ExternalPlannerAcceptanceIdV1(pub [u8; 32]);

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ExternalPlannerProfileV1 {
    pub id: PlannerProfileIdV1,
    pub cost_ceiling_microunits_per_gib: u64,
    pub latency_ceiling_ms: u32,
    pub energy_ceiling_millijoules_per_gib: u64,
    pub locality_ceiling_km: u32,
}

impl From<PlannerNormalizationProfileV1> for ExternalPlannerProfileV1 {
    fn from(value: PlannerNormalizationProfileV1) -> Self {
        Self {
            id: value.id,
            cost_ceiling_microunits_per_gib: value.cost_ceiling_microunits_per_gib,
            latency_ceiling_ms: value.latency_ceiling_ms,
            energy_ceiling_millijoules_per_gib: value.energy_ceiling_millijoules_per_gib,
            locality_ceiling_km: value.locality_ceiling_km,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct ExternalPlannerPreferencesV1 {
    pub target_latency_ms: Option<u32>,
    pub cost_weight: u16,
    pub latency_weight: u16,
    pub energy_weight: u16,
    pub locality_weight: u16,
}

impl From<PlacementPreferencesV1> for ExternalPlannerPreferencesV1 {
    fn from(value: PlacementPreferencesV1) -> Self {
        Self {
            target_latency_ms: value.target_latency_ms,
            cost_weight: value.cost_weight,
            latency_weight: value.latency_weight,
            energy_weight: value.energy_weight,
            locality_weight: value.locality_weight,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub struct ExternalFailureDomainRequirementV1 {
    pub kind: FailureDomainKindV1,
    pub minimum_distinct: u16,
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub struct ExternalAcceptedFailureDomainV1 {
    pub kind: FailureDomainKindV1,
    pub value: String,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ExternalPlannerCandidateV1 {
    pub availability_action: ActionRefV1,
    pub advertisement_action: ActionRefV1,
    pub provider: AgentRefV1,
    pub baseline_rank: u64,
    pub baseline_weighted_penalty: u64,
    pub cost_penalty_ppm: u32,
    pub latency_penalty_ppm: u32,
    pub energy_penalty_ppm: u32,
    pub locality_penalty_ppm: u32,
    pub evidence_state: SoftEvidenceStateV1,
    pub missing_weighted_metrics: Vec<SoftMetricKindV1>,
    /// Hard-policy facts are supplied only so the external planner can avoid
    /// proposing obviously invalid diversity subsets. Mycelix revalidates them.
    pub accepted_failure_domains: Vec<ExternalAcceptedFailureDomainV1>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ExternalPlannerRequestV1 {
    pub schema_version: u16,
    pub id: ExternalPlannerRequestIdV1,
    pub planner_input_id: PlannerInputIdV1,
    pub storage_intent_id: StorageIntentIdV1,
    pub target: PlacementTargetV1,
    pub evaluated_at_unix_ms: u64,
    pub profile: ExternalPlannerProfileV1,
    pub preferences: ExternalPlannerPreferencesV1,
    pub minimum_replicas: u16,
    pub failure_domain_requirements: Vec<ExternalFailureDomainRequirementV1>,
    pub baseline_proposal_id: PlacementProposalIdV1,
    pub baseline_selected_availability_actions: Vec<ActionRefV1>,
    pub candidates: Vec<ExternalPlannerCandidateV1>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ExternalPlannerRecommendationV1 {
    pub schema_version: u16,
    pub request_id: ExternalPlannerRequestIdV1,
    pub planner_input_id: PlannerInputIdV1,
    /// Lowercase token identifying the recommendation engine implementation.
    pub engine_id: String,
    /// Version/revision token for audit and reproducibility.
    pub engine_version: String,
    /// Complete best-to-worst permutation of request candidates.
    pub ranking: Vec<ActionRefV1>,
    /// Selected subset, preserving the same relative order as `ranking`.
    pub selected_availability_actions: Vec<ActionRefV1>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct AcceptedExternalPlannerRecommendationV1 {
    pub schema_version: u16,
    pub id: ExternalPlannerAcceptanceIdV1,
    pub request_id: ExternalPlannerRequestIdV1,
    pub planner_input_id: PlannerInputIdV1,
    pub storage_intent_id: StorageIntentIdV1,
    pub target: PlacementTargetV1,
    pub evaluated_at_unix_ms: u64,
    pub engine_id: String,
    pub engine_version: String,
    pub authority: PlannerAuthorityV1,
    pub ranking: Vec<ActionRefV1>,
    pub selected_availability_actions: Vec<ActionRefV1>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum ExternalPlannerProtocolErrorV1 {
    Planner(PlannerErrorV1),
    Json(String),
    UnsupportedSchemaVersion,
    RequestIdMismatch,
    RequestPoolMismatch,
    PlannerInputMismatch,
    InvalidEngineToken,
    DuplicateRankingAction { action: ActionRefV1 },
    UnknownRankingAction { action: ActionRefV1 },
    IncompleteRanking,
    DuplicateSelectedAction { action: ActionRefV1 },
    UnknownSelectedAction { action: ActionRefV1 },
    SelectedOrderDoesNotFollowRanking,
    PolicyRejectedSelection(SelectionPolicyErrorV1),
}

impl From<PlannerErrorV1> for ExternalPlannerProtocolErrorV1 {
    fn from(value: PlannerErrorV1) -> Self {
        Self::Planner(value)
    }
}

impl ExternalPlannerCandidateV1 {
    pub(crate) fn from_score(
        score: &CandidateScoreV1,
        baseline_rank: u64,
        accepted_failure_domains: Vec<ExternalAcceptedFailureDomainV1>,
    ) -> Self {
        Self {
            availability_action: score.availability_action,
            advertisement_action: score.advertisement_action,
            provider: score.provider,
            baseline_rank,
            baseline_weighted_penalty: score.weighted_penalty,
            cost_penalty_ppm: score.cost_penalty_ppm,
            latency_penalty_ppm: score.latency_penalty_ppm,
            energy_penalty_ppm: score.energy_penalty_ppm,
            locality_penalty_ppm: score.locality_penalty_ppm,
            evidence_state: score.evidence_state,
            missing_weighted_metrics: score.missing_weighted_metrics.clone(),
            accepted_failure_domains,
        }
    }
}

pub type ExternalPlannerSoftEvidenceV1 = CandidateSoftEvidenceV1;
