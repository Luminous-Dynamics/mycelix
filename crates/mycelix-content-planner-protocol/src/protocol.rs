use std::collections::{BTreeMap, BTreeSet};

use mycelix_content_core::{FailureDomainKindV1, StorageIntentV1};
use mycelix_content_planner::{
    plan_deterministic_v1, CandidateSoftEvidenceV1, PlannerAuthorityV1,
    PlannerNormalizationProfileV1, SoftEvidenceStateV1, SoftMetricKindV1,
};
use mycelix_content_policy::PolicyQualifiedPoolV1;
use mycelix_content_state::ActionRefV1;

use crate::{
    AcceptedExternalPlannerRecommendationV1, ExternalAcceptedFailureDomainV1,
    ExternalFailureDomainRequirementV1, ExternalPlannerAcceptanceIdV1,
    ExternalPlannerCandidateV1, ExternalPlannerPreferencesV1, ExternalPlannerProfileV1,
    ExternalPlannerProtocolErrorV1, ExternalPlannerRecommendationV1, ExternalPlannerRequestIdV1,
    ExternalPlannerRequestV1, EXTERNAL_PLANNER_PROTOCOL_V1,
};

const REQUEST_MAGIC: &[u8] = b"MYCELIX-EXTERNAL-PLANNER-REQUEST\0";
const ACCEPTANCE_MAGIC: &[u8] = b"MYCELIX-EXTERNAL-PLANNER-ACCEPTANCE\0";

fn put_field(hasher: &mut blake3::Hasher, value: &[u8]) {
    hasher.update(&(value.len() as u64).to_be_bytes());
    hasher.update(value);
}

fn put_string(hasher: &mut blake3::Hasher, value: &str) {
    put_field(hasher, value.as_bytes());
}

fn put_option_u32(hasher: &mut blake3::Hasher, value: Option<u32>) {
    match value {
        Some(value) => {
            hasher.update(&[1]);
            hasher.update(&value.to_be_bytes());
        }
        None => {
            hasher.update(&[0]);
        }
    }
}

fn failure_domain_tag(kind: FailureDomainKindV1) -> u8 {
    match kind {
        FailureDomainKindV1::Operator => 0,
        FailureDomainKindV1::Machine => 1,
        FailureDomainKindV1::Site => 2,
        FailureDomainKindV1::NetworkAsn => 3,
        FailureDomainKindV1::Region => 4,
        FailureDomainKindV1::Jurisdiction => 5,
        FailureDomainKindV1::PowerDomain => 6,
    }
}

fn evidence_state_tag(state: SoftEvidenceStateV1) -> u8 {
    match state {
        SoftEvidenceStateV1::Fresh => 0,
        SoftEvidenceStateV1::Missing => 1,
        SoftEvidenceStateV1::Stale => 2,
        SoftEvidenceStateV1::Future => 3,
        SoftEvidenceStateV1::InvalidWindow => 4,
        SoftEvidenceStateV1::IdentityMismatch => 5,
        SoftEvidenceStateV1::Conflicting => 6,
    }
}

fn metric_kind_tag(kind: SoftMetricKindV1) -> u8 {
    match kind {
        SoftMetricKindV1::Cost => 0,
        SoftMetricKindV1::Latency => 1,
        SoftMetricKindV1::Energy => 2,
        SoftMetricKindV1::Locality => 3,
    }
}

fn request_id(request: &ExternalPlannerRequestV1) -> ExternalPlannerRequestIdV1 {
    let mut hasher = blake3::Hasher::new();
    hasher.update(REQUEST_MAGIC);
    hasher.update(&EXTERNAL_PLANNER_PROTOCOL_V1.to_be_bytes());
    hasher.update(&request.planner_input_id.0);
    hasher.update(&request.storage_intent_id.0);
    hasher.update(&request.target.object_id.0);
    put_field(&mut hasher, request.target.digest.algorithm.tag().as_bytes());
    hasher.update(&request.target.digest.bytes);
    hasher.update(&request.target.size_bytes.to_be_bytes());
    hasher.update(&[u8::from(request.target.client_side_encrypted)]);
    hasher.update(&request.evaluated_at_unix_ms.to_be_bytes());

    hasher.update(&request.profile.id.0);
    hasher.update(&request.profile.cost_ceiling_microunits_per_gib.to_be_bytes());
    hasher.update(&request.profile.latency_ceiling_ms.to_be_bytes());
    hasher.update(&request.profile.energy_ceiling_millijoules_per_gib.to_be_bytes());
    hasher.update(&request.profile.locality_ceiling_km.to_be_bytes());

    put_option_u32(&mut hasher, request.preferences.target_latency_ms);
    hasher.update(&request.preferences.cost_weight.to_be_bytes());
    hasher.update(&request.preferences.latency_weight.to_be_bytes());
    hasher.update(&request.preferences.energy_weight.to_be_bytes());
    hasher.update(&request.preferences.locality_weight.to_be_bytes());

    hasher.update(&request.minimum_replicas.to_be_bytes());
    hasher.update(&(request.failure_domain_requirements.len() as u64).to_be_bytes());
    for requirement in &request.failure_domain_requirements {
        hasher.update(&[failure_domain_tag(requirement.kind)]);
        hasher.update(&requirement.minimum_distinct.to_be_bytes());
    }

    hasher.update(&request.baseline_proposal_id.0);
    hasher.update(&(request.baseline_selected_availability_actions.len() as u64).to_be_bytes());
    for action in &request.baseline_selected_availability_actions {
        hasher.update(&action.0);
    }

    hasher.update(&(request.candidates.len() as u64).to_be_bytes());
    for candidate in &request.candidates {
        hasher.update(&candidate.availability_action.0);
        hasher.update(&candidate.advertisement_action.0);
        hasher.update(&candidate.provider.0);
        hasher.update(&candidate.baseline_rank.to_be_bytes());
        hasher.update(&candidate.baseline_weighted_penalty.to_be_bytes());
        hasher.update(&candidate.cost_penalty_ppm.to_be_bytes());
        hasher.update(&candidate.latency_penalty_ppm.to_be_bytes());
        hasher.update(&candidate.energy_penalty_ppm.to_be_bytes());
        hasher.update(&candidate.locality_penalty_ppm.to_be_bytes());
        hasher.update(&[evidence_state_tag(candidate.evidence_state)]);
        hasher.update(&(candidate.missing_weighted_metrics.len() as u64).to_be_bytes());
        for metric in &candidate.missing_weighted_metrics {
            hasher.update(&[metric_kind_tag(*metric)]);
        }
        hasher.update(&(candidate.accepted_failure_domains.len() as u64).to_be_bytes());
        for domain in &candidate.accepted_failure_domains {
            hasher.update(&[failure_domain_tag(domain.kind)]);
            put_string(&mut hasher, &domain.value);
        }
    }

    ExternalPlannerRequestIdV1(*hasher.finalize().as_bytes())
}

fn acceptance_id(
    value: &AcceptedExternalPlannerRecommendationV1,
) -> ExternalPlannerAcceptanceIdV1 {
    let mut hasher = blake3::Hasher::new();
    hasher.update(ACCEPTANCE_MAGIC);
    hasher.update(&EXTERNAL_PLANNER_PROTOCOL_V1.to_be_bytes());
    hasher.update(&value.request_id.0);
    hasher.update(&value.planner_input_id.0);
    hasher.update(&value.storage_intent_id.0);
    hasher.update(&value.target.object_id.0);
    put_field(&mut hasher, value.target.digest.algorithm.tag().as_bytes());
    hasher.update(&value.target.digest.bytes);
    hasher.update(&value.target.size_bytes.to_be_bytes());
    hasher.update(&[u8::from(value.target.client_side_encrypted)]);
    hasher.update(&value.evaluated_at_unix_ms.to_be_bytes());
    put_string(&mut hasher, &value.engine_id);
    put_string(&mut hasher, &value.engine_version);
    hasher.update(&[match value.authority {
        PlannerAuthorityV1::RecommendationOnly => 0,
    }]);
    hasher.update(&(value.ranking.len() as u64).to_be_bytes());
    for action in &value.ranking {
        hasher.update(&action.0);
    }
    hasher.update(&(value.selected_availability_actions.len() as u64).to_be_bytes());
    for action in &value.selected_availability_actions {
        hasher.update(&action.0);
    }
    ExternalPlannerAcceptanceIdV1(*hasher.finalize().as_bytes())
}

fn valid_token(value: &str) -> bool {
    let bytes = value.as_bytes();
    !bytes.is_empty()
        && bytes.len() <= 64
        && bytes[0].is_ascii_lowercase()
        && bytes.iter().all(|byte| {
            byte.is_ascii_lowercase()
                || byte.is_ascii_digit()
                || matches!(*byte, b'-' | b'_' | b'.' | b'/' | b':')
        })
}

fn validate_request_against_pool(
    intent: &StorageIntentV1,
    profile: PlannerNormalizationProfileV1,
    pool: &PolicyQualifiedPoolV1,
    request: &ExternalPlannerRequestV1,
) -> Result<(), ExternalPlannerProtocolErrorV1> {
    if request.schema_version != EXTERNAL_PLANNER_PROTOCOL_V1 {
        return Err(ExternalPlannerProtocolErrorV1::UnsupportedSchemaVersion);
    }
    if request.id != request_id(request) {
        return Err(ExternalPlannerProtocolErrorV1::RequestIdMismatch);
    }
    if intent.validate().is_err()
        || request.storage_intent_id != intent.id
        || request.storage_intent_id != pool.storage_intent_id
        || request.target != pool.target
        || request.evaluated_at_unix_ms != pool.evaluated_at_unix_ms
        || request.preferences != ExternalPlannerPreferencesV1::from(intent.preferences)
        || request.minimum_replicas != pool.requirements.minimum_replicas
        || request.profile != ExternalPlannerProfileV1::from(profile)
    {
        return Err(ExternalPlannerProtocolErrorV1::RequestPoolMismatch);
    }

    let expected_requirements = pool
        .requirements
        .failure_domains
        .iter()
        .map(|(kind, minimum_distinct)| ExternalFailureDomainRequirementV1 {
            kind: *kind,
            minimum_distinct: *minimum_distinct,
        })
        .collect::<Vec<_>>();
    if request.failure_domain_requirements != expected_requirements {
        return Err(ExternalPlannerProtocolErrorV1::RequestPoolMismatch);
    }

    let request_actions = request
        .candidates
        .iter()
        .map(|candidate| candidate.availability_action)
        .collect::<BTreeSet<_>>();
    let pool_actions = pool
        .candidates
        .iter()
        .map(|candidate| candidate.candidate.availability_action)
        .collect::<BTreeSet<_>>();
    if request_actions.len() != request.candidates.len()
        || request_actions != pool_actions
        || request.candidates.len() != pool.candidates.len()
    {
        return Err(ExternalPlannerProtocolErrorV1::RequestPoolMismatch);
    }

    for (index, candidate) in request.candidates.iter().enumerate() {
        if candidate.baseline_rank != index as u64 {
            return Err(ExternalPlannerProtocolErrorV1::RequestPoolMismatch);
        }
        let Some(pool_candidate) = pool
            .candidates
            .iter()
            .find(|value| value.candidate.availability_action == candidate.availability_action)
        else {
            return Err(ExternalPlannerProtocolErrorV1::RequestPoolMismatch);
        };
        if candidate.advertisement_action != pool_candidate.candidate.advertisement_action
            || candidate.provider != pool_candidate.candidate.provider
        {
            return Err(ExternalPlannerProtocolErrorV1::RequestPoolMismatch);
        }
        let expected_domains = pool_candidate
            .accepted_failure_domains
            .iter()
            .map(|(kind, value)| ExternalAcceptedFailureDomainV1 {
                kind: *kind,
                value: value.clone(),
            })
            .collect::<Vec<_>>();
        if candidate.accepted_failure_domains != expected_domains {
            return Err(ExternalPlannerProtocolErrorV1::RequestPoolMismatch);
        }
    }

    pool.validate_selection(&request.baseline_selected_availability_actions)
        .map_err(ExternalPlannerProtocolErrorV1::PolicyRejectedSelection)
}

/// Build the exact external-planner request from the same auditable inputs used
/// by the deterministic baseline.
pub fn build_external_planner_request_v1(
    intent: &StorageIntentV1,
    pool: &PolicyQualifiedPoolV1,
    profile: PlannerNormalizationProfileV1,
    evidence: Vec<CandidateSoftEvidenceV1>,
) -> Result<ExternalPlannerRequestV1, ExternalPlannerProtocolErrorV1> {
    let baseline = plan_deterministic_v1(intent, pool, profile, evidence)?;
    let by_action = pool
        .candidates
        .iter()
        .map(|candidate| (candidate.candidate.availability_action, candidate))
        .collect::<BTreeMap<_, _>>();

    let candidates = baseline
        .ranking
        .iter()
        .enumerate()
        .map(|(index, score)| {
            let pool_candidate = by_action
                .get(&score.availability_action)
                .expect("baseline scores only qualified pool candidates");
            let domains = pool_candidate
                .accepted_failure_domains
                .iter()
                .map(|(kind, value)| ExternalAcceptedFailureDomainV1 {
                    kind: *kind,
                    value: value.clone(),
                })
                .collect::<Vec<_>>();
            ExternalPlannerCandidateV1::from_score(score, index as u64, domains)
        })
        .collect::<Vec<_>>();

    let failure_domain_requirements = pool
        .requirements
        .failure_domains
        .iter()
        .map(|(kind, minimum_distinct)| ExternalFailureDomainRequirementV1 {
            kind: *kind,
            minimum_distinct: *minimum_distinct,
        })
        .collect::<Vec<_>>();

    let mut request = ExternalPlannerRequestV1 {
        schema_version: EXTERNAL_PLANNER_PROTOCOL_V1,
        id: ExternalPlannerRequestIdV1([0; 32]),
        planner_input_id: baseline.planner_input_id,
        storage_intent_id: pool.storage_intent_id,
        target: pool.target,
        evaluated_at_unix_ms: pool.evaluated_at_unix_ms,
        profile: profile.into(),
        preferences: intent.preferences.into(),
        minimum_replicas: pool.requirements.minimum_replicas,
        failure_domain_requirements,
        baseline_proposal_id: baseline.id,
        baseline_selected_availability_actions: baseline.selected_availability_actions,
        candidates,
    };
    request.id = request_id(&request);
    validate_request_against_pool(intent, profile, pool, &request)?;
    Ok(request)
}

fn validate_complete_ranking(
    request: &ExternalPlannerRequestV1,
    ranking: &[ActionRefV1],
) -> Result<BTreeMap<ActionRefV1, usize>, ExternalPlannerProtocolErrorV1> {
    let known = request
        .candidates
        .iter()
        .map(|candidate| candidate.availability_action)
        .collect::<BTreeSet<_>>();
    let mut positions = BTreeMap::new();
    for (index, action) in ranking.iter().enumerate() {
        if !known.contains(action) {
            return Err(ExternalPlannerProtocolErrorV1::UnknownRankingAction { action: *action });
        }
        if positions.insert(*action, index).is_some() {
            return Err(ExternalPlannerProtocolErrorV1::DuplicateRankingAction { action: *action });
        }
    }
    if positions.len() != known.len() || ranking.len() != known.len() {
        return Err(ExternalPlannerProtocolErrorV1::IncompleteRanking);
    }
    Ok(positions)
}

/// Accept a recommendation only after identity/replay checks and authoritative
/// CF-06A subset validation. The result remains recommendation-only.
pub fn accept_external_recommendation_v1(
    intent: &StorageIntentV1,
    profile: PlannerNormalizationProfileV1,
    pool: &PolicyQualifiedPoolV1,
    request: &ExternalPlannerRequestV1,
    recommendation: &ExternalPlannerRecommendationV1,
) -> Result<AcceptedExternalPlannerRecommendationV1, ExternalPlannerProtocolErrorV1> {
    validate_request_against_pool(intent, profile, pool, request)?;
    if recommendation.schema_version != EXTERNAL_PLANNER_PROTOCOL_V1 {
        return Err(ExternalPlannerProtocolErrorV1::UnsupportedSchemaVersion);
    }
    if recommendation.request_id != request.id {
        return Err(ExternalPlannerProtocolErrorV1::RequestIdMismatch);
    }
    if recommendation.planner_input_id != request.planner_input_id {
        return Err(ExternalPlannerProtocolErrorV1::PlannerInputMismatch);
    }
    if !valid_token(&recommendation.engine_id) || !valid_token(&recommendation.engine_version) {
        return Err(ExternalPlannerProtocolErrorV1::InvalidEngineToken);
    }

    let positions = validate_complete_ranking(request, &recommendation.ranking)?;
    let mut selected_seen = BTreeSet::new();
    let mut previous_position = None;
    for action in &recommendation.selected_availability_actions {
        if !selected_seen.insert(*action) {
            return Err(ExternalPlannerProtocolErrorV1::DuplicateSelectedAction { action: *action });
        }
        let Some(position) = positions.get(action).copied() else {
            return Err(ExternalPlannerProtocolErrorV1::UnknownSelectedAction { action: *action });
        };
        if previous_position.is_some_and(|previous| position <= previous) {
            return Err(ExternalPlannerProtocolErrorV1::SelectedOrderDoesNotFollowRanking);
        }
        previous_position = Some(position);
    }

    pool.validate_selection(&recommendation.selected_availability_actions)
        .map_err(ExternalPlannerProtocolErrorV1::PolicyRejectedSelection)?;

    let mut accepted = AcceptedExternalPlannerRecommendationV1 {
        schema_version: EXTERNAL_PLANNER_PROTOCOL_V1,
        id: ExternalPlannerAcceptanceIdV1([0; 32]),
        request_id: request.id,
        planner_input_id: request.planner_input_id,
        storage_intent_id: request.storage_intent_id,
        target: request.target,
        evaluated_at_unix_ms: request.evaluated_at_unix_ms,
        engine_id: recommendation.engine_id.clone(),
        engine_version: recommendation.engine_version.clone(),
        authority: PlannerAuthorityV1::RecommendationOnly,
        ranking: recommendation.ranking.clone(),
        selected_availability_actions: recommendation.selected_availability_actions.clone(),
    };
    accepted.id = acceptance_id(&accepted);
    Ok(accepted)
}

impl ExternalPlannerRequestV1 {
    pub fn recompute_id(&self) -> ExternalPlannerRequestIdV1 {
        request_id(self)
    }

    pub fn validate_id(&self) -> bool {
        self.schema_version == EXTERNAL_PLANNER_PROTOCOL_V1 && self.id == self.recompute_id()
    }
}

impl AcceptedExternalPlannerRecommendationV1 {
    pub fn recompute_id(&self) -> ExternalPlannerAcceptanceIdV1 {
        acceptance_id(self)
    }

    pub fn validate_id(&self) -> bool {
        self.schema_version == EXTERNAL_PLANNER_PROTOCOL_V1 && self.id == self.recompute_id()
    }
}

/// JSON is transport encoding only. Do not use the encoded byte sequence as a
/// stable identity; use `ExternalPlannerRequestIdV1` instead.
pub fn encode_request_json_v1(
    value: &ExternalPlannerRequestV1,
) -> Result<Vec<u8>, ExternalPlannerProtocolErrorV1> {
    serde_json::to_vec(value).map_err(|error| ExternalPlannerProtocolErrorV1::Json(error.to_string()))
}

pub fn decode_request_json_v1(
    bytes: &[u8],
) -> Result<ExternalPlannerRequestV1, ExternalPlannerProtocolErrorV1> {
    serde_json::from_slice(bytes)
        .map_err(|error| ExternalPlannerProtocolErrorV1::Json(error.to_string()))
}

pub fn encode_recommendation_json_v1(
    value: &ExternalPlannerRecommendationV1,
) -> Result<Vec<u8>, ExternalPlannerProtocolErrorV1> {
    serde_json::to_vec(value).map_err(|error| ExternalPlannerProtocolErrorV1::Json(error.to_string()))
}

pub fn decode_recommendation_json_v1(
    bytes: &[u8],
) -> Result<ExternalPlannerRecommendationV1, ExternalPlannerProtocolErrorV1> {
    serde_json::from_slice(bytes)
        .map_err(|error| ExternalPlannerProtocolErrorV1::Json(error.to_string()))
}
