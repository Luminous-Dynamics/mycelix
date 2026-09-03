use std::collections::BTreeSet;

use mycelix_content_core::{FailureDomainKindV1, StorageIntentV1};
use mycelix_content_policy::PolicyQualifiedPoolV1;
use mycelix_content_state::{ActionRefV1, WithdrawalObservationV1};

use crate::score::{normalize_soft_evidence, score_candidate_v1};
use crate::{
    CandidateScoreV1, CandidateSoftEvidenceV1, PlacementProposalIdV1, PlacementProposalV1,
    PlannerAuthorityV1, PlannerErrorV1, PlannerInputIdV1, PlannerNormalizationProfileV1,
    PlannerSelectionStrategyV1, SoftEvidenceStateV1, SoftMetricKindV1, PLANNER_SCHEMA_V1,
};

const INPUT_MAGIC: &[u8] = b"MYCELIX-CONTENT-PLANNER-INPUT\0";
const PROPOSAL_MAGIC: &[u8] = b"MYCELIX-CONTENT-PLANNER-PROPOSAL\0";

fn put_field(hasher: &mut blake3::Hasher, value: &[u8]) {
    hasher.update(&(value.len() as u64).to_be_bytes());
    hasher.update(value);
}

fn put_option_u64(hasher: &mut blake3::Hasher, value: Option<u64>) {
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

fn planner_input_id(
    pool: &PolicyQualifiedPoolV1,
    evidence: &[CandidateSoftEvidenceV1],
) -> PlannerInputIdV1 {
    let mut hasher = blake3::Hasher::new();
    hasher.update(INPUT_MAGIC);
    hasher.update(&PLANNER_SCHEMA_V1.to_be_bytes());
    hasher.update(&pool.storage_intent_id.0);
    hasher.update(&pool.target.object_id.0);
    put_field(&mut hasher, pool.target.digest.algorithm.tag().as_bytes());
    hasher.update(&pool.target.digest.bytes);
    hasher.update(&pool.target.size_bytes.to_be_bytes());
    hasher.update(&[u8::from(pool.target.client_side_encrypted)]);
    hasher.update(&pool.evaluated_at.0.to_be_bytes());
    hasher.update(&pool.evaluated_at_unix_ms.to_be_bytes());

    let mut candidates = pool.candidates.iter().collect::<Vec<_>>();
    candidates.sort_by_key(|candidate| candidate.candidate.availability_action);
    hasher.update(&(candidates.len() as u64).to_be_bytes());
    for candidate in candidates {
        hasher.update(&candidate.candidate.availability_action.0);
        hasher.update(&candidate.candidate.advertisement_action.0);
        hasher.update(&candidate.candidate.provider.0);
        hasher.update(&candidate.candidate.iroh_endpoint_id);
        hasher.update(&candidate.candidate.claim_authored_at.0.to_be_bytes());
        hasher.update(&candidate.candidate.effective_until.0.to_be_bytes());

        hasher.update(&(candidate.accepted_jurisdictions.len() as u64).to_be_bytes());
        for jurisdiction in &candidate.accepted_jurisdictions {
            put_field(&mut hasher, jurisdiction.as_str().as_bytes());
        }
        hasher.update(&(candidate.accepted_failure_domains.len() as u64).to_be_bytes());
        for (kind, value) in &candidate.accepted_failure_domains {
            hasher.update(&[failure_domain_tag(*kind)]);
            put_field(&mut hasher, value.as_bytes());
        }
    }

    let candidate_actions: BTreeSet<_> = pool
        .candidates
        .iter()
        .map(|candidate| candidate.candidate.availability_action)
        .collect();
    let mut relevant = evidence
        .iter()
        .filter(|item| candidate_actions.contains(&item.availability_action))
        .cloned()
        .collect::<Vec<_>>();
    relevant.sort();
    relevant.dedup();
    hasher.update(&(relevant.len() as u64).to_be_bytes());
    for item in relevant {
        hasher.update(&item.availability_action.0);
        hasher.update(&item.advertisement_action.0);
        hasher.update(&item.provider.0);
        hasher.update(&item.observed_at_unix_ms.to_be_bytes());
        hasher.update(&item.valid_until_unix_ms.to_be_bytes());
        put_option_u64(&mut hasher, item.cost_microunits_per_gib);
        put_option_u32(&mut hasher, item.latency_ms);
        put_option_u64(&mut hasher, item.energy_millijoules_per_gib);
        put_option_u32(&mut hasher, item.locality_distance_km);
    }

    PlannerInputIdV1(*hasher.finalize().as_bytes())
}

fn proposal_id(proposal: &PlacementProposalV1) -> PlacementProposalIdV1 {
    let mut hasher = blake3::Hasher::new();
    hasher.update(PROPOSAL_MAGIC);
    hasher.update(&PLANNER_SCHEMA_V1.to_be_bytes());
    hasher.update(&proposal.planner_input_id.0);
    hasher.update(&proposal.profile_id.0);
    hasher.update(&proposal.storage_intent_id.0);
    hasher.update(&proposal.target.object_id.0);
    put_field(&mut hasher, proposal.target.digest.algorithm.tag().as_bytes());
    hasher.update(&proposal.target.digest.bytes);
    hasher.update(&proposal.target.size_bytes.to_be_bytes());
    hasher.update(&[u8::from(proposal.target.client_side_encrypted)]);
    hasher.update(&proposal.evaluated_at_unix_ms.to_be_bytes());
    hasher.update(&[match proposal.authority {
        PlannerAuthorityV1::RecommendationOnly => 0,
    }]);
    hasher.update(&[match proposal.strategy {
        PlannerSelectionStrategyV1::PolicyPreservingEliminationV1 => 0,
    }]);

    hasher.update(&(proposal.ranking.len() as u64).to_be_bytes());
    for score in &proposal.ranking {
        hasher.update(&score.availability_action.0);
        hasher.update(&score.advertisement_action.0);
        hasher.update(&score.provider.0);
        hasher.update(&score.weighted_penalty.to_be_bytes());
        hasher.update(&score.cost_penalty_ppm.to_be_bytes());
        hasher.update(&score.latency_penalty_ppm.to_be_bytes());
        hasher.update(&score.energy_penalty_ppm.to_be_bytes());
        hasher.update(&score.locality_penalty_ppm.to_be_bytes());
        hasher.update(&[evidence_state_tag(score.evidence_state)]);
        hasher.update(&(score.missing_weighted_metrics.len() as u64).to_be_bytes());
        for kind in &score.missing_weighted_metrics {
            hasher.update(&[metric_kind_tag(*kind)]);
        }
    }

    hasher.update(&(proposal.selected_availability_actions.len() as u64).to_be_bytes());
    for action in &proposal.selected_availability_actions {
        hasher.update(&action.0);
    }

    PlacementProposalIdV1(*hasher.finalize().as_bytes())
}

fn validate_pool_binding(
    intent: &StorageIntentV1,
    pool: &PolicyQualifiedPoolV1,
) -> Result<(), PlannerErrorV1> {
    if intent.validate().is_err() {
        return Err(PlannerErrorV1::InvalidStorageIntent);
    }
    if pool.storage_intent_id != intent.id {
        return Err(PlannerErrorV1::IntentPoolMismatch);
    }
    if pool.requirements != intent.requirements {
        return Err(PlannerErrorV1::RequirementsPoolMismatch);
    }
    if pool.target.object_id != intent.object_id {
        return Err(PlannerErrorV1::TargetPoolMismatch);
    }
    if pool.evaluated_at.0 < 0
        || u64::try_from(pool.evaluated_at.0 / 1_000).ok() != Some(pool.evaluated_at_unix_ms)
    {
        return Err(PlannerErrorV1::InvalidPoolTimestamp);
    }

    for candidate in &pool.candidates {
        if candidate.candidate.digest != pool.target.digest
            || candidate.candidate.size_bytes != pool.target.size_bytes
            || candidate.candidate.claim_authored_at > pool.evaluated_at
            || candidate.candidate.effective_until <= pool.evaluated_at
            || !matches!(
                &candidate.candidate.withdrawal,
                WithdrawalObservationV1::NoWithdrawalObserved
            )
        {
            return Err(PlannerErrorV1::InvalidQualifiedCandidate);
        }
    }

    let all_actions = pool
        .candidates
        .iter()
        .map(|candidate| candidate.candidate.availability_action)
        .collect::<Vec<_>>();
    pool.validate_selection(&all_actions)
        .map_err(PlannerErrorV1::InvalidQualifiedPoolSelection)
}

fn policy_preserving_selection(
    pool: &PolicyQualifiedPoolV1,
    ranking: &[CandidateScoreV1],
) -> Result<Vec<ActionRefV1>, PlannerErrorV1> {
    let mut selected = ranking
        .iter()
        .map(|score| score.availability_action)
        .collect::<Vec<_>>();

    for score in ranking.iter().rev() {
        let tentative = selected
            .iter()
            .copied()
            .filter(|action| *action != score.availability_action)
            .collect::<Vec<_>>();
        if pool.validate_selection(&tentative).is_ok() {
            selected = tentative;
        }
    }

    pool.validate_selection(&selected)
        .map_err(PlannerErrorV1::FinalSelectionRejected)?;
    Ok(selected)
}

/// Produce a deterministic recommendation from a hard-policy-qualified pool.
///
/// Soft evidence can only influence ranking. Every returned subset is validated
/// again through CF-06A before the recommendation is emitted.
pub fn plan_deterministic_v1(
    intent: &StorageIntentV1,
    pool: &PolicyQualifiedPoolV1,
    profile: PlannerNormalizationProfileV1,
    evidence: Vec<CandidateSoftEvidenceV1>,
) -> Result<PlacementProposalV1, PlannerErrorV1> {
    profile.validate()?;
    validate_pool_binding(intent, pool)?;

    let planner_input_id = planner_input_id(pool, &evidence);
    let normalized = normalize_soft_evidence(evidence);
    let preferences = intent.preferences;

    let mut ranking = pool
        .candidates
        .iter()
        .map(|candidate| {
            score_candidate_v1(
                candidate,
                normalized.get(&candidate.candidate.availability_action),
                pool.evaluated_at_unix_ms,
                preferences,
                profile,
            )
        })
        .collect::<Vec<_>>();
    ranking.sort_by_key(|score| (score.weighted_penalty, score.availability_action));

    let selected_availability_actions = policy_preserving_selection(pool, &ranking)?;

    let mut proposal = PlacementProposalV1 {
        schema_version: PLANNER_SCHEMA_V1,
        id: PlacementProposalIdV1([0; 32]),
        planner_input_id,
        storage_intent_id: pool.storage_intent_id,
        target: pool.target,
        evaluated_at_unix_ms: pool.evaluated_at_unix_ms,
        profile_id: profile.id,
        authority: PlannerAuthorityV1::RecommendationOnly,
        strategy: PlannerSelectionStrategyV1::PolicyPreservingEliminationV1,
        ranking,
        selected_availability_actions,
    };
    proposal.id = proposal_id(&proposal);
    Ok(proposal)
}

impl PlacementProposalV1 {
    pub fn recompute_id(&self) -> PlacementProposalIdV1 {
        proposal_id(self)
    }

    pub fn validate_id(&self) -> bool {
        self.schema_version == PLANNER_SCHEMA_V1 && self.id == self.recompute_id()
    }
}
