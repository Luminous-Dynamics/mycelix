use std::collections::{BTreeMap, BTreeSet};

use mycelix_content_core::{
    ContentDigestV1, DigestAlgorithmV1, EncryptionRequirementV1, FailureDomainKindV1,
    FailureDomainPolicyV1, ObjectIdV1, PlacementPreferencesV1, PlacementRequirementsV1,
    RetentionRequirementV1, StorageClassV1, StorageIntentV1,
};
use mycelix_content_planner::{CandidateSoftEvidenceV1, PlannerNormalizationProfileV1};
use mycelix_content_planner_protocol::*;
use mycelix_content_policy::{PlacementTargetV1, PolicyEligibleCandidateV1, PolicyQualifiedPoolV1};
use mycelix_content_state::{
    ActionRefV1, AgentRefV1, SnapshotServiceCandidateV1, TimestampMicrosV1,
    WithdrawalObservationV1,
};
use mycelix_infrastructure_types::{PartyIdV1, UnixMillisV1};

fn action(value: u8) -> ActionRefV1 {
    ActionRefV1([value; 39])
}

fn agent(value: u8) -> AgentRefV1 {
    AgentRefV1([value; 39])
}

fn digest() -> ContentDigestV1 {
    ContentDigestV1::compute(DigestAlgorithmV1::Blake3_256, b"external-planner-target")
}

fn intent(minimum_replicas: u16, require_two_sites: bool) -> StorageIntentV1 {
    let failure_domains = if require_two_sites {
        FailureDomainPolicyV1::new(vec![(FailureDomainKindV1::Site, 2)]).unwrap()
    } else {
        FailureDomainPolicyV1::empty()
    };
    StorageIntentV1::new(
        ObjectIdV1([7; 32]),
        PartyIdV1([8; 32]),
        StorageClassV1::Durable,
        PlacementRequirementsV1::new(
            minimum_replicas,
            failure_domains,
            BTreeSet::new(),
            BTreeSet::new(),
            EncryptionRequirementV1::NotRequired,
            RetentionRequirementV1::BestEffort,
        )
        .unwrap(),
        PlacementPreferencesV1::new(Some(50), 60, 20, 10, 10).unwrap(),
        UnixMillisV1(1_000),
    )
    .unwrap()
}

fn target() -> PlacementTargetV1 {
    PlacementTargetV1 {
        object_id: ObjectIdV1([7; 32]),
        digest: digest(),
        size_bytes: 100,
        client_side_encrypted: false,
    }
}

fn candidate(
    availability: u8,
    advertisement: u8,
    provider: u8,
    site: Option<&str>,
) -> PolicyEligibleCandidateV1 {
    let mut accepted_failure_domains = BTreeMap::new();
    if let Some(site) = site {
        accepted_failure_domains.insert(FailureDomainKindV1::Site, site.to_string());
    }
    PolicyEligibleCandidateV1 {
        candidate: SnapshotServiceCandidateV1 {
            availability_action: action(availability),
            advertisement_action: action(advertisement),
            provider: agent(provider),
            iroh_endpoint_id: [provider; 32],
            digest: digest(),
            size_bytes: 100,
            failure_domains: Vec::new(),
            claim_authored_at: TimestampMicrosV1(2_000_000),
            effective_until: TimestampMicrosV1(60_000_000),
            withdrawal: WithdrawalObservationV1::NoWithdrawalObserved,
        },
        accepted_jurisdictions: BTreeSet::new(),
        accepted_failure_domains,
    }
}

fn pool(intent: &StorageIntentV1, candidates: Vec<PolicyEligibleCandidateV1>) -> PolicyQualifiedPoolV1 {
    PolicyQualifiedPoolV1 {
        storage_intent_id: intent.id,
        target: target(),
        evaluated_at: TimestampMicrosV1(10_000_000),
        evaluated_at_unix_ms: 10_000,
        requirements: intent.requirements.clone(),
        candidates,
    }
}

fn profile() -> PlannerNormalizationProfileV1 {
    PlannerNormalizationProfileV1::new(100, 200, 1_000, 10_000).unwrap()
}

fn soft(
    availability: u8,
    advertisement: u8,
    provider: u8,
    cost: u64,
) -> CandidateSoftEvidenceV1 {
    CandidateSoftEvidenceV1 {
        availability_action: action(availability),
        advertisement_action: action(advertisement),
        provider: agent(provider),
        observed_at_unix_ms: 9_000,
        valid_until_unix_ms: 20_000,
        cost_microunits_per_gib: Some(cost),
        latency_ms: Some(40),
        energy_millijoules_per_gib: Some(200),
        locality_distance_km: Some(100),
    }
}

fn make_fixture() -> (
    StorageIntentV1,
    PolicyQualifiedPoolV1,
    PlannerNormalizationProfileV1,
    Vec<CandidateSoftEvidenceV1>,
) {
    let intent = intent(2, true);
    let pool = pool(
        &intent,
        vec![
            candidate(1, 11, 21, Some("site-a")),
            candidate(2, 12, 22, Some("site-a")),
            candidate(3, 13, 23, Some("site-b")),
        ],
    );
    let evidence = vec![soft(1, 11, 21, 10), soft(2, 12, 22, 20), soft(3, 13, 23, 90)];
    (intent, pool, profile(), evidence)
}

fn recommendation_from_request(request: &ExternalPlannerRequestV1) -> ExternalPlannerRecommendationV1 {
    let ranking = request
        .candidates
        .iter()
        .map(|candidate| candidate.availability_action)
        .collect::<Vec<_>>();
    ExternalPlannerRecommendationV1 {
        schema_version: EXTERNAL_PLANNER_PROTOCOL_V1,
        request_id: request.id,
        planner_input_id: request.planner_input_id,
        engine_id: "symthaea-content".to_string(),
        engine_version: "v1".to_string(),
        ranking,
        selected_availability_actions: request.baseline_selected_availability_actions.clone(),
    }
}

#[test]
fn request_json_round_trip_preserves_stable_id() {
    let (intent, pool, profile, evidence) = make_fixture();
    let request = build_external_planner_request_v1(&intent, &pool, profile, evidence).unwrap();
    let bytes = encode_request_json_v1(&request).unwrap();
    let decoded = decode_request_json_v1(&bytes).unwrap();
    assert_eq!(decoded, request);
    assert!(decoded.validate_id());
}

#[test]
fn request_id_detects_transport_field_mutation() {
    let (intent, pool, profile, evidence) = make_fixture();
    let mut request = build_external_planner_request_v1(&intent, &pool, profile, evidence).unwrap();
    assert!(request.validate_id());
    request.candidates[0].baseline_weighted_penalty += 1;
    assert!(!request.validate_id());
}

#[test]
fn unknown_candidate_telemetry_does_not_perturb_request_identity() {
    let (intent, pool, profile, evidence) = make_fixture();
    let baseline = build_external_planner_request_v1(&intent, &pool, profile, evidence.clone()).unwrap();
    let mut noisy = evidence;
    noisy.push(soft(99, 98, 97, 0));
    let with_noise = build_external_planner_request_v1(&intent, &pool, profile, noisy).unwrap();
    assert_eq!(baseline, with_noise);
}

#[test]
fn response_for_an_old_request_cannot_be_replayed_against_a_new_request() {
    let (intent, pool, profile, mut evidence) = make_fixture();
    let old_request = build_external_planner_request_v1(&intent, &pool, profile, evidence.clone()).unwrap();
    let old_recommendation = recommendation_from_request(&old_request);

    evidence[0].cost_microunits_per_gib = Some(99);
    let new_request = build_external_planner_request_v1(&intent, &pool, profile, evidence).unwrap();
    assert_ne!(old_request.id, new_request.id);

    assert_eq!(
        accept_external_recommendation_v1(
            &intent,
            profile,
            &pool,
            &new_request,
            &old_recommendation,
        ),
        Err(ExternalPlannerProtocolErrorV1::RequestIdMismatch)
    );
}

#[test]
fn ranking_must_be_a_complete_duplicate_free_candidate_permutation() {
    let (intent, pool, profile, evidence) = make_fixture();
    let request = build_external_planner_request_v1(&intent, &pool, profile, evidence).unwrap();
    let mut recommendation = recommendation_from_request(&request);
    recommendation.ranking[1] = recommendation.ranking[0];

    assert!(matches!(
        accept_external_recommendation_v1(&intent, profile, &pool, &request, &recommendation),
        Err(ExternalPlannerProtocolErrorV1::DuplicateRankingAction { .. })
    ));

    let mut recommendation = recommendation_from_request(&request);
    recommendation.ranking.pop();
    assert_eq!(
        accept_external_recommendation_v1(&intent, profile, &pool, &request, &recommendation),
        Err(ExternalPlannerProtocolErrorV1::IncompleteRanking)
    );
}

#[test]
fn external_planner_cannot_return_a_same_site_policy_bad_subset() {
    let (intent, pool, profile, evidence) = make_fixture();
    let request = build_external_planner_request_v1(&intent, &pool, profile, evidence).unwrap();
    let mut recommendation = recommendation_from_request(&request);
    recommendation.selected_availability_actions = vec![action(1), action(2)];

    assert!(matches!(
        accept_external_recommendation_v1(&intent, profile, &pool, &request, &recommendation),
        Err(ExternalPlannerProtocolErrorV1::PolicyRejectedSelection(_))
    ));
}

#[test]
fn selected_subset_must_preserve_external_ranking_order() {
    let (intent, pool, profile, evidence) = make_fixture();
    let request = build_external_planner_request_v1(&intent, &pool, profile, evidence).unwrap();
    let mut recommendation = recommendation_from_request(&request);
    recommendation.selected_availability_actions = vec![recommendation.ranking[2], recommendation.ranking[0]];

    assert_eq!(
        accept_external_recommendation_v1(&intent, profile, &pool, &request, &recommendation),
        Err(ExternalPlannerProtocolErrorV1::SelectedOrderDoesNotFollowRanking)
    );
}

#[test]
fn accepted_recommendation_is_stably_committed_and_remains_non_authoritative() {
    let (intent, pool, profile, evidence) = make_fixture();
    let request = build_external_planner_request_v1(&intent, &pool, profile, evidence).unwrap();
    let recommendation = recommendation_from_request(&request);
    let mut accepted = accept_external_recommendation_v1(
        &intent,
        profile,
        &pool,
        &request,
        &recommendation,
    )
    .unwrap();

    assert!(accepted.validate_id());
    assert_eq!(
        accepted.authority,
        mycelix_content_planner::PlannerAuthorityV1::RecommendationOnly
    );
    accepted.engine_version = "v2".to_string();
    assert!(!accepted.validate_id());
}

#[test]
fn recommendation_json_round_trip_is_interoperable_transport_only() {
    let (intent, pool, profile, evidence) = make_fixture();
    let request = build_external_planner_request_v1(&intent, &pool, profile, evidence).unwrap();
    let recommendation = recommendation_from_request(&request);
    let bytes = encode_recommendation_json_v1(&recommendation).unwrap();
    assert_eq!(decode_recommendation_json_v1(&bytes).unwrap(), recommendation);
}
