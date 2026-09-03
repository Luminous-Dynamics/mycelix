use std::collections::{BTreeMap, BTreeSet};

use mycelix_content_core::{
    ContentDigestV1, DigestAlgorithmV1, EncryptionRequirementV1, FailureDomainKindV1,
    FailureDomainPolicyV1, ObjectIdV1, PlacementPreferencesV1, PlacementRequirementsV1,
    RetentionRequirementV1, StorageClassV1, StorageIntentV1,
};
use mycelix_content_planner::*;
use mycelix_content_policy::{PlacementTargetV1, PolicyEligibleCandidateV1, PolicyQualifiedPoolV1};
use mycelix_content_state::{
    ActionRefV1, AgentRefV1, SnapshotServiceCandidateV1, TimestampMicrosV1,
    WithdrawalObservationV1,
};
use mycelix_infrastructure_types::{PartyIdV1, UnixMillisV1};
use proptest::prelude::*;

fn action(value: u8) -> ActionRefV1 {
    ActionRefV1([value; 39])
}

fn agent(value: u8) -> AgentRefV1 {
    AgentRefV1([value; 39])
}

fn digest() -> ContentDigestV1 {
    ContentDigestV1::compute(DigestAlgorithmV1::Blake3_256, b"planner-target")
}

fn preferences(cost: u16, latency: u16, energy: u16, locality: u16) -> PlacementPreferencesV1 {
    PlacementPreferencesV1::new(Some(50), cost, latency, energy, locality).unwrap()
}

fn intent(
    minimum_replicas: u16,
    failure_domains: Vec<(FailureDomainKindV1, u16)>,
    preferences: PlacementPreferencesV1,
) -> StorageIntentV1 {
    StorageIntentV1::new(
        ObjectIdV1([7; 32]),
        PartyIdV1([8; 32]),
        StorageClassV1::Durable,
        PlacementRequirementsV1::new(
            minimum_replicas,
            FailureDomainPolicyV1::new(failure_domains).unwrap(),
            BTreeSet::new(),
            BTreeSet::new(),
            EncryptionRequirementV1::NotRequired,
            RetentionRequirementV1::BestEffort,
        )
        .unwrap(),
        preferences,
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

fn eligible(
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

fn evidence(
    availability: u8,
    advertisement: u8,
    provider: u8,
    cost: Option<u64>,
    latency: Option<u32>,
    energy: Option<u64>,
    locality: Option<u32>,
) -> CandidateSoftEvidenceV1 {
    CandidateSoftEvidenceV1 {
        availability_action: action(availability),
        advertisement_action: action(advertisement),
        provider: agent(provider),
        observed_at_unix_ms: 9_000,
        valid_until_unix_ms: 20_000,
        cost_microunits_per_gib: cost,
        latency_ms: latency,
        energy_millijoules_per_gib: energy,
        locality_distance_km: locality,
    }
}

#[test]
fn cost_only_baseline_selects_the_cheapest_policy_valid_candidate() {
    let intent = intent(1, Vec::new(), preferences(100, 0, 0, 0));
    let pool = pool(
        &intent,
        vec![eligible(1, 11, 21, None), eligible(2, 12, 22, None), eligible(3, 13, 23, None)],
    );
    let proposal = plan_deterministic_v1(
        &intent,
        &pool,
        profile(),
        vec![
            evidence(1, 11, 21, Some(100), None, None, None),
            evidence(2, 12, 22, Some(20), None, None, None),
            evidence(3, 13, 23, Some(50), None, None, None),
        ],
    )
    .unwrap();

    assert_eq!(proposal.ranking[0].availability_action, action(2));
    assert_eq!(proposal.selected_availability_actions, vec![action(2)]);
    assert!(proposal.validate_id());
}

#[test]
fn missing_weighted_metric_cannot_improve_rank() {
    let intent = intent(1, Vec::new(), preferences(100, 0, 0, 0));
    let pool = pool(&intent, vec![eligible(1, 11, 21, None), eligible(2, 12, 22, None)]);
    let proposal = plan_deterministic_v1(
        &intent,
        &pool,
        profile(),
        vec![evidence(1, 11, 21, Some(80), None, None, None)],
    )
    .unwrap();

    assert_eq!(proposal.ranking[0].availability_action, action(1));
    assert_eq!(proposal.ranking[1].evidence_state, SoftEvidenceStateV1::Missing);
    assert_eq!(proposal.ranking[1].cost_penalty_ppm, PENALTY_PPM_MAX);
}

#[test]
fn stale_telemetry_is_conservatively_treated_as_unusable() {
    let intent = intent(1, Vec::new(), preferences(100, 0, 0, 0));
    let pool = pool(&intent, vec![eligible(1, 11, 21, None), eligible(2, 12, 22, None)]);
    let mut stale = evidence(1, 11, 21, Some(1), None, None, None);
    stale.valid_until_unix_ms = 10_000;
    let proposal = plan_deterministic_v1(
        &intent,
        &pool,
        profile(),
        vec![stale, evidence(2, 12, 22, Some(70), None, None, None)],
    )
    .unwrap();

    assert_eq!(proposal.ranking[0].availability_action, action(2));
    let stale_score = proposal
        .ranking
        .iter()
        .find(|score| score.availability_action == action(1))
        .unwrap();
    assert_eq!(stale_score.evidence_state, SoftEvidenceStateV1::Stale);
    assert_eq!(stale_score.cost_penalty_ppm, PENALTY_PPM_MAX);
}

#[test]
fn soft_scoring_cannot_remove_required_site_diversity() {
    let intent = intent(
        2,
        vec![(FailureDomainKindV1::Site, 2)],
        preferences(100, 0, 0, 0),
    );
    let pool = pool(
        &intent,
        vec![
            eligible(1, 11, 21, Some("site-a")),
            eligible(2, 12, 22, Some("site-a")),
            eligible(3, 13, 23, Some("site-b")),
        ],
    );
    let proposal = plan_deterministic_v1(
        &intent,
        &pool,
        profile(),
        vec![
            evidence(1, 11, 21, Some(10), None, None, None),
            evidence(2, 12, 22, Some(20), None, None, None),
            evidence(3, 13, 23, Some(100), None, None, None),
        ],
    )
    .unwrap();

    assert_eq!(proposal.selected_availability_actions, vec![action(1), action(3)]);
    assert!(pool
        .validate_selection(&proposal.selected_availability_actions)
        .is_ok());
}

#[test]
fn conflicting_soft_evidence_only_hurts_rank_and_never_changes_hard_policy() {
    let intent = intent(1, Vec::new(), preferences(100, 0, 0, 0));
    let pool = pool(&intent, vec![eligible(1, 11, 21, None), eligible(2, 12, 22, None)]);
    let proposal = plan_deterministic_v1(
        &intent,
        &pool,
        profile(),
        vec![
            evidence(1, 11, 21, Some(1), None, None, None),
            evidence(1, 11, 21, Some(2), None, None, None),
            evidence(2, 12, 22, Some(90), None, None, None),
        ],
    )
    .unwrap();

    assert_eq!(proposal.ranking[0].availability_action, action(2));
    let conflicted = proposal
        .ranking
        .iter()
        .find(|score| score.availability_action == action(1))
        .unwrap();
    assert_eq!(conflicted.evidence_state, SoftEvidenceStateV1::Conflicting);
    assert_eq!(proposal.selected_availability_actions, vec![action(2)]);
}

#[test]
fn planner_rejects_preferences_from_a_different_storage_intent() {
    let original = intent(1, Vec::new(), preferences(100, 0, 0, 0));
    let swapped = intent(1, Vec::new(), preferences(0, 100, 0, 0));
    let pool = pool(&original, vec![eligible(1, 11, 21, None)]);

    assert_eq!(
        plan_deterministic_v1(&swapped, &pool, profile(), Vec::new()),
        Err(PlannerErrorV1::IntentPoolMismatch)
    );
}

#[test]
fn proposal_id_detects_post_plan_mutation() {
    let intent = intent(1, Vec::new(), preferences(100, 0, 0, 0));
    let pool = pool(&intent, vec![eligible(1, 11, 21, None), eligible(2, 12, 22, None)]);
    let mut proposal = plan_deterministic_v1(&intent, &pool, profile(), Vec::new()).unwrap();
    assert!(proposal.validate_id());
    proposal.selected_availability_actions.push(action(99));
    assert!(!proposal.validate_id());
}

proptest! {
    #[test]
    fn candidate_and_evidence_order_do_not_change_the_proposal(
        reverse_candidates in any::<bool>(),
        reverse_evidence in any::<bool>()
    ) {
        let intent = intent(
            2,
            vec![(FailureDomainKindV1::Site, 2)],
            preferences(50, 30, 10, 10),
        );
        let mut candidates = vec![
            eligible(1, 11, 21, Some("site-a")),
            eligible(2, 12, 22, Some("site-b")),
            eligible(3, 13, 23, Some("site-c")),
        ];
        let mut evidence = vec![
            evidence(1, 11, 21, Some(20), Some(40), Some(300), Some(100)),
            evidence(2, 12, 22, Some(40), Some(70), Some(200), Some(200)),
            evidence(3, 13, 23, Some(60), Some(30), Some(100), Some(300)),
        ];
        let baseline_pool = pool(&intent, candidates.clone());
        let baseline = plan_deterministic_v1(&intent, &baseline_pool, profile(), evidence.clone()).unwrap();

        if reverse_candidates {
            candidates.reverse();
        }
        if reverse_evidence {
            evidence.reverse();
        }
        let permuted_pool = pool(&intent, candidates);
        let permuted = plan_deterministic_v1(&intent, &permuted_pool, profile(), evidence).unwrap();

        prop_assert_eq!(baseline, permuted);
    }
}
