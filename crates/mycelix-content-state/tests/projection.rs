use mycelix_content_core::{ContentDigestV1, DigestAlgorithmV1};
use mycelix_content_state::*;
use proptest::prelude::*;

const SECOND: i64 = 1_000_000;

fn action(id: u8) -> ActionRefV1 {
    ActionRefV1([id; 39])
}

fn agent(id: u8) -> AgentRefV1 {
    AgentRefV1([id; 39])
}

fn at(seconds: i64) -> TimestampMicrosV1 {
    TimestampMicrosV1(seconds * SECOND)
}

fn digest(seed: u8) -> ContentDigestV1 {
    ContentDigestV1::compute(DigestAlgorithmV1::Blake3_256, &[seed; 8])
}

fn ad(id: u8, provider: u8, authored: i64, ttl: u32) -> ProviderAdvertisementEvidenceV1 {
    ProviderAdvertisementEvidenceV1 {
        action: action(id),
        authored_at: at(authored),
        provider: agent(provider),
        iroh_endpoint_id: [provider; 32],
        supported_algorithms: vec![DigestAlgorithmV1::Blake3_256],
        max_blob_size_bytes: 1024,
        ttl_seconds: ttl,
        failure_domains: Vec::new(),
    }
}

fn claim(
    id: u8,
    provider: u8,
    advertisement: u8,
    authored: i64,
    ttl: u32,
    seed: u8,
) -> AvailabilityEvidenceV1 {
    AvailabilityEvidenceV1 {
        action: action(id),
        authored_at: at(authored),
        provider: agent(provider),
        advertisement: action(advertisement),
        digest: digest(seed),
        size_bytes: 8,
        ttl_seconds: ttl,
    }
}

fn snapshot(
    advertisements: Vec<ProviderAdvertisementEvidenceV1>,
    availability: Vec<AvailabilityEvidenceV1>,
    withdrawals: Vec<WithdrawalEvidenceV1>,
) -> EvidenceSnapshotV1 {
    EvidenceSnapshotV1 {
        coverage: SnapshotCoverageV1::Partial,
        advertisements,
        availability,
        withdrawals,
        observations: Vec::new(),
    }
}

#[test]
fn parent_expiry_clamps_claim_and_boundary_is_half_open() {
    let evidence = snapshot(vec![ad(1, 7, 0, 10)], vec![claim(2, 7, 1, 8, 5, 42)], vec![]);
    let before = project_content_state_v1(evidence.clone(), at(9));
    assert_eq!(before.service_candidates.len(), 1);
    assert_eq!(before.service_candidates[0].effective_until, at(10));

    let at_boundary = project_content_state_v1(evidence, at(10));
    assert!(at_boundary.service_candidates.is_empty());
}

#[test]
fn withdrawal_suppresses_existing_claims_and_diagnoses_later_claims() {
    let withdrawal = WithdrawalEvidenceV1 {
        action: action(3),
        authored_at: at(5),
        provider: agent(7),
        advertisement: action(1),
    };
    let evidence = snapshot(
        vec![ad(1, 7, 0, 20)],
        vec![claim(2, 7, 1, 2, 10, 42), claim(4, 7, 1, 6, 10, 43)],
        vec![withdrawal],
    );

    assert_eq!(
        project_content_state_v1(evidence.clone(), at(4))
            .service_candidates
            .len(),
        1
    );
    let withdrawn = project_content_state_v1(evidence, at(7));
    assert!(withdrawn.service_candidates.is_empty());
    assert!(matches!(
        withdrawn.advertisements[0].withdrawal,
        WithdrawalObservationV1::Withdrawn { authored_at, .. } if authored_at == at(5)
    ));
    assert!(withdrawn.issues.contains(&ProjectionIssueV1 {
        action: action(4),
        kind: ProjectionIssueKindV1::AvailabilityAfterObservedWithdrawal,
    }));
}

#[test]
fn partial_snapshot_never_upgrades_absence_to_not_withdrawn() {
    let state = project_content_state_v1(
        snapshot(vec![ad(1, 7, 0, 20)], vec![claim(2, 7, 1, 1, 10, 42)], vec![]),
        at(2),
    );
    assert_eq!(state.coverage, SnapshotCoverageV1::Partial);
    assert_eq!(state.service_candidates.len(), 1);
    assert_eq!(
        state.service_candidates[0].withdrawal,
        WithdrawalObservationV1::NoWithdrawalObserved
    );
}

#[test]
fn historical_replay_is_causally_isolated_from_future_evidence() {
    let evidence = snapshot(vec![ad(1, 7, 10, 20)], vec![claim(2, 7, 1, 11, 10, 42)], vec![]);
    let before = project_content_state_v1(evidence.clone(), at(9));
    assert!(before.advertisements.is_empty());
    assert!(before.service_candidates.is_empty());
    assert!(before.issues.is_empty());

    let after = project_content_state_v1(evidence, at(11));
    assert_eq!(after.advertisements.len(), 1);
    assert_eq!(after.service_candidates.len(), 1);
}

#[test]
fn future_conflicting_duplicate_cannot_poison_earlier_replay() {
    let current = ad(1, 7, 0, 20);
    let mut future_conflict = current.clone();
    future_conflict.authored_at = at(10);
    future_conflict.max_blob_size_bytes = 999;

    let state = project_content_state_v1(
        snapshot(vec![current], vec![claim(2, 7, 1, 1, 10, 42)], vec![]),
        at(5),
    );
    let with_future_conflict = project_content_state_v1(
        snapshot(
            vec![ad(1, 7, 0, 20), future_conflict],
            vec![claim(2, 7, 1, 1, 10, 42)],
            vec![],
        ),
        at(5),
    );
    assert_eq!(state, with_future_conflict);
}

#[test]
fn missing_parent_is_excluded_with_diagnostic_not_guessed() {
    let evidence = snapshot(vec![], vec![claim(2, 7, 99, 1, 10, 42)], vec![]);
    let state = project_content_state_v1(evidence, at(2));
    assert!(state.service_candidates.is_empty());
    assert_eq!(
        state.issues,
        vec![ProjectionIssueV1 {
            action: action(2),
            kind: ProjectionIssueKindV1::MissingAdvertisement {
                advertisement: action(99)
            }
        }]
    );
}

#[test]
fn conflicting_duplicate_action_is_never_resolved_by_arrival_order() {
    let first = ad(1, 7, 0, 20);
    let mut conflicting = first.clone();
    conflicting.max_blob_size_bytes = 999;
    let evidence = snapshot(vec![first, conflicting], vec![], vec![]);
    let state = project_content_state_v1(evidence, at(1));
    assert!(state.advertisements.is_empty());
    assert_eq!(
        state.issues,
        vec![ProjectionIssueV1 {
            action: action(1),
            kind: ProjectionIssueKindV1::ConflictingDuplicateAction
        }]
    );
}

#[test]
fn availability_authored_after_parent_expiry_is_diagnostic_not_candidate() {
    let evidence = snapshot(vec![ad(1, 7, 0, 5)], vec![claim(2, 7, 1, 6, 2, 42)], vec![]);
    let state = project_content_state_v1(evidence, at(7));
    assert!(state.service_candidates.is_empty());
    assert!(state.issues.contains(&ProjectionIssueV1 {
        action: action(2),
        kind: ProjectionIssueKindV1::AvailabilityAfterAdvertisementExpiry,
    }));
}

#[test]
fn observations_remain_separate_from_service_candidate_truth() {
    let observation = ObservationEvidenceV1 {
        action: action(3),
        authored_at: at(3),
        observer: agent(8),
        provider: agent(7),
        advertisement: action(1),
        digest: digest(42),
        outcome: ObservationOutcomeEvidenceV1::VerifiedComplete { size_bytes: 8 },
        latency_ms: Some(12),
    };
    let mut evidence = snapshot(vec![ad(1, 7, 0, 20)], vec![], vec![]);
    evidence.observations.push(observation);
    let state = project_content_state_v1(evidence, at(5));
    assert!(state.service_candidates.is_empty());
    assert_eq!(state.observations.len(), 1);
    assert_eq!(state.observations[0].age_micros, (2 * SECOND) as u64);
}

proptest! {
    #[test]
    fn projection_is_invariant_to_input_order(flags in any::<u8>()) {
        let mut advertisements = vec![ad(1, 7, 0, 30), ad(10, 9, 0, 30)];
        let mut availability = vec![claim(2, 7, 1, 1, 20, 42), claim(11, 9, 10, 1, 20, 43)];
        let mut withdrawals = vec![
            WithdrawalEvidenceV1 {
                action: action(3),
                authored_at: at(25),
                provider: agent(7),
                advertisement: action(1),
            },
            WithdrawalEvidenceV1 {
                action: action(12),
                authored_at: at(26),
                provider: agent(9),
                advertisement: action(10),
            },
        ];
        let mut observations = vec![
            ObservationEvidenceV1 {
                action: action(4),
                authored_at: at(2),
                observer: agent(8),
                provider: agent(7),
                advertisement: action(1),
                digest: digest(42),
                outcome: ObservationOutcomeEvidenceV1::Busy,
                latency_ms: Some(1),
            },
            ObservationEvidenceV1 {
                action: action(13),
                authored_at: at(2),
                observer: agent(8),
                provider: agent(9),
                advertisement: action(10),
                digest: digest(43),
                outcome: ObservationOutcomeEvidenceV1::TransferFailed,
                latency_ms: Some(2),
            },
        ];

        let baseline = project_content_state_v1(
            EvidenceSnapshotV1 {
                coverage: SnapshotCoverageV1::QueriedIndexesComplete,
                advertisements: advertisements.clone(),
                availability: availability.clone(),
                withdrawals: withdrawals.clone(),
                observations: observations.clone(),
            },
            at(5),
        );

        if flags & 1 != 0 { advertisements.reverse(); }
        if flags & 2 != 0 { availability.reverse(); }
        if flags & 4 != 0 { withdrawals.reverse(); }
        if flags & 8 != 0 { observations.reverse(); }

        let permuted = project_content_state_v1(
            EvidenceSnapshotV1 {
                coverage: SnapshotCoverageV1::QueriedIndexesComplete,
                advertisements,
                availability,
                withdrawals,
                observations,
            },
            at(5),
        );
        prop_assert_eq!(baseline, permuted);
    }
}
