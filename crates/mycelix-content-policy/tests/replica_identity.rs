use std::collections::BTreeSet;

use mycelix_content_core::{
    BlobDescriptorV1, ContentDigestV1, DigestAlgorithmV1, EncryptionRequirementV1,
    FailureDomainPolicyV1, ObjectManifestV1, PlacementPreferencesV1, PlacementRequirementsV1,
    RetentionRequirementV1, StorageClassV1, StorageIntentV1,
};
use mycelix_content_policy::{
    evaluate_hard_policy_v1, HardPolicyGateConfigV1, PlacementTargetV1, PoolFailureV1,
};
use mycelix_content_state::{
    ActionRefV1, AgentRefV1, ProjectedContentStateV1, SnapshotCoverageV1,
    SnapshotServiceCandidateV1, TimestampMicrosV1, WithdrawalObservationV1,
};
use mycelix_infrastructure_types::{PartyIdV1, UnixMillisV1};

fn action(value: u8) -> ActionRefV1 {
    ActionRefV1([value; 39])
}

fn agent(value: u8) -> AgentRefV1 {
    AgentRefV1([value; 39])
}

fn target_digest() -> ContentDigestV1 {
    ContentDigestV1::compute(DigestAlgorithmV1::Blake3_256, b"target")
}

fn unrelated_digest() -> ContentDigestV1 {
    ContentDigestV1::compute(DigestAlgorithmV1::Blake3_256, b"unrelated")
}

fn manifest() -> ObjectManifestV1 {
    ObjectManifestV1::new(
        vec![BlobDescriptorV1 {
            digest: target_digest(),
            size_bytes: 100,
            media_type: None,
        }],
        None,
    )
    .unwrap()
}

fn intent(minimum_replicas: u16) -> StorageIntentV1 {
    StorageIntentV1::new(
        manifest().id,
        PartyIdV1([8; 32]),
        StorageClassV1::Durable,
        PlacementRequirementsV1::new(
            minimum_replicas,
            FailureDomainPolicyV1::empty(),
            BTreeSet::new(),
            BTreeSet::new(),
            EncryptionRequirementV1::NotRequired,
            RetentionRequirementV1::BestEffort,
        )
        .unwrap(),
        PlacementPreferencesV1::new(None, 0, 0, 0, 0).unwrap(),
        UnixMillisV1(1_000),
    )
    .unwrap()
}

fn candidate(
    availability: u8,
    advertisement: u8,
    provider: u8,
    digest: ContentDigestV1,
    authored_at: i64,
) -> SnapshotServiceCandidateV1 {
    SnapshotServiceCandidateV1 {
        availability_action: action(availability),
        advertisement_action: action(advertisement),
        provider: agent(provider),
        iroh_endpoint_id: [provider; 32],
        digest,
        size_bytes: 100,
        failure_domains: Vec::new(),
        claim_authored_at: TimestampMicrosV1(authored_at),
        effective_until: TimestampMicrosV1(60_000_000),
        withdrawal: WithdrawalObservationV1::NoWithdrawalObserved,
    }
}

fn state(candidates: Vec<SnapshotServiceCandidateV1>) -> ProjectedContentStateV1 {
    ProjectedContentStateV1 {
        evaluated_at: TimestampMicrosV1(10_000_000),
        coverage: SnapshotCoverageV1::QueriedIndexesComplete,
        advertisements: Vec::new(),
        service_candidates: candidates,
        observations: Vec::new(),
        issues: Vec::new(),
    }
}

fn target() -> PlacementTargetV1 {
    PlacementTargetV1 {
        object_id: manifest().id,
        digest: target_digest(),
        size_bytes: 100,
        client_side_encrypted: false,
    }
}

fn run(
    minimum_replicas: u16,
    candidates: Vec<SnapshotServiceCandidateV1>,
) -> mycelix_content_policy::HardPolicyEvaluationV1 {
    let manifest = manifest();
    evaluate_hard_policy_v1(
        &intent(minimum_replicas),
        &manifest,
        target(),
        &state(candidates),
        Vec::new(),
        HardPolicyGateConfigV1::strict(),
    )
}

#[test]
fn renewed_claims_from_one_advertisement_count_as_one_replica() {
    let result = run(
        2,
        vec![
            candidate(1, 11, 21, target_digest(), 2_000_000),
            candidate(2, 11, 21, target_digest(), 3_000_000),
            candidate(3, 12, 22, unrelated_digest(), 4_000_000),
        ],
    );

    assert!(result.qualified_pool.is_none());
    assert_eq!(
        result.failures,
        vec![PoolFailureV1::InsufficientEligibleReplicas {
            required: 2,
            observed: 1,
        }]
    );
}

#[test]
fn newest_live_claim_is_the_canonical_candidate_for_an_advertisement() {
    let result = run(
        1,
        vec![
            candidate(1, 11, 21, target_digest(), 2_000_000),
            candidate(2, 11, 21, target_digest(), 3_000_000),
        ],
    );

    let pool = result.qualified_pool.expect("one advertisement is one replica");
    assert_eq!(pool.candidates.len(), 1);
    assert_eq!(pool.candidates[0].candidate.availability_action, action(2));
}
