use std::collections::{BTreeMap, BTreeSet};

use mycelix_content_core::{
    ContentDigestV1, DigestAlgorithmV1, EncryptionRequirementV1, FailureDomainPolicyV1,
    ObjectIdV1, PlacementRequirementsV1, RetentionRequirementV1, StorageIntentIdV1,
};
use mycelix_content_policy::{
    PlacementTargetV1, PolicyEligibleCandidateV1, PolicyQualifiedPoolV1, SelectionPolicyErrorV1,
};
use mycelix_content_state::{
    ActionRefV1, AgentRefV1, SnapshotServiceCandidateV1, TimestampMicrosV1,
    WithdrawalObservationV1,
};

fn action(value: u8) -> ActionRefV1 {
    ActionRefV1([value; 39])
}

fn pool() -> PolicyQualifiedPoolV1 {
    let digest = ContentDigestV1::compute(DigestAlgorithmV1::Blake3_256, b"selection-target");
    let requirements = PlacementRequirementsV1::new(
        1,
        FailureDomainPolicyV1::empty(),
        BTreeSet::new(),
        BTreeSet::new(),
        EncryptionRequirementV1::NotRequired,
        RetentionRequirementV1::BestEffort,
    )
    .unwrap();
    let candidate = SnapshotServiceCandidateV1 {
        availability_action: action(1),
        advertisement_action: action(11),
        provider: AgentRefV1([21; 39]),
        iroh_endpoint_id: [21; 32],
        digest,
        size_bytes: 100,
        failure_domains: Vec::new(),
        claim_authored_at: TimestampMicrosV1(2_000_000),
        effective_until: TimestampMicrosV1(60_000_000),
        withdrawal: WithdrawalObservationV1::NoWithdrawalObserved,
    };

    PolicyQualifiedPoolV1 {
        storage_intent_id: StorageIntentIdV1([9; 32]),
        target: PlacementTargetV1 {
            object_id: ObjectIdV1([7; 32]),
            digest,
            size_bytes: 100,
            client_side_encrypted: false,
        },
        evaluated_at: TimestampMicrosV1(10_000_000),
        evaluated_at_unix_ms: 10_000,
        requirements,
        candidates: vec![PolicyEligibleCandidateV1 {
            candidate,
            accepted_jurisdictions: BTreeSet::new(),
            accepted_failure_domains: BTreeMap::new(),
        }],
    }
}

#[test]
fn duplicate_selected_id_is_rejected_before_counting() {
    assert_eq!(
        pool().validate_selection(&[action(1), action(1)]),
        Err(SelectionPolicyErrorV1::DuplicateCandidate {
            availability_action: action(1),
        })
    );
}

#[test]
fn unknown_selected_id_is_rejected() {
    assert_eq!(
        pool().validate_selection(&[action(99)]),
        Err(SelectionPolicyErrorV1::UnknownCandidate {
            availability_action: action(99),
        })
    );
}
