use std::collections::BTreeSet;

use mycelix_content_core::{
    ContentDigestV1, DigestAlgorithmV1, EncryptionRequirementV1, FailureDomainKindV1,
    FailureDomainPolicyV1, JurisdictionV1, ObjectIdV1, PlacementPreferencesV1,
    PlacementRequirementsV1, RetentionRequirementV1, StorageClassV1, StorageIntentV1,
};
use mycelix_content_policy::*;
use mycelix_content_state::{
    ActionRefV1, AgentRefV1, FailureDomainClaimEvidenceV1, ProjectedContentStateV1,
    SnapshotCoverageV1, SnapshotServiceCandidateV1, TimestampMicrosV1,
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
    ContentDigestV1::compute(DigestAlgorithmV1::Blake3_256, b"policy-target")
}

fn requirements(
    minimum_replicas: u16,
    failure_domains: Vec<(FailureDomainKindV1, u16)>,
    allowed: &[&str],
    forbidden: &[&str],
    encryption: EncryptionRequirementV1,
    retention: RetentionRequirementV1,
) -> PlacementRequirementsV1 {
    PlacementRequirementsV1::new(
        minimum_replicas,
        FailureDomainPolicyV1::new(failure_domains).unwrap(),
        allowed
            .iter()
            .map(|value| JurisdictionV1::new(*value).unwrap())
            .collect::<BTreeSet<_>>(),
        forbidden
            .iter()
            .map(|value| JurisdictionV1::new(*value).unwrap())
            .collect::<BTreeSet<_>>(),
        encryption,
        retention,
    )
    .unwrap()
}

fn intent(requirements: PlacementRequirementsV1) -> StorageIntentV1 {
    StorageIntentV1::new(
        ObjectIdV1([7; 32]),
        PartyIdV1([8; 32]),
        StorageClassV1::Durable,
        requirements,
        PlacementPreferencesV1::new(None, 10, 20, 30, 40).unwrap(),
        UnixMillisV1(1_000),
    )
    .unwrap()
}

fn target(client_side_encrypted: bool) -> PlacementTargetV1 {
    PlacementTargetV1 {
        object_id: ObjectIdV1([7; 32]),
        digest: digest(),
        size_bytes: 100,
        client_side_encrypted,
    }
}

fn candidate(
    availability: u8,
    advertisement: u8,
    provider: u8,
    site: &str,
    operator: &str,
) -> SnapshotServiceCandidateV1 {
    SnapshotServiceCandidateV1 {
        availability_action: action(availability),
        advertisement_action: action(advertisement),
        provider: agent(provider),
        iroh_endpoint_id: [provider; 32],
        digest: digest(),
        size_bytes: 100,
        failure_domains: vec![
            FailureDomainClaimEvidenceV1 {
                kind: FailureDomainKindV1::Operator,
                value: operator.to_string(),
            },
            FailureDomainClaimEvidenceV1 {
                kind: FailureDomainKindV1::Site,
                value: site.to_string(),
            },
        ],
        claim_authored_at: TimestampMicrosV1(2_000_000),
        effective_until: TimestampMicrosV1(60_000_000),
        withdrawal: WithdrawalObservationV1::NoWithdrawalObserved,
    }
}

fn state(
    coverage: SnapshotCoverageV1,
    candidates: Vec<SnapshotServiceCandidateV1>,
) -> ProjectedContentStateV1 {
    ProjectedContentStateV1 {
        evaluated_at: TimestampMicrosV1(10_000_000),
        coverage,
        advertisements: Vec::new(),
        service_candidates: candidates,
        observations: Vec::new(),
        issues: Vec::new(),
    }
}

fn policy_evidence(
    advertisement: u8,
    provider: u8,
    assurance: PolicyAssuranceV1,
    jurisdiction: &str,
    site: &str,
    operator: &str,
) -> ProviderPolicyEvidenceV1 {
    ProviderPolicyEvidenceV1 {
        advertisement: action(advertisement),
        provider: agent(provider),
        valid_from_unix_ms: 0,
        valid_until_unix_ms: 60_000,
        storage_jurisdictions: vec![AssuredJurisdictionV1 {
            jurisdiction: JurisdictionV1::new(jurisdiction).unwrap(),
            assurance,
        }],
        provider_at_rest_encryption: Some(assurance),
        retention: Some(RetentionCapabilityEvidenceV1 {
            assurance,
            guaranteed_until_unix_ms: Some(120_000),
            supports_indefinite: false,
        }),
        failure_domains: vec![
            AssuredFailureDomainV1 {
                kind: FailureDomainKindV1::Operator,
                value: operator.to_string(),
                assurance,
            },
            AssuredFailureDomainV1 {
                kind: FailureDomainKindV1::Site,
                value: site.to_string(),
                assurance,
            },
        ],
    }
}

#[test]
fn strict_gate_refuses_partial_snapshot() {
    let intent = intent(requirements(
        1,
        Vec::new(),
        &[],
        &[],
        EncryptionRequirementV1::NotRequired,
        RetentionRequirementV1::BestEffort,
    ));
    let result = evaluate_hard_policy_v1(
        &intent,
        target(false),
        &state(
            SnapshotCoverageV1::Partial,
            vec![candidate(1, 11, 21, "site-a", "operator-a")],
        ),
        Vec::new(),
        HardPolicyGateConfigV1::strict(),
    );
    assert!(result.qualified_pool.is_none());
    assert_eq!(
        result.failures,
        vec![PoolFailureV1::SnapshotCoverageInsufficient]
    );
}

#[test]
fn self_claimed_jurisdiction_cannot_satisfy_strict_gate() {
    let intent = intent(requirements(
        1,
        Vec::new(),
        &["ZA"],
        &[],
        EncryptionRequirementV1::NotRequired,
        RetentionRequirementV1::BestEffort,
    ));
    let result = evaluate_hard_policy_v1(
        &intent,
        target(false),
        &state(
            SnapshotCoverageV1::QueriedIndexesComplete,
            vec![candidate(1, 11, 21, "site-a", "operator-a")],
        ),
        vec![policy_evidence(
            11,
            21,
            PolicyAssuranceV1::SelfClaimed,
            "ZA",
            "site-a",
            "operator-a",
        )],
        HardPolicyGateConfigV1::strict(),
    );
    assert!(result.qualified_pool.is_none());
    assert!(result.rejections[0]
        .reasons
        .contains(&CandidateRejectionReasonV1::InsufficientJurisdictionAssurance));
}

#[test]
fn stale_policy_evidence_fails_closed() {
    let intent = intent(requirements(
        1,
        Vec::new(),
        &["ZA"],
        &[],
        EncryptionRequirementV1::NotRequired,
        RetentionRequirementV1::BestEffort,
    ));
    let mut evidence = policy_evidence(
        11,
        21,
        PolicyAssuranceV1::IndependentlyAttested,
        "ZA",
        "site-a",
        "operator-a",
    );
    evidence.valid_until_unix_ms = 10_000;
    let result = evaluate_hard_policy_v1(
        &intent,
        target(false),
        &state(
            SnapshotCoverageV1::QueriedIndexesComplete,
            vec![candidate(1, 11, 21, "site-a", "operator-a")],
        ),
        vec![evidence],
        HardPolicyGateConfigV1::strict(),
    );
    assert!(result.qualified_pool.is_none());
    assert!(result.rejections[0]
        .reasons
        .contains(&CandidateRejectionReasonV1::ProviderPolicyEvidenceNotCurrent));
}

#[test]
fn forbidden_jurisdiction_is_filtered_before_pool_qualification() {
    let intent = intent(requirements(
        1,
        Vec::new(),
        &[],
        &["US"],
        EncryptionRequirementV1::NotRequired,
        RetentionRequirementV1::BestEffort,
    ));
    let result = evaluate_hard_policy_v1(
        &intent,
        target(false),
        &state(
            SnapshotCoverageV1::QueriedIndexesComplete,
            vec![candidate(1, 11, 21, "site-a", "operator-a")],
        ),
        vec![policy_evidence(
            11,
            21,
            PolicyAssuranceV1::IndependentlyAttested,
            "US",
            "site-a",
            "operator-a",
        )],
        HardPolicyGateConfigV1::strict(),
    );
    assert!(result.qualified_pool.is_none());
    assert!(result.rejections[0].reasons.iter().any(|reason| matches!(
        reason,
        CandidateRejectionReasonV1::JurisdictionNotAllowed { jurisdiction }
            if jurisdiction.as_str() == "US"
    )));
}

#[test]
fn client_side_encryption_is_a_local_object_fact() {
    let intent = intent(requirements(
        1,
        Vec::new(),
        &[],
        &[],
        EncryptionRequirementV1::ClientSide,
        RetentionRequirementV1::BestEffort,
    ));
    let projected = state(
        SnapshotCoverageV1::QueriedIndexesComplete,
        vec![candidate(1, 11, 21, "site-a", "operator-a")],
    );
    let denied = evaluate_hard_policy_v1(
        &intent,
        target(false),
        &projected,
        Vec::new(),
        HardPolicyGateConfigV1::strict(),
    );
    assert!(denied.rejections[0]
        .reasons
        .contains(&CandidateRejectionReasonV1::ClientSideEncryptionRequired));

    let allowed = evaluate_hard_policy_v1(
        &intent,
        target(true),
        &projected,
        Vec::new(),
        HardPolicyGateConfigV1::strict(),
    );
    assert!(allowed.qualified_pool.is_some());
}

#[test]
fn pool_requires_distinct_independently_attested_failure_domains() {
    let intent = intent(requirements(
        3,
        vec![
            (FailureDomainKindV1::Operator, 3),
            (FailureDomainKindV1::Site, 2),
        ],
        &[],
        &[],
        EncryptionRequirementV1::NotRequired,
        RetentionRequirementV1::BestEffort,
    ));
    let projected = state(
        SnapshotCoverageV1::QueriedIndexesComplete,
        vec![
            candidate(1, 11, 21, "site-a", "operator-a"),
            candidate(2, 12, 22, "site-a", "operator-b"),
            candidate(3, 13, 23, "site-b", "operator-c"),
        ],
    );
    let evidence = vec![
        policy_evidence(
            11,
            21,
            PolicyAssuranceV1::IndependentlyAttested,
            "ZA",
            "site-a",
            "operator-a",
        ),
        policy_evidence(
            12,
            22,
            PolicyAssuranceV1::IndependentlyAttested,
            "ZA",
            "site-a",
            "operator-b",
        ),
        policy_evidence(
            13,
            23,
            PolicyAssuranceV1::IndependentlyAttested,
            "ZA",
            "site-b",
            "operator-c",
        ),
    ];
    let result = evaluate_hard_policy_v1(
        &intent,
        target(false),
        &projected,
        evidence,
        HardPolicyGateConfigV1::strict(),
    );
    assert!(result.failures.is_empty());
    assert_eq!(result.qualified_pool.unwrap().candidates.len(), 3);
}

#[test]
fn selection_validator_rejects_a_policy_bad_subset() {
    let intent = intent(requirements(
        2,
        vec![(FailureDomainKindV1::Site, 2)],
        &[],
        &[],
        EncryptionRequirementV1::NotRequired,
        RetentionRequirementV1::BestEffort,
    ));
    let projected = state(
        SnapshotCoverageV1::QueriedIndexesComplete,
        vec![
            candidate(1, 11, 21, "site-a", "operator-a"),
            candidate(2, 12, 22, "site-a", "operator-b"),
            candidate(3, 13, 23, "site-b", "operator-c"),
        ],
    );
    let evidence = vec![
        policy_evidence(
            11,
            21,
            PolicyAssuranceV1::IndependentlyAttested,
            "ZA",
            "site-a",
            "operator-a",
        ),
        policy_evidence(
            12,
            22,
            PolicyAssuranceV1::IndependentlyAttested,
            "ZA",
            "site-a",
            "operator-b",
        ),
        policy_evidence(
            13,
            23,
            PolicyAssuranceV1::IndependentlyAttested,
            "ZA",
            "site-b",
            "operator-c",
        ),
    ];
    let pool = evaluate_hard_policy_v1(
        &intent,
        target(false),
        &projected,
        evidence,
        HardPolicyGateConfigV1::strict(),
    )
    .qualified_pool
    .unwrap();

    assert!(matches!(
        pool.validate_selection(&[action(1), action(2)]),
        Err(SelectionPolicyErrorV1::InsufficientFailureDomainDiversity {
            kind: FailureDomainKindV1::Site,
            required: 2,
            observed: 1,
        })
    ));
    assert!(pool.validate_selection(&[action(1), action(3)]).is_ok());
}

proptest! {
    #[test]
    fn input_order_does_not_change_policy_result(
        reverse_candidates in any::<bool>(),
        reverse_evidence in any::<bool>()
    ) {
        let intent = intent(requirements(
            2,
            vec![(FailureDomainKindV1::Site, 2)],
            &["ZA"],
            &[],
            EncryptionRequirementV1::ProviderAtRest,
            RetentionRequirementV1::MinimumSeconds(30),
        ));
        let mut candidates = vec![
            candidate(1, 11, 21, "site-a", "operator-a"),
            candidate(2, 12, 22, "site-b", "operator-b"),
        ];
        let mut evidence = vec![
            policy_evidence(
                11,
                21,
                PolicyAssuranceV1::IndependentlyAttested,
                "ZA",
                "site-a",
                "operator-a",
            ),
            policy_evidence(
                12,
                22,
                PolicyAssuranceV1::IndependentlyAttested,
                "ZA",
                "site-b",
                "operator-b",
            ),
        ];
        let baseline = evaluate_hard_policy_v1(
            &intent,
            target(false),
            &state(
                SnapshotCoverageV1::QueriedIndexesComplete,
                candidates.clone(),
            ),
            evidence.clone(),
            HardPolicyGateConfigV1::strict(),
        );
        if reverse_candidates {
            candidates.reverse();
        }
        if reverse_evidence {
            evidence.reverse();
        }
        let permuted = evaluate_hard_policy_v1(
            &intent,
            target(false),
            &state(SnapshotCoverageV1::QueriedIndexesComplete, candidates),
            evidence,
            HardPolicyGateConfigV1::strict(),
        );
        prop_assert_eq!(baseline, permuted);
    }
}
