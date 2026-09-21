use mycelix_finance_sync_graph::{BoundedText, Commitment32, SemanticProfileRefV1};

use crate::*;

fn profile(id: &str, revision: u64, fill: u8) -> SemanticProfileRefV1 {
    SemanticProfileRefV1::new(id, revision, Commitment32::from_bytes([fill; 32])).unwrap()
}

fn sample_input() -> RailCapabilityProfileInputV1 {
    RailCapabilityProfileInputV1 {
        adapter_profile: profile("adapter:synthetic:pvp", 1, 0xaa),
        adapter_build_profile: profile("adapter-build:synthetic:v1", 1, 0xbb),
        provider_profile: profile("provider:synthetic:v1", 1, 0xcc),
        rail: BoundedText::new("synthetic-rtgs").unwrap(),
        network: BoundedText::new("testnet-a").unwrap(),
        operation_profile: profile("operation:pvp-payment:v1", 1, 0xdd),
        capacity_lock_capabilities: vec![
            CapacityLockCapabilityV1::ProviderReserve,
            CapacityLockCapabilityV1::FundsLock,
        ],
        prepare_capabilities: vec![PrepareCapabilityV1::PreparedOperationHandle],
        commit_capabilities: vec![
            CommitCapabilityV1::CommitAgainstPreparedState,
            CommitCapabilityV1::ExternallySynchronizedCommit,
        ],
        cancel_capabilities: vec![
            CancelCapabilityV1::CancelableWhilePrepared,
            CancelCapabilityV1::GuaranteedAbortOfPreparedState,
        ],
        query_capabilities: vec![
            QueryCapabilityV1::ExactOperationState,
            QueryCapabilityV1::OperationBySemanticIdentity,
            QueryCapabilityV1::DurableCommitClassificationQuery,
            QueryCapabilityV1::FinalityQuery,
        ],
        evidence_capabilities: vec![
            EvidenceCapabilityV1::ProviderAcknowledgement,
            EvidenceCapabilityV1::DefinitelyNotCommittedClassifier,
            EvidenceCapabilityV1::DefinitelyCommittedClassifier,
            EvidenceCapabilityV1::FinalityEvidenceProduction,
        ],
        reversal_capabilities: vec![ReversalCapabilityV1::ExplicitCompensatingTransferOnly],
        atomicity_capabilities: vec![AtomicityCapabilityV1::ExternalSynchronizationProtocol],
        idempotency: IdempotencyProfileV1 {
            mechanism: IdempotencyMechanismV1::ProviderKeyDedupWindow,
            key_scope: IdempotencyKeyScopeV1::RailNetwork,
            semantic_scope: IdempotencySemanticScopeV1::ExactLeg,
            retention: RetentionHorizonV1::BoundedSeconds { seconds: 86_400 },
            retry_after_unknown: UnknownOutcomeRetryV1::QueryBeforeReplay,
            collision_behavior: IdempotencyCollisionBehaviorV1::ProviderRejectsSemanticMismatch,
        },
        producible_finality_profiles: vec![profile("finality:synthetic:v2", 2, 0xee)],
        synchronization_profiles: vec![profile("sync:pvp:synthetic:v1", 1, 0xff)],
        disclosure_profile: profile("disclosure:minimal:v1", 1, 0x11),
        timing: TimingProfileV1 {
            max_provider_deadline_ms: 30_000,
            max_prepare_lifetime_seconds: Some(120),
            min_query_poll_interval_ms: Some(250),
            max_query_attempts_per_window: Some(20),
        },
        resources: ResourceProfileV1 {
            max_request_bytes: 65_536,
            max_batch_items: 32,
            max_inflight_per_subject: 8,
        },
        profile_revision: 1,
    }
}

#[test]
fn frozen_profile_vector_matches_independent_commitment() {
    let built = build_rail_capability_profile_v1(sample_input()).unwrap();
    assert_eq!(
        built.profile_commitment().to_hex(),
        "9b7e2d1f6976f85a568c78cd2be92013e3ab8776119c0477f46aa91348280c0d"
    );
}

#[test]
fn capability_and_profile_input_order_is_not_authoritative() {
    let first = build_rail_capability_profile_v1(sample_input()).unwrap();
    let mut second_input = sample_input();
    second_input.capacity_lock_capabilities.reverse();
    second_input.commit_capabilities.reverse();
    second_input.cancel_capabilities.reverse();
    second_input.query_capabilities.reverse();
    second_input.evidence_capabilities.reverse();
    second_input.producible_finality_profiles.push(profile("finality:synthetic:v1", 1, 0x44));
    second_input.producible_finality_profiles.reverse();

    let mut first_with_extra = sample_input();
    first_with_extra
        .producible_finality_profiles
        .push(profile("finality:synthetic:v1", 1, 0x44));
    let canonical_first = build_rail_capability_profile_v1(first_with_extra).unwrap();
    let canonical_second = build_rail_capability_profile_v1(second_input).unwrap();

    assert_ne!(first.profile_commitment(), canonical_first.profile_commitment());
    assert_eq!(
        canonical_first.profile_commitment(),
        canonical_second.profile_commitment()
    );
}

#[test]
fn negative_marker_cannot_coexist_with_positive_capability() {
    let mut input = sample_input();
    input
        .capacity_lock_capabilities
        .push(CapacityLockCapabilityV1::NoCapacityLock);
    assert_eq!(
        build_rail_capability_profile_v1(input),
        Err(CapabilityError::ContradictoryCapabilityDimension)
    );
}

#[test]
fn duplicate_capability_is_rejected_instead_of_silently_deduplicated() {
    let mut input = sample_input();
    input
        .query_capabilities
        .push(QueryCapabilityV1::ExactOperationState);
    assert_eq!(
        build_rail_capability_profile_v1(input),
        Err(CapabilityError::DuplicateCapability)
    );
}

#[test]
fn prepared_commit_requires_real_prepare_primitive() {
    let mut input = sample_input();
    input.prepare_capabilities = vec![PrepareCapabilityV1::LocalPreValidationOnly];
    assert_eq!(
        build_rail_capability_profile_v1(input),
        Err(CapabilityError::InvalidPreparedStateCombination)
    );
}

#[test]
fn synchronization_requires_commit_scope_atomicity_and_profile_together() {
    let mut input = sample_input();
    input.synchronization_profiles.clear();
    assert_eq!(
        build_rail_capability_profile_v1(input),
        Err(CapabilityError::InvalidSynchronizationCombination)
    );
}

#[test]
fn finality_profile_requires_explicit_evidence_production_capability() {
    let mut input = sample_input();
    input
        .evidence_capabilities
        .retain(|value| *value != EvidenceCapabilityV1::FinalityEvidenceProduction);
    assert_eq!(
        build_rail_capability_profile_v1(input),
        Err(CapabilityError::InvalidFinalityCombination)
    );
}

#[test]
fn single_owner_atomicity_cannot_be_claimed_without_matching_commit_primitive() {
    let mut input = sample_input();
    input
        .atomicity_capabilities
        .push(AtomicityCapabilityV1::MultiInstructionSingleTransactionOwner);
    assert_eq!(
        build_rail_capability_profile_v1(input),
        Err(CapabilityError::InvalidAtomicityCombination)
    );
}

#[test]
fn no_idempotency_guarantee_cannot_authorize_blind_replay() {
    let mut input = sample_input();
    input.idempotency = IdempotencyProfileV1 {
        mechanism: IdempotencyMechanismV1::NoGuarantee,
        key_scope: IdempotencyKeyScopeV1::None,
        semantic_scope: IdempotencySemanticScopeV1::None,
        retention: RetentionHorizonV1::NotGuaranteed,
        retry_after_unknown: UnknownOutcomeRetryV1::ReplaySameSemanticOperationWithinHorizon,
        collision_behavior: IdempotencyCollisionBehaviorV1::Undefined,
    };
    assert_eq!(
        build_rail_capability_profile_v1(input),
        Err(CapabilityError::InvalidIdempotencyProfile)
    );
}

#[test]
fn provider_dedup_window_must_have_a_nonzero_bounded_horizon() {
    let mut input = sample_input();
    input.idempotency.retention = RetentionHorizonV1::BoundedSeconds { seconds: 0 };
    assert_eq!(
        build_rail_capability_profile_v1(input),
        Err(CapabilityError::InvalidIdempotencyProfile)
    );
}

#[test]
fn idempotency_horizon_is_authority_significant_profile_identity() {
    let first = build_rail_capability_profile_v1(sample_input()).unwrap();
    let mut changed = sample_input();
    changed.idempotency.retention = RetentionHorizonV1::BoundedSeconds { seconds: 3_600 };
    let second = build_rail_capability_profile_v1(changed).unwrap();
    assert_ne!(first.profile_commitment(), second.profile_commitment());
}

#[test]
fn adapter_build_identity_is_authority_significant_profile_identity() {
    let first = build_rail_capability_profile_v1(sample_input()).unwrap();
    let mut changed = sample_input();
    changed.adapter_build_profile = profile("adapter-build:synthetic:v2", 2, 0x22);
    let second = build_rail_capability_profile_v1(changed).unwrap();
    assert_ne!(first.profile_commitment(), second.profile_commitment());
}

#[test]
fn timing_and_resource_bounds_are_not_optional_zeroes() {
    let mut timing = sample_input();
    timing.timing.max_provider_deadline_ms = 0;
    assert_eq!(
        build_rail_capability_profile_v1(timing),
        Err(CapabilityError::InvalidTimingProfile)
    );

    let mut resources = sample_input();
    resources.resources.max_request_bytes = 0;
    assert_eq!(
        build_rail_capability_profile_v1(resources),
        Err(CapabilityError::InvalidResourceProfile)
    );
}

#[test]
fn unknown_capability_enum_fails_closed_at_serde_boundary() {
    let mut value = serde_json::to_value(sample_input()).unwrap();
    value["capacity_lock_capabilities"] =
        serde_json::json!(["provider_reserve", "quantum_lock"]);
    let decoded = serde_json::from_value::<RailCapabilityProfileInputV1>(value);
    assert!(decoded.is_err());
}

#[test]
fn positive_profile_serialization_contains_no_bearer_secret_surface() {
    let profile = build_rail_capability_profile_v1(sample_input()).unwrap();
    let json = serde_json::to_string(&profile).unwrap().to_ascii_lowercase();
    for forbidden in ["bearer", "credential", "password", "private_key", "api_token"] {
        assert!(!json.contains(forbidden));
    }
}
