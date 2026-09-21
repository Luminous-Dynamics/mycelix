use mycelix_finance_sync_capability::*;
use mycelix_finance_sync_graph::{BoundedText, Commitment32, SemanticProfileRefV1};

use crate::*;

fn profile(id: &str, revision: u64, fill: u8) -> SemanticProfileRefV1 {
    SemanticProfileRefV1::new(id, revision, Commitment32::from_bytes([fill; 32])).unwrap()
}

fn static_profile() -> RailCapabilityProfileV1 {
    build_rail_capability_profile_v1(RailCapabilityProfileInputV1 {
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
    })
    .unwrap()
}

fn adapter_item() -> RailCapabilityEvidenceItemInputV1 {
    RailCapabilityEvidenceItemInputV1 {
        dimension: CapabilityEvidenceDimensionV1::AdapterArtifact,
        evidence_id: BoundedText::new("evidence:adapter:artifact:1").unwrap(),
        evidence_commitment: Commitment32::from_bytes([0x11; 32]),
        source_profile: profile("evidence:artifact:synthetic:v1", 1, 0xaa),
        claim: StructuralEvidenceClaimV1::SupportsDeclaredStaticSemantics,
        claim_subject_commitment: Commitment32::from_bytes([0xcc; 32]),
        source_revision: Some(7),
        chronology: Some(SourceChronologyEvidenceV1 {
            chronology_profile: profile("chronology:source-signed:v1", 1, 0xbb),
            chronology_commitment: Commitment32::from_bytes([0x22; 32]),
        }),
    }
}

fn provider_item() -> RailCapabilityEvidenceItemInputV1 {
    RailCapabilityEvidenceItemInputV1 {
        dimension: CapabilityEvidenceDimensionV1::ProviderApiProfile,
        evidence_id: BoundedText::new("evidence:provider:api:1").unwrap(),
        evidence_commitment: Commitment32::from_bytes([0x33; 32]),
        source_profile: profile("evidence:provider:synthetic:v1", 1, 0xac),
        claim: StructuralEvidenceClaimV1::SupportsDeclaredStaticSemantics,
        claim_subject_commitment: Commitment32::from_bytes([0xcd; 32]),
        source_revision: Some(4),
        chronology: None,
    }
}

fn bundle_input() -> RailCapabilityEvidenceBundleInputV1 {
    RailCapabilityEvidenceBundleInputV1 {
        static_profile_commitment: static_profile().profile_commitment(),
        evidence_profile: profile("fin-sync:capability-evidence:v1", 1, 0xdd),
        evidence_context_commitment: Commitment32::from_bytes([0xee; 32]),
        items: vec![adapter_item(), provider_item()],
    }
}

#[test]
fn frozen_vectors_match_independent_oracle() {
    let static_profile = static_profile();
    assert_eq!(
        static_profile.profile_commitment().to_hex(),
        "9b7e2d1f6976f85a568c78cd2be92013e3ab8776119c0477f46aa91348280c0d"
    );

    let bundle = build_rail_capability_evidence_bundle_v1(&static_profile, bundle_input()).unwrap();
    let mut item_commitments: Vec<_> = bundle
        .evidence_records()
        .iter()
        .map(|record| record.item_commitment().to_hex())
        .collect();
    item_commitments.sort();
    assert_eq!(
        item_commitments,
        vec![
            "a27525fbcd3ac7e94560bc5336e57ed7a0c59bc69edc5d3820ce98e639f94da4",
            "fad252e445eefdc1a63a86697f0a271ef57b8806bcc4aa22b29de8089853a8b4",
        ]
    );
    assert_eq!(
        bundle.disposition(),
        EvidenceBundleDispositionV1::NoDetectedConflict
    );
    assert_eq!(
        bundle.bundle_commitment().to_hex(),
        "3493d337413c6efa6d372f67ce45bf2c9072d2269f1edbb1e78c0063207816d4"
    );
}

#[test]
fn no_detected_conflict_is_not_a_positive_capability_claim() {
    let mut input = bundle_input();
    input.items.clear();
    let bundle = build_rail_capability_evidence_bundle_v1(&static_profile(), input).unwrap();
    assert_eq!(
        bundle.disposition(),
        EvidenceBundleDispositionV1::NoDetectedConflict
    );
    assert!(bundle.evidence_records().is_empty());
}

#[test]
fn permutation_is_not_authoritative() {
    let first = build_rail_capability_evidence_bundle_v1(&static_profile(), bundle_input()).unwrap();
    let mut permuted = bundle_input();
    permuted.items.reverse();
    let second = build_rail_capability_evidence_bundle_v1(&static_profile(), permuted).unwrap();
    assert_eq!(first.bundle_commitment(), second.bundle_commitment());
}

#[test]
fn exact_duplicate_identity_and_semantics_is_idempotent() {
    let baseline = build_rail_capability_evidence_bundle_v1(&static_profile(), bundle_input()).unwrap();
    let mut duplicated = bundle_input();
    duplicated.items.push(adapter_item());
    let result = build_rail_capability_evidence_bundle_v1(&static_profile(), duplicated).unwrap();
    assert_eq!(baseline.bundle_commitment(), result.bundle_commitment());
    assert_eq!(result.evidence_records().len(), 2);
}

#[test]
fn changed_identity_reuse_is_separate_from_semantic_contradiction() {
    let mut input = bundle_input();
    let mut reused = adapter_item();
    reused.evidence_commitment = Commitment32::from_bytes([0x44; 32]);
    input.items.push(reused);
    let result = build_rail_capability_evidence_bundle_v1(&static_profile(), input).unwrap();
    assert_eq!(result.disposition(), EvidenceBundleDispositionV1::IdentityConflicted);
    assert_eq!(result.conflicted_evidence_ids().len(), 1);
    assert!(result.contradicted_claim_subjects().is_empty());
}

#[test]
fn independent_support_and_contradiction_are_preserved() {
    let mut input = bundle_input();
    let mut contrary = provider_item();
    contrary.evidence_id = BoundedText::new("evidence:provider:api:contrary").unwrap();
    contrary.claim = StructuralEvidenceClaimV1::ContradictsDeclaredStaticSemantics;
    contrary.evidence_commitment = Commitment32::from_bytes([0x45; 32]);
    input.items.push(contrary);
    let result = build_rail_capability_evidence_bundle_v1(&static_profile(), input).unwrap();
    assert_eq!(
        result.disposition(),
        EvidenceBundleDispositionV1::SemanticallyContradicted
    );
    assert_eq!(result.contradicted_claim_subjects().len(), 1);
    assert_eq!(
        result.contradicted_claim_subjects()[0].claim_subject_commitment(),
        Commitment32::from_bytes([0xcd; 32])
    );
}

#[test]
fn both_conflict_axes_can_coexist() {
    let mut input = bundle_input();
    let mut reused = adapter_item();
    reused.evidence_commitment = Commitment32::from_bytes([0x47; 32]);
    input.items.push(reused);
    let mut contrary = provider_item();
    contrary.evidence_id = BoundedText::new("evidence:provider:api:contrary").unwrap();
    contrary.claim = StructuralEvidenceClaimV1::ContradictsDeclaredStaticSemantics;
    contrary.evidence_commitment = Commitment32::from_bytes([0x48; 32]);
    input.items.push(contrary);
    let result = build_rail_capability_evidence_bundle_v1(&static_profile(), input).unwrap();
    assert_eq!(
        result.disposition(),
        EvidenceBundleDispositionV1::IdentityConflictedAndSemanticallyContradicted
    );
}

#[test]
fn source_revision_and_chronology_do_not_resolve_currentness() {
    let baseline = build_rail_capability_evidence_bundle_v1(&static_profile(), bundle_input()).unwrap();
    let mut input = bundle_input();
    let mut later = provider_item();
    later.evidence_id = BoundedText::new("evidence:provider:api:later").unwrap();
    later.source_revision = Some(999);
    later.evidence_commitment = Commitment32::from_bytes([0x46; 32]);
    input.items.push(later);
    let result = build_rail_capability_evidence_bundle_v1(&static_profile(), input).unwrap();
    assert_eq!(result.evidence_records().len(), 3);

    let mut chronology_changed = bundle_input();
    chronology_changed.items[0].chronology = Some(SourceChronologyEvidenceV1 {
        chronology_profile: profile("chronology:source-signed:v1", 1, 0xbb),
        chronology_commitment: Commitment32::from_bytes([0x23; 32]),
    });
    let changed = build_rail_capability_evidence_bundle_v1(&static_profile(), chronology_changed).unwrap();
    assert_ne!(baseline.bundle_commitment(), changed.bundle_commitment());
}

#[test]
fn wrong_parent_and_resource_overflow_fail_closed() {
    let mut wrong_parent = bundle_input();
    wrong_parent.static_profile_commitment = Commitment32::from_bytes([0x99; 32]);
    assert_eq!(
        build_rail_capability_evidence_bundle_v1(&static_profile(), wrong_parent),
        Err(EvidenceError::StaticProfileCommitmentMismatch)
    );

    let mut too_many = bundle_input();
    too_many.items.clear();
    for index in 0..=MAX_EVIDENCE_ITEMS_PER_DIMENSION {
        let mut item = adapter_item();
        item.evidence_id = BoundedText::new(format!("evidence:adapter:{index}")).unwrap();
        item.evidence_commitment = Commitment32::from_bytes([(index & 0xff) as u8; 32]);
        too_many.items.push(item);
    }
    assert_eq!(
        build_rail_capability_evidence_bundle_v1(&static_profile(), too_many),
        Err(EvidenceError::TooManyEvidenceItemsForDimension)
    );
}

#[test]
fn portable_positive_bundle_has_no_currentness_or_secret_surface() {
    let result = build_rail_capability_evidence_bundle_v1(&static_profile(), bundle_input()).unwrap();
    let json = serde_json::to_string(&result).unwrap();
    assert!(json.contains("bundle_commitment"));
    assert!(json.contains("no_detected_conflict"));
    assert!(!json.contains("valid_now"));
    assert!(!json.contains("credential"));
    assert!(!json.contains("bearer"));
    assert!(!json.contains("secret"));
}
