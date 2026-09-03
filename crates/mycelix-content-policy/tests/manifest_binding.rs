use std::collections::BTreeSet;

use mycelix_content_core::{
    BlobDescriptorV1, ContentDigestV1, DigestAlgorithmV1, EncryptionRequirementV1,
    FailureDomainPolicyV1, ObjectManifestV1, PlacementPreferencesV1, PlacementRequirementsV1,
    RetentionRequirementV1, StorageClassV1, StorageIntentV1,
};
use mycelix_content_policy::{
    evaluate_hard_policy_v1, HardPolicyGateConfigV1, PlacementTargetV1, PoolFailureV1,
};
use mycelix_content_state::{ProjectedContentStateV1, SnapshotCoverageV1, TimestampMicrosV1};
use mycelix_infrastructure_types::{PartyIdV1, UnixMillisV1};

fn digest(bytes: &[u8]) -> ContentDigestV1 {
    ContentDigestV1::compute(DigestAlgorithmV1::Blake3_256, bytes)
}

fn manifest(bytes: &[u8]) -> ObjectManifestV1 {
    ObjectManifestV1::new(
        vec![BlobDescriptorV1 {
            digest: digest(bytes),
            size_bytes: 100,
            media_type: None,
        }],
        None,
    )
    .unwrap()
}

fn intent_for(manifest: &ObjectManifestV1) -> StorageIntentV1 {
    StorageIntentV1::new(
        manifest.id,
        PartyIdV1([8; 32]),
        StorageClassV1::Durable,
        PlacementRequirementsV1::new(
            1,
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

fn state() -> ProjectedContentStateV1 {
    ProjectedContentStateV1 {
        evaluated_at: TimestampMicrosV1(10_000_000),
        coverage: SnapshotCoverageV1::QueriedIndexesComplete,
        advertisements: Vec::new(),
        service_candidates: Vec::new(),
        observations: Vec::new(),
        issues: Vec::new(),
    }
}

#[test]
fn target_digest_must_be_a_member_of_the_intent_manifest() {
    let manifest = manifest(b"authorized");
    let intent = intent_for(&manifest);
    let result = evaluate_hard_policy_v1(
        &intent,
        &manifest,
        PlacementTargetV1 {
            object_id: manifest.id,
            digest: digest(b"not-in-manifest"),
            size_bytes: 100,
            client_side_encrypted: false,
        },
        &state(),
        Vec::new(),
        HardPolicyGateConfigV1::strict(),
    );

    assert_eq!(result.failures, vec![PoolFailureV1::TargetNotInManifest]);
    assert!(result.qualified_pool.is_none());
}

#[test]
fn manifest_must_match_the_storage_intent_object() {
    let authorized = manifest(b"authorized");
    let other = manifest(b"other");
    let intent = intent_for(&authorized);
    let result = evaluate_hard_policy_v1(
        &intent,
        &other,
        PlacementTargetV1 {
            object_id: other.id,
            digest: other.blobs[0].digest,
            size_bytes: other.blobs[0].size_bytes,
            client_side_encrypted: false,
        },
        &state(),
        Vec::new(),
        HardPolicyGateConfigV1::strict(),
    );

    assert_eq!(result.failures, vec![PoolFailureV1::ManifestIntentMismatch]);
    assert!(result.qualified_pool.is_none());
}
