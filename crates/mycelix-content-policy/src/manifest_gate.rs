use mycelix_content_core::{ObjectManifestV1, StorageIntentV1};
use mycelix_content_state::ProjectedContentStateV1;

use crate::{
    gate, HardPolicyEvaluationV1, HardPolicyGateConfigV1, PlacementTargetV1, PoolFailureV1,
    ProviderPolicyEvidenceV1,
};

/// Public CF-06A admission boundary.
///
/// A storage intent authorizes policy evaluation for an immutable object, not for an
/// arbitrary caller-supplied digest. The validated object manifest therefore has to
/// bind the intent to the exact target blob before provider qualification begins.
pub fn evaluate_hard_policy_v1(
    intent: &StorageIntentV1,
    manifest: &ObjectManifestV1,
    target: PlacementTargetV1,
    state: &ProjectedContentStateV1,
    provider_evidence: Vec<ProviderPolicyEvidenceV1>,
    config: HardPolicyGateConfigV1,
) -> HardPolicyEvaluationV1 {
    let mut failures = Vec::new();

    if manifest.validate().is_err() {
        failures.push(PoolFailureV1::InvalidObjectManifest);
    } else {
        if manifest.id != intent.object_id {
            failures.push(PoolFailureV1::ManifestIntentMismatch);
        }
        if target.object_id != manifest.id {
            failures.push(PoolFailureV1::TargetObjectMismatch);
        } else if !manifest
            .blobs
            .iter()
            .any(|blob| blob.digest == target.digest && blob.size_bytes == target.size_bytes)
        {
            failures.push(PoolFailureV1::TargetNotInManifest);
        }
    }

    failures.sort();
    failures.dedup();
    if !failures.is_empty() {
        return HardPolicyEvaluationV1 {
            qualified_pool: None,
            rejections: Vec::new(),
            failures,
        };
    }

    gate::evaluate_hard_policy_v1(intent, target, state, provider_evidence, config)
}
