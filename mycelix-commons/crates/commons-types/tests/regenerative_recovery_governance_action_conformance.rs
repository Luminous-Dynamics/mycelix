// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Independent conformance checks for the exact governance execution-action
//! byte contract reused by recovery-fork authority composition.
//!
//! This test deliberately reconstructs the registered preimage independently
//! from the production streaming hasher. It therefore detects drift in domain,
//! framing, proposal binding, compact action JSON, or the registered profile.

use commons_types::{
    GOVERNANCE_ACTIONS_DIGEST_PROFILE_V1, RECOVERY_GOVERNANCE_ACTION_PROTOCOL_V1,
    REGENERATIVE_RECOVERY_FORK_RESOLUTION_SCHEMA_V1,
    RegenerativeRecoveryForkResolutionEvidenceV1,
    RegenerativeRecoveryForkResolutionOutcomeV1,
    qualify_regenerative_recovery_governance_action,
};

const REGISTERED_EXECUTION_AUTHORITY_DOMAIN: &[u8] =
    b"mycelix-governance-execution-authority-v1\0";
const REGISTERED_EXECUTION_AUTHORITY_PROFILE: &str =
    "mycelix-governance-execution-authority-v1-blake3-exact-json";

fn resolution() -> RegenerativeRecoveryForkResolutionEvidenceV1 {
    RegenerativeRecoveryForkResolutionEvidenceV1 {
        schema_version: REGENERATIVE_RECOVERY_FORK_RESOLUTION_SCHEMA_V1,
        resolution_id: "resolution-conformance-1".into(),
        external_recovery_reserve_id: "reserve-test".into(),
        external_recovery_reserve_binding: "reserve:test:evidence".into(),
        viability_evidence_content_digest: "11".repeat(32),
        support_closure_continuity_content_digest: "22".repeat(32),
        fork_spend_sequence: 2,
        fork_predecessor_recovery_evidence_content_digest: Some("33".repeat(32)),
        conflicting_recovery_evidence_content_digests: vec!["44".repeat(32), "55".repeat(32)],
        outcome: RegenerativeRecoveryForkResolutionOutcomeV1::RetainSelectedBranch,
        selected_recovery_evidence_content_digest: Some("44".repeat(32)),
        fork_observation_evidence_binding: "fork-observation:reserve-test:2".into(),
        governance_authority_binding: "governance:conformance:test".into(),
        resolution_evidence_binding: "resolution-evidence:reserve-test:2:v1".into(),
    }
}

fn registered_digest(proposal_id: &str, exact_action_json: &str) -> String {
    let mut preimage = Vec::new();
    preimage.extend_from_slice(REGISTERED_EXECUTION_AUTHORITY_DOMAIN);
    preimage.extend_from_slice(&(proposal_id.len() as u64).to_le_bytes());
    preimage.extend_from_slice(proposal_id.as_bytes());
    preimage.extend_from_slice(&(exact_action_json.len() as u64).to_le_bytes());
    preimage.extend_from_slice(exact_action_json.as_bytes());
    blake3::hash(&preimage).to_hex().to_string()
}

#[test]
fn recovery_action_matches_registered_execution_authority_contract() {
    let resolution = resolution();
    let resolution_digest = resolution.content_digest().unwrap();
    let action = qualify_regenerative_recovery_governance_action("MIP-42", &resolution).unwrap();

    let expected_json = format!(
        "{{\"protocol_version\":\"{}\",\"action_type\":\"resolve_regenerative_recovery_fork\",\"resolution_content_digest\":\"{}\",\"resolution_evidence_binding\":\"{}\"}}",
        RECOVERY_GOVERNANCE_ACTION_PROTOCOL_V1,
        resolution_digest,
        resolution.resolution_evidence_binding,
    );

    assert_eq!(action.proposal_id(), "MIP-42");
    assert_eq!(action.exact_action_json(), expected_json);
    assert_eq!(action.actions_digest_profile(), REGISTERED_EXECUTION_AUTHORITY_PROFILE);
    assert_eq!(
        GOVERNANCE_ACTIONS_DIGEST_PROFILE_V1,
        REGISTERED_EXECUTION_AUTHORITY_PROFILE
    );
    assert_eq!(
        action.actions_digest(),
        registered_digest("MIP-42", &expected_json)
    );
}

#[test]
fn proposal_identity_is_digest_significant_even_when_action_json_is_identical() {
    let resolution = resolution();
    let first = qualify_regenerative_recovery_governance_action("MIP-42", &resolution).unwrap();
    let second = qualify_regenerative_recovery_governance_action("MIP-43", &resolution).unwrap();

    assert_eq!(first.exact_action_json(), second.exact_action_json());
    assert_ne!(first.actions_digest(), second.actions_digest());
    assert_eq!(
        first.actions_digest(),
        registered_digest("MIP-42", first.exact_action_json())
    );
    assert_eq!(
        second.actions_digest(),
        registered_digest("MIP-43", second.exact_action_json())
    );
}
