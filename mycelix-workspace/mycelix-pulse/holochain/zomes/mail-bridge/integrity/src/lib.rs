#![deny(unsafe_code)]
// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Mail Bridge Integrity Zome
//!
//! Entry types for cross-cluster dispatch in the unified Mycelix hApp.

use hdi::prelude::*;

/// Cross-cluster query entry for audit trail
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct MailBridgeQuery {
    /// Domain being queried
    pub domain: String,
    /// Query type
    pub query_type: String,
    /// Requester agent
    pub requester: AgentPubKey,
    /// Serialized parameters
    pub params: Vec<u8>,
    /// Result (filled after execution)
    pub result: Option<Vec<u8>>,
    /// When queried
    pub queried_at: Timestamp,
    /// Whether successful
    pub success: bool,
}

/// Cross-cluster event notification
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct MailBridgeEvent {
    /// Event type
    pub event_type: String,
    /// Source domain
    pub source: String,
    /// Serialized payload
    pub payload: Vec<u8>,
    /// Timestamp
    pub timestamp: Timestamp,
}

/// Append-only reproducible-build identity for one exact verifier artifact.
/// Raw binaries and signatures remain outside the DHT; this entry stores only
/// canonical hashes, build identity, and the authenticated rebuild quorum.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct SenderProofVerifierArtifactRecord {
    pub contract_id: String,
    pub circuit_id: String,
    pub backend: String,
    pub artifact_hash: [u8; 32],
    pub artifact_size_bytes: u64,
    pub manifest_hash: [u8; 32],
    pub source_tree_hash: [u8; 32],
    pub dependency_lock_hash: [u8; 32],
    pub toolchain_hash: [u8; 32],
    pub build_recipe_hash: [u8; 32],
    pub build_target: String,
    pub build_profile: String,
    pub rebuild_quorum_hash: [u8; 32],
    pub independent_rebuild_count: u16,
    pub recorded_by: AgentPubKey,
    pub recorded_at: Timestamp,
}

/// Append-only signed-transparency lifecycle record for one exact verifier.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct SenderProofVerifierTransparencyRecord {
    pub contract_id: String,
    pub circuit_id: String,
    pub manifest_hash: [u8; 32],
    pub artifact_hash: [u8; 32],
    pub record_hash: [u8; 32],
    pub previous_record_hash: Option<[u8; 32]>,
    pub sequence: u64,
    pub status: String,
    pub successor_manifest_hash: Option<[u8; 32]>,
    pub reason_hash: Option<[u8; 32]>,
    pub authority_set_hash: [u8; 32],
    pub authority_set_epoch: u64,
    pub rebuild_quorum_hash: [u8; 32],
    pub published_at_micros: i64,
    pub recorded_by: AgentPubKey,
    pub recorded_at: Timestamp,
}

/// Append-only record created only after an exact sender-membership verifier,
/// authenticated envelope signature, and replay preflight all succeed.
///
/// Raw proof and signature bytes are deliberately excluded. The DHT stores the
/// immutable verifier identity, public statement hashes, and both canonical
/// replay keys needed to detect reuse.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct SenderProofConsumption {
    pub contract_id: String,
    pub circuit_id: String,
    pub backend: String,
    pub verifier_hash: [u8; 32],
    pub public_inputs_hash: [u8; 32],
    pub proof_hash: [u8; 32],
    pub message_id_hash: [u8; 32],
    pub group_root: [u8; 32],
    pub epoch: u64,
    pub envelope_replay_key: [u8; 32],
    pub nullifier_replay_key: [u8; 32],
    pub verifier: AgentPubKey,
    pub verified_at: Timestamp,
}

/// Append-only, hash-only evidence that two authenticated reservation grants
/// overlap on at least one canonical replay dimension.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct SenderProofConflictEvidence {
    pub first_receipt_hash: [u8; 32],
    pub second_receipt_hash: [u8; 32],
    pub first_acceptance_id: [u8; 32],
    pub second_acceptance_id: [u8; 32],
    pub first_reservation_slot_id: [u8; 32],
    pub second_reservation_slot_id: [u8; 32],
    pub shared_envelope_replay_key: Option<[u8; 32]>,
    pub shared_nullifier_replay_key: Option<[u8; 32]>,
    pub first_authority_set_hash: [u8; 32],
    pub second_authority_set_hash: [u8; 32],
    pub first_authority_set_epoch: u64,
    pub second_authority_set_epoch: u64,
    pub detected_by: AgentPubKey,
    pub detected_at: Timestamp,
}

/// Append-only evidence that a predecessor quorum authenticated two successors.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct SenderProofAuthorityForkEvidence {
    pub previous_authority_set_hash: [u8; 32],
    pub previous_authority_set_epoch: u64,
    pub first_transition_hash: [u8; 32],
    pub second_transition_hash: [u8; 32],
    pub first_next_authority_set_hash: [u8; 32],
    pub second_next_authority_set_hash: [u8; 32],
    pub next_authority_set_epoch: u64,
    pub effective_at_micros: i64,
    pub detected_by: AgentPubKey,
    pub detected_at: Timestamp,
}

/// Fail-closed disposition for two authenticated grants sharing replay state.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct SenderProofConflictQuarantine {
    pub conflict_evidence_hash: [u8; 32],
    pub adjudication_id: [u8; 32],
    pub first_acceptance_id: [u8; 32],
    pub second_acceptance_id: [u8; 32],
    pub disposition: String,
    pub quarantined_by: AgentPubKey,
    pub quarantined_at: Timestamp,
}

/// Append-only evidence for one quorum-authenticated activation policy record.
///
/// Signature bytes are not replicated. The record stores the canonical hashes
/// and quorum summary produced by the trusted authenticator.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct SenderProofActivationPolicyRecord {
    pub contract_id: String,
    pub policy_id: String,
    pub record_protocol: String,
    pub record_hash: [u8; 32],
    pub previous_record_hash: Option<[u8; 32]>,
    pub operator_set_hash: [u8; 32],
    pub operator_set_epoch: u64,
    pub operator_quorum_hash: [u8; 32],
    pub operator_threshold: u16,
    pub operator_participants: u16,
    pub chain_commitment: [u8; 32],
    pub checkpoint_hash: [u8; 32],
    pub activation_epoch: u64,
    pub transition_kind: String,
    pub requested_mode: String,
    pub emergency_halt: bool,
    pub halt_recovery_ticket_hash: Option<[u8; 32]>,
    pub issued_at_micros: i64,
    pub expires_at_micros: i64,
    pub recorded_by: AgentPubKey,
    pub recorded_at: Timestamp,
}

/// Deterministic append-only audit event for one activation decision.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct SenderProofActivationAuditEvent {
    pub audit_event_protocol: String,
    pub policy_record_hash: [u8; 32],
    pub operator_set_hash: [u8; 32],
    pub chain_commitment: [u8; 32],
    pub activation_epoch: u64,
    pub transition_kind: String,
    pub requested_mode: String,
    pub effective_mode: String,
    pub promotion_ready: bool,
    pub provenance_ready: bool,
    pub emergency_halt: bool,
    pub acceptance_enabled: bool,
    /// Canonically sorted, duplicate-free machine-readable blocker codes.
    pub blocker_codes: Vec<String>,
    pub decision_hash: [u8; 32],
    pub observed_by: AgentPubKey,
    pub observed_at: Timestamp,
}

/// Immutable zero-mismatch differential campaign for an exact verifier pair.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct SenderProofVerifierCompatibilityRecord {
    pub contract_id: String,
    pub predecessor_verifier_hash: [u8; 32],
    pub candidate_verifier_hash: [u8; 32],
    pub evidence_hash: [u8; 32],
    pub corpus_hash: [u8; 32],
    pub valid_proof_cases: u64,
    pub invalid_proof_cases: u64,
    pub boundary_cases: u64,
    pub decision_mismatches: u64,
    pub executed_at_micros: i64,
    pub recorded_by: AgentPubKey,
    pub recorded_at: Timestamp,
}

/// Immutable deployment plan binding the exact pair, evidence, cohort, and window.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct SenderProofVerifierRolloutPlanRecord {
    pub contract_id: String,
    pub plan_hash: [u8; 32],
    pub predecessor_verifier_hash: [u8; 32],
    pub candidate_verifier_hash: [u8; 32],
    pub compatibility_evidence_hash: [u8; 32],
    pub cohort_seed: [u8; 32],
    pub canary_basis_points: u16,
    pub compatibility_window_start_micros: i64,
    pub compatibility_window_end_micros: i64,
    pub created_at_micros: i64,
    pub recorded_by: AgentPubKey,
    pub recorded_at: Timestamp,
}

/// Append-only rollout-state transition. Authentication is performed before write.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct SenderProofVerifierRolloutStateRecord {
    pub contract_id: String,
    pub plan_hash: [u8; 32],
    pub state_hash: [u8; 32],
    pub previous_state_hash: Option<[u8; 32]>,
    pub state_epoch: u64,
    pub stage: String,
    pub effective_at_micros: i64,
    pub reason_hash: [u8; 32],
    pub authenticated_by: AgentPubKey,
    pub recorded_at: Timestamp,
}

/// Externally pinned exact rollout state used to prevent rollback or truncation.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct SenderProofVerifierRolloutCheckpointRecord {
    pub contract_id: String,
    pub checkpoint_hash: [u8; 32],
    pub plan_hash: [u8; 32],
    pub state_hash: [u8; 32],
    pub state_epoch: u64,
    pub stage: String,
    pub pinned_at_micros: i64,
    pub pinned_by: AgentPubKey,
    pub recorded_at: Timestamp,
}

/// Authenticated aggregate health evidence for one exact rollout-state epoch.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct SenderProofVerifierRolloutHealthRecord {
    pub contract_id: String,
    pub health_policy_protocol: String,
    pub telemetry_window_protocol: String,
    pub observer_set_protocol: String,
    pub health_attestation_protocol: String,
    pub health_checkpoint_protocol: String,
    pub plan_hash: [u8; 32],
    pub state_hash: [u8; 32],
    pub state_epoch: u64,
    pub policy_hash: [u8; 32],
    pub telemetry_window_hash: [u8; 32],
    pub observer_set_hash: [u8; 32],
    pub health_decision_hash: [u8; 32],
    pub observer_quorum_hash: [u8; 32],
    pub health_checkpoint_hash: [u8; 32],
    pub observer_threshold: u16,
    pub observer_participants: u16,
    pub window_start_micros: i64,
    pub window_end_micros: i64,
    pub candidate_executions: u64,
    pub decision_mismatches: u64,
    pub candidate_error_basis_points: u16,
    pub dropped_sample_basis_points: u16,
    pub latency_regression_basis_points: u16,
    pub action: String,
    pub authenticated_by: AgentPubKey,
    pub recorded_at: Timestamp,
}

/// Immutable evidence that an automatic freeze/rollback decision was applied.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct SenderProofVerifierRolloutSafetyActionRecord {
    pub contract_id: String,
    pub plan_hash: [u8; 32],
    pub triggering_state_hash: [u8; 32],
    pub triggering_state_epoch: u64,
    pub health_decision_hash: [u8; 32],
    pub health_checkpoint_hash: [u8; 32],
    pub required_action: String,
    pub successor_state_hash: [u8; 32],
    pub successor_state_epoch: u64,
    pub applied_at_micros: i64,
    pub applied_by: AgentPubKey,
    pub recorded_at: Timestamp,
}

/// Append-only commitment to continuous aggregate rollout-health retention.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct SenderProofVerifierTelemetryRetentionRecord {
    pub contract_id: String,
    pub retention_policy_protocol: String,
    pub retention_checkpoint_protocol: String,
    pub plan_hash: [u8; 32],
    pub retention_policy_hash: [u8; 32],
    pub retention_checkpoint_hash: [u8; 32],
    pub previous_checkpoint_hash: Option<[u8; 32]>,
    pub windows_chain_commitment: [u8; 32],
    pub first_window_start_micros: i64,
    pub last_window_end_micros: i64,
    pub window_count: u32,
    pub first_state_epoch: u64,
    pub last_state_epoch: u64,
    pub total_candidate_executions: u64,
    pub total_decision_mismatches: u64,
    pub retention_satisfied: bool,
    pub recorded_by: AgentPubKey,
    pub recorded_at: Timestamp,
}

/// Immutable evidence that the bounded rollback path was exercised.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct SenderProofVerifierRollbackDrillRecord {
    pub contract_id: String,
    pub drill_policy_protocol: String,
    pub drill_report_protocol: String,
    pub plan_hash: [u8; 32],
    pub drill_policy_hash: [u8; 32],
    pub drill_report_hash: [u8; 32],
    pub rollout_health_decision_hash: [u8; 32],
    pub rollout_health_checkpoint_hash: [u8; 32],
    pub started_at_micros: i64,
    pub completed_at_micros: i64,
    pub candidate_acceptance_events_after_trigger: u64,
    pub successful_recovery_requests: u64,
    pub failed_recovery_requests: u64,
    pub final_stage: String,
    pub passed: bool,
    pub recorded_by: AgentPubKey,
    pub recorded_at: Timestamp,
}

/// Quorum-authenticated exact release record. Only hashes and quorum summary are stored.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct SenderProofVerifierReleaseAttestationRecord {
    pub contract_id: String,
    pub release_authority_set_protocol: String,
    pub release_attestation_protocol: String,
    pub plan_hash: [u8; 32],
    pub rollout_state_hash: [u8; 32],
    pub artifact_transparency_hash: [u8; 32],
    pub rollout_health_checkpoint_hash: [u8; 32],
    pub telemetry_retention_checkpoint_hash: [u8; 32],
    pub rollback_drill_report_hash: [u8; 32],
    pub activation_checkpoint_hash: [u8; 32],
    pub custody_policy_hash: [u8; 32],
    pub custody_ceremony_hash: [u8; 32],
    pub custody_ceremony_epoch: u64,
    pub authority_set_hash: [u8; 32],
    pub authority_set_epoch: u64,
    pub release_attestation_hash: [u8; 32],
    pub previous_release_hash: Option<[u8; 32]>,
    pub release_sequence: u64,
    pub issued_at_micros: i64,
    pub expires_at_micros: i64,
    pub release_quorum_hash: [u8; 32],
    pub authority_threshold: u16,
    pub authority_participants: u16,
    pub recorded_by: AgentPubKey,
    pub recorded_at: Timestamp,
}

/// Append-only evidence for one hardware-backed verifier-authority ceremony.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct SenderProofVerifierCustodyCeremonyRecord {
    pub contract_id: String,
    pub custody_policy_protocol: String,
    pub ceremony_record_protocol: String,
    pub authority_role: String,
    pub custody_policy_hash: [u8; 32],
    pub ceremony_hash: [u8; 32],
    pub previous_ceremony_hash: Option<[u8; 32]>,
    pub ceremony_epoch: u64,
    pub custodian_set_hash: [u8; 32],
    pub hardware_attestation_set_hash: [u8; 32],
    pub ceremony_transcript_hash: [u8; 32],
    pub ceremony_quorum_hash: [u8; 32],
    pub custodian_threshold: u16,
    pub custodian_participants: u16,
    pub role_separation_verified: bool,
    pub created_at_micros: i64,
    pub expires_at_micros: i64,
    pub recorded_by: AgentPubKey,
    pub recorded_at: Timestamp,
}

/// Append-only notice that an exact authority ceremony or key set is compromised.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct SenderProofVerifierCustodyCompromiseRecord {
    pub contract_id: String,
    pub compromise_notice_protocol: String,
    pub authority_role: String,
    pub compromise_notice_hash: [u8; 32],
    pub affected_ceremony_hash: [u8; 32],
    pub affected_key_set_hash: [u8; 32],
    pub reason_code_hash: [u8; 32],
    pub notice_sequence: u64,
    pub previous_notice_hash: Option<[u8; 32]>,
    pub detected_at_micros: i64,
    pub effective_at_micros: i64,
    pub recorded_by: AgentPubKey,
    pub recorded_at: Timestamp,
}

/// Append-only recovery binding a compromise notice to a replacement ceremony.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct SenderProofVerifierCustodyRecoveryRecord {
    pub contract_id: String,
    pub recovery_record_protocol: String,
    pub authority_role: String,
    pub recovery_record_hash: [u8; 32],
    pub compromise_notice_hash: [u8; 32],
    pub previous_ceremony_hash: [u8; 32],
    pub replacement_ceremony_hash: [u8; 32],
    pub recovery_authority_hash: [u8; 32],
    pub recovery_epoch: u64,
    pub issued_at_micros: i64,
    pub expires_at_micros: i64,
    pub recorded_by: AgentPubKey,
    pub recorded_at: Timestamp,
}

/// Append-only authorization for one exact in-service custody-key rotation.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct SenderProofVerifierCustodyRotationRecord {
    pub contract_id: String,
    pub rotation_policy_protocol: String,
    pub rotation_record_protocol: String,
    pub authority_role: String,
    pub rotation_mode: String,
    pub rotation_policy_hash: [u8; 32],
    pub rotation_hash: [u8; 32],
    pub rotation_sequence: u64,
    pub previous_rotation_hash: Option<[u8; 32]>,
    pub retiring_ceremony_hash: [u8; 32],
    pub retiring_ceremony_epoch: u64,
    pub retiring_threshold: u16,
    pub replacement_ceremony_hash: [u8; 32],
    pub replacement_ceremony_epoch: u64,
    pub replacement_threshold: u16,
    pub retiring_quorum_hash: Option<[u8; 32]>,
    pub replacement_quorum_hash: [u8; 32],
    pub recovery_quorum_hash: Option<[u8; 32]>,
    pub threshold_not_reduced: bool,
    pub retiring_keys_disabled: bool,
    pub initiated_at_micros: i64,
    pub effective_at_micros: i64,
    pub retiring_keys_disabled_at_micros: i64,
    pub recorded_by: AgentPubKey,
    pub recorded_at: Timestamp,
}

/// Append-only evidence that a specific hardware custody device was lost,
/// destroyed, or suspected compromised. Raw device material is never stored.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct SenderProofVerifierDeviceLossNoticeRecord {
    pub contract_id: String,
    pub loss_notice_protocol: String,
    pub authority_role: String,
    pub custody_slot_id: [u8; 32],
    pub loss_notice_hash: [u8; 32],
    pub lost_device_id: [u8; 32],
    pub lost_generation: u64,
    pub reason: String,
    pub evidence_hash: [u8; 32],
    pub reported_at_micros: i64,
    pub expires_at_micros: i64,
    pub recorded_by: AgentPubKey,
    pub recorded_at: Timestamp,
}

/// Append-only authorization for one exact cross-device custody succession.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct SenderProofVerifierDeviceSuccessionRecord {
    pub contract_id: String,
    pub succession_policy_protocol: String,
    pub succession_record_protocol: String,
    pub authority_role: String,
    pub succession_mode: String,
    pub custody_slot_id: [u8; 32],
    pub policy_hash: [u8; 32],
    pub succession_hash: [u8; 32],
    pub succession_sequence: u64,
    pub previous_succession_hash: Option<[u8; 32]>,
    pub retiring_device_id: [u8; 32],
    pub retiring_generation: u64,
    pub replacement_device_id: [u8; 32],
    pub replacement_generation: u64,
    pub retiring_device_approval_hash: Option<[u8; 32]>,
    pub replacement_device_approval_hash: [u8; 32],
    pub recovery_quorum_hash: Option<[u8; 32]>,
    pub loss_notice_hash: Option<[u8; 32]>,
    pub retiring_device_disabled: bool,
    pub initiated_at_micros: i64,
    pub effective_at_micros: i64,
    pub retiring_device_disabled_at_micros: i64,
    pub recorded_by: AgentPubKey,
    pub recorded_at: Timestamp,
}

/// Externally pinned active-device checkpoint after verified succession.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct SenderProofVerifierDeviceSuccessionCheckpointRecord {
    pub contract_id: String,
    pub succession_checkpoint_protocol: String,
    pub authority_role: String,
    pub custody_slot_id: [u8; 32],
    pub checkpoint_hash: [u8; 32],
    pub previous_checkpoint_hash: Option<[u8; 32]>,
    pub succession_hash: [u8; 32],
    pub succession_sequence: u64,
    pub active_device_id: [u8; 32],
    pub active_device_generation: u64,
    pub issued_at_micros: i64,
    pub recorded_by: AgentPubKey,
    pub recorded_at: Timestamp,
}

/// Append-only measured-state evidence for one active custody device.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct SenderProofVerifierDeviceAttestationRecord {
    pub contract_id: String,
    pub attestation_policy_protocol: String,
    pub attestation_record_protocol: String,
    pub authority_role: String,
    pub custody_slot_id: [u8; 32],
    pub device_id: [u8; 32],
    pub device_generation: u64,
    pub succession_checkpoint_hash: [u8; 32],
    pub policy_hash: [u8; 32],
    pub attestation_hash: [u8; 32],
    pub attestation_sequence: u64,
    pub previous_attestation_hash: Option<[u8; 32]>,
    pub platform_measurement_hash: [u8; 32],
    pub firmware_measurement_hash: [u8; 32],
    pub endorsement_key_hash: [u8; 32],
    pub challenge_nonce_hash: [u8; 32],
    pub secure_boot_enabled: bool,
    pub debug_disabled: bool,
    pub monotonic_boot_counter: u64,
    pub issued_at_micros: i64,
    pub expires_at_micros: i64,
    pub evidence_hash: [u8; 32],
    pub recorded_by: AgentPubKey,
    pub recorded_at: Timestamp,
}

/// Canonical evidence that one device identity emitted two incompatible
/// authenticated attestations at the same sequence.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct SenderProofVerifierDeviceCloneEvidenceRecord {
    pub contract_id: String,
    pub clone_evidence_protocol: String,
    pub device_id: [u8; 32],
    pub endorsement_key_hash: [u8; 32],
    pub attestation_sequence: u64,
    pub first_attestation_hash: [u8; 32],
    pub second_attestation_hash: [u8; 32],
    pub observed_at_micros: i64,
    pub recorded_by: AgentPubKey,
    pub recorded_at: Timestamp,
}

/// Append-only evidence that a retired custody device was zeroized and revoked.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct SenderProofVerifierDeviceDecommissionRecord {
    pub contract_id: String,
    pub decommission_record_protocol: String,
    pub authority_role: String,
    pub custody_slot_id: [u8; 32],
    pub device_id: [u8; 32],
    pub device_generation: u64,
    pub final_attestation_hash: [u8; 32],
    pub succession_checkpoint_hash: [u8; 32],
    pub decommission_hash: [u8; 32],
    pub key_zeroized: bool,
    pub credentials_revoked: bool,
    pub destruction_evidence_hash: [u8; 32],
    pub decommissioned_at_micros: i64,
    pub recorded_by: AgentPubKey,
    pub recorded_at: Timestamp,
}

/// Externally pinned fresh-device attestation checkpoint.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct SenderProofVerifierDeviceAttestationCheckpointRecord {
    pub contract_id: String,
    pub attestation_checkpoint_protocol: String,
    pub authority_role: String,
    pub custody_slot_id: [u8; 32],
    pub active_device_id: [u8; 32],
    pub active_device_generation: u64,
    pub checkpoint_hash: [u8; 32],
    pub previous_checkpoint_hash: Option<[u8; 32]>,
    pub attestation_hash: [u8; 32],
    pub attestation_sequence: u64,
    pub monotonic_boot_counter: u64,
    pub clone_evidence_hash: Option<[u8; 32]>,
    pub latest_decommission_hash: Option<[u8; 32]>,
    pub issued_at_micros: i64,
    pub recorded_by: AgentPubKey,
    pub recorded_at: Timestamp,
}

/// Canonical fail-closed policy governing attestation roots and measurements.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct SenderProofVerifierDeviceTrustPolicyRecord {
    pub contract_id: String,
    pub trust_policy_protocol: String,
    pub trust_policy_hash: [u8; 32],
    pub require_dual_root_approval: bool,
    pub require_measurement_allowlist: bool,
    pub require_monotonic_policy_epochs: bool,
    pub require_external_checkpoint: bool,
    pub maximum_root_handoff_micros: i64,
    pub recorded_by: AgentPubKey,
    pub recorded_at: Timestamp,
}

/// Hash-only publication of one canonical approved measurement-policy epoch.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct SenderProofVerifierDeviceMeasurementPolicyRecord {
    pub contract_id: String,
    pub measurement_policy_protocol: String,
    pub policy_hash: [u8; 32],
    pub policy_epoch: u64,
    pub previous_policy_hash: Option<[u8; 32]>,
    pub approved_measurement_set_hash: [u8; 32],
    pub approved_measurement_count: u16,
    pub issued_at_micros: i64,
    pub expires_at_micros: i64,
    pub recorded_by: AgentPubKey,
    pub recorded_at: Timestamp,
}

/// Dual-root authorization for one exact attestation-root replacement.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct SenderProofVerifierDeviceAttestationRootRotationRecord {
    pub contract_id: String,
    pub root_rotation_protocol: String,
    pub rotation_hash: [u8; 32],
    pub rotation_sequence: u64,
    pub previous_rotation_hash: Option<[u8; 32]>,
    pub current_root_hash: [u8; 32],
    pub replacement_root_hash: [u8; 32],
    pub current_root_approval_hash: [u8; 32],
    pub replacement_root_approval_hash: [u8; 32],
    pub issued_at_micros: i64,
    pub effective_at_micros: i64,
    pub current_root_retire_at_micros: i64,
    pub recorded_by: AgentPubKey,
    pub recorded_at: Timestamp,
}

/// Exact endorsement-chain and measurement authorization for one attestation.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct SenderProofVerifierDeviceTrustBindingRecord {
    pub contract_id: String,
    pub trust_binding_protocol: String,
    pub binding_hash: [u8; 32],
    pub attestation_hash: [u8; 32],
    pub trust_policy_hash: [u8; 32],
    pub attestation_root_hash: [u8; 32],
    pub measurement_policy_hash: [u8; 32],
    pub measurement_policy_epoch: u64,
    pub security_epoch: u64,
    pub endorsement_certificate_hash: [u8; 32],
    pub issued_at_micros: i64,
    pub expires_at_micros: i64,
    pub recorded_by: AgentPubKey,
    pub recorded_at: Timestamp,
}

/// Externally pinned root/policy/attestation trust checkpoint.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct SenderProofVerifierDeviceTrustCheckpointRecord {
    pub contract_id: String,
    pub trust_checkpoint_protocol: String,
    pub checkpoint_hash: [u8; 32],
    pub checkpoint_sequence: u64,
    pub previous_checkpoint_hash: Option<[u8; 32]>,
    pub trust_policy_hash: [u8; 32],
    pub active_root_hash: [u8; 32],
    pub root_rotation_sequence: u64,
    pub measurement_policy_hash: [u8; 32],
    pub measurement_policy_epoch: u64,
    pub trust_binding_hash: [u8; 32],
    pub issued_at_micros: i64,
    pub recorded_by: AgentPubKey,
    pub recorded_at: Timestamp,
}

/// Canonical measured-boot verification policy for the active custody device.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct SenderProofVerifierMeasuredBootPolicyRecord {
    pub contract_id: String,
    pub measured_boot_policy_protocol: String,
    pub policy_hash: [u8; 32],
    pub required_pcr_set_hash: [u8; 32],
    pub required_pcr_count: u8,
    pub minimum_event_count: u32,
    pub maximum_event_count: u32,
    pub maximum_evidence_age_micros: i64,
    pub require_complete_log: bool,
    pub require_revocation_check: bool,
    pub require_external_checkpoint: bool,
    pub recorded_by: AgentPubKey,
    pub recorded_at: Timestamp,
}

/// Hash-only publication of one verified ordered boot-event log.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct SenderProofVerifierMeasuredBootEvidenceRecord {
    pub contract_id: String,
    pub measured_boot_evidence_protocol: String,
    pub evidence_hash: [u8; 32],
    pub policy_hash: [u8; 32],
    pub device_id: [u8; 32],
    pub device_attestation_hash: [u8; 32],
    pub attestation_root_hash: [u8; 32],
    pub platform_measurement_hash: [u8; 32],
    pub firmware_measurement_hash: [u8; 32],
    pub boot_counter: u64,
    pub evidence_sequence: u64,
    pub event_log_hash: [u8; 32],
    pub event_count: u32,
    pub final_pcr_set_hash: [u8; 32],
    pub issued_at_micros: i64,
    pub expires_at_micros: i64,
    pub recorded_by: AgentPubKey,
    pub recorded_at: Timestamp,
}

/// Monotonic root and measurement revocation state used for boot trust.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct SenderProofVerifierDeviceTrustRevocationSetRecord {
    pub contract_id: String,
    pub revocation_set_protocol: String,
    pub revocation_set_hash: [u8; 32],
    pub revocation_epoch: u64,
    pub previous_revocation_set_hash: Option<[u8; 32]>,
    pub revocation_entry_set_hash: [u8; 32],
    pub revocation_entry_count: u16,
    pub issued_at_micros: i64,
    pub recorded_by: AgentPubKey,
    pub recorded_at: Timestamp,
}

/// Recovery linkage from a compromised measurement policy to its replacement.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct SenderProofVerifierMeasurementRecoveryRecord {
    pub contract_id: String,
    pub measurement_recovery_protocol: String,
    pub recovery_hash: [u8; 32],
    pub incident_hash: [u8; 32],
    pub revoked_measurement_policy_hash: [u8; 32],
    pub replacement_measurement_policy_hash: [u8; 32],
    pub replacement_policy_epoch: u64,
    pub revocation_set_hash: [u8; 32],
    pub revocation_epoch: u64,
    pub authorized_at_micros: i64,
    pub effective_at_micros: i64,
    pub recorded_by: AgentPubKey,
    pub recorded_at: Timestamp,
}

/// Externally pinned measured-boot, revocation, and recovery checkpoint.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct SenderProofVerifierMeasuredBootCheckpointRecord {
    pub contract_id: String,
    pub measured_boot_checkpoint_protocol: String,
    pub checkpoint_hash: [u8; 32],
    pub checkpoint_sequence: u64,
    pub previous_checkpoint_hash: Option<[u8; 32]>,
    pub measured_boot_evidence_hash: [u8; 32],
    pub measured_boot_evidence_sequence: u64,
    pub revocation_set_hash: [u8; 32],
    pub revocation_epoch: u64,
    pub recovery_record_hash: Option<[u8; 32]>,
    pub issued_at_micros: i64,
    pub recorded_by: AgentPubKey,
    pub recorded_at: Timestamp,
}

/// Canonical policy governing endorsement chains and firmware anti-rollback.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct SenderProofVerifierEndorsementPolicyRecord {
    pub contract_id: String,
    pub endorsement_policy_protocol: String,
    pub policy_hash: [u8; 32],
    pub maximum_chain_depth: u8,
    pub minimum_certificate_security_epoch: u64,
    pub minimum_firmware_security_version: u64,
    pub maximum_credential_lifetime_micros: i64,
    pub require_manufacturer_root_set: bool,
    pub require_revocation_check: bool,
    pub require_firmware_anti_rollback: bool,
    pub require_external_checkpoint: bool,
    pub recorded_by: AgentPubKey,
    pub recorded_at: Timestamp,
}

/// Hash-only publication of the active manufacturer-root governance set.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct SenderProofVerifierManufacturerRootSetRecord {
    pub contract_id: String,
    pub manufacturer_root_set_protocol: String,
    pub root_set_hash: [u8; 32],
    pub root_set_epoch: u64,
    pub previous_root_set_hash: Option<[u8; 32]>,
    pub active_root_set_hash: [u8; 32],
    pub active_root_count: u16,
    pub issued_at_micros: i64,
    pub expires_at_micros: i64,
    pub recorded_by: AgentPubKey,
    pub recorded_at: Timestamp,
}

/// Hash-only evidence for one exact endorsement chain and firmware floor.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct SenderProofVerifierEndorsementChainEvidenceRecord {
    pub contract_id: String,
    pub endorsement_chain_evidence_protocol: String,
    pub evidence_hash: [u8; 32],
    pub evidence_sequence: u64,
    pub previous_evidence_hash: Option<[u8; 32]>,
    pub endorsement_policy_hash: [u8; 32],
    pub manufacturer_root_set_hash: [u8; 32],
    pub manufacturer_root_set_epoch: u64,
    pub active_manufacturer_root_hash: [u8; 32],
    pub leaf_certificate_hash: [u8; 32],
    pub certificate_chain_hash: [u8; 32],
    pub certificate_count: u8,
    pub endorsement_key_hash: [u8; 32],
    pub certificate_security_epoch: u64,
    pub firmware_security_version: u64,
    pub endorsement_revocation_set_hash: [u8; 32],
    pub endorsement_revocation_epoch: u64,
    pub verified_at_micros: i64,
    pub recorded_by: AgentPubKey,
    pub recorded_at: Timestamp,
}

/// Monotonic manufacturer-root, certificate, and endorsement-key revocations.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct SenderProofVerifierEndorsementRevocationSetRecord {
    pub contract_id: String,
    pub endorsement_revocation_set_protocol: String,
    pub revocation_set_hash: [u8; 32],
    pub revocation_epoch: u64,
    pub previous_revocation_set_hash: Option<[u8; 32]>,
    pub revoked_root_set_hash: [u8; 32],
    pub revoked_certificate_set_hash: [u8; 32],
    pub revoked_key_set_hash: [u8; 32],
    pub revoked_item_count: u16,
    pub issued_at_micros: i64,
    pub recorded_by: AgentPubKey,
    pub recorded_at: Timestamp,
}

/// Externally pinned endorsement-chain and anti-rollback checkpoint.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct SenderProofVerifierEndorsementCheckpointRecord {
    pub contract_id: String,
    pub endorsement_checkpoint_protocol: String,
    pub checkpoint_hash: [u8; 32],
    pub checkpoint_sequence: u64,
    pub previous_checkpoint_hash: Option<[u8; 32]>,
    pub endorsement_policy_hash: [u8; 32],
    pub manufacturer_root_set_hash: [u8; 32],
    pub manufacturer_root_set_epoch: u64,
    pub active_manufacturer_root_hash: [u8; 32],
    pub endorsement_evidence_hash: [u8; 32],
    pub endorsement_evidence_sequence: u64,
    pub endorsement_revocation_set_hash: [u8; 32],
    pub endorsement_revocation_epoch: u64,
    pub firmware_security_version: u64,
    pub issued_at_micros: i64,
    pub recorded_by: AgentPubKey,
    pub recorded_at: Timestamp,
}

/// Externally pinned active-custody checkpoint after a verified rotation.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct SenderProofVerifierCustodyRotationCheckpointRecord {
    pub contract_id: String,
    pub rotation_checkpoint_protocol: String,
    pub authority_role: String,
    pub checkpoint_hash: [u8; 32],
    pub previous_checkpoint_hash: Option<[u8; 32]>,
    pub rotation_hash: [u8; 32],
    pub rotation_sequence: u64,
    pub active_ceremony_hash: [u8; 32],
    pub active_ceremony_epoch: u64,
    pub issued_at_micros: i64,
    pub recorded_by: AgentPubKey,
    pub recorded_at: Timestamp,
}

#[hdk_link_types]
pub enum LinkTypes {
    /// Agent to their bridge queries
    AgentToQueries,
    /// Anchor to events
    AnchorToEvents,
    /// Canonical authenticated-envelope nonce replay key to consumption.
    EnvelopeReplayKeyToConsumption,
    /// Canonical sender nullifier replay key to consumption.
    NullifierReplayKeyToConsumption,
    /// Deterministic conflict anchor to immutable conflict evidence.
    ReservationConflictToEvidence,
    /// Predecessor authority-set hash to authenticated fork evidence.
    AuthoritySetToForkEvidence,
    /// Acceptance identity to fail-closed conflict quarantine.
    AcceptanceToConflictQuarantine,
    /// Verifier manifest hash to its immutable reproducible-build record.
    VerifierManifestToArtifactRecord,
    /// Verifier manifest hash to signed transparency lifecycle records.
    VerifierManifestToTransparencyRecord,
    /// Candidate verifier hash to zero-mismatch differential evidence.
    VerifierCandidateToCompatibilityEvidence,
    /// Rollout-plan hash to immutable state transitions.
    VerifierRolloutPlanToState,
    /// Rollout-plan hash to externally pinned state checkpoints.
    VerifierRolloutPlanToCheckpoint,
    /// Rollout-plan hash to authenticated aggregate health evidence.
    VerifierRolloutPlanToHealthEvidence,
    /// Health-decision hash to automatic freeze/rollback application evidence.
    VerifierHealthDecisionToSafetyAction,
    /// Rollout-plan hash to retained aggregate telemetry checkpoints.
    VerifierRolloutPlanToRetentionCheckpoint,
    /// Rollout-plan hash to bounded rollback-drill reports.
    VerifierRolloutPlanToRollbackDrill,
    /// Rollout-plan hash to quorum-authenticated release attestations.
    VerifierRolloutPlanToReleaseAttestation,
    /// Activation chain commitment to one immutable policy record.
    ActivationChainToPolicyRecord,
    /// Activation policy record hash to deterministic audit events.
    ActivationPolicyRecordToAuditEvent,
    /// Custody policy hash to exact ceremony records.
    VerifierCustodyPolicyToCeremony,
    /// Custody ceremony hash to compromise notices.
    VerifierCustodyCeremonyToCompromise,
    /// Compromise notice hash to authenticated recovery evidence.
    VerifierCustodyCompromiseToRecovery,
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    #[entry_type(required_validations = 1)]
    MailBridgeQuery(MailBridgeQuery),
    #[entry_type(required_validations = 1)]
    MailBridgeEvent(MailBridgeEvent),
    #[entry_type(required_validations = 3)]
    SenderProofConsumption(SenderProofConsumption),
    #[entry_type(required_validations = 3)]
    SenderProofVerifierArtifactRecord(SenderProofVerifierArtifactRecord),
    #[entry_type(required_validations = 3)]
    SenderProofVerifierTransparencyRecord(SenderProofVerifierTransparencyRecord),
    #[entry_type(required_validations = 3)]
    SenderProofVerifierCompatibilityRecord(SenderProofVerifierCompatibilityRecord),
    #[entry_type(required_validations = 3)]
    SenderProofVerifierRolloutPlanRecord(SenderProofVerifierRolloutPlanRecord),
    #[entry_type(required_validations = 3)]
    SenderProofVerifierRolloutStateRecord(SenderProofVerifierRolloutStateRecord),
    #[entry_type(required_validations = 3)]
    SenderProofVerifierRolloutCheckpointRecord(SenderProofVerifierRolloutCheckpointRecord),
    #[entry_type(required_validations = 3)]
    SenderProofVerifierRolloutHealthRecord(SenderProofVerifierRolloutHealthRecord),
    #[entry_type(required_validations = 3)]
    SenderProofVerifierRolloutSafetyActionRecord(SenderProofVerifierRolloutSafetyActionRecord),
    #[entry_type(required_validations = 3)]
    SenderProofVerifierTelemetryRetentionRecord(SenderProofVerifierTelemetryRetentionRecord),
    #[entry_type(required_validations = 3)]
    SenderProofVerifierRollbackDrillRecord(SenderProofVerifierRollbackDrillRecord),
    #[entry_type(required_validations = 3)]
    SenderProofVerifierReleaseAttestationRecord(SenderProofVerifierReleaseAttestationRecord),
    #[entry_type(required_validations = 3)]
    SenderProofVerifierCustodyCeremonyRecord(SenderProofVerifierCustodyCeremonyRecord),
    #[entry_type(required_validations = 3)]
    SenderProofVerifierCustodyCompromiseRecord(SenderProofVerifierCustodyCompromiseRecord),
    #[entry_type(required_validations = 3)]
    SenderProofVerifierCustodyRecoveryRecord(SenderProofVerifierCustodyRecoveryRecord),
    #[entry_type(required_validations = 3)]
    SenderProofVerifierCustodyRotationRecord(SenderProofVerifierCustodyRotationRecord),
    #[entry_type(required_validations = 3)]
    SenderProofVerifierCustodyRotationCheckpointRecord(
        SenderProofVerifierCustodyRotationCheckpointRecord,
    ),
    #[entry_type(required_validations = 3)]
    SenderProofVerifierDeviceLossNoticeRecord(SenderProofVerifierDeviceLossNoticeRecord),
    #[entry_type(required_validations = 3)]
    SenderProofVerifierDeviceSuccessionRecord(SenderProofVerifierDeviceSuccessionRecord),
    #[entry_type(required_validations = 3)]
    SenderProofVerifierDeviceSuccessionCheckpointRecord(
        SenderProofVerifierDeviceSuccessionCheckpointRecord,
    ),
    #[entry_type(required_validations = 3)]
    SenderProofVerifierDeviceAttestationRecord(SenderProofVerifierDeviceAttestationRecord),
    #[entry_type(required_validations = 3)]
    SenderProofVerifierDeviceCloneEvidenceRecord(SenderProofVerifierDeviceCloneEvidenceRecord),
    #[entry_type(required_validations = 3)]
    SenderProofVerifierDeviceDecommissionRecord(SenderProofVerifierDeviceDecommissionRecord),
    #[entry_type(required_validations = 3)]
    SenderProofVerifierDeviceAttestationCheckpointRecord(
        SenderProofVerifierDeviceAttestationCheckpointRecord,
    ),
    #[entry_type(required_validations = 3)]
    SenderProofVerifierDeviceTrustPolicyRecord(SenderProofVerifierDeviceTrustPolicyRecord),
    #[entry_type(required_validations = 3)]
    SenderProofVerifierDeviceMeasurementPolicyRecord(
        SenderProofVerifierDeviceMeasurementPolicyRecord,
    ),
    #[entry_type(required_validations = 3)]
    SenderProofVerifierDeviceAttestationRootRotationRecord(
        SenderProofVerifierDeviceAttestationRootRotationRecord,
    ),
    #[entry_type(required_validations = 3)]
    SenderProofVerifierDeviceTrustBindingRecord(SenderProofVerifierDeviceTrustBindingRecord),
    #[entry_type(required_validations = 3)]
    SenderProofVerifierDeviceTrustCheckpointRecord(SenderProofVerifierDeviceTrustCheckpointRecord),
    #[entry_type(required_validations = 3)]
    SenderProofVerifierMeasuredBootPolicyRecord(SenderProofVerifierMeasuredBootPolicyRecord),
    #[entry_type(required_validations = 3)]
    SenderProofVerifierMeasuredBootEvidenceRecord(SenderProofVerifierMeasuredBootEvidenceRecord),
    #[entry_type(required_validations = 3)]
    SenderProofVerifierDeviceTrustRevocationSetRecord(
        SenderProofVerifierDeviceTrustRevocationSetRecord,
    ),
    #[entry_type(required_validations = 3)]
    SenderProofVerifierMeasurementRecoveryRecord(SenderProofVerifierMeasurementRecoveryRecord),
    #[entry_type(required_validations = 3)]
    SenderProofVerifierMeasuredBootCheckpointRecord(
        SenderProofVerifierMeasuredBootCheckpointRecord,
    ),
    #[entry_type(required_validations = 3)]
    SenderProofVerifierEndorsementPolicyRecord(SenderProofVerifierEndorsementPolicyRecord),
    #[entry_type(required_validations = 3)]
    SenderProofVerifierManufacturerRootSetRecord(SenderProofVerifierManufacturerRootSetRecord),
    #[entry_type(required_validations = 3)]
    SenderProofVerifierEndorsementChainEvidenceRecord(
        SenderProofVerifierEndorsementChainEvidenceRecord,
    ),
    #[entry_type(required_validations = 3)]
    SenderProofVerifierEndorsementRevocationSetRecord(
        SenderProofVerifierEndorsementRevocationSetRecord,
    ),
    #[entry_type(required_validations = 3)]
    SenderProofVerifierEndorsementCheckpointRecord(SenderProofVerifierEndorsementCheckpointRecord),
    #[entry_type(required_validations = 3)]
    SenderProofConflictEvidence(SenderProofConflictEvidence),
    #[entry_type(required_validations = 3)]
    SenderProofAuthorityForkEvidence(SenderProofAuthorityForkEvidence),
    #[entry_type(required_validations = 3)]
    SenderProofConflictQuarantine(SenderProofConflictQuarantine),
    #[entry_type(required_validations = 3)]
    SenderProofActivationPolicyRecord(SenderProofActivationPolicyRecord),
    #[entry_type(required_validations = 3)]
    SenderProofActivationAuditEvent(SenderProofActivationAuditEvent),
}

#[hdk_extern]
pub fn genesis_self_check(_data: GenesisSelfCheckData) -> ExternResult<ValidateCallbackResult> {
    Ok(ValidateCallbackResult::Valid)
}

#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::MailBridgeQuery(query) => {
                    if query.requester != action.author {
                        return Ok(ValidateCallbackResult::Invalid(
                            "Requester must match author".to_string(),
                        ));
                    }
                    if query.domain.is_empty() || query.query_type.is_empty() {
                        return Ok(ValidateCallbackResult::Invalid(
                            "Domain and query_type cannot be empty".to_string(),
                        ));
                    }
                    Ok(ValidateCallbackResult::Valid)
                }
                EntryTypes::SenderProofConsumption(consumption) => {
                    validate_sender_proof_consumption(&consumption, &action)
                }
                EntryTypes::SenderProofVerifierArtifactRecord(record) => {
                    validate_sender_proof_verifier_artifact_record(&record, &action)
                }
                EntryTypes::SenderProofVerifierTransparencyRecord(record) => {
                    validate_sender_proof_verifier_transparency_record(&record, &action)
                }
                EntryTypes::SenderProofVerifierCompatibilityRecord(record) => {
                    validate_sender_proof_verifier_compatibility_record(&record, &action)
                }
                EntryTypes::SenderProofVerifierRolloutPlanRecord(record) => {
                    validate_sender_proof_verifier_rollout_plan_record(&record, &action)
                }
                EntryTypes::SenderProofVerifierRolloutStateRecord(record) => {
                    validate_sender_proof_verifier_rollout_state_record(&record, &action)
                }
                EntryTypes::SenderProofVerifierRolloutCheckpointRecord(record) => {
                    validate_sender_proof_verifier_rollout_checkpoint_record(&record, &action)
                }
                EntryTypes::SenderProofVerifierRolloutHealthRecord(record) => {
                    validate_sender_proof_verifier_rollout_health_record(&record, &action)
                }
                EntryTypes::SenderProofVerifierRolloutSafetyActionRecord(record) => {
                    validate_sender_proof_verifier_rollout_safety_action_record(&record, &action)
                }
                EntryTypes::SenderProofVerifierTelemetryRetentionRecord(record) => {
                    validate_sender_proof_verifier_telemetry_retention_record(&record, &action)
                }
                EntryTypes::SenderProofVerifierRollbackDrillRecord(record) => {
                    validate_sender_proof_verifier_rollback_drill_record(&record, &action)
                }
                EntryTypes::SenderProofVerifierReleaseAttestationRecord(record) => {
                    validate_sender_proof_verifier_release_attestation_record(&record, &action)
                }
                EntryTypes::SenderProofVerifierCustodyCeremonyRecord(record) => {
                    validate_sender_proof_verifier_custody_ceremony_record(&record, &action)
                }
                EntryTypes::SenderProofVerifierCustodyCompromiseRecord(record) => {
                    validate_sender_proof_verifier_custody_compromise_record(&record, &action)
                }
                EntryTypes::SenderProofVerifierCustodyRecoveryRecord(record) => {
                    validate_sender_proof_verifier_custody_recovery_record(&record, &action)
                }
                EntryTypes::SenderProofVerifierCustodyRotationRecord(record) => {
                    validate_sender_proof_verifier_custody_rotation_record(&record, &action)
                }
                EntryTypes::SenderProofVerifierCustodyRotationCheckpointRecord(record) => {
                    validate_sender_proof_verifier_custody_rotation_checkpoint_record(
                        &record, &action,
                    )
                }
                EntryTypes::SenderProofVerifierDeviceLossNoticeRecord(record) => {
                    validate_sender_proof_verifier_device_loss_notice_record(&record, &action)
                }
                EntryTypes::SenderProofVerifierDeviceSuccessionRecord(record) => {
                    validate_sender_proof_verifier_device_succession_record(&record, &action)
                }
                EntryTypes::SenderProofVerifierDeviceSuccessionCheckpointRecord(record) => {
                    validate_sender_proof_verifier_device_succession_checkpoint_record(
                        &record, &action,
                    )
                }
                EntryTypes::SenderProofVerifierDeviceAttestationRecord(record) => {
                    validate_sender_proof_verifier_device_attestation_record(&record, &action)
                }
                EntryTypes::SenderProofVerifierDeviceCloneEvidenceRecord(record) => {
                    validate_sender_proof_verifier_device_clone_evidence_record(&record, &action)
                }
                EntryTypes::SenderProofVerifierDeviceDecommissionRecord(record) => {
                    validate_sender_proof_verifier_device_decommission_record(&record, &action)
                }
                EntryTypes::SenderProofVerifierDeviceAttestationCheckpointRecord(record) => {
                    validate_sender_proof_verifier_device_attestation_checkpoint_record(
                        &record, &action,
                    )
                }
                EntryTypes::SenderProofVerifierDeviceTrustPolicyRecord(record) => {
                    validate_sender_proof_verifier_device_trust_policy_record(&record, &action)
                }
                EntryTypes::SenderProofVerifierDeviceMeasurementPolicyRecord(record) => {
                    validate_sender_proof_verifier_device_measurement_policy_record(
                        &record, &action,
                    )
                }
                EntryTypes::SenderProofVerifierDeviceAttestationRootRotationRecord(record) => {
                    validate_sender_proof_verifier_device_attestation_root_rotation_record(
                        &record, &action,
                    )
                }
                EntryTypes::SenderProofVerifierDeviceTrustBindingRecord(record) => {
                    validate_sender_proof_verifier_device_trust_binding_record(&record, &action)
                }
                EntryTypes::SenderProofVerifierDeviceTrustCheckpointRecord(record) => {
                    validate_sender_proof_verifier_device_trust_checkpoint_record(&record, &action)
                }
                EntryTypes::SenderProofVerifierMeasuredBootPolicyRecord(record) => {
                    validate_sender_proof_verifier_measured_boot_policy_record(&record, &action)
                }
                EntryTypes::SenderProofVerifierMeasuredBootEvidenceRecord(record) => {
                    validate_sender_proof_verifier_measured_boot_evidence_record(&record, &action)
                }
                EntryTypes::SenderProofVerifierDeviceTrustRevocationSetRecord(record) => {
                    validate_sender_proof_verifier_device_trust_revocation_set_record(
                        &record, &action,
                    )
                }
                EntryTypes::SenderProofVerifierMeasurementRecoveryRecord(record) => {
                    validate_sender_proof_verifier_measurement_recovery_record(&record, &action)
                }
                EntryTypes::SenderProofVerifierMeasuredBootCheckpointRecord(record) => {
                    validate_sender_proof_verifier_measured_boot_checkpoint_record(&record, &action)
                }
                EntryTypes::SenderProofVerifierEndorsementPolicyRecord(record) => {
                    validate_sender_proof_verifier_endorsement_policy_record(&record, &action)
                }
                EntryTypes::SenderProofVerifierManufacturerRootSetRecord(record) => {
                    validate_sender_proof_verifier_manufacturer_root_set_record(&record, &action)
                }
                EntryTypes::SenderProofVerifierEndorsementChainEvidenceRecord(record) => {
                    validate_sender_proof_verifier_endorsement_chain_evidence_record(
                        &record, &action,
                    )
                }
                EntryTypes::SenderProofVerifierEndorsementRevocationSetRecord(record) => {
                    validate_sender_proof_verifier_endorsement_revocation_set_record(
                        &record, &action,
                    )
                }
                EntryTypes::SenderProofVerifierEndorsementCheckpointRecord(record) => {
                    validate_sender_proof_verifier_endorsement_checkpoint_record(&record, &action)
                }
                EntryTypes::SenderProofConflictEvidence(evidence) => {
                    validate_sender_proof_conflict_evidence(&evidence, &action)
                }
                EntryTypes::SenderProofAuthorityForkEvidence(evidence) => {
                    validate_sender_proof_authority_fork_evidence(&evidence, &action)
                }
                EntryTypes::SenderProofConflictQuarantine(quarantine) => {
                    validate_sender_proof_conflict_quarantine(&quarantine, &action)
                }
                EntryTypes::SenderProofActivationPolicyRecord(record) => {
                    validate_sender_proof_activation_policy_record(&record, &action)
                }
                EntryTypes::SenderProofActivationAuditEvent(event) => {
                    validate_sender_proof_activation_audit_event(&event, &action)
                }
                EntryTypes::MailBridgeEvent(_) => Ok(ValidateCallbackResult::Valid),
            },
            OpEntry::UpdateEntry { app_entry, .. } => match app_entry {
                EntryTypes::SenderProofConsumption(_) => Ok(ValidateCallbackResult::Invalid(
                    "sender-proof consumption records are append-only".to_string(),
                )),
                EntryTypes::SenderProofVerifierArtifactRecord(_) => {
                    Ok(ValidateCallbackResult::Invalid(
                        "sender-proof verifier artifact records are append-only".to_string(),
                    ))
                }
                EntryTypes::SenderProofVerifierTransparencyRecord(_) => {
                    Ok(ValidateCallbackResult::Invalid(
                        "sender-proof verifier transparency records are append-only".to_string(),
                    ))
                }
                EntryTypes::SenderProofVerifierCompatibilityRecord(_)
                | EntryTypes::SenderProofVerifierRolloutPlanRecord(_)
                | EntryTypes::SenderProofVerifierRolloutStateRecord(_)
                | EntryTypes::SenderProofVerifierRolloutCheckpointRecord(_)
                | EntryTypes::SenderProofVerifierRolloutHealthRecord(_)
                | EntryTypes::SenderProofVerifierRolloutSafetyActionRecord(_)
                | EntryTypes::SenderProofVerifierTelemetryRetentionRecord(_)
                | EntryTypes::SenderProofVerifierRollbackDrillRecord(_)
                | EntryTypes::SenderProofVerifierReleaseAttestationRecord(_)
                | EntryTypes::SenderProofVerifierCustodyCeremonyRecord(_)
                | EntryTypes::SenderProofVerifierCustodyCompromiseRecord(_)
                | EntryTypes::SenderProofVerifierCustodyRecoveryRecord(_)
                | EntryTypes::SenderProofVerifierCustodyRotationRecord(_)
                | EntryTypes::SenderProofVerifierCustodyRotationCheckpointRecord(_)
                | EntryTypes::SenderProofVerifierDeviceLossNoticeRecord(_)
                | EntryTypes::SenderProofVerifierDeviceSuccessionRecord(_)
                | EntryTypes::SenderProofVerifierDeviceSuccessionCheckpointRecord(_)
                | EntryTypes::SenderProofVerifierDeviceAttestationRecord(_)
                | EntryTypes::SenderProofVerifierDeviceCloneEvidenceRecord(_)
                | EntryTypes::SenderProofVerifierDeviceDecommissionRecord(_)
                | EntryTypes::SenderProofVerifierDeviceAttestationCheckpointRecord(_)
                | EntryTypes::SenderProofVerifierDeviceTrustPolicyRecord(_)
                | EntryTypes::SenderProofVerifierDeviceMeasurementPolicyRecord(_)
                | EntryTypes::SenderProofVerifierDeviceAttestationRootRotationRecord(_)
                | EntryTypes::SenderProofVerifierDeviceTrustBindingRecord(_)
                | EntryTypes::SenderProofVerifierDeviceTrustCheckpointRecord(_)
                | EntryTypes::SenderProofVerifierMeasuredBootPolicyRecord(_)
                | EntryTypes::SenderProofVerifierMeasuredBootEvidenceRecord(_)
                | EntryTypes::SenderProofVerifierDeviceTrustRevocationSetRecord(_)
                | EntryTypes::SenderProofVerifierMeasurementRecoveryRecord(_)
                | EntryTypes::SenderProofVerifierMeasuredBootCheckpointRecord(_)
                | EntryTypes::SenderProofVerifierEndorsementPolicyRecord(_)
                | EntryTypes::SenderProofVerifierManufacturerRootSetRecord(_)
                | EntryTypes::SenderProofVerifierEndorsementChainEvidenceRecord(_)
                | EntryTypes::SenderProofVerifierEndorsementRevocationSetRecord(_)
                | EntryTypes::SenderProofVerifierEndorsementCheckpointRecord(_) => {
                    Ok(ValidateCallbackResult::Invalid(
                        "sender-proof verifier rollout records are append-only".to_string(),
                    ))
                }
                EntryTypes::SenderProofConflictEvidence(_) => Ok(ValidateCallbackResult::Invalid(
                    "sender-proof conflict evidence is append-only".to_string(),
                )),
                EntryTypes::SenderProofAuthorityForkEvidence(_) => {
                    Ok(ValidateCallbackResult::Invalid(
                        "sender-proof authority-fork evidence is append-only".to_string(),
                    ))
                }
                EntryTypes::SenderProofConflictQuarantine(_) => {
                    Ok(ValidateCallbackResult::Invalid(
                        "sender-proof conflict quarantine is append-only".to_string(),
                    ))
                }
                EntryTypes::SenderProofActivationPolicyRecord(_) => {
                    Ok(ValidateCallbackResult::Invalid(
                        "sender-proof activation policy records are append-only".to_string(),
                    ))
                }
                EntryTypes::SenderProofActivationAuditEvent(_) => {
                    Ok(ValidateCallbackResult::Invalid(
                        "sender-proof activation audit events are append-only".to_string(),
                    ))
                }
                _ => Ok(ValidateCallbackResult::Valid),
            },
            // Neither entry type has a real update_entry call anywhere in the coordinator
            // (confirmed via direct grep) -- reject outright rather than leave the previous
            // unbound dead-code path (P0 wide-open RegisterUpdate gap, confirmed 50+ times
            // elsewhere in this pass).
            OpEntry::UpdateEntry { .. } => Ok(ValidateCallbackResult::Invalid(
                "Mail bridge entries cannot be updated".to_string(),
            )),
            _ => Ok(ValidateCallbackResult::Valid),
        },
        // Mail-bridge entries are audit records and are never deleted. This
        // also makes sender-proof consumption durable once observed.
        FlatOp::RegisterDelete(_) => Ok(ValidateCallbackResult::Invalid(
            "mail-bridge audit records cannot be deleted".to_string(),
        )),
        FlatOp::RegisterUpdate(_) => Ok(ValidateCallbackResult::Invalid(
            "Mail bridge entries cannot be updated".to_string(),
        )),
        _ => Ok(ValidateCallbackResult::Valid),
    }
}

fn validate_sender_proof_verifier_compatibility_record(
    record: &SenderProofVerifierCompatibilityRecord,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    const CONTRACT_ID: &str = "mycelix.mail.sender-membership.v1";
    const MIN_CASES: u64 = 128;
    let case_count = record
        .valid_proof_cases
        .checked_add(record.invalid_proof_cases)
        .and_then(|count| count.checked_add(record.boundary_cases));
    if record.contract_id != CONTRACT_ID
        || record.predecessor_verifier_hash == record.candidate_verifier_hash
        || record.valid_proof_cases == 0
        || record.invalid_proof_cases == 0
        || record.boundary_cases == 0
        || case_count.map_or(true, |count| count < MIN_CASES)
        || record.decision_mismatches != 0
        || record.executed_at_micros <= 0
    {
        return Ok(ValidateCallbackResult::Invalid(
            "invalid or non-equivalent verifier compatibility evidence".to_string(),
        ));
    }
    for (name, value) in [
        (
            "predecessor_verifier_hash",
            record.predecessor_verifier_hash,
        ),
        ("candidate_verifier_hash", record.candidate_verifier_hash),
        ("evidence_hash", record.evidence_hash),
        ("corpus_hash", record.corpus_hash),
    ] {
        if value == [0u8; 32] {
            return Ok(ValidateCallbackResult::Invalid(format!("zero {name}")));
        }
    }
    if record.recorded_by != action.author || record.recorded_at != action.timestamp {
        return Ok(ValidateCallbackResult::Invalid(
            "verifier compatibility record provenance must match its action".to_string(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_sender_proof_verifier_rollout_plan_record(
    record: &SenderProofVerifierRolloutPlanRecord,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    const CONTRACT_ID: &str = "mycelix.mail.sender-membership.v1";
    const MAX_WINDOW_MICROS: i64 = 30 * 24 * 60 * 60 * 1_000_000;
    let window = record
        .compatibility_window_end_micros
        .checked_sub(record.compatibility_window_start_micros);
    if record.contract_id != CONTRACT_ID
        || record.predecessor_verifier_hash == record.candidate_verifier_hash
        || record.canary_basis_points == 0
        || record.canary_basis_points >= 10_000
        || record.created_at_micros <= 0
        || record.compatibility_window_start_micros < record.created_at_micros
        || window.map_or(true, |lifetime| {
            lifetime <= 0 || lifetime > MAX_WINDOW_MICROS
        })
    {
        return Ok(ValidateCallbackResult::Invalid(
            "invalid verifier rollout pair, canary allocation, or compatibility window".to_string(),
        ));
    }
    for (name, value) in [
        ("plan_hash", record.plan_hash),
        (
            "predecessor_verifier_hash",
            record.predecessor_verifier_hash,
        ),
        ("candidate_verifier_hash", record.candidate_verifier_hash),
        (
            "compatibility_evidence_hash",
            record.compatibility_evidence_hash,
        ),
        ("cohort_seed", record.cohort_seed),
    ] {
        if value == [0u8; 32] {
            return Ok(ValidateCallbackResult::Invalid(format!("zero {name}")));
        }
    }
    if record.recorded_by != action.author || record.recorded_at != action.timestamp {
        return Ok(ValidateCallbackResult::Invalid(
            "verifier rollout plan provenance must match its action".to_string(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn rollout_stage_known(stage: &str) -> bool {
    matches!(
        stage,
        "disabled" | "shadow" | "canary" | "general" | "frozen" | "rolled_back" | "retired"
    )
}

fn validate_sender_proof_verifier_rollout_state_record(
    record: &SenderProofVerifierRolloutStateRecord,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    const CONTRACT_ID: &str = "mycelix.mail.sender-membership.v1";
    let genesis_shape = record.state_epoch != 1
        || (record.stage == "disabled" && record.previous_state_hash.is_none());
    let successor_shape = record.state_epoch == 1 || record.previous_state_hash.is_some();
    if record.contract_id != CONTRACT_ID
        || record.state_epoch == 0
        || !rollout_stage_known(&record.stage)
        || !genesis_shape
        || !successor_shape
        || record.previous_state_hash == Some([0u8; 32])
        || record.effective_at_micros <= 0
    {
        return Ok(ValidateCallbackResult::Invalid(
            "invalid verifier rollout state shape".to_string(),
        ));
    }
    for (name, value) in [
        ("plan_hash", record.plan_hash),
        ("state_hash", record.state_hash),
        ("reason_hash", record.reason_hash),
    ] {
        if value == [0u8; 32] {
            return Ok(ValidateCallbackResult::Invalid(format!("zero {name}")));
        }
    }
    if record.authenticated_by != action.author || record.recorded_at != action.timestamp {
        return Ok(ValidateCallbackResult::Invalid(
            "verifier rollout state provenance must match its action".to_string(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_sender_proof_verifier_rollout_checkpoint_record(
    record: &SenderProofVerifierRolloutCheckpointRecord,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    const CONTRACT_ID: &str = "mycelix.mail.sender-membership.v1";
    if record.contract_id != CONTRACT_ID
        || record.state_epoch == 0
        || !rollout_stage_known(&record.stage)
        || record.pinned_at_micros <= 0
    {
        return Ok(ValidateCallbackResult::Invalid(
            "invalid verifier rollout checkpoint shape".to_string(),
        ));
    }
    for (name, value) in [
        ("checkpoint_hash", record.checkpoint_hash),
        ("plan_hash", record.plan_hash),
        ("state_hash", record.state_hash),
    ] {
        if value == [0u8; 32] {
            return Ok(ValidateCallbackResult::Invalid(format!("zero {name}")));
        }
    }
    if record.pinned_by != action.author || record.recorded_at != action.timestamp {
        return Ok(ValidateCallbackResult::Invalid(
            "verifier rollout checkpoint provenance must match its action".to_string(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_sender_proof_verifier_rollout_health_record(
    record: &SenderProofVerifierRolloutHealthRecord,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    const CONTRACT_ID: &str = "mycelix.mail.sender-membership.v1";
    const POLICY: &str = "mycelix.proof.verifier-rollout.health-policy.v1";
    const WINDOW: &str = "mycelix.proof.verifier-rollout.telemetry-window.v1";
    const OBSERVERS: &str = "mycelix.proof.verifier-rollout.health-observer-set.v1";
    const ATTESTATION: &str = "mycelix.proof.verifier-rollout.health-attestation.v1";
    const CHECKPOINT: &str = "mycelix.proof.verifier-rollout.health-checkpoint.v1";
    let window = record
        .window_end_micros
        .checked_sub(record.window_start_micros);
    if record.contract_id != CONTRACT_ID
        || record.health_policy_protocol != POLICY
        || record.telemetry_window_protocol != WINDOW
        || record.observer_set_protocol != OBSERVERS
        || record.health_attestation_protocol != ATTESTATION
        || record.health_checkpoint_protocol != CHECKPOINT
        || record.state_epoch == 0
        || record.observer_threshold == 0
        || record.observer_participants < record.observer_threshold
        || record.candidate_executions == 0
        || record.decision_mismatches > record.candidate_executions
        || record.candidate_error_basis_points >= 10_000
        || record.dropped_sample_basis_points >= 10_000
        || record.latency_regression_basis_points >= 10_000
        || window.map_or(true, |duration| duration <= 0)
        || !matches!(record.action.as_str(), "continue" | "freeze" | "rollback")
    {
        return Ok(ValidateCallbackResult::Invalid(
            "invalid verifier rollout health evidence".to_string(),
        ));
    }
    for (name, value) in [
        ("plan_hash", record.plan_hash),
        ("state_hash", record.state_hash),
        ("policy_hash", record.policy_hash),
        ("telemetry_window_hash", record.telemetry_window_hash),
        ("observer_set_hash", record.observer_set_hash),
        ("health_decision_hash", record.health_decision_hash),
        ("observer_quorum_hash", record.observer_quorum_hash),
        ("health_checkpoint_hash", record.health_checkpoint_hash),
    ] {
        if value == [0u8; 32] {
            return Ok(ValidateCallbackResult::Invalid(format!("zero {name}")));
        }
    }
    if (record.action == "continue" && record.decision_mismatches != 0)
        || (record.decision_mismatches > 0 && record.action != "rollback")
    {
        return Ok(ValidateCallbackResult::Invalid(
            "rollout health action contradicts mismatch evidence".to_string(),
        ));
    }
    if record.authenticated_by != action.author || record.recorded_at != action.timestamp {
        return Ok(ValidateCallbackResult::Invalid(
            "verifier rollout health provenance must match its action".to_string(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_sender_proof_verifier_rollout_safety_action_record(
    record: &SenderProofVerifierRolloutSafetyActionRecord,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    const CONTRACT_ID: &str = "mycelix.mail.sender-membership.v1";
    let expected_epoch = record.triggering_state_epoch.checked_add(1);
    if record.contract_id != CONTRACT_ID
        || record.triggering_state_epoch == 0
        || expected_epoch != Some(record.successor_state_epoch)
        || record.applied_at_micros <= 0
        || !matches!(record.required_action.as_str(), "freeze" | "rollback")
    {
        return Ok(ValidateCallbackResult::Invalid(
            "invalid verifier rollout automatic safety action".to_string(),
        ));
    }
    for (name, value) in [
        ("plan_hash", record.plan_hash),
        ("triggering_state_hash", record.triggering_state_hash),
        ("health_decision_hash", record.health_decision_hash),
        ("health_checkpoint_hash", record.health_checkpoint_hash),
        ("successor_state_hash", record.successor_state_hash),
    ] {
        if value == [0u8; 32] {
            return Ok(ValidateCallbackResult::Invalid(format!("zero {name}")));
        }
    }
    if record.applied_by != action.author || record.recorded_at != action.timestamp {
        return Ok(ValidateCallbackResult::Invalid(
            "verifier rollout safety action provenance must match its action".to_string(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_sender_proof_verifier_telemetry_retention_record(
    record: &SenderProofVerifierTelemetryRetentionRecord,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    const CONTRACT_ID: &str = "mycelix.mail.sender-membership.v1";
    const POLICY: &str = "mycelix.proof.verifier-rollout.telemetry-retention-policy.v1";
    const CHECKPOINT: &str = "mycelix.proof.verifier-rollout.telemetry-retention-checkpoint.v1";
    if record.contract_id != CONTRACT_ID
        || record.retention_policy_protocol != POLICY
        || record.retention_checkpoint_protocol != CHECKPOINT
        || record.window_count < 2
        || record.first_state_epoch == 0
        || record.last_state_epoch < record.first_state_epoch
        || record.first_window_start_micros <= 0
        || record.last_window_end_micros <= record.first_window_start_micros
        || !record.retention_satisfied
        || record.previous_checkpoint_hash == Some([0u8; 32])
    {
        return Ok(ValidateCallbackResult::Invalid(
            "invalid verifier telemetry retention checkpoint".to_string(),
        ));
    }
    for (name, value) in [
        ("plan_hash", record.plan_hash),
        ("retention_policy_hash", record.retention_policy_hash),
        (
            "retention_checkpoint_hash",
            record.retention_checkpoint_hash,
        ),
        ("windows_chain_commitment", record.windows_chain_commitment),
    ] {
        if value == [0u8; 32] {
            return Ok(ValidateCallbackResult::Invalid(format!("zero {name}")));
        }
    }
    if record.recorded_by != action.author || record.recorded_at != action.timestamp {
        return Ok(ValidateCallbackResult::Invalid(
            "telemetry retention provenance must match its action".to_string(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_sender_proof_verifier_rollback_drill_record(
    record: &SenderProofVerifierRollbackDrillRecord,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    const CONTRACT_ID: &str = "mycelix.mail.sender-membership.v1";
    const POLICY: &str = "mycelix.proof.verifier-rollout.rollback-drill-policy.v1";
    const REPORT: &str = "mycelix.proof.verifier-rollout.rollback-drill-report.v1";
    if record.contract_id != CONTRACT_ID
        || record.drill_policy_protocol != POLICY
        || record.drill_report_protocol != REPORT
        || record.started_at_micros <= 0
        || record.completed_at_micros <= record.started_at_micros
        || record.candidate_acceptance_events_after_trigger != 0
        || record.successful_recovery_requests < 16
        || record.failed_recovery_requests != 0
        || record.final_stage != "rolled_back"
        || !record.passed
    {
        return Ok(ValidateCallbackResult::Invalid(
            "invalid verifier rollback drill evidence".to_string(),
        ));
    }
    for (name, value) in [
        ("plan_hash", record.plan_hash),
        ("drill_policy_hash", record.drill_policy_hash),
        ("drill_report_hash", record.drill_report_hash),
        (
            "rollout_health_decision_hash",
            record.rollout_health_decision_hash,
        ),
        (
            "rollout_health_checkpoint_hash",
            record.rollout_health_checkpoint_hash,
        ),
    ] {
        if value == [0u8; 32] {
            return Ok(ValidateCallbackResult::Invalid(format!("zero {name}")));
        }
    }
    if record.recorded_by != action.author || record.recorded_at != action.timestamp {
        return Ok(ValidateCallbackResult::Invalid(
            "rollback drill provenance must match its action".to_string(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_sender_proof_verifier_release_attestation_record(
    record: &SenderProofVerifierReleaseAttestationRecord,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    const CONTRACT_ID: &str = "mycelix.mail.sender-membership.v1";
    const AUTHORITY_SET: &str = "mycelix.proof.verifier-release.authority-set.v1";
    const ATTESTATION: &str = "mycelix.proof.verifier-release.attestation.v1";
    const MAX_RELEASE_LIFETIME_MICROS: i64 = 30 * 24 * 60 * 60 * 1_000_000;
    let lifetime = record
        .expires_at_micros
        .checked_sub(record.issued_at_micros);
    let genesis_shape = record.release_sequence != 1 || record.previous_release_hash.is_none();
    let successor_shape = record.release_sequence == 1 || record.previous_release_hash.is_some();
    if record.contract_id != CONTRACT_ID
        || record.release_authority_set_protocol != AUTHORITY_SET
        || record.release_attestation_protocol != ATTESTATION
        || record.authority_set_epoch == 0
        || record.custody_ceremony_epoch == 0
        || record.release_sequence == 0
        || !genesis_shape
        || !successor_shape
        || record.previous_release_hash == Some([0u8; 32])
        || record.authority_threshold == 0
        || record.authority_participants < record.authority_threshold
        || record.issued_at_micros <= 0
        || lifetime.map_or(true, |value| {
            value <= 0 || value > MAX_RELEASE_LIFETIME_MICROS
        })
    {
        return Ok(ValidateCallbackResult::Invalid(
            "invalid verifier release attestation shape".to_string(),
        ));
    }
    for (name, value) in [
        ("plan_hash", record.plan_hash),
        ("rollout_state_hash", record.rollout_state_hash),
        (
            "artifact_transparency_hash",
            record.artifact_transparency_hash,
        ),
        (
            "rollout_health_checkpoint_hash",
            record.rollout_health_checkpoint_hash,
        ),
        (
            "telemetry_retention_checkpoint_hash",
            record.telemetry_retention_checkpoint_hash,
        ),
        (
            "rollback_drill_report_hash",
            record.rollback_drill_report_hash,
        ),
        (
            "activation_checkpoint_hash",
            record.activation_checkpoint_hash,
        ),
        ("custody_policy_hash", record.custody_policy_hash),
        ("custody_ceremony_hash", record.custody_ceremony_hash),
        ("authority_set_hash", record.authority_set_hash),
        ("release_attestation_hash", record.release_attestation_hash),
        ("release_quorum_hash", record.release_quorum_hash),
    ] {
        if value == [0u8; 32] {
            return Ok(ValidateCallbackResult::Invalid(format!("zero {name}")));
        }
    }
    if record.recorded_by != action.author || record.recorded_at != action.timestamp {
        return Ok(ValidateCallbackResult::Invalid(
            "verifier release provenance must match its action".to_string(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn verifier_authority_role_known(role: &str) -> bool {
    matches!(
        role,
        "replay_reservation"
            | "activation_policy"
            | "artifact_transparency"
            | "rollout_health"
            | "release_approval"
            | "custody_recovery"
    )
}

fn validate_sender_proof_verifier_custody_ceremony_record(
    record: &SenderProofVerifierCustodyCeremonyRecord,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    const CONTRACT_ID: &str = "mycelix.mail.sender-membership.v1";
    const POLICY: &str = "mycelix.proof.verifier-key-custody.policy.v1";
    const CEREMONY: &str = "mycelix.proof.verifier-key-custody.ceremony.v1";
    const MAX_LIFETIME_MICROS: i64 = 366 * 24 * 60 * 60 * 1_000_000;
    let lifetime = record
        .expires_at_micros
        .checked_sub(record.created_at_micros);
    let genesis_shape = record.ceremony_epoch != 1 || record.previous_ceremony_hash.is_none();
    let successor_shape = record.ceremony_epoch == 1 || record.previous_ceremony_hash.is_some();
    if record.contract_id != CONTRACT_ID
        || record.custody_policy_protocol != POLICY
        || record.ceremony_record_protocol != CEREMONY
        || !verifier_authority_role_known(&record.authority_role)
        || record.ceremony_epoch == 0
        || record.custodian_threshold < 2
        || record.custodian_participants < record.custodian_threshold
        || !record.role_separation_verified
        || !genesis_shape
        || !successor_shape
        || record.previous_ceremony_hash == Some([0u8; 32])
        || record.created_at_micros <= 0
        || lifetime.map_or(true, |value| value <= 0 || value > MAX_LIFETIME_MICROS)
    {
        return Ok(ValidateCallbackResult::Invalid(
            "invalid verifier custody ceremony evidence".to_string(),
        ));
    }
    for (name, value) in [
        ("custody_policy_hash", record.custody_policy_hash),
        ("ceremony_hash", record.ceremony_hash),
        ("custodian_set_hash", record.custodian_set_hash),
        (
            "hardware_attestation_set_hash",
            record.hardware_attestation_set_hash,
        ),
        ("ceremony_transcript_hash", record.ceremony_transcript_hash),
        ("ceremony_quorum_hash", record.ceremony_quorum_hash),
    ] {
        if value == [0u8; 32] {
            return Ok(ValidateCallbackResult::Invalid(format!("zero {name}")));
        }
    }
    if record.recorded_by != action.author || record.recorded_at != action.timestamp {
        return Ok(ValidateCallbackResult::Invalid(
            "custody ceremony provenance must match its action".to_string(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_sender_proof_verifier_custody_compromise_record(
    record: &SenderProofVerifierCustodyCompromiseRecord,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    const CONTRACT_ID: &str = "mycelix.mail.sender-membership.v1";
    const NOTICE: &str = "mycelix.proof.verifier-key-custody.compromise-notice.v1";
    let genesis_shape = record.notice_sequence != 1 || record.previous_notice_hash.is_none();
    let successor_shape = record.notice_sequence == 1 || record.previous_notice_hash.is_some();
    if record.contract_id != CONTRACT_ID
        || record.compromise_notice_protocol != NOTICE
        || !verifier_authority_role_known(&record.authority_role)
        || record.notice_sequence == 0
        || !genesis_shape
        || !successor_shape
        || record.previous_notice_hash == Some([0u8; 32])
        || record.detected_at_micros <= 0
        || record.effective_at_micros < record.detected_at_micros
    {
        return Ok(ValidateCallbackResult::Invalid(
            "invalid verifier custody compromise notice".to_string(),
        ));
    }
    for (name, value) in [
        ("compromise_notice_hash", record.compromise_notice_hash),
        ("affected_ceremony_hash", record.affected_ceremony_hash),
        ("affected_key_set_hash", record.affected_key_set_hash),
        ("reason_code_hash", record.reason_code_hash),
    ] {
        if value == [0u8; 32] {
            return Ok(ValidateCallbackResult::Invalid(format!("zero {name}")));
        }
    }
    if record.recorded_by != action.author || record.recorded_at != action.timestamp {
        return Ok(ValidateCallbackResult::Invalid(
            "custody compromise provenance must match its action".to_string(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_sender_proof_verifier_custody_recovery_record(
    record: &SenderProofVerifierCustodyRecoveryRecord,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    const CONTRACT_ID: &str = "mycelix.mail.sender-membership.v1";
    const RECOVERY: &str = "mycelix.proof.verifier-key-custody.recovery.v1";
    const MAX_RECOVERY_LIFETIME_MICROS: i64 = 7 * 24 * 60 * 60 * 1_000_000;
    let lifetime = record
        .expires_at_micros
        .checked_sub(record.issued_at_micros);
    if record.contract_id != CONTRACT_ID
        || record.recovery_record_protocol != RECOVERY
        || !verifier_authority_role_known(&record.authority_role)
        || record.recovery_epoch == 0
        || record.previous_ceremony_hash == record.replacement_ceremony_hash
        || record.issued_at_micros <= 0
        || lifetime.map_or(true, |value| {
            value <= 0 || value > MAX_RECOVERY_LIFETIME_MICROS
        })
    {
        return Ok(ValidateCallbackResult::Invalid(
            "invalid verifier custody recovery evidence".to_string(),
        ));
    }
    for (name, value) in [
        ("recovery_record_hash", record.recovery_record_hash),
        ("compromise_notice_hash", record.compromise_notice_hash),
        ("previous_ceremony_hash", record.previous_ceremony_hash),
        (
            "replacement_ceremony_hash",
            record.replacement_ceremony_hash,
        ),
        ("recovery_authority_hash", record.recovery_authority_hash),
    ] {
        if value == [0u8; 32] {
            return Ok(ValidateCallbackResult::Invalid(format!("zero {name}")));
        }
    }
    if record.recorded_by != action.author || record.recorded_at != action.timestamp {
        return Ok(ValidateCallbackResult::Invalid(
            "custody recovery provenance must match its action".to_string(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_sender_proof_verifier_custody_rotation_record(
    record: &SenderProofVerifierCustodyRotationRecord,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    const CONTRACT_ID: &str = "mycelix.mail.sender-membership.v1";
    const POLICY: &str = "mycelix.proof.verifier-key-rotation.policy.v1";
    const ROTATION: &str = "mycelix.proof.verifier-key-rotation.record.v1";
    const MAX_HANDOFF_MICROS: i64 = 24 * 60 * 60 * 1_000_000;
    let genesis_shape = record.rotation_sequence != 1 || record.previous_rotation_hash.is_none();
    let successor_shape = record.rotation_sequence == 1 || record.previous_rotation_hash.is_some();
    let routine_shape = record.rotation_mode != "routine_dual_quorum"
        || (record.retiring_quorum_hash.is_some() && record.recovery_quorum_hash.is_none());
    let recovery_shape = record.rotation_mode != "compromise_recovery"
        || (record.retiring_quorum_hash.is_none() && record.recovery_quorum_hash.is_some());
    let handoff = record
        .retiring_keys_disabled_at_micros
        .checked_sub(record.effective_at_micros);
    if record.contract_id != CONTRACT_ID
        || record.rotation_policy_protocol != POLICY
        || record.rotation_record_protocol != ROTATION
        || !verifier_authority_role_known(&record.authority_role)
        || !matches!(
            record.rotation_mode.as_str(),
            "routine_dual_quorum" | "compromise_recovery"
        )
        || record.rotation_sequence == 0
        || !genesis_shape
        || !successor_shape
        || !routine_shape
        || !recovery_shape
        || record.previous_rotation_hash == Some([0u8; 32])
        || record.retiring_ceremony_epoch == 0
        || record.replacement_ceremony_epoch != record.retiring_ceremony_epoch.saturating_add(1)
        || record.retiring_threshold < 2
        || record.replacement_threshold < record.retiring_threshold
        || !record.threshold_not_reduced
        || !record.retiring_keys_disabled
        || record.initiated_at_micros <= 0
        || record.effective_at_micros < record.initiated_at_micros
        || handoff.map_or(true, |value| value < 0 || value > MAX_HANDOFF_MICROS)
    {
        return Ok(ValidateCallbackResult::Invalid(
            "invalid verifier custody rotation evidence".to_string(),
        ));
    }
    for (name, value) in [
        ("rotation_policy_hash", record.rotation_policy_hash),
        ("rotation_hash", record.rotation_hash),
        ("retiring_ceremony_hash", record.retiring_ceremony_hash),
        (
            "replacement_ceremony_hash",
            record.replacement_ceremony_hash,
        ),
        ("replacement_quorum_hash", record.replacement_quorum_hash),
    ] {
        if value == [0u8; 32] {
            return Ok(ValidateCallbackResult::Invalid(format!("zero {name}")));
        }
    }
    if record.retiring_quorum_hash == Some([0u8; 32])
        || record.recovery_quorum_hash == Some([0u8; 32])
        || record.recorded_by != action.author
        || record.recorded_at != action.timestamp
    {
        return Ok(ValidateCallbackResult::Invalid(
            "rotation quorum or provenance is invalid".to_string(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_sender_proof_verifier_custody_rotation_checkpoint_record(
    record: &SenderProofVerifierCustodyRotationCheckpointRecord,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    const CONTRACT_ID: &str = "mycelix.mail.sender-membership.v1";
    const CHECKPOINT: &str = "mycelix.proof.verifier-key-rotation.checkpoint.v1";
    let genesis_shape = record.rotation_sequence != 1 || record.previous_checkpoint_hash.is_none();
    let successor_shape =
        record.rotation_sequence == 1 || record.previous_checkpoint_hash.is_some();
    if record.contract_id != CONTRACT_ID
        || record.rotation_checkpoint_protocol != CHECKPOINT
        || !verifier_authority_role_known(&record.authority_role)
        || record.rotation_sequence == 0
        || record.active_ceremony_epoch == 0
        || !genesis_shape
        || !successor_shape
        || record.previous_checkpoint_hash == Some([0u8; 32])
        || record.issued_at_micros <= 0
    {
        return Ok(ValidateCallbackResult::Invalid(
            "invalid verifier custody rotation checkpoint".to_string(),
        ));
    }
    for (name, value) in [
        ("checkpoint_hash", record.checkpoint_hash),
        ("rotation_hash", record.rotation_hash),
        ("active_ceremony_hash", record.active_ceremony_hash),
    ] {
        if value == [0u8; 32] {
            return Ok(ValidateCallbackResult::Invalid(format!("zero {name}")));
        }
    }
    if record.recorded_by != action.author || record.recorded_at != action.timestamp {
        return Ok(ValidateCallbackResult::Invalid(
            "rotation checkpoint provenance must match its action".to_string(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_sender_proof_verifier_device_loss_notice_record(
    record: &SenderProofVerifierDeviceLossNoticeRecord,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    const CONTRACT_ID: &str = "mycelix.mail.sender-membership.v1";
    const PROTOCOL: &str = "mycelix.proof.verifier-device-loss.notice.v1";
    const MAX_RECOVERY_MICROS: i64 = 7 * 24 * 60 * 60 * 1_000_000;
    let lifetime = record
        .expires_at_micros
        .checked_sub(record.reported_at_micros);
    if record.contract_id != CONTRACT_ID
        || record.loss_notice_protocol != PROTOCOL
        || !verifier_authority_role_known(&record.authority_role)
        || record.lost_generation == 0
        || !matches!(
            record.reason.as_str(),
            "lost" | "destroyed" | "suspected_compromise"
        )
        || record.reported_at_micros <= 0
        || lifetime.map_or(true, |value| value <= 0 || value > MAX_RECOVERY_MICROS)
        || [
            record.custody_slot_id,
            record.loss_notice_hash,
            record.lost_device_id,
            record.evidence_hash,
        ]
        .iter()
        .any(|value| *value == [0u8; 32])
        || record.recorded_by != action.author
        || record.recorded_at != action.timestamp
    {
        return Ok(ValidateCallbackResult::Invalid(
            "invalid verifier device loss notice".to_string(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_sender_proof_verifier_device_succession_record(
    record: &SenderProofVerifierDeviceSuccessionRecord,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    const CONTRACT_ID: &str = "mycelix.mail.sender-membership.v1";
    const POLICY: &str = "mycelix.proof.verifier-device-succession.policy.v1";
    const RECORD: &str = "mycelix.proof.verifier-device-succession.record.v1";
    const MAX_HANDOFF_MICROS: i64 = 24 * 60 * 60 * 1_000_000;
    let genesis_shape =
        record.succession_sequence != 1 || record.previous_succession_hash.is_none();
    let successor_shape =
        record.succession_sequence == 1 || record.previous_succession_hash.is_some();
    let planned_shape = record.succession_mode != "planned_dual_device"
        || (record.retiring_device_approval_hash.is_some()
            && record.recovery_quorum_hash.is_none()
            && record.loss_notice_hash.is_none());
    let lost_shape = record.succession_mode != "lost_device_recovery"
        || (record.retiring_device_approval_hash.is_none()
            && record.recovery_quorum_hash.is_some()
            && record.loss_notice_hash.is_some());
    let handoff = record
        .retiring_device_disabled_at_micros
        .checked_sub(record.effective_at_micros);
    if record.contract_id != CONTRACT_ID
        || record.succession_policy_protocol != POLICY
        || record.succession_record_protocol != RECORD
        || !verifier_authority_role_known(&record.authority_role)
        || !matches!(
            record.succession_mode.as_str(),
            "planned_dual_device" | "lost_device_recovery"
        )
        || record.succession_sequence == 0
        || !genesis_shape
        || !successor_shape
        || !planned_shape
        || !lost_shape
        || record.previous_succession_hash == Some([0u8; 32])
        || record.retiring_generation == 0
        || record.replacement_generation != record.retiring_generation.saturating_add(1)
        || record.retiring_device_id == record.replacement_device_id
        || !record.retiring_device_disabled
        || record.initiated_at_micros <= 0
        || record.effective_at_micros < record.initiated_at_micros
        || handoff.map_or(true, |value| value < 0 || value > MAX_HANDOFF_MICROS)
        || [
            record.custody_slot_id,
            record.policy_hash,
            record.succession_hash,
            record.retiring_device_id,
            record.replacement_device_id,
            record.replacement_device_approval_hash,
        ]
        .iter()
        .any(|value| *value == [0u8; 32])
        || record.retiring_device_approval_hash == Some([0u8; 32])
        || record.recovery_quorum_hash == Some([0u8; 32])
        || record.loss_notice_hash == Some([0u8; 32])
        || record.recorded_by != action.author
        || record.recorded_at != action.timestamp
    {
        return Ok(ValidateCallbackResult::Invalid(
            "invalid verifier device succession evidence".to_string(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_sender_proof_verifier_device_succession_checkpoint_record(
    record: &SenderProofVerifierDeviceSuccessionCheckpointRecord,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    const CONTRACT_ID: &str = "mycelix.mail.sender-membership.v1";
    const CHECKPOINT: &str = "mycelix.proof.verifier-device-succession.checkpoint.v1";
    let genesis_shape =
        record.succession_sequence != 1 || record.previous_checkpoint_hash.is_none();
    let successor_shape =
        record.succession_sequence == 1 || record.previous_checkpoint_hash.is_some();
    if record.contract_id != CONTRACT_ID
        || record.succession_checkpoint_protocol != CHECKPOINT
        || !verifier_authority_role_known(&record.authority_role)
        || record.succession_sequence == 0
        || record.active_device_generation == 0
        || !genesis_shape
        || !successor_shape
        || record.previous_checkpoint_hash == Some([0u8; 32])
        || record.issued_at_micros <= 0
        || [
            record.custody_slot_id,
            record.checkpoint_hash,
            record.succession_hash,
            record.active_device_id,
        ]
        .iter()
        .any(|value| *value == [0u8; 32])
        || record.recorded_by != action.author
        || record.recorded_at != action.timestamp
    {
        return Ok(ValidateCallbackResult::Invalid(
            "invalid verifier device succession checkpoint".to_string(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_sender_proof_verifier_device_attestation_record(
    record: &SenderProofVerifierDeviceAttestationRecord,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    const CONTRACT_ID: &str = "mycelix.mail.sender-membership.v1";
    const POLICY: &str = "mycelix.proof.verifier-device-attestation.policy.v1";
    const RECORD: &str = "mycelix.proof.verifier-device-attestation.record.v1";
    const MAX_AGE_MICROS: i64 = 60 * 60 * 1_000_000;
    let lifetime = record
        .expires_at_micros
        .checked_sub(record.issued_at_micros);
    let genesis_shape =
        record.attestation_sequence != 1 || record.previous_attestation_hash.is_none();
    let successor_shape =
        record.attestation_sequence == 1 || record.previous_attestation_hash.is_some();
    if record.contract_id != CONTRACT_ID
        || record.attestation_policy_protocol != POLICY
        || record.attestation_record_protocol != RECORD
        || !verifier_authority_role_known(&record.authority_role)
        || record.device_generation == 0
        || record.attestation_sequence == 0
        || record.monotonic_boot_counter == 0
        || !record.secure_boot_enabled
        || !record.debug_disabled
        || !genesis_shape
        || !successor_shape
        || record.previous_attestation_hash == Some([0u8; 32])
        || record.issued_at_micros <= 0
        || lifetime.map_or(true, |value| value <= 0 || value > MAX_AGE_MICROS)
        || [
            record.custody_slot_id,
            record.device_id,
            record.succession_checkpoint_hash,
            record.policy_hash,
            record.attestation_hash,
            record.platform_measurement_hash,
            record.firmware_measurement_hash,
            record.endorsement_key_hash,
            record.challenge_nonce_hash,
            record.evidence_hash,
        ]
        .iter()
        .any(|value| *value == [0u8; 32])
        || record.recorded_by != action.author
        || record.recorded_at != action.timestamp
    {
        return Ok(ValidateCallbackResult::Invalid(
            "invalid verifier device attestation evidence".to_string(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_sender_proof_verifier_device_clone_evidence_record(
    record: &SenderProofVerifierDeviceCloneEvidenceRecord,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    const CONTRACT_ID: &str = "mycelix.mail.sender-membership.v1";
    const PROTOCOL: &str = "mycelix.proof.verifier-device-clone-evidence.v1";
    if record.contract_id != CONTRACT_ID
        || record.clone_evidence_protocol != PROTOCOL
        || record.attestation_sequence == 0
        || record.first_attestation_hash >= record.second_attestation_hash
        || record.observed_at_micros <= 0
        || [
            record.device_id,
            record.endorsement_key_hash,
            record.first_attestation_hash,
            record.second_attestation_hash,
        ]
        .iter()
        .any(|value| *value == [0u8; 32])
        || record.recorded_by != action.author
        || record.recorded_at != action.timestamp
    {
        return Ok(ValidateCallbackResult::Invalid(
            "invalid verifier device clone evidence".to_string(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_sender_proof_verifier_device_decommission_record(
    record: &SenderProofVerifierDeviceDecommissionRecord,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    const CONTRACT_ID: &str = "mycelix.mail.sender-membership.v1";
    const PROTOCOL: &str = "mycelix.proof.verifier-device-decommission.record.v1";
    if record.contract_id != CONTRACT_ID
        || record.decommission_record_protocol != PROTOCOL
        || !verifier_authority_role_known(&record.authority_role)
        || record.device_generation == 0
        || !record.key_zeroized
        || !record.credentials_revoked
        || record.decommissioned_at_micros <= 0
        || [
            record.custody_slot_id,
            record.device_id,
            record.final_attestation_hash,
            record.succession_checkpoint_hash,
            record.decommission_hash,
            record.destruction_evidence_hash,
        ]
        .iter()
        .any(|value| *value == [0u8; 32])
        || record.recorded_by != action.author
        || record.recorded_at != action.timestamp
    {
        return Ok(ValidateCallbackResult::Invalid(
            "invalid verifier device decommission evidence".to_string(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_sender_proof_verifier_device_attestation_checkpoint_record(
    record: &SenderProofVerifierDeviceAttestationCheckpointRecord,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    const CONTRACT_ID: &str = "mycelix.mail.sender-membership.v1";
    const PROTOCOL: &str = "mycelix.proof.verifier-device-attestation.checkpoint.v1";
    if record.contract_id != CONTRACT_ID
        || record.attestation_checkpoint_protocol != PROTOCOL
        || !verifier_authority_role_known(&record.authority_role)
        || record.active_device_generation == 0
        || record.attestation_sequence == 0
        || record.monotonic_boot_counter == 0
        || record.clone_evidence_hash.is_some()
        || record.previous_checkpoint_hash == Some([0u8; 32])
        || record.latest_decommission_hash == Some([0u8; 32])
        || record.issued_at_micros <= 0
        || [
            record.custody_slot_id,
            record.active_device_id,
            record.checkpoint_hash,
            record.attestation_hash,
        ]
        .iter()
        .any(|value| *value == [0u8; 32])
        || record.recorded_by != action.author
        || record.recorded_at != action.timestamp
    {
        return Ok(ValidateCallbackResult::Invalid(
            "invalid verifier device attestation checkpoint".to_string(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_sender_proof_verifier_device_trust_policy_record(
    record: &SenderProofVerifierDeviceTrustPolicyRecord,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    const CONTRACT_ID: &str = "mycelix.mail.sender-membership.v1";
    const PROTOCOL: &str = "mycelix.proof.verifier-device-trust.policy.v1";
    const MAX_HANDOFF_MICROS: i64 = 7 * 24 * 60 * 60 * 1_000_000;
    if record.contract_id != CONTRACT_ID
        || record.trust_policy_protocol != PROTOCOL
        || record.trust_policy_hash == [0u8; 32]
        || !record.require_dual_root_approval
        || !record.require_measurement_allowlist
        || !record.require_monotonic_policy_epochs
        || !record.require_external_checkpoint
        || record.maximum_root_handoff_micros <= 0
        || record.maximum_root_handoff_micros > MAX_HANDOFF_MICROS
        || record.recorded_by != action.author
        || record.recorded_at != action.timestamp
    {
        return Ok(ValidateCallbackResult::Invalid(
            "invalid verifier device trust policy".to_string(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_sender_proof_verifier_device_measurement_policy_record(
    record: &SenderProofVerifierDeviceMeasurementPolicyRecord,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    const CONTRACT_ID: &str = "mycelix.mail.sender-membership.v1";
    const PROTOCOL: &str = "mycelix.proof.verifier-device-measurement.policy.v1";
    const MAX_POLICY_MICROS: i64 = 366 * 24 * 60 * 60 * 1_000_000;
    let lifetime = record
        .expires_at_micros
        .checked_sub(record.issued_at_micros);
    let genesis_shape = record.policy_epoch != 1 || record.previous_policy_hash.is_none();
    let successor_shape = record.policy_epoch == 1 || record.previous_policy_hash.is_some();
    if record.contract_id != CONTRACT_ID
        || record.measurement_policy_protocol != PROTOCOL
        || record.policy_epoch == 0
        || !genesis_shape
        || !successor_shape
        || record.previous_policy_hash == Some([0u8; 32])
        || record.approved_measurement_count == 0
        || record.approved_measurement_count > 128
        || record.policy_hash == [0u8; 32]
        || record.approved_measurement_set_hash == [0u8; 32]
        || record.issued_at_micros <= 0
        || lifetime.map_or(true, |value| value <= 0 || value > MAX_POLICY_MICROS)
        || record.recorded_by != action.author
        || record.recorded_at != action.timestamp
    {
        return Ok(ValidateCallbackResult::Invalid(
            "invalid verifier device measurement policy".to_string(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_sender_proof_verifier_device_attestation_root_rotation_record(
    record: &SenderProofVerifierDeviceAttestationRootRotationRecord,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    const CONTRACT_ID: &str = "mycelix.mail.sender-membership.v1";
    const PROTOCOL: &str = "mycelix.proof.verifier-device-attestation-root-rotation.v1";
    const MAX_HANDOFF_MICROS: i64 = 7 * 24 * 60 * 60 * 1_000_000;
    let handoff = record
        .current_root_retire_at_micros
        .checked_sub(record.effective_at_micros);
    let genesis_shape = record.rotation_sequence != 1 || record.previous_rotation_hash.is_none();
    let successor_shape = record.rotation_sequence == 1 || record.previous_rotation_hash.is_some();
    if record.contract_id != CONTRACT_ID
        || record.root_rotation_protocol != PROTOCOL
        || record.rotation_sequence == 0
        || !genesis_shape
        || !successor_shape
        || record.previous_rotation_hash == Some([0u8; 32])
        || record.current_root_hash == record.replacement_root_hash
        || record.issued_at_micros <= 0
        || record.effective_at_micros < record.issued_at_micros
        || handoff.map_or(true, |value| value < 0 || value > MAX_HANDOFF_MICROS)
        || [
            record.rotation_hash,
            record.current_root_hash,
            record.replacement_root_hash,
            record.current_root_approval_hash,
            record.replacement_root_approval_hash,
        ]
        .iter()
        .any(|value| *value == [0u8; 32])
        || record.recorded_by != action.author
        || record.recorded_at != action.timestamp
    {
        return Ok(ValidateCallbackResult::Invalid(
            "invalid verifier device attestation-root rotation".to_string(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_sender_proof_verifier_device_trust_binding_record(
    record: &SenderProofVerifierDeviceTrustBindingRecord,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    const CONTRACT_ID: &str = "mycelix.mail.sender-membership.v1";
    const PROTOCOL: &str = "mycelix.proof.verifier-device-trust-binding.v1";
    if record.contract_id != CONTRACT_ID
        || record.trust_binding_protocol != PROTOCOL
        || record.measurement_policy_epoch == 0
        || record.security_epoch == 0
        || record.issued_at_micros <= 0
        || record.expires_at_micros <= record.issued_at_micros
        || [
            record.binding_hash,
            record.attestation_hash,
            record.trust_policy_hash,
            record.attestation_root_hash,
            record.measurement_policy_hash,
            record.endorsement_certificate_hash,
        ]
        .iter()
        .any(|value| *value == [0u8; 32])
        || record.recorded_by != action.author
        || record.recorded_at != action.timestamp
    {
        return Ok(ValidateCallbackResult::Invalid(
            "invalid verifier device trust binding".to_string(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_sender_proof_verifier_device_trust_checkpoint_record(
    record: &SenderProofVerifierDeviceTrustCheckpointRecord,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    const CONTRACT_ID: &str = "mycelix.mail.sender-membership.v1";
    const PROTOCOL: &str = "mycelix.proof.verifier-device-trust-checkpoint.v1";
    let genesis_shape =
        record.checkpoint_sequence != 1 || record.previous_checkpoint_hash.is_none();
    let successor_shape =
        record.checkpoint_sequence == 1 || record.previous_checkpoint_hash.is_some();
    if record.contract_id != CONTRACT_ID
        || record.trust_checkpoint_protocol != PROTOCOL
        || record.checkpoint_sequence == 0
        || record.measurement_policy_epoch == 0
        || !genesis_shape
        || !successor_shape
        || record.previous_checkpoint_hash == Some([0u8; 32])
        || record.issued_at_micros <= 0
        || [
            record.checkpoint_hash,
            record.trust_policy_hash,
            record.active_root_hash,
            record.measurement_policy_hash,
            record.trust_binding_hash,
        ]
        .iter()
        .any(|value| *value == [0u8; 32])
        || record.recorded_by != action.author
        || record.recorded_at != action.timestamp
    {
        return Ok(ValidateCallbackResult::Invalid(
            "invalid verifier device trust checkpoint".to_string(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_sender_proof_verifier_measured_boot_policy_record(
    record: &SenderProofVerifierMeasuredBootPolicyRecord,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    const CONTRACT_ID: &str = "mycelix.mail.sender-membership.v1";
    const PROTOCOL: &str = "mycelix.proof.verifier-measured-boot.policy.v1";
    const MAX_AGE_MICROS: i64 = 60 * 60 * 1_000_000;
    if record.contract_id != CONTRACT_ID
        || record.measured_boot_policy_protocol != PROTOCOL
        || record.policy_hash == [0u8; 32]
        || record.required_pcr_set_hash == [0u8; 32]
        || record.required_pcr_count == 0
        || record.required_pcr_count > 24
        || record.minimum_event_count == 0
        || record.maximum_event_count < record.minimum_event_count
        || record.maximum_event_count > 4096
        || record.maximum_evidence_age_micros <= 0
        || record.maximum_evidence_age_micros > MAX_AGE_MICROS
        || !record.require_complete_log
        || !record.require_revocation_check
        || !record.require_external_checkpoint
        || record.recorded_by != action.author
        || record.recorded_at != action.timestamp
    {
        return Ok(ValidateCallbackResult::Invalid(
            "invalid verifier measured-boot policy".to_string(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_sender_proof_verifier_measured_boot_evidence_record(
    record: &SenderProofVerifierMeasuredBootEvidenceRecord,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    const CONTRACT_ID: &str = "mycelix.mail.sender-membership.v1";
    const PROTOCOL: &str = "mycelix.proof.verifier-measured-boot.evidence.v1";
    const MAX_LIFETIME_MICROS: i64 = 15 * 60 * 1_000_000;
    let lifetime = record
        .expires_at_micros
        .checked_sub(record.issued_at_micros);
    if record.contract_id != CONTRACT_ID
        || record.measured_boot_evidence_protocol != PROTOCOL
        || record.boot_counter == 0
        || record.evidence_sequence == 0
        || record.event_count == 0
        || record.event_count > 4096
        || record.issued_at_micros <= 0
        || lifetime.map_or(true, |value| value <= 0 || value > MAX_LIFETIME_MICROS)
        || [
            record.evidence_hash,
            record.policy_hash,
            record.device_id,
            record.device_attestation_hash,
            record.attestation_root_hash,
            record.platform_measurement_hash,
            record.firmware_measurement_hash,
            record.event_log_hash,
            record.final_pcr_set_hash,
        ]
        .iter()
        .any(|value| *value == [0u8; 32])
        || record.recorded_by != action.author
        || record.recorded_at != action.timestamp
    {
        return Ok(ValidateCallbackResult::Invalid(
            "invalid verifier measured-boot evidence".to_string(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_sender_proof_verifier_device_trust_revocation_set_record(
    record: &SenderProofVerifierDeviceTrustRevocationSetRecord,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    const CONTRACT_ID: &str = "mycelix.mail.sender-membership.v1";
    const PROTOCOL: &str = "mycelix.proof.verifier-device-trust-revocations.v1";
    let genesis_shape =
        record.revocation_epoch != 1 || record.previous_revocation_set_hash.is_none();
    let successor_shape =
        record.revocation_epoch == 1 || record.previous_revocation_set_hash.is_some();
    if record.contract_id != CONTRACT_ID
        || record.revocation_set_protocol != PROTOCOL
        || record.revocation_epoch == 0
        || !genesis_shape
        || !successor_shape
        || record.previous_revocation_set_hash == Some([0u8; 32])
        || record.revocation_entry_count > 1024
        || record.revocation_set_hash == [0u8; 32]
        || record.revocation_entry_set_hash == [0u8; 32]
        || record.issued_at_micros <= 0
        || record.recorded_by != action.author
        || record.recorded_at != action.timestamp
    {
        return Ok(ValidateCallbackResult::Invalid(
            "invalid verifier device trust revocation set".to_string(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_sender_proof_verifier_measurement_recovery_record(
    record: &SenderProofVerifierMeasurementRecoveryRecord,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    const CONTRACT_ID: &str = "mycelix.mail.sender-membership.v1";
    const PROTOCOL: &str = "mycelix.proof.verifier-measurement-recovery.v1";
    if record.contract_id != CONTRACT_ID
        || record.measurement_recovery_protocol != PROTOCOL
        || record.replacement_policy_epoch == 0
        || record.revocation_epoch == 0
        || record.revoked_measurement_policy_hash == record.replacement_measurement_policy_hash
        || record.authorized_at_micros <= 0
        || record.effective_at_micros < record.authorized_at_micros
        || [
            record.recovery_hash,
            record.incident_hash,
            record.revoked_measurement_policy_hash,
            record.replacement_measurement_policy_hash,
            record.revocation_set_hash,
        ]
        .iter()
        .any(|value| *value == [0u8; 32])
        || record.recorded_by != action.author
        || record.recorded_at != action.timestamp
    {
        return Ok(ValidateCallbackResult::Invalid(
            "invalid verifier measurement recovery record".to_string(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_sender_proof_verifier_measured_boot_checkpoint_record(
    record: &SenderProofVerifierMeasuredBootCheckpointRecord,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    const CONTRACT_ID: &str = "mycelix.mail.sender-membership.v1";
    const PROTOCOL: &str = "mycelix.proof.verifier-measured-boot.checkpoint.v1";
    let genesis_shape =
        record.checkpoint_sequence != 1 || record.previous_checkpoint_hash.is_none();
    let successor_shape =
        record.checkpoint_sequence == 1 || record.previous_checkpoint_hash.is_some();
    if record.contract_id != CONTRACT_ID
        || record.measured_boot_checkpoint_protocol != PROTOCOL
        || record.checkpoint_sequence == 0
        || record.measured_boot_evidence_sequence == 0
        || record.revocation_epoch == 0
        || !genesis_shape
        || !successor_shape
        || record.previous_checkpoint_hash == Some([0u8; 32])
        || record.recovery_record_hash == Some([0u8; 32])
        || record.issued_at_micros <= 0
        || [
            record.checkpoint_hash,
            record.measured_boot_evidence_hash,
            record.revocation_set_hash,
        ]
        .iter()
        .any(|value| *value == [0u8; 32])
        || record.recorded_by != action.author
        || record.recorded_at != action.timestamp
    {
        return Ok(ValidateCallbackResult::Invalid(
            "invalid verifier measured-boot checkpoint".to_string(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_sender_proof_verifier_endorsement_policy_record(
    record: &SenderProofVerifierEndorsementPolicyRecord,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    const CONTRACT_ID: &str = "mycelix.mail.sender-membership.v1";
    const PROTOCOL: &str = "mycelix.proof.verifier-endorsement.policy.v1";
    const MAX_LIFETIME_MICROS: i64 = 10 * 366 * 24 * 60 * 60 * 1_000_000;
    if record.contract_id != CONTRACT_ID
        || record.endorsement_policy_protocol != PROTOCOL
        || record.policy_hash == [0u8; 32]
        || record.maximum_chain_depth == 0
        || record.maximum_chain_depth > 8
        || record.minimum_certificate_security_epoch == 0
        || record.minimum_firmware_security_version == 0
        || record.maximum_credential_lifetime_micros <= 0
        || record.maximum_credential_lifetime_micros > MAX_LIFETIME_MICROS
        || !record.require_manufacturer_root_set
        || !record.require_revocation_check
        || !record.require_firmware_anti_rollback
        || !record.require_external_checkpoint
        || record.recorded_by != action.author
        || record.recorded_at != action.timestamp
    {
        return Ok(ValidateCallbackResult::Invalid(
            "invalid verifier endorsement policy".to_string(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_sender_proof_verifier_manufacturer_root_set_record(
    record: &SenderProofVerifierManufacturerRootSetRecord,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    const CONTRACT_ID: &str = "mycelix.mail.sender-membership.v1";
    const PROTOCOL: &str = "mycelix.proof.verifier-endorsement.manufacturer-root-set.v1";
    let genesis_shape = record.root_set_epoch != 1 || record.previous_root_set_hash.is_none();
    let successor_shape = record.root_set_epoch == 1 || record.previous_root_set_hash.is_some();
    if record.contract_id != CONTRACT_ID
        || record.manufacturer_root_set_protocol != PROTOCOL
        || record.root_set_epoch == 0
        || !genesis_shape
        || !successor_shape
        || record.previous_root_set_hash == Some([0u8; 32])
        || record.active_root_count == 0
        || record.active_root_count > 64
        || record.issued_at_micros <= 0
        || record.expires_at_micros <= record.issued_at_micros
        || [record.root_set_hash, record.active_root_set_hash]
            .iter()
            .any(|value| *value == [0u8; 32])
        || record.recorded_by != action.author
        || record.recorded_at != action.timestamp
    {
        return Ok(ValidateCallbackResult::Invalid(
            "invalid verifier manufacturer root set".to_string(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_sender_proof_verifier_endorsement_chain_evidence_record(
    record: &SenderProofVerifierEndorsementChainEvidenceRecord,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    const CONTRACT_ID: &str = "mycelix.mail.sender-membership.v1";
    const PROTOCOL: &str = "mycelix.proof.verifier-endorsement.chain-evidence.v1";
    let genesis_shape = record.evidence_sequence != 1 || record.previous_evidence_hash.is_none();
    let successor_shape = record.evidence_sequence == 1 || record.previous_evidence_hash.is_some();
    if record.contract_id != CONTRACT_ID
        || record.endorsement_chain_evidence_protocol != PROTOCOL
        || record.evidence_sequence == 0
        || !genesis_shape
        || !successor_shape
        || record.previous_evidence_hash == Some([0u8; 32])
        || record.manufacturer_root_set_epoch == 0
        || record.certificate_count == 0
        || record.certificate_count > 8
        || record.certificate_security_epoch == 0
        || record.firmware_security_version == 0
        || record.endorsement_revocation_epoch == 0
        || record.verified_at_micros <= 0
        || [
            record.evidence_hash,
            record.endorsement_policy_hash,
            record.manufacturer_root_set_hash,
            record.active_manufacturer_root_hash,
            record.leaf_certificate_hash,
            record.certificate_chain_hash,
            record.endorsement_key_hash,
            record.endorsement_revocation_set_hash,
        ]
        .iter()
        .any(|value| *value == [0u8; 32])
        || record.recorded_by != action.author
        || record.recorded_at != action.timestamp
    {
        return Ok(ValidateCallbackResult::Invalid(
            "invalid verifier endorsement-chain evidence".to_string(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_sender_proof_verifier_endorsement_revocation_set_record(
    record: &SenderProofVerifierEndorsementRevocationSetRecord,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    const CONTRACT_ID: &str = "mycelix.mail.sender-membership.v1";
    const PROTOCOL: &str = "mycelix.proof.verifier-endorsement.revocations.v1";
    let genesis_shape =
        record.revocation_epoch != 1 || record.previous_revocation_set_hash.is_none();
    let successor_shape =
        record.revocation_epoch == 1 || record.previous_revocation_set_hash.is_some();
    if record.contract_id != CONTRACT_ID
        || record.endorsement_revocation_set_protocol != PROTOCOL
        || record.revocation_epoch == 0
        || !genesis_shape
        || !successor_shape
        || record.previous_revocation_set_hash == Some([0u8; 32])
        || record.revoked_item_count > 2048
        || record.issued_at_micros <= 0
        || [
            record.revocation_set_hash,
            record.revoked_root_set_hash,
            record.revoked_certificate_set_hash,
            record.revoked_key_set_hash,
        ]
        .iter()
        .any(|value| *value == [0u8; 32])
        || record.recorded_by != action.author
        || record.recorded_at != action.timestamp
    {
        return Ok(ValidateCallbackResult::Invalid(
            "invalid verifier endorsement revocation set".to_string(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_sender_proof_verifier_endorsement_checkpoint_record(
    record: &SenderProofVerifierEndorsementCheckpointRecord,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    const CONTRACT_ID: &str = "mycelix.mail.sender-membership.v1";
    const PROTOCOL: &str = "mycelix.proof.verifier-endorsement.checkpoint.v1";
    let genesis_shape =
        record.checkpoint_sequence != 1 || record.previous_checkpoint_hash.is_none();
    let successor_shape =
        record.checkpoint_sequence == 1 || record.previous_checkpoint_hash.is_some();
    if record.contract_id != CONTRACT_ID
        || record.endorsement_checkpoint_protocol != PROTOCOL
        || record.checkpoint_sequence == 0
        || !genesis_shape
        || !successor_shape
        || record.previous_checkpoint_hash == Some([0u8; 32])
        || record.manufacturer_root_set_epoch == 0
        || record.endorsement_evidence_sequence == 0
        || record.endorsement_revocation_epoch == 0
        || record.firmware_security_version == 0
        || record.issued_at_micros <= 0
        || [
            record.checkpoint_hash,
            record.endorsement_policy_hash,
            record.manufacturer_root_set_hash,
            record.active_manufacturer_root_hash,
            record.endorsement_evidence_hash,
            record.endorsement_revocation_set_hash,
        ]
        .iter()
        .any(|value| *value == [0u8; 32])
        || record.recorded_by != action.author
        || record.recorded_at != action.timestamp
    {
        return Ok(ValidateCallbackResult::Invalid(
            "invalid verifier endorsement checkpoint".to_string(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_sender_proof_verifier_artifact_record(
    record: &SenderProofVerifierArtifactRecord,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    const CONTRACT_ID: &str = "mycelix.mail.sender-membership.v1";
    const MAX_ARTIFACT_BYTES: u64 = 64 * 1024 * 1024;
    if record.contract_id != CONTRACT_ID
        || record.circuit_id.trim().is_empty()
        || record.backend.trim().is_empty()
        || record.build_target.is_empty()
        || record.build_target.len() > 128
        || record.build_profile.is_empty()
        || record.build_profile.len() > 64
        || !record
            .build_target
            .bytes()
            .all(|byte| byte.is_ascii_graphic())
        || !record
            .build_profile
            .bytes()
            .all(|byte| byte.is_ascii_graphic())
        || record.artifact_size_bytes == 0
        || record.artifact_size_bytes > MAX_ARTIFACT_BYTES
        || record.independent_rebuild_count < 2
    {
        return Ok(ValidateCallbackResult::Invalid(
            "invalid sender-proof verifier artifact identity or build contract".to_string(),
        ));
    }
    for (name, value) in [
        ("artifact_hash", record.artifact_hash),
        ("manifest_hash", record.manifest_hash),
        ("source_tree_hash", record.source_tree_hash),
        ("dependency_lock_hash", record.dependency_lock_hash),
        ("toolchain_hash", record.toolchain_hash),
        ("build_recipe_hash", record.build_recipe_hash),
        ("rebuild_quorum_hash", record.rebuild_quorum_hash),
    ] {
        if value == [0u8; 32] {
            return Ok(ValidateCallbackResult::Invalid(format!("zero {name}")));
        }
    }
    if record.recorded_by != action.author || record.recorded_at != action.timestamp {
        return Ok(ValidateCallbackResult::Invalid(
            "verifier artifact record author and timestamp must match action provenance"
                .to_string(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_sender_proof_verifier_transparency_record(
    record: &SenderProofVerifierTransparencyRecord,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    const CONTRACT_ID: &str = "mycelix.mail.sender-membership.v1";
    if record.contract_id != CONTRACT_ID
        || record.circuit_id.trim().is_empty()
        || record.sequence == 0
        || record.authority_set_epoch == 0
        || record.published_at_micros <= 0
        || record.previous_record_hash == Some([0u8; 32])
        || record.successor_manifest_hash == Some([0u8; 32])
        || record.reason_hash == Some([0u8; 32])
    {
        return Ok(ValidateCallbackResult::Invalid(
            "invalid verifier transparency identity, sequence, or optional hash".to_string(),
        ));
    }
    for (name, value) in [
        ("manifest_hash", record.manifest_hash),
        ("artifact_hash", record.artifact_hash),
        ("record_hash", record.record_hash),
        ("authority_set_hash", record.authority_set_hash),
        ("rebuild_quorum_hash", record.rebuild_quorum_hash),
    ] {
        if value == [0u8; 32] {
            return Ok(ValidateCallbackResult::Invalid(format!("zero {name}")));
        }
    }
    let lifecycle_valid = match record.status.as_str() {
        "published" => record.successor_manifest_hash.is_none() && record.reason_hash.is_none(),
        "superseded" => record.successor_manifest_hash.is_some() && record.reason_hash.is_none(),
        "revoked" => record.successor_manifest_hash.is_none() && record.reason_hash.is_some(),
        _ => false,
    };
    if !lifecycle_valid {
        return Ok(ValidateCallbackResult::Invalid(
            "invalid verifier transparency lifecycle fields".to_string(),
        ));
    }
    if record.recorded_by != action.author || record.recorded_at != action.timestamp {
        return Ok(ValidateCallbackResult::Invalid(
            "verifier transparency record author and timestamp must match action provenance"
                .to_string(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_sender_proof_activation_policy_record(
    record: &SenderProofActivationPolicyRecord,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    const CONTRACT_ID: &str = "mycelix.mail.sender-membership.v1";
    const POLICY_ID: &str = "mycelix.mail.sender-proof-activation.operator-quorum.v2";
    const RECORD_PROTOCOL: &str = "mycelix.mail.sender-proof-activation.policy-record.v2";
    const MAX_LIFETIME_MICROS: i64 = 24 * 60 * 60 * 1_000_000;
    if record.contract_id != CONTRACT_ID
        || record.policy_id != POLICY_ID
        || record.record_protocol != RECORD_PROTOCOL
    {
        return Ok(ValidateCallbackResult::Invalid(
            "unsupported sender-proof activation policy record".to_string(),
        ));
    }
    for (name, value) in [
        ("record_hash", record.record_hash),
        ("operator_set_hash", record.operator_set_hash),
        ("operator_quorum_hash", record.operator_quorum_hash),
        ("chain_commitment", record.chain_commitment),
        ("checkpoint_hash", record.checkpoint_hash),
    ] {
        if value == [0u8; 32] {
            return Ok(ValidateCallbackResult::Invalid(format!("zero {name}")));
        }
    }
    if record.previous_record_hash == Some([0u8; 32])
        || record.halt_recovery_ticket_hash == Some([0u8; 32])
        || record.operator_set_epoch == 0
        || record.operator_threshold == 0
        || record.operator_participants < record.operator_threshold
    {
        return Ok(ValidateCallbackResult::Invalid(
            "invalid activation predecessor, operator quorum, or recovery hash".to_string(),
        ));
    }
    if record.issued_at_micros <= 0
        || record.expires_at_micros <= record.issued_at_micros
        || record.expires_at_micros - record.issued_at_micros > MAX_LIFETIME_MICROS
    {
        return Ok(ValidateCallbackResult::Invalid(
            "activation policy record lifetime is invalid or exceeds 24 hours".to_string(),
        ));
    }
    let mode_known = matches!(
        record.requested_mode.as_str(),
        "disabled" | "audit_only" | "enforced"
    );
    let transition_valid = match record.transition_kind.as_str() {
        "bootstrap" => {
            record.activation_epoch == 0
                && record.requested_mode == "disabled"
                && !record.emergency_halt
                && record.previous_record_hash.is_none()
                && record.halt_recovery_ticket_hash.is_none()
        }
        "configure" => {
            record.previous_record_hash.is_some() && record.halt_recovery_ticket_hash.is_none()
        }
        "engage_emergency_halt" => {
            record.previous_record_hash.is_some()
                && record.emergency_halt
                && record.halt_recovery_ticket_hash.is_none()
        }
        "release_emergency_halt" => {
            record.previous_record_hash.is_some()
                && !record.emergency_halt
                && record.requested_mode != "enforced"
                && record.halt_recovery_ticket_hash.is_some()
        }
        _ => false,
    };
    if !mode_known || !transition_valid {
        return Ok(ValidateCallbackResult::Invalid(
            "invalid activation mode or transition semantics".to_string(),
        ));
    }
    if record.recorded_by != action.author || record.recorded_at != action.timestamp {
        return Ok(ValidateCallbackResult::Invalid(
            "activation policy record author and timestamp must match action provenance"
                .to_string(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_sender_proof_activation_audit_event(
    event: &SenderProofActivationAuditEvent,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    const AUDIT_PROTOCOL: &str = "mycelix.mail.sender-proof-activation.audit-event.v1";
    if event.audit_event_protocol != AUDIT_PROTOCOL {
        return Ok(ValidateCallbackResult::Invalid(
            "unsupported sender-proof activation audit protocol".to_string(),
        ));
    }
    for (name, value) in [
        ("policy_record_hash", event.policy_record_hash),
        ("operator_set_hash", event.operator_set_hash),
        ("chain_commitment", event.chain_commitment),
        ("decision_hash", event.decision_hash),
    ] {
        if value == [0u8; 32] {
            return Ok(ValidateCallbackResult::Invalid(format!("zero {name}")));
        }
    }
    if !matches!(
        event.requested_mode.as_str(),
        "disabled" | "audit_only" | "enforced"
    ) || !matches!(
        event.effective_mode.as_str(),
        "disabled" | "audit_only" | "enforced"
    ) || !matches!(
        event.transition_kind.as_str(),
        "bootstrap" | "configure" | "engage_emergency_halt" | "release_emergency_halt"
    ) {
        return Ok(ValidateCallbackResult::Invalid(
            "activation audit event contains an unknown mode or transition".to_string(),
        ));
    }
    if event.acceptance_enabled
        && (event.effective_mode != "enforced"
            || !event.promotion_ready
            || !event.provenance_ready
            || event.emergency_halt
            || !event.blocker_codes.is_empty())
    {
        return Ok(ValidateCallbackResult::Invalid(
            "activation audit event enables acceptance without the complete conjunction"
                .to_string(),
        ));
    }
    if event
        .blocker_codes
        .iter()
        .any(|code| code.trim().is_empty())
        || event
            .blocker_codes
            .windows(2)
            .any(|pair| pair[0] >= pair[1])
    {
        return Ok(ValidateCallbackResult::Invalid(
            "activation audit blocker codes must be nonempty, unique, and canonical".to_string(),
        ));
    }
    if event.observed_by != action.author || event.observed_at != action.timestamp {
        return Ok(ValidateCallbackResult::Invalid(
            "activation audit author and timestamp must match action provenance".to_string(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_sender_proof_conflict_evidence(
    evidence: &SenderProofConflictEvidence,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    for (name, value) in [
        ("first_receipt_hash", evidence.first_receipt_hash),
        ("second_receipt_hash", evidence.second_receipt_hash),
        ("first_acceptance_id", evidence.first_acceptance_id),
        ("second_acceptance_id", evidence.second_acceptance_id),
        (
            "first_reservation_slot_id",
            evidence.first_reservation_slot_id,
        ),
        (
            "second_reservation_slot_id",
            evidence.second_reservation_slot_id,
        ),
        (
            "first_authority_set_hash",
            evidence.first_authority_set_hash,
        ),
        (
            "second_authority_set_hash",
            evidence.second_authority_set_hash,
        ),
    ] {
        if value == [0u8; 32] {
            return Ok(ValidateCallbackResult::Invalid(format!("zero {name}")));
        }
    }
    if evidence.first_receipt_hash >= evidence.second_receipt_hash {
        return Ok(ValidateCallbackResult::Invalid(
            "conflict evidence receipt hashes must be distinct and canonical".to_string(),
        ));
    }
    if evidence.first_acceptance_id == evidence.second_acceptance_id {
        return Ok(ValidateCallbackResult::Invalid(
            "conflict evidence must identify two distinct authenticated grants".to_string(),
        ));
    }
    let shared_envelope = evidence.shared_envelope_replay_key;
    let shared_nullifier = evidence.shared_nullifier_replay_key;
    if shared_envelope.is_none() && shared_nullifier.is_none() {
        return Ok(ValidateCallbackResult::Invalid(
            "conflict evidence must bind at least one shared replay key".to_string(),
        ));
    }
    if shared_envelope == Some([0u8; 32]) || shared_nullifier == Some([0u8; 32]) {
        return Ok(ValidateCallbackResult::Invalid(
            "conflict evidence contains a zero shared replay key".to_string(),
        ));
    }
    if evidence.first_authority_set_epoch == 0 || evidence.second_authority_set_epoch == 0 {
        return Ok(ValidateCallbackResult::Invalid(
            "conflict evidence authority epochs must be nonzero".to_string(),
        ));
    }
    if evidence.detected_by != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "conflict detector must match action author".to_string(),
        ));
    }
    if evidence.detected_at != action.timestamp {
        return Ok(ValidateCallbackResult::Invalid(
            "conflict detection timestamp must match action timestamp".to_string(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_sender_proof_authority_fork_evidence(
    evidence: &SenderProofAuthorityForkEvidence,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    for (name, value) in [
        (
            "previous_authority_set_hash",
            evidence.previous_authority_set_hash,
        ),
        ("first_transition_hash", evidence.first_transition_hash),
        ("second_transition_hash", evidence.second_transition_hash),
        (
            "first_next_authority_set_hash",
            evidence.first_next_authority_set_hash,
        ),
        (
            "second_next_authority_set_hash",
            evidence.second_next_authority_set_hash,
        ),
    ] {
        if value == [0u8; 32] {
            return Ok(ValidateCallbackResult::Invalid(format!("zero {name}")));
        }
    }
    if evidence.previous_authority_set_epoch == 0
        || evidence.next_authority_set_epoch != evidence.previous_authority_set_epoch + 1
    {
        return Ok(ValidateCallbackResult::Invalid(
            "authority-fork evidence epochs are not contiguous".to_string(),
        ));
    }
    if evidence.first_transition_hash >= evidence.second_transition_hash
        || evidence.first_next_authority_set_hash == evidence.second_next_authority_set_hash
    {
        return Ok(ValidateCallbackResult::Invalid(
            "authority-fork evidence must be canonical and bind distinct successors".to_string(),
        ));
    }
    if evidence.effective_at_micros <= 0 {
        return Ok(ValidateCallbackResult::Invalid(
            "authority-fork effective time must be positive".to_string(),
        ));
    }
    if evidence.detected_by != action.author || evidence.detected_at != action.timestamp {
        return Ok(ValidateCallbackResult::Invalid(
            "authority-fork detector must match action provenance".to_string(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_sender_proof_conflict_quarantine(
    quarantine: &SenderProofConflictQuarantine,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    for (name, value) in [
        ("conflict_evidence_hash", quarantine.conflict_evidence_hash),
        ("adjudication_id", quarantine.adjudication_id),
        ("first_acceptance_id", quarantine.first_acceptance_id),
        ("second_acceptance_id", quarantine.second_acceptance_id),
    ] {
        if value == [0u8; 32] {
            return Ok(ValidateCallbackResult::Invalid(format!("zero {name}")));
        }
    }
    if quarantine.first_acceptance_id >= quarantine.second_acceptance_id {
        return Ok(ValidateCallbackResult::Invalid(
            "conflict quarantine acceptance IDs must be distinct and canonical".to_string(),
        ));
    }
    if quarantine.disposition != "reject_both_v1" {
        return Ok(ValidateCallbackResult::Invalid(
            "unsupported sender-proof conflict disposition".to_string(),
        ));
    }
    if quarantine.quarantined_by != action.author || quarantine.quarantined_at != action.timestamp {
        return Ok(ValidateCallbackResult::Invalid(
            "conflict quarantine author and timestamp must match action provenance".to_string(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_sender_proof_consumption(
    consumption: &SenderProofConsumption,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    const CONTRACT_ID: &str = "mycelix.mail.sender-membership.v1";

    if consumption.contract_id != CONTRACT_ID {
        return Ok(ValidateCallbackResult::Invalid(
            "unsupported sender-proof contract".to_string(),
        ));
    }
    if consumption.circuit_id.trim().is_empty() || consumption.backend.trim().is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "exact verifier identity is incomplete".to_string(),
        ));
    }
    for (name, value) in [
        ("verifier_hash", consumption.verifier_hash),
        ("public_inputs_hash", consumption.public_inputs_hash),
        ("proof_hash", consumption.proof_hash),
        ("message_id_hash", consumption.message_id_hash),
        ("group_root", consumption.group_root),
        ("envelope_replay_key", consumption.envelope_replay_key),
        ("nullifier_replay_key", consumption.nullifier_replay_key),
    ] {
        if value == [0u8; 32] {
            return Ok(ValidateCallbackResult::Invalid(format!("zero {name}")));
        }
    }
    if consumption.envelope_replay_key == consumption.nullifier_replay_key {
        return Ok(ValidateCallbackResult::Invalid(
            "replay keys must use distinct namespaces".to_string(),
        ));
    }
    if consumption.epoch == 0 {
        return Ok(ValidateCallbackResult::Invalid("zero epoch".to_string()));
    }
    if consumption.verifier != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "verifier must match action author".to_string(),
        ));
    }
    if consumption.verified_at != action.timestamp {
        return Ok(ValidateCallbackResult::Invalid(
            "verified_at must match action timestamp".to_string(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}
