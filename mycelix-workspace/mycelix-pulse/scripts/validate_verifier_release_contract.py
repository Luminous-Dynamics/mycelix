#!/usr/bin/env python3
"""Fail-closed source contract for Pulse verifier release evidence protocol v12."""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
FILES = {
    "types": ROOT / "crates/mail-leptos-types/src/lib.rs",
    "caps": ROOT / "holochain/zomes/capabilities/coordinator/src/lib.rs",
    "integrity": ROOT / "holochain/zomes/mail-bridge/integrity/src/lib.rs",
    "bridge": ROOT / "holochain/zomes/mail-bridge/coordinator/src/lib.rs",
    "nav": ROOT / "apps/leptos/src/components/nav.rs",
    "proof": ROOT / "scripts/live-browser-proof/live-proof.mjs",
}

required = {
    "types": [
        "PULSE_RUNTIME_PROTOCOL_VERSION: u16 = 12",
        "pub struct ProofVerifierReleaseManifest",
        "pub proof_verifier_releases: Vec<ProofVerifierReleaseManifest>",
        "verifier_rollout_telemetry_retention_not_verified",
        "verifier_rollout_rollback_drill_not_verified",
        "verifier_release_quorum_not_verified",
        "verifier_release_attestation_not_current",
        "release_ready == computed.is_empty()",
        "release_device_attestation_policy_hash",
        "release_device_attestation_checkpoint_hash",
        "release_device_attestation_sequence",
        "release_device_trust_policy_hash",
        "release_device_measurement_policy_hash",
        "release_device_measurement_policy_epoch",
        "release_device_attestation_root_hash",
        "release_device_attestation_root_rotation_sequence",
        "release_device_trust_checkpoint_hash",
        "release_measured_boot_policy_hash",
        "release_measured_boot_evidence_hash",
        "release_measured_boot_evidence_sequence",
        "release_device_trust_revocation_set_hash",
        "release_device_trust_revocation_epoch",
        "release_measurement_recovery_state_hash",
        "release_measured_boot_checkpoint_hash",
        "release_device_decommission_record_hash",
    ],
    "caps": [
        "let proof_verifier_releases",
        "retention_satisfied: false",
        "rollback_drill_passed: false",
        "release_quorum_verified: false",
        "release_current: false",
        "release_ready: false",
        "release_device_attestation_policy_hash: None",
        "release_device_attestation_checkpoint_hash: None",
        "release_measured_boot_policy_hash: None",
        "release_measured_boot_evidence_hash: None",
        "release_device_trust_revocation_set_hash: None",
        "release_measurement_recovery_state_hash: None",
        "release_measured_boot_checkpoint_hash: None",
        "release_device_decommission_record_hash: None",
    ],
    "integrity": [
        "pub struct SenderProofVerifierTelemetryRetentionRecord",
        "pub struct SenderProofVerifierRollbackDrillRecord",
        "pub struct SenderProofVerifierReleaseAttestationRecord",
        "validate_sender_proof_verifier_telemetry_retention_record",
        "validate_sender_proof_verifier_rollback_drill_record",
        "validate_sender_proof_verifier_release_attestation_record",
        "sender-proof verifier rollout records are append-only",
    ],
    "bridge": [
        "telemetry_retention_policy_hash",
        "telemetry_retention_checkpoint_hash",
        "rollback_drill_policy_hash",
        "rollback_drill_report_hash",
        "release_authority_set_hash",
        "release_attestation_hash",
        "release_authorized: false",
    ],
    "nav": [
        "data-runtime-proof-release-count",
        "data-runtime-proof-retention-satisfied",
        "data-runtime-proof-rollback-drill",
        "data-runtime-proof-release-quorum",
        "data-runtime-proof-release-current",
        "data-runtime-proof-release-blockers",
    ],
    "proof": [
        "evidence.protocol !== '12'",
        "proofRetentionSatisfied",
        "proofRollbackDrillPassed",
        "proofReleaseQuorumVerified",
        "proofReleaseCurrent",
        "verifier_release_attestation_not_current",
    ],
}

errors = []
for name, path in FILES.items():
    if not path.exists():
        errors.append(f"missing {path.relative_to(ROOT)}")
        continue
    text = path.read_text()
    for token in required[name]:
        if token not in text:
            errors.append(f"{path.relative_to(ROOT)} missing {token!r}")

caps = FILES["caps"].read_text()
for forbidden in [
    "retention_satisfied: true",
    "rollback_drill_passed: true",
    "release_quorum_verified: true",
    "release_current: true",
    "release_ready: true",
]:
    if forbidden in caps:
        errors.append(f"capability manifest prematurely enables release evidence: {forbidden}")

if errors:
    print("verifier release contract invalid:", file=sys.stderr)
    for error in errors:
        print(f"- {error}", file=sys.stderr)
    raise SystemExit(1)

print(
    "verifier release contract valid: protocol v12 exposes unavailable retained telemetry, "
    "rollback-drill, and release-quorum evidence; exact challenges bind their hashes; "
    "append-only records exist; acceptance remains disabled"
)
