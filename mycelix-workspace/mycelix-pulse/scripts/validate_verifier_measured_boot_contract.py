#!/usr/bin/env python3
"""Gate protocol-v18 measured-boot and revocation truth."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TYPES = ROOT / "crates/mail-leptos-types/src/lib.rs"
CAPS = ROOT / "holochain/zomes/capabilities/coordinator/src/lib.rs"
INTEGRITY = ROOT / "holochain/zomes/mail-bridge/integrity/src/lib.rs"
BRIDGE = ROOT / "holochain/zomes/mail-bridge/coordinator/src/lib.rs"
NAV = ROOT / "apps/leptos/src/components/nav.rs"
PROOF = ROOT / "scripts/live-browser-proof/live-proof.mjs"

checks = {
    TYPES: [
        "PULSE_RUNTIME_PROTOCOL_VERSION: u16 = 18",
        "PROOF_VERIFIER_MEASURED_BOOT_POLICY_V1",
        "PROOF_VERIFIER_MEASURED_BOOT_EVIDENCE_V1",
        "PROOF_VERIFIER_DEVICE_TRUST_REVOCATION_SET_V1",
        "PROOF_VERIFIER_MEASUREMENT_RECOVERY_RECORD_V1",
        "PROOF_VERIFIER_MEASURED_BOOT_CHECKPOINT_V1",
        "measured_boot_event_log_verified",
        "device_trust_revocation_state_verified",
        "measurement_recovery_state_verified",
        "measured_boot_checkpoint_pinned",
        "verifier_measured_boot_event_log_not_verified",
        "release_measured_boot_checkpoint_hash",
    ],
    CAPS: [
        "measured_boot_policy_protocol: PROOF_VERIFIER_MEASURED_BOOT_POLICY_V1.into()",
        "measured_boot_event_log_verified: false",
        "device_trust_revocation_state_verified: false",
        "measurement_recovery_state_verified: false",
        "measured_boot_checkpoint_pinned: false",
        "release_measured_boot_checkpoint_hash: None",
    ],
    BRIDGE: [
        "measured_boot_policy_hash: Option<[u8; 32]>",
        "measured_boot_evidence_hash: Option<[u8; 32]>",
        "device_trust_revocation_set_hash: Option<[u8; 32]>",
        "measurement_recovery_state_hash: Option<[u8; 32]>",
        "measured_boot_checkpoint_hash: Option<[u8; 32]>",
        "measured_boot_event_log_verified: false",
        "measured_boot_checkpoint_pinned: false",
    ],
    INTEGRITY: [
        "SenderProofVerifierMeasuredBootPolicyRecord",
        "SenderProofVerifierMeasuredBootEvidenceRecord",
        "SenderProofVerifierDeviceTrustRevocationSetRecord",
        "SenderProofVerifierMeasurementRecoveryRecord",
        "SenderProofVerifierMeasuredBootCheckpointRecord",
        "require_complete_log",
        "event_log_hash",
        "revocation_entry_set_hash",
        "revoked_measurement_policy_hash == record.replacement_measurement_policy_hash",
    ],
    NAV: [
        "data-runtime-proof-custody-measured-boot-policy",
        "data-runtime-proof-custody-measured-boot-verified",
        "data-runtime-proof-custody-revocations-verified",
        "data-runtime-proof-custody-measurement-recovery",
        "data-runtime-proof-custody-measured-boot-checkpoint",
    ],
    PROOF: [
        "evidence.protocol !== '18'",
        "proofCustodyMeasuredBootVerified",
        "proofCustodyRevocationsVerified",
        "proofCustodyMeasurementRecovery",
        "proofCustodyMeasuredBootCheckpoint",
        "verifier_measured_boot_policy_not_pinned",
        "verifier_measured_boot_event_log_not_verified",
        "verifier_device_trust_revocation_state_not_verified",
        "verifier_measurement_recovery_state_not_verified",
        "verifier_measured_boot_checkpoint_not_pinned",
    ],
}
for path, markers in checks.items():
    text = path.read_text()
    missing = [marker for marker in markers if marker not in text]
    if missing:
        raise SystemExit(f"{path}: missing measured-boot markers: {missing}")

for path in [TYPES, CAPS, PROOF]:
    text = path.read_text()
    if "PULSE_RUNTIME_PROTOCOL_VERSION: u16 = 17" in text or "evidence.protocol !== '17'" in text:
        raise SystemExit(f"{path}: stale protocol-v17 measured-boot gate remains")

print("Pulse verifier measured-boot contract valid: protocol v18 remains fail-closed without event log, revocations, recovery, or checkpoint")
