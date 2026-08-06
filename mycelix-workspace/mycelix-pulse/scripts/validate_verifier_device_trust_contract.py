#!/usr/bin/env python3
"""Gate protocol-v17 device trust-root and measurement-policy truth."""
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
        "PULSE_RUNTIME_PROTOCOL_VERSION: u16 = 17",
        "PROOF_VERIFIER_DEVICE_TRUST_POLICY_V1",
        "PROOF_VERIFIER_DEVICE_MEASUREMENT_POLICY_V1",
        "PROOF_VERIFIER_DEVICE_ATTESTATION_ROOT_ROTATION_V1",
        "PROOF_VERIFIER_DEVICE_TRUST_BINDING_V1",
        "PROOF_VERIFIER_DEVICE_TRUST_CHECKPOINT_V1",
        "device_trust_policy_hash",
        "device_measurement_policy_verified",
        "device_attestation_root_rotation_continuity_verified",
        "device_trust_checkpoint_pinned",
        "verifier_device_trust_policy_not_pinned",
        "verifier_device_measurement_policy_not_verified",
        "verifier_device_attestation_root_rotation_not_verified",
        "verifier_device_trust_checkpoint_not_pinned",
        "release_device_trust_checkpoint_hash",
    ],
    CAPS: [
        "device_trust_policy_protocol: PROOF_VERIFIER_DEVICE_TRUST_POLICY_V1.into()",
        "device_measurement_policy_verified: false",
        "device_attestation_root_rotation_continuity_verified: false",
        "device_trust_checkpoint_pinned: false",
        "release_device_trust_checkpoint_hash: None",
    ],
    BRIDGE: [
        "device_trust_policy_hash: Option<[u8; 32]>",
        "device_measurement_policy_hash: Option<[u8; 32]>",
        "device_measurement_policy_epoch: Option<u64>",
        "active_device_attestation_root_hash: Option<[u8; 32]>",
        "device_attestation_root_rotation_sequence: Option<u64>",
        "device_measurement_policy_verified: false",
        "device_attestation_root_rotation_continuity_verified: false",
        "device_trust_checkpoint_hash: Option<[u8; 32]>",
        "device_trust_checkpoint_pinned: false",
    ],
    INTEGRITY: [
        "SenderProofVerifierDeviceTrustPolicyRecord",
        "SenderProofVerifierDeviceMeasurementPolicyRecord",
        "SenderProofVerifierDeviceAttestationRootRotationRecord",
        "SenderProofVerifierDeviceTrustBindingRecord",
        "SenderProofVerifierDeviceTrustCheckpointRecord",
        "require_dual_root_approval",
        "approved_measurement_count",
        "current_root_approval_hash",
        "replacement_root_approval_hash",
        "measurement_policy_epoch",
        "trust_binding_hash",
    ],
    NAV: [
        "data-runtime-proof-custody-trust-policy",
        "data-runtime-proof-custody-measurement-verified",
        "data-runtime-proof-custody-root-continuity",
        "data-runtime-proof-custody-trust-checkpoint",
    ],
    PROOF: [
        "evidence.protocol !== '17'",
        "proofCustodyMeasurementVerified",
        "proofCustodyRootContinuity",
        "verifier_device_trust_policy_not_pinned",
        "verifier_device_measurement_policy_not_verified",
        "verifier_device_attestation_root_rotation_not_verified",
        "verifier_device_trust_checkpoint_not_pinned",
    ],
}
for path, markers in checks.items():
    text = path.read_text()
    missing = [marker for marker in markers if marker not in text]
    if missing:
        raise SystemExit(f"{path}: missing device-trust markers: {missing}")

for path in [TYPES, CAPS, PROOF]:
    if "PULSE_RUNTIME_PROTOCOL_VERSION: u16 = 16" in path.read_text():
        raise SystemExit(f"{path}: stale protocol-v16 gate remains")

print("Pulse verifier device trust contract valid: protocol v17 remains fail-closed without roots, policy, rotation continuity, or checkpoint")
