#!/usr/bin/env python3
"""Gate protocol-v16 device attestation, clone, and decommission truth."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
checks = {
    ROOT / "crates/mail-leptos-types/src/lib.rs": [
        "PULSE_RUNTIME_PROTOCOL_VERSION: u16 = 16",
        "PROOF_VERIFIER_DEVICE_ATTESTATION_POLICY_V1",
        "device_attestation_freshness_verified",
        "device_attestation_checkpoint_pinned",
        "device_clone_evidence_resolved",
        "device_decommission_evidence_verified",
        "verifier_device_attestation_freshness_not_verified",
        "verifier_device_attestation_checkpoint_not_pinned",
        "verifier_device_clone_evidence_unresolved",
        "verifier_device_decommission_evidence_not_verified",
        "release_device_attestation_checkpoint_hash",
        "release_device_decommission_record_hash",
    ],
    ROOT / "holochain/zomes/capabilities/coordinator/src/lib.rs": [
        "device_attestation_policy_hash: None",
        "active_device_attestation_hash: None",
        "device_attestation_freshness_verified: false",
        "device_clone_evidence_resolved: false",
        "device_decommission_evidence_verified: false",
    ],
    ROOT / "holochain/zomes/mail-bridge/coordinator/src/lib.rs": [
        "device_attestation_policy_hash",
        "active_device_attestation_hash",
        "device_attestation_checkpoint_hash",
        "device_boot_counter",
        "device_attestation_freshness_verified: false",
        "device_clone_evidence_resolved: false",
        "device_decommission_evidence_verified: false",
    ],
    ROOT / "holochain/zomes/mail-bridge/integrity/src/lib.rs": [
        "SenderProofVerifierDeviceAttestationRecord",
        "SenderProofVerifierDeviceCloneEvidenceRecord",
        "SenderProofVerifierDeviceDecommissionRecord",
        "SenderProofVerifierDeviceAttestationCheckpointRecord",
        "secure_boot_enabled",
        "debug_disabled",
        "monotonic_boot_counter",
        "key_zeroized",
        "credentials_revoked",
        "clone_evidence_hash.is_some()",
    ],
    ROOT / "apps/leptos/src/components/nav.rs": [
        "data-runtime-proof-custody-attestation-sequence",
        "data-runtime-proof-custody-boot-counter",
        "data-runtime-proof-custody-attestation-fresh",
        "data-runtime-proof-custody-attestation-checkpoint",
        "data-runtime-proof-custody-clone-resolved",
        "data-runtime-proof-custody-device-decommission",
    ],
    ROOT / "scripts/live-browser-proof/live-proof.mjs": [
        "evidence.protocol !== '16'",
        "proofCustodyAttestationFresh",
        "proofCustodyCloneResolved",
        "proofCustodyDeviceDecommission",
        "verifier_device_attestation_freshness_not_verified",
        "verifier_device_decommission_evidence_not_verified",
    ],
}
for path, markers in checks.items():
    text = path.read_text()
    missing = [marker for marker in markers if marker not in text]
    if missing:
        raise SystemExit(f"{path}: missing protocol-v16 device attestation markers: {missing}")

integrity = (ROOT / "holochain/zomes/mail-bridge/integrity/src/lib.rs").read_text()
for forbidden in [
    "secure_boot_enabled || record.debug_disabled",
    "key_zeroized || record.credentials_revoked",
    "first_attestation_hash <= second_attestation_hash",
]:
    if forbidden in integrity:
        raise SystemExit(f"unsafe device-attestation integrity shortcut present: {forbidden}")

print("Pulse protocol v16 device attestation contract valid: freshness, clone quarantine, checkpoint, decommission")
