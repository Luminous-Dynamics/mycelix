#!/usr/bin/env python3
"""Gate protocol-v15 cross-device verifier custody truth."""
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
        "PULSE_RUNTIME_PROTOCOL_VERSION: u16 = 15",
        "PROOF_VERIFIER_DEVICE_SUCCESSION_POLICY_V1",
        "device_succession_policy_protocol",
        "device_succession_sequence",
        "active_device_generation",
        "planned_dual_device_verified",
        "lost_device_recovery_quorum_verified",
        "retiring_device_disabled",
        "device_succession_checkpoint_pinned",
        "verifier_device_succession_continuity_verified",
        "release_device_succession_checkpoint_hash",
    ],
    CAPS: [
        "device_succession_policy_hash: None",
        "active_device_succession_hash: None",
        "device_succession_sequence: None",
        "active_device_generation: None",
        "device_succession_continuity_verified: false",
        "verifier_device_succession_checkpoint_pinned: false",
    ],
    INTEGRITY: [
        "SenderProofVerifierDeviceLossNoticeRecord",
        "SenderProofVerifierDeviceSuccessionRecord",
        "SenderProofVerifierDeviceSuccessionCheckpointRecord",
        "planned_dual_device",
        "lost_device_recovery",
        "retiring_device_disabled",
    ],
    BRIDGE: [
        "device_succession_policy_hash",
        "device_succession_sequence",
        "active_device_generation",
        "device_succession_checkpoint_hash",
        "device_succession_continuity_verified: false",
        "custody_authorized: false",
    ],
    NAV: [
        "data-runtime-proof-custody-device-sequence",
        "data-runtime-proof-custody-device-generation",
        "data-runtime-proof-custody-device-continuity",
        "data-runtime-proof-custody-device-checkpoint",
        "data-runtime-proof-custody-retiring-device-disabled",
    ],
    PROOF: [
        "proofCustodyDeviceSequence",
        "proofCustodyDeviceContinuity",
        "verifier_device_succession_policy_not_pinned",
        "verifier_device_succession_continuity_not_verified",
        "verifier_device_succession_checkpoint_not_pinned",
    ],
}
for path, markers in checks.items():
    text = path.read_text()
    missing = [marker for marker in markers if marker not in text]
    if missing:
        raise SystemExit(f"{path}: missing device-succession markers: {missing}")

truth = TYPES.read_text()
if "self.custody_ready == computed.is_empty()" not in truth:
    raise SystemExit("custody readiness is not derived from complete device-succession blockers")
if "replacement_generation >=" in INTEGRITY.read_text():
    raise SystemExit("device generation may not advance by an arbitrary jump")

print("verifier device succession contract valid: protocol v15 exposes absent device chain, dual consent/recovery, checkpoint, and disabled acceptance")
