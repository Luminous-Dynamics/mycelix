#!/usr/bin/env python3
"""Gate protocol-v19 endorsement-chain and manufacturer-root truth."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TYPES = ROOT / "crates/mail-leptos-types/src/lib.rs"
CAPS = ROOT / "holochain/zomes/capabilities/coordinator/src/lib.rs"
INTEGRITY = ROOT / "holochain/zomes/mail-bridge/integrity/src/lib.rs"
COORD = ROOT / "holochain/zomes/mail-bridge/coordinator/src/lib.rs"
NAV = ROOT / "apps/leptos/src/components/nav.rs"
BROWSER = ROOT / "scripts/live-browser-proof/live-proof.mjs"

checks = {
    TYPES: [
        "PULSE_RUNTIME_PROTOCOL_VERSION: u16 = 19",
        "PROOF_VERIFIER_ENDORSEMENT_POLICY_V1",
        "PROOF_VERIFIER_MANUFACTURER_ROOT_SET_V1",
        "PROOF_VERIFIER_ENDORSEMENT_CHAIN_EVIDENCE_V1",
        "PROOF_VERIFIER_ENDORSEMENT_REVOCATION_SET_V1",
        "PROOF_VERIFIER_ENDORSEMENT_CHECKPOINT_V1",
        "endorsement_chain_verified",
        "firmware_anti_rollback_verified",
        "verifier_endorsement_policy_not_pinned",
        "verifier_manufacturer_root_set_not_pinned",
        "verifier_endorsement_chain_not_verified",
        "verifier_endorsement_revocation_state_not_verified",
        "verifier_endorsement_checkpoint_not_pinned",
        "release_endorsement_policy_hash",
        "release_endorsement_checkpoint_hash",
    ],
    CAPS: [
        "endorsement_policy_protocol: PROOF_VERIFIER_ENDORSEMENT_POLICY_V1",
        "manufacturer_root_set_protocol: PROOF_VERIFIER_MANUFACTURER_ROOT_SET_V1",
        "endorsement_chain_evidence_protocol: PROOF_VERIFIER_ENDORSEMENT_CHAIN_EVIDENCE_V1",
        "endorsement_policy_hash: None",
        "endorsement_chain_verified: false",
        "firmware_anti_rollback_verified: false",
    ],
    INTEGRITY: [
        "SenderProofVerifierEndorsementPolicyRecord",
        "SenderProofVerifierManufacturerRootSetRecord",
        "SenderProofVerifierEndorsementChainEvidenceRecord",
        "SenderProofVerifierEndorsementRevocationSetRecord",
        "SenderProofVerifierEndorsementCheckpointRecord",
        "sender-proof verifier rollout records are append-only",
    ],
    COORD: [
        "endorsement_policy_hash: Option<[u8; 32]>",
        "manufacturer_root_set_epoch: Option<u64>",
        "endorsement_chain_verified: bool",
        "firmware_anti_rollback_verified: bool",
        "endorsement_checkpoint_pinned: bool",
    ],
    NAV: [
        "data-runtime-proof-custody-endorsement-policy",
        "data-runtime-proof-custody-manufacturer-root-epoch",
        "data-runtime-proof-custody-endorsement-verified",
        "data-runtime-proof-custody-firmware-anti-rollback",
        "data-runtime-proof-custody-endorsement-checkpoint",
    ],
    BROWSER: [
        "evidence.protocol !== '19'",
        "proofCustodyEndorsementPolicy",
        "proofCustodyManufacturerRootEpoch",
        "proofCustodyEndorsementVerified",
        "proofCustodyFirmwareAntiRollback",
        "verifier_endorsement_checkpoint_not_pinned",
    ],
}

for path, markers in checks.items():
    text = path.read_text()
    missing = [marker for marker in markers if marker not in text]
    if missing:
        raise SystemExit(f"{path}: missing endorsement-chain markers: {missing}")

for path in (TYPES, CAPS, COORD, NAV, BROWSER):
    text = path.read_text()
    if "PULSE_RUNTIME_PROTOCOL_VERSION: u16 = 18" in text:
        raise SystemExit(f"{path}: stale protocol-v18 runtime claim")

integrity = INTEGRITY.read_text()
for forbidden in [
    "firmware_security_version == 0 || record.firmware_security_version < 0",
    "active_root_count >= 0",
    "previous_revocation_set_hash.unwrap_or",
    "endorsement_chain_verified: true",
]:
    if forbidden in integrity:
        raise SystemExit(f"unsafe endorsement DHT shortcut present: {forbidden}")

print("Pulse endorsement-chain contract valid: protocol v19, fail-closed runtime, append-only evidence, browser truth")
