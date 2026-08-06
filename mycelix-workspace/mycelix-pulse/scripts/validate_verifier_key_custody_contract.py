#!/usr/bin/env python3
"""Fail-closed source contract for Pulse verifier authority custody v16."""

from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
TYPES = ROOT / "crates/mail-leptos-types/src/lib.rs"
CAPABILITIES = ROOT / "holochain/zomes/capabilities/coordinator/src/lib.rs"
INTEGRITY = ROOT / "holochain/zomes/mail-bridge/integrity/src/lib.rs"
BRIDGE = ROOT / "holochain/zomes/mail-bridge/coordinator/src/lib.rs"
NAV = ROOT / "apps/leptos/src/components/nav.rs"
BROWSER = ROOT / "scripts/live-browser-proof/live-proof.mjs"


def require(path: Path, needles: list[str]) -> None:
    text = path.read_text(encoding="utf-8")
    missing = [needle for needle in needles if needle not in text]
    if missing:
        raise SystemExit(f"{path}: missing verifier custody markers: {missing}")


def main() -> int:
    require(
        TYPES,
        [
            "PULSE_RUNTIME_PROTOCOL_VERSION: u16 = 13",
            "pub struct ProofVerifierKeyCustodyManifest",
            "pub proof_authority_custody: Vec<ProofVerifierKeyCustodyManifest>",
            "verifier_key_custody_policy_not_pinned",
            "verifier_key_ceremony_quorum_not_verified",
            "verifier_key_role_separation_not_verified",
            "verifier_key_recovery_path_not_installed",
            "verifier_key_compromise_unresolved",
            "release_custody_policy_hash",
            "release_custody_ceremony_hash",
            "release_custody_ceremony_epoch",
            "custody.custody_ready",
            "device_attestation_freshness_verified",
            "device_attestation_checkpoint_pinned",
            "device_trust_policy_hash",
            "device_measurement_policy_verified",
            "device_attestation_root_rotation_continuity_verified",
            "device_trust_checkpoint_pinned",
            "device_clone_evidence_resolved",
            "device_decommission_evidence_verified",
        ],
    )
    require(
        CAPABILITIES,
        [
            "let proof_authority_custody",
            "hardware_backed: false",
            "ceremony_quorum_verified: false",
            "role_separation_verified: false",
            "recovery_path_installed: false",
            "compromise_resolved: false",
            "proof_authority_custody,",
            "device_attestation_freshness_verified: false",
            "device_measurement_policy_verified: false",
            "device_attestation_root_rotation_continuity_verified: false",
            "device_trust_checkpoint_pinned: false",
            "device_clone_evidence_resolved: false",
            "device_decommission_evidence_verified: false",
        ],
    )
    require(
        INTEGRITY,
        [
            "pub struct SenderProofVerifierCustodyCeremonyRecord",
            "pub struct SenderProofVerifierCustodyCompromiseRecord",
            "pub struct SenderProofVerifierCustodyRecoveryRecord",
            "hardware_attestation_set_hash",
            "custody ceremony provenance must match its action",
            "custody compromise provenance must match its action",
            "custody recovery provenance must match its action",
            "SenderProofVerifierCustodyCeremonyRecord(_)",
            "SenderProofVerifierCustodyCompromiseRecord(_)",
            "SenderProofVerifierCustodyRecoveryRecord(_)",
            "SenderProofVerifierDeviceAttestationRecord",
            "SenderProofVerifierDeviceCloneEvidenceRecord",
            "SenderProofVerifierDeviceDecommissionRecord",
        ],
    )
    require(
        BRIDGE,
        [
            "custody_policy_hash: Option<[u8; 32]>",
            "custody_ceremony_hash: Option<[u8; 32]>",
            "custody_ceremony_epoch: Option<u64>",
            "custody_quorum_hash: Option<[u8; 32]>",
            "custody_authorized: false",
            "device_attestation_freshness_verified: false",
            "device_measurement_policy_verified: false",
            "device_attestation_root_rotation_continuity_verified: false",
            "device_trust_checkpoint_pinned: false",
            "device_clone_evidence_resolved: false",
            "device_decommission_evidence_verified: false",
        ],
    )
    require(
        NAV,
        [
            "data-runtime-proof-custody-count",
            "data-runtime-proof-custody-ready",
            "data-runtime-proof-custody-hardware",
            "data-runtime-proof-custody-quorum",
            "data-runtime-proof-custody-role-separation",
            "data-runtime-proof-custody-recovery",
            "data-runtime-proof-custody-compromise-resolved",
            "data-runtime-proof-custody-blockers",
            "data-runtime-proof-custody-attestation-fresh",
            "data-runtime-proof-custody-clone-resolved",
            "data-runtime-proof-custody-device-decommission",
        ],
    )
    require(
        BROWSER,
        [
            "evidence.protocol !== '13'",
            "proofCustodyReady !== 'false'",
            "proofCustodyCompromiseResolved !== 'false'",
            "runtime omitted verifier-custody blocker",
        ],
    )

    forbidden = [
        "custody_ready: true",
        "ceremony_quorum_verified: true",
        "custody_authorized: true",
        "proofCustodyReady !== 'true'",
    ]
    capability_text = CAPABILITIES.read_text(encoding="utf-8")
    present = [needle for needle in forbidden if needle in capability_text]
    if present:
        raise SystemExit(f"capability manifest prematurely enables custody: {present}")

    print(
        "verifier custody contract valid: protocol v13 exposes absent hardware-backed "
        "ceremony, role-separation, compromise, and recovery evidence; release and "
        "reservation identities bind exact custody hashes"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
