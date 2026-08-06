#!/usr/bin/env python3
"""Static admission checks for Prism hardening Wave 15.

Wave 15 adds hybrid-signed release health, dual-channel promotion-policy
rotation, target-sequence policy selection, and audited cross-channel promotion
recovery. This verifier is structural and does not claim Cargo, Nix, verifier
agents, or release signers executed.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import sys
from pathlib import Path
from typing import Any

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[1]
WAVE14 = ROOT / "scripts/verify-wave14-static.py"
SUPERSEDED_WAVE14_FAILURES = {
    "required Wave 14 invariant missing: Rust evidence verifier Wave 14 command",
    "required Wave 14 invariant missing: Rust evidence verifier Wave 14 path",
    "required Wave 14 invariant missing: evidence capture Wave 14 command",
    "required Wave 14 invariant missing: evidence capture Wave 14 path",
    "required Wave 14 invariant missing: Python evidence verifier Wave 14 command",
    "required Wave 14 invariant missing: Python evidence verifier Wave 14 path",
    "required Wave 14 invariant missing: Nix Wave 14 static gate",
}
KEY_FILES = [
    "Cargo.toml",
    "Cargo.lock",
    "flake.nix",
    "prism-ingest/src/review_agent.rs",
    "prism-attestation/Cargo.toml",
    "prism-attestation/src/lib.rs",
    "prism-attestation/src/release_health.rs",
    "prism-attestation/src/promotion.rs",
    "prism-attestation/src/promotion_state.rs",
    "prism-attestation/src/promotion_recovery.rs",
    "prism-attestation/src/promotion_policy_history.rs",
    "prism-attestation/src/bin/prepare_release_incident.rs",
    "prism-attestation/src/bin/assemble_release_incident.rs",
    "prism-attestation/src/bin/prepare_release_health.rs",
    "prism-attestation/src/bin/assemble_release_health.rs",
    "prism-attestation/src/bin/verify_release_health.rs",
    "prism-attestation/src/bin/prepare_promotion_policy_transition.rs",
    "prism-attestation/src/bin/assemble_promotion_policy_transition.rs",
    "prism-attestation/src/bin/assemble_promotion_policy_history.rs",
    "prism-attestation/src/bin/verify_promotion_policy_history.rs",
    "prism-attestation/src/bin/verify_promotion_recovery.rs",
    "prism-attestation/src/bin/verify_release_promotion.rs",
    "prism-attestation/src/bin/admit_release_promotion.rs",
    "prism-attestation/src/bin/audit_release_promotions.rs",
    "prism-security-tests/src/lib.rs",
    "scripts/capture-build-evidence.py",
    "scripts/prism_build_evidence.py",
    "scripts/run-wave15-continuity-gates.py",
    "scripts/run-wave15-live-gates.py",
    "scripts/verify-wave14-static.py",
    "scripts/verify-wave15-static.py",
    "security/wave15-continuity.example.json",
    "PRISM_HARDENING_WAVE_15.md",
    "PRISM_RELEASE_CEREMONY_WAVE_15.md",
    "security/BUILD_EVIDENCE.md",
    "security/RELEASE_CEREMONY.md",
]


def load_wave14():
    spec = importlib.util.spec_from_file_location("prism_wave14_static", WAVE14)
    if spec is None or spec.loader is None:
        raise RuntimeError("could not load Wave 14 verifier")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def production(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8").split("#[cfg(test)]", 1)[0]


def require(text: str, needle: str, label: str, failures: list[str]) -> None:
    if needle not in text:
        failures.append(f"required Wave 15 invariant missing: {label}")


def forbid(text: str, needle: str, label: str, failures: list[str]) -> None:
    if needle in text:
        failures.append(f"forbidden Wave 15 pattern present: {label}")


def generate() -> tuple[dict[str, Any], list[str]]:
    receipt, inherited = load_wave14().generate()
    failures = [failure for failure in inherited if failure not in SUPERSEDED_WAVE14_FAILURES]
    missing = sorted(SUPERSEDED_WAVE14_FAILURES.difference(inherited))
    if missing:
        failures.append(
            "Wave 14 supersession set no longer matches historical verifier output: "
            + "; ".join(missing)
        )

    health = production("prism-attestation/src/release_health.rs")
    promotion = production("prism-attestation/src/promotion.rs")
    recovery = production("prism-attestation/src/promotion_recovery.rs")
    history = production("prism-attestation/src/promotion_policy_history.rs")
    verify_promotion = production("prism-attestation/src/bin/verify_release_promotion.rs")
    admit_promotion = production("prism-attestation/src/bin/admit_release_promotion.rs")
    audit_promotion = production("prism-attestation/src/bin/audit_release_promotions.rs")
    verify_history = production("prism-attestation/src/bin/verify_promotion_policy_history.rs")
    verify_recovery = production("prism-attestation/src/bin/verify_promotion_recovery.rs")
    review_agent = production("prism-ingest/src/review_agent.rs")
    security_tests = (ROOT / "prism-security-tests/src/lib.rs").read_text()
    continuity = (ROOT / "scripts/run-wave15-continuity-gates.py").read_text()
    live = (ROOT / "scripts/run-wave15-live-gates.py").read_text()
    build = production("prism-attestation/src/build_evidence.rs")
    capture = (ROOT / "scripts/capture-build-evidence.py").read_text()
    pybundle = (ROOT / "scripts/prism_build_evidence.py").read_text()
    flake = (ROOT / "flake.nix").read_text()

    for needle, label in [
        ("SignedIncidentRevocation", "hybrid-signed incident action"),
        ("SignedReleaseHealthAttestation", "hybrid-signed release health"),
        ("ReleaseHealthStatus", "derived health state"),
        ("require_promotion_healthy", "promotion health capability"),
        ("IncidentAction::SuspendPromotion", "promotion suspension incident"),
        ("incident_component_messages", "domain-separated incident messages"),
        ("health_component_messages", "domain-separated health messages"),
    ]:
        require(health, needle, label, failures)
    for needle, label in [
        ("source_health_attestation_json", "embedded source health evidence"),
        ("target_health_attestation_json", "embedded target health evidence"),
        ("require_promotion_healthy", "healthy source and target gate"),
    ]:
        require(promotion, needle, label, failures)

    for needle, label in [
        ("PromotionRecoveryReceipt", "typed promotion recovery receipt"),
        ("MissingSourceRelease", "missing source history rejection"),
        ("MissingTargetRelease", "missing target history rejection"),
        ("SourceStatementMismatch", "source statement substitution rejection"),
        ("TargetStatementMismatch", "target statement substitution rejection"),
    ]:
        require(recovery, needle, label, failures)
    for needle, label in [
        ("PRISM_PROMOTION_SOURCE_RELEASE_AUDIT_BLAKE3", "source audit external anchor"),
        ("PRISM_PROMOTION_TARGET_RELEASE_AUDIT_BLAKE3", "target audit external anchor"),
        ("PRISM_PROMOTION_AUDIT_BLAKE3", "promotion audit external anchor"),
        ("atomic_write_private_idempotent", "idempotent recovery receipt publication"),
    ]:
        require(verify_recovery, needle, label, failures)

    for needle, label in [
        ("SignedPromotionPolicyTransition", "dual-channel promotion policy transition"),
        ("PromotionPolicyHistory", "append-only promotion policy history"),
        ("PromotionPolicyAuthority::SourceChannel", "source-channel authority"),
        ("PromotionPolicyAuthority::TargetChannel", "target-channel authority"),
        ("effective_target_sequence", "non-retroactive policy activation"),
        ("policy_for_target_sequence", "target-sequence policy selection"),
        ("verify_history_with_agent", "measured verifier-agent history audit"),
        ("history_prefix_root_blake3", "rotation provenance prefix root"),
        ("PromotionPolicyHistoryVerificationReceipt", "self-contained verified history receipt"),
    ]:
        require(history, needle, label, failures)
    require(review_agent, "PromotionPolicyTransition", "verifier-agent promotion transition purpose", failures)
    require(history, "effective_target_sequence <= target_release_policy.minimum_sequence", "non-retroactive activation after target signer epoch", failures)
    require(history, "self.history_root_blake3 == [0; 32]", "non-zero policy history root", failures)
    forbid(promotion, "pub struct LoadedPromotionPolicy", "direct promotion policy loader bypass", failures)
    for text, label in [
        (verify_promotion, "promotion creation"),
        (admit_promotion, "promotion admission"),
        (audit_promotion, "promotion history audit"),
    ]:
        require(text, "LoadedPromotionPolicyHistory", f"{label} loads anchored policy history", failures)
        require(text, "policy_for_target_sequence", f"{label} selects target-sequence policy", failures)
        forbid(text, "LoadedPromotionPolicy::from_environment", f"{label} direct mutable policy anchor", failures)
    require(verify_history, "PRISM_PROMOTION_POLICY_CURRENT_BLAKE3", "externally anchored current policy", failures)
    require(verify_history, "ReviewVerificationPurpose::PromotionPolicyTransition", "policy transition verifier purpose", failures)

    for needle, label in [
        ("promotion_rejects_a_hybrid_signed_suspension_incident", "health suspension cross-crate gate"),
        ("promotion_rejects_payload_substitution_across_channels", "payload substitution gate retained"),
    ]:
        require(security_tests, needle, label, failures)

    for text, label in [
        (build, "Rust evidence verifier"),
        (capture, "evidence capture"),
        (pybundle, "Python evidence verifier"),
    ]:
        require(text, '"wave15-static"', f"{label} Wave 15 command", failures)
        require(text, "verify-wave15-static.py", f"{label} Wave 15 path", failures)
    require(flake, "verify-wave15-static.py", "Nix Wave 15 static gate", failures)

    for needle, label in [
        ("prism-verify-promotion-policy-history", "continuity policy history verification"),
        ("prism-verify-promotion-recovery", "continuity cross-channel recovery"),
        ("state_manifest(state_channel) != state_before", "zero promotion-state mutation"),
        ("len(set(paths.values()))", "continuity path alias rejection"),
    ]:
        require(continuity, needle, label, failures)
    forbid(continuity, "shell=True", "shell-mediated continuity execution", failures)
    for needle, label in [
        ("wave15-static", "live Wave 15 static command"),
        ("run-wave15-continuity-gates.py", "live Wave 15 continuity command"),
        ("PRISM_PROMOTION_POLICY_HISTORY_PATH", "live promotion policy history path"),
        ("PRISM_PROMOTION_POLICY_CURRENT_BLAKE3", "live current promotion policy anchor"),
    ]:
        require(live, needle, label, failures)

    missing_files = [relative for relative in KEY_FILES if not (ROOT / relative).is_file()]
    if missing_files:
        failures.append("missing Wave 15 key files: " + ", ".join(missing_files))

    receipt.update({
        "schema_version": 1,
        "campaign": "prism-hardening-wave-15",
        "status": "pass" if not failures else "fail",
        "limitations": [
            "Does not compile Rust code or evaluate the Nix flake in this environment",
            "Does not execute verifier agents, release signers, incident responders, or cross-channel promotion recovery",
            "A generated, semantically admitted, hybrid-signed flake.lock remains required for release admission",
        ],
        "superseded_wave14_failures": sorted(SUPERSEDED_WAVE14_FAILURES),
        "key_file_sha256": {
            relative: digest(ROOT / relative)
            for relative in KEY_FILES
            if (ROOT / relative).is_file()
        },
        "failures": failures,
    })
    receipt.setdefault("release_blockers", [])
    receipt["release_blockers"] = [
        blocker for blocker in receipt["release_blockers"]
        if not blocker.startswith("flake.lock has not been generated")
    ]
    if not (ROOT / "flake.lock").is_file():
        receipt["release_blockers"].append(
            "flake.lock has not been generated, semantically reviewed, and hybrid-signed on a Nix-enabled host"
        )
    return receipt, failures


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path)
    parser.add_argument("--check", type=Path)
    arguments = parser.parse_args()
    receipt, failures = generate()
    encoded = json.dumps(receipt, indent=2, sort_keys=True) + "\n"
    if arguments.check:
        expected = arguments.check if arguments.check.is_absolute() else ROOT / arguments.check
        if not expected.is_file() or expected.read_text() != encoded:
            failures.append(f"receipt mismatch: {expected}")
    if arguments.output:
        output = arguments.output if arguments.output.is_absolute() else ROOT / arguments.output
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(encoded)
    elif not arguments.check:
        sys.stdout.write(encoded)
    return 0 if not failures else 1


if __name__ == "__main__":
    raise SystemExit(main())
