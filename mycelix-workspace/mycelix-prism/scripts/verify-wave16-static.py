#!/usr/bin/env python3
"""Static admission checks for Prism hardening Wave 16.

Wave 16 adds expiring health epochs, independently governed incident-policy
history, threshold promotion-recovery authorization, exact evidence
reconstruction, and a promoted canonical evidence lane. This verifier is
structural and does not claim Cargo, Nix, verifier agents, or offline signers
executed.
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
WAVE15 = ROOT / "scripts/verify-wave15-static.py"
SUPERSEDED_WAVE15_FAILURES = {
    "required Wave 15 invariant missing: Rust evidence verifier Wave 15 command",
    "required Wave 15 invariant missing: Rust evidence verifier Wave 15 path",
    "required Wave 15 invariant missing: evidence capture Wave 15 command",
    "required Wave 15 invariant missing: evidence capture Wave 15 path",
    "required Wave 15 invariant missing: Python evidence verifier Wave 15 command",
    "required Wave 15 invariant missing: Python evidence verifier Wave 15 path",
    "required Wave 15 invariant missing: Nix Wave 15 static gate",
}
KEY_FILES = [
    "Cargo.toml",
    "Cargo.lock",
    "flake.nix",
    "prism-ingest/src/review_agent.rs",
    "prism-attestation/Cargo.toml",
    "prism-attestation/src/lib.rs",
    "prism-attestation/src/release_health.rs",
    "prism-attestation/src/incident_policy.rs",
    "prism-attestation/src/promotion.rs",
    "prism-attestation/src/promotion_recovery.rs",
    "prism-attestation/src/recovery_authorization.rs",
    "prism-attestation/src/promotion_evidence_recovery.rs",
    "prism-attestation/src/bin/prepare_incident_policy_transition.rs",
    "prism-attestation/src/bin/assemble_incident_policy_transition.rs",
    "prism-attestation/src/bin/assemble_incident_policy_history.rs",
    "prism-attestation/src/bin/verify_incident_policy_history.rs",
    "prism-attestation/src/bin/prepare_promotion_recovery_authorization.rs",
    "prism-attestation/src/bin/assemble_promotion_recovery_authorization.rs",
    "prism-attestation/src/bin/assemble_promotion_recovery_bundle.rs",
    "prism-attestation/src/bin/verify_promotion_recovery_authorization.rs",
    "prism-attestation/src/bin/finalize_promotion_recovery.rs",
    "prism-attestation/src/bin/reconstruct_promotion_evidence.rs",
    "prism-attestation/src/bin/verify_promotion_evidence_reconstruction.rs",
    "prism-security-tests/src/lib.rs",
    "scripts/capture-build-evidence.py",
    "scripts/prism_build_evidence.py",
    "scripts/run-wave16-continuity-gates.py",
    "scripts/run-wave16-live-gates.py",
    "scripts/verify-wave15-static.py",
    "scripts/verify-wave16-static.py",
    "security/wave16-continuity.example.json",
    "PRISM_HARDENING_WAVE_16.md",
    "PRISM_RELEASE_CEREMONY_WAVE_16.md",
    "security/BUILD_EVIDENCE.md",
    "security/RELEASE_CEREMONY.md",
]


def load_wave15():
    spec = importlib.util.spec_from_file_location("prism_wave15_static", WAVE15)
    if spec is None or spec.loader is None:
        raise RuntimeError("could not load Wave 15 verifier")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def production(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8").split("#[cfg(test)]", 1)[0]


def require(text: str, needle: str, label: str, failures: list[str]) -> None:
    if needle not in text:
        failures.append(f"required Wave 16 invariant missing: {label}")


def forbid(text: str, needle: str, label: str, failures: list[str]) -> None:
    if needle in text:
        failures.append(f"forbidden Wave 16 pattern present: {label}")


def generate() -> tuple[dict[str, Any], list[str]]:
    receipt, inherited = load_wave15().generate()
    failures = [failure for failure in inherited if failure not in SUPERSEDED_WAVE15_FAILURES]
    missing = sorted(SUPERSEDED_WAVE15_FAILURES.difference(inherited))
    if missing:
        failures.append(
            "Wave 15 supersession set no longer matches historical verifier output: "
            + "; ".join(missing)
        )

    health = production("prism-attestation/src/release_health.rs")
    incident = production("prism-attestation/src/incident_policy.rs")
    promotion = production("prism-attestation/src/promotion.rs")
    authorization = production("prism-attestation/src/recovery_authorization.rs")
    reconstruction = production("prism-attestation/src/promotion_evidence_recovery.rs")
    finalize = production("prism-attestation/src/bin/finalize_promotion_recovery.rs")
    verify_authorization = production("prism-attestation/src/bin/verify_promotion_recovery_authorization.rs")
    reconstruct_bin = production("prism-attestation/src/bin/reconstruct_promotion_evidence.rs")
    verify_reconstruction = production("prism-attestation/src/bin/verify_promotion_evidence_reconstruction.rs")
    prepare_incident = production("prism-attestation/src/bin/prepare_incident_policy_transition.rs")
    assemble_incident = production("prism-attestation/src/bin/assemble_incident_policy_transition.rs")
    assemble_history = production("prism-attestation/src/bin/assemble_incident_policy_history.rs")
    verify_incident = production("prism-attestation/src/bin/verify_incident_policy_history.rs")
    review_agent = production("prism-ingest/src/review_agent.rs")
    security_tests = (ROOT / "prism-security-tests/src/lib.rs").read_text()
    continuity = (ROOT / "scripts/run-wave16-continuity-gates.py").read_text()
    live = (ROOT / "scripts/run-wave16-live-gates.py").read_text()
    build = production("prism-attestation/src/build_evidence.rs")
    capture = (ROOT / "scripts/capture-build-evidence.py").read_text()
    pybundle = (ROOT / "scripts/prism_build_evidence.py").read_text()
    flake = (ROOT / "flake.nix").read_text()

    for needle, label in [
        ("health_epoch", "monotonic release health epoch"),
        ("issued_at_unix", "health issue time"),
        ("valid_from_unix", "health validity start"),
        ("expires_at_unix", "health expiry"),
        ("MAX_HEALTH_VALIDITY_SECONDS", "bounded health validity"),
        ("incident_policy_history_root_blake3", "incident policy history binding"),
        ("require_promotion_healthy_at", "explicit promotion evaluation time"),
        ("EvaluationTimeRequired", "timeless health capability rejection"),
    ]:
        require(health, needle, label, failures)
    forbid(health, "fn require_promotion_healthy(&self) -> Result<PromotionHealthPermit", "timeless promotion permit", failures)

    for needle, label in [
        ("IncidentAuthorityPolicy", "independent incident authority policy"),
        ("SignedIncidentPolicyTransition", "dual-authorized incident policy transition"),
        ("previous_authority_signature", "prior incident-authority signature"),
        ("release_cosigner_signature", "prior release-cosigner signature"),
        ("IncidentPolicyHistory", "append-only incident policy history"),
        ("effective_from_release_sequence", "non-retroactive activation"),
        ("LoadedVerifiedIncidentPolicyHistory", "verified policy-history capability"),
        ("from_environment_prefix", "channel-scoped policy loading"),
        ("verify_history_with_agent", "measured agent history verification"),
        ("UnsignedIncidentPolicyTransitionRequest", "offline transition request"),
    ]:
        require(incident, needle, label, failures)
    for text, label in [
        (prepare_incident, "incident transition preparation"),
        (assemble_incident, "incident transition hybrid assembly"),
        (assemble_history, "incident history assembly"),
        (verify_incident, "incident history verification"),
    ]:
        require(text, "incident", label, failures)
    require(review_agent, "IncidentPolicyTransition", "incident transition verifier purpose", failures)
    require(review_agent, "RecoveryAuthorization", "recovery authorization verifier purpose", failures)

    for needle, label in [
        ("source_incident_policy_history_root_blake3", "source incident history root"),
        ("target_incident_policy_history_root_blake3", "target incident history root"),
        ("evaluation_time_unix", "promotion evaluation time"),
        ("source_health_epoch", "source health epoch"),
        ("target_health_epoch", "target health epoch"),
    ]:
        require(promotion, needle, label, failures)

    for needle, label in [
        ("RecoveryAuthorizationPolicy", "threshold recovery policy"),
        ("threshold", "guardian threshold"),
        ("max_validity_seconds", "authorization validity bound"),
        ("nonce", "fresh recovery nonce"),
        ("VerifiedRecoveryGuardian", "unforgeable guardian verification token"),
        ("RecoveryAuthorizationReceipt", "self-contained authorization receipt"),
        ("AuthorizedPromotionRecoveryReceipt", "actionable authorized recovery receipt"),
        ("validate_verified_tokens", "final quorum token comparison"),
        ("AuthorizationExpired", "expired authorization rejection"),
    ]:
        require(authorization, needle, label, failures)
    for needle, label in [
        ("ReviewVerificationPurpose::RecoveryAuthorization", "measured recovery signature verification"),
        ("RecoveryAuthorizationReceipt::from_verified_bundle", "token-gated receipt construction"),
    ]:
        require(verify_authorization, needle, label, failures)
    for needle, label in [
        ("LoadedRecoveryAuthorizationPolicy::from_environment", "final anchored recovery policy"),
        ("validate_verified_tokens", "final guardian reverification"),
        ("ReviewVerificationPurpose::RecoveryAuthorization", "final measured-agent reverification"),
        ("AuthorizedPromotionRecoveryReceipt::from_receipts", "authorized finalization"),
    ]:
        require(finalize, needle, label, failures)

    for needle, label in [
        ("PromotionEvidenceReconstructionReceipt", "typed evidence reconstruction receipt"),
        ("PromotionEvidenceKind", "closed evidence-kind set"),
        ("expected_filenames", "exact reconstructed filename set"),
        ("promotion_receipt_root_blake3", "promotion receipt root binding"),
    ]:
        require(reconstruction, needle, label, failures)
    for needle, label in [
        ("reject_extra_entries", "extra recovery file rejection"),
        ("atomic_write_private_idempotent", "idempotent reconstruction publication"),
        ("require_private_directory", "private recovery directory"),
        ("file_type().is_symlink", "symlink rejection"),
    ]:
        require(reconstruct_bin, needle, label, failures)
    for needle, label in [
        ("directory_entries", "independent exact-set enumeration"),
        ("actual_receipt != expected_receipt", "receipt substitution rejection"),
        ("bytes != object.bytes", "byte-for-byte object verification"),
    ]:
        require(verify_reconstruction, needle, label, failures)

    for needle, label in [
        ("recovery_authorization_is_thresholded_expiring_and_identity_bound", "cross-crate recovery authorization gate"),
        ("reconstructed_promotion_evidence_uses_a_closed_filename_set", "cross-crate reconstruction filename gate"),
    ]:
        require(security_tests, needle, label, failures)

    for text, label in [
        (build, "Rust evidence verifier"),
        (capture, "evidence capture"),
        (pybundle, "Python evidence verifier"),
    ]:
        require(text, '"wave16-static"', f"{label} Wave 16 command", failures)
        require(text, "verify-wave16-static.py", f"{label} Wave 16 path", failures)
    require(flake, "verify-wave16-static.py", "Nix Wave 16 static gate", failures)

    for needle, label in [
        ("run-wave15-continuity-gates.py", "retained Wave 15 continuity"),
        ("prism-verify-incident-policy-history", "incident history continuity"),
        ("prism-verify-promotion-recovery-authorization", "recovery quorum continuity"),
        ("prism-finalize-promotion-recovery", "authorized recovery continuity"),
        ("prism-reconstruct-promotion-evidence", "evidence reconstruction continuity"),
        ("prism-verify-promotion-evidence-reconstruction", "independent reconstruction audit"),
        ("directory_manifest(reconstruction) != reconstruction_before", "zero reconstruction mutation"),
    ]:
        require(continuity, needle, label, failures)
    forbid(continuity, "shell=True", "shell-mediated continuity execution", failures)
    for needle, label in [
        ("wave16-static", "live Wave 16 static command"),
        ("run-wave16-continuity-gates.py", "live Wave 16 continuity command"),
    ]:
        require(live, needle, label, failures)

    missing_files = [relative for relative in KEY_FILES if not (ROOT / relative).is_file()]
    if missing_files:
        failures.append("missing Wave 16 key files: " + ", ".join(missing_files))

    receipt.update({
        "schema_version": 1,
        "campaign": "prism-hardening-wave-16",
        "status": "pass" if not failures else "fail",
        "limitations": [
            "Does not compile Rust code or evaluate the Nix flake in this environment",
            "Does not execute verifier agents, incident-policy rotations, recovery guardians, or evidence reconstruction",
            "A generated, semantically admitted, hybrid-signed flake.lock remains required for release admission",
        ],
        "superseded_wave15_failures": sorted(SUPERSEDED_WAVE15_FAILURES),
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
        output.write_text(encoded)
    elif not arguments.check:
        print(encoded, end="")
    if failures:
        for failure in failures:
            print(failure, file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
