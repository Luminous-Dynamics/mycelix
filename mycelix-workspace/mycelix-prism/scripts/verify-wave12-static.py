#!/usr/bin/env python3
"""Static admission checks for Prism hardening Wave 12.

Wave 12 adds purpose/challenge-bound verifier requests, threshold release
transparency witnesses, externally anchored recovery, and a post-admission live
gate that requires zero journal mutation. It does not claim that Rust, Nix,
Bubblewrap, Chromium, or production cryptographic agents executed here.
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
WAVE11 = ROOT / "scripts/verify-wave11-static.py"
SUPERSEDED_WAVE11_FAILURES = {
    "required Wave 11 invariant missing: verifier-agent protocol v2",
    "required Wave 11 invariant missing: Rust evidence verifier Wave 11 command",
    "required Wave 11 invariant missing: Rust evidence verifier Wave 11 path",
    "required Wave 11 invariant missing: evidence capture Wave 11 command",
    "required Wave 11 invariant missing: evidence capture Wave 11 path",
    "required Wave 11 invariant missing: Python evidence verifier Wave 11 command",
    "required Wave 11 invariant missing: Python evidence verifier Wave 11 path",
    "required Wave 11 invariant missing: Nix Wave 11 static gate",
}
KEY_FILES = [
    "Cargo.toml",
    "Cargo.lock",
    "flake.nix",
    ".github/workflows/core-security.yml",
    "prism-attestation/Cargo.toml",
    "prism-attestation/src/lib.rs",
    "prism-attestation/src/build_evidence.rs",
    "prism-attestation/src/release_state.rs",
    "prism-attestation/src/retention.rs",
    "prism-attestation/src/transparency.rs",
    "prism-attestation/src/bin/prepare_release_witness.rs",
    "prism-attestation/src/bin/assemble_release_witness.rs",
    "prism-attestation/src/bin/assemble_witness_bundle.rs",
    "prism-attestation/src/bin/verify_witness_quorum.rs",
    "prism-attestation/src/bin/audit_transparency_quorum.rs",
    "prism-attestation/src/bin/recover_release_state.rs",
    "prism-attestation/src/bin/plan_release_retention.rs",
    "prism-ingest/Cargo.toml",
    "prism-ingest/src/review_agent.rs",
    "prism-search/src/bin/build_index.rs",
    "scripts/capture-build-evidence.py",
    "scripts/prism_build_evidence.py",
    "scripts/run-wave12-live-gates.py",
    "scripts/verify-wave11-static.py",
    "scripts/verify-wave12-static.py",
    "scripts/verify-supply-chain.py",
    "security/dependency-policy.json",
    "security/release-transparency-policy.example.json",
    "security/release-retention-policy.example.json",
    "security/RELEASE_CEREMONY.md",
]


def load_wave11():
    spec = importlib.util.spec_from_file_location("prism_wave11_static", WAVE11)
    if spec is None or spec.loader is None:
        raise RuntimeError("could not load Wave 11 verifier")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def production(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8").split("#[cfg(test)]", 1)[0]


def require(text: str, needle: str, label: str, failures: list[str]) -> None:
    if needle not in text:
        failures.append(f"required Wave 12 invariant missing: {label}")


def forbid(text: str, needle: str, label: str, failures: list[str]) -> None:
    if needle in text:
        failures.append(f"forbidden Wave 12 pattern present: {label}")


def generate() -> tuple[dict[str, Any], list[str]]:
    receipt, inherited = load_wave11().generate()
    failures = [failure for failure in inherited if failure not in SUPERSEDED_WAVE11_FAILURES]
    missing = sorted(SUPERSEDED_WAVE11_FAILURES.difference(inherited))
    if missing:
        failures.append(
            "Wave 11 supersession set no longer matches historical verifier output: "
            + "; ".join(missing)
        )

    agent = production("prism-ingest/src/review_agent.rs")
    transparency = production("prism-attestation/src/transparency.rs")
    retention = production("prism-attestation/src/retention.rs")
    retention_bin = production("prism-attestation/src/bin/plan_release_retention.rs")
    recovery = production("prism-attestation/src/bin/recover_release_state.rs")
    state = production("prism-attestation/src/release_state.rs")
    prepare_witness = production("prism-attestation/src/bin/prepare_release_witness.rs")
    assemble_witness = production("prism-attestation/src/bin/assemble_release_witness.rs")
    assemble_bundle = production("prism-attestation/src/bin/assemble_witness_bundle.rs")
    verify_quorum = production("prism-attestation/src/bin/verify_witness_quorum.rs")
    audit_quorum = production("prism-attestation/src/bin/audit_transparency_quorum.rs")
    build_index = production("prism-search/src/bin/build_index.rs")
    verify_flake = production("prism-attestation/src/bin/verify_flake_review.rs")
    verify_release = production("prism-attestation/src/bin/verify_release.rs")
    audit = production("prism-attestation/src/bin/audit_release_journal.rs")
    build = production("prism-attestation/src/build_evidence.rs")
    capture = (ROOT / "scripts/capture-build-evidence.py").read_text()
    pybundle = (ROOT / "scripts/prism_build_evidence.py").read_text()
    live = (ROOT / "scripts/run-wave12-live-gates.py").read_text()
    flake = (ROOT / "flake.nix").read_text()

    for needle, label in [
        ("AGENT_PROTOCOL_VERSION: u16 = 3", "verifier-agent protocol v3"),
        ("prism-review-verifier-agent-v3", "verifier-agent v3 domain"),
        ("ReviewVerificationPurpose", "typed verification purpose"),
        ("challenge", "fresh verifier challenge"),
        ("getrandom::fill", "OS randomness for verifier challenges"),
        ("response.purpose", "response purpose binding"),
        ("response.challenge", "response challenge binding"),
    ]:
        require(agent, needle, label, failures)
    forbid(agent, "AGENT_PROTOCOL_VERSION: u16 = 2", "active verifier protocol v2", failures)

    for text, needle, label in [
        (build_index, "ReviewVerificationPurpose::ReviewLedger", "review-ledger purpose"),
        (verify_flake, "ReviewVerificationPurpose::FlakeLockReview", "flake-review purpose"),
        (verify_release, "ReviewVerificationPurpose::ReleaseAttestation", "release purpose"),
        (audit, "ReviewVerificationPurpose::ReleaseJournalAudit", "journal-audit purpose"),
        (recovery, "ReviewVerificationPurpose::ReleaseJournalRecovery", "journal-recovery purpose"),
        (verify_quorum, "ReviewVerificationPurpose::TransparencyWitness", "transparency-witness purpose"),
        (audit_quorum, "ReviewVerificationPurpose::TransparencyWitnessAudit", "transparency-witness-audit purpose"),
    ]:
        require(text, needle, label, failures)

    for needle, label in [
        ("pub struct TransparencyPolicy", "typed transparency policy"),
        ("threshold < 2", "minimum two-witness quorum"),
        ("head_file_blake3", "exact release-head file binding"),
        ("audit_receipt_file_blake3", "exact audit-receipt file binding"),
        ("ReleaseWitnessBundle", "typed witness bundle"),
        ("TransparencyQuorumReceipt", "typed quorum receipt"),
        ("HYBRID_REVIEW_ALGORITHM", "hybrid witness algorithm"),
        ("TRANSPARENCY_RECEIPT_SCHEMA_VERSION: u16 = 2", "self-contained quorum receipt schema v2"),
        ("transparency_policy_json", "embedded transparency policy bytes"),
        ("witness_bundle_json", "embedded signed witness bundle bytes"),
        ("TransparencyQuorumAuditReceipt", "typed transparency re-audit receipt"),
        ("prism-release-transparency-audit-v1", "transparency audit root domain"),
        ("witnesses must be unique and sorted", "canonical distinct witnesses"),
    ]:
        require(transparency, needle, label, failures)

    for needle, label in [
        ("VerifiedReleaseWitness", "unforgeable verified-witness token"),
        ("verified: &[VerifiedReleaseWitness]", "quorum requires verified-witness tokens"),
        ("token.signature_blake3", "verified token exact signature binding"),
        ("token.statement_blake3", "verified token exact statement binding"),
    ]:
        require(transparency, needle, label, failures)
    require(verify_quorum, "let mut verified", "quorum verifier collects verification tokens", failures)

    for text, label in [
        (prepare_witness, "offline witness request preparation"),
        (assemble_witness, "offline witness signature assembly"),
        (assemble_bundle, "threshold witness bundle assembly"),
        (verify_quorum, "threshold witness verification"),
    ]:
        require(text, "LoadedTransparencyPolicy", f"{label} anchored policy", failures)
        require(text, "require_distinct_paths", f"{label} path separation", failures)
    require(assemble_witness, "ED25519_SIGNATURE_BYTES", "fixed Ed25519 witness signature size", failures)
    require(assemble_witness, "ML_DSA_65_SIGNATURE_BYTES", "fixed ML-DSA witness signature size", failures)
    require(verify_quorum, "atomic_write_private_idempotent", "idempotent quorum receipt publication", failures)
    require(verify_quorum, "TransparencyQuorumAuditReceipt::from_verified_receipt", "same-ceremony quorum audit", failures)
    for needle, label in [
        ("PRISM_RELEASE_TRANSPARENCY_RECEIPT_BLAKE3", "externally anchored quorum receipt"),
        ("TransparencyWitnessAudit", "separate audit verification purpose"),
        ("from_verified_receipt", "verified audit receipt construction"),
        ("atomic_write_private_idempotent", "idempotent transparency audit publication"),
    ]:
        require(audit_quorum, needle, label, failures)

    for needle, label in [
        ("ReleaseRecoveryReceipt", "typed recovery receipt"),
        ("head_file_blake3", "recovery exact head bytes"),
        ("audit_receipt_file_blake3", "recovery exact audit bytes"),
        ("prism-release-recovery-receipt-v1", "recovery receipt domain"),
    ]:
        require(state, needle, label, failures)
    for needle, label in [
        ("PRISM_RELEASE_RECOVERY_EXPECTED_STATEMENT_BLAKE3", "external statement recovery anchor"),
        ("PRISM_RELEASE_RECOVERY_EXPECTED_CHECKPOINT_BLAKE3", "external checkpoint recovery anchor"),
        ("checkpoints_for_audit", "complete recovery journal audit"),
        ("attestation.verify", "historical signature re-verification during recovery"),
        ("atomic_write_private_idempotent", "idempotent recovery publication"),
    ]:
        require(recovery, needle, label, failures)

    for needle, label in [
        ("ReleaseRetentionPolicy", "typed release-retention policy"),
        ("keep_latest_online", "bounded online retention window"),
        ("JournalCheckpointAction::RetainOnline", "permanent journal retention"),
        ("EligibleForOfflineArchive", "non-destructive cold-archive classification"),
        ("transparency_quorum_root_blake3", "witnessed retention evidence"),
        ("plan_root_blake3", "deterministic retention plan root"),
    ]:
        require(retention, needle, label, failures)
    for needle, label in [
        ("PRISM_RELEASE_RETENTION_POLICY_BLAKE3", "external retention-policy anchor"),
        ("PRISM_RELEASE_HEAD_BLAKE3", "external release-head anchor"),
        ("PRISM_RELEASE_AUDIT_RECEIPT_BLAKE3", "external journal-audit anchor"),
        ("PRISM_RELEASE_TRANSPARENCY_AUDIT_BLAKE3", "external transparency-audit anchor"),
        ("atomic_write_private_idempotent", "idempotent retention-plan publication"),
    ]:
        require(retention_bin, needle, label, failures)

    for text, label in [
        (build, "Rust evidence verifier"),
        (capture, "evidence capture"),
        (pybundle, "Python evidence verifier"),
    ]:
        require(text, '"wave12-static"', f"{label} Wave 12 command", failures)
        require(text, "verify-wave12-static.py", f"{label} Wave 12 path", failures)
    require(flake, "verify-wave12-static.py", "Nix Wave 12 static gate", failures)

    for needle, label in [
        ("--witness-bundle", "live witness bundle input"),
        ("PRISM_RELEASE_TRANSPARENCY_POLICY_PATH", "live transparency policy"),
        ("PRISM_RELEASE_RECOVERY_RECEIPT_OUTPUT", "live recovery receipt"),
        ("PRISM_RELEASE_TRANSPARENCY_RECEIPT_OUTPUT", "live transparency receipt"),
        ("PRISM_RELEASE_TRANSPARENCY_AUDIT_OUTPUT", "live transparency audit receipt"),
        ("prism-recover-release-state", "live recovery execution"),
        ("prism-verify-witness-quorum", "live witness verification"),
        ("transparency_quorum_audit", "live transparency audit evidence"),
        ("require_no_checkpoint_change", "post-admission zero journal mutation"),
        ("wave12-static", "live Wave 12 static command"),
        ("verify-wave12-static.py", "live Wave 12 verifier"),
    ]:
        require(live, needle, label, failures)
    forbid(live, "require_single_checkpoint_advance", "Wave 11 advancement semantics in Wave 12", failures)

    try:
        example = json.loads((ROOT / "security/release-transparency-policy.example.json").read_text())
        if (
            set(example) != {"schema_version", "policy_id", "project", "channel", "threshold", "witnesses"}
            or example.get("threshold", 0) < 2
            or not isinstance(example.get("witnesses"), list)
            or len(example["witnesses"]) < example["threshold"]
        ):
            failures.append("release transparency policy example is not an exact threshold policy")
    except (OSError, ValueError, TypeError) as error:
        failures.append(f"release transparency policy example is invalid: {error}")

    try:
        retention_example = json.loads(
            (ROOT / "security/release-retention-policy.example.json").read_text()
        )
        if (
            set(retention_example)
            != {
                "schema_version",
                "policy_id",
                "channel",
                "keep_latest_online",
                "preserve_sequences",
                "permit_offline_archive",
            }
            or retention_example.get("keep_latest_online", 0) < 2
            or retention_example.get("preserve_sequences") != sorted(
                set(retention_example.get("preserve_sequences", []))
            )
        ):
            failures.append("release retention policy example is not exact and conservative")
    except (OSError, ValueError, TypeError) as error:
        failures.append(f"release retention policy example is invalid: {error}")

    receipt.update(
        {
            "schema_version": 1,
            "campaign": "prism-hardening-wave-12",
            "status": "pass" if not failures else "fail",
            "limitations": [
                "Does not compile Rust code or evaluate the Nix flake in this environment",
                "Does not execute the verifier agent, Bubblewrap, Chromium, network namespaces, or witness signers",
                "A generated, semantically admitted, hybrid-signed flake.lock remains required for release admission",
            ],
            "superseded_wave11_failures": sorted(SUPERSEDED_WAVE11_FAILURES),
            "key_file_sha256": {relative: digest(ROOT / relative) for relative in KEY_FILES},
            "failures": failures,
        }
    )
    receipt.setdefault("release_blockers", [])
    receipt["release_blockers"] = [
        blocker
        for blocker in receipt["release_blockers"]
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
