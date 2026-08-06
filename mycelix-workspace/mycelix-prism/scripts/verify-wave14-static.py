#!/usr/bin/env python3
"""Static admission checks for Prism hardening Wave 14.

Wave 14 adds exact cold-archive restore verification, deterministic recovery
convergence, byte-identical cross-channel release promotion, and an immutable
promotion journal. This verifier is structural and does not claim that Cargo,
Nix, Bubblewrap, production verifier agents, or release signers ran.
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
WAVE13 = ROOT / "scripts/verify-wave13-static.py"
SUPERSEDED_WAVE13_FAILURES = {
    "required Wave 13 invariant missing: Rust evidence verifier Wave 13 command",
    "required Wave 13 invariant missing: Rust evidence verifier Wave 13 path",
    "required Wave 13 invariant missing: evidence capture Wave 13 command",
    "required Wave 13 invariant missing: evidence capture Wave 13 path",
    "required Wave 13 invariant missing: Python evidence verifier Wave 13 command",
    "required Wave 13 invariant missing: Python evidence verifier Wave 13 path",
    "required Wave 13 invariant missing: Nix Wave 13 static gate",
}
KEY_FILES = [
    "Cargo.toml",
    "Cargo.lock",
    "flake.nix",
    "prism-attestation/Cargo.toml",
    "prism-attestation/src/lib.rs",
    "prism-attestation/src/archive_handoff.rs",
    "prism-attestation/src/archive_restore.rs",
    "prism-attestation/src/recovery_drill.rs",
    "prism-attestation/src/promotion.rs",
    "prism-attestation/src/promotion_state.rs",
    "prism-attestation/src/bin/verify_archive_restore.rs",
    "prism-attestation/src/bin/prepare_recovery_drill.rs",
    "prism-attestation/src/bin/verify_recovery_drill.rs",
    "prism-attestation/src/bin/verify_release_promotion.rs",
    "prism-attestation/src/bin/admit_release_promotion.rs",
    "prism-attestation/src/bin/audit_release_promotions.rs",
    "prism-security-tests/src/lib.rs",
    "scripts/capture-build-evidence.py",
    "scripts/prism_build_evidence.py",
    "scripts/run-wave14-continuity-gates.py",
    "scripts/run-wave14-live-gates.py",
    "scripts/verify-wave13-static.py",
    "scripts/verify-wave14-static.py",
    "security/wave14-continuity.example.json",
    "PRISM_HARDENING_WAVE_14.md",
    "PRISM_RELEASE_CEREMONY_WAVE_14.md",
    "security/BUILD_EVIDENCE.md",
    "security/RELEASE_CEREMONY.md",
]


def load_wave13():
    spec = importlib.util.spec_from_file_location("prism_wave13_static", WAVE13)
    if spec is None or spec.loader is None:
        raise RuntimeError("could not load Wave 13 verifier")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def production(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8").split("#[cfg(test)]", 1)[0]


def require(text: str, needle: str, label: str, failures: list[str]) -> None:
    if needle not in text:
        failures.append(f"required Wave 14 invariant missing: {label}")


def forbid(text: str, needle: str, label: str, failures: list[str]) -> None:
    if needle in text:
        failures.append(f"forbidden Wave 14 pattern present: {label}")


def generate() -> tuple[dict[str, Any], list[str]]:
    receipt, inherited = load_wave13().generate()
    failures = [failure for failure in inherited if failure not in SUPERSEDED_WAVE13_FAILURES]
    missing = sorted(SUPERSEDED_WAVE13_FAILURES.difference(inherited))
    if missing:
        failures.append(
            "Wave 13 supersession set no longer matches historical verifier output: "
            + "; ".join(missing)
        )

    restore = production("prism-attestation/src/archive_restore.rs")
    verify_restore = production("prism-attestation/src/bin/verify_archive_restore.rs")
    drill = production("prism-attestation/src/recovery_drill.rs")
    prepare_drill = production("prism-attestation/src/bin/prepare_recovery_drill.rs")
    verify_drill = production("prism-attestation/src/bin/verify_recovery_drill.rs")
    promotion = production("prism-attestation/src/promotion.rs")
    promotion_state = production("prism-attestation/src/promotion_state.rs")
    verify_promotion = production("prism-attestation/src/bin/verify_release_promotion.rs")
    admit_promotion = production("prism-attestation/src/bin/admit_release_promotion.rs")
    audit_promotion = production("prism-attestation/src/bin/audit_release_promotions.rs")
    continuity = (ROOT / "scripts/run-wave14-continuity-gates.py").read_text()
    live = (ROOT / "scripts/run-wave14-live-gates.py").read_text()
    security_tests = (ROOT / "prism-security-tests/src/lib.rs").read_text()
    build = production("prism-attestation/src/build_evidence.rs")
    capture = (ROOT / "scripts/capture-build-evidence.py").read_text()
    pybundle = (ROOT / "scripts/prism_build_evidence.py").read_text()
    flake = (ROOT / "flake.nix").read_text()

    for needle, label in [
        ("ArchiveRestoreReceipt", "typed archive restore receipt"),
        ("walk_regular_files", "exact restored file enumeration"),
        ("FileSetMismatch", "extra or missing restore rejection"),
        ("measure_admitted_file", "stable restored file measurement"),
        ("release_statement_blake3", "release statement binding"),
        ("archive_handoff_statement_blake3", "archive handoff binding"),
        ("canonical_private_root", "private restore root"),
        ("object.kind.code()", "stable archive object coding"),
    ]:
        require(restore, needle, label, failures)
    require(verify_restore, "ReviewVerificationPurpose::ArchiveHandoff", "signed restore verifier purpose", failures)
    require(verify_restore, "PRISM_RELEASE_ARCHIVE_MANIFEST_BLAKE3", "restore manifest anchor", failures)
    require(verify_restore, "PRISM_RELEASE_ATTESTATION_BLAKE3", "restore release anchor", failures)

    for needle, label in [
        ("RecoveryDrillPlan", "typed recovery drill plan"),
        ("RecoveryDrillReceipt", "typed recovery drill receipt"),
        ("release_journal_audit_root_blake3", "release history convergence"),
        ("transparency_journal_audit_root_blake3", "transparency history convergence"),
        ("archive_restore_receipt_root_blake3", "restored evidence convergence"),
        ("minimum_restored_objects", "minimum restore threshold"),
    ]:
        require(drill, needle, label, failures)
    for text, label in [(prepare_drill, "drill preparation"), (verify_drill, "drill verification")]:
        require(text, "require_distinct_paths", f"{label} path separation", failures)
    require(prepare_drill, "PRISM_ARCHIVE_RESTORE_RECEIPT_BLAKE3", "restore receipt anchor during drill preparation", failures)
    require(verify_drill, "PRISM_RECOVERY_DRILL_PLAN_BLAKE3", "drill plan anchor", failures)

    for needle, label in [
        ("ReleasePromotionPolicy", "anchored promotion policy"),
        ("ReleasePromotionReceipt", "self-contained promotion receipt"),
        ("VerifiedPromotion", "promotion verification capability"),
        ("source.statement.release_version != target.statement.release_version", "same release version gate"),
        ("source.statement.source_commit != target.statement.source_commit", "same source commit gate"),
        ("source.statement.source_tree != target.statement.source_tree", "same source tree gate"),
        ("source_payload.sha256 != target_payload.sha256", "same payload digest gate"),
        ("source_attestation_json", "embedded source attestation"),
        ("target_attestation_json", "embedded target attestation"),
    ]:
        require(promotion, needle, label, failures)
    require(verify_promotion, "PRISM_PROMOTION_SOURCE_ATTESTATION_BLAKE3", "source release anchor", failures)
    require(verify_promotion, "PRISM_PROMOTION_TARGET_ATTESTATION_BLAKE3", "target release anchor", failures)
    require(verify_promotion, "ReviewVerificationPurpose::ReleaseAttestation", "release-signature re-verification", failures)

    for needle, label in [
        ("PromotionCheckpoint", "immutable promotion checkpoint"),
        ("PromotionJournalAuditReceipt", "promotion history audit"),
        ("Equivocation", "same-sequence promotion equivocation rejection"),
        ("Rollback", "promotion rollback rejection"),
        ("libc::LOCK_EX", "kernel-released promotion lock"),
        ("directory_fingerprint", "promotion journal directory identity"),
        ("atomic_write_private(&path", "create-once promotion checkpoint publication"),
    ]:
        require(promotion_state, needle, label, failures)
    require(admit_promotion, "journal.admit(&verified", "verified capability required for admission", failures)
    require(audit_promotion, "receipt.verify_embedded", "complete promotion history re-verification", failures)

    for needle, label in [
        ("promotion_rejects_payload_substitution_across_channels", "payload substitution regression"),
        ("recovery_drill_plan_is_bound_to_all_three_evidence_roots", "recovery root substitution regression"),
    ]:
        require(security_tests, needle, label, failures)

    for text, label in [(build, "Rust evidence verifier"), (capture, "evidence capture"), (pybundle, "Python evidence verifier")]:
        require(text, '"wave14-static"', f"{label} Wave 14 command", failures)
        require(text, "verify-wave14-static.py", f"{label} Wave 14 path", failures)
    require(flake, "verify-wave14-static.py", "Nix Wave 14 static gate", failures)

    for needle, label in [
        ("wave14-static", "live Wave 14 static command"),
        ("run-wave14-continuity-gates.py", "live continuity command"),
        ("--continuity-config", "continuity configuration admission"),
    ]:
        require(live, needle, label, failures)
    for needle, label in [
        ("EXPECTED_FIELDS", "exact continuity schema"),
        ("prism-verify-recovery-drill", "live recovery drill"),
        ("prism-admit-release-promotion", "live promotion admission"),
        ("prism-audit-release-promotions", "live promotion audit"),
        ("len(set(paths.values()))", "continuity path alias rejection"),
    ]:
        require(continuity, needle, label, failures)
    forbid(continuity, "shell=True", "shell-mediated continuity execution", failures)

    receipt.update({
        "schema_version": 1,
        "campaign": "prism-hardening-wave-14",
        "status": "pass" if not failures else "fail",
        "limitations": [
            "Does not compile Rust code or evaluate the Nix flake in this environment",
            "Does not execute verifier agents, release signers, archive restoration, Bubblewrap, Chromium, or network namespaces",
            "A generated, semantically admitted, hybrid-signed flake.lock remains required for release admission",
        ],
        "superseded_wave13_failures": sorted(SUPERSEDED_WAVE13_FAILURES),
        "key_file_sha256": {relative: digest(ROOT / relative) for relative in KEY_FILES},
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
