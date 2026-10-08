#!/usr/bin/env python3
"""Independent evaluator for the D6U trusted verifier.

This evaluator is intentionally kept outside the candidate test suite. It loads
the verifier under test as a module and supplies its own policy/record fixtures.
It must remain small, deterministic, and stdlib-only.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import pathlib
import subprocess
import sys


def load_module(path: pathlib.Path):
    spec = importlib.util.spec_from_file_location("candidate_d6u_verifier", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"unable to load candidate verifier: {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def assert_rejected(fn, message: str) -> None:
    try:
        fn()
    except AssertionError:
        return
    raise AssertionError(message)


def policy() -> dict:
    return {
        "record_fields": [
            "status",
            "workflow_run_id",
            "workflow_run_attempt",
            "source_commit",
            "source_branch",
            "source_repository",
            "trigger_workflow_run_id",
            "trigger_workflow_run_attempt",
            "trigger_workflow_name",
            "trigger_workflow_path",
            "trigger_workflow_blob_sha",
            "executor_run_id",
            "executor_run_attempt",
            "executor_workflow_commit_sha",
            "executor_workflow_ref",
            "executor_workflow_file_path",
            "executor_workflow_repository",
            "attestation_status",
            "d6s2_manifest_git_blob_sha",
            "d6s2_fixture_git_blob_sha",
            "d6s2_authority_ledger_schema",
            "d6s1_corpus_sha256",
            "test_log_sha256",
            "manifest_version",
            "manifest_git_blob_sha",
            "evidence_verifier_git_blob_sha",
            "lock_verifier_git_blob_sha",
            "case_coverage",
            "supplemental_coverage",
            "application_check_coverage",
            "case_outcome_classes",
            "runtime",
            "hdk",
            "hdi",
            "rust",
            "rust_verbose_commit",
            "cargo",
            "cargo_lock_sha256",
            "test",
            "supported_cases",
            "unsupported_cases",
            "claim_ceiling",
        ],
        "d6s2_authority_ledger_schema": "v1",
        "d6s1_corpus_sha256": "a" * 64,
        "manifest_version": 11,
        "required_source_blobs": {
            "docs/integral/d6u-runtime-manifest.json": "b" * 40,
            "scripts/integral/verify_d6u_runtime_evidence.py": "c" * 40,
            "scripts/integral/verify_d6u_runtime_lock.py": "d" * 40,
        },
        "expected_case_coverage": "14-of-14",
        "expected_supplemental_coverage": "4-of-4",
        "expected_application_check_coverage": "1-of-1",
        "expected_case_outcome_classes": [
            "accepted",
            "semantic-rejected",
            "authentication-failed",
        ],
        "runtime": {
            "holochain": "0.7.0",
            "hdk": "0.7.0",
            "hdi": "0.8.0",
        },
        "cases": {str(index): {} for index in range(14)},
        "unsupported_reference_cases": [
            "wire-signature-valid",
            "nonce-stale",
            "payload-mutation",
        ],
        "claim_ceiling": "ReferenceModelOnly",
    }


def valid_record() -> dict[str, str]:
    return {
        "status": "passed",
        "workflow_run_id": "200",
        "workflow_run_attempt": "1",
        "source_commit": "a" * 40,
        "source_branch": "myc-int-demo-d6u-holochain-07-runtime",
        "source_repository": "Luminous-Dynamics/mycelix",
        "trigger_workflow_run_id": "100",
        "trigger_workflow_run_attempt": "1",
        "trigger_workflow_name": "D6S Canonical Qualification",
        "trigger_workflow_path": ".github/workflows/d6s-canonical-qualification.yml",
        "trigger_workflow_blob_sha": "e" * 40,
        "executor_run_id": "200",
        "executor_run_attempt": "1",
        "executor_workflow_commit_sha": "f" * 40,
        "executor_workflow_ref": "Luminous-Dynamics/mycelix/.github/workflows/d6u-exact-head-runtime-executor.yml@refs/heads/main",
        "executor_workflow_file_path": ".github/workflows/d6u-exact-head-runtime-executor.yml",
        "executor_workflow_repository": "Luminous-Dynamics/mycelix",
        "attestation_status": "deferred",
        "d6s2_manifest_git_blob_sha": "a" * 40,
        "d6s2_fixture_git_blob_sha": "b" * 40,
        "d6s2_authority_ledger_schema": "v1",
        "d6s1_corpus_sha256": "a" * 64,
        "test_log_sha256": "b" * 64,
        "manifest_version": "11",
        "manifest_git_blob_sha": "b" * 40,
        "evidence_verifier_git_blob_sha": "c" * 40,
        "lock_verifier_git_blob_sha": "d" * 40,
        "case_coverage": "14-of-14",
        "supplemental_coverage": "4-of-4",
        "application_check_coverage": "1-of-1",
        "case_outcome_classes": "accepted,semantic-rejected,authentication-failed",
        "runtime": "holochain-0.7.0",
        "hdk": "0.7.0",
        "hdi": "0.8.0",
        "rust": "rustc 1.96.1 (fixture)",
        "rust_verbose_commit": "f" * 40,
        "cargo": "cargo 1.96.1 (fixture)",
        "cargo_lock_sha256": "e" * 64,
        "test": "d6u_authority_boundary:passed",
        "supported_cases": "14",
        "unsupported_cases": "wire-signature-valid,nonce-stale,payload-mutation",
        "claim_ceiling": "ReferenceModelOnly",
    }


def run(candidate_root: pathlib.Path) -> None:
    verifier_path = candidate_root / "scripts/integral/verify_d6u_trusted_artifacts.py"
    assert verifier_path.is_file(), f"candidate verifier missing: {verifier_path}"

    verifier = load_module(verifier_path)

    p = policy()
    record = valid_record()
    verifier.verify_record_metadata(record, p)

    tampered = dict(record)
    tampered["workflow_run_id"] = "201"
    assert_rejected(
        lambda: verifier.verify_record_metadata(tampered, p),
        "candidate verifier accepted aliased workflow/executor run identities",
    )

    tampered = dict(record)
    tampered["executor_run_attempt"] = "2"
    assert_rejected(
        lambda: verifier.verify_record_metadata(tampered, p),
        "candidate verifier accepted mismatched executor run attempt",
    )

    tampered = dict(record)
    tampered["unexpected"] = "attacker-controlled"
    assert_rejected(
        lambda: verifier.verify_record_metadata(tampered, p),
        "candidate verifier accepted an unexpected record field",
    )

    tampered = dict(record)
    del tampered["workflow_run_attempt"]
    assert_rejected(
        lambda: verifier.verify_record_metadata(tampered, p),
        "candidate verifier accepted a missing record field",
    )

    source = verifier_path.read_text(encoding="utf-8")
    assert "if not __debug__:" in source
    assert "trusted D6U program must not run with Python optimization enabled" in source

    probe = "import runpy, sys; runpy.run_path(sys.argv[1], run_name='__independent_opt_probe__')"
    completed = subprocess.run(
        [sys.executable, "-O", "-c", probe, str(verifier_path)],
        capture_output=True,
        text=True,
        check=False,
    )
    assert completed.returncode != 0, "candidate verifier executed under optimized Python"
    assert (
        "trusted D6U program must not run with Python optimization enabled"
        in completed.stderr
    )

    print("independent D6U verifier evaluator: PASS")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--candidate-root", required=True)
    args = parser.parse_args()
    run(pathlib.Path(args.candidate_root).resolve())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
