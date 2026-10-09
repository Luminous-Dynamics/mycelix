#!/usr/bin/env python3
"""Independent evaluator for the D6U trusted verifier.

This evaluator is intentionally kept outside the candidate test suite. It loads
the verifier under test as a module and supplies its own policy/record fixtures.
It must remain small, deterministic, and stdlib-only.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import importlib.util
import os
import pathlib
import subprocess
import traceback
import tempfile
import sys
from unittest.mock import patch


def load_module(path: pathlib.Path):
    spec = importlib.util.spec_from_file_location("candidate_d6u_verifier", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"unable to load candidate verifier: {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def assert_rejected(fn, expected_fragment: str, message: str) -> None:
    try:
        fn()
    except AssertionError as exc:
        if expected_fragment not in str(exc):
            frames = traceback.extract_tb(exc.__traceback__)
            source_lines = [frame.line or "" for frame in frames]
            if not any(expected_fragment in line for line in source_lines):
                raise AssertionError(
                    f"{message}: rejection came from the wrong check; "
                    f"expected={expected_fragment!r}, "
                    f"observed_message={str(exc)!r}, source_lines={source_lines!r}"
                ) from exc
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
        "repository_identity": {
            "full_name": "Luminous-Dynamics/mycelix",
            "repository_id": 1176351975,
        },
        "trigger_workflow": {
            "name": "D6S Canonical Qualification",
            "path": ".github/workflows/d6s-canonical-qualification.yml",
            "workflow_id": 371215723,
        },
        "executor_workflow": {
            "name": "D6U Exact-Head Runtime Executor",
            "path": ".github/workflows/d6u-exact-head-runtime-executor.yml",
        },
        "source_branch": "myc-int-demo-d6u-holochain-07-runtime",
        "required_source_blobs": {
            "docs/integral/d6u-runtime-manifest.json": "b" * 40,
            "scripts/integral/verify_d6u_runtime_evidence.py": "c" * 40,
            "scripts/integral/verify_d6u_runtime_lock.py": "d" * 40,
            ".github/workflows/d6s-canonical-qualification.yml": "e" * 40,
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


def git_blob_sha1(content: bytes) -> str:
    header = f"blob {len(content)}\0".encode("utf-8")
    return hashlib.sha1(header + content).hexdigest()


def verify_candidate_policy(
    candidate_policy: dict,
    expected_record_fields: list[str],
    observed_verifier_blob: str,
) -> None:
    assert isinstance(candidate_policy, dict), "candidate trusted policy is not an object"
    assert candidate_policy.get("record_fields") == expected_record_fields, (
        "candidate trusted policy record schema mismatch"
    )
    assert candidate_policy.get("claim_ceiling") == "ReferenceModelOnly", (
        "candidate trusted policy widened claim ceiling"
    )
    runtime = candidate_policy.get("runtime")
    assert isinstance(runtime, dict), "candidate trusted policy runtime is not an object"
    assert runtime.get("holochain") == "0.7.0"
    assert runtime.get("hdk") == "0.7.0"
    assert runtime.get("hdi") == "0.8.0"

    trusted_programs = candidate_policy.get("trusted_programs")
    assert isinstance(trusted_programs, dict), (
        "candidate trusted policy programs are not an object"
    )
    verifier_entry = trusted_programs.get(
        "scripts/integral/verify_d6u_trusted_artifacts.py"
    )
    assert isinstance(verifier_entry, dict), (
        "candidate trusted policy verifier pin is missing"
    )
    assert verifier_entry.get("path") == (
        "scripts/integral/verify_d6u_trusted_artifacts.py"
    )
    assert verifier_entry.get("blob_sha") == observed_verifier_blob, (
        "candidate verifier blob pin mismatch"
    )


def exercise_main_record_guards(
    verifier,
    candidate_root: pathlib.Path,
    candidate_policy: dict,
    record: dict[str, str],
) -> None:
    repository = candidate_policy["repository_identity"]["full_name"]
    valid_record = dict(record)
    valid_record["status"] = "runtime-reference-evidence"
    valid_record["attestation_status"] = "deferred-to-trusted-builder"

    with tempfile.TemporaryDirectory() as scratch:
        scratch_path = pathlib.Path(scratch)
        policy_path = scratch_path / "policy.json"
        policy_path.write_text(
            json.dumps(candidate_policy, sort_keys=True), encoding="utf-8"
        )
        event_path = scratch_path / "event.json"
        executor_cfg = candidate_policy["executor_workflow"]
        repo_id = candidate_policy["repository_identity"]["repository_id"]
        event_path.write_text(
            json.dumps(
                {
                    "repository": {"full_name": repository},
                    "workflow_run": {
                        "name": executor_cfg["name"],
                        "path": executor_cfg["path"],
                        "event": "workflow_run",
                        "conclusion": "success",
                        "repository": {"full_name": repository, "id": repo_id},
                        "head_repository": {"full_name": repository, "id": repo_id},
                        "head_branch": "main",
                        "head_sha": valid_record["executor_workflow_commit_sha"],
                        "id": int(valid_record["executor_run_id"]),
                        "run_attempt": int(valid_record["executor_run_attempt"]),
                    },
                }
            ),
            encoding="utf-8",
        )
        artifact_dir = scratch_path / "artifacts"
        artifact_dir.mkdir()
        evidence_path = artifact_dir / "d6u-runtime-evidence.txt"
        test_log_path = artifact_dir / "d6u-runtime-test.log"
        lock_path = artifact_dir / "Cargo.lock"
        for path in (evidence_path, test_log_path, lock_path):
            path.write_text("fixture\n", encoding="utf-8")

        manifest_bytes = b"[package]\nname='fixture'\nversion='0.1.0'\nedition='2021'\n"
        expected_manifest_blob = candidate_policy["required_source_blobs"][
            candidate_policy["lock_graph"]["manifest_path"]
        ]

        def invoke_main(observed_record: dict[str, str]) -> None:
            with (
                patch.object(verifier, "POLICY", policy_path),
                patch.dict(
                    os.environ,
                    {
                        "GITHUB_EVENT_PATH": str(event_path),
                        "GITHUB_REPOSITORY": repository,
                        "GITHUB_TOKEN": "fixture-token",
                        "GITHUB_WORKFLOW_REF": (
                            f"{repository}/.github/workflows/"
                            "d6u-trusted-evidence-attestation.yml@refs/heads/main"
                        ),
                        "GITHUB_WORKFLOW_SHA": "c" * 40,
                    },
                    clear=True,
                ),
                patch.object(sys, "argv", [str(candidate_root), str(artifact_dir)]),
                patch.object(verifier, "verify_trusted_workflow_identity"),
                patch.object(verifier, "verify_executor_workflow_against_run_head"),
                patch.object(verifier, "verify_artifact_layout"),
                patch.object(verifier, "verify_artifact_size_limits"),
                patch.object(verifier, "load_record", return_value=observed_record),
                patch.object(verifier, "verify_record_metadata"),
                patch.object(verifier, "verify_executor_workflow_identity"),
                patch.object(
                    verifier,
                    "verify_trigger_run",
                    return_value={"head_sha": observed_record["source_commit"]},
                ),
                patch.object(verifier, "sha256", return_value="b" * 64),
                patch.object(verifier, "verify_cases"),
                patch.object(verifier, "contents_bytes_from_api", return_value=manifest_bytes),
                patch.object(verifier, "_git_blob_sha1", return_value=expected_manifest_blob),
                patch.object(verifier, "verify_lock"),
            ):
                verifier.main()

        invoke_main(valid_record)

        bad_status = dict(valid_record)
        bad_status["status"] = "passed"
        assert_rejected(
            lambda: invoke_main(bad_status),
            "runtime evidence status mismatch",
            "candidate verifier accepted a record with the wrong main-path status",
        )

        bad_handoff = dict(valid_record)
        bad_handoff["attestation_status"] = "passed"
        assert_rejected(
            lambda: invoke_main(bad_handoff),
            "trusted-builder handoff status mismatch",
            "candidate verifier accepted an incorrect trusted-builder handoff status",
        )


def run(candidate_root: pathlib.Path) -> None:
    verifier_path = candidate_root / "scripts/integral/verify_d6u_trusted_artifacts.py"
    assert verifier_path.is_file(), f"candidate verifier missing: {verifier_path}"

    verifier = load_module(verifier_path)

    p = policy()
    policy_path = candidate_root / "docs/integral/d6u-trusted-builder-policy.json"
    candidate_policy = json.loads(policy_path.read_text(encoding="utf-8"))
    observed_verifier_blob = git_blob_sha1(verifier_path.read_bytes())
    verify_candidate_policy(candidate_policy, p["record_fields"], observed_verifier_blob)

    tampered_policy = dict(candidate_policy)
    tampered_policy["record_fields"] = list(candidate_policy["record_fields"]) + ["extra"]
    assert_rejected(
        lambda: verify_candidate_policy(
            tampered_policy, p["record_fields"], observed_verifier_blob
        ),
        "candidate trusted policy record schema mismatch",
        "candidate policy accepted an extra runtime record field",
    )

    tampered_policy = dict(candidate_policy)
    tampered_policy["claim_ceiling"] = "OperationallyQualified"
    assert_rejected(
        lambda: verify_candidate_policy(
            tampered_policy, p["record_fields"], observed_verifier_blob
        ),
        "candidate trusted policy widened claim ceiling",
        "candidate policy accepted a widened claim ceiling",
    )

    tampered_policy = dict(candidate_policy)
    tampered_programs = dict(candidate_policy["trusted_programs"])
    tampered_verifier_entry = dict(
        tampered_programs["scripts/integral/verify_d6u_trusted_artifacts.py"]
    )
    tampered_verifier_entry["blob_sha"] = "0" * 40
    tampered_programs["scripts/integral/verify_d6u_trusted_artifacts.py"] = (
        tampered_verifier_entry
    )
    tampered_policy["trusted_programs"] = tampered_programs
    assert_rejected(
        lambda: verify_candidate_policy(
            tampered_policy, p["record_fields"], observed_verifier_blob
        ),
        "candidate verifier blob pin mismatch",
        "candidate policy accepted a verifier blob pin mismatch",
    )

    record = valid_record()
    verifier.verify_record_metadata(record, p)

    exercise_main_record_guards(verifier, candidate_root, candidate_policy, record)

    repository = "Luminous-Dynamics/mycelix"
    trigger_run = {
        "name": "D6S Canonical Qualification",
        "path": ".github/workflows/d6s-canonical-qualification.yml",
        "workflow_id": 371215723,
        "event": "pull_request",
        "conclusion": "success",
        "head_repository": {"full_name": repository, "id": 1176351975},
        "repository": {"full_name": repository, "id": 1176351975},
        "head_branch": "myc-int-demo-d6u-holochain-07-runtime",
        "head_sha": "a" * 40,
        "id": 100,
        "run_attempt": 1,
    }
    verifier.verify_trigger_run_record(record, trigger_run, p, repository)

    tampered_trigger = dict(trigger_run)
    tampered_trigger["head_sha"] = "0" * 40
    assert_rejected(
        lambda: verifier.verify_trigger_run_record(
            record, tampered_trigger, p, repository
        ),
        'trigger["head_sha"] == record["source_commit"]',
        "candidate verifier accepted a D6S trigger run for a different source SHA",
    )

    tampered_trigger = dict(trigger_run)
    tampered_trigger["workflow_id"] = 999
    assert_rejected(
        lambda: verifier.verify_trigger_run_record(
            record, tampered_trigger, p, repository
        ),
        'int(trigger["workflow_id"]) == int(cfg["workflow_id"])',
        "candidate verifier accepted a different trigger workflow identity",
    )

    executor_run = {
        "name": "D6U Exact-Head Runtime Executor",
        "path": ".github/workflows/d6u-exact-head-runtime-executor.yml",
        "event": "workflow_run",
        "conclusion": "success",
        "repository": {"full_name": repository, "id": 1176351975},
        "head_repository": {"full_name": repository, "id": 1176351975},
        "head_branch": "main",
        "head_sha": "f" * 40,
        "id": 200,
        "run_attempt": 1,
    }
    verifier.verify_executor_run_event(executor_run, p, repository)
    verifier.verify_executor_run_record(executor_run, record, p, repository)

    tampered_event = dict(executor_run)
    tampered_event["name"] = "Untrusted Workflow"
    assert_rejected(
        lambda: verifier.verify_executor_run_event(
            tampered_event, p, repository
        ),
        "executor workflow name mismatch",
        "candidate verifier accepted an unexpected executor workflow name",
    )

    tampered_event = dict(executor_run)
    tampered_event["id"] = True
    assert_rejected(
        lambda: verifier.verify_executor_run_event(
            tampered_event, p, repository
        ),
        "executor id is not a positive integer",
        "candidate verifier accepted a boolean executor run ID",
    )

    tampered_executor = dict(executor_run)
    tampered_executor["head_sha"] = "0" * 40
    assert_rejected(
        lambda: verifier.verify_executor_run_record(
            tampered_executor, record, p, repository
        ),
        "executor workflow commit does not match evidence record",
        "candidate verifier accepted an executor run with a different workflow commit",
    )

    tampered_executor = dict(executor_run)
    tampered_executor["run_attempt"] = 2
    assert_rejected(
        lambda: verifier.verify_executor_run_record(
            tampered_executor, record, p, repository
        ),
        "executor run attempt does not match evidence record",
        "candidate verifier accepted an executor run from a different attempt",
    )

    record_header = "D6U HOLOCHAIN 0.7 RUNTIME EVIDENCE\n"
    with tempfile.TemporaryDirectory() as scratch:
        scratch_path = pathlib.Path(scratch)
        valid_record_path = scratch_path / "valid-record.txt"
        valid_record_path.write_text(
            record_header + "status=passed\nworkflow_run_id=200\n",
            encoding="utf-8",
        )
        assert verifier.load_record(valid_record_path) == {
            "status": "passed",
            "workflow_run_id": "200",
        }

        duplicate_record_path = scratch_path / "duplicate-record.txt"
        duplicate_record_path.write_text(
            record_header + "status=passed\nstatus=failed\n",
            encoding="utf-8",
        )
        assert_rejected(
            lambda: verifier.load_record(duplicate_record_path),
            "duplicate evidence key",
            "candidate verifier accepted duplicate record keys",
        )

        malformed_record_path = scratch_path / "malformed-record.txt"
        malformed_record_path.write_text(
            record_header + "not-a-key-value-line\n",
            encoding="utf-8",
        )
        assert_rejected(
            lambda: verifier.load_record(malformed_record_path),
            "malformed evidence record line",
            "candidate verifier accepted a malformed record line",
        )

        wrong_header_path = scratch_path / "wrong-header.txt"
        wrong_header_path.write_text(
            "UNTRUSTED HEADER\nstatus=passed\n",
            encoding="utf-8",
        )
        assert_rejected(
            lambda: verifier.load_record(wrong_header_path),
            "D6U HOLOCHAIN 0.7 RUNTIME EVIDENCE",
            "candidate verifier accepted an unexpected record header",
        )

        artifact_dir = scratch_path / "artifact-valid"
        artifact_dir.mkdir()
        (artifact_dir / "d6u-runtime-evidence.txt").write_text(
            "inert evidence fixture\n", encoding="utf-8"
        )
        verifier.verify_artifact_layout(
            artifact_dir,
            {"d6u-runtime-evidence.txt"},
            max_entries=2,
        )

        extra_file = artifact_dir / "unexpected.txt"
        extra_file.write_text("extra\n", encoding="utf-8")
        assert_rejected(
            lambda: verifier.verify_artifact_layout(
                artifact_dir,
                {"d6u-runtime-evidence.txt"},
                max_entries=4,
            ),
            "unexpected trusted input files",
            "candidate verifier accepted an extra artifact file",
        )

        symlink_dir = scratch_path / "artifact-symlink"
        symlink_dir.mkdir()
        (symlink_dir / "link").symlink_to(valid_record_path)
        assert_rejected(
            lambda: verifier.verify_artifact_layout(
                symlink_dir, {"link"}, max_entries=2
            ),
            "trusted artifact contains symlink",
            "candidate verifier accepted a symlink artifact member",
        )

        nested_dir = scratch_path / "artifact-nested"
        nested_dir.mkdir()
        (nested_dir / "nested").mkdir()
        assert_rejected(
            lambda: verifier.verify_artifact_layout(
                nested_dir, set(), max_entries=4
            ),
            "trusted artifact contains nested directory",
            "candidate verifier accepted nested artifact directories",
        )

    tampered = dict(record)
    tampered["workflow_run_id"] = "201"
    assert_rejected(
        lambda: verifier.verify_record_metadata(tampered, p),
        'record["workflow_run_id"] == record["executor_run_id"]',
        "candidate verifier accepted aliased workflow/executor run identities",
    )

    tampered = dict(record)
    tampered["executor_run_attempt"] = "2"
    assert_rejected(
        lambda: verifier.verify_record_metadata(tampered, p),
        'record["workflow_run_attempt"] == record["executor_run_attempt"]',
        "candidate verifier accepted mismatched executor run attempt",
    )

    tampered = dict(record)
    tampered["workflow_run_id"] = "0200"
    assert_rejected(
        lambda: verifier.verify_record_metadata(tampered, p),
        "runtime evidence run identity is not canonical",
        "candidate verifier accepted a noncanonical workflow run ID",
    )

    tampered = dict(record)
    tampered["executor_run_attempt"] = "0"
    assert_rejected(
        lambda: verifier.verify_record_metadata(tampered, p),
        "runtime evidence run identity is not canonical",
        "candidate verifier accepted a zero executor run attempt",
    )

    tampered = dict(record)
    tampered["trigger_workflow_run_id"] = "0100"
    assert_rejected(
        lambda: verifier.verify_record_metadata(tampered, p),
        "runtime evidence run identity is not canonical",
        "candidate verifier accepted a noncanonical trigger run ID",
    )

    tampered = dict(record)
    tampered["trigger_workflow_run_attempt"] = "01"
    assert_rejected(
        lambda: verifier.verify_record_metadata(tampered, p),
        "runtime evidence run identity is not canonical",
        "candidate verifier accepted a noncanonical trigger run attempt",
    )

    tampered = dict(record)
    tampered["claim_ceiling"] = "OperationallyQualified"
    assert_rejected(
        lambda: verifier.verify_record_metadata(tampered, p),
        'record["claim_ceiling"] == policy["claim_ceiling"]',
        "candidate verifier accepted a widened claim ceiling",
    )

    duplicated_policy = policy()
    duplicated_policy["record_fields"].append("workflow_run_id")
    assert_rejected(
        lambda: verifier.verify_record_metadata(record, duplicated_policy),
        'len(expected_record_fields) == len(policy["record_fields"])',
        "candidate verifier accepted duplicate policy record fields",
    )

    tampered = dict(record)
    tampered["unexpected"] = "attacker-controlled"
    assert_rejected(
        lambda: verifier.verify_record_metadata(tampered, p),
        "runtime evidence record schema mismatch",
        "candidate verifier accepted an unexpected record field",
    )

    tampered = dict(record)
    del tampered["workflow_run_attempt"]
    assert_rejected(
        lambda: verifier.verify_record_metadata(tampered, p),
        "runtime evidence record schema mismatch",
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
