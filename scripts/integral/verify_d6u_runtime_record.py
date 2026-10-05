#!/usr/bin/env python3
"""Verify D6U runtime evidence against the upstream D6S run and executor context."""

import hashlib
import json
import os
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).parents[2]
MANIFEST = ROOT / "docs/integral/d6u-runtime-manifest.json"
D6S2_MANIFEST = ROOT / "docs/integral/d6s-canon-2-manifest.json"
D6S2_FIXTURE = ROOT / "docs/integral/d6s-canon-2-authority-boundary-fixture.json"
D6S1_CORPUS = ROOT / "docs/integral/d6s-canon-1-golden-vectors.json"
TRIGGER_WORKFLOW = ROOT / ".github/workflows/d6s-canonical-qualification.yml"
EVIDENCE = ROOT / "d6u-runtime-harness/d6u-runtime-evidence.txt"
TEST_LOG = ROOT / "d6u-runtime-harness/d6u-runtime-test.log"
LOCKFILE = ROOT / "d6u-runtime-harness/Cargo.lock"

EXPECTED_TRIGGER_WORKFLOW_NAME = "D6S Canonical Qualification"
EXPECTED_TRIGGER_WORKFLOW_PATH = ".github/workflows/d6s-canonical-qualification.yml"
EXPECTED_EXECUTOR_WORKFLOW_NAME = "D6U Exact-Head Runtime Executor"
EXPECTED_EXECUTOR_WORKFLOW_PATH = ".github/workflows/d6u-exact-head-runtime-executor.yml"
EXPECTED_SOURCE_BRANCH = "myc-int-demo-d6u-holochain-07-runtime"

EXPECTED_RECORD_FIELDS = {
    "status",
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
}


def git_blob_sha(path: Path) -> str:
    return subprocess.run(
        ["git", "hash-object", str(path)],
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_record(path: Path) -> dict[str, str]:
    lines = path.read_text(encoding="utf-8").splitlines()
    assert lines and lines[0] == "D6U HOLOCHAIN 0.7 RUNTIME EVIDENCE"
    record: dict[str, str] = {}
    for line in lines[1:]:
        assert "=" in line, f"malformed evidence record line: {line!r}"
        key, value = line.split("=", 1)
        assert key and key not in record, f"duplicate evidence key: {key!r}"
        record[key] = value
    return record


def run_sha(event: dict) -> str:
    upstream = event["workflow_run"]
    sha = upstream["head_sha"]
    assert re.fullmatch(r"[0-9a-f]{40}", sha)
    return sha


def assert_identity(event: dict, record: dict[str, str], repository: str) -> None:
    upstream = event["workflow_run"]

    assert upstream["name"] == EXPECTED_TRIGGER_WORKFLOW_NAME
    assert upstream["path"] == EXPECTED_TRIGGER_WORKFLOW_PATH
    assert upstream["event"] == "pull_request"
    assert upstream["conclusion"] == "success"
    assert upstream["head_repository"]["full_name"] == repository
    assert upstream["head_branch"] == EXPECTED_SOURCE_BRANCH
    assert record["trigger_workflow_name"] == upstream["name"]
    assert record["trigger_workflow_path"] == upstream["path"]
    assert record["trigger_workflow_run_id"] == str(upstream["id"])
    assert record["trigger_workflow_run_attempt"] == str(upstream["run_attempt"])
    assert record["source_commit"] == upstream["head_sha"]
    assert record["source_branch"] == upstream["head_branch"]
    assert record["source_repository"] == upstream["head_repository"]["full_name"]

    assert record["executor_run_id"] == os.environ["GITHUB_RUN_ID"]
    assert record["executor_run_attempt"] == os.environ["GITHUB_RUN_ATTEMPT"]
    assert record["executor_workflow_commit_sha"] == os.environ["GITHUB_WORKFLOW_SHA"]
    assert record["executor_workflow_ref"] == os.environ["GITHUB_WORKFLOW_REF"]
    assert record["executor_workflow_repository"] == repository
    assert record["executor_workflow_file_path"] == EXPECTED_EXECUTOR_WORKFLOW_PATH

    assert re.fullmatch(r"[0-9a-f]{40}", record["executor_workflow_commit_sha"])
    assert record["executor_workflow_ref"].startswith(
        f"{repository}/{EXPECTED_EXECUTOR_WORKFLOW_PATH}@"
    )
    assert re.fullmatch(r"[0-9a-f]{40}", record["trigger_workflow_blob_sha"])
    assert record["trigger_workflow_blob_sha"] == git_blob_sha(TRIGGER_WORKFLOW)


def main() -> None:
    assert len(sys.argv) == 1, "usage: verify_d6u_runtime_record.py"
    event = json.loads(
        Path(os.environ["GITHUB_EVENT_PATH"]).read_text(encoding="utf-8")
    )
    record = load_record(EVIDENCE)
    repository = os.environ["GITHUB_REPOSITORY"]
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    d6s2 = json.loads(D6S2_MANIFEST.read_text(encoding="utf-8"))
    test_log_lines = TEST_LOG.read_text(encoding="utf-8").splitlines()

    expected_source = run_sha(event)
    actual_head = subprocess.run(
        ["git", "rev-parse", "HEAD"],
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()

    observed_case_count = sum(
        1 for line in test_log_lines if line.startswith("D6U_CASE\t")
    )
    observed_supplemental_count = sum(
        1 for line in test_log_lines if line.startswith("D6U_SUBSTRATE_CHECK\t")
    )
    observed_application_count = sum(
        1 for line in test_log_lines if line.startswith("D6U_APPLICATION_CHECK\t")
    )

    d6s2_ledger_schema = f"v{d6s2['version']}"
    case_coverage = (
        f"{observed_case_count}-of-{len(manifest['supported_reference_cases'])}"
    )
    supplemental_coverage = (
        f"{observed_supplemental_count}-of-"
        f"{len(manifest.get('supplemental_substrate_checks', []))}"
    )
    application_check_coverage = (
        f"{observed_application_count}-of-"
        f"{len(manifest.get('supplemental_application_checks', []))}"
    )

    assert actual_head == expected_source, (
        f"executor checkout drift: expected {expected_source}, observed {actual_head}"
    )
    assert set(record) == EXPECTED_RECORD_FIELDS, (
        f"evidence field mismatch: extra={set(record) - EXPECTED_RECORD_FIELDS}, "
        f"missing={EXPECTED_RECORD_FIELDS - set(record)}"
    )

    assert_identity(event, record, repository)

    expected = {
        "status": "runtime-reference-evidence",
        "source_commit": actual_head,
        "source_branch": record["source_branch"],
        "source_repository": repository,
        "trigger_workflow_run_id": record["trigger_workflow_run_id"],
        "trigger_workflow_run_attempt": record["trigger_workflow_run_attempt"],
        "trigger_workflow_name": EXPECTED_TRIGGER_WORKFLOW_NAME,
        "trigger_workflow_path": EXPECTED_TRIGGER_WORKFLOW_PATH,
        "trigger_workflow_blob_sha": git_blob_sha(TRIGGER_WORKFLOW),
        "executor_run_id": os.environ["GITHUB_RUN_ID"],
        "executor_run_attempt": os.environ["GITHUB_RUN_ATTEMPT"],
        "executor_workflow_commit_sha": os.environ["GITHUB_WORKFLOW_SHA"],
        "executor_workflow_ref": os.environ["GITHUB_WORKFLOW_REF"],
        "executor_workflow_file_path": EXPECTED_EXECUTOR_WORKFLOW_PATH,
        "executor_workflow_repository": repository,
        "attestation_status": manifest["attestation_policy"]["mode"],
        "d6s2_manifest_git_blob_sha": git_blob_sha(D6S2_MANIFEST),
        "d6s2_fixture_git_blob_sha": git_blob_sha(D6S2_FIXTURE),
        "d6s2_authority_ledger_schema": d6s2_ledger_schema,
        "d6s1_corpus_sha256": sha256(D6S1_CORPUS),
        "test_log_sha256": sha256(TEST_LOG),
        "manifest_version": str(manifest["version"]),
        "manifest_git_blob_sha": git_blob_sha(MANIFEST),
        "evidence_verifier_git_blob_sha": git_blob_sha(
            ROOT / manifest["evidence_verifier_path"]
        ),
        "lock_verifier_git_blob_sha": git_blob_sha(
            ROOT / manifest["lock_verifier_path"]
        ),
        "case_coverage": case_coverage,
        "supplemental_coverage": supplemental_coverage,
        "application_check_coverage": application_check_coverage,
        "case_outcome_classes": ",".join(manifest["evidence_outcome_classes"]),
        "runtime": f"holochain-{manifest['substrate']['holochain']}",
        "hdk": manifest["substrate"]["hdk"],
        "hdi": manifest["substrate"]["hdi"],
        "rust": subprocess.run(
            ["rustc", "--version"],
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip(),
        "rust_verbose_commit": subprocess.run(
            ["rustc", "--version", "--verbose"],
            check=True,
            capture_output=True,
            text=True,
        ).stdout.split("commit-hash: ", 1)[1].splitlines()[0],
        "cargo": subprocess.run(
            ["cargo", "--version"],
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip(),
        "cargo_lock_sha256": sha256(LOCKFILE),
        "test": "d6u_authority_boundary:passed",
        "supported_cases": str(len(manifest["supported_reference_cases"])),
        "unsupported_cases": ",".join(manifest["unsupported_reference_cases"]),
        "claim_ceiling": manifest["claim_ceiling"],
    }

    mismatches = {
        key: {"record": record[key], "expected": value}
        for key, value in expected.items()
        if record[key] != value
    }
    assert not mismatches, "evidence record mismatch: " + repr(mismatches)

    assert d6s2["profile"] == "D6S-CANON-2"
    assert d6s2["claim_ceiling"] == "ReferenceModelOnly"
    assert d6s2["runtime_evidence"]["status"] == "NotExecuted"

    print(
        "verified emitted D6U runtime evidence record: "
        f"trigger_run={record['trigger_workflow_run_id']}, "
        f"executor_run={record['executor_run_id']}, "
        f"source={record['source_commit']}"
    )


if __name__ == "__main__":
    main()
