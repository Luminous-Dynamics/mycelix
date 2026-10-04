#!/usr/bin/env python3
"""Verify the emitted D6U runtime evidence record against its execution subject."""

import hashlib
import json
import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).parents[2]
MANIFEST = ROOT / "docs/integral/d6u-runtime-manifest.json"
D6S2_MANIFEST = ROOT / "docs/integral/d6s-canon-2-manifest.json"
D6S2_FIXTURE = ROOT / "docs/integral/d6s-canon-2-authority-boundary-fixture.json"
D6S1_CORPUS = ROOT / "docs/integral/d6s-canon-1-golden-vectors.json"
WORKFLOW = ROOT / ".github/workflows/d6u-runtime-qualification.yml"
EVIDENCE = ROOT / "d6u-runtime-harness/d6u-runtime-evidence.txt"
TEST_LOG = ROOT / "d6u-runtime-harness/d6u-runtime-test.log"
LOCKFILE = ROOT / "d6u-runtime-harness/Cargo.lock"


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
    assert lines[0] == "D6U HOLOCHAIN 0.7 RUNTIME EVIDENCE"
    record: dict[str, str] = {}
    for line in lines[1:]:
        assert "=" in line, f"malformed evidence record line: {line!r}"
        key, value = line.split("=", 1)
        assert key and key not in record, f"duplicate evidence key: {key!r}"
        record[key] = value
    return record


def main() -> None:
    assert len(sys.argv) == 1, "usage: verify_d6u_runtime_record.py"
    record = load_record(EVIDENCE)
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    d6s2 = json.loads(D6S2_MANIFEST.read_text(encoding="utf-8"))
    test_log_lines = TEST_LOG.read_text(encoding="utf-8").splitlines()
    observed_case_count = sum(
        1 for line in test_log_lines if line.startswith("D6U_CASE\t")
    )
    observed_supplemental_count = sum(
        1
        for line in test_log_lines
        if line.startswith("D6U_SUBSTRATE_CHECK\t")
    )

    expected = {
        "status": "runtime-reference-evidence",
        "source_commit": subprocess.run(
            ["git", "rev-parse", "HEAD"],
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip(),
        "workflow_run_id": os.environ["GITHUB_RUN_ID"],
        "workflow_run_attempt": os.environ["GITHUB_RUN_ATTEMPT"],
        "workflow_sha": git_blob_sha(WORKFLOW),
        "d6s2_manifest_git_blob_sha": git_blob_sha(D6S2_MANIFEST),
        "d6s2_fixture_git_blob_sha": git_blob_sha(D6S2_FIXTURE),
        "d6s2_authority_ledger_schema": "v1",
        "d6s1_corpus_sha256": sha256(D6S1_CORPUS),
        "test_log_sha256": sha256(TEST_LOG),
        "manifest_version": str(manifest["version"]),
        "manifest_git_blob_sha": git_blob_sha(MANIFEST),
        "evidence_verifier_git_blob_sha": git_blob_sha(
            ROOT / manifest["evidence_verifier_path"]
        ),
        "case_coverage": (
            f"{observed_case_count}-of-{len(manifest["supported_reference_cases"])}"
        ),
        "supplemental_coverage": (
            f"{observed_supplemental_count}-of-"
            f"{len(manifest.get("supplemental_substrate_checks", []))}"
        ),
        "case_outcome_classes": ",".join(manifest["evidence_outcome_classes"]),
        "runtime": manifest["substrate"]["holochain"],
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

    assert set(record) == set(expected), (
        f"evidence field mismatch: extra={set(record) - set(expected)}, "
        f"missing={set(expected) - set(record)}"
    )
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
        f"run={record['workflow_run_id']}, attempt={record['workflow_run_attempt']}, "
        f"source={record['source_commit']}"
    )


if __name__ == "__main__":
    main()
