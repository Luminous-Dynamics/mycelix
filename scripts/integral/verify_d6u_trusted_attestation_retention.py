#!/usr/bin/env python3
"""Validate a retained D6U attestation/offline-verification evidence packet.

Cryptographic verification remains the responsibility of gh attestation verify.
This verifier makes the retained packet itself deterministic and fail-closed.
"""

from __future__ import annotations

import hashlib
import json
import os
import sys
from pathlib import Path


SCHEMA = "d6u-attestation-retention/v1"
SUBJECTS = (
    "d6u-runtime-evidence.txt",
    "d6u-runtime-test.log",
    "Cargo.lock",
)
FILES = (
    "d6u-runtime-evidence.attestation.jsonl",
    "d6u-runtime-test.attestation.jsonl",
    "Cargo_lock.attestation.jsonl",
    "d6u-runtime-evidence.offline.json",
    "d6u-runtime-test.offline.json",
    "Cargo_lock.offline.json",
    "trusted_root.jsonl",
    "retention-transcript.json",
)
MAX_FILE_BYTES = 4 * 1024 * 1024
MAX_ROOT_BYTES = 2 * 1024 * 1024
MAX_TOTAL_BYTES = 18 * 1024 * 1024
MAX_JSONL_LINES = 64
RETAINED_FILES = tuple(name for name in FILES if name != "retention-transcript.json")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_json(path: Path) -> object:
    return json.loads(path.read_text(encoding="utf-8"))


def load_jsonl(path: Path) -> list[dict]:
    lines = path.read_text(encoding="utf-8").splitlines()
    assert 1 <= len(lines) <= MAX_JSONL_LINES
    rows = []
    for line in lines:
        value = json.loads(line)
        assert isinstance(value, dict)
        rows.append(value)
    return rows


def assert_hash_record(base: Path, filename: str, expected: dict) -> None:
    assert isinstance(expected, dict)
    assert set(expected) == {"sha256", "bytes"}
    path = base / filename
    assert path.is_file() and not path.is_symlink()
    observed_bytes = path.stat().st_size
    assert observed_bytes == expected["bytes"]
    assert 0 <= observed_bytes <= MAX_FILE_BYTES
    assert expected["sha256"] == sha256(path)
    assert isinstance(expected["sha256"], str) and len(expected["sha256"]) == 64
    assert all(ch in "0123456789abcdef" for ch in expected["sha256"])


def current_run_uri(repository: str, run_id: str, run_attempt: str) -> str:
    return (
        "https://github.com/" + repository + "/actions/runs/"
        + run_id + "/attempts/" + run_attempt
    )


def verify_report(
    path: Path,
    subject_name: str,
    subject_digest: str,
    expected_run_uri: str,
    predicate_type: str,
) -> None:
    report = load_json(path)
    assert isinstance(report, list) and len(report) >= 1
    matches = []
    for entry in report:
        assert isinstance(entry, dict)
        result = entry.get("verificationResult")
        assert isinstance(result, dict)
        statement = result.get("statement")
        assert isinstance(statement, dict)
        assert statement.get("predicateType") == predicate_type

        certificate = result.get("signature", {}).get("certificate", {})
        assert isinstance(certificate, dict)
        assert certificate.get("runInvocationURI") == expected_run_uri

        timestamps = result.get("verifiedTimestamps", [])
        assert isinstance(timestamps, list)
        assert any(
            isinstance(timestamp, dict) and timestamp.get("type") == "Tlog"
            for timestamp in timestamps
        )

        subjects = statement.get("subject")
        assert isinstance(subjects, list)
        assert any(
            isinstance(subject, dict)
            and subject.get("name") == subject_name
            and isinstance(subject.get("digest"), dict)
            and subject["digest"].get("sha256") == subject_digest
            for subject in subjects
        )
        matches.append(entry)

    assert len(matches) == 1


def main() -> None:
    if len(sys.argv) != 2:
        raise SystemExit(
            "usage: verify_d6u_trusted_attestation_retention.py RETENTION_DIR"
        )

    root = Path(sys.argv[1])
    assert root.is_dir() and not root.is_symlink()

    actual = sorted(path.name for path in root.iterdir())
    assert actual == sorted(FILES), f"unexpected retention packet files: {actual}"

    total = sum((root / name).stat().st_size for name in actual)
    assert total <= MAX_TOTAL_BYTES
    for name in FILES:
        path = root / name
        assert path.is_file() and not path.is_symlink()
        if name == "trusted_root.jsonl":
            assert path.stat().st_size <= MAX_ROOT_BYTES
        else:
            assert path.stat().st_size <= MAX_FILE_BYTES

    transcript = load_json(root / "retention-transcript.json")
    assert isinstance(transcript, dict)
    required = {
        "schema", "policy_version", "repository", "source_ref", "source_digest",
        "signer_workflow", "signer_workflow_digest", "certificate_identity",
        "certificate_oidc_issuer", "run_id", "run_attempt", "run_invocation_uri",
        "cli_version", "predicate_type", "predicate_schema", "claim_ceiling",
        "public_good_instance_required", "tlog_required", "offline_verified",
        "no_public_good_rejected", "subjects", "retained_files",
    }
    assert set(transcript) == required
    assert transcript["schema"] == SCHEMA
    assert transcript["policy_version"] == int(os.environ["D6U_TRUSTED_POLICY_VERSION"])
    assert transcript["repository"] == os.environ["GITHUB_REPOSITORY"]
    assert transcript["source_ref"] == os.environ["GITHUB_REF"]
    assert transcript["source_digest"] == os.environ["GITHUB_SHA"]
    assert transcript["signer_workflow"] == (
        os.environ["GITHUB_REPOSITORY"]
        + "/.github/workflows/d6u-trusted-evidence-attestation.yml"
    )
    assert transcript["signer_workflow_digest"] == os.environ["GITHUB_WORKFLOW_SHA"]
    assert transcript["certificate_identity"] == (
        "https://github.com/"
        + os.environ["GITHUB_REPOSITORY"]
        + "/.github/workflows/d6u-trusted-evidence-attestation.yml@refs/heads/main"
    )
    assert transcript["certificate_oidc_issuer"] == (
        "https://token.actions.githubusercontent.com"
    )
    assert str(transcript["run_id"]) == os.environ["GITHUB_RUN_ID"]
    assert str(transcript["run_attempt"]) == os.environ["GITHUB_RUN_ATTEMPT"]
    assert transcript["run_invocation_uri"] == current_run_uri(
        os.environ["GITHUB_REPOSITORY"],
        os.environ["GITHUB_RUN_ID"],
        os.environ["GITHUB_RUN_ATTEMPT"],
    )
    assert transcript["cli_version"] == "2.101.0"
    assert transcript["predicate_type"] == (
        "https://luminousdynamics.io/attestations/d6u-runtime-evidence/v1"
    )
    assert transcript["predicate_schema"] == "d6u-trusted-runtime-evidence/v1"
    assert transcript["claim_ceiling"] == "ReferenceModelOnly"
    assert transcript["public_good_instance_required"] is True
    assert transcript["tlog_required"] is True
    assert transcript["offline_verified"] is True
    assert transcript["no_public_good_rejected"] is True

    subjects = transcript["subjects"]
    assert isinstance(subjects, list) and len(subjects) == len(SUBJECTS)
    by_name = {item["name"]: item for item in subjects}
    assert set(by_name) == set(SUBJECTS)
    for item in subjects:
        assert set(item) == {"name", "sha256"}
        assert isinstance(item["sha256"], str) and len(item["sha256"]) == 64
        assert all(ch in "0123456789abcdef" for ch in item["sha256"])

    expected_subject_env = {
        "d6u-runtime-evidence.txt": "D6U_SUBJECT_D6U_RUNTIME_EVIDENCE_TXT_SHA256",
        "d6u-runtime-test.log": "D6U_SUBJECT_D6U_RUNTIME_TEST_LOG_SHA256",
        "Cargo.lock": "D6U_SUBJECT_CARGO_LOCK_SHA256",
    }
    for subject_name, env_name in expected_subject_env.items():
        assert by_name[subject_name]["sha256"] == os.environ[env_name]

    retained = transcript["retained_files"]
    assert isinstance(retained, dict)
    assert set(retained) == set(RETAINED_FILES)

    for name in RETAINED_FILES:
        assert_hash_record(root, name, retained[name])

    for subject_name in SUBJECTS:
        safe = subject_name.replace(".", "_").replace("-", "_")
        bundle_name = safe + ".attestation.jsonl"
        report_name = safe + ".offline.json"
        load_jsonl(root / bundle_name)
        verify_report(
            root / report_name,
            subject_name,
            by_name[subject_name]["sha256"],
            transcript["run_invocation_uri"],
            transcript["predicate_type"],
        )


if __name__ == "__main__":
    main()
