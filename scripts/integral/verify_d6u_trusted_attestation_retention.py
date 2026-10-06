#!/usr/bin/env python3
"""Validate a retained D6U attestation/offline-verification evidence packet.

Cryptographic verification remains the responsibility of gh attestation verify.
This verifier makes the retained packet itself deterministic and fail-closed,
including explicit online/offline/negative-control cross-links.
"""

from __future__ import annotations

import base64
import hashlib
import json
import os
import sys
from pathlib import Path


if not __debug__:
    raise RuntimeError("trusted D6U program must not run with Python optimization enabled")


SCHEMA = "d6u-attestation-retention/v2"
CONTROL_SCHEMA = "d6u-no-public-good-control/v1"
SUBJECTS = (
    "d6u-runtime-evidence.txt",
    "d6u-runtime-test.log",
    "Cargo.lock",
)
FILES = (
    "d6u-runtime-evidence.attestation.jsonl",
    "d6u-runtime-test.attestation.jsonl",
    "Cargo_lock.attestation.jsonl",
    "d6u-runtime-evidence.online.json",
    "d6u-runtime-test.online.json",
    "Cargo_lock.online.json",
    "d6u-runtime-evidence.offline.json",
    "d6u-runtime-test.offline.json",
    "Cargo_lock.offline.json",
    "d6u-runtime-evidence.no-public-good.json",
    "d6u-runtime-test.no-public-good.json",
    "Cargo_lock.no-public-good.json",
    "d6u-trusted-evidence-predicate.json",
    "trusted_root.jsonl",
    "retention-transcript.json",
)
MAX_FILE_BYTES = 4 * 1024 * 1024
MAX_ROOT_BYTES = 2 * 1024 * 1024
MAX_TOTAL_BYTES = 18 * 1024 * 1024
MAX_JSONL_LINES = 64
RETAINED_FILES = tuple(name for name in FILES if name != "retention-transcript.json")
PREDICATE_TYPE = "https://luminousdynamics.io/attestations/d6u-runtime-evidence/v1"
CANONICAL_PREDICATE_SCHEMA = "d6u-trusted-runtime-evidence/v1"
ATTESTATION_PREDICATE_SCHEMA = "d6u-trusted-runtime-evidence-attestation/v2"
PUBLIC_GOOD_INSTANCE = "sigstore-public-good"
SIGNER_WORKFLOW_SUFFIX = "/.github/workflows/d6u-trusted-evidence-attestation.yml"
CERT_OIDC_ISSUER = "https://token.actions.githubusercontent.com"


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
    assert isinstance(expected["bytes"], int)
    path = base / filename
    assert path.is_file() and not path.is_symlink()
    observed_bytes = path.stat().st_size
    assert observed_bytes == expected["bytes"]
    limit = MAX_ROOT_BYTES if filename == "trusted_root.jsonl" else MAX_FILE_BYTES
    assert 0 < observed_bytes <= limit
    assert isinstance(expected["sha256"], str) and len(expected["sha256"]) == 64
    assert all(ch in "0123456789abcdef" for ch in expected["sha256"])
    assert expected["sha256"] == sha256(path)


def current_run_uri(repository: str, run_id: str, run_attempt: str) -> str:
    return (
        "https://github.com/" + repository + "/actions/runs/"
        + run_id + "/attempts/" + run_attempt
    )


def expected_verify_command(
    subject_path: str,
    repository: str,
    root_path: str | None = None,
    bundle_path: str | None = None,
    no_public_good: bool = False,
) -> list[str]:
    command = [
        "gh",
        "attestation",
        "verify",
        subject_path,
        "--repo",
        repository,
        "--limit",
        "8",
    ]
    if bundle_path is not None:
        command.extend(["--bundle", bundle_path])
    if root_path is not None:
        command.extend(["--custom-trusted-root", root_path])
    command.extend(
        [
            "--signer-workflow",
            repository + SIGNER_WORKFLOW_SUFFIX,
            "--signer-digest",
            os.environ["GITHUB_WORKFLOW_SHA"],
            "--cert-identity",
            "https://github.com/" + repository + SIGNER_WORKFLOW_SUFFIX + "@refs/heads/main",
            "--cert-oidc-issuer",
            CERT_OIDC_ISSUER,
            "--source-digest",
            os.environ["GITHUB_SHA"],
            "--source-ref",
            os.environ["GITHUB_REF"],
            "--predicate-type",
            PREDICATE_TYPE,
            "--deny-self-hosted-runners",
        ]
    )
    if no_public_good:
        command.append("--no-public-good")
    command.append("--format=json")
    return command


def canonical_json_sha256(value: object) -> str:
    encoded = json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def verify_report(
    path: Path,
    expected_subjects: list[tuple[str, str]],
    expected_context: dict[str, str],
) -> str:
    report = load_json(path)
    assert isinstance(report, list) and len(report) >= 1
    matches = []
    predicate_hashes = []
    for entry in report:
        assert isinstance(entry, dict)
        result = entry.get("verificationResult")
        assert isinstance(result, dict)
        statement = result.get("statement")
        assert isinstance(statement, dict)
        assert statement.get("predicateType") == expected_context["predicate_type"]

        certificate = result.get("signature", {}).get("certificate", {})
        assert isinstance(certificate, dict)
        assert certificate.get("subjectAlternativeName") == expected_context["certificate_identity"]
        assert certificate.get("issuer") == expected_context["certificate_oidc_issuer"]
        assert certificate.get("githubWorkflowRepository") == expected_context["repository"]
        assert certificate.get("githubWorkflowRef") == "refs/heads/main"
        assert certificate.get("sourceRepositoryURI") == ("https://github.com/" + expected_context["repository"])
        assert certificate.get("sourceRepositoryDigest") == expected_context["source_digest"]
        assert certificate.get("runnerEnvironment") == "github-hosted"
        assert certificate.get("runInvocationURI") == expected_context["run_invocation_uri"]

        timestamps = result.get("verifiedTimestamps", [])
        assert isinstance(timestamps, list)
        assert any(isinstance(timestamp, dict) and timestamp.get("type") == "Tlog" for timestamp in timestamps)

        predicate = statement.get("predicate")
        assert isinstance(predicate, dict)
        assert set(predicate) == {
            "attestation_kind",
            "canonical_predicate_sha256",
            "claim_ceiling",
            "policy_version",
            "schema",
        }
        assert predicate.get("schema") == expected_context["predicate_schema"]
        assert predicate.get("canonical_predicate_sha256") == expected_context["canonical_predicate_sha256"]
        assert predicate.get("policy_version") == int(expected_context["policy_version"])
        assert predicate.get("claim_ceiling") == expected_context["claim_ceiling"]
        assert predicate.get("attestation_kind") == "verified-runtime-evidence"

        subjects = statement.get("subject")
        assert isinstance(subjects, list)
        observed_subjects = []
        for subject in subjects:
            assert isinstance(subject, dict)
            digest = subject.get("digest")
            assert isinstance(digest, dict)
            assert set(digest) == {"sha256"}
            observed_subjects.append((subject.get("name"), digest["sha256"]))
        assert len(observed_subjects) == len(set(observed_subjects))
        assert sorted(observed_subjects) == sorted(expected_subjects)
        matches.append(entry)
        predicate_hashes.append(canonical_json_sha256(predicate))

    assert len(matches) == 1
    assert len(predicate_hashes) == 1
    return predicate_hashes[0]

def verify_no_public_good_control(
    path: Path,
    root: Path,
    expected_subject_name: str,
    expected_subject_sha256: str,
    expected_bundle_name: str,
    expected_bundle_sha256: str,
    expected_offline_name: str,
    expected_offline_sha256: str,
    expected_root_sha256: str,
) -> None:
    control = load_json(path)
    assert isinstance(control, dict)
    required = {
        "schema",
        "public_good_instance",
        "subject_name",
        "subject_path",
        "subject_sha256",
        "bundle_filename",
        "bundle_path",
        "bundle_sha256",
        "trusted_root_filename",
        "trusted_root_path",
        "trusted_root_sha256",
        "baseline_offline_filename",
        "baseline_offline_path",
        "baseline_offline_sha256",
        "command",
        "exit_status",
        "combined_output_base64",
        "combined_output_bytes",
        "combined_output_sha256",
    }
    assert set(control) == required
    assert control["schema"] == CONTROL_SCHEMA
    assert control["public_good_instance"] == PUBLIC_GOOD_INSTANCE
    assert control["subject_name"] == expected_subject_name
    assert control["subject_sha256"] == expected_subject_sha256

    evidence_root = Path(os.environ["RUNNER_TEMP"]) / "d6u-auditor-handoff"
    subject_path = evidence_root / expected_subject_name
    bundle_path = root / expected_bundle_name
    offline_path = root / expected_offline_name
    root_path = root / "trusted_root.jsonl"

    assert control["subject_path"] == str(subject_path)
    assert control["bundle_filename"] == expected_bundle_name
    assert control["bundle_path"] == str(bundle_path)
    assert control["bundle_sha256"] == expected_bundle_sha256
    assert control["trusted_root_filename"] == "trusted_root.jsonl"
    assert control["trusted_root_path"] == str(root_path)
    assert control["trusted_root_sha256"] == expected_root_sha256
    assert control["baseline_offline_filename"] == expected_offline_name
    assert control["baseline_offline_path"] == str(offline_path)
    assert control["baseline_offline_sha256"] == expected_offline_sha256

    expected_command = expected_verify_command(
        str(subject_path),
        os.environ["GITHUB_REPOSITORY"],
        root_path=str(root_path),
        bundle_path=str(bundle_path),
        no_public_good=True,
    )
    assert control["command"] == expected_command
    assert control["command"].count("--no-public-good") == 1
    assert control["exit_status"] != 0
    assert isinstance(control["combined_output_base64"], str)
    raw = base64.b64decode(control["combined_output_base64"], validate=True)
    assert 0 < len(raw) <= MAX_FILE_BYTES
    assert control["combined_output_bytes"] == len(raw)
    assert control["combined_output_sha256"] == hashlib.sha256(raw).hexdigest()


def main() -> None:
    if len(sys.argv) != 2:
        raise SystemExit(
            "usage: verify_d6u_trusted_attestation_retention.py RETENTION_DIR"
        )

    root = Path(sys.argv[1])
    assert root.is_dir() and not root.is_symlink()

    actual = sorted(path.name for path in root.iterdir())
    assert actual == sorted(FILES), f"unexpected retention packet files: {actual}"

    total = 0
    for name in actual:
        path = root / name
        assert path.is_file() and not path.is_symlink()
        limit = MAX_ROOT_BYTES if name == "trusted_root.jsonl" else MAX_FILE_BYTES
        size = path.stat().st_size
        assert 0 < size <= limit
        total += size
    assert total <= MAX_TOTAL_BYTES

    transcript = load_json(root / "retention-transcript.json")
    assert isinstance(transcript, dict)
    required = {
        "schema",
        "policy_version",
        "repository",
        "source_ref",
        "source_digest",
        "signer_workflow",
        "signer_workflow_digest",
        "certificate_identity",
        "certificate_oidc_issuer",
        "run_id",
        "run_attempt",
        "run_invocation_uri",
        "triggered_repository",
        "triggered_head_branch",
        "triggered_head_sha",
        "triggered_run_id",
        "triggered_run_attempt",
        "cli_version",
        "predicate_type",
        "predicate_schema",
        "canonical_predicate_schema",
        "canonical_predicate_sha256",
        "claim_ceiling",
        "public_good_instance_required",
        "public_good_instance",
        "tlog_required",
        "online_verified",
        "offline_verified",
        "no_public_good_rejected",
        "trusted_root",
        "subjects",
        "subject_bundle_bindings",
        "retained_files",
    }
    assert set(transcript) == required
    assert transcript["schema"] == SCHEMA
    assert transcript["policy_version"] == int(os.environ["D6U_TRUSTED_POLICY_VERSION"])
    assert transcript["repository"] == os.environ["GITHUB_REPOSITORY"]
    assert transcript["source_ref"] == os.environ["GITHUB_REF"]
    assert transcript["source_digest"] == os.environ["GITHUB_SHA"]
    assert transcript["signer_workflow"] == (
        os.environ["GITHUB_REPOSITORY"] + SIGNER_WORKFLOW_SUFFIX
    )
    assert transcript["signer_workflow_digest"] == os.environ["GITHUB_WORKFLOW_SHA"]
    assert transcript["certificate_identity"] == (
        "https://github.com/"
        + os.environ["GITHUB_REPOSITORY"]
        + SIGNER_WORKFLOW_SUFFIX
        + "@refs/heads/main"
    )
    assert transcript["certificate_oidc_issuer"] == CERT_OIDC_ISSUER
    assert str(transcript["run_id"]) == os.environ["GITHUB_RUN_ID"]
    assert str(transcript["run_attempt"]) == os.environ["GITHUB_RUN_ATTEMPT"]
    assert transcript["triggered_repository"] == os.environ["D6U_TRIGGER_REPOSITORY"]
    assert transcript["triggered_head_branch"] == os.environ["D6U_TRIGGER_HEAD_BRANCH"]
    assert transcript["triggered_head_sha"] == os.environ["D6U_TRIGGER_HEAD_SHA"]
    assert str(transcript["triggered_run_id"]) == os.environ["D6U_TRIGGER_RUN_ID"]
    assert str(transcript["triggered_run_attempt"]) == os.environ["D6U_TRIGGER_RUN_ATTEMPT"]
    assert transcript["run_invocation_uri"] == current_run_uri(
        os.environ["GITHUB_REPOSITORY"],
        os.environ["GITHUB_RUN_ID"],
        os.environ["GITHUB_RUN_ATTEMPT"],
    )
    assert transcript["cli_version"] == "2.101.0"
    assert transcript["predicate_type"] == PREDICATE_TYPE
    assert transcript["predicate_schema"] == ATTESTATION_PREDICATE_SCHEMA
    assert transcript["canonical_predicate_schema"] == CANONICAL_PREDICATE_SCHEMA
    assert transcript["claim_ceiling"] == "ReferenceModelOnly"
    assert transcript["public_good_instance_required"] is True
    assert transcript["public_good_instance"] == PUBLIC_GOOD_INSTANCE
    assert transcript["tlog_required"] is True
    assert transcript["online_verified"] is True
    assert transcript["offline_verified"] is True
    assert transcript["no_public_good_rejected"] is True

    subjects = transcript["subjects"]
    assert isinstance(subjects, list) and len(subjects) == len(SUBJECTS)
    by_name = {item["name"]: item for item in subjects}
    assert set(by_name) == set(SUBJECTS)
    assert len(by_name) == len(SUBJECTS)
    for item in subjects:
        assert set(item) == {"name", "sha256"}
        assert isinstance(item["name"], str) and item["name"] in SUBJECTS
        assert isinstance(item["sha256"], str) and len(item["sha256"]) == 64
        assert all(ch in "0123456789abcdef" for ch in item["sha256"])

    expected_subject_env = {
        "d6u-runtime-evidence.txt": "D6U_SUBJECT_D6U_RUNTIME_EVIDENCE_TXT_SHA256",
        "d6u-runtime-test.log": "D6U_SUBJECT_D6U_RUNTIME_TEST_LOG_SHA256",
        "Cargo.lock": "D6U_SUBJECT_CARGO_LOCK_SHA256",
    }
    for subject_name, env_name in expected_subject_env.items():
        assert by_name[subject_name]["sha256"] == os.environ[env_name]

    canonical_path = root / "d6u-trusted-evidence-predicate.json"
    canonical_predicate_sha256 = sha256(canonical_path)
    assert transcript["canonical_predicate_sha256"] == canonical_predicate_sha256
    canonical_predicate = load_json(canonical_path)
    assert isinstance(canonical_predicate, dict)
    assert set(canonical_predicate) == {
        "attestation_kind", "claim_ceiling", "evidence", "executor",
        "nonclaims", "policy_version", "schema", "source", "subjects", "trigger",
    }
    assert canonical_predicate["schema"] == CANONICAL_PREDICATE_SCHEMA
    assert canonical_predicate["attestation_kind"] == "verified-runtime-evidence"
    assert canonical_predicate["claim_ceiling"] == transcript["claim_ceiling"]
    assert canonical_predicate["policy_version"] == transcript["policy_version"]
    source = canonical_predicate["source"]
    assert isinstance(source, dict)
    assert set(source) == {"branch", "commit", "repository"}
    assert source["repository"] == transcript["repository"]
    assert source["repository"] == transcript["triggered_repository"]
    assert source["branch"] == transcript["triggered_head_branch"]
    assert source["commit"] == transcript["triggered_head_sha"]
    executor = canonical_predicate["executor"]
    assert isinstance(executor, dict)
    assert executor.get("run_id") == transcript["triggered_run_id"]
    assert executor.get("run_attempt") == transcript["triggered_run_attempt"]
    assert isinstance(source["branch"], str) and source["branch"]
    assert isinstance(source["commit"], str) and len(source["commit"]) == 40
    assert all(ch in "0123456789abcdef" for ch in source["commit"])
    assert canonical_predicate["subjects"] == [
        {"name": name, "digest": {"sha256": by_name[name]["sha256"]}}
        for name in SUBJECTS
    ]

    retained = transcript["retained_files"]
    assert isinstance(retained, dict)
    assert set(retained) == set(RETAINED_FILES)
    for name in RETAINED_FILES:
        assert_hash_record(root, name, retained[name])

    trusted_root = transcript["trusted_root"]
    assert isinstance(trusted_root, dict)
    assert set(trusted_root) == {"filename", "sha256"}
    assert trusted_root["filename"] == "trusted_root.jsonl"
    assert trusted_root["sha256"] == retained["trusted_root.jsonl"]["sha256"]
    load_jsonl(root / "trusted_root.jsonl")

    expected_subjects = [
        (subject_name, by_name[subject_name]["sha256"])
        for subject_name in SUBJECTS
    ]
    bindings = transcript["subject_bundle_bindings"]
    assert isinstance(bindings, list) and len(bindings) == len(SUBJECTS)
    binding_names = {item["subject_name"] for item in bindings}
    assert binding_names == set(SUBJECTS)
    assert len(binding_names) == len(SUBJECTS)

    root_sha256 = retained["trusted_root.jsonl"]["sha256"]
    for binding in bindings:
        assert set(binding) == {
            "subject_name",
            "subject_sha256",
            "bundle_filename",
            "bundle_sha256",
            "online_report_filename",
            "online_report_sha256",
            "offline_report_filename",
            "offline_report_sha256",
            "predicate_sha256",
            "canonical_predicate_sha256",
            "negative_control_filename",
            "negative_control_sha256",
            "trusted_root_sha256",
        }
        name = binding["subject_name"]
        assert name in SUBJECTS
        subject_sha256 = by_name[name]["sha256"]
        assert binding["subject_sha256"] == subject_sha256
        safe = name.replace(".", "_").replace("-", "_")
        expected_bundle = safe + ".attestation.jsonl"
        expected_online = safe + ".online.json"
        expected_offline = safe + ".offline.json"
        assert binding["bundle_filename"] == expected_bundle
        assert binding["online_report_filename"] == expected_online
        assert binding["offline_report_filename"] == expected_offline
        expected_control = safe + ".no-public-good.json"
        assert binding["negative_control_filename"] == expected_control
        assert binding["bundle_sha256"] == retained[expected_bundle]["sha256"]
        assert binding["online_report_sha256"] == retained[expected_online]["sha256"]
        assert binding["offline_report_sha256"] == retained[expected_offline]["sha256"]
        assert binding["canonical_predicate_sha256"] == canonical_predicate_sha256
        assert binding["negative_control_sha256"] == retained[expected_control]["sha256"]
        assert binding["trusted_root_sha256"] == root_sha256

        load_jsonl(root / expected_bundle)
        expected_context = {
            "repository": transcript["repository"],
            "source_digest": transcript["source_digest"],
            "run_invocation_uri": transcript["run_invocation_uri"],
            "certificate_identity": transcript["certificate_identity"],
            "certificate_oidc_issuer": transcript["certificate_oidc_issuer"],
            "predicate_type": transcript["predicate_type"],
            "predicate_schema": transcript["predicate_schema"],
            "policy_version": str(transcript["policy_version"]),
            "canonical_predicate_sha256": transcript["canonical_predicate_sha256"],
            "claim_ceiling": transcript["claim_ceiling"],
        }
        online_predicate_sha256 = verify_report(
            root / expected_online,
            expected_subjects,
            expected_context,
        )
        offline_predicate_sha256 = verify_report(
            root / expected_offline,
            expected_subjects,
            expected_context,
        )
        assert online_predicate_sha256 == offline_predicate_sha256
        assert binding["predicate_sha256"] == online_predicate_sha256

    for subject_name in SUBJECTS:
        safe = subject_name.replace(".", "_").replace("-", "_")
        expected_bundle = safe + ".attestation.jsonl"
        expected_offline = safe + ".offline.json"
        expected_control = safe + ".no-public-good.json"
        verify_no_public_good_control(
            root / expected_control,
            root,
            expected_subject_name=subject_name,
            expected_subject_sha256=by_name[subject_name]["sha256"],
            expected_bundle_name=expected_bundle,
            expected_bundle_sha256=retained[expected_bundle]["sha256"],
            expected_offline_name=expected_offline,
            expected_offline_sha256=retained[expected_offline]["sha256"],
            expected_root_sha256=root_sha256,
        )

    print(
        "verified D6U retained attestation packet: "
        f"subjects={len(SUBJECTS)}, files={len(FILES)}, "
        f"run={transcript['run_id']}, attempt={transcript['run_attempt']}, "
        f"policy=v{transcript['policy_version']}"
    )


if __name__ == "__main__":
    main()
