#!/usr/bin/env python3
"""Deterministic, read-only tests for the D6U trusted verifier."""

import json
import hashlib
import os
import re
import subprocess
import stat
import struct
import tempfile
import sys
from zipfile import ZipFile, ZipInfo
from unittest.mock import patch
from pathlib import Path

from fetch_d6u_trusted_artifact import (
    EXPECTED_FILES,
    HANDOFF_EXPECTED_FILES,
    download_archive,
    expected_artifact,
    expected_current_run_artifact,
    extract_members,
    verify_zip_members,
)
from verify_d6u_trusted_attestation import main as verify_attestation_main
from verify_d6u_trusted_artifacts import (
    load_record,
    verify_artifact_layout,
    verify_cases,
    verify_executor_run_record,
    verify_trusted_workflow_identity,
    verify_executor_workflow_record,
    verify_executor_workflow_against_run_head,
    verify_lock,
    verify_lock_graph_against_manifest,
    _git_blob_sha1,
    verify_required_tracked_blobs,
    verify_record_metadata,
    verify_trigger_run_record,
    verify_forbidden_cargo_config_paths,
    verify_exact_harness_file_set,
    verify_artifact_size_limits,
)


def base_policy() -> dict:
    return {
        "cases": {
            "canonical-payload-accepted": {
                "outcome": "accepted",
                "zome_reached": True,
            }
        },
        "supplemental_substrate": {
            "future-expiry-rejection": "Future",
        },
        "application_check": {
            "id": "probe-local-d6s-commitment-mutation",
            "fragment": "result=d6s-commitment-mismatch;zome-reached=true",
        },
    }


def valid_log() -> str:
    return "\n".join(
        [
            "D6U_CASE\tcanonical-payload-accepted\taccepted\tzome-reached=true\tPASS",
            'D6U_RUNTIME_WITNESS\tfuture-expiry-rejection\tBadNonce("Future")',
            'D6U_SUBSTRATE_CHECK\tfuture-expiry-rejection\tBadNonce("Future")\tPASS',
            "D6U_APPLICATION_CHECK\tprobe-local-d6s-commitment-mutation\tresult=d6s-commitment-mismatch;zome-reached=true\tPASS",
        ]
    )


def assert_rejected(fn, message: str) -> None:
    try:
        fn()
    except AssertionError:
        return
    raise AssertionError(message)


def test_policy_pins_d6s_prerequisite_boundary() -> None:
    policy_path = Path(__file__).parents[2] / "docs/integral/d6u-trusted-builder-policy.json"
    policy = json.loads(policy_path.read_text(encoding="utf-8"))
    required = policy["required_source_blobs"]
    expected_paths = {
        "docs/integral/d6s-canon-1-manifest.json",
        "docs/integral/d6s-canon-1-golden-vectors.json",
        "scripts/integral/verify_d6s_canon_1.py",
        "docs/integral/d6s-canon-2-manifest.json",
        "docs/integral/d6s-canon-2-authority-boundary-fixture.json",
        "scripts/integral/verify_d6s_canon_2_authority_boundary.py",
        ".github/workflows/d6s-canonical-qualification.yml",
    }
    assert expected_paths <= set(required)
    for path in expected_paths:
        assert len(required[path]) == 40
        assert all(ch in "0123456789abcdef" for ch in required[path])

    assert policy["policy_version"] == 61
    assert policy["repository_identity"] == {
        "full_name": "Luminous-Dynamics/mycelix",
        "repository_id": 1176351975,
    }
    assert policy["trigger_workflow"]["workflow_id"] == 371215723
    assert policy["trigger_workflow"]["name"] == "D6S Canonical Qualification"
    assert policy["trigger_workflow"]["path"] == ".github/workflows/d6s-canonical-qualification.yml"
    assert policy["trigger_workflow"]["blob_sha"] == "e2ee0dd880d5ee0b48ef9667608294d46b2fc1b4"
    assert policy["trigger_workflow"]["blob_sha"] == required[".github/workflows/d6s-canonical-qualification.yml"]
    trusted_source = policy["trusted_source_root"]
    assert trusted_source == {
        "ref": "refs/heads/main",
        "commit_sha": "445a84abdaa64f3d05e2d5c51115e4d060b78ec7",
        "purpose": "Immutable source root for policy-pinned D6S workflow definition.",
    }
    trusted_root_path = Path(
        os.environ.get(
            "D6U_TRUSTED_SOURCE_ROOT",
            str(Path(__file__).parents[2]),
        )
    )
    d6s_workflow_path = trusted_root_path / ".github/workflows/d6s-canonical-qualification.yml"
    assert d6s_workflow_path.is_file()
    assert _git_blob_sha1(d6s_workflow_path.read_bytes()) == policy["trigger_workflow"]["blob_sha"]

    assert policy["attestation_integrity_revision"] == policy["policy_version"]

    assert policy["forbidden_cargo_config_paths"] == [
        ".cargo/config",
        ".cargo/config.toml",
        "d6u-runtime-harness/.cargo/config",
        "d6u-runtime-harness/.cargo/config.toml",
    ]

    assert policy["d6u_harness_tracked_files"] == [
        "d6u-runtime-harness/Cargo.toml",
        "d6u-runtime-harness/README.md",
        "d6u-runtime-harness/rust-toolchain.toml",
        "d6u-runtime-harness/src/lib.rs",
        "d6u-runtime-harness/tests/d6u_authority_boundary.rs",
    ]
    assert policy["artifact_max_bytes"] == {
        "d6u-runtime-evidence.txt": 262144,
        "d6u-runtime-test.log": 8388608,
        "Cargo.lock": 4194304,
    }
    assert policy["artifact_max_total_bytes"] == 12582912
    assert policy["attestation_verification"] == {
        "signer_workflow": "Luminous-Dynamics/mycelix/.github/workflows/d6u-trusted-evidence-attestation.yml",
        "signer_digest_source": "GITHUB_WORKFLOW_SHA",
        "certificate_identity": "https://github.com/Luminous-Dynamics/mycelix/.github/workflows/d6u-trusted-evidence-attestation.yml@refs/heads/main",
        "certificate_oidc_issuer": "https://token.actions.githubusercontent.com",
        "deny_self_hosted_runners": True,
        "source_digest_source": "GITHUB_SHA",
        "source_ref_source": "GITHUB_REF",
        "predicate_type": "https://luminousdynamics.io/attestations/d6u-runtime-evidence/v1",
        "predicate_schema": "d6u-trusted-runtime-evidence-attestation/v2",
        "canonical_predicate_schema": "d6u-trusted-runtime-evidence/v1",
        "current_run_identity": {
            "certificate_field": "runInvocationURI",
            "template": "https://github.com/{repository}/actions/runs/{run_id}/attempts/{run_attempt}",
        },
        "require_current_run_identity": True,
        "require_subject_digest_match": True,
        "subject_set_exact": True,
        "require_signed_predicate_subject_binding": True,
        "require_verified_timestamp": True,
        "require_verified_tlog": True,
        "max_report_bytes": 4194304,
    }
    assert policy["lock_graph"] == {
        "required_registry_source": "registry+https://github.com/rust-lang/crates.io-index",
        "allowed_local_packages": ["d6u-runtime-harness"],
        "lockfile_format_version": 4,
        "manifest_path": "d6u-runtime-harness/Cargo.toml",
        "require_manifest_root_binding": True,
        "require_dependency_edge_closure": True,
        "require_all_packages_reachable": True,
    }
    assert policy["artifact_max_entries"] == 32
    assert policy["trusted_artifact_fetcher"]["path"] == (
        "scripts/integral/fetch_d6u_trusted_artifact.py"
    )
    assert len(policy["trusted_artifact_fetcher"]["blob_sha"]) == 40
    assert all(
        ch in "0123456789abcdef"
        for ch in policy["trusted_artifact_fetcher"]["blob_sha"]
    )
    assert policy["trusted_artifact_fetcher"]["blob_sha"] == policy["trusted_programs"][
        "scripts/integral/fetch_d6u_trusted_artifact.py"
    ]["blob_sha"]
    assert set(policy["trusted_programs"]) == {
        "scripts/integral/verify_d6u_trusted_artifacts.py",
        "scripts/integral/fetch_d6u_trusted_artifact.py",
        "scripts/integral/verify_d6u_trusted_attestation.py",
        "scripts/integral/emit_d6u_trusted_attestation_predicate.py",
        "scripts/integral/verify_d6u_trusted_attestation_retention.py",
    }
    for path, descriptor in policy["trusted_programs"].items():
        assert descriptor["path"] == path
        assert len(descriptor["blob_sha"]) == 40
        tree_record = subprocess.run(
            ["git", "ls-tree", "--full-tree", "-r", "HEAD", "--", path],
            check=True,
            capture_output=True,
            text=True,
            cwd=Path(__file__).parents[2],
        ).stdout.strip()
        assert tree_record
        mode, kind, observed, observed_path = tree_record.split(None, 3)
        assert kind == "blob"
        assert observed_path == path
        assert mode in {"100644", "100755"}
        assert observed == descriptor["blob_sha"]

    assert policy["trusted_attestation_verifier"]["path"] == (
        "scripts/integral/verify_d6u_trusted_attestation.py"
    )
    assert len(policy["trusted_attestation_verifier"]["blob_sha"]) == 40
    assert policy["trusted_attestation_verifier"]["blob_sha"] == policy["trusted_programs"][
        "scripts/integral/verify_d6u_trusted_attestation.py"
    ]["blob_sha"]
    assert policy["attestation_verification"]["max_report_bytes"] == 4194304
    assert policy["attestation_verification"]["predicate_type"] == (
        "https://luminousdynamics.io/attestations/d6u-runtime-evidence/v1"
    )
    assert policy["attestation_verification"]["predicate_schema"] == (
        "d6u-trusted-runtime-evidence-attestation/v2"
    )
    assert policy["attestation_verification"]["canonical_predicate_schema"] == (
        "d6u-trusted-runtime-evidence/v1"
    )
    assert policy["attestation_verification"]["require_current_run_identity"] is True
    assert policy["attestation_verification"]["subject_set_exact"] is True
    assert policy["attestation_verification"]["require_verified_timestamp"] is True
    assert policy["attestation_verification"]["require_verified_tlog"] is True

    assert policy["attestation_retention"] == {
        "schema": "d6u-attestation-retention/v2",
        "expected_file_count": 15,
        "max_file_bytes": 4194304,
        "max_trusted_root_bytes": 2097152,
        "max_total_bytes": 18874368,
        "max_jsonl_lines": 64,
        "subjects": [
            "d6u-runtime-evidence.txt",
            "d6u-runtime-test.log",
            "Cargo.lock",
        ],
        "trusted_root_filename": "trusted_root.jsonl",
        "offline_verified": True,
        "online_verified": True,
        "require_public_good_instance": True,
        "public_good_instance": "sigstore-public-good",
        "require_tlog": True,
        "require_no_public_good_rejection": True,
        "retention_artifact": {
            "name_template": "d6u-trusted-attestation-retention-run-{run_id}-attempt-{run_attempt}",
            "retention_days": 90,
        },
        "negative_control_schema": "d6u-no-public-good-control/v1",
        "retain_online_verification": True,
        "retain_negative_control": True,
        "max_attestations_per_verify": 8,
        "predicate_schema": "d6u-trusted-runtime-evidence-attestation/v2",
        "canonical_predicate_filename": "d6u-trusted-evidence-predicate.json",
        "canonical_predicate_schema": "d6u-trusted-runtime-evidence/v1",
        "retain_canonical_predicate": True,
    }

    assert policy["workflow_default_permissions"] == {}
    assert policy["attestation_trigger"]["workflow_name"] == "D6U Exact-Head Runtime Executor"
    assert policy["attestation_trigger"]["event"] == "workflow_run"
    assert policy["attestation_trigger"]["types"] == ["completed"]
    assert policy["attestation_trigger"]["conclusion"] == "success"
    assert policy["attestation_trigger"]["repository"] == "Luminous-Dynamics/mycelix"
    assert policy["attestation_trigger"]["head_branch"] == "myc-int-demo-d6u-holochain-07-runtime"
    assert policy["attestation_trigger"]["workflow_path"] == ".github/workflows/d6u-exact-head-runtime-executor.yml"
    assert policy["attestation_trigger"]["require_record_source_binding"] is True
    assert policy["attestation_trigger"]["require_record_executor_run_binding"] is True
    workflow_text = (Path(__file__).parents[2] / policy["trusted_workflow"]["path"]).read_text(encoding="utf-8")
    literal_policy_version_pattern = re.compile(
        r'^\s*(?:D6U_TRUSTED_POLICY_VERSION|TRUSTED_POLICY_VERSION):\s*"([0-9]+)"\s*$'
    )
    inline_policy_version_pattern = re.compile(
        r'^\s*D6U_TRUSTED_POLICY_VERSION="([0-9]+)"'
    )
    trust_policy_versions = []
    for line in workflow_text.splitlines():
        match = literal_policy_version_pattern.match(line) or inline_policy_version_pattern.match(line)
        if match:
            trust_policy_versions.append(match.group(1))
    assert len(trust_policy_versions) >= 5
    assert all(version == str(policy["policy_version"]) for version in trust_policy_versions)
    trusted_policy_env_versions = [
        line.split("D6U_TRUSTED_POLICY_VERSION=", 1)[1].strip().split('"', 2)[1]
        for line in workflow_text.splitlines()
        if "D6U_TRUSTED_POLICY_VERSION=\"" in line
        and line.split("D6U_TRUSTED_POLICY_VERSION=", 1)[1].strip().startswith('"')
        and line.split("D6U_TRUSTED_POLICY_VERSION=", 1)[1].strip().split('"', 2)[1].isdigit()
    ]
    assert trusted_policy_env_versions
    assert all(version == str(policy["policy_version"]) for version in trusted_policy_env_versions)
    _, signer_section = workflow_text.split("\n  signer:\n", 1)
    assert '"policy_version": %d,' % policy["policy_version"] in signer_section
    assert '"policy_version": 37,' not in signer_section

    assert policy["trusted_actions"] == {
        "actions/checkout": {
            "ref": "fbc6f3992d24b796d5a048ff273f7fcc4a7b6c09",
            "version": "v5.1.0",
        },
        "actions/attest": {
            "ref": "1e69f48acb82d1966a394da916b4c169aa569d6",
            "version": "v4.2.2",
        },
        "actions/upload-artifact": {
            "ref": "ea165f8d65b6e75b540449e92b4886f43607fa02",
            "version": "v4.6.2",
        },
    }
    workflow_path = Path(__file__).parents[2] / policy["trusted_workflow"]["path"]
    uses = []
    for line in workflow_path.read_text(encoding="utf-8").splitlines():
        stripped = line.strip()
        if stripped.startswith("uses:"):
            uses.append(stripped.split("uses:", 1)[1].strip())
    expected_uses = {
        f"{name}@{config['ref']} # {config['version']}"
        for name, config in policy["trusted_actions"].items()
    }
    assert set(uses) == expected_uses

    assert policy["attestation_publication"] == {
        "push_to_registry": False,
        "create_storage_record": False,
        "show_summary": False,
    }
    assert policy["trusted_permissions"] == {
        "verifier": {
            "actions": "read",
            "contents": "read",
        },
        "signer": {
            "contents": "read",
            "id-token": "write",
            "attestations": "write",
        },
        "auditor": {
            "actions": "read",
            "contents": "read",
            "attestations": "read",
        },
    }
    workflow_path = Path(__file__).parents[2] / policy["trusted_workflow"]["path"]
    workflow_text = workflow_path.read_text(encoding="utf-8")
    verifier_section, signer_section = workflow_text.split("\n  signer:\n", 1)
    assert "id-token: write" not in verifier_section
    assert "attestations: write" not in verifier_section
    assert "id-token: write" in signer_section
    assert "attestations: write" in signer_section
    assert "actions: write" not in workflow_text
    assert "contents: write" not in workflow_text
    assert "artifact-metadata: write" not in workflow_text
    assert "uses: actions/download-artifact@" not in workflow_text
    assert "python3 scripts/integral/fetch_d6u_trusted_artifact.py" in workflow_text
    assert "--current-run-handoff" in workflow_text
    assert "d6u-trusted-auditor-handoff" in workflow_text
    assert "uses: actions/upload-artifact@ea165f8d65b6e75b540449e92b4886f43607fa02 # v4.6.2" in workflow_text

    assert policy["auditor_handoff"] == {
        "schema": "d6u-trusted-auditor-handoff/v1",
        "expected_file_count": 6,
        "manifest_filename": "d6u-auditor-handoff.manifest.sha256",
        "context_filename": "d6u-auditor-context.txt",
        "predicate_filename": "d6u-trusted-evidence-predicate.json",
        "subject_files": [
            "d6u-runtime-evidence.txt",
            "d6u-runtime-test.log",
            "Cargo.lock",
        ],
        "artifact_name_template": "d6u-trusted-auditor-handoff-run-{run_id}-attempt-{run_attempt}",
        "retention_days": 1,
        "require_exact_file_set": True,
        "require_manifest_sha256": True,
        "require_current_run_context": True,
        "claim_ceiling": "ReferenceModelOnly",
        "artifact_max_bytes": {
            "d6u-runtime-evidence.txt": 262144,
            "d6u-runtime-test.log": 8388608,
            "Cargo.lock": 4194304,
            "d6u-auditor-context.txt": 4096,
            "d6u-auditor-handoff.manifest.sha256": 1024,
            "d6u-trusted-evidence-predicate.json": 16384,
        },
        "artifact_max_entries": 16,
        "artifact_max_total_bytes": 12866560,
        "artifact_max_archive_bytes": 13631488,
        "expected_member_count": 6,
        "require_match_after_download": True,
        "archive_format": "zip",
        "extraction_mode": "bounded-members",
        "reject_encrypted_members": True,
        "reject_symlink_members": True,
        "reject_zip64": True,
        "eocd_entry_count_preflight": True,
        "allowed_compression_methods": ["stored", "deflate"],
    }

    assert policy["attestation_commitment"] == {
        "schema": "d6u-trusted-runtime-evidence-attestation/v2",
        "predicate_type": "https://luminousdynamics.io/attestations/d6u-runtime-evidence/v1",
        "canonical_predicate_schema": "d6u-trusted-runtime-evidence/v1",
        "canonical_predicate_filename": "d6u-trusted-evidence-predicate.json",
        "subject_source": "verifier_job_outputs",
        "signing_mode": "digest-only-subject-checksums",
        "signer_must_not_receive_original_subject_files": True,
        "signer_must_not_download_auditor_handoff": True,
    }
    assert policy["artifact_integrity"] == {        "algorithm": "sha256",
        "source": "github-artifact-api",
        "require_match_after_download": True,
        "archive_format": "zip",
        "extraction_mode": "bounded-members",
        "reject_encrypted_members": True,
        "reject_symlink_members": True,
        "expected_member_count": 3,
        "policy_revision": 30,
        "allowed_compression_methods": ["stored", "deflate"],
    }
    assert policy["artifact_run_binding"] == {
        "require_exact_run_attempt": True,
        "require_head_branch_match_trigger": True,
        "require_head_sha_match_trigger": True,
    }


def record_metadata_policy() -> dict:
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


def valid_record_metadata() -> dict:
    policy = record_metadata_policy()
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
	

def test_record_metadata_is_canonicalized() -> None:
    policy = record_metadata_policy()
    record = valid_record_metadata()
    verify_record_metadata(record, policy)

    tampered = dict(record)
    tampered["workflow_run_id"] = "201"
    assert_rejected(
        lambda: verify_record_metadata(tampered, policy),
        "workflow run identity aliasing was accepted",
    )

    tampered = dict(record)
    tampered["trigger_workflow_run_id"] = "0100"
    assert_rejected(
        lambda: verify_record_metadata(tampered, policy),
        "noncanonical trigger workflow run ID was accepted",
    )

    tampered = dict(record)
    tampered["trigger_workflow_run_attempt"] = "01"
    assert_rejected(
        lambda: verify_record_metadata(tampered, policy),
        "noncanonical trigger workflow run attempt was accepted",
    )

    tampered = dict(record)
    tampered["case_coverage"] = "13-of-14"
    assert_rejected(
        lambda: verify_record_metadata(tampered, policy),
        "tampered deterministic evidence metadata was accepted",
    )

    tampered = dict(record)
    tampered["manifest_git_blob_sha"] = "e" * 40
    assert_rejected(
        lambda: verify_record_metadata(tampered, policy),
        "tampered verifier blob identity was accepted",
    )

    tampered = dict(record)
    tampered["unexpected_record_field"] = "attacker-controlled"
    assert_rejected(
        lambda: verify_record_metadata(tampered, policy),
        "extra runtime evidence record field was accepted",
    )

    tampered = dict(record)
    tampered.pop("claim_ceiling")
    assert_rejected(
        lambda: verify_record_metadata(tampered, policy),
        "missing runtime evidence record field was accepted",
    )


def test_policy_pins_current_trusted_workflow() -> None:
    root = Path(__file__).parents[2]
    policy = json.loads(
        (root / "docs/integral/d6u-trusted-builder-policy.json").read_text(
            encoding="utf-8"
        )
    )
    workflow = root / policy["trusted_workflow"]["path"]
    observed = subprocess.run(
        ["git", "hash-object", str(workflow)],
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()
    assert policy["trusted_workflow"]["blob_sha"] == observed


def test_trusted_github_api_readers_are_response_bounded() -> None:
    root = Path(__file__).parents[2]
    sources = [
        root / "scripts/integral/verify_d6u_trusted_artifacts.py",
        root / "scripts/integral/fetch_d6u_trusted_artifact.py",
    ]
    for path in sources:
        source = path.read_text(encoding="utf-8")
        assert "MAX_GITHUB_JSON_BYTES = 8 * 1024 * 1024" in source
        assert "response.read(MAX_GITHUB_JSON_BYTES + 1)" in source
        assert "len(payload) > MAX_GITHUB_JSON_BYTES" in source
    policy = json.loads(
        (root / "docs/integral/d6u-trusted-builder-policy.json").read_text(
            encoding="utf-8"
        )
    )
    assert policy["trusted_network"]["github_api_response_max_bytes"] == 8 * 1024 * 1024


def test_trusted_python_programs_reject_optimized_mode() -> None:
    root = Path(__file__).parents[2]
    trusted_programs = [
        "scripts/integral/verify_d6u_trusted_artifacts.py",
        "scripts/integral/fetch_d6u_trusted_artifact.py",
        "scripts/integral/emit_d6u_trusted_attestation_predicate.py",
        "scripts/integral/verify_d6u_trusted_attestation.py",
        "scripts/integral/verify_d6u_trusted_attestation_retention.py",
    ]
    probe = (
        "import runpy, sys; "
        "runpy.run_path(sys.argv[1], run_name='__trusted_opt_test__')"
    )
    for relative in trusted_programs:
        source = (root / relative).read_text(encoding="utf-8")
        assert "if not __debug__:" in source
        assert "trusted D6U program must not run with Python optimization enabled" in source
        completed = subprocess.run(
            [sys.executable, "-O", "-c", probe, str(root / relative)],
            capture_output=True,
            text=True,
            check=False,
        )
        assert completed.returncode != 0
        assert (
            "trusted D6U program must not run with Python optimization enabled"
            in completed.stderr
        )


def test_attestation_verifier_contains_no_optimization_sensitive_asserts() -> None:
    import ast

    root = Path(__file__).parents[2]
    source = (
        root / "scripts/integral/verify_d6u_trusted_attestation.py"
    ).read_text(encoding="utf-8")
    tree = ast.parse(source)
    assert not any(isinstance(node, ast.Assert) for node in ast.walk(tree))


def test_policy_pins_current_trusted_fetcher() -> None:
    root = Path(__file__).parents[2]
    policy = json.loads(
        (root / "docs/integral/d6u-trusted-builder-policy.json").read_text(
            encoding="utf-8"
        )
    )
    fetcher_path = root / policy["trusted_artifact_fetcher"]["path"]
    observed = subprocess.run(
        ["git", "hash-object", str(fetcher_path)],
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()
    assert policy["trusted_artifact_fetcher"]["blob_sha"] == observed
    assert policy["trusted_artifact_fetcher"]["blob_sha"] == policy["trusted_programs"][
        "scripts/integral/fetch_d6u_trusted_artifact.py"
    ]["blob_sha"]


def test_retention_workflow_bounds_inputs_before_verification() -> None:
    workflow = (
        Path(__file__).parents[2]
        / ".github/workflows/d6u-trusted-evidence-attestation.yml"
    ).read_text(encoding="utf-8")
    assert 'wc -c < "$report")" -le 4194304' in workflow
    assert 'wc -c < "$retention_dir/trusted_root.jsonl")" -le 2097152' in workflow
    assert 'wc -l < "$retention_dir/trusted_root.jsonl")" -le 64' in workflow
    assert 'wc -c < "$bundle")" -le 4194304' in workflow
    assert 'wc -c < "$online_report")" -le 4194304' in workflow
    assert 'wc -c < "$offline_report")" -le 4194304' in workflow
    assert 'wc -c < "$control_raw")" -le 4194304' in workflow


def test_retention_workflow_contains_offline_controls() -> None:
    root = Path(__file__).parents[2]
    policy = json.loads(
        (root / "docs/integral/d6u-trusted-builder-policy.json").read_text(
            encoding="utf-8"
        )
    )
    workflow = (root / policy["trusted_workflow"]["path"]).read_text(encoding="utf-8")
    required_fragments = [
        "gh attestation trusted-root",
        "gh attestation download",
        "--predicate-type \"https://luminousdynamics.io/attestations/d6u-runtime-evidence/v1\"",
        "--bundle \"$bundle\"",
        "--custom-trusted-root \"$retention_dir/trusted_root.jsonl\"",
        "--no-public-good",
        "--deny-self-hosted-runners",
        "--format=json",
        "--limit 8",
        "d6u-runtime-evidence.online.json",
        "d6u-runtime-evidence.no-public-good.json",
        "actions/upload-artifact@ea165f8d65b6e75b540449e92b4886f43607fa02 # v4.6.2",
        "retention-days: 90",
        "d6u-attestation-retention/v2",
        '"schema": "d6u-no-public-good-control/v1",',

    ]
    for fragment in required_fragments:
        assert fragment in workflow, f"retention workflow control missing: {fragment}"
    assert workflow.count("gh attestation verify") == 4
    assert workflow.count("--limit 8") >= 5


def test_privilege_split_handoff_topology_is_fail_closed() -> None:
    root = Path(__file__).parents[2]
    policy = json.loads(
        (root / "docs/integral/d6u-trusted-builder-policy.json").read_text(encoding="utf-8")
    )
    workflow = (root / policy["trusted_workflow"]["path"]).read_text(encoding="utf-8")
    jobs = workflow.split("\n  signer:\n", 1)
    assert len(jobs) == 2
    verifier, remainder = jobs
    signer, auditor = remainder.split("\n  auditor:\n", 1)

    assert verifier.count("    permissions:\n") == 1
    assert signer.count("    permissions:\n") == 1
    assert auditor.count("    permissions:\n") == 1
    assert "    permissions:\n      actions: read\n      contents: read\n" in verifier
    assert "    permissions:\n      contents: read\n      id-token: write\n      attestations: write\n" in signer
    assert "    permissions:\n      actions: read\n      contents: read\n      attestations: read\n" in auditor

    assert "id-token: write" not in verifier
    assert "attestations: write" not in verifier
    assert "uses: actions/attest@" not in verifier
    assert "gh attestation verify" not in verifier
    for trigger_key in (
        "D6U_TRIGGER_REPOSITORY:",
        "D6U_TRIGGER_HEAD_BRANCH:",
        "D6U_TRIGGER_HEAD_SHA:",
        "D6U_TRIGGER_RUN_ID:",
        "D6U_TRIGGER_RUN_ATTEMPT:",
    ):
        assert verifier.count(trigger_key) == 1

    assert "id-token: write" in signer
    assert "actions: read" not in signer
    assert "uses: actions/download-artifact@" not in signer
    expected_verifier_outputs = {
        "evidence_sha256",
        "runtime_test_sha256",
        "cargo_lock_sha256",
        "canonical_predicate_sha256",
    }
    verifier_output_lines = re.search(
        r"\n    outputs:\n(?P<body>(?:      [^\n]+\n)+)    steps:",
        verifier,
    )
    assert verifier_output_lines is not None
    observed_verifier_outputs = {
        line.strip().split(":", 1)[0]
        for line in verifier_output_lines.group("body").splitlines()
        if line.strip()
    }
    assert observed_verifier_outputs == expected_verifier_outputs
    verifier_output_references = re.findall(
        r"needs\.verifier\.outputs\.([A-Za-z0-9_]+)",
        signer,
    )
    assert set(verifier_output_references) == expected_verifier_outputs
    assert len(verifier_output_references) == 8

    assert "subject-checksums: ${{ steps.subject_manifest.outputs.manifest }}" in signer
    assert "predicate-path: ${{ steps.commitment_predicate.outputs.predicate }}" in signer
    assert "push-to-registry: false" in signer
    assert "create-storage-record: false" in signer
    assert "show-summary: false" in signer
    assert "${{ needs.verifier.outputs.canonical_predicate_sha256 }}" in signer
    assert "d6u-attestation-commitment.json" in signer
    assert "attestations: write" in signer
    assert "uses: actions/attest@1e69f48acb82d1966a394da916b4c169aa569d6 # v4.2.2" in signer
    assert "branches:\n      - myc-int-demo-d6u-holochain-07-runtime" in workflow
    assert workflow.count("github.event.workflow_run.head_branch == 'myc-int-demo-d6u-holochain-07-runtime'") == 3
    assert workflow.count("github.event.workflow_run.repository.full_name == github.repository") == 3
    assert workflow.count("github.event.workflow_run.head_repository.full_name == github.repository") == 3
    assert workflow.count("github.event.repository.id == github.repository_id") == 3
    assert workflow.count("github.event.workflow_run.repository.id == github.repository_id") == 3
    assert workflow.count("github.event.workflow_run.head_repository.id == github.repository_id") == 3
    assert workflow.count("D6U_TRIGGER_REPOSITORY_ID: " + "${{ github.repository_id }}") == 1
    assert workflow.count("D6U_TRUSTED_REPOSITORY_ID: " + "${{ github.repository_id }}") == 2
    assert workflow.count("github.event.workflow_run.name == 'D6U Exact-Head Runtime Executor'") == 3
    assert workflow.count("github.event.workflow_run.path == '.github/workflows/d6u-exact-head-runtime-executor.yml'") == 3
    assert "gh attestation verify" not in signer
    assert "uses: actions/download-artifact@" not in signer

    assert "id-token: write" not in auditor
    assert "attestations: write" not in auditor
    assert "attestations: read" in auditor
    assert "gh attestation verify" in auditor
    assert "uses: actions/upload-artifact@ea165f8d65b6e75b540449e92b4886f43607fa02 # v4.6.2" in auditor
    assert "d6u-trusted-input" not in signer
    assert "d6u-trusted-input" not in auditor
    assert "d6u-trusted-auditor-handoff-run-${{ github.run_id }}-attempt-${{ github.run_attempt }}" in verifier
    assert 'python3 scripts/integral/fetch_d6u_trusted_artifact.py \\' in auditor
    assert "--current-run-handoff" in auditor
    assert workflow.count("id-token: write") == 1
    assert workflow.count("attestations: write") == 1
    assert workflow.count("uses: actions/attest@1e69f48acb82d1966a394da916b4c169aa569d6 # v4.2.2") == 1

def test_trusted_cli_policy_is_explicit() -> None:
    policy = json.loads(
        (Path(__file__).parents[2] / "docs/integral/d6u-trusted-builder-policy.json").read_text(
            encoding="utf-8"
        )
    )
    assert policy["trusted_cli"] == {
        "name": "gh",
        "version": "2.101.0",
        "configuration_directory": "${{ runner.temp }}/d6u-gh-config",
        "required_fresh_configuration": True,
        "forbidden_environment_overrides": ["GH_HOST", "GH_ENTERPRISE_TOKEN", "GH_REPO"],
        "executable_path_must_not_resolve_under": ["${{ github.workspace }}", "${{ runner.temp }}"],
    }
    workflow = (Path(__file__).parents[2] / policy["trusted_workflow"]["path"]).read_text(encoding="utf-8")
    assert "gh version" in workflow
    assert "2.101.0" in workflow
    assert "GH_CONFIG_DIR: ${{ runner.temp }}/d6u-gh-config" in workflow
    assert 'gh_path="$(command -v gh)"' in workflow
    assert 'gh_path="$(readlink -f "$gh_path")"' in workflow
    assert '"$GITHUB_WORKSPACE"/*|"$RUNNER_TEMP"/*' in workflow
    assert "GH_HOST GH_ENTERPRISE_TOKEN GH_REPO" in workflow

def test_privileged_actions_are_exactly_pinned() -> None:
    policy = json.loads(
        (Path(__file__).parents[2] / "docs/integral/d6u-trusted-builder-policy.json").read_text(
            encoding="utf-8"
        )
    )
    workflow_path = Path(__file__).parents[2] / policy["trusted_workflow"]["path"]
    uses = []
    for line in workflow_path.read_text(encoding="utf-8").splitlines():
        stripped = line.strip()
        if not re.match(r"^(?:-\\s*)?uses\\s*:", stripped):
            continue
        value = stripped.split(":", 1)[1].strip()
        uses.append(value)

    expected_uses = {
        f"{name}@{config['ref']} # {config['version']}"
        for name, config in policy["trusted_actions"].items()
    }
    assert set(uses) == expected_uses
    for action in uses:
        reference = action.split("#", 1)[0].strip().rsplit("@", 1)[-1]
        assert re.fullmatch(r"[0-9a-f]{40}", reference), (
            f"privileged workflow action is not pinned to a full commit SHA: {action!r}"
        )


def test_forbidden_cargo_config_is_rejected() -> None:
    forbidden = [
        ".cargo/config",
        ".cargo/config.toml",
        "d6u-runtime-harness/.cargo/config",
        "d6u-runtime-harness/.cargo/config.toml",
    ]
    tree = {
        "truncated": False,
        "tree": [
            {
                "path": "d6u-runtime-harness/.cargo/config.toml",
                "mode": "100644",
                "type": "blob",
                "sha": "a" * 40,
            }
        ],
    }
    assert_rejected(
        lambda: verify_forbidden_cargo_config_paths(tree, forbidden),
        "Cargo config inherited by D6U was accepted",
    )


def test_harness_file_set_rejects_extra_build_script() -> None:
    expected = [
        "d6u-runtime-harness/Cargo.toml",
        "d6u-runtime-harness/README.md",
        "d6u-runtime-harness/rust-toolchain.toml",
        "d6u-runtime-harness/src/lib.rs",
        "d6u-runtime-harness/tests/d6u_authority_boundary.rs",
    ]
    tree = {
        "truncated": False,
        "tree": [
            *[
                {
                    "path": path,
                    "mode": "100644",
                    "type": "blob",
                    "sha": "a" * 40,
                }
                for path in expected
            ],
            {
                "path": "d6u-runtime-harness/build.rs",
                "mode": "100644",
                "type": "blob",
                "sha": "b" * 40,
            },
        ],
    }
    assert_rejected(
        lambda: verify_exact_harness_file_set(tree, expected),
        "extra executable harness file was accepted",
    )


def test_artifact_size_limits_are_enforced() -> None:
    maximums = {
        "d6u-runtime-evidence.txt": 16,
        "d6u-runtime-test.log": 32,
        "Cargo.lock": 16,
    }
    with tempfile.TemporaryDirectory() as tmp:
        artifact_dir = Path(tmp)
        for name in maximums:
            (artifact_dir / name).write_text("ok", encoding="utf-8")
        verify_artifact_size_limits(artifact_dir, maximums, 64)

        oversized = artifact_dir / "d6u-runtime-test.log"
        oversized.write_bytes(b"x" * (maximums["d6u-runtime-test.log"] + 1))
        assert_rejected(
            lambda: verify_artifact_size_limits(artifact_dir, maximums, 64),
            "oversized trusted artifact member was accepted",
        )

        oversized.write_text("ok", encoding="utf-8")
        (artifact_dir / "Cargo.lock").write_bytes(b"x" * 16)
        (artifact_dir / "d6u-runtime-evidence.txt").write_bytes(b"x" * 16)
        assert_rejected(
            lambda: verify_artifact_size_limits(artifact_dir, maximums, 16),
            "oversized trusted artifact aggregate was accepted",
        )


def test_valid_log_is_accepted() -> None:
    verify_cases(valid_log(), base_policy())


def test_case_tampering_is_rejected() -> None:
    tampered = valid_log().replace(
        "canonical-payload-accepted\taccepted",
        "canonical-payload-accepted\tsemantic-rejected",
    )
    assert_rejected(
        lambda: verify_cases(tampered, base_policy()),
        "tampered D6U case was accepted",
    )


def test_duplicate_case_is_rejected() -> None:
    tampered = valid_log() + "\n" + (
        "D6U_CASE\tcanonical-payload-accepted\taccepted\tzome-reached=true\tPASS"
    )
    assert_rejected(
        lambda: verify_cases(tampered, base_policy()),
        "duplicate D6U case was accepted",
    )


def test_executor_workflow_identity_tampering_is_rejected() -> None:
    policy = {
        "executor_workflow": {
            "name": "D6U Exact-Head Runtime Executor",
            "path": ".github/workflows/d6u-exact-head-runtime-executor.yml",
            "blob_sha": "a" * 40,
        },
    }
    valid = {
        "executor_workflow_file_path": ".github/workflows/d6u-exact-head-runtime-executor.yml",
        "executor_workflow_repository": "Luminous-Dynamics/mycelix",
        "executor_workflow_commit_sha": "c" * 40,
        "executor_workflow_ref": "Luminous-Dynamics/mycelix/.github/workflows/d6u-exact-head-runtime-executor.yml@refs/heads/main",
    }
    verify_executor_workflow_record(valid, policy, "Luminous-Dynamics/mycelix")

    tampered = {**valid, "executor_workflow_repository": "attacker/repo"}
    assert_rejected(
        lambda: verify_executor_workflow_record(tampered, policy, "Luminous-Dynamics/mycelix"),
        "tampered executor repository was accepted",
    )
    wrong_ref = {
        **valid,
        "executor_workflow_ref":
            "Luminous-Dynamics/mycelix/.github/workflows/d6u-exact-head-runtime-executor.yml@refs/heads/not-main",
    }
    assert_rejected(
        lambda: verify_executor_workflow_record(
            wrong_ref, policy, "Luminous-Dynamics/mycelix"
        ),
        "non-default executor ref was accepted",
    )


def test_executor_workflow_is_bound_to_run_head() -> None:
    policy = {
        "repository_identity": {"repository_id": 9000},
        "executor_workflow": {
            "name": "D6U Exact-Head Runtime Executor",
            "path": ".github/workflows/d6u-exact-head-runtime-executor.yml",
            "blob_sha": "a" * 40,
        }
    }
    run = {
        "repository": {"id": 9000, "full_name": "Luminous-Dynamics/mycelix"},
        "head_repository": {"id": 9000, "full_name": "Luminous-Dynamics/mycelix"},
        "head_branch": "main",
        "head_sha": "b" * 40,
    }
    tree = {
        "truncated": False,
        "tree": [{
            "path": policy["executor_workflow"]["path"],
            "mode": "100644",
            "type": "blob",
            "sha": "a" * 40,
        }],
    }
    with patch(
        "verify_d6u_trusted_artifacts.git_tree_from_api",
        return_value=tree,
    ) as fetch_tree:
        verify_executor_workflow_against_run_head(
            run, policy, "Luminous-Dynamics/mycelix", "token"
        )
    fetch_tree.assert_called_once_with(
        "Luminous-Dynamics/mycelix", "b" * 40, "token"
    )


def test_executor_run_live_identity_is_rejected() -> None:
    policy = {
        "repository_identity": {"repository_id": 9001},
        "executor_workflow": {
            "name": "D6U Exact-Head Runtime Executor",
            "path": ".github/workflows/d6u-exact-head-runtime-executor.yml",
        },
    }
    record = {
        "executor_run_id": "42",
        "executor_run_attempt": "3",
        "executor_workflow_commit_sha": "b" * 40,
    }
    valid = {
        "id": 42,
        "run_attempt": 3,
        "name": "D6U Exact-Head Runtime Executor",
        "path": ".github/workflows/d6u-exact-head-runtime-executor.yml",
        "event": "workflow_run",
        "conclusion": "success",
        "repository": {"id": 9001, "full_name": "Luminous-Dynamics/mycelix"},
        "head_repository": {"id": 9001, "full_name": "Luminous-Dynamics/mycelix"},
        "head_branch": "main",
        "head_sha": "b" * 40,
    }
    verify_executor_run_record(valid, record, policy, "Luminous-Dynamics/mycelix")

    for field, value, message in [
        ("repository", {"full_name": "attacker/repo"}, "executor repository"),
        ("head_repository", {"full_name": "attacker/repo"}, "executor head repository"),
        ("head_branch", "attacker-branch", "executor non-main branch"),
        ("id", 43, "executor run ID"),
        ("run_attempt", 4, "executor run attempt"),
        ("conclusion", "failure", "executor conclusion"),
        ("head_sha", "c" * 40, "executor workflow commit/run-head mismatch"),
    ]:
        tampered = {**valid, field: value}
        assert_rejected(
            lambda tampered=tampered: verify_executor_run_record(
                tampered, record, policy, "Luminous-Dynamics/mycelix"
            ),
            f"{message} was accepted",
        )

def test_trigger_run_identity_tampering_is_rejected() -> None:
    policy = {
        "repository_identity": {"repository_id": 9002},
        "source_branch": "myc-int-demo-d6u-holochain-07-runtime",
        "trigger_workflow": {
            "name": "D6S Canonical Qualification",
            "path": ".github/workflows/d6s-canonical-qualification.yml",
            "workflow_id": 371215723,
        },
        "required_source_blobs": {
            ".github/workflows/d6s-canonical-qualification.yml": "f" * 40,
        },
    }
    record = {
        "trigger_workflow_run_attempt": "2",
        "trigger_workflow_name": "D6S Canonical Qualification",
        "trigger_workflow_path": ".github/workflows/d6s-canonical-qualification.yml",
        "trigger_workflow_blob_sha": "f" * 40,
        "source_repository": "Luminous-Dynamics/mycelix",
        "source_branch": "myc-int-demo-d6u-holochain-07-runtime",
        "source_commit": "d" * 40,
    }
    trigger = {
        "repository": {"id": 9002, "full_name": "Luminous-Dynamics/mycelix"},
        "name": "D6S Canonical Qualification",
        "path": ".github/workflows/d6s-canonical-qualification.yml",
        "workflow_id": 371215723,
        "event": "push",
        "conclusion": "success",
        "id": 11,
        "head_repository": {"id": 9002, "full_name": "Luminous-Dynamics/mycelix"},
        "head_branch": "myc-int-demo-d6u-holochain-07-runtime",
        "run_attempt": 2,
        "head_sha": "d" * 40,
    }
    verify_trigger_run_record(record, trigger, policy, "Luminous-Dynamics/mycelix")

    bad_repository = {**trigger, "repository": {"full_name": "attacker/repo"}}
    assert_rejected(
        lambda: verify_trigger_run_record(record, bad_repository, policy, "Luminous-Dynamics/mycelix"),
        "tampered trigger repository was accepted",
    )

    bad_trigger = {**trigger, "id": 12}
    assert_rejected(
        lambda: verify_trigger_run_record(record, bad_trigger, policy, "Luminous-Dynamics/mycelix"),
        "mismatched trigger run ID was accepted",
    )

    bad_trigger = {**trigger, "head_sha": "e" * 40}
    assert_rejected(
        lambda: verify_trigger_run_record(record, bad_trigger, policy, "Luminous-Dynamics/mycelix"),
        "tampered trigger SHA was accepted",
    )

    bad_workflow_id = {**trigger, "workflow_id": 999}
    assert_rejected(
        lambda: verify_trigger_run_record(
            record, bad_workflow_id, policy, "Luminous-Dynamics/mycelix"
        ),
        "tampered trigger workflow ID was accepted",
    )

    bad_blob = {**record, "trigger_workflow_blob_sha": "e" * 40}
    assert_rejected(
        lambda: verify_trigger_run_record(bad_blob, trigger, policy, "Luminous-Dynamics/mycelix"),
        "tampered trigger workflow blob was accepted",
    )


def _graph_policy() -> dict:
    return {
        "lock_packages": {},
        "lock_source": "registry+https://github.com/rust-lang/crates.io-index",
        "lock_graph": {
            "required_registry_source": "registry+https://github.com/rust-lang/crates.io-index",
            "allowed_local_packages": ["d6u-runtime-harness"],
            "lockfile_format_version": 4,
            "manifest_path": "d6u-runtime-harness/Cargo.toml",
            "require_manifest_root_binding": True,
            "require_dependency_edge_closure": True,
            "require_all_packages_reachable": True,
        },
    }


def test_lock_graph_binds_root_to_trusted_manifest() -> None:
    policy = _graph_policy()
    registry = policy["lock_graph"]["required_registry_source"]
    lock = {
        "version": 4,
        "package": [
            {
                "name": "d6u-runtime-harness",
                "version": "0.1.0",
                "dependencies": ["serde 1.0.0"],
            },
            {
                "name": "serde",
                "version": "1.0.0",
                "source": registry,
                "checksum": "a" * 64,
                "dependencies": [],
            },
        ],
    }
    manifest = {
        "package": {"name": "d6u-runtime-harness", "version": "0.1.0"},
        "dependencies": {"serde": "1"},
    }
    verify_lock_graph_against_manifest(lock, manifest, policy)


def test_lock_graph_rejects_root_manifest_dependency_mismatch() -> None:
    policy = _graph_policy()
    registry = policy["lock_graph"]["required_registry_source"]
    lock = {
        "version": 4,
        "package": [
            {
                "name": "d6u-runtime-harness",
                "version": "0.1.0",
                "dependencies": ["serde 1.0.0"],
            },
            {
                "name": "serde",
                "version": "1.0.0",
                "source": registry,
                "checksum": "a" * 64,
                "dependencies": [],
            },
        ],
    }
    manifest = {
        "package": {"name": "d6u-runtime-harness", "version": "0.1.0"},
        "dependencies": {"serde": "1", "sha2": "0.10"},
    }
    assert_rejected(
        lambda: verify_lock_graph_against_manifest(lock, manifest, policy),
        "Cargo.lock root dependency set diverged from trusted Cargo.toml",
    )


def test_lock_graph_rejects_dangling_or_ambiguous_dependency_reference() -> None:
    policy = _graph_policy()
    registry = policy["lock_graph"]["required_registry_source"]
    lock = {
        "version": 4,
        "package": [
            {
                "name": "d6u-runtime-harness",
                "version": "0.1.0",
                "dependencies": ["serde 9.9.9"],
            },
            {
                "name": "serde",
                "version": "1.0.0",
                "source": registry,
                "checksum": "a" * 64,
                "dependencies": [],
            },
        ],
    }
    manifest = {
        "package": {"name": "d6u-runtime-harness", "version": "0.1.0"},
        "dependencies": {"serde": "1"},
    }
    assert_rejected(
        lambda: verify_lock_graph_against_manifest(lock, manifest, policy),
        "dangling Cargo.lock dependency reference was accepted",
    )

    lock["package"].insert(
        2,
        {
            "name": "serde",
            "version": "1.1.0",
            "source": registry,
            "checksum": "b" * 64,
            "dependencies": [],
        },
    )
    lock["package"][0]["dependencies"] = ["serde"]
    assert_rejected(
        lambda: verify_lock_graph_against_manifest(lock, manifest, policy),
        "ambiguous bare Cargo.lock dependency reference was accepted",
    )


def test_lock_graph_rejects_unreachable_package_node() -> None:
    policy = _graph_policy()
    registry = policy["lock_graph"]["required_registry_source"]
    lock = {
        "version": 4,
        "package": [
            {
                "name": "d6u-runtime-harness",
                "version": "0.1.0",
                "dependencies": ["serde 1.0.0"],
            },
            {
                "name": "serde",
                "version": "1.0.0",
                "source": registry,
                "checksum": "a" * 64,
                "dependencies": [],
            },
            {
                "name": "orphan",
                "version": "1.0.0",
                "source": registry,
                "checksum": "b" * 64,
                "dependencies": [],
            },
        ],
    }
    manifest = {
        "package": {"name": "d6u-runtime-harness", "version": "0.1.0"},
        "dependencies": {"serde": "1"},
    }
    assert_rejected(
        lambda: verify_lock_graph_against_manifest(lock, manifest, policy),
        "unreachable Cargo.lock package node was accepted",
    )


def test_lock_graph_rejects_multiple_local_roots() -> None:
    policy = _graph_policy()
    registry = policy["lock_graph"]["required_registry_source"]
    lock = {
        "version": 4,
        "package": [
            {
                "name": "d6u-runtime-harness",
                "version": "0.1.0",
                "dependencies": [],
            },
            {
                "name": "second-local",
                "version": "0.1.0",
                "dependencies": [],
            },
        ],
    }
    manifest = {
        "package": {"name": "d6u-runtime-harness", "version": "0.1.0"},
        "dependencies": {},
    }
    assert_rejected(
        lambda: verify_lock_graph_against_manifest(lock, manifest, policy),
        "multiple local Cargo.lock roots were accepted",
    )


def test_lockfile_format_version_is_pinned() -> None:
    policy = _graph_policy()
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "Cargo.lock"
        path.write_text(
            'version = 3\n\n[[package]]\nname = "d6u-runtime-harness"\nversion = "0.1.0"\n',
            encoding="utf-8",
        )
        assert_rejected(
            lambda: verify_lock(path, policy),
            "Cargo.lock version 3 was accepted under a version-4 policy",
        )


def test_lock_provenance_is_rejected_when_tampered() -> None:
    package = {
        "name": "holochain",
        "version": "0.7.0",
        "source": "registry+https://github.com/rust-lang/crates.io-index",
        "checksum": "a" * 64,
    }
    policy = {
        "lock_packages": {"holochain": "0.7.0"},
        "lock_source": "registry+https://github.com/rust-lang/crates.io-index",
        "lock_graph": {
            "required_registry_source": "registry+https://github.com/rust-lang/crates.io-index",
            "allowed_local_packages": ["d6u-runtime-harness"],
        },
    }

    with tempfile.TemporaryDirectory() as tmp:
        lock = Path(tmp) / "Cargo.lock"
        lock_text = (
            'version = 3\n\n'
            '[[package]]\n'
            'name = "d6u-runtime-harness"\n'
            'version = "0.1.0"\n\n'
            '[[package]]\n'
            + "\n".join(f'{key} = "{value}"' for key, value in package.items())
            + "\n"
        )
        lock.write_text(lock_text, encoding="utf-8")
        verify_lock(lock, policy)

        tampered = lock_text.replace(
            'source = "registry+https://github.com/rust-lang/crates.io-index"',
            'source = "git+https://example.invalid/hostile.git"',
        )
        lock.write_text(tampered, encoding="utf-8")
        assert_rejected(
            lambda: verify_lock(lock, policy),
            "untrusted lock registry source was accepted",
        )

        tampered = lock_text.replace(
            'checksum = "' + ("a" * 64) + '"',
            "",
        )
        lock.write_text(tampered, encoding="utf-8")
        assert_rejected(
            lambda: verify_lock(lock, policy),
            "malformed lock checksum was accepted",
        )

        tampered = lock_text + (
            '[[package]]\n'
            'name = "unauthorized-local"\n'
            'version = "1.0.0"\n'
        )
        lock.write_text(tampered, encoding="utf-8")
        assert_rejected(
            lambda: verify_lock(lock, policy),
            "unauthorized local Cargo.lock package was accepted",
        )


def test_duplicate_record_key_is_rejected() -> None:
    with tempfile.TemporaryDirectory() as tmp:
        record = Path(tmp) / "record.txt"
        record.write_text(
            "D6U HOLOCHAIN 0.7 RUNTIME EVIDENCE\n"
            "status=runtime-reference-evidence\n"
            "status=runtime-reference-evidence\n",
            encoding="utf-8",
        )
        assert_rejected(
            lambda: load_record(record),
            "duplicate evidence key was accepted",
        )


def test_tracked_source_tree_accepts_exact_blobs() -> None:
    required = {
        "tracked.txt": "a" * 40,
        "script.sh": "b" * 40,
    }
    tree = {
        "truncated": False,
        "tree": [
            {
                "path": "tracked.txt",
                "mode": "100644",
                "type": "blob",
                "sha": "a" * 40,
            },
            {
                "path": "script.sh",
                "mode": "100755",
                "type": "blob",
                "sha": "b" * 40,
            },
        ],
    }
    verify_required_tracked_blobs(tree, required)


def test_tracked_source_tree_rejects_symlink_mode() -> None:
    required = {"tracked.txt": "a" * 40}
    tree = {
        "truncated": False,
        "tree": [
            {
                "path": "tracked.txt",
                "mode": "120000",
                "type": "blob",
                "sha": "a" * 40,
            }
        ],
    }
    assert_rejected(
        lambda: verify_required_tracked_blobs(tree, required),
        "Git symlink mode was accepted",
    )


def test_tracked_source_tree_rejects_nonblob_entry() -> None:
    required = {"tracked.txt": "a" * 40}
    tree = {
        "truncated": False,
        "tree": [
            {
                "path": "tracked.txt",
                "mode": "160000",
                "type": "commit",
                "sha": "a" * 40,
            }
        ],
    }
    assert_rejected(
        lambda: verify_required_tracked_blobs(tree, required),
        "Git submodule entry was accepted",
    )


def test_truncated_source_tree_is_rejected() -> None:
    required = {"tracked.txt": "a" * 40}
    tree = {
        "truncated": True,
        "tree": [
            {
                "path": "tracked.txt",
                "mode": "100644",
                "type": "blob",
                "sha": "a" * 40,
            }
        ],
    }
    assert_rejected(
        lambda: verify_required_tracked_blobs(tree, required),
        "truncated Git tree was accepted",
    )


def artifact_policy() -> dict:
    return {
        "artifact_max_bytes": {
            "d6u-runtime-evidence.txt": 64,
            "d6u-runtime-test.log": 128,
            "Cargo.lock": 64,
        },
        "artifact_max_entries": 32,
        "artifact_max_total_bytes": 256,
        "artifact_integrity": {
            "allowed_compression_methods": ["stored", "deflate"],
        },
    }


def write_valid_artifact_zip(path: Path) -> None:
    with ZipFile(path, "w") as archive:
        archive.writestr("d6u-runtime-evidence.txt", "evidence")
        archive.writestr("d6u-runtime-test.log", "log")
        archive.writestr("Cargo.lock", "lock")


def attestation_entry(
    subjects: list[dict],
    source: dict[str, str],
    trigger: dict[str, int | str],
    executor: dict[str, int | str],
    policy_version: int,
    run_id: str = "42",
    attempt: str = "3",
) -> dict:
    repo = "Luminous-Dynamics/mycelix"
    workflow = f"{repo}/.github/workflows/d6u-trusted-evidence-attestation.yml"
    return {
        "verificationResult": {
            "signature": {
                "certificate": {
                    "subjectAlternativeName": f"https://github.com/{workflow}@refs/heads/main",
                    "issuer": "https://token.actions.githubusercontent.com",
                    "githubWorkflowRepository": repo,
                    "githubWorkflowRef": "refs/heads/main",
                    "sourceRepositoryURI": f"https://github.com/{repo}",
                    "sourceRepositoryDigest": "a" * 40,
                    "runnerEnvironment": "github-hosted",
                    "runInvocationURI": (
                        f"https://github.com/{repo}/actions/runs/{run_id}/attempts/{attempt}"
                    ),
                }
            },
            "verifiedTimestamps": [{"type": "Tlog"}],
            "statement": {
                "predicateType": "https://luminousdynamics.io/attestations/d6u-runtime-evidence/v1",
                "subject": subjects,
                "predicate": {
                    "schema": "d6u-trusted-runtime-evidence/v1",
                    "attestation_kind": "verified-runtime-evidence",
                    "claim_ceiling": "ReferenceModelOnly",
                    "policy_version": policy_version,
                    "source": source,
                    "trigger": trigger,
                    "executor": executor,
                    "subjects": subjects,
                    "evidence": {
                        "case_coverage": "14-of-14",
                        "supplemental_coverage": "4-of-4",
                        "application_check_coverage": "1-of-1",
                        "case_outcome_classes": [
                            "accepted",
                            "semantic-rejected",
                            "authentication-failed",
                        ],
                        "runtime": "holochain-0.7.0",
                        "hdk": "0.7.0",
                        "hdi": "0.8.0",
                        "unsupported_cases": [
                            "wire-signature-valid",
                            "nonce-stale",
                            "payload-mutation",
                        ],
                    },
                    "nonclaims": [
                        "semantic-truth",
                        "production-safety",
                        "legal-authority",
                        "actuation-authority",
                    ],
                },
            },
        }
    }



def synthetic_commitment_attestation_entry(subjects: list[dict], canonical_sha256: str, run_id: str) -> dict:
    repo = "Luminous-Dynamics/mycelix"
    return {
        "verificationResult": {
            "signature": {
                "certificate": {
                    "subjectAlternativeName": (
                        "https://github.com/" + repo + "/.github/workflows/"
                        "d6u-trusted-evidence-attestation.yml@refs/heads/main"
                    ),
                    "issuer": "https://token.actions.githubusercontent.com",
                    "githubWorkflowRepository": repo,
                    "githubWorkflowRef": "refs/heads/main",
                    "sourceRepositoryURI": "https://github.com/" + repo,
                    "sourceRepositoryDigest": "a" * 40,
                    "runnerEnvironment": "github-hosted",
                    "runInvocationURI": (
                        "https://github.com/" + repo + "/actions/runs/" + run_id + "/attempts/3"
                    ),
                }
            },
            "verifiedTimestamps": [{"type": "Tlog"}],
            "statement": {
                "predicateType": "https://luminousdynamics.io/attestations/d6u-runtime-evidence/v1",
                "subject": subjects,
                "predicate": {
                    "schema": "d6u-trusted-runtime-evidence-attestation/v2",
                    "attestation_kind": "verified-runtime-evidence",
                    "claim_ceiling": "ReferenceModelOnly",
                    "policy_version": 22,
                    "canonical_predicate_sha256": canonical_sha256,
                },
            },
        }
    }


def _valid_negative_control_fixture():
    import base64
    import hashlib
    import verify_d6u_trusted_attestation_retention as retention

    tmp = tempfile.TemporaryDirectory()
    root = Path(tmp.name)
    evidence_root = root / "d6u-auditor-handoff"
    evidence_root.mkdir()
    subject = evidence_root / "d6u-runtime-evidence.txt"
    subject.write_text("subject\n", encoding="utf-8")
    bundle = root / "d6u-runtime-evidence.attestation.jsonl"
    bundle.write_text("{}\n", encoding="utf-8")
    offline = root / "d6u-runtime-evidence.offline.json"
    offline.write_text("[]\n", encoding="utf-8")
    trusted_root = root / "trusted_root.jsonl"
    trusted_root.write_text("{}\n", encoding="utf-8")
    raw = b"expected public-good rejection\n"

    env = {
        "RUNNER_TEMP": str(root),
        "GITHUB_REPOSITORY": "Luminous-Dynamics/mycelix",
        "GITHUB_WORKFLOW_SHA": "c" * 40,
        "GITHUB_SHA": "a" * 40,
        "GITHUB_REF": "refs/heads/main",
    }
    with patch.dict(os.environ, env, clear=False):
        command = retention.expected_verify_command(
            str(subject),
            env["GITHUB_REPOSITORY"],
            root_path=str(trusted_root),
            bundle_path=str(bundle),
            no_public_good=True,
        )
    control = {
        "schema": "d6u-no-public-good-control/v1",
        "public_good_instance": "sigstore-public-good",
        "subject_name": "d6u-runtime-evidence.txt",
        "subject_path": str(subject),
        "subject_sha256": hashlib.sha256(subject.read_bytes()).hexdigest(),
        "bundle_filename": bundle.name,
        "bundle_path": str(bundle),
        "bundle_sha256": hashlib.sha256(bundle.read_bytes()).hexdigest(),
        "trusted_root_filename": trusted_root.name,
        "trusted_root_path": str(trusted_root),
        "trusted_root_sha256": hashlib.sha256(trusted_root.read_bytes()).hexdigest(),
        "baseline_offline_filename": offline.name,
        "baseline_offline_path": str(offline),
        "baseline_offline_sha256": hashlib.sha256(offline.read_bytes()).hexdigest(),
        "command": command,
        "exit_status": 1,
        "combined_output_base64": base64.b64encode(raw).decode("ascii"),
        "combined_output_bytes": len(raw),
        "combined_output_sha256": hashlib.sha256(raw).hexdigest(),
    }
    return tmp, root, control, bundle, offline, trusted_root, retention


def test_trusted_root_jsonl_line_limit_is_enforced() -> None:
    import verify_d6u_trusted_attestation_retention as retention

    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "trusted_root.jsonl"
        path.write_text(("{}\n" * (retention.MAX_JSONL_LINES + 1)), encoding="utf-8")
        assert_rejected(
            lambda: retention.load_jsonl(path),
            "trusted root JSONL line limit was not enforced",
        )


def test_negative_control_requires_nonzero_exit() -> None:
    tmp, root, control, bundle, offline, trusted_root, retention = _valid_negative_control_fixture()
    try:
        control["exit_status"] = 0
        path = root / "control.json"
        path.write_text(json.dumps(control), encoding="utf-8")
        with patch.dict(
            os.environ,
            {
                "RUNNER_TEMP": str(root),
                "GITHUB_REPOSITORY": "Luminous-Dynamics/mycelix",
                "GITHUB_WORKFLOW_SHA": "c" * 40,
                "GITHUB_SHA": "a" * 40,
                "GITHUB_REF": "refs/heads/main",
            },
            clear=False,
        ):
            assert_rejected(
                lambda: retention.verify_no_public_good_control(
                    path, root, "d6u-runtime-evidence.txt",
                    control["subject_sha256"], bundle.name,
                    control["bundle_sha256"], offline.name,
                    control["baseline_offline_sha256"],
                    control["trusted_root_sha256"],
                ),
                "successful --no-public-good control was accepted",
            )
    finally:
        tmp.cleanup()


def test_negative_control_command_must_disable_public_good() -> None:
    tmp, root, control, bundle, offline, trusted_root, retention = _valid_negative_control_fixture()
    try:
        control["command"] = [item for item in control["command"] if item != "--no-public-good"]
        path = root / "control.json"
        path.write_text(json.dumps(control), encoding="utf-8")
        with patch.dict(
            os.environ,
            {
                "RUNNER_TEMP": str(root),
                "GITHUB_REPOSITORY": "Luminous-Dynamics/mycelix",
                "GITHUB_WORKFLOW_SHA": "c" * 40,
                "GITHUB_SHA": "a" * 40,
                "GITHUB_REF": "refs/heads/main",
            },
            clear=False,
        ):
            assert_rejected(
                lambda: retention.verify_no_public_good_control(
                    path, root, "d6u-runtime-evidence.txt",
                    control["subject_sha256"], bundle.name,
                    control["bundle_sha256"], offline.name,
                    control["baseline_offline_sha256"],
                    control["trusted_root_sha256"],
                ),
                "negative control without --no-public-good was accepted",
            )
    finally:
        tmp.cleanup()


def test_negative_control_cross_link_mismatch_is_rejected() -> None:
    tmp, root, control, bundle, offline, trusted_root, retention = _valid_negative_control_fixture()
    try:
        original = control["bundle_sha256"]
        control["bundle_sha256"] = "b" * 64
        path = root / "control.json"
        path.write_text(json.dumps(control), encoding="utf-8")
        with patch.dict(
            os.environ,
            {
                "RUNNER_TEMP": str(root),
                "GITHUB_REPOSITORY": "Luminous-Dynamics/mycelix",
                "GITHUB_WORKFLOW_SHA": "c" * 40,
                "GITHUB_SHA": "a" * 40,
                "GITHUB_REF": "refs/heads/main",
            },
            clear=False,
        ):
            assert_rejected(
                lambda: retention.verify_no_public_good_control(
                    path, root, "d6u-runtime-evidence.txt",
                    control["subject_sha256"], bundle.name,
                    original, offline.name,
                    control["baseline_offline_sha256"],
                    control["trusted_root_sha256"],
                ),
                "cross-linked negative control bundle identity was accepted",
            )
    finally:
        tmp.cleanup()



def test_retained_report_identity_and_predicate_are_bound() -> None:
    import verify_d6u_trusted_attestation_retention as retention

    subjects = [
        {"name": "d6u-runtime-evidence.txt", "digest": {"sha256": "a" * 64}},
        {"name": "d6u-runtime-test.log", "digest": {"sha256": "b" * 64}},
        {"name": "Cargo.lock", "digest": {"sha256": "c" * 64}},
    ]
    canonical_hash = hashlib.sha256(b"canonical-predicate").hexdigest()
    entry = synthetic_commitment_attestation_entry(subjects, canonical_hash, "42")
    context = {
        "repository": "Luminous-Dynamics/mycelix",
        "source_digest": "a" * 40,
        "run_invocation_uri": (
            "https://github.com/Luminous-Dynamics/mycelix/actions/runs/42/attempts/3"
        ),
        "certificate_identity": (
            "https://github.com/Luminous-Dynamics/mycelix/.github/workflows/"
            "d6u-trusted-evidence-attestation.yml@refs/heads/main"
        ),
        "certificate_oidc_issuer": "https://token.actions.githubusercontent.com",
        "predicate_type": "https://luminousdynamics.io/attestations/d6u-runtime-evidence/v1",
        "predicate_schema": "d6u-trusted-runtime-evidence-attestation/v2",
        "policy_version": "22",
        "canonical_predicate_sha256": canonical_hash,
        "claim_ceiling": "ReferenceModelOnly",
    }
    with tempfile.TemporaryDirectory() as tmp:
        report = Path(tmp) / "report.json"
        report.write_text(json.dumps([entry]), encoding="utf-8")
        observed = retention.verify_report(report, subjects and [
            (item["name"], item["digest"]["sha256"]) for item in subjects
        ], context)
        assert len(observed) == 64

        tampered_certificate = json.loads(report.read_text(encoding="utf-8"))
        tampered_certificate[0]["verificationResult"]["signature"]["certificate"][
            "sourceRepositoryDigest"
        ] = "b" * 40
        report.write_text(json.dumps(tampered_certificate), encoding="utf-8")
        assert_rejected(
            lambda: retention.verify_report(
                report,
                [(item["name"], item["digest"]["sha256"]) for item in subjects],
                context,
            ),
            "retained report with mismatched source identity was accepted",
        )

        report.write_text(json.dumps([entry]), encoding="utf-8")
        tampered_predicate = json.loads(report.read_text(encoding="utf-8"))
        tampered_predicate[0]["verificationResult"]["statement"]["predicate"][
            "claim_ceiling"
        ] = "Production"
        report.write_text(json.dumps(tampered_predicate), encoding="utf-8")
        assert_rejected(
            lambda: retention.verify_report(
                report,
                [(item["name"], item["digest"]["sha256"]) for item in subjects],
                context,
            ),
            "retained report with mismatched claim ceiling was accepted",
        )


def test_retention_packet_rejects_extra_member() -> None:
    import verify_d6u_trusted_attestation_retention as retention

    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        (root / "unexpected.json").write_text("{}", encoding="utf-8")
        with patch("sys.argv", ["verify_d6u_trusted_attestation_retention.py", str(root)]):
            assert_rejected(
                lambda: retention.main(),
                "retention packet accepted an unexpected member",
            )


def test_trusted_zip_accepts_exact_members() -> None:
    with tempfile.TemporaryDirectory() as tmp:
        archive = Path(tmp) / "artifact.zip"
        write_valid_artifact_zip(archive)
        infos = verify_zip_members(archive, artifact_policy())
        assert {info.filename for info in infos} == EXPECTED_FILES


def test_handoff_zip_uses_handoff_compression_policy() -> None:
    policy = {
        "auditor_handoff": {
            "allowed_compression_methods": ["stored"],
        }
    }
    expected_files = HANDOFF_EXPECTED_FILES
    maximums = {name: 64 for name in expected_files}
    with tempfile.TemporaryDirectory() as tmp:
        archive = Path(tmp) / "handoff.zip"
        with ZipFile(archive, "w", compression=8) as zip_file:
            for name in sorted(expected_files):
                zip_file.writestr(name, "x")
        assert_rejected(
            lambda: verify_zip_members(
                archive,
                policy,
                expected_files=expected_files,
                maximums=maximums,
                maximum_entries=16,
                maximum_total=256,
                allowed_compression_methods=policy["auditor_handoff"]["allowed_compression_methods"],
            ),
            "handoff ZIP accepted a compression method outside its policy",
        )


def test_trusted_zip_rejects_duplicate_member() -> None:
    with tempfile.TemporaryDirectory() as tmp:
        archive = Path(tmp) / "artifact.zip"
        with ZipFile(archive, "w") as zip_file:
            zip_file.writestr("d6u-runtime-evidence.txt", "one")
            zip_file.writestr("d6u-runtime-evidence.txt", "two")
            zip_file.writestr("d6u-runtime-test.log", "log")
            zip_file.writestr("Cargo.lock", "lock")
        assert_rejected(
            lambda: verify_zip_members(archive, artifact_policy()),
            "duplicate ZIP member was accepted",
        )


def test_trusted_zip_rejects_symlink_member() -> None:
    with tempfile.TemporaryDirectory() as tmp:
        archive = Path(tmp) / "artifact.zip"
        link = ZipInfo("d6u-runtime-test.log")
        link.external_attr = (stat.S_IFLNK | 0o777) << 16
        with ZipFile(archive, "w") as zip_file:
            zip_file.writestr("d6u-runtime-evidence.txt", "evidence")
            zip_file.writestr(link, "Cargo.lock")
            zip_file.writestr("Cargo.lock", "lock")
        assert_rejected(
            lambda: verify_zip_members(archive, artifact_policy()),
            "symlink ZIP member was accepted",
        )


def test_trusted_zip_rejects_unexpected_member_path() -> None:
    with tempfile.TemporaryDirectory() as tmp:
        archive = Path(tmp) / "artifact.zip"
        with ZipFile(archive, "w") as zip_file:
            zip_file.writestr("../escape", "bad")
            zip_file.writestr("d6u-runtime-test.log", "log")
            zip_file.writestr("Cargo.lock", "lock")
        assert_rejected(
            lambda: verify_zip_members(archive, artifact_policy()),
            "unexpected ZIP member path was accepted",
        )



def synthetic_record() -> dict[str, str]:
    return {
        "claim_ceiling": "ReferenceModelOnly",
        "source_repository": "Luminous-Dynamics/mycelix",
        "source_branch": "myc-int-demo-d6u-holochain-07-runtime",
        "source_commit": "b" * 40,
        "trigger_workflow_name": "D6S Canonical Qualification",
        "trigger_workflow_path": ".github/workflows/d6s-canonical-qualification.yml",
        "trigger_workflow_run_id": "77",
        "trigger_workflow_run_attempt": "2",
        "executor_workflow_file_path": ".github/workflows/d6u-exact-head-runtime-executor.yml",
        "executor_run_id": "42",
        "executor_run_attempt": "3",
        "executor_workflow_commit_sha": "c" * 40,
        "case_coverage": "14-of-14",
        "supplemental_coverage": "4-of-4",
        "application_check_coverage": "1-of-1",
        "case_outcome_classes": "accepted,semantic-rejected,authentication-failed",
        "runtime": "holochain-0.7.0",
        "hdk": "0.7.0",
        "hdi": "0.8.0",
        "unsupported_cases": "wire-signature-valid,nonce-stale,payload-mutation",
    }


def write_synthetic_attestation_fixture(evidence_dir: Path) -> list[dict]:
    record = synthetic_record()
    lines = ["D6U HOLOCHAIN 0.7 RUNTIME EVIDENCE"]
    lines.extend(f"{key}={value}" for key, value in record.items())
    (evidence_dir / "d6u-runtime-evidence.txt").write_text(
        "\n".join(lines) + "\n", encoding="utf-8"
    )
    (evidence_dir / "d6u-runtime-test.log").write_text("synthetic log\n", encoding="utf-8")
    (evidence_dir / "Cargo.lock").write_text("version = 3\n", encoding="utf-8")
    subjects = [
        {"name": name, "digest": {"sha256": hashlib.sha256((evidence_dir / name).read_bytes()).hexdigest()}}
        for name in ("d6u-runtime-evidence.txt", "d6u-runtime-test.log", "Cargo.lock")
    ]
    predicate = {
        "schema": "d6u-trusted-runtime-evidence/v1",
        "attestation_kind": "verified-runtime-evidence",
        "claim_ceiling": record["claim_ceiling"],
        "policy_version": 22,
        "source": {
            "repository": record["source_repository"],
            "branch": record["source_branch"],
            "commit": record["source_commit"],
        },
        "trigger": {
            "workflow_name": record["trigger_workflow_name"],
            "workflow_path": record["trigger_workflow_path"],
            "run_id": int(record["trigger_workflow_run_id"]),
            "run_attempt": int(record["trigger_workflow_run_attempt"]),
        },
        "executor": {
            "workflow_name": "D6U Exact-Head Runtime Executor",
            "workflow_path": record["executor_workflow_file_path"],
            "run_id": int(record["executor_run_id"]),
            "run_attempt": int(record["executor_run_attempt"]),
            "workflow_commit": record["executor_workflow_commit_sha"],
        },
        "subjects": subjects,
        "evidence": {
            "case_coverage": record["case_coverage"],
            "supplemental_coverage": record["supplemental_coverage"],
            "application_check_coverage": record["application_check_coverage"],
            "case_outcome_classes": record["case_outcome_classes"].split(","),
            "runtime": record["runtime"],
            "hdk": record["hdk"],
            "hdi": record["hdi"],
            "unsupported_cases": record["unsupported_cases"].split(","),
        },
        "nonclaims": [
            "semantic-truth",
            "production-safety",
            "legal-authority",
            "actuation-authority",
        ],
    }
    (evidence_dir / "d6u-trusted-evidence-predicate.json").write_text(
        json.dumps(predicate, sort_keys=True, indent=2) + "\n",
        encoding="utf-8",
    )
    return subjects

def test_attestation_verifier_accepts_current_run() -> None:
    subjects = None
    with tempfile.TemporaryDirectory() as tmp:
        evidence_dir = Path(tmp)
        subjects = write_synthetic_attestation_fixture(evidence_dir)
        report = Path(tmp) / "attestation.json"
        report.write_text(
            json.dumps([synthetic_commitment_attestation_entry(subjects, hashlib.sha256((evidence_dir / "d6u-trusted-evidence-predicate.json").read_bytes()).hexdigest(), "42")]),
            encoding="utf-8",
        )
        with patch.dict(
            os.environ,
            {
                "GITHUB_REPOSITORY": "Luminous-Dynamics/mycelix",
                "GITHUB_RUN_ID": "42",
                "GITHUB_RUN_ATTEMPT": "3",
                "GITHUB_SHA": "a" * 40,
                "GITHUB_WORKFLOW_SHA": "c" * 40,
                "GITHUB_REF": "refs/heads/main",
                "D6U_ATTESTATION_SUBJECT": str(evidence_dir / "d6u-runtime-evidence.txt"),
                "D6U_TRUSTED_EVIDENCE_DIR": str(evidence_dir),
                "D6U_TRUSTED_POLICY_VERSION": "22",
                "D6U_TRIGGER_REPOSITORY": "Luminous-Dynamics/mycelix",
                "D6U_TRIGGER_HEAD_BRANCH": "myc-int-demo-d6u-holochain-07-runtime",
                "D6U_TRIGGER_HEAD_SHA": "b" * 40,
                "D6U_TRIGGER_RUN_ID": "42",
                "D6U_TRIGGER_RUN_ATTEMPT": "3",
            },
            clear=False,
        ), patch("sys.argv", ["verify_d6u_trusted_attestation.py", str(report)]):
            verify_attestation_main()


def test_attestation_verifier_rejects_old_run() -> None:
    with tempfile.TemporaryDirectory() as tmp:
        evidence_dir = Path(tmp)
        subjects = write_synthetic_attestation_fixture(evidence_dir)
        report = Path(tmp) / "attestation.json"
        report.write_text(
            json.dumps([synthetic_commitment_attestation_entry(subjects, hashlib.sha256((evidence_dir / "d6u-trusted-evidence-predicate.json").read_bytes()).hexdigest(), "41")]),
            encoding="utf-8",
        )
        with patch.dict(
            os.environ,
            {
                "GITHUB_REPOSITORY": "Luminous-Dynamics/mycelix",
                "GITHUB_RUN_ID": "42",
                "GITHUB_RUN_ATTEMPT": "3",
                "GITHUB_SHA": "a" * 40,
                "GITHUB_WORKFLOW_SHA": "c" * 40,
                "GITHUB_REF": "refs/heads/main",
                "D6U_ATTESTATION_SUBJECT": str(evidence_dir / "d6u-runtime-evidence.txt"),
                "D6U_TRUSTED_EVIDENCE_DIR": str(evidence_dir),
                "D6U_TRUSTED_POLICY_VERSION": "22",
                "D6U_TRIGGER_REPOSITORY": "Luminous-Dynamics/mycelix",
                "D6U_TRIGGER_HEAD_BRANCH": "myc-int-demo-d6u-holochain-07-runtime",
                "D6U_TRIGGER_HEAD_SHA": "b" * 40,
                "D6U_TRIGGER_RUN_ID": "42",
                "D6U_TRIGGER_RUN_ATTEMPT": "3",
            },
            clear=False,
        ), patch("sys.argv", ["verify_d6u_trusted_attestation.py", str(report)]):
            assert_rejected(
                lambda: verify_attestation_main(),
                "historical attestation was accepted as the current trusted run",
            )


def test_commitment_attestation_rejects_malformed_entry_types() -> None:
    import verify_d6u_trusted_attestation as verifier

    with tempfile.TemporaryDirectory() as tmp:
        evidence_dir = Path(tmp)
        subjects = write_synthetic_attestation_fixture(evidence_dir)
        canonical_sha = hashlib.sha256(
            (evidence_dir / "d6u-trusted-evidence-predicate.json").read_bytes()
        ).hexdigest()
        with patch.dict(os.environ, {
            "GITHUB_REPOSITORY": "Luminous-Dynamics/mycelix",
            "GITHUB_RUN_ID": "42",
            "GITHUB_RUN_ATTEMPT": "3",
            "GITHUB_SHA": "a" * 40,
            "D6U_TRUSTED_POLICY_VERSION": "22",
        }, clear=False):
            assert verifier.verify_commitment_entry(
                [],
                synthetic_record(),
                subjects,
                canonical_sha,
            ) is False


def test_commitment_attestation_rejects_canonical_predicate_tampering() -> None:
    import verify_d6u_trusted_attestation as verifier

    with tempfile.TemporaryDirectory() as tmp:
        evidence_dir = Path(tmp)
        subjects = write_synthetic_attestation_fixture(evidence_dir)
        canonical = evidence_dir / "d6u-trusted-evidence-predicate.json"
        canonical_hash = hashlib.sha256(canonical.read_bytes()).hexdigest()
        report = Path(tmp) / "attestation.json"
        report.write_text(json.dumps([synthetic_commitment_attestation_entry(subjects, canonical_hash, "42")]), encoding="utf-8")
        original = canonical.read_bytes()
        canonical.write_text(canonical.read_text(encoding="utf-8").replace("14-of-14", "13-of-14"), encoding="utf-8")
        with patch.dict(os.environ, {
            "GITHUB_REPOSITORY": "Luminous-Dynamics/mycelix",
            "GITHUB_RUN_ID": "42",
            "GITHUB_RUN_ATTEMPT": "3",
            "GITHUB_SHA": "a" * 40,
            "GITHUB_WORKFLOW_SHA": "c" * 40,
            "GITHUB_REF": "refs/heads/main",
            "D6U_ATTESTATION_SUBJECT": str(evidence_dir / "d6u-runtime-evidence.txt"),
            "D6U_TRUSTED_EVIDENCE_DIR": str(evidence_dir),
            "D6U_TRUSTED_POLICY_VERSION": "22",
            "D6U_TRIGGER_REPOSITORY": "Luminous-Dynamics/mycelix",
            "D6U_TRIGGER_HEAD_BRANCH": "myc-int-demo-d6u-holochain-07-runtime",
            "D6U_TRIGGER_HEAD_SHA": "b" * 40,
            "D6U_TRIGGER_RUN_ID": "42",
            "D6U_TRIGGER_RUN_ATTEMPT": "3",
        }, clear=False), patch("sys.argv", ["verify_d6u_trusted_attestation.py", str(report)]):
            assert_rejected(lambda: verifier.main(), "tampered canonical predicate was accepted")

def test_commitment_attestation_rejects_trigger_source_mismatch() -> None:
    import verify_d6u_trusted_attestation as verifier

    with tempfile.TemporaryDirectory() as tmp:
        evidence_dir = Path(tmp)
        subjects = write_synthetic_attestation_fixture(evidence_dir)
        canonical_sha = hashlib.sha256((evidence_dir / "d6u-trusted-evidence-predicate.json").read_bytes()).hexdigest()
        report = Path(tmp) / "attestation.json"
        report.write_text(json.dumps([synthetic_commitment_attestation_entry(subjects, canonical_sha, "42")]), encoding="utf-8")
        with patch.dict(os.environ, {
            "GITHUB_REPOSITORY": "Luminous-Dynamics/mycelix",
            "GITHUB_RUN_ID": "42",
            "GITHUB_RUN_ATTEMPT": "3",
            "GITHUB_SHA": "a" * 40,
            "GITHUB_WORKFLOW_SHA": "c" * 40,
            "GITHUB_REF": "refs/heads/main",
            "D6U_ATTESTATION_SUBJECT": str(evidence_dir / "d6u-runtime-evidence.txt"),
            "D6U_TRUSTED_EVIDENCE_DIR": str(evidence_dir),
            "D6U_TRUSTED_POLICY_VERSION": "22",
            "D6U_TRIGGER_REPOSITORY": "Luminous-Dynamics/mycelix",
            "D6U_TRIGGER_HEAD_BRANCH": "myc-int-demo-d6u-holochain-07-runtime",
            "D6U_TRIGGER_HEAD_SHA": "e" * 40,
            "D6U_TRIGGER_RUN_ID": "42",
            "D6U_TRIGGER_RUN_ATTEMPT": "3",
        }, clear=False), patch("sys.argv", ["verify_d6u_trusted_attestation.py", str(report)]):
            assert_rejected(lambda: verifier.main(), "mismatched triggering source commit was accepted")

        with patch.dict(os.environ, {
            "GITHUB_REPOSITORY": "Luminous-Dynamics/mycelix",
            "GITHUB_RUN_ID": "42",
            "GITHUB_RUN_ATTEMPT": "3",
            "GITHUB_SHA": "a" * 40,
            "GITHUB_WORKFLOW_SHA": "c" * 40,
            "GITHUB_REF": "refs/heads/main",
            "D6U_ATTESTATION_SUBJECT": str(evidence_dir / "d6u-runtime-evidence.txt"),
            "D6U_TRUSTED_EVIDENCE_DIR": str(evidence_dir),
            "D6U_TRUSTED_POLICY_VERSION": "22",
            "D6U_TRIGGER_REPOSITORY": "Luminous-Dynamics/mycelix",
            "D6U_TRIGGER_HEAD_BRANCH": "myc-int-demo-d6u-holochain-07-runtime",
            "D6U_TRIGGER_HEAD_SHA": "b" * 40,
            "D6U_TRIGGER_RUN_ID": "43",
            "D6U_TRIGGER_RUN_ATTEMPT": "3",
        }, clear=False), patch(
            "sys.argv", ["verify_d6u_trusted_attestation.py", str(report)]
        ):
            assert_rejected(lambda: verifier.main(), "mismatched triggering run ID was accepted")

def test_commitment_attestation_rejects_hash_mismatch() -> None:
    import verify_d6u_trusted_attestation as verifier

    with tempfile.TemporaryDirectory() as tmp:
        evidence_dir = Path(tmp)
        subjects = write_synthetic_attestation_fixture(evidence_dir)
        report = Path(tmp) / "attestation.json"
        report.write_text(json.dumps([synthetic_commitment_attestation_entry(subjects, "b" * 64, "42")]), encoding="utf-8")
        with patch.dict(os.environ, {
            "GITHUB_REPOSITORY": "Luminous-Dynamics/mycelix",
            "GITHUB_RUN_ID": "42",
            "GITHUB_RUN_ATTEMPT": "3",
            "GITHUB_SHA": "a" * 40,
            "GITHUB_WORKFLOW_SHA": "c" * 40,
            "GITHUB_REF": "refs/heads/main",
            "D6U_ATTESTATION_SUBJECT": str(evidence_dir / "d6u-runtime-evidence.txt"),
            "D6U_TRUSTED_EVIDENCE_DIR": str(evidence_dir),
            "D6U_TRUSTED_POLICY_VERSION": "22",
            "D6U_TRIGGER_REPOSITORY": "Luminous-Dynamics/mycelix",
            "D6U_TRIGGER_HEAD_BRANCH": "myc-int-demo-d6u-holochain-07-runtime",
            "D6U_TRIGGER_HEAD_SHA": "b" * 40,
            "D6U_TRIGGER_RUN_ID": "42",
            "D6U_TRIGGER_RUN_ATTEMPT": "3",
        }, clear=False), patch("sys.argv", ["verify_d6u_trusted_attestation.py", str(report)]):
            assert_rejected(lambda: verifier.main(), "attestation hash mismatch was accepted")

def test_commitment_attestation_requires_verified_timestamp() -> None:
    import verify_d6u_trusted_attestation as verifier

    with tempfile.TemporaryDirectory() as tmp:
        evidence_dir = Path(tmp)
        subjects = write_synthetic_attestation_fixture(evidence_dir)
        canonical_sha = hashlib.sha256((evidence_dir / "d6u-trusted-evidence-predicate.json").read_bytes()).hexdigest()
        entry = synthetic_commitment_attestation_entry(subjects, canonical_sha, "42")
        entry["verificationResult"]["verifiedTimestamps"] = []
        with patch.dict(os.environ, {
            "GITHUB_REPOSITORY": "Luminous-Dynamics/mycelix",
            "GITHUB_RUN_ID": "42",
            "GITHUB_RUN_ATTEMPT": "3",
            "GITHUB_SHA": "a" * 40,
            "D6U_TRUSTED_POLICY_VERSION": "22",
            "D6U_TRIGGER_REPOSITORY": "Luminous-Dynamics/mycelix",
            "D6U_TRIGGER_HEAD_BRANCH": "myc-int-demo-d6u-holochain-07-runtime",
            "D6U_TRIGGER_HEAD_SHA": "b" * 40,
            "D6U_TRIGGER_RUN_ID": "42",
            "D6U_TRIGGER_RUN_ATTEMPT": "3",
        }, clear=False):
            assert verifier.verify_commitment_entry(entry, synthetic_record(), subjects, canonical_sha) is False


def test_commitment_attestation_rejects_non_tlog_timestamp() -> None:
    import verify_d6u_trusted_attestation as verifier

    with tempfile.TemporaryDirectory() as tmp:
        evidence_dir = Path(tmp)
        subjects = write_synthetic_attestation_fixture(evidence_dir)
        canonical_sha = hashlib.sha256((evidence_dir / "d6u-trusted-evidence-predicate.json").read_bytes()).hexdigest()
        entry = synthetic_commitment_attestation_entry(subjects, canonical_sha, "42")
        entry["verificationResult"]["verifiedTimestamps"] = [{"type": "RFC3161"}]
        with patch.dict(os.environ, {
            "GITHUB_REPOSITORY": "Luminous-Dynamics/mycelix",
            "GITHUB_RUN_ID": "42",
            "GITHUB_RUN_ATTEMPT": "3",
            "GITHUB_SHA": "a" * 40,
            "D6U_TRUSTED_POLICY_VERSION": "22",
            "D6U_TRIGGER_REPOSITORY": "Luminous-Dynamics/mycelix",
            "D6U_TRIGGER_HEAD_BRANCH": "myc-int-demo-d6u-holochain-07-runtime",
            "D6U_TRIGGER_HEAD_SHA": "b" * 40,
            "D6U_TRIGGER_RUN_ID": "42",
            "D6U_TRIGGER_RUN_ATTEMPT": "3",
        }, clear=False):
            assert verifier.verify_commitment_entry(entry, synthetic_record(), subjects, canonical_sha) is False


def test_commitment_attestation_subject_set_is_order_independent_but_exact() -> None:
    import verify_d6u_trusted_attestation as verifier

    with tempfile.TemporaryDirectory() as tmp:
        evidence_dir = Path(tmp)
        subjects = write_synthetic_attestation_fixture(evidence_dir)
        canonical_sha = hashlib.sha256((evidence_dir / "d6u-trusted-evidence-predicate.json").read_bytes()).hexdigest()
        reordered = [subjects[2], subjects[0], subjects[1]]
        current = synthetic_commitment_attestation_entry(subjects, canonical_sha, "42")
        current["verificationResult"]["statement"]["subject"] = reordered
        duplicate = synthetic_commitment_attestation_entry(subjects, canonical_sha, "42")
        duplicate["verificationResult"]["statement"]["subject"] = subjects + [dict(subjects[0])]
        with patch.dict(os.environ, {
            "GITHUB_REPOSITORY": "Luminous-Dynamics/mycelix",
            "GITHUB_RUN_ID": "42",
            "GITHUB_RUN_ATTEMPT": "3",
            "GITHUB_SHA": "a" * 40,
            "D6U_TRUSTED_POLICY_VERSION": "22",
            "D6U_TRIGGER_REPOSITORY": "Luminous-Dynamics/mycelix",
            "D6U_TRIGGER_HEAD_BRANCH": "myc-int-demo-d6u-holochain-07-runtime",
            "D6U_TRIGGER_HEAD_SHA": "b" * 40,
            "D6U_TRIGGER_RUN_ID": "42",
            "D6U_TRIGGER_RUN_ATTEMPT": "3",
        }, clear=False):
            assert verifier.verify_commitment_entry(current, synthetic_record(), subjects, canonical_sha) is True
            assert verifier.verify_commitment_entry(duplicate, synthetic_record(), subjects, canonical_sha) is False


def test_commitment_attestation_accepts_current_run_and_rejects_old_run() -> None:
    import verify_d6u_trusted_attestation as verifier

    with tempfile.TemporaryDirectory() as tmp:
        evidence_dir = Path(tmp)
        subjects = write_synthetic_attestation_fixture(evidence_dir)
        canonical_sha = hashlib.sha256((evidence_dir / "d6u-trusted-evidence-predicate.json").read_bytes()).hexdigest()
        current = synthetic_commitment_attestation_entry(subjects, canonical_sha, "42")
        old = synthetic_commitment_attestation_entry(subjects, canonical_sha, "41")
        with patch.dict(os.environ, {
            "GITHUB_REPOSITORY": "Luminous-Dynamics/mycelix",
            "GITHUB_RUN_ID": "42",
            "GITHUB_RUN_ATTEMPT": "3",
            "GITHUB_SHA": "a" * 40,
            "D6U_TRUSTED_POLICY_VERSION": "22",
            "D6U_TRIGGER_REPOSITORY": "Luminous-Dynamics/mycelix",
            "D6U_TRIGGER_HEAD_BRANCH": "myc-int-demo-d6u-holochain-07-runtime",
            "D6U_TRIGGER_HEAD_SHA": "b" * 40,
            "D6U_TRIGGER_RUN_ID": "42",
            "D6U_TRIGGER_RUN_ATTEMPT": "3",
        }, clear=False):
            assert verifier.verify_commitment_entry(current, synthetic_record(), subjects, canonical_sha) is True
            assert verifier.verify_commitment_entry(old, synthetic_record(), subjects, canonical_sha) is False

def test_artifact_entry_limit_is_enforced() -> None:
    maximums = {"evidence.txt": 16, "runtime.log": 16, "Cargo.lock": 16}
    with tempfile.TemporaryDirectory() as tmp:
        artifact_dir = Path(tmp)
        for name in maximums:
            (artifact_dir / name).write_text("ok", encoding="utf-8")
        verify_artifact_layout(artifact_dir, set(maximums), 3)
        (artifact_dir / "extra.txt").write_text("x", encoding="utf-8")
        assert_rejected(
            lambda: verify_artifact_layout(artifact_dir, set(maximums), 3),
            "artifact entry-count limit was not enforced",
        )


def test_trusted_workflow_policy_shape_is_pinned() -> None:
    policy = {
        "trusted_workflow": {
            "path": ".github/workflows/d6u-trusted-evidence-attestation.yml",
            "blob_sha": "a" * 40,
        }
    }
    tree = {
        "truncated": False,
        "tree": [{
            "path": policy["trusted_workflow"]["path"],
            "mode": "100644",
            "type": "blob",
            "sha": "a" * 40,
        }],
    }
    verify_required_tracked_blobs(
        tree,
        {policy["trusted_workflow"]["path"]: policy["trusted_workflow"]["blob_sha"]},
    )


def test_artifact_layout_rejects_symlink() -> None:
    with tempfile.TemporaryDirectory() as tmp:
        artifact_dir = Path(tmp)
        (artifact_dir / "d6u-runtime-evidence.txt").write_text(
            "D6U HOLOCHAIN 0.7 RUNTIME EVIDENCE\nstatus=runtime-reference-evidence\n",
            encoding="utf-8",
        )
        (artifact_dir / "Cargo.lock").write_text("version = 3\n", encoding="utf-8")
        (artifact_dir / "d6u-runtime-test.log").symlink_to(
            artifact_dir / "Cargo.lock"
        )

        assert_rejected(
            lambda: verify_artifact_layout(
                artifact_dir,
                {
                    "d6u-runtime-evidence.txt",
                    "d6u-runtime-test.log",
                    "Cargo.lock",
                },
                8,
            ),
            "symlinked trusted input was accepted",
        )



def test_executor_artifact_binds_trigger_head_and_attempt() -> None:
    import fetch_d6u_trusted_artifact as fetcher

    policy = {
        "workflow_name": "D6U Exact-Head Runtime Executor",
        "workflow_path": ".github/workflows/d6u-exact-head-runtime-executor.yml",
        "source_branch": "myc-int-demo-d6u-holochain-07-runtime",
        "artifact_max_total_bytes": 4096,
        "repository_identity": {
            "full_name": "Luminous-Dynamics/mycelix",
            "repository_id": 900,
        },
    }
    event = {
        "workflow_run": {
            "id": 700,
            "run_attempt": 3,
            "event": "workflow_run",
            "name": "D6U Exact-Head Runtime Executor",
            "path": ".github/workflows/d6u-exact-head-runtime-executor.yml",
            "conclusion": "success",
            "repository": {"id": 900, "full_name": "Luminous-Dynamics/mycelix"},
            "head_repository": {"id": 900, "full_name": "Luminous-Dynamics/mycelix"},
            "head_branch": "myc-int-demo-d6u-holochain-07-runtime",
            "head_sha": "a" * 40,
        },
        "repository": {"id": 900, "full_name": "Luminous-Dynamics/mycelix"},
    }
    current_run = {
        "id": 700,
        "run_attempt": 3,
        "repository": {"id": 900, "full_name": "Luminous-Dynamics/mycelix"},
        "head_repository": {"id": 900, "full_name": "Luminous-Dynamics/mycelix"},
        "head_branch": "myc-int-demo-d6u-holochain-07-runtime",
        "head_sha": "a" * 40,
    }
    payload = {
        "artifacts": [{
            "id": 9100,
            "name": "d6u-runtime-evidence-run-700-attempt-3",
            "expired": False,
            "size_in_bytes": 512,
            "digest": "sha256:" + "b" * 64,
            "workflow_run": {
                "id": 700,
                "repository_id": 900,
                "head_repository_id": 900,
                "head_branch": "myc-int-demo-d6u-holochain-07-runtime",
                "head_sha": "a" * 40,
            },
        }]
    }

    def fake_github_get(_repo, api_path, _token):
        if api_path == "/actions/runs/700":
            return current_run
        if api_path.startswith("/actions/runs/700/artifacts?"):
            return payload
        raise AssertionError(f"unexpected GitHub API path: {api_path}")

    with patch.dict(
        os.environ,
        {
            "GITHUB_TOKEN": "token",
        "D6U_TRUSTED_REPOSITORY_ID": "9001",
            "D6U_TRIGGER_HEAD_BRANCH": "myc-int-demo-d6u-holochain-07-runtime",
            "D6U_TRIGGER_HEAD_SHA": "a" * 40,
            "D6U_TRUSTED_REPOSITORY_ID": "900",
        },
        clear=False,
    ), patch.object(fetcher, "github_get", side_effect=fake_github_get):
        observed = expected_artifact("Luminous-Dynamics/mycelix", event, policy)
    assert observed["id"] == 9100

    for event_field, bad_value, message in [
        ("head_branch", "main", "trigger event branch mismatch was accepted"),
        ("head_sha", "c" * 40, "trigger event SHA mismatch was accepted"),
    ]:
        bad_event = json.loads(json.dumps(event))
        bad_event["workflow_run"][event_field] = bad_value
        with patch.dict(
            os.environ,
            {
                "GITHUB_TOKEN": "token",
        "D6U_TRUSTED_REPOSITORY_ID": "9001",
                "D6U_TRIGGER_HEAD_BRANCH": "myc-int-demo-d6u-holochain-07-runtime",
                "D6U_TRIGGER_HEAD_SHA": "a" * 40,
            },
            clear=False,
        ):
            assert_rejected(
                lambda: expected_artifact("Luminous-Dynamics/mycelix", bad_event, policy),
                message,
            )

    bad_event = json.loads(json.dumps(event))
    bad_event["repository"]["id"] = 901
    with patch.dict(os.environ, {"D6U_TRUSTED_REPOSITORY_ID": "900"}, clear=False):
        assert_rejected(
            lambda: expected_artifact("Luminous-Dynamics/mycelix", bad_event, policy),
            "trigger event with mismatched repository ID was accepted",
        )

    bad_event = json.loads(json.dumps(event))
    bad_event["workflow_run"]["head_repository"]["id"] = 901
    with patch.dict(os.environ, {"D6U_TRUSTED_REPOSITORY_ID": "900"}, clear=False):
        assert_rejected(
            lambda: expected_artifact("Luminous-Dynamics/mycelix", bad_event, policy),
            "trigger run with mismatched head repository ID was accepted",
        )

    for field, bad_value, message in [
        ("head_branch", "main", "executor artifact with mismatched trigger branch was accepted"),
        ("head_sha", "c" * 40, "executor artifact with mismatched trigger SHA was accepted"),
        ("id", 701, "executor artifact with mismatched run ID was accepted"),
    ]:
        bad = json.loads(json.dumps(payload))
        bad["artifacts"][0]["workflow_run"][field] = bad_value
        with patch.dict(
            os.environ,
            {
                "GITHUB_TOKEN": "token",
        "D6U_TRUSTED_REPOSITORY_ID": "9001",
                "D6U_TRIGGER_HEAD_BRANCH": "myc-int-demo-d6u-holochain-07-runtime",
                "D6U_TRIGGER_HEAD_SHA": "a" * 40,
            },
            clear=False,
        ), patch.object(
            fetcher,
            "github_get",
            side_effect=lambda _repo, api_path, _token: current_run if api_path == "/actions/runs/700" else bad,
        ):
            assert_rejected(
                lambda: expected_artifact("Luminous-Dynamics/mycelix", event, policy),
                message,
            )

    bad = json.loads(json.dumps(payload))
    bad["artifacts"][0]["workflow_run"]["head_repository_id"] = 901
    with patch.dict(
        os.environ,
        {
            "GITHUB_TOKEN": "token",
        "D6U_TRUSTED_REPOSITORY_ID": "9001",
            "D6U_TRIGGER_HEAD_BRANCH": "myc-int-demo-d6u-holochain-07-runtime",
            "D6U_TRIGGER_HEAD_SHA": "a" * 40,
        },
        clear=False,
    ), patch.object(
        fetcher,
        "github_get",
        side_effect=lambda _repo, api_path, _token: current_run if api_path == "/actions/runs/700" else bad,
    ):
        assert_rejected(
            lambda: expected_artifact("Luminous-Dynamics/mycelix", event, policy),
            "executor artifact with mismatched head repository ID was accepted",
        )


def test_current_run_handoff_artifact_accepts_exact_identity() -> None:
    import fetch_d6u_trusted_artifact as fetcher

    policy = {
        "auditor_handoff": {
            "artifact_name_template": "d6u-trusted-auditor-handoff-run-{run_id}-attempt-{run_attempt}",
            "artifact_max_archive_bytes": 1024,
        },
        "repository_identity": {
            "full_name": "Luminous-Dynamics/mycelix",
            "repository_id": 9001,
        },
    }
    env = {
        "GITHUB_RUN_ID": "501",
        "GITHUB_RUN_ATTEMPT": "2",
        "GITHUB_SHA": "d" * 40,
        "GITHUB_REF": "refs/heads/main",
        "D6U_TRIGGER_HEAD_BRANCH": "myc-int-demo-d6u-holochain-07-runtime",
        "D6U_TRIGGER_HEAD_SHA": "a" * 40,
        "GITHUB_TOKEN": "token",
        "D6U_TRUSTED_REPOSITORY_ID": "9001",
    }
    current_run = {
        "id": 501,
        "run_attempt": 2,
        "head_branch": "main",
        "head_sha": "d" * 40,
        "repository": {"id": 9001, "full_name": "Luminous-Dynamics/mycelix"},
        "head_repository": {"id": 9001, "full_name": "Luminous-Dynamics/mycelix"},
    }
    payload = {
        "artifacts": [{
            "id": 9001,
            "name": "d6u-trusted-auditor-handoff-run-501-attempt-2",
            "expired": False,
            "size_in_bytes": 512,
            "digest": "sha256:" + "b" * 64,
            "workflow_run": {
                "id": 501,
                "repository_id": 9001,
                "head_repository_id": 9001,
                "head_branch": "main",
                "head_sha": "d" * 40,
            },
        }]
    }

    def fake_github_get(_repo, api_path, _token):
        if api_path == "/actions/runs/501":
            return current_run
        if api_path.startswith("/actions/runs/501/artifacts?"):
            return payload
        raise AssertionError(f"unexpected GitHub API path: {api_path}")

    with patch.dict(os.environ, env, clear=False), patch.object(fetcher, "github_get", side_effect=fake_github_get):
        observed = expected_current_run_artifact("Luminous-Dynamics/mycelix", policy)
    assert observed["id"] == 9001

    bad_run = dict(current_run)
    bad_run["head_sha"] = "c" * 40
    with patch.dict(os.environ, env, clear=False), patch.object(
        fetcher,
        "github_get",
        side_effect=lambda _repo, api_path, _token: bad_run if api_path == "/actions/runs/501" else payload,
    ):
        assert_rejected(
            lambda: expected_current_run_artifact("Luminous-Dynamics/mycelix", policy),
            "handoff artifact was accepted after current workflow head changed",
        )

    bad_repository_run = json.loads(json.dumps(current_run))
    bad_repository_run["repository"]["id"] = 9002
    with patch.dict(os.environ, env, clear=False), patch.object(
        fetcher,
        "github_get",
        side_effect=lambda _repo, api_path, _token: (
            bad_repository_run if api_path == "/actions/runs/501" else payload
        ),
    ):
        assert_rejected(
            lambda: expected_current_run_artifact("Luminous-Dynamics/mycelix", policy),
            "handoff artifact was accepted from a different current repository ID",
        )

    bad_head_repository_run = json.loads(json.dumps(current_run))
    bad_head_repository_run["head_repository"]["id"] = 9002
    with patch.dict(os.environ, env, clear=False), patch.object(
        fetcher,
        "github_get",
        side_effect=lambda _repo, api_path, _token: (
            bad_head_repository_run if api_path == "/actions/runs/501" else payload
        ),
    ):
        assert_rejected(
            lambda: expected_current_run_artifact("Luminous-Dynamics/mycelix", policy),
            "handoff artifact was accepted from a current run with a different head repository ID",
        )

    bad_branch_run = dict(current_run)
    bad_branch_run["head_branch"] = "unexpected-branch"
    with patch.dict(os.environ, env, clear=False), patch.object(
        fetcher,
        "github_get",
        side_effect=lambda _repo, api_path, _token: (
            bad_branch_run if api_path == "/actions/runs/501" else payload
        ),
    ):
        assert_rejected(
            lambda: expected_current_run_artifact("Luminous-Dynamics/mycelix", policy),
            "handoff artifact was accepted from a current run on an unexpected branch",
        )


def test_current_run_handoff_artifact_rejects_oversized_archive_metadata() -> None:
    import fetch_d6u_trusted_artifact as fetcher

    policy = {
        "auditor_handoff": {
            "artifact_name_template": "d6u-trusted-auditor-handoff-run-{run_id}-attempt-{run_attempt}",
            "artifact_max_archive_bytes": 1024,
        },
        "repository_identity": {
            "full_name": "Luminous-Dynamics/mycelix",
            "repository_id": 9001,
        },
    }
    env = {
        "GITHUB_RUN_ID": "501",
        "GITHUB_RUN_ATTEMPT": "2",
        "GITHUB_SHA": "a" * 40,
        "GITHUB_REF": "refs/heads/main",
        "D6U_TRIGGER_HEAD_BRANCH": "myc-int-demo-d6u-holochain-07-runtime",
        "D6U_TRIGGER_HEAD_SHA": "b" * 40,
        "GITHUB_TOKEN": "token",
        "D6U_TRUSTED_REPOSITORY_ID": "9001",
    }
    current_run = {
        "id": 501,
        "run_attempt": 2,
        "head_branch": "main",
        "head_sha": "a" * 40,
        "repository": {"id": 9001, "full_name": "Luminous-Dynamics/mycelix"},
        "head_repository": {"id": 9001, "full_name": "Luminous-Dynamics/mycelix"},
    }
    payload = {
        "artifacts": [{
            "id": 9001,
            "name": "d6u-trusted-auditor-handoff-run-501-attempt-2",
            "expired": False,
            "size_in_bytes": 1025,
            "digest": "sha256:" + "b" * 64,
            "workflow_run": {
                "id": 501,
                "repository_id": 9001,
                "head_repository_id": 9001,
                "head_branch": "main",
                "head_sha": "a" * 40,
            },
        }]
    }

    def fake_github_get(_repo, api_path, _token):
        if api_path == "/actions/runs/501":
            return current_run
        if api_path.startswith("/actions/runs/501/artifacts?"):
            return payload
        raise AssertionError(f"unexpected GitHub API path: {api_path}")

    with patch.dict(os.environ, env, clear=False), patch.object(fetcher, "github_get", side_effect=fake_github_get):
        assert_rejected(
            lambda: expected_current_run_artifact("Luminous-Dynamics/mycelix", policy),
            "oversized auditor handoff archive was accepted",
        )



def test_trusted_zip_rejects_zip64_eocd() -> None:
    with tempfile.TemporaryDirectory() as tmp:
        archive = Path(tmp) / "artifact.zip"
        write_valid_artifact_zip(archive)
        raw = archive.read_bytes()
        eocd = raw.rfind(b"PK\\x05\\x06")
        assert eocd >= 0
        total_entries, central_size, central_offset = struct.unpack_from(
            "<HII", raw, eocd + 10
        )
        zip64_offset = eocd
        zip64_record = struct.pack(
            "<4sQ2H2I4Q",
            b"PK\\x06\\x06",
            44,
            45,
            45,
            0,
            0,
            total_entries,
            total_entries,
            central_size,
            central_offset,
        )
        zip64_locator = struct.pack(
            "<4sIQI",
            b"PK\\x06\\x07",
            0,
            zip64_offset,
            1,
        )
        archive.write_bytes(raw[:eocd] + zip64_record + zip64_locator + raw[eocd:])
        assert_rejected(
            lambda: verify_zip_members(archive, artifact_policy()),
            "ZIP64 EOCD/locator was accepted",
        )


def test_trusted_zip_rejects_zip64_central_directory_sentinel() -> None:
    with tempfile.TemporaryDirectory() as tmp:
        archive = Path(tmp) / "artifact.zip"
        write_valid_artifact_zip(archive)
        raw = bytearray(archive.read_bytes())
        eocd = raw.rfind(b"PK\\x05\\x06")
        assert eocd >= 0
        struct.pack_into("<I", raw, eocd + 12, 0xFFFFFFFF)
        archive.write_bytes(raw)
        assert_rejected(
            lambda: verify_zip_members(archive, artifact_policy()),
            "ZIP64 central-directory sentinel was accepted",
        )


def test_bounded_artifact_download_rejects_stream_overflow() -> None:
    import fetch_d6u_trusted_artifact as fetcher

    class OversizeResponse:
        def __enter__(self):
            return self

        def __exit__(self, exc_type, exc, tb):
            return False

        def read(self, size):
            return b"x" * (size + 1)

    with tempfile.TemporaryDirectory() as tmp:
        destination = Path(tmp) / "artifact.zip"
        class OversizeOpener:
            def open(self, request, timeout):
                return OversizeResponse()

        with patch.object(
            fetcher.urllib.request,
            "build_opener",
            return_value=OversizeOpener(),
        ):
            assert_rejected(
                lambda: download_archive(
                    "Luminous-Dynamics/mycelix",
                    9001,
                    "sha256:" + "b" * 64,
                    destination,
                    1024,
                ),
                "oversized streamed artifact archive was accepted",
            )


def test_github_api_reader_uses_non_forwarding_redirect_handler() -> None:
    import fetch_d6u_trusted_artifact as fetcher
    import verify_d6u_trusted_artifacts as verifier

    class EmptyResponse:
        def __init__(self, final_url: str):
            self.final_url = final_url

        def __enter__(self):
            return self

        def __exit__(self, exc_type, exc, tb):
            return False

        def geturl(self):
            return self.final_url

        def read(self, _size):
            return b"{}"

    class FakeOpener:
        def __init__(self, final_url: str):
            self.final_url = final_url

        def open(self, request, timeout):
            assert request.full_url.startswith("https://api.github.com/")
            assert request.headers["Authorization"] == "Bearer token"
            assert timeout == 30
            return EmptyResponse(self.final_url)

    for module in (fetcher, verifier):
        with patch.object(
            module.urllib.request,
            "build_opener",
            return_value=FakeOpener("https://api.github.com/repos/Luminous-Dynamics/mycelix/actions/runs/1"),
        ) as build_opener:
            assert module.github_get(
                "Luminous-Dynamics/mycelix",
                "/actions/runs/1",
                "token",
            ) == {}
        assert len(build_opener.call_args.args) == 1
        assert isinstance(
            build_opener.call_args.args[0],
            module.NoAuthorizationRedirectHandler,
        )


def test_github_api_reader_rejects_cross_host_final_url() -> None:
    import fetch_d6u_trusted_artifact as fetcher
    import verify_d6u_trusted_artifacts as verifier

    class Response:
        def __enter__(self):
            return self

        def __exit__(self, exc_type, exc, tb):
            return False

        def geturl(self):
            return "https://attacker.example/redirected-json"

        def read(self, _size):
            return b"{}"

    class Opener:
        def open(self, request, timeout):
            return Response()

    for module in (fetcher, verifier):
        with patch.object(module.urllib.request, "build_opener", return_value=Opener()):
            assert_rejected(
                lambda module=module: module.github_get(
                    "Luminous-Dynamics/mycelix", "/actions/runs/1", "token"
                ),
                f"cross-host JSON API redirect was accepted by {module.__name__}",
            )


def test_artifact_redirect_strips_authorization_header() -> None:
    import fetch_d6u_trusted_artifact as fetcher
    import verify_d6u_trusted_artifacts as verifier

    for module in (fetcher, verifier):
        request = module.urllib.request.Request(
            "https://api.github.com/repos/Luminous-Dynamics/mycelix/actions/artifacts/9001/zip",
            headers={"Authorization": "Bearer secret"},
        )
        redirected = module.NoAuthorizationRedirectHandler().redirect_request(
            request,
            None,
            302,
            "Found",
            {},
            "https://objects.githubusercontent.com/example/archive.zip",
        )
        assert redirected is not None
        assert redirected.headers.get("Authorization") is None

        assert_rejected(
            lambda module=module: module.NoAuthorizationRedirectHandler().redirect_request(
                request,
                None,
                302,
                "Found",
                {},
                "http://objects.githubusercontent.com/example/archive.zip",
            ),
            f"plaintext redirect was accepted by {module.__name__}",
        )
        assert_rejected(
            lambda module=module: module.NoAuthorizationRedirectHandler().redirect_request(
                request,
                None,
                302,
                "Found",
                {},
                "https://user:secret@objects.githubusercontent.com/example/archive.zip",
            ),
            f"credential-bearing redirect URL was accepted by {module.__name__}",
        )


def test_git_blob_sha1_uses_git_object_framing() -> None:
    content = b"hello\n"
    expected = hashlib.sha1(b"blob 6\0hello\n").hexdigest()
    assert _git_blob_sha1(content) == expected


def test_trusted_zip_entry_count_is_preflighted_before_zip_parsing() -> None:
    import fetch_d6u_trusted_artifact as fetcher

    with tempfile.TemporaryDirectory() as tmp:
        archive = Path(tmp) / "artifact.zip"
        with ZipFile(archive, "w") as zip_file:
            for index in range(33):
                zip_file.writestr(f"unexpected-{index}", "x")
        with patch.object(fetcher, "ZipFile", side_effect=AssertionError("ZipFile opened")):
            assert_rejected(
                lambda: verify_zip_members(archive, artifact_policy()),
                "ZIP parser was opened before entry-count preflight rejected the archive",
            )



def test_trusted_zip_rejects_unknown_configured_compression() -> None:
    with tempfile.TemporaryDirectory() as tmp:
        archive = Path(tmp) / "artifact.zip"
        with ZipFile(archive, "w") as zip_file:
            zip_file.writestr("d6u-runtime-evidence.txt", "evidence")
            zip_file.writestr("d6u-runtime-test.log", "log")
            zip_file.writestr("Cargo.lock", "lock")
        assert_rejected(
            lambda: verify_zip_members(
                archive,
                artifact_policy(),
                allowed_compression_methods=["stored", "definitely-unknown"],
            ),
            "unknown ZIP compression policy name was accepted",
        )


def test_trusted_zip_rejects_non_zlib_compression() -> None:
    from zipfile import ZIP_BZIP2, ZIP_LZMA

    for compression in (ZIP_BZIP2, ZIP_LZMA):
        with tempfile.TemporaryDirectory() as tmp:
            archive = Path(tmp) / "artifact.zip"
            with ZipFile(archive, "w", compression=compression) as zip_file:
                zip_file.writestr("d6u-runtime-evidence.txt", "evidence")
                zip_file.writestr("d6u-runtime-test.log", "log")
                zip_file.writestr("Cargo.lock", "lock")
            assert_rejected(
                lambda: verify_zip_members(archive, artifact_policy()),
                f"unsupported ZIP compression method was accepted: {compression}",
            )



def test_trusted_builder_documentation_is_current() -> None:
    root = Path(__file__).parents[2]
    documentation = (root / "docs/integral/d6u-trusted-builder.md").read_text(
        encoding="utf-8"
    )
    assert "Current trusted policy revision: v61." in documentation
    assert "seventy-three deterministic checks" in documentation
    assert "`push-to-registry: false`" in documentation
    assert "`create-storage-record: false`" in documentation
    assert "keeps redirects on HTTPS" in documentation
    assert "rejects URL userinfo" in documentation


def test_registry_is_complete_and_unique() -> None:
    import ast

    source_path = Path(__file__).resolve()
    tree = ast.parse(source_path.read_text(encoding="utf-8"))

    defined = [
        node.name
        for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name.startswith("test_")
    ]
    registry = None
    for node in tree.body:
        if not isinstance(node, ast.If):
            continue
        if not (
            isinstance(node.test, ast.Compare)
            and isinstance(node.test.left, ast.Name)
            and node.test.left.id == "__name__"
            and len(node.test.ops) == 1
            and isinstance(node.test.ops[0], ast.Eq)
            and len(node.test.comparators) == 1
            and isinstance(node.test.comparators[0], ast.Constant)
            and node.test.comparators[0].value == "__main__"
        ):
            continue
        for statement in node.body:
            if not isinstance(statement, ast.Assign):
                continue
            if not any(
                isinstance(target, ast.Name) and target.id == "tests"
                for target in statement.targets
            ):
                continue
            value = statement.value
            if not isinstance(value, (ast.List, ast.Tuple)):
                raise AssertionError("trusted test registry is not a list/tuple")
            registry = [
                element.id
                for element in value.elts
                if isinstance(element, ast.Name)
            ]
            break
        break

    assert registry is not None, "trusted test registry was not found"
    assert len(defined) == len(set(defined)), "duplicate test function definitions found"
    assert len(registry) == len(set(registry)), "duplicate tests in executable registry"
    assert set(registry) == set(defined), (
        f"test registry mismatch: defined={defined!r}, registered={registry!r}"
    )


if __name__ == "__main__":
    tests = [
        test_trusted_builder_documentation_is_current,
        test_registry_is_complete_and_unique,
        test_policy_pins_d6s_prerequisite_boundary,
        test_record_metadata_is_canonicalized,
        test_policy_pins_current_trusted_workflow,
        test_policy_pins_current_trusted_fetcher,
        test_privilege_split_handoff_topology_is_fail_closed,
        test_trusted_cli_policy_is_explicit,
        test_privileged_actions_are_exactly_pinned,
        test_forbidden_cargo_config_is_rejected,
        test_harness_file_set_rejects_extra_build_script,
        test_artifact_size_limits_are_enforced,
        test_artifact_entry_limit_is_enforced,
        test_executor_artifact_binds_trigger_head_and_attempt,
        test_valid_log_is_accepted,
        test_case_tampering_is_rejected,
        test_duplicate_case_is_rejected,
        test_executor_workflow_identity_tampering_is_rejected,
        test_executor_workflow_is_bound_to_run_head,
        test_executor_run_live_identity_is_rejected,
        test_github_api_reader_uses_non_forwarding_redirect_handler,
        test_lock_graph_binds_root_to_trusted_manifest,
        test_lock_graph_rejects_root_manifest_dependency_mismatch,
        test_lock_graph_rejects_dangling_or_ambiguous_dependency_reference,
        test_lock_graph_rejects_unreachable_package_node,
        test_lock_graph_rejects_multiple_local_roots,
        test_lockfile_format_version_is_pinned,
        test_lock_provenance_is_rejected_when_tampered,
        test_duplicate_record_key_is_rejected,
        test_trigger_run_identity_tampering_is_rejected,
        test_tracked_source_tree_accepts_exact_blobs,
        test_tracked_source_tree_rejects_symlink_mode,
        test_tracked_source_tree_rejects_nonblob_entry,
        test_truncated_source_tree_is_rejected,
        test_trusted_zip_accepts_exact_members,
        test_handoff_zip_uses_handoff_compression_policy,
        test_trusted_zip_rejects_duplicate_member,
        test_trusted_zip_rejects_symlink_member,
        test_trusted_zip_rejects_unexpected_member_path,
        test_trusted_github_api_readers_are_response_bounded,
        test_trusted_python_programs_reject_optimized_mode,
        test_attestation_verifier_contains_no_optimization_sensitive_asserts,
        test_attestation_verifier_accepts_current_run,
        test_attestation_verifier_rejects_old_run,
        test_commitment_attestation_rejects_malformed_entry_types,
        test_commitment_attestation_rejects_canonical_predicate_tampering,
        test_commitment_attestation_rejects_trigger_source_mismatch,
        test_commitment_attestation_rejects_hash_mismatch,
        test_commitment_attestation_requires_verified_timestamp,
        test_commitment_attestation_rejects_non_tlog_timestamp,
        test_commitment_attestation_subject_set_is_order_independent_but_exact,
        test_retention_workflow_bounds_inputs_before_verification,
        test_retention_workflow_contains_offline_controls,
        test_trusted_root_jsonl_line_limit_is_enforced,
        test_negative_control_requires_nonzero_exit,
        test_negative_control_command_must_disable_public_good,
        test_negative_control_cross_link_mismatch_is_rejected,
        test_retained_report_identity_and_predicate_are_bound,
        test_retention_packet_rejects_extra_member,
        test_commitment_attestation_accepts_current_run_and_rejects_old_run,
        test_trusted_workflow_policy_shape_is_pinned,
        test_artifact_layout_rejects_symlink,
        test_current_run_handoff_artifact_accepts_exact_identity,
        test_current_run_handoff_artifact_rejects_oversized_archive_metadata,
        test_bounded_artifact_download_rejects_stream_overflow,
        test_github_api_reader_rejects_cross_host_final_url,
        test_artifact_redirect_strips_authorization_header,
        test_git_blob_sha1_uses_git_object_framing,
        test_trusted_zip_entry_count_is_preflighted_before_zip_parsing,
        test_trusted_zip_rejects_unknown_configured_compression,
        test_trusted_zip_rejects_non_zlib_compression,
        test_trusted_zip_rejects_zip64_eocd,
        test_trusted_zip_rejects_zip64_central_directory_sentinel,
    ]
    for test in tests:
        test()
    print(f"verified D6U trusted verifier self-tests: {len(tests)}/{len(tests)}")