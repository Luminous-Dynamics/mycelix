#!/usr/bin/env python3
"""Deterministic, read-only tests for the D6U trusted verifier."""

import json
import hashlib
import os
import subprocess
import stat
import tempfile
from zipfile import ZipFile, ZipInfo
from unittest.mock import patch
from pathlib import Path

from fetch_d6u_trusted_artifact import EXPECTED_FILES, extract_members, verify_zip_members
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

    assert policy["policy_version"] == 25

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
    assert set(policy["trusted_programs"]) == {
        "scripts/integral/verify_d6u_trusted_artifacts.py",
        "scripts/integral/fetch_d6u_trusted_artifact.py",
        "scripts/integral/verify_d6u_trusted_attestation.py",
    }
    for path, descriptor in policy["trusted_programs"].items():
        assert descriptor["path"] == path
        assert len(descriptor["blob_sha"]) == 40
        index_record = subprocess.run(
            ["git", "ls-files", "--stage", "--", path],
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
        assert index_record
        mode, observed = index_record.split()[:2]
        assert mode in {"100644", "100755"}
        assert observed == descriptor["blob_sha"]

    assert policy["trusted_attestation_verifier"]["path"] == (
        "scripts/integral/verify_d6u_trusted_attestation.py"
    )
    assert len(policy["trusted_attestation_verifier"]["blob_sha"]) == 40
    assert policy["attestation_verification"]["predicate_type"] == (
        "https://luminousdynamics.io/attestations/d6u-runtime-evidence/v1"
    )
    assert policy["attestation_verification"]["predicate_schema"] == (
        "d6u-trusted-runtime-evidence/v1"
    )
    assert policy["attestation_verification"]["require_current_run_identity"] is True
    assert policy["attestation_verification"]["subject_set_exact"] is True
    assert policy["attestation_verification"]["require_verified_timestamp"] is True

    assert policy["trusted_actions"] == {
        "actions/checkout": {
            "ref": "fbc6f3992d24b796d5a048ff273f7fcc4a7b6c09",
            "version": "v5.1.0",
        },
        "actions/attest": {
            "ref": "1e69f48acb82d1966a394da916b4c169aa569d6",
            "version": "v4.2.2",
        },
    }
    workflow_path = Path(__file__).parents[2] / policy["trusted_workflow"]["path"]
    uses = []
    for line in workflow_path.read_text(encoding="utf-8").splitlines():
        stripped = line.strip()
        if stripped.startswith("uses:"):
            uses.append(stripped.split("uses:", 1)[1].strip())
    assert set(uses) == {
        "actions/checkout@fbc6f3992d24b796d5a048ff273f7fcc4a7b6c09 # v5.1.0",
        "actions/attest@1e69f48acb82d1966a394da916b4c169aa569d6 # v4.2.2",
    }

    assert policy["trusted_permissions"] == {
        "actions": "read",
        "contents": "read",
        "id-token": "write",
        "attestations": "write",
    }
    workflow_path = Path(__file__).parents[2] / policy["trusted_workflow"]["path"]
    workflow_text = workflow_path.read_text(encoding="utf-8")
    assert "artifact-metadata: write" not in workflow_text
    assert "actions: write" not in workflow_text
    assert "contents: write" not in workflow_text
    assert "attestations: write" in workflow_text
    assert "id-token: write" in workflow_text
    assert "actions: read" in workflow_text
    assert "contents: read" in workflow_text

    assert policy["artifact_integrity"] == {
        "algorithm": "sha256",
        "source": "github-artifact-api",
        "require_match_after_download": True,
        "archive_format": "zip",
        "extraction_mode": "bounded-members",
        "reject_encrypted_members": True,
        "reject_symlink_members": True,
        "expected_member_count": 3,
        "policy_revision": 23,
    }


def record_metadata_policy() -> dict:
    return {
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
        "d6s2_authority_ledger_schema": "v1",
        "d6s1_corpus_sha256": "a" * 64,
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


def test_privileged_actions_are_exactly_pinned() -> None:
    policy = json.loads(
        (Path(__file__).parents[2] / "docs/integral/d6u-trusted-builder-policy.json").read_text(
            encoding="utf-8"
        )
    )
    workflow_path = Path(__file__).parents[2] / policy["trusted_workflow"]["path"]
    uses = [
        line.strip().split("uses:", 1)[1].strip()
        for line in workflow_path.read_text(encoding="utf-8").splitlines()
        if line.strip().startswith("uses:")
    ]
    assert set(uses) == {
        "actions/checkout@fbc6f3992d24b796d5a048ff273f7fcc4a7b6c09 # v5.1.0",
        "actions/attest@1e69f48acb82d1966a394da916b4c169aa569d6 # v4.2.2",
    }


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
        "executor_workflow": {
            "name": "D6U Exact-Head Runtime Executor",
            "path": ".github/workflows/d6u-exact-head-runtime-executor.yml",
            "blob_sha": "a" * 40,
        }
    }
    run = {
        "repository": {"full_name": "Luminous-Dynamics/mycelix"},
        "head_repository": {"full_name": "Luminous-Dynamics/mycelix"},
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
        "executor_workflow": {
            "name": "D6U Exact-Head Runtime Executor",
            "path": ".github/workflows/d6u-exact-head-runtime-executor.yml",
        },
    }
    record = {
        "executor_run_id": "42",
        "executor_run_attempt": "3",
    }
    valid = {
        "id": 42,
        "run_attempt": 3,
        "name": "D6U Exact-Head Runtime Executor",
        "path": ".github/workflows/d6u-exact-head-runtime-executor.yml",
        "event": "workflow_run",
        "conclusion": "success",
        "repository": {"full_name": "Luminous-Dynamics/mycelix"},
        "head_repository": {"full_name": "Luminous-Dynamics/mycelix"},
        "head_branch": "main",
    }
    verify_executor_run_record(valid, record, policy, "Luminous-Dynamics/mycelix")

    for field, value, message in [
        ("repository", {"full_name": "attacker/repo"}, "executor repository"),
        ("head_repository", {"full_name": "attacker/repo"}, "executor head repository"),
        ("head_branch", "attacker-branch", "executor non-main branch"),
        ("id", 43, "executor run ID"),
        ("run_attempt", 4, "executor run attempt"),
        ("conclusion", "failure", "executor conclusion"),
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
        "source_branch": "myc-int-demo-d6u-holochain-07-runtime",
        "trigger_workflow": {
            "name": "D6S Canonical Qualification",
            "path": ".github/workflows/d6s-canonical-qualification.yml",
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
        "repository": {"full_name": "Luminous-Dynamics/mycelix"},
        "name": "D6S Canonical Qualification",
        "path": ".github/workflows/d6s-canonical-qualification.yml",
        "event": "pull_request",
        "conclusion": "success",
        "head_repository": {"full_name": "Luminous-Dynamics/mycelix"},
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

    bad_trigger = {**trigger, "head_sha": "e" * 40}
    assert_rejected(
        lambda: verify_trigger_run_record(record, bad_trigger, policy, "Luminous-Dynamics/mycelix"),
        "tampered trigger SHA was accepted",
    )

    bad_blob = {**record, "trigger_workflow_blob_sha": "e" * 40}
    assert_rejected(
        lambda: verify_trigger_run_record(bad_blob, trigger, policy, "Luminous-Dynamics/mycelix"),
        "tampered trigger workflow blob was accepted",
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
    }

    with tempfile.TemporaryDirectory() as tmp:
        lock = Path(tmp) / "Cargo.lock"
        lock.write_text(
            'version = 3\n\n[[package]]\n'
            + "\n".join(f'{key} = "{value}"' for key, value in package.items())
            + "\n",
            encoding="utf-8",
        )
        verify_lock(lock, policy)

        lock.write_text(
            lock.read_text(encoding="utf-8").replace(
                'checksum = "' + ("a" * 64) + '"',
                "",
            ),
            encoding="utf-8",
        )
        assert_rejected(
            lambda: verify_lock(lock, policy),
            "malformed lock checksum was accepted",
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


def test_trusted_zip_accepts_exact_members() -> None:
    with tempfile.TemporaryDirectory() as tmp:
        archive = Path(tmp) / "artifact.zip"
        write_valid_artifact_zip(archive)
        infos = verify_zip_members(archive, artifact_policy())
        assert {info.filename for info in infos} == EXPECTED_FILES


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



def synthetic_attestation_entry(subjects: list[dict], run_id: str) -> dict:
    repo = "Luminous-Dynamics/mycelix"
    return {
        "verificationResult": {
            "signature": {
                "certificate": {
                    "subjectAlternativeName": (
                        f"https://github.com/{repo}/.github/workflows/"
                        "d6u-trusted-evidence-attestation.yml@refs/heads/main"
                    ),
                    "issuer": "https://token.actions.githubusercontent.com",
                    "githubWorkflowRepository": repo,
                    "githubWorkflowRef": "refs/heads/main",
                    "sourceRepositoryURI": f"https://github.com/{repo}",
                    "sourceRepositoryDigest": "a" * 40,
                    "runnerEnvironment": "github-hosted",
                    "runInvocationURI": (
                        f"https://github.com/{repo}/actions/runs/{run_id}/attempts/3"
                    ),
                }
            },
            "statement": {
                "predicateType": "https://luminousdynamics.io/attestations/d6u-runtime-evidence/v1",
                "subject": subjects,
                "predicate": {
                    "schema": "d6u-trusted-runtime-evidence/v1",
                    "attestation_kind": "verified-runtime-evidence",
                    "claim_ceiling": "ReferenceModelOnly",
                    "policy_version": 22,
                    "source": {
                        "repository": repo,
                        "branch": "myc-int-demo-d6u-holochain-07-runtime",
                        "commit": "b" * 40,
                    },
                    "trigger": {
                        "workflow_name": "D6S Canonical Qualification",
                        "workflow_path": ".github/workflows/d6s-canonical-qualification.yml",
                        "run_id": 77,
                        "run_attempt": 2,
                    },
                    "executor": {
                        "workflow_name": "D6U Exact-Head Runtime Executor",
                        "workflow_path": ".github/workflows/d6u-exact-head-runtime-executor.yml",
                        "run_id": 42,
                        "run_attempt": 3,
                        "workflow_commit": "c" * 40,
                    },
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


def test_custom_attestation_requires_verified_timestamp() -> None:
    import verify_d6u_trusted_attestation as verifier

    subjects = [
        {"name": "d6u-runtime-evidence.txt", "digest": {"sha256": "a" * 64}},
        {"name": "d6u-runtime-test.log", "digest": {"sha256": "b" * 64}},
        {"name": "Cargo.lock", "digest": {"sha256": "c" * 64}},
    ]
    record = synthetic_record()
    entry = synthetic_attestation_entry(subjects, "42")
    entry["verificationResult"]["verifiedTimestamps"] = []
    with patch.dict(
        os.environ,
        {
            "GITHUB_REPOSITORY": "Luminous-Dynamics/mycelix",
            "GITHUB_RUN_ID": "42",
            "GITHUB_RUN_ATTEMPT": "3",
            "GITHUB_SHA": "a" * 40,
            "GITHUB_WORKFLOW_SHA": "c" * 40,
            "GITHUB_REF": "refs/heads/main",
        },
        clear=False,
    ):
        assert verifier.verify_entry(entry, record, subjects) is False


def test_custom_attestation_subject_set_is_order_independent_but_exact() -> None:
    import verify_d6u_trusted_attestation as verifier

    subjects = [
        {"name": "d6u-runtime-evidence.txt", "digest": {"sha256": "a" * 64}},
        {"name": "d6u-runtime-test.log", "digest": {"sha256": "b" * 64}},
        {"name": "Cargo.lock", "digest": {"sha256": "c" * 64}},
    ]
    reordered = [subjects[2], subjects[0], subjects[1]]

    record = synthetic_record()
    current = synthetic_attestation_entry(subjects, "42")
    current["verificationResult"]["statement"]["subject"] = reordered
    current["verificationResult"]["statement"]["predicate"]["subjects"] = reordered

    duplicate = synthetic_attestation_entry(subjects, "42")
    duplicate_subjects = list(subjects) + [dict(subjects[0])]
    duplicate["verificationResult"]["statement"]["subject"] = duplicate_subjects
    duplicate["verificationResult"]["statement"]["predicate"]["subjects"] = duplicate_subjects

    with patch.dict(
        os.environ,
        {
            "GITHUB_REPOSITORY": "Luminous-Dynamics/mycelix",
            "GITHUB_RUN_ID": "42",
            "GITHUB_RUN_ATTEMPT": "3",
            "GITHUB_SHA": "a" * 40,
            "GITHUB_WORKFLOW_SHA": "c" * 40,
            "GITHUB_REF": "refs/heads/main",
        },
        clear=False,
    ):
        assert verifier.verify_entry(current, record, subjects) is True
        assert verifier.verify_entry(duplicate, record, subjects) is False

def test_custom_attestation_accepts_current_run_and_rejects_old_run() -> None:
    import verify_d6u_trusted_attestation as verifier

    subjects = [
        {"name": name, "digest": {"sha256": "d" * 64}}
        for name in ("d6u-runtime-evidence.txt", "d6u-runtime-test.log", "Cargo.lock")
    ]
    record = synthetic_record()
    with patch.dict(
        os.environ,
        {
            "GITHUB_REPOSITORY": "Luminous-Dynamics/mycelix",
            "GITHUB_RUN_ID": "42",
            "GITHUB_RUN_ATTEMPT": "3",
            "GITHUB_SHA": "a" * 40,
            "GITHUB_WORKFLOW_SHA": "c" * 40,
            "GITHUB_REF": "refs/heads/main",
        },
        clear=False,
    ):
        current = synthetic_attestation_entry(subjects, "42")
        old = synthetic_attestation_entry(subjects, "41")
        assert verifier.verify_entry(current, record, subjects) is True
        assert verifier.verify_entry(old, record, subjects) is False


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


if __name__ == "__main__":
    tests = [
        test_policy_pins_d6s_prerequisite_boundary,
        test_policy_pins_current_trusted_workflow,
        test_privileged_actions_are_exactly_pinned,
        test_policy_pins_current_trusted_fetcher,
        test_forbidden_cargo_config_is_rejected,
        test_record_metadata_is_canonicalized,
        test_harness_file_set_rejects_extra_build_script,
        test_artifact_size_limits_are_enforced,
        test_artifact_entry_limit_is_enforced,
        test_custom_attestation_requires_verified_timestamp,
        test_custom_attestation_subject_set_is_order_independent_but_exact,
        test_custom_attestation_accepts_current_run_and_rejects_old_run,
        test_attestation_verifier_accepts_current_run,
        test_attestation_verifier_rejects_old_run,
        test_trusted_zip_accepts_exact_members,
        test_trusted_zip_rejects_duplicate_member,
        test_trusted_zip_rejects_symlink_member,
        test_trusted_zip_rejects_unexpected_member_path,
        test_trusted_workflow_policy_shape_is_pinned,
        test_valid_log_is_accepted,
        test_case_tampering_is_rejected,
        test_duplicate_case_is_rejected,
        test_executor_workflow_identity_tampering_is_rejected,
        test_executor_workflow_is_bound_to_run_head,
        test_executor_run_live_identity_is_rejected,
        test_lock_provenance_is_rejected_when_tampered,
        test_duplicate_record_key_is_rejected,
        test_trigger_run_identity_tampering_is_rejected,
        test_tracked_source_tree_accepts_exact_blobs,
        test_tracked_source_tree_rejects_symlink_mode,
        test_truncated_source_tree_is_rejected,
        test_artifact_layout_rejects_symlink,
    ]
    for test in tests:
        test()
    print(f"verified D6U trusted verifier self-tests: {len(tests)}/{len(tests)}")
