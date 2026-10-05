#!/usr/bin/env python3
"""Deterministic, read-only tests for the D6U trusted verifier."""

import json
import tempfile
from pathlib import Path

from verify_d6u_trusted_artifacts import (
    load_record,
    verify_artifact_layout,
    verify_cases,
    verify_executor_run_record,
    verify_executor_workflow_record,
    verify_lock,
    verify_required_tracked_blobs,
    verify_trigger_run_record,
    verify_forbidden_cargo_config_paths,
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

    assert policy["forbidden_cargo_config_paths"] == [
        ".cargo/config",
        ".cargo/config.toml",
        "d6u-runtime-harness/.cargo/config",
        "d6u-runtime-harness/.cargo/config.toml",
    ]


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
            ),
            "symlinked trusted input was accepted",
        )


if __name__ == "__main__":
    tests = [
        test_policy_pins_d6s_prerequisite_boundary,
        test_forbidden_cargo_config_is_rejected,
        test_valid_log_is_accepted,
        test_case_tampering_is_rejected,
        test_duplicate_case_is_rejected,
        test_executor_workflow_identity_tampering_is_rejected,
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
