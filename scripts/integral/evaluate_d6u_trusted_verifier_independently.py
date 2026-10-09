#!/usr/bin/env python3
"""Independent evaluator for the D6U trusted verifier.

This evaluator is intentionally kept outside the candidate test suite. It loads
the verifier under test as a module and supplies its own policy/record fixtures.
It must remain small, deterministic, and stdlib-only.
"""

from __future__ import annotations

import argparse
import hashlib
import io
import json
import importlib.util
import os
import pathlib
import subprocess
import traceback
import tempfile
import stat
import zipfile
import urllib.request
import copy
import sys
from unittest.mock import patch


EVALUATOR_OPTIMIZATION_GUARD_MESSAGE = (
    "independent D6U evaluator must not run with Python optimization enabled"
)
if not __debug__:
    raise SystemExit(EVALUATOR_OPTIMIZATION_GUARD_MESSAGE)


EXPECTED_TRUSTED_POLICY_BLOB = "f7c7581017cc5773243a2d25093e5de3cd522e2e"
EXPECTED_TRUSTED_PROGRAM_PATHS = (
    "scripts/integral/verify_d6u_trusted_artifacts.py",
    "scripts/integral/fetch_d6u_trusted_artifact.py",
    "scripts/integral/verify_d6u_trusted_attestation.py",
    "scripts/integral/emit_d6u_trusted_attestation_predicate.py",
    "scripts/integral/verify_d6u_trusted_attestation_retention.py",
)


def load_module(path: pathlib.Path, source_bytes: bytes):
    """Execute exactly the source snapshot whose digest was verified."""
    spec = importlib.util.spec_from_file_location("candidate_d6u_verifier", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"unable to load candidate verifier: {path}")
    module = importlib.util.module_from_spec(spec)
    code = compile(source_bytes, str(path), "exec")
    exec(code, module.__dict__)
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


def exercise_evaluator_optimized_mode_guard() -> None:
    """Prove the evaluator fails closed before candidate access under python -O."""
    with tempfile.TemporaryDirectory(prefix="d6u-evaluator-optimized-") as scratch:
        completed = subprocess.run(
            [
                sys.executable,
                "-O",
                str(pathlib.Path(__file__).resolve()),
                "--candidate-root",
                scratch,
            ],
            capture_output=True,
            text=True,
            check=False,
        )
    assert completed.returncode != 0, (
        "independent evaluator ran under optimized Python"
    )
    assert EVALUATOR_OPTIMIZATION_GUARD_MESSAGE in completed.stderr, (
        "independent evaluator did not fail at its optimized-mode guard: "
        f"returncode={completed.returncode}, stderr={completed.stderr!r}"
    )


def exercise_module_snapshot_loading() -> None:
    """Prove candidate-module loading does not reopen a changed source path."""
    with tempfile.TemporaryDirectory(prefix="d6u-module-snapshot-") as scratch:
        path = pathlib.Path(scratch) / "candidate.py"
        path.write_text("SNAPSHOT_MARKER = 'reopened-path'\\n", encoding="utf-8")
        pinned_source = b"SNAPSHOT_MARKER = 'validated-bytes'\\n"
        path.write_text("SNAPSHOT_MARKER = 'reopened-path'\\n", encoding="utf-8")
        module = load_module(path, pinned_source)
        assert module.SNAPSHOT_MARKER == "validated-bytes", (
            "candidate module loader executed bytes from the path instead of "
            "the validated source snapshot"
        )


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
        "trusted_workflow": {
            "path": ".github/workflows/d6u-trusted-evidence-attestation.yml",
            "blob_sha": "f" * 40,
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


def verify_candidate_policy_blob(content: bytes) -> str:
    observed = git_blob_sha1(content)
    assert observed == EXPECTED_TRUSTED_POLICY_BLOB, (
        "candidate trusted policy blob mismatch: "
        f"expected={EXPECTED_TRUSTED_POLICY_BLOB}, observed={observed}"
    )
    return observed


def candidate_regular_file(
    candidate_root: pathlib.Path,
    relative_path: str,
    label: str,
) -> pathlib.Path:
    """Resolve one expected candidate file without following any tree symlink."""
    relative = pathlib.PurePosixPath(relative_path)
    assert (
        relative_path
        and not relative.is_absolute()
        and ".." not in relative.parts
        and relative.as_posix() == relative_path
        and "\\" not in relative_path
    ), f"candidate path is not canonical relative path: {relative_path!r}"

    assert candidate_root.is_absolute(), (
        "candidate root must be an absolute path"
    )
    assert not candidate_root.is_symlink(), (
        "candidate root is symlinked"
    )
    assert candidate_root.is_dir(), (
        "candidate root is missing or not a directory"
    )
    root = candidate_root.resolve(strict=True)
    assert root == candidate_root, (
        "candidate root is not a canonical absolute path"
    )
    current = root
    for index, part in enumerate(relative.parts):
        current = current / part
        assert not current.is_symlink(), (
            f"candidate path component is symlinked: {relative_path}; component={part}"
        )
        if index < len(relative.parts) - 1:
            assert current.is_dir(), (
                f"candidate path parent is missing or not a directory: {relative_path}"
            )
        else:
            assert current.is_file(), (
                f"candidate regular file is missing or not a file: {relative_path}"
            )

    resolved = current.resolve(strict=True)
    assert resolved.is_relative_to(root), (
        f"candidate path escapes candidate root: {relative_path}"
    )
    return current


def exercise_candidate_root_cli_preserves_alias(
    candidate_root_alias: pathlib.Path,
) -> None:
    """Prove the CLI does not normalize away a supplied symlinked root."""
    observed_roots: list[pathlib.Path] = []

    def capture_root(root: pathlib.Path) -> None:
        observed_roots.append(root)

    original_run = globals()["run"]
    globals()["run"] = capture_root
    try:
        with patch.object(
            sys,
            "argv",
            [
                "evaluate_d6u_trusted_verifier_independently.py",
                "--candidate-root",
                str(candidate_root_alias),
            ],
        ):
            exit_status = main()
    finally:
        globals()["run"] = original_run

    assert exit_status == 0, (
        "candidate-root CLI alias control did not return through the expected probe"
    )
    assert observed_roots == [candidate_root_alias], (
        "candidate-root CLI normalized or altered the supplied root path: "
        f"expected={str(candidate_root_alias)!r}, "
        f"observed={[str(root) for root in observed_roots]!r}"
    )
    assert observed_roots[0].is_symlink(), (
        "candidate-root CLI alias control did not preserve the symlink"
    )


def exercise_candidate_path_guard() -> None:
    """Negative controls for path traversal and symlinked intermediate directories."""
    with tempfile.TemporaryDirectory() as scratch:
        scratch_root = pathlib.Path(scratch)
        candidate_root = scratch_root / "candidate"
        outside_root = scratch_root / "outside"
        candidate_root.mkdir()
        outside_root.mkdir()
        safe_file = candidate_root / "safe.py"
        safe_file.write_text("trusted-looking bytes\n", encoding="utf-8")
        outside_file = outside_root / "program.py"
        outside_file.write_text("trusted-looking bytes\n", encoding="utf-8")

        candidate_root_link = scratch_root / "candidate-root-link"
        candidate_root_link.symlink_to(candidate_root, target_is_directory=True)
        exercise_candidate_root_cli_preserves_alias(candidate_root_link)
        assert_rejected(
            lambda: candidate_regular_file(
                candidate_root_link, "safe.py", "candidate-root symlink fixture"
            ),
            "candidate root is symlinked",
            "candidate path guard accepted a symlinked candidate root",
        )

        candidate_parent_link = scratch_root / "candidate-parent-link"
        candidate_parent_link.symlink_to(scratch_root, target_is_directory=True)
        aliased_candidate_root = candidate_parent_link / "candidate"
        assert_rejected(
            lambda: candidate_regular_file(
                aliased_candidate_root,
                "safe.py",
                "candidate-root parent symlink fixture",
            ),
            "candidate root is not a canonical absolute path",
            "candidate path guard accepted a root reached through a symlinked parent",
        )

        (candidate_root / "scripts").symlink_to(
            outside_root, target_is_directory=True
        )
        (candidate_root / "leaf.py").symlink_to(outside_file)

        assert_rejected(
            lambda: candidate_regular_file(
                candidate_root, "scripts/program.py", "path-containment fixture"
            ),
            "candidate path component is symlinked",
            "candidate path guard accepted a symlinked intermediate directory",
        )
        assert_rejected(
            lambda: candidate_regular_file(
                candidate_root, "leaf.py", "leaf-symlink fixture"
            ),
            "candidate path component is symlinked",
            "candidate path guard accepted a symlinked final file",
        )
        assert_rejected(
            lambda: candidate_regular_file(
                candidate_root, "../outside/program.py", "path-traversal fixture"
            ),
            "candidate path is not canonical relative path",
            "candidate path guard accepted parent-directory traversal",
        )
        assert_rejected(
            lambda: candidate_regular_file(
                candidate_root, str(outside_file), "absolute-path fixture"
            ),
            "candidate path is not canonical relative path",
            "candidate path guard accepted an absolute file path",
        )
        assert_rejected(
            lambda: candidate_regular_file(
                candidate_root, r"scripts\program.py", "backslash-path fixture"
            ),
            "candidate path is not canonical relative path",
            "candidate path guard accepted a backslash-containing path",
        )


def verify_candidate_trusted_program_blobs(
    candidate_root: pathlib.Path,
    candidate_policy: dict,
    source_snapshots: dict[str, bytes] | None = None,
) -> dict[str, str]:
    trusted_programs = candidate_policy.get("trusted_programs")
    assert isinstance(trusted_programs, dict), (
        "candidate trusted programs mapping is missing"
    )
    assert set(trusted_programs) == set(EXPECTED_TRUSTED_PROGRAM_PATHS), (
        "candidate trusted program path set mismatch"
    )

    observed: dict[str, str] = {}
    for path in EXPECTED_TRUSTED_PROGRAM_PATHS:
        entry = trusted_programs.get(path)
        assert isinstance(entry, dict), (
            f"candidate trusted program pin is missing: {path}"
        )
        assert entry.get("path") == path, (
            f"candidate trusted program path mismatch: {path}"
        )
        source_path = candidate_regular_file(
            candidate_root, path, "trusted program"
        )
        source_bytes = source_path.read_bytes()
        observed_blob = git_blob_sha1(source_bytes)
        if source_snapshots is not None:
            source_snapshots[path] = source_bytes
        expected_blob = entry.get("blob_sha")
        assert expected_blob == observed_blob, (
            f"candidate trusted program blob mismatch: {path}; "
            f"expected={expected_blob!r}, observed={observed_blob!r}"
        )
        observed[path] = observed_blob

    aliases = (
        ("trusted_artifact_fetcher", "scripts/integral/fetch_d6u_trusted_artifact.py"),
        ("trusted_attestation_verifier", "scripts/integral/verify_d6u_trusted_attestation.py"),
    )
    for alias_name, path in aliases:
        alias = candidate_policy.get(alias_name)
        assert isinstance(alias, dict), (
            f"candidate trusted-program alias missing: {alias_name}"
        )
        assert alias.get("path") == path, (
            f"candidate trusted-program alias path mismatch: {alias_name}"
        )
        assert alias.get("blob_sha") == observed[path], (
            f"candidate trusted-program alias blob mismatch: {alias_name}"
        )
    return observed


def verify_candidate_policy(
    candidate_policy: dict,
    expected_record_fields: list[str],
    observed_verifier_blob: str,
    observed_fetcher_blob: str,
    observed_workflow_blob: str,
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

    fetcher_path = "scripts/integral/fetch_d6u_trusted_artifact.py"
    fetcher_cfg = candidate_policy.get("trusted_artifact_fetcher")
    assert isinstance(fetcher_cfg, dict), "candidate trusted policy fetcher pin is missing"
    assert fetcher_cfg.get("path") == fetcher_path, (
        "candidate trusted policy fetcher path mismatch"
    )
    assert fetcher_cfg.get("blob_sha") == observed_fetcher_blob, (
        "candidate fetcher blob pin mismatch"
    )
    fetcher_entry = trusted_programs.get(fetcher_path)
    assert isinstance(fetcher_entry, dict), (
        "candidate trusted policy programs omit the artifact fetcher"
    )
    assert fetcher_entry.get("path") == fetcher_path
    assert fetcher_entry.get("blob_sha") == observed_fetcher_blob, (
        "candidate trusted program fetcher pin mismatch"
    )

    workflow_path = ".github/workflows/d6u-trusted-evidence-attestation.yml"
    trusted_workflow = candidate_policy.get("trusted_workflow")
    assert isinstance(trusted_workflow, dict), (
        "candidate trusted policy workflow pin is missing"
    )
    assert trusted_workflow.get("path") == workflow_path, (
        "candidate trusted workflow path mismatch"
    )
    assert trusted_workflow.get("blob_sha") == observed_workflow_blob, (
        "candidate trusted workflow blob pin mismatch"
    )


def exercise_fetcher_api_binding(fetcher, candidate_policy: dict) -> None:
    repository = candidate_policy["repository_identity"]["full_name"]
    repository_id = int(candidate_policy["repository_identity"]["repository_id"])
    branch = candidate_policy["source_branch"]
    subject_sha = "a" * 40
    run_id = 300
    run_attempt = 2
    expected_name = f"d6u-runtime-evidence-run-{run_id}-attempt-{run_attempt}"

    event = {
        "repository": {"full_name": repository, "id": repository_id},
        "workflow_run": {
            "id": run_id,
            "run_attempt": run_attempt,
            "name": candidate_policy["workflow_name"],
            "path": candidate_policy["workflow_path"],
            "event": "workflow_run",
            "conclusion": "success",
            "repository": {"full_name": repository, "id": repository_id},
            "head_repository": {"full_name": repository, "id": repository_id},
            "head_branch": branch,
            "head_sha": subject_sha,
        },
    }
    current_run = {
        "id": run_id,
        "run_attempt": run_attempt,
        "repository": {"full_name": repository, "id": repository_id},
        "head_repository": {"full_name": repository, "id": repository_id},
        "head_branch": branch,
        "head_sha": subject_sha,
    }
    artifact = {
        "id": 17,
        "name": expected_name,
        "expired": False,
        "workflow_run": {
            "id": run_id,
            "repository_id": repository_id,
            "head_repository_id": repository_id,
            "head_branch": branch,
            "head_sha": subject_sha,
        },
        "digest": "sha256:" + "d" * 64,
        "size_in_bytes": 1024,
    }
    environment = {
        "D6U_TRUSTED_REPOSITORY_ID": str(repository_id),
        "D6U_TRIGGER_HEAD_BRANCH": branch,
        "D6U_TRIGGER_HEAD_SHA": subject_sha,
        "GITHUB_TOKEN": "fixture-token",
    }
    artifact_response = {"artifacts": [artifact]}

    with patch.dict(os.environ, environment, clear=True):
        with patch.object(
            fetcher, "github_get", side_effect=[current_run, artifact_response]
        ) as api:
            observed = fetcher.expected_artifact(repository, event, candidate_policy)
        assert observed == artifact
        assert api.call_count == 2
        assert api.call_args_list[0].args[1] == f"/actions/runs/{run_id}"
        assert api.call_args_list[1].args[1] == (
            f"/actions/runs/{run_id}/artifacts?name={expected_name}"
        )

        bad_run = copy.deepcopy(current_run)
        bad_run["id"] = run_id + 1
        assert_rejected(
            lambda: _expect_fetcher_result(
                fetcher,
                repository,
                event,
                candidate_policy,
                environment,
                [bad_run, artifact_response],
            ),
            'current_run["id"] == run_id',
            "candidate fetcher accepted mismatched current-run identity",
        )

        boolean_run = copy.deepcopy(current_run)
        boolean_run["id"] = True
        assert_rejected(
            lambda: _expect_fetcher_result(
                fetcher,
                repository,
                event,
                candidate_policy,
                environment,
                [boolean_run, artifact_response],
            ),
            "current workflow run ID must be a positive integer",
            "candidate fetcher accepted a boolean current-run ID",
        )

        boolean_attempt = copy.deepcopy(current_run)
        boolean_attempt["run_attempt"] = True
        assert_rejected(
            lambda: _expect_fetcher_result(
                fetcher,
                repository,
                event,
                candidate_policy,
                environment,
                [boolean_attempt, artifact_response],
            ),
            "current workflow run attempt must be a positive integer",
            "candidate fetcher accepted a boolean current-run attempt",
        )

        boolean_event = copy.deepcopy(event)
        boolean_event["workflow_run"]["id"] = True
        with patch.dict(os.environ, environment, clear=True):
            assert_rejected(
                lambda: fetcher.expected_artifact(
                    repository, boolean_event, candidate_policy
                ),
                "trigger workflow run ID must be a positive integer",
                "candidate fetcher accepted a boolean trigger run ID",
            )

        boolean_event_attempt = copy.deepcopy(event)
        boolean_event_attempt["workflow_run"]["run_attempt"] = True
        with patch.dict(os.environ, environment, clear=True):
            assert_rejected(
                lambda: fetcher.expected_artifact(
                    repository, boolean_event_attempt, candidate_policy
                ),
                "trigger workflow run attempt must be a positive integer",
                "candidate fetcher accepted a boolean trigger attempt",
            )

        noncanonical_repository_env = {
            **environment,
            "D6U_TRUSTED_REPOSITORY_ID": "01176351975",
        }
        with patch.dict(os.environ, noncanonical_repository_env, clear=True):
            assert_rejected(
                lambda: fetcher.expected_artifact(
                    repository, event, candidate_policy
                ),
                "D6U_TRUSTED_REPOSITORY_ID is not a canonical positive decimal integer",
                "candidate fetcher accepted a noncanonical repository ID environment value",
            )

        boolean_repo = copy.deepcopy(current_run)
        boolean_repo["repository"]["id"] = True
        assert_rejected(
            lambda: _expect_fetcher_result(
                fetcher,
                repository,
                event,
                candidate_policy,
                environment,
                [boolean_repo, artifact_response],
            ),
            "current repository ID must be a positive integer",
            "candidate fetcher accepted a boolean repository ID",
        )

        bad_attempt_run = copy.deepcopy(current_run)
        bad_attempt_run["run_attempt"] = run_attempt - 1
        assert_rejected(
            lambda: _expect_fetcher_result(
                fetcher,
                repository,
                event,
                candidate_policy,
                environment,
                [bad_attempt_run, artifact_response],
            ),
            'current_run["run_attempt"] == run_attempt',
            "candidate fetcher accepted a different current-run attempt",
        )

        boolean_artifact_id = copy.deepcopy(artifact)
        boolean_artifact_id["workflow_run"]["id"] = True
        assert_rejected(
            lambda: _expect_fetcher_result(
                fetcher,
                repository,
                event,
                candidate_policy,
                environment,
                [current_run, {"artifacts": [boolean_artifact_id]}],
            ),
            "artifact workflow run ID must be a positive integer",
            "candidate fetcher accepted a boolean artifact run ID",
        )

        bad_artifact = copy.deepcopy(artifact)
        bad_artifact["workflow_run"]["head_sha"] = "0" * 40
        assert_rejected(
            lambda: _expect_fetcher_result(
                fetcher,
                repository,
                event,
                candidate_policy,
                environment,
                [current_run, {"artifacts": [bad_artifact]}],
            ),
            'workflow_artifact_run["head_sha"] == workflow_run["head_sha"]',
            "candidate fetcher accepted an artifact from a different head SHA",
        )

        wrong_attempt_artifact = copy.deepcopy(artifact)
        wrong_attempt_artifact["name"] = (
            f"d6u-runtime-evidence-run-{run_id}-attempt-{run_attempt - 1}"
        )
        assert_rejected(
            lambda: _expect_fetcher_result(
                fetcher,
                repository,
                event,
                candidate_policy,
                environment,
                [current_run, {"artifacts": [wrong_attempt_artifact]}],
            ),
            'artifact["name"] == expected_name',
            "candidate fetcher accepted an artifact from a different run attempt",
        )

        assert_rejected(
            lambda: _expect_fetcher_result(
                fetcher,
                repository,
                event,
                candidate_policy,
                environment,
                [current_run, {"artifacts": []}],
            ),
            "expected exactly one trusted artifact",
            "candidate fetcher accepted a missing attempt-bound artifact",
        )

        assert_rejected(
            lambda: _expect_fetcher_result(
                fetcher,
                repository,
                event,
                candidate_policy,
                environment,
                [current_run, {"artifacts": [artifact, copy.deepcopy(artifact)]}],
            ),
            "expected exactly one trusted artifact",
            "candidate fetcher accepted multiple matching artifacts",
        )

        boolean_artifact_id = copy.deepcopy(artifact)
        boolean_artifact_id["id"] = True
        assert_rejected(
            lambda: _expect_fetcher_result(
                fetcher,
                repository,
                event,
                candidate_policy,
                environment,
                [current_run, {"artifacts": [boolean_artifact_id]}],
            ),
            "artifact ID must be a positive integer",
            "candidate fetcher accepted a boolean runtime artifact ID",
        )

        string_artifact_id = copy.deepcopy(artifact)
        string_artifact_id["id"] = "17"
        assert_rejected(
            lambda: _expect_fetcher_result(
                fetcher,
                repository,
                event,
                candidate_policy,
                environment,
                [current_run, {"artifacts": [string_artifact_id]}],
            ),
            "artifact ID must be a positive integer",
            "candidate fetcher accepted a string runtime artifact ID",
        )

        bad_digest = copy.deepcopy(artifact)
        bad_digest["digest"] = "sha512:" + "d" * 128
        assert_rejected(
            lambda: _expect_fetcher_result(
                fetcher,
                repository,
                event,
                candidate_policy,
                environment,
                [current_run, {"artifacts": [bad_digest]}],
            ),
            "missing or malformed GitHub artifact digest",
            "candidate fetcher accepted a malformed artifact digest",
        )

        nonhex_digest = copy.deepcopy(artifact)
        nonhex_digest["digest"] = "sha256:" + "g" * 64
        assert_rejected(
            lambda: _expect_fetcher_result(
                fetcher,
                repository,
                event,
                candidate_policy,
                environment,
                [current_run, {"artifacts": [nonhex_digest]}],
            ),
            "missing or malformed GitHub artifact digest",
            "candidate fetcher accepted a non-hex SHA-256 digest",
        )

        boolean_size = copy.deepcopy(artifact)
        boolean_size["size_in_bytes"] = True
        assert_rejected(
            lambda: _expect_fetcher_result(
                fetcher,
                repository,
                event,
                candidate_policy,
                environment,
                [current_run, {"artifacts": [boolean_size]}],
            ),
            "artifact archive size is not a nonnegative integer",
            "candidate fetcher accepted a boolean runtime-artifact size",
        )

        oversized = copy.deepcopy(artifact)
        oversized["size_in_bytes"] = int(candidate_policy["artifact_max_total_bytes"]) + 1
        assert_rejected(
            lambda: _expect_fetcher_result(
                fetcher,
                repository,
                event,
                candidate_policy,
                environment,
                [current_run, {"artifacts": [oversized]}],
            ),
            "artifact archive exceeds trusted maximum",
            "candidate fetcher accepted an oversized artifact archive",
        )

    payload = b"bounded artifact archive"
    correct_digest = "sha256:" + hashlib.sha256(payload).hexdigest()

    class FixtureOpener:
        def open(self, request, timeout=120):
            assert request.full_url == (
                f"https://api.github.com/repos/{repository}/actions/artifacts/17/zip"
            )
            assert request.get_header("Authorization") == "Bearer fixture-token"
            assert timeout == 120
            return io.BytesIO(payload)

    with tempfile.TemporaryDirectory() as scratch:
        destination = pathlib.Path(scratch) / "download.zip"
        with patch.dict(os.environ, {"GITHUB_TOKEN": "fixture-token"}, clear=True):
            with patch.object(
                fetcher.urllib.request, "build_opener", return_value=FixtureOpener()
            ):
                fetcher.download_archive(
                    repository, 17, correct_digest, destination, len(payload)
                )
        assert destination.read_bytes() == payload

        assert_rejected(
            lambda: fetcher.download_archive(
                repository, True, correct_digest, destination, len(payload)
            ),
            "artifact download ID must be a positive integer",
            "candidate fetcher attempted to download using a boolean artifact ID",
        )

        assert_rejected(
            lambda: fetcher.download_archive(
                repository, 17, "sha512:" + "0" * 128, destination, len(payload)
            ),
            "expected artifact digest is not canonical SHA-256",
            "candidate fetcher downloaded with a noncanonical expected digest",
        )

        assert_rejected(
            lambda: _download_fixture_archive(
                fetcher,
                repository,
                destination,
                payload,
                "sha256:" + "0" * 64,
                len(payload) + 1,
            ),
            "artifact archive digest mismatch",
            "candidate fetcher accepted bytes with the wrong SHA-256 digest",
        )
        assert_rejected(
            lambda: _download_fixture_archive(
                fetcher,
                repository,
                destination,
                payload,
                correct_digest,
                len(payload) - 1,
            ),
            "downloaded artifact archive exceeds trusted maximum",
            "candidate fetcher accepted an archive over its download bound",
        )

    handler = fetcher.NoAuthorizationRedirectHandler()
    request = urllib.request.Request(
        "https://api.github.com/repos/Luminous-Dynamics/mycelix/actions/artifacts/1/zip",
        headers={"Authorization": "Bearer fixture-token"},
    )
    signed_url = "https://artifact-storage.example/signed/object"
    redirected = handler.redirect_request(
        request, None, 302, "Found", {"Location": signed_url}, signed_url
    )
    assert redirected is not None
    assert redirected.full_url == signed_url
    assert redirected.get_header("Authorization") is None, (
        "artifact redirect retained the GitHub bearer token"
    )

    http_url = "http://artifact-storage.example/object"
    assert_rejected(
        lambda: handler.redirect_request(
            request, None, 302, "Found", {"Location": http_url}, http_url
        ),
        "trusted artifact redirect must remain on HTTPS",
        "candidate fetcher accepted an HTTP redirect",
    )

    credentialed_url = "https://user:password@artifact-storage.example/object"
    assert_rejected(
        lambda: handler.redirect_request(
            request,
            None,
            302,
            "Found",
            {"Location": credentialed_url},
            credentialed_url,
        ),
        "trusted artifact redirect must not introduce URL credentials",
        "candidate fetcher accepted a credentialed redirect URL",
    )


def exercise_fetcher_current_run_handoff(fetcher, candidate_policy: dict) -> None:
    repository = candidate_policy["repository_identity"]["full_name"]
    repository_id = int(candidate_policy["repository_identity"]["repository_id"])
    run_id = 450
    run_attempt = 3
    head_branch = "main"
    head_sha = "c" * 40
    expected_name = candidate_policy["auditor_handoff"]["artifact_name_template"].format(
        run_id=run_id, run_attempt=run_attempt
    )
    current_run = {
        "id": run_id,
        "run_attempt": run_attempt,
        "repository": {"full_name": repository, "id": repository_id},
        "head_repository": {"full_name": repository, "id": repository_id},
        "head_branch": head_branch,
        "head_sha": head_sha,
    }
    artifact = {
        "id": 451,
        "name": expected_name,
        "expired": False,
        "workflow_run": {
            "id": run_id,
            "repository_id": repository_id,
            "head_repository_id": repository_id,
            "head_branch": head_branch,
            "head_sha": head_sha,
        },
        "digest": "sha256:" + "e" * 64,
        "size_in_bytes": 4096,
    }
    environment = {
        "D6U_TRUSTED_REPOSITORY_ID": str(repository_id),
        "GITHUB_RUN_ID": str(run_id),
        "GITHUB_RUN_ATTEMPT": str(run_attempt),
        "GITHUB_TOKEN": "fixture-token",
        "GITHUB_REF": f"refs/heads/{head_branch}",
        "GITHUB_SHA": head_sha,
    }
    artifact_response = {"artifacts": [artifact]}

    with patch.dict(os.environ, environment, clear=True):
        with patch.object(
            fetcher, "github_get", side_effect=[current_run, artifact_response]
        ) as api:
            observed = fetcher.expected_current_run_artifact(repository, candidate_policy)
        assert observed == artifact
        assert api.call_count == 2
        assert api.call_args_list[0].args[1] == f"/actions/runs/{run_id}"
        assert api.call_args_list[1].args[1] == (
            f"/actions/runs/{run_id}/artifacts?name={expected_name}"
        )

    def expect_rejection(
        mutated_run: dict | None = None,
        response: dict | None = None,
        env_overrides: dict | None = None,
        expected: str = "",
        message: str = "",
    ) -> None:
        observed_run = current_run if mutated_run is None else mutated_run
        observed_response = artifact_response if response is None else response
        observed_env = {**environment, **(env_overrides or {})}
        _expect_current_handoff_result(
            fetcher,
            repository,
            candidate_policy,
            observed_env,
            [observed_run, observed_response],
            expected,
            message,
        )

    bad_run = copy.deepcopy(current_run)
    bad_run["id"] = run_id + 1
    expect_rejection(
        mutated_run=bad_run,
        expected='current_run["id"] == run_id',
        message="candidate fetcher accepted a current-run ID mismatch",
    )

    bool_run = copy.deepcopy(current_run)
    bool_run["id"] = True
    expect_rejection(
        mutated_run=bool_run,
        expected="current workflow run ID must be a positive integer",
        message="candidate fetcher accepted a boolean current-run ID",
    )

    expect_rejection(
        env_overrides={"GITHUB_RUN_ID": "0450"},
        expected="GITHUB_RUN_ID is not a canonical positive decimal integer",
        message="candidate fetcher accepted a noncanonical run ID environment value",
    )

    expect_rejection(
        env_overrides={"GITHUB_RUN_ATTEMPT": "03"},
        expected="GITHUB_RUN_ATTEMPT is not a canonical positive decimal integer",
        message="candidate fetcher accepted a noncanonical attempt environment value",
    )

    bad_attempt = copy.deepcopy(current_run)
    bad_attempt["run_attempt"] = run_attempt - 1
    expect_rejection(
        mutated_run=bad_attempt,
        expected='current_run["run_attempt"] == run_attempt',
        message="candidate fetcher accepted a current-run attempt mismatch",
    )

    bad_repository = copy.deepcopy(current_run)
    bad_repository["repository"]["id"] = repository_id + 1
    expect_rejection(
        mutated_run=bad_repository,
        expected='positive_json_int(current_run["repository"]["id"], "current repository ID") == expected_repository_id',
        message="candidate fetcher accepted a current-run repository ID mismatch",
    )

    expect_rejection(
        env_overrides={"GITHUB_REF": "refs/heads/not-main"},
        expected='os.environ["GITHUB_REF"] == expected_ref',
        message="candidate fetcher accepted a mismatched current-run ref",
    )

    expect_rejection(
        env_overrides={"GITHUB_SHA": "0" * 40},
        expected='current_run["head_sha"] == os.environ["GITHUB_SHA"]',
        message="candidate fetcher accepted a mismatched current-run SHA",
    )

    bool_artifact_id = copy.deepcopy(artifact)
    bool_artifact_id["workflow_run"]["id"] = True
    expect_rejection(
        response={"artifacts": [bool_artifact_id]},
        expected="handoff artifact run ID must be a positive integer",
        message="candidate fetcher accepted a boolean handoff artifact ID",
    )

    bad_artifact_size = copy.deepcopy(artifact)
    bad_artifact_size["size_in_bytes"] = True
    expect_rejection(
        response={"artifacts": [bad_artifact_size]},
        expected="auditor handoff archive size is not a nonnegative integer",
        message="candidate fetcher accepted a boolean handoff size",
    )

    bad_artifact_sha = copy.deepcopy(artifact)
    bad_artifact_sha["workflow_run"]["head_sha"] = "0" * 40
    expect_rejection(
        response={"artifacts": [bad_artifact_sha]},
        expected='workflow_artifact_run["head_sha"] == current_run["head_sha"]',
        message="candidate fetcher accepted a handoff artifact from another SHA",
    )

    bad_artifact_run_id = copy.deepcopy(artifact)
    bad_artifact_run_id["workflow_run"]["id"] = run_id + 1
    expect_rejection(
        response={"artifacts": [bad_artifact_run_id]},
        expected='positive_json_int(workflow_artifact_run["id"], "handoff artifact run ID") == run_id',
        message="candidate fetcher accepted a handoff artifact from another run ID",
    )

    bad_artifact_repo_id = copy.deepcopy(artifact)
    bad_artifact_repo_id["workflow_run"]["repository_id"] = repository_id + 1
    expect_rejection(
        response={"artifacts": [bad_artifact_repo_id]},
        expected='positive_json_int(workflow_artifact_run["repository_id"], "handoff artifact repository ID") == expected_repository_id',
        message="candidate fetcher accepted a handoff artifact from another repository",
    )

    bad_artifact_branch = copy.deepcopy(artifact)
    bad_artifact_branch["workflow_run"]["head_branch"] = "other-branch"
    expect_rejection(
        response={"artifacts": [bad_artifact_branch]},
        expected='workflow_artifact_run["head_branch"] == current_run["head_branch"]',
        message="candidate fetcher accepted a handoff artifact from another branch",
    )

    bad_artifact_attempt_name = copy.deepcopy(artifact)
    bad_artifact_attempt_name["name"] = candidate_policy["auditor_handoff"][
        "artifact_name_template"
    ].format(run_id=run_id, run_attempt=run_attempt - 1)
    expect_rejection(
        response={"artifacts": [bad_artifact_attempt_name]},
        expected='artifact["name"] == expected_name',
        message="candidate fetcher accepted a handoff artifact from another attempt",
    )

    expect_rejection(
        response={"artifacts": []},
        expected="expected exactly one current-run auditor handoff artifact",
        message="candidate fetcher accepted a missing handoff artifact",
    )

    expect_rejection(
        response={"artifacts": [artifact, copy.deepcopy(artifact)]},
        expected="expected exactly one current-run auditor handoff artifact",
        message="candidate fetcher accepted multiple current-run handoff artifacts",
    )

    expired = copy.deepcopy(artifact)
    expired["expired"] = True
    expect_rejection(
        response={"artifacts": [expired]},
        expected='artifact["expired"] is False',
        message="candidate fetcher accepted an expired handoff artifact",
    )

    bool_handoff_id = copy.deepcopy(artifact)
    bool_handoff_id["id"] = True
    expect_rejection(
        response={"artifacts": [bool_handoff_id]},
        expected="handoff artifact ID must be a positive integer",
        message="candidate fetcher accepted a boolean handoff artifact ID",
    )

    noninteger_handoff_id = copy.deepcopy(artifact)
    noninteger_handoff_id["id"] = "451"
    expect_rejection(
        response={"artifacts": [noninteger_handoff_id]},
        expected="handoff artifact ID must be a positive integer",
        message="candidate fetcher accepted a string handoff artifact ID",
    )

    bad_digest = copy.deepcopy(artifact)
    bad_digest["digest"] = "sha512:" + "e" * 128
    expect_rejection(
        response={"artifacts": [bad_digest]},
        expected="missing or malformed GitHub artifact digest",
        message="candidate fetcher accepted an invalid handoff digest prefix",
    )

    nonhex_digest = copy.deepcopy(artifact)
    nonhex_digest["digest"] = "sha256:" + "g" * 64
    expect_rejection(
        response={"artifacts": [nonhex_digest]},
        expected="missing or malformed GitHub artifact digest",
        message="candidate fetcher accepted a non-hex handoff digest",
    )

    too_large = copy.deepcopy(artifact)
    too_large["size_in_bytes"] = int(
        candidate_policy["auditor_handoff"]["artifact_max_archive_bytes"]
    ) + 1
    expect_rejection(
        response={"artifacts": [too_large]},
        expected="auditor handoff archive exceeds trusted maximum",
        message="candidate fetcher accepted an oversized handoff artifact",
    )


def _expect_current_handoff_result(
    fetcher,
    repository: str,
    policy: dict,
    environment: dict,
    api_responses: list[dict],
    expected: str,
    message: str,
) -> None:
    with patch.dict(os.environ, environment, clear=True):
        with patch.object(fetcher, "github_get", side_effect=api_responses):
            assert_rejected(
                lambda: fetcher.expected_current_run_artifact(repository, policy),
                expected,
                message,
            )


def _download_fixture_archive(
    fetcher,
    repository: str,
    destination: pathlib.Path,
    payload: bytes,
    digest: str,
    maximum: int,
) -> None:
    class FixtureOpener:
        def open(self, request, timeout=120):
            return io.BytesIO(payload)

    with patch.dict(os.environ, {"GITHUB_TOKEN": "fixture-token"}, clear=True):
        with patch.object(
            fetcher.urllib.request, "build_opener", return_value=FixtureOpener()
        ):
            fetcher.download_archive(repository, 17, digest, destination, maximum)


def _expect_fetcher_result(
    fetcher,
    repository: str,
    event: dict,
    policy: dict,
    environment: dict,
    api_responses: list[dict],
) -> dict:
    with patch.dict(os.environ, environment, clear=True):
        with patch.object(fetcher, "github_get", side_effect=api_responses):
            return fetcher.expected_artifact(repository, event, policy)


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
                patch.object(
                    verifier,
                    "sha256",
                    side_effect=lambda path: (
                        "b" * 64 if path.name == "d6u-runtime-test.log" else "e" * 64
                    ),
                ),
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
    exercise_candidate_path_guard()
    exercise_evaluator_optimized_mode_guard()
    exercise_module_snapshot_loading()
    policy_path = candidate_regular_file(
        candidate_root, "docs/integral/d6u-trusted-builder-policy.json", "policy"
    )
    verifier_path = candidate_regular_file(
        candidate_root, "scripts/integral/verify_d6u_trusted_artifacts.py", "verifier"
    )
    fetcher_path = candidate_regular_file(
        candidate_root, "scripts/integral/fetch_d6u_trusted_artifact.py", "artifact fetcher"
    )
    workflow_path = candidate_regular_file(
        candidate_root,
        ".github/workflows/d6u-trusted-evidence-attestation.yml",
        "trusted workflow",
    )

    p = policy()
    policy_bytes = policy_path.read_bytes()
    verify_candidate_policy_blob(policy_bytes)
    assert_rejected(
        lambda: verify_candidate_policy_blob(policy_bytes + b"\\n"),
        "candidate trusted policy blob mismatch",
        "candidate policy fingerprint accepted a byte-modified policy",
    )
    candidate_policy = json.loads(policy_bytes.decode("utf-8"))
    workflow_bytes = workflow_path.read_bytes()
    candidate_program_bytes: dict[str, bytes] = {}
    observed_program_blobs = verify_candidate_trusted_program_blobs(
        candidate_root, candidate_policy, source_snapshots=candidate_program_bytes
    )
    observed_verifier_blob = observed_program_blobs[
        "scripts/integral/verify_d6u_trusted_artifacts.py"
    ]
    observed_fetcher_blob = observed_program_blobs[
        "scripts/integral/fetch_d6u_trusted_artifact.py"
    ]
    observed_workflow_blob = git_blob_sha1(workflow_bytes)
    verify_candidate_policy(
        candidate_policy,
        p["record_fields"],
        observed_verifier_blob,
        observed_fetcher_blob,
        observed_workflow_blob,
    )

    # Import the same immutable byte snapshots whose blob identities passed.
    verifier = load_module(
        verifier_path,
        candidate_program_bytes["scripts/integral/verify_d6u_trusted_artifacts.py"],
    )
    fetcher = load_module(
        fetcher_path,
        candidate_program_bytes["scripts/integral/fetch_d6u_trusted_artifact.py"],
    )

    altered_program_policy = copy.deepcopy(candidate_policy)
    altered_program_policy["trusted_programs"][
        "scripts/integral/verify_d6u_trusted_attestation_retention.py"
    ]["blob_sha"] = "0" * 40
    assert_rejected(
        lambda: verify_candidate_trusted_program_blobs(
            candidate_root, altered_program_policy
        ),
        "candidate trusted program blob mismatch: scripts/integral/verify_d6u_trusted_attestation_retention.py",
        "candidate evaluator accepted an altered trusted-program blob pin",
    )

    altered_alias_policy = copy.deepcopy(candidate_policy)
    altered_alias_policy["trusted_attestation_verifier"]["blob_sha"] = "0" * 40
    assert_rejected(
        lambda: verify_candidate_trusted_program_blobs(
            candidate_root, altered_alias_policy
        ),
        "candidate trusted-program alias blob mismatch: trusted_attestation_verifier",
        "candidate evaluator accepted an altered trusted-program alias pin",
    )

    exercise_fetcher_api_binding(fetcher, candidate_policy)

    exercise_fetcher_current_run_handoff(fetcher, candidate_policy)

    tampered_policy = dict(candidate_policy)
    tampered_policy["record_fields"] = list(candidate_policy["record_fields"]) + ["extra"]
    assert_rejected(
        lambda: verify_candidate_policy(
            tampered_policy, p["record_fields"], observed_verifier_blob, observed_fetcher_blob, observed_workflow_blob
        ),
        "candidate trusted policy record schema mismatch",
        "candidate policy accepted an extra runtime record field",
    )

    tampered_policy = dict(candidate_policy)
    tampered_policy["claim_ceiling"] = "OperationallyQualified"
    assert_rejected(
        lambda: verify_candidate_policy(
            tampered_policy, p["record_fields"], observed_verifier_blob, observed_fetcher_blob, observed_workflow_blob
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
            tampered_policy, p["record_fields"], observed_verifier_blob, observed_fetcher_blob, observed_workflow_blob
        ),
        "candidate verifier blob pin mismatch",
        "candidate policy accepted a verifier blob pin mismatch",
    )

    tampered_policy = dict(candidate_policy)
    tampered_fetcher = dict(candidate_policy["trusted_artifact_fetcher"])
    tampered_fetcher["blob_sha"] = "0" * 40
    tampered_policy["trusted_artifact_fetcher"] = tampered_fetcher
    assert_rejected(
        lambda: verify_candidate_policy(
            tampered_policy, p["record_fields"], observed_verifier_blob, observed_fetcher_blob, observed_workflow_blob
        ),
        "candidate fetcher blob pin mismatch",
        "candidate policy accepted a fetcher blob pin mismatch",
    )

    tampered_policy = dict(candidate_policy)
    tampered_workflow = dict(candidate_policy["trusted_workflow"])
    tampered_workflow["blob_sha"] = "0" * 40
    tampered_policy["trusted_workflow"] = tampered_workflow
    assert_rejected(
        lambda: verify_candidate_policy(
            tampered_policy, p["record_fields"], observed_verifier_blob, observed_fetcher_blob, observed_workflow_blob
        ),
        "candidate trusted workflow blob pin mismatch",
        "candidate policy accepted a trusted-workflow blob pin mismatch",
    )

    tampered_policy = dict(candidate_policy)
    tampered_workflow = dict(candidate_policy["trusted_workflow"])
    tampered_workflow["path"] = ".github/workflows/untrusted.yml"
    tampered_policy["trusted_workflow"] = tampered_workflow
    assert_rejected(
        lambda: verify_candidate_policy(
            tampered_policy, p["record_fields"], observed_verifier_blob, observed_fetcher_blob, observed_workflow_blob
        ),
        "candidate trusted workflow path mismatch",
        "candidate policy accepted a trusted-workflow path substitution",
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

        zip_expected = set(fetcher.EXPECTED_FILES)
        valid_zip = scratch_path / "valid-artifact.zip"
        with zipfile.ZipFile(
            valid_zip, "w", compression=zipfile.ZIP_STORED
        ) as archive:
            for name in sorted(zip_expected):
                archive.writestr(name, b"safe fixture\\n")
        valid_infos = fetcher.verify_zip_members(valid_zip, candidate_policy)
        extracted = scratch_path / "valid-extracted"
        fetcher.extract_members(valid_zip, extracted, valid_infos, candidate_policy)
        assert {path.name for path in extracted.iterdir()} == zip_expected

        assert_rejected(
            lambda: fetcher.preflight_zip_entry_count(valid_zip, 2),
            "trusted artifact ZIP entry count exceeds trusted maximum",
            "candidate fetcher accepted an archive over the entry-count limit",
        )

        zero_limits = {name: 0 for name in zip_expected}
        assert_rejected(
            lambda: fetcher.verify_zip_members(
                valid_zip, candidate_policy, maximums=zero_limits
            ),
            "trusted artifact member is too large",
            "candidate fetcher accepted members over the per-file size limit",
        )

        total_limit_rejection = False
        try:
            fetcher.verify_zip_members(
                valid_zip, candidate_policy, maximum_total=0
            )
        except AssertionError as exc:
            total_limit_rejection = "uncompressed size is too large" in str(exc)
        assert total_limit_rejection, (
            "candidate fetcher accepted total uncompressed bytes over the limit"
        )

        duplicate_zip = scratch_path / "duplicate-members.zip"
        duplicate_names_expected = zip_expected | {"unlisted-extra.txt"}
        with zipfile.ZipFile(
            duplicate_zip, "w", compression=zipfile.ZIP_STORED
        ) as archive:
            for name in sorted(zip_expected):
                archive.writestr(name, b"safe\\n")
            archive.writestr(sorted(zip_expected)[0], b"duplicate\\n")
        assert_rejected(
            lambda: fetcher.verify_zip_members(
                duplicate_zip,
                candidate_policy,
                expected_files=duplicate_names_expected,
            ),
            "duplicate ZIP members",
            "candidate fetcher accepted duplicate archive member names",
        )

        symlink_zip = scratch_path / "symlink-member.zip"
        with zipfile.ZipFile(
            symlink_zip, "w", compression=zipfile.ZIP_STORED
        ) as archive:
            for name in sorted(zip_expected):
                if name == "d6u-runtime-evidence.txt":
                    info = zipfile.ZipInfo(name)
                    info.create_system = 3
                    info.external_attr = (stat.S_IFLNK | 0o777) << 16
                    archive.writestr(info, b"../../outside.txt")
                else:
                    archive.writestr(name, b"safe\\n")
        assert_rejected(
            lambda: fetcher.verify_zip_members(symlink_zip, candidate_policy),
            "trusted artifact contains symlink member",
            "candidate fetcher accepted a symlink ZIP member",
        )

        bzip_zip = scratch_path / "bzip2-members.zip"
        with zipfile.ZipFile(
            bzip_zip, "w", compression=zipfile.ZIP_BZIP2
        ) as archive:
            for name in sorted(zip_expected):
                archive.writestr(name, b"safe fixture\\n")
        assert_rejected(
            lambda: fetcher.verify_zip_members(bzip_zip, candidate_policy),
            "unsupported ZIP compression method",
            "candidate fetcher accepted an unapproved ZIP compression method",
        )

        traversal_zip = scratch_path / "nested-path.zip"
        with zipfile.ZipFile(
            traversal_zip, "w", compression=zipfile.ZIP_STORED
        ) as archive:
            archive.writestr("nested/escape.txt", b"no extraction\\n")
        with zipfile.ZipFile(traversal_zip) as archive:
            traversal_infos = archive.infolist()
        assert_rejected(
            lambda: fetcher.extract_members(
                traversal_zip,
                scratch_path / "traversal-extract",
                traversal_infos,
                candidate_policy,
                maximums={"nested/escape.txt": 1024},
            ),
            "trusted artifact member is not a root file",
            "candidate fetcher extracted a nested member path",
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

    probe = (
        "import sys; exec(compile(sys.stdin.buffer.read(), sys.argv[1], 'exec'), "
        "{'__name__': '__independent_opt_probe__', '__file__': sys.argv[1]})"
    )
    for trusted_program_path in EXPECTED_TRUSTED_PROGRAM_PATHS:
        trusted_path = candidate_root / trusted_program_path
        label = trusted_program_path
        source = candidate_program_bytes[trusted_program_path].decode("utf-8")
        assert "if not __debug__:" in source, (
            f"candidate trusted program lacks optimized-mode guard: {label}"
        )
        assert (
            "trusted D6U program must not run with Python optimization enabled"
            in source
        ), f"candidate trusted program lacks the required guard message: {label}"
        completed = subprocess.run(
            [sys.executable, "-O", "-c", probe, str(trusted_path)],
            input=candidate_program_bytes[trusted_program_path],
            capture_output=True,
            check=False,
        )
        assert completed.returncode != 0, (
            f"candidate trusted program executed under optimized Python: {label}"
        )
        stderr = completed.stderr.decode("utf-8", errors="replace")
        assert (
            "trusted D6U program must not run with Python optimization enabled"
            in stderr
        ), (
            f"candidate trusted program did not fail at its optimized-mode guard: "
            f"{label}; stderr={stderr!r}"
        )

    print("independent D6U five-program closure and optimized-mode evaluator: PASS")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--candidate-root", required=True)
    args = parser.parse_args()
    run(pathlib.Path(args.candidate_root))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
