#!/usr/bin/env python3
"""Offline exact-subject qualifier for CI-GOV-001D-A/B.

This script never calls GitHub and never mutates Actions. It verifies exact Git
object identity for the frozen planner/executor subjects, reconstructs only the
exact scripts needed for their committed tests with ``git archive``, audits the
mutation surface, executes its own adversarial qualifier tests, and emits a
canonical receipt.

V0.2 additionally refuses local Git rewrite/redirect state, disables replacement
objects for every Git subprocess, verifies the three qualifier working files
byte-for-byte against HEAD, and executes Python tests with isolated startup
flags so ambient PYTHON* or sitecustomize state cannot strengthen the theorem.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import subprocess
import sys
import tarfile
import tempfile
from pathlib import Path
from typing import Any

SCHEMA = "mycelix.ci-gov.001d.exact-qualification.v0.2"

BASE_001A = "884a14e14758a91d3c1d370d49648946dc8b89ef"
BASE_001A_TREE = "1a3190fef7ce9fc4ea9fba05aa20f3c277ec030f"
PLANNER = "4fc54eee4bc8a590d8cbe348bfa81ba649ddd1a0"
PLANNER_TREE = "b63d2443205e0488b2b796c0b058fc42495d281d"
EXECUTOR = "b55758c19bb60e3a9266f2144116dee10c5de274"
EXECUTOR_TREE = "cd898c904f3985077446c4fad87bd51d3addfa59"

QUALIFIER_SCRIPT_PATH = ".github/scripts/ci_gov_001d_exact_qualify.py"
QUALIFIER_PATHS = [
    ".github/scripts/CI_GOV_001D_Q.md",
    QUALIFIER_SCRIPT_PATH,
    ".github/scripts/test_ci_gov_001d_exact_qualify.py",
]
PLANNER_BLOBS = {
    ".github/scripts/CI_GOV_001D_A.md": "3162d033827b3595eed1eee6765dad7cce419f5c",
    ".github/scripts/ci_superseded_run_plan.py": "d8b3f8d8a1311832c4f691157ce10b85d427d04b",
    ".github/scripts/test_ci_superseded_run_plan.py": "01f73ed39610df91a26ed8faacfc3d646f29e716",
}
EXECUTOR_BLOBS = {
    ".github/scripts/CI_GOV_001D_B.lock.json": "6ba83a64657ca8701c3fcd7e35b2021ace3182db",
    ".github/scripts/CI_GOV_001D_B.md": "1f6184a731fc717be8377c0702e956495ac58fc1",
    ".github/scripts/ci_superseded_run_execute.py": "05c04120ec09908421dcf9d3599c90216805378c",
    ".github/scripts/test_ci_superseded_run_execute.py": "4affe2aff58ecf1b30d416ed8884824034b7937e",
}
READONLY_CLIENT_PATH = ".github/scripts/ci_queue_census.py"
READONLY_CLIENT_BLOB = "3782de74d3b67f9e4a234a53d76c3d6eca3f84a0"

SUBJECT_ARCHIVE_PATHS = [
    READONLY_CLIENT_PATH,
    ".github/scripts/ci_superseded_run_plan.py",
    ".github/scripts/test_ci_superseded_run_plan.py",
    ".github/scripts/ci_superseded_run_execute.py",
    ".github/scripts/test_ci_superseded_run_execute.py",
]

# These can redirect the repository/index/object database observed by Git. Exact
# qualification refuses them rather than trying to reason about caller intent.
GIT_REDIRECT_ENV = (
    "GIT_DIR",
    "GIT_WORK_TREE",
    "GIT_INDEX_FILE",
    "GIT_OBJECT_DIRECTORY",
    "GIT_ALTERNATE_OBJECT_DIRECTORIES",
    "GIT_COMMON_DIR",
    "GIT_REPLACE_REF_BASE",
)
PYTHON_TEST_FLAGS = ("-E", "-s", "-S", "-B")

EXPECTED_LOCK = {
    "schema": "mycelix.ci-gov.001d-b.lock.v0.1",
    "parent_001d_a_commit": PLANNER,
    "parent_001d_a_tree": PLANNER_TREE,
    "parent_planner_blob_oid": PLANNER_BLOBS[".github/scripts/ci_superseded_run_plan.py"],
    "parent_readonly_client_blob_oid": READONLY_CLIENT_BLOB,
    "payload_git_blobs": {
        ".github/scripts/CI_GOV_001D_B.md": EXECUTOR_BLOBS[".github/scripts/CI_GOV_001D_B.md"],
        ".github/scripts/ci_superseded_run_execute.py": EXECUTOR_BLOBS[".github/scripts/ci_superseded_run_execute.py"],
        ".github/scripts/test_ci_superseded_run_execute.py": EXECUTOR_BLOBS[".github/scripts/test_ci_superseded_run_execute.py"],
    },
    "expected_added_paths": sorted(EXECUTOR_BLOBS),
    "mutation_contract": {
        "allowed_http_methods": ["POST"],
        "allowed_endpoint_template": "/repos/{repository}/actions/runs/{run_id}/cancel",
        "max_selected_runs": 5,
        "dry_run_default": True,
        "apply_requires_receipt_output": True,
        "pr_advanced_since_plan_policy": "refuse-and-replan",
    },
}


class QualificationError(RuntimeError):
    pass


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def canonical_sha256(value: Any) -> str:
    encoded = json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True).encode()
    return sha256_bytes(encoded)


def git_env() -> dict[str, str]:
    env = os.environ.copy()
    env["GIT_NO_REPLACE_OBJECTS"] = "1"
    return env


def git_bytes(*args: str, cwd: Path | None = None) -> bytes:
    proc = subprocess.run(
        ["git", *args],
        cwd=cwd,
        env=git_env(),
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    if proc.returncode != 0:
        raise QualificationError(
            f"git {' '.join(args)} failed ({proc.returncode}): "
            f"{proc.stderr.decode(errors='replace').strip()}"
        )
    return proc.stdout


def git_text(*args: str, cwd: Path | None = None) -> str:
    return git_bytes(*args, cwd=cwd).decode("utf-8").strip()


def require_equal(label: str, actual: Any, expected: Any) -> None:
    if actual != expected:
        raise QualificationError(f"{label}: expected {expected!r}, got {actual!r}")


def ensure_no_git_rewrite_state() -> None:
    present = sorted(name for name in GIT_REDIRECT_ENV if os.environ.get(name))
    if present:
        raise QualificationError(
            "Git repository redirect environment is forbidden during exact qualification: "
            + ", ".join(present)
        )

    replace_refs = git_text("for-each-ref", "--format=%(refname)", "refs/replace")
    if replace_refs:
        raise QualificationError(
            "Git replace refs are forbidden during exact qualification: "
            + ", ".join(replace_refs.splitlines())
        )

    grafts = Path(git_text("rev-parse", "--git-path", "info/grafts"))
    if not grafts.is_absolute():
        grafts = (Path.cwd() / grafts).resolve()
    if grafts.exists() and grafts.read_bytes().strip():
        raise QualificationError("non-empty .git/info/grafts is forbidden during exact qualification")


def commit_tree(commit: str) -> str:
    return git_text("rev-parse", f"{commit}^{{tree}}")


def commit_parents(commit: str) -> list[str]:
    fields = git_text("rev-list", "--parents", "-n", "1", commit).split()
    if not fields or fields[0] != commit:
        raise QualificationError(f"unexpected rev-list identity for {commit}")
    return fields[1:]


def changed_paths(parent: str, child: str) -> list[str]:
    raw = git_text("diff-tree", "--no-commit-id", "--name-only", "-r", parent, child)
    return sorted(line for line in raw.splitlines() if line)


def blob_oid(commit: str, path: str) -> str:
    return git_text("rev-parse", f"{commit}:{path}")


def git_show(commit: str, path: str) -> bytes:
    return git_bytes("show", f"{commit}:{path}")


def verify_qualifier_files_match_head(root: Path, head: str) -> None:
    for path in QUALIFIER_PATHS:
        disk_path = root / path
        if not disk_path.is_file():
            raise QualificationError(f"qualifier working file missing: {path}")
        require_equal(
            f"qualifier working bytes {path}",
            disk_path.read_bytes(),
            git_show(head, path),
        )


def verify_qualifier_checkout() -> tuple[str, Path]:
    head = git_text("rev-parse", "HEAD")
    root = Path(git_text("rev-parse", "--show-toplevel")).resolve()
    require_equal(
        "executed qualifier path",
        Path(__file__).resolve(),
        (root / QUALIFIER_SCRIPT_PATH).resolve(),
    )
    require_equal("qualifier parents", commit_parents(head), [EXECUTOR])
    require_equal("qualifier added paths", changed_paths(EXECUTOR, head), sorted(QUALIFIER_PATHS))
    require_equal(
        "qualifier preflight cleanliness",
        git_text("status", "--porcelain", "--untracked-files=all"),
        "",
    )
    verify_qualifier_files_match_head(root, head)
    return head, root


def verify_identity() -> dict[str, Any]:
    require_equal("001A tree", commit_tree(BASE_001A), BASE_001A_TREE)
    require_equal("planner tree", commit_tree(PLANNER), PLANNER_TREE)
    require_equal("executor tree", commit_tree(EXECUTOR), EXECUTOR_TREE)
    require_equal("planner parents", commit_parents(PLANNER), [BASE_001A])
    require_equal("executor parents", commit_parents(EXECUTOR), [PLANNER])
    require_equal("planner added paths", changed_paths(BASE_001A, PLANNER), sorted(PLANNER_BLOBS))
    require_equal("executor added paths", changed_paths(PLANNER, EXECUTOR), sorted(EXECUTOR_BLOBS))

    observed: dict[str, str] = {}
    for path, expected in {**PLANNER_BLOBS, **EXECUTOR_BLOBS}.items():
        commit = PLANNER if path in PLANNER_BLOBS else EXECUTOR
        actual = blob_oid(commit, path)
        require_equal(f"blob {path}", actual, expected)
        observed[path] = actual
    actual_readonly = blob_oid(BASE_001A, READONLY_CLIENT_PATH)
    require_equal("qualified read-only client blob", actual_readonly, READONLY_CLIENT_BLOB)
    observed[READONLY_CLIENT_PATH] = actual_readonly
    return observed


def verify_lock() -> str:
    data = git_show(EXECUTOR, ".github/scripts/CI_GOV_001D_B.lock.json")
    try:
        lock = json.loads(data)
    except json.JSONDecodeError as exc:
        raise QualificationError(f"executor lock is invalid JSON: {exc}") from exc
    require_equal("executor lock", lock, EXPECTED_LOCK)
    return sha256_bytes(data)


def verify_authority_surface() -> dict[str, Any]:
    executor = git_show(EXECUTOR, ".github/scripts/ci_superseded_run_execute.py").decode()
    planner = git_show(PLANNER, ".github/scripts/ci_superseded_run_plan.py").decode()
    readonly = git_show(BASE_001A, READONLY_CLIENT_PATH).decode()

    require_equal("executor POST count", executor.count('method="POST"'), 1)
    if '/actions/runs/{run_id}/cancel' not in executor:
        raise QualificationError("executor cancel endpoint missing")
    for forbidden in ('method="DELETE"', 'method="PATCH"', 'method="PUT"'):
        if forbidden in executor:
            raise QualificationError(f"forbidden executor mutation method present: {forbidden}")
    for forbidden_arg in ('--all', '--workflow-path', '--now'):
        if forbidden_arg in executor:
            raise QualificationError(f"forbidden executor CLI widening present: {forbidden_arg}")

    if "GitHubReadOnlyClient" not in planner:
        raise QualificationError("planner no longer imports qualified read-only client")
    if "cancel_run(" in planner or 'method="POST"' in planner:
        raise QualificationError("planner contains mutation authority")
    require_equal("read-only client GET count", readonly.count('method="GET"'), 1)
    for forbidden in ('method="POST"', 'method="DELETE"', 'method="PATCH"', 'method="PUT"'):
        if forbidden in readonly:
            raise QualificationError(f"qualified read-only client contains {forbidden}")

    return {
        "executor_post_count": 1,
        "allowed_endpoint": "/repos/{repository}/actions/runs/{run_id}/cancel",
        "planner_mutation_authority": False,
        "readonly_client_get_only": True,
    }


def safe_extract_archive(archive: Path, destination: Path) -> list[str]:
    regular_files: list[str] = []
    with tarfile.open(archive, "r") as tf:
        for member in tf.getmembers():
            member_path = Path(member.name)
            if member_path.is_absolute() or ".." in member_path.parts:
                raise QualificationError(f"unsafe archive member path: {member.name}")
            if member.issym() or member.islnk() or member.isdev():
                raise QualificationError(f"unsafe archive member type: {member.name}")
            if not (member.isdir() or member.isfile()):
                raise QualificationError(f"unsupported archive member type: {member.name}")
            if member.isfile():
                regular_files.append(member.name)
        tf.extractall(destination)
    return sorted(regular_files)


def verify_archive_files(observed: list[str]) -> None:
    require_equal("subject archive regular files", sorted(observed), sorted(SUBJECT_ARCHIVE_PATHS))


def run_python_test(root: Path, relative_test: str) -> dict[str, Any]:
    command = [sys.executable, *PYTHON_TEST_FLAGS, relative_test]
    proc = subprocess.run(
        command,
        cwd=root,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    item = {
        "command": ["python", *PYTHON_TEST_FLAGS, relative_test],
        "returncode": proc.returncode,
        "stdout_sha256": sha256_bytes(proc.stdout),
        "stderr_sha256": sha256_bytes(proc.stderr),
    }
    if proc.returncode != 0:
        raise QualificationError(
            f"test failed: {relative_test}\n"
            f"stdout:\n{proc.stdout.decode(errors='replace')}\n"
            f"stderr:\n{proc.stderr.decode(errors='replace')}"
        )
    return item


def run_qualifier_tests(root: Path) -> list[dict[str, Any]]:
    return [run_python_test(root, ".github/scripts/test_ci_gov_001d_exact_qualify.py")]


def run_subject_tests() -> list[dict[str, Any]]:
    with tempfile.TemporaryDirectory(prefix="ci-gov-001d-q-") as tmp:
        root = Path(tmp)
        archive = root / "subject.tar"
        command = ["git", "archive", "--format=tar", EXECUTOR, "--", *SUBJECT_ARCHIVE_PATHS]
        with archive.open("wb") as handle:
            proc = subprocess.run(
                command,
                env=git_env(),
                stdout=handle,
                stderr=subprocess.PIPE,
            )
        if proc.returncode != 0:
            raise QualificationError(
                f"git archive failed: {proc.stderr.decode(errors='replace').strip()}"
            )
        subject = root / "subject"
        subject.mkdir()
        observed_files = safe_extract_archive(archive, subject)
        verify_archive_files(observed_files)
        return [
            run_python_test(subject, ".github/scripts/test_ci_superseded_run_plan.py"),
            run_python_test(subject, ".github/scripts/test_ci_superseded_run_execute.py"),
        ]


def build_receipt() -> dict[str, Any]:
    qualifier_head, qualifier_root = verify_qualifier_checkout()
    qualifier_tests = run_qualifier_tests(qualifier_root)
    observed_blobs = verify_identity()
    lock_sha = verify_lock()
    authority = verify_authority_surface()
    tests = run_subject_tests()
    require_equal(
        "qualifier postflight cleanliness",
        git_text("status", "--porcelain", "--untracked-files=all"),
        "",
    )
    verify_qualifier_files_match_head(qualifier_root, qualifier_head)

    receipt: dict[str, Any] = {
        "schema": SCHEMA,
        "qualifier_head": qualifier_head,
        "qualifier_tree": commit_tree(qualifier_head),
        "qualified_001a": {"commit": BASE_001A, "tree": BASE_001A_TREE},
        "planner": {"commit": PLANNER, "tree": PLANNER_TREE},
        "executor": {"commit": EXECUTOR, "tree": EXECUTOR_TREE},
        "observed_blob_oids": observed_blobs,
        "executor_lock_sha256": lock_sha,
        "subject_archive_paths": list(SUBJECT_ARCHIVE_PATHS),
        "git_object_rewrite_policy": {
            "replacement_objects_disabled": True,
            "replace_refs_forbidden": True,
            "grafts_forbidden": True,
            "redirect_environment_forbidden": list(GIT_REDIRECT_ENV),
        },
        "python_test_flags": list(PYTHON_TEST_FLAGS),
        "authority_surface": authority,
        "qualifier_tests": qualifier_tests,
        "subject_tests": tests,
        "python": sys.version.split()[0],
        "proposition": (
            "Exact frozen CI-GOV-001D-A/B Git objects satisfy their bound source/lock "
            "identity, isolated offline committed unit tests, and narrow read-only/planned "
            "cancel authority surface under this execution environment."
        ),
        "nonclaims": [
            "No GitHub Actions run was cancelled or mutated.",
            "No current product or scientific subject is qualified by this receipt.",
            "No runner availability, branch protection, GitHub platform integrity, or future plan eligibility is established.",
            "A future cancellation still requires a fresh unexpired plan and explicit operator authority.",
        ],
    }
    receipt["receipt_commitment"] = canonical_sha256(receipt)
    return receipt


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--receipt-output", type=Path)
    return parser.parse_args()


def ensure_receipt_output_outside_checkout(path: Path | None) -> None:
    if path is None:
        return
    checkout = Path(git_text("rev-parse", "--show-toplevel")).resolve()
    output = path.expanduser().resolve(strict=False)
    if output == checkout or checkout in output.parents:
        raise QualificationError(
            "--receipt-output must be outside the repository checkout so postflight cleanliness remains meaningful"
        )


def main() -> int:
    args = parse_args()
    try:
        ensure_no_git_rewrite_state()
        ensure_receipt_output_outside_checkout(args.receipt_output)
        receipt = build_receipt()
    except QualificationError as exc:
        print(f"QUALIFICATION FAIL: {exc}", file=sys.stderr)
        return 1
    rendered = json.dumps(receipt, indent=2, sort_keys=True) + "\n"
    if args.receipt_output:
        args.receipt_output.parent.mkdir(parents=True, exist_ok=True)
        args.receipt_output.write_text(rendered, encoding="utf-8")
    print(rendered, end="")
    return 0


if __name__ == "__main__":
    sys.exit(main())
