#!/usr/bin/env python3
"""Exact-base/head Forge changed-path evidence producer.

Read-only Git plumbing only. This producer does not decide which CI lanes run.
A future selector may consume its complete path set only when the workflow binds
base/head to the exact PR event subject.

Any refusal from this tool must be interpreted upstream as FullMatrix.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import subprocess
import sys
from pathlib import Path


SCHEMA = "mycelix-forge-change-set-v1"
DIFF_PROFILE = "git-tree-diff-no-renames-v1"
OBJECT_FORMAT = "sha1"
_OID_RE = re.compile(r"^[0-9a-f]{40}$")
MAX_DIFF_BYTES = 16 * 1024 * 1024
GIT_TIMEOUT_SECONDS = 30


class ChangeSetError(RuntimeError):
    pass


def _canonical_path(raw: str) -> str:
    if not raw or raw.startswith("/") or "\\" in raw:
        raise ChangeSetError(f"non-canonical repository path: {raw!r}")
    parts = raw.split("/")
    if any(part in ("", ".", "..") for part in parts):
        raise ChangeSetError(f"non-canonical repository path: {raw!r}")
    return raw


def _validate_oid(raw: str, label: str) -> str:
    if _OID_RE.fullmatch(raw) is None:
        raise ChangeSetError(f"{label} must be lowercase 40-hex SHA-1")
    return raw


def _git_env() -> dict[str, str]:
    env = os.environ.copy()
    env["GIT_CONFIG_NOSYSTEM"] = "1"
    env["GIT_TERMINAL_PROMPT"] = "0"
    env["LC_ALL"] = "C"
    env["LANG"] = "C"
    return env


def _run_git(
    repo_root: Path,
    args: list[str],
    *,
    allowed_returncodes: tuple[int, ...] = (0,),
) -> subprocess.CompletedProcess[bytes]:
    command = ["git", "-C", str(repo_root), *args]
    try:
        completed = subprocess.run(
            command,
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            env=_git_env(),
            timeout=GIT_TIMEOUT_SECONDS,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise ChangeSetError(f"Git execution failed: {type(exc).__name__}") from exc

    if completed.returncode not in allowed_returncodes:
        diagnostic = completed.stderr[:2048].decode("utf-8", errors="replace").strip()
        raise ChangeSetError(
            f"Git command refused ({completed.returncode}): {' '.join(args)}"
            + (f": {diagnostic}" if diagnostic else "")
        )
    if len(completed.stdout) > MAX_DIFF_BYTES:
        raise ChangeSetError("Git output exceeds v1 bound")
    return completed


def _git_version(repo_root: Path) -> str:
    result = _run_git(repo_root, ["--version"])
    try:
        version = result.stdout.decode("utf-8", errors="strict").strip()
    except UnicodeDecodeError as exc:
        raise ChangeSetError("Git version output is not UTF-8") from exc
    if not version.startswith("git version ") or "\n" in version or "\r" in version:
        raise ChangeSetError("unexpected Git version output")
    return version


def _require_object_format(repo_root: Path) -> str:
    result = _run_git(repo_root, ["rev-parse", "--show-object-format"])
    try:
        object_format = result.stdout.decode("ascii", errors="strict").strip()
    except UnicodeDecodeError as exc:
        raise ChangeSetError("repository object format is not ASCII") from exc
    if object_format != OBJECT_FORMAT:
        raise ChangeSetError(f"unsupported repository object format: {object_format!r}")
    return object_format


def _require_commit(repo_root: Path, oid: str, label: str) -> None:
    _run_git(repo_root, ["cat-file", "-e", f"{oid}^{{commit}}"])


def _require_ancestor(repo_root: Path, base: str, head: str) -> None:
    result = _run_git(
        repo_root,
        ["merge-base", "--is-ancestor", base, head],
        allowed_returncodes=(0, 1),
    )
    if result.returncode == 1:
        raise ChangeSetError("v1 requires base commit to be an ancestor of head")


def _changed_paths(repo_root: Path, base: str, head: str) -> list[str]:
    result = _run_git(
        repo_root,
        [
            "-c",
            "core.quotepath=false",
            "diff",
            "--no-ext-diff",
            "--no-textconv",
            "--no-renames",
            "--name-only",
            "-z",
            base,
            head,
            "--",
        ],
    )
    raw = result.stdout
    if not raw:
        return []
    if not raw.endswith(b"\0"):
        raise ChangeSetError("Git changed-path stream is not NUL terminated")

    paths: set[str] = set()
    for item in raw[:-1].split(b"\0"):
        if not item:
            raise ChangeSetError("Git changed-path stream contains empty path")
        try:
            decoded = item.decode("utf-8", errors="strict")
        except UnicodeDecodeError as exc:
            raise ChangeSetError("v1 refuses non-UTF-8 repository paths") from exc
        paths.add(_canonical_path(decoded))
    return sorted(paths)


def _commitment(payload: dict) -> str:
    canonical = json.dumps(payload, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode("utf-8")
    return hashlib.sha256(b"mycelix-forge/change-set/v1\0" + canonical).hexdigest()


def derive_change_set(repo_root: Path, base: str, head: str) -> dict:
    root = repo_root.resolve(strict=True)
    base = _validate_oid(base, "base")
    head = _validate_oid(head, "head")

    object_format = _require_object_format(root)
    _require_commit(root, base, "base")
    _require_commit(root, head, "head")
    _require_ancestor(root, base, head)
    version = _git_version(root)
    paths = _changed_paths(root, base, head)

    payload = {
        "schema": SCHEMA,
        "git_object_format": object_format,
        "git_version": version,
        "base_commit": base,
        "head_commit": head,
        "diff_profile": DIFF_PROFILE,
        "changed_paths": paths,
        "changed_path_count": len(paths),
    }
    result = dict(payload)
    result["evidence_commitment"] = _commitment(payload)
    return result


def _repository_root() -> Path:
    return Path(__file__).resolve().parent.parent.parent


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("base", help="exact PR/base commit SHA-1")
    parser.add_argument("head", help="exact PR/head commit SHA-1")
    args = parser.parse_args(argv)

    try:
        result = derive_change_set(_repository_root(), args.base, args.head)
    except (OSError, ChangeSetError) as exc:
        # Future workflow integration MUST interpret refusal as FullMatrix.
        print(f"forge change-set producer refused input: {exc}", file=sys.stderr)
        return 2

    print(json.dumps(result, sort_keys=True, separators=(",", ":"), ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
