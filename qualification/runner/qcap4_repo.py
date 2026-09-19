from __future__ import annotations

import ctypes
import os
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path
from urllib.parse import urlparse

from qcap_canon import CapsuleError


def git(repo, *args, check=True):
    try:
        return subprocess.run(
            ["git", "-C", str(repo), *args],
            check=check,
            capture_output=True,
            text=True,
        )
    except (OSError, subprocess.CalledProcessError) as error:
        raise CapsuleError(f"git {' '.join(args)} failed") from error


def normalize_github_identity(url):
    if not url:
        return None
    value = url.strip()
    if value.startswith("git@github.com:"):
        path = value[len("git@github.com:") :]
    else:
        parsed = urlparse(value)
        if parsed.hostname not in {"github.com", "www.github.com"}:
            return None
        path = parsed.path.lstrip("/")
    if path.endswith(".git"):
        path = path[:-4]
    parts = [part for part in path.split("/") if part]
    return "/".join(parts[:2]) if len(parts) >= 2 else None


def preflight(manifest, repo, repository_identity):
    if repository_identity != manifest["repository_identity"]:
        raise CapsuleError("repository identity mismatch")
    if git(
        repo,
        "cat-file",
        "-e",
        manifest["product_subject_sha"] + "^{commit}",
        check=False,
    ).returncode:
        raise CapsuleError("subject unavailable")
    origin = git(repo, "remote", "get-url", "origin", check=False)
    if origin.returncode == 0:
        observed = normalize_github_identity(origin.stdout.strip())
        if observed is not None and observed != manifest["repository_identity"]:
            raise CapsuleError("configured repository origin mismatch")


def state(worktree):
    return (
        git(worktree, "rev-parse", "HEAD").stdout.strip(),
        git(worktree, "status", "--porcelain=v1", "--untracked-files=all").stdout.strip(),
    )


def add_worktree(repo, subject):
    parent = Path(tempfile.mkdtemp(prefix="qcap4-"))
    worktree = parent / "subject"
    result = git(
        repo,
        "worktree",
        "add",
        "--detach",
        "--force",
        str(worktree),
        subject,
        check=False,
    )
    if result.returncode:
        shutil.rmtree(parent, ignore_errors=True)
        raise CapsuleError("worktree materialization failed")
    return parent, worktree


def remove_worktree(repo, parent, worktree):
    result = git(repo, "worktree", "remove", "--force", str(worktree), check=False)
    cleanup_error = False
    try:
        if os.path.lexists(parent):
            shutil.rmtree(parent)
    except OSError:
        cleanup_error = True
    if result.returncode or cleanup_error or os.path.lexists(parent):
        raise CapsuleError("worktree cleanup failed")


def enable_linux_subreaper():
    if not sys.platform.startswith("linux"):
        return
    try:
        libc = ctypes.CDLL(None, use_errno=True)
        prctl = libc.prctl
        prctl.argtypes = [
            ctypes.c_int,
            ctypes.c_ulong,
            ctypes.c_ulong,
            ctypes.c_ulong,
            ctypes.c_ulong,
        ]
        prctl.restype = ctypes.c_int
        if prctl(36, 1, 0, 0, 0) != 0:
            raise OSError(ctypes.get_errno(), "prctl(PR_SET_CHILD_SUBREAPER)")
    except (AttributeError, OSError) as error:
        raise CapsuleError(f"linux subreaper setup failed: {error}") from error


def reap_children_bounded(ceiling_ms):
    if not sys.platform.startswith("linux"):
        return
    deadline = time.monotonic() + ceiling_ms / 1000
    while True:
        try:
            pid, _ = os.waitpid(-1, os.WNOHANG)
        except ChildProcessError:
            return
        if pid != 0:
            continue
        if time.monotonic() >= deadline:
            return
        time.sleep(0.01)
