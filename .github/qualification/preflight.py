#!/usr/bin/env python3
"""Non-evidentiary exact-subject rustfmt preflight v1."""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

SCHEMA = "mycelix-qualification-preflight-v1"
DOMAIN = b"MYCELIX_QUALIFICATION_PREFLIGHT_V1\0"
IMPL_DOMAIN = b"MYCELIX_QUALIFICATION_PREFLIGHT_IMPLEMENTATION_V1\0"
MAT_DOMAIN = b"MYCELIX_PREFLIGHT_MATERIALIZATION_V1\0"
OID = re.compile(r"^[0-9a-f]{40}$")
ID = re.compile(r"^[a-z0-9][a-z0-9._-]{0,127}$")
TOOLCHAIN = re.compile(r"^[0-9]+(?:\.[0-9]+){1,2}$")
FIELDS = {
    "schema", "profile_id", "repository", "required_parent_sha",
    "expected_changed_paths", "toolchain", "manifest_path",
    "probe_timeout_seconds", "rustfmt_timeout_seconds",
}
MAX_OUT = 4 * 1024 * 1024
MAX_TIME = 900
TAIL = 2048

class PreflightError(RuntimeError):
    pass

class Unavailable(RuntimeError):
    pass

# Freeze the implementation identity from the exact bytes imported, not from
# whatever bytes might later exist at the same path.
_IMPLEMENTATION_BYTES = Path(__file__).read_bytes()
IMPLEMENTATION_COMMITMENT = hashlib.sha256(
    IMPL_DOMAIN + _IMPLEMENTATION_BYTES
).hexdigest()
del _IMPLEMENTATION_BYTES

def implementation_commitment() -> str:
    return IMPLEMENTATION_COMMITMENT

def _safe_base_env(root: Path | None = None) -> dict[str, str]:
    source = os.environ
    e = dict(source)
    for key in list(e):
        if (
            key == "CARGO"
            or key.startswith("CARGO_")
            or key.startswith("RUST")
            or key.startswith("GIT_CONFIG_")
            or key in {
                "GIT_DIR", "GIT_WORK_TREE", "GIT_INDEX_FILE",
                "GIT_OBJECT_DIRECTORY", "GIT_ALTERNATE_OBJECT_DIRECTORIES",
                "GIT_COMMON_DIR",
            }
        ):
            e.pop(key, None)
    e.update({
        "GIT_CONFIG_NOSYSTEM": "1",
        "GIT_CONFIG_GLOBAL": os.devnull,
        "GIT_TERMINAL_PROMPT": "0",
        "LC_ALL": "C",
        "LANG": "C",
        "TZ": "UTC",
        "CARGO_TERM_COLOR": "never",
        "CARGO_NET_OFFLINE": "true",
        "RUST_BACKTRACE": "0",
        "RUSTUP_AUTO_INSTALL": "0",
    })
    rustup_home = source.get("RUSTUP_HOME")
    e["RUSTUP_HOME"] = rustup_home or str(
        Path(source.get("HOME", str(Path.home()))) / ".rustup"
    )
    if root is not None:
        dirs = {
            "HOME": root / "home",
            "CARGO_HOME": root / "cargo-home",
            "XDG_CONFIG_HOME": root / "xdg",
            "XDG_CACHE_HOME": root / "cache",
            "TMPDIR": root / "tmp",
        }
        for directory in dirs.values():
            directory.mkdir(parents=True, exist_ok=True)
        e.update({key: str(value) for key, value in dirs.items()})
    return e

def run(
    argv: list[str], cwd: Path, timeout: int = 60, *,
    command_root: Path | None = None
) -> subprocess.CompletedProcess[bytes]:
    try:
        p = subprocess.run(
            argv, cwd=cwd, stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            timeout=timeout, env=_safe_base_env(command_root),
        )
    except (FileNotFoundError, PermissionError, subprocess.TimeoutExpired) as exc:
        raise Unavailable(
            f"command unavailable: {argv[0]}: {type(exc).__name__}"
        ) from exc
    if len(p.stdout) > MAX_OUT or len(p.stderr) > MAX_OUT:
        raise Unavailable("command output exceeded bound")
    return p

def git(repo: Path, *args: str) -> bytes:
    p = run(
        ["git", "-c", "core.hooksPath=/dev/null", "-C", str(repo), *args],
        repo,
    )
    if p.returncode:
        raise PreflightError(
            f"git refused ({p.returncode}): {' '.join(args)}: "
            f"{p.stderr[-TAIL:].decode(errors='replace')}"
        )
    return p.stdout

def text(value: bytes, label: str) -> str:
    try:
        return value.decode("utf-8", "strict")
    except UnicodeDecodeError as exc:
        raise PreflightError(f"{label} is not UTF-8") from exc

def canonical_path(value: object) -> str:
    if (
        not isinstance(value, str)
        or not value
        or value.startswith("/")
        or "\\" in value
        or "\0" in value
        or any(part in ("", ".", "..") for part in value.split("/"))
    ):
        raise PreflightError(f"non-canonical path: {value!r}")
    return value

def bounded_timeout(value: object, label: str) -> int:
    if (
        isinstance(value, bool)
        or not isinstance(value, int)
        or not 1 <= value <= MAX_TIME
    ):
        raise PreflightError(f"invalid {label}")
    return value

def closed_pairs(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise PreflightError(f"duplicate JSON key: {key}")
        result[key] = value
    return result

def load_profile(filename: Path):
    raw = Path(filename).read_bytes()
    if len(raw) > 262144:
        raise PreflightError("profile too large")
    try:
        profile = json.loads(raw, object_pairs_hook=closed_pairs)
    except (json.JSONDecodeError, UnicodeDecodeError) as exc:
        raise PreflightError("invalid UTF-8 JSON profile") from exc
    if (
        not isinstance(profile, dict)
        or set(profile) != FIELDS
        or profile["schema"] != SCHEMA
    ):
        raise PreflightError("profile schema/fields mismatch")
    if (
        not isinstance(profile["profile_id"], str)
        or not ID.fullmatch(profile["profile_id"])
    ):
        raise PreflightError("invalid profile_id")
    if (
        not isinstance(profile["repository"], str)
        or not re.fullmatch(
            r"[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+", profile["repository"]
        )
    ):
        raise PreflightError("invalid repository")
    if (
        not isinstance(profile["required_parent_sha"], str)
        or not OID.fullmatch(profile["required_parent_sha"])
    ):
        raise PreflightError("invalid parent SHA")
    paths = profile["expected_changed_paths"]
    if not isinstance(paths, list) or len(paths) > 4096:
        raise PreflightError("invalid changed-path list")
    paths = [canonical_path(item) for item in paths]
    if paths != sorted(paths) or len(paths) != len(set(paths)):
        raise PreflightError("changed paths must be sorted/unique")
    if (
        not isinstance(profile["toolchain"], str)
        or not TOOLCHAIN.fullmatch(profile["toolchain"])
    ):
        raise PreflightError("invalid pinned Rust toolchain")
    canonical_path(profile["manifest_path"])
    bounded_timeout(profile["probe_timeout_seconds"], "probe timeout")
    bounded_timeout(profile["rustfmt_timeout_seconds"], "rustfmt timeout")
    canonical = json.dumps(
        profile, sort_keys=True, separators=(",", ":"), ensure_ascii=False
    ).encode()
    return profile, hashlib.sha256(DOMAIN + canonical).hexdigest()

_load_profile = load_profile

def normalize_origin(url: str) -> str | None:
    for regex in (
        r"https://github\.com/([^/]+/[^/]+?)(?:\.git)?$",
        r"git@github\.com:([^/]+/[^/]+?)(?:\.git)?$",
        r"ssh://git@github\.com/([^/]+/[^/]+?)(?:\.git)?$",
    ):
        match = re.fullmatch(regex, url.strip())
        if match:
            return match.group(1)
    return None

def status(repo: Path, *, ignored: bool = False) -> bytes:
    args = ["status", "--porcelain=v1", "-z", "--untracked-files=all"]
    if ignored:
        args.append("--ignored=matching")
    return git(repo, *args)

def changed(repo: Path, parent: str, subject: str) -> list[str]:
    raw = git(
        repo, "diff", "--no-ext-diff", "--no-textconv", "--no-renames",
        "--name-only", "-z", parent, subject, "--",
    )
    if not raw:
        return []
    if not raw.endswith(b"\0"):
        raise PreflightError("unterminated path output")
    paths = [
        canonical_path(text(item, "changed path"))
        for item in raw[:-1].split(b"\0")
    ]
    if len(paths) != len(set(paths)):
        raise PreflightError("duplicate changed path from Git")
    return sorted(paths)

def blob_oid(data: bytes) -> str:
    return hashlib.sha1(f"blob {len(data)}\0".encode() + data).hexdigest()

def materialization(repo: Path) -> str:
    raw = git(repo, "ls-files", "--stage", "-z")
    digest = hashlib.sha256(MAT_DOMAIN)
    if raw and not raw.endswith(b"\0"):
        raise PreflightError("unterminated index output")
    records = [] if not raw else raw[:-1].split(b"\0")
    for record in records:
        try:
            meta, raw_path = record.split(b"\t", 1)
            mode, oid, stage = meta.split(b" ", 2)
            oid_text = oid.decode("ascii")
        except (ValueError, UnicodeDecodeError) as exc:
            raise PreflightError("malformed index record") from exc
        if stage != b"0" or not OID.fullmatch(oid_text):
            raise PreflightError("invalid index entry")
        rel = canonical_path(text(raw_path, "tracked path"))
        mode_text = mode.decode("ascii")
        digest.update(mode + b"\0" + raw_path + b"\0" + oid + b"\0")
        target = repo / rel
        if mode_text == "160000":
            if target.exists() and not target.is_dir():
                raise PreflightError(f"bad gitlink materialization: {rel}")
            continue
        if mode_text == "120000":
            try:
                data = os.readlink(os.fsencode(target))
            except OSError as exc:
                raise PreflightError(
                    f"bad symlink materialization: {rel}"
                ) from exc
        elif mode_text in ("100644", "100755"):
            if not target.is_file() or target.is_symlink():
                raise PreflightError(f"bad file materialization: {rel}")
            data = target.read_bytes()
        else:
            raise PreflightError(f"unsupported mode {mode_text}: {rel}")
        if blob_oid(data) != oid_text:
            raise PreflightError(f"tracked bytes differ from Git blob: {rel}")
    return digest.hexdigest()

def clone_isolated(repo: Path, destination: Path, subject: str) -> None:
    p = run(
        [
            "git", "-c", "core.hooksPath=/dev/null",
            "clone", "--quiet", "--local", "--no-hardlinks",
            "--no-checkout", "--", str(repo), str(destination),
        ],
        repo, 120,
    )
    if p.returncode:
        raise PreflightError(
            f"isolated clone failed: {p.stderr[-TAIL:].decode(errors='replace')}"
        )
    if (destination / ".git" / "objects" / "info" / "alternates").exists():
        raise PreflightError("isolated clone unexpectedly uses object alternates")
    git(destination, "checkout", "--quiet", "--detach", subject)

def command_record(
    argv: list[str], p: subprocess.CompletedProcess[bytes]
) -> dict:
    return {
        "argv": argv,
        "exit_code": p.returncode,
        "stdout_sha256": hashlib.sha256(p.stdout).hexdigest(),
        "stderr_sha256": hashlib.sha256(p.stderr).hexdigest(),
        "stderr_tail": p.stderr[-TAIL:].decode(errors="replace"),
    }

def executable_identity(name: str) -> dict[str, str] | None:
    found = shutil.which(name)
    if not found:
        return None
    path = Path(found)
    try:
        resolved = path.resolve(strict=True)
        digest = hashlib.sha256(resolved.read_bytes()).hexdigest()
    except (OSError, RuntimeError):
        return {"path": str(path), "resolved": "", "sha256": ""}
    return {"path": str(path), "resolved": str(resolved), "sha256": digest}

def _toolchain_installed(output: bytes, toolchain: str) -> bool:
    for line in text(output, "rustup toolchain list").splitlines():
        token = line.strip().split(" ", 1)[0]
        if token == toolchain or token.startswith(toolchain + "-"):
            return True
    return False

def evaluate(repo: Path, profile_file: Path, subject: str) -> dict:
    if not isinstance(subject, str) or not OID.fullmatch(subject):
        raise PreflightError("subject must be lowercase 40-hex SHA-1")
    profile, profile_commitment = load_profile(profile_file)
    repo = Path(
        text(git(Path(repo), "rev-parse", "--show-toplevel"), "repo root").strip()
    ).resolve()
    if (
        text(git(repo, "rev-parse", "--show-object-format"), "object format").strip()
        != "sha1"
    ):
        raise PreflightError("v1 requires SHA-1 repository")
    origin = text(git(repo, "remote", "get-url", "origin"), "origin").strip()
    if normalize_origin(origin) != profile["repository"]:
        raise PreflightError("repository identity mismatch")
    if (
        text(
            git(repo, "rev-parse", "--verify", f"{subject}^{{commit}}"), "subject"
        ).strip()
        != subject
    ):
        raise PreflightError("subject resolution mismatch")
    parent = profile["required_parent_sha"]
    parents = text(
        git(repo, "rev-list", "--parents", "-n", "1", subject), "parents"
    ).split()
    if parents != [subject, parent]:
        raise PreflightError("single-parent ancestry mismatch")
    paths = changed(repo, parent, subject)
    if paths != profile["expected_changed_paths"]:
        raise PreflightError(f"changed-path mismatch: {paths!r}")

    head_before = text(git(repo, "rev-parse", "HEAD"), "caller HEAD").strip()
    status_before = status(repo, ignored=True)
    result = {
        "schema": SCHEMA,
        "preflight_implementation_commitment": IMPLEMENTATION_COMMITMENT,
        "profile_id": profile["profile_id"],
        "profile_commitment": profile_commitment,
        "repository": profile["repository"],
        "origin": origin,
        "subject_sha": subject,
        "required_parent_sha": parent,
        "changed_paths": paths,
        "classification": "INVALID",
        "qualification_result": None,
        "qualification_authority": False,
        "python_version": sys.version.split()[0],
        "git_executable": executable_identity("git"),
        "rustup_executable": executable_identity("rustup"),
        "checks": [],
    }

    with tempfile.TemporaryDirectory(prefix="mycelix-preflight-") as td:
        root = Path(td)
        subject_dir = root / "subject"
        command_root = root / "command-env"
        runtime_cwd = root / "runner"
        runtime_cwd.mkdir()
        try:
            clone_isolated(repo, subject_dir, subject)
            if text(
                git(subject_dir, "rev-parse", "HEAD"), "sandbox HEAD"
            ).strip() != subject:
                raise PreflightError("sandbox subject mismatch")
            if status(subject_dir, ignored=True):
                raise PreflightError("sandbox not clean before gate")
            mat_before = materialization(subject_dir)
            result["tracked_materialization_commitment"] = mat_before
            toolchain = profile["toolchain"]
            probe_timeout = profile["probe_timeout_seconds"]

            try:
                toolchains = run(
                    ["rustup", "toolchain", "list"], runtime_cwd,
                    probe_timeout, command_root=command_root,
                )
            except Unavailable as exc:
                result.update(classification="UNAVAILABLE", reason=str(exc))
                return result
            result["rustup_toolchain_list"] = command_record(
                ["rustup", "toolchain", "list"], toolchains
            )
            if (
                toolchains.returncode
                or not _toolchain_installed(toolchains.stdout, toolchain)
            ):
                result.update(
                    classification="UNAVAILABLE",
                    reason="pinned Rust toolchain is not locally installed",
                )
                return result

            cargo_argv = ["rustup", "run", toolchain, "cargo", "--version"]
            try:
                cargo_probe = run(
                    cargo_argv, runtime_cwd, probe_timeout,
                    command_root=command_root,
                )
            except Unavailable as exc:
                result.update(classification="UNAVAILABLE", reason=str(exc))
                return result
            result["cargo_probe"] = command_record(cargo_argv, cargo_probe)
            if (
                cargo_probe.returncode
                or not text(cargo_probe.stdout, "cargo version").startswith(
                    f"cargo {toolchain}"
                )
            ):
                result.update(
                    classification="UNAVAILABLE",
                    reason=(
                        "pinned Cargo toolchain probe failed or resolved "
                        "a different version"
                    ),
                )
                return result

            rustfmt_argv = ["rustup", "run", toolchain, "rustfmt", "--version"]
            try:
                rustfmt_probe = run(
                    rustfmt_argv, runtime_cwd, probe_timeout,
                    command_root=command_root,
                )
            except Unavailable as exc:
                result.update(classification="UNAVAILABLE", reason=str(exc))
                return result
            result["rustfmt_probe"] = command_record(
                rustfmt_argv, rustfmt_probe
            )
            if rustfmt_probe.returncode:
                result.update(
                    classification="UNAVAILABLE",
                    reason="pinned rustfmt component probe failed",
                )
                return result

            manifest = subject_dir / profile["manifest_path"]
            if not manifest.is_file() or manifest.is_symlink():
                raise PreflightError(
                    "manifest path does not materialize as a regular file"
                )
            fmt_argv = [
                "rustup", "run", toolchain, "cargo", "fmt",
                "--manifest-path", str(manifest), "--", "--check",
            ]
            try:
                fmt = run(
                    fmt_argv, runtime_cwd,
                    profile["rustfmt_timeout_seconds"],
                    command_root=command_root,
                )
            except Unavailable as exc:
                result.update(
                    classification="UNAVAILABLE", reason=f"rustfmt: {exc}"
                )
                return result
            check = command_record(fmt_argv, fmt)
            check.update(
                id="rustfmt", status="PASS" if fmt.returncode == 0 else "FAIL"
            )
            result["checks"].append(check)

            try:
                mat_after = materialization(subject_dir)
            except PreflightError as exc:
                result.update(
                    classification="NOT_ELIGIBLE",
                    reason=f"rustfmt: subject bytes mutated: {exc}",
                )
                return result
            if (
                status(subject_dir, ignored=True)
                or mat_after != mat_before
            ):
                result.update(
                    classification="NOT_ELIGIBLE",
                    reason="rustfmt: subject checkout mutated",
                )
                return result
            if fmt.returncode:
                result.update(
                    classification="NOT_ELIGIBLE",
                    reason="rustfmt: deterministic preflight gate failed",
                )
                return result
            result.update(
                classification="ELIGIBLE",
                reason="registered rustfmt preflight gate passed",
            )
            return result
        finally:
            if (
                text(git(repo, "rev-parse", "HEAD"), "caller HEAD").strip()
                != head_before
                or status(repo, ignored=True) != status_before
            ):
                raise PreflightError("preflight changed caller checkout state")

def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", default=".")
    parser.add_argument("--profile", required=True)
    parser.add_argument("--subject", required=True)
    args = parser.parse_args()
    try:
        result = evaluate(Path(args.repo), Path(args.profile), args.subject)
    except (OSError, PreflightError, Unavailable) as exc:
        result = {
            "schema": SCHEMA,
            "preflight_implementation_commitment": IMPLEMENTATION_COMMITMENT,
            "classification": "INVALID",
            "qualification_result": None,
            "qualification_authority": False,
            "reason": str(exc),
        }
    print(json.dumps(
        result, sort_keys=True, separators=(",", ":"), ensure_ascii=False
    ))
    return {
        "ELIGIBLE": 0, "NOT_ELIGIBLE": 2,
        "UNAVAILABLE": 3, "INVALID": 4,
    }.get(result["classification"], 4)

if __name__ == "__main__":
    raise SystemExit(main())
