#!/usr/bin/env python3
"""Bound execution-environment adapter for Mycelix non-evidentiary preflight."""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import platform
import re
import shutil
import stat
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Any

SCHEMA = "mycelix-preflight-execution-environment-v1"
DOMAIN = b"MYCELIX_PREFLIGHT_EXECUTION_ENVIRONMENT_V1\0"
IMPL_DOMAIN = b"MYCELIX_PREFLIGHT_ENV_ADAPTER_IMPLEMENTATION_V1\0"
CHILD_SCHEMA = "mycelix-qualification-preflight-v1"
CHILD_IMPL_DOMAIN = b"MYCELIX_QUALIFICATION_PREFLIGHT_IMPLEMENTATION_V1\0"
CHILD_PROFILE_DOMAIN = b"MYCELIX_QUALIFICATION_PREFLIGHT_V1\0"
POLICY_REVISION = "bound-child-env-v2"
CLASS_EXIT = {"ELIGIBLE": 0, "NOT_ELIGIBLE": 2, "UNAVAILABLE": 3, "INVALID": 4}
TOOLCHAIN = re.compile(r"^[0-9]+(?:\.[0-9]+){1,2}$")
MAX_OUTPUT = 4 * 1024 * 1024
MAX_PROFILE = 256 * 1024
MAX_CHILD = 1024 * 1024
MAX_TIMEOUT = 3600

_IMPL_BYTES = Path(__file__).read_bytes()
IMPLEMENTATION_COMMITMENT = hashlib.sha256(IMPL_DOMAIN + _IMPL_BYTES).hexdigest()
del _IMPL_BYTES


class EnvironmentError(RuntimeError):
    pass


class EnvironmentUnavailable(RuntimeError):
    pass


def _sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def _strict_json_object(raw_bytes: bytes, label: str) -> dict[str, Any]:
    try:
        text = raw_bytes.decode("utf-8", "strict")
    except UnicodeDecodeError as exc:
        raise EnvironmentError(f"{label} is not UTF-8") from exc

    def closed_pairs(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
        result: dict[str, Any] = {}
        for key, value in pairs:
            if key in result:
                raise EnvironmentError(f"duplicate JSON key in {label}: {key}")
            result[key] = value
        return result

    try:
        value = json.loads(text, object_pairs_hook=closed_pairs)
    except json.JSONDecodeError as exc:
        raise EnvironmentError(f"{label} is not valid JSON") from exc
    if not isinstance(value, dict):
        raise EnvironmentError(f"{label} must be a JSON object")
    return value


def _bind_profile(profile_path: Path) -> dict[str, Any]:
    try:
        raw = profile_path.read_bytes()
    except OSError as exc:
        raise EnvironmentError("cannot read preflight profile") from exc
    if len(raw) > MAX_PROFILE:
        raise EnvironmentError("preflight profile exceeds size bound")
    profile = _strict_json_object(raw, "preflight profile")
    toolchain = profile.get("toolchain")
    if not isinstance(toolchain, str) or not TOOLCHAIN.fullmatch(toolchain):
        raise EnvironmentError("profile has no valid numeric Rust toolchain")
    canonical = json.dumps(
        profile, sort_keys=True, separators=(",", ":"), ensure_ascii=False
    ).encode("utf-8")
    return {
        "raw": raw,
        "raw_sha256": _sha256_bytes(raw),
        "profile_commitment": hashlib.sha256(
            CHILD_PROFILE_DOMAIN + canonical
        ).hexdigest(),
        "toolchain": toolchain,
    }


def _source_bytes_still_match(path: Path, expected_sha256: str, *, max_bytes: int) -> bool:
    try:
        data = path.read_bytes()
    except OSError:
        return False
    if len(data) > max_bytes:
        return False
    return _sha256_bytes(data) == expected_sha256


def _symlink_chain(path: Path) -> list[str]:
    chain: list[str] = []
    current = path
    for _ in range(40):
        chain.append(str(current))
        try:
            if not current.is_symlink():
                return chain
            target = Path(os.readlink(current))
        except OSError as exc:
            raise EnvironmentUnavailable(
                f"cannot read executable symlink: {current}"
            ) from exc
        current = target if target.is_absolute() else current.parent / target
    raise EnvironmentUnavailable("executable symlink chain too deep")


def identity_for_path(requested_name: str, discovered: Path) -> dict[str, Any]:
    discovered = discovered.absolute()
    chain = _symlink_chain(discovered)
    try:
        resolved = discovered.resolve(strict=True)
        discovered_stat = discovered.lstat()
        resolved_stat = resolved.stat()
    except (OSError, RuntimeError) as exc:
        raise EnvironmentUnavailable(
            f"cannot resolve executable: {requested_name}"
        ) from exc
    if not stat.S_ISREG(resolved_stat.st_mode) or not os.access(resolved, os.X_OK):
        raise EnvironmentUnavailable(
            f"resolved executable is not executable regular file: {requested_name}"
        )
    return {
        "requested_name": requested_name,
        "discovered_path": str(discovered),
        "symlink_chain": chain,
        "resolved_path": str(resolved),
        "discovered_mode": stat.S_IMODE(discovered_stat.st_mode),
        "resolved_mode": stat.S_IMODE(resolved_stat.st_mode),
        "size": resolved_stat.st_size,
        "device": resolved_stat.st_dev,
        "inode": resolved_stat.st_ino,
        "sha256": _sha256_file(resolved),
    }


def resolve_executable(name: str) -> dict[str, Any]:
    found = shutil.which(name)
    if not found:
        raise EnvironmentUnavailable(f"required executable not found: {name}")
    return identity_for_path(name, Path(found))


def identity_still_matches(identity: dict[str, Any]) -> bool:
    try:
        current = identity_for_path(
            identity["requested_name"], Path(identity["discovered_path"])
        )
    except (EnvironmentUnavailable, KeyError, TypeError):
        return False
    return current == identity


def _run(
    argv: list[str],
    env: dict[str, str],
    cwd: Path,
    timeout: int = 60,
) -> subprocess.CompletedProcess[bytes]:
    if isinstance(timeout, bool) or not isinstance(timeout, int) or not 1 <= timeout <= MAX_TIMEOUT:
        raise EnvironmentError("invalid execution timeout")
    try:
        completed = subprocess.run(
            argv,
            cwd=cwd,
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=timeout,
            env=env,
        )
    except (FileNotFoundError, PermissionError, subprocess.TimeoutExpired) as exc:
        raise EnvironmentUnavailable(
            f"command unavailable: {argv[0]}: {type(exc).__name__}"
        ) from exc
    if len(completed.stdout) > MAX_OUTPUT or len(completed.stderr) > MAX_OUTPUT:
        raise EnvironmentUnavailable("command output exceeded bound")
    return completed


def _rustup_which(
    rustup: str,
    toolchain: str,
    command: str,
    env: dict[str, str],
    cwd: Path,
) -> dict[str, Any]:
    completed = _run(
        [rustup, "which", "--toolchain", toolchain, command], env, cwd
    )
    if completed.returncode:
        raise EnvironmentUnavailable(
            f"rustup cannot resolve {command} for {toolchain}"
        )
    try:
        value = completed.stdout.decode("utf-8", "strict").strip()
    except UnicodeDecodeError as exc:
        raise EnvironmentUnavailable(
            f"rustup returned non-UTF-8 path for {command}"
        ) from exc
    if not value or "\n" in value:
        raise EnvironmentUnavailable(f"rustup returned invalid path for {command}")
    path = Path(value)
    if not path.is_absolute():
        raise EnvironmentUnavailable(
            f"rustup returned non-absolute path for {command}"
        )
    return identity_for_path(command, path)


def _canonical_commitment(manifest: dict[str, Any]) -> str:
    raw = json.dumps(
        manifest, sort_keys=True, separators=(",", ":"), ensure_ascii=False
    ).encode("utf-8")
    return hashlib.sha256(DOMAIN + raw).hexdigest()


def _source_rustup_home() -> str:
    explicit = os.environ.get("RUSTUP_HOME")
    if explicit:
        path = Path(explicit)
        if not path.is_absolute():
            raise EnvironmentUnavailable("RUSTUP_HOME must be absolute when explicitly set")
        return str(path)
    home = Path(os.environ.get("HOME", str(Path.home())))
    if not home.is_absolute():
        raise EnvironmentUnavailable("HOME must be absolute when deriving RUSTUP_HOME")
    return str(home / ".rustup")


def _child_env(root: Path, bin_dir: Path, rustup_home: str) -> dict[str, str]:
    home = root / "home"
    tmp = root / "tmp"
    xdg = root / "xdg"
    cache = root / "cache"
    for directory in (home, tmp, xdg, cache):
        directory.mkdir(parents=True, exist_ok=True)
    return {
        "PATH": str(bin_dir),
        "HOME": str(home),
        "TMPDIR": str(tmp),
        "XDG_CONFIG_HOME": str(xdg),
        "XDG_CACHE_HOME": str(cache),
        "RUSTUP_HOME": rustup_home,
        "LC_ALL": "C",
        "LANG": "C",
        "TZ": "UTC",
        "PYTHONHASHSEED": "0",
    }


def _install_bound_name(
    bin_dir: Path, name: str, identity: dict[str, Any]
) -> None:
    target = Path(identity["resolved_path"])
    link = bin_dir / name
    try:
        link.symlink_to(target)
    except OSError as exc:
        raise EnvironmentUnavailable(
            f"cannot create bound executable link: {name}"
        ) from exc


def _write_bound_file(path: Path, data: bytes, mode: int) -> None:
    try:
        path.write_bytes(data)
        path.chmod(mode)
    except OSError as exc:
        raise EnvironmentUnavailable(f"cannot materialize bound input: {path.name}") from exc


def _environment_manifest(
    python_id: dict[str, Any],
    git_id: dict[str, Any],
    rustup_id: dict[str, Any],
    rustup_home: str,
    toolchain: str,
    cargo_id: dict[str, Any],
    rustfmt_id: dict[str, Any],
    cargo_fmt_id: dict[str, Any],
) -> dict[str, Any]:
    return {
        "schema": SCHEMA,
        "policy_revision": POLICY_REVISION,
        "adapter_implementation_commitment": IMPLEMENTATION_COMMITMENT,
        "platform_system": platform.system(),
        "platform_machine": platform.machine(),
        "python_implementation": platform.python_implementation(),
        "python_version": platform.python_version(),
        "python": python_id,
        "git": git_id,
        "rustup": rustup_id,
        "rustup_home": rustup_home,
        "toolchain": toolchain,
        "cargo": cargo_id,
        "rustfmt": rustfmt_id,
        "cargo_fmt": cargo_fmt_id,
        "path_policy": "bound-bin-only-before-rustup-toolchain-injection",
        "bound_input_policy": "adapter-owned-copy-plus-source-postflight-v1",
        "inherited_environment": ["RUSTUP_HOME-source-location-only"],
        "locale": "C",
        "timezone": "UTC",
        "python_hash_seed": "0",
        "python_isolated_mode": True,
    }


def execute_bound(
    *,
    repo: Path,
    profile: Path,
    subject: str,
    preflight_script: Path,
    timeout: int = 1200,
) -> dict[str, Any]:
    profile_binding = _bind_profile(profile)
    toolchain = profile_binding["toolchain"]
    python_id = identity_for_path("python", Path(sys.executable))
    git_id = resolve_executable("git")
    rustup_id = resolve_executable("rustup")
    rustup_home = _source_rustup_home()

    try:
        child_bytes = preflight_script.read_bytes()
    except OSError as exc:
        raise EnvironmentUnavailable(
            "cannot read preflight child implementation"
        ) from exc
    if len(child_bytes) > MAX_CHILD:
        raise EnvironmentError("preflight child implementation exceeds size bound")
    child_raw_sha256 = _sha256_bytes(child_bytes)
    child_impl_commitment = hashlib.sha256(
        CHILD_IMPL_DOMAIN + child_bytes
    ).hexdigest()

    with tempfile.TemporaryDirectory(prefix="mycelix-preflight-env-") as temporary:
        root = Path(temporary)
        bin_dir = root / "bin"
        bound_dir = root / "bound"
        bin_dir.mkdir()
        bound_dir.mkdir()
        _install_bound_name(bin_dir, "git", git_id)
        _install_bound_name(bin_dir, "rustup", rustup_id)

        bound_profile = bound_dir / "profile.json"
        bound_child = bound_dir / "preflight.py"
        _write_bound_file(bound_profile, profile_binding["raw"], 0o400)
        _write_bound_file(bound_child, child_bytes, 0o500)

        env = _child_env(root, bin_dir, rustup_home)
        cwd = root / "runner"
        cwd.mkdir()

        bound_rustup = str(bin_dir / "rustup")
        cargo_id = _rustup_which(bound_rustup, toolchain, "cargo", env, cwd)
        rustfmt_id = _rustup_which(
            bound_rustup, toolchain, "rustfmt", env, cwd
        )
        cargo_fmt_id = _rustup_which(
            bound_rustup, toolchain, "cargo-fmt", env, cwd
        )

        manifest = _environment_manifest(
            python_id,
            git_id,
            rustup_id,
            rustup_home,
            toolchain,
            cargo_id,
            rustfmt_id,
            cargo_fmt_id,
        )
        commitment = _canonical_commitment(manifest)

        for identity in (
            python_id,
            git_id,
            rustup_id,
            cargo_id,
            rustfmt_id,
            cargo_fmt_id,
        ):
            if not identity_still_matches(identity):
                raise EnvironmentError(
                    f"executable identity drift before launch: "
                    f"{identity['requested_name']}"
                )

        if not _source_bytes_still_match(
            profile, profile_binding["raw_sha256"], max_bytes=MAX_PROFILE
        ):
            raise EnvironmentError("preflight profile source drift before launch")
        if not _source_bytes_still_match(
            preflight_script, child_raw_sha256, max_bytes=MAX_CHILD
        ):
            raise EnvironmentError("preflight child source drift before launch")

        argv = [
            python_id["resolved_path"],
            "-I",
            str(bound_child),
            "--repo",
            str(Path(repo).resolve()),
            "--profile",
            str(bound_profile),
            "--subject",
            subject,
        ]
        completed = _run(argv, env, cwd, timeout)

        for identity in (
            python_id,
            git_id,
            rustup_id,
            cargo_id,
            rustfmt_id,
            cargo_fmt_id,
        ):
            if not identity_still_matches(identity):
                raise EnvironmentError(
                    f"executable identity drift after launch: "
                    f"{identity['requested_name']}"
                )

        # Stable component files are insufficient if Rustup resolver state can
        # retarget the toolchain name during the attempt.
        post_cargo_id = _rustup_which(
            bound_rustup, toolchain, "cargo", env, cwd
        )
        post_rustfmt_id = _rustup_which(
            bound_rustup, toolchain, "rustfmt", env, cwd
        )
        post_cargo_fmt_id = _rustup_which(
            bound_rustup, toolchain, "cargo-fmt", env, cwd
        )
        if (
            post_cargo_id,
            post_rustfmt_id,
            post_cargo_fmt_id,
        ) != (cargo_id, rustfmt_id, cargo_fmt_id):
            raise EnvironmentError("Rustup toolchain resolver drift after launch")

        # The child executes adapter-owned frozen copies. Persistent changes to
        # the source paths are still rejected so the result cannot be attached
        # to a path that no longer denotes the bound inputs.
        if not _source_bytes_still_match(
            profile, profile_binding["raw_sha256"], max_bytes=MAX_PROFILE
        ):
            raise EnvironmentError("preflight profile source drift after launch")
        if not _source_bytes_still_match(
            preflight_script, child_raw_sha256, max_bytes=MAX_CHILD
        ):
            raise EnvironmentError("preflight child source drift after launch")

        if _sha256_file(bound_profile) != profile_binding["raw_sha256"]:
            raise EnvironmentError("bound preflight profile mutated during launch")
        if _sha256_file(bound_child) != child_raw_sha256:
            raise EnvironmentError("bound preflight child mutated during launch")

        try:
            result = json.loads(completed.stdout.decode("utf-8", "strict"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise EnvironmentError(
                "preflight child did not emit one valid JSON result"
            ) from exc
        if not isinstance(result, dict):
            raise EnvironmentError("preflight child result is not an object")
        if result.get("schema") != CHILD_SCHEMA:
            raise EnvironmentError("preflight child schema mismatch")
        if (
            result.get("preflight_implementation_commitment")
            != child_impl_commitment
        ):
            raise EnvironmentError(
                "preflight child implementation identity mismatch"
            )
        if result.get("profile_commitment") != profile_binding["profile_commitment"]:
            raise EnvironmentError("preflight child profile identity mismatch")
        if (
            result.get("qualification_authority") is not False
            or result.get("qualification_result") is not None
        ):
            raise EnvironmentError("preflight child attempted to broaden authority")
        classification = result.get("classification")
        if classification not in CLASS_EXIT:
            raise EnvironmentError("preflight child returned unknown classification")
        if completed.returncode != CLASS_EXIT[classification]:
            raise EnvironmentError(
                "preflight child exit code/classification mismatch"
            )

        result["environment_adapter_schema"] = SCHEMA
        result["environment_adapter_implementation_commitment"] = (
            IMPLEMENTATION_COMMITMENT
        )
        result["bound_profile_raw_sha256"] = profile_binding["raw_sha256"]
        result["bound_profile_commitment"] = profile_binding["profile_commitment"]
        result["bound_preflight_raw_sha256"] = child_raw_sha256
        result["bound_preflight_implementation_commitment"] = (
            child_impl_commitment
        )
        result["preflight_environment"] = manifest
        result["preflight_environment_commitment"] = commitment
        result["child_process"] = {
            "exit_code": completed.returncode,
            "stdout_sha256": _sha256_bytes(completed.stdout),
            "stderr_sha256": _sha256_bytes(completed.stderr),
        }
        return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", default=".")
    parser.add_argument("--profile", required=True)
    parser.add_argument("--subject", required=True)
    parser.add_argument(
        "--preflight-script",
        default=str(Path(__file__).with_name("preflight.py")),
    )
    args = parser.parse_args()
    try:
        result = execute_bound(
            repo=Path(args.repo),
            profile=Path(args.profile),
            subject=args.subject,
            preflight_script=Path(args.preflight_script),
        )
    except EnvironmentUnavailable as exc:
        result = {
            "environment_adapter_schema": SCHEMA,
            "environment_adapter_implementation_commitment": (
                IMPLEMENTATION_COMMITMENT
            ),
            "classification": "UNAVAILABLE",
            "qualification_result": None,
            "qualification_authority": False,
            "reason": str(exc),
        }
    except (OSError, EnvironmentError) as exc:
        result = {
            "environment_adapter_schema": SCHEMA,
            "environment_adapter_implementation_commitment": (
                IMPLEMENTATION_COMMITMENT
            ),
            "classification": "INVALID",
            "qualification_result": None,
            "qualification_authority": False,
            "reason": str(exc),
        }
    print(
        json.dumps(
            result,
            sort_keys=True,
            separators=(",", ":"),
            ensure_ascii=False,
        )
    )
    return CLASS_EXIT.get(result["classification"], 4)


if __name__ == "__main__":
    raise SystemExit(main())
