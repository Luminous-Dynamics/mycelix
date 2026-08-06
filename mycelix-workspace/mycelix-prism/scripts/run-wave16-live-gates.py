#!/usr/bin/env python3
"""Execute Prism Wave 16's post-admission release gates and publish exact evidence.

This is the live complement to `verify-wave16-static.py`. It is intentionally a
fixed post-admission ceremony: callers provide an already admitted release, its
external head, complete audit receipt, and independently assembled witness
bundle. The gate requires zero release-journal mutation while it re-verifies and
recovers every external artifact. Every stdout/stderr stream is
retained and hashed, the Git source identity is rechecked after the ceremony,
and publication is atomic.
"""

from __future__ import annotations

import argparse
import ctypes
import errno
import hashlib
import json
import os
import platform
import shutil
import stat
import subprocess
import sys
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

from prism_strict_json import StrictJsonError, load_path

ROOT = Path(__file__).resolve().parents[1]
SCHEMA_VERSION = 1
MAX_INPUT_BYTES = 512 * 1024 * 1024
MAX_POLICY_BYTES = 64 * 1024
AT_FDCWD = -100
RENAME_NOREPLACE = 1
REQUIRED_ENV = (
    "PRISM_RELEASE_VERIFIER_SOCKET",
    "PRISM_RELEASE_VERIFIER_UID",
    "PRISM_FLAKE_REVIEWER_KEY_ID",
    "PRISM_RELEASE_POLICY_PATH",
    "PRISM_RELEASE_POLICY_BLAKE3",
    "PRISM_RELEASE_STATE_DIR",
    "PRISM_GIT_EXECUTABLE",
    "PRISM_RELEASE_PAYLOAD_DIR",
    "PRISM_RELEASE_HEAD_BLAKE3",
    "PRISM_RELEASE_TRANSPARENCY_POLICY_PATH",
    "PRISM_RELEASE_TRANSPARENCY_POLICY_BLAKE3",
    "PRISM_RELEASE_RECOVERY_RECEIPT_OUTPUT",
    "PRISM_RELEASE_TRANSPARENCY_RECEIPT_OUTPUT",
    "PRISM_RELEASE_TRANSPARENCY_AUDIT_OUTPUT",
    "PRISM_TRANSPARENCY_JOURNAL_DIR",
    "PRISM_TRANSPARENCY_JOURNAL_AUDIT_OUTPUT",
    "PRISM_TRANSPARENCY_JOURNAL_HEAD_OUTPUT",
    "PRISM_TRANSPARENCY_JOURNAL_RECOVERY_OUTPUT",
    "PRISM_ARCHIVE_HANDOFF_VERIFICATION_OUTPUT",
    "PRISM_RELEASE_ARCHIVE_MANIFEST_BLAKE3",
    "PRISM_RELEASE_ARCHIVE_HANDOFF_BLAKE3",
    "PRISM_RELEASE_ATTESTATION_BLAKE3",
    "PRISM_RELEASE_TRANSPARENCY_RECEIPT_BLAKE3",
    "PRISM_TRANSPARENCY_RECOVERY_EXPECTED_SEQUENCE",
    "PRISM_TRANSPARENCY_RECOVERY_EXPECTED_STATEMENT_BLAKE3",
    "PRISM_TRANSPARENCY_RECOVERY_EXPECTED_CHECKPOINT_BLAKE3",
    "PRISM_TRANSPARENCY_RECOVERY_EXPECTED_AUDIT_ROOT_BLAKE3",
    "PRISM_RECOVERY_DRILL_PLAN_BLAKE3",
    "PRISM_RELEASE_JOURNAL_AUDIT_BLAKE3",
    "PRISM_TRANSPARENCY_JOURNAL_AUDIT_BLAKE3",
    "PRISM_ARCHIVE_RESTORE_RECEIPT_BLAKE3",
    "PRISM_PROMOTION_POLICY_HISTORY_PATH",
    "PRISM_PROMOTION_POLICY_HISTORY_BLAKE3",
    "PRISM_PROMOTION_POLICY_CURRENT_BLAKE3",
    "PRISM_PROMOTION_STATE_DIR",
    "PRISM_RELEASE_PROMOTION_RECEIPT_BLAKE3",
)
FORBIDDEN_ENV = (
    "RUSTFLAGS",
    "CARGO_ENCODED_RUSTFLAGS",
    "RUSTC_WRAPPER",
    "RUSTC_WORKSPACE_WRAPPER",
    "CARGO_BUILD_RUSTC",
    "CARGO_BUILD_RUSTC_WRAPPER",
    "NIX_CONFIG",
)


class LiveGateError(RuntimeError):
    pass


@dataclass(frozen=True)
class Inputs:
    rust_evidence: Path
    nix_evidence: Path
    index_receipt: Path
    flake_review: Path
    payload_manifest: Path
    attestation: Path
    head: Path
    audit_receipt: Path
    witness_bundle: Path
    archive_manifest: Path
    archive_staging: Path
    archive_handoff: Path
    continuity_config: Path
    output: Path


@dataclass(frozen=True)
class CommandSpec:
    label: str
    argv: tuple[str, ...]


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _stat_fingerprint(metadata: os.stat_result) -> tuple[int, ...]:
    return (
        metadata.st_dev,
        metadata.st_ino,
        metadata.st_mode,
        metadata.st_uid,
        metadata.st_gid,
        metadata.st_nlink,
        metadata.st_size,
        metadata.st_mtime_ns,
        metadata.st_ctime_ns,
    )


def stable_file_identity(path: Path) -> dict[str, object]:
    flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0)
    descriptor = os.open(path, flags)
    try:
        before = os.fstat(descriptor)
        if not stat.S_ISREG(before.st_mode):
            raise LiveGateError(f"live-gate artifact is not a regular file: {path}")
        digest = hashlib.sha256()
        byte_count = 0
        while True:
            chunk = os.read(descriptor, 1024 * 1024)
            if not chunk:
                break
            byte_count += len(chunk)
            digest.update(chunk)
        after = os.fstat(descriptor)
        if _stat_fingerprint(before) != _stat_fingerprint(after):
            raise LiveGateError(f"live-gate artifact changed while being read: {path}")
    finally:
        os.close(descriptor)
    path_after = path.lstat()
    if path_after.st_dev != after.st_dev or path_after.st_ino != after.st_ino:
        raise LiveGateError(f"live-gate artifact path changed while being read: {path}")
    return {"path": str(path), "bytes": byte_count, "sha256": digest.hexdigest()}


def sha256_file(path: Path) -> str:
    return str(stable_file_identity(path)["sha256"])



def require_tool(name: str) -> str:
    resolved = shutil.which(name)
    if resolved is None:
        raise LiveGateError(f"required live-gate command is unavailable: {name}")
    return str(Path(resolved).resolve())


def run(argv: Sequence[str], env: dict[str, str]) -> subprocess.CompletedProcess[bytes]:
    return subprocess.run(
        list(argv),
        cwd=ROOT,
        env=env,
        stdin=subprocess.DEVNULL,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )


def hardened_git_environment() -> dict[str, str]:
    return {
        "HOME": "/nonexistent",
        "PATH": "/usr/bin:/bin",
        "TZ": "UTC",
        "LC_ALL": "C.UTF-8",
        "LANG": "C.UTF-8",
        "GIT_CONFIG_NOSYSTEM": "1",
        "GIT_CONFIG_GLOBAL": "/dev/null",
        "GIT_CONFIG_SYSTEM": "/dev/null",
        "GIT_OPTIONAL_LOCKS": "0",
        "GIT_NO_REPLACE_OBJECTS": "1",
        "GIT_TERMINAL_PROMPT": "0",
        "GIT_ALLOW_PROTOCOL": "file",
    }


def git_text(git: str, *arguments: str) -> str:
    result = subprocess.run(
        [
            git,
            "-c",
            "core.hooksPath=/dev/null",
            "-c",
            "core.fsmonitor=false",
            "-c",
            "credential.helper=",
            *arguments,
        ],
        cwd=ROOT,
        env=hardened_git_environment(),
        stdin=subprocess.DEVNULL,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    if result.returncode != 0:
        raise LiveGateError(result.stderr.decode("utf-8", "replace").strip())
    return result.stdout.decode("utf-8", "strict").strip()


def require_clean_source(git: str) -> None:
    status = git_text(git, "status", "--porcelain=v1", "--untracked-files=all")
    if status:
        raise LiveGateError(f"Wave 16 live gates require a clean source tree:\n{status}")


def admitted_path(value: str, *, directory: bool) -> Path:
    path = Path(value)
    if not path.is_absolute():
        raise LiveGateError(f"live-gate artifact path must be absolute: {path}")
    metadata = path.lstat()
    if stat.S_ISLNK(metadata.st_mode):
        raise LiveGateError(f"live-gate artifact must not be a symlink: {path}")
    if directory != path.is_dir():
        expected = "directory" if directory else "file"
        raise LiveGateError(f"live-gate artifact must be a {expected}: {path}")
    if os.name == "posix" and metadata.st_mode & 0o022:
        raise LiveGateError(f"live-gate artifact is writable by group or others: {path}")
    if not directory and getattr(metadata, "st_nlink", 1) != 1:
        raise LiveGateError(f"live-gate artifact is unexpectedly hard-linked: {path}")
    if not directory and (metadata.st_size == 0 or metadata.st_size > MAX_INPUT_BYTES):
        raise LiveGateError(f"live-gate artifact size is inadmissible: {path}")
    resolved = path.resolve()
    if resolved != path:
        raise LiveGateError(f"live-gate artifact path is not canonical: {path}")
    return resolved


def paths_overlap(left: Path, right: Path) -> bool:
    return left == right or left in right.parents or right in left.parents


def require_non_overlapping(paths: Sequence[tuple[str, Path]]) -> None:
    for index, (left_label, left) in enumerate(paths):
        for right_label, right in paths[index + 1 :]:
            if paths_overlap(left, right):
                raise LiveGateError(
                    f"live-gate paths overlap: {left_label}={left} and {right_label}={right}"
                )


def private_directory(path: Path, label: str) -> Path:
    admitted = admitted_path(str(path.resolve(strict=True)), directory=True)
    if os.name == "posix":
        metadata = admitted.stat()
        if metadata.st_uid != os.geteuid() or metadata.st_mode & 0o077:
            raise LiveGateError(
                f"{label} must be caller-owned with mode 0700: {admitted}"
            )
    return admitted


def private_output_parent(output: Path) -> Path:
    return private_directory(output.parent, "live-gate output parent")


def private_create_once_path(value: str, label: str) -> Path:
    path = Path(value)
    if not path.is_absolute():
        raise LiveGateError(f"{label} must be an absolute path")
    if path.exists() or path.is_symlink():
        raise LiveGateError(f"{label} already exists: {path}")
    parent = private_directory(path.parent, f"{label} parent")
    candidate = parent / path.name
    if candidate != path:
        raise LiveGateError(f"{label} path is not canonical: {path}")
    return path


def parse_inputs(arguments: argparse.Namespace) -> Inputs:
    output = Path(arguments.output)
    if not output.is_absolute():
        raise LiveGateError("live-gate output path must be absolute")
    inputs = Inputs(
        rust_evidence=admitted_path(arguments.rust_evidence, directory=True),
        nix_evidence=admitted_path(arguments.nix_evidence, directory=True),
        index_receipt=admitted_path(arguments.index_receipt, directory=False),
        flake_review=admitted_path(arguments.flake_review, directory=False),
        payload_manifest=admitted_path(arguments.payload_manifest, directory=False),
        attestation=admitted_path(arguments.attestation, directory=False),
        head=admitted_path(arguments.head, directory=False),
        audit_receipt=admitted_path(arguments.audit_receipt, directory=False),
        witness_bundle=admitted_path(arguments.witness_bundle, directory=False),
        archive_manifest=admitted_path(arguments.archive_manifest, directory=False),
        archive_staging=admitted_path(arguments.archive_staging, directory=True),
        archive_handoff=admitted_path(arguments.archive_handoff, directory=False),
        continuity_config=admitted_path(arguments.continuity_config, directory=False),
        output=output,
    )
    require_non_overlapping(
        (
            ("rust evidence", inputs.rust_evidence),
            ("Nix evidence", inputs.nix_evidence),
            ("index receipt", inputs.index_receipt),
            ("flake review", inputs.flake_review),
            ("payload manifest", inputs.payload_manifest),
            ("release attestation", inputs.attestation),
            ("release head", inputs.head),
            ("release audit receipt", inputs.audit_receipt),
            ("witness bundle", inputs.witness_bundle),
            ("archive manifest", inputs.archive_manifest),
            ("archive staging", inputs.archive_staging),
            ("archive handoff", inputs.archive_handoff),
            ("Wave 16 continuity config", inputs.continuity_config),
            ("live output", inputs.output.resolve(strict=False)),
        )
    )
    return inputs


def canonical_environment(git: str) -> dict[str, str]:
    missing = [name for name in REQUIRED_ENV if not os.environ.get(name)]
    if missing:
        raise LiveGateError("missing live release environment: " + ", ".join(missing))
    forbidden = [name for name in FORBIDDEN_ENV if os.environ.get(name)]
    if forbidden:
        raise LiveGateError("forbidden build environment overrides: " + ", ".join(forbidden))
    environment = dict(os.environ)
    for name in FORBIDDEN_ENV:
        environment.pop(name, None)
    environment.update(
        {
            "CARGO_INCREMENTAL": "0",
            "SOURCE_DATE_EPOCH": git_text(git, "show", "-s", "--format=%ct", "HEAD"),
            "TZ": "UTC",
            "LC_ALL": "C.UTF-8",
            "LANG": "C.UTF-8",
        }
    )
    return environment



def tool_identity(path: str, environment: dict[str, str]) -> dict[str, str]:
    executable = Path(path)
    before = sha256_file(executable)
    result = run((path, "--version"), environment)
    if result.returncode != 0:
        raise LiveGateError(f"could not read tool version: {path}")
    version = (result.stdout + result.stderr).decode("utf-8", "replace").strip()
    return {"path": path, "sha256": before, "version": version}


def remeasure_tools(tools: dict[str, dict[str, str]]) -> None:
    for name, identity in tools.items():
        if sha256_file(Path(identity["path"])) != identity["sha256"]:
            raise LiveGateError(f"tool executable changed during live gates: {name}")

def namespace_probe(bwrap: str, python: str) -> CommandSpec:
    probe = (
        "import socket,sys; "
        "s=socket.socket(); s.settimeout(1.0); "
        "\ntry: s.connect(('1.1.1.1',443))\n"
        "except OSError: sys.exit(0)\n"
        "else: sys.exit(71)"
    )
    return CommandSpec(
        "kernel-netns-no-direct-egress",
        (
            bwrap,
            "--die-with-parent",
            "--new-session",
            "--unshare-user",
            "--unshare-pid",
            "--unshare-net",
            "--ro-bind",
            "/",
            "/",
            "--proc",
            "/proc",
            "--dev",
            "/dev",
            "--",
            python,
            "-c",
            probe,
        ),
    )


def command_specs(
    inputs: Inputs,
    bwrap: str,
    python: str,
    cargo: str,
    nix: str,
    channel: str,
    head_output: Path,
    audit_output: Path,
    recovery_output: Path,
    transparency_output: Path,
    transparency_audit_output: Path,
    transparency_journal_audit_output: Path,
    transparency_journal_head_output: Path,
    transparency_journal_recovery_output: Path,
    archive_verification_output: Path,
) -> tuple[CommandSpec, ...]:
    return (
        CommandSpec(
            "wave16-static",
            (python, "scripts/verify-wave16-static.py", "--check", "security/wave16-static-evidence.json"),
        ),
        CommandSpec(
            "supply-chain-release",
            (python, "scripts/verify-supply-chain.py", "--release"),
        ),
        CommandSpec(
            "rust-evidence",
            (
                python,
                "scripts/verify-build-evidence.py",
                str(inputs.rust_evidence),
                "--lane",
                "rust",
                "--release",
                "--current-source",
            ),
        ),
        CommandSpec(
            "nix-evidence",
            (
                python,
                "scripts/verify-build-evidence.py",
                str(inputs.nix_evidence),
                "--lane",
                "nix",
                "--release",
                "--current-source",
            ),
        ),
        CommandSpec(
            "adversarial-rust-tests",
            (
                cargo,
                "test",
                "--frozen",
                "-p",
                "prism-egress",
                "-p",
                "prism-compat",
                "-p",
                "prism-security-tests",
                "-p",
                "prism-attestation",
                "--all-targets",
            ),
        ),
        namespace_probe(bwrap, python),
        CommandSpec(
            "nix-lock-immutability",
            (
                nix,
                "--extra-experimental-features",
                "nix-command flakes",
                "flake",
                "metadata",
                "--json",
                "--no-update-lock-file",
            ),
        ),
        CommandSpec(
            "nix-closure",
            (
                nix,
                "--extra-experimental-features",
                "nix-command flakes",
                "flake",
                "check",
                "--no-update-lock-file",
                "--print-build-logs",
            ),
        ),
        CommandSpec(
            "signed-flake-review",
            (
                cargo,
                "run",
                "--frozen",
                "-p",
                "prism-attestation",
                "--bin",
                "prism-verify-flake-review",
                "--",
                str(inputs.flake_review),
                str((ROOT / "flake.lock").resolve()),
            ),
        ),
        CommandSpec(
            "signed-release",
            (
                cargo,
                "run",
                "--frozen",
                "-p",
                "prism-attestation",
                "--bin",
                "prism-verify-release",
                "--",
                str(inputs.attestation),
                str((ROOT / "Cargo.lock").resolve()),
                str((ROOT / "flake.lock").resolve()),
                str(inputs.flake_review),
                str(inputs.rust_evidence),
                str(inputs.nix_evidence),
                str(inputs.index_receipt),
                str(inputs.payload_manifest),
            ),
        ),
        CommandSpec(
            "release-journal-audit",
            (
                cargo,
                "run",
                "--frozen",
                "-p",
                "prism-attestation",
                "--bin",
                "prism-audit-release-journal",
                "--",
                channel,
                str(head_output),
                str(audit_output),
            ),
        ),
        CommandSpec(
            "release-state-recovery",
            (
                cargo,
                "run",
                "--frozen",
                "-p",
                "prism-attestation",
                "--bin",
                "prism-recover-release-state",
                "--",
                channel,
                str(head_output),
                str(audit_output),
                str(recovery_output),
            ),
        ),
        CommandSpec(
            "transparency-witness-quorum",
            (
                cargo,
                "run",
                "--frozen",
                "-p",
                "prism-attestation",
                "--bin",
                "prism-verify-witness-quorum",
                "--",
                str(head_output),
                str(audit_output),
                str(inputs.witness_bundle),
                str(transparency_output),
                str(transparency_audit_output),
            ),
        ),
        CommandSpec(
            "transparency-journal-admission",
            (
                cargo, "run", "--frozen", "-p", "prism-attestation", "--bin",
                "prism-admit-transparency-quorum", "--", channel,
                str(transparency_output),
            ),
        ),
        CommandSpec(
            "transparency-journal-audit",
            (
                cargo, "run", "--frozen", "-p", "prism-attestation", "--bin",
                "prism-audit-transparency-journal", "--", channel,
                str(transparency_journal_audit_output),
            ),
        ),
        CommandSpec(
            "transparency-journal-recovery",
            (
                cargo, "run", "--frozen", "-p", "prism-attestation", "--bin",
                "prism-recover-transparency-journal", "--", channel,
                str(transparency_journal_head_output),
                str(transparency_journal_audit_output),
                str(transparency_journal_recovery_output),
            ),
        ),
        CommandSpec(
            "wave16-continuity",
            (python, "scripts/run-wave16-continuity-gates.py", "--config", str(inputs.continuity_config)),
        ),
        CommandSpec(
            "signed-archive-handoff",
            (
                cargo, "run", "--frozen", "-p", "prism-attestation", "--bin",
                "prism-verify-archive-handoff", "--",
                str(inputs.archive_manifest),
                str(inputs.archive_staging),
                str(inputs.archive_handoff),
                str(inputs.attestation),
                str(archive_verification_output),
            ),
        ),
    )


def execute(specs: Sequence[CommandSpec], environment: dict[str, str], staging: Path) -> list[dict[str, object]]:
    logs = staging / "logs"
    logs.mkdir(parents=True)
    records: list[dict[str, object]] = []
    for index, spec in enumerate(specs, start=1):
        started = time.monotonic_ns()
        result = run(spec.argv, environment)
        elapsed_ms = (time.monotonic_ns() - started) // 1_000_000
        stdout_name = f"{index:02d}-{spec.label}.stdout"
        stderr_name = f"{index:02d}-{spec.label}.stderr"
        (logs / stdout_name).write_bytes(result.stdout)
        (logs / stderr_name).write_bytes(result.stderr)
        record = {
            "label": spec.label,
            "argv": list(spec.argv),
            "exit_code": result.returncode,
            "elapsed_ms": elapsed_ms,
            "stdout": {"file": f"logs/{stdout_name}", "bytes": len(result.stdout), "sha256": sha256_bytes(result.stdout)},
            "stderr": {"file": f"logs/{stderr_name}", "bytes": len(result.stderr), "sha256": sha256_bytes(result.stderr)},
        }
        records.append(record)
        if result.returncode != 0:
            raise LiveGateError(f"live gate failed: {spec.label} (exit {result.returncode})")
    return records


def file_identity(path: Path) -> dict[str, object]:
    return stable_file_identity(path)


def load_release_channel(policy_path: Path) -> str:
    try:
        policy = load_path(policy_path, maximum_bytes=MAX_POLICY_BYTES)
    except StrictJsonError as error:
        raise LiveGateError(f"invalid release policy JSON: {error}") from error
    expected = {
        "schema_version",
        "policy_id",
        "project",
        "channel",
        "signer_key_id",
        "classical_verification_key_blake3",
        "post_quantum_verification_key_blake3",
        "minimum_sequence",
        "previous_statement_blake3",
        "version_prefix",
    }
    if not isinstance(policy, dict) or set(policy) != expected:
        raise LiveGateError("release policy does not match the exact schema")
    channel = policy.get("channel")
    if (
        not isinstance(channel, str)
        or not channel
        or len(channel) > 32
        or not all(character.islower() or character.isdigit() or character == "-" for character in channel)
        or channel.startswith("-")
        or channel.endswith("-")
    ):
        raise LiveGateError("release policy channel is not canonical")
    return channel


def state_manifest(channel_root: Path) -> list[dict[str, object]]:
    if not channel_root.is_absolute() or not channel_root.is_dir():
        raise LiveGateError("release state channel must be an absolute directory")
    records: list[dict[str, object]] = []
    for path in sorted(channel_root.iterdir()):
        relative = path.name
        metadata = path.lstat()
        if stat.S_ISLNK(metadata.st_mode):
            raise LiveGateError(f"release state contains a symlink: {relative}")
        if relative == ".lock":
            if not path.is_file() or metadata.st_nlink != 1 or metadata.st_mode & 0o077:
                raise LiveGateError("release state lock is not private and unique")
            continue
        if (
            not path.is_file()
            or metadata.st_nlink != 1
            or metadata.st_mode & 0o077
            or len(relative) != 25
            or not relative.endswith(".json")
            or not relative[:20].isdigit()
        ):
            raise LiveGateError(f"unexpected release state entry: {relative}")
        identity = stable_file_identity(path)
        records.append(
            {
                "path": relative,
                "bytes": identity["bytes"],
                "sha256": identity["sha256"],
            }
        )
    return records


def require_no_checkpoint_change(
    before: Sequence[dict[str, object]], after: Sequence[dict[str, object]]
) -> None:
    if list(before) != list(after):
        raise LiveGateError(
            "Wave 16 post-admission verification changed the release journal"
        )


def load_head_identity(path: Path) -> tuple[str, str, str]:
    try:
        head = load_path(path, maximum_bytes=MAX_POLICY_BYTES)
    except StrictJsonError as error:
        raise LiveGateError(f"invalid release head JSON: {error}") from error
    expected = {
        "schema_version",
        "channel",
        "sequence",
        "statement_blake3",
        "checkpoint_blake3",
    }
    if not isinstance(head, dict) or set(head) != expected:
        raise LiveGateError("release head does not match the exact schema")
    statement = head.get("statement_blake3")
    checkpoint = head.get("checkpoint_blake3")
    channel = head.get("channel")
    for label, value in (("statement", statement), ("checkpoint", checkpoint)):
        if (
            not isinstance(value, list)
            or len(value) != 32
            or not all(isinstance(byte, int) and 0 <= byte <= 255 for byte in value)
            or all(byte == 0 for byte in value)
        ):
            raise LiveGateError(f"release head {label} digest is invalid")
    if not isinstance(channel, str) or not channel:
        raise LiveGateError("release head channel is invalid")
    return (
        channel,
        bytes(statement).hex(),
        bytes(checkpoint).hex(),
    )


def rename_noreplace(staging: Path, output: Path) -> None:
    if os.name != "posix":
        raise LiveGateError("Wave 16 live publication requires Linux renameat2")
    libc = ctypes.CDLL(None, use_errno=True)
    renameat2 = getattr(libc, "renameat2", None)
    if renameat2 is None:
        raise LiveGateError("renameat2(RENAME_NOREPLACE) is unavailable")
    renameat2.argtypes = [ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_uint]
    renameat2.restype = ctypes.c_int
    result = renameat2(
        AT_FDCWD,
        os.fsencode(staging),
        AT_FDCWD,
        os.fsencode(output),
        RENAME_NOREPLACE,
    )
    if result != 0:
        error = ctypes.get_errno()
        if error == errno.EEXIST:
            raise LiveGateError(f"live-gate output already exists: {output}")
        raise LiveGateError(f"could not publish live-gate evidence: {os.strerror(error)}")


def publish(staging: Path, output: Path) -> None:
    rename_noreplace(staging, output)
    directory = os.open(output.parent, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
    try:
        os.fsync(directory)
    finally:
        os.close(directory)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--rust-evidence", required=True)
    parser.add_argument("--nix-evidence", required=True)
    parser.add_argument("--index-receipt", required=True)
    parser.add_argument("--flake-review", required=True)
    parser.add_argument("--payload-manifest", required=True)
    parser.add_argument("--attestation", required=True)
    parser.add_argument("--head", required=True)
    parser.add_argument("--audit-receipt", required=True)
    parser.add_argument("--witness-bundle", required=True)
    parser.add_argument("--archive-manifest", required=True)
    parser.add_argument("--archive-staging", required=True)
    parser.add_argument("--archive-handoff", required=True)
    parser.add_argument("--continuity-config", required=True)
    parser.add_argument("--output", required=True)
    arguments = parser.parse_args()
    try:
        configured_git = os.environ.get("PRISM_GIT_EXECUTABLE")
        if not configured_git or not Path(configured_git).is_absolute():
            raise LiveGateError("PRISM_GIT_EXECUTABLE must name an absolute Git executable")
        git = str(admitted_path(configured_git, directory=False))
        cargo = require_tool("cargo")
        nix = require_tool("nix")
        python = require_tool("python3")
        bwrap = os.environ.get("PRISM_COMPAT_BWRAP_PATH") or require_tool("bwrap")
        bwrap = str(Path(bwrap).resolve())
        inputs = parse_inputs(arguments)
        if not (ROOT / "flake.lock").is_file():
            raise LiveGateError("Wave 16 live gates require a committed flake.lock")
        require_clean_source(git)
        source_before = {
            "commit": git_text(git, "rev-parse", "HEAD"),
            "tree": git_text(git, "rev-parse", "HEAD^{tree}"),
            "status": "clean",
        }
        environment = canonical_environment(git)
        environment["PRISM_WAVE14_CARGO"] = cargo
        release_policy_path = admitted_path(
            environment["PRISM_RELEASE_POLICY_PATH"], directory=False
        )
        release_state_root = private_directory(
            Path(environment["PRISM_RELEASE_STATE_DIR"]), "release state root"
        )
        payload_root = admitted_path(
            environment["PRISM_RELEASE_PAYLOAD_DIR"], directory=True
        )
        channel = load_release_channel(release_policy_path)
        head_output = inputs.head
        audit_output = inputs.audit_receipt
        head_channel, expected_statement, expected_checkpoint = load_head_identity(head_output)
        if head_channel != channel:
            raise LiveGateError("release head and release policy channels differ")
        environment["PRISM_RELEASE_HEAD_OUTPUT"] = str(head_output)
        environment["PRISM_RELEASE_RECOVERY_EXPECTED_STATEMENT_BLAKE3"] = expected_statement
        environment["PRISM_RELEASE_RECOVERY_EXPECTED_CHECKPOINT_BLAKE3"] = expected_checkpoint
        recovery_output = private_create_once_path(
            environment["PRISM_RELEASE_RECOVERY_RECEIPT_OUTPUT"],
            "release recovery receipt output",
        )
        transparency_output = private_create_once_path(
            environment["PRISM_RELEASE_TRANSPARENCY_RECEIPT_OUTPUT"],
            "release transparency receipt output",
        )
        transparency_audit_output = private_create_once_path(
            environment["PRISM_RELEASE_TRANSPARENCY_AUDIT_OUTPUT"],
            "release transparency audit output",
        )
        transparency_journal_audit_output = private_create_once_path(
            environment["PRISM_TRANSPARENCY_JOURNAL_AUDIT_OUTPUT"],
            "transparency journal audit output",
        )
        transparency_journal_head_output = private_create_once_path(
            environment["PRISM_TRANSPARENCY_JOURNAL_HEAD_OUTPUT"],
            "transparency journal head output",
        )
        transparency_journal_recovery_output = private_create_once_path(
            environment["PRISM_TRANSPARENCY_JOURNAL_RECOVERY_OUTPUT"],
            "transparency journal recovery output",
        )
        archive_verification_output = private_create_once_path(
            environment["PRISM_ARCHIVE_HANDOFF_VERIFICATION_OUTPUT"],
            "archive handoff verification output",
        )
        transparency_journal_root = private_directory(
            Path(environment["PRISM_TRANSPARENCY_JOURNAL_DIR"]),
            "transparency journal root",
        )
        transparency_journal_channel = private_directory(
            transparency_journal_root / channel,
            "transparency journal channel",
        )
        transparency_policy_path = admitted_path(
            environment["PRISM_RELEASE_TRANSPARENCY_POLICY_PATH"], directory=False
        )
        release_state_channel = private_directory(
            release_state_root / channel, "release state channel"
        )
        require_non_overlapping(
            (
                ("repository", ROOT.resolve()),
                ("release policy", release_policy_path),
                ("release state", release_state_root),
                ("release head output", head_output),
                ("release audit output", audit_output),
                ("release recovery output", recovery_output),
                ("release transparency output", transparency_output),
                ("release transparency audit output", transparency_audit_output),
                ("transparency journal", transparency_journal_root),
                ("transparency journal audit output", transparency_journal_audit_output),
                ("transparency journal head output", transparency_journal_head_output),
                ("transparency journal recovery output", transparency_journal_recovery_output),
                ("archive verification output", archive_verification_output),
                ("release transparency policy", transparency_policy_path),
                ("witness bundle", inputs.witness_bundle),
                ("archive manifest", inputs.archive_manifest),
                ("archive staging", inputs.archive_staging),
                ("archive handoff", inputs.archive_handoff),
                ("Wave 16 continuity config", inputs.continuity_config),
                ("rust evidence", inputs.rust_evidence),
                ("Nix evidence", inputs.nix_evidence),
                ("index receipt", inputs.index_receipt),
                ("flake review", inputs.flake_review),
                ("payload manifest", inputs.payload_manifest),
                ("payload root", payload_root),
                ("release attestation", inputs.attestation),
                ("live output", inputs.output.resolve(strict=False)),
            )
        )
        state_before = state_manifest(release_state_channel)
        transparency_state_before = state_manifest(transparency_journal_channel)
        tools = {
            "git": tool_identity(git, environment),
            "cargo": tool_identity(cargo, environment),
            "nix": tool_identity(nix, environment),
            "python3": tool_identity(python, environment),
            "bwrap": tool_identity(bwrap, environment),
        }
        output_parent = private_output_parent(inputs.output)
        staging = Path(tempfile.mkdtemp(prefix=".prism-wave16-live-", dir=output_parent))
        try:
            commands = execute(
                command_specs(
                    inputs, bwrap, python, cargo, nix, channel, head_output, audit_output,
                    recovery_output, transparency_output, transparency_audit_output,
                    transparency_journal_audit_output, transparency_journal_head_output,
                    transparency_journal_recovery_output, archive_verification_output
                ),
                environment,
                staging,
            )
            remeasure_tools(tools)
            require_clean_source(git)
            source_after = {
                "commit": git_text(git, "rev-parse", "HEAD"),
                "tree": git_text(git, "rev-parse", "HEAD^{tree}"),
                "status": "clean",
            }
            if source_after != source_before:
                raise LiveGateError("source identity changed during Wave 16 live gates")
            state_after = state_manifest(release_state_channel)
            require_no_checkpoint_change(state_before, state_after)
            transparency_state_after = state_manifest(transparency_journal_channel)
            require_no_checkpoint_change(transparency_state_before, transparency_state_after)
            head_identity = file_identity(admitted_path(str(head_output), directory=False))
            audit_identity = file_identity(admitted_path(str(audit_output), directory=False))
            recovery_identity = file_identity(admitted_path(str(recovery_output), directory=False))
            transparency_identity = file_identity(
                admitted_path(str(transparency_output), directory=False)
            )
            transparency_audit_identity = file_identity(
                admitted_path(str(transparency_audit_output), directory=False)
            )
            transparency_journal_audit_identity = file_identity(
                admitted_path(str(transparency_journal_audit_output), directory=False)
            )
            transparency_journal_head_identity = file_identity(
                admitted_path(str(transparency_journal_head_output), directory=False)
            )
            transparency_journal_recovery_identity = file_identity(
                admitted_path(str(transparency_journal_recovery_output), directory=False)
            )
            archive_verification_identity = file_identity(
                admitted_path(str(archive_verification_output), directory=False)
            )
            receipt = {
                "schema_version": SCHEMA_VERSION,
                "campaign": "prism-hardening-wave-14-live",
                "status": "pass",
                "platform": {"system": platform.system(), "release": platform.release(), "machine": platform.machine()},
                "source": source_before,
                "tools": tools,
                "artifacts": {
                    "Cargo.lock": file_identity((ROOT / "Cargo.lock").resolve()),
                    "flake.lock": file_identity((ROOT / "flake.lock").resolve()),
                    "index_receipt": file_identity(inputs.index_receipt),
                    "flake_review": file_identity(inputs.flake_review),
                    "payload_manifest": file_identity(inputs.payload_manifest),
                    "attestation": file_identity(inputs.attestation),
                    "release_policy": file_identity(release_policy_path),
                    "release_head": head_identity,
                    "release_journal_audit": audit_identity,
                    "release_recovery": recovery_identity,
                    "transparency_policy": file_identity(transparency_policy_path),
                    "witness_bundle": file_identity(inputs.witness_bundle),
                    "transparency_quorum": transparency_identity,
                    "transparency_quorum_audit": transparency_audit_identity,
                    "transparency_journal_audit": transparency_journal_audit_identity,
                    "transparency_journal_head": transparency_journal_head_identity,
                    "transparency_journal_recovery": transparency_journal_recovery_identity,
                    "archive_manifest": file_identity(inputs.archive_manifest),
                    "archive_handoff": file_identity(inputs.archive_handoff),
                    "wave16_continuity_config": file_identity(inputs.continuity_config),
                    "archive_handoff_verification": archive_verification_identity,
                },
                "release_state": {
                    "root": str(release_state_root),
                    "channel": channel,
                    "channel_root": str(release_state_channel),
                    "before": state_before,
                    "after": state_after,
                    "transparency_journal_root": str(transparency_journal_root),
                    "transparency_journal_before": transparency_state_before,
                    "transparency_journal_after": transparency_state_after,
                },
                "evidence_directories": {
                    "rust": str(inputs.rust_evidence),
                    "nix": str(inputs.nix_evidence),
                    "payloads": str(payload_root),
                    "archive_staging": str(inputs.archive_staging),
                },
                "commands": commands,
            }
            encoded = (json.dumps(receipt, indent=2, sort_keys=True) + "\n").encode()
            (staging / "receipt.json").write_bytes(encoded)
            with (staging / "receipt.json").open("rb") as handle:
                os.fsync(handle.fileno())
            publish(staging, inputs.output)
        except Exception:
            shutil.rmtree(staging, ignore_errors=True)
            raise
        print(inputs.output / "receipt.json")
        return 0
    except (OSError, ValueError, LiveGateError) as error:
        print(f"Wave 16 live gates failed: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
