#!/usr/bin/env python3
"""Reverify Wave 16 incident, recovery-authorization, and evidence continuity.

This post-admission gate is idempotent. It first runs the complete Wave 15
continuity gate, then independently re-verifies both incident-policy histories,
the threshold recovery authorization, final recovery capability, and exact
promotion-evidence reconstruction. Existing evidence must reproduce byte for
byte and no durable directory may change.
"""
from __future__ import annotations

import argparse
import hashlib
import os
import stat
import subprocess
import sys
from pathlib import Path

from prism_strict_json import StrictJsonError, load_path

ROOT = Path(__file__).resolve().parents[1]
MAX_CONFIG_BYTES = 128 * 1024
EXPECTED_FIELDS = {
    "schema_version",
    "wave15_continuity_config",
    "source_incident_policy_history",
    "source_incident_policy_receipt",
    "source_incident_policy_history_blake3",
    "target_incident_policy_history",
    "target_incident_policy_receipt",
    "target_incident_policy_history_blake3",
    "promotion_recovery_receipt",
    "recovery_authorization_bundle",
    "recovery_authorization_receipt",
    "authorized_recovery_receipt",
    "promotion_receipt",
    "reconstruction_directory",
}


class ContinuityGateError(RuntimeError):
    pass


def admitted_file(value: object, label: str) -> Path:
    if not isinstance(value, str):
        raise ContinuityGateError(f"{label} must be a string path")
    path = Path(value)
    if not path.is_absolute() or path.resolve(strict=True) != path:
        raise ContinuityGateError(f"{label} must be an existing canonical absolute path")
    metadata = path.lstat()
    if not stat.S_ISREG(metadata.st_mode) or stat.S_ISLNK(metadata.st_mode):
        raise ContinuityGateError(f"{label} must be a regular non-symlink file")
    if metadata.st_nlink != 1 or metadata.st_mode & 0o077 or metadata.st_size == 0:
        raise ContinuityGateError(f"{label} must be private, uniquely linked, and non-empty")
    return path


def private_directory(value: object, label: str) -> Path:
    if not isinstance(value, str):
        raise ContinuityGateError(f"{label} must be a string path")
    path = Path(value)
    if not path.is_absolute() or path.resolve(strict=True) != path:
        raise ContinuityGateError(f"{label} must be an existing canonical absolute directory")
    metadata = path.lstat()
    if not stat.S_ISDIR(metadata.st_mode) or stat.S_ISLNK(metadata.st_mode) or metadata.st_mode & 0o077:
        raise ContinuityGateError(f"{label} must be a real private directory")
    return path


def file_identity(path: Path) -> tuple[int, int, int, str]:
    metadata = path.lstat()
    return (
        metadata.st_mode & 0o777,
        metadata.st_size,
        metadata.st_ino,
        hashlib.sha256(path.read_bytes()).hexdigest(),
    )


def directory_manifest(root: Path) -> tuple[tuple[str, int, int, str], ...]:
    records: list[tuple[str, int, int, str]] = []
    for directory, names, filenames in os.walk(root, followlinks=False):
        names.sort()
        filenames.sort()
        current = Path(directory)
        for name in names:
            path = current / name
            if stat.S_ISLNK(path.lstat().st_mode):
                raise ContinuityGateError(f"continuity directory contains a symlink: {path}")
        for name in filenames:
            path = current / name
            metadata = path.lstat()
            if not stat.S_ISREG(metadata.st_mode) or stat.S_ISLNK(metadata.st_mode):
                raise ContinuityGateError(f"continuity directory contains a non-regular file: {path}")
            records.append((
                str(path.relative_to(root)),
                metadata.st_mode & 0o777,
                metadata.st_size,
                hashlib.sha256(path.read_bytes()).hexdigest(),
            ))
    return tuple(records)


def run_command(argv: list[str], updates: dict[str, str] | None = None) -> None:
    environment = dict(os.environ)
    if updates:
        environment.update(updates)
    result = subprocess.run(
        argv,
        cwd=ROOT,
        env=environment,
        stdin=subprocess.DEVNULL,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    sys.stdout.buffer.write(result.stdout)
    sys.stderr.buffer.write(result.stderr)
    if result.returncode != 0:
        raise ContinuityGateError(f"command failed with exit {result.returncode}: {' '.join(argv)}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True, type=Path)
    arguments = parser.parse_args()
    try:
        config_path = admitted_file(str(arguments.config), "Wave 16 continuity configuration")
        try:
            config = load_path(config_path, maximum_bytes=MAX_CONFIG_BYTES)
        except StrictJsonError as error:
            raise ContinuityGateError(f"invalid continuity configuration: {error}") from error
        if not isinstance(config, dict) or set(config) != EXPECTED_FIELDS or config.get("schema_version") != 1:
            raise ContinuityGateError("continuity configuration does not match the exact Wave 16 schema")

        files = {
            name: admitted_file(config[name], name.replace("_", " "))
            for name in EXPECTED_FIELDS
            if name not in {"schema_version", "reconstruction_directory", "source_incident_policy_history_blake3", "target_incident_policy_history_blake3"}
        }
        reconstruction = private_directory(config["reconstruction_directory"], "reconstruction directory")
        if len(set(files.values())) != len(files):
            raise ContinuityGateError("Wave 16 continuity artifact paths must be distinct")
        if any(path.is_relative_to(reconstruction) for path in files.values()):
            raise ContinuityGateError("input and receipt files must not be inside the reconstruction directory")

        receipt_before = {
            name: file_identity(files[name])
            for name in (
                "source_incident_policy_receipt",
    "source_incident_policy_history_blake3",
                "target_incident_policy_receipt",
    "target_incident_policy_history_blake3",
                "recovery_authorization_receipt",
                "authorized_recovery_receipt",
            )
        }
        reconstruction_before = directory_manifest(reconstruction)

        python = sys.executable
        cargo = os.environ.get("PRISM_WAVE16_CARGO") or "cargo"
        prefix = [cargo, "run", "--frozen", "-p", "prism-attestation", "--bin"]
        run_command([python, "scripts/run-wave15-continuity-gates.py", "--config", str(files["wave15_continuity_config"])])
        for role in ("source", "target"):
            history = files[f"{role}_incident_policy_history"]
            receipt = files[f"{role}_incident_policy_receipt"]
            run_command(
                prefix + ["prism-verify-incident-policy-history", "--", str(receipt)],
                {
                    "PRISM_INCIDENT_POLICY_HISTORY_PATH": str(history),
                    "PRISM_INCIDENT_POLICY_HISTORY_BLAKE3": digest_value(config[f"{role}_incident_policy_history_blake3"], f"{role} incident policy history BLAKE3"),
                },
            )
        run_command(prefix + [
            "prism-verify-promotion-recovery-authorization", "--",
            str(files["promotion_recovery_receipt"]),
            str(files["recovery_authorization_bundle"]),
            str(files["recovery_authorization_receipt"]),
        ])
        run_command(prefix + [
            "prism-finalize-promotion-recovery", "--",
            str(files["promotion_recovery_receipt"]),
            str(files["recovery_authorization_receipt"]),
            str(files["authorized_recovery_receipt"]),
        ])
        run_command(prefix + [
            "prism-reconstruct-promotion-evidence", "--",
            str(files["promotion_receipt"]),
            str(reconstruction),
        ])
        run_command(prefix + [
            "prism-verify-promotion-evidence-reconstruction", "--",
            str(files["promotion_receipt"]),
            str(reconstruction),
        ])

        for name, identity in receipt_before.items():
            if file_identity(files[name]) != identity:
                raise ContinuityGateError(f"idempotent verification mutated {name}")
        if directory_manifest(reconstruction) != reconstruction_before:
            raise ContinuityGateError("idempotent evidence reconstruction mutated durable state")
        return 0
    except (OSError, ValueError, ContinuityGateError) as error:
        print(f"Wave 16 continuity gates failed: {error}", file=sys.stderr)
        return 1


def digest_value(value: object, label: str) -> str:
    if not isinstance(value, str) or len(value) != 64:
        raise ContinuityGateError(f"{label} must be a 32-byte lowercase hex digest")
    if value.lower() != value or any(character not in "0123456789abcdef" for character in value):
        raise ContinuityGateError(f"{label} must be canonical lowercase hex")
    return value


if __name__ == "__main__":
    raise SystemExit(main())
