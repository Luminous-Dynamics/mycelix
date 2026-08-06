#!/usr/bin/env python3
"""Reverify Wave 15 restore, recovery, and promotion continuity artifacts.

This gate is intentionally idempotent: every output named by the configuration
must already exist as create-once evidence. The Rust commands recompute the
same bytes, reverify all signatures through the admitted verifier agent, and
fail if the existing evidence differs.
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
MAX_CONFIG_BYTES = 64 * 1024
EXPECTED_FIELDS = {
    "schema_version",
    "recovery_drill_plan",
    "release_journal_audit",
    "transparency_journal_audit",
    "archive_restore_receipt",
    "recovery_drill_receipt",
    "promotion_receipt",
    "promotion_head",
    "promotion_audit",
    "source_release_audit",
    "target_release_audit",
    "promotion_recovery_receipt",
    "promotion_policy_history",
    "promotion_policy_history_receipt",
    "source_channel",
    "target_channel",
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
    if metadata.st_nlink != 1 or metadata.st_mode & 0o077:
        raise ContinuityGateError(f"{label} must be private and uniquely linked")
    if metadata.st_size == 0:
        raise ContinuityGateError(f"{label} must not be empty")
    return path



def private_directory(path: Path, label: str) -> Path:
    if not path.is_absolute() or path.resolve(strict=True) != path:
        raise ContinuityGateError(f"{label} must be an existing canonical absolute directory")
    metadata = path.lstat()
    if not stat.S_ISDIR(metadata.st_mode) or stat.S_ISLNK(metadata.st_mode):
        raise ContinuityGateError(f"{label} must be a real directory")
    if metadata.st_mode & 0o077:
        raise ContinuityGateError(f"{label} must have mode 0700")
    return path


def state_manifest(root: Path) -> tuple[tuple[str, int, int, str], ...]:
    records: list[tuple[str, int, int, str]] = []
    for directory, names, filenames in os.walk(root, followlinks=False):
        names.sort()
        filenames.sort()
        current = Path(directory)
        for name in names:
            path = current / name
            metadata = path.lstat()
            if stat.S_ISLNK(metadata.st_mode):
                raise ContinuityGateError(f"promotion state contains a symlink: {path}")
        for name in filenames:
            path = current / name
            metadata = path.lstat()
            if not stat.S_ISREG(metadata.st_mode) or stat.S_ISLNK(metadata.st_mode):
                raise ContinuityGateError(f"promotion state contains a non-regular file: {path}")
            digest = hashlib.sha256(path.read_bytes()).hexdigest()
            records.append((str(path.relative_to(root)), metadata.st_mode & 0o777, metadata.st_size, digest))
    return tuple(records)

def channel(value: object, label: str) -> str:
    if (
        not isinstance(value, str)
        or not value
        or len(value) > 32
        or value.startswith("-")
        or value.endswith("-")
        or not all(character.islower() or character.isdigit() or character == "-" for character in value)
    ):
        raise ContinuityGateError(f"{label} is not canonical")
    return value


def run_command(argv: list[str]) -> None:
    result = subprocess.run(
        argv,
        cwd=ROOT,
        env=dict(os.environ),
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
        config_path = admitted_file(str(arguments.config), "continuity configuration")
        try:
            config = load_path(config_path, maximum_bytes=MAX_CONFIG_BYTES)
        except StrictJsonError as error:
            raise ContinuityGateError(f"invalid continuity configuration: {error}") from error
        if not isinstance(config, dict) or set(config) != EXPECTED_FIELDS:
            raise ContinuityGateError("continuity configuration does not match the exact schema")
        if config["schema_version"] != 1:
            raise ContinuityGateError("unsupported continuity configuration schema")

        paths = {
            name: admitted_file(config[name], name.replace("_", " "))
            for name in EXPECTED_FIELDS
            if name not in {"schema_version", "source_channel", "target_channel"}
        }
        if len(set(paths.values())) != len(paths):
            raise ContinuityGateError("continuity artifact paths must be distinct")
        source = channel(config["source_channel"], "source channel")
        target = channel(config["target_channel"], "target channel")
        if source == target:
            raise ContinuityGateError("promotion source and target channels must differ")

        state_value = os.environ.get("PRISM_PROMOTION_STATE_DIR")
        if not state_value:
            raise ContinuityGateError("PRISM_PROMOTION_STATE_DIR is required")
        state_root = private_directory(Path(state_value), "promotion state root")
        state_channel = private_directory(state_root / f"{source}-to-{target}", "promotion state channel")
        state_before = state_manifest(state_channel)

        cargo = os.environ.get("PRISM_WAVE15_CARGO") or "cargo"
        prefix = [cargo, "run", "--frozen", "-p", "prism-attestation", "--bin"]
        run_command(prefix + [
            "prism-verify-promotion-policy-history", "--",
            str(paths["promotion_policy_history"]),
            str(paths["promotion_policy_history_receipt"]),
        ])
        run_command(prefix + [
            "prism-verify-promotion-recovery", "--",
            str(paths["source_release_audit"]),
            str(paths["target_release_audit"]),
            str(paths["promotion_audit"]),
            str(paths["promotion_recovery_receipt"]),
        ])
        run_command(prefix + [
            "prism-verify-recovery-drill", "--",
            str(paths["recovery_drill_plan"]),
            str(paths["release_journal_audit"]),
            str(paths["transparency_journal_audit"]),
            str(paths["archive_restore_receipt"]),
            str(paths["recovery_drill_receipt"]),
        ])
        run_command(prefix + [
            "prism-admit-release-promotion", "--",
            str(paths["promotion_receipt"]),
            str(paths["promotion_head"]),
        ])
        run_command(prefix + [
            "prism-audit-release-promotions", "--",
            source,
            target,
            str(paths["promotion_audit"]),
        ])
        if state_manifest(state_channel) != state_before:
            raise ContinuityGateError("promotion continuity verification mutated durable state")
        return 0
    except (OSError, ValueError, ContinuityGateError) as error:
        print(f"Wave 15 continuity gates failed: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
