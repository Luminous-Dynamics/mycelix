#!/usr/bin/env python3
"""Canonical retained-evidence manifests for real-conductor scenarios.

The manifest is tamper-evident, path-confined, size-bounded, and bound to the
exact scenario contract. It does not prove that a claimed conductor was real;
that remains an execution-environment trust and release-attestation question.
"""

from __future__ import annotations

import base64
import binascii
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
from typing import Any

MANIFEST_SCHEMA = "mycelix-conductor-evidence-manifest-v1"
MAX_MANIFEST_BYTES = 512 * 1024
MAX_EVIDENCE_FILES = 512
MAX_EVIDENCE_FILE_BYTES = 64 * 1024 * 1024
MAX_EVIDENCE_TOTAL_BYTES = 256 * 1024 * 1024
REQUIRED_CONDUCTORS = {"verifier", "prover_a", "prover_b"}


class EvidenceError(RuntimeError):
    """A retained-evidence validation failure."""


def require(condition: bool, message: str) -> None:
    if not condition:
        raise EvidenceError(message)


def canonical_json_bytes(value: Any) -> bytes:
    return json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=False,
        allow_nan=False,
    ).encode("utf-8")


def sha256_b64(data: bytes) -> str:
    return base64.b64encode(hashlib.sha256(data).digest()).decode("ascii")


def file_sha256_b64(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        while chunk := handle.read(1024 * 1024):
            digest.update(chunk)
    return base64.b64encode(digest.digest()).decode("ascii")


def require_hash32(value: Any, name: str) -> str:
    require(isinstance(value, str) and bool(value), f"{name} must be non-empty text")
    try:
        decoded = base64.b64decode(value, validate=True)
    except (ValueError, binascii.Error) as error:
        raise EvidenceError(f"{name} must be canonical standard base64") from error
    require(len(decoded) == 32, f"{name} must decode to 32 bytes")
    require(any(decoded), f"{name} must not be the all-zero hash")
    require(base64.b64encode(decoded).decode("ascii") == value, f"{name} is not canonical base64")
    return value


def require_text(value: Any, name: str, maximum: int = 1024) -> str:
    require(isinstance(value, str) and bool(value), f"{name} must be non-empty text")
    require(len(value.encode("utf-8")) <= maximum, f"{name} exceeds {maximum} UTF-8 bytes")
    return value


def confined_relative_path(value: Any, name: str) -> PurePosixPath:
    text = require_text(value, name, 4096)
    path = PurePosixPath(text)
    require(not path.is_absolute(), f"{name} must be relative")
    require(".." not in path.parts, f"{name} may not contain '..'")
    require("." not in path.parts, f"{name} may not contain '.' components")
    require(str(path) == text, f"{name} must use normalized POSIX separators")
    require(bool(path.parts), f"{name} must not be empty")
    return path


def resolve_confined(root: Path, relative: PurePosixPath, name: str) -> Path:
    root = root.resolve(strict=True)
    candidate = root.joinpath(*relative.parts)
    # lstat every component so a symlink cannot escape after the lexical check.
    current = root
    for part in relative.parts:
        current = current / part
        require(current.exists(), f"{name} does not exist: {relative}")
        require(not current.is_symlink(), f"{name} may not traverse symlinks: {relative}")
    resolved = candidate.resolve(strict=True)
    require(resolved.is_relative_to(root), f"{name} escapes evidence root")
    return resolved


def load_json_object(path: Path, maximum_bytes: int, description: str) -> dict[str, Any]:
    size = path.stat().st_size
    require(size <= maximum_bytes, f"{description} exceeds {maximum_bytes} bytes")
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as error:
        raise EvidenceError(f"invalid {description}: {error}") from error
    require(isinstance(value, dict), f"{description} must be a JSON object")
    return value


def manifest_hash(manifest: dict[str, Any]) -> str:
    return sha256_b64(canonical_json_bytes(manifest))


def contract_hash(contract_path: Path) -> str:
    return file_sha256_b64(contract_path)


def validate_manifest(
    manifest: dict[str, Any],
    manifest_dir: Path,
    *,
    expected_contract_hash: str,
    expected_scenario_id: str,
    expected_state: str,
    expected_dna_hash: str | None = None,
) -> dict[str, Any]:
    require(manifest.get("schema") == MANIFEST_SCHEMA, "unsupported evidence manifest schema")
    require_hash32(manifest.get("scenario_contract_hash"), "scenario_contract_hash")
    require(
        manifest["scenario_contract_hash"] == expected_contract_hash,
        "evidence manifest is bound to a different scenario contract",
    )
    require(
        require_text(manifest.get("scenario_id"), "scenario_id") == expected_scenario_id,
        "evidence manifest scenario_id mismatch",
    )
    require(
        require_text(manifest.get("expected_state"), "expected_state") == expected_state,
        "evidence manifest expected_state mismatch",
    )
    require(
        require_text(manifest.get("observed_state"), "observed_state") == expected_state,
        "evidence manifest did not observe the required state",
    )
    dna_hash = require_text(manifest.get("dna_hash"), "dna_hash", 4096)
    if expected_dna_hash is not None:
        require(dna_hash == expected_dna_hash, "evidence DNA hash differs from adapter capabilities")
    require_text(manifest.get("happ_hash"), "happ_hash", 4096)
    require_hash32(manifest.get("adapter_release_hash"), "adapter_release_hash")

    started = manifest.get("started_at_unix_micros")
    completed = manifest.get("completed_at_unix_micros")
    require(isinstance(started, int) and started > 0, "started_at_unix_micros must be positive")
    require(isinstance(completed, int) and completed >= started, "completed_at_unix_micros precedes start")

    conductors = manifest.get("conductors")
    require(isinstance(conductors, list), "conductors must be a list")
    require(3 <= len(conductors) <= 16, "evidence requires 3-16 conductors")
    conductor_ids: list[str] = []
    database_ids: list[str] = []
    network_ids: list[str] = []
    for index, conductor_value in enumerate(conductors):
        require(isinstance(conductor_value, dict), f"conductors[{index}] must be an object")
        conductor_id = require_text(conductor_value.get("id"), f"conductors[{index}].id")
        conductor_ids.append(conductor_id)
        require_text(conductor_value.get("agent_pub_key"), f"conductors[{index}].agent_pub_key", 4096)
        database_ids.append(require_hash32(conductor_value.get("database_identity_hash"), f"conductors[{index}].database_identity_hash"))
        network_ids.append(require_hash32(conductor_value.get("network_identity_hash"), f"conductors[{index}].network_identity_hash"))
    require(len(conductor_ids) == len(set(conductor_ids)), "conductor ids must be unique")
    require(len(database_ids) == len(set(database_ids)), "conductors must use distinct database identities")
    require(len(network_ids) == len(set(network_ids)), "conductors must use distinct network identities")
    require(set(conductor_ids) >= REQUIRED_CONDUCTORS, "required conductor roles are missing")

    files = manifest.get("files")
    require(isinstance(files, list), "files must be a list")
    require(1 <= len(files) <= MAX_EVIDENCE_FILES, "invalid evidence file count")
    listed_paths: list[PurePosixPath] = []
    total_bytes = 0
    for index, file_value in enumerate(files):
        require(isinstance(file_value, dict), f"files[{index}] must be an object")
        relative = confined_relative_path(file_value.get("path"), f"files[{index}].path")
        listed_paths.append(relative)
        expected_size = file_value.get("size")
        require(isinstance(expected_size, int) and expected_size >= 0, f"files[{index}].size must be non-negative")
        require(expected_size <= MAX_EVIDENCE_FILE_BYTES, f"files[{index}] exceeds per-file size limit")
        expected_hash = require_hash32(file_value.get("sha256"), f"files[{index}].sha256")
        resolved = resolve_confined(manifest_dir, relative, f"files[{index}].path")
        require(resolved.is_file(), f"files[{index}] is not a regular file")
        actual_size = resolved.stat().st_size
        require(actual_size == expected_size, f"files[{index}] size mismatch")
        require(file_sha256_b64(resolved) == expected_hash, f"files[{index}] hash mismatch")
        total_bytes += actual_size
        require(total_bytes <= MAX_EVIDENCE_TOTAL_BYTES, "evidence bundle exceeds total size limit")
    require(len(listed_paths) == len(set(listed_paths)), "evidence file paths must be unique")
    actual_paths: list[PurePosixPath] = []
    for candidate in sorted(manifest_dir.rglob("*")):
        require(not candidate.is_symlink(), f"evidence directory contains a symlink: {candidate}")
        if candidate.is_dir():
            continue
        require(candidate.is_file(), f"evidence directory contains a non-regular file: {candidate}")
        relative = PurePosixPath(candidate.relative_to(manifest_dir).as_posix())
        if relative == PurePosixPath("manifest.json"):
            continue
        actual_paths.append(relative)
    require(
        set(actual_paths) == set(listed_paths),
        "evidence directory contains missing or unlisted files",
    )

    observations = manifest.get("observations")
    require(isinstance(observations, list), "observations must be a list")
    require(len(observations) == len(conductors), "every conductor must contribute one final observation")
    observed_ids: list[str] = []
    for index, observation_value in enumerate(observations):
        require(isinstance(observation_value, dict), f"observations[{index}] must be an object")
        conductor_id = require_text(observation_value.get("conductor_id"), f"observations[{index}].conductor_id")
        observed_ids.append(conductor_id)
        require(conductor_id in conductor_ids, f"observations[{index}] names an unknown conductor")
        require(
            require_text(observation_value.get("state"), f"observations[{index}].state") == expected_state,
            f"conductor {conductor_id} did not converge to {expected_state}",
        )
        observed_at = observation_value.get("observed_at_unix_micros")
        require(isinstance(observed_at, int) and started <= observed_at <= completed, f"observations[{index}] timestamp is outside run bounds")
        response_path = confined_relative_path(observation_value.get("response_path"), f"observations[{index}].response_path")
        require(response_path in listed_paths, f"observations[{index}] response is not listed in files")
        response_hash = require_hash32(observation_value.get("response_sha256"), f"observations[{index}].response_sha256")
        resolved = resolve_confined(manifest_dir, response_path, f"observations[{index}].response_path")
        require(file_sha256_b64(resolved) == response_hash, f"observations[{index}] response hash mismatch")
    require(len(observed_ids) == len(set(observed_ids)), "each conductor may contribute only one final observation")
    require(set(observed_ids) == set(conductor_ids), "final observations do not cover all conductors")

    events = manifest.get("events")
    require(isinstance(events, list) and bool(events), "events must be a non-empty list")
    previous_sequence = -1
    previous_time = started
    for index, event_value in enumerate(events):
        require(isinstance(event_value, dict), f"events[{index}] must be an object")
        sequence = event_value.get("sequence")
        event_time = event_value.get("at_unix_micros")
        require(isinstance(sequence, int) and sequence == previous_sequence + 1, "event sequence must be contiguous from zero")
        require(isinstance(event_time, int) and previous_time <= event_time <= completed, "event timestamps must be monotonic and within run bounds")
        require_text(event_value.get("kind"), f"events[{index}].kind")
        require_text(event_value.get("actor"), f"events[{index}].actor")
        previous_sequence = sequence
        previous_time = event_time

    return {
        "conductor_ids": conductor_ids,
        "dna_hash": dna_hash,
        "manifest_hash": manifest_hash(manifest),
        "total_evidence_bytes": total_bytes,
    }


def load_and_validate_manifest(
    manifest_path: Path,
    evidence_root: Path,
    **expected: Any,
) -> tuple[dict[str, Any], dict[str, Any]]:
    evidence_root = evidence_root.resolve(strict=True)
    require(evidence_root.is_dir(), "evidence root must be a directory")
    require(not manifest_path.is_symlink(), "manifest path may not be a symlink")
    resolved_manifest = manifest_path.resolve(strict=True)
    require(resolved_manifest.is_relative_to(evidence_root), "manifest escapes evidence root")
    require(resolved_manifest.is_file(), "manifest is not a regular file")
    manifest = load_json_object(resolved_manifest, MAX_MANIFEST_BYTES, "evidence manifest")
    summary = validate_manifest(manifest, resolved_manifest.parent, **expected)
    return manifest, summary
