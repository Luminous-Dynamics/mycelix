#!/usr/bin/env python3
"""Independent reference verifier for Security Kernel evidence-retention commitments.

This verifier is deliberately self-contained: it performs no network access, imports
no repository-local modules, and uses a distinct length-framed commitment encoding.
It validates the exact retention-binding v1 schema and checks that every binding field
changes the independent commitment.
"""

import datetime
import hashlib
import json
import struct
import sys
from pathlib import Path

SCHEMA = "security-kernel-evidence-retention-binding-v1"
EXPECTED_REFERENCE_PATH = ".github/security-kernel/reference_verify_evidence_retention_binding.py"

KEYS = frozenset(
    {
        "schema",
        "parent_evidence_binding_sha256",
        "candidate_pr",
        "candidate_sha",
        "run_id",
        "run_attempt",
        "artifact_name",
        "artifact_id",
        "artifact_digest",
        "artifact_size",
        "content_sha256",
        "receipt_artifact_expires_at",
        "transcript_artifact_expires_at",
        "reference_verifier_path",
        "reference_verifier_blob_sha",
    }
)

INT_KEYS = frozenset(
    {"candidate_pr", "run_id", "run_attempt", "artifact_id", "artifact_size"}
)
HEX40_KEYS = frozenset({"candidate_sha", "reference_verifier_blob_sha"})
HEX64_KEYS = frozenset(
    {"parent_evidence_binding_sha256", "content_sha256"}
)


def fail(message: str) -> None:
    raise SystemExit(f"RETENTION_REFERENCE_FAIL: {message}")


def validate(binding: object) -> dict:
    if not isinstance(binding, dict):
        fail("binding must be a JSON object")
    if set(binding) != KEYS:
        fail(
            "closed-world schema mismatch: "
            f"missing={sorted(KEYS - set(binding))!r} "
            f"extra={sorted(set(binding) - KEYS)!r}"
        )
    if binding["schema"] != SCHEMA:
        fail(f"unexpected schema: {binding['schema']!r}")

    for key in INT_KEYS:
        value = binding[key]
        if type(value) is not int or value < 1:
            fail(f"{key} must be a positive integer")

    for key in HEX40_KEYS:
        value = binding[key]
        if (
            not isinstance(value, str)
            or len(value) != 40
            or not all(char in "0123456789abcdef" for char in value)
        ):
            fail(f"{key} must be lowercase 40-character hexadecimal")

    for key in HEX64_KEYS:
        value = binding[key]
        if (
            not isinstance(value, str)
            or len(value) != 64
            or not all(char in "0123456789abcdef" for char in value)
        ):
            fail(f"{key} must be lowercase 64-character hexadecimal")

    artifact_digest = binding["artifact_digest"]
    if (
        not isinstance(artifact_digest, str)
        or len(artifact_digest) != 71
        or not artifact_digest.startswith("sha256:")
        or not all(char in "0123456789abcdef" for char in artifact_digest[7:])
    ):
        fail("artifact_digest must be sha256:<64 lowercase hex>")

    for key in (
        "artifact_name",
        "receipt_artifact_expires_at",
        "transcript_artifact_expires_at",
        "reference_verifier_path",
    ):
        if not isinstance(binding[key], str) or not binding[key]:
            fail(f"{key} must be a non-empty string")

    if not binding["artifact_name"].startswith("security-kernel-negative-controls-"):
        fail("artifact_name is outside the registered negative-control namespace")
    if not binding["artifact_name"].endswith(".log"):
        fail("artifact_name must end in .log")
    if binding["reference_verifier_path"] != EXPECTED_REFERENCE_PATH:
        fail("reference_verifier_path is not the registered retention oracle path")

    for key in ("receipt_artifact_expires_at", "transcript_artifact_expires_at"):
        try:
            parsed = datetime.datetime.fromisoformat(binding[key].replace("Z", "+00:00"))
        except ValueError as exc:
            fail(f"{key} is not RFC3339/ISO-8601 parseable: {binding[key]!r}")
        if parsed.tzinfo is None:
            fail(f"{key} must include an explicit timezone")

    return binding


def frame(raw: bytes) -> bytes:
    return struct.pack(">Q", len(raw)) + raw


def scalar(value: object) -> bytes:
    if type(value) is int:
        return b"i" + struct.pack(">q", value)
    if isinstance(value, str):
        return b"s" + value.encode("utf-8")
    fail(f"unsupported scalar type: {type(value).__name__}")


def reference_digest(binding: dict) -> str:
    pieces = [frame(b"security-kernel-retention-reference-binding-v1")]
    for key in sorted(binding):
        pieces.append(frame(key.encode("utf-8")))
        pieces.append(frame(scalar(binding[key])))
    return hashlib.sha256(b"".join(pieces)).hexdigest()


def mutated_value(value: object) -> object:
    if type(value) is int:
        return value + 1
    if isinstance(value, str):
        if value.startswith("sha256:") and len(value) == 71:
            replacement = "f" if value[7] != "f" else "e"
            return "sha256:" + replacement + value[8:]
        if len(value) == 64 and all(char in "0123456789abcdef" for char in value):
            replacement = "f" if value[0] != "f" else "e"
            return replacement + value[1:]
        if len(value) == 40 and all(char in "0123456789abcdef" for char in value):
            replacement = "f" if value[0] != "f" else "e"
            return replacement + value[1:]
        if value.endswith("Z"):
            return value.replace("Z", "+00:00")
        return value + "-mutated"
    fail(f"cannot mutate {type(value).__name__}")


def main() -> None:
    if len(sys.argv) != 2:
        fail("usage: reference_verify_evidence_retention_binding.py <binding.json>")

    binding = validate(json.loads(Path(sys.argv[1]).read_text(encoding="utf-8")))
    digest = reference_digest(binding)
    mutations_verified = 0

    for key in sorted(KEYS - {"schema"}):
        mutated = dict(binding)
        mutated[key] = mutated_value(binding[key])
        if reference_digest(mutated) == digest:
            fail(f"reference digest invariant under mutation of {key}")
        mutations_verified += 1

    print(f"schema={SCHEMA}")
    print(f"reference_digest={digest}")
    print(f"mutations_verified={mutations_verified}")


if __name__ == "__main__":
    main()
