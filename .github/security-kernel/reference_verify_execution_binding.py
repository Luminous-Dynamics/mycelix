#!/usr/bin/env python3
"""Independent reference verifier for Security Kernel execution/evidence bindings.

No network access and no repository-local imports. The verifier validates the exact
execution-binding v3 schema currently emitted by S2, then computes a distinct
length-framed commitment and verifies that every non-schema field affects it.
"""

import hashlib
import json
import struct
import sys
from pathlib import Path

SCHEMA = "security-kernel-execution-binding-v4"

V1_KEYS = frozenset(
    {
        "schema",
        "repository_id",
        "repository",
        "workflow_id",
        "candidate_repository_id",
        "candidate_repository",
        "candidate_pr",
        "candidate_sha",
        "dispatcher_workflow_path",
        "dispatcher_workflow_sha",
        "s1_workflow_path",
        "s1_workflow_blob_sha",
        "run_id",
        "run_attempt",
        "run_number",
        "resolver_job_id",
        "s1_job_id",
    }
)

V2_KEYS = V1_KEYS | frozenset(
    {
        "artifact_name",
        "artifact_id",
        "artifact_digest",
        "receipt_content_sha256",
    }
)

V3_KEYS = V2_KEYS | frozenset(
    {
        "verifier_workflow_path",
        "verifier_workflow_sha",
        "verifier_workflow_blob_sha",
    }
)

V4_KEYS = V3_KEYS | frozenset({"trigger_event"})

INT_KEYS = frozenset(
    {
        "repository_id",
        "workflow_id",
        "candidate_repository_id",
        "candidate_pr",
        "run_id",
        "run_attempt",
        "run_number",
        "resolver_job_id",
        "s1_job_id",
        "artifact_id",
    }
)

HEX40_KEYS = frozenset(
    {
        "candidate_sha",
        "dispatcher_workflow_sha",
        "s1_workflow_blob_sha",
        "verifier_workflow_sha",
        "verifier_workflow_blob_sha",
    }
)

HEX64_KEYS = frozenset({"receipt_content_sha256"})
REPOSITORY_ID = "Luminous-Dynamics/mycelix"


def fail(message: str) -> None:
    raise SystemExit(f"EXECUTION_REFERENCE_FAIL: {message}")


def validate(binding: object) -> dict:
    if not isinstance(binding, dict):
        fail("binding must be a JSON object")
    if set(binding) != V4_KEYS:
        fail(
            "closed-world schema mismatch: "
            f"missing={sorted(V4_KEYS - set(binding))!r} "
            f"extra={sorted(set(binding) - V4_KEYS)!r}"
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

    if binding["repository"] != REPOSITORY_ID:
        fail("trusted repository identity mismatch")
    if binding["candidate_pr"] < 1:
        fail("candidate PR must be positive")
    if not isinstance(binding["candidate_repository"], str) or "/" not in binding["candidate_repository"]:
        fail("candidate repository identity is malformed")
    for key in (
        "dispatcher_workflow_path",
        "s1_workflow_path",
        "artifact_name",
        "verifier_workflow_path",
        "trigger_event",
    ):
        if not isinstance(binding[key], str) or not binding[key]:
            fail(f"{key} must be a non-empty string")

    if binding["verifier_workflow_path"] != ".github/workflows/security-kernel-trusted-result-verifier.yml":
        fail("verifier_workflow_path is not the registered S2 verifier path")

    if binding["trigger_event"] not in {"pull_request_target", "schedule"}:
        fail("trigger_event is outside the registered Security Kernel trigger witness set")

    if not isinstance(binding["artifact_digest"], str):
        fail("artifact_digest must be a string")
    if (
        len(binding["artifact_digest"]) != 71
        or not binding["artifact_digest"].startswith("sha256:")
        or not all(char in "0123456789abcdef" for char in binding["artifact_digest"][7:])
    ):
        fail("artifact_digest must be sha256:<64 lowercase hex>")

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
    pieces = [frame(b"security-kernel-execution-reference-v3")]
    for key in sorted(binding):
        pieces.append(frame(key.encode("utf-8")))
        pieces.append(frame(scalar(binding[key])))
    return hashlib.sha256(b"".join(pieces)).hexdigest()


def mutate(value: object) -> object:
    if type(value) is int:
        return value + 1
    if isinstance(value, str):
        if value.startswith("sha256:"):
            replacement = "f" if value[7] != "f" else "e"
            return "sha256:" + replacement + value[8:]
        if value and value[0] != "f":
            return "f" + value[1:]
        return value + "-mutated"
    fail(f"unsupported mutation type: {type(value).__name__}")


def main() -> None:
    if len(sys.argv) != 2:
        fail("usage: reference_verify_execution_binding.py <binding.json>")
    binding = validate(json.loads(Path(sys.argv[1]).read_text(encoding="utf-8")))
    digest = reference_digest(binding)
    mutations_verified = 0
    for key in sorted(V4_KEYS - {"schema"}):
        mutated = dict(binding)
        mutated[key] = mutate(binding[key])
        if reference_digest(mutated) == digest:
            fail(f"reference digest invariant under mutation of {key}")
        mutations_verified += 1
    print(f"schema={SCHEMA}")
    print(f"reference_digest={digest}")
    print(f"mutations_verified={mutations_verified}")


if __name__ == "__main__":
    main()
