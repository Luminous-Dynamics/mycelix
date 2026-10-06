#!/usr/bin/env python3
"""Independent reference verifier for Security Kernel causal-binding commitments.

This file is intentionally self-contained: it performs no network access, imports no
repository-local modules, and uses a distinct length-framed encoding rather than the
S2 verifier's JSON canonicalization. It validates the complete v1/v2 binding schema,
then emits a reference SHA-256 commitment.
"""
import hashlib
import json
import struct
import sys
from pathlib import Path

SCHEMA_V1 = "security-kernel-execution-binding-v1"
SCHEMA_V2 = "security-kernel-execution-binding-v2"

V1_KEYS = frozenset(
    {
        "schema",
        "repository_id",
        "repository",
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
        "reference_verifier_path",
        "reference_verifier_blob_sha",
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

INT_KEYS = frozenset(
    {
        "repository_id",
        "candidate_repository_id",
        "candidate_pr",
        "run_id",
        "run_attempt",
        "run_number",
        "resolver_job_id",
        "s1_job_id",
    }
)

HEX40_KEYS = frozenset(
    {
        "candidate_sha",
        "dispatcher_workflow_sha",
        "s1_workflow_blob_sha",
        "reference_verifier_blob_sha",
    }
)

HEX64_KEYS = frozenset({"receipt_content_sha256"})
SHA256_PREFIX_KEYS = frozenset({"artifact_digest"})


def fail(message: str) -> None:
    raise SystemExit(f"REFERENCE_VERIFY_FAIL: {message}")


def validate(binding: object) -> dict:
    if not isinstance(binding, dict):
        fail("binding must be a JSON object")

    schema = binding.get("schema")
    if schema == SCHEMA_V1:
        expected = V1_KEYS
    elif schema == SCHEMA_V2:
        expected = V2_KEYS
    else:
        fail(f"unsupported schema: {schema!r}")

    if set(binding) != expected:
        fail(
            "closed-world schema mismatch: "
            f"missing={sorted(expected - set(binding))!r} "
            f"extra={sorted(set(binding) - expected)!r}"
        )

    for key in INT_KEYS:
        value = binding.get(key)
        if type(value) is not int or value < 1:
            fail(f"{key} must be a positive integer")

    for key in HEX40_KEYS:
        value = binding.get(key)
        if not isinstance(value, str) or len(value) != 40:
            fail(f"{key} must be a 40-character string")
        if not all(char in "0123456789abcdef" for char in value):
            fail(f"{key} must use lowercase hexadecimal")

    for key in HEX64_KEYS:
        value = binding.get(key)
        if not isinstance(value, str) or len(value) != 64:
            fail(f"{key} must be a 64-character string")
        if not all(char in "0123456789abcdef" for char in value):
            fail(f"{key} must use lowercase hexadecimal")

    for key in SHA256_PREFIX_KEYS:
        value = binding.get(key)
        if (
            not isinstance(value, str)
            or len(value) != 71
            or not value.startswith("sha256:")
            or not all(char in "0123456789abcdef" for char in value[7:])
        ):
            fail(f"{key} must be sha256:<64 lowercase hex>")

    for key in (
        "repository",
        "candidate_repository",
        "dispatcher_workflow_path",
        "s1_workflow_path",
        "reference_verifier_path",
        "artifact_name",
    ):
        if key in binding and (not isinstance(binding[key], str) or not binding[key]):
            fail(f"{key} must be a non-empty string")

    if binding.get("schema") == SCHEMA_V2:
        if not binding["artifact_name"].startswith(
            "security-kernel-independent-qualification-"
        ):
            fail("artifact_name does not use the registered qualification namespace")

    return binding


def frame(raw: bytes) -> bytes:
    return struct.pack(">Q", len(raw)) + raw


def scalar(value: object) -> bytes:
    if type(value) is int:
        return b"i" + struct.pack(">q", value)
    if isinstance(value, str):
        return b"s" + value.encode("utf-8")
    fail(f"unsupported value type: {type(value).__name__}")


def reference_digest(binding: dict) -> str:
    pieces = [frame(b"security-kernel-reference-binding-v1")]
    for key in sorted(binding):
        key_bytes = key.encode("utf-8")
        value_bytes = scalar(binding[key])
        pieces.append(frame(key_bytes))
        pieces.append(frame(value_bytes))
    return hashlib.sha256(b"".join(pieces)).hexdigest()


def mutation_digest(binding: dict, key: str, value: object) -> str:
    mutated = dict(binding)
    mutated[key] = value
    return reference_digest(mutated)


def mutations(binding: dict) -> dict:
    out = {}
    for key, value in binding.items():
        if key == "schema":
            continue
        if type(value) is int:
            out[key] = value + 1
        elif isinstance(value, str):
            if len(value) == 40 and all(c in "0123456789abcdef" for c in value):
                out[key] = ("f" if value[0] != "f" else "e") + value[1:]
            elif len(value) == 64 and all(c in "0123456789abcdef" for c in value):
                out[key] = ("f" if value[0] != "f" else "e") + value[1:]
            elif value.startswith("sha256:") and len(value) == 71:
                out[key] = "sha256:" + ("f" if value[7] != "f" else "e") + value[8:]
            else:
                out[key] = value + "-mutated"
        else:
            fail(f"cannot construct mutation for {key}")
    return out


def main() -> None:
    if len(sys.argv) != 2:
        fail("usage: reference_verify_execution_binding.py <binding.json>")

    binding = validate(json.loads(Path(sys.argv[1]).read_text(encoding="utf-8")))
    digest = reference_digest(binding)

    for key, replacement in mutations(binding).items():
        if mutation_digest(binding, key, replacement) == digest:
            fail(f"reference digest invariant under mutation of {key}")

    print(f"schema={binding['schema']}")
    print(f"reference_digest={digest}")
    print(f"mutations_verified={len(binding) - 1}")


if __name__ == "__main__":
    main()
