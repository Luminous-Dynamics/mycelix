#!/usr/bin/env python3
"""Independent CIV-013 durable-adapter golden-digest verifier.

This script verifies versioned canonical encoding and frozen expected outcomes.
It does not implement the Rust adapter's transition reducer. Scenario outcomes
must also be exercised by adapter tests.
"""
from __future__ import annotations

import hashlib
import json
import re
import struct
import sys
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[2]
FIXTURE_PATH = ROOT / "docs/integral/civ-013-durable-adapter-conformance-v1.json"
MANIFEST_PATH = ROOT / "docs/integral/civ-013-durable-adapter-conformance-v1-manifest.json"
VERIFIER_PATH = Path(__file__).resolve()

PROFILE_ID = "civ-013-durable-adapter-v1"
SPEC_VERSION = "civ-013-durable-adapter-conformance-v1"
RECORD_DOMAIN = b"mycelix-civ013-durable-record-v1\0"
FORK_DOMAIN = b"mycelix-civ013-durable-fork-v1\0"

RECORD_IDS = ["DA001-bootstrap-record", "DA002-successor-record"]
FORK_IDS = ["DA003-first-fork-evidence", "DA004-linked-fork-evidence"]
STATE_IDS = [
    "DA005-trusted-genesis-bootstrap", "DA006-exact-retry-after-accept",
    "DA007-stale-predecessor", "DA008-receipt-tail-regression",
    "DA009-receipt-tail-equivocation", "DA010-anchor-unavailable-before-prepare",
    "DA011-anchor-unavailable-after-prepare", "DA012-recover-exact-prepared-successor",
    "DA013-anchor-ahead-without-candidate", "DA014-anchor-same-generation-digest-mismatch",
    "DA015-local-history-behind-anchor", "DA016-conflicting-prepared-candidate",
    "DA017-late-finalize-historical-candidate",
    "DA018-same-generation-retry-tampered-head",
    "DA019-late-finalize-corrupt-head-metadata",
    "DA020-late-finalize-tampered-head-record",
    "DA021-anchor-ahead-after-candidate-cas",
]
OUTCOMES = {
    "ACCEPT_AFTER_EXTERNAL_CAS", "IDEMPOTENT_ACCEPT", "REJECT_STALE",
    "REJECT_RECEIPT_ROLLBACK", "REJECT_RECEIPT_EQUIVOCATION",
    "FAIL_CLOSED_NO_LOCAL_WRITE", "RETAIN_PREPARED_NO_ACCEPTANCE",
    "PROMOTE_PREPARED", "REJECT_ROLLBACK_OR_MISSING_HISTORY",
    "REJECT_EXTERNAL_MISMATCH", "REJECT_CONFLICT_PRESERVE_EVIDENCE",
    "IDEMPOTENT_HISTORICAL_SUCCESS_NO_HEAD_REGRESSION",
    "REJECT_TAMPERED_CURRENT_HEAD",
    "REJECT_CORRUPT_HEAD_METADATA",
    "REJECT_AHEAD_ANCHOR_NO_FALSE_FORK",
}

def fail(message: str) -> None:
    raise SystemExit(f"FAIL: {message}")

def read_json(path: Path) -> Any:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        fail(f"cannot read {path.relative_to(ROOT)}: {error}")

def digest_hex(value: Any, field: str, optional: bool = False) -> bytes | None:
    if value is None and optional:
        return None
    if not isinstance(value, str) or not re.fullmatch(r"[0-9a-f]{64}", value):
        fail(f"{field} must be a lowercase 32-byte hex digest")
    return bytes.fromhex(value)

def encode_field(value: str) -> bytes:
    raw = value.encode("utf-8")
    return struct.pack(">Q", len(raw)) + raw

def record_bytes(vector: dict[str, Any]) -> bytes:
    protocol = vector["protocol_version"]
    generation = vector["generation"]
    sequence = vector["receipt_sequence"]
    if not isinstance(protocol, int) or not 0 <= protocol < 2**16:
        fail(f"{vector['id']}: protocol version outside u16")
    if not isinstance(generation, int) or not 1 <= generation < 2**64:
        fail(f"{vector['id']}: generation outside positive u64")
    if not isinstance(sequence, int) or not 0 <= sequence < 2**64:
        fail(f"{vector['id']}: receipt sequence outside u64")
    receipt = digest_hex(vector["receipt_digest"], vector["id"] + ".receipt_digest", optional=True)
    previous = digest_hex(vector["previous_record_digest"], vector["id"] + ".previous_record_digest", optional=True)
    if (sequence == 0) != (receipt is None):
        fail(f"{vector['id']}: invalid receipt sequence/digest shape")
    return (
        RECORD_DOMAIN + struct.pack(">H", protocol) + struct.pack(">Q", generation)
        + encode_field(vector["log_id"]) + encode_field(vector["policy_version"])
        + digest_hex(vector["anchor_digest"], vector["id"] + ".anchor_digest")
        + struct.pack(">Q", sequence)
        + (b"\x00" if receipt is None else b"\x01" + receipt)
        + (b"\x00" if previous is None else b"\x01" + previous)
    )

def fork_bytes(vector: dict[str, Any]) -> bytes:
    generation = vector["generation"]
    if not isinstance(generation, int) or not 1 <= generation < 2**64:
        fail(f"{vector['id']}: generation outside positive u64")
    first = digest_hex(vector["first_record_digest"], vector["id"] + ".first_record_digest")
    conflicting = digest_hex(vector["conflicting_record_digest"], vector["id"] + ".conflicting_record_digest")
    previous = digest_hex(vector["previous_evidence_digest"], vector["id"] + ".previous_evidence_digest", optional=True)
    if first == conflicting:
        fail(f"{vector['id']}: fork digests must differ")
    return (
        FORK_DOMAIN + encode_field(vector["log_id"]) + struct.pack(">Q", generation)
        + first + conflicting
        + (b"\x00" if previous is None else b"\x01" + previous)
    )

def exact_ids(vectors: Any, expected: list[str], label: str) -> None:
    if not isinstance(vectors, list):
        fail(f"{label} must be an array")
    if len(vectors) != len(expected):
        fail(f"{label} count mismatch: expected {len(expected)}, got {len(vectors)}")
    if any(not isinstance(item, dict) for item in vectors):
        fail(f"{label} contains a non-object vector")
    ids = [item.get("id") for item in vectors]
    if ids != expected:
        fail(f"{label} IDs/order mismatch: got {ids!r}")

def main() -> int:
    fixture = read_json(FIXTURE_PATH)
    manifest = read_json(MANIFEST_PATH)
    if fixture.get("profile_id") != PROFILE_ID or fixture.get("spec_version") != SPEC_VERSION:
        fail("fixture profile/version mismatch")
    if manifest.get("profile_id") != PROFILE_ID or manifest.get("spec_version") != SPEC_VERSION:
        fail("manifest profile/version mismatch")
    expected_encoding = {
        "digest": "SHA-256",
        "record_domain_hex": RECORD_DOMAIN.hex(),
        "fork_domain_hex": FORK_DOMAIN.hex(),
        "protocol_version": "u16 big-endian",
        "generation": "u64 big-endian",
        "byte_string": "u64 byte length big-endian followed by UTF-8 bytes",
        "receipt_sequence": "u64 big-endian",
        "optional_digest": "0x00 for None; 0x01 followed by exactly 32 bytes for Some",
    }
    if fixture.get("encoding") != expected_encoding:
        fail("fixture encoding declaration does not match checker implementation")
    source_sha = fixture.get("source_adapter_commit")
    if not isinstance(source_sha, str) or not re.fullmatch(r"[0-9a-f]{40}", source_sha):
        fail("source_adapter_commit must be an exact 40-hex SHA")
    if manifest.get("source_adapter_commit") != source_sha:
        fail("fixture and manifest source commits differ")
    if manifest.get("fixture_path") != str(FIXTURE_PATH.relative_to(ROOT)):
        fail("fixture path mismatch")
    if manifest.get("verifier_path") != str(VERIFIER_PATH.relative_to(ROOT)):
        fail("verifier path mismatch")
    checker_sha = manifest.get("checker_commit_sha")
    if not isinstance(checker_sha, str) or not re.fullmatch(r"[0-9a-f]{40}", checker_sha):
        fail("checker_commit_sha must pin the commit containing this verifier")
    for field in ("fixture_sha256", "verifier_sha256"):
        if not re.fullmatch(r"[0-9a-f]{64}", str(manifest.get(field, ""))):
            fail(f"{field} is missing or malformed")
    if hashlib.sha256(FIXTURE_PATH.read_bytes()).hexdigest() != manifest["fixture_sha256"]:
        fail("fixture SHA-256 differs from frozen manifest")
    if hashlib.sha256(VERIFIER_PATH.read_bytes()).hexdigest() != manifest["verifier_sha256"]:
        fail("verifier SHA-256 differs from frozen manifest")

    exact_ids(fixture.get("records"), RECORD_IDS, "record vector")
    exact_ids(fixture.get("fork_evidence"), FORK_IDS, "fork evidence vector")
    exact_ids(fixture.get("state_vectors"), STATE_IDS, "state vector")
    if manifest.get("expected_record_count") != len(RECORD_IDS):
        fail("manifest record count mismatch")
    if manifest.get("expected_fork_evidence_count") != len(FORK_IDS):
        fail("manifest fork-evidence count mismatch")
    if manifest.get("expected_state_vector_count") != len(STATE_IDS):
        fail("manifest state-vector count mismatch")
    if manifest.get("required_vector_ids") != RECORD_IDS + FORK_IDS + STATE_IDS:
        fail("manifest required vector IDs/order mismatch")

    for vector in fixture["records"]:
        raw = record_bytes(vector)
        actual = hashlib.sha256(raw).hexdigest()
        if raw.hex() != vector.get("canonical_bytes_hex"):
            fail(f"{vector['id']}: canonical byte fixture mismatch")
        if actual != vector.get("expected_digest"):
            fail(f"{vector['id']}: golden record digest mismatch")
        print(f"PASS {vector['id']} sha256={actual}")

    previous_by_log: dict[str, str] = {}
    for vector in fixture["fork_evidence"]:
        raw = fork_bytes(vector)
        actual = hashlib.sha256(raw).hexdigest()
        if raw.hex() != vector.get("canonical_bytes_hex"):
            fail(f"{vector['id']}: canonical byte fixture mismatch")
        if actual != vector.get("expected_digest"):
            fail(f"{vector['id']}: golden fork digest mismatch")
        if vector.get("previous_evidence_digest") != previous_by_log.get(vector["log_id"]):
            fail(f"{vector['id']}: fork-evidence predecessor chain mismatch")
        previous_by_log[vector["log_id"]] = actual
        print(f"PASS {vector['id']} sha256={actual}")

    for vector in fixture["state_vectors"]:
        if vector.get("expected_outcome") not in OUTCOMES:
            fail(f"{vector.get('id')}: unsupported expected outcome")
        if vector.get("operation") not in {"initialize", "advance", "recover", "prepare", "finalize"}:
            fail(f"{vector.get('id')}: unsupported operation")
        print(f"RECORDED {vector['id']} outcome={vector['expected_outcome']}")

    expected_ceiling = (
        "Golden canonical-byte conformance and declared state expectations only; "
        "not proof that the Rust adapter tests passed, SQLite survives power loss, "
        "an independent anchor is implemented, signatures are authentic, or quorum/anti-rollback is production-qualified."
    )
    if manifest.get("claim_ceiling") != expected_ceiling:
        fail("claim ceiling changed without profile review")
    print(
        "SUMMARY: 4 golden digest vectors verified; 13 adapter state expectations frozen; "
        "no transition reducer or production backend was executed by this checker."
    )
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
