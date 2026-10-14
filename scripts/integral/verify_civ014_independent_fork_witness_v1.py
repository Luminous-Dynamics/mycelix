#!/usr/bin/env python3
"""Independent CIV-014 canonical event/frontier/receipt profile checker.

This verifier is standard-library Python. It checks frozen canonical bytes and
protocol-level rejection cases; it does not import or execute the Symthaea Rust
crate and does not qualify a production witness backend or local/remote recovery.
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
FIXTURE = ROOT / "docs/integral/civ-014-independent-fork-witness-conformance-v1.json"
MANIFEST = ROOT / "docs/integral/civ-014-independent-fork-witness-conformance-v1-manifest.json"
PROFILE_ID = "civ-014-independent-fork-witness-v1"
SOURCE_COMMIT = "5e2a06f2b6a0aa12c76d652bc249271050a8c0cd"
SOURCE_MODULE_BLOB = "eadb5e4209c45d49fe19f7aa488af2ce591218db"
SOURCE_LIB_BLOB = "fc5484dd88a8219f29cc3105493a8f324461bf7c"
SOURCE_CRATE_MANIFEST_BLOB = "387c0121674f3caf7712a89e361f64f0308952f6"
SOURCE_WORKSPACE_MANIFEST_BLOB = "b618fd4e5415a86b8e82c6676a7df2d2f6aa021a"
SOURCE_LOCK_BLOB = "07ee104e552c160b84302c2a0c323fc2d68ca844"
SOURCE_TOOLCHAIN_BLOB = "4f0430eac96d545bcfaa0df23ce475faf4aee96a"
SOURCE_RUST_VERSION = "1.96.0"
SOURCE_RUSQLITE_VERSION = "0.40.2"
SOURCE_LIBSQLITE3_SYS_VERSION = "0.38.2"
SOURCE_BUNDLED_SQLITE_VERSION = "3.53.2"
EVENT_SCHEMA_VERSION = 1
EVENT_DOMAIN = b"mycelix-civ014-fork-event-v1\0"
RECEIPT_DOMAIN = b"mycelix-civ014-fork-receipt-v1\0"
ZERO_DIGEST = "00" * 32
U64_MAX = (1 << 64) - 1

REQUIRED_IDS = {
    "DF001-event-canonical-golden",
    "DF002-receipt-canonical-golden",
    "DF003-event-zero-fork-generation",
    "DF004-genesis-head-nonzero-digest",
    "DF005-event-ahead-more-than-one",
    "DF006-identical-competing-records",
    "DF007-empty-frontier-nonzero-tail",
    "DF008-event-prior-frontier-mismatch",
    "DF009-receipt-skipped-count",
    "DF010-tampered-event-digest",
    "DF011-tampered-receipt-digest",
    "DF012-unprovisioned-epoch",
    "DF013-exact-event-replay",
    "DF014-replay-changed-payload",
    "DF015-stale-frontier-no-write",
    "DF016-wrong-log-epoch-scope",
}


class ProfileError(Exception):
    def __init__(self, outcome: str, message: str) -> None:
        super().__init__(message)
        self.outcome = outcome


def require_u64(value: Any, field: str) -> int:
    if type(value) is not int or value < 0 or value > U64_MAX:
        raise ProfileError("RejectInvalidForkEvent", f"{field} is not a u64")
    return value


def digest_bytes(value: Any, field: str) -> bytes:
    if not isinstance(value, str) or re.fullmatch(r"[0-9a-f]{64}", value) is None:
        raise ProfileError("RejectInvalidForkEvent", f"{field} is not a lowercase 32-byte digest")
    return bytes.fromhex(value)


def field(value: str) -> bytes:
    encoded = value.encode("utf-8")
    if not encoded:
        raise ProfileError("RejectInvalidForkEvent", "text field is empty")
    return struct.pack(">Q", len(encoded)) + encoded


def validate_frontier(frontier: dict[str, Any]) -> None:
    log_id = frontier.get("log_id")
    if not isinstance(log_id, str) or not log_id:
        raise ProfileError("RejectInvalidFrontier", "frontier log ID is empty")
    require_u64(frontier.get("witness_epoch"), "witness_epoch")
    count = require_u64(frontier.get("event_count"), "event_count")
    tail = digest_bytes(frontier.get("tail_digest"), "tail_digest")
    if count == 0 and tail != bytes(32):
        raise ProfileError("RejectInvalidFrontier", "empty frontier must use zero tail digest")


def canonical_event_bytes(event: dict[str, Any]) -> bytes:
    schema = event.get("schema_version")
    if type(schema) is not int or schema != EVENT_SCHEMA_VERSION:
        raise ProfileError("RejectInvalidForkEvent", "unsupported event schema version")
    log_id = event.get("log_id")
    if not isinstance(log_id, str) or not log_id:
        raise ProfileError("RejectInvalidForkEvent", "event log ID is empty")
    epoch = require_u64(event.get("witness_epoch"), "witness_epoch")
    prior_count = require_u64(event.get("previous_event_count"), "previous_event_count")
    prior_digest = digest_bytes(event.get("previous_event_digest"), "previous_event_digest")
    accepted_generation = require_u64(event.get("accepted_generation"), "accepted_generation")
    accepted_digest = digest_bytes(event.get("accepted_head_digest"), "accepted_head_digest")
    fork_generation = require_u64(event.get("fork_generation"), "fork_generation")
    first = digest_bytes(event.get("first_record_digest"), "first_record_digest")
    conflicting = digest_bytes(event.get("conflicting_record_digest"), "conflicting_record_digest")

    if fork_generation == 0:
        raise ProfileError("RejectInvalidForkEvent", "fork evidence cannot use generation zero")
    if accepted_generation == 0 and accepted_digest != bytes(32):
        raise ProfileError("RejectInvalidForkEvent", "genesis accepted head must use the zero digest")
    if fork_generation > min(U64_MAX, accepted_generation + 1):
        raise ProfileError("RejectInvalidForkEvent", "fork generation is more than one ahead of accepted head")
    if first == conflicting:
        raise ProfileError("RejectInvalidForkEvent", "competing record digests must differ")
    if prior_count == 0 and prior_digest != bytes(32):
        raise ProfileError("RejectInvalidForkEvent", "empty prior frontier must use zero digest")

    return b"".join(
        (
            EVENT_DOMAIN,
            struct.pack(">H", schema),
            field(log_id),
            struct.pack(">Q", epoch),
            struct.pack(">Q", prior_count),
            prior_digest,
            struct.pack(">Q", accepted_generation),
            accepted_digest,
            struct.pack(">Q", fork_generation),
            first,
            conflicting,
        )
    )


def validate_event(event: dict[str, Any]) -> str:
    canonical = canonical_event_bytes(event)
    calculated = hashlib.sha256(canonical).hexdigest()
    claimed = event.get("event_digest")
    if not isinstance(claimed, str) or claimed != calculated:
        raise ProfileError("RejectInvalidEventDigest", "event digest does not match canonical bytes")
    return calculated


def validate_event_for_frontier(event: dict[str, Any], frontier: dict[str, Any]) -> str:
    validate_frontier(frontier)
    event_digest = validate_event(event)
    if event["log_id"] != frontier["log_id"] or event["witness_epoch"] != frontier["witness_epoch"]:
        raise ProfileError("RejectFrontierScopeMismatch", "event log/epoch differs from frontier")
    if (
        event["previous_event_count"] != frontier["event_count"]
        or event["previous_event_digest"] != frontier["tail_digest"]
    ):
        raise ProfileError("RejectFrontierConflict", "event prior frontier differs from expected frontier")
    return event_digest


def canonical_receipt_bytes(receipt: dict[str, Any]) -> bytes:
    previous = receipt.get("previous_frontier")
    after = receipt.get("frontier_after")
    event = receipt.get("event")
    if not isinstance(previous, dict) or not isinstance(after, dict) or not isinstance(event, dict):
        raise ProfileError("RejectInvalidReceipt", "receipt is missing event/frontier objects")
    validate_frontier(previous)
    event_digest = validate_event_for_frontier(event, previous)
    validate_frontier(after)

    expected_count = previous["event_count"] + 1
    if expected_count > U64_MAX:
        raise ProfileError("RejectInvalidReceipt", "frontier count overflow")
    if (
        after["log_id"] != previous["log_id"]
        or after["witness_epoch"] != previous["witness_epoch"]
        or after["event_count"] != expected_count
        or after["tail_digest"] != event_digest
    ):
        raise ProfileError("RejectInvalidReceipt", "receipt frontier does not advance exactly one event")

    return b"".join(
        (
            RECEIPT_DOMAIN,
            field(previous["log_id"]),
            struct.pack(">Q", previous["witness_epoch"]),
            struct.pack(">Q", previous["event_count"]),
            digest_bytes(previous["tail_digest"], "previous_frontier.tail_digest"),
            bytes.fromhex(event_digest),
            struct.pack(">Q", after["event_count"]),
            digest_bytes(after["tail_digest"], "frontier_after.tail_digest"),
        )
    )


def validate_receipt(receipt: dict[str, Any]) -> str:
    calculated = hashlib.sha256(canonical_receipt_bytes(receipt)).hexdigest()
    if receipt.get("receipt_digest") != calculated:
        raise ProfileError("RejectInvalidReceiptDigest", "receipt digest does not match canonical bytes")
    return calculated


def evaluate(vector: dict[str, Any]) -> str:
    kind = vector.get("kind")
    if kind == "event_digest":
        calculated = validate_event_for_frontier(vector["event"], vector["previous_frontier"])
        if calculated != vector["expected_event_digest"]:
            return "EventGoldenDigestMismatch"
        return "VALID_EVENT"

    if kind == "receipt":
        calculated = validate_receipt(vector["receipt"])
        if calculated != vector["expected_receipt_digest"]:
            return "ReceiptGoldenDigestMismatch"
        return "VALID_RECEIPT"

    if kind == "event_validation":
        validate_event(vector["event_fields"])
        return "VALID_EVENT_UNEXPECTEDLY"

    if kind == "frontier_validation":
        validate_frontier(vector["frontier"])
        return "VALID_FRONTIER_UNEXPECTEDLY"

    if kind == "event_frontier_binding":
        validate_event(vector["event"])
        validate_event_for_frontier(vector["event"], vector["expected_frontier"])
        return "VALID_EVENT_BINDING_UNEXPECTEDLY"

    if kind == "receipt_validation":
        validate_receipt(vector["receipt"])
        return "VALID_RECEIPT_UNEXPECTEDLY"

    if kind == "frontier_lookup":
        if vector.get("provisioned") is False and vector.get("implicit_genesis_allowed") is False:
            return "Unavailable"
        return "ImplicitGenesisNotRejected"

    if kind == "append_replay":
        requested = vector["requested_event"]
        stored = vector["stored_event"]
        validate_event(requested)
        validate_event(stored)
        validate_receipt(vector["stored_receipt"])
        if requested.get("event_digest") != stored.get("event_digest"):
            return "EventIdentityMismatch"
        if requested != stored:
            return "RejectEventIdentityConflict"
        return "ReturnExistingReceipt"

    if kind == "stale_append":
        expected = vector["expected_frontier"]
        current = vector["current_frontier"]
        validate_event_for_frontier(vector["event"], expected)
        validate_frontier(current)
        if current != expected:
            if vector.get("frontier_after_attempt") != current:
                return "UnexpectedStateMutation"
            return "RejectFrontierConflict"
        return "AppendWouldBeAllowed"

    raise ProfileError("InvalidInput", f"unknown vector kind {kind!r}")


def git_blob_sha(path: Path) -> str:
    content = path.read_bytes()
    header = b"blob " + str(len(content)).encode("ascii") + b"\0"
    return hashlib.sha1(header + content).hexdigest()


def verify_asset_bindings(manifest: dict[str, Any]) -> None:
    paths = {
        "fixture_git_blob_sha": FIXTURE,
        "checker_git_blob_sha": Path(__file__).resolve(),
        "spec_git_blob_sha": ROOT / "docs/integral/civ-014-independent-fork-witness-conformance-v1.md",
        "workflow_git_blob_sha": ROOT / ".github/workflows/civ014-independent-fork-witness-conformance.yml",
    }
    for key, path in paths.items():
        expected = manifest.get(key)
        if not isinstance(expected, str) or re.fullmatch(r"[0-9a-f]{40}", expected) is None:
            raise SystemExit(f"FAIL: missing or malformed manifest {key}")
        actual = git_blob_sha(path)
        if actual != expected:
            raise SystemExit(f"FAIL: {key} mismatch; expected={expected} actual={actual}")


def main() -> int:
    if sys.version_info < (3, 10):
        raise SystemExit("FAIL: CPython >= 3.10 is required")
    fixture = json.loads(FIXTURE.read_text(encoding="utf-8"))
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    if manifest.get("schema_version") != 1 or fixture.get("schema_version") != 1:
        raise SystemExit("FAIL: unsupported fixture/manifest schema version")
    if fixture.get("profile_id") != PROFILE_ID or manifest.get("profile_id") != PROFILE_ID:
        raise SystemExit("FAIL: profile ID mismatch")
    if fixture.get("source_commit") != SOURCE_COMMIT or manifest.get("source_commit") != SOURCE_COMMIT:
        raise SystemExit("FAIL: pinned source commit mismatch")
    if fixture.get("source_repository") != "Luminous-Dynamics/symthaea" or manifest.get("source_repository") != "Luminous-Dynamics/symthaea":
        raise SystemExit("FAIL: source repository identity mismatch")
    if fixture.get("event_schema_version") != EVENT_SCHEMA_VERSION or manifest.get("event_schema_version") != EVENT_SCHEMA_VERSION:
        raise SystemExit("FAIL: event schema version mismatch")
    if fixture.get("event_domain") != EVENT_DOMAIN.decode("ascii") or manifest.get("event_domain") != EVENT_DOMAIN.decode("ascii"):
        raise SystemExit("FAIL: event domain separator mismatch")
    if fixture.get("receipt_domain") != RECEIPT_DOMAIN.decode("ascii") or manifest.get("receipt_domain") != RECEIPT_DOMAIN.decode("ascii"):
        raise SystemExit("FAIL: receipt domain separator mismatch")
    if manifest.get("source_event_module_blob_sha") != SOURCE_MODULE_BLOB:
        raise SystemExit("FAIL: pinned source event module blob mismatch")
    if manifest.get("source_lib_blob_sha") != SOURCE_LIB_BLOB:
        raise SystemExit("FAIL: pinned source library blob mismatch")
    if manifest.get("source_crate_manifest_blob_sha") != SOURCE_CRATE_MANIFEST_BLOB:
        raise SystemExit("FAIL: pinned source crate manifest blob mismatch")
    if manifest.get("source_workspace_manifest_blob_sha") != SOURCE_WORKSPACE_MANIFEST_BLOB:
        raise SystemExit("FAIL: pinned workspace manifest blob mismatch")
    if manifest.get("source_lockfile_blob_sha") != SOURCE_LOCK_BLOB:
        raise SystemExit("FAIL: pinned lockfile blob mismatch")
    if manifest.get("source_toolchain_blob_sha") != SOURCE_TOOLCHAIN_BLOB:
        raise SystemExit("FAIL: pinned toolchain blob mismatch")
    if manifest.get("source_rust_version") != SOURCE_RUST_VERSION:
        raise SystemExit("FAIL: pinned Rust toolchain version mismatch")
    if manifest.get("source_rusqlite_version") != SOURCE_RUSQLITE_VERSION:
        raise SystemExit("FAIL: pinned rusqlite version mismatch")
    if manifest.get("source_libsqlite3_sys_version") != SOURCE_LIBSQLITE3_SYS_VERSION:
        raise SystemExit("FAIL: pinned libsqlite3-sys version mismatch")
    if manifest.get("source_bundled_sqlite_version") != SOURCE_BUNDLED_SQLITE_VERSION:
        raise SystemExit("FAIL: pinned bundled SQLite version mismatch")

    vectors = fixture.get("vectors")
    if not isinstance(vectors, list):
        raise SystemExit("FAIL: vectors must be an array")
    ids = [vector.get("id") for vector in vectors]
    if len(ids) != len(REQUIRED_IDS) or set(ids) != REQUIRED_IDS or len(set(ids)) != len(ids):
        raise SystemExit("FAIL: vector IDs do not match the frozen required set")
    if manifest.get("required_vector_ids") != ids:
        raise SystemExit("FAIL: manifest vector order differs from fixture")
    if manifest.get("expected_vector_count") != len(vectors):
        raise SystemExit("FAIL: manifest vector count differs from fixture")
    if manifest.get("claim_ceiling") != fixture.get("claim_ceiling"):
        raise SystemExit("FAIL: claim ceiling mismatch")

    verify_asset_bindings(manifest)
    failures = []
    for vector in vectors:
        try:
            actual = evaluate(vector)
        except ProfileError as exc:
            actual = exc.outcome
        except (KeyError, TypeError, ValueError, OverflowError) as exc:
            actual = f"InvalidInput:{type(exc).__name__}"
        expected = vector.get("expected_outcome")
        passed = actual == expected
        print(f'{"PASS" if passed else "FAIL"} {vector.get("id")}: expected={expected} actual={actual}')
        if not passed:
            failures.append(vector.get("id"))
    if failures:
        raise SystemExit(f"FAIL: {len(failures)} vector(s) mismatched: {', '.join(failures)}")
    print(f"PASS: profile={PROFILE_ID} vectors={len(vectors)} source={SOURCE_COMMIT}")
    print("claim_ceiling=ForkEventFrontierReceiptReferenceConformanceOnly")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
