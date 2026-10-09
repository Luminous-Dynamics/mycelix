#!/usr/bin/env python3
"""Independent CIV-014 fork-witness reference checker.

This verifies canonical event/receipt bytes and a deterministic recovery model
against an exact producer revision. It does not make network calls and does not
prove that any production backend is durable or independently operated.
"""
from __future__ import annotations

import hashlib
import json
import os
import re
import subprocess
import sys
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[2]
FIXTURE = ROOT / "docs/integral/civ-014-fork-witness-conformance-v1.json"
MANIFEST = ROOT / "docs/integral/civ-014-fork-witness-conformance-v1-manifest.json"
SPEC = ROOT / "docs/integral/civ-014-fork-witness-conformance-v1.md"
WORKFLOW = ROOT / ".github/workflows/civ014-fork-witness-conformance.yml"
SOURCE_COMMIT = "84cbae9e16823f7947e4ce6adbd20ac9167562df"
PROFILE_ID = "civ-014-fork-witness-v1"
EVENT_DOMAIN = b"mycelix-civ014-fork-event-v1\0"
RECEIPT_DOMAIN = b"mycelix-civ014-fork-receipt-v1\0"
ZERO = bytes(32)
U64_MAX = (1 << 64) - 1
RESTORE_HARD_LIMIT = 100_000

REQUIRED_IDS = [
    "FW001-valid-event-at-empty-frontier",
    "FW002-valid-event-chained-to-prior-event",
    "FW003-valid-single-step-append-receipt",
    "FW004-receipt-cannot-skip-frontier-count",
    "FW005-empty-frontier-requires-zero-tail",
    "FW006-nonempty-frontier-rejects-zero-tail",
    "FW007-identical-competing-records-rejected",
    "FW008-event-digest-tampering-rejected",
    "FW009-ambiguous-append-readback-is-idempotent",
    "FW010-pending-retry-appends-once",
    "FW011-remote-commit-before-local-finalize",
    "FW012-restore-refuses-missing-payload",
    "FW013-restore-refuses-moving-frontier",
    "FW014-hard-restore-limit-before-remote-read",
    "FW015-caller-restore-limit-before-write",
    "FW016-full-erasure-restores-exact-ordered-prefix",
    "FW017-conflicting-remote-prefix-cannot-overwrite-local",
    "FW018-remote-behind-local-history-fails-closed",
    "FW019-fork-observation-does-not-advance-accepted-head",
    "FW020-witness-unavailable-preserves-pending",
    "FW021-stale-predecessor-rejects-competing-append",
    "FW022-restore-refuses-missing-receipt",
    "FW023-restore-refuses-broken-event-chain",
    "FW024-orphaned-local-event-rows-are-corruption",
    "FW025-receipt-digest-tampering-rejected",
    "FW026-unavailable-first-lookup-keeps-local-pending",
    "FW027-historical-retry-after-later-successor-is-idempotent",
    "FW028-anchored-event-missing-remotely-is-not-replayed",
]


def fail(message: str) -> None:
    raise SystemExit(f"FAIL: {message}")


def unsigned(value: Any, width: int) -> bytes:
    if type(value) is not int or value < 0 or value >= 1 << (width * 8):
        raise ValueError(f"expected unsigned {width * 8}-bit integer")
    return value.to_bytes(width, "big")


def parse_digest(value: Any, field: str) -> bytes:
    if not isinstance(value, str) or re.fullmatch(r"[0-9a-f]{64}", value) is None:
        raise ValueError(f"{field} must be 64 lowercase hexadecimal characters")
    return bytes.fromhex(value)


def field_bytes(value: Any, field: str) -> bytes:
    if not isinstance(value, str) or not value:
        raise ValueError(f"{field} must be a non-empty string")
    raw = value.encode("utf-8")
    return unsigned(len(raw), 8) + raw


def event_bytes(event: dict[str, Any]) -> bytes:
    schema = event["schema_version"]
    if schema != 1:
        raise ValueError("unsupported event schema")
    log_id = event["log_id"]
    if not isinstance(log_id, str) or not log_id:
        raise ValueError("empty log ID")
    previous_count = event["previous_event_count"]
    previous_digest = parse_digest(event["previous_event_digest"], "previous_event_digest")
    accepted_generation = event["accepted_generation"]
    accepted_digest = parse_digest(event["accepted_head_digest"], "accepted_head_digest")
    fork_generation = event["fork_generation"]
    first = parse_digest(event["first_record_digest"], "first_record_digest")
    conflicting = parse_digest(event["conflicting_record_digest"], "conflicting_record_digest")
    if type(previous_count) is not int or previous_count < 0:
        raise ValueError("invalid previous event count")
    if (previous_count == 0 and previous_digest != ZERO) or (
        previous_count > 0 and previous_digest == ZERO
    ):
        raise ValueError("previous frontier count/tail mismatch")
    if type(accepted_generation) is not int or accepted_generation < 0:
        raise ValueError("invalid accepted generation")
    if accepted_generation == 0 and accepted_digest != ZERO:
        raise ValueError("non-zero genesis accepted-head digest")
    if type(fork_generation) is not int or fork_generation == 0:
        raise ValueError("fork generation must be positive")
    if fork_generation > accepted_generation + 1:
        raise ValueError("fork generation is ahead of the next accepted generation")
    if first == conflicting:
        raise ValueError("identical competing record digests")

    return b"".join((
        EVENT_DOMAIN,
        unsigned(schema, 2),
        field_bytes(log_id, "log_id"),
        unsigned(event["witness_epoch"], 8),
        unsigned(previous_count, 8),
        previous_digest,
        unsigned(accepted_generation, 8),
        accepted_digest,
        unsigned(fork_generation, 8),
        first,
        conflicting,
    ))


def validate_event(event: dict[str, Any]) -> str:
    try:
        actual = hashlib.sha256(event_bytes(event)).hexdigest()
    except ValueError as exc:
        message = str(exc)
        if "identical competing" in message:
            return "RejectIdenticalCompetitors"
        return "RejectMalformedEvent"
    if actual != event.get("event_digest"):
        return "RejectEventDigest"
    return "ValidEvent"


def frontier_valid(frontier: dict[str, Any]) -> bool:
    try:
        count = frontier["event_count"]
        tail = parse_digest(frontier["tail_digest"], "tail_digest")
        if type(count) is not int or count < 0 or count > U64_MAX:
            return False
        if not isinstance(frontier.get("log_id"), str) or not frontier["log_id"]:
            return False
        unsigned(frontier["witness_epoch"], 8)
        return (count == 0 and tail == ZERO) or (count > 0 and tail != ZERO)
    except (KeyError, TypeError, ValueError):
        return False


def receipt_bytes(receipt: dict[str, Any]) -> bytes:
    return b"".join((
        RECEIPT_DOMAIN,
        field_bytes(receipt["log_id"], "log_id"),
        unsigned(receipt["witness_epoch"], 8),
        unsigned(receipt["previous_event_count"], 8),
        parse_digest(receipt["previous_event_digest"], "previous_event_digest"),
        parse_digest(receipt["event_digest"], "event_digest"),
        unsigned(receipt["frontier_after_count"], 8),
        parse_digest(receipt["frontier_after_digest"], "frontier_after_digest"),
    ))


def validate_receipt(event: dict[str, Any], receipt: dict[str, Any] | None) -> str:
    if receipt is None:
        return "RejectMissingReceipt"
    event_status = validate_event(event)
    if event_status != "ValidEvent":
        return event_status
    try:
        previous_count = event["previous_event_count"]
        previous_digest = parse_digest(event["previous_event_digest"], "previous_event_digest")
        if receipt.get("log_id") != event.get("log_id") or receipt.get("witness_epoch") != event.get("witness_epoch"):
            return "RejectReceiptScope"
        if receipt.get("event_digest") != event.get("event_digest"):
            return "RejectReceiptEventBinding"
        if receipt.get("previous_event_count") != previous_count or receipt.get("previous_event_digest") != event.get("previous_event_digest"):
            return "RejectReceiptPreviousFrontier"
        if (previous_count == 0 and previous_digest != ZERO) or (previous_count > 0 and previous_digest == ZERO):
            return "RejectReceiptShape"
        if previous_count >= U64_MAX or receipt.get("frontier_after_count") != previous_count + 1:
            return "RejectReceiptShape"
        if receipt.get("frontier_after_digest") != event.get("event_digest"):
            return "RejectReceiptShape"
        calculated = hashlib.sha256(receipt_bytes(receipt)).hexdigest()
    except (KeyError, TypeError, ValueError):
        return "RejectReceiptShape"
    if calculated != receipt.get("receipt_digest"):
        return "RejectReceiptDigest"
    return "ValidReceipt"


def restore_outcome(vector: dict[str, Any]) -> tuple[str, int]:
    before = vector["remote_frontier_before"]
    after = vector["remote_frontier_after"]
    remote_count = before["event_count"]
    maximum = U64_MAX if vector.get("max_events") == "u64::MAX" else vector["max_events"]
    if type(remote_count) is not int or remote_count < 0 or remote_count > U64_MAX:
        return "RejectMalformedFrontier", 0
    if remote_count > maximum:
        return "RejectCallerRestoreLimit", 0
    hard_limit = vector.get("hard_limit", RESTORE_HARD_LIMIT)
    if remote_count > min(hard_limit, RESTORE_HARD_LIMIT):
        return "RejectHardRestoreLimit", 0

    remote_events = vector.get("remote_events", [])
    if len(remote_events) < remote_count:
        return "RejectMissingPayload", 0
    if len(remote_events) > remote_count:
        return "RejectEventChain", 0
    expected_count = 0
    expected_tail = "0" * 64
    calculated_digests: list[str] = []
    for item in remote_events:
        event = item.get("event")
        if event is None:
            return "RejectMissingPayload", 0
        if validate_event(event) != "ValidEvent":
            return "RejectEventChain", 0
        if event.get("log_id") != vector["log_id"] or event.get("witness_epoch") != vector["witness_epoch"]:
            return "RejectEventChain", 0
        if event.get("previous_event_count") != expected_count or event.get("previous_event_digest") != expected_tail:
            return "RejectEventChain", 0
        receipt = item.get("receipt")
        receipt_status = validate_receipt(event, receipt)
        if receipt_status != "ValidReceipt":
            return receipt_status, 0
        if receipt.get("frontier_after_count") != expected_count + 1 or receipt.get("frontier_after_digest") != event["event_digest"]:
            return "RejectEventChain", 0
        expected_count += 1
        expected_tail = event["event_digest"]
        calculated_digests.append(expected_tail)

    if expected_count != remote_count or expected_tail != before["tail_digest"]:
        return "RejectEventChain", 0
    if after != before:
        return "RejectFrontierChanged", 0

    local_present = vector.get("local_frontier_present", False)
    local_digests = vector.get("local_event_digests", [])
    local_count = vector.get("local_frontier_count", 0)
    if not local_present and local_digests:
        return "RejectCorruptLocalJournal", 0
    if type(local_count) is not int or local_count < 0:
        return "RejectCorruptLocalJournal", 0
    if local_count > remote_count:
        return "RejectRemoteBehindLocal", 0
    if local_count > len(local_digests):
        return "RejectCorruptLocalJournal", 0
    if len(local_digests) > local_count:
        return "RejectCorruptLocalJournal", 0
    if local_digests != calculated_digests[:local_count]:
        return "RejectLocalPrefixConflict", 0

    if local_count == remote_count:
        return ("RestoreIdempotentPrefix", 0)
    if local_count == 0 and remote_count > 0:
        return ("RestoreFullPrefix", remote_count)
    return ("RestoreExtendedPrefix", remote_count - local_count)


def evaluate(vector: dict[str, Any]) -> tuple[str, int | None]:
    kind = vector["kind"]
    if kind == "event":
        return validate_event(vector["event"]), None
    if kind == "frontier":
        return ("ValidFrontier" if frontier_valid(vector["frontier"]) else "RejectFrontier", None)
    if kind == "receipt":
        return validate_receipt(vector["event"], vector.get("receipt")), None
    if kind == "ambiguous_append":
        event = vector["event"]
        receipt = vector.get("receipt")
        if vector.get("response") == "AppendIndeterminate" and vector.get("readback_matches") is True:
            if validate_receipt(event, receipt) == "ValidReceipt" and vector.get("remote_event_count") == event["previous_event_count"] + 1:
                return "ResolveAmbiguousExactlyOnce", vector["remote_event_count"]
        return "RemainPendingNoSuccess", None
    if kind == "pending_before_remote_read":
        event = vector["event"]
        if validate_event(event) != "ValidEvent":
            return validate_event(event), 0
        if vector.get("witness_available") is not True:
            if vector.get("local_pending_persisted") is True:
                return "PendingNoSuccess", 1
            return "PendingNoSuccess", 0
        return "UnexpectedAvailableWitness", 0
    if kind == "historical_retry":
        event = vector["event"]
        receipt = vector.get("receipt")
        if vector.get("local_anchored") is not True:
            return "RejectUnanchoredLocalState", 0
        if validate_event(event) != "ValidEvent":
            return validate_event(event), 0
        if validate_receipt(event, receipt) != "ValidReceipt":
            return "RejectRemoteReceipt", 0
        if vector.get("local_receipt_digest") != receipt.get("receipt_digest"):
            return "RejectLocalReceiptConflict", 0
        if vector.get("remote_event_present") is not True:
            return "RejectRollbackDetected", 0
        if vector.get("remote_event") != event or vector.get("remote_receipt") != receipt:
            return "RejectRemoteReceipt", 0
        event_sequence = event["previous_event_count"] + 1
        if vector.get("remote_frontier_count", -1) < event_sequence:
            return "RejectRollbackDetected", 0
        if vector.get("remote_event_at_sequence_digest") != event["event_digest"]:
            return "RejectRemoteReceipt", 0
        if vector.get("remote_frontier_count") == event_sequence and vector.get("remote_frontier_digest") != event["event_digest"]:
            return "RejectFrontierConflict", 0
        return "ReturnExistingReceipt", 0
    if kind == "pending_recovery":
        event = vector["event"]
        if vector.get("witness_available") is not True:
            return "PendingNoSuccess", 0
        if vector.get("remote_event_present") is True:
            if vector.get("remote_event") != event or validate_receipt(event, vector.get("remote_receipt")) != "ValidReceipt":
                return "RejectRemoteReceipt", 0
            if vector.get("remote_frontier_count") != event["previous_event_count"] + 1 or vector.get("remote_frontier_digest") != event["event_digest"]:
                return "RejectFrontierConflict", 0
            return "FinalizeExistingReceipt", 1
        if vector.get("remote_frontier_count") == event["previous_event_count"] and vector.get("remote_frontier_digest") == event["previous_event_digest"]:
            return "AppendOnce", 1
        return "RejectFrontierConflict", 0
    if kind == "append_attempt":
        event = vector["event"]
        if validate_event(event) != "ValidEvent":
            return validate_event(event), 0
        if vector.get("remote_frontier_count") != event["previous_event_count"] or vector.get("remote_frontier_digest") != event["previous_event_digest"]:
            return "RejectFrontierConflict", 0
        return "AppendAllowed", 1
    if kind == "restore":
        return restore_outcome(vector)
    if kind == "accepted_head_invariant":
        return ("AcceptedHeadUnchanged" if vector.get("before") == vector.get("after") else "AcceptedHeadChanged", None)
    raise ValueError(f"unknown vector kind: {kind}")


def git_blob_sha(path: Path) -> str:
    content = path.read_bytes()
    return hashlib.sha1(b"blob " + str(len(content)).encode("ascii") + b"\0" + content).hexdigest()


def verify_exact_sources(manifest: dict[str, Any]) -> None:
    source_raw = os.environ.get("CIV014_SOURCE_TREE", "")
    if not source_raw:
        fail("CIV014_SOURCE_TREE must point to the exact detached producer checkout")
    source_tree = Path(source_raw).resolve()
    actual_commit = subprocess.check_output(
        ["git", "-C", str(source_tree), "rev-parse", "HEAD"], text=True
    ).strip()
    if actual_commit != SOURCE_COMMIT or manifest.get("source_adapter_commit") != SOURCE_COMMIT:
        fail(f"producer commit mismatch: expected {SOURCE_COMMIT}, got {actual_commit}")
    source_paths = {
        "source_adapter_blob_sha": source_tree / "crates/domains/symthaea-civ-witness/src/lib.rs",
        "source_fork_witness_blob_sha": source_tree / "crates/domains/symthaea-civ-witness/src/fork_witness.rs",
        "source_workspace_manifest_blob_sha": source_tree / "Cargo.toml",
        "source_lockfile_blob_sha": source_tree / "Cargo.lock",
        "source_toolchain_blob_sha": source_tree / "rust-toolchain.toml",
        "source_crate_manifest_blob_sha": source_tree / "crates/domains/symthaea-civ-witness/Cargo.toml",
    }
    for key, path in source_paths.items():
        if not path.is_file():
            fail(f"missing producer pin source: {path}")
        actual = git_blob_sha(path)
        if actual != manifest.get(key):
            fail(f"{key} mismatch: expected {manifest.get(key)}, got {actual}")
    toolchain = (source_tree / "rust-toolchain.toml").read_text(encoding="utf-8")
    match = re.search(r'^\s*channel\s*=\s*"([^"]+)"', toolchain, re.MULTILINE)
    if match is None or match.group(1) != manifest.get("source_rust_version"):
        fail("producer rust-toolchain.toml does not match the manifest")

    workspace_manifest = (source_tree / "Cargo.toml").read_text(encoding="utf-8")
    required_rusqlite_line = (
        f'rusqlite = {{ version = "{manifest.get("source_rusqlite_version")}", '
        'features = ["bundled", "fallible_uint"] }'
    )
    if workspace_manifest.count(required_rusqlite_line) != 1:
        fail("workspace rusqlite version/features do not match the profile")

    lock_text = (source_tree / "Cargo.lock").read_text(encoding="utf-8")
    package_blocks = re.split(r"\n(?=\[\[package\]\]\n)", lock_text)

    def locked_package_version(name: str) -> str:
        found: list[str] = []
        for block in package_blocks:
            match = re.search(
                rf'^name = "{re.escape(name)}"\nversion = "([^"]+)"',
                block,
                re.MULTILINE,
            )
            if match is not None:
                found.append(match.group(1))
        if len(found) != 1:
            fail(f"expected exactly one locked package {name}, found {found}")
        return found[0]

    if locked_package_version("rusqlite") != manifest.get("source_rusqlite_version"):
        fail("Cargo.lock rusqlite version does not match the manifest")
    if locked_package_version("libsqlite3-sys") != manifest.get("source_libsqlite3_sys_version"):
        fail("Cargo.lock libsqlite3-sys version does not match the manifest")

    source_lib = source_paths["source_adapter_blob_sha"].read_text(encoding="utf-8")
    minimum_match = re.search(
        r"const MIN_SQLITE_VERSION_NUMBER: i32 = ([0-9_]+);", source_lib
    )
    bundled_match = re.search(
        r"const EXPECTED_BUNDLED_SQLITE_VERSION_NUMBER: i32 = ([0-9_]+);",
        source_lib,
    )
    if minimum_match is None or int(minimum_match.group(1).replace("_", "")) != manifest.get("source_min_sqlite_version_number"):
        fail("producer SQLite minimum version does not match the manifest")
    if bundled_match is None or int(bundled_match.group(1).replace("_", "")) != manifest.get("source_bundled_sqlite_version_number"):
        fail("producer bundled SQLite version number does not match the manifest")
    parts = tuple(int(part) for part in str(manifest.get("source_bundled_sqlite_version", "")).split("."))
    if len(parts) != 3 or parts[0] * 1_000_000 + parts[1] * 1_000 + parts[2] != manifest.get("source_bundled_sqlite_version_number"):
        fail("bundled SQLite version text/number encoding is inconsistent")
    if "validate_sqlite_runtime_version(rusqlite::version_number())?;" not in source_lib:
        fail("adapter connection is missing the linked SQLite runtime guard")
    if "fn rejects_sqlite_runtime_below_wal_reset_fix_floor()" not in source_lib:
        fail("adapter is missing the SQLite below-floor regression test")

    required_adapter_markers = (
        "const DB_SCHEMA_VERSION: i64 = 3;",
        "const MAX_FORK_WITNESS_RESTORE_EVENTS: u64 = 100_000;",
        "pub fn append_fork_witness_event(",
        "pub fn retry_pending_fork_witness_event(",
        "pub fn restore_fork_witness_journal(",
        "fn witness_unavailable_before_readback_preserves_local_pending_event()",
        "fn historical_fork_witness_retry_is_idempotent_after_a_later_successor()",
        "fn locally_anchored_event_is_not_replayed_if_remote_history_disappears()",
    )
    for marker in required_adapter_markers:
        if marker not in source_lib:
            fail(f"producer adapter is missing required CIV-014 source marker: {marker}")

    adapter_test_count = len(re.findall(r"#\[test\]\s*fn ", source_lib))
    if adapter_test_count != manifest.get("source_adapter_test_definition_count"):
        fail(
            f"adapter test-definition count mismatch: expected "
            f"{manifest.get('source_adapter_test_definition_count')}, got {adapter_test_count}"
        )

    source_fork = source_paths["source_fork_witness_blob_sha"].read_text(encoding="utf-8")
    protocol_test_count = len(re.findall(r"#\[test\]\s*fn ", source_fork))
    if protocol_test_count != manifest.get("source_protocol_test_definition_count"):
        fail(
            f"protocol test-definition count mismatch: expected "
            f"{manifest.get('source_protocol_test_definition_count')}, got {protocol_test_count}"
        )
    required_protocol_markers = (
        "AppendIndeterminate",
        "fn ambiguous_remote_append_is_resolved_without_duplicate_evidence()",
        "fn nonempty_frontier_rejects_zero_tail_digest()",
    )
    for marker in required_protocol_markers:
        if marker not in source_fork:
            fail(f"producer protocol is missing required CIV-014 source marker: {marker}")


def main() -> int:
    if sys.version_info < (3, 10):
        fail("CPython >= 3.10 is required")
    try:
        fixture = json.loads(FIXTURE.read_text(encoding="utf-8"))
        manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        fail(f"cannot read profile assets: {exc}")

    if manifest.get("profile_id") != PROFILE_ID or fixture.get("profile_id") != PROFILE_ID:
        fail("profile ID mismatch")
    if fixture.get("source_adapter_commit") != SOURCE_COMMIT:
        fail("fixture source commit mismatch")
    if manifest.get("source_adapter_commit") != SOURCE_COMMIT:
        fail("manifest source commit mismatch")
    vectors = fixture.get("vectors")
    if not isinstance(vectors, list):
        fail("fixture vectors must be an array")
    ids = [vector.get("id") for vector in vectors]
    if len(ids) != len(set(ids)):
        fail("duplicate vector IDs")
    if ids != REQUIRED_IDS or manifest.get("required_vector_ids") != REQUIRED_IDS:
        fail("ordered fixture/manifest IDs do not equal the frozen required ID list")
    if len(vectors) != len(REQUIRED_IDS) or manifest.get("expected_vector_count") != len(REQUIRED_IDS):
        fail("vector count mismatch")

    asset_paths = {
        "fixture_git_blob_sha": FIXTURE,
        "checker_git_blob_sha": Path(__file__).resolve(),
        "spec_git_blob_sha": SPEC,
        "workflow_git_blob_sha": WORKFLOW,
    }
    for key, path in asset_paths.items():
        actual = git_blob_sha(path)
        if actual != manifest.get(key):
            fail(f"{key} mismatch: expected {manifest.get(key)}, got {actual}")
    if manifest.get("source_repository") != "Luminous-Dynamics/symthaea":
        fail("unexpected source repository")
    if manifest.get("claim_ceiling") != "ForkWitnessReferenceConformanceOnly":
        fail("unexpected claim ceiling")
    if fixture.get("claim_ceiling") != manifest.get("claim_ceiling"):
        fail("fixture/manifest claim ceilings differ")
    if fixture.get("encoding", {}).get("restore_hard_limit_events") != RESTORE_HARD_LIMIT:
        fail("fixture restore hard limit differs from checker policy")
    if manifest.get("restore_hard_limit_events") != RESTORE_HARD_LIMIT:
        fail("manifest restore hard limit differs from checker policy")
    if fixture.get("source_repository") != manifest.get("source_repository"):
        fail("fixture/manifest source repository mismatch")
    if manifest.get("source_crate_manifest_blob_sha") is None:
        fail("crate manifest blob pin missing")
    verify_exact_sources(manifest)

    failures = 0
    for vector in vectors:
        try:
            observed, mutations = evaluate(vector)
        except Exception as exc:  # malformed vectors must fail the checker, not pass by exception
            observed, mutations = f"CHECKER_ERROR:{type(exc).__name__}:{exc}", None
        expected = vector.get("expected_outcome")
        if observed != expected:
            print(f"FAIL {vector.get('id')}: expected={expected} observed={observed}")
            failures += 1
            continue
        if "expected_local_mutations" in vector and mutations != vector["expected_local_mutations"]:
            print(
                f"FAIL {vector.get('id')}: expected_local_mutations="
                f"{vector['expected_local_mutations']} observed={mutations}"
            )
            failures += 1
            continue
        print(f"PASS {vector['id']}: {observed}")
    if failures:
        fail(f"{failures}/{len(vectors)} reference vectors failed")
    print(f"PASS: {len(vectors)}/{len(vectors)} CIV-014 reference vectors matched")
    print("claim_ceiling=ForkWitnessReferenceConformanceOnly")
    print("rust_execution=separate_exact_head_workflow_step")
    print("production_independent_witness=not_qualified")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
