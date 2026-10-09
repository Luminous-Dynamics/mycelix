#!/usr/bin/env python3
"""Independent CIV-013 durable-adapter profile reference checker.

This checks canonical record/fork hashes and the abstract outcome vectors against
an exact source-adapter revision. It does not import the Rust crate and does not
run SQLite. PASS means reference-vector conformance only, not a production
durability or external-anchor qualification.
"""
from __future__ import annotations

import hashlib
import json
import re
import sys
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[2]
FIXTURE = ROOT / "docs/integral/civ-013-durable-witness-conformance-v1.json"
MANIFEST = ROOT / "docs/integral/civ-013-durable-witness-conformance-v1-manifest.json"
RECORD_DOMAIN = b"mycelix-civ013-durable-record-v1\0"
FORK_DOMAIN = b"mycelix-civ013-durable-fork-v1\0"
PROFILE_ID = "civ-013-durable-adapter-v1"
SOURCE_COMMIT = "2b3ea63771fb9f24c131b05ee9d69b81c085d8dc"
SQLITE_INTEGER_MAX = (1 << 63) - 1

REQUIRED_IDS = {
    "DA001-bootstrap-record",
    "DA002-successor-record",
    "DA003-fork-evidence",
    "DA004-fork-log-scope",
    "DA005-fork-evidence-successor",
    "DA006-stale-predecessor",
    "DA007-stale-expected-generation",
    "DA008-receipt-rollback",
    "DA009-receipt-equivocation",
    "DA010-invalid-tail-shape",
    "DA011-anchor-unavailable",
    "DA012-anchor-wrong-log",
    "DA013-anchor-same-generation-mismatch",
    "DA014-anchor-ahead-multiple",
    "DA015-recover-exact-prepared-successor",
    "DA016-conflicting-prepared-candidate",
    "DA017-late-finalize-idempotence",
    "DA018-reordered-fork-evidence",
    "DA019-record-digest-tampered",
    "DA020-fork-digest-tampered",
    "DA021-fork-chain-cross-log",
    "DA022-generation-signed-range-overflow",
    "DA023-receipt-sequence-signed-range-overflow",
    "DA024-recovery-equal-generation-missing-local-digest",
    "DA025-late-finalize-current-head-pointer-valid",
    "DA026-late-finalize-current-head-pointer-mismatch",
    "DA027-late-finalize-current-head-record-tampered",
    "DA028-same-generation-finalize-tampered-current-head",
    "DA029-anchor-advances-past-candidate",
    "DA030-anchor-advances-before-prepare",
}


def sha256(raw: bytes) -> bytes:
    return hashlib.sha256(raw).digest()


def uint(value: Any, width: int) -> bytes:
    if type(value) is not int or value < 0 or value >= 1 << (8 * width):
        raise ValueError(f"expected unsigned {width * 8}-bit integer")
    return value.to_bytes(width, "big")


def parse_digest(value: Any, name: str) -> bytes:
    if not isinstance(value, str) or re.fullmatch(r"[0-9a-f]{64}", value) is None:
        raise ValueError(f"{name} must be 64 lowercase hex characters")
    result = bytes.fromhex(value)
    if len(result) != 32:
        raise ValueError(f"{name} must be 32 bytes")
    return result


def length_prefixed(value: Any, name: str) -> bytes:
    if not isinstance(value, str) or not value:
        raise ValueError(f"{name} must be a non-empty string")
    raw = value.encode("utf-8")
    return uint(len(raw), 8) + raw


def optional_digest(value: Any, name: str) -> bytes:
    return b"\x00" if value is None else b"\x01" + parse_digest(value, name)


def canonical_record(record: dict[str, Any]) -> bytes:
    if record["protocol_version"] != 1:
        raise ValueError("unsupported protocol version")
    return b"".join((
        RECORD_DOMAIN,
        uint(record["protocol_version"], 2),
        uint(record["generation"], 8),
        length_prefixed(record["log_id"], "log_id"),
        length_prefixed(record["policy_version"], "policy_version"),
        parse_digest(record["anchor_digest"], "anchor_digest"),
        uint(record["receipt_sequence"], 8),
        optional_digest(record["receipt_digest"], "receipt_digest"),
        optional_digest(record["previous_record_digest"], "previous_record_digest"),
    ))


def canonical_fork(evidence: dict[str, Any]) -> bytes:
    first = parse_digest(evidence["first_record_digest"], "first_record_digest")
    conflicting = parse_digest(evidence["conflicting_record_digest"], "conflicting_record_digest")
    if first == conflicting:
        raise ValueError("fork evidence must reference distinct records")
    return b"".join((
        FORK_DOMAIN,
        length_prefixed(evidence["log_id"], "log_id"),
        uint(evidence["generation"], 8),
        first,
        conflicting,
        optional_digest(evidence["previous_evidence_digest"], "previous_evidence_digest"),
    ))


def valid_fork_chain(chain: list[dict[str, Any]]) -> bool:
    previous: str | None = None
    chain_log_id: str | None = None
    for item in chain:
        log_id = item.get("log_id")
        if not isinstance(log_id, str) or not log_id:
            return False
        if chain_log_id is None:
            chain_log_id = log_id
        elif log_id != chain_log_id:
            return False
        if item.get("previous_evidence_digest") != previous:
            return False
        try:
            calculated = sha256(canonical_fork(item)).hex()
        except (KeyError, TypeError, ValueError):
            return False
        if calculated != item.get("digest"):
            return False
        previous = calculated
    return True


def evaluate(vector: dict[str, Any], accepted: dict[str, dict[str, Any]]) -> str:
    kind = vector["kind"]

    if kind == "record" or kind == "record_negative":
        record = vector["record"]
        calculated = sha256(canonical_record(record)).hex()
        if calculated != record.get("digest"):
            return "RejectRecordDigest" if kind == "record_negative" else "INVALID_RECORD_DIGEST"
        sequence = record["receipt_sequence"]
        tail = record["receipt_digest"]
        if (sequence == 0) != (tail is None):
            return "INVALID_RECEIPT_TAIL"
        predecessor_id = vector.get("requires_predecessor")
        if not predecessor_id and (
            record["generation"] != 1 or record["previous_record_digest"] is not None
        ):
            return "INVALID_BOOTSTRAP_SHAPE"
        if predecessor_id:
            predecessor = accepted.get(predecessor_id)
            if predecessor is None:
                return "MISSING_PREDECESSOR"
            if record["generation"] != predecessor["generation"] + 1:
                return "INVALID_GENERATION"
            if record["previous_record_digest"] != predecessor["digest"]:
                return "INVALID_PREDECESSOR"
            if record["log_id"] != predecessor["log_id"]:
                return "CROSS_LOG_SUCCESSOR"
            if record["policy_version"] != predecessor["policy_version"]:
                return "POLICY_VERSION_CHANGED"
            if sequence < predecessor["receipt_sequence"] or (
                sequence == predecessor["receipt_sequence"] and tail != predecessor["receipt_digest"]
            ):
                return "INVALID_RECEIPT_HISTORY"
        if kind == "record_negative":
            return "VALID_RECORD_UNEXPECTEDLY"
        accepted[vector["id"]] = record
        return "VALID_RECORD_DIGEST"

    if kind == "fork" or kind == "fork_negative":
        evidence = vector["evidence"]
        calculated = sha256(canonical_fork(evidence)).hex()
        if calculated != evidence.get("digest"):
            return "RejectForkDigest" if kind == "fork_negative" else "INVALID_FORK_DIGEST"
        return "VALID_FORK_DIGEST"

    if kind == "transition":
        if (
            vector["accepted_generation"] != vector["expected_generation"]
            or vector["accepted_record_digest"] != vector["expected_digest"]
        ):
            return "StalePredecessor"
        candidate_gen = vector["candidate_generation"]
        if candidate_gen != vector["expected_generation"] + 1:
            return "StalePredecessor"
        if vector["candidate_previous_record_digest"] != vector["expected_digest"]:
            return "StalePredecessor"
        return "ACCEPT"

    if kind == "receipt_tail":
        current_seq = vector["previous_sequence"]
        current_digest = vector["previous_digest"]
        candidate_seq = vector["candidate_sequence"]
        candidate_digest = vector["candidate_digest"]
        if (candidate_seq == 0) != (candidate_digest is None):
            return "InvalidInput"
        if candidate_seq < current_seq:
            return "ReceiptRollback"
        if candidate_seq == current_seq and candidate_digest != current_digest:
            return "ReceiptTailEquivocation"
        return "ACCEPT"

    if kind == "recovery":
        external = vector.get("external_anchor")
        if external is None:
            return "AnchorUnavailable"
        local_log = vector.get("local_log_id")
        if local_log is not None and external.get("log_id") != local_log:
            return "ExternalAnchorMismatch"
        local_generation = vector["local_generation"]
        external_generation = external["generation"]
        local_digest = vector.get("local_record_digest")
        if external_generation < local_generation:
            return "RollbackDetected"
        if external_generation == local_generation:
            # Equal generation is not sufficient: the local accepted-head digest
            # must be present (unless this is the generation-zero genesis state),
            # well-formed, and byte-identical to the externally retained digest.
            expected_local_digest = "0" * 64 if local_generation == 0 else local_digest
            if not isinstance(expected_local_digest, str):
                return "ExternalAnchorMismatch"
            try:
                external_digest = parse_digest(external.get("record_digest"), "external record digest")
                accepted_digest = parse_digest(expected_local_digest, "local record digest")
            except (TypeError, ValueError):
                return "ExternalAnchorMismatch"
            if external_digest != accepted_digest:
                return "ExternalAnchorMismatch"
            return "ACCEPT_RECOVERY"
        prepared_generation = vector.get("prepared_generation")
        prepared_digest = vector.get("prepared_record_digest")
        prepared_previous = vector.get("prepared_previous_digest")
        if (
            prepared_generation == local_generation + 1
            and prepared_previous == local_digest
            and external_generation == prepared_generation
            and external.get("record_digest") == prepared_digest
        ):
            return "RecoverPreparedSuccessor"
        return "RollbackDetected"

    if kind == "prepared_conflict":
        if (
            vector["existing_prepared_generation"] == vector["proposed_generation"]
            and vector["existing_prepared_digest"] != vector["proposed_digest"]
        ):
            return "PreparedCandidateConflict"
        return "NO_CONFLICT"

    if kind == "late_finalize":
        exact_predecessor = (
            vector["candidate_generation"] == vector["expected_predecessor_generation"] + 1
            and vector["candidate_previous_digest"] == vector["expected_predecessor_digest"]
        )
        if (
            exact_predecessor
            and vector["current_head_generation"] > vector["candidate_generation"]
            and vector["stored_candidate_status"] == "accepted"
        ):
            return "IdempotentAcceptedHistory"
        return "StalePredecessor"

    if kind == "fork_chain":
        chain = vector["chain"]
        if not valid_fork_chain(chain):
            return "RejectCorruptForkEvidence"
        reordered = [chain[index] for index in vector["order"]]
        return "RejectCorruptForkEvidence" if not valid_fork_chain(reordered) else "VALID_FORK_CHAIN"

    if kind == "late_finalize_head_pointer":
        exact_predecessor = (
            vector["candidate_generation"] == vector["expected_predecessor_generation"] + 1
            and vector["candidate_previous_digest"] == vector["expected_predecessor_digest"]
        )
        if (
            not exact_predecessor
            or vector["current_head_generation"] <= vector["candidate_generation"]
            or vector["stored_candidate_status"] != "accepted"
        ):
            return "StalePredecessor"
        if vector.get("stored_current_head_status") != "accepted":
            return "CorruptCurrentHeadMetadata"
        try:
            metadata_digest = parse_digest(vector["metadata_head_digest"], "metadata_head_digest")
            stored_digest = parse_digest(vector["stored_current_head_digest"], "stored_current_head_digest")
        except (KeyError, TypeError, ValueError):
            return "CorruptCurrentHeadMetadata"
        if metadata_digest != stored_digest:
            return "CorruptCurrentHeadMetadata"
        return "IdempotentAcceptedHistory"

    if kind == "late_finalize_head_record":
        exact_predecessor = (
            vector["candidate_generation"] == vector["expected_predecessor_generation"] + 1
            and vector["candidate_previous_digest"] == vector["expected_predecessor_digest"]
        )
        if (
            not exact_predecessor
            or vector["current_head_generation"] <= vector["candidate_generation"]
            or vector["stored_candidate_status"] != "accepted"
        ):
            return "StalePredecessor"
        if vector.get("stored_current_head_status") != "accepted":
            return "CorruptCurrentHeadRecord"
        try:
            metadata_digest = parse_digest(vector["metadata_head_digest"], "metadata_head_digest")
            stored_digest = parse_digest(vector["stored_record_digest"], "stored_record_digest")
            calculated_digest = sha256(canonical_record(vector["current_head_record"]))
            row_digest = parse_digest(vector["current_head_record"]["digest"], "current_head_record.digest")
        except (KeyError, TypeError, ValueError):
            return "CorruptCurrentHeadRecord"
        if metadata_digest != stored_digest:
            return "CorruptCurrentHeadMetadata"
        if row_digest != stored_digest or calculated_digest != stored_digest:
            return "CorruptCurrentHeadRecord"
        return "IdempotentAcceptedHistory"

    if kind == "same_generation_finalize_head_record":
        exact_predecessor = (
            vector["candidate_generation"] == vector["expected_predecessor_generation"] + 1
            and vector["candidate_previous_digest"] == (
                vector["expected_predecessor_digest"]
                if vector["expected_predecessor_generation"] > 0
                else None
            )
        )
        if (
            not exact_predecessor
            or vector["current_head_generation"] != vector["candidate_generation"]
            or vector["stored_candidate_status"] != "accepted"
        ):
            return "StalePredecessor"
        if vector.get("stored_current_head_status") != "accepted":
            return "CorruptCurrentHeadRecord"
        try:
            metadata_digest = parse_digest(vector["metadata_head_digest"], "metadata_head_digest")
            stored_digest = parse_digest(vector["stored_record_digest"], "stored_record_digest")
            calculated_digest = sha256(canonical_record(vector["current_head_record"]))
            row_digest = parse_digest(vector["current_head_record"]["digest"], "current_head_record.digest")
        except (KeyError, TypeError, ValueError):
            return "CorruptCurrentHeadRecord"
        if metadata_digest != stored_digest:
            return "CorruptCurrentHeadMetadata"
        if row_digest != stored_digest or calculated_digest != stored_digest:
            return "CorruptCurrentHeadRecord"
        return "IdempotentAcceptedHistory"

    if kind == "anchor_ahead_after_prepare":
        if (
            type(vector.get("accepted_generation")) is not int
            or type(vector.get("candidate_generation")) is not int
            or vector["candidate_generation"] != vector["accepted_generation"] + 1
            or vector.get("stage") != "after_prepare"
            or vector.get("expected_fork_evidence") is not False
        ):
            return "InvalidInput"
        if vector["observed_anchor_log_id"] != vector["log_id"]:
            return "ExternalAnchorMismatch"
        if vector["observed_anchor_generation"] > vector["candidate_generation"]:
            return "RollbackDetected"
        if vector["observed_anchor_generation"] == vector["candidate_generation"]:
            return (
                "ACCEPT_ALREADY_ANCHORED"
                if vector["observed_anchor_digest"] == vector.get("candidate_digest")
                else "ExternalAnchorMismatch"
            )
        return "AnchorNotAhead"

    if kind == "anchor_ahead_before_prepare":
        if (
            type(vector.get("accepted_generation")) is not int
            or type(vector.get("candidate_generation")) is not int
            or vector["candidate_generation"] != vector["accepted_generation"] + 1
            or vector.get("stage") != "before_prepare"
            or vector.get("expected_candidate_persisted") is not False
            or vector.get("expected_fork_evidence") is not False
        ):
            return "InvalidInput"
        if vector["observed_anchor_log_id"] != vector["log_id"]:
            return "ExternalAnchorMismatch"
        if vector["observed_anchor_generation"] > vector["candidate_generation"]:
            return "RollbackDetected"
        if vector["observed_anchor_generation"] == vector["candidate_generation"]:
            return (
                "ACCEPT_ALREADY_ANCHORED"
                if vector["observed_anchor_digest"] == vector.get("candidate_digest")
                else "ExternalAnchorMismatch"
            )
        return "AnchorNotAhead"

    if kind == "sqlite_integer_range":
        generation = vector["record_generation"]
        sequence = vector["receipt_sequence"]
        if (
            type(generation) is not int or generation < 0
            or type(sequence) is not int or sequence < 0
        ):
            return "InvalidInput"
        if generation > SQLITE_INTEGER_MAX or sequence > SQLITE_INTEGER_MAX:
            return "RejectSqliteIntegerRange"
        return "VALID_SQLITE_INTEGER_RANGE"

    raise ValueError(f"unknown vector kind: {kind!r}")


def verify_manifest_file_bindings(manifest: dict[str, Any]) -> None:
    bound_paths = {
        "fixture_git_blob_sha": FIXTURE,
        "checker_git_blob_sha": Path(__file__).resolve(),
        "spec_git_blob_sha": ROOT / "docs/integral/civ-013-durable-witness-conformance-v1.md",
        "workflow_git_blob_sha": ROOT / ".github/workflows/civ013-durable-witness-conformance.yml",
    }
    for key, path in bound_paths.items():
        expected = manifest.get(key)
        if not isinstance(expected, str) or len(expected) != 40:
            raise SystemExit(f"FAIL: manifest {key} is missing or malformed")
        actual = git_blob_sha(path)
        if actual != expected:
            raise SystemExit(f"FAIL: {key} mismatch for {path.relative_to(ROOT)}; expected={expected} actual={actual}")


def main() -> int:
    with FIXTURE.open("r", encoding="utf-8") as handle:
        corpus = json.load(handle)
    with MANIFEST.open("r", encoding="utf-8") as handle:
        manifest = json.load(handle)

    for name, value in (
        ("manifest profile", manifest.get("profile_id")),
        ("fixture profile", corpus.get("profile_id")),
    ):
        if value != PROFILE_ID:
            raise SystemExit(f"FAIL: unexpected {name}: {value!r}")
    if corpus.get("spec_version") != PROFILE_ID or manifest.get("spec_version") != PROFILE_ID:
        raise SystemExit("FAIL: spec_version/profile mismatch")
    if corpus.get("source_adapter_commit") != SOURCE_COMMIT or manifest.get("source_adapter_commit") != SOURCE_COMMIT:
        raise SystemExit("FAIL: source adapter commit mismatch")
    if manifest.get("schema_version") != 1:
        raise SystemExit("FAIL: unsupported manifest schema_version")
    verify_manifest_file_bindings(manifest)
    if manifest.get("fixture_path") != "docs/integral/civ-013-durable-witness-conformance-v1.json":
        raise SystemExit("FAIL: manifest fixture path mismatch")
    if manifest.get("verifier_path") != "scripts/integral/verify_civ013_durable_witness_conformance_v1.py":
        raise SystemExit("FAIL: manifest verifier path mismatch")
    enc = corpus.get("encoding", {})
    if enc.get("record_domain_utf8") != "mycelix-civ013-durable-record-v1\u0000":
        raise SystemExit("FAIL: record domain mismatch")
    if enc.get("fork_domain_utf8") != "mycelix-civ013-durable-fork-v1\u0000":
        raise SystemExit("FAIL: fork domain mismatch")

    vectors = corpus.get("vectors")
    if not isinstance(vectors, list):
        raise SystemExit("FAIL: vectors is not a list")
    ids = [v.get("id") for v in vectors]
    if len(ids) != len(set(ids)) or set(ids) != REQUIRED_IDS:
        raise SystemExit("FAIL: vector identity set mismatch")
    if len(vectors) != manifest.get("expected_vector_count"):
        raise SystemExit("FAIL: manifest vector count mismatch")
    if ids != manifest.get("required_vector_ids"):
        raise SystemExit("FAIL: manifest ordered vector IDs mismatch")

    accepted: dict[str, dict[str, Any]] = {}
    failures: list[str] = []
    for vector in vectors:
        try:
            actual = evaluate(vector, accepted)
        except (KeyError, TypeError, ValueError, OverflowError) as exc:
            actual = f"ERROR:{type(exc).__name__}"
        expected = vector.get("expected_outcome")
        ok = actual == expected
        print(f"{'PASS' if ok else 'FAIL'} {vector['id']} expected={expected} actual={actual}")
        if not ok:
            failures.append(vector["id"])

    print(f"SUMMARY profile={PROFILE_ID} vectors={len(vectors)} passed={len(vectors)-len(failures)} failed={len(failures)}")
    print(f"CLAIM_CEILING {corpus.get('claim_ceiling', 'MISSING')}")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
