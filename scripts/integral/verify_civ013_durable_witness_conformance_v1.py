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
SOURCE_COMMIT = "b5a93838272e37f856f78e0e415bcd52f557ba7a"
SQLITE_INTEGER_MAX = (1 << 63) - 1
SQLITE_MINIMUM_VERSION_NUMBER = 3_051_003

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
    "DA031-fork-evidence-tail-truncated",
    "DA032-fork-evidence-tail-pointer-tampered",
    "DA033-fork-evidence-append-refuses-corrupt-tail",
    "DA034-legacy-fork-meta-migration-valid",
    "DA035-legacy-fork-meta-migration-rejects-corruption",
    "DA036-startup-integrity-detects-fork-tail-truncation",
    "DA037-subprocess-crash-after-anchor-commit-recovers-exact-prepared",
    "DA038-unprovisioned-persistent-anchor-fails-closed",
    "DA039-sqlite-runtime-minimum-accepted",
    "DA040-sqlite-runtime-below-minimum-rejected",
    "DA041-coordinated-local-fork-erasure-outside-claim",
    "DA042-recovery-records-same-generation-divergence",
    "DA043-recovery-genesis-mismatch-not-a-record",
    "DA044-receipt-sequence-overflow-is-state-mutation-free",
    "DA045-generation-overflow-before-anchor-read",
    "DA046-integrity-check-uses-one-snapshot-during-concurrent-advance",
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

    if kind == "legacy_fork_meta_migration":
        chain = vector.get("stored_chain")
        before = vector.get("schema_user_version")
        after = vector.get("expected_schema_user_version_after")
        if (
            type(before) is not int
            or vector.get("has_existing_adapter_schema") is not True
            or vector.get("metadata_present") is not False
            or type(after) is not int
        ):
            return "InvalidInput"
        if before not in (0, 1):
            return "UnsupportedSchemaVersion"
        chain_valid = isinstance(chain, list) and valid_fork_chain(chain)
        count_matches = isinstance(chain, list) and vector.get("stored_count") == len(chain)
        if not chain_valid or not count_matches:
            if after != before:
                return "SchemaVersionChangedOnFailedMigration"
            return "CorruptForkEvidence"
        if after != 2:
            return "InvalidInput"
        return "MigrateLegacyForkMetadata"

    if kind == "semantic_integrity_check_fork_tail":
        chain = vector.get("stored_chain")
        if not isinstance(chain, list) or not valid_fork_chain(chain):
            return "CorruptForkEvidence"
        if vector.get("stored_count") != len(chain):
            return "CorruptForkEvidence"
        if vector.get("metadata_count") != vector.get("stored_count"):
            return "CorruptForkEvidence"
        try:
            tail = parse_digest(vector["metadata_tail_digest"], "metadata_tail_digest")
            actual_tail = parse_digest(chain[-1]["digest"], "stored chain tail")
        except (KeyError, IndexError, TypeError, ValueError):
            return "CorruptForkEvidence"
        if tail != actual_tail or vector.get("checks_all_log_tables") is not True:
            return "CorruptForkEvidence"
        return "VALID_SEMANTIC_INTEGRITY"

    if kind in {"fork_evidence_tail_integrity", "fork_evidence_append_after_corruption"}:
        chain = vector.get("stored_chain")
        corrupt = not isinstance(chain, list) or not valid_fork_chain(chain)
        if not corrupt:
            if vector.get("stored_count") != len(chain):
                corrupt = True
            elif vector.get("metadata_count") != vector.get("stored_count"):
                corrupt = True
            else:
                try:
                    tail = parse_digest(vector["metadata_tail_digest"], "metadata_tail_digest")
                    actual_tail = parse_digest(chain[-1]["digest"], "stored chain tail")
                    corrupt = tail != actual_tail
                except (KeyError, IndexError, TypeError, ValueError):
                    corrupt = True
        if kind == "fork_evidence_tail_integrity":
            return "CorruptForkEvidence" if corrupt else "VALID_FORK_EVIDENCE_TAIL"
        if not corrupt:
            return "VALID_FORK_EVIDENCE_TAIL"
        if vector.get("rows_after_attempt") != vector.get("rows_before_append"):
            return "ForkEvidenceAppendUnexpectedlyChangedState"
        return "CorruptForkEvidence"

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

    if kind == "unprovisioned_test_anchor":
        if (
            vector.get("row_present") is not False
            or vector.get("missing_row_means_genesis") is not False
            or vector.get("adapter_fallback_generation") is not None
            or vector.get("adapter_fallback_digest") is not None
        ):
            return "UnsafeImplicitGenesis"
        return "AnchorUnavailable"

    if kind == "subprocess_crash_recovery":
        accepted = vector.get("accepted_generation")
        candidate = vector.get("candidate_generation")
        try:
            accepted_digest = parse_digest(vector.get("accepted_digest"), "accepted_digest")
            candidate_digest = parse_digest(vector.get("candidate_digest"), "candidate_digest")
            anchor_digest = parse_digest(
                vector.get("external_anchor_digest_after_child"), "external_anchor_digest_after_child"
            )
            local_digest = parse_digest(
                vector.get("local_head_digest_after_child"), "local_head_digest_after_child"
            )
            prepared_digest = parse_digest(
                vector.get("prepared_digest_after_child"), "prepared_digest_after_child"
            )
            recovered_digest = parse_digest(vector.get("recovered_digest"), "recovered_digest")
        except (TypeError, ValueError):
            return "InvalidInput"
        if (
            type(accepted) is not int or type(candidate) is not int
            or candidate != accepted + 1
            or vector.get("child_exit_code") != 86
            or vector.get("external_anchor_generation_after_child") != candidate
            or anchor_digest != candidate_digest
            or vector.get("local_head_generation_after_child") != accepted
            or local_digest != accepted_digest
            or vector.get("prepared_generation_after_child") != candidate
            or prepared_digest != candidate_digest
            or vector.get("prepared_status_after_child") != 0
            or vector.get("recovered_in_distinct_process") is not True
            or vector.get("recovery_exit_code") != 0
            or vector.get("recovered_generation") != candidate
            or recovered_digest != candidate_digest
            or vector.get("fork_evidence_count_after_crash") != 0
            or vector.get("pre_recovery_split_state_asserted") is not True
            or vector.get("test_anchor_is_same_host_fixture") is not True
        ):
            return "SubprocessCrashStateMismatch"
        return "RecoverExactPreparedSuccessor"

    if kind == "recovery_genesis_mismatch":
        local_generation = vector.get("local_accepted_generation")
        anchor_generation = vector.get("external_anchor_generation")
        count = vector.get("expected_fork_evidence_count_after")
        if (
            type(local_generation) is not int or local_generation != 0
            or type(anchor_generation) is not int or anchor_generation != 0
            or type(count) is not int or count != 0
        ):
            return "InvalidInput"
        try:
            local_digest = parse_digest(vector.get("local_accepted_digest"), "local_accepted_digest")
            anchor_digest = parse_digest(vector.get("external_anchor_digest"), "external_anchor_digest")
        except (TypeError, ValueError):
            return "InvalidInput"
        if (
            not isinstance(vector.get("log_id"), str) or not vector["log_id"]
            or vector.get("external_anchor_log_id") != vector.get("log_id")
            or local_digest != bytes(32)
            or anchor_digest == bytes(32)
            or vector.get("expected_recovery_error") != "ExternalAnchorMismatch"
            or vector.get("accepted_head_unchanged") is not True
            or vector.get("prepared_candidate_created") is not False
            or vector.get("expected_fork_row_inserted") is not False
        ):
            return "GenesisForkContractMismatch"
        return "ExternalAnchorMismatchWithoutForkEvidence"

    if kind == "recovery_same_generation_fork":
        local_generation = vector.get("local_accepted_generation")
        anchor_generation = vector.get("external_anchor_generation")
        fork_count = vector.get("expected_fork_evidence_count_after")
        retry_count = vector.get("expected_identical_retry_count")
        if (
            type(local_generation) is not int or local_generation <= 0
            or type(anchor_generation) is not int
            or type(fork_count) is not int
            or type(retry_count) is not int
        ):
            return "InvalidInput"
        try:
            local_digest = parse_digest(vector.get("local_accepted_digest"), "local_accepted_digest")
            anchor_digest = parse_digest(vector.get("external_anchor_digest"), "external_anchor_digest")
            first_digest = parse_digest(vector.get("expected_first_fork_digest"), "expected_first_fork_digest")
            conflicting_digest = parse_digest(vector.get("expected_conflicting_fork_digest"), "expected_conflicting_fork_digest")
        except (TypeError, ValueError):
            return "InvalidInput"
        if (
            not isinstance(vector.get("log_id"), str) or not vector["log_id"]
            or vector.get("external_anchor_log_id") != vector.get("log_id")
            or anchor_generation != local_generation
            or anchor_digest == local_digest
            or first_digest != anchor_digest
            or conflicting_digest != local_digest
            or vector.get("expected_recovery_error") != "ExternalAnchorMismatch"
            or fork_count != 1
            or retry_count != 2
            or vector.get("accepted_head_unchanged") is not True
            or vector.get("prepared_candidate_created") is not False
        ):
            return "SameGenerationRecoveryForkContractMismatch"
        return "ExternalAnchorMismatchAndForkEvidenceRecorded"

    if kind == "fork_evidence_full_erasure_boundary":
        if (
            vector.get("local_fork_evidence_rows_present") is not False
            or vector.get("local_fork_meta_present") is not False
            or vector.get("external_anchor_has_fork_frontier") is not False
            or vector.get("external_anchor_tracks_accepted_head_only") is not True
            or vector.get("claim_includes_hostile_full_database_rewrite") is not False
        ):
            return "ForkErasureClaimBoundaryMismatch"
        return "UnanchoredForkEvidenceErasure"

    if kind == "sqlite_runtime_minimum":
        actual = vector.get("reported_version_number")
        minimum = vector.get("minimum_version_number")
        if (
            type(actual) is not int or type(minimum) is not int
            or actual < 0 or minimum <= 0
            or vector.get("source_rusqlite_version") != "0.40.2"
            or vector.get("source_libsqlite3_sys_version") != "0.38.2"
            or vector.get("pinned_bundled_sqlite_version") != "3.53.2"
            or vector.get("bundled") is not True
        ):
            return "InvalidInput"
        version_text = vector.get("reported_version")
        if not isinstance(version_text, str) or re.fullmatch(r"\d+\.\d+\.\d+", version_text) is None:
            return "InvalidInput"
        major, minor, patch = (int(part) for part in version_text.split("."))
        if actual != major * 1_000_000 + minor * 1_000 + patch:
            return "RuntimeVersionEncodingMismatch"
        return "AcceptSqliteRuntimeVersion" if actual >= minimum else "UnsupportedSqliteVersion"

    if kind == "generation_range_guard":
        expected_generation = vector.get("expected_generation")
        sqlite_max = vector.get("sqlite_integer_max")
        if (
            type(expected_generation) is not int or type(sqlite_max) is not int
            or expected_generation < 0 or sqlite_max != SQLITE_INTEGER_MAX
        ):
            return "InvalidInput"
        if expected_generation + 1 <= sqlite_max:
            return "InvalidInput"
        if (
            vector.get("expected_error") != "GenerationOverflow"
            or vector.get("input_validation_before_recovery") is not True
            or vector.get("anchor_reads_after_attempt") != 0
            or vector.get("external_anchor_unchanged") is not True
            or vector.get("local_accepted_records_after") != 0
            or vector.get("local_prepared_records_after") != 0
            or vector.get("integrity_check_after") != "ok"
        ):
            return "UnexpectedStateMutation"
        return "RejectBeforeStateMutation"

    if kind == "receipt_sequence_range_guard":
        sequence = vector.get("receipt_sequence")
        sqlite_max = vector.get("sqlite_integer_max")
        if (
            type(sequence) is not int or type(sqlite_max) is not int
            or sqlite_max != SQLITE_INTEGER_MAX
            or sequence <= sqlite_max
        ):
            return "InvalidInput"
        if (
            vector.get("expected_error") != "receipt sequence exceeds SQLite INTEGER range"
            or vector.get("input_validation_before_recovery") is not True
            or vector.get("anchor_reads_after_attempt") != 0
            or vector.get("external_anchor_unchanged") is not True
            or vector.get("local_accepted_records_after") != 0
            or vector.get("local_prepared_records_after") != 0
            or vector.get("integrity_check_after") != "ok"
        ):
            return "UnexpectedStateMutation"
        return "RejectBeforeStateMutation"

    if kind == "integrity_check_snapshot_consistency":
        if (
            vector.get("sqlite_journal_mode") != "wal"
            or vector.get("snapshot_generation") != 1
            or vector.get("snapshot_established_before_writer") is not True
            or vector.get("concurrent_writer_commit") is not True
            or vector.get("committed_generation") != 2
            or vector.get("snapshot_integrity_result") != "ok"
            or vector.get("snapshot_visible_record_generations") != [1]
            or vector.get("same_snapshot_for_integrity_fk_logs_and_semantic_checks") is not True
            or vector.get("fresh_integrity_result") != "ok"
            or vector.get("fresh_snapshot_visible_record_generations") != [1, 2]
            or vector.get("integrity_check_uses_deferred_read_transaction") is not True
            or vector.get("claim_is_power_loss_durability") is not False
        ):
            return "MixedSnapshotOrFalseCorruption"
        return "ConsistentSnapshotMaintained"

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
    if (
        manifest.get("source_repository") != "Luminous-Dynamics/symthaea"
        or corpus.get("source_repository") != manifest.get("source_repository")
    ):
        raise SystemExit("FAIL: source repository mismatch")
    if manifest.get("claim_ceiling") != corpus.get("claim_ceiling"):
        raise SystemExit("FAIL: claim ceiling differs between fixture and manifest")
    checker_commit = manifest.get("checker_commit_sha")
    if not isinstance(checker_commit, str) or not re.fullmatch(r"[0-9a-f]{40}", checker_commit):
        raise SystemExit("FAIL: checker commit pin is missing or malformed")
    if manifest.get("source_toolchain_path") != "rust-toolchain.toml":
        raise SystemExit("FAIL: producer toolchain path mismatch")
    if manifest.get("source_rust_version") != "1.96.0":
        raise SystemExit("FAIL: producer Rust toolchain version mismatch")
    source_toolchain_blob = manifest.get("source_toolchain_blob_sha")
    if not isinstance(source_toolchain_blob, str) or not re.fullmatch(r"[0-9a-f]{40}", source_toolchain_blob):
        raise SystemExit("FAIL: source toolchain blob pin is missing or malformed")
    for field in ("source_workspace_manifest_blob_sha", "source_lockfile_blob_sha"):
        value = manifest.get(field)
        if not isinstance(value, str) or not re.fullmatch(r"[0-9a-f]{40}", value):
            raise SystemExit(f"FAIL: {field} is missing or malformed")
    if manifest.get("source_rusqlite_version") != "0.40.2":
        raise SystemExit("FAIL: rusqlite version does not match the pinned producer manifest")
    if manifest.get("source_libsqlite3_sys_version") != "0.38.2":
        raise SystemExit("FAIL: libsqlite3-sys version does not match the pinned producer manifest")
    if manifest.get("source_bundled_sqlite_min_version_number") != SQLITE_MINIMUM_VERSION_NUMBER:
        raise SystemExit("FAIL: required bundled SQLite runtime floor is not 3.51.3")
    runtime = manifest.get("checker_runtime")
    if not isinstance(runtime, dict):
        raise SystemExit("FAIL: checker runtime policy is missing")
    if (
        runtime.get("implementation") != "CPython"
        or runtime.get("minimum_version") != "3.10"
        or runtime.get("runner") != "ubuntu-24.04"
        or runtime.get("records_exact_version_in_artifact") is not True
    ):
        raise SystemExit("FAIL: unsupported checker runtime policy")
    if sys.implementation.name != "cpython" or sys.version_info < (3, 10):
        raise SystemExit(
            f"FAIL: requires CPython >= 3.10, got {sys.implementation.name} {sys.version.split()[0]}"
        )
    source_blob = manifest.get("source_adapter_blob_sha")
    if not isinstance(source_blob, str) or not re.fullmatch(r"[0-9a-f]{40}", source_blob):
        raise SystemExit("FAIL: source adapter blob pin is missing or malformed")
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
