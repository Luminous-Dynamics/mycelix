#!/usr/bin/env python3
"""Independent CIV-013 v1 fixture verifier; standard library only.

This validates canonical reference vectors and state-transition outcomes.
It is not a production witness, filesystem crash test, signature verifier,
quorum implementation, or anti-rollback service.
"""
from __future__ import annotations

import hashlib
import json
import sys
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[2]
FIXTURE = ROOT / "docs/integral/civ-013-witness-conformance-v1.json"

RECORD_DOMAIN = b"mycelix-civ013-witness-record-v1\0"
COMMIT_DOMAIN = b"mycelix-civ013-commit-marker-v1\0"
FORK_DOMAIN = b"mycelix-civ013-fork-evidence-v1\0"
PROTOCOL_VERSION = 1

REQUIRED_IDS = {
    "V001-bootstrap-valid",
    "V002-successor-valid",
    "V003-stale-predecessor",
    "V004-generation-gap",
    "V005-rollback-anchor-ahead",
    "V006-anchor-unavailable",
    "V007-commit-marker-tampered",
    "V008-record-digest-tampered",
    "V009-fork-evidence-valid",
    "V010-fork-evidence-reordered",
    "V011-identical-retry",
    "V012-competing-successor",
    "V013-transition-receipt-rollback",
    "V014-transition-same-sequence-equivocation",
    "V015-cross-log-successor",
    "V016-history-receipt-regression",
    "V017-history-same-sequence-equivocation",
    "V018-transition-invalid-tail-shape",
    "V019-untrusted-bootstrap-anchor",
    "V020-anchor-same-generation-digest-mismatch",
    "V021-local-commit-awaiting-anchor",
    "V022-anchor-wrong-log",
}


def sha256(data: bytes) -> bytes:
    return hashlib.sha256(data).digest()


def uint(value: Any, width: int) -> bytes:
    if type(value) is not int or value < 0 or value >= 1 << (8 * width):
        raise ValueError(f"expected unsigned {width * 8}-bit integer, got {value!r}")
    return value.to_bytes(width, "big")


def digest(value: Any, field: str) -> bytes:
    if not isinstance(value, str):
        raise ValueError(f"{field} must be a hex string")
    try:
        result = bytes.fromhex(value)
    except ValueError as exc:
        raise ValueError(f"{field} is not valid hex") from exc
    if len(result) != 32 or len(value) != 64:
        raise ValueError(f"{field} must be exactly 32 bytes / 64 hex characters")
    return result


def field(value: Any, name: str) -> bytes:
    if not isinstance(value, str) or not value:
        raise ValueError(f"{name} must be a non-empty string")
    raw = value.encode("utf-8")
    return uint(len(raw), 8) + raw


def optional_digest(value: Any, name: str) -> bytes:
    return b"\x00" if value is None else b"\x01" + digest(value, name)


def canonical_record(record: dict[str, Any]) -> bytes:
    version = uint(record["protocol_version"], 2)
    if record["protocol_version"] != PROTOCOL_VERSION:
        raise ValueError("unsupported protocol version")
    return b"".join(
        (
            RECORD_DOMAIN,
            version,
            uint(record["generation"], 8),
            field(record["log_id"], "log_id"),
            field(record["policy_version"], "policy_version"),
            digest(record["anchor_digest"], "anchor_digest"),
            uint(record["receipt_sequence"], 8),
            optional_digest(record["receipt_digest"], "receipt_digest"),
            optional_digest(record["previous_record_digest"], "previous_record_digest"),
        )
    )


def canonical_marker(generation: int, record_digest_hex: str) -> bytes:
    return COMMIT_DOMAIN + uint(generation, 8) + digest(record_digest_hex, "record_digest")


def canonical_fork(evidence: dict[str, Any]) -> bytes:
    first = digest(evidence["first_record_digest"], "first_record_digest")
    conflict = digest(evidence["conflicting_record_digest"], "conflicting_record_digest")
    if first == conflict:
        raise ValueError("fork evidence must identify distinct competing records")
    return b"".join(
        (
            FORK_DOMAIN,
            uint(evidence["generation"], 8),
            first,
            conflict,
            optional_digest(evidence["previous_evidence_digest"], "previous_evidence_digest"),
        )
    )


def valid_fork_chain(chain: list[dict[str, Any]]) -> bool:
    previous: str | None = None
    for item in chain:
        if item.get("previous_evidence_digest") != previous:
            return False
        try:
            actual = sha256(canonical_fork(item)).hex()
        except (KeyError, ValueError, TypeError):
            return False
        if item.get("digest") != actual:
            return False
        previous = actual
    return True


def evaluate(vector: dict[str, Any], accepted: dict[str, dict[str, Any]]) -> str:
    kind = vector["kind"]

    if kind == "record":
        record = vector["record"]
        actual = sha256(canonical_record(record)).hex()
        if actual != record["digest"]:
            return "REJECT_BAD_RECORD_DIGEST"

        predecessor_id = vector.get("requires_predecessor")
        if predecessor_id is None:
            if record["generation"] != 1 or record["previous_record_digest"] is not None:
                return "REJECT_BAD_BOOTSTRAP"
            trusted_anchor = vector.get("trusted_bootstrap_anchor_digest")
            if trusted_anchor is None or digest(trusted_anchor, "trusted_bootstrap_anchor_digest") != digest(record["anchor_digest"], "anchor_digest"):
                return "REJECT_UNTRUSTED_BOOTSTRAP"
        else:
            previous = accepted.get(predecessor_id)
            if previous is None:
                return "REJECT_MISSING_PREDECESSOR"
            if record["generation"] != previous["generation"] + 1:
                return "REJECT_GENERATION_GAP"
            if record["previous_record_digest"] != previous["digest"]:
                return "REJECT_STALE_PREDECESSOR"
            if record["log_id"] != previous["log_id"]:
                return "REJECT_LOG_ID_MISMATCH"
            if record["receipt_sequence"] < previous["receipt_sequence"] or (
                record["receipt_sequence"] == previous["receipt_sequence"]
                and record["receipt_digest"] != previous["receipt_digest"]
            ):
                return "REJECT_RECEIPT_TAIL_HISTORY_INVALID"

        if record["receipt_sequence"] == 0 and record["receipt_digest"] is not None:
            return "REJECT_BAD_RECEIPT_TAIL"
        if record["receipt_sequence"] > 0 and record["receipt_digest"] is None:
            return "REJECT_BAD_RECEIPT_TAIL"

        marker = vector["commit_marker"]
        if marker["generation"] != record["generation"] or marker["record_digest"] != record["digest"]:
            return "REJECT_BAD_MARKER"
        if sha256(canonical_marker(marker["generation"], marker["record_digest"])).hex() != marker["digest"]:
            return "REJECT_BAD_MARKER"

        accepted[vector["id"]] = record
        return "ACCEPT"

    if kind == "transition":
        current_generation = vector["current_generation"]
        candidate_generation = vector["candidate_generation"]
        candidate_digest = vector.get("candidate_record_digest")
        current_digest = vector["current_record_digest"]

        if candidate_generation == current_generation:
            if candidate_digest == current_digest:
                return "IDEMPOTENT_SAME_RECEIPT"
            return "REJECT_CAS_LOST"
        if candidate_generation != current_generation + 1:
            return "REJECT_GENERATION_GAP"
        if vector.get("candidate_previous_record_digest") != current_digest:
            return "REJECT_STALE_PREDECESSOR"
        if "candidate_receipt_sequence" in vector:
            candidate_sequence = vector["candidate_receipt_sequence"]
            current_sequence = vector["current_receipt_sequence"]
            candidate_tail = vector["candidate_receipt_digest"]
            current_tail = vector["current_receipt_digest"]
            if (candidate_sequence == 0) != (candidate_tail is None):
                return "REJECT_BAD_RECEIPT_TAIL"
            if candidate_tail is not None:
                digest(candidate_tail, "candidate_receipt_digest")
            if (current_sequence == 0) != (current_tail is None):
                return "REJECT_BAD_CURRENT_RECEIPT_TAIL"
            if current_tail is not None:
                digest(current_tail, "current_receipt_digest")
            if candidate_sequence < current_sequence:
                return "REJECT_RECEIPT_ROLLBACK"
            if candidate_sequence == current_sequence and candidate_tail != current_tail:
                return "REJECT_RECEIPT_TAIL_EQUIVOCATION"
        return "ACCEPT_TRANSITION"

    if kind == "recovery":
        if vector.get("external_anchor") is None and "external_anchor" in vector:
            return "REJECT_ANCHOR_UNAVAILABLE"
        external_generation = vector.get("external_generation")
        if external_generation is None:
            return "REJECT_ANCHOR_UNAVAILABLE"
        local_log_id = vector.get("local_log_id")
        external_log_id = vector.get("external_log_id")
        anchor = vector.get("external_anchor")
        if external_log_id is not None and local_log_id is not None and external_log_id != local_log_id:
            return "REJECT_EXTERNAL_ANCHOR_MISMATCH"
        if isinstance(anchor, dict):
            if (
                anchor.get("generation") != external_generation
                or anchor.get("log_id") != external_log_id
                or anchor.get("record_digest") != vector.get("external_record_digest")
            ):
                return "REJECT_EXTERNAL_ANCHOR_MISMATCH"
        if external_generation > vector["local_generation"]:
            return "REJECT_ROLLBACK_DETECTED"
        if external_generation < vector["local_generation"]:
            return "PENDING_EXTERNAL_ANCHOR"
        local_digest = vector.get("local_record_digest")
        external_digest = vector.get("external_record_digest")
        if local_digest is not None and external_digest is not None and local_digest != external_digest:
            return "REJECT_EXTERNAL_ANCHOR_MISMATCH"
        return "ACCEPT_RECOVERY"

    if kind == "marker_negative":
        expected = sha256(canonical_marker(vector["generation"], vector["record_digest"])).hex()
        return "REJECT_BAD_MARKER" if expected != vector["marker_digest"] else "ACCEPT_MARKER"

    if kind == "record_negative":
        expected = sha256(canonical_record(vector["record"])).hex()
        return "REJECT_BAD_RECORD_DIGEST" if expected != vector["record"]["digest"] else "ACCEPT_RECORD"

    if kind == "bootstrap_negative":
        record = vector["record"]
        if sha256(canonical_record(record)).hex() != record.get("digest"):
            return "REJECT_BAD_RECORD_DIGEST"
        marker = vector["commit_marker"]
        if (
            marker.get("generation") != record["generation"]
            or marker.get("record_digest") != record["digest"]
            or sha256(canonical_marker(marker["generation"], marker["record_digest"])).hex()
            != marker.get("digest")
        ):
            return "REJECT_BAD_MARKER"
        if record["generation"] != 1 or record["previous_record_digest"] is not None:
            return "REJECT_BAD_BOOTSTRAP"
        if digest(vector["trusted_bootstrap_anchor_digest"], "trusted_bootstrap_anchor_digest") != digest(record["anchor_digest"], "anchor_digest"):
            return "REJECT_UNTRUSTED_BOOTSTRAP"
        return "ACCEPT_BOOTSTRAP"

    if kind == "fork_evidence":
        evidence = vector["evidence"]
        actual = sha256(canonical_fork(evidence)).hex()
        if actual != evidence["digest"]:
            return "REJECT_BAD_FORK_EVIDENCE"
        return "ACCEPT_EVIDENCE_ONLY"

    if kind == "fork_chain_negative":
        chain = vector["evidence_chain"]
        if not valid_fork_chain(chain):
            return "REJECT_BAD_SOURCE_CHAIN"
        reordered = [chain[index] for index in vector["order"]]
        return "REJECT_REORDERED_FORK_CHAIN" if not valid_fork_chain(reordered) else "ACCEPT_FORK_CHAIN"

    if kind == "history_chain_negative":
        records = vector["records"]
        markers = vector["commit_markers"]
        if not records or len(records) != len(markers):
            return "REJECT_BAD_HISTORY_SHAPE"
        previous = None
        for index, record in enumerate(records):
            if sha256(canonical_record(record)).hex() != record.get("digest"):
                return "REJECT_BAD_RECORD_DIGEST"
            marker = markers[index]
            if (
                marker.get("generation") != record["generation"]
                or marker.get("record_digest") != record["digest"]
                or sha256(canonical_marker(marker["generation"], marker["record_digest"])).hex()
                != marker.get("digest")
            ):
                return "REJECT_BAD_MARKER"
            if record["receipt_sequence"] == 0 and record["receipt_digest"] is not None:
                return "REJECT_BAD_RECEIPT_TAIL"
            if record["receipt_sequence"] > 0 and record["receipt_digest"] is None:
                return "REJECT_BAD_RECEIPT_TAIL"
            if previous is None:
                if record["generation"] != 1 or record["previous_record_digest"] is not None:
                    return "REJECT_BAD_HISTORY_SHAPE"
            else:
                if record["generation"] != previous["generation"] + 1:
                    return "REJECT_GENERATION_GAP"
                if record["previous_record_digest"] != previous["digest"]:
                    return "REJECT_STALE_PREDECESSOR"
                if record["log_id"] != previous["log_id"]:
                    return "REJECT_LOG_ID_MISMATCH"
                if record["receipt_sequence"] < previous["receipt_sequence"] or (
                    record["receipt_sequence"] == previous["receipt_sequence"]
                    and record["receipt_digest"] != previous["receipt_digest"]
                ):
                    return "REJECT_RECEIPT_TAIL_HISTORY_INVALID"
            previous = record
        return "ACCEPT_HISTORY_CHAIN"

    raise ValueError(f"unknown vector kind: {kind!r}")


def main() -> int:
    with FIXTURE.open("r", encoding="utf-8") as handle:
        corpus = json.load(handle)

    manifest_path = ROOT / "docs/integral/civ-013-witness-conformance-v1-manifest.json"
    with manifest_path.open("r", encoding="utf-8") as handle:
        manifest = json.load(handle)
    if manifest.get("schema_version") != 1:
        raise SystemExit("FAIL: unsupported manifest schema_version")
    if manifest.get("profile_id") != "civ-013-model-smoke-v1":
        raise SystemExit("FAIL: unsupported or missing manifest profile_id")
    if manifest.get("spec_version") != corpus.get("spec_version"):
        raise SystemExit("FAIL: manifest/fixture spec_version mismatch")
    if manifest.get("source_model_commit") != corpus.get("source_model_commit"):
        raise SystemExit("FAIL: manifest/fixture source commit mismatch")
    if manifest.get("fixture_path") != "docs/integral/civ-013-witness-conformance-v1.json":
        raise SystemExit("FAIL: unexpected fixture path in manifest")
    if manifest.get("verifier_path") != "scripts/integral/verify_civ013_witness_conformance_v1.py":
        raise SystemExit("FAIL: unexpected verifier path in manifest")

    if corpus.get("spec_version") != "civ-013-conformance-v1":
        raise SystemExit("FAIL: unexpected fixture spec_version")
    encoding = corpus.get("encoding", {})
    if bytes.fromhex(encoding.get("record_domain_hex", "")) != RECORD_DOMAIN:
        raise SystemExit("FAIL: record domain does not match verifier")
    if bytes.fromhex(encoding.get("commit_domain_hex", "")) != COMMIT_DOMAIN:
        raise SystemExit("FAIL: commit domain does not match verifier")
    if bytes.fromhex(encoding.get("fork_domain_hex", "")) != FORK_DOMAIN:
        raise SystemExit("FAIL: fork domain does not match verifier")
    if encoding.get("digest") != "SHA-256":
        raise SystemExit("FAIL: unexpected digest algorithm")

    vectors = corpus.get("vectors")
    if not isinstance(vectors, list):
        raise SystemExit("FAIL: vectors must be a JSON array")
    ids = [item.get("id") for item in vectors]
    if len(ids) != len(set(ids)):
        raise SystemExit("FAIL: duplicate vector IDs")
    if set(ids) != REQUIRED_IDS:
        raise SystemExit(f"FAIL: vector ID set mismatch; got={sorted(ids)}")

    if manifest.get("expected_vector_count") != len(vectors):
        raise SystemExit("FAIL: manifest expected_vector_count mismatch")
    if set(manifest.get("required_vector_ids", [])) != REQUIRED_IDS:
        raise SystemExit("FAIL: manifest required vector IDs mismatch")
    if set(ids) != set(manifest.get("required_vector_ids", [])):
        raise SystemExit("FAIL: fixture IDs differ from manifest")

    accepted: dict[str, dict[str, Any]] = {}
    failures: list[str] = []
    for vector in vectors:
        try:
            actual = evaluate(vector, accepted)
        except (KeyError, TypeError, ValueError) as exc:
            actual = f"ERROR:{type(exc).__name__}"
        expected = vector.get("expected_outcome")
        ok = actual == expected
        print(f"{'PASS' if ok else 'FAIL'} {vector['id']} expected={expected} actual={actual}")
        if not ok:
            failures.append(vector["id"])

    print(f"SUMMARY vectors={len(vectors)} passed={len(vectors) - len(failures)} failed={len(failures)}")
    print(f"CLAIM_CEILING {corpus.get('claim_ceiling', 'MISSING')}")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
