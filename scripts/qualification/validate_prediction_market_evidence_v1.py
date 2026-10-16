#!/usr/bin/env python3
"""Structural and cross-record qualification for prediction-market evidence v1.

Qualification tooling only; not cryptographic, identity, authority, source-truth, or settlement verification.
"""
from __future__ import annotations

import hashlib
import json
import sys
from datetime import datetime
from pathlib import Path
from typing import Any
import rfc8785
from jsonschema import Draft202012Validator, FormatChecker

ROOT = Path(__file__).resolve().parents[2]
CONTRACT_SCHEMA_PATH = ROOT / "docs/finance/schemas/prediction_market_contract_v1.schema.json"
EVENT_SCHEMA_PATH = ROOT / "docs/finance/schemas/prediction_market_resolution_event_v1.schema.json"
CONTRACT_VALID_PATH = ROOT / "docs/finance/fixtures/prediction_market_contract_v1.valid.json"
CONTRACT_INVALID_PATH = ROOT / "docs/finance/fixtures/prediction_market_contract_v1.invalid.json"
EVENT_VALID_PATH = ROOT / "docs/finance/fixtures/prediction_market_resolution_event_v1.proposed.valid.json"
EVENT_INVALID_PATH = ROOT / "docs/finance/fixtures/prediction_market_resolution_event_v1.predecessor-invalid.json"


class DuplicateJsonKey(ValueError):
    """Raised when a JSON object repeats a key."""


def _unique_pairs(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise DuplicateJsonKey(f"duplicate JSON key: {key}")
        result[key] = value
    return result


def load_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"), object_pairs_hook=_unique_pairs)


def make_validator(path: Path) -> Draft202012Validator:
    schema = load_json(path)
    Draft202012Validator.check_schema(schema)
    return Draft202012Validator(schema, format_checker=FormatChecker())


def _parse_utc(value: str) -> datetime:
    return datetime.fromisoformat(value[:-1] + "+00:00")

def contract_content_digest(record: dict[str, Any]) -> str:
    """Recompute the contract commitment using RFC 8785 JCS, omitting its digest field."""
    payload = dict(record)
    payload.pop("contract_digest_sha256", None)
    return hashlib.sha256(rfc8785.dumps(payload)).hexdigest()


def event_content_digest(event: dict[str, Any]) -> str:
    """Recompute the event commitment, omitting its digest and separate attestation envelope."""
    payload = dict(event)
    payload.pop("event_digest_sha256", None)
    payload.pop("attestation", None)
    return hashlib.sha256(rfc8785.dumps(payload)).hexdigest()




def validate_contract_record(record: dict[str, Any], validator: Draft202012Validator) -> list[str]:
    errors = [
        f"schema:{'/'.join(str(part) for part in error.absolute_path)}:{error.message}"
        for error in validator.iter_errors(record)
    ]
    if errors:
        return sorted(errors)
    if contract_content_digest(record).lower() != record["contract_digest_sha256"].lower():
        errors.append("semantic:contract_digest_sha256 does not match RFC 8785 canonical bytes")
    created = _parse_utc(record["created_at"])
    event_deadline = _parse_utc(record["event_deadline_at"])
    resolution_deadline = _parse_utc(record["resolution_policy"]["resolution_deadline_at"])
    if created > event_deadline:
        errors.append("semantic:created_at after event_deadline_at")
    if resolution_deadline < event_deadline:
        errors.append("semantic:resolution deadline before event deadline")
    return sorted(errors)


def validate_resolution_event_record(
    event: dict[str, Any], contract: dict[str, Any], validator: Draft202012Validator
) -> list[str]:
    errors = [
        f"schema:{'/'.join(str(part) for part in error.absolute_path)}:{error.message}"
        for error in validator.iter_errors(event)
    ]
    if errors:
        return sorted(errors)

    if event_content_digest(event).lower() != event["event_digest_sha256"].lower():
        errors.append("semantic:event_digest_sha256 does not match RFC 8785 canonical bytes")
    if event["contract_id"] != contract["contract_id"]:
        errors.append("semantic:contract_id mismatch")
    if event["contract_revision"] != contract["revision"]:
        errors.append("semantic:contract_revision mismatch")
    if event["contract_digest_sha256"].lower() != contract["contract_digest_sha256"].lower():
        errors.append("semantic:contract digest mismatch")
    if event["attestation"]["signed_event_digest_sha256"].lower() != event["event_digest_sha256"].lower():
        errors.append("semantic:attestation digest does not match declared event digest")
    if _parse_utc(event["recorded_at"]) > _parse_utc(contract["resolution_policy"]["resolution_deadline_at"]):
        errors.append("semantic:event recorded after resolution deadline")

    for i, artifact in enumerate(event["evidence_artifacts"]):
        published = artifact.get("published_at")
        if published is not None and _parse_utc(artifact["retrieved_at"]) < _parse_utc(published):
            errors.append(f"semantic:evidence_artifacts/{i}:retrieval precedes publication")
    return sorted(errors)


def main() -> int:
    cv = make_validator(CONTRACT_SCHEMA_PATH)
    ev = make_validator(EVENT_SCHEMA_PATH)
    contract = load_json(CONTRACT_VALID_PATH)
    event = load_json(EVENT_VALID_PATH)
    invalid_contract = load_json(CONTRACT_INVALID_PATH)
    invalid_event = load_json(EVENT_INVALID_PATH)

    checks = [
        ("valid contract", validate_contract_record(contract, cv), False),
        ("valid event", validate_resolution_event_record(event, contract, ev), False),
        ("invalid contract", validate_contract_record(invalid_contract, cv), True),
        ("invalid event", validate_resolution_event_record(invalid_event, contract, ev), True),
    ]
    for label, errors, should_fail in checks:
        if bool(errors) != should_fail:
            print(f"FAIL {label}: {errors}", file=sys.stderr)
            return 1
        print(f"PASS {label}")
    print("PASS both schemas satisfy Draft 2020-12 meta-schema")
    print("PASS RFC 8785/SHA-256 content commitments and cross-record digest-envelope/timestamp relations")
    print("CLAIM CEILING: source-artifact bytes, signatures, identity, authority, source truth, and settlement are not verified")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
