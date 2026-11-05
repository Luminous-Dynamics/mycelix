#!/usr/bin/env python3
"""CI-only schema and fixture contract checks; this is not an evidence verifier."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from importlib.metadata import version
from pathlib import Path
from typing import Any

from jsonschema import Draft202012Validator


EXPECTED_CASES = {
    "valid-reference-001": "eligible_for_policy_evaluation",
    "legacy-url-002": "unassessed_legacy_locator",
    "wrong-boundary-003": "blocked",
    "future-known-004": "blocked",
    "unauthorized-signer-005": "blocked",
    "superseded-evidence-006": "historical_only",
    "copied-source-family-007": "eligible_but_not_independent_corroboration",
    "embargoed-public-view-008": "blocked",
}

EXPECTED_MUTATIONS = {
    "wrong-boundary": "blocked",
    "future-known": "blocked_for_historical_frontier",
    "unauthorized-role": "blocked",
    "event-hash-substitution": "blocked",
    "superseded": "historical_only",
    "public-embargo-leak": "blocked",
}


def fail(message: str) -> None:
    raise SystemExit(f"energy evidence-reference conformance: FAIL: {message}")


def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def walk_refs(value: Any) -> list[str]:
    refs: list[str] = []
    if isinstance(value, dict):
        ref = value.get("$ref")
        if isinstance(ref, str):
            refs.append(ref)
        for child in value.values():
            refs.extend(walk_refs(child))
    elif isinstance(value, list):
        for child in value:
            refs.extend(walk_refs(child))
    return refs


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--semantic-receipt", type=Path, required=True)
    args = parser.parse_args()

    energy_root = Path(__file__).resolve().parents[1]
    schema_path = energy_root / "docs" / "project-evidence-reference-v1.schema.json"
    fixtures_path = energy_root / "docs" / "project-evidence-reference-v1.fixtures.json"

    try:
        schema_bytes = schema_path.read_bytes()
        fixture_bytes = fixtures_path.read_bytes()
        schema = json.loads(schema_bytes)
        fixtures = json.loads(fixture_bytes)
    except (OSError, json.JSONDecodeError) as exc:
        fail(f"cannot load schema/fixtures: {exc}")

    if schema.get("$schema") != "https://json-schema.org/draft/2020-12/schema":
        fail("schema must declare JSON Schema Draft 2020-12")

    try:
        Draft202012Validator.check_schema(schema)
    except Exception as exc:  # validator supplies several schema-error subclasses
        fail(f"schema meta-validation failed: {exc}")

    refs = walk_refs(schema)
    remote_refs = [ref for ref in refs if not ref.startswith("#/")]
    if remote_refs:
        fail(f"external schema references are not permitted in this offline contract: {remote_refs}")

    validator = Draft202012Validator(schema)
    examples = {
        "valid_reference_example": fixtures.get("valid_reference_example"),
        "legacy_reference_example": fixtures.get("legacy_reference_example"),
    }
    for name, example in examples.items():
        if not isinstance(example, dict):
            fail(f"missing object: {name}")
        errors = sorted(validator.iter_errors(example), key=lambda err: list(err.absolute_path))
        if errors:
            first = errors[0]
            fail(f"{name} violates schema at {list(first.absolute_path)}: {first.message}")

    canonical = examples["valid_reference_example"]
    desci = canonical["canonical_desci_reference"]
    receipt = desci["append_receipt"]
    provenance = canonical["provenance"]
    if receipt["event_id"] != desci["evidence_attached_event_id"]:
        fail("canonical append receipt event_id does not match EvidenceAttached event")
    if receipt["stream_id"] != desci["stream_id"]:
        fail("canonical append receipt stream_id does not match claim stream")
    if receipt["sequence"] != desci["stream_sequence"]:
        fail("canonical append receipt sequence does not match claim stream sequence")
    if receipt["event_hash"] != desci["evidence_attached_event_hash"]:
        fail("canonical append receipt event_hash does not match EvidenceAttached event hash")
    if desci["artifact_content_hash"] != provenance["artifact_bytes_hash"]:
        fail("artifact content hash does not match the provenance byte digest")
    if canonical["temporal_scope"]["known_at"] != receipt["received_at"]:
        fail("fixture known_at must be anchored to canonical server receipt time")

    legacy = examples["legacy_reference_example"]
    if legacy["evidence_class"] != "unassessed_legacy_locator":
        fail("legacy example is not explicitly classified as unassessed")
    if legacy.get("legacy_locator_ref") is None:
        fail("legacy example must preserve its raw locator")
    for field in (
        "issuer_profile",
        "canonical_desci_reference",
        "temporal_scope",
        "provenance",
        "verification_receipt",
    ):
        if legacy[field] is not None:
            fail(f"legacy example must not populate canonical field: {field}")
    if legacy["lifecycle"]["state"] != "unassessed_legacy_locator":
        fail("legacy lifecycle state must remain unassessed")

    if fixtures.get("schema_version") != schema.get("properties", {}).get("schema_version", {}).get("const"):
        fail("fixture schema_version does not match schema const")

    cases = fixtures.get("cases")
    if not isinstance(cases, list):
        fail("cases must be an array")
    case_map: dict[str, str] = {}
    for case in cases:
        if not isinstance(case, dict):
            fail("every case entry must be an object")
        case_id = case.get("case_id")
        disposition = case.get("expected_disposition")
        reason = case.get("reason")
        if not isinstance(case_id, str) or not case_id:
            fail("every case needs a non-empty case_id")
        if case_id in case_map:
            fail(f"duplicate case_id: {case_id}")
        if not isinstance(disposition, str) or not disposition:
            fail(f"case has no expected disposition: {case_id}")
        if not isinstance(reason, str) or not reason:
            fail(f"case has no rationale: {case_id}")
        case_map[case_id] = disposition

    if case_map != EXPECTED_CASES:
        missing = sorted(set(EXPECTED_CASES) - set(case_map))
        unexpected = sorted(set(case_map) - set(EXPECTED_CASES))
        changed = {
            case_id: (EXPECTED_CASES[case_id], case_map[case_id])
            for case_id in set(EXPECTED_CASES) & set(case_map)
            if EXPECTED_CASES[case_id] != case_map[case_id]
        }
        fail(f"case census/disposition drift; missing={missing}, unexpected={unexpected}, changed={changed}")

    mutations = fixtures.get("mutations")
    if not isinstance(mutations, list):
        fail("mutations must be an array")
    mutation_map: dict[str, str] = {}
    for mutation in mutations:
        if not isinstance(mutation, dict):
            fail("every mutation entry must be an object")
        case_id = mutation.get("case_id")
        path = mutation.get("path")
        disposition = mutation.get("expected_disposition")
        if not isinstance(case_id, str) or not case_id:
            fail("every mutation needs a non-empty case_id")
        if case_id in mutation_map:
            fail(f"duplicate mutation case_id: {case_id}")
        if not isinstance(path, str) or not path:
            fail(f"mutation has no JSON path: {case_id}")
        if "replacement" not in mutation:
            fail(f"mutation has no replacement value: {case_id}")
        if not isinstance(disposition, str) or not disposition:
            fail(f"mutation has no expected disposition: {case_id}")
        mutation_map[case_id] = disposition

    if mutation_map != EXPECTED_MUTATIONS:
        missing = sorted(set(EXPECTED_MUTATIONS) - set(mutation_map))
        unexpected = sorted(set(mutation_map) - set(EXPECTED_MUTATIONS))
        changed = {
            case_id: (EXPECTED_MUTATIONS[case_id], mutation_map[case_id])
            for case_id in set(EXPECTED_MUTATIONS) & set(mutation_map)
            if EXPECTED_MUTATIONS[case_id] != mutation_map[case_id]
        }
        fail(f"mutation census/disposition drift; missing={missing}, unexpected={unexpected}, changed={changed}")

    source_commit = os.environ.get("ENERGY_EVIDENCE_SOURCE_COMMIT")
    if not source_commit:
        fail("ENERGY_EVIDENCE_SOURCE_COMMIT is required for a reproducible CI receipt")

    try:
        semantic_receipt = json.loads(args.semantic_receipt.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        fail(f"cannot load Rust semantic receipt: {exc}")

    if semantic_receipt.get("source_commit") != source_commit:
        fail("Rust semantic receipt is not bound to the exact checked-out source commit")
    if semantic_receipt.get("schema_sha256") != sha256(schema_bytes):
        fail("Rust semantic receipt schema digest does not match the checked-out schema")
    if semantic_receipt.get("fixtures_sha256") != sha256(fixture_bytes):
        fail("Rust semantic receipt fixture digest does not match the checked-out fixtures")
    if semantic_receipt.get("result") != "pass":
        fail("Rust semantic evaluator did not pass")
    if semantic_receipt.get("scope") != "declared_fixture_semantics_only":
        fail("Rust semantic receipt has an unexpected claim scope")

    semantic_results = semantic_receipt.get("mutation_results")
    if not isinstance(semantic_results, list):
        fail("Rust semantic receipt must contain mutation_results")
    semantic_map = {}
    for result in semantic_results:
        if not isinstance(result, dict):
            fail("Rust mutation result must be an object")
        case_id = result.get("case_id")
        expected = result.get("expected_disposition")
        actual = result.get("actual_disposition")
        passed = result.get("passed")
        if not isinstance(case_id, str) or case_id in semantic_map:
            fail(f"missing or duplicate Rust semantic case_id: {case_id}")
        if expected != EXPECTED_MUTATIONS.get(case_id):
            fail(f"Rust expected disposition drift for mutation: {case_id}")
        if actual != expected or passed is not True:
            fail(f"Rust semantic mutation did not pass: {case_id}")
        semantic_map[case_id] = actual
    if semantic_map != EXPECTED_MUTATIONS:
        fail("Rust semantic receipt mutation census does not match the frozen fixture contract")

    if semantic_receipt.get("canonical_example", {}).get("actual_disposition") != "eligible_for_policy_evaluation":
        fail("Rust semantic receipt did not qualify the canonical synthetic example")
    if semantic_receipt.get("legacy_example", {}).get("actual_disposition") != "unassessed_legacy_locator":
        fail("Rust semantic receipt did not preserve the legacy example as unassessed")

    receipt_out = {
        "receipt_version": "2",
        "source_commit": source_commit,
        "scope": "schema_meta_and_declared_fixture_semantics",
        "semantic_evaluator": semantic_receipt.get("evaluator"),
        "semantic_scope": semantic_receipt.get("scope"),
        "semantic_result": semantic_receipt.get("result"),
        "semantic_mutation_count": len(semantic_results),
        "schema_draft": "2020-12",
        "schema_sha256": sha256(schema_bytes),
        "fixtures_sha256": sha256(fixture_bytes),
        "validator": "python-jsonschema",
        "validator_version": version("jsonschema"),
        "case_count": len(cases),
        "mutation_count": len(mutations),
        "canonical_example": "schema_valid_and_cross_reference_consistent",
        "legacy_example": "schema_valid_and_canonical_fields_null",
        "external_schema_refs": 0,
        "result": "pass",
    }

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(receipt_out, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(receipt_out, sort_keys=True))


if __name__ == "__main__":
    main()
