#!/usr/bin/env python3
"""Independent static conformance check for the Praxis Symthaea response schema.

This is a source/schema drift gate. It does not execute Rust, authenticate a
Symthaea process, or establish pedagogical correctness.
"""

from __future__ import annotations

import json
import re
import sys
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
RUST_PATH = ROOT / "crates/praxis-core/src/symthaea_advisory.rs"
SCHEMA_PATH = ROOT / "schemas/symthaea-advisory-v1.schema.json"

EXPECTED_FIELDS = {
    "schema_version",
    "model_id",
    "model_version",
    "skill_id",
    "curriculum_revision",
    "input_digest",
    "action",
    "reported_confidence_permille",
    "reason",
    "supporting_content_ids",
}

REQUIRED_TESTS = {
    "accepts_scoped_advisory_suggestion",
    "disabled_advisor_is_rejected",
    "sandbox_suppresses_model_adaptivity",
    "stale_request_digest_is_rejected",
    "a_different_skill_cannot_receive_the_advice",
    "untrusted_content_reference_is_rejected",
    "unsupported_confidence_range_is_rejected",
    "suggestions_need_trusted_content_grounding",
    "a_model_cannot_order_a_break_even_by_claiming_the_learner_requested_it",
    "model_cannot_fabricate_a_learner_requested_reason",
    "break_is_allowed_only_when_trusted_context_enables_it",
    "no_suggestion_is_allowed_when_evidence_is_insufficient",
    "rejects_unknown_schema_and_model_version",
    "a_different_model_cannot_submit_advice",
    "an_old_curriculum_revision_is_rejected",
    "oversized_reference_lists_are_rejected_before_reference_lookup",
    "action_and_reason_must_be_semantically_compatible",
    "identifiers_with_control_characters_are_rejected",
    "wire_contract_rejects_unknown_fields",
    "malformed_input_digests_are_rejected",
    "request_digest_matches_blake3_empty_input_golden_vector",
    "request_digest_matches_blake3_nonempty_golden_vector",
    "duplicate_references_are_rejected_deterministically",
}


def require(condition: bool, message: str, errors: list[str]) -> None:
    if not condition:
        errors.append(message)


def enum_variants(source: str, name: str) -> list[str]:
    match = re.search(
        rf"pub enum {re.escape(name)}\s*\{{(.*?)^\}}",
        source,
        flags=re.MULTILINE | re.DOTALL,
    )
    if not match:
        raise ValueError(f"could not locate Rust enum {name}")
    body = re.sub(r"(?m)^\s*///.*$", "", match.group(1))
    variants = re.findall(r"(?m)^\s*([A-Z][A-Za-z0-9]*)\s*,\s*$", body)
    if not variants:
        raise ValueError(f"could not parse Rust enum variants for {name}")
    return [
        re.sub(r"([a-z0-9])([A-Z])", r"\1_\2", variant).lower()
        for variant in variants
    ]


def integer_constant(source: str, name: str) -> int:
    match = re.search(
        rf"pub const {re.escape(name)}:\s*usize\s*=\s*(\d+)\s*;",
        source,
    )
    if not match:
        raise ValueError(f"could not locate Rust constant {name}")
    return int(match.group(1))


def main() -> int:
    errors: list[str] = []
    try:
        source = RUST_PATH.read_text(encoding="utf-8")
        schema: dict[str, Any] = json.loads(SCHEMA_PATH.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        print(f"Symthaea advisory static conformance FAILED: {error}", file=sys.stderr)
        return 1

    try:
        action_variants = enum_variants(source, "LearningSuggestion")
        reason_variants = enum_variants(source, "AdviceReason")
        schema_version = int(
            re.search(
                r"pub const SYMTHAEA_ADVISORY_SCHEMA_VERSION:\s*u16\s*=\s*(\d+)\s*;",
                source,
            ).group(1)
        )
        max_refs = integer_constant(source, "MAX_SUPPORTING_CONTENT_IDS")
        max_identifier_chars = integer_constant(source, "MAX_ADVISORY_IDENTIFIER_CHARS")
    except (AttributeError, TypeError, ValueError) as error:
        print(f"Symthaea advisory static conformance FAILED: {error}", file=sys.stderr)
        return 1

    properties = schema.get("properties", {})
    schema_fields = set(properties)
    require(schema.get("$schema") == "https://json-schema.org/draft/2020-12/schema",
            "schema must declare JSON Schema 2020-12", errors)
    require(schema.get("additionalProperties") is False,
            "schema must reject unknown fields", errors)
    require(schema.get("required") is not None and set(schema["required"]) == EXPECTED_FIELDS,
            "schema required fields must exactly match the frozen response fields", errors)
    require(schema_fields == EXPECTED_FIELDS,
            "schema properties must exactly match the frozen response fields", errors)
    require(properties.get("schema_version", {}).get("const") == schema_version,
            "schema version must match the Rust protocol constant", errors)
    require(sorted(properties.get("action", {}).get("enum", [])) == sorted(action_variants),
            "schema action enum drifted from Rust LearningSuggestion", errors)
    require(sorted(properties.get("reason", {}).get("enum", [])) == sorted(reason_variants),
            "schema reason enum drifted from Rust AdviceReason", errors)

    confidence = properties.get("reported_confidence_permille", {})
    require(confidence.get("type") == "integer" and confidence.get("minimum") == 0
            and confidence.get("maximum") == 1000,
            "reported confidence must be an integer bounded to 0..=1000", errors)
    require("if candidate.reported_confidence_permille > 1000" in source,
            "Rust validator must enforce the upper confidence bound", errors)

    references = properties.get("supporting_content_ids", {})
    require(references.get("type") == "array" and references.get("maxItems") == max_refs
            and references.get("uniqueItems") is True,
            "content references must be unique and bounded to the Rust maximum", errors)
    require(references.get("items", {}).get("maxLength") == max_identifier_chars,
            "content reference length must match the Rust identifier maximum", errors)

    for field in (
        "model_id",
        "model_version",
        "skill_id",
        "curriculum_revision",
    ):
        require(properties.get(field, {}).get("maxLength") == max_identifier_chars,
                f"{field} maxLength must match Rust", errors)

    digest = properties.get("input_digest", {})
    require(digest.get("minLength") == 71 and digest.get("maxLength") == 71
            and digest.get("pattern") == "^blake3:[0-9a-f]{64}$",
            "input digest must be exactly blake3: plus 64 lowercase hex characters", errors)
    require("fn valid_blake3_digest(value: &str) -> bool" in source,
            "Rust must validate canonical BLAKE3 digest format", errors)
    require("pub fn digest_advisory_request_bytes(canonical_request_bytes: &[u8]) -> String" in source,
            "Rust must expose a helper to hash the exact request bytes sent to the advisor", errors)
    require("blake3:af1349b9f5f9a1a6a0404dea36dcc9499bcb25c9adc112b7cc9a93cae41f3262" in source,
            "Rust must retain the BLAKE3 empty-input golden vector", errors)
    require("blake3:6437b3ac38465133ffb63b75273a8db548c558465d79db03fd359c6cd5bd9d85" in source,
            "Rust must retain the BLAKE3 nonempty-input golden vector", errors)
    require("if !valid_blake3_digest(&candidate.input_digest)" in source,
            "Rust must reject malformed input digests", errors)

    require("#[serde(deny_unknown_fields)]" in source,
            "Rust wire type must reject unknown fields", errors)
    require("!value.chars().any(char::is_control)" in source,
            "Rust identifier validation must reject control characters", errors)
    require("context.allowed_actions.contains(&candidate.action)" in source,
            "candidate action must be checked against trusted per-turn policy", errors)
    require("context.allowed_reasons.contains(&candidate.reason)" in source,
            "candidate reason must be checked against trusted per-turn evidence", errors)
    require("candidate.input_digest != context.input_digest" in source,
            "candidate must be bound to the exact trusted request digest", errors)
    require("if context.sandbox_active" in source,
            "sandbox must suppress model-driven adaptivity", errors)

    authored_tests = set(re.findall(r"fn\s+([a-zA-Z0-9_]+)\s*\(\s*\)\s*\{", source))
    missing_tests = sorted(REQUIRED_TESTS - authored_tests)
    require(not missing_tests, f"missing authored Rust regression tests: {missing_tests}", errors)

    if errors:
        print("Symthaea advisory static conformance FAILED:", file=sys.stderr)
        for error in errors:
            print(f"  - {error}", file=sys.stderr)
        return 1

    print(
        "Symthaea advisory static conformance PASS: "
        f"{len(action_variants)} actions, {len(reason_variants)} reasons, "
        f"{len(REQUIRED_TESTS)} required Rust test cases, "
        "strict schema/source parity and authority-boundary checks."
    )
    print("Claim ceiling: static source/schema conformance only; Rust runtime tests remain separate.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
