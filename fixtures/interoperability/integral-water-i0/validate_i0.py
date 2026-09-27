#!/usr/bin/env python3
"""Strict validator for the MYC-INT-006C Water I0 conformance corpus."""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import sys
from pathlib import Path
from typing import Any

VALIDATOR_PROFILE = "myc-int-006d-i0-validator-v1"
CORPUS_FILENAME = "myc-int-006c-water-i0-corpus.v1.json"
SCHEMA_FILENAME = "myc-int-006c-water-i0-corpus.schema.json"
CORPUS_ID = "myc-int-006c-water-i0"
CORPUS_VERSION = "1.0.0"
CORPUS_PROFILE = "runtime-neutral-semantic-conformance-v1"
CORPUS_SCHEMA_REF = f"./{SCHEMA_FILENAME}"

SOURCE_REFS = {
    "scenario_issue": 3119,
    "evaluation_issue": 3147,
    "integral_source_registry_pr": 3144,
}

SUBJECT_KINDS = frozenset({
    "Resource", "Issue", "Observation", "ActorReport", "EvidenceBundle",
    "Alternative", "Objection", "Prediction", "Recommendation", "Decision",
    "ForeignDecision", "Authorization", "ImplementationAttempt", "ImplementationReceipt",
    "OutcomeObservation", "ReviewCandidate", "SupersedingDecisionCandidate", "Credential",
    "Certification", "DerivedSummary", "ForeignAuthority", "DeliveryReceipt",
})
CASE_KINDS = frozenset({"Positive", "Hostile", "Partition", "Migration"})
DISPOSITIONS = frozenset({"Accepted", "Rejected", "Indeterminate", "Unsupported"})
PREDICATES = frozenset({
    "DistinctSemanticSubjects", "NoEffectAuthority", "BindsExactSubject",
    "PreservesSourceSchema", "PreservesProvenance", "PreservesUnknownState",
    "PreservesConflict", "SingleLogicalEffect", "NoHistoricalMutation",
    "CreatesReviewCandidate", "NoLocalAuthority", "NoObservationPromotion",
    "NoReceiptPromotion", "NoOutcomePromotion", "DeclaresTranslationLoss",
    "IdempotentReplay", "RejectsExpiredAuthority", "RejectsStaleSchema",
})
REASON_CODES = frozenset({
    "LINEAGE_PRESERVED",
    "RECOMMENDATION_NOT_AUTHORITY",
    "FOREIGN_DECISION_NOT_LOCAL_AUTHORITY",
    "CREDENTIAL_NOT_AUTHORITY",
    "IDEMPOTENT_REPLAY",
    "DELIVERY_OUTCOME_UNKNOWN",
    "STALE_SCHEMA_GENERATION",
    "EXTERNAL_CERT_NOT_LOCAL_ACCEPTANCE",
    "DERIVED_NOT_SOURCE_FACT",
    "AUTHORIZATION_NOT_CURRENT",
    "HISTORICAL_MUTATION_FORBIDDEN",
    "ORDER_INDEPENDENT_SEMANTICS",
    "SCHEMA_QUALIFIED_IDENTITY_MISMATCH",
    "UNDECLARED_TRANSLATION_LOSS",
    "FOREIGN_AUTHORITY_NOT_LOCAL",
    "ARRIVAL_ORDER_NOT_CONFLICT_RESOLUTION",
    "PREDICTION_NOT_OBSERVATION",
    "RECEIPT_ROLE_MISMATCH",
    "SUCCESS_NOT_DESIRED_OUTCOME",
})
EXPECTED_CASE_IDS = frozenset({"I0-POS-001", *(f"I0-HOSTILE-{i:03d}" for i in range(1, 19))})

SUBJECT_ID_RE = re.compile(r"^[a-z0-9][a-z0-9._-]{2,63}$")
CASE_ID_RE = re.compile(r"^I0-[A-Z]+-[0-9]{3}$")
SEMANTIC_NAME_RE = re.compile(r"^[a-z0-9][a-z0-9._-]*$")
REASON_CODE_RE = re.compile(r"^[A-Z][A-Z0-9_]{2,63}$")
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")

RUNTIME_IDENTITY_SENTENCE = (
    "runtime identifiers are provenance only unless explicitly promoted by a source profile"
)


class ValidationError(ValueError):
    """Corpus/schema validation failure."""


def _strict_object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    obj: dict[str, Any] = {}
    for key, value in pairs:
        if key in obj:
            raise ValidationError(f"duplicate JSON object member: {key!r}")
        obj[key] = value
    return obj


def parse_strict_json(raw: bytes, *, label: str) -> Any:
    try:
        text = raw.decode("utf-8", errors="strict")
    except UnicodeDecodeError as exc:
        raise ValidationError(f"{label}: invalid UTF-8: {exc}") from exc
    try:
        return json.loads(text, object_pairs_hook=_strict_object)
    except ValidationError:
        raise
    except json.JSONDecodeError as exc:
        raise ValidationError(f"{label}: invalid JSON: {exc}") from exc


def _mapping(value: Any, ctx: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValidationError(f"{ctx}: expected object")
    return value


def _list(value: Any, ctx: str) -> list[Any]:
    if not isinstance(value, list):
        raise ValidationError(f"{ctx}: expected array")
    return value


def _nonempty_string(value: Any, ctx: str) -> str:
    if not isinstance(value, str) or not value:
        raise ValidationError(f"{ctx}: expected non-empty string")
    return value


def _exact_keys(obj: dict[str, Any], required: set[str], optional: set[str], ctx: str) -> None:
    keys = set(obj)
    missing = required - keys
    unknown = keys - required - optional
    if missing:
        raise ValidationError(f"{ctx}: missing fields: {sorted(missing)}")
    if unknown:
        raise ValidationError(f"{ctx}: unknown fields: {sorted(unknown)}")


def _validate_semantic_ref(value: Any, ctx: str) -> None:
    ref = _mapping(value, ctx)
    _exact_keys(ref, {"namespace", "name", "version"}, set(), ctx)
    namespace = _nonempty_string(ref["namespace"], f"{ctx}.namespace")
    name = _nonempty_string(ref["name"], f"{ctx}.name")
    version = _nonempty_string(ref["version"], f"{ctx}.version")
    if not SEMANTIC_NAME_RE.fullmatch(namespace):
        raise ValidationError(f"{ctx}.namespace: malformed semantic namespace")
    if not SEMANTIC_NAME_RE.fullmatch(name):
        raise ValidationError(f"{ctx}.name: malformed semantic name")
    if len(version) > 128:
        raise ValidationError(f"{ctx}.version: exceeds 128 characters")


def _validate_schema_registry(schema: dict[str, Any]) -> None:
    if schema.get("$schema") != "https://json-schema.org/draft/2020-12/schema":
        raise ValidationError("schema: expected JSON Schema Draft 2020-12")
    properties = _mapping(schema.get("properties"), "schema.properties")
    defs = _mapping(schema.get("$defs"), "schema.$defs")

    if properties.get("$schema", {}).get("const") != CORPUS_SCHEMA_REF:
        raise ValidationError("schema: corpus $schema const drifted")
    if properties.get("corpus_id", {}).get("const") != CORPUS_ID:
        raise ValidationError("schema: corpus_id const drifted")
    if properties.get("profile", {}).get("const") != CORPUS_PROFILE:
        raise ValidationError("schema: corpus profile const drifted")

    source_props = _mapping(properties.get("source_refs", {}).get("properties"), "schema source refs")
    for key, expected in SOURCE_REFS.items():
        if source_props.get(key, {}).get("const") != expected:
            raise ValidationError(f"schema: source ref {key!r} const drifted")

    schema_subjects = frozenset(defs["subject"]["properties"]["kind"]["enum"])
    schema_cases = frozenset(defs["case"]["properties"]["kind"]["enum"])
    schema_dispositions = frozenset(defs["disposition"]["properties"]["class"]["enum"])
    schema_predicates = frozenset(defs["assertion"]["properties"]["predicate"]["enum"])
    if schema_subjects != SUBJECT_KINDS:
        raise ValidationError("schema: subject-kind registry differs from validator")
    if schema_cases != CASE_KINDS:
        raise ValidationError("schema: case-kind registry differs from validator")
    if schema_dispositions != DISPOSITIONS:
        raise ValidationError("schema: disposition registry differs from validator")
    if schema_predicates != PREDICATES:
        raise ValidationError("schema: predicate registry differs from validator")


def _validate_subjects(value: Any) -> set[str]:
    subjects = _mapping(value, "subjects")
    if not subjects:
        raise ValidationError("subjects: must not be empty")
    for subject_id, raw_subject in subjects.items():
        if not SUBJECT_ID_RE.fullmatch(subject_id):
            raise ValidationError(f"subjects.{subject_id}: malformed subject ID")
        ctx = f"subjects.{subject_id}"
        subject = _mapping(raw_subject, ctx)
        _exact_keys(subject, {"kind", "semantic_ref", "description"}, {"runtime_identity_semantics"}, ctx)
        if subject["kind"] not in SUBJECT_KINDS:
            raise ValidationError(f"{ctx}.kind: unregistered kind {subject['kind']!r}")
        _validate_semantic_ref(subject["semantic_ref"], f"{ctx}.semantic_ref")
        _nonempty_string(subject["description"], f"{ctx}.description")
        if "runtime_identity_semantics" in subject:
            if subject["runtime_identity_semantics"] != RUNTIME_IDENTITY_SENTENCE:
                raise ValidationError(f"{ctx}.runtime_identity_semantics: unexpected semantics")
    return set(subjects)


def _validate_disposition(value: Any, ctx: str) -> None:
    disposition = _mapping(value, ctx)
    _exact_keys(disposition, {"class", "reason_code"}, set(), ctx)
    disposition_class = disposition["class"]
    if disposition_class not in DISPOSITIONS:
        raise ValidationError(f"{ctx}.class: unknown disposition {disposition_class!r}")
    reason = _nonempty_string(disposition["reason_code"], f"{ctx}.reason_code")
    if not REASON_CODE_RE.fullmatch(reason):
        raise ValidationError(f"{ctx}.reason_code: malformed reason code")
    if reason not in REASON_CODES:
        raise ValidationError(f"{ctx}.reason_code: unregistered reason code {reason!r}")


def _validate_assertion(value: Any, subject_ids: set[str], ctx: str) -> None:
    assertion = _mapping(value, ctx)
    _exact_keys(assertion, {"predicate", "subjects", "note"}, set(), ctx)
    predicate = assertion["predicate"]
    if predicate not in PREDICATES:
        raise ValidationError(f"{ctx}.predicate: unknown predicate {predicate!r}")
    refs = _list(assertion["subjects"], f"{ctx}.subjects")
    if not refs:
        raise ValidationError(f"{ctx}.subjects: must not be empty")
    for ref in refs:
        if not isinstance(ref, str) or ref not in subject_ids:
            raise ValidationError(f"{ctx}.subjects: dangling subject reference {ref!r}")
    _nonempty_string(assertion["note"], f"{ctx}.note")


def _validate_cases(value: Any, subject_ids: set[str]) -> None:
    cases = _mapping(value, "cases")
    if set(cases) != EXPECTED_CASE_IDS:
        missing = sorted(EXPECTED_CASE_IDS - set(cases))
        extra = sorted(set(cases) - EXPECTED_CASE_IDS)
        raise ValidationError(f"cases: I0 v1 case set drifted; missing={missing}, extra={extra}")
    for case_id, raw_case in cases.items():
        if not CASE_ID_RE.fullmatch(case_id):
            raise ValidationError(f"cases.{case_id}: malformed case ID")
        ctx = f"cases.{case_id}"
        case = _mapping(raw_case, ctx)
        _exact_keys(case, {"kind", "summary", "inputs", "expected_disposition", "assertions"}, set(), ctx)
        if case["kind"] not in CASE_KINDS:
            raise ValidationError(f"{ctx}.kind: unknown case kind {case['kind']!r}")
        _nonempty_string(case["summary"], f"{ctx}.summary")
        inputs = _list(case["inputs"], f"{ctx}.inputs")
        if not inputs:
            raise ValidationError(f"{ctx}.inputs: must not be empty")
        for ref in inputs:
            if not isinstance(ref, str) or ref not in subject_ids:
                raise ValidationError(f"{ctx}.inputs: dangling subject reference {ref!r}")
        _validate_disposition(case["expected_disposition"], f"{ctx}.expected_disposition")
        assertions = _list(case["assertions"], f"{ctx}.assertions")
        if not assertions:
            raise ValidationError(f"{ctx}.assertions: must not be empty")
        for index, assertion in enumerate(assertions):
            _validate_assertion(assertion, subject_ids, f"{ctx}.assertions[{index}]")


def validate_document(document: Any) -> dict[str, int | str]:
    root = _mapping(document, "corpus")
    _exact_keys(root, {"$schema", "corpus_id", "corpus_version", "profile", "source_refs", "subjects", "cases", "nonclaims"}, set(), "corpus")
    if root["$schema"] != CORPUS_SCHEMA_REF:
        raise ValidationError("corpus.$schema: unexpected schema reference")
    if root["corpus_id"] != CORPUS_ID:
        raise ValidationError("corpus.corpus_id: unexpected corpus identity")
    if root["corpus_version"] != CORPUS_VERSION:
        raise ValidationError("corpus.corpus_version: unexpected corpus version")
    if root["profile"] != CORPUS_PROFILE:
        raise ValidationError("corpus.profile: unexpected profile")

    source_refs = _mapping(root["source_refs"], "corpus.source_refs")
    _exact_keys(source_refs, set(SOURCE_REFS), set(), "corpus.source_refs")
    if source_refs != SOURCE_REFS:
        raise ValidationError(f"corpus.source_refs: expected {SOURCE_REFS}, got {source_refs}")

    subject_ids = _validate_subjects(root["subjects"])
    _validate_cases(root["cases"], subject_ids)

    nonclaims = _list(root["nonclaims"], "corpus.nonclaims")
    if not nonclaims:
        raise ValidationError("corpus.nonclaims: must not be empty")
    if len(nonclaims) != len(set(nonclaims)):
        raise ValidationError("corpus.nonclaims: duplicate entries")
    for index, value in enumerate(nonclaims):
        _nonempty_string(value, f"corpus.nonclaims[{index}]")

    return {"subjects": len(subject_ids), "cases": len(root["cases"])}


def validate_bytes(corpus_raw: bytes, *, schema_raw: bytes | None = None, expected_sha256: str | None = None) -> dict[str, int | str]:
    digest = hashlib.sha256(corpus_raw).hexdigest()
    if expected_sha256 is not None:
        if not SHA256_RE.fullmatch(expected_sha256):
            raise ValidationError("expected SHA-256 must be 64 lowercase hex characters")
        if digest != expected_sha256:
            raise ValidationError(f"corpus commitment mismatch: expected {expected_sha256}, got {digest}")

    document = parse_strict_json(corpus_raw, label="corpus")
    if schema_raw is not None:
        schema = _mapping(parse_strict_json(schema_raw, label="schema"), "schema")
        _validate_schema_registry(schema)
    counts = validate_document(document)
    return {
        "validator_profile": VALIDATOR_PROFILE,
        "corpus_id": CORPUS_ID,
        "corpus_version": CORPUS_VERSION,
        "subjects": counts["subjects"],
        "cases": counts["cases"],
        "sha256": digest,
    }


def validate_path(path: Path, *, expected_sha256: str | None = None) -> dict[str, int | str]:
    corpus_raw = path.read_bytes()
    schema_path = path.with_name(SCHEMA_FILENAME)
    if not schema_path.is_file():
        raise ValidationError(f"schema not found beside corpus: {schema_path}")
    return validate_bytes(corpus_raw, schema_raw=schema_path.read_bytes(), expected_sha256=expected_sha256)


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("corpus", nargs="?", type=Path, default=Path(__file__).with_name(CORPUS_FILENAME), help="I0 corpus JSON path (defaults to sibling v1 corpus)")
    parser.add_argument("--expected-sha256", help="bind validation to exact corpus bytes")
    parser.add_argument("--json", action="store_true", help="emit machine-readable summary")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    try:
        summary = validate_path(args.corpus, expected_sha256=args.expected_sha256)
    except (OSError, ValidationError) as exc:
        print(f"INVALID: {exc}", file=sys.stderr)
        return 2

    if args.json:
        print(json.dumps(summary, sort_keys=True, separators=(",", ":")))
    else:
        print(
            "VALID "
            f"profile={summary['validator_profile']} "
            f"corpus={summary['corpus_id']}@{summary['corpus_version']} "
            f"subjects={summary['subjects']} cases={summary['cases']} "
            f"sha256={summary['sha256']}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
