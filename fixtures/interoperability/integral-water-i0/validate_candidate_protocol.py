#!/usr/bin/env python3
"""Strict validator for MYC-INT-006I neutral candidate input/results."""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import sys
from pathlib import Path
from typing import Any

VALIDATOR_PROFILE = "myc-int-006k-neutral-protocol-validator-v1"

INPUT_SCHEMA_REF = "./myc-int-006i-i0-candidate-input.schema.json"
INPUT_ID = "myc-int-i0-candidate-input"
INPUT_VERSION = "1.0.0"
INPUT_PROFILE = "runtime-neutral-candidate-input-v1"
CORPUS_ID = "myc-int-006c-water-i0"
CORPUS_VERSION = "1.0.0"
STIMULUS_ID = "myc-int-006e-water-i0-stimulus"
STIMULUS_VERSION = "1.0.0"

RESULT_SCHEMA_REF = "./myc-int-006i-i0-candidate-results.schema.json"
RESULT_ID = "myc-int-i0-candidate-results"
RESULT_VERSION = "1.0.0"
RESULT_PROFILE = "runtime-neutral-candidate-results-v1"

SUBJECT_KINDS = frozenset({
    "Resource", "Issue", "Observation", "ActorReport", "EvidenceBundle",
    "Alternative", "Objection", "Prediction", "Recommendation", "Decision",
    "ForeignDecision", "Authorization", "ImplementationAttempt",
    "ImplementationReceipt", "OutcomeObservation", "ReviewCandidate",
    "SupersedingDecisionCandidate", "Credential", "Certification",
    "DerivedSummary", "ForeignAuthority", "DeliveryReceipt",
})
OPERATIONS = frozenset({
    "RecordLineage", "ExecuteRecommendation", "ImportForeignDecisionAsLocalAuthority",
    "UseCredentialAsAuthority", "DeliverImplementationAttempt", "ClassifyTimeoutOutcome",
    "AdmitStaleSchema", "AcceptExternalCertificationLocally",
    "TreatDerivedSummaryAsSourceFact", "ExecuteWithExpiredAuthorization",
    "RewriteHistoricalDecisionFromOutcome", "ProcessReorderedTransport",
    "CollapseSchemaQualifiedIdentity", "TranslateWithUndeclaredLoss",
    "ImportForeignAuthorityAsLocal", "ResolvePartitionByLastArrival",
    "PromotePredictionToObservation", "PromoteDeliveryReceiptToImplementationReceipt",
    "PromoteExecutionReceiptToDesiredOutcome",
})
DISPOSITIONS = frozenset({"Accepted", "Rejected", "Indeterminate", "Unsupported"})
FACT_TYPES: dict[str, type] = {
    "distinct_semantic_subjects": bool,
    "effect_authority_granted": bool,
    "authorization_subject_bound": bool,
    "source_schema_preserved": bool,
    "provenance_preserved": bool,
    "unknown_state_preserved": bool,
    "conflict_preserved": bool,
    "logical_effect_count": int,
    "historical_mutation": bool,
    "review_candidate_created": bool,
    "local_authority_granted": bool,
    "observation_promotion": bool,
    "receipt_promotion": bool,
    "outcome_promotion": bool,
    "translation_loss_declared": bool,
    "idempotent_replay": bool,
    "expired_authority_rejected": bool,
    "stale_schema_rejected": bool,
}
FORBIDDEN_KEYS = frozenset({
    "expected_disposition", "expected_result", "expected", "oracle", "reason_code",
    "assertion", "assertions", "predicate", "predicates",
})

SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
SUBJECT_ID_RE = re.compile(r"^[a-z0-9][a-z0-9._-]{2,63}$")
CASE_ID_RE = re.compile(r"^I0-[A-Z]+-[0-9]{3}$")
SEMANTIC_RE = re.compile(r"^[a-z0-9][a-z0-9._-]*$")
IMPLEMENTATION_ID_RE = re.compile(r"^[a-z0-9][a-z0-9._-]{2,127}$")


class ProtocolValidationError(ValueError):
    """Neutral candidate protocol validation failure."""


def _strict_object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    out: dict[str, Any] = {}
    for key, value in pairs:
        if key in out:
            raise ProtocolValidationError(f"duplicate JSON object member: {key!r}")
        out[key] = value
    return out


def parse_strict_json(raw: bytes, *, label: str) -> Any:
    try:
        text = raw.decode("utf-8", errors="strict")
    except UnicodeDecodeError as exc:
        raise ProtocolValidationError(f"{label}: invalid UTF-8: {exc}") from exc
    try:
        return json.loads(text, object_pairs_hook=_strict_object)
    except ProtocolValidationError:
        raise
    except json.JSONDecodeError as exc:
        raise ProtocolValidationError(f"{label}: invalid JSON: {exc}") from exc


def canonical_bytes(document: Any) -> bytes:
    return (json.dumps(document, indent=2, sort_keys=True) + "\n").encode("utf-8")


def _object(value: Any, ctx: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ProtocolValidationError(f"{ctx}: expected object")
    return value


def _array(value: Any, ctx: str) -> list[Any]:
    if not isinstance(value, list):
        raise ProtocolValidationError(f"{ctx}: expected array")
    return value


def _string(value: Any, ctx: str, *, min_len: int = 1, max_len: int | None = None) -> str:
    if not isinstance(value, str) or len(value) < min_len:
        raise ProtocolValidationError(f"{ctx}: expected string length >= {min_len}")
    if max_len is not None and len(value) > max_len:
        raise ProtocolValidationError(f"{ctx}: string exceeds {max_len} characters")
    return value


def _int(value: Any, ctx: str, *, minimum: int = 0) -> int:
    if type(value) is not int or value < minimum:
        raise ProtocolValidationError(f"{ctx}: expected integer >= {minimum}")
    return value


def _bool(value: Any, ctx: str) -> bool:
    if type(value) is not bool:
        raise ProtocolValidationError(f"{ctx}: expected boolean")
    return value


def _exact_keys(obj: dict[str, Any], required: set[str], optional: set[str], ctx: str) -> None:
    keys = set(obj)
    missing = required - keys
    unknown = keys - required - optional
    if missing:
        raise ProtocolValidationError(f"{ctx}: missing fields: {sorted(missing)}")
    if unknown:
        raise ProtocolValidationError(f"{ctx}: unknown fields: {sorted(unknown)}")


def _reject_forbidden(value: Any, ctx: str = "$") -> None:
    if isinstance(value, dict):
        for key, child in value.items():
            if key in FORBIDDEN_KEYS or key.startswith("expected_"):
                raise ProtocolValidationError(f"{ctx}: oracle-bearing key forbidden: {key!r}")
            _reject_forbidden(child, f"{ctx}.{key}")
    elif isinstance(value, list):
        for index, child in enumerate(value):
            _reject_forbidden(child, f"{ctx}[{index}]")


def _sha256(value: Any, ctx: str) -> str:
    text = _string(value, ctx)
    if not SHA256_RE.fullmatch(text):
        raise ProtocolValidationError(f"{ctx}: expected lowercase SHA-256")
    return text


def _semantic_ref(value: Any, ctx: str) -> None:
    ref = _object(value, ctx)
    _exact_keys(ref, {"namespace", "name", "version"}, set(), ctx)
    namespace = _string(ref["namespace"], f"{ctx}.namespace")
    name = _string(ref["name"], f"{ctx}.name")
    _string(ref["version"], f"{ctx}.version", max_len=128)
    if not SEMANTIC_RE.fullmatch(namespace):
        raise ProtocolValidationError(f"{ctx}.namespace: malformed")
    if not SEMANTIC_RE.fullmatch(name):
        raise ProtocolValidationError(f"{ctx}.name: malformed")


def _enum_string(value: Any, allowed: set[str] | frozenset[str], ctx: str) -> str:
    text = _string(value, ctx)
    if text not in allowed:
        raise ProtocolValidationError(f"{ctx}: unknown value {text!r}")
    return text


def _transport(value: Any, subject_ids: set[str], ctx: str) -> None:
    obj = _object(value, ctx)
    allowed = {
        "delivery_count", "semantic_attempt_count", "receiver_persistence",
        "acknowledgement", "sender_result", "delivery_order", "total_order_promised",
        "partitioned", "reconnected", "proposed_resolution",
    }
    _exact_keys(obj, set(), allowed, ctx)
    if not obj:
        raise ProtocolValidationError(f"{ctx}: must not be empty")
    if "delivery_count" in obj:
        _int(obj["delivery_count"], f"{ctx}.delivery_count")
    if "semantic_attempt_count" in obj:
        _int(obj["semantic_attempt_count"], f"{ctx}.semantic_attempt_count")
    if "receiver_persistence" in obj:
        _enum_string(obj["receiver_persistence"], {"possible", "known", "absent"}, f"{ctx}.receiver_persistence")
    if "acknowledgement" in obj:
        _enum_string(obj["acknowledgement"], {"present", "missing"}, f"{ctx}.acknowledgement")
    if "sender_result" in obj:
        _enum_string(obj["sender_result"], {"success", "failure", "timeout"}, f"{ctx}.sender_result")
    if "delivery_order" in obj:
        order = _array(obj["delivery_order"], f"{ctx}.delivery_order")
        if not order:
            raise ProtocolValidationError(f"{ctx}.delivery_order: must not be empty")
        for ref in order:
            if not isinstance(ref, str) or ref not in subject_ids:
                raise ProtocolValidationError(f"{ctx}.delivery_order: dangling subject {ref!r}")
    for key in ("total_order_promised", "partitioned", "reconnected"):
        if key in obj:
            _bool(obj[key], f"{ctx}.{key}")
    if "proposed_resolution" in obj:
        _enum_string(obj["proposed_resolution"], {"last_arrival_wins"}, f"{ctx}.proposed_resolution")


def _authority(value: Any, ctx: str) -> None:
    obj = _object(value, ctx)
    _exact_keys(obj, set(), {"status"}, ctx)
    if not obj:
        raise ProtocolValidationError(f"{ctx}: must not be empty")
    if "status" in obj:
        _enum_string(obj["status"], {"current", "expired", "revoked"}, f"{ctx}.status")


def _schema_condition(value: Any, ctx: str) -> None:
    obj = _object(value, ctx)
    allowed = {"supplied_generation", "required_generation", "visible_identifier_equal", "source_schema_equal"}
    _exact_keys(obj, set(), allowed, ctx)
    if not obj:
        raise ProtocolValidationError(f"{ctx}: must not be empty")
    for key in ("supplied_generation", "required_generation"):
        if key in obj:
            _string(obj[key], f"{ctx}.{key}")
    for key in ("visible_identifier_equal", "source_schema_equal"):
        if key in obj:
            _bool(obj[key], f"{ctx}.{key}")


def _translation(value: Any, ctx: str) -> None:
    obj = _object(value, ctx)
    _exact_keys(obj, set(), {"declared_loss", "actual_loss"}, ctx)
    if not obj:
        raise ProtocolValidationError(f"{ctx}: must not be empty")
    if "declared_loss" in obj:
        _bool(obj["declared_loss"], f"{ctx}.declared_loss")
    if "actual_loss" in obj:
        losses = _array(obj["actual_loss"], f"{ctx}.actual_loss")
        if len(losses) != len(set(losses)):
            raise ProtocolValidationError(f"{ctx}.actual_loss: duplicate values")
        for loss in losses:
            _enum_string(loss, {"source_schema", "provenance"}, f"{ctx}.actual_loss[]")


def validate_candidate_input_document(document: Any, *, corpus_raw: bytes | None = None, stimulus_raw: bytes | None = None) -> dict[str, int]:
    root = _object(document, "candidate_input")
    _reject_forbidden(root)
    required = {
        "$schema", "candidate_input_id", "candidate_input_version", "profile",
        "corpus_id", "corpus_version", "stimulus_id", "stimulus_version",
        "source_commitments", "subjects", "cases",
    }
    _exact_keys(root, required, set(), "candidate_input")
    constants = {
        "$schema": INPUT_SCHEMA_REF,
        "candidate_input_id": INPUT_ID,
        "candidate_input_version": INPUT_VERSION,
        "profile": INPUT_PROFILE,
        "corpus_id": CORPUS_ID,
        "corpus_version": CORPUS_VERSION,
        "stimulus_id": STIMULUS_ID,
        "stimulus_version": STIMULUS_VERSION,
    }
    for key, expected in constants.items():
        if root[key] != expected:
            raise ProtocolValidationError(f"candidate_input.{key}: expected {expected!r}")

    commitments = _object(root["source_commitments"], "candidate_input.source_commitments")
    _exact_keys(commitments, {"corpus_sha256", "stimulus_sha256"}, set(), "candidate_input.source_commitments")
    corpus_digest = _sha256(commitments["corpus_sha256"], "candidate_input.source_commitments.corpus_sha256")
    stimulus_digest = _sha256(commitments["stimulus_sha256"], "candidate_input.source_commitments.stimulus_sha256")
    if corpus_raw is not None and corpus_digest != hashlib.sha256(corpus_raw).hexdigest():
        raise ProtocolValidationError("candidate_input: corpus commitment mismatch")
    if stimulus_raw is not None and stimulus_digest != hashlib.sha256(stimulus_raw).hexdigest():
        raise ProtocolValidationError("candidate_input: stimulus commitment mismatch")

    subjects = _object(root["subjects"], "candidate_input.subjects")
    if not subjects:
        raise ProtocolValidationError("candidate_input.subjects: must not be empty")
    subject_ids = set(subjects)
    for subject_id, raw_subject in subjects.items():
        if not SUBJECT_ID_RE.fullmatch(subject_id):
            raise ProtocolValidationError(f"candidate_input.subjects: malformed subject ID {subject_id!r}")
        ctx = f"candidate_input.subjects.{subject_id}"
        subject = _object(raw_subject, ctx)
        _exact_keys(subject, {"kind", "semantic_ref"}, set(), ctx)
        _enum_string(subject["kind"], SUBJECT_KINDS, f"{ctx}.kind")
        _semantic_ref(subject["semantic_ref"], f"{ctx}.semantic_ref")

    cases = _object(root["cases"], "candidate_input.cases")
    if not cases:
        raise ProtocolValidationError("candidate_input.cases: must not be empty")
    for case_id, raw_case in cases.items():
        if not CASE_ID_RE.fullmatch(case_id):
            raise ProtocolValidationError(f"candidate_input.cases: malformed case ID {case_id!r}")
        ctx = f"candidate_input.cases.{case_id}"
        case = _object(raw_case, ctx)
        _exact_keys(case, {"operation", "subjects"}, {"transport", "authority", "schema", "translation"}, ctx)
        _enum_string(case["operation"], OPERATIONS, f"{ctx}.operation")
        refs = _array(case["subjects"], f"{ctx}.subjects")
        if not refs:
            raise ProtocolValidationError(f"{ctx}.subjects: must not be empty")
        for ref in refs:
            if not isinstance(ref, str) or ref not in subject_ids:
                raise ProtocolValidationError(f"{ctx}.subjects: dangling subject {ref!r}")
        if "transport" in case:
            _transport(case["transport"], subject_ids, f"{ctx}.transport")
        if "authority" in case:
            _authority(case["authority"], f"{ctx}.authority")
        if "schema" in case:
            _schema_condition(case["schema"], f"{ctx}.schema")
        if "translation" in case:
            _translation(case["translation"], f"{ctx}.translation")
    return {"subjects": len(subjects), "cases": len(cases)}


def validate_candidate_input_bytes(raw: bytes, *, corpus_raw: bytes | None = None, stimulus_raw: bytes | None = None) -> dict[str, int | str]:
    document = parse_strict_json(raw, label="candidate_input")
    counts = validate_candidate_input_document(document, corpus_raw=corpus_raw, stimulus_raw=stimulus_raw)
    return {"validator_profile": VALIDATOR_PROFILE, "kind": "candidate_input", "sha256": hashlib.sha256(raw).hexdigest(), **counts}


def _implementation(value: Any, ctx: str) -> None:
    obj = _object(value, ctx)
    _exact_keys(obj, {"implementation_id", "family", "version"}, set(), ctx)
    implementation_id = _string(obj["implementation_id"], f"{ctx}.implementation_id", max_len=128)
    if not IMPLEMENTATION_ID_RE.fullmatch(implementation_id):
        raise ProtocolValidationError(f"{ctx}.implementation_id: malformed")
    _string(obj["family"], f"{ctx}.family", max_len=128)
    _string(obj["version"], f"{ctx}.version", max_len=128)


def _facts(value: Any, ctx: str) -> None:
    obj = _object(value, ctx)
    _exact_keys(obj, set(), set(FACT_TYPES), ctx)
    for key, val in obj.items():
        if FACT_TYPES[key] is bool:
            _bool(val, f"{ctx}.{key}")
        else:
            _int(val, f"{ctx}.{key}")


def validate_candidate_results_document(document: Any, *, candidate_input_raw: bytes) -> dict[str, int]:
    root = _object(document, "candidate_results")
    _reject_forbidden(root)
    required = {
        "$schema", "result_protocol_id", "result_protocol_version", "profile",
        "implementation", "candidate_input_id", "candidate_input_version",
        "candidate_input_sha256", "results",
    }
    _exact_keys(root, required, set(), "candidate_results")
    constants = {
        "$schema": RESULT_SCHEMA_REF,
        "result_protocol_id": RESULT_ID,
        "result_protocol_version": RESULT_VERSION,
        "profile": RESULT_PROFILE,
        "candidate_input_id": INPUT_ID,
        "candidate_input_version": INPUT_VERSION,
    }
    for key, expected in constants.items():
        if root[key] != expected:
            raise ProtocolValidationError(f"candidate_results.{key}: expected {expected!r}")
    _implementation(root["implementation"], "candidate_results.implementation")
    input_digest = _sha256(root["candidate_input_sha256"], "candidate_results.candidate_input_sha256")
    if input_digest != hashlib.sha256(candidate_input_raw).hexdigest():
        raise ProtocolValidationError("candidate_results: candidate input commitment mismatch")

    input_document = parse_strict_json(candidate_input_raw, label="candidate_input")
    validate_candidate_input_document(input_document)
    expected_case_ids = set(input_document["cases"])
    rows = _array(root["results"], "candidate_results.results")
    if not rows:
        raise ProtocolValidationError("candidate_results.results: must not be empty")
    seen: set[str] = set()
    for index, raw_row in enumerate(rows):
        ctx = f"candidate_results.results[{index}]"
        row = _object(raw_row, ctx)
        _exact_keys(row, {"case_id", "disposition", "implementation_rule", "effect_count", "tags", "facts"}, set(), ctx)
        case_id = _string(row["case_id"], f"{ctx}.case_id")
        if not CASE_ID_RE.fullmatch(case_id):
            raise ProtocolValidationError(f"{ctx}.case_id: malformed")
        if case_id in seen:
            raise ProtocolValidationError(f"{ctx}.case_id: duplicate result for {case_id}")
        seen.add(case_id)
        _enum_string(row["disposition"], DISPOSITIONS, f"{ctx}.disposition")
        _string(row["implementation_rule"], f"{ctx}.implementation_rule", max_len=256)
        _int(row["effect_count"], f"{ctx}.effect_count")
        tags = _array(row["tags"], f"{ctx}.tags")
        values = [_string(tag, f"{ctx}.tags[]", max_len=128) for tag in tags]
        if len(values) != len(set(values)):
            raise ProtocolValidationError(f"{ctx}.tags: duplicate tags")
        _facts(row["facts"], f"{ctx}.facts")
    if seen != expected_case_ids:
        raise ProtocolValidationError(
            f"candidate_results: case-set mismatch; missing={sorted(expected_case_ids-seen)}, extra={sorted(seen-expected_case_ids)}"
        )
    return {"cases": len(rows), "facts_registered": len(FACT_TYPES)}


def validate_candidate_results_bytes(raw: bytes, *, candidate_input_raw: bytes) -> dict[str, int | str]:
    document = parse_strict_json(raw, label="candidate_results")
    counts = validate_candidate_results_document(document, candidate_input_raw=candidate_input_raw)
    return {"validator_profile": VALIDATOR_PROFILE, "kind": "candidate_results", "sha256": hashlib.sha256(raw).hexdigest(), **counts}


def validate_schema_registries(input_schema_raw: bytes, result_schema_raw: bytes) -> None:
    input_schema = _object(parse_strict_json(input_schema_raw, label="input_schema"), "input_schema")
    result_schema = _object(parse_strict_json(result_schema_raw, label="result_schema"), "result_schema")
    if input_schema.get("$schema") != "https://json-schema.org/draft/2020-12/schema":
        raise ProtocolValidationError("input_schema: expected Draft 2020-12")
    if result_schema.get("$schema") != "https://json-schema.org/draft/2020-12/schema":
        raise ProtocolValidationError("result_schema: expected Draft 2020-12")
    ip = _object(input_schema.get("properties"), "input_schema.properties")
    rp = _object(result_schema.get("properties"), "result_schema.properties")
    input_consts = {
        "$schema": INPUT_SCHEMA_REF, "candidate_input_id": INPUT_ID,
        "candidate_input_version": INPUT_VERSION, "profile": INPUT_PROFILE,
        "corpus_id": CORPUS_ID, "corpus_version": CORPUS_VERSION,
        "stimulus_id": STIMULUS_ID, "stimulus_version": STIMULUS_VERSION,
    }
    result_consts = {
        "$schema": RESULT_SCHEMA_REF, "result_protocol_id": RESULT_ID,
        "result_protocol_version": RESULT_VERSION, "profile": RESULT_PROFILE,
        "candidate_input_id": INPUT_ID, "candidate_input_version": INPUT_VERSION,
    }
    for key, expected in input_consts.items():
        if _object(ip.get(key), f"input_schema.properties.{key}").get("const") != expected:
            raise ProtocolValidationError(f"input_schema: const drift for {key}")
    for key, expected in result_consts.items():
        if _object(rp.get(key), f"result_schema.properties.{key}").get("const") != expected:
            raise ProtocolValidationError(f"result_schema: const drift for {key}")
    idefs = _object(input_schema.get("$defs"), "input_schema.$defs")
    rdefs = _object(result_schema.get("$defs"), "result_schema.$defs")
    if frozenset(idefs["subject"]["properties"]["kind"]["enum"]) != SUBJECT_KINDS:
        raise ProtocolValidationError("input_schema: subject-kind registry drift")
    if frozenset(idefs["operation"]["enum"]) != OPERATIONS:
        raise ProtocolValidationError("input_schema: operation registry drift")
    if frozenset(rdefs["disposition"]["enum"]) != DISPOSITIONS:
        raise ProtocolValidationError("result_schema: disposition registry drift")
    if frozenset(rdefs["facts"]["properties"].keys()) != frozenset(FACT_TYPES):
        raise ProtocolValidationError("result_schema: semantic-fact registry drift")


def validate_pair(candidate_input_raw: bytes, candidate_results_raw: bytes, *, input_schema_raw: bytes | None = None, result_schema_raw: bytes | None = None, corpus_raw: bytes | None = None, stimulus_raw: bytes | None = None) -> dict[str, Any]:
    if (input_schema_raw is None) != (result_schema_raw is None):
        raise ProtocolValidationError("both neutral schemas must be supplied together")
    if input_schema_raw is not None and result_schema_raw is not None:
        validate_schema_registries(input_schema_raw, result_schema_raw)
    input_summary = validate_candidate_input_bytes(candidate_input_raw, corpus_raw=corpus_raw, stimulus_raw=stimulus_raw)
    result_summary = validate_candidate_results_bytes(candidate_results_raw, candidate_input_raw=candidate_input_raw)
    return {"validator_profile": VALIDATOR_PROFILE, "status": "PASS", "input": input_summary, "results": result_summary}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--results", type=Path, required=True)
    parser.add_argument("--input-schema", type=Path)
    parser.add_argument("--result-schema", type=Path)
    parser.add_argument("--corpus", type=Path)
    parser.add_argument("--stimulus", type=Path)
    args = parser.parse_args(argv)
    try:
        summary = validate_pair(
            args.input.read_bytes(), args.results.read_bytes(),
            input_schema_raw=args.input_schema.read_bytes() if args.input_schema else None,
            result_schema_raw=args.result_schema.read_bytes() if args.result_schema else None,
            corpus_raw=args.corpus.read_bytes() if args.corpus else None,
            stimulus_raw=args.stimulus.read_bytes() if args.stimulus else None,
        )
    except (OSError, ProtocolValidationError) as exc:
        print(f"INVALID: {exc}", file=sys.stderr)
        return 1
    sys.stdout.buffer.write(canonical_bytes(summary))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
