#!/usr/bin/env python3
"""Independent integrity/census verifier for the D6S-CANON-1 vector corpus."""
from __future__ import annotations

import hashlib
import json
import sys
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[2]
MANIFEST = ROOT / "docs/integral/d6s-canon-1-manifest.json"


def unique_pairs(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate JSON key: {key}")
        result[key] = value
    return result


def load(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"), object_pairs_hook=unique_pairs,
                      parse_constant=lambda token: (_ for _ in ()).throw(ValueError(f"invalid constant: {token}")))


def git_blob_sha(data: bytes) -> str:
    return hashlib.sha1(b"blob " + str(len(data)).encode("ascii") + b"\0" + data).hexdigest()


def main() -> int:
    manifest = load(MANIFEST)
    if manifest.get("manifest_version") != "d6s-canon-1-manifest/1":
        raise ValueError("MANIFEST_VERSION_MISMATCH")
    if manifest.get("corpus_id") != "D6S-CANON-1":
        raise ValueError("CORPUS_ID_MISMATCH")
    path_text = manifest.get("corpus_path")
    if path_text != "docs/integral/d6s-canon-1-golden-vectors.json":
        raise ValueError("CORPUS_PATH_NOT_ALLOWLISTED")
    path = (ROOT / path_text).resolve()
    if ROOT.resolve() not in path.parents or not path.is_file():
        raise ValueError("CORPUS_PATH_INVALID_OR_MISSING")
    actual_blob = git_blob_sha(path.read_bytes())
    if actual_blob != manifest.get("corpus_git_blob_sha"):
        raise ValueError(f"CORPUS_BLOB_PIN_MISMATCH expected={manifest.get('corpus_git_blob_sha')} actual={actual_blob}")
    corpus = load(path)
    if corpus.get("schema_version") != "d6s-canon-1-golden-vectors/1":
        raise ValueError("CORPUS_SCHEMA_VERSION_MISMATCH")
    if corpus.get("corpus_id") != manifest["corpus_id"] or corpus.get("status") != "ReferenceModelOnly":
        raise ValueError("CORPUS_METADATA_MISMATCH")
    vectors = corpus.get("vectors")
    if not isinstance(vectors, list) or len(vectors) != 16 or corpus.get("vector_count") != 16:
        raise ValueError("VECTOR_COUNT_MISMATCH")
    ids = [vector.get("test_id") for vector in vectors]
    expected_ids = manifest.get("required_test_ids")
    if ids != expected_ids or len(set(ids)) != len(ids):
        raise ValueError("VECTOR_ID_CENSUS_MISMATCH")
    required = set(manifest.get("required_formal_obligations", []))
    if required != {f"COS-FV-{n:03d}" for n in range(1, 11)}:
        raise ValueError("FORMAL_OBLIGATION_MANIFEST_INCOMPLETE")
    covered: set[str] = set()
    allowed_bindings = {
        "plan_execution", "requirement_availability", "assignment_observed_work",
        "plan_consumption", "output_qualification", "foreign_recognition",
        "itc_projection", "frs_projection", "recommendation_authorized",
        "effect_recorded", "quality_current", "denominator_explicit",
        "general_capability_evidence", "failure_history_preserved",
        "source_observation_created", "physical_work_binding",
        "external_dependency_classified", "local_capability_claim", "local_origin_claim",
    }
    allowed_evidence_profiles = {"current_local", "stale_local", "current_foreign_node_b"}
    required_positive_bindings = {
        # Valid counterexamples retain the full context they are refining.
        "COS-N-009": {"external_dependency_classified", "foreign_recognition"},
        "COS-N-010": {"frs_projection", "source_observation_created"},
        "COS-N-011": {"source_observation_created", "itc_projection"},
        "COS-N-012": {"itc_projection", "physical_work_binding"},
        "COS-N-013": {"foreign_recognition"},
    }
    required_negative_bindings = {
        # Invalid claims must be exercised in the presence of the evidence they misuse.
        "COS-N-009": {"local_capability_claim"},
        "COS-N-010": {"frs_projection"},
        "COS-N-011": {"source_observation_created"},
        "COS-N-012": {"itc_projection"},
        "COS-N-013": {"local_origin_claim"},
        "COS-N-014": {"frs_projection"},
    }
    allowed_decisions = set(corpus.get("decision_enum", []))
    if allowed_decisions != {"Accepted", "Rejected", "Unknown", "Conflicting", "Stale", "Superseded", "Unauthorized", "Unbound"}:
        raise ValueError("DECISION_ENUM_MISMATCH")
    for vector in vectors:
        obligations = vector.get("formal_obligations")
        if not isinstance(obligations, list) or not obligations or not set(obligations) <= required:
            raise ValueError(f"INVALID_FORMAL_MAPPING {vector.get('test_id')}")
        covered.update(obligations)
        if vector.get("expected_negative") not in allowed_decisions or vector.get("expected_positive") != "Accepted":
            raise ValueError(f"INVALID_EXPECTED_DECISIONS {vector.get('test_id')}")
        for side in ("negative_input", "positive_input"):
            input_record = vector.get(side)
            if not isinstance(input_record, dict):
                raise ValueError(f"EXPLICIT_INPUT_MISSING {vector.get('test_id')}/{side}")
            names = input_record.get("bindings")
            if not isinstance(names, list) or any(not isinstance(name, str) for name in names):
                raise ValueError(f"INVALID_BINDING_LIST {vector.get('test_id')}/{side}")
            if len(names) != len(set(names)) or not set(names) <= allowed_bindings:
                raise ValueError(f"UNKNOWN_OR_DUPLICATE_BINDING {vector.get('test_id')}/{side}")
            if input_record.get("evidence_profile") not in allowed_evidence_profiles:
                raise ValueError(f"UNKNOWN_EVIDENCE_PROFILE {vector.get('test_id')}/{side}")
        test_id = vector.get("test_id")
        if test_id == "COS-N-005" and vector["negative_input"]["evidence_profile"] != "stale_local":
            raise ValueError("STALE_EVIDENCE_NEGATIVE_CONTROL_MISSING")
        if test_id in {"COS-N-009", "COS-N-013"}:
            if vector["negative_input"]["evidence_profile"] != "current_foreign_node_b" or vector["positive_input"]["evidence_profile"] != "current_foreign_node_b":
                raise ValueError(f"FOREIGN_ORIGIN_FIXTURE_LOST {test_id}")
        required = required_positive_bindings.get(test_id)
        if required is not None and not required <= set(vector["positive_input"]["bindings"]):
            raise ValueError(f"POSITIVE_CONTEXT_BINDING_MISSING {test_id}: {sorted(required - set(vector['positive_input']['bindings']))}")
        forbidden_collapse = required_negative_bindings.get(test_id)
        if forbidden_collapse is not None and not forbidden_collapse <= set(vector["negative_input"]["bindings"]):
            raise ValueError(f"NEGATIVE_COLLAPSE_CONTROL_MISSING {test_id}: {sorted(forbidden_collapse - set(vector['negative_input']['bindings']))}")
        if not vector.get("input_class") or not vector.get("claim_ceiling"):
            raise ValueError(f"INCOMPLETE_VECTOR_METADATA {vector.get('test_id')}")
    if covered != required:
        raise ValueError(f"FORMAL_OBLIGATION_COVERAGE_MISMATCH missing={sorted(required - covered)}")
    print(f"PASS: D6S-CANON-1 manifest and {len(vectors)} pinned vectors verified; all {len(required)} COS-FV obligations are mapped.")
    print("Claim ceiling: corpus integrity and reference-test coverage only; executable semantic results are evaluated separately by Rust tests.")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, UnicodeError, json.JSONDecodeError, ValueError, TypeError) as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        raise SystemExit(1)
