#!/usr/bin/env python3
"""Independent structural validator for the mobility disposition evidence corpus."""

from __future__ import annotations

import json
from pathlib import Path

EXPECTED_SCHEMA = "mycelix.mobility.evidence_disposition_transition.v1"
EXPECTED_COUNT = 112
EXPECTED_OUTCOME_CLASSES = [
    "accepted",
    "rejected",
    "typed_structural_error",
    "unresolved_at_protocol_layer",
    "explicit_branch",
    "not_silently_collapsed",
    "adapter_boundary_error",
]
ALLOWED_OUTCOMES = set(EXPECTED_OUTCOME_CLASSES)
EXPECTED_BINDING_VECTORS = [
    ("EDT-096", "logical_dependency_binds_to_exactly_one_runtime_address", "accepted"),
    ("EDT-097", "logical_dependency_rebinding_is_rejected_as_typed_structural_error", "typed_structural_error"),
    ("EDT-098", "unbound_logical_dependency_remains_unresolved", "unresolved_at_protocol_layer"),
    ("EDT-099", "logical_dependency_resolution_is_invariant_to_request_order", "accepted"),
    ("EDT-100", "malformed_requested_logical_dependency_is_definitive_structural_error", "typed_structural_error"),
    ("EDT-101", "bound_protocol_address_wrong_for_selected_retrieval_primitive_is_adapter_error", "adapter_boundary_error"),
    ("EDT-102", "valid_record_binding_dispatches_only_to_must_get_valid_record", "accepted"),
    ("EDT-103", "action_binding_dispatches_only_to_must_get_action", "accepted"),
    ("EDT-104", "entry_binding_dispatches_only_to_must_get_entry", "accepted"),
    ("EDT-105", "adapter_retrieval_never_derives_protocol_hash_from_logical_identity_text", "accepted"),
    ("EDT-106", "unbound_logical_identity_has_no_protocol_hash_until_explicit_runtime_binding_exists", "accepted"),
    ("EDT-107", "bound_missing_dependency_is_delegated_to_the_matching_must_get_unresolved_path", "accepted"),
    ("EDT-108", "definitive_valid_and_invalid_decisions_map_directly_to_holochain_results", "accepted"),
    ("EDT-109", "pure_unresolved_decision_cannot_be_mapped_until_runtime_address_binding_exists", "adapter_boundary_error"),
    ("EDT-110", "malformed_logical_identity_is_semantic_invalid_before_runtime_binding_validation", "typed_structural_error"),
    ("EDT-111", "duplicate_runtime_binding_is_adapter_boundary_failure", "adapter_boundary_error"),
    ("EDT-112", "one_callback_seam_maps_semantic_invalidity_and_preserves_true_adapter_errors", "accepted"),
]


def load(path: Path) -> dict:
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise SystemExit(f"cannot read disposition corpus: {exc}") from exc
    if not isinstance(document, dict):
        raise SystemExit("disposition corpus must be a JSON object")
    return document


def main() -> int:
    root = Path(__file__).resolve().parent.parent
    path = root / "docs/mobility/MOBILITY_EVIDENCE_DISPOSITION_TRANSITION_V1.json"
    document = load(path)

    if set(document) != {"schema", "authority", "semantics", "outcome_classes", "vectors"}:
        raise SystemExit("disposition corpus top-level schema drifted")
    if document.get("schema") != EXPECTED_SCHEMA:
        raise SystemExit("unexpected disposition corpus schema")
    if document.get("outcome_classes") != EXPECTED_OUTCOME_CLASSES:
        raise SystemExit("disposition corpus outcome vocabulary drifted")
    vectors = document.get("vectors")
    if not isinstance(vectors, list) or len(vectors) != EXPECTED_COUNT:
        raise SystemExit(f"expected {EXPECTED_COUNT} disposition vectors")

    for index, vector in enumerate(vectors, start=1):
        if not isinstance(vector, dict):
            raise SystemExit(f"EDT-{index:03} must be an object")
        expected_id = f"EDT-{index:03}"
        if vector.get("id") != expected_id:
            raise SystemExit(f"expected {expected_id}, found {vector.get('id')!r}")
        if set(vector) != {"id", "case", "expected"}:
            raise SystemExit(f"{expected_id} has unexpected corpus fields")
        if vector.get("expected") not in ALLOWED_OUTCOMES:
            raise SystemExit(
                f"{expected_id} has unsupported outcome {vector.get('expected')!r}"
            )

    for expected, actual in zip(EXPECTED_BINDING_VECTORS, vectors[-17:], strict=True):
        if (
            actual.get("id"),
            actual.get("case"),
            actual.get("expected"),
        ) != expected:
            raise SystemExit(f"binding vector drift detected: expected {expected!r}")

    print("mobility disposition corpus reference qualification: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
