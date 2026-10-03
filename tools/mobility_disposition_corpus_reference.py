#!/usr/bin/env python3
"""Independent structural validator for the mobility disposition evidence corpus."""

from __future__ import annotations

import json
from pathlib import Path

EXPECTED_SCHEMA = "mycelix.mobility.evidence_disposition_transition.v1"
EXPECTED_COUNT = 100
ALLOWED_OUTCOMES = {
    "accepted",
    "rejected",
    "typed_structural_error",
    "unresolved_at_protocol_layer",
}
EXPECTED_BINDING_VECTORS = [
    (
        "EDT-096",
        "logical_dependency_binds_to_exactly_one_runtime_address",
        "accepted",
    ),
    (
        "EDT-097",
        "logical_dependency_rebinding_is_rejected_as_typed_structural_error",
        "typed_structural_error",
    ),
    (
        "EDT-098",
        "unbound_logical_dependency_remains_unresolved",
        "unresolved_at_protocol_layer",
    ),
    (
        "EDT-099",
        "logical_dependency_resolution_is_invariant_to_request_order",
        "accepted",
    ),
    (
        "EDT-100",
        "malformed_requested_logical_dependency_is_definitive_structural_error",
        "typed_structural_error",
    ),
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

    if document.get("schema") != EXPECTED_SCHEMA:
        raise SystemExit("unexpected disposition corpus schema")

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

    for expected, actual in zip(EXPECTED_BINDING_VECTORS, vectors[-5:], strict=True):
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
