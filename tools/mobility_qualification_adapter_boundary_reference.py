#!/usr/bin/env python3
"""Independent structural validator for the mobility qualification adapter contract."""

from __future__ import annotations

import json
import sys
from pathlib import Path

EXPECTED_SCHEMA = "mycelix.mobility.qualification_adapter_boundary.v1"
EXPECTED_OUTCOMES = [
    ("valid", "ValidateCallbackResult::Valid"),
    ("invalid", "ValidateCallbackResult::Invalid(String)"),
    (
        "unresolved",
        "ValidateCallbackResult::UnresolvedDependencies(UnresolvedDependencies)",
    ),
]


def load_contract(path: Path) -> dict:
    try:
        with path.open("r", encoding="utf-8") as handle:
            document = json.load(handle)
    except (OSError, json.JSONDecodeError) as exc:
        raise SystemExit(f"cannot read adapter boundary contract: {exc}") from exc
    if not isinstance(document, dict):
        raise SystemExit("adapter boundary contract must be a JSON object")
    return document


def require_bool(mapping: dict, key: str, expected: bool) -> None:
    value = mapping.get(key)
    if value is not expected:
        raise SystemExit(f"{key} must be {str(expected).lower()}")


def main() -> int:
    default = (
        Path(__file__).resolve().parent.parent
        / "docs/mobility/MOBILITY_QUALIFICATION_ADAPTER_BOUNDARY_V1.json"
    )
    path = Path(sys.argv[1]) if len(sys.argv) > 1 else default
    document = load_contract(path)

    if document.get("schema") != EXPECTED_SCHEMA:
        raise SystemExit("unexpected adapter boundary schema")

    outcomes = document.get("semantic_outcomes")
    if not isinstance(outcomes, list) or len(outcomes) != len(EXPECTED_OUTCOMES):
        raise SystemExit("adapter boundary must define exactly three semantic outcomes")

    for actual, (pure, holochain) in zip(outcomes, EXPECTED_OUTCOMES, strict=True):
        if actual.get("pure") != pure or actual.get("holochain") != holochain:
            raise SystemExit(f"incorrect mapping for {pure}")

    errors = document.get("error_boundary")
    if not isinstance(errors, dict):
        raise SystemExit("missing error boundary")
    if errors.get("semantic_invalidity") != (
        "must be returned as a validation result, not ExternResult::Err"
    ):
        raise SystemExit("semantic invalidity must remain a validation result")
    if errors.get("runtime_failure") != "may remain an ExternResult::Err":
        raise SystemExit("runtime failures must remain outside semantic validation")
    if errors.get("dependency_absence") != (
        "must remain unresolved until the required addressable dependency is available"
    ):
        raise SystemExit("dependency absence must remain unresolved")

    identity = document.get("identity_boundary")
    if not isinstance(identity, dict):
        raise SystemExit("missing identity boundary")
    if identity.get("logical_identity_type") != "IdentityRef":
        raise SystemExit("logical identity must be IdentityRef")
    require_bool(identity, "implicit_string_to_hash_conversion", False)
    require_bool(identity, "adapter_binding_required", True)

    dependencies = document.get("dependency_retrieval")
    if not isinstance(dependencies, dict):
        raise SystemExit("missing dependency retrieval boundary")
    if dependencies.get("deterministic_host_function_family") != "must_get_*":
        raise SystemExit("dependency retrieval must use must_get_*")
    require_bool(dependencies, "mutable_link_collections_as_validation_dependencies", False)
    require_bool(dependencies, "missing_addressable_dependency_is_semantic_invalidity", False)

    determinism = document.get("determinism")
    if not isinstance(determinism, dict):
        raise SystemExit("missing determinism contract")
    for key in (
        "retrieval_order_independent",
        "serialization_order_independent",
        "batching_partition_independent",
        "wall_clock_independent",
        "peer_identity_independent",
    ):
        require_bool(determinism, key, True)

    print("adapter boundary contract validated")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
