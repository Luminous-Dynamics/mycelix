#!/usr/bin/env python3
"""Independent structural validator for the mobility qualification adapter contract."""

from __future__ import annotations

import json
import sys
from pathlib import Path

EXPECTED_SCHEMA = "mycelix.mobility.qualification_adapter_boundary.v1"
EXPECTED_OUTCOMES = [
    (
        "valid",
        "qualification completed for the supplied dependency set",
        "ValidateCallbackResult::Valid",
    ),
    (
        "invalid",
        "supplied records contain a definitive structural contradiction",
        "ValidateCallbackResult::Invalid(String)",
    ),
    (
        "unresolved",
        "qualification cannot complete because required logical dependencies are unavailable",
        "ValidateCallbackResult::UnresolvedDependencies(...) after explicit address binding; an unbound logical identity remains pure Unresolved",
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



def require_exact_keys(mapping: dict, expected: set[str], label: str) -> None:
    actual = set(mapping)
    if actual != expected:
        missing = sorted(expected - actual)
        unexpected = sorted(actual - expected)
        raise SystemExit(
            f"{label} keys mismatch; missing={missing}, unexpected={unexpected}"
        )


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

    require_exact_keys(
        document,
        {"schema", "version", "semantic_outcomes", "error_boundary", "identity_boundary",
         "binding_provenance", "dependency_retrieval", "determinism", "compatibility"},
        "top-level contract",
    )
    if document.get("schema") != EXPECTED_SCHEMA:
        raise SystemExit("unexpected adapter boundary schema")
    if document.get("version") != "1":
        raise SystemExit("adapter boundary version must be 1")

    outcomes = document.get("semantic_outcomes")
    if not isinstance(outcomes, list) or len(outcomes) != len(EXPECTED_OUTCOMES):
        raise SystemExit("adapter boundary must define exactly three semantic outcomes")

    for actual, (pure, meaning, holochain) in zip(outcomes, EXPECTED_OUTCOMES, strict=True):
        if not isinstance(actual, dict):
            raise SystemExit("each semantic outcome must be an object")
        require_exact_keys(actual, {"pure", "meaning", "holochain"}, f"semantic outcome {pure}")
        if (
            actual.get("pure") != pure
            or actual.get("meaning") != meaning
            or actual.get("holochain") != holochain
        ):
            raise SystemExit(f"incorrect semantic outcome contract for {pure}")

    errors = document.get("error_boundary")
    if not isinstance(errors, dict):
        raise SystemExit("missing error boundary")
    require_exact_keys(
        errors,
        {"semantic_invalidity", "runtime_failure", "dependency_absence",
         "adapter_boundary_error", "adapter_boundary_error_is_semantic_invalidity"},
        "error boundary",
    )
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
    if errors.get("adapter_boundary_error") != (
        "protocol binding/type mismatch is an adapter contract failure, not semantic Invalid"
    ):
        raise SystemExit("adapter-boundary failures must remain outside semantic Invalid")
    require_bool(errors, "adapter_boundary_error_is_semantic_invalidity", False)

    identity = document.get("identity_boundary")
    if not isinstance(identity, dict):
        raise SystemExit("missing identity boundary")
    require_exact_keys(
        identity,
        {"logical_identity_type", "protocol_address_type", "implicit_string_to_hash_conversion",
         "adapter_binding_required", "exactly_one_address_per_logical_identity",
         "duplicate_logical_identity_binding", "unbound_logical_identity_is_semantic_invalidity",
         "binding_order_independent", "resolution_operation", "resolution_success",
         "resolution_invalid", "resolution_missing", "duplicate_requests_deduplicated",
         "address_derivation", "retrieval_kind_type", "allowed_retrieval_kinds",
         "retrieval_kind_required", "retrieval_kind_mapping_runtime_owned",
         "retrieval_kind_mismatch_is_adapter_boundary_error"},
        "identity boundary",
    )
    if identity.get("logical_identity_type") != "IdentityRef":
        raise SystemExit("logical identity must be IdentityRef")
    if identity.get("protocol_address_type") != "Holochain hash or other runtime-specific addressable dependency":
        raise SystemExit("protocol address type must remain runtime-specific and addressable")
    require_bool(identity, "implicit_string_to_hash_conversion", False)
    require_bool(identity, "adapter_binding_required", True)
    require_bool(identity, "exactly_one_address_per_logical_identity", True)
    if identity.get("duplicate_logical_identity_binding") != "rejected":
        raise SystemExit("duplicate logical identity binding must be rejected")
    require_bool(identity, "unbound_logical_identity_is_semantic_invalidity", False)
    require_bool(identity, "binding_order_independent", True)
    if identity.get("resolution_operation") != "QualificationDependencyBindingSet::resolve_required":
        raise SystemExit("unexpected binding resolution operation")
    if identity.get("resolution_success") != (
        "all requested logical identities bound; return canonical identity/address pairs"
    ):
        raise SystemExit("binding resolution success semantics changed")
    if identity.get("resolution_invalid") != (
        "malformed logical identity is definitive invalidity"
    ):
        raise SystemExit("binding resolution invalid semantics changed")
    if identity.get("resolution_missing") != (
        "return canonical missing logical identities and preserve unresolved semantic state"
    ):
        raise SystemExit("binding resolution missing semantics changed")
    require_bool(identity, "duplicate_requests_deduplicated", True)
    require_bool(identity, "address_derivation", False)
    if identity.get("retrieval_kind_type") != "QualificationDependencyRetrievalKind":
        raise SystemExit("unexpected retrieval kind type")
    if identity.get("allowed_retrieval_kinds") != ["ValidRecord", "Action", "Entry"]:
        raise SystemExit("unexpected allowed retrieval kinds")
    require_bool(identity, "retrieval_kind_required", True)
    require_bool(identity, "retrieval_kind_mapping_runtime_owned", True)
    require_bool(identity, "retrieval_kind_mismatch_is_adapter_boundary_error", True)

    provenance = document.get("binding_provenance")
    if not isinstance(provenance, dict):
        raise SystemExit("missing binding provenance boundary")
    require_exact_keys(
        provenance,
        {
            "witness_type",
            "witness_identity_type",
            "witness_identity_kind",
            "logical_identity_type",
            "authority_identity_type",
            "basis_identity_type",
            "basis_required",
            "basis_unique",
            "basis_must_include_exact_authority",
            "witness_must_name_exact_logical_identity",
            "protocol_address_excluded_from_pure_witness",
            "cryptographic_proof",
            "global_completeness_claim",
            "binding_requires_provenance",
            "provenance_carried_through_runtime_resolution",
        },
        "binding provenance",
    )
    expected_provenance = {
        "witness_type": "QualificationDependencyBindingProvenance",
        "witness_identity_type": "IdentityRef",
        "witness_identity_kind": "ReconciliationWitness",
        "logical_identity_type": "IdentityRef",
        "authority_identity_type": "IdentityRef",
        "basis_identity_type": "IdentityRef",
        "basis_required": True,
        "basis_unique": True,
        "basis_must_include_exact_authority": True,
        "witness_must_name_exact_logical_identity": True,
        "protocol_address_excluded_from_pure_witness": True,
        "cryptographic_proof": False,
        "global_completeness_claim": False,
        "binding_requires_provenance": True,
        "provenance_carried_through_runtime_resolution": True,
    }
    if provenance != expected_provenance:
        raise SystemExit("binding provenance contract drifted")

    dependencies = document.get("dependency_retrieval")
    if not isinstance(dependencies, dict):
        raise SystemExit("missing dependency retrieval boundary")
    require_exact_keys(
        dependencies,
        {"deterministic_host_function_family", "mutable_link_collections_as_validation_dependencies",
         "missing_addressable_dependency_is_semantic_invalidity", "binding_preserves_address_kind",
         "valid_record_requires_action_hash", "wrong_address_kind_is_adapter_boundary_error"},
        "dependency retrieval",
    )
    if dependencies.get("deterministic_host_function_family") != "must_get_*":
        raise SystemExit("dependency retrieval must use must_get_*")
    require_bool(dependencies, "mutable_link_collections_as_validation_dependencies", False)
    require_bool(dependencies, "missing_addressable_dependency_is_semantic_invalidity", False)
    require_bool(dependencies, "binding_preserves_address_kind", True)
    require_bool(dependencies, "valid_record_requires_action_hash", True)
    require_bool(dependencies, "wrong_address_kind_is_adapter_boundary_error", True)
    mappings = dependencies.get("retrieval_kind_mappings")
    expected_mappings = [
        {"pure": "ValidRecord", "protocol_address_type": "ActionHash", "host_function": "must_get_valid_record"},
        {"pure": "Action", "protocol_address_type": "ActionHash", "host_function": "must_get_action"},
        {"pure": "Entry", "protocol_address_type": "EntryHash", "host_function": "must_get_entry"},
    ]
    if mappings != expected_mappings:
        raise SystemExit("retrieval-kind host-function mapping contract changed")
    require_bool(dependencies, "binding_preserves_address_kind", True)
    require_bool(dependencies, "valid_record_requires_action_hash", True)
    require_bool(dependencies, "wrong_address_kind_is_adapter_boundary_error", True)

    determinism = document.get("determinism")
    if not isinstance(determinism, dict):
        raise SystemExit("missing determinism contract")
    require_exact_keys(
        determinism,
        {"retrieval_order_independent", "serialization_order_independent",
         "batching_partition_independent", "wall_clock_independent", "peer_identity_independent"},
        "determinism",
    )
    for key in (
        "retrieval_order_independent",
        "serialization_order_independent",
        "batching_partition_independent",
        "wall_clock_independent",
        "peer_identity_independent",
    ):
        require_bool(determinism, key, True)

    compatibility = document.get("compatibility")
    if not isinstance(compatibility, dict):
        raise SystemExit("missing compatibility boundary")
    require_exact_keys(
        compatibility,
        {"pure_layer_holochain_dependency", "current_workspace_line",
         "recommended_runtime_line", "migration_isolation"},
        "compatibility",
    )
    if compatibility.get("pure_layer_holochain_dependency") is not None:
        raise SystemExit("pure qualification layer must remain Holochain-free")
    if compatibility.get("current_workspace_line") != "Holochain 0.6-compatible":
        raise SystemExit("unexpected current workspace compatibility line")
    if compatibility.get("recommended_runtime_line") != "Holochain 0.7":
        raise SystemExit("unexpected recommended runtime line")
    if compatibility.get("migration_isolation") != "runtime adapter only":
        raise SystemExit("Holochain migration must remain isolated to the runtime adapter")

    print("adapter boundary contract validated")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
