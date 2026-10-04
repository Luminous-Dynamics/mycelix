#!/usr/bin/env python3
"""Independent structural validator for the mobility disposition evidence corpus."""

from __future__ import annotations

import json
from pathlib import Path

EXPECTED_SCHEMA = "mycelix.mobility.evidence_disposition_transition.v1"
EXPECTED_COUNT = 184
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
    ("EDT-113", "binding_provenance_accepts_exact_logical_dependency_and_authority_basis", "accepted"),
    ("EDT-114", "binding_provenance_rejects_wrong_witness_identity_kind", "typed_structural_error"),
    ("EDT-115", "binding_provenance_rejects_duplicate_basis_witness", "typed_structural_error"),
    ("EDT-116", "binding_provenance_rejects_authority_omission_from_basis", "typed_structural_error"),
    ("EDT-117", "binding_provenance_rejects_logical_identity_mismatch_at_runtime_binding", "typed_structural_error"),
    ("EDT-118", "runtime_binding_requires_explicit_provenance_witness", "accepted"),
    ("EDT-119", "resolved_runtime_dependency_carries_exact_binding_provenance", "accepted"),
    ("EDT-120", "binding_provenance_contains_no_protocol_address_derivation", "accepted"),
    ("EDT-121", "duplicate_provenance_witness_cannot_justify_multiple_runtime_bindings", "adapter_boundary_error"),
    ("EDT-122", "binding_provenance_rejects_witness_identity_equal_to_authority", "typed_structural_error"),
    ("EDT-123", "binding_provenance_rejects_self-referential_basis", "typed_structural_error"),
    ("EDT-124", "binding_provenance_is_serializable_as_a_closed_schema", "accepted"),
    ("EDT-125", "unresolved_partial_runtime_binding_preserves_exact_provenance_witness", "accepted"),
    ("EDT-126", "binding_provenance_requires_exact_authority_scope_reference", "accepted"),
    ("EDT-127", "binding_provenance_requires_exact_authority_delegation_reference", "accepted"),
    ("EDT-128", "binding_provenance_allows_no_identity_reuse_across_provenance_roles", "typed_structural_error"),
    ("EDT-129", "binding_provenance_basis_cannot_contain_its_own_witness_identity", "typed_structural_error"),
    ("EDT-130", "binding_provenance_basis_cannot_use_bound_dependency_as_its_own_justification", "typed_structural_error"),
    ("EDT-131", "provenance_witness_identity_is_unique_per_runtime_binding", "adapter_boundary_error"),
    ("EDT-132", "signed_binding_attestation_payload_binds_schema_provenance_address_and_retrieval_kind", "accepted"),
    ("EDT-133", "signed_binding_attestation_uses_deterministic_canonical_signature_verification", "accepted"),
    ("EDT-134", "unverified_binding_attestation_is_semantic_invalidity", "typed_structural_error"),
    ("EDT-135", "signature_verification_host_failure_remains_runtime_error", "adapter_boundary_error"),
    ("EDT-136", "attested_runtime_binding_requires_verified_signature", "accepted"),
    ("EDT-137", "signature_attestation_does_not_resolve_authority_identity_by_itself", "accepted"),
    ("EDT-138", "signed_binding_attestation_verification_returns_valid_when_deterministic_host_verifies_payload", "accepted"),
    ("EDT-139", "signed_binding_attestation_signature_mismatch_is_semantic_invalidity", "typed_structural_error"),
    ("EDT-140", "signed_binding_attestation_verification_host_failure_remains_extern_result_error", "adapter_boundary_error"),
    ("EDT-141", "attested_binding_acceptance_occurs_only_after_signature_verification", "accepted"),
    ("EDT-142", "authority_agent_credential_binds_exact_domain_authority_to_agent_pub_key", "accepted"),
    ("EDT-143", "authority_agent_credential_requires_issuer_to_equal_bound_agent", "typed_structural_error"),
    ("EDT-144", "authority_agent_registry_rejects_duplicate_authority_binding", "adapter_boundary_error"),
    ("EDT-145", "runtime_binding_requires_authority_agent_binding_before_signer_acceptance", "unresolved_at_protocol_layer"),
    ("EDT-146", "runtime_binding_signer_must_match_registered_authority_agent", "typed_structural_error"),
    ("EDT-147", "authority_agent_binding_retains_domain_provenance_requirement", "accepted"),
    ("EDT-148", "authority_agent_signature_does_not_infer_domain_authority_identity", "accepted"),
    ("EDT-149", "authority_agent_binding_is_order_independent_and_immutable", "accepted"),
    ("EDT-150", "authority_agent_provenance_uses_a_dedicated_subject_binding_type", "accepted"),
    ("EDT-151", "authority_agent_provenance_requires_distinct_witness_authority_scope_and_delegation_roles", "typed_structural_error"),
    ("EDT-152", "authority_agent_provenance_requires_exact_scope_and_delegation_basis", "typed_structural_error"),
    ("EDT-153", "authority_agent_provenance_is_closed_schema_serializable", "accepted"),
    ("EDT-154", "verified_authority_agent_credential_is_admitted_to_immutable_registry", "accepted"),
    ("EDT-155", "authority_agent_credential_with_wrong_issuer_is_semantic_invalidity", "typed_structural_error"),
    ("EDT-156", "unregistered_authority_agent_binding_keeps_runtime_dependency_unresolved", "unresolved_at_protocol_layer"),
    ("EDT-157", "registered_authority_agent_signer_is_required_for_runtime_binding", "typed_structural_error"),
    ("EDT-158", "authority_agent_registry_rejects_second_binding_for_same_authority", "adapter_boundary_error"),
    ("EDT-159", "authority_agent_registry_retains_verified_credential_for_auditability", "accepted"),
    ("EDT-160", "runtime_binding_requires_exact_registered_authority_scope", "typed_structural_error"),
    ("EDT-161", "runtime_binding_requires_exact_registered_authority_delegation", "typed_structural_error"),
    ("EDT-162", "runtime_binding_preserves_registered_authority_credential_provenance_continuity", "accepted"),
    ("EDT-163", "runtime_binding_rejects_dropped_authority_credential_basis", "typed_structural_error"),
    ("EDT-164", "runtime_binding_accepts_complete_registered_authority_credential_basis", "accepted"),
    ("EDT-165", "runtime_binding_rejects_authority_credential_witness_reuse", "typed_structural_error"),
    ("EDT-166", "authority_agent_payload_has_explicit_canonical_serialized_bytes_roundtrip", "accepted"),
    ("EDT-167", "binding_attestation_payload_has_explicit_canonical_serialized_bytes_roundtrip", "accepted"),
    ("EDT-168", "signed_payload_schema_is_owned_and_canonically_roundtrippable", "accepted"),
    ("EDT-169", "authority_agent_payload_rejects_unknown_wire_fields", "typed_structural_error"),
    ("EDT-170", "binding_attestation_payload_rejects_unknown_wire_fields", "typed_structural_error"),
    ("EDT-171", "signed_payload_breaking_changes_require_new_schema_identifier", "accepted"),
    ("EDT-172", "authority_agent_payload_rejects_schema_identifier_change", "typed_structural_error"),
    ("EDT-173", "binding_attestation_payload_rejects_schema_identifier_change", "typed_structural_error"),
    ("EDT-174", "valid_record_retrieval_is_explicitly_inductive_and_does_not_prove_later_operations", "accepted"),
    ("EDT-175", "authority_agent_registry_rejects_reuse_of_the_same_provenance_witness", "adapter_boundary_error"),
    ("EDT-176", "missing_authority_agent_binding_is_preflight_only_and_makes_no_host_calls", "unresolved_at_protocol_layer"),
    ("EDT-177", "caller_supplied_runtime_dependency_order_is_canonicalized_before_host_retrieval", "accepted"),
    ("EDT-178", "must_get_valid_record_upstream_invalidity_remains_protocol_unresolved_not_transport_absence", "unresolved_at_protocol_layer"),
    ("EDT-179", "malformed_runtime_binding_payload_precedes_missing_authority_preflight", "typed_structural_error"),
    ("EDT-180", "runtime_binding_rejects_provenance_witness_from_other_authority_registry_binding", "adapter_boundary_error"),
    ("EDT-181", "runtime_retrieval_dispatch_passes_exact_bound_protocol_address", "accepted"),
    ("EDT-182", "canonical_retrieval_order_reaches_host_boundary_before_failure", "accepted"),
    ("EDT-183", "invalid_runtime_attestation_signature_precedes_missing_authority_preflight", "typed_structural_error"),
    ("EDT-184", "runtime_attestation_signature_host_failure_precedes_missing_authority_preflight", "adapter_boundary_error"),
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

    for expected, actual in zip(EXPECTED_BINDING_VECTORS, vectors[-len(EXPECTED_BINDING_VECTORS):], strict=True):
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
