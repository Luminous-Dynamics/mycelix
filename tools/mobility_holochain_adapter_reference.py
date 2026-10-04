#!/usr/bin/env python3
"""Independent contract checks for the concrete mobility Holochain adapter."""

from __future__ import annotations

import json
from pathlib import Path

ADAPTER_DIR = (
    Path(__file__).resolve().parent.parent
    / "mycelix-commons/crates/mobility-qualification-holochain-adapter"
)
LIB = ADAPTER_DIR / "src/lib.rs"
CARGO = ADAPTER_DIR / "Cargo.toml"

EXPECTED_DISPATCH = (
    ("ValidRecord", "ActionHash", "must_get_valid_record"),
    ("Action", "ActionHash", "must_get_action"),
    ("Entry", "EntryHash", "must_get_entry"),
)


def main() -> int:
    cargo = CARGO.read_text(encoding="utf-8")
    source = LIB.read_text(encoding="utf-8")
    contract_path = Path(__file__).resolve().parent.parent / "docs/mobility/MOBILITY_QUALIFICATION_ADAPTER_BOUNDARY_V1.json"
    try:
        contract = json.loads(contract_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise SystemExit(f"cannot read adapter boundary contract: {exc}") from exc

    for fragment in (
        'hdi = "=0.8.0"',
        'holochain_serialized_bytes = "=0.0.57"',
        'mobility-configuration-qualification = { path = "../mobility-configuration-qualification" }',
        "[workspace]",
    ):
        if fragment not in cargo:
            raise SystemExit(f"adapter manifest missing required fragment: {fragment}")

    forbidden = (
        "IdentityRef.id",
        "hash_entry(",
        "hash_action(",
        "blake2",
        "get_links(",
        "get_links_details(",
        "SystemTime",
        "Instant",
    )
    for fragment in forbidden:
        if fragment in source:
            raise SystemExit(
                f"adapter contains forbidden semantic/address derivation fragment: {fragment}"
            )

    for retrieval, address, host_fn in EXPECTED_DISPATCH:
        for fragment in (retrieval, address, host_fn):
            if fragment not in source:
                raise SystemExit(f"adapter missing required dispatch fragment: {fragment}")

    if "QualificationDecision::Valid" not in source:
        raise SystemExit("adapter must preserve the pure valid decision")
    if "QualificationDependencyBindingProvenance" not in source:
        raise SystemExit("adapter must require a pure provenance witness for each runtime binding")
    if "HolochainAuthorityAgentBindingSet" not in source:
        raise SystemExit("adapter must provide an immutable authority-agent registry")
    if "QualificationAuthorityAgentBindingProvenance" not in source:
        raise SystemExit("authority-agent credentials must use the dedicated provenance type")
    if "SignedHolochainAuthorityAgentBinding" not in source:
        raise SystemExit("adapter must expose signed authority-agent credentials")
    if "HOLOCHAIN_AUTHORITY_AGENT_BINDING_SCHEMA" not in source:
        raise SystemExit("authority-agent credential schema must be explicit")
    if "issuer != self.payload.agent" not in source:
        raise SystemExit("authority-agent credential issuer must equal the bound agent")
    if "one AgentPubKey in an immutable binding set" not in source:
        raise SystemExit("authority-agent registry must reject duplicate authority bindings")

    authority_contract = contract.get("authority_agent_binding")
    if not isinstance(authority_contract, dict):
        raise SystemExit("machine contract missing authority_agent_binding object")
    expected_authority_contract = {
        "registry_retains_verified_credential": True,
        "audit_credential_accessor": "HolochainAuthorityAgentBindingSet::credential_for",
        "runtime_binding_requires_exact_authority_scope_match": True,
        "runtime_binding_requires_exact_authority_delegation_match": True,
        "authority_credential_provenance_continuity_is_checked": True,
        "registered_credential_basis_is_minimum_runtime_basis": True,
        "runtime_binding_witness_must_differ_from_authority_credential_witness": True,
        "provenance_witness_unique_per_registry_binding": True,
        "preflight_missing_is_not_protocol_unresolved": True,
        "missing_binding_stops_before_host_calls": True,
    }
    for key, expected in expected_authority_contract.items():
        if authority_contract.get(key) != expected:
            raise SystemExit(f"machine contract drift for {key}: expected {expected!r}")

    serialization_contract = contract.get("signed_payload_serialization")
    if not isinstance(serialization_contract, dict):
        raise SystemExit("machine contract missing signed_payload_serialization object")
    expected_serialization_contract = {
        "library": "holochain_serialized_bytes",
        "version": "0.0.57",
        "representation": "SerializedBytes",
        "schema_fields_are_owned_strings": True,
        "explicit_try_from_roundtrip_tested": True,
        "verify_signature_remains_single_crypto_verifier": True,
        "unknown_fields_rejected_by_v1": True,
        "schema_identifier_is_signed": True,
        "breaking_changes_require_new_schema_identifier": True,
        "version_policy": "strict_v1",
        "schema_identifier_changes_rejected_as_semantic_invalidity": True,
    }
    for key, expected in expected_serialization_contract.items():
        if serialization_contract.get(key) != expected:
            raise SystemExit(f"machine serialization contract drift for {key}: expected {expected!r}")
    if serialization_contract.get("payloads") != [
        "HolochainAuthorityAgentBindingPayload",
        "HolochainBindingAttestationPayload",
    ]:
        raise SystemExit("machine serialization contract payload list drifted")
    if serialization_contract.get("unknown_field_tests") != [
        "authority_agent_payload_rejects_unknown_wire_fields",
        "binding_attestation_payload_rejects_unknown_wire_fields",
    ]:
        raise SystemExit("machine serialization contract unknown-field test list drifted")
    if serialization_contract.get("schema_identifier_tests") != [
        "authority_agent_payload_rejects_schema_identifier_change",
        "binding_attestation_payload_rejects_schema_identifier_change",
    ]:
        raise SystemExit("machine serialization contract schema-identifier test list drifted")
    dependency_contract = contract.get("dependency_retrieval")
    if not isinstance(dependency_contract, dict):
        raise SystemExit("machine contract missing dependency_retrieval object")
    expected_dependency_contract = {
        "valid_record_requires_action_hash": True,
        "valid_record_does_not_prove_later_operations_valid": True,
        "unavailable_valid_record_maps_to_protocol_unresolved": True,
        "valid_record_semantics": "inductive_validity_of_the_referenced_create_record_as_reported_by_visible_validation_authorities",
    }
    for key, expected in expected_dependency_contract.items():
        if dependency_contract.get(key) != expected:
            raise SystemExit(f"machine retrieval contract drift for {key}: expected {expected!r}")

    if "agent_for(&authority)" not in source:
        raise SystemExit("runtime binding must expose authority-agent lookup")
    if "credential_for(&authority)" not in source:
        raise SystemExit("runtime binding must consult the retained authority-agent credential")
    if "authority_agent_registry_rejects_duplicate_provenance_witness" not in source:
        raise SystemExit("registry witness reuse must have a regression test")
    if "missing authority-agent registration must stop before host calls" not in source:
        raise SystemExit("missing authority-agent registration must stop before host calls")
    if "binding.signer != authorized_credential.payload.agent" not in source:
        raise SystemExit("runtime binding signer must match the registered authority agent")
    compact_source = "".join(source.split())
    if "authorized_credential.payload.provenance.authority_scope" not in compact_source:
        raise SystemExit("runtime binding must preserve exact authority scope continuity")
    if "authorized_credential.payload.provenance.authority_delegation" not in compact_source:
        raise SystemExit("runtime binding must preserve exact authority delegation continuity")
    if "authorized_credential.payload.provenance.basis" not in compact_source:
        raise SystemExit("runtime binding must preserve the registered authority credential basis")
    if "runtime binding witness identity must differ from the registered authority credential witness".replace(" ", "") not in compact_source:
        raise SystemExit("runtime binding must use a distinct witness identity")
    if ".find(|basis| !binding.payload.provenance.basis.contains(basis))" not in source:
        raise SystemExit("runtime binding must reject dropped authority credential basis witnesses")
    if "one AgentPubKey in an immutable binding set" not in source:
        raise SystemExit("authority-agent registry must remain immutable per authority")
    if "an authority-agent provenance witness may justify only one registry binding" not in source:
        raise SystemExit("authority-agent registry must reject provenance witness reuse")
    if ".values().any(|existing|" not in compact_source:
        raise SystemExit("authority-agent registry must inspect existing credentials for witness reuse")

    for fragment in (
        "HolochainBindingAttestationVerification",
        "verify_binding_attestation",
        "bind_attested",
        "extern",
    ):
        if fragment.lower() not in source.lower():
            raise SystemExit(f"adapter missing signed attestation fragment: {fragment}")
    if "ExternResult<Result<(), HolochainAdapterBoundaryError>>" not in source:
        raise SystemExit("attested bind must preserve host errors outside semantic invalidity")
    if "hdi::ed25519::verify_signature" not in source:
        raise SystemExit("signed binding attestation must use deterministic Holochain signature verification")

    payload_markers = (
        ("HolochainAuthorityAgentBindingPayload", "serde::Serialize, serde::Deserialize, SerializedBytes"),
        ("HolochainBindingAttestationPayload", "serde::Serialize, serde::Deserialize, SerializedBytes"),
    )
    for payload, derive_fragment in payload_markers:
        payload_start = source.find(f"pub struct {payload}")
        if payload_start < 0:
            raise SystemExit(f"{payload} definition missing")
        if derive_fragment not in source[:payload_start]:
            raise SystemExit(
                f"{payload} must derive serde Serialize/Deserialize and SerializedBytes"
            )
        payload_end = source.find("\n}", payload_start)
        if payload_end < 0:
            raise SystemExit(f"{payload} definition is unterminated")
        payload_body = source[payload_start:payload_end + 2]
        if "pub schema: String" not in payload_body:
            raise SystemExit(f"{payload} schema identifier must remain an owned String")
    if "SerializedBytes" not in source:
        raise SystemExit("signed payloads must expose canonical SerializedBytes round-trips")
    if "SerializedBytes::try_from(payload.clone())" not in source:
        raise SystemExit("canonical payload bytes must be produced through SerializedBytes TryFrom")
    if ".try_from(encoded)" not in source:
        raise SystemExit("canonical payload bytes must be decoded through the declared payload type")
    if "pub schema: String" not in source:
        raise SystemExit("signed payload schema identifiers must be owned Strings")
    if source.count("#[serde(deny_unknown_fields)]") != 2:
        raise SystemExit("both signed payload types must reject unknown wire fields")
    for test_name in (
        "authority_agent_payload_rejects_schema_identifier_change",
        "binding_attestation_payload_rejects_schema_identifier_change",
    ):
        if test_name not in source:
            raise SystemExit(f"missing schema-identifier regression test: {test_name}")

    if "provenance: QualificationDependencyBindingProvenance" not in source:
        raise SystemExit("resolved dependencies must carry their provenance witness")
    if "provenance.validate()" not in source:
        raise SystemExit("binding must structurally validate provenance before accepting the address")
    if "matches_logical_identity(&identity)" not in source:
        raise SystemExit("binding provenance must name the exact logical identity")
    if "finalize_callback" not in source:
        raise SystemExit("adapter must expose one callback-facing semantic mapping seam")
    if "ValidateCallbackResult::Valid" not in source:
        raise SystemExit("adapter must map pure Valid directly")
    if "ValidateCallbackResult::Invalid" not in source:
        raise SystemExit("adapter must map pure Invalid directly")
    if "LogicalDependencyNotBound" not in source:
        raise SystemExit("adapter must refuse to synthesize unresolved hashes from unbound logical identities")
    if "SemanticInvalid" not in source:
        raise SystemExit("adapter must preserve semantic invalidity as a distinct preflight class")
    if "duplicate_binding_remains_an_adapter_contract_failure" not in source:
        raise SystemExit("adapter must pin duplicate binding as an adapter-contract defect")
    if "QualificationDecision::Invalid" not in source:
        raise SystemExit("adapter must preserve the pure invalid decision")
    if "QualificationDecision::Unresolved" not in source:
        raise SystemExit("adapter must preserve the pure unresolved decision")
    if "inductive-validity dependency" not in source:
        raise SystemExit("adapter must document ValidRecord as an inductive-validity dependency")
    if "later operation" not in source:
        raise SystemExit("adapter must not overclaim that ValidRecord proves later operation validity")
    if (
        'unreachable!("HolochainDependencyBindingSet prevents address-kind mismatch")'
        not in source
    ):
        raise SystemExit("host retrieval must be unreachable after binding-time type validation")
    bind_start = source.index("pub fn bind(")
    bind_end = source.index("\n    /// Bind from a cryptographically", bind_start)
    bind_body = source[bind_start:bind_end]
    required_bind_fragments = (
        "provenance: QualificationDependencyBindingProvenance",
        "provenance\n            .validate()",
        "matches_logical_identity(&identity)",
        "identity\n            .validate()",
        "validate_address_kind(&address, retrieval)",
        "self.provenance.insert(identity, provenance)",
    )
    for fragment in required_bind_fragments:
        if fragment not in bind_body:
            raise SystemExit(f"binding boundary missing invariant fragment: {fragment}")

    provenance_validation = bind_body.index("provenance\n            .validate()")
    address_validation = bind_body.index("validate_address_kind(&address, retrieval)")
    if provenance_validation > address_validation:
        raise SystemExit("provenance validation must precede protocol address validation")

    print("mobility Holochain adapter independent reference qualification: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
