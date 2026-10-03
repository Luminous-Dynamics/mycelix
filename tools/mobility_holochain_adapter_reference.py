#!/usr/bin/env python3
"""Independent contract checks for the concrete mobility Holochain adapter."""

from __future__ import annotations

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

    for fragment in (
        'hdi = "=0.8.0"',
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
    if "agent_for(&authority)" not in source:
        raise SystemExit("runtime binding must expose authority-agent lookup")
    if "credential_for(&authority)" not in source:
        raise SystemExit("runtime binding must consult the retained authority-agent credential")
    if "binding.signer != authorized_credential.payload.agent" not in source:
        raise SystemExit("runtime binding signer must match the registered authority agent")
    if "authorized_credential.payload.provenance.authority_scope" not in source:
        raise SystemExit("runtime binding must preserve exact authority scope continuity")
    if "authorized_credential.payload.provenance.authority_delegation" not in source:
        raise SystemExit("runtime binding must preserve exact authority delegation continuity")
    if "one AgentPubKey in an immutable binding set" not in source:
        raise SystemExit("authority-agent registry must remain immutable per authority")

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
