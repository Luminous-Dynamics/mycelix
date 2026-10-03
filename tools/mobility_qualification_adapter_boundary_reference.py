        "signature_does_not_resolve_authority_identity": True,
    }
    if crypto != expected_crypto:
        raise SystemExit("cryptographic attestation contract drifted")

    authority_binding = document.get("authority_agent_binding")
    if not isinstance(authority_binding, dict):
        raise SystemExit("missing authority-agent binding boundary")
    require_exact_keys(
        authority_binding,
        {
            "registry_type",
            "authority_identity_type",
            "agent_identity_type",
            "credential_type",
            "credential_schema",
            "provenance_type",
            "issuer_must_equal_agent",
            "credential_signature_required",
            "authority_must_be_present_before_runtime_binding",
            "one_agent_per_authority",
            "duplicate_authority_binding",
            "signer_must_match_registered_agent",
            "signature_does_not_resolve_domain_authority_identity",
            "domain_provenance_remains_required",
            "missing_authority_agent_binding",
        },
        "authority-agent binding",
    )
    expected_authority_binding = {
        "registry_type":"HolochainAuthorityAgentBindingSet",
        "authority_identity_type":"IdentityRef",
        "agent_identity_type":"AgentPubKey",
        "credential_type":"SignedHolochainAuthorityAgentBinding",
        "credential_schema":"mycelix.mobility.holochain_authority_agent_binding.v1",
        "provenance_type":"QualificationAuthorityAgentBindingProvenance",
        "issuer_must_equal_agent":True,
        "credential_signature_required":True,
        "authority_must_be_present_before_runtime_binding":True,
        "one_agent_per_authority":True,
        "duplicate_authority_binding":"rejected",
        "signer_must_match_registered_agent":True,
        "signature_does_not_resolve_domain_authority_identity":True,
        "domain_provenance_remains_required":True,
        "missing_authority_agent_binding":"unresolved",
    }
    if authority_binding != expected_authority_binding:
        raise SystemExit("authority-agent binding contract drifted")

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