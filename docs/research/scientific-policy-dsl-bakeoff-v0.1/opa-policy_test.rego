package mycelix.authz

import rego.v1

test_contributor_can_propose_genesis_claim if {
    decision.effect == "allow"
    decision.rule == "CLAIM_PROPOSED_ROLE"
    with input as {
        "action": "claim_proposed",
        "actor": {"id": "actor:alice", "roles": ["contributor"], "organizations": []},
        "claim": {"exists": false}
    }
}

test_reviewer_cannot_propose_genesis_claim if {
    decision.effect == "deny"
    with input as {
        "action": "claim_proposed",
        "actor": {"id": "actor:reviewer", "roles": ["reviewer"], "organizations": []},
        "claim": {"exists": false}
    }
}

test_migration_service_can_import_legacy_claim if {
    decision.effect == "allow"
    with input as {
        "action": "legacy_claim_imported",
        "actor": {"id": "actor:migrator", "roles": ["migration_service"], "organizations": []},
        "claim": {"exists": false}
    }
}

test_unrelated_contributor_cannot_attach_evidence if {
    decision.effect == "deny"
    with input as {
        "action": "evidence_attached",
        "actor": {"id": "actor:bob", "roles": ["contributor"], "organizations": []},
        "claim": {"exists": true, "creatorActorId": "actor:alice"}
    }
}

test_creator_cannot_claim_independent_replication if {
    decision.effect == "deny"
    with input as {
        "action": "attestation_recorded",
        "actor": {"id": "actor:alice", "roles": ["reviewer"], "organizations": []},
        "claim": {"exists": true, "creatorActorId": "actor:alice"},
        "attestation": {"kind": "independent_replication"}
    }
}

test_creator_organization_cannot_claim_independent_replication if {
    decision.effect == "deny"
    with input as {
        "action": "attestation_recorded",
        "actor": {"id": "actor:bob", "roles": ["reviewer"], "organizations": ["org:creator"]},
        "claim": {
            "exists": true,
            "creatorActorId": "actor:alice",
            "creatorOrganizationId": "org:creator"
        },
        "attestation": {"kind": "independent_replication"},
        "actingOrganizationId": "org:creator"
    }
}

test_attestation_author_can_withdraw_own_attestation if {
    decision.effect == "allow"
    with input as {
        "action": "attestation_withdrawn",
        "actor": {"id": "actor:reviewer", "roles": ["reviewer"], "organizations": []},
        "claim": {"exists": true, "creatorActorId": "actor:alice"},
        "targetAttestation": {"exists": true, "authorActorId": "actor:reviewer"}
    }
}

test_unknown_action_fails_closed if {
    decision.effect == "deny"
    decision.rule == "DEFAULT_DENY"
    with input as {
        "action": "unrecognized_action",
        "actor": {"id": "actor:alice", "roles": ["contributor"], "organizations": []},
        "claim": {"exists": true, "creatorActorId": "actor:alice"}
    }
}
