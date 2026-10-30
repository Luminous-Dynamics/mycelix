package mycelix.authz

import rego.v1

// Frozen 30-case authorization corpus, generated from cases.json.

test_auth_auth_001 if {
    decision.effect == "allow"
    with input as {
        "action": "claim_proposed",
        "actor": {
            "id": "actor:alice",
            "roles": [
                "contributor"
            ],
            "organizations": []
        },
        "claim": {
            "exists": false
        },
        "attestation": {
            "kind": "ordinary"
        },
        "targetAttestation": {
            "exists": false
        }
    }
}

test_auth_auth_002 if {
    decision.effect == "allow"
    with input as {
        "action": "claim_proposed",
        "actor": {
            "id": "actor:editor",
            "roles": [
                "editor"
            ],
            "organizations": []
        },
        "claim": {
            "exists": false
        },
        "attestation": {
            "kind": "ordinary"
        },
        "targetAttestation": {
            "exists": false
        }
    }
}

test_auth_auth_003 if {
    decision.effect == "allow"
    with input as {
        "action": "claim_proposed",
        "actor": {
            "id": "actor:institution",
            "roles": [
                "institution"
            ],
            "organizations": []
        },
        "claim": {
            "exists": false
        },
        "attestation": {
            "kind": "ordinary"
        },
        "targetAttestation": {
            "exists": false
        }
    }
}

test_auth_auth_004 if {
    decision.effect == "deny"
    with input as {
        "action": "claim_proposed",
        "actor": {
            "id": "actor:reviewer",
            "roles": [
                "reviewer"
            ],
            "organizations": []
        },
        "claim": {
            "exists": false
        },
        "attestation": {
            "kind": "ordinary"
        },
        "targetAttestation": {
            "exists": false
        }
    }
}

test_auth_auth_005 if {
    decision.effect == "deny"
    with input as {
        "action": "claim_proposed",
        "actor": {
            "id": "actor:alice",
            "roles": [
                "contributor"
            ],
            "organizations": []
        },
        "claim": {
            "exists": true,
            "creatorActorId": "actor:alice"
        },
        "attestation": {
            "kind": "ordinary"
        },
        "targetAttestation": {
            "exists": false
        }
    }
}

test_auth_auth_006 if {
    decision.effect == "allow"
    with input as {
        "action": "legacy_claim_imported",
        "actor": {
            "id": "actor:migrator",
            "roles": [
                "migration_service"
            ],
            "organizations": []
        },
        "claim": {
            "exists": false
        },
        "attestation": {
            "kind": "ordinary"
        },
        "targetAttestation": {
            "exists": false
        }
    }
}

test_auth_auth_007 if {
    decision.effect == "deny"
    with input as {
        "action": "legacy_claim_imported",
        "actor": {
            "id": "actor:alice",
            "roles": [
                "contributor"
            ],
            "organizations": []
        },
        "claim": {
            "exists": false
        },
        "attestation": {
            "kind": "ordinary"
        },
        "targetAttestation": {
            "exists": false
        }
    }
}

test_auth_auth_008 if {
    decision.effect == "deny"
    with input as {
        "action": "legacy_claim_imported",
        "actor": {
            "id": "actor:migrator",
            "roles": [
                "migration_service"
            ],
            "organizations": []
        },
        "claim": {
            "exists": true,
            "creatorActorId": "actor:alice"
        },
        "attestation": {
            "kind": "ordinary"
        },
        "targetAttestation": {
            "exists": false
        }
    }
}

test_auth_auth_009 if {
    decision.effect == "allow"
    with input as {
        "action": "evidence_attached",
        "actor": {
            "id": "actor:alice",
            "roles": [
                "contributor"
            ],
            "organizations": []
        },
        "claim": {
            "exists": true,
            "creatorActorId": "actor:alice"
        },
        "attestation": {
            "kind": "ordinary"
        },
        "targetAttestation": {
            "exists": false
        }
    }
}

test_auth_auth_010 if {
    decision.effect == "allow"
    with input as {
        "action": "evidence_attached",
        "actor": {
            "id": "actor:editor",
            "roles": [
                "editor"
            ],
            "organizations": []
        },
        "claim": {
            "exists": true,
            "creatorActorId": "actor:alice"
        },
        "attestation": {
            "kind": "ordinary"
        },
        "targetAttestation": {
            "exists": false
        }
    }
}

test_auth_auth_011 if {
    decision.effect == "allow"
    with input as {
        "action": "evidence_attached",
        "actor": {
            "id": "actor:institution",
            "roles": [
                "institution"
            ],
            "organizations": []
        },
        "claim": {
            "exists": true,
            "creatorActorId": "actor:alice"
        },
        "attestation": {
            "kind": "ordinary"
        },
        "targetAttestation": {
            "exists": false
        }
    }
}

test_auth_auth_012 if {
    decision.effect == "deny"
    with input as {
        "action": "evidence_attached",
        "actor": {
            "id": "actor:bob",
            "roles": [
                "contributor"
            ],
            "organizations": []
        },
        "claim": {
            "exists": true,
            "creatorActorId": "actor:alice"
        },
        "attestation": {
            "kind": "ordinary"
        },
        "targetAttestation": {
            "exists": false
        }
    }
}

test_auth_auth_013 if {
    decision.effect == "allow"
    with input as {
        "action": "attestation_recorded",
        "actor": {
            "id": "actor:reviewer",
            "roles": [
                "reviewer"
            ],
            "organizations": []
        },
        "claim": {
            "exists": true,
            "creatorActorId": "actor:alice"
        },
        "attestation": {
            "kind": "ordinary"
        },
        "targetAttestation": {
            "exists": false
        }
    }
}

test_auth_auth_014 if {
    decision.effect == "allow"
    with input as {
        "action": "attestation_recorded",
        "actor": {
            "id": "actor:contributor",
            "roles": [
                "contributor"
            ],
            "organizations": []
        },
        "claim": {
            "exists": true,
            "creatorActorId": "actor:alice"
        },
        "attestation": {
            "kind": "ordinary"
        },
        "targetAttestation": {
            "exists": false
        }
    }
}

test_auth_auth_015 if {
    decision.effect == "allow"
    with input as {
        "action": "attestation_recorded",
        "actor": {
            "id": "actor:institution",
            "roles": [
                "institution"
            ],
            "organizations": []
        },
        "claim": {
            "exists": true,
            "creatorActorId": "actor:alice"
        },
        "attestation": {
            "kind": "ordinary"
        },
        "targetAttestation": {
            "exists": false
        }
    }
}

test_auth_auth_016 if {
    decision.effect == "deny"
    with input as {
        "action": "attestation_recorded",
        "actor": {
            "id": "actor:editor",
            "roles": [
                "editor"
            ],
            "organizations": []
        },
        "claim": {
            "exists": true,
            "creatorActorId": "actor:alice"
        },
        "attestation": {
            "kind": "ordinary"
        },
        "targetAttestation": {
            "exists": false
        }
    }
}

test_auth_auth_017 if {
    decision.effect == "deny"
    with input as {
        "action": "attestation_recorded",
        "actor": {
            "id": "actor:alice",
            "roles": [
                "reviewer"
            ],
            "organizations": []
        },
        "claim": {
            "exists": true,
            "creatorActorId": "actor:alice"
        },
        "attestation": {
            "kind": "independent_replication"
        },
        "targetAttestation": {
            "exists": false
        }
    }
}

test_auth_auth_018 if {
    decision.effect == "deny"
    with input as {
        "action": "attestation_recorded",
        "actor": {
            "id": "actor:bob",
            "roles": [
                "reviewer"
            ],
            "organizations": [
                "org:creator"
            ]
        },
        "claim": {
            "exists": true,
            "creatorActorId": "actor:alice",
            "creatorOrganizationId": "org:creator"
        },
        "attestation": {
            "kind": "independent_replication"
        },
        "targetAttestation": {
            "exists": false
        },
        "actingOrganizationId": "org:creator"
    }
}

test_auth_auth_019 if {
    decision.effect == "allow"
    with input as {
        "action": "attestation_recorded",
        "actor": {
            "id": "actor:bob",
            "roles": [
                "reviewer"
            ],
            "organizations": [
                "org:independent"
            ]
        },
        "claim": {
            "exists": true,
            "creatorActorId": "actor:alice",
            "creatorOrganizationId": "org:creator"
        },
        "attestation": {
            "kind": "independent_replication"
        },
        "targetAttestation": {
            "exists": false
        },
        "actingOrganizationId": "org:independent"
    }
}

test_auth_auth_020 if {
    decision.effect == "allow"
    with input as {
        "action": "attestation_corrected",
        "actor": {
            "id": "actor:reviewer",
            "roles": [
                "reviewer"
            ],
            "organizations": []
        },
        "claim": {
            "exists": true,
            "creatorActorId": "actor:alice"
        },
        "attestation": {
            "kind": "ordinary"
        },
        "targetAttestation": {
            "exists": true,
            "authorActorId": "actor:reviewer"
        }
    }
}

test_auth_auth_021 if {
    decision.effect == "allow"
    with input as {
        "action": "attestation_corrected",
        "actor": {
            "id": "actor:editor",
            "roles": [
                "editor"
            ],
            "organizations": []
        },
        "claim": {
            "exists": true,
            "creatorActorId": "actor:alice"
        },
        "attestation": {
            "kind": "ordinary"
        },
        "targetAttestation": {
            "exists": true,
            "authorActorId": "actor:reviewer"
        }
    }
}

test_auth_auth_022 if {
    decision.effect == "deny"
    with input as {
        "action": "attestation_corrected",
        "actor": {
            "id": "actor:bob",
            "roles": [
                "reviewer"
            ],
            "organizations": []
        },
        "claim": {
            "exists": true,
            "creatorActorId": "actor:alice"
        },
        "attestation": {
            "kind": "ordinary"
        },
        "targetAttestation": {
            "exists": true,
            "authorActorId": "actor:reviewer"
        }
    }
}

test_auth_auth_023 if {
    decision.effect == "deny"
    with input as {
        "action": "attestation_corrected",
        "actor": {
            "id": "actor:reviewer",
            "roles": [
                "reviewer"
            ],
            "organizations": []
        },
        "claim": {
            "exists": true,
            "creatorActorId": "actor:alice"
        },
        "attestation": {
            "kind": "ordinary"
        },
        "targetAttestation": {
            "exists": false
        }
    }
}

test_auth_auth_024 if {
    decision.effect == "allow"
    with input as {
        "action": "claim_retracted",
        "actor": {
            "id": "actor:alice",
            "roles": [
                "contributor"
            ],
            "organizations": []
        },
        "claim": {
            "exists": true,
            "creatorActorId": "actor:alice"
        },
        "attestation": {
            "kind": "ordinary"
        },
        "targetAttestation": {
            "exists": false
        }
    }
}

test_auth_auth_025 if {
    decision.effect == "allow"
    with input as {
        "action": "claim_superseded",
        "actor": {
            "id": "actor:editor",
            "roles": [
                "editor"
            ],
            "organizations": []
        },
        "claim": {
            "exists": true,
            "creatorActorId": "actor:alice"
        },
        "attestation": {
            "kind": "ordinary"
        },
        "targetAttestation": {
            "exists": false
        }
    }
}

test_auth_auth_026 if {
    decision.effect == "deny"
    with input as {
        "action": "claim_retracted",
        "actor": {
            "id": "actor:bob",
            "roles": [
                "contributor"
            ],
            "organizations": []
        },
        "claim": {
            "exists": true,
            "creatorActorId": "actor:alice"
        },
        "attestation": {
            "kind": "ordinary"
        },
        "targetAttestation": {
            "exists": false
        }
    }
}

test_auth_auth_027 if {
    decision.effect == "deny"
    with input as {
        "action": "evidence_attached",
        "actor": {
            "id": "actor:alice",
            "roles": [
                "contributor"
            ],
            "organizations": []
        },
        "claim": {
            "exists": false
        },
        "attestation": {
            "kind": "ordinary"
        },
        "targetAttestation": {
            "exists": false
        }
    }
}

test_auth_auth_028 if {
    decision.effect == "allow"
    with input as {
        "action": "attestation_withdrawn",
        "actor": {
            "id": "actor:reviewer",
            "roles": [
                "reviewer"
            ],
            "organizations": []
        },
        "claim": {
            "exists": true,
            "creatorActorId": "actor:alice"
        },
        "attestation": {
            "kind": "ordinary"
        },
        "targetAttestation": {
            "exists": true,
            "authorActorId": "actor:reviewer"
        }
    }
}

test_auth_auth_029 if {
    decision.effect == "allow"
    with input as {
        "action": "attestation_recorded",
        "actor": {
            "id": "actor:bob",
            "roles": [
                "reviewer"
            ],
            "organizations": []
        },
        "claim": {
            "exists": true,
            "creatorActorId": "actor:alice",
            "creatorOrganizationId": "org:creator"
        },
        "attestation": {
            "kind": "independent_replication"
        },
        "targetAttestation": {
            "exists": false
        }
    }
}

test_auth_auth_030 if {
    decision.effect == "allow"
    with input as {
        "action": "attestation_recorded",
        "actor": {
            "id": "actor:alice",
            "roles": [
                "contributor"
            ],
            "organizations": []
        },
        "claim": {
            "exists": true,
            "creatorActorId": "actor:alice"
        },
        "attestation": {
            "kind": "ordinary_review"
        },
        "targetAttestation": {
            "exists": false
        }
    }
}

// Additional focused regression controls.
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
