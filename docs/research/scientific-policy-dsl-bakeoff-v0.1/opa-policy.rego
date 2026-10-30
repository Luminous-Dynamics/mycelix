package mycelix.authz

import rego.v1

// Default deny. This package implements only the pure policy layer.
// Signature, identity, key validity, acting-organization membership,
// replay/idempotency, receipt signing and durable commit stay in Rust.
default decision := {"effect": "deny", "rule": "DEFAULT_DENY"}

has_role(role) if {
    some candidate in input.actor.roles
    candidate == role
}

has_any_role(allowed) if {
    some candidate in input.actor.roles
    candidate in allowed
}

is_editor_or_institution if {
    has_any_role({"editor", "institution"})
}

is_claim_owner_or_editor if {
    input.actor.id == input.claim.creatorActorId
}

is_claim_owner_or_editor if {
    is_editor_or_institution
}

creator_organization_conflict if {
    input.actingOrganizationId == input.claim.creatorOrganizationId
}

independent_replication_is_eligible if {
    input.attestation.kind != "independent_replication"
}

independent_replication_is_eligible if {
    input.attestation.kind == "independent_replication"
    input.actor.id != input.claim.creatorActorId
    not creator_organization_conflict
}

attestation_author_or_editor if {
    input.targetAttestation.authorActorId == input.actor.id
}

attestation_author_or_editor if {
    is_editor_or_institution
}

decision := {"effect": "allow", "rule": "CLAIM_PROPOSED_ROLE"} if {
    input.action == "claim_proposed"
    input.claim.exists == false
    has_any_role({"contributor", "editor", "institution"})
}

decision := {"effect": "allow", "rule": "LEGACY_IMPORT_ROLE"} if {
    input.action == "legacy_claim_imported"
    input.claim.exists == false
    has_role("migration_service")
}

decision := {"effect": "allow", "rule": "EVIDENCE_OWNER_OR_EDITOR"} if {
    input.action == "evidence_attached"
    input.claim.exists == true
    is_claim_owner_or_editor
}

decision := {"effect": "allow", "rule": "ATTESTATION_ROLE_AND_INDEPENDENCE"} if {
    input.action == "attestation_recorded"
    input.claim.exists == true
    has_any_role({"reviewer", "contributor", "institution"})
    independent_replication_is_eligible
}

decision := {"effect": "allow", "rule": "ATTESTATION_AUTHOR_OR_EDITOR"} if {
    input.action == "attestation_corrected"
    input.claim.exists == true
    input.targetAttestation.exists == true
    attestation_author_or_editor
}

decision := {"effect": "allow", "rule": "ATTESTATION_AUTHOR_OR_EDITOR"} if {
    input.action == "attestation_withdrawn"
    input.claim.exists == true
    input.targetAttestation.exists == true
    attestation_author_or_editor
}

decision := {"effect": "allow", "rule": "CLAIM_OWNER_OR_EDITOR_LIFECYCLE"} if {
    input.action == "claim_superseded"
    input.claim.exists == true
    is_claim_owner_or_editor
}

decision := {"effect": "allow", "rule": "CLAIM_OWNER_OR_EDITOR_LIFECYCLE"} if {
    input.action == "claim_retracted"
    input.claim.exists == true
    is_claim_owner_or_editor
}
