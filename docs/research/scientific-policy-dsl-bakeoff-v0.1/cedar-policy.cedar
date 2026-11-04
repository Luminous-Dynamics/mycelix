// Reference Cedar candidate for the Mycelix scientific authorization corpus.
// This encodes pure authorization decisions only. Signature, identity, key,
// organization-membership, replay, receipt and commit gates remain in Rust.

permit (
    principal,
    action == Mycelix::Action::"claim_proposed",
    resource
)
when {
    !resource.exists &&
    (
        principal.roles.contains("contributor") ||
        principal.roles.contains("editor") ||
        principal.roles.contains("institution")
    )
};

permit (
    principal,
    action == Mycelix::Action::"legacy_claim_imported",
    resource
)
when {
    !resource.exists &&
    principal.roles.contains("migration_service")
};

permit (
    principal,
    action == Mycelix::Action::"evidence_attached",
    resource
)
when {
    resource.exists &&
    (
        (resource has creatorActorId && principal.actorId == resource.creatorActorId) ||
        principal.roles.contains("editor") ||
        principal.roles.contains("institution")
    )
};

permit (
    principal,
    action == Mycelix::Action::"attestation_recorded",
    resource
)
when {
    resource.exists &&
    (
        principal.roles.contains("reviewer") ||
        principal.roles.contains("contributor") ||
        principal.roles.contains("institution")
    ) &&
    (
        context.attestationKind != "independent_replication" ||
        (
            resource has creatorActorId &&
            principal.actorId != resource.creatorActorId &&
            !(
                (context has actingOrganizationId) &&
                (resource has creatorOrganizationId) &&
                context.actingOrganizationId == resource.creatorOrganizationId
            )
        )
    )
};

permit (
    principal,
    action == Mycelix::Action::"attestation_corrected",
    resource
)
when {
    resource.exists &&
    context.targetAttestationExists &&
    (context has targetAttestationAuthorId) &&
    (
        principal.actorId == context.targetAttestationAuthorId ||
        principal.roles.contains("editor") ||
        principal.roles.contains("institution")
    )
};

permit (
    principal,
    action == Mycelix::Action::"attestation_withdrawn",
    resource
)
when {
    resource.exists &&
    context.targetAttestationExists &&
    (context has targetAttestationAuthorId) &&
    (
        principal.actorId == context.targetAttestationAuthorId ||
        principal.roles.contains("editor") ||
        principal.roles.contains("institution")
    )
};

permit (
    principal,
    action == Mycelix::Action::"claim_superseded",
    resource
)
when {
    resource.exists &&
    (
        (resource has creatorActorId && principal.actorId == resource.creatorActorId) ||
        principal.roles.contains("editor") ||
        principal.roles.contains("institution")
    )
};

permit (
    principal,
    action == Mycelix::Action::"claim_retracted",
    resource
)
when {
    resource.exists &&
    (
        (resource has creatorActorId && principal.actorId == resource.creatorActorId) ||
        principal.roles.contains("editor") ||
        principal.roles.contains("institution")
    )
};
