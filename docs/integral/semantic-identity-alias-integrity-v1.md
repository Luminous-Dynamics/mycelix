# Semantic identity and alias integrity v1

Status: **ReferenceModelOnly**

This tranche keeps identifiers, credentials, principals, accounts, devices, resources, locators, and underlying entities as distinct semantic types.

## Identity classes

The reference oracle distinguishes:

- `SameRepresentation` — the exact same namespace, identifier, and type;
- `SameScopedEntity` — a qualified equivalence claim valid under an explicit scope/profile;
- `PotentiallySame` — similarity evidence without identity authority;
- `Distinct` — explicit evidence that the references differ;
- `Unknown` — no qualified identity conclusion.

String equality, naming similarity, retrieval rank, and AI inference do not establish `SameScopedEntity`.

## Substitution

Credential-to-principal, account-to-entity, device-to-resource, and similar substitutions require an explicit typed `EquivalenceProfile` and an exact `SemanticIdentityClaim` with matching scope, profile, direction, and equivalence class.

An identity claim is therefore not a universal alias. It is a bounded semantic bridge.

## Lifecycle

Merge, split, supersession, and revocation are append-only lifecycle events. Existing claims and bindings remain historical records. A later equivalence or merge never rewrites earlier provenance or retroactively authorizes earlier effects.

Concurrent lifecycle events that terminate the same predecessor are retained and surfaced as `IdentityLifecycleConflict`; the reference model does not select a winner by arrival order.

## Resource aliasing

Two resource identifiers may refer to the same scoped resource, but overlapping conserved-capacity claims prevent automatic union. Capacity must be explicitly reallocated rather than cloned by identity resolution.

## Privacy

Pairwise/pseudonymous references remain scoped to their namespace/profile. A privacy projection is descriptive and is structurally non-authoritative. Cross-domain identity bridges must remain explicit; they cannot silently create a universal global identity.

## Symthaea boundary

Symthaea may propose candidate identity matches or explain evidence, but those proposals are evidence for review, not authoritative identity bindings. Mycelix remains the semantic authority for the meaning and scope of identity claims.

## Qualification boundary

The executable tests qualify only deterministic reference behavior. They do not establish real-world identity, legal identity, production privacy compliance, governance legitimacy, or human outcomes.