# AC-036 — Economic Scope Fingerprint Binding

## Purpose

AC-035 binds an economic action's scope identifier across its lifecycle.

AC-036 closes the remaining identity gap: an identifier alone does not prove that the scope contents remained unchanged.

A lifecycle revision now carries a deterministic SHA-256 fingerprint of the complete AC-030 scope.

## Canonical fingerprint

The fingerprint is calculated over the serialized scope using an explicit version prefix:

`MYCELIX-ECONOMIC-ACTION-SCOPE-V1\0 || canonical-scope-bytes`

The version prefix makes the canonicalization scheme explicit and allows future changes to use a new domain/version rather than silently changing historical meaning.

The complete scope is covered, including:

- scope ID;
- action reference;
- purpose;
- required dimensions;
- policy reference;
- authority reference;
- attestation reference;
- evidence references;
- declaration timestamp.

## Lifecycle enforcement

Ordinary lifecycle updates recompute the supplied scope fingerprint and require it to match the active lifecycle fingerprint.

Therefore this is rejected:

`same scope ID + changed authority/policy/evidence -> ordinary update`

A legitimate change requires `amend_scope()`, creating a new scope identity and fingerprint while preserving the predecessor scope in history.

Historical validation also checks every adjacent revision, not just the final active fingerprint.

## Why identifiers are insufficient

A mutable registry could retain:

`scope_id = scope-17`

while changing the policy document, authority, dimensions, or evidence behind that identifier.

A lifecycle that stores only `scope-17` would therefore appear continuous while its authorization semantics had changed.

AC-036 turns the scope into content-addressed evidence:

`scope ID + fingerprint`

## Validation

The reference implementation covers:

- scope fingerprint changes when material scope content changes;
- same scope ID with changed contents is rejected;
- malformed fingerprints are rejected;
- historical scope transitions are validated;
- active fingerprint must match the final historical revision.

## Dependencies

AC-030: explicit action scope

AC-035: append-only lifecycle binding

AC-036: content binding for the scope itself

## Security contribution

This closes the sequence:

`scope identifier drift`

→

`scope content drift`

→

`lifecycle authorization drift`

without changing the economic policy itself.

The result is a more durable chain:

`policy -> scoped declaration -> content fingerprint -> lifecycle revision -> integrity assessment -> execution`
