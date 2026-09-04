# Mycelix Authority Delegation v0.1 — Normative Invariants

Status: **pure semantic qualification kernel; not a runtime authority source**

This crate answers two narrow questions:

1. Does one exact, currently fresh parent grant lawfully attenuate into one exact, currently fresh child grant under one independently authorized delegation attestation?
2. Do already-qualified edges form one complete, generation-consistent, current root-to-target delegation lineage?

It does not store grants, choose a current revocation record, verify cryptography itself, execute effects, or infer delegation authority from reputation, role prestige, model output, or possession of a capability.

## 1. Possession is not delegability

A parent grant containing capability `X` does **not** imply that its holder may delegate `X`.

Every `DelegationAttestation` must bind a separate immutable `DelegationAuthorityBinding`:

- exact authority reference;
- exact non-zero authority digest; and
- exact authority digest profile.

`VerifiedDelegationAttestation` must independently verify that exact binding in addition to verifying the delegation proof itself.

A cryptographically valid proof with the wrong/missing delegation-authority binding is insufficient.

## 2. Exact grant identity

Parent and child grants are identified with the shared canonical profile from `mycelix-authority-identity`:

`mycelix-authority-grant-v1-blake3-framed-semantic`

A textual grant ID is never enough.

The attestation commits the exact parent and child canonical grant digests. Same ID with changed holder, capability, rulebook, source/proof lineage, lifetime, or delegation parent is a different authority object.

## 3. Exact parent/child relation

A qualified edge requires:

- `child.delegated_from == parent.id`;
- attestation parent/child IDs exactly match the grants;
- attestation parent/child digests exactly match canonical grant identities;
- delegator equals parent holder;
- delegate equals child holder; and
- parent, child, and attestation have the same institution, jurisdiction, and rulebook.

## 4. Authority may only attenuate

The child role set must be a subset of the parent role set.

The child capability set must be a subset of the parent capability set.

The attestation's role/capability sets must exactly equal the child's sets.

Set ordering is non-semantic. Duplicate delegated roles/capabilities are invalid rather than silently normalized.

Malformed deserialized role/capability identifiers are rejected before canonicalization.

## 5. Delegation provenance is exact

The child grant must contain exactly one `AuthoritySourceKind::Delegation` source.

That source must bind the exact delegation ID and exact proof reference from the attestation.

A proof/source from another parent-child pair cannot be replayed onto a changed child because both the child canonical digest and source/proof lineage are identity-bearing.

## 6. Lifetime can only shrink

The delegation cannot begin before the parent grant exists or outlive the parent grant.

The child cannot begin before the delegation exists and cannot outlive either the delegation or parent grant.

No delegation edge may extend authority lifetime.

## 7. Current freshness is separate from historical identity

Parent grant, child grant, and delegation attestation each require current freshness evidence from `mycelix-authority-freshness`.

Their complete current freshness set must qualify under the registered bundle profile.

A freshness status for a grant may not become effective before the immutable grant exists.

A freshness status for a delegation may not become effective before the immutable delegation attestation exists.

Later re-verification of the same generation/state may refresh proof metadata and lease horizon without changing edge identity.

A generation/state change changes the current freshness commitment and therefore the current edge identity.

## 8. Delegation binds exact generations

The attestation commits exact parent and child generations.

Those generations must equal the current freshness snapshots supplied for parent and child.

The parent generation must already have been effective when the delegation was issued.

A stale parent/child generation cannot be replayed into a current edge.

## 9. Current edge identity is stable but revocation-sensitive

`QualifiedDelegationEdge.current_edge_digest` commits:

- exact delegation-attestation semantic identity; and
- exact generation-bound current-freshness bundle identity.

Verifier invocation timestamps and verification references are not part of this stable identity.

The qualified edge itself is not `Deserialize` and has private fields. Downstream code must obtain it through qualification rather than reconstructing authority from application data.

## 10. Complete lineage is reconstructed, not selected

`qualify_complete_delegation_lineage` receives already-qualified current edges and reconstructs the exact target-to-root parent chain.

It must never choose a winner by:

- timestamp;
- DHT arrival order;
- vector order;
- highest score;
- reputation;
- Phi/consciousness; or
- model recommendation.

Input edge order is irrelevant.

## 11. Generation continuity across edges

For every intermediate grant:

`previous_edge.child_generation == next_edge.parent_generation`

The canonical grant identity must also match across adjacent edges.

A lineage may not splice a generation-1 child view into a generation-2 parent view of the same intermediate immutable grant, even when both edge objects were independently valid at different times.

## 12. Closed-set fail-closed lineage

The complete lineage rejects:

- missing parent edges;
- duplicate children;
- cycles;
- unrelated/extraneous edges;
- identity discontinuity;
- generation discontinuity;
- expired edge/freshness leases;
- delegated roots; and
- depth greater than 16.

Every supplied edge must belong to the one exact root-to-target chain.

## 13. Historical verification is separate

This crate qualifies **current** delegation authority.

Later revocation must prevent new authority use without destroying auditability of a delegation or action that was valid historically.

Historical replay must resolve the generations effective as-of the historical event rather than reusing the current-authority function.

## 14. Advisory systems have no authority here

Phi, consciousness scores, reputation, stake, intelligence/model output, and advisory recommendations are not inputs to delegation qualification.

They cannot:

- authorize delegation;
- expand a child grant;
- select a lineage branch;
- choose a generation;
- revoke a grant; or
- restore revoked authority.

## 15. Runtime follow-on remains fail-closed

A future runtime provider must independently establish the exact:

- canonical parent and child grant records;
- grant proof validity;
- delegation authority decision/policy validity;
- delegation proof validity;
- current generation/status source for every required subject; and
- bounded freshness lease.

Missing provider, lookup failure, malformed receipt, ambiguous current state, revoked generation, or stale lease must deny.

This crate does **not** enable external effects.