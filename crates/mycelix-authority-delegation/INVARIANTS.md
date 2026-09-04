# Mycelix Authority Delegation v0.1 — Normative Invariants

Status: **pure semantic qualification kernel; not a runtime authority source**

This crate answers two narrow questions:

1. Does one exact, currently fresh parent grant lawfully attenuate into one exact, currently fresh child grant under one **already-qualified current delegation policy authority** from `mycelix-authority-delegation-policy`?
2. Do already-qualified edges form one complete, generation-consistent, policy-consistent current root-to-target delegation lineage?

It does not store grants, choose current revocation records, adopt delegation policy, verify cryptography itself, execute effects, or infer delegation authority from reputation, role prestige, model output, or possession of a capability.

## 1. Possession is not delegability

A parent grant containing capability `X` does **not** imply that its holder may delegate `X`.

`qualify_delegation_edge` requires a real `QualifiedDelegationAuthority` produced by the policy kernel. Application data, a policy ID, a policy digest, or a cryptographically valid delegation proof is insufficient.

The attestation commits the exact current `DelegationAuthorityRef`, including policy identity and current parent/policy generations. It must equal the ref exposed by the qualified policy authority.

## 2. Policy scope is mechanically enforced

The qualified policy authority must bind the same:

- parent grant ID;
- parent canonical grant digest;
- parent generation;
- delegator;
- institution;
- jurisdiction; and
- rulebook.

The edge qualifier calls `QualifiedDelegationAuthority::validate_delegation_scope` over the actual child holder, child roles, child capabilities, delegation issue time, and delegation expiry.

Policy meaning is therefore executable semantics rather than a provider convention.

## 3. Exact grant identity

Parent and child grants use the shared canonical profile:

`mycelix-authority-grant-v1-blake3-framed-semantic`

A textual grant ID is never enough.

The attestation commits exact parent and child canonical grant digests. Same ID with changed holder, capability, rulebook, source/proof lineage, lifetime, or delegation parent is a different authority object.

## 4. Exact parent/child relation

A qualified edge requires:

- `child.delegated_from == parent.id`;
- attestation parent/child IDs exactly match the grants;
- attestation parent/child digests exactly match canonical grant identities;
- delegator equals parent holder;
- delegate equals child holder; and
- parent, child, attestation, and qualified delegation policy agree on institution, jurisdiction, and rulebook.

## 5. Authority may only attenuate

The child role set must be a subset of the parent role set.

The child capability set must be a subset of the parent capability set.

The attestation role/capability sets must exactly equal the child's sets.

The qualified policy may impose an even narrower role/capability/delegate/lifetime scope, which must also pass.

Set ordering is non-semantic. Duplicate delegated roles/capabilities are invalid rather than silently normalized. Malformed deserialized role/capability identifiers are rejected before canonicalization.

## 6. Delegation provenance is exact

The child grant must contain exactly one `AuthoritySourceKind::Delegation` source.

That source must bind the exact delegation ID and exact proof reference from the attestation.

A proof/source from another parent-child pair cannot be replayed onto changed child bytes because canonical grant identity and proof/source lineage are identity-bearing.

## 7. Lifetime can only shrink

The delegation cannot begin before the parent grant exists or outlive the parent grant.

The child cannot begin before the delegation exists and cannot outlive either the delegation or parent grant.

The qualified delegation policy may further reduce the permitted child interval and contributes its own current lease horizon.

No edge may extend parent or policy authority lifetime.

## 8. Current freshness is separate from historical identity

Parent grant, child grant, and delegation attestation each require current freshness evidence from `mycelix-authority-freshness`.

The complete edge freshness set must qualify under `BUNDLE_IDENTITY_PROFILE`.

The qualified delegation policy separately proves current parent+policy freshness and exposes a bounded lease. The final edge lease is the minimum of all relevant current-authority and semantic expiry horizons.

A grant freshness status may not become effective before the immutable grant exists. A delegation freshness status may not become effective before the immutable attestation exists.

Later re-verification of unchanged generations may refresh proof metadata/leases without changing stable authority identity. Generation or state changes alter current authority.

## 9. Delegation binds exact generations

The attestation commits exact parent and child generations.

Those generations must equal current grant freshness snapshots. The parent generation must already have been effective when delegation was issued.

The attestation's `DelegationAuthorityRef` also commits the exact current parent/policy generations qualified by the policy layer, preventing stale policy authority from being silently reused after revocation/re-authorization.

## 10. Current edge identity is policy- and revocation-sensitive

`QualifiedDelegationEdge.current_edge_digest` commits:

- exact delegation-attestation semantic identity;
- exact generation-bound edge freshness bundle; and
- the effective `allow_redelegation` decision inherited from the exact qualified policy authority.

The attestation itself commits the exact current `DelegationAuthorityRef`.

Dynamic verifier invocation timestamps/references do not define edge identity.

Qualified edge/lineage types have private fields and intentionally do not implement `Deserialize`.

## 11. Re-delegation is explicit authority

A child grant being technically usable as a parent does not imply onward delegation is lawful.

For every intermediate hop in a multi-edge lineage, the **incoming** edge must carry `allow_redelegation == true` from its exact qualified policy authority.

If any incoming policy did not explicitly authorize onward delegation, complete lineage qualification returns `RedelegationNotAuthorized`.

A one-edge root→target delegation does not require onward re-delegation authority because the target is not being used as another parent.

## 12. Complete lineage is reconstructed, not selected

`qualify_complete_delegation_lineage` receives already-qualified current edges and reconstructs the exact target-to-root chain.

It never chooses a winner by timestamp, DHT/input order, author identity, score, reputation, Phi/consciousness, stake, or model recommendation.

Input edge order is irrelevant.

## 13. Generation continuity across edges

For every intermediate grant:

`incoming.child_generation == outgoing.parent_generation`

Canonical grant identity must also match across adjacent edges.

A lineage may not splice a generation-1 child view into a generation-2 parent view of the same intermediate grant, even if each edge was independently valid at a different time.

## 14. Closed-set fail-closed lineage

The complete lineage rejects:

- missing parent edges;
- duplicate children;
- cycles;
- unrelated/extraneous edges;
- canonical identity discontinuity;
- generation discontinuity;
- unauthorized re-delegation;
- expired edge/current-policy/freshness leases;
- delegated roots; and
- depth greater than 16.

Every supplied edge must belong to the one exact root-to-target chain.

## 15. Historical verification is separate

This crate qualifies **current** delegation authority.

Later revocation must prevent new authority use without destroying auditability of a delegation/action that was valid historically.

Historical replay must resolve the policy/grant/delegation generations effective as-of the historical event rather than calling current-authority qualification with stale receipts.

## 16. Advisory systems have no authority here

Phi, consciousness scores, reputation, stake, intelligence/model output, and advisory recommendations are not inputs to delegation qualification.

They cannot authorize delegation, expand a child grant, select a lineage branch/generation, bypass a delegation policy, create re-delegation permission, revoke authority, or restore revoked authority.

## 17. Runtime follow-on remains fail-closed

A future runtime provider must independently establish the exact:

- canonical parent and child grant records;
- grant proof validity;
- institution-adopted delegation policy and policy proof/source validity;
- current parent/policy freshness used to construct `QualifiedDelegationAuthority`;
- delegation proof validity;
- current parent/child/delegation status for every required edge; and
- bounded freshness leases.

Missing provider, lookup failure, malformed receipt, ambiguous current state, revoked/superseded generation, stale lease, policy mismatch, or missing re-delegation authority must deny.

This crate does **not** enable external effects.
