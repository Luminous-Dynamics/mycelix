# Authority Control-Plane Freshness v0.1 — Invariants

## Purpose

This layer proves currentness for the **operational authority-policy plane** without asking that plane to prove itself.

It composes:

`current constitution-rooted bootstrap root`
→ `non-authoritative probe`
→ `root-embedded context/coverage semantics`
→ `source/witness coverage`
→ `complete append-only transition lineage`
→ `current control-plane policy freshness`.

This is the first ordinary `VerifiedAuthorityFreshness` boundary after the non-circular bootstrap root.

## 1. Exact bootstrap root is mandatory

Every positive `QualifiedControlPlaneSubjectFreshness` commits the exact `QualifiedAuthorityStateBootstrapRoot` qualification digest/profile.

The root must be current at qualification time.

If the constitutional/root lease expires or the constitution advances, old control-plane freshness cannot remain usable merely because source/witness evidence has a longer lease.

The emitted freshness lease is capped by `root.valid_until_ms()`.

## 2. Root policies are projected, not recursively freshness-qualified

The exact bootstrap `AuthorityCoveragePolicy` and `CoverageTrustContextPolicy` were already adopted through the constitution-rooted manifest.

This layer projects those exact policies into the #94/#96 verification ABIs using the non-deserializable qualified root.

It does **not** ask `qualify_current_freshness` whether those bootstrap policies are current before using them. Doing so would recreate the #109 cycle.

## 3. Narrow control-plane subject classes only

A subject may be qualified here only when:

- its namespace equals the root `control_plane_namespace`; and
- its kind is `AuthorityCoveragePolicy`, `CoverageTrustContextPolicy`, or an allowed root `WitnessTrustPolicy`.

Operational subjects are denied here, including grants, signing policy, threshold authorization, executor designation, effect-safety policy and delegation.

Those belong to the later operational freshness plane after the policy plane is proven current.

## 4. Probe is evidence, not policy authority

The `VerifiedCoverageChallenge` input proves only the private randomness/provenance properties of the probe.

The probe's candidate context/coverage digests must match the exact root-embedded context and coverage policy when #96 requalifies it.

A caller-selected candidate digest therefore cannot broaden authority.

## 5. Full #96 context coverage is mandatory

A positive path must call `qualify_context_bound_coverage` with:

- root-projected exact context policy;
- exact probe receipt;
- root-projected exact coverage policy;
- independently verified source-head receipt;
- exact witness receipts when required; and
- exact root trust bindings when required.

The output is not replaced by a local DHT query or a caller-provided head.

## 6. Full #91 current projection is mandatory

The resulting `VerifiedAuthorityStateCoverage` is passed to `project_current_authority_state` together with the exact independently verified transition receipts.

Therefore the transition set must be:

- non-empty;
- contiguous from generation 1;
- fork-free;
- parent-digest linked;
- source-consistent;
- causally time-monotonic; and
- exactly equal to the covered authoritative head.

A valid prefix is not current authority.

## 7. Historical state cannot become live freshness

This layer calls only `project_current_authority_state` and then `to_current_freshness_receipt`.

It does not accept historical/as-of projections as current authority.

Later revocation may block new authority without destroying historical auditability.

## 8. Root lease caps all downstream freshness

The final `VerifiedAuthorityFreshness` has:

- `verified_at_ms = max(source/projection verification, bootstrap-root verification)`; and
- `lease_until_ms = min(source/projection lease, bootstrap-root lease)`.

The final receipt is revalidated after that narrowing.

Root expiry therefore invalidates all current policy freshness derived under that root.

## 9. Stable qualification identity

One qualified subject identity commits:

- exact bootstrap-root qualification;
- exact context-bound coverage digest;
- exact current state-projection digest; and
- exact current freshness snapshot digest.

Dynamic verifier timestamps do not replace these semantic commitments.

## 10. Closed-set composition remains exact

`qualify_control_plane_freshness_set` requires an explicit semantic required-subject set and the same number of qualified subject proofs.

Every qualified subject must belong to the same current bootstrap root.

The final set is passed through #74 `qualify_current_freshness`, which rejects:

- missing subjects;
- extra/unexpected subjects;
- conflicting same-subject snapshots;
- inactive/revoked/superseded subjects; and
- stale leases.

This layer does not select a latest subject by timestamp or DHT order.

## 11. No observation heuristics

The following can never establish current control-plane authority:

- highest observed generation;
- newest timestamp;
- DHT arrival order;
- absence of a newer record;
- cached previous coverage;
- source reputation;
- Phi/consciousness/model output; or
- probe author identity.

Currentness requires the exact root + covered source head + complete transition projection.

## 12. No super-authority

This layer cannot qualify ordinary execution authority.

It establishes only the currentness of the policy plane that later operational freshness may rely on.

It cannot create:

- AuthorityGrant;
- threshold authority;
- executor designation;
- delegation authority;
- effect-safety permission;
- lifecycle claims; or
- external effects.

## 13. Adapter projection

`QualifiedControlPlaneSubjectFreshness` and `QualifiedControlPlaneFreshnessSet` are serializable but not deserializable into positive authority.

A runtime may project a qualified subject into the existing `VerifiedAuthorityFreshness` ABI only after obtaining the non-deserializable qualified object from this pure kernel.

## 14. Containment

This crate is pure Rust.

It performs no:

- HDK/Holochain calls;
- persistence;
- DHT queries;
- remote calls;
- signature verification;
- challenge generation;
- policy discovery;
- lifecycle mutation; or
- external effects.

## 15. Pre-production qualification

Before a runtime current-freshness verifier consumes this layer:

- rustfmt/tests/Clippy must pass;
- bootstrap-root rotation tests must pass;
- hidden-later-revocation and source-head omission must deny;
- root policy mismatch must deny;
- wrong control-plane namespace/kind must deny;
- revoked policy must produce a non-Active freshness receipt and fail closed-set qualification;
- historical projection replay must deny; and
- root expiry during an otherwise live source lease must deny current use.

Related: #114, #111, #109, #105, #103, #100, #96, #94, #91, #74, #73.
