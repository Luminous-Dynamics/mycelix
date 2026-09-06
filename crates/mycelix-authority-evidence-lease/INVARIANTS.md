# Authority Evidence Lease v0.2 — Normative Invariants

Status: **pure dynamic-evidence lease + provenance primitive; no authority source**

## 1. Lease is not authority identity

`EvidenceLease` describes only when already-qualified evidence may be reused. It MUST NOT determine subject identity, institutional permission, quorum, currentness, execution authority, or external-effect permission.

## 2. Intersections are monotone

For any live leases A and B:

- `verified_at(intersect(A,B)) = max(verified_at(A), verified_at(B))`;
- `valid_until(intersect(A,B)) = min(valid_until(A), valid_until(B))`.

Composition can therefore never make evidence look older-verified or longer-lived than any dependency.

## 3. Expiry cannot be revived

Zero-time, future-verified, inverted, or expired leases deny. Intersecting a dead lease with a live lease MUST NOT revive it.

An empty lease set is not interpreted as infinite validity.

## 4. Caps can only shorten

`cap_valid_until` may preserve or shorten a lease. A larger requested cap cannot extend the existing horizon.

## 5. Transport envelope grants no authority

`LeasedEvidence<T>` is serializable/deserializable because it crosses local runtime boundaries. Deserialization proves nothing about `T`.

A consuming authority runtime MUST independently qualify the enclosed evidence according to its domain rules and use the lease only as an additional upper bound.

## 6. Intended compatibility use

This primitive exists for legacy evidence ABIs whose semantic object carries a longer lifetime than the independent verifier proof that authenticated it.

The safe composition rule is:

`semantic/proof receipt + explicit verifier lease -> local qualification -> final authority lease <= verifier lease`.

The verifier lease MUST NOT be silently dropped before any reusable positive authority leaves the composition boundary.

## 7. Provenance manifest is audit evidence, not authority

`EvidenceLeaseContribution` binds one exact typed dependency to:

- a fixed `EvidenceLeaseRole`;
- a non-zero evidence identity digest/profile;
- an exact verification/evidence reference; and
- its exact dynamic lease.

`QualifiedEvidenceLeaseManifest` proves only that a non-empty exact contributor set was live at qualification time, that duplicate exact contributions were absent, and that its aggregate lease is the intersection of every member.

The manifest digest MUST NOT be interpreted as institutional permission, currentness, quorum, source completeness, execution authority, or external-effect permission.

## 8. Contributor order has no authority meaning

Manifest qualification canonicalizes contribution identities before hashing.

Reordering the exact same contribution set MUST preserve the manifest digest.

Changing any role, evidence digest/profile/reference, verification time, or validity horizon MUST change that contribution identity and therefore the manifest digest, subject to the cryptographic hash assumption.

## 9. Omission is visible to manifest identity

The manifest commits the exact contributor count and every canonical contribution identity.

Removing or adding a contributor changes the manifest digest. The manifest does not itself know which roles a particular authority theorem requires; the consuming domain/runtime MUST enforce its required role/set closure before treating the manifest as complete audit provenance.

## 10. Duplicate contributions are forbidden

An exact duplicate contribution identity MUST deny rather than silently appearing twice in the manifest. This keeps contributor counts and audit interpretation unambiguous.

## 11. Manifest aggregate is conservative

`aggregate_lease` is recomputed locally as:

`verified_at = max(all contribution verified_at)`

`valid_until = min(all contribution valid_until)`.

A supplied or serialized aggregate is never accepted as authoritative input to qualification.

## 12. Stable evidence identity remains separate from dynamic lease evidence

A contribution digest commits both its already-qualified evidence identity/reference and its dynamic lease. This creates an audit commitment over *why this lease exists now* without folding verifier timing into the underlying semantic authority identity.

Refreshing a verifier proof may therefore change contribution/manifest evidence identity while leaving the semantic policy/source/authority identity unchanged.

## 13. Bounded canonical set

A manifest accepts at most 1024 contributions and requires non-empty, bounded profiles/references and non-zero evidence digests.

This is a deterministic audit structure, not an unbounded provider-controlled log.

## 14. Closed role vocabulary is protocol-versioned

`EvidenceLeaseRole` is a closed semantic vocabulary. Extending that vocabulary changes the meaning of acceptable provenance and therefore MUST advance the evidence-lease protocol/manifest profile rather than silently reusing an older profile.

v0.2 retains v0.1 role codes 1-9 unchanged and appends:

- `CurrentConstitution = 10`; and
- `BootstrapRootAdoption = 11`.

Historical role codes MUST NOT be renumbered. The v0.2 contribution and manifest domain separators/profile names are distinct from v0.1 so an expanded contributor vocabulary cannot masquerade as an old canonical manifest.

## 15. Root establishment facts remain separable

`CurrentConstitution`, `BootstrapRootAdoption`, and `BootstrapRoot` are distinct provenance roles.

A consuming runtime may require all three when its authority theorem actually depended on an independently observed current constitution, a separately verified root-adoption proof, and the final qualified bootstrap root. The root role MUST NOT erase those lower-level dynamic evidence facts when they are available.

## 16. Containment

Pure Rust only. No HDK/DHT calls, persistence, randomness, discovery, policy selection, currentness, reputation, model score, lifecycle mutation, or external effects.
