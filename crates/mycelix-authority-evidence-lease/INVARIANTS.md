# Authority Evidence Lease v0.1 — Normative Invariants

Status: **pure dynamic-evidence primitive; no authority source**

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

## 7. Containment

Pure Rust only. No HDK/DHT calls, persistence, randomness, discovery, policy selection, currentness, reputation, model score, lifecycle mutation, or external effects.
