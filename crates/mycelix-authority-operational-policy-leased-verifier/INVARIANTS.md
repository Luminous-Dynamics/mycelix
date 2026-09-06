# Operational Policy Leased Verifier v0.1 — Normative Invariants

Status: **pure lease-complete compatibility verifier; no currentness authority**

## 1. #172 remains unchanged

This crate does not weaken or replace #172. The historical #172 qualifier continues to deny when its legacy policy receipt cannot represent a shorter independent verifier horizon.

For cases where the proof horizon covers the semantic policy lifetime, this verifier MUST reconstruct the exact same legacy policy receipt as #172.

## 2. Short verifier leases remain explicit

For lease-complete qualification:

`valid_until = min(policy semantic expiry, record-proof horizon, adoption-proof horizon)`.

A currently valid short proof lease may therefore qualify even when it does not span the policy's entire semantic lifetime. The shorter lease must remain attached to the transport and cap downstream reusable authority.

## 3. Record authenticity and adoption remain separate

Record proof and institutional adoption proof are independently validated against the exact locally recomputed policy digest/profile. Record proof cannot certify institutional authority; adoption proof cannot certify record location/authenticity.

## 4. Currentness remains #115/#116

This kernel proves immutable policy record authenticity plus exact institutional adoption only. It does not inspect authority-state lineage, decide `Active`, or create operational authority.

`policy semantics ≠ record authenticity ≠ institutional adoption ≠ generation currentness`.

## 5. Lease is dynamic evidence

The lease is not part of stable policy identity. Refreshing a proof may update the evidence lease/verification reference without changing the semantic policy identity.

## 6. Positive results are non-deserializable

The `QualifiedLeased*PolicyVerification` types derive `Serialize` but not `Deserialize`. Their `LeasedEvidence` projections are transport only and create no authority by deserialization.

## 7. Containment

Pure Rust only. No HDK/DHT, discovery, policy selection, currentness, lifecycle mutation, external effect, reputation, Phi or model-score authority.
