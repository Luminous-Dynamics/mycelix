# Authority Coordinator Release Hybrid Verifier v0.1 — Normative Invariants

Status: **native cryptographic verifier candidate; current key-policy provenance and broader deployment/effect admission remain separate**

## 1. Key authorization precedes crypto

The verifier accepts only a non-deserializable `QualifiedCoordinatorReleaseSigningKey` from the corrected #307 theorem.

It MUST NOT accept caller-supplied trusted public-key files/blobs or construct release key policy itself.

The qualified key is already bound to one exact candidate manifest digest/profile and one independently current semantic key policy.

## 2. Candidate manifest binding is rechecked locally

Before using key bytes, the verifier recomputes the #269 candidate manifest digest and requires exact equality with the qualified key's manifest digest/profile.

It also rechecks exact release authority and release-policy digest/profile equality.

Manifest substitution after key qualification denies before cryptographic success can become release authentication.

## 3. The signed message has one exact v0.1 meaning

Both algorithms sign the identical deterministic message:

- fixed message domain;
- verifier protocol version;
- fixed signature-message profile;
- fixed signature-evidence profile;
- exact #269 manifest digest/profile;
- exact release authority;
- exact release-policy digest/profile; and
- exact authorized signing key id.

Changing the manifest or key id changes the signed bytes.

## 4. Exact crypto implementations and wires

The crate pins:

- `ed25519-dalek = 2.2.0`;
- `ml-dsa = 0.1.1`.

v0.1 requires exact signature wires:

- Ed25519 signature: 64 bytes;
- ML-DSA-65 signature: 3309 bytes.

The qualified key policy separately fixes public-key wires at 32-byte Ed25519 and 1952-byte ML-DSA-65.

A future algorithm, encoding or semantic change requires a versioned successor.

## 5. Hybrid means AND

Ed25519 and ML-DSA-65 must both verify over the same message.

The Ed25519 half uses Dalek strict verification. The ML-DSA-65 half decodes the exact RustCrypto encoded key/signature types and verifies the same message.

Failure of either component denies. No OR/fallback/classical-only/PQ-only success mode exists in v0.1.

The reused Luminous hybrid construction remains experimental/unaudited; this crate does not upgrade that assurance status.

## 6. Detached signature input carries no trusted-key or proof-authority fields

`CoordinatorReleaseHybridSignature` is deserializable input containing only:

- protocol version;
- exact signature-evidence profile;
- signing key id;
- Ed25519 signature bytes; and
- ML-DSA-65 signature bytes.

It contains no public key, verification timestamp, validity horizon, verifier reference or signature reference.

## 7. Signature reference is verifier-owned

The verifier derives the #269 `signature_ref` from BLAKE3 over the exact protocol/profile, key id and both signature byte arrays using a fixed domain/profile.

A caller cannot relabel valid signature bytes with arbitrary provenance text in the positive proof.

## 8. Qualification clock follows crypto

The verifier samples host time only after both cryptographic verifications succeed.

A key or manifest that expires while verification is executing fails closed when the post-crypto clock is checked.

The caller supplies no proof clock.

## 9. Proof horizon is verifier-owned and monotone

The private #269 proof lifetime is:

`valid_until = min(manifest-bound qualified key, candidate manifest, verified_at + 5 seconds)`.

The caller cannot choose or widen that horizon.

## 10. Evidence-shaped signature proof stays private

`VerifiedCoordinatorReleaseSignatureProof` is constructed only inside the private verifier helper after successful crypto.

The public live API MUST NOT return that deserializable receipt.

It immediately calls #269 `qualify_coordinator_release` locally and returns only non-deserializable `QualifiedCoordinatorReleaseRequirement`.

Thus the active live boundary is:

`manifest-bound qualified key + detached signatures -> real hybrid crypto -> private proof -> local #269 -> authenticated release`.

## 11. This does not prove release currentness

Successful hybrid authentication establishes #269 release authenticity only.

#275 current registry head and exact status-at-head remain required afterward. A signed release may still be withdrawn or superseded.

## 12. No deployment/effect authority

This crate contains no Holochain conductor observation, target CellId selection, coordinator-set matching, stability fencing, lifecycle decision or external-effect execution.

## 13. Provisioning remains blocked

Effect-capable provisioning still requires independently qualified key-policy currentness provenance, release registry-head/status provenance, native conductor/target provenance, pre/post orchestration, post-observation race policy and final lifecycle/effect admission.
