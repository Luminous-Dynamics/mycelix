# Mycelix Stewardship Protected Content — STEW-020

STEW-020 freezes the **protected representation boundary** before Mycelix chooses a concrete encrypted-storage or key-management implementation.

A public STEW-002 envelope may safely advertise that a representation exists and is protected. The protected-content binding then points to an opaque ciphertext/container object and the policies/profiles needed to interpret its protection posture.

The binding contains **no plaintext and no key material**.

Core separations:

```text
ciphertext possession != decrypt authority
key capability != access authority
access authority != disclosure authority
access authority != AI-training authority
successful decryption != permission to redistribute
revocation != historical erasure
protected storage != culturally safe handling
```

## Required references

A v1 binding requires separate opaque references for:

- ciphertext/container location or content object;
- encryption profile;
- key policy;
- access/use policy;
- threat model;
- revocation semantics.

Optional cultural-protocol references remain a separate collection. A key policy therefore cannot silently stand in for cultural authority.

## Threat-model minimum

The referenced threat model should explicitly address at least:

- metadata and index leakage;
- access-pattern leakage;
- key compromise;
- authorized-client plaintext copying;
- revoked-member copies;
- local plaintext caches and backups;
- model embeddings, retrieval caches, or derived artifacts;
- side channels;
- historical plaintext already replicated to a DHT or other immutable medium.

This crate does not claim those hazards have been mitigated. It ensures the protected-content object has a place to bind the relevant threat model and revocation semantics instead of implying that `encrypted == safe`.

## Cryptography boundary

Concrete cryptographic transport, storage, key custody, PQC, capabilities, revocation, and secure execution should be supplied by later Xenia/Mycelix security layers. STEW-020 is crypto-agnostic and makes no algorithm-strength claim.
