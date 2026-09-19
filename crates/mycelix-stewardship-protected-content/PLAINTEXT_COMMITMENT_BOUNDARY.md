# Protected plaintext commitment boundary

STEW-020 treats the public STEW-002 `ProtectedRepresentation` identity as the identity of the **protected container / ciphertext representation**, not as permission to publish a raw digest of restricted plaintext.

A digest of secret plaintext can itself be sensitive metadata: known-content matching, small-domain guessing, and cross-collection equality tests can reveal information even when the payload remains encrypted.

Therefore the v1 profile requires:

```text
public protected-envelope digest
= commitment to public-safe protected-container bytes
!= raw digest of restricted plaintext
```

Consequences:

- re-encryption or key rotation may create a new exact representation while preserving the logical subject;
- provenance can relate old and new protected representations without treating migration as replacement;
- a future private/plaintext identity or blinded/keyed commitment must live behind an explicitly protected theorem and must not be inferred from the public envelope;
- STEW-020 does not claim that `ciphertext_ref` bytes were actually hashed into the STEW-001 digest; that binding requires a later cryptographic verification theorem;
- publishing a ciphertext digest can still leak equality between ciphertext objects, so threat-model review remains required.

This boundary is conservative by design: confidentiality is not weakened merely to preserve a convenient public content fingerprint.
