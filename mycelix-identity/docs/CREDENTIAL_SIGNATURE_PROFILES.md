# Credential signature profiles

## Scope

The coordinator uses two different signed-preimage profiles. They must not be
conflated.

- `mycelix-credential-hash-v1` is the current Mycelix-internal profile.
- Historical records that used `eddsa-rdfc-2022` (or had no cryptosuite) used
  the old Mycelix subset hash. The old label did not mean that the W3C
  EdDSA/RDFC transformation had been implemented.

The V1 profile uses the distinct `MycelixCredentialProof` proof type. It is
**not** an implementation of W3C Data Integrity `eddsa-rdfc-2022`, RFC 8785
JCS, or RDF Dataset Canonicalization. The
credential data model uses W3C VC-shaped fields, but this profile alone does
not establish W3C VC or Data Integrity conformance.

## Mycelix V1 signed payload

A V1 proof signs a BLAKE2b-256 digest over:

1. The domain separator
   `MYCELIX:VERIFIABLE-CREDENTIAL:SIGNATURE:V1\0`.
2. The deterministic, recursively key-sorted JSON serialization of the full
   serialized `VerifiableCredential`, with only `proof.proofValue` removed.

Consequently the signed payload covers:

- `@context`, `id`, `type`, `issuer`, `validFrom`, and `validUntil`;
- `credentialSubject` identity and all nested claims;
- `credentialSchema` and `credentialStatus`;
- proof `type`, `created`, `verificationMethod`, `proofPurpose`,
  `cryptosuite`, `algorithm`, `challenge`, and `domain`;
- `mycelix_schema_id` and `mycelix_created`.

The signature value itself is excluded because it is the output being produced.
V1 verification additionally requires a `MycelixCredentialProof` assertion proof,
a declared algorithm matching the tagged signature algorithm, and a
verification-method controller matching the issuer DID.

The sorted-key serialization and field contract are versioned by the profile
identifier. Changing the serialized fields, JSON representation, domain
separator, hash function, or algorithm binding requires a new profile version,
not a silent V1 reinterpretation.

This describes the **typed Mycelix credential model**, not arbitrary JSON-LD
documents. The current issuance path constructs the credential from typed
fields. The integrity zome now rejects unknown top-level credential members
and unknown nested issuer, schema, status, proof-header and presentation
members rather than silently discarding them during deserialization. The
subject's flattened claims remain intentionally open-ended JSON values and
are included recursively. An external JSON-LD extension can enter this
profile only as a subject claim or after a future typed schema/profile update;
it cannot be silently accepted as an unsigned top-level extension.


## Independent V1 test vector

A fixed test vector lives at
[`tests/fixtures/credential-profile-v1/`](../tests/fixtures/credential-profile-v1/).
It contains the exact canonical payload, BLAKE2b-256 digest, Ed25519 public key,
and a signature generated independently with OpenSSL-compatible Ed25519. Its
private seed is not published.

The coordinator's Rust test reconstructs the typed sample credential and
requires byte-for-byte equality with the fixture payload, equality with the
fixed digest, and successful verification of the independent signature using
RustCrypto `ed25519-dalek`. It then mutates each of 20 covered fields, one at
a time, and requires the independent signature verification to fail. The
fixture also has an OpenSSL CLI verifier:

```sh
cd mycelix-identity/tests/fixtures/credential-profile-v1
sh verify-openssl.sh
```

This is cross-implementation evidence for the internal Mycelix V1 transform
and Ed25519 primitive. The fixture public key is a standalone deterministic
test key; the fixture does not claim that the sample DID resolves to that key
through a DID Document. It does not test Holochain DID-key resolution or key
rotation. It is not a W3C Data Integrity conformance vector.

## Mycelix presentation V1

Presentations have a separate internal profile, `mycelix-presentation-hash-v1`,
and proof type `MycelixPresentationProof`. This is also not a W3C Data
Integrity cryptosuite.

The V1 presentation hash is BLAKE2b-256 over the domain separator
`MYCELIX:VERIFIABLE-PRESENTATION:SIGNATURE:V1\\0` followed by the recursively
key-sorted JSON serialization of the complete `VerifiablePresentation`,
excluding only the holder's outer `proof.proofValue`. It therefore binds the
holder, presentation ID/type/context, every embedded credential including its
issuer proofValue and status/schema fields, and every holder proof header field.

A holder V1 proof must use `MycelixPresentationProof`, purpose
`authentication`, the Ed25519 algorithm identifier, and the current
`holder DID#keys-1` verification method. As with credential V1, the current
WASM verifier derives its key directly from the DID and does not yet resolve
DID Documents or rotating key references.

Every new Mycelix V1 authentication presentation requires a non-empty
verifier challenge (1–512 UTF-8 bytes), and verification requires the exact
expected challenge. The integrity zome enforces the stored challenge so a
modified coordinator cannot commit a challenge-less new presentation.

This check establishes **challenge binding**, not freshness or single-use.
The verifier must generate an unpredictable challenge for the current
transaction/session, bind it to the intended verifier and request, and consume
it once. Without an authoritative concurrency-safe challenge registry, local
equality alone is not a global replay-prevention guarantee.

Historical presentations using the old concatenated subset
(presentation ID, holder DID, credential IDs, challenge, and domain) remain
readable, but a successful check is **PARTIAL**. Historical challenge-less
presentations fail the high-assurance authentication challenge check. Unknown
presentation profiles are **UNSUPPORTED**. The result API exposes
`holder_signature_status` separately from each embedded issuer's
`proof_signature_status`; overall presentation validity requires the holder
proof and all embedded credentials to be VERIFIED, a matching verifier
challenge, and the non-signature checks to pass.

## Selective-disclosure proof status

The current `verify_selective_disclosure` endpoint does not implement
STARK/AIR proof verification or reconstruct the credential commitment. It
therefore returns `proof_status=unsupported` and
`commitment_status=unsupported` for structurally well-formed buffers.
Structural byte-length checks are diagnostics only. The compatibility booleans
`proof_valid` and `satisfied` remain false unless both assurance statuses
are explicitly `verified`.

An empty/oversized proof buffer or malformed commitment length is
`invalid`, not `verified`. No predicate—including `NotExpired`—can currently
be treated as satisfied based on this endpoint. The domain tag is an
identifier, not a proof.

## Historical subset-hash V0

The historical implementation hashed only credential ID, issuer DID, subject
DID, valid-from, claims, and Mycelix schema ID. It did not bind all serialized
credential and proof-header fields. Some records declared
`eddsa-rdfc-2022` despite using this custom preimage.

Those records remain decodable for compatibility. A successful Ed25519 check
under the old subset hash is reported as **PARTIAL**, never **VERIFIED**.
Untagged historical Ed25519 proof values are also PARTIAL. Unknown profile
identifiers are UNSUPPORTED. No legacy value is silently relabeled as V1.

## Current assurance boundary

- V1 + Ed25519 can reach VERIFIED only after signature verification and
  consistency checks on the declared algorithm, the `MycelixCredentialProof`
  type, and the exact `issuer DID#keys-1` method currently created by issuance.
  The verifier does not yet resolve rotating key references from DID Documents;
  a future resolver-backed profile is required before key rotation can use
  different verification-method fragments.
- Hybrid Ed25519 + ML-DSA remains PARTIAL while the Holochain WASM verifier
  checks only the Ed25519 component.
- Pure ML-DSA and SLH-DSA remain UNSUPPORTED in this verifier; structural
  length checks are not cryptographic verification.
- Invalid proof bytes or inconsistent V1 header semantics are INVALID.
- Unknown proof profiles are UNSUPPORTED.

Policy consumers must accept only VERIFIED for authorization or other
high-assurance decisions. A parseable credential, successful legacy signature,
or structurally well-formed PQC envelope is not sufficient.

## Migration

An existing signature cannot be upgraded by editing its cryptosuite label or
re-hashing the old record. The issuer must reissue (or issue a separately
verifiable migration attestation for) the full credential under V1. Preserve
the old record and proof as historical evidence; do not rewrite it as though
it had always used V1.

Before external conformance claims, separately qualify the selected W3C
Data Integrity cryptosuite or an interoperable alternative, its canonicalization
and cryptographic implementation, key binding, status handling, and standard
test vectors. The Mycelix V1 profile is an internal integrity boundary, not a
substitute for that work.
