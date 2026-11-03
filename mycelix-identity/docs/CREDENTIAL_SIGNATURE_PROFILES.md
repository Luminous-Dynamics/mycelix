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
V1 verification additionally requires a `DataIntegrityProof` assertion proof,
a declared algorithm matching the tagged signature algorithm, and a
verification-method controller matching the issuer DID.

The sorted-key serialization and field contract are versioned by the profile
identifier. Changing the serialized fields, JSON representation, domain
separator, hash function, or algorithm binding requires a new profile version,
not a silent V1 reinterpretation.

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
