# PSI-002B2B r2 — Policy-Trusted Xenia Registry Provider Provenance

Status: **corrected source candidate / not compile-qualified / provider provenance only**

This r2 supersedes the unqualified #2352 candidate after a claim-ceiling audit found that consumer-side signature verification alone cannot establish registry authenticity unless the Xenia producer contract is also independently qualified.

## Corrected theorem

```text
trusted Ed25519 + ML-DSA-65 provider signatures
+ exact registry subject/profile/receipt binding
+ exact verifier admitted by exact Mycelix registry trust policy
    -> PolicyTrustedXeniaRegistryProviderProvenanceV1
```

The strongest positive reports:

```text
provider_signatures_verified = true
provider_identity_trusted_under_policy = true
provider_claims_registry_authenticity = true
producer_contract_qualified = false
registry_authenticated = false
registry_current = false
psi_security_established = false
contact_discovery_composition_qualified = false
application_authority_granted = false
```

## Why registry authentication remains false

A valid provider signature proves which trusted key signed a claim. It does not prove that the signer implementation only signs claims that passed the intended registry-verification theorem.

That producer-side theorem belongs to `xenia-peer#357`:

```text
raw registry digest
    != permitted signing input

private-construction verified registry result
    -> provider attestor input
```

Only after that producer contract is independently qualified and exact qualification evidence is joined to this consumer provenance may a later type establish registry authenticity.

## Wire compatibility target

The portable envelope remains the v1 contract already communicated to xenia-peer #357:

```text
XeniaRegistryProviderAttestedReceiptV1 {
  subject_commitment_sha256,
  provider_namespace,
  verification_profile,
  provider_receipt_commitment_sha256,
  verifier_identity_commitment_sha256,
  ed25519_signature,
  ml_dsa_65_signature,
}
```

Provider transcript domain:

```text
xenia-mycelix-registry-provider-attestation-v1\0
```

Verifier identity domain:

```text
xenia-mycelix-registry/verifier-identity/v1\0
```

Fixed claim marker:

```text
registry-authenticity-established-v1
```

The transcript contains no currentness field.

## Structural join

The verifier recomputes the exact B2A structural join from subject + policy + raw observation and requires equality with the supplied structural positive before verifying provider signatures.

The envelope verifier identity must match both the B2A observation and independently supplied trusted hybrid keys.

## Positive construction

`PolicyTrustedXeniaRegistryProviderProvenanceV1` has private fields and implements `Serialize` but not `Deserialize`/`Default`.

Portable bytes cannot manufacture it.

## Source tests

The committed corpus covers:

- trusted signatures yield provenance but not registry authentication;
- self-signed verifier substitution;
- provider signature tampering;
- subject borrowing;
- provider currentness claims remain non-authoritative;
- portable envelope serde round trip without positive construction.

## Next join

A later tranche should consume:

```text
PolicyTrustedXeniaRegistryProviderProvenanceV1
+ exact qualified xenia-peer#357 producer-profile evidence
    -> RegistryAuthenticatedUnderProducerProfileV1
```

That join must bind the exact provider namespace, verification profile, verifier identity, transcript version, producer code/profile identity and qualification receipt.

Currentness remains independently handled by PSI-002B2C.

## Qualification boundary

```text
source exists
!= source compiles
!= tests pass
!= Xenia producer contract qualified
!= registry authenticated
!= registry current
```

A fresh exact-source qualifier is required; #2353 is superseded with #2352 and must not be executed.
