# PSI-002B2A — Registry Subject and Structural Evidence Core

Status: **structural source only / not compile-qualified / not provider verified**

This crate defines the exact subject and trust-policy semantics needed before a contact-discovery registry can ever be authenticated or treated as current.

It deliberately implements no Holochain, Xenia, signature verification, transparency log, trusted clock, or head-witness protocol.

## Exact registry subject

`RegistrySnapshotSubjectV1` binds:

- service domain;
- exact snapshot SHA-256;
- registry epoch;
- monotonic sequence;
- predecessor commitment for non-genesis snapshots;
- exact VOPRF key epoch;
- VOPRF backend/profile;
- canonicalization profile;
- equality domain;
- registry construction profile.

The subject commitment is domain separated and length prefixed.

## Trust policy

`RegistryTrustPolicyV1` contains exact trusted pairs:

```text
(provider_namespace, verifier_identity_sha256)
```

The set is canonicalized and duplicate pairs fail closed.

Policy rotation is lineage-significant: non-genesis policies bind the exact previous policy commitment.

## Structural observation

`RegistryProviderObservationV1` is untrusted wire-level data. It binds the exact subject commitment, provider/verifier identity, verification profile, provider receipt commitment, observed sequence, and provider claims about authenticity/currentness.

`bind_structurally_consistent_observation_v1` can produce only:

```text
StructurallyConsistentRegistryObservationV1
```

The name is intentional.

Even if the provider claims both authenticity and currentness, this type reports:

```text
provider_cryptographically_verified = false
registry_authenticated = false
registry_current = false
contact_discovery_composition_qualified = false
```

## Why this split matters

A caller can construct a trust policy and raw observation. Therefore structural consistency is useful for eliminating subject/policy confusion, but it is not a cryptographic trust root.

A later provider adapter must independently verify provider evidence for the exact subject and verifier identity. A later service/project trust join must then prove that the independently verified provider is trusted by the exact policy state.

Currentness remains another theorem again: an old authentic snapshot can still be stale.

## Source tests

The committed corpus covers:

- key epoch changing subject identity;
- non-genesis predecessor requirement;
- verifier ordering/canonicalization;
- duplicate provider/verifier rejection;
- provider namespace scoping;
- cross-subject evidence borrowing rejection;
- structural positive never promoting provider claims;
- policy rotation changing policy commitment;
- claimed currentness remaining non-authoritative;
- exact observed-sequence binding.

## Qualification boundary

```text
source exists
!= source compiles
!= tests pass
!= provider cryptography verified
!= registry authenticated
!= registry current
```

A separate exact-source qualifier and later provider adapter are required.
