# PSI-002B2B — Xenia Registry Provider Verification

Status: **source candidate / not compile-qualified / authenticity only / not currentness**

This crate consumes the PSI-002B2A structural registry subject/policy model and adds one concrete provider-verification path for Xenia.

## Exact theorem

```text
B2A exact structural subject/policy/observation join
+ portable Xenia provider envelope
+ independently configured trusted Ed25519 verifier key
+ independently configured trusted ML-DSA-65 verifier key
+ both signatures valid over the same exact registry transcript
+ exact provider/verifier pair admitted by the exact B2A trust policy
    -> ProjectPolicyTrustedXeniaRegistrySnapshotV1
```

The positive result establishes provider provenance and registry authenticity **under that exact project policy**.

It does not establish currentness.

## No envelope-carried authority

The portable envelope contains signatures and commitments but no authoritative verifier keys. Trusted verifier keys are supplied independently by the consumer.

A self-signed attacker therefore cannot choose its own keys and promote its own envelope.

## Registry-specific transcript

The v1 provider transcript is domain separated as:

```text
xenia-mycelix-registry-provider-attestation-v1\0
```

and length-prefix binds:

- exact registry subject commitment;
- provider namespace;
- verification profile;
- provider receipt commitment;
- registry-specific hybrid verifier identity commitment;
- fixed authenticity-only claim marker.

There is deliberately **no currentness claim field** in this envelope profile.

## Verifier identity

The exact trusted Ed25519 + ML-DSA-65 public-key pair is committed under:

```text
xenia-mycelix-registry/verifier-identity/v1\0
```

The resulting SHA-256 identity must match both:

- the B2A raw provider observation admitted by the trust policy; and
- the portable Xenia provider envelope.

## Recomputed structural join

The consumer does not inspect private fields of the B2A structural-positive type. Instead it recomputes:

```text
bind_structurally_consistent_observation_v1(subject, policy, raw_observation)
```

and requires exact equality with the supplied structural positive before provider crypto is considered.

This preserves B2A's private-field boundary while preventing a valid structural positive from being paired with a different raw observation.

## Currentness boundary

Even a successful provider verification returns:

```text
provider_provenance_verified = true
registry_authenticated_under_policy = true
registry_current = false
psi_security_established = false
contact_discovery_composition_qualified = false
application_authority_granted = false
```

Currentness belongs to PSI-002B2C / #2341.

## Source corpus

The committed tests cover:

- exact trusted hybrid envelope → policy-trusted authenticity positive;
- self-signed verifier substitution;
- Ed25519 signature tampering;
- cross-registry-subject evidence borrowing;
- verifier/trust-policy rotation;
- claimed currentness not promoted;
- non-positive authenticity declaration rejection;
- portable envelope serde round-trip without authority construction.

## Producer boundary

The matching Xenia producer is tracked by `xenia-peer#357`. The production Xenia signing API must accept only a private-construction verified registry positive type; it must not expose a generic `sign arbitrary registry digest` API.

The source tests in this crate generate provider envelopes only as test fixtures. That does not establish the Xenia producer theorem.

## Qualification boundary

```text
source exists
!= source compiles
!= tests pass
!= Xenia producer qualified
!= registry current
!= contact-discovery composition qualified
```

A separate exact-source qualifier is required before any executable PASS claim.
