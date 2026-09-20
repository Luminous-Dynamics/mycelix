# PSI-002B2C — Registry Currentness Structural Core

Status: **structural claim-evaluation source / not provider evidence / not currentness**

This crate is intentionally a sibling of PSI-002B2B provider authenticity. It depends on the B2A registry subject model but does not depend on the Xenia authenticity adapter.

That topology reflects the semantic rule:

```text
registry authenticity
!= registry currentness
```

A later composition must join both results for the exact same registry subject.

## What this crate does

It evaluates caller-supplied registry-head observations against one exact currentness requirement and candidate `RegistrySnapshotSubjectV1`.

It can detect structural conditions such as:

- no evidence;
- duplicate-provider/quorum amplification;
- provider disagreement/forks;
- candidate sequence behind the agreed claim;
- candidate sequence ahead of available claims;
- insufficient distinct providers;
- service/registry-epoch mismatch;
- evidence-profile mismatch;
- missing/mismatched clock profile;
- future observations;
- observations outside a configured maximum-age window.

## What it cannot do

Even if all supplied observations agree that the candidate is latest, the strongest result is:

```text
StructurallyConsistentLatestClaim
```

and the result still reports:

```text
registry_current = false
completeness_established = false
provider_evidence_verified = false
contact_discovery_composition_qualified = false
```

This is deliberate. Caller-supplied claims cannot manufacture `CurrentUnderProfile`.

## Quorum semantics

Distinct providers are counted by exact pair:

```text
(provider_namespace, verifier_identity_sha256)
```

A duplicate pair fails closed instead of counting twice.

The first profile is deliberately conservative: all admitted observations must agree on the exact claimed latest `(sequence, subject commitment)`. Divergent claims are a conflict rather than being majority-voted away.

## Clock boundary

A maximum-age requirement must name an exact clock profile, and each observation plus the supplied clock reference must name that same profile.

Even then:

```text
clock profile names match
!= clock trusted
```

Provider/trusted-clock verification belongs to a later adapter/evidence layer.

## Fork boundary

Same-sequence different-subject claims are first-class conflicts.

```text
N -> A
N -> B
A != B
-> Conflict
```

No lexicographic or arrival-order winner exists.

## Source corpus

The committed tests cover:

- agreeing multi-provider claims remain structural-only;
- empty evidence;
- duplicate-provider amplification;
- same-sequence fork/conflict;
- candidate-behind claimed-stale result;
- candidate-ahead incomplete result;
- insufficient distinct providers;
- stale observation under an exact clock profile;
- future observation rejection.

## Qualification boundary

```text
source exists
!= source compiles
!= tests pass
!= head evidence cryptographically verified
!= clock trusted
!= completeness established
!= registry current
```

A separate exact-source qualifier and later provider-specific currentness evidence adapters are required.
