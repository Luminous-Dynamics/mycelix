# MYC-ZKP-RANGE-001R-P — Witness Privacy Boundary

## Status

Normative companion to `ZKP_RANGE_PROOF_REPLACEMENT_CONTRACT.md` and issue #1899.

This document narrows the meaning of privacy-related language in MYC-ZKP-RANGE-001R before any replacement AIR is promoted.

## Governing distinction

```text
witness absent from public inputs
    != witness privacy
    != zero knowledge

AIR soundness
    != proof-system privacy
```

The current Mycelix Winterfell dependency is `winterfell = 0.13.1`. Upstream Winterfell documents that its current prover is not perfect zero-knowledge and that proofs may leak information about secret inputs. Therefore a Mycelix circuit implemented with this profile MUST NOT acquire a zero-knowledge or witness-confidentiality claim merely because the witness is omitted from `PublicInputs`.

## Normative interpretation of MYC-ZKP-RANGE-001R

Where the parent contract currently says `hidden value`, `hidden x`, `private witness`, or similar language, the authoritative interpretation for the Winterfell 0.13.1 candidate is:

```text
non-public trace witness
```

and not:

```text
cryptographically hidden witness
zero-knowledge witness
confidential witness
```

`RangeMembershipV1` may establish only the range relation after its AIR is independently qualified.

## Initial implementation claim

The first implementation profile should be named as an unqualified candidate, for example:

```text
candidate-range-membership-v1
```

Its intended theorem is only:

```text
there exists one admitted signed fixed-point trace witness x
such that
min <= x <= max
```

The public statement must bind the exact numeric/range/profile identity and both public bounds.

The candidate does not claim that the resulting proof transcript conceals `x`.

## Privacy capability is a separate profile

Any future application that requires witness privacy MUST bind an explicit privacy capability/profile in addition to the range theorem.

Conceptually:

```rust
pub enum WitnessPrivacyProfile {
    None,
    // Future profiles require independent design + qualification.
}
```

A future non-`None` profile must name at least:

```text
backend + exact version
privacy construction/mechanism
statement/circuit profile
security assumptions
composition assumptions
qualification evidence identity
```

A generic backend name such as `Winterfell` is insufficient to establish a privacy property.

## Evidence taxonomy

Mycelix should keep the following claims distinct:

```text
StatementSoundness
WitnessNotPublicInput
WitnessPrivacy
CommitmentBinding
Authentication
ApplicationAuthority
```

For the initial Winterfell candidate, a future PASS may establish:

```text
StatementSoundness
WitnessNotPublicInput
```

but MUST NOT automatically establish:

```text
WitnessPrivacy
CommitmentBinding
ApplicationAuthority
```

## Qualification split

### MYC-ZKP-RANGE-001AQ — statement soundness

Qualify independently:

1. exact signed-domain encoding;
2. bit binaryity;
3. accumulator recurrence;
4. exact public minimum binding;
5. exact public maximum binding;
6. shared-witness relation between low/high differences;
7. interval translation/substitution rejection;
8. malformed-trace rejection;
9. exact trace/profile identity;
10. accepted proof-parameter profile;
11. exact-head source/dependency identity.

### Future witness-privacy qualification

A separate qualification must establish whatever privacy property the chosen mechanism actually promises, including at minimum:

1. transcript leakage analysis;
2. trace-commitment/query leakage analysis;
3. whether repeated proofs are linkable;
4. whether auxiliary metadata leaks witness information;
5. composition behavior across proofs/applications;
6. backend/version-specific assumptions;
7. adversarial privacy tests appropriate to the claimed definition;
8. an explicit statement of what is *not* hidden.

A simple census that the raw witness is absent from `PublicInputs` is useful, but it is only a `WitnessNotPublicInput` check and not a zero-knowledge test.

## Application consequences

Until a witness-privacy profile qualifies, applications MUST NOT describe the Winterfell `RangeMembershipV1` candidate as privately proving any of the following:

```text
health value
financial balance
federated-learning gradient/update
jurisdiction/location
credential attribute
anonymous membership fact
```

Those applications may eventually compose a sound range theorem with a separately qualified privacy mechanism.

## Backend migration

If Mycelix later adopts a Winterfell release/profile with an appropriate zero-knowledge construction, or another backend that supplies the required privacy property, that is a new proof/privacy profile.

Historical proofs and soundness evidence do not automatically inherit the new privacy claim.

```text
same statement semantics
    != same privacy semantics
```

## Nonclaims

MYC-ZKP-RANGE-001R-P does not claim:

- that Winterfell 0.13.1 is computationally unsound;
- that every Winterfell proof necessarily reveals the witness;
- that omission from public inputs provides no privacy at all;
- that a specific replacement privacy backend is already selected;
- that `RangeMembershipV1` has already passed soundness qualification.

It establishes the narrower rule that **Mycelix will not equate statement soundness or non-public inputs with zero knowledge**.