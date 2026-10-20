# PEC-000A — Privacy-Enhancing Computation Semantic Boundary v1

Status: architectural contract only

Tracks: #2109

## Purpose

This contract freezes the semantic boundary for privacy-enhancing computation (PEC) across Mycelix and Symthaea before any new MPC, FHE, PSI, PIR, ORAM, or differential-privacy backend is treated as an authority-bearing privacy capability.

The governing rule is:

```text
privacy objective
    != primitive name
    != backend availability
    != cryptographic qualification
    != composition qualification
    != application authority
```

PEC-000A adds no cryptographic implementation, no runtime selector, no production admission, and no application authority.

## Closed v1 primitive vocabulary

The v1 primitive families are exactly:

```text
ZeroKnowledge
MultiPartyComputation
HomomorphicEncryption
PrivateSetOperation
PrivateInformationRetrieval
DifferentialPrivacy
```

`ObliviousRam` is reserved for a future version because access-pattern privacy is not equivalent to single-query PIR.

## Closed v1 objective vocabulary

The v1 privacy objectives are exactly:

```text
WitnessConfidentiality
JointInputConfidentiality
ComputeOnCiphertext
PrivateSetRelation
QueryIndexPrivacy
StatisticalDisclosureLimitation
```

An objective names the property sought by a caller. It does not assert that a selected mechanism establishes that property.

## Required non-equivalences

The following distinctions are normative:

```text
encrypted != private
ciphertext exists != plaintext confidentiality established
witness absent from public inputs != witness privacy
zero knowledge != anonymity
MPC != no leakage
MPC output correctness != malicious security
FHE != zero knowledge
FHE evaluation succeeds != parameter security established
PSI != anonymous membership
PSI result privacy != set-size privacy
PIR != anonymous network access
query-index privacy != access-pattern privacy
DP != cryptographic confidentiality
aggregate != anonymous
backend linked != backend operational
backend operational != backend qualified
backend qualified != composition qualified
composition qualified != application authority
primitive supported != requirement satisfied
receipt exists != privacy property established
```

## Primitive-to-objective relationship

PEC-000A does not freeze a universal one-to-one selector. A primitive may support several objectives, and an objective may have several valid implementations.

Typical relationships are descriptive only:

- ZeroKnowledge can support witness-confidential proof statements.
- MultiPartyComputation can support joint computation over private inputs.
- HomomorphicEncryption can support computation over ciphertext.
- PrivateSetOperation can support private intersection/cardinality/aggregate relations.
- PrivateInformationRetrieval can support query-index privacy under a named model.
- DifferentialPrivacy can limit statistical disclosure from released outputs.

These mappings do not establish security or fitness for a deployment.

## Qualification vocabulary

Any future backend exposed through the PEC semantic waist must carry one of the following dispositions:

```text
SimulationOnly
Experimental
Measured
Qualified
ProductionAdmitted
```

The ordering is not itself evidence. In particular:

```text
SimulationOnly -> must never satisfy production privacy admission
Experimental -> existence/operation evidence only
Measured -> measurement evidence only
Qualified -> exact theorem/profile qualification only
ProductionAdmitted -> requires a separate admission theorem
```

No application may infer a stronger disposition merely from a variant name, feature flag, dependency, benchmark, or successful round trip.

## Historical FHE containment

Historical TypeScript FHE/secure-aggregation code in the older workspace explicitly describes its default provider as a simulation and not cryptographically secure.

PEC-000A freezes the only permissible current classification for that implementation as:

```text
SimulationOnly
```

and therefore:

```text
historical FHE API exists
    != FHE confidentiality
    != threshold-decryption security
    != production privacy capability
```

Preserving simulation code for demos, fixtures, migration, or API prototyping is allowed. Re-labeling it as a qualified backend without independent implementation and qualification is forbidden.

## Threshold/MPC separation

Existing Feldman DKG / verifiable secret-sharing machinery is not general MPC.

```text
Shamir sharing
    != Feldman verifiability
    != DKG
    != threshold signing/decryption
    != general MPC
```

A future MPC profile must name its computation class, corruption/adversary model, leakage, interaction/round assumptions, dropout behavior, exact backend identity, and qualification evidence.

## FL composition preservation

MYC-FL-SA-000R remains an independent federated-learning composition contract.

PEC-000A does not inherit or widen its claims.

In particular:

```text
secure aggregation + Byzantine-robust aggregation
    != automatically composable secure system
```

A future PEC planner must reject a requested composition when the chosen privacy primitive hides information required by the requested robust algorithm unless another separately qualified secure-compute layer supplies the missing operation.

## Requirement model direction

PEC-001 may introduce a dependency-light Rust semantic waist with types for:

```text
PrivacyPrimitive
PrivacyObjective
PrivacyRequirement
LeakageProfile
ParticipantModel
AdversaryModel
InteractionModel
PrimitiveCapability
BackendIdentity
QualificationState
CompositionDisposition
```

That crate must model claims and compatibility only. It must not implement cryptography or mint application authority.

## Fail-closed composition

Unknown, unsupported, or unqualified composition requirements must not silently degrade to a weaker primitive.

```text
requested property unavailable
    -> Unsatisfied / Incompatible
```

not:

```text
requested property unavailable
    -> choose nearest available primitive and claim privacy
```

## Symthaea planner boundary

A future Symthaea privacy planner may recommend or compose primitives, but:

```text
planner recommendation
    != cryptographic proof
    != qualification evidence
    != application authority
```

The planner should prefer the least-disclosing practical architecture, including `DoNotCollect` or local computation when those satisfy the use case better than advanced cryptography.

## Receipts

A future privacy-computation receipt may bind exact plan/backend/transcript/leakage/qualification evidence, but receipt presence is never sufficient by itself:

```text
PrivacyComputationReceipt exists
    != computation private
    != backend secure
    != composition sound
    != policy authorized
```

## Nonclaims

PEC-000A establishes no cryptographic confidentiality, anonymity, zero knowledge, malicious-secure MPC, FHE security, PSI security, PIR security, differential-privacy budget, ORAM security, legal/privacy compliance, production admission, application authority, or deployment readiness.
