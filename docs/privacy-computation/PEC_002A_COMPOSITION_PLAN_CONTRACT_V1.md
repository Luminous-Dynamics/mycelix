# PEC-002A — Privacy Composition Plan Contract v1

Status: architectural contract only

Tracks: #2120, #2109, #2110

Parent semantic waist: PEC-001A / #2116

## Purpose

Freeze a typed composition boundary for privacy-enhancing computation so a planner can combine local computation, ordinary encryption, ZKP, MPC, FHE, PSI, PIR, DP, and later ORAM without inheriting claims that the composition has not established.

The governing rule is:

```text
primitive A qualified
+
primitive B qualified
    != composition A+B qualified
```

and:

```text
stronger-sounding cryptography != better privacy architecture
```

PEC-002A adds no cryptographic implementation and authorizes no plan.

## Closed v1 step classes

A v1 plan step is exactly one of:

```text
DoNotCollect
LocalOnlyComputation
OrdinaryEncryption
QualifiedPrimitive
```

`QualifiedPrimitive` references one PEC primitive family and an exact backend/profile evidence reference. It does not embed an unverified `qualified: bool`.

## Non-cryptographic options are first-class

A planner must be able to express that the safest architecture is to avoid collection or keep computation local.

```text
advanced cryptography available
    != remote collection necessary
```

`DoNotCollect` and `LocalOnlyComputation` still require functional compatibility with the application requirement; they are not universal answers.

## Exact requirement binding

Every plan binds an exact privacy requirement/profile identity. Human-readable labels are insufficient.

```text
same request text != same requirement profile
```

A plan produced for requirement profile A is not automatically admissible for profile B.

## Exact step identity

Every cryptographic step must bind at least:

- PEC primitive family;
- exact backend/profile identity;
- exact qualification/evidence reference;
- exact operation/function/circuit/profile identity where required by the primitive;
- declared leakage profile;
- participant/adversary/interaction assumptions.

A raw `QualificationState` declaration is not authenticated evidence.

## Dataflow edges are theorem-bearing

Composition is not just an ordered list. Every dataflow edge must bind what artifact leaves one step and what the next step assumes about it.

Examples include:

```text
plaintext -> encrypted value
credential -> ZK witness
PSI result -> aggregate input
MPC output -> DP release
FHE ciphertext -> ZK execution statement
```

The edge must state the permitted disclosure class and exact producer/consumer profile identities.

```text
step output type matches
    != privacy assumptions match
```

## Composition failure

Unknown or incompatible edges fail closed.

Examples:

```text
PIR single-query profile
+
requirement: repeated access-pattern privacy
=> Incompatible(AccessPatternPrivacyUnavailable)
```

```text
secure-sum profile
+
consumer requires per-update pairwise geometry
=> Incompatible(RequiredIntermediateUnavailable)
```

```text
PSI cardinality requirement
+
profile reveals matching elements
=> Incompatible(ExcessDisclosure)
```

## Qualification references

Production-oriented planning must distinguish:

```text
capability declaration
!= measured evidence
!= qualified backend/profile
!= admitted composition
```

A future authenticated qualification reference should bind an exact receipt/evidence digest and verification profile. PEC-002A does not define the verifier implementation.

## Composition admission

A multi-step plan has its own disposition and, where required, its own qualification/admission evidence.

```text
all steps individually admitted
    != plan admitted
```

The composition layer must be able to return:

```text
NoQualifiedPlan
```

rather than silently weakening requirements.

## Cost evidence

Latency, memory, bandwidth, proof/ciphertext size, rounds, mobile feasibility, and energy may guide planning, but:

```text
benchmark winner != privacy winner != production admission
```

Performance evidence must remain separable from security/privacy evidence.

## Symthaea planner boundary

Symthaea may propose a `PrivacyPlan`, but:

```text
planner recommendation
    != authenticated capability evidence
    != composition qualification
    != application authority
```

A production planner should consume exact admitted capability/composition references rather than trusting model-generated labels.

## Authorization boundary

Even a fully qualified privacy computation does not authorize the underlying data use.

```text
privacy requirement satisfied
    != collection authorized
    != processing authorized
    != result disclosure authorized
    != downstream action authorized
```

## Relationship to domain contracts

PEC-002A is the cross-cutting composition waist. Domain-specific contracts remain authoritative for their own semantics, including:

- MYC-FL-SA-000R / #1865 for FL robustness + secure-compute compatibility;
- PSI-001A / #2132 for private-set semantics;
- PIR-001A / #2133 for retrieval semantics;
- FHE-001A / #2135 for FHE semantics;
- MPC-002A / #2207 for general MPC semantics.

PEC-002A references but does not widen those claims.

## Required non-equivalences

```text
component qualification != composition qualification
backend identity != authenticated qualification evidence
planner recommendation != qualification
local computation != automatically private from local compromise
ordinary encryption != computation privacy
privacy-preserving computation != metadata privacy
privacy-preserving computation != application authorization
NoQualifiedPlan != permission to weaken the requirement
```

## Nonclaims

PEC-002A establishes no backend security, composition security, planner correctness, qualification-verifier correctness, production admission, application authorization, legal/privacy compliance, or deployment readiness.
