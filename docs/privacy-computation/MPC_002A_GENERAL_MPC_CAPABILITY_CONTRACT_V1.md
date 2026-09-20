# MPC-002A — General MPC Capability Contract v1

Status: architectural contract only

Tracks: #2123, #2109, #2110

Parent semantic waist: PEC-001A / #2116

## Purpose

Freeze a backend-neutral contract for general secure multi-party computation before secret sharing, secure aggregation, threshold cryptography, or an MPC-shaped API is treated as a general MPC capability.

The governing rule is:

```text
secret sharing
    != threshold cryptography
    != secure aggregation
    != general MPC
    != malicious-secure MPC
    != application authority
```

MPC-002A adds no cryptographic backend and qualifies no protocol.

## Closed v1 computation classes

```text
SecureAggregation
ArithmeticCircuit
BooleanCircuit
QualifiedCustomFunction
```

A backend/profile must name the exact supported class. Supporting secure sums does not imply arbitrary circuits or application-specific predicates.

## Corruption model

Every profile must bind:

- semi-honest vs malicious behavior;
- static vs adaptive corruption;
- maximum corrupted parties;
- honest-majority vs dishonest-majority assumptions where relevant;
- authenticated-channel/broadcast/setup assumptions;
- collusion assumptions.

```text
semi-honest security != malicious security
static corruption != adaptive corruption
honest-majority profile != dishonest-majority profile
```

## Fairness and output delivery

Correctness/privacy do not imply fairness or guaranteed output delivery.

Every profile must state dispositions for:

```text
Fairness
GuaranteedOutputDelivery
AbortAllowed
SelectiveAbortResistance
```

```text
MPC computes correct result
    != every honest party receives the result
```

## Preprocessing/setup

Every profile must bind setup/preprocessing requirements, including where applicable:

```text
None
PublicParameters
TrustedDealer
CorrelatedRandomness
OfflineMpcPreprocessing
ProtocolSpecificSetup
```

Setup identity and currentness are theorem-bearing; successful online execution does not validate setup provenance.

## Dropout and rejoin

Every profile must bind whether parties may disappear or rejoin and how that changes:

- privacy threshold;
- correctness threshold;
- liveness/output delivery;
- reconstruction authority;
- transcript semantics.

```text
dropout tolerated != privacy threshold unchanged
```

## Exact computation identity

An authority-bearing execution must bind the exact computation/function identity rather than a human label such as `aggregate` or `score`.

Candidate bindings include a qualified circuit/program/function digest plus canonical public-parameter/input schema identities.

```text
same function name != same computation
```

## Input/output disclosure

Profiles must bind which parties learn:

- their own inputs;
- public inputs;
- intermediate values, if any;
- final output;
- abort/failure information;
- participant membership/count;
- transcript/network metadata permitted by the profile.

MPC privacy is not equivalent to anonymity or metadata privacy.

## Secure aggregation boundary

`SecureAggregation` is one narrow MPC-like computation class.

```text
secure sum
    != access to individual updates
    != pairwise geometry
    != Byzantine-robust selection
    != arbitrary MPC
```

MYC-FL-SA-000R / #1865 remains the domain-specific FL composition contract. MPC-002A does not widen its claims.

## Threshold/DKG boundary

Existing `feldman-dkg` may provide threshold/DKG building blocks but:

```text
VSS/DKG available != MPC backend available
```

MPC-001B/#2136 and MPC-001C/#2137 remain prerequisites for relying on that threshold foundation in higher-level constructions.

## Receipt boundary

A future `MpcReceipt` may bind computation, parties, backend/profile, transcript, setup, result and qualification evidence, but:

```text
MpcReceipt exists
    != input privacy established
    != malicious security established
    != fairness established
    != output authorized
```

## First implementation boundary

The first concrete backend should demonstrate a narrow synthetic computation with externally checkable reference vectors and an explicit corruption model before arbitrary-circuit or application-sensitive claims.

The preferred first profile is not predetermined by this contract.

## Required non-equivalences

```text
secret sharing != MPC
threshold cryptography != MPC
secure aggregation != arbitrary MPC
correctness != input privacy
input privacy != malicious security
MPC != fairness
MPC != guaranteed output delivery
MPC != anonymity
backend operational != backend qualified
backend qualified != composition qualified
composition qualified != application authority
```

## Nonclaims

MPC-002A establishes no general MPC implementation, semi-honest or malicious security, adaptive security, fairness, guaranteed output delivery, dropout security, setup security, secure aggregation qualification, threshold-cryptography qualification, application authorization, production admission, or deployment readiness.
