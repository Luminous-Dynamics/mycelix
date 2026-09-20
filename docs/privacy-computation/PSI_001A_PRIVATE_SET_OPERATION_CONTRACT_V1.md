# PSI-001A — Private Set Operation Capability Contract v1

Status: architectural contract only

Tracks: #2114, #2109, #2110

Parent semantic waist: PEC-001A / #2116

## Purpose

Freeze a backend-neutral contract for private set operations before any concrete PSI implementation is treated as a privacy capability.

The governing rule is:

```text
private set operation name
    != exact functionality
    != leakage profile
    != backend security
    != authorization to compare datasets
```

PSI-001A adds no cryptographic implementation and qualifies no backend.

## Closed v1 operation vocabulary

The v1 operation set is exactly:

```text
Intersection
IntersectionCardinality
IntersectionAggregate
```

These operations are deliberately distinct. A backend that can reveal the intersection is not automatically suitable for a cardinality-only requirement, because revealing more information is not a privacy-preserving substitution.

## Set semantics are theorem-bearing

Every exact profile must state whether inputs are interpreted as `MathematicalSet`, `Multiset`, or `OrderedList`. The profile must also bind element canonicalization/version, duplicate handling, equality semantics, invalid-element behavior, empty-set behavior, and maximum input cardinality/resource policy.

```text
same raw records != same set
```

unless canonicalization and equality semantics are identical.

## Exact input binding

A PSI receipt/profile must bind the exact input snapshot identity used by each party through a named mechanism whose own semantics are separately defined.

```text
input reference exists
    != input contents authenticated
    != input collection authorized
```

PSI-001A does not select a universal commitment scheme.

## Output-recipient semantics

Every profile must bind who learns the result. The v1 vocabulary is exactly:

```text
LeftPartyOnly
RightPartyOnly
BothParties
NamedRecipient
```

`NamedRecipient` requires an independently authenticated recipient reference. Output direction is theorem-bearing.

## IntersectionAggregate

`IntersectionAggregate` must bind the exact aggregation function and value ownership semantics. Candidate functions may include `Count`, `Sum`, `BoundedSum`, or `CustomQualifiedFunction`, but PSI-001A authorizes none.

```text
private matching keys + plaintext values != private aggregate
```

unless the selected protocol/profile protects the required values under the stated adversary model.

## Leakage profile

A future executable profile must declare, at minimum, the disposition of:

```text
LeftInputCardinality
RightInputCardinality
IntersectionCardinality
MatchingElements
NonMatchingElements
DuplicateMultiplicity
ElementOrdering
ProtocolMessageSizes
ProtocolTiming
AbortBehavior
CrossSessionLinkability
```

Each dimension should use the PEC leakage vocabulary rather than `private: bool`.

## Adversary and collusion model

A concrete profile must bind semi-honest vs malicious behavior, static vs adaptive corruption, party count/roles, tolerated collusion, role asymmetry, abort/selective-failure behavior, and relevant auxiliary-information assumptions.

```text
PSI correct for honest parties != malicious-secure PSI
2-party profile != multi-party profile
```

## Low-entropy identifier boundary

Contact discovery often uses phone numbers or email addresses with predictable structure or low effective entropy.

```text
hash(identifier) != private set intersection
hash(identifier) != resistance to offline enumeration
```

A contact-discovery profile must bind canonicalization, domain separation, enumeration/guessing threat model, abuse/rate controls where applicable, cross-service correlation policy, retention policy for derived identifiers, and the exact protocol mechanism protecting the identifier domain.

## OPRF/VOPRF boundary

An OPRF/VOPRF may be a building block for PSI/contact-discovery constructions, but:

```text
OPRF available != PSI implemented
VOPRF verification != set-operation correctness != input authorization
```

Any such profile must bind exact protocol suite/version, ciphersuite/group, public-info semantics, server-key identity/currentness, transcript domain, and the building block's role in the full PSI construction.

## Session and replay binding

Every authority-bearing PSI execution must bind a fresh session/domain context sufficient for its protocol, including profile identity, session identity, participant role references, input snapshot references, operation identity, output-recipient policy, and result reference where applicable.

```text
valid transcript in session A != valid transcript in session B
```

unless safe replayability is explicitly established by that profile.

## Authorization boundary

```text
PSI protocol succeeds
    != inputs were authorized for collection
    != datasets were authorized for comparison
    != result disclosure was authorized
    != downstream action is authorized
```

## Sybil and identity boundary

```text
matching identifier
    != unique human
    != current identity
    != Sybil resistance
    != membership authority
```

## Privacy-preserving substitution rule

A planner must not silently choose a profile that leaks more than requested.

```text
requested: IntersectionCardinality
candidate: reveals MatchingElements
=> Incompatible(ExcessDisclosure)
```

## First product prototype boundary

The first Mycelix PSI prototype should use synthetic data only and demonstrate contact-discovery semantics without reading a real address book. Measure input sizes, intersection size, bytes sent/received, CPU, latency, memory, repeated-query/linkability behavior, and malformed/malicious input behavior supported by the selected profile.

Prototype success remains `Experimental` until separately qualified.

## Relationship to PEC

PSI-001A consumes PEC semantics for requirements, leakage, participant/adversary/interaction models, backend identity, qualification state, and composition disposition. It must not duplicate those into a second mutable authority source.

## Required non-equivalences

```text
PSI != anonymous membership
PSI != Sybil resistance
PSI != authorization to join datasets
PSI != anonymity of transport metadata
intersection privacy != set-size privacy
intersection privacy != timing privacy
intersection privacy != cross-session unlinkability
hashing identifiers != PSI
OPRF != PSI
protocol correctness != malicious security
backend operational != backend qualified
backend qualified != application authority
```

## Nonclaims

PSI-001A establishes no PSI protocol security, OPRF/VOPRF security, malicious security, contact-discovery privacy, enumeration resistance, anonymity, Sybil resistance, identity validity, application authorization, legal/privacy compliance, production admission, or deployment readiness.
