# Mobility Commons — Temporal Evidence Applicability V1

## Purpose

MOBILITY-COMMONS-027 separates two temporal facts that are easy to collapse:

1. event time — when an inspection, test, observation, or maintenance event occurred;
2. effectivity time — the interval during which that evidence is explicitly declared applicable to a particular configuration and physical artifact.

These are not interchangeable.

NIST digital-thread work emphasizes internal temporal associations across heterogeneous lifecycle data and reports explicit timestamp intervals for lifecycle execution data. NIST also treats inspection, maintenance/repair, retirement, and other downstream lifecycle information as part of the extended digital thread. citeturn2search3turn1search12

## Primitive

TemporalEvidenceApplicability requires:

- a typed evidence identity;
- a ConfigurationRevision;
- a PhysicalArtifact;
- an exact evidence-to-artifact lineage edge;
- an exact configuration-to-artifact AppliesTo edge;
- an explicit event interval;
- an explicit effectivity interval.

Supported typed evidence identities are:

- InspectionRecord → InspectedAs;
- TestRecord → TestedAs;
- OperationalObservation → ObservedAs;
- MaintenanceEvent → MaintainedAs.

## Interval semantics

Both intervals use deterministic half-open semantics [start, end):

- start is included;
- bounded end is excluded;
- bounded end must be greater than start;
- omitted end means open-ended.

The effectivity interval reuses the same interval semantics established by MOBILITY-COMMONS-026.

## Critical non-inferences

An effectivity interval does not imply:

- that the evidence is true;
- that the physical artifact is safe;
- roadworthiness, seaworthiness, or flightworthiness;
- regulatory acceptance or certification;
- that the evidence remains epistemically supported;
- that a successor configuration inherits the predecessor's evidence;
- that a replacement artifact inherits the predecessor's evidence;
- that Holochain validation makes the engineering evidence authoritative.

An event interval also does not automatically define effectivity.

## Configuration containment

The evidence effectivity interval must be contained within the explicit temporal applicability interval of the configuration-to-artifact binding. This is a scope constraint: evidence cannot claim applicability outside the configuration/artifact context it names. It does not imply that the evidence is causally valid, sufficient, true, safe, or certified.

An effectivity interval may be shorter than configuration applicability. Exact equality is permitted. A finite configuration applicability interval cannot contain open-ended evidence effectivity. Because intervals are half-open, an evidence interval beginning exactly at the configuration end is outside the configuration scope and is rejected.

This separation is intentional: digital-thread systems need temporal alignment and persistent associations, while the semantics of those associations must remain explicit rather than being inferred from timestamps alone. citeturn1search1turn2search0

## Historical preservation

Ending an effectivity interval does not delete or invalidate the historical event. A later record does not erase an earlier record. Lifecycle supersession remains orthogonal to temporal effectivity.

## Qualification boundary

This is semantic and structural qualification only. It does not establish physical test validity, engineering adequacy, certification, regulatory approval, or safety.

## Holochain boundary

The primitive is suitable for deterministic application-level validation because all declared inputs are explicit data. Holochain validation itself must remain deterministic; dependencies used during validation must be addressable, and unavailable dependencies produce unresolved-dependency outcomes rather than silently changing the semantic model. citeturn0search0turn0search2
