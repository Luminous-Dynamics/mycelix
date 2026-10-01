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

## Configuration applicability containment

MOBILITY-COMMONS-028 makes the temporal scope relationship explicit: the evidence effectivity interval must be fully contained within the referenced `TemporalConfigurationApplicability` interval for the exact same configuration revision and physical artifact.

Under the half-open `[start, end)` semantics:

- exact equality is accepted;
- strict interior containment is accepted;
- an effectivity ending exactly at the configuration end is accepted;
- an effectivity beginning before configuration applicability is rejected;
- an effectivity extending past configuration applicability is rejected;
- a finite configuration interval cannot contain an open-ended evidence effectivity;
- an open-ended configuration interval may contain a finite or open-ended evidence effectivity.

This is a scope constraint, not an epistemic judgment. Event time may be before, inside, or after the effectivity interval. Event/effectivity overlap remains descriptive and is not used to establish validity.

The implementation references the explicit temporal applicability witness rather than copying its interval into the evidence record. That avoids a second independently editable interval becoming an implicit or divergent source of truth.

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

This separation is intentional: digital-thread systems need temporal alignment and persistent associations, while the semantics of those associations must remain explicit rather than being inferred from timestamps alone. citeturn1search1turn2search0

## Historical preservation

Ending an effectivity interval does not delete or invalidate the historical event. A later record does not erase an earlier record. Lifecycle supersession remains orthogonal to temporal effectivity. Configuration successors and replacement physical artifacts require new explicit temporal applicability witnesses; containment never creates inheritance.

## Qualification boundary

This is semantic and structural qualification only. It does not establish physical test validity, engineering adequacy, certification, regulatory approval, or safety.

## Holochain boundary

The primitive is suitable for deterministic application-level validation because all declared inputs are explicit data. Holochain validation itself must remain deterministic; dependencies used during validation must be addressable, and unavailable dependencies produce unresolved-dependency outcomes rather than silently changing the semantic model. citeturn0search0turn0search2
