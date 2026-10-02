# CIV-RES-001D — Reachable option-space evidence contract v1

Status: semantic contract only
Parent: CIV-RES-001A / `bf4f63099e68bd67c02dc741ac4eff340d321b0b`
Tracking issue: #3831
Program: #2006

## Purpose

Freeze a neutral representation for the availability and reachability of civic pathways without introducing individual risk scoring, prediction, diagnosis, or authority.

The contract connects existing Civic Resilience observation/need/intervention semantics to a descriptive question:

> What non-destructive pathways are actually available to a defined scope, under what constraints, and with what evidence?

This is not a claim that the listed pathways are effective, sufficient, preferred, or appropriate for a person.

## Conceptual chain

```text
resource / service / programme observation
            ->
       option pathway
            ->
    access / barrier evidence
            ->
    resolution attempt
            ->
   outcome observation
```

None of these edges mint institutional authority or establish causal effectiveness.

`model recommendation != pathway authority`

## Candidate semantic artifacts

```text
OptionPathway
OptionAccessEvidence
OptionCapacityObservation
BarrierObservation
ResolutionAttempt
ResolutionOutcomeObservation
```

These names are domain-neutral candidates for future executable implementation. This tranche freezes their intended boundaries, not a Rust owner.

## 1. OptionPathway

An `OptionPathway` identifies a bounded, substantively distinct route from a declared current context toward a declared target state, service, resource, or resolution process.

Candidate fields:

```text
pathway_id
subject_scope_ref
target_ref
pathway_class_ref
provider_or_owner_ref
constraint_refs[]
availability_window_ref
dependency_refs[]
evidence_refs[]
limitations[]
```

An option pathway is descriptive:

```text
pathway recorded != pathway effective
pathway available != pathway suitable
pathway suitable != pathway accepted
pathway accepted != outcome improved
```

## 2. OptionAccessEvidence

An `OptionAccessEvidence` records evidence relevant to whether a pathway is reachable under an explicit scope and time/context.

Reachability should remain decomposable into:

```text
existence
geographic accessibility
temporal availability
eligibility
capacity
transport or process constraints
dependency availability
other declared access conditions
```

These dimensions must not silently collapse into a single `reachable=true` claim unless a later profile explicitly defines and evidences that predicate.

```text
resource exists != resource reachable
resource reachable != resource usable
resource usable != resource credible to subject
```

## 3. OptionCapacityObservation

An `OptionCapacityObservation` records observed capacity for a pathway/resource under an explicit measurement/source profile.

Examples include:

```text
available slots
inventory units
service hours
queue length
response capacity
transport capacity
repair capacity
```

Capacity is always scoped and time-bound.

```text
stale capacity != current capacity
missing capacity != zero capacity
arrival order != currentness
aggregate capacity != individual entitlement
aggregate option capacity != individual risk
```

## 4. BarrierObservation

A `BarrierObservation` records a bounded observed constraint that can reduce access to a pathway.

Candidate fields:

```text
barrier_id
subject_scope_ref
pathway_ref
barrier_class_ref
observed_window_ref
evidence_refs[]
severity_or_extent_ref
limitations[]
```

The universal layer does not turn a barrier into an individual danger, vulnerability, criminality, or worthiness score.

## 5. ResolutionAttempt

A `ResolutionAttempt` records an operational attempt to use a pathway under an exact scope.

It may bind:

```text
attempt_id
pathway_ref
request_or_case_ref
initiator_ref
execution_or_procedure_ref
attempt_window_ref
result_ref
limitations[]
```

`ResolutionAttempt` is not proof that the pathway was effective.

```text
attempt != completion
completion != outcome improvement
outcome improvement != causal effect
```

Existing Support/administrative/commitment/execution systems remain semantic owners of the stronger operational facts they already represent.

## 6. ResolutionOutcomeObservation

An `ResolutionOutcomeObservation` records later evidence about what happened after an attempt.

It must remain distinct from the original pathway description and from causal attribution.

```text
OutcomeObservation != CausalEffectEstimate
Observed change != intervention effect
Intervention effect != universal truth
```

## Option-space dimensions

Where evidence exists, preserve these independently:

```text
breadth
accessibility
independence/redundancy
eligibility/constraints
temporal availability
provider/dependency concentration
credibility/perceived availability
scope
currentness
uncertainty
```

Do not create a universal scalar called `resilience`, `capability`, `risk`, `safety`, or `option_score`.

## Independence and redundancy

Three nominal pathways are not three independent alternatives if all depend on the same upstream institution or resource.

```text
nominal option count != independent option count
shared dependency != redundant path
```

Any later redundancy calculation must expose its dependency assumptions rather than hiding them inside a score.

## Temporal semantics

Option availability, evidence event time, and evidence effectivity remain distinct.

```text
event time != effectivity time
effectivity time != currentness proof
currentness proof != future availability
```

A pathway becoming available later does not make it retroactively available during an earlier crisis window.

## Perceived availability

Universal CIV-RES must not infer a person's perceived option set from aggregate data.

A later research profile may measure perceptions with appropriate consent and privacy controls, but:

```text
objective availability != perceived availability
aggregate perception trend != individual state
model estimate != person-level finding
```

## Privacy and authority boundary

No universal person-risk semantics are introduced.

The semantic waist must reject or remain structurally unable to express:

```text
Person -> suicide risk score
Person -> violence risk score
Person -> criminality score
Person -> social value score
Person -> policing priority
Person -> service worthiness
```

Protected person-linked access remains governed by the existing accountability/service/safety lines.

Public projection must preserve the CIV-RES-001B release boundary:

```text
one admissible release != safe release sequence
aggregate != anonymous
coarsened != automatically safe
```

## Evidence ownership

001D does not create another generic provenance or uncertainty stack.

Use opaque typed references to existing/qualified evidence surfaces until the shared evidence waist converges.

```text
EvidenceRef != evidence valid
MeasurementRef != measurement correct
MethodRef != method qualified
OptionAccessEvidence != real-world access truth
```

## Relationship to existing CIV-RES layers

001D extends, rather than replaces, the existing semantic waist:

```text
CIV-RES-001A
  CivicObservation
  CivicNeedStatement
  CivicInterventionProposal
  OutcomeTarget
  OutcomeObservation
       |
       +--> 001D option-space evidence
```

001B remains the public/protected projection boundary.

001C remains the commitment -> completion evidence -> verification -> outcome boundary.

SUP-CIV remains the operational ticket/action authority hardening line.

FIN-SYS / economics remains the owner of economic accounting observations.

## Required adversarial cases

1. resource exists outside the declared geographic scope;
2. resource exists but is unavailable during the requested time window;
3. access requires an unmet declared eligibility condition;
4. capacity evidence is stale;
5. two capacity sources conflict;
6. transport arrival order differs from observation time;
7. multiple pathways share one hidden dependency;
8. membership is incorrectly promoted into successful support evidence;
9. aggregate availability is converted into an individual-risk claim;
10. model recommendation is converted into pathway authority;
11. pathway completion is converted into causal outcome improvement;
12. missing evidence is silently treated as zero;
13. synthetic provenance is stripped during projection;
14. public outputs allow repeated-query reconstruction;
15. a later-available pathway is incorrectly treated as available during an earlier window.

## Future executable direction

The runtime owner remains deferred until the shared evidence/AC convergence work is sufficiently qualified to avoid a duplicate generic evidence stack.

Candidate future location:

```text
narrow civic-types module
dependency-light civic-resilience-types crate
shared cross-domain evidence core
```

Runtime selection must be an explicit architecture decision.

## Qualification claim ceiling

A PASS for 001D can establish only that the exact semantic corpus preserves neutral pathway, access, capacity, barrier, resolution, temporal, privacy, provenance, and non-authority boundaries.

It does not establish:

```text
service effectiveness
community resilience
suicide prediction
violence prediction
clinical judgment
causal effect
public safety
municipal authority
legal compliance
deployment readiness
```