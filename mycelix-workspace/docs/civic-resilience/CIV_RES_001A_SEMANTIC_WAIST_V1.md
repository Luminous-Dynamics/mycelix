# CIV-RES-001A — Domain-neutral civic semantics v1

Status: semantic contract only  
Parent: CIV-RES-000A / `c6a96d40895e71642292b0e2c433b13cc58ecd4d`  
Tracking issue: #2015  
Program: #2006

## Purpose

Freeze a small, universal Civic Resilience semantic waist for describing civic conditions, needs, intervention proposals, outcome targets and post-intervention observations without creating authority, entitlement, priority, causal-effect or deployment semantics.

This tranche deliberately branches from CIV-RES-000A rather than CIV-RES-000B.

```text
                 CIV-RES-000A
                    /      \
                   /        \
       CIV-RES-000B          CIV-RES-001A
       Johannesburg          universal semantics
       deployment profile
```

Johannesburg is one deployment profile. Its legal, threat, privacy, service-channel and rollout assumptions must not become universal protocol semantics.

## Central theorem

```text
observation -> need statement -> intervention proposal -> outcome target
```

is a descriptive/propositional chain only.

It is **not**:

```text
observation -> priority -> entitlement -> authorised action
```

and it is not:

```text
post-intervention observation -> causal effect
```

## Why contract-first

Current repository lines already contain overlapping but differently scoped evidence concepts:

- current mainline `civic-types::Evidence` is a justice/media custody/fact-check oriented record;
- AC-002 draft work contains useful measurement, provenance and uncertainty semantics, but they are intentionally tied to institutional capture analysis;
- wider Mycelix evidence/currentness/qualification work is still evolving.

CIV-RES-001A therefore **does not define another generic provenance, confidence, uncertainty, measurement or evidence object**.

Until the shared evidence waist is converged, CIV-RES uses opaque typed references to those external artifacts.

```text
EvidenceRef != evidence valid
MeasurementRef != measurement correct
MethodRef != method qualified
Observation recorded != observation true
```

## Universal subject classes

001A permits only non-person subject classes:

```text
Place
Service
InfrastructureAsset
InstitutionalProcess
Programme
Resource
Aggregate
```

A subject carries a stable opaque external/domain reference plus one class.

There is deliberately no `Person`, `HouseholdRisk`, `Suspect`, `Offender`, `VictimScore`, `CitizenScore`, `TrustScore` or equivalent subject class.

Person-linked operational case data belongs in later protected service/safety profiles with the CIV-RES-000A accountability/privacy boundaries.

## 1. CivicObservation

A `CivicObservation` means:

> one bounded statement about an observed civic condition, tied to an explicit subject and opaque evidence/measurement/method references.

Semantic fields:

```text
observation_id
subject_ref + subject_class
phenomenon_ref
measurement_refs[]
evidence_refs[]
method_refs[]
observed_at_ref
limitations[]
```

`observed_at_ref` is an evidence/time reference, not necessarily trusted wall-clock truth.

An observation must carry at least one evidence reference and at least one stated limitation or explicit unknown/none-applicable limitation marker defined by a future executable profile.

001A does not decide whether referenced evidence is authentic, sufficient, current or representative.

## 2. CivicNeedStatement

A `CivicNeedStatement` means:

> a proposition that a civic condition warrants attention or improvement for a defined scope.

Semantic fields:

```text
need_id
scope_ref
need_class_ref
statement
supporting_observation_refs[]
supporting_evidence_refs[]
proposer_or_source_ref
alternatives_or_limitations[]
```

A need statement is not automatically a municipal finding, priority, entitlement, budget claim or mandate.

Community input may support a need statement, but:

```text
community input != representative mandate
number of submissions != democratic authority
platform activity != priority authority
```

Representative/participatory claims require their own sampling/governance evidence.

## 3. CivicInterventionProposal

A `CivicInterventionProposal` means:

> a proposed bounded action or programme intended to address one or more need statements.

Semantic fields:

```text
proposal_id
addresses_need_refs[]
intervention_class_ref
proposal_statement
mechanism_hypothesis_refs[]
outcome_target_refs[]
constraint_refs[]
evidence_refs[]
proposer_ref
```

No proposal field grants authority.

```text
proposal exists != institution adopted proposal
proposal supported != action authorised
proposal popular != action legitimate
model recommended != action authorised
```

Adoption/authorization belongs to an owning governance/public-institution process. CIV-RES-001A may later carry an opaque external `AdoptionDecisionRef`; it does not mint that decision.

## 4. OutcomeTarget

An `OutcomeTarget` means:

> a preregistered description of what later evidence should measure if an intervention is adopted/executed.

Semantic fields:

```text
target_id
metric_ref
population_or_scope_ref
baseline_window_ref
evaluation_window_ref
direction_or_range_ref
measurement_method_ref
adverse_outcome_refs[]
spillover_indicator_refs[]
displacement_indicator_refs[]
reporting_measurement_indicator_refs[]
```

A target is not a promise, forecast, KPI truth, causal estimand or evidence that the intervention occurred.

```text
OutcomeTarget != OutcomeObservation
OutcomeTarget != InterventionEffect
```

## 5. OutcomeObservation

An `OutcomeObservation` means:

> post-intervention-period evidence relevant to one or more outcome targets.

Semantic fields:

```text
outcome_observation_id
target_refs[]
observation_refs[]
evidence_refs[]
measurement_method_refs[]
evaluation_window_ref
limitations[]
```

It does not establish causal attribution.

```text
before/after difference != causal effect
association != intervention effect
OutcomeObservation != CausalEffectEstimate
```

Causal estimates belong in the later SYM-CIVIC scientific line with a frozen study manifest, exact evidence cut, execution/model identity, identification assumptions, diagnostics and sensitivity analysis.

## Required non-equivalences

The v1 registry is machine checked:

```text
Observation != Need
Need != Priority
Need != Entitlement
NeedStatement != AdministrativeFinding
InterventionProposal != AdoptedIntervention
AdoptedIntervention != ExternalEffect
OutcomeTarget != OutcomeObservation
OutcomeObservation != CausalEffect
CausalEffectEstimate != OutcomeObservation
ReportedCondition != VerifiedCondition
Verification != CausalAttribution
CommunityPreference != InstitutionalAuthority
ModelRecommendation != InterventionAuthority
CommunityInput != RepresentativeMandate
PlatformActivity != PriorityAuthority
```

## Opaque reference boundary

001A may name only opaque references for external semantics it does not own:

```text
EvidenceRef
MeasurementRef
MethodRef
TimeEvidenceRef
AuthorityDecisionRef
ExternalEffectRef
VerificationRef
StudyRef
```

An opaque reference is a typed dependency edge, not proof that its target exists, is valid, is current or grants authority.

Later adapters must validate the referenced artifact under its owning subsystem before using it for a stronger proposition.

## No hidden priority scalar

001A deliberately has no universal priority score.

Different legitimate institutions/communities may use different, explicit prioritisation procedures. CIV-RES can carry evidence into those procedures and preserve their output references, but the universal semantic layer must not silently collapse urgency, rights, cost, benefit, public preference, vulnerability, feasibility and political mandate into one system-wide number.

## No person-risk model

The universal semantics cannot express:

```text
person -> criminality score
person -> trust score
person -> social value
person -> policing priority
person -> service worthiness
```

This is stronger than merely promising not to display such values: there is no universal subject/type surface for them in 001A.

## Lifecycle non-authority

001A defines no `Approved`, `Official`, `Authorised`, `Funded`, `Awarded`, `Entitled`, `Sanctioned`, `Executed`, or `Paid` state.

A future operational composition may carry exact external references to such institutional states, but it must preserve the CIV-RES-000A ownership theorem.

## Future executable direction

Rust ownership is intentionally deferred.

Candidate options to evaluate after evidence/AC convergence:

1. a narrow `civic-types` module if shared Civic evidence primitives converge cleanly;
2. a dependency-light `civic-resilience-types` crate if isolation is demonstrably preferable;
3. extraction of a cross-domain evidence core if several independent domains require the same evidence/measurement/uncertainty primitives.

```text
candidate runtime location != selected runtime owner
```

A later implementation tranche must justify its placement and demonstrate that it does not duplicate the qualified generic evidence surface.

## Deployment composition

A Johannesburg deployment later needs an explicit composition theorem:

```text
exact qualified CIV-RES universal semantics
+ exact qualified Johannesburg deployment profile
+ exact qualified privacy/accountability/service adapters
+ exact institutional authority bindings
= bounded deployment subject
```

not:

```text
Johannesburg profile exists -> universal semantics changed
```

## Continuation

After this contract and the sibling Johannesburg profile are reviewed/qualified, the next universal layers are:

```text
CIV-RES-001B  privacy projection + accountability composition
CIV-RES-001C  commitment -> evidence -> verification -> outcome-observation theorem
CIV-RES-002A  service-issue lifecycle composition
SYM-CIVIC-000A immutable civic study/evidence-cut bridge
```

## Qualification claim ceiling

A PASS for CIV-RES-001A can establish only that this semantic corpus preserves the closed subject/artifact/reference vocabularies, non-equivalences, no-person/no-priority/no-authority boundaries, deferred-runtime-owner rule and nonclaims.

It does not establish runtime types, evidence authenticity, observation truth, need validity, representativeness, priority, entitlement, intervention authority, external effects, outcome improvement, causal effects, municipal legitimacy, Johannesburg deployment, or deployment readiness.
