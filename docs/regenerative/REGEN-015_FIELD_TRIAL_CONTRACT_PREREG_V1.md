# REGEN-015 — Regenerative Field-Trial Contract Preregistration v1

Status: architecture preregistration only. No agronomic recommendation, treatment instruction, causal claim, governance authority, carbon authority, or physical-action authority.

## 1. Purpose

Freeze the evidence contract for regenerative field trials before implementation so local experimentation can accumulate useful knowledge without converting weak study designs, selective outcomes, or post-hoc interpretation into stronger claims than the evidence supports.

The core theorem is:

```text
intervention occurred
+ outcome changed
!= intervention caused the change
```

and:

```text
successful local trial
!= universal recipe
```

## 2. Trial identity and protocol identity

A `FieldTrialId` identifies one trial subject. The trial MUST separately bind an immutable preregistered protocol identity or content digest before outcome interpretation begins.

```text
FieldTrialId
!= protocol revision
!= analysis revision
```

A protocol change after evidence collection begins creates an explicit amendment/deviation lineage; it MUST NOT silently rewrite the original protocol.

## 3. Minimum trial structure

A trial contract SHOULD preserve, where applicable:

- exact `FieldTrialId`;
- exact participating `RegenerativeSiteId` / `SoilPlotId` subjects;
- exact `TreatmentArmId` values;
- trial-design class;
- protocol identity/digest;
- declared start/end or explicit unknown timing;
- intervention-material subject IDs and batch identities;
- baseline PEF observation bindings;
- preregistered endpoint bindings;
- observation schedule or schedule reference;
- assignment method;
- blocking/stratification method when used;
- declared analysis-plan identity;
- withdrawals / exclusions / missingness;
- protocol deviations;
- adverse and null outcomes;
- completion/termination state.

Absence remains explicit rather than being replaced with a favorable default.

## 4. Trial-design class

The contract MUST distinguish at least conceptually among:

```text
Observational
BeforeAfter
ControlledNonRandomized
RandomizedControlled
OtherDeclaredDesign
```

The label records the declared design; it does not prove that execution actually satisfied the design.

```text
randomized label
!= valid randomization execution
```

Execution evidence remains separate.

## 5. Arms and controls

Each arm has its own stable `TreatmentArmId` and declared role.

An arm MAY be a treatment, comparator, untreated/local-practice control, or another explicitly declared comparison.

```text
control arm exists
!= comparable counterfactual established
```

Differences in baseline soil, water, crop, management, geography, timing, or measurement method remain potential explanatory variables unless controlled or modeled explicitly.

## 6. Intervention identity

An intervention MUST reference the exact material/process subject actually used where that identity exists.

Examples include a `BiocharBatchId`, `CompostBatchId`, or `CoCompostedAmendmentBatchId`.

```text
material category = biochar
!= exact batch identity
```

A treatment record SHOULD distinguish intended intervention from observed execution evidence. A protocol saying that an amendment was to be applied does not prove application occurred as declared.

## 7. Application-event boundary

REGEN-015 records evidence about an intervention; it does not create an actuator or field-operation command.

```text
planned intervention
!= authorized intervention
!= executed intervention
!= verified intervention state
```

Any later machinery/irrigation/application automation belongs behind a separately qualified physical-action boundary.

## 8. Baseline evidence

Baseline state SHOULD be represented through the qualified REGEN-010 evidence waist and PEF rather than copied into field-trial-specific scalar fields.

A baseline observation retains its exact evidence class and provenance.

```text
baseline Inferred
!= baseline Observed
```

No trial layer may relabel computed baseline evidence as direct observation.

## 9. Endpoint preregistration

Primary and secondary endpoints SHOULD be declared before outcome interpretation.

Each endpoint SHOULD bind at least:

- endpoint identity/key;
- expected PEF phenomenon;
- acceptable evidence class or explicit policy;
- measurement/use context;
- declared analysis role (`Primary`, `Secondary`, `Safety`, `Exploratory`, or equivalent);
- temporal assessment window/reference where known.

Adding an endpoint after outcomes are visible is permitted only as an explicitly post-hoc/exploratory endpoint.

```text
post-hoc endpoint
!= preregistered primary endpoint
```

## 10. Outcome manifest completeness

A completed trial SHOULD preserve an outcome manifest for every preregistered endpoint and arm.

Outcome state should distinguish at least:

```text
ObservedResult
Missing
NotCollected
Withdrawn
Invalidated
NotApplicable
OtherDeclared
```

Missing or adverse outcomes MUST NOT disappear from the evidence bundle merely because they weaken the preferred interpretation.

## 11. Null and adverse evidence

Null and adverse findings are first-class results.

```text
no detected improvement
!= no information
```

and:

```text
adverse result
!= failed record to delete
```

The REGEN knowledge layer should retain them so later experiment selection does not repeatedly rediscover harmful or ineffective configurations.

## 12. Protocol deviations

A deviation record SHOULD bind:

- affected trial/arm;
- deviation type;
- declared time when known;
- evidence/reference;
- affected endpoint or intervention scope where known;
- whether the deviation was known before analysis.

A deviation does not automatically invalidate a trial, but it MUST remain visible to later interpretation.

## 13. Missingness and exclusions

Exclusion after observing outcomes can substantially alter interpretation. Therefore the contract SHOULD distinguish:

```text
preregistered exclusion rule
!= post-hoc exclusion decision
```

and retain the original subject/arm membership plus the exclusion/deviation lineage where legally and ethically appropriate.

## 14. Analysis-plan identity

A statistical/model analysis is a derived evidence product, not raw trial truth.

Its outputs SHOULD use PEF-2 lineage and bind the exact analysis implementation/configuration/data selection where feasible.

```text
raw outcome observations
-> declared analysis lineage
-> Derived / Inferred result
```

not:

```text
raw observations
-> overwrite with effect estimate
```

## 15. Causal-claim firewall

The field-trial record itself MUST NOT automatically emit a causal claim.

At minimum:

```text
BeforeAfter
!= causal effect

ControlledNonRandomized
!= randomized causal estimate

RandomizedControlled label
!= proof randomization/execution was valid
```

A later analysis layer may make a bounded causal claim only when its assumptions, design evidence, analysis lineage, uncertainty, and limitations are explicit.

## 16. Replication/context firewall

```text
one site
!= region

one crop
!= all crops

one soil context
!= all soils

one season
!= long-term effect
```

Replication uses a new `FieldTrialId` and preserves the contextual differences rather than silently pooling them.

## 17. Treatment material quality firewall

A trial can produce scientifically useful evidence even when a treatment batch later fails some quality profile, but the batch state must remain visible.

Conversely:

```text
quality-profile conformance
!= trial efficacy
```

REGEN-003 quality/adoption semantics and REGEN-011/012/013 material lineage remain separate inputs to trial interpretation.

## 18. Carbon/climate firewall

```text
soil carbon increased during trial
!= durable carbon removal
!= carbon-credit eligibility
```

Climate/MRV authority remains downstream and separately qualified.

## 19. Ethics, rights, and governance boundary

A valid trial record does not establish:

- land access or ownership;
- participant consent;
- legal authority;
- community legitimacy;
- environmental permission;
- food/feed safety;
- human-subject approval where applicable.

Those authorities remain in their owning domains and jurisdictions.

## 20. Proposed later executable layering

The eventual executable field-trial core should compose:

```text
REGEN-002 typed subject identities
+ qualified REGEN-010 evidence waist
+ qualified material/batch lineage where relevant
+ optional qualified REGEN-003 quality-profile references
```

It SHOULD NOT depend directly on Symthaea, Climate, Marketplace, Finance, Holochain networking, or physical-control runtimes in its dependency-light core.

## 21. Qualification target

A future REGEN-015 implementation campaign should demonstrate at least:

1. exact typed trial/arm/site/plot preservation;
2. immutable protocol identity binding;
3. endpoint-role preservation;
4. duplicate arm/endpoint rejection;
5. missing/null/adverse outcome preservation;
6. deviation/amendment lineage;
7. raw-vs-computed PEF evidence preservation;
8. post-hoc endpoint distinction;
9. no automatic causal conclusion;
10. serde/top-level revalidation where enabled;
11. ProductFrozen dependency qualification per REGEN-008;
12. clean checkout and exact ProductHead evidence.

## 22. Deliberate non-claims

REGEN-015 preregistration establishes no treatment efficacy, universal recipe, causal effect, sample representativeness, statistical significance, ecological superiority, carbon removal, legal compliance, ethical approval, economic value, governance legitimacy, or physical-action authority.
