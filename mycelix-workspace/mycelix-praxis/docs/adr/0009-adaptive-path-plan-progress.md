# ADR-0009: Adaptive Path Plan / Progress Separation

- Status: Proposed
- Subject: `PRAX-STATE-001C`
- Dependency: `PRAX-STATE-001B` / PR #2546

## Context

The legacy adaptive `AdaptivePath` mixes proposed path structure with mutable execution state:

- `steps` include `is_completed`;
- the path stores `current_step` and `completed_steps`;
- adaptation count/reason are mutable fields on the same object;
- estimated completion and confidence are mixed into the stored path.

That makes a proposed learning sequence, historical adaptation, evidence-derived progress, and UX navigation state look like one authority object.

## Decision

Praxis separates versioned path plans from evidence-derived path progress.

```text
path plan
!= step completion evidence
!= progress projection
!= credential completion
!= authorization
```

### AdaptivePathPlan

A plan version contains:

- stable plan ID + monotonically versioned plan structure;
- learner identity and optional exact goal-intent reference;
- ordered step IDs, targets, step kinds, expected durations, optionality, and rationale;
- explicit origin (`LearnerAuthored` or a named/versioned derived planner with exact inputs);
- parent plan version + adaptation reason for later versions;
- optional estimated total duration.

The plan and its steps have **no** `current_step`, `completed_steps`, or `is_completed` field.

Adaptation creates a new plan version rather than mutating historical execution state.

### AdaptivePathProgressProjection

Progress is a separate policy-relative receipt bound to:

- exact plan ID/version;
- learner identity;
- named/versioned progress policy + parameter digest;
- exact dependency refs;
- one ordered progress outcome per exact plan step;
- support/confidence metadata;
- generation time and optional estimated completion.

`validate_against_plan` requires exact plan identity, learner identity, step coverage, and step ordering.

### Step outcome vocabulary

Positive completion is named:

```text
AdmittedCompleteUnderProfile
```

not simply `Completed`.

`SkippedByLearner` is distinct from completion and may satisfy overall path navigation only for optional steps. A required step cannot be treated as complete merely because it was skipped.

### Derived navigation

The replacement model stores no `current_step` or `completed_steps` counters.

They are derived from step outcomes through functions such as:

- `admitted_complete_count()`;
- `next_unresolved_step()`;
- `admitted_complete_under_profile(plan)`.

This avoids stale counters becoming authority.

### Authority boundary

```text
AdaptivePathPlan.is_source_evidence() == false
AdaptivePathProgressProjection.is_source_evidence() == false
AdaptivePathProgressProjection.grants_credential_authority() == false
AdaptivePathProgressProjection.grants_authorization() == false
```

A learner can finish an adaptive learning path without that fact independently establishing an external credential or authorization.

## Privacy

Personalized paths reveal goals, inferred capability gaps, recommendations, and planned behavior. The eventual Holochain materialization should therefore default plan/progress state private/local and avoid public metadata indexing by convenience.

## Holochain migration

This ADR changes no DNA schema.

The later migration should keep historical `AdaptivePath` values as historical outputs. It must not synthesize exact evidence/projection dependencies or reinterpret stored `is_completed/current_step/completed_steps` values as provenance-complete evidence.

Plan revisions should be explicit new versions. Derived progress/caches should remain recomputable and non-authoritative.

## Follow-up

The next semantic state tranche should repair session/aggregate analytics vocabulary and lineage before any broad adaptive-DNA rewrite.

## Tests

The contract proves that:

1. plan structure has no execution-progress fields;
2. derived plans require planner provenance and exact inputs;
3. adapted plan versions require prior-version lineage and reason;
4. progress must match exact plan version, learner, step set, and order;
5. evaluated outcomes require exact declared inputs;
6. required steps cannot be counted complete when skipped;
7. navigation/completion counters are derived rather than stored;
8. path completion grants no credential or authorization authority.

## Core theorem

```text
new plan version
!= rewritten learning history

path navigation state
!= capability evidence

path completion under profile
!= credential completion
```
