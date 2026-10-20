# ADR-0007: Goal Intent and Progress Projection Separation

- Status: Proposed
- Subject: `PRAX-STATE-001A`
- Dependency: `PRAX-EVIDENCE-002A` / PR #2508

## Context

The legacy adaptive `LearningGoal` entry combines learner-authored intent with mutable derived state: `progress_permille` and `is_completed` live in the same object as title, targets, priority, and target date. The entry is also currently public in the adaptive DNA.

That shape creates two authority/privacy risks:

1. changing or recomputing progress can look like rewriting the learner's source intent;
2. persistence or UI presentation can make a derived percentage/completion boolean look like source truth.

Learner goals may also reveal sensitive interests, deficiencies, school/work plans, or future intentions. Full goal intent therefore should not be public by default.

## Decision

Praxis separates goal intent from progress state at the semantic root.

```text
learner-authored intent
!= learning evidence
!= derived progress
!= completion admission
!= credential
!= authorization
```

### GoalIntent

`GoalIntent` is learner-authored source state. It contains:

- stable goal-intent ID;
- learner-authored revision version;
- learner identity;
- title and description;
- typed goal targets;
- priority;
- optional target date and estimated hours;
- explicit disclosure state;
- authorship/revision timestamps.

It deliberately contains **no** progress, completion, mastery, credential, or authorization field.

A revision changes the learner's stated intent. It does not rewrite historical evidence or derived projections.

### Privacy default

`GoalDisclosure::default()` is `Private`.

The full source intent has no generic `Public` variant. Explicit sharing names an audience. If Praxis later needs public goal sharing, it should introduce a separately minimized disclosure projection instead of publishing the full source object by convenience.

### GoalProgressProjection

`GoalProgressProjection` is derived state bound to:

- exact `GoalIntentId` + intent version;
- learner identity;
- named/versioned policy + parameter digest;
- exact evidence-event IDs and/or exact derived-projection references;
- progress estimate;
- support/confidence metadata;
- policy-relative completion outcome;
- generation time.

A projection must consume at least one explicit input and rejects duplicate/malformed input references.

### Completion naming

The positive completion state is:

```text
AdmittedCompleteUnderProfile
```

not simply `Completed`.

The numeric progress estimate and completion admission remain separate. A policy may have threshold/rule semantics, but those semantics belong to the named/versioned policy rather than to the scalar itself.

### Authority boundary

A valid goal-progress projection means only that the receipt is structurally well formed for the named policy.

```text
GoalProgressProjection.is_source_evidence() == false
GoalProgressProjection.grants_credential_authority() == false
GoalProgressProjection.grants_authorization() == false
```

## Holochain migration

This ADR changes no DNA schema.

The eventual adaptive-DNA migration should replace the current public mixed `LearningGoal` shape with a private/local intent representation and separately materialized/cacheable derived progress where useful. Migration must occur only under the reviewed Praxis Holochain workspace authority.

Historical `LearningGoal.progress_permille` and `is_completed` must not be imported as provenance-complete evidence merely because they were persisted previously.

## Follow-up state work

Subsequent `PRAX-STATE-001` tranches should apply the same pattern to:

- recommendations: bind to exact state/policy versions rather than timeless `is_valid`;
- adaptive paths: separate authored/planned path structure from derived completion state;
- session/aggregate analytics: replace unqualified `mastery_gained`, `skills_mastered`, and `skills_unlocked` language with estimator/policy-relative projections;
- caches: permit persistence for UX/performance without upgrading a cache into source evidence.

## Tests

The semantic contract proves that:

1. source intent defaults private;
2. duplicate/empty targets and malformed sharing audiences are rejected;
3. progress requires exact evidence/projection inputs;
4. duplicate evidence inputs are rejected;
5. completion remains explicitly policy-relative;
6. derived progress cannot grant credential or authorization authority.

## Core theorem

```text
intent can change
without rewriting evidence

projection can be recomputed
without rewriting intent

persistence of either
!= credential authority
```
