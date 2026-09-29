# ADR-0008: Recommendation Projection and Freshness

- Status: Proposed
- Subject: `PRAX-STATE-001B`
- Dependency: `PRAX-STATE-001A` / PR #2541

## Context

The legacy adaptive `Recommendation` entry stores ranking/scoring outputs together with a mutable `is_valid` boolean. That makes an inherently time- and dependency-relative judgment look like persistent truth.

A recommendation may become stale because:

- its expiry time passes;
- the learner revises the underlying goal;
- a referenced capability/progress projection is superseded;
- evidence changes the relevant learner-state estimate;
- the recommendation policy/model version changes.

Persisting `is_valid = true` cannot express those dependencies safely.

## Decision

Praxis models recommendations as expiring, recomputable advisory projections.

```text
recommendation persisted
!= recommendation currently fresh
!= learner should comply
!= credential decision
!= authorization
```

### Exact dependencies

`RecommendationProjection` declares one exact unique input set composed of:

- immutable evidence-event IDs;
- derived projection references with kind/ID/digest;
- goal-intent references with exact revision version.

Each recommendation signal and reason must cite the exact subset of declared inputs that supports it. Undeclared dependencies are rejected, duplicate dependencies are rejected, and every globally declared dependency must actually support a signal or reason.

This makes explanation/provenance inspectable rather than leaving a human-readable `reason` string as the only lineage.

### No timeless validity bit

The replacement contract has no `is_valid` field.

Temporal state is derived:

```text
now < generated_at      -> NotYetGenerated
generated_at <= now < expires_at -> Fresh
now >= expires_at       -> Expired
```

Dependency freshness remains a comparison against the exact referenced goal/projection versions rather than a mutable boolean on the recommendation itself.

### Advisory authority

Recommendations are planning aids, not evidence or authorization.

```text
RecommendationProjection.is_source_evidence() == false
RecommendationProjection.grants_credential_authority() == false
RecommendationProjection.grants_authorization() == false
```

A recommendation may suggest assessment, practice, or a credential-related learning step; that does not make the recommendation itself credential evidence.

## Privacy

Recommendations reveal inferred weaknesses, goals, retention risk, interests, and planned actions. The eventual Holochain materialization should therefore default them private/local and must not create public DHT indexes for recommendation metadata merely for discovery convenience.

## Holochain migration

This ADR changes no DNA schema.

The later adaptive-DNA migration should replace the legacy mutable `Recommendation.is_valid` shape with private derived recommendation receipts or caches. A stored cache is a performance/UX artifact, not source evidence and not timeless validity.

Historical `Recommendation` records should remain historical outputs; they must not be assigned synthetic exact dependency receipts during migration.

## Follow-up

Apply the same source-versus-projection pattern to:

- adaptive path step/completion state;
- session analytics and aggregate analytics;
- retention forecasts/caches;
- unlock suggestions and gamification projections.

## Tests

The semantic contract proves that:

1. recommendations require exact dependencies;
2. every signal/reason dependency is declared;
3. every declared dependency actually supports a signal/reason;
4. duplicate signal kinds are rejected;
5. expiry must follow generation;
6. temporal freshness is recomputed from time rather than stored as `is_valid`;
7. recommendations grant no evidence, credential, or authorization authority.

## Core theorem

```text
exact inputs + named policy + current time
-> recommendation projection

persistence alone
!= freshness
!= authority
```
