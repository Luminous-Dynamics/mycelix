# ADR-0010: Analytics Observation and Projection Authority

- Status: Proposed
- Subject: `PRAX-STATE-001D`
- Scope: DNA-neutral `praxis-core` analytics contracts

## Context

The legacy adaptive `SessionAnalytics` and `AggregatedAnalytics` entries mix descriptive telemetry with inferred and policy-relative conclusions.

Examples include:

- `focus_estimate_permille` and `flow_balance_permille` beside raw activity counts;
- `mastery_gained` and `skills_unlocked` as unqualified session outcomes;
- `skills_mastered (>800 permille)` in period aggregates;
- `mastery_improvement_permille` without binding the value to an estimator identity/version.

Those fields are useful product signals, but they do not all have the same authority or provenance.

## Decision

Praxis separates analytics into three layers:

```text
source learning events
        |
        v
SessionObservationSummary
        |
        v
SessionAnalysisProjection
        |
        v
PeriodAnalyticsProjection
```

The core invariant is:

```text
telemetry summary
!= inferred learner state
!= capability estimate
!= admitted capability
!= credential
!= authorization
```

### 1. Session observation summaries

`SessionObservationSummary` contains descriptive/session telemetry only:

- exact source evidence-event IDs;
- learner/session identity;
- collector ID/version/parameter digest;
- session time bounds;
- attempt/completion/correct counts;
- hints/skips;
- active time;
- response-time sample count + total, from which an average can be derived;
- capabilities touched.

It deliberately contains no focus, flow, mastery, unlock, credential, or completion claim.

The summary is itself derived state over source events. Persistence does not turn it into source learning evidence.

### 2. Session analysis

Inferred signals such as focus or challenge/skill balance live in `SessionAnalysisProjection`.

Every component records:

- a descriptive component kind;
- normalized estimate;
- support-confidence metadata;
- the exact dependency subset used by that component.

The projection records analyzer ID/version/parameter digest and must include the exact `SessionObservationRef` it claims to analyze.

There is no global session mastery/authenticity/trust score.

### 3. Period analytics

`PeriodAnalyticsProjection` aggregates exact session/evidence/projection dependencies.

Descriptive metrics use descriptive names such as:

- `ObservedSessionCount`;
- `ObservedActiveTimeSeconds`;
- `ObservedItemsCompleted`;
- `ObservedAccuracyPermille`.

Policy/model-relative metrics retain that relativity in the type itself:

```text
AdmittedCapabilityCountUnderProfile {
    profile_id,
    profile_version,
    ...
}

CapabilityEstimateChangePermille {
    estimator_id,
    estimator_version,
    ...
}
```

There is intentionally no replacement metric named simply `skills_mastered` or `mastery_gained`.

## Exact-input topology

Session-analysis components and period metrics must cite exact declared inputs. The union of inputs actually used must equal the projection's declared input set.

Therefore unrelated evidence cannot be added merely to make a receipt look better supported:

```text
declared provenance
!= decorative provenance
```

Unused declared inputs are rejected.

## Authority boundary

All new analytics objects explicitly remain non-authoritative for:

- source learning evidence;
- general trust;
- credentials;
- runtime authorization.

Analytics can inform UX, reflection, planning, recommendations, and separately governed decisions. Storage does not promote them into stronger truth.

## Privacy

Session telemetry and analytics can reveal schedules, performance patterns, inferred attention, difficulties, and personal learning behavior.

Later Holochain materialization should therefore default these objects private/local and avoid public DHT indexes by convenience.

Any sharing flow should use explicit disclosure semantics and minimized projections rather than publishing full raw analytics.

## Legacy migration

Do not reinterpret historical legacy fields as provenance-complete source evidence.

In particular:

```text
legacy skills_mastered
!= independently admitted capability set

legacy mastery_gained
!= estimator-bound capability change

legacy skills_unlocked
!= authorization
```

Historical values may be retained as legacy analytics for reproducibility/display, but migration must not invent profile IDs, estimator versions, or exact event/projection lineage that the old records never captured.

## DNA boundary

This ADR changes no Holochain entry type. DNA migration remains downstream of the isolated Praxis Holochain migration authority and should implement the semantic contracts only after executable qualification.
