# ADR-0004: BKT Is a Deterministic Advisory Projection

- Status: Proposed
- Subject: `PRAX-ESTIMATE-001`
- Depends on: `PRAX-EVIDENCE-001`, `PRAX-EVIDENCE-002A`

## Context

The historical adaptive zome stores a mutable `SkillMastery` entry and updates it directly from a caller-provided correctness bit. That is useful personalization state, but the storage shape collapses several distinct claims:

```text
observation
→ model update
→ "mastery" state
→ threshold-derived "just mastered"
```

A model estimate should instead be reproducible from the exact admitted evidence and exact estimator configuration that produced it.

Classic BKT is also sequence-dependent. If evidence order is implicit, two implementations can produce different values from the same set of observations. Floating-point execution can additionally create avoidable native/WASM drift near thresholds.

## Decision

Praxis models BKT as a pure advisory projection over explicitly admitted, provenance-complete attempt observations.

```text
complete observation
+ explicit admission
+ explicit estimator parameters
+ explicit observation encoding
+ canonical event order
→ advisory capability estimate
```

The estimate remains distinct from mastery, credential eligibility, and authorization.

## Input authority

`project_bkt` requires each input to contain both:

1. a `Complete` `AttemptEvidence`; and
2. an `EvidenceAdmissionDecision` whose event ID and named/versioned profile exactly match the projection.

Rejected or inconclusive evidence is not silently filtered: supplying it to the admitted-evidence projection is an error.

Legacy-incomplete `record_attempt` observations cannot enter this estimator merely because some older mutable `SkillMastery` value exists.

## Dimension-specific projection

A BKT projection targets exactly one `(learner, capability, dimension)` tuple. Every attempt must match that tuple.

This prevents a correct recall item, for example, from silently becoming evidence for transfer or practical performance.

## Explicit scored-outcome encoding

Classical BKT consumes binary observations. Praxis therefore never silently thresholds a scored assessment.

Two policies exist:

- `BinaryOnly`: scored observations are rejected;
- `ScoredThreshold { correct_at_or_above_permille }`: the threshold is explicit and becomes part of the estimator parameter digest.

## Canonical ordering

BKT is sequence-dependent, so input vector order is not authoritative. Praxis sorts admitted observations by:

```text
(observed_at, event_id)
```

The receipt records the exact ordered event IDs consumed by the estimator. Equal timestamps therefore have a deterministic stable tie-breaker.

Duplicate event IDs are rejected.

## Integer determinism

`integer-v1` performs BKT probability arithmetic in permille integers with `u64` intermediates and round-half-up normalization. It does not use floating-point arithmetic.

This is intended to make the same source evidence, configuration, and ordering produce the same projection across native and WASM execution.

The algorithm version is frozen as:

```text
estimator_id      = praxis:bkt-projection
estimator_version = integer-v1
```

Regression vectors freeze representative correct/incorrect transitions.

## Provenance

Every projection binds:

- learner;
- capability;
- capability dimension;
- admission profile ID/version;
- canonical ordered evidence-event IDs;
- estimator ID/version;
- BKT parameters;
- scored-observation encoding policy;
- BLAKE3 parameter digest;
- generation time.

The support-confidence value remains an explicit estimator heuristic (`support_confidence_step_permille`), not a claim of calibrated statistical certainty.

## Non-authority rule

Neither `BktProjectionReceipt` nor the underlying `AdvisoryCapabilityEstimate` can grant credential authority.

In particular:

```text
estimate >= 800
!= mastered
!= passed
!= credential eligible
!= authorized
```

Any consequential threshold belongs to a later named/versioned policy boundary that records the evidence and estimator it consumed.

## Compatibility

This tranche does not mutate `SkillMastery`, the adaptive integrity zome, or the DNA. The historical BKT path remains present for compatibility until a later migration derives adaptive state from evidence receipts.

No historical mutable mastery value is reinterpreted as if it had complete evidence provenance.

## Follow-up

A later adaptive-state migration should:

1. consume these receipts as advisory projections;
2. stop treating mutable BKT state as source truth;
3. preserve old state as explicitly legacy-derived where needed;
4. recompute learner summaries when evidence admission profiles or estimator versions change.
