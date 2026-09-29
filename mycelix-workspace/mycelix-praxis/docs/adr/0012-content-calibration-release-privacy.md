# ADR-0012: Content Calibration Provenance and Release Privacy

- Status: Proposed
- Subject: `PRAX-STATE-001F`
- Scope: DNA-neutral content-difficulty calibration contracts

## Context

The legacy adaptive `DifficultyCalibration` is public and contains aggregate learner-performance information:

- total attempts;
- successful attempts;
- average completion time;
- completion-time standard deviation;
- calibrated difficulty/variance;
- discrimination estimate.

The object does not identify the exact source aggregate/evidence lineage, analyzer version, cohort release threshold, or privacy mechanism.

For large cohorts, public calibration can be useful. For small cohorts, publishing detailed aggregate behavior can become learner telemetry by another route.

## Decision

Praxis separates:

```text
PrivateCalibrationAggregate
        |
        v
ContentDifficultyCalibrationProjection
        |
        v
ContentDifficultyCalibrationDisclosure
```

### PrivateCalibrationAggregate

The private aggregate may retain exact `EvidenceEventId` inputs, attempt counts, distinct learner count, success count, and completion-time totals.

It is explicitly **not** a shareable calibration merely because the fields are aggregated.

### ContentDifficultyCalibrationProjection

The private projection binds:

- exact content identity/kind;
- exact private aggregate ID/content/digest;
- exact aggregate cohort/attempt counts;
- analyzer ID/version/parameter digest;
- calibrated difficulty, variance, completion-time and discrimination estimates;
- generation and expiry.

It remains a model projection rather than learning evidence, credential authority, trust authority, or runtime authorization.

### ContentDifficultyCalibrationDisclosure

Shareable release is a separate minimized object. It contains no learner IDs and no event IDs.

Release requires a named/versioned `CalibrationReleasePolicy` and a `CalibrationReleaseAdmission` bound to the exact aggregate digest.

The admission exposes an admitted lower bound for cohort size instead of the exact private learner count.

## Release mechanisms

Two mechanism classes are intentionally distinct:

### ThresholdedAggregate

A cohort-threshold release requires at least the policy's admitted minimum number of distinct learners.

This is **not** labeled differential privacy and must not be represented as providing DP guarantees.

### DifferentialPrivacy

A DP release additionally freezes:

- mechanism ID/version;
- epsilon encoded in millionths;
- delta encoded in parts per billion;
- exact noise-parameter digest.

This contract records parameters; it does not independently prove that an implementation sampled noise correctly. Runtime qualification must test the actual release implementation and its deterministic/non-deterministic boundaries as appropriate.

## Privacy theorem

```text
aggregated learner data
!= automatically public-safe data
```

and:

```text
private aggregate persistence
!= release admission
```

No future default learner/content link should expose exact private calibration event lineage on the DHT.

## Authority boundary

Neither private nor disclosed calibration grants:

- source learning-evidence authority;
- mastery/capability authority;
- credential authority;
- general trust authority;
- runtime authorization.

Calibration may inform recommendation/planning models under explicit policies.

## Legacy migration

Do not synthesize exact event lineage or privacy admission for historical public `DifficultyCalibration` records.

Historical values may remain compatibility data, but a new privacy-qualified disclosure must be recomputed from provenance-complete private aggregates when available.

## DNA boundary

This ADR changes no Holochain visibility or link type. Later materialization should:

- keep exact private aggregates and full calibration projections private/local;
- publish/share only minimized release objects that have passed explicit release policy;
- validate release-policy/profile identifiers and aggregate commitments;
- avoid learner-index links for private aggregate data;
- qualify small-cohort rejection and disclosure behavior adversarially.
