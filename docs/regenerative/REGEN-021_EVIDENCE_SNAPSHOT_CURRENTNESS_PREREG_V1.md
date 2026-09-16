# REGEN-021 — Evidence Snapshot / Currentness Preregistration v1

Status: preregistration only. This document defines evidence-snapshot and purpose-specific currentness semantics. It establishes no universal freshness window, trusted-time infrastructure, scientific truth, recommendation, governance authority, or physical-action authority.

## 1. Purpose

PEF correctly permits `TemporalExtent::Unspecified` and REGEN-010 deliberately does not invent a generic soil freshness rule. That is necessary, but downstream decisions still need a reproducible answer to a different question:

> which exact evidence set was considered, and was each required item current enough under the exact policy/profile used for that decision?

REGEN-021 freezes that boundary.

Core theorem:

```text
valid evidence
!= current-enough evidence
```

and:

```text
current enough for purpose A
!= current enough for purpose B
```

## 2. Currentness is contextual

No observation has one universal `fresh=true` property.

Acceptable age/temporal relation depends on the consuming purpose and evidence role.

Examples may differ for:

- soil pH baseline;
- recent soil moisture;
- contaminant laboratory result;
- biomass moisture state;
- custody/seal status;
- weather/forecast input;
- field-trial endpoint;
- agronomic suitability assessment.

The generic contract therefore records policy and result; it does not hard-code one threshold.

## 3. Evidence snapshot

A downstream assessment SHOULD be able to bind an exact evidence snapshot rather than a live query whose membership can change later.

Conceptually:

```text
EvidenceSnapshot {
    snapshot_ref,
    evidence_refs,
    captured_at?,
    source_revision_refs?,
    policy_input_refs?
}
```

Exact naming is not frozen here.

The snapshot records the evidence identities used by the assessment. It does not prove those identities are content-addressed unless the owning evidence systems provide immutable/content commitments.

## 4. Snapshot != truth

```text
snapshot complete enough to replay inputs
!= evidence true
!= evidence scientifically sufficient
```

A snapshot is an input-set identity/provenance mechanism.

It does not upgrade evidence class, source reliability, representativeness, laboratory competence, or model validity.

## 5. Live query != reproducible decision input

A decision derived from:

```text
all current observations for plot X
```

is not reproducible if the query later returns a different set and no exact snapshot/revision was retained.

REGEN-021 therefore prefers:

```text
query / resolution
-> exact evidence snapshot
-> currentness evaluation
-> downstream assessment
```

rather than treating the live query itself as the durable evidence subject.

## 6. Observation time remains PEF-owned

PEF temporal support remains authoritative:

```text
Unspecified
Instant(t)
Interval { start, end }
```

REGEN-021 MUST NOT rewrite those fields.

```text
TemporalExtent::Unspecified
!= ingestion time
!= retrieval time
!= current time
```

If a consuming policy requires known observation time, `Unspecified` cannot satisfy that rule merely because the record was recently received.

## 7. Multiple time concepts must not collapse

A system may know several distinct times:

- event/observation time;
- interval of physical support;
- specimen collection time;
- analysis completion time;
- lineage-step completion time;
- external artifact retrieval time;
- record ingestion time;
- snapshot capture time;
- currentness evaluation time.

These timestamps answer different questions.

REGEN-021 MUST NOT silently substitute one for another.

## 8. Currentness policy reference

A currentness result SHOULD bind an exact policy/profile revision.

Conceptually:

```text
CurrentnessPolicyRef {
    policy_ref,
    revision_ref,
    content_digest?
}
```

A logical policy name without immutable content identity is a weaker proposition than an exact policy revision/digest.

## 9. Role-specific rule

A policy may define a rule for an evidence role conceptually such as:

```text
CurrentnessRule {
    evidence_role,
    required_temporal_form?,
    maximum_age?,
    required_overlap?,
    future_validity_rule?,
    clock_requirement?,
}
```

Exact rule types should be reviewed before implementation.

The generic contract does not prescribe values.

## 10. Evaluation time identity

A currentness evaluation is relative to some declared evaluation time.

```text
age = relation(observation temporal support, evaluation time)
```

But an arbitrary wall-clock value is not automatically trusted.

The record SHOULD distinguish the evaluation timestamp from the trust/clock source used to justify it when that matters.

## 11. Clock trust is separate

```text
timestamp present
!= trusted time
```

A local application clock, signed institutional timestamp, monotonic device counter, GNSS clock, or other time source may have different assurance.

REGEN-021 v1 does not establish a universal trusted-time infrastructure.

A policy may require a stronger clock/source profile for high-consequence decisions.

## 12. Clock uncertainty

Where clock uncertainty is material, currentness evaluation SHOULD be able to fail closed or return indeterminate rather than pretending the comparison is exact.

Conceptually:

```text
age near threshold
+ clock uncertainty crossing threshold
=> not a definite PASS
```

No universal tolerance is introduced.

## 13. Instant evidence

For an `Instant(t)` observation, a policy may evaluate age relative to evaluation time.

It MUST define behavior for:

- event in the past;
- timestamp apparently in the future;
- clock uncertainty;
- exact threshold boundary.

REGEN-021 does not assume future timestamps are valid merely because they parse.

## 14. Interval evidence

For `Interval { start, end }`, currentness semantics depend on purpose.

Possible policies may care about:

- interval end age;
- full interval overlap with a decision window;
- any overlap;
- minimum duration;
- relation to treatment/intervention time.

These are distinct propositions and MUST NOT collapse into one generic age calculation.

## 15. Forecasts

A forecast is not made current merely because it was generated recently.

The consuming profile may need to bind:

- forecast issue/generation lineage;
- forecast validity temporal support;
- target decision time/window;
- model/product revision.

```text
recently generated forecast
!= forecast valid for current target time
```

## 16. Scenarios

`EvidenceClass::Scenario` is hypothetical/counterfactual/planning evidence.

A scenario does not become an observed-current fact through a freshness evaluation.

Currentness MUST preserve evidence class.

## 17. Reported vs observed

`Reported` and `Observed` remain distinct PEF raw classes.

Neither class receives a universal freshness preference from REGEN-021.

A consuming policy may require one or the other explicitly.

```text
raw class
!= freshness ranking
```

## 18. Computed evidence currentness

For Derived/Inferred/Forecast products, a consuming policy may need to consider not only output time but relevant input/root times and model/configuration revision.

A fresh computation from stale inputs is not automatically fresh evidence for every purpose.

```text
computed now
!= source evidence now
```

REGEN-021 SHOULD allow the policy to require lineage-aware temporal conditions rather than inspecting only the output timestamp.

## 19. Specimen / analysis timing

REGEN-018 distinguishes specimen collection, custody, preparation, and analytical-result lineage.

A recently completed laboratory analysis of an old specimen may be scientifically appropriate for one property and inappropriate for another.

Therefore:

```text
analysis completed recently
!= specimen represents current material state
```

The policy must choose the relevant temporal proposition.

## 20. Material-state snapshots

REGEN-011 and later material contracts distinguish stable lot identity from changing material state.

A currentness evaluation SHOULD bind the exact material-state observation/snapshot used.

```text
same BiomassLotId
!= unchanged moisture/composition/current condition
```

## 21. Corrections and invalidations

Freshness/currentness does not override invalidation or correction evidence.

A current record that is known invalid is not acceptable merely because it is recent.

Likewise, an older superseded assertion may remain part of history while a newer correction changes downstream admissibility.

Currentness and validity are orthogonal.

## 22. Snapshot membership

A snapshot SHOULD preserve:

- all evidence intentionally included;
- explicit required-but-missing roles where the consuming contract needs them;
- evidence excluded by declared policy where material;
- exact policy/profile identities used to select/evaluate evidence where practical.

A system MUST NOT silently omit adverse or inconvenient evidence merely to obtain a currentness PASS.

## 23. Missing evidence != stale evidence

These states are distinct:

```text
missing
stale
unknown-time
invalid
current
indeterminate
```

A future error/result taxonomy SHOULD retain those distinctions.

## 24. Suggested evaluation result

A future result may be approximately:

```text
CurrentnessEvaluation {
    snapshot_ref,
    policy_revision_ref,
    evaluated_at,
    clock_evidence_ref?,
    per_role_results,
    overall_result
}
```

Possible outcomes might include `Pass`, `Fail`, and `Indeterminate`, with stable reason codes.

No result grants authority by itself.

## 25. No hidden re-resolution

After a snapshot is fixed, evaluation SHOULD operate over the bound evidence set rather than silently resolving newer records mid-evaluation.

If the evidence set changes, create a new snapshot/evaluation.

This gives:

```text
snapshot A -> assessment A
snapshot B -> assessment B
```

rather than mutating the provenance of assessment A.

## 26. Suitability integration

REGEN-017 already defines suitability relative to an evidence snapshot/profile.

REGEN-021 supplies the missing explicit currentness semantics:

```text
exact material/site context
+ evidence snapshot
+ currentness policy evaluation
+ suitability profile
-> contextual assessment
```

A favorable historical suitability result does not automatically remain current after evidence or material/site state changes.

## 27. Contamination integration

A contaminant result may have policy-specific validity/currentness depending on material identity, storage/processing events, specimen lineage, and adopted quality profile.

REGEN-021 does not invent one universal laboratory-result expiration period.

## 28. Trial integration

REGEN-015 endpoint windows should bind exact temporal rules.

A measurement taken outside a preregistered endpoint window should not be made conforming merely because it is recent at analysis time.

## 29. Spatial integration

REGEN-020 spatial/sampling-frame revisions may also affect currentness.

A recent observation evaluated against an obsolete plot geometry/sampling-frame revision may be inappropriate for a current spatial proposition.

Temporal freshness does not replace revision compatibility.

## 30. Shared admission integration

REGEN-019 admits structurally/provenance-valid PEF evidence.

REGEN-021 evaluates a separate proposition:

```text
REGEN-019 admission
!= REGEN-021 currentness
```

Keeping these separate prevents evidence validation code from accumulating purpose-specific time policies.

## 31. No recommendation or action authority

A currentness PASS means only that the supplied snapshot meets the exact temporal policy conditions evaluated.

```text
current-enough evidence
!= correct recommendation
!= permission to intervene
!= actuator authority
```

Recommendation, governance, legal rights, and physical control remain separate.

## 32. Proposed implementation boundary

A later dependency-light implementation may live in a narrow module/crate such as:

```text
mycelix-regenerative-evidence::snapshot
mycelix-regenerative-evidence::currentness
```

or an equally small sibling crate after dependency review.

It should depend on qualified PEF/shared admission types and minimal policy/reference primitives, not Symthaea, Holochain networking, Marketplace, Finance, Climate authority, or physical-control runtimes.

## 33. Qualification target

A future campaign SHOULD test at least:

1. `TemporalExtent::Unspecified` cannot satisfy a known-time rule by ingestion-time substitution;
2. distinct observation/retrieval/ingestion/evaluation times remain distinct;
3. currentness differs by policy revision/purpose;
4. stale != missing != invalid != indeterminate;
5. instant threshold boundary semantics are deterministic;
6. interval overlap semantics are explicit;
7. future/clock-anomalous timestamps fail or become indeterminate per policy;
8. clock uncertainty can prevent definite PASS;
9. forecast validity window remains distinct from generation recency;
10. Scenario remains Scenario;
11. fresh computed output does not automatically make stale lineage roots current;
12. analysis completion time does not replace specimen collection/material-state time;
13. snapshot membership is immutable for the evaluation;
14. new evidence produces a new snapshot/evaluation rather than rewriting the old one;
15. serialization revalidates policy/snapshot reference bounds;
16. no recommendation/authority field exists in the core result.

Use REGEN-008 ProductFrozen dependency semantics and REGEN-Q001 machine-readable qualification receipts where composed cleanly.

## 34. Deliberate non-claims

REGEN-021 establishes no:

- universal freshness threshold;
- trusted global clock;
- truth of any observation;
- scientific sufficiency;
- sampling representativeness;
- specimen validity;
- material safety;
- agronomic suitability;
- treatment efficacy;
- climate/carbon claim;
- legal/governance authority;
- recommendation correctness;
- process execution authority;
- physical actuation.

Its proposition is deliberately narrow:

> make the exact evidence set, temporal policy, evaluation time/clock assumptions, and currentness result explicit enough that downstream regenerative decisions can be reviewed/replayed without replacing unknown time, stale evidence, or live-query drift with hidden defaults.
