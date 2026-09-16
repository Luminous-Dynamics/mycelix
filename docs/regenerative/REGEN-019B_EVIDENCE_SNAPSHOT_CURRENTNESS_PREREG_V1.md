# REGEN-019B — Evidence Snapshot / Currentness Preregistration v1

Status: preregistration only. This document supersedes the accidentally colliding REGEN-021 draft in #1239. It defines evidence-snapshot and purpose-specific currentness semantics. It establishes no universal freshness window, trusted-time infrastructure, scientific truth, recommendation, governance authority, or physical-action authority.

## Purpose

PEF correctly permits `TemporalExtent::Unspecified`, and qualified REGEN-010B deliberately does not invent a generic soil freshness rule. Downstream assessments still need a reproducible answer to a different question:

> Which exact evidence set was considered, and was each required item current enough under the exact policy/profile used for that decision?

REGEN-019B freezes that boundary while preserving the original program numbering in which REGEN-021 remains reserved for crop/nutritional-service semantics.

Core theorems:

```text
valid evidence
!= current-enough evidence
```

and:

```text
current enough for purpose A
!= current enough for purpose B
```

## Evidence snapshot

A downstream assessment should bind an exact evidence snapshot rather than a live query whose membership can change later. A future snapshot may bind a snapshot reference, evidence references, capture time, source-revision references, and policy-input references. Exact field names are not frozen here.

```text
snapshot complete enough to replay inputs
!= evidence true
!= scientifically sufficient evidence
```

If the evidence set changes, create a new snapshot/evaluation rather than silently mutating the provenance of the old assessment.

## Time concepts remain distinct

PEF temporal support remains authoritative:

```text
Unspecified
Instant(t)
Interval { start, end }
```

REGEN-019B must not rewrite those fields or substitute other timestamps.

```text
TemporalExtent::Unspecified
!= ingestion time
!= retrieval time
!= current time
```

Observation/event time, specimen collection time, analysis completion time, lineage-step time, retrieval time, ingestion time, snapshot time, and evaluation time answer different questions and must not collapse.

## Currentness is policy-specific

No observation has one universal `fresh=true` property. Acceptable temporal relation depends on the consuming purpose and evidence role. Soil moisture, soil pH, contaminant results, biomass moisture, custody/seal state, forecasts, and trial endpoints may legitimately use different rules.

A currentness result should bind an exact policy/profile revision. A logical policy name without immutable revision/content identity is a weaker proposition.

Possible rule concepts include required temporal form, maximum age, interval overlap, forecast validity, and clock-assurance requirements. This contract prescribes no values.

## Clock trust and uncertainty

```text
timestamp present
!= trusted time
```

A local wall clock, signed institutional time, monotonic counter, GNSS-derived time, or other source may have different assurance. Where clock uncertainty crosses a decision threshold, the result should fail closed or become indeterminate rather than pretending the comparison is exact.

## Instant, interval, forecast, and scenario semantics

For `Instant(t)`, threshold boundaries and apparently future timestamps require explicit policy behavior.

For intervals, end-age, any-overlap, full-overlap, minimum-duration, and treatment-window relations are distinct propositions.

A recently generated forecast is not necessarily valid for the target decision time. Forecast generation lineage, validity support, target window, and model/product revision may all matter.

`EvidenceClass::Scenario` remains hypothetical evidence and does not become observed-current fact through a freshness evaluation.

## Computed evidence

Fresh computation does not automatically make stale inputs current:

```text
computed now
!= source evidence now
```

For Derived/Inferred/Forecast products, a consuming policy may require lineage-aware temporal conditions over roots/inputs and model/configuration revisions.

This composes naturally with REGEN-019 shared admission but remains a separate proposition:

```text
REGEN-019 admission
!= REGEN-019B currentness
```

## Specimen and material state

A recently completed laboratory analysis of an old specimen may be appropriate for one property and inappropriate for another.

```text
analysis completed recently
!= specimen represents current material state
```

Stable lot identity also does not imply unchanged current moisture/composition/condition. Currentness should bind the exact material-state observation or snapshot used.

## State taxonomy

A future result taxonomy should preserve at least:

```text
missing
stale
unknown-time
invalid
current
indeterminate
```

Those states are not interchangeable.

A future evaluation may bind snapshot reference, policy revision, evaluation time, clock-evidence reference, per-role results, overall result, and stable reason codes.

## Spatial compatibility

REGEN-019A spatial/sampling-frame revisions remain separate from temporal freshness. A recent observation evaluated against an obsolete geometry or sampling-frame revision may still be inappropriate for the intended proposition.

## No hidden re-resolution

After a snapshot is fixed, evaluation operates over the bound evidence set. Later evidence creates snapshot B / assessment B rather than rewriting snapshot A / assessment A.

## Initial implementation boundary

A later dependency-light implementation may live under the shared regenerative evidence crate as `snapshot` / `currentness` modules or a small sibling after dependency review. It should depend on qualified PEF/shared-admission primitives and minimal policy/reference types, not Symthaea, Holochain networking, Marketplace, Finance, Climate authority, or physical-control runtimes.

## Qualification target

A later ProductFrozen campaign should exercise at least:

1. `TemporalExtent::Unspecified` cannot pass a known-time rule by ingestion-time substitution;
2. observation/retrieval/ingestion/evaluation times remain distinct;
3. currentness differs by purpose/policy revision;
4. stale != missing != invalid != indeterminate;
5. instant threshold semantics are deterministic;
6. interval overlap semantics are explicit;
7. clock uncertainty can prevent definite PASS;
8. forecast validity remains distinct from generation recency;
9. Scenario remains Scenario;
10. fresh computed output does not automatically refresh stale roots;
11. analysis completion does not replace specimen/material-state time;
12. snapshot membership is immutable for one evaluation;
13. new evidence creates a new snapshot/evaluation;
14. bounded policy/snapshot references revalidate on deserialization;
15. no recommendation/authority field exists.

## Deliberate non-claims

REGEN-019B establishes no universal freshness threshold, trusted global clock, truth of any observation, scientific sufficiency, sampling representativeness, specimen validity, material safety, agronomic suitability, treatment efficacy, climate/carbon claim, legal/governance authority, recommendation correctness, process execution authority, or physical actuation.

Its proposition is deliberately narrow: make the exact evidence set, temporal policy, clock assumptions, and currentness result explicit enough that downstream regenerative assessments can be reviewed and replayed without replacing unknown time, stale evidence, or live-query drift with hidden defaults.
