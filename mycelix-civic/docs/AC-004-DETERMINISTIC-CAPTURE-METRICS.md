# AC-004 — Deterministic Capture Metrics

Status: draft implementation contract

Parent: AC-003 institutional relationship graph

## Purpose

AC-004 turns validated AC-003 graph relationships into AC-002 observations using deterministic, replayable calculations. It deliberately stops at observation production.

It does not create findings, accusations, sanctions, or rights-affecting decisions.

## First metrics

AC-004 v1 implements three narrow metrics:

1. **Procurement supplier concentration** — Herfindahl-Hirschman concentration over award relationships, weighting each award edge equally.
2. **Authority-grant concentration** — Herfindahl-Hirschman concentration over authority edges, weighting each grant equally by holder.
3. **Evidence deficit share** — share of graph assertions not in AC-003's `Corroborated` state.

These definitions are intentionally modest and explicit. They should not be interpreted as complete models of market power, political power, or evidentiary quality.

## Exact arithmetic

Concentration uses the exact rational form:

`sum(count_i^2) / total^2`

No floating-point arithmetic is used in the authoritative calculation.

For example, two suppliers with two awards each produce:

`(2^2 + 2^2) / 4^2 = 8 / 16`

The ratio may be simplified by consumers for display, but the evidence record preserves the exact numerator and denominator produced by the metric implementation.

## Input qualification

Every graph edge that enters a metric population is first validated with the AC-003 `InstitutionalGraphContract`.

An invalid edge fails the metric calculation with its original population index and graph-contract violations. Invalid input is never silently skipped.

An empty population is an error, not a zero measurement.

## Derivation lineage

AC-004 returns `DerivedCaptureObservation`:

- the AC-002 `CaptureObservation`; and
- the exact sorted set of AC-003 input edge references used to produce it.

This prevents a metric from becoming an unexplained number whose source population cannot be reconstructed.

External source provenance from the input edges is also deduplicated and propagated into the AC-002 observation.

## Uncertainty discipline

Every metric appends a metric-specific limitation to caller-supplied uncertainty metadata.

Examples:

- supplier HHI states that award edges are equally weighted and contract value/complexity are not represented;
- authority HHI states that each grant is equally weighted and does not model mandate scope, budget, or practical importance;
- evidence deficit states that AC-003 corroboration means at least two distinct provenance sources and that an authoritative single-source record still counts as uncorroborated.

These limitations are part of the observation, not optional documentation.

## Constitutional boundary

All AC-004 outputs remain AC-002 observations and therefore map to:

`ConsequenceBasis::Observation`

They cannot become `AdjudicatedFinding` inside AC-004.

The permitted chain remains:

`validated graph -> metric -> observation -> hypothesis/signal -> human review`

not:

`metric threshold -> automatic consequence`

## Non-goals

AC-004 v1 does not:

- establish a corruption threshold;
- infer collusion;
- infer guilt;
- weight contracts by monetary value;
- weight public authority by substantive scope or budget;
- calculate personal risk or trust scores;
- create rankings of citizens;
- automatically generate sanctions;
- claim an evidence deficit means an assertion is false.

## Qualification gate

Before AC-004 is qualified infrastructure:

1. exact-subject `cargo test -p civic-types` passes;
2. warnings-denied Clippy passes;
3. mutation tests prove invalid AC-003 edges cannot enter metric populations;
4. golden vectors freeze HHI and evidence-deficit arithmetic;
5. independent calculation reproduces the exact rational results;
6. review confirms every output remains `ConsequenceBasis::Observation`;
7. procurement experts review whether equal-award weighting is appropriate for the intended pilot before the metric is used operationally.

## Next tranche

AC-005 should add standards adapters for OCDS procurement records and BODS-compatible beneficial-ownership identifiers, producing AC-003 edges without creating a second proprietary source-of-truth format.
