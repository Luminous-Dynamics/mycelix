# AC-011 — Procurement Robustness Matrix Builder

Status: implementation candidate; execution qualification not yet established.

## Purpose

AC-011 is the first domain-specific consumer of AC-010.

It builds a procurement supplier-concentration robustness matrix from real AC-003 edges while preserving the exact assumptions and lineage behind every scenario.

The first supported perturbation axes are:

- raw vs AC-008-qualified identity-resolved supplier identity;
- all valid assertions vs AC-003 `Corroborated`-only assertions;
- explicit `recorded_at` time windows.

The builder enumerates the complete Cartesian product of the selected axis options, including joint specifications.

## One trust snapshot

Identity qualification is deliberately performed once per matrix at one `evaluated_at` time.

AC-011 calls AC-008's public `build_equivalence_view` boundary exactly once, using the supplied qualification inputs, policy reference and injected verifier. The resulting `QualifiedEquivalenceView` is then reused across every procurement scenario.

This avoids a matrix in which verifier/revocation state changes between scenario calculations.

The matrix records the qualification receipt references that formed that frozen identity trust snapshot.

## Baseline

The baseline always means:

- raw source supplier identities;
- all valid AC-003 assertions;
- no additional recorded-at window.

At least one perturbation axis is required. A baseline-only plan fails closed rather than being called a robustness analysis.

## Identity axis

When an identity assumption and qualified AC-008 inputs are supplied, the matrix contains both:

- raw supplier identity;
- qualified identity-resolved supplier identity.

For each scenario, only equivalence components containing at least two suppliers present in that scenario's current award population are applied.

Projected edges are clones. AC-003 source edges are never mutated.

Identity resolution preserves the AC-007 rule:

`correction changes the analytical view, not source history`.

## Evidence axis

When a corroborated-only assumption is supplied, each compatible specification is evaluated both with:

- every graph-valid assertion;
- only assertions whose AC-003 status is `Corroborated`.

This does not claim that `Declared` evidence is false. It measures how sensitive concentration is to a stricter evidence-admission policy.

## Time-window axis

Each `ProcurementTimeWindow` provides:

- an assumption descriptor;
- inclusive `start_recorded_at`;
- exclusive `end_recorded_at`.

The baseline has no additional time restriction. Each window creates an alternative `TimeWindow` assumption and is crossed with the selected identity/evidence options.

The initial implementation bounds the matrix to at most 12 non-baseline time windows to prevent unbounded Cartesian expansion.

The window applies to AC-003 `recorded_at`, not `valid_from`/`valid_until`, award date, contract period or inferred economic period. Those require future explicit semantics rather than silent substitution.

## Assumption evidence

Every perturbation descriptor must contain:

- stable assumption ID;
- human-readable statement;
- admissibility reference;
- non-empty digest-bearing provenance.

Assumption IDs must be unique across all selected axes.

## Scenario enumeration

If all three initial axes are enabled with `N` time windows, the number of scenarios is:

`2 identity options * 2 evidence options * (1 + N time options)`.

For example, one time window produces eight scenarios:

1. raw / all evidence / all time — baseline;
2. resolved / all evidence / all time;
3. raw / corroborated-only / all time;
4. resolved / corroborated-only / all time;
5. raw / all evidence / selected window;
6. resolved / all evidence / selected window;
7. raw / corroborated-only / selected window;
8. resolved / corroborated-only / selected window.

AC-011 labels alternatives as `SingleDimension` or `Joint` according to the number of dimensions actually changed from baseline.

## Metric semantics

Every scenario uses AC-004 procurement supplier HHI:

`sum(award_count_i^2) / total_awards^2`.

Each award relationship is weighted equally. AC-011 does **not** yet introduce value weighting.

That is intentional: value weighting needs its own source/evidence contract and method identity before it can become a defensible AC-010 method-choice/procurement-weighting axis.

## Scenario lineage

`ProcurementScenarioLineage` records for every scenario:

- scenario reference;
- exact AC-003 input edge references used by AC-004;
- exact active equivalence-component references;
- exact AC-006 identity-link references applied.

This allows an auditor to reconstruct why two specifications produced different concentration values.

## Robustness envelope

After scenario calculation, AC-011 delegates comparison and bounds to AC-010.

Therefore the matrix inherits AC-010's guarantees around:

- one baseline;
- explicit assumptions;
- subject/metric/unit comparability;
- exact rational ordering;
- exact extrema;
- duplicate-specification rejection;
- explicit exploratory vs declared-scope coverage.

AC-011 does not automatically claim exhaustive defensible specifications merely because it fully enumerates the selected axes. The supplied AC-010 `RobustnessCoverage` remains authoritative about the coverage claim.

## Failure semantics

AC-011 fails closed for, among other cases:

- empty matrix ID;
- non-procurement subject;
- empty/invalid limitation set;
- no perturbation axis;
- more than 12 time windows;
- invalid or reversed time window;
- malformed assumption descriptors;
- duplicate assumption IDs;
- identity descriptor/input mismatch;
- missing identity policy reference;
- AC-008 qualification/verifier failure;
- any scenario with no usable award population;
- AC-004 metric failure;
- AC-010 envelope validation failure.

A scenario that becomes empty under a strict evidence/window specification is not silently dropped. The matrix fails so the analyst must address whether that specification is meaningful.

## Non-goals

AC-011 does not yet:

- weight awards by contract value;
- infer missing registry entities;
- impute missing bids or awards;
- interpret `recorded_at` as award date;
- create legal findings;
- label a supplier or official as corrupt;
- choose one specification as the uniquely correct model;
- claim its selected axes exhaust every defensible procurement model.

## Validation authored

Current tests cover:

- full Cartesian enumeration of identity/evidence/window axes;
- one-time qualification receipt lineage;
- invalid-window rejection before qualification;
- baseline-only plans being rejected as non-robustness analyses.

## Qualification gate

Before AC-011 is qualified:

1. exact-subject Civic workspace tests pass;
2. rustfmt passes;
3. warnings-denied Clippy passes;
4. verifier call-count tests prove each identity receipt is verified once per matrix, not once per scenario;
5. mutation tests prove invalid windows/assumption IDs fail before scenario execution;
6. property tests reproduce the expected Cartesian scenario count for every supported axis combination;
7. source-edge immutability tests prove matrix construction never mutates AC-003 inputs;
8. identity lineage tests reproduce active components/link refs independently for each filtered scenario;
9. scenario metric arithmetic is independently reproduced from exact award counts;
10. strict evidence/window scenarios that empty the award population fail rather than disappearing;
11. scenario order and assumption input order do not alter canonical envelope contents;
12. a real OCDS/BODS-derived fixture traverses AC-005 -> AC-006 -> AC-008 -> AC-011 -> AC-010 end to end.

## Next tranche

AC-012 should add a qualified **procurement value-weighting contract** rather than stuffing monetary values into AC-011 informally.

That contract should define currency identity, amount scale, award/contract-value semantics, amendments, missing-value handling, exchange-rate provenance when currencies differ, and exact arithmetic. Only after that boundary exists should AC-011 add `ProcurementWeighting` as a real robustness axis.
