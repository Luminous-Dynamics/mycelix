# AC-014 — Full Procurement Robustness Composition

## Status

Draft product tranche stacked directly on AC-013. Execution qualification is not established until exact-subject CI or equivalent independent execution runs against the frozen head.

## Purpose

AC-014 composes the procurement robustness axes already qualified separately:

- raw vs AC-008-qualified resolved supplier identity;
- all valid vs AC-003-corroborated-only evidence;
- baseline vs explicit `recorded_at` windows;
- AC-004 equal-award-count vs AC-012 qualified award-value weighting.

It produces one AC-010 robustness envelope over the complete Cartesian product of the selected options.

## Governing rule

`one frozen identity trust snapshot + explicit evidence/time/weighting assumptions + exact edge/value lineage -> robustness matrix`

not:

`many analytical variants -> universal truth`.

## Baseline

The baseline is always:

- raw supplier identities;
- all graph-valid assertions;
- no additional time window;
- AC-004 equal-award-count supplier HHI.

It carries no assumption refs.

## Weighting semantics

A value-weighted scenario must carry two assumptions:

1. `ProcurementWeighting` — the substantive choice to weight suppliers by qualified award value;
2. `MethodChoice` — the explicit change from AC-004 count HHI to AC-012 value HHI.

Therefore value weighting is never hidden as a same-method perturbation.

## Exact population binding

For every value-weighted scenario AC-014:

1. filters the AC-003 graph according to evidence and time assumptions;
2. applies the frozen identity view to cloned edges when requested;
3. selects only AC-012 value records whose `award_edge_ref` is in that exact filtered award population;
4. recomputes AC-004 count HHI over the same projected edges;
5. computes AC-012 value HHI;
6. requires the count and value observations to have identical `input_edge_refs`.

If the edge populations differ, the scenario fails with `InputEdgeLineageMismatch`.

This prevents a claimed weighting sensitivity from silently changing the sample.

## Monetary subset semantics

AC-012 still owns monetary validity. AC-014 does not introduce FX, rounding, imputation, negative/concession weighting, joint-total allocation, or currency conversion.

The full supplied value set is preflighted once against the unfiltered award population before scenario enumeration. Each filtered value scenario then receives the exact matching subset of those already-qualified records.

## Identity trust snapshot

When identity reconciliation is enabled, AC-014 calls AC-008 `build_equivalence_view` once at `evaluated_at` and reuses that view for the entire matrix.

Verifier/revocation state therefore cannot drift between scenarios in one robustness envelope.

## Scenario lineage

Every scenario records:

- scenario ID;
- weighting mode;
- exact award-edge refs;
- active equivalence-component refs;
- applied AC-006 identity-link refs;
- value snapshot ref when applicable;
- exact AC-012 value-record refs when applicable;
- exact metric method ref.

Source graph edges are never mutated.

## Scenario bounds

AC-014 refuses matrices larger than 128 scenarios.

With `I` identity options, `E` evidence options, `T` time options including baseline, and `W` weighting options:

`scenario_count = I * E * T * W`.

The current AC-011 maximum of 12 non-baseline windows keeps the fully enabled matrix at 104 scenarios (`2 * 2 * 13 * 2`).

## Coverage semantics

The Cartesian product is complete only for the options explicitly selected by the plan.

AC-014 does not upgrade AC-010 coverage automatically. The caller must still choose either:

- `Exploratory`, or
- `EnumeratedWithinDeclaredScope` with evidence-bearing scope provenance.

## Epistemic boundary

AC-014 does not produce:

- a corruption score;
- a guilt probability;
- a finding of wrongdoing;
- a recommendation to sanction a person or organization;
- a claim that one weighting scheme is normatively correct.

It answers how supplier-concentration observations change across declared, auditable specifications.

## Authored tests

The tranche includes tests for:

- Cartesian enumeration of evidence × time × weighting;
- exact monetary-record subset lineage after filtering;
- one-time identity verification across the full matrix;
- explicit failure when weighting is enabled without a value set.

## Qualification gate

Before AC-014 is treated as qualified:

1. Civic workspace tests pass on the exact subject;
2. rustfmt passes;
3. warnings-denied Clippy passes;
4. full four-axis fixtures reproduce expected Cartesian scenario counts;
5. verifier call-count fixtures prove one identity trust snapshot per matrix;
6. independent count/value calculations reproduce every scenario ratio;
7. every value scenario proves count/value edge-lineage equality;
8. time/evidence filtering produces exactly matching monetary subsets;
9. source-edge and source-value input ordering does not alter canonical outputs;
10. scenario-bound mutations fail closed;
11. AC-010 rejects any hidden method change if the `MethodChoice` assumption is removed;
12. an OCDS/BODS-derived end-to-end fixture traverses AC-005 through AC-014.

## Next tranche

AC-015 should stop adding analytical abstraction and introduce the first adversarial end-to-end municipal fixture: real-shaped OCDS/BODS records with identity collisions, missing values, joint awards, stale qualification receipts, contradictory registry claims, time-window changes, and deliberately misleading concentration patterns. The goal should be to prove the whole AC-001→AC-014 chain fails closed and explains why.
