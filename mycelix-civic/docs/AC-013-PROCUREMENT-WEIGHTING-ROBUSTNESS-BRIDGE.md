# AC-013 — Procurement Weighting Robustness Bridge

Status: implementation candidate; execution qualification not yet established.

## Purpose

AC-013 is the first integration between AC-012 monetary semantics and AC-010 robustness analysis.

It deliberately does not yet expand AC-011's full identity/evidence/time Cartesian matrix. Instead, it proves the smaller and more fundamental comparison theorem first:

> AC-004 equal-award-count supplier HHI and AC-012 value-weighted supplier HHI can be compared as two explicit specifications only when they operate over the exact same award-edge population and the weighting/method change is declared.

This keeps the monetary integration independently reviewable before it is multiplied across other robustness axes.

## Governing rule

`same award-edge population + count-HHI + qualified value-HHI + explicit weighting assumption + explicit method assumption -> AC-010 robustness envelope`

not:

`different HHI number -> call it sensitivity`.

## Exact population binding

AC-013 computes:

1. AC-004 equal-award-count supplier concentration;
2. AC-012 supplier-attributed value-weighted concentration.

It then requires:

`count.input_edge_refs == value.input_edge_refs`.

If the value calculation covers a different award population, the bridge fails rather than allowing a weighting comparison confounded by population change.

## Explicit paired assumptions

The value-weighted scenario must carry two distinct assumptions:

- `ProcurementWeighting` — the economic choice to weight suppliers by qualified supplier-attributed award value rather than award count;
- `MethodChoice` — the analytical change from the AC-004 count-HHI method to the AC-012 value-HHI method.

This is intentionally redundant-looking but epistemically important. AC-010 rejects hidden method changes, so AC-013 makes both the substantive weighting assumption and the method identity change visible.

The two assumption IDs must be distinct.

## Scenario structure

AC-013 produces exactly two scenarios:

- baseline: AC-004 award-count HHI, no alternative assumptions;
- alternative: AC-012 value-weighted HHI, typed as a `Joint` scenario with both weighting and method-choice assumptions.

The observations retain the same AC-002 subject, metric family and unit. Their method references are intentionally different.

## Lineage

The output preserves:

- count-specification input edge refs;
- value-specification input edge refs;
- AC-012 value snapshot reference;
- sorted AC-012 value-record references;
- the complete AC-010 envelope with both observations and both assumptions.

The bridge method is versioned as:

`mycelix:ac-013:procurement-weighting-robustness-bridge:v1`.

## Interpretation

The result can legitimately support a statement such as:

> Supplier concentration is 0.50 when each award is weighted equally and 0.625 when the same awards are weighted by their qualified supplier-attributed values. The difference is attributable to the explicit procurement-weighting/method specification, not to a changed award population.

It cannot support:

> Value-weighted concentration proves corruption.

Nor does it say that one weighting method is normatively superior. Count weighting answers how awards are distributed; value weighting answers how qualified award value is distributed.

## Relationship to AC-011

AC-013 is a bridge, not yet the full matrix composition.

This avoids duplicating AC-011's one-time identity qualification and scenario enumeration. After AC-013 qualifies independently, a later tranche can extend AC-011 so each identity/evidence/time specification can optionally branch into count-weighted and value-weighted variants while preserving one frozen identity trust snapshot.

## Failure semantics

AC-013 fails closed for:

- missing envelope ID;
- duplicate weighting/method assumption IDs;
- AC-004 count-metric failure;
- AC-012 monetary/value-metric failure;
- count/value input-edge lineage mismatch;
- AC-010 robustness-envelope rejection, including invalid assumption provenance or hidden method semantics.

## Validation authored

Current tests cover:

- exact count/value input-edge lineage equality;
- value scenarios carrying both weighting and method-choice assumption refs;
- exact count-HHI versus value-HHI arithmetic;
- preservation of the same AC-002 metric family across the method change;
- duplicate assumption-ID rejection.

## Qualification gate

Before AC-013 is qualified:

1. exact-subject Civic workspace tests pass;
2. rustfmt passes;
3. warnings-denied Clippy passes;
4. independent fixtures reproduce AC-004 and AC-012 exact ratios;
5. mutation tests prove changing one specification's edge population fails with `InputEdgeLineageMismatch`;
6. mutation tests prove deleting `MethodChoice` semantics makes AC-010 reject the method change;
7. weighting and method assumptions require valid AC-010 provenance/admissibility evidence;
8. value snapshot/value-record lineage matches the AC-012 receipt;
9. input ordering does not alter the two scenario observations or bounds;
10. review confirms the bridge creates no finding, guilt label or sanction authority.

## Next tranche

AC-014 should compose the qualified AC-013 weighting pair into AC-011's procurement robustness matrix without re-running identity verification per scenario.

The combined scenario generator should preserve one AC-008 identity trust snapshot and enumerate the selected axes:

- identity resolution;
- evidence admission;
- time window;
- weighting/method pair.

Every value-weighted scenario must use the AC-012 value subset corresponding exactly to that scenario's filtered award-edge population, and every scenario receipt must preserve both edge and monetary-record lineage.
