# AC-009 — Identity-Resolution Sensitivity Diagnostics

Status: implementation candidate; execution qualification not yet established.

## Purpose

AC-009 explains how much public-entity identity reconciliation changes procurement supplier concentration and which qualified equivalence components account for that change.

It is a robustness diagnostic, not a corruption detector, adjudication, or sanction authority.

## Public semantic boundary

The low-level arithmetic/lineage engine is crate-private.

External callers use the public `IdentitySensitivityContract` facade in `qualified_identity_sensitivity`.

Before the internal engine is entered, the facade requires both the baseline and projected observations to declare:

`CaptureMetric::ProcurementSupplierConcentration`

The internal engine separately requires the pinned AC-004 supplier-HHI method reference.

This two-part binding prevents a receipt from retaining supplier-HHI bytes and method metadata while relabeling them as another capture metric family.

## Exact decomposition

For an equivalence component with raw award counts `c1..cn`, merging those source records changes the HHI numerator by:

`(c1 + ... + cn)^2 - (c1^2 + ... + cn^2)`

The denominator remains:

`total_awards^2`

Because AC-007 components are disjoint, AC-009 requires:

`sum(component_deltas) == projected_hhi - baseline_hhi`

exactly.

If that equality fails, the receipt is inconsistent and AC-009 refuses to explain it.

## Receipt checks before explanation

The internal engine independently validates:

1. non-empty projection ID;
2. the pinned AC-007 projection method revision;
3. the pinned AC-004 supplier-HHI method revision;
4. every award edge through AC-003;
5. unique, non-empty source award-edge IDs;
6. baseline and projected input-edge lineage exactly equals the source award-edge set;
7. applied identity-link lineage exactly equals the union of component link references;
8. component IDs, member lists and link lists are canonical;
9. component membership is disjoint;
10. raw HHI recomputed from source supplier records matches the baseline observation;
11. projected HHI recomputed from the equivalence grouping matches the projected observation;
12. exact per-component contributions sum to the total identity-induced HHI delta.

The public facade adds the metric-enum check before all of the above.

## Outputs

`IdentityProjectionSensitivityReport` preserves:

- projection ID;
- award count;
- distinct supplier records before reconciliation;
- effective suppliers after reconciliation;
- exact reduced identity-induced HHI delta;
- each component's member award counts;
- each component's exact HHI contribution;
- each component's exact share of the identity-induced delta;
- dominant component and its exact share;
- diagnostic method reference.

All authoritative arithmetic is integer/rational.

## Example

With raw awards:

- A: 1
- B: 1
- C: 2

raw HHI is:

`(1^2 + 1^2 + 2^2) / 4^2 = 6/16`

If qualified identity evidence groups A and B:

`(2^2 + 2^2) / 4^2 = 8/16`

The identity-induced delta is:

`2/16 = 1/8`

The A/B component independently contributes:

`(1 + 1)^2 - (1^2 + 1^2) = 2`

so AC-009 can prove that this component explains 100% of the identity-induced change.

## No repeated trust-verifier calls

AC-009 does not repeatedly invoke AC-008's external verifier for leave-one-out experiments.

A verifier may consult revocation or authority state that changes between calls, creating a time-of-check/time-of-use inconsistency inside one report.

Instead AC-009 deterministically decomposes the already-produced projection from its source edges and receipt lineage.

## Trust inheritance

AC-009 validates semantic, mathematical and lineage consistency. It does not create trust merely because a fabricated object is internally self-consistent.

Deployment-authoritative interpretation still requires the projection to originate from AC-008's verifier-gated public path and its surrounding Xenia/Mycelix qualification evidence.

AC-009 answers:

> Given this qualified projection, what identity assumptions drive the concentration change?

It does not answer:

> Is concentration illegal or corrupt?

## Failure semantics

AC-009 fails closed for:

- baseline metric enum mismatch;
- projected metric enum mismatch;
- missing projection identity;
- unknown projection or metric method revision;
- invalid or duplicate award edges;
- source edge lineage mismatch;
- component/link lineage mismatch;
- duplicate or malformed components;
- overlapping component membership;
- components that do not actually regroup at least two suppliers;
- arithmetic overflow;
- baseline metric mismatch;
- projected metric mismatch;
- component contributions not summing exactly to the observed identity delta.

## Qualification gate

Before AC-009 is qualified:

1. exact-subject Civic workspace tests pass;
2. rustfmt and warnings-denied Clippy pass;
3. metric-enum relabeling tests fail before internal analysis;
4. independent arithmetic reproduces all exact-ratio fixtures;
5. baseline/projected metric mutations are detected;
6. edge/link lineage mutations are detected;
7. overlapping-component fixtures fail closed;
8. multi-component contributions sum exactly to total delta;
9. zero-change fixtures produce no false dominant component;
10. large-count checked-arithmetic boundaries are exercised;
11. review confirms sensitivity output cannot authorize adjudication or sanctions.

## Next tranche

AC-010 should generalize this pattern into a typed institutional robustness envelope covering identity reconciliation, procurement weighting assumptions, registry completeness, evidence-corroboration assumptions and time-window choices while preserving each dimension separately rather than collapsing them into one opaque score.
