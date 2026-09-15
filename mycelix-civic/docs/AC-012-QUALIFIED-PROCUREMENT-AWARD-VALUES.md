# AC-012 — Qualified Supplier-Attributed Procurement Award Values

Status: implementation candidate; execution qualification not yet established.

## Purpose

AC-012 defines the monetary evidence boundary required before procurement supplier concentration can be weighted by economic value rather than by award count.

The first version is deliberately narrow:

- supplier-attributed award values only;
- one currency per calculation;
- exact decimal representation;
- exact integer/rational arithmetic;
- no rounding;
- no foreign-exchange conversion;
- no negative/concession-flow weighting;
- no duplicated joint multi-supplier award totals;
- no mixing award values with contract/current/final values or implementation payments.

The governing rule is:

`typed supplier-attributed award values + exact currency semantics + AC-003 award edges -> value-weighted supplier concentration`

not:

`any money-looking field -> HHI weight`.

## OCDS alignment

AC-012 is designed for an upstream semantics reference such as OCDS `award.value`.

OCDS defines award `value` as the total value of an award. Its guidance also says that when different suppliers are awarded different items or values, those should be represented using separate award blocks rather than one indivisible multi-supplier total.

AC-012 therefore requires every monetary record to be attributed to one concrete AC-003 awardee edge and one unique upstream award-block reference. Reusing the same `award_ref` for two supplier edges fails closed.

This is intentionally conservative. If source data contains one joint total for several suppliers, an upstream qualified transformation must establish supplier-specific values before AC-012 can use them.

OCDS permits negative award values in contexts such as concessions where money flows from supplier to buyer. AC-012 v1 uses a non-negative coefficient and therefore leaves negative/concession economics outside the measurement theorem instead of silently taking absolute values or reversing signs.

## Currency semantics

`ExactCurrencyAmount` stores:

- unsigned decimal coefficient;
- decimal scale;
- uppercase three-letter currency code.

The amount is exactly:

`coefficient / 10^scale`.

AC-012 checks the three-letter uppercase shape but deliberately does not hard-code a currency registry into `civic-types`. The caller supplies:

- `currency_registry_ref`;
- digest-bearing registry provenance.

A production adapter should independently establish that the code is valid under the referenced registry, normally ISO 4217 or another explicitly qualified monetary registry.

All records in one AC-012 calculation must use exactly one currency. Mixed-currency inputs fail closed. AC-012 performs no FX conversion.

## Exact decimal normalization

Different decimal scales within the same currency are normalized to the maximum supplied scale by checked multiplication by powers of ten.

No division and no rounding occur.

For example:

- `1000 / 10^2 = 10.00`
- `500 / 10^1 = 50.0`

normalize exactly at scale 2 to weights `1000` and `5000`.

The initial maximum scale is 18 decimal places. Checked arithmetic rejects values that cannot be represented safely.

## Value set

`ProcurementAwardValueSet` contains:

- snapshot reference;
- explicit value-semantics reference;
- currency-registry reference;
- digest-bearing currency-registry provenance;
- supplier-attributed award-value records.

Every `SupplierAttributedAwardValue` contains:

- unique value reference;
- exact AC-003 award-edge reference;
- unique upstream award-block reference;
- exact amount;
- digest-bearing source provenance.

Every awardee edge supplied to the calculation must have exactly one value record, and every value record must reference an awardee edge in the current population.

Missing values are not imputed and records are not silently dropped.

## Value-weighted HHI

After exact scale normalization, AC-012 aggregates supplier weights:

`w_supplier = sum(normalized supplier-attributed award values)`.

The resulting supplier concentration is:

`HHI_value = sum(w_supplier^2) / (sum(w_supplier))^2`.

All authoritative calculations use checked integer arithmetic. The resulting AC-002 metric remains `ProcurementSupplierConcentration`, but carries the distinct method reference:

`mycelix:ac-012:procurement-value-weighted-supplier-hhi:v1`.

This method identity matters: award-count HHI and value-weighted HHI answer related but different questions and must never be presented as the same specification.

## Provenance

The resulting observation unions provenance from:

- every AC-003 award edge in the population;
- every supplier-attributed monetary record;
- the referenced currency registry evidence.

The returned receipt additionally preserves:

- value snapshot reference;
- sorted value-record references;
- currency;
- normalized decimal scale;
- exact AC-003 input-edge references through `DerivedCaptureObservation`.

## Fail-closed cases

AC-012 rejects, among other cases:

- missing snapshot/value-semantics/currency-registry references;
- missing or non-digest-bearing monetary provenance;
- empty value sets;
- malformed currency-code shape;
- excessive decimal scale;
- duplicate value references;
- duplicate award-edge value references;
- duplicate upstream award references;
- mixed currencies;
- invalid AC-003 awardee edges;
- duplicate award-edge IDs;
- value records pointing outside the award population;
- award edges with missing values;
- arithmetic overflow;
- zero total value;
- output that fails the AC-002 observation contract.

## Explicit non-goals

AC-012 v1 does not:

- perform currency conversion;
- choose exchange rates;
- round monetary values;
- support negative/concession flows;
- infer supplier allocation from a joint multi-supplier total;
- use tender estimates as award values;
- mix award values with contract values, amended/current contract values, payments, invoices, or lifecycle cost;
- infer missing amounts;
- decide whether value-weighted or count-weighted concentration is the normatively correct measure;
- create findings of corruption or wrongdoing.

Those semantics require separate typed evidence contracts rather than implicit transformations.

## Relationship to AC-010 and AC-011

AC-012 creates the monetary method needed for a future procurement-weighting robustness axis.

It is **not yet wired into AC-011**. This separation is intentional: the monetary theorem should qualify independently before it changes the scenario matrix.

A later integration can compare:

- AC-004 equal-award-count supplier HHI;
- AC-012 value-weighted supplier HHI.

Because those scenarios change the metric method as well as the weighting assumption, the AC-010 envelope should make both analytical choices explicit rather than hiding the method change.

## Validation authored

Current tests cover:

- exact value-weighted HHI arithmetic;
- exact cross-scale normalization without rounding;
- mixed-currency rejection;
- joint award totals being rejected when duplicated across supplier edges;
- every award edge requiring an explicit value record.

## Qualification gate

Before AC-012 is qualified:

1. exact-subject Civic workspace tests pass;
2. rustfmt passes;
3. warnings-denied Clippy passes;
4. independent decimal-normalization fixtures reproduce exact integer weights;
5. independent value-weighted HHI arithmetic reproduces all golden ratios;
6. mixed-currency and malformed-currency fixtures fail closed;
7. scale/overflow boundary fixtures fail closed without rounding;
8. duplicated joint-award fixtures fail closed;
9. missing/extra value-record mutations fail closed;
10. source edge and value-record input ordering does not change canonical output lineage;
11. a standards fixture proves that an OCDS multi-supplier award with one unsplit total is rejected rather than double counted;
12. a standards fixture proves separately attributed award blocks can be used safely;
13. currency-registry review confirms real code membership is verified upstream and not inferred from three-letter shape alone;
14. output AC-002 contract violations remain inspectable during qualification even though the v1 serialized error surface currently reports the generic `OutputContractViolation` category;
15. review confirms the observation remains non-adjudicative.

## Next tranche

AC-013 should integrate AC-012 into the AC-011 procurement robustness matrix as an explicit weighting/method specification.

To preserve AC-010's hidden-method-change rule, the value-weighted scenario should carry both:

- a `ProcurementWeighting` assumption describing the economic weighting choice; and
- a `MethodChoice` assumption declaring the switch from AC-004 award-count HHI to AC-012 value-weighted HHI.

That avoids rebasing AC-010 while keeping the analytical change fully visible.
