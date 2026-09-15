# AC-010 — Explicit Institutional Robustness Envelopes

Status: implementation candidate; execution qualification not yet established.

## Purpose

AC-010 turns analytical uncertainty into an explicit, evidence-bearing specification set rather than hiding it behind one preferred model or one scalar robustness score.

The governing question is:

> Which conclusions survive across the defensible assumptions we actually supplied?

not:

> What single number should users trust?

AC-010 is deliberately non-adjudicative. It can show that a civic measurement is stable, fragile, or specification-dependent across a declared scenario set; it cannot convert that stability or fragility into a finding of wrongdoing.

## Core objects

AC-010 introduces:

- `RobustnessDimension`
- `RobustnessAssumption`
- `RobustnessScenario`
- `RobustnessCoverage`
- `RobustnessBounds`
- `InstitutionalRobustnessEnvelope`
- `InstitutionalRobustnessContract`

The initial dimension vocabulary includes:

- identity reconciliation;
- procurement weighting;
- registry completeness;
- evidence corroboration;
- time window;
- method choice;
- explicit custom dimensions.

Each assumption must carry a stable ID, a human-readable statement, an admissibility/evidence reference, and digest-bearing provenance.

## Scenario semantics

A robustness envelope contains exactly one baseline plus one or more alternative scenarios.

Alternative scenarios are typed as either:

- `SingleDimension` — all referenced assumptions belong to exactly one robustness dimension;
- `Joint` — assumptions span at least two dimensions.

Every non-baseline scenario must state the assumptions that distinguish it from baseline. Assumption references must be canonical, sorted and unique.

Duplicate specifications are rejected. The same assumption set cannot be repeated under multiple scenario IDs and presented as if it were independent robustness evidence.

Every declared assumption must be exercised by at least one scenario.

## Hidden analytical flexibility

A scenario may not silently change its measurement algorithm.

If `observation.measurement.method_ref` differs from the baseline method, the scenario must reference an assumption in the `MethodChoice` dimension.

This prevents a scenario labelled as, for example, `IdentityReconciliation` from quietly changing both identity assumptions and the metric implementation.

The contract does not infer all possible hidden changes from source data. Future domain-specific scenario builders should add stronger checks where they can prove a particular dimension changed.

## Comparability gate

Every scenario observation must independently pass AC-002 validation.

All scenarios must preserve:

- the same `CaptureSubject`;
- the same `CaptureMetric` enum;
- the same measurement unit.

Method references may differ only when `MethodChoice` is explicitly declared.

This allows AC-010 to compare alternative defensible models of the same civic question without accidentally comparing unrelated measurements.

## Exact bounds

AC-010 does not use floating point as evidence authority.

Metric ratios are ordered by checked cross multiplication:

`a/b < c/d` iff `a*d < c*b`.

The envelope records:

- the exact minimum metric value;
- every scenario tied for that minimum;
- the exact maximum metric value;
- every scenario tied for that maximum;
- whether every supplied scenario is exactly equivalent to baseline.

`invariant_across_supplied_scenarios = true` means only that all values in this concrete envelope compare equal.

It does **not** mean the real-world conclusion is universally robust.

## Coverage semantics

AC-010 has two coverage states.

### Exploratory

`Exploratory { limitation }`

This is the conservative default. The limitation must explain why the supplied scenario set is not claimed to exhaust all defensible specifications.

### EnumeratedWithinDeclaredScope

`EnumeratedWithinDeclaredScope { scope_ref, provenance }`

This is a stronger claim, but still only within an explicit scope. The scope reference and digest-bearing provenance are mandatory.

AC-010 deliberately does not infer enumeration completeness merely from scenario count or Cartesian-looking inputs. The module validates that the claim is structurally evidenced; external review or a future independent enumerator must establish whether the declared scope was itself appropriate and actually complete.

Therefore:

`many scenarios != exhaustive robustness proof`.

## Epistemic boundary

AC-010 never produces:

- a corruption score;
- a trust score;
- a probability that an institution is corrupt;
- a finding of guilt;
- a sanction recommendation;
- a universal claim that a conclusion is robust.

A valid output says only what was observed across the declared scenario set and what coverage claim accompanied that set.

## Example interpretation

A consumer such as Symthaea should render an envelope in language like:

> Baseline procurement supplier concentration is X. Across the six supplied defensible specifications it ranges from A to B. The largest value occurs under scenarios S2 and S5. The scenario set is exploratory and does not claim to enumerate every defensible model.

It should not collapse the envelope into:

> Robustness score: 82/100.

## Relationship to earlier AC layers

AC-010 consumes AC-002-valid observations rather than reaching around the existing epistemic boundary.

Typical future inputs include:

- AC-004 raw concentration observations;
- AC-007 identity-projected observations;
- AC-009 sensitivity-informed alternative identity specifications;
- future procurement-value-weighted metrics;
- alternative registry-completeness assumptions;
- alternative evidence-corroboration gates;
- alternative time windows.

AC-010 does not itself manufacture those scenarios. Domain-specific builders should produce each observation with its own evidence lineage and uncertainty statement.

## Failure semantics

AC-010 fails closed for, among other cases:

- missing envelope ID;
- fewer than two scenarios;
- malformed assumptions or provenance;
- duplicate assumption IDs;
- malformed or AC-002-invalid observations;
- no baseline or multiple baselines;
- assumptions attached to baseline;
- alternative scenarios with no assumptions;
- non-canonical assumption references;
- unknown assumption references;
- `SingleDimension` scenarios spanning zero or multiple dimensions;
- `Joint` scenarios spanning fewer than two dimensions;
- duplicate specifications;
- declared assumptions never exercised by any scenario;
- hidden measurement-method changes;
- subject, metric-enum or unit mismatch;
- unsupported/unevidenced coverage claims;
- exact-ratio arithmetic overflow.

## Validation authored

Current tests cover:

- mathematically equivalent fractions being recognized as invariant;
- exact minimum/maximum preservation across unequal denominators;
- duplicate specifications failing closed;
- hidden method changes failing closed;
- explicitly declared method-choice changes remaining admissible.

## Qualification gate

Before AC-010 is qualified:

1. exact-subject Civic workspace tests pass;
2. rustfmt passes;
3. warnings-denied Clippy passes;
4. independent rational-arithmetic fixtures reproduce ordering and tied extrema;
5. zero-denominator observations are rejected through AC-002;
6. duplicate scenario/specification mutations fail closed;
7. hidden method changes fail unless `MethodChoice` is declared;
8. subject/metric/unit mutation tests fail closed;
9. exploratory coverage cannot be upgraded to enumerated coverage without a scope reference and digest-bearing provenance;
10. property tests confirm scenario input order does not alter canonical envelope contents or bounds;
11. review confirms no output field can be interpreted as an adjudication or sanction authority;
12. a future domain-specific builder demonstrates at least one real AC-004/AC-007/AC-009 robustness matrix instead of only synthetic fixtures.

## Next tranche

AC-011 should provide the first domain-specific robustness-matrix builder for procurement concentration.

It should construct AC-010 scenarios from typed, independently evidenced perturbations such as:

- raw vs qualified identity-resolved suppliers;
- award-count vs value-weighted concentration;
- alternate defensible time windows;
- registry-completeness bounds;
- corroborated-only vs declared evidence policies.

The builder must preserve which source edges, identity receipts, weighting assumptions and time-window definitions generated each observation. It should refuse to call the matrix exhaustive unless an independently reviewable specification scope has actually been enumerated.
