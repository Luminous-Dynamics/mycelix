# Dual-Axis KPI Comparability Benchmark v0.1

**Status:** synthetic known-answer benchmark  
**Fixture:** [24 adversarial KPI pairs](../../../fixtures/nrvca-kpi-dual-axis-comparability-synthetic-v0.1.json)  
**Rust test:** `mycelix-workspace/tools/nrvca-validator/tests/kpi_dual_axis_comparability.rs`  
**Tracking issue:** [NRVCA-014 / #5437](https://github.com/Luminous-Dynamics/mycelix/issues/5437)  
**Prior increment:** [NRVCA-013 / PR #5435](https://github.com/Luminous-Dynamics/mycelix/pull/5435)

## Paradigm shift: comparability has two independent axes

It is not enough to ask “is this KPI the same measure?” We must separately ask “is the target policy the same?” Otherwise a target revision contaminates an otherwise valid actual-value trend, while the reverse failure lets an unchanged target conceal a changed measurement definition.

This contract returns:
- **Measurement comparability:** `comparable`, `not_comparable`, or `indeterminate`.
- **Target-policy comparability:** `same_target_policy`, `changed_target_policy`, or `indeterminate`.

A target changing from 90% to 95% does not necessarily invalidate the actual KPI time series; it does mean the target-gap series crosses a policy revision. A deadline, denominator, unit, event inclusion, geographic scope or aggregation-window change, by contrast, blocks a direct measurement trend unless a separately validated bridge exists. Critical missing semantics force an abstention.

## Synthetic benchmark composition

The fixture contains **24 known-answer pairs** including exact repeat, wording-only changes, source-vintage changes, local code changes, target-only changes, deadline/category drift, changed eligibility/numerator/denominator, unit, clock start, geography, aggregation and inclusion rule, unknown deadline/denominator/target fields, a changed canonical concept, and a misleading declared fingerprint.

The naive baseline is deliberately transparent: consider the KPI comparable if and only if its local metric ID and display label match exactly. It represents the common “same name = same KPI” anti-pattern, not a claim about every commercial utility benchmarking product.

### Frozen pair-level decisions

| Measurement status | Pairs |
|---|---:|
| Comparable | 10 |
| Not comparable | 12 |
| Indeterminate | 2 |
| **Total** | **24** |

| Target-policy status | Pairs |
|---|---:|
| Same policy | 18 |
| Changed policy | 4 |
| Indeterminate | 2 |
| **Total** | **24** |

### Baseline errors on this fixture

The label-and-ID baseline makes **14 false-comparable decisions** (the expected result is not-comparable or indeterminate) and **2 false-incomparable decisions** (semantic fields match even though wording or local identifier changed). These are exact counts on this deliberately adversarial 24-pair fixture, not an estimate of real-world error rates.

The semantic comparator in the reference tests checks actual field values rather than trusting the `declared_fingerprint`. Known semantic mismatch takes precedence over unknown fields: if one known field has materially changed, the pair is not directly comparable; if no known mismatch exists but a critical field is unknown, the result is indeterminate.

## Industry and standards comparison

This work should compose with existing practices rather than rebrand them:

- The [AWWA Utility Benchmarking Program](https://www.awwa.org/programs/benchmarking/) compares performance indicators by utility type, size and region, with more than 100 measures. AWWA's full definitions and access terms would need to be obtained before an apples-to-apples empirical comparison.
- The [International Benchmarking Network for Water and Sanitation Utilities (IBNET)](https://newibnet.org/about-ibnet) publishes standardized indicators and compares service providers across country and regional contexts, reporting a network spanning over 5,000 providers in more than 150 countries.
- [ISO 24510:2024](https://www.iso.org/standard/81484.html) covers user-relevant drinking-water/wastewater service characteristics and service assessment.
- The [SDMX glossary](https://sdmx.org/wp-content/uploads/SDMX_Glossary_Version_2_1_December_2020.htm) and the IMF/World Bank's [Data Quality Assessment Framework discussion](https://datahelpdesk.worldbank.org/knowledgebase/articles/906534-data-quality-and-effectiveness) emphasize conceptual definitions, consistency, revisions, coverage and quality metadata.

NRVCA's proposed incremental advantage is a machine-checkable *decision at comparison time*: preserve both actual-value and target-policy semantics, explicitly abstain under uncertainty, and block a downstream chart/optimizer from silently comparing incompatible series. The current fixture does not establish that it outperforms AWWA or IBNET; those programmes use broader data and industry processes.

## Benchmark targets

These targets are for a future qualification dataset, not current measured performance.

| Metric | Target |
|---|---|
| Hard-semantic mismatch recall | 100% on the registered deadline/population/numerator/denominator/unit/clock/geography/aggregation mutations |
| False-comparable rate | 0 on the registered hard-change controls |
| Missing-semantics handling | 100% abstention when a critical field is unknown and no known mismatch decides the result |
| Label-noise robustness | 100% preservation of comparability for wording-only changes when all semantic fields match |
| Target-policy separation | 100% identification of target changes without automatically invalidating actual-value comparability |
| Fingerprint integrity | Zero trust in a declared fingerprint when actual semantic fields differ |
| Pair-level auditability | Every result has two axis statuses and machine-readable reasons |
| Source integrity | Every real source either has acquired bytes plus hash/retrieval receipt or explicitly remains unpinned |

The right next evaluation is not more handpicked JSON alone. Build at least 100 real KPI-version pairs across water, wastewater and other local services, draw examples from at least three source families, and obtain two independent domain-review labels plus adjudication for disagreements. Report class-wise precision/recall, false-comparable severity, and abstention rate. Do not optimize headline accuracy if the cost of a false trend is much higher than the cost of abstaining.

## Qualification gates

1. Run the Rust integration tests on an exact recorded head and retain failed results.
2. Add property-based mutations for every semantic and target field, including simultaneous known-mismatch-plus-unknown combinations.
3. Validate against a blinded expert-labelled corpus; keep the expert labels and disagreements in the evidence ledger.
4. Obtain definitions and access terms for relevant AWWA/IBNET indicators before adding external benchmark pairs.
5. For Johannesburg Water SS1, obtain both source PDFs and underlying numerator/denominator rules. If possible, request a parallel-run extract applying both old and new definitions to the same records; only then consider an empirical bridge.
6. Test whether the comparator can reject misleading trends in downstream reports, not merely pass unit tests in isolation.

## Evidence boundary

All 24 pairs are synthetic test cases. They do not describe 24 real utility KPIs, estimate false-comparability prevalence in industry, or demonstrate better delivered water service. The Johannesburg Water values in NRVCA-013 remain official reported values transcribed from search-indexed table text; source PDFs were not acquired or hashed in that work session.
