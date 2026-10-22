# Utility Service KPI Semantic Drift v0.1

**Status:** official-reported, search-indexed transcription; source PDFs are not byte-pinned  
**Fixture:** [KPI definitions, observations and comparability decisions](../../../fixtures/nrvca-jw-service-kpi-semantic-drift-v0.1.json)  
**Rust checks:** `mycelix-workspace/tools/nrvca-validator/tests/service_kpi_semantic_drift.rs`  
**Tracking issue:** [NRVCA-013 / #5434](https://github.com/Luminous-Dynamics/mycelix/issues/5434)

## The concrete failure mode

Johannesburg Water's official Annual Report 2023/24 Table 28 describes SS1 as the percentage of water-supply interruptions concluded within 12 hours of notification: target 90%, actual 11.65%, reported variance -78.35 percentage points. Its Annual Report 2024/25 Table 28 describes SS1 as **planned** water-supply interruptions completed within category-specific notification deadlines (A=12 hours, B=24 hours, C=72 hours): target 90%, actual 85.19%, reported variance -4.81 points.

Sources:
- Annual Report 2023/24, Table 28: https://www.johannesburgwater.co.za/wp-content/uploads/2025/02/FINAL-JOHANNESBURG-WATER-ANNUAL-REPORT-23_2024-20-02-2025.pdf
- Annual Report 2024/25, Table 28: https://www.johannesburgwater.co.za/wp-content/uploads/2026/05/Annual-Report-2425.pdf

Those numbers should not be linked as one year-over-year trend just because the row label/identifier says SS1. Eligibility and deadline semantics changed. Computing 85.19 - 11.65 = +73.54 percentage points and calling it a service improvement would confound metric-definition change with any actual operational change.

The annual report's 2024/25 narrative says 3 of 13 service standards were achieved; it separately notes 10 standards were missed. The two KPIs transcribed here do not represent the complete 13-row table and must not be used to reconstruct the report-level count.

## A provisional comparable indicator

SS2 is reported in both years as the percentage of fire hydrants repaired within 48 hours of notification:
- 2023/24: target 80%, actual 68.11%, variance -11.89 percentage points.
- 2024/25: target 80%, actual 71.06%, variance -8.94 percentage points.

The published percentage moved by **+2.95 percentage points**, and the target gap narrowed by the same amount. This is labelled **provisionally definition-comparable, denominator unknown**: the visible report extracts have the same stated deadline and label, but they do not provide the raw numerator, denominator, exclusion rules, or record-completeness evidence needed to independently prove method consistency. This is a reported KPI trend, not proof of a causal effect or of better household water availability.

## Paradigm shift: benchmark semantic contracts, not names

A KPI should be represented as a versioned contract that includes:
- canonical metric identity and definition vintage;
- event/population eligibility;
- numerator and denominator;
- timing/deadline rules, categories and clock start;
- unit, target, reported actual and reported variance;
- reference period, report source, table/page and source-byte status;
- known exclusions, record completeness and definition uncertainty;
- comparability status plus machine-readable reasons.

The validator binds the semantic fingerprint back to actual definition fields. A changed deadline or eligibility cannot be masked by copying the old fingerprint. Numeric deltas are suppressed when fingerprints differ. Matching fingerprints can only permit a provisional comparison when raw denominator/method equivalence remains unverified.

This is a defense against Goodhart-type dashboard failure: an organization can appear to improve when a denominator, service population, timeout, event category or recording practice changes. Preserve both published series; don't normalize away the definition change.

## Existing practice and what would improve it

Versioned metadata and statistical revision histories are established practices; NRVCA should not claim novelty for keeping vintages. The incremental objective is a machine-checkable comparability decision embedded beside the reported values so a downstream map, dashboard, report, or optimizer cannot silently create a trend from semantically different measures.

The next data tier should obtain source bytes and, where possible, event-level numerator/denominator definitions for each reporting year. If the definitions truly changed, publish a **parallel-run overlap study** using both old and new definitions on the same incident/work-order data, then estimate a bridge only if the underlying records support it. Never invent a conversion coefficient from the two annual headline percentages.

## Benchmarks: acceptance targets, not achieved results

| Benchmark | Baseline | Target |
|---|---|---|
| Semantic drift detection | Compare same-named KPI labels directly | Detect deadline/eligibility/unit/fingerprint changes and suppress false trends in 100% of registered negative controls |
| Definition traceability | Store a label and one annual percentage | Every measure has a versioned definition with numerator/denominator knowledge state, deadline and source vintage |
| Arithmetic integrity | Trust reported variance blindly | Recompute every actual-minus-target percentage-point variance and flag mismatches |
| Source integrity | Dashboard-copied values with unclear provenance | Acquired bytes hashed; otherwise explicitly unpinned and no digest |
| Uncertainty integrity | Treat matching labels as full comparability | Distinguish confirmed comparability, provisional comparability and not-comparable |
| Trend quality | Report one average change | Report value change, target-gap change, confidence interval where denominators are known, and an explicit causal non-claim |
| Robustness | Hand-reviewed examples alone | Mutation suite for changing deadlines, categories, denominator, unit, target, source, period, vintage and achieved-count scope |

For production qualification, aim for zero accepted false comparability results on the preregistered semantic-mutation corpus and zero silent denominator changes. On real source updates, measure source-reconciliation error rate and the proportion of KPIs whose comparability could be independently confirmed.

Do not optimize for a target of “improve every KPI by x%” until metric semantics, ownership and service outcomes are qualified. That target encourages gaming, can conflict with priorities, and is not a measure of whether households received more reliable water.

## What the current prototype can and cannot claim

It can demonstrate:
- the SS1 definition change is captured and its year-over-year numeric delta is suppressed;
- the SS2 reported percentage and target gaps reconcile, with a provisional comparability label;
- the report-level “3 of 13 achieved” remains source-attributed;
- source hashes are null because the PDF bytes were not acquired.

It cannot claim a source-byte-pinned extraction, explain the actual number of hydrant jobs, compute confidence intervals without denominator counts, infer household supply continuity, or attribute changes to any intervention. The original PDFs were too large/blocked for source-byte acquisition in this pass; the values are from official-site search-indexed table text and the fixture remains discovery-only.

## Qualification gates

1. Acquire both annual report PDFs, record byte lengths, retrieval times and SHA-256 hashes.
2. Independently parse Table 28 in each file and compare every transcribed value/definition; retain discrepancies.
3. Obtain numerator/denominator definitions and counting/exclusion rules from the underlying service-standard specification.
4. Request an event-level export or parallel-run definitions for SS1, and verify whether the apparent new target counts only planned outages.
5. Reassess SS2 comparability against raw work-order inclusion, closure clocks, reopened work orders, backdated records and missing/duplicate jobs.
6. Run Rust tests against the exact latest head before any passing status is claimed; then independent review and version the comparability policy.

