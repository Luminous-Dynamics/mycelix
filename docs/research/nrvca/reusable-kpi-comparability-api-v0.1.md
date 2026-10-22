# NRVCA-015: Reusable dual-axis KPI comparability API v0.1

**Status:** implementation draft; Rust qualification pending  
**Tracking issue:** [#5442](https://github.com/Luminous-Dynamics/mycelix/issues/5442)  
**Prior contract:** [NRVCA-014 / PR #5438](https://github.com/Luminous-Dynamics/mycelix/pull/5438)  
**Implementation:** `mycelix-workspace/tools/nrvca-validator/src/comparability.rs`  
**Frozen known-answer corpus:** [24-pair synthetic fixture](../../../fixtures/nrvca-kpi-dual-axis-comparability-synthetic-v0.1.json)

## Paradigm shift

The comparability rule is no longer intended to live only inside integration-test logic. This increment promotes it to a public typed Rust API so future KPI bridge/report code can reuse the same decision and field-level evidence instead of reimplementing the policy.

The result is a vector, not one opaque score:

- `measurement.status`: `comparable`, `not_comparable`, or `indeterminate`.
- `target_policy.status`: `same_target_policy`, `changed_target_policy`, or `indeterminate`.
- Each axis returns deterministic `changed_fields` and `unknown_fields` in a fixed canonical field order.

The public API fixes its measurement and target fields. Labels, local identifiers, source-vintage strings and declared semantic fingerprints do not override the actual measured contract. A known semantic mismatch takes precedence over unknown fields; otherwise, any missing/null critical field yields an indeterminate result. Target-policy changes do not invalidate comparability of actual measurements by themselves.

## Qualification gates

The committed integration test exercises the library API against every frozen known answer and aggregate count, then mutates every measurement field and every target-policy field. It also checks unknown-field abstention, mismatch precedence, source-vintage/wording/local-ID invariance, misleading fingerprint resistance, numeric JSON representation equivalence for targets, and deterministic results.

Required gates before calling the implementation qualified:

1. Exact expected decisions on all 24 synthetic pairs.
2. Zero false-comparable results for known semantic mismatches in the synthetic adversarial suite.
3. 100% abstention for a critical missing field when no known mismatch decides the case.
4. 100% target/measurement-axis separation for target-only mutations.
5. Independent Rust test execution on the exact commit SHA.
6. No field accuracy or state-of-the-art claim until tested against an independently adjudicated multi-provider corpus.

## Benchmark roadmap and industry comparison

The synthetic corpus is a contract test, not a realistic prevalence sample. Its confusion counts must not be quoted as industry rates. To establish external validity, next acquire definition material with use rights from multiple utilities and benchmark organizations, then build a blinded expert-adjudicated corpus spanning different units, denominators, event clocks, eligible populations, category rules, inclusion/exclusion policies, geography, aggregation and target-policy revisions.

Proposed *future* acceptance targets (not current results):

- Zero false-comparable decisions on adjudicated material definition mismatches.
- At least 95% exact three-way agreement with adjudication, while separately reporting coverage and abstention rate.
- Publish false-comparable and false-incomparable rates, per-field error breakdowns, confidence intervals, provider-held-out results and inter-rater agreement.
- Split train/tuning and evaluation by provider and time vintage to prevent near-duplicate leakage.
- Compare against an explicit same-ID/label baseline and each partner's actual production definition mapping on identical held-out pairs.
- Report throughput and latency only after realistic scale tests; speed cannot compensate for unsafe semantic equivalence errors.

AWWA and IBNET are important comparison targets because they provide established utility benchmarking definitions and cross-provider experience. They are not assumed weak baselines: acquire and respect their access/licensing terms and compare only where a fair, rights-compatible dataset is available.

## Limits

This code compares declared JSON fields; it does not prove the source is truthful, complete, licensed, correctly transcribed, or implemented by an operator as declared. It does not independently establish that prose definitions with different wording are semantically identical. The current fixture is synthetic and small; human adjudication and real source bytes remain prerequisites for empirical claims. No automatic target revision, policy setting, funding ranking or service dispatch is enabled.
