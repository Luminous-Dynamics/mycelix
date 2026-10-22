# NRVCA-015: Reusable dual-axis KPI comparability API v0.1

**Status:** implementation draft; Rust qualification pending  
**Tracking issue:** [#5442](https://github.com/Luminous-Dynamics/mycelix/issues/5442)  
**Prior contract:** [NRVCA-014 / PR #5438](https://github.com/Luminous-Dynamics/mycelix/pull/5438)  
**Implementation:** `mycelix-workspace/tools/nrvca-validator/src/comparability.rs`  
**Frozen known-answer corpus:** [24-pair synthetic fixture](../../../fixtures/nrvca-kpi-dual-axis-comparability-synthetic-v0.1.json)

## Paradigm shift

The comparability rule is no longer intended to live only inside integration-test logic. This increment promotes it to a public typed Rust API so future KPI bridge/report code can reuse the same decision and field-level evidence instead of reimplementing the policy.

The result is a vector, not one opaque score. The public module requires nonempty strings for semantic metadata fields and a finite numeric target value; missing, null, empty, and ill-typed values are unknown. Numeric target equality is compared as exact decimal values, so integer/decimal encodings such as 90 and 90.0 agree without collapsing distinct large integers through floating-point rounding.


- `measurement.status`: `comparable`, `not_comparable`, or `indeterminate`.
- `target_policy.status`: `same_target_policy`, `changed_target_policy`, or `indeterminate`.
- Each axis returns deterministic `changed_fields` and `unknown_fields` in a fixed canonical field order, plus sorted `unsupported_fields` for keys that this version does not understand.

The public API fixes its measurement and target fields. Labels, local identifiers, source-vintage strings and declared semantic fingerprints do not override the actual measured contract. A known semantic mismatch takes precedence over unknown fields; otherwise, any missing, null, empty, ill-typed critical field—or any unrecognized extra key inside an axis object—forces an indeterminate result. This prevents a future field such as a target comparison operator (`at_least` versus `at_most`) from being silently ignored before the contract is versioned. Target-policy changes do not invalidate comparability of actual measurements by themselves.

## Qualification gates

The committed integration test exercises the library API against every frozen known answer and aggregate count, then mutates every measurement field and every target-policy field. It also checks unknown-field and unversioned-extension abstention, mismatch precedence, source-vintage/wording/local-ID invariance, misleading fingerprint resistance, exact numeric JSON comparison, symmetric decisions, and deterministic results.

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

AWWA and IBNET are important comparison targets because they provide established utility benchmarking definitions and cross-provider experience. AWWA's currently published benchmarking page advertises its 2026 utility report and comparisons by utility type, size, and geographic region. IBNET describes standardized indicators and utility-specific/contextual comparisons; its About page describes cumulative participation from over 5,000 providers in over 150 countries, while its home page separately labels 2,792 utilities and 152 countries as the IBNET 1.0 submitted dataset. These figures refer to different scopes and should not be conflated. The work here is **not** a claim that those programs use the simplistic label-and-ID baseline; that baseline is intentionally local and transparent. A fair empirical comparison requires access to actual indicator definitions and permitted evaluation data.

The standardization strategy should be compositional, not a competing vocabulary invented from scratch:

- **SDMX** defines statistical metadata and comparability as a question about applied statistical concepts/definitions over geography and time. Its current guidance includes explicit modelling guidance for data vintages and units of measure. NRVCA should eventually map to SDMX concepts, code lists, unit representations, and reference-metadata where those fit; the current fixed field contract is not an SDMX implementation.
- **ISO 24510:2024** emphasizes water-service elements that matter to users, assessment criteria, and performance indicators. This is a guard against defining “better” solely as internal operator throughput or spend.
- **W3C PROV** offers a provenance vocabulary for entities, activities, and agents, including versioning and derivation. A field comparator does not replace provenance: each source artifact and extraction should remain independently traceable.
- **AWWA 2026** and **IBNET** are real industry benchmark baselines for future *fair* evaluation, subject to obtaining definition materials/data and respecting access terms.

Sources: [AWWA Utility Benchmarking](https://www.awwa.org/programs/benchmarking/), [IBNET About](https://newibnet.org/about-ibnet), [IBNET Use Data](https://newibnet.org/use-data), [SDMX comparability glossary](https://sdmx.org/wp-content/uploads/SDMX_Glossary_Version_2_1_December_2020.htm), [SDMX guidelines (including vintage and unit guidance)](https://sdmx.org/guidelines/), [ISO 24510:2024](https://www.iso.org/standard/81484.html), [W3C PROV overview](https://www.w3.org/TR/prov-overview/).

## Limits

This API performs strict declared-field comparison, not open-ended natural-language semantic inference. A human-reviewed rewrite of a numerator/denominator definition can still be marked non-comparable if the declared canonical fields are changed; that conservative false-incomparable risk is preferable to silently asserting equivalence. A future alias/bridge registry must itself be versioned, source-cited, adjudicated, and tested rather than quietly normalizing text. The API also does not prove a source is truthful, complete, licensed, correctly transcribed, or implemented by an operator as declared. The current fixture is synthetic and small; human adjudication and real source bytes remain prerequisites for empirical claims. No automatic target revision, policy setting, funding ranking or service dispatch is enabled.
