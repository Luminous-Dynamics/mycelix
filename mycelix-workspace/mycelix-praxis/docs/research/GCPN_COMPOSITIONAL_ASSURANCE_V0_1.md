# GCPN Compositional Assurance Under Interacting Evidence-Chain Failures v0.1

**Status:** Stage A specification; standalone deterministic Rust implementation is Stage B.  
**Parent issue:** #5672.  
**Scope:** synthetic, offline compositional assurance; no operational warning authority.

## 1. Research question

Can independent evaluators reconstruct the same claim-level conclusions from the same evidence package when several components fail simultaneously?

Component tests cannot establish that a composed evidence chain is reliable. Common-mode observation errors, selective verification, stale message versions, missing population denominators, late forecasts and hindsight evidence can interact to create a plausible but unsupported institutional conclusion.

## 2. Paradigm shift

Move from component qualification to **compositional assurance under known-truth fault injection**.

The system must return a vector of typed claim dispositions and violation codes—not one compensatory score. A good forecast score cannot compensate for a stale alert receipt, unknown target population, unsupported causal attribution, or post-cutoff evidence.

### Typed claim dispositions

- Supports
- Refutes
- Indeterminate
- OutOfScope
- Unavailable

### Evidence validity

- Current
- Stale
- Revoked
- Invalid

Common-cause lineage must be evaluated before evidence is counted as independent corroboration. Two evaluators with different names but the same source/calibration/power/telemetry lineage are not independent witnesses.

## 3. Benchmark stage families

1. Forecast and post-processing provenance.
2. Observation/reference integrity.
3. CAP message/version/receipt lifecycle.
4. Aggregate population reach.
5. Decision timing and decision-value context.
6. Evidence-reconstructible after-event review.

Each stage emits typed claim evidence with exact subject, source/version, valid time, recorded/known time, and common-cause lineage.

## 4. Required fault catalogue

- Common-mode calibration, power, telemetry or processing error.
- Outcome-dependent missing verification and frame undercoverage.
- Stale/superseded CAP Update/Cancel and mismatched receipt.
- Candidate/reference source, version, case-frame or subject mismatch.
- Late forecast and insufficient action lead.
- Unknown denominator and mobile-population undercoverage.
- Post-cutoff evidence leakage into AER.
- Chronology or association improperly promoted to causal effect.
- Post-processing/calibration leakage into intrinsic forecast-skill claims.
- Duplicate/related evaluators presented as an independent portfolio.

Faults carry stable identifiers and explicit affected stages/claims. Detection must be attributed to a concrete rule, not a learned or opaque aggregate score.

## 5. Stage B implementation contract

Standalone dependency-free Rust package and deterministic CLI. Typed scenario input includes:
- scenario ID and frozen specification digest;
- a list of stage evidence records;
- explicit fault injections;
- evaluator lineage groups;
- as-of cutoff;
- expected claim dispositions and expected violation codes (benchmark oracle only).

The engine validates input structure, applies a declared set of deterministic audit rules, and emits canonical sorted outputs. The oracle is not trusted as evaluator evidence; it is used only to score the evaluator's output against known-truth fixtures.

## 6. Interaction and metamorphic programme

- Single-fault positive/negative controls.
- Pairwise interactions for all predeclared high-risk fault pairs.
- Selected three-way and higher-order scenarios with an explicit coverage matrix.
- Metamorphic invariants: record reordering; consistent opaque-ID renaming; irrelevant out-of-scope evidence insertion; repeat replay.
- Mutation controls: remove one fault, inject one fault, change a lineage root, change source/version, shift the as-of cutoff.
- Independent second implementation of the fixture oracle; no shared evaluation code.
- Deterministic canonical result bytes and content digest.

Do not attempt an unbounded Cartesian product. Use risk-based pairwise coverage plus hand-selected higher-order interactions.

## 7. Proposed qualification targets (not standards or achieved results)

| Dimension | Proposed target |
|---|---|
| Frozen oracle | 100% exact reproduction of scenario dispositions |
| Hard negatives | Zero false passes across the specified fatal negative controls |
| Fatal invariants | 100% detection in the declared test catalogue |
| Common cause | Zero independence credit for shared fault-domain lineage |
| Temporal integrity | Zero post-cutoff evidence leakage |
| Accounting | Zero untyped or silently dropped eligible records |
| Determinism | 100% stable canonical result digest on replay |
| Independent replay | Exact claim-disposition agreement between separately implemented evaluators |
| Fault interactions | Full coverage of predeclared high-risk pairs and selected higher-order combinations |
| Reporting | Detection, false-positive, abstention, coverage, per-fault sensitivity and worst-case interaction metrics reported separately |

No single overall score may allow one subsystem's success to compensate for another subsystem's fatal failure.

## 8. Relation to existing GCPN work

This benchmark composes and stress-tests, but does not replace:
- #5652 forecast interval qualification;
- #5653 bitemporal evidence graph;
- #5660 mobile-population reach;
- #5661 after-event review;
- #5664 spatial ensemble scoring;
- #5667 decision-value verification;
- #5668 outcome ascertainment;
- #5670 reference-observation uncertainty;
- #5659 typed oracle portfolio composition.

It is a meta-evaluation benchmark for their interactions, not another forecast model or warning authority.

## 9. Industry/research positioning

WeatherBench 2 explicitly rejects the idea that one score captures forecast quality; it highlights limitations of univariate probabilistic scores for spatial/temporal dependence and extremes. Operational verification research documents incomplete/uncertain observations, difficulty capturing user experience and institutional barriers. Bonavita (2026) distinguishes intrinsic forecast information from noise and processing effects. User-centred decision-relevant evaluation reinforces that statistical skill and practical decision value are separate questions.

GCPN's proposed contribution is reproducible end-to-end failure interaction testing and evidence lineage. No superiority in forecast generation is claimed.

## 10. Research basis

- Rasp et al. (2024), WeatherBench 2: https://agupubs.onlinelibrary.wiley.com/doi/full/10.1029/2023MS004019
- Bonavita (2026), Forecast verification using information and noise: https://doi.org/10.1002/qj.70109
- Brown et al. (2024), Challenges of Operational Weather Forecast Verification and Evaluation: https://journals.ametsoc.org/view/journals/bams/105/4/BAMS-D-22-0257.1.xml
- Emerton et al. (2026), How Good Is Your Forecast?: https://rmets.onlinelibrary.wiley.com/doi/full/10.1002/met.70234

## 11. Non-claims

No operational readiness, real-world forecast superiority, complete causal identification, zero risk, validated emergency procedures, or universal evaluator independence is claimed. Initial qualification is deterministic synthetic assurance only.
