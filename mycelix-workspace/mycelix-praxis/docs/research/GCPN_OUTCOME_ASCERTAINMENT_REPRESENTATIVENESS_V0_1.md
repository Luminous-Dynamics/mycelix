# GCPN Outcome Ascertainment and Verification Representativeness v0.1

**Status:** Stage A research specification; implementation and confirmatory experiments outstanding.  
**Parent issue:** #5668.  
**Baseline:** integrated GCPN evaluation v0.6 plus the six forecast/evidence/mobile/AER programmes.  
**Scope:** offline synthetic qualification of outcome-observation coverage and missingness; no operational warning service.

## 1. Research motivation

Forecast verification is conditional on an outcome being available and accepted by a verification pipeline. That set is not automatically representative of all eligible forecast cases. Observation stations, field reports, remote sensing, communications networks, local institutions and event-reconstruction capacity are distributed unevenly. Missingness may depend on event severity, infrastructure damage, remoteness, mobility, access restrictions, or the outcome itself.

A model can therefore appear accurate on the easiest cases to verify while its performance in poorly observed areas remains unknown. The direction of the bias is not predetermined: missingness can make apparent performance look either better or worse.

The new GCPN layer evaluates **how the verification sample came to exist**, not just the scores inside it.

This is not a substitute for forecast calibration, spatial ensemble scoring, decision-value analysis, mobile-population reach or after-event review. It supplies a shared ascertainment contract those programmes can consume.

## 2. Paradigm shift: from score-only to evidence-availability qualification

For a predeclared eligible population of forecast cases, let:

- (N) be all eligible cases in the target frame at a declared cutoff;
- (R_i=1) when case (i)'s outcome has a verified score available by the cutoff, and (0) otherwise;
- (S_i) be the score or paired score difference for case (i);
- (S_i in [a,b]) be a declared, validated score range;
- (X_i) be pre-outcome strata/covariates used to describe or model verification inclusion;
- (pi_i=P(R_i=1mid X_i)) be a declared inclusion propensity when a propensity analysis is attempted.

The package must not treat missing outcomes as negative events, zero loss, zero score, or successful verification.

Every eligible case receives exactly one typed status at the as-of cutoff. Pending, not-collected, sensor-unavailable, communication-unavailable, quality-rejected, disputed and unknown-reason states remain distinguishable. A record observed after the cutoff must not leak into the historical estimate.

## 3. Estimands and outputs

### 3.1 Eligible-frame coverage

Report:
- eligible, observed and missing case counts;
- observed fraction (n_{obs}/N);
- missingness-reason counts;
- stratum denominators and coverage;
- independent-system counts;
- the as-of cutoff, target-frame ID, score name and score bounds.

Report missingness as a property of the evaluation frame. Do not call it forecast failure unless a separate forecast-availability status establishes that.

### 3.2 Complete-case estimate

The observed-case mean is:

[
ar S_{obs}=rac{sum_i R_i S_i}{sum_i R_i}.
]

It is descriptive of the verified subset. By default it is **not** a population estimate. Publish it beside coverage and the bounds below.

### 3.3 Assumption-light finite-population bounds

When all scores lie in ([a,b]), the sharp no-assumption bounds for the finite-frame mean are:

[
mu in left[
rac{sum_i R_iS_i+(N-n_{obs})a}{N},
rac{sum_i R_iS_i+(N-n_{obs})b}{N}
ight].
]

For Brier loss, use ([0,1]). For candidate-minus-reference paired score differences, use the corresponding signed range, e.g. ([-1,1]) for Brier differences. A negative difference means lower candidate loss; it is not an absolute value.

These bounds may be wide. That is an honest representation of unidentified outcomes, not a defect to be hidden by imputation.

### 3.4 Inverse-probability estimates (assumption-dependent)

If every eligible case has a predeclared, strictly positive inclusion propensity, report the Horvitz–Thompson mean:

[
widehatmu_{HT}=rac{1}{N}sum_irac{R_iS_i}{pi_i}.
]

Also report the Hájek ratio form:

[
widehatmu_H=
rac{sum_i R_iS_i/pi_i}{sum_i R_i/pi_i}.
]

Required diagnostics:
- propensity model/profile identifier;
- minimum and maximum inclusion probability;
- maximum inverse-probability weight;
- observed-weight effective sample size ((sum w_i)^2/sum w_i^2);
- eligible/observed counts by stratum;
- explicit support/positivity failures.

The evaluator may compute these arithmetic quantities; it cannot prove that the propensity model is correct or that missingness is conditionally ignorable. The report must state the identifying assumption. Do not silently clip weights. Any future stabilized/truncated estimator must be separately named, predeclared and reported beside the untruncated estimator.

If propensity values are absent or any eligible case has (pi_i=0), do not emit an IPW point estimate. Retain the complete-case result, diagnostics and assumption-light bounds.

### 3.5 MNAR sensitivity analysis

Provide a pattern-mixture sensitivity interface that evaluates only caller-supplied deltas. A delta represents the assumed difference between the mean missing-case score and the observed-case mean, after clipping to the declared ([a,b]) score range.

For a fixed delta (delta):

[
ar S_{mis}(delta)=operatorname{clip}(ar S_{obs}+delta,a,b)
]

[
mu(delta)=
rac{sum_iR_iS_i+(N-n_{obs})ar S_{mis}(delta)}{N}.
]

The function must evaluate the supplied delta grid without selecting a preferred delta, fitting a hidden delta, or optimizing on the evaluation data. If no outcomes are observed, the observed-mean-anchored sensitivity model is unavailable; report only the assumption-light bounds.

This is a sensitivity model, not identification of MNAR outcomes.

## 4. Stratification and representativeness

The target frame may declare coarse strata such as:
- broad geography or coarse zone;
- access or mobility class;
- hazard severity band;
- observation-network or infrastructure availability;
- event/system family.

No person ID, device ID, GPS trajectory, household identifier or persistent individual movement record is accepted by this package.

For each stratum, report eligible and observed denominators, coverage, observed mean when defined, missingness reasons and no-assumption bounds. A stratum with no verified outcomes has no complete-case mean; do not substitute the global mean.

Strata and their boundaries must be predeclared for confirmatory evaluation. Do not use post-outcome labels to define a favorable subgroup.

## 5. Paired model comparisons

Candidate and reference comparisons require:
- the same eligible case IDs;
- the same observed outcome;
- the same as-of cutoff;
- the same verification source and version;
- the same score definition and bounds;
- the same independent-system grouping.

The primary paired estimand is candidate score minus reference score. A difference computed on unmatched cases is rejected, not silently reindexed or averaged independently.

The package reports ascertainment and bounds; uncertainty intervals that depend on event/system sampling must use the declared independent-system bootstrap family in the forecast-qualification programme. Cases and events nested inside a shared system must not be treated as independent clusters by default.

## 6. Known-truth DGP programme

Confirmatory DGPs should include:
1. MCAR verification with constant inclusion probability.
2. MAR selection driven by observed pre-outcome strata with a correctly specified propensity.
3. MAR with a misspecified propensity model.
4. MNAR selection directly dependent on the unobserved score or outcome.
5. Severity-dependent selection with high-impact cases under-observed.
6. Access, remoteness, mobility and infrastructure-dependent selection.
7. Rare events with differential outcome ascertainment.
8. Clustered systems with shared observation-network outages.
9. Delayed verification arriving after an as-of cutoff.
10. Positivity failure and near-zero inclusion probabilities.
11. An entire stratum with no observed outcomes.
12. A paired candidate/reference ranking reversal induced by selection.

Every DGP must have a separately coded population oracle. Simulation truth must not be computed from the same observed sample used to estimate the result.

## 7. Negative controls and invariants

- Missing records cannot become negative outcomes or zero scores.
- Observations recorded after the as-of cutoff cannot enter the snapshot.
- Each eligible case is counted exactly once.
- Duplicate case IDs and invalid parent/system references are rejected.
- Invalid score bounds, non-finite scores and out-of-range scores are rejected.
- Zero inclusion probability or incomplete propensity coverage disables IPW rather than being repaired silently.
- A zero-observation stratum returns no point estimate.
- Sensitivity outputs are a pure function of the frozen data and supplied delta list.
- Case order does not affect results.
- Paired candidate/reference case and source/version mismatches are rejected.
- Aggregate mobile-population data do not become individual tracking inputs.

## 8. Proposed qualification targets (not standards or achieved results)

| Dimension | Research target |
|---|---|
| Frame accounting | 100% of eligible cases observed or assigned one typed missing status |
| Missingness semantics | Zero implicit missing-to-negative or missing-to-zero conversion |
| Partial identification | Exact analytical finite-frame bound reproduction |
| IPW arithmetic | Exact HT/Hájek oracle reproduction under declared positive propensities |
| Positivity | 100% explicit refusal when a required inclusion propensity is absent or zero |
| Sensitivity | Exact caller-grid evaluation; no automatic delta selection |
| Subgroups | 100% explicit denominators and no point estimate for zero-observation strata |
| Historical integrity | Zero inclusion of post-cutoff observations |
| Privacy | No individual IDs, device IDs or precise movement traces |
| Paired fairness | 100% matched eligible IDs, outcome source/version and score contract |

These are proposed research gates. They are not official WMO standards or achieved results.

## 9. Comparison with existing practice and intended contribution

The programme complements rather than replaces:
- WMO/ECMWF forecast-verification guidance and proper scores;
- WeatherBench 2's common-source and scorecard approach;
- operational verification research on incomplete, uncertain and spatially uneven observations;
- mobile-population reach accounting;
- bitemporal evidence reconstruction;
- cluster-aware paired-score uncertainty.

Its intended contribution is an auditable distinction between **forecast performance on observed cases**, **assumption-dependent population estimates**, and **what remains unidentified without assumptions**. That distinction can be independently replayed by institutions that do not share a single operator or verification database.

## 10. Implementation boundary

Stage B is a dependency-free Rust library and deterministic CLI. It should expose validated input types, frame coverage, complete-case summaries, sharp bounds, HT/Hájek diagnostics, caller-supplied sensitivity grids and stratum reports.

No network service, live observation adapter, model selector, warning authority, user-tracking feature, causal impact estimator or operational policy threshold is introduced. The package is standalone and does not alter the existing v0.6 integration or its eight gates.

## 11. Research basis

- Brown et al., *Challenges of Operational Weather Forecast Verification and Evaluation*, Bulletin of the American Meteorological Society (2024): https://journals.ametsoc.org/view/journals/bams/105/4/BAMS-D-22-0257.1.xml
- ECMWF, *Behind every forecast: millions of observations we trust* (2 October 2026): https://www.ecmwf.int/en/about/media-centre/focus/2026/behind-every-forecast-millions-observations
- Caron et al., *On the Reliability of Surface Observations and the Pitfalls of Verification Against Own Analyses*, Meteorological Applications (2025): https://rmets.onlinelibrary.wiley.com/doi/10.1002/met.70129
- Manski, *Partial identification with missing data: concepts and findings* (2005): https://www.sciencedirect.com/science/article/pii/S0888613X04001124

## 12. Non-claims

No real forecast superiority, real population representativeness, unbiased real-world estimator, MNAR identification, causal warning-effect estimate, operational readiness, legal compliance or humanitarian impact is claimed.