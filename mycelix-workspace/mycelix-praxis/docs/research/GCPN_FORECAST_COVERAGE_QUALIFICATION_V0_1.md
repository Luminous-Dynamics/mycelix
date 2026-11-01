# GCPN Known-DGP Forecast Interval Qualification v0.1

**Status:** research specification and machine-readable design contract; not yet an executable qualification result.  
**Parent issue:** #5644.  
**Stacked on:** integrated GCPN evaluation v0.6, branch `research/gcpn-integrated-evaluation-v0-6`.  
**Scope:** offline synthetic probabilistic-forecast comparison and statistical-method qualification.

## 1. Why v0.1 needs a qualification layer

The existing `gcpn-forecast-uncertainty` component demonstrates deterministic paired percentile-bootstrap computation on 12 complete synthetic event IDs. It correctly keeps candidate, climatology and lead-reference scores paired, and explicitly says that `event_id` is not a validated meteorological dependence cluster.

That is a useful regression oracle, but it does not answer the next scientific question: **does the interval procedure achieve its advertised coverage under the data-generating process and dependence structure that produced the forecasts?**

A fixed seed and 20,000 bootstrap draws establish reproducibility of an algorithmic output. They do not establish that its 95% interval covers the target parameter 95% of the time. Coverage is a property of the full inferential procedure across repeated datasets, and must be measured under known population truths.

This specification makes that distinction testable.

## 2. Industry and research alignment

### Existing practice to preserve

- ECMWF's probabilistic verification guidance distinguishes forecast validation from verification, and accuracy, skill and utility. It discusses reliability, sharpness and the need for adequate samples; a sharp forecast is not automatically a better forecast if it is unreliable. GCPN must retain proper scores, reference forecasts, calibration diagnostics and decomposition rather than replace them with an interval.
- WMO's 2025 hydrological forecast verification guidelines describe accuracy, bias, reliability, resolution and sharpness as distinct properties. GCPN's binary Brier track is a starting point, not a complete probabilistic verification framework.
- Meteorological forecast uncertainty literature has long warned that forecast-observation pairs can be correlated in time and space. Treating correlated pairs as IID can overstate effective sample size and understate uncertainty. Matched forecast comparisons should use identical cases, with a resampling design that preserves dependence.
- Weather-forecast studies have used stationary/block bootstrap schemes for score uncertainty. The correct block or cluster is determined by the sampling design, not by whichever identifier happens to be present in a CSV.
- Cluster-robust inference research shows that small cluster counts and unequal cluster sizes can materially change finite-sample performance. Methods developed for regression must be re-derived or justified before being applied to paired proper-score differences.

### What GCPN can contribute

The opportunity is **not a new weather model**. It is a public, deterministic, reproducible qualification harness that makes the assumptions behind forecast-comparison uncertainty inspectable:

1. the population estimand is explicit;
2. model comparisons use the same held-out cases;
3. the true dependence hierarchy is known in synthetic trials;
4. resampling units preserve that hierarchy;
5. empirical coverage and Monte Carlo uncertainty are measured;
6. methods that fail under particular regimes are reported as failed or unsupported;
7. every result is reproducible from a pinned design, seed and method version.

That would complement forecast centres' verification practice. It would not establish forecast superiority or operational readiness.

## 3. Statistical contract: define the estimand before resampling

For each forecast case (i), retain the matched outcome (y_i) and the candidate/reference forecasts (p_{Ai}), (p_{Bi}). For binary Brier loss, define the paired case difference as:

`d_i = (p_Ai - y_i)^2 - (p_Bi - y_i)^2`.

Negative values favour candidate A; positive values favour reference B. Form the paired difference case-by-case before any aggregation or resampling.

### Primary estimand: forecast-case-weighted difference

Let `g = 1,...,G` index independent top-level sampling units (for example, independently defined storm-system blocks), `n_g` be the number of eligible matched forecast cases in unit `g`, and `D_g = sum(i in g) d_i`. The primary target is:

`theta_case = E[D_g] / E[n_g]`.

The sample estimator is the ratio:

`theta_hat_case = sum_g D_g / sum_g n_g`.

A cluster bootstrap replicate samples complete top-level units and recomputes this ratio. It must not average cluster means unless the estimand is explicitly the cluster-weighted quantity.

### Secondary estimand: cluster-weighted difference

`theta_cluster = E[D_g / n_g]`.

This answers a different question: the average independent system's mean score difference, giving small and large systems equal weight. When cluster size is informative about hazard type or score difference, the two estimands need not agree. Report them under distinct names; never silently substitute one for the other.

### Pairing and held-out evidence

All compared models must be scored on identical eligible forecast cases and the same independently established outcome vintage. Preserve issue time, valid time, lead, hazard, region, event and dependence identifiers. Calibration/training, tuning, validation and confirmatory evaluation remain distinct. Forecasts must be issued before the verification outcome is available.

A paired interval is not valid merely because two models have similar row counts. It requires exact case matching, common outcome semantics and a disclosed missing-case policy.

## 4. Dependence hierarchy and sampling unit

The synthetic record hierarchy is:

[
	ext{case} ightarrow 	ext{event_id} ightarrow
	ext{dependence_cluster_id} ightarrow
	ext{independent sampling block}.
]

Not every dataset will need all four levels. The DGP must make the levels explicit and provide an oracle identifier for the true independent top-level unit.

- **Case:** one forecast/observation pair, such as one location, lead, threshold and valid time.
- **Event:** an event label that may collect multiple cases.
- **Dependence cluster:** a storm, synoptic system, linked hazard episode or other source-defined common-cause grouping.
- **Independent sampling block:** the highest-level unit assumed independent for the target population. If even these blocks are serially dependent, a block method over their declared order is required.

The v0.1 source fixture only has `event_id`. It must remain labelled as such. A real forecast archive needs an independently reviewed cluster-construction protocol, version, provenance and uncertainty. Spatiotemporal proximity may be an input to clustering, not proof that a grouping is meteorologically correct.

### Resampling controls

1. IID case bootstrap is a negative control when cases are dependent.
2. Event-ID bootstrap is a negative control when multiple event IDs share a higher-level system.
3. Independent-top-level-unit bootstrap is the primary candidate under independent top-level units.
4. Stationary/circular block bootstrap is a candidate when top-level units remain serially dependent and the process is sufficiently stationary within the stated evaluation window.

A resample must never split a top-level unit into unrelated rows. For the primary case-weighted estimand, a replicate samples (G) complete independent units with replacement and recomputes (sum D_g / sum n_g).

Known regime breaks are not to be bootstrapped as though one stationary population generated the entire record. Evaluate predeclared regimes or target windows separately.

## 5. Known data-generating-process (DGP) design

The machine-readable profile freezes a scenario matrix. The confirmatory implementation should include these families.

| Family | Construction | Failure it is designed to expose |
|---|---|---|
| IID baseline | Independent forecast cases and outcomes | Implementation baseline; confirms no unnecessary penalty when independence is true |
| Nested storm systems | Multiple event IDs and cases share a latent system state | Event-ID and row resampling understate uncertainty |
| Unequal cluster sizes | Balanced, moderate and heavy-tailed case counts | Size-weighting errors and unstable small-cluster inference |
| Informative cluster size | Cluster size correlated with latent hazard/score regime | Confusion between case-weighted and cluster-weighted estimands |
| Null heterogeneous skill | Opposite signed system-level score effects with known population mean zero | False superiority under heterogeneity |
| Better/worse alternatives | Known nonzero candidate-minus-reference expected score | Power and bias under controlled effect sizes |
| Rare events | Binary base rates such as 1%, 5% and 20% | Sparse events, boundary forecasts and degenerate resamples |
| Within-system dependence | Shared system latent risk and event effects | Incorrect effective sample size |
| Ordered serial dependence | Markov/AR-like state persistence across top-level units | Ordinary cluster bootstrap versus stationary block bootstrap |
| Regime shift | Abrupt or gradual change in the DGP | Invalid pooling across nonstationary periods |
| Integrity mutation | Missing, delayed, duplicate, conflicting, invalid or contaminated outcomes | Data-quality errors being mislabeled as sampling uncertainty |

### Analytic population oracle

For a binary outcome with `Y ~ Bernoulli(q)`, the expected Brier loss for forecast probability `p` is:

`E[(p - Y)^2 | q] = (p - q)^2 + q(1 - q)`.

For a known DGP state `z`, the conditional expected paired Brier difference is:

`Delta(z) = (p_A(z) - q(z))^2 - (p_B(z) - q(z))^2`.

The DGP computes the exact case-weighted population target using its declared system-state and cluster-size probabilities:

`theta_case = E[n_g * Delta_g] / E[n_g]`.

This makes informative cluster-size experiments analytically auditable. The cluster-weighted target is `E[Delta_g]` and is separately checked. Simulation output must not be used as its own truth oracle.

## 6. Candidate confidence-interval methods

The first implementation should compare methods by assumption, not produce one universal winner.

| Method | Role in qualification | Limitation to test |
|---|---|---|
| IID case percentile bootstrap | Negative control | Fails when dependence is present |
| Event-cluster percentile bootstrap | Negative control / current v0.1 bridge | Fails when events share a larger dependence cluster |
| Independent-unit paired percentile bootstrap | Primary candidate under independent top-level units | Small sample, imbalance and skewness |
| Independent-unit basic bootstrap | Alternative interval construction | Tail and bias behavior |
| Independent-unit studentized bootstrap | Alternative where variance estimates are stable | Nested computation and small-cluster variance quality |
| Independent-unit BCa bootstrap | Alternative where jackknife acceleration is estimable | Discrete/degenerate samples and too few independent units |
| Stationary/circular block bootstrap | Candidate for ordered serial dependence | Block-length sensitivity and stationarity assumptions |
| Wild-cluster bootstrap-t | Research candidate only | Requires a justified score-difference formulation; regression results cannot be imported by label |

Every method must identify the estimand, resampling unit, interval convention, variance/acceleration method, assumptions and unsupported regimes in its output.

No method gets to report an interval as qualified simply because it ran. The suite measures when the interval covers the known truth.

## 7. Qualification measurements and Monte Carlo precision

For each scenario, sample-size regime, method and nominal level, report:

- empirical coverage at 90%, 95% and 99%;
- Monte Carlo standard error (MCSE) and a binomial confidence interval for empirical coverage;
- lower-tail and upper-tail miss rates separately;
- mean and median interval width;
- point-estimator bias and RMSE;
- invalid/non-finite interval and abstention rates;
- Type-I error under null DGPs;
- power under declared alternatives;
- false superiority and false inferiority declarations under an explicit decision rule;
- runtime, bootstrap replicate count and outer simulation count.

For `M` independent Monte Carlo datasets and estimated coverage `c_hat`, compute:

`MCSE(c_hat) = sqrt(c_hat * (1 - c_hat) / M)`.

At `M = 10,000` and coverage near 0.95, MCSE is approximately 0.00218 (0.218 percentage points). A 95% Monte Carlo interval around the measured coverage should accompany every reported coverage rate. The Monte Carlo interval is not the forecast-score confidence interval; these quantify two different sources of uncertainty.

Use separately seeded pilot and confirmatory phases. Freeze scenario parameters, RNG streams, interval methods and counts before the confirmatory run. A pilot may discover bugs; it cannot be relabelled as confirmatory evidence.

### Proposed targets (research hypotheses, not standards)

- Nominal 95% empirical coverage between 93% and 97% in each sufficiently replicated, predeclared confirmatory DGP cell, with MC uncertainty shown.
- Nominal 90% and 99% interval coverage evaluated independently, not inferred from 95% performance.
- Zero row-wise leakage across a declared cluster/block resample.
- Zero silent acceptance when the supplied independence identifier is missing or inconsistent with the synthetic oracle.
- 100% deterministic replay from the exact profile, seed, method version and source fingerprint.
- Explicit unsupported/inconclusive output for regimes where assumptions or sample size do not support a defensible interval.

These are internal qualification targets. They are not official WMO, ECMWF, ITU, UNDRR or SADC requirements, and passing synthetic cases is not operational certification.

## 8. Decision semantics and multiplicity

- A paired interval containing zero means that the data/design do not resolve the direction at that interval level. It is not proof of equivalence or proof of no effect.
- A superiority claim requires a predeclared comparison, valid dependence assumptions, a predeclared evaluation population and a stated error/multiplicity policy.
- Practical significance requires a domain-defined effect margin. Do not invent a single universal Brier-score margin across hazards, lead times and base rates.
- Multiple model/hazard/lead comparisons must be grouped in advance. Use simultaneous intervals or a justified multiplicity procedure; do not select only the favourable comparison after seeing results.
- Proper scoring rules, calibration/decomposition, uncertainty intervals, drift diagnostics and subgroup results remain distinct dimensions. No composite score is introduced.

## 9. Human outcomes and the warning evidence graph

Forecast verification remains only one link in the warning chain. UNDRR/WMO's 2025 global status material states that warnings must be received, understood, trusted and acted upon, and places local actors and communities at the centre of MHEWS design. The 2025 report records 119 countries (60%) reporting an MHEWS, while identifying persistent regional capability gaps and Africa's relatively low comprehensiveness.

Therefore the next human-outcomes work should not turn the current 1,800 invented trials into claims about people. It should proceed through community co-design, accessible message testing, independent ethics review and consented human-subject research. Measure at least: correct hazard/location/time understanding, action-intent recall, uncertainty interpretation, false certainty, accessible representation, trust calibration and response burden. Report intended-exposure denominators and subgroup distributions. Any numerical target remains provisional until the study population and use context are agreed.

### Proof-carrying warning evidence graph

A second, complementary research track should treat each warning as a typed, versioned evidence graph:

[
	ext{source observation} ightarrow 	ext{forecast evidence}
ightarrow 	ext{authority assertion} ightarrow 	ext{message version}
ightarrow 	ext{CAP exchange} ightarrow 	ext{channel receipt}
ightarrow 	ext{accessible representation} ightarrow 	ext{comprehension evidence}
ightarrow 	ext{authorized action} ightarrow 	ext{independently evaluated outcome}.
]

Every edge should carry provenance, time semantics, version, authority and uncertainty. The graph must preserve missingness, contradiction, supersession, revocation and late arrival as explicit states. A valid cryptographic receipt proves the receipt of particular bytes by an endpoint, not that a person understood a warning. A comprehension observation does not prove action; action does not prove impact or causality.

CAP 1.2 (ITU-T X.1303 bis) remains the interoperable alert message format. GCPN's evidence graph is a separate provenance/evaluation sidecar and must not redefine CAP or grant warning authority.

## 10. Staged evidence and implementation plan

| Stage | Work | Completion evidence |
|---|---|---|
| A — Specification | Estimand, hierarchy, DGPs, method matrix and metrics | This document and profile reviewed |
| B — Executable core | Isolated dependency-free Rust crate; deterministic synthetic DGP; paired scoring | Source review, exact oracle tests, deterministic JSON |
| C — Coverage qualification | Outer Monte Carlo simulations, coverage MCSE, interval comparisons | Confirmatory report with pinned seeds and raw summaries |
| D — Adversarial integrity | Under-clustering, row leakage, missing/duplicate/outcome contamination, size informativeness | Negative controls fail closed |
| E — Standalone review | Independent statistical review of target, clustering and method assumptions | Written review and unresolved-gaps register |
| F — Integration | Consider a narrow v0.7/module-8 integration | Only after B–E; maintain the eight-gate strict conjunction |

No runner or CI result is required to begin Stages A–B, but the exact Rust toolchain/build/test evidence remains mandatory before merge. This v0.1 specification does not claim that Rust compilation, tests, CI or coverage qualification has occurred.

## 11. Sources and comparison references

1. ECMWF, *Statistical Concepts — Probabilistic Data*, Forecast User Guide, updated 23 May 2026: https://confluence.ecmwf.int/spaces/FUG/pages/673551875/Section%2B12.B%2BStatistical%2BConcepts%2B-%2BProbabilistic%2BData
2. WMO, *Guidelines on the Verification of Hydrological Forecasts*, WMO-No. 1364, and overview (11 July 2025): https://public.wmo.int/resources/meteoworld/meteoworld-june-2025/guidelines-streamflow-forecast-verification
3. *Sampling Uncertainty and Confidence Intervals for the Brier Score and Brier Skill Score*, Weather and Forecasting (2007): https://journals.ametsoc.org/view/journals/wefo/23/5/2007waf2007049_1.xml
4. *Testing the Tests: What Are the Impacts of Incorrect Assumptions When Applying Confidence Intervals or Hypothesis Tests to Compare Competing Forecasts?*, Monthly Weather Review (2018): https://doi.org/10.1175/MWR-D-17-0295.1
5. MacKinnon, Nielsen & Webb, *Fast and reliable jackknife and bootstrap methods for cluster-robust inference*, Journal of Applied Econometrics (2023): https://onlinelibrary.wiley.com/doi/full/10.1002/jae.2969
6. Webb, *Reworking wild bootstrap-based inference for clustered errors*, Canadian Journal of Economics (2023): https://doi.org/10.1111/caje.12661
7. Waghmare & Ziegel, *Proper Scoring Rules for Estimation and Forecast Evaluation*, Annual Review of Statistics and Its Application (2026): https://www.annualreviews.org/content/journals/10.1146/annurev-statistics-042424-050626
8. UNDRR/WMO, *Global Status of Multi-Hazard Early Warning Systems 2025*: https://www.undrr.org/reports/global-status-mhews-2025
9. ITU-T, X.1303 bis, *Common alerting protocol (CAP 1.2)*: https://www.itu.int/itu-t/recommendations/rec.aspx?lang=en&rec=12150

## 12. Non-claims

This is a statistical qualification specification, not a meteorological forecasting model, operational early-warning service, CAP implementation, national MHEWS, alert authority, human-subject study, or evidence of lives saved. No synthetic benchmark result demonstrates real-world superiority. National warning authorities and meteorological services retain their mandates.
