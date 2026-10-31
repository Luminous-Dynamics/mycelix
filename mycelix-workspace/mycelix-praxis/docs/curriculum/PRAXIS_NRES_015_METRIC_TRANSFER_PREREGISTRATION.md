# PRAXIS-NRES-015: Loss-aware transport geometry and bulk-tail ambiguity — preregistration protocol

**Status:** research protocol proposal for review; no learner, ambiguity set or confirmatory result is claimed by this document.  
**Prerequisite baselines:** NRES-012 Bernoulli TV decisions, NRES-013 finite-support weighted-Hamming Wasserstein decisions, NRES-014 block split-conformal radius calibration.  
**Issue:** https://github.com/Luminous-Dynamics/mycelix/issues/5559.

## 1. Research question

Can a decision-loss-aware ambiguity geometry, calibrated on a genuinely separate source-family split, improve decisions on independent shifted service-system scenarios without merely making its training ambiguity bound look better?

The problem is not solved by minimizing a robust bound over the same cost profiles and cases used for evaluation. NRES-015 must distinguish:
- forecast quality;
- ambiguity-set validity and calibration;
- policy selection under the ambiguity set;
- realized decision cost and regret;
- transfer to new cost profiles, source families, institutions and shift families.

NRES-013 provides a deterministic finite-support W1 evaluator. NRES-014 provides a split-conformal block empirical-distribution radius. Neither learns a loss-aware transport geometry. This protocol establishes the split and comparison rules before that learner is built.

## 2. Research frontier and honest comparison

The method comparisons must account for the actual 2026 frontier:
- **Yamao et al., UAI 2026:** robust decision-focused learning under observation error and deployment shift, with worst-case regret and Wasserstein ambiguity around conditional empirical distributions. https://proceedings.mlr.press/v337/yamao26a.html
- **Ohnemus et al., ICML 2026:** trainable loss-aware optimal-transport ambiguity sets using bilevel optimization and hypergradients. Their motivation explicitly includes excessive conservatism from constructing ambiguity independently of the downstream loss. https://proceedings.mlr.press/v306/ohnemus26a.html
- **Chen et al., ICML 2026:** bulk-calibrated credal ambiguity, separating a high-mass bulk region from tail contribution and warning that arbitrary contamination can make robust risk infinite or vacuous without boundedness/support assumptions. https://proceedings.mlr.press/v306/chen26e.html
- **Long et al., 2026:** a unified conformal/Wasserstein-DRO view that distinguishes quantile-level correction from value-space radius correction. https://arxiv.org/abs/2608.29789
- **Ramos et al., PMLR 2026:** calibration-conditional coverage laws and departures under clustered or dependent data. https://proceedings.mlr.press/v329/r-ramos26a.html
- **Shi et al., UAI 2026:** federated conformal calibration under joint shift using privacy-preserving score-distribution sketches and local importance-weighted calibration. https://proceedings.mlr.press/v337/shi26a.html

NRES-015 is not a reproduction of these papers. The first contribution should be a strict, common-protocol comparison on complete multivariate service-state forecasts, with source-family custody, held-out losses, explicit coverage targets and separate realized-decision metrics.

## 3. Non-negotiable split governance

The unit of partition is an independent **source family**, not an individual observation, time row, service, radius, action or random seed. Every row, forecast, derived trajectory, scenario variant, repeated exercise and synthetic child generated from one source family inherits its split assignment.

The five disjoint roles are:

1. **Training:** fit the forecaster and any loss-aware transport parameters. Outcomes may be used only under the frozen training objective.
2. **Validation:** choose model family, regularization, optimizer settings and hyperparameters. No calibration quantile or confirmatory metric is selected here.
3. **Calibration:** after model, service-state encoding, metric and score function are frozen, calculate the pre-registered block nonconformity scores and radius. Calibration outcomes cannot be used to retune the model or metric.
4. **Public development:** open examples, evaluator debugging, arithmetic oracles and contract tests. It is never confirmatory and never merged into calibration or hidden challenge custody.
5. **Confirmatory hidden:** held by a custodian independent of the model author. Only frozen model outputs, evaluator reports and authorized aggregate results leave custody. Hidden outcomes never enter the public Git repository.

A random seed is not an independent source family. A repeated radius, multiple cost profiles, or all 16 states from one scenario do not multiply the independent sample count. The protocol records source-family count separately from row count, block count, service count and seed count.

### 3.1 Custody and tamper evidence

Before training starts, the custodian fixes the source-family-to-split manifest and retains the mapping from opaque random family IDs to source provenance. Public artifacts must not reveal natural-person identifiers or low-entropy hashes of private identifiers.

For every split, publish a manifest commitment containing:
- schema and manifest version;
- opaque family IDs or salted commitment references;
- family count and declared block-size/horizon rules;
- canonicalization rules;
- SHA-256 digest of each canonical source envelope and a Merkle root;
- custodian identity/role, signing-key fingerprint and detached signature;
- freeze timestamp and protocol version.

A digest or signature proves only that a committed byte sequence was not silently changed after commitment and identifies the signing key. It does not prove source truth, completeness, independence, legal authority or honest sampling. The custodian's attestation and audit procedure are separate evidence.

For hidden challenge execution, record model binary/source digest, parameters, metric, calibration artifact, evaluator version, analysis-plan digest and exact input-manifest commitment. No hidden outcome may be used to select a new checkpoint, radius, metric, cost profile, subgroup, metric weight or stopping point. A changed method requires a new preregistration and a fresh untouched confirmatory set.

## 4. Candidate method families

Evaluate on the same source families, complete forecast distributions, outcome masks, feasible actions and declared cost profiles:

1. **Climatology / frequency / constant policies:** transparent statistical and action baselines.
2. **Nominal joint-distribution decisions:** optimize expected loss under the forecast distribution.
3. **NRES-012 Bernoulli TV:** include only when the selected binary projection is explicitly defined; do not imply it preserves the complete joint state.
4. **NRES-013 fixed-metric W1:** exact finite-support weighted-Hamming DRO, with a fixed radius sensitivity sweep.
5. **NRES-014 split-conformal W1:** the same fixed metric, with radius calibrated on a separate block-level calibration split.
6. **Loss-aware trainable OT:** learn the transport geometry on training families using training loss profiles; select all tuning on validation families; freeze before calibration.
7. **Bulk-calibrated credal ambiguity:** separately represent high-mass bulk contamination and bounded tail contribution.
8. **CVaR / expected shortfall:** evaluate as a separate tail-risk decision objective, not as a disguised ambiguity-set calibration.
9. **Perfect-information reference:** report only as an infeasible/statewise oracle lower bound, never as a deployable candidate.

Worst-case expected loss and minimax regret remain separate objectives for every applicable ambiguity family. No method may select the metric on its confirmatory regret results.

## 5. Metric-learning constraints

A learned transport cost must remain a valid, interpretable metric on the declared finite state space.

Initial permissible families:
- weighted Hamming with a finite strictly-positive weight floor, finite upper bounds, a frozen state encoding and a fixed normalization convention;
- shortest-path distance on a connected, undirected graph with strictly-positive edge lengths.

Any more general candidate must be checked for:
- finite, non-negative values;
- identity of indiscernibles;
- symmetry;
- triangle inequality;
- connectedness where a graph representation is used;
- fixed support and service-bit semantics;
- radius normalization and unit compatibility.

The weighted-Hamming weights must be normalized to a pre-registered total diameter or another frozen scale. Otherwise a radius comparison can change merely because the metric was multiplied by a constant.

Metric learning must be assessed on more than one training loss profile. At least two decision profiles must remain fully unseen until final evaluation. The held-out profiles should include both different resource/avoidance trade-offs and a different service-cost ranking. Training and held-out profiles must be specified before results are inspected.

A learned metric's value must be judged by both:
- changes in selected actions and realized regret; and
- validity/coverage of the ambiguity set.

A lower worst-case regret upper bound alone is not sufficient: the metric can shrink ambiguity in regions that flatter the training loss while failing to cover or transfer to unseen losses. Report metric weight sensitivity and profile-specific failures.

## 6. Bulk-tail credal comparison

A bulk-tail model must state:
- how the high-mass bulk set is selected and frozen;
- the contamination allowance inside the bulk;
- how the remaining tail mass or tail loss is bounded;
- whether support or loss boundedness is assumed;
- the resulting worst-case upper expectation and when it becomes infinite or vacuous.

Arbitrary contamination with unbounded loss must not be silently clipped. If no defensible tail bound exists, report an infinite or vacuous robust objective. Compare this method with W1, conformal W1 and CVaR at the same tail mass / loss profile where meaningful.

## 7. Calibration and dependence

Forecast calibration and ambiguity coverage are distinct:
- Brier/log/joint proper scores assess forecast probabilities.
- NRES-014's block W1 score assesses distance between a block's empirical joint-state distribution and its frozen forecast.
- Any population-law correction needs an explicit within-block sampling/dependence assumption.

For clustered or dependent observations, rows within a source family are not iid calibration units. The confirmatory protocol must compare ordinary block split conformal with a declared dependence-aware method or reject the coverage claim. Do not use the NRES-014 conditional-iid correction unless its within-block assumption is defensible.

Report:
- calibration block count and unique source-family count;
- conformal order-statistic rank, finite/unbounded-radius events and score distribution;
- marginal, calibration-conditional and any population-law guarantees as separate statements;
- effective sample size under any importance weighting;
- service-wise and deployment-slice coverage with uncertainty intervals;
- coverage at client/institution level as well as pooled coverage for federated settings.

Federated score sketches require a stated privacy model. Quantization or aggregation alone is not a differential-privacy guarantee. If differential privacy or secure aggregation is used, publish the privacy/cryptographic assumptions and account for their effect on score and coverage error. A pooled average must not hide a badly calibrated client.

## 8. Shift challenge matrix

Confirmatory families should be independently custodied and include, at minimum:
- common-mode disruptions that move several service bits together;
- component-specific shifts;
- temporal and seasonal shifts;
- covariate and base-rate shifts;
- dependency-structure changes;
- evidence degradation, source conflict and missingness;
- clustered client/institution shift for federated calibration;
- loss-profile shift, where the evaluator changes the downstream cost profile without changing the forecast.

The split manifest is by source family before any scenario transformations. Every transformed or augmented copy inherits the source-family split. Shift severity, seed, service, action and profile are repeated measures, not independent samples.

## 9. Required reporting — no composite score

Per method × split × shift × profile, report separately:
- joint and marginal proper forecast scores;
- ambiguity block-score coverage, calibration-conditional diagnostics where available, radius distribution and vacuity rate;
- realized decision cost and statewise regret;
- worst-case expected-loss upper bound and worst-case regret upper bound;
- CVaR95 / expected shortfall where defined;
- action counts and policy disagreements;
- resource commitments and each non-compensatory service-floor result;
- abstention and reviewer-deferral cost;
- effective sample size and importance-weight concentration;
- confidence intervals clustered at independent source-family level.

Repeated radii, action alternatives, forecast horizons and state masks from the same source family must not be bootstrapped as independent units. Use paired cluster bootstrap or another preregistered method at the independent source-family level. Adjust for multiple confirmatory comparisons or designate one primary estimand and a strict hierarchy of secondary estimands.

## 10. Proposed confirmatory targets (targets, not current results)

Primary target:
- at least 5% lower mean realized statewise decision regret than the strongest pre-registered fixed-metric NRES-013/NRES-014 baseline on hidden shift families, with a paired 95% confidence interval excluding zero.

Guardrails:
- no more than 2% degradation in nominal in-distribution realized cost;
- 100% metric validity and split-family disjointness;
- no silent resource or critical-service-floor violations;
- separate calibration coverage and forecast probability calibration;
- published vacuity, abstention, infinite-risk and non-improvement rates;
- all profiles and shift families reported, including negative results.

These targets are not SOTA claims. Sample size and statistical power must be justified before custody is opened. If the primary effect is not resolved, report it as inconclusive rather than changing the primary loss, subgroup, radius or metric after seeing the results.

## 11. Initial public pilot and current warning

A small exploratory grid over 81 positive weighted-Hamming vectors on the eight public NRES-009 cases did not identify a transferable metric improvement after the dual implementation was corrected. The original exploratory numbers were withdrawn because their first breakpoint implementation was incorrect. The corrected grid selected the existing fixed metric when tuned on the low- and moderate-cost profiles. A profile-specific metric found by selecting directly against the purported high-cost evaluation outcomes is leakage, not evidence of generalization.

This is not a failed research programme. It demonstrates why NRES-015 must keep metric training, cost-profile transfer and confirmatory outcomes separated, and why numerical oracles need independent formulations.

## 12. Standards and industry composition

Praxis should compose with, not replace:
- FEMA HSEEP's exercise program management, design/conduct/evaluation and improvement-planning cycle;
- ISO 22301 business-continuity management system requirements;
- WHO's 2026 public-health EOC framework, including governance, plans, information systems/data standards, infrastructure/technology and workforce/surge capacity;
- IFRC preparedness and locally-led accountability practices.

NRES-015's proposed contribution is a reproducible quantitative layer for comparing uncertainty and decision consequences while retaining evidence custody and disaggregated operational constraints. No standards certification, institutional endorsement, legal compliance or replacement claim is made.

## 13. Reproducibility and status

This PR contains a research protocol, schema and conformance requirements only. It does not contain a learned metric, bulk-tail optimizer, hidden challenge data, trained model or SOTA result. Public NRES fixtures remain synthetic, public-development-only and inactive in the live curriculum.
