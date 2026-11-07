# Robust decision-focused adaptive soil experiment design v0.1

**Status:** research specification. No experiment, field trial, application, procurement or material-use authorization is conferred. All numerical gates are proposed research targets.

## 1. Paradigm shift: choose measurements for decisions, not model entropy

A soil research programme should not only predict outcomes. It should decide which measurement or experiment is most valuable next, subject to experimental design, cost, representativeness, physical identifiability and governance constraints.

A naive active-learning policy chooses the point with greatest expected information gain under its current model. This can create a feedback loop: the model samples where it expects information, the dataset shifts toward the model's own assumptions, and poorly modelled but decision-relevant soil or forcing regimes become underrepresented.

The new target is a **robust, representative, decision-focused value-of-information policy**. It asks: which feasible next action is expected to reduce uncertainty in a predeclared service decision, while preserving coverage of the target population and exposing—not hiding—model misspecification?

## 2. Recent evidence and why the naive loop is inadequate

Tang, Sloman and Kaski (AISTATS 2026) analyze Bayesian active learning under model misspecification. Their work shows that informativeness-only acquisition can create covariate shift and amplify generalization error. They introduce R-IDeA, balancing representativeness, informativeness and error de-amplification. This is a particularly relevant challenge for soil models trained on heterogeneous sites, batches and hydrologic regimes. https://proceedings.mlr.press/v300/tang26d.html

Yang et al. (2026) propose Bayesian experimental design for model-discrepancy calibration using an auto-differentiable ensemble Kalman inversion approach. The important lesson for Symtropy is that design should account for model discrepancy rather than treat the chosen physical model as exact. https://doi.org/10.1016/j.jcp.2025.114469

A 2026 review of on-farm experimentation describes operational datasets with spatial heterogeneity, sparse/noisy observations, limited replication and temporal variation; it discusses mixed, Bayesian spatial and permutation-based methods for field analytics. The design and analysis must reflect field structure rather than assume independent, identically distributed plot observations. https://doi.org/10.1080/1343943X.2026.2664854

Bayesian optimization has shown substantial experiment-efficiency gains in other domains, including a 2025 cell-culture-media study reporting 3–30 times fewer experiments than estimated for selected conventional methods. This is evidence that iterative experimentation can be useful in its studied contexts—not an expected reduction for soils or an apples-to-apples soil benchmark. https://www.nature.com/articles/s41467-025-61113-5

## 3. Existing work this connects

- #5513: evidence custody, grouped validation, source lineage and the Terra Preta research protocol.
- #5528: hydraulic state and wetting/drying hysteresis.
- #5535: treatment assignment and causal contrasts.
- #5536: known-truth preferential-flow and parameter-identifiability scenarios.
- #5580: root/pore connectivity mechanisms.
- #5586: function-conditioned hydraulic service envelopes.
- #5598: dose × root × wet–dry response surfaces.

This benchmark decides what to measure next to discriminate among the mechanisms and service outcomes in those workstreams.

## 4. Candidate measurement actions

An action can be a new independent experimental unit, an additional dose in an approved design, a new site or soil source, an additional material batch, a CT scan, a dye section, a tracer sample, a depth sensor, a water-retention point, a gas-permeability measurement, a wet–dry cycle, or a follow-up measurement after aging.

Each action must declare:
- expected monetary cost, elapsed time, equipment/throughput and sample requirement;
- the population/domain it represents and the strata it does not represent;
- which parameter, contrast, mechanism or service decision it can inform;
- independent assignment unit and nested measurement structure;
- expected information gain under each candidate model, not just the selected model;
- feasibility and required independent approval;
- data quality, detection limits, calibration and expected missingness.

An action is infeasible if it fails the relevant safety, governance, permission, laboratory or protocol prerequisite. Such actions are excluded before ranking; they do not receive a high score because of a hypothetical reward.

## 5. Robust acquisition objective

Evaluate candidate policies across an ensemble of plausible models and misspecification scenarios. A policy should consider:

1. **Decision-relevant information:** expected reduction in posterior expected loss or service-decision uncertainty for the preregistered endpoint and service profile.
2. **Representativeness:** coverage of the intended distribution across soil texture/mineralogy, climate, root state, forcing, antecedent moisture, management, material batch and site.
3. **Discrimination:** ability to distinguish competing mechanisms, including matrix versus active macropore flow, root-channel occupation, exchange limitation, cracking and hysteresis.
4. **Model-discrepancy exposure:** probability that models disagree, residual structure remains, or the candidate action falls outside calibrated support.
5. **Replication and precision:** the action must preserve the actual randomized unit and support a defensible precision or power target.
6. **Cost and delay:** compare cost per reduction in decision-relevant uncertainty, not information per sample alone.

Use hard constraints for protocol feasibility, assignment-unit support, coverage floors and independent replication. Do not use safety or regulatory requirements as soft penalties that a sufficiently high information score can override.

A candidate robust objective can be preregistered as a weighted, constrained multi-criterion policy, but the weights must be justified and sensitivity-tested. Report the underlying criteria separately; do not publish only a composite utility.

## 6. Comparator policies

Under the same budget, action constraints, initial data and stopping rules, compare:

- random feasible sampling;
- expert-fixed measurement schedule;
- one-factor-at-a-time;
- classical factorial / response-surface design;
- D-optimal or I-optimal design for a declared model;
- Bayesian expected information gain;
- decision-focused value of information;
- representative + informative + error-de-amplifying policy;
- Symthaea policy combining model ensemble disagreement, service-decision uncertainty and physical identifiability.

Every policy must use identical cost definitions and observation noise. Candidate models are evaluated on the same external target distribution. A policy cannot be rewarded merely for collecting more observations or for shifting the test distribution toward its own selected points.

## 7. Known-truth and adversarial test programme

Build synthetic worlds with known process truth but realistic observation operators:
- true linear, saturating, thresholded, U-shaped and inverted-U responses;
- dose × root and dose × wet–dry interactions;
- unmodelled third mechanism that reverses a response under a subset of soils;
- soil/source shift and missing soil strata;
- material-batch drift correlated with selected doses;
- biased sensor calibration, missing-not-at-random observations and unequal measurement costs;
- identical aggregate outputs generated by distinct latent mechanisms;
- measurement actions that reduce model entropy but worsen external representativeness;
- null effects and negative-control strata;
- hard-to-identify parameters and observationally equivalent models.

The generator must keep latent truth separate from model-visible observations. Split evaluation by scenario family and target-population stratum; do not allow the same generator seed or latent mechanism family to leak across train and confirmatory test.

## 8. Metrics and proposed qualification targets

Targets (not results):

- **Decision regret:** lower cumulative regret than random, expert-fixed, one-factor-at-a-time and informativeness-only policies at equal monetary and time budgets.
- **Representativeness:** no target stratum may silently fall below its predeclared minimum coverage; report target-vs-selected distribution divergence and effective sample size.
- **De-amplification:** misspecification stress tests must not show systematic external error inflation versus the fixed/random baseline; report worst-stratum harm.
- **Mechanism discrimination:** high-signal known-truth worlds should separate competing mechanisms or return an explicit unidentifiable status.
- **Calibration:** nominal 90% predictive intervals should achieve 85–95% coverage on independent target groups with width and proper scores.
- **Budget efficiency:** target at least 20% lower decision-relevant uncertainty or regret per equal budget than the strongest feasible fixed-design comparator, without worse worst-stratum calibration. The 20% is a proposed target, not an expected result.
- **Reproducibility:** 100% of selected actions have an immutable input digest, candidate-policy version, declared cost, population coverage calculation, reason code and approval status.
- **No replication laundering:** nested observations must not increment the randomized-unit count.

Also report failure rate, abstention rate, cost distribution, wall-clock time, coverage by site/soil/batch/forcing, model disagreement and decision stability under objective-weight sensitivity.

## 9. Industry comparison and proposed differentiator

Bayesian optimization and active learning already provide strong iterative experiment-selection methods. Symthaea should not claim novelty merely for selecting the next point with a Gaussian process or information-gain score.

The differentiation to test is a coupled research loop:
- Mycelix maintains source, batch, treatment, experimental-unit and approval provenance.
- Symtropy supplies physical predictions, conservation checks and parameter-identifiability diagnostics.
- Symthaea maintains competing hypotheses, tracks uncertainty and model discrepancy, and selects representative experiments that discriminate mechanisms or improve a named service decision.
- Independent reviewers control experimental authority and confirmatory analysis.

The advantage must be measured against strong active-learning and design-of-experiments baselines on equal budgets and held-out target distributions. Until then it is a hypothesis, not a SOTA claim.

## 10. Execution sequence

1. Implement a machine-readable experiment-action and acquisition-receipt contract.
2. Build known-truth worlds where information-gain-only acquisition fails under misspecification and where representative/de-amplifying choices preserve external performance.
3. Run fixed-policy, random, OFAT, factorial, optimal-design, Bayesian EIG and decision-VOI baselines under matched budgets.
4. Use #5598 response surfaces and #5536 transport truth to generate decision-focused mechanism-discrimination tasks.
5. Freeze the target population and grouped test split before policy training or tuning.
6. Reproduce an empirical case only after datasets, source licenses, design units, measurement costs and target population are audited.
7. Keep experiment suggestions separate from field authorization and deployment.

## 11. Boundaries

This is a research planning and benchmark specification only. It does not authorize soil amendments, field trials, application rates, collection of samples, material handling or commercial release. Model-selected experiments require independent scientific, landholder, institutional, safety and regulatory approval. Local South African transfer requires representative local data and qualified review.
