# Terra Preta spectral screening and confirmatory referral bake-off v0.1

**Status:** synthetic known-truth benchmark specification. This is a measurement-workflow benchmark, not an agronomic recommendation, diagnosis, laboratory replacement or model result on real Highveld data.

## 1. Question

Given a low-cost NIR screening result for a soil sample, when should the workflow pay for confirmatory MIR or wet chemistry?

The wrong objective is: maximise one global spectral-model R².

The useful objective is: minimise total measurement cost and decision loss while meeting predeclared risk, coverage, calibration, capacity and applicability-domain constraints. A system should be allowed to say "screen insufficient — refer".

## 2. Operating pathway

`sample → NIR screen → property-specific uncertainty and applicability checks → route decision → (screen only | MIR | reference lab) → decision record`

A route is property-specific. A sample may be in-domain for pH(KCl) but out-of-domain for Bray-1 P. NIR, MIR and wet chemistry are distinct methods; their target semantics and uncertainty remain explicit.

## 3. Policies to compare

Under the same sample pool, property-specific screen predictions, threshold/loss profile, referral capacity and reference-test assumptions:

1. **NIR-only:** no referral.
2. **All-reference:** refer every sample (cost/quality upper reference).
3. **Random fixed-budget:** random referral at the available capacity with deterministic seed.
4. **Fixed uncertainty threshold:** refer when predictive standard uncertainty reaches a predeclared threshold.
5. **Uncertainty rank:** refer the highest predictive standard uncertainty until capacity is exhausted.
6. **Applicability-domain:** refer every out-of-domain sample first, then abstain / remain unqualified if capacity is exhausted.
7. **Decision-risk rank:** refer cases with the highest expected loss of the current threshold decision under a declared Gaussian predictive approximation and externally supplied false-positive / false-negative costs.
8. **Perfect-information bounded oracle:** evaluator-only hindsight comparator that refers the samples whose current NIR decision is actually wrong. This is not deployable; it bounds the achievable routing opportunity for this fixture.

The decision-risk policy is an expected threshold-classification-risk heuristic, **not a full Bayesian EVSI implementation**. It assumes the supplied predictive mean/standard uncertainty are usable for a normal approximation and that the confirmatory reference resolves the synthetic target label. Real reference uncertainty, test sensitivity/specificity and repeatability must be modelled before an operational EVSI policy is claimed.

## 4. Inputs and authority separation

The selector receives only:
- stable synthetic sample ID and property ID;
- NIR predicted value, predictive standard uncertainty and threshold;
- which side of the threshold is the positive class;
- whether the prediction is inside the property-specific applicability domain;
- externally declared false-positive and false-negative loss weights;
- referral capacity, NIR screening cost and confirmatory test cost;
- policy configuration and deterministic seed.

The evaluator's hidden truth is held in a separate structure and is never passed to policy selection. A regression test mutates evaluator truth and requires selection to remain unchanged.

Decision thresholds and loss weights must come from qualified local agronomic review and the intended decision context. The software does not invent nutrient sufficiency thresholds. This v0.1 fixture uses arbitrary synthetic values only.

## 5. Cost and quality metrics

Report for each policy:
- samples screened, referrals and referral fraction;
- total NIR screening plus confirmatory cost;
- error count, false positives, false negatives and weighted decision loss;
- post-referral residual error / decision loss;
- accuracy and sensitivity / specificity where defined;
- decision-loss regret relative to all-reference and the perfect-information bounded oracle;
- deterministic selection digest / replay equality.

Do not report one scalar without the error/cost frontier. A route with low expense but an unacceptable false-negative rate is not a win.

## 6. Qualification gates

- Hidden evaluator truth is structurally inaccessible to the selector.
- Referral count never exceeds capacity.
- Ineligible actions / routes cannot be admitted by score.
- Out-of-domain samples are prioritized for referral or remain explicitly unresolved if referral capacity is exhausted.
- Threshold direction and false-positive / false-negative costs are explicit.
- Exact cost and loss accounting is checked on manufactured examples.
- Mutation of hidden truth cannot alter the selected sample set.
- Deterministic replay with identical inputs yields identical selected IDs and report digest.
- The workflow validates the input/report schemas and negative controls.
- Real-data evaluation is blocked until exact source version, file SHA-256, NIR intensity semantics, cohort crosswalk, sample grouping, and approved decision thresholds are established.

## 7. SOTA and industry comparison

This is a **workflow-level** comparator, not an attempt to beat PLSR, Cubist, RF or spectral transformers on model fit. It compares the operating pathway against:
- full wet chemistry for every sample;
- full MIR for every sample;
- NIR-only screen;
- fixed/random confirmatory sampling;
- uncertainty-only and applicability-only routing;
- decision-risk routing under fixed referral capacity.

A meaningful gain is a better cost–false-negative–coverage Pareto frontier on independent groups, not a lower average RMSE alone. Model comparisons remain on #6414; this issue owns the follow-up measurement-routing decision.

## 8. Real-world extension requirements

Before production use:
- Reproduce property-specific NIR/MIR and local/global calibration baselines.
- Resolve the Western Highveld source's raw reflectance vs converted absorbance conflict.
- Quantify reference wet-chemistry repeatability and method uncertainty.
- Include false-positive / false-negative consequences approved by local agronomists.
- Model imperfect confirmatory methods, repeat tests, missing/censored results, referral queue capacity, latency and instrument downtime.
- Use grouped external site/field/season/instrument testing; no random-row leakage.
- Test selective prediction calibration on referred and non-referred subpopulations.
- Show which decisions remain unresolved if referral capacity is insufficient.
- Keep land access, sampling permissions, data rights, safety and governance outside the model's authority.

## 9. Boundaries

Synthetic performance is not soil-property accuracy. This benchmark does not diagnose soil, prescribe fertilizer, approve treatments, or certify a handheld instrument. No field or laboratory action is authorized.
