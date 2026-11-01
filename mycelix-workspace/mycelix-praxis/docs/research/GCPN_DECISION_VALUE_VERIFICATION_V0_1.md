# GCPN User-Centred Decision-Value Forecast Verification v0.1

**Status:** research specification and machine-readable decision-profile contract; executable engine outstanding.  
**Parent issue:** #5665.  
**Baseline:** integrated GCPN evaluation v0.6.  
**Scope:** offline synthetic cost/loss and decision-value analysis; no operational action recommendation.

## 1. Research basis

Emerton et al., *How Good Is Your Forecast? The Importance of User-Centred, Decision-Relevant Evaluation of Forecasts and Climate Services*, first published 30 August 2026, argues that traditional statistical accuracy measures are necessary but insufficient to determine whether forecast information supports practical decisions. Forecast usefulness depends on how probabilities intersect with a user's decision context and thresholds.

A 2024 user-focused verification study similarly emphasizes that different proper scoring rules can rank systems differently and that performance at user-relevant decision thresholds must be understood. Pagano et al. (2025) distinguish objective numerical verification from broader performance assessment, including timeliness, relevance, accessibility and use.

This supports a separate GCPN evaluation track. It does not mean statistical verification should be replaced with a single impact metric.

## 2. Three independent questions

1. **Forecast quality:** are the probability/distribution forecasts statistically good, calibrated, sharp and informative?
2. **Decision value:** under a declared synthetic action/loss context, how much does the forecast change expected/realized loss relative to named reference policies?
3. **Actual warning/action effectiveness:** did real authorities and people understand, authorize and act, and what outcome followed? This requires evidence outside a synthetic decision simulator.

Do not collapse these into one weighted score. A forecast may have better proper score but lower value for one cost/loss profile, while the reverse may occur for another profile. Both results can be correct.

## 3. Frozen decision contract

A synthetic decision profile declares:
- binary event definition and outcome semantics;
- action cost (C_a);
- avoidable loss (C_l) if the event occurs and no action is taken;
- residual loss under action, if an action-effectiveness model is included;
- issue/valid time, decision cutoff and minimum usable lead-time window;
- the threshold or a fully specified analytic derivation;
- named forecast and reference models/policies;
- feasibility constraints and any synthetic abstention rule.

For the simplest profile, with (0 < C_a < C_l):
- no action, no event: loss 0;
- no action, event: loss (C_l);
- action, either event state: loss (C_a).

If (p) is the event probability, the expected loss from action is (C_a), and the expected loss from no action is (p C_l). The analytic switching threshold is:

`p^* = C_a / C_l`.

This is a synthetic cost/loss calculation, not a recommendation for a real warning threshold. An operational authority or user must determine its own policy and legal responsibilities.

If residual action loss or asymmetric costs are declared, the evaluator must derive the threshold from the complete frozen loss matrix. If no valid threshold exists or action is infeasible, the profile must abstain rather than invent one.

## 4. DGP families

| Family | DGP variation | What it tests |
|---|---|---|
| Cost/loss sweep | Multiple predeclared (C_a/C_l) ratios | Context-specific value and threshold behavior |
| Better score, lower utility | Forecast A has better Brier/CRPS but worse value for one profile | Proper score is not identical to decision utility |
| Worse score, higher utility | Forecast B has worse aggregate score but higher declared profile value | User relevance and target event alignment |
| Asymmetric loss | Different false-action and missed-event consequences | Decision threshold and profile dependence |
| Partial effectiveness | Action leaves a specified residual loss | Value under a non-perfect intervention |
| Lead-time feasibility | Forecast must be available before a declared cutoff | Timeliness and unusable late forecasts |
| Rare events | Low base rates and high consequence | Small sample and false-action sensitivity |
| Subgroup miscalibration | Global calibration but profile/stratum miscalibration | Hidden utility inequity |
| Threshold-value curve | Fixed grid of policy thresholds | Sensitivity without test-set threshold tuning |
| Missing/late records | Stale, invalid, out-of-domain, abstained forecasts | Missingness and decision failure |
| Cluster dependence | Multiple correlated events within independent systems | Uncertainty around paired utility differences |
| Undefined/infeasible action | No valid cost/loss or action constraint | Explicit abstention |

The DGP must expose an exact analytical expected-loss/value oracle or an independently checked reference calculation. The simulation sample is not its own truth oracle.

## 5. Estimands and metrics

For case (i), let outcome (y_i in {0,1}), decision (a_i in {0,1}) and profile loss (L(a_i,y_i)) be determined by the frozen profile.

Report separately:
- proper score for candidate and reference forecasts;
- realized policy loss per exposed case;
- expected population loss under each named reference policy;
- incremental value against each baseline (baseline expected loss minus candidate expected loss);
- false-action and missed-event counts;
- perfect-information loss lower bound (infeasible oracle, never a candidate);
- decision regret relative to that lower bound;
- value-versus-threshold curve on a predeclared grid;
- fraction of cases unusable due to late forecast, missing evidence or infeasible action;
- profile-specific outcomes and worst-profile results;
- cluster-aware paired uncertainty for score and utility differences.

Do not average different cost/loss contexts into a universal user utility score. A predeclared weighted aggregation is permitted only if the target population and its weights are specified independently before evaluation; always publish the separate profile results too.

## 6. Baselines

Compare the candidate forecast policy with:
1. never act;
2. always act, where allowed by the declared profile;
3. climatology forecast policy;
4. named reference forecast policy;
5. perfect information (oracle lower bound on loss only).

The perfect-information comparator is infeasible and must never be described as a deployable system. Candidate and reference policies must be evaluated on identical cases, outcomes and decision cutoffs.

## 7. Fair comparison and leakage controls

- Freeze thresholds, cost/loss profiles, action-effectiveness assumptions, decision windows, subgroup families and value weights before confirmatory evaluation.
- Do not tune decision thresholds on the same confirmatory cases used to report performance.
- Candidate and reference forecasts use identical event cases, outcomes and verification source/version.
- Preserve chronological partitions and the forecast issue time relative to decision cutoff.
- Late forecasts remain visible and incur the declared unusability treatment; they are not silently dropped.
- Use independent-system clusters for utility-difference uncertainty when events share system-level dependence.
- Report the proper forecast score beside decision value.
- If a profile is undefined or infeasible, report abstention, not a guessed threshold.

## 8. Proposed research targets (not standards or results)

- 100% cost/loss profile fingerprinting and provenance.
- 100% threshold derivation reproducibility from the frozen profile.
- Zero confirmatory threshold tuning.
- 100% paired-case equality for candidate/reference utility comparisons.
- Exact analytical expected-loss oracle reproduction.
- Zero conflation of proper score, policy loss and decision value.
- Explicit late/missing/infeasible/undefined-profile accounting.
- No cross-profile aggregate without predeclared target-population weights.
- Calibrated 90%, 95% and 99% uncertainty for score and utility differences under known dependent DGPs.

## 9. Industry and research alignment

The approach complements:
- ECMWF/WMO probabilistic verification and proper scores;
- the 2026 user-centred decision-relevant forecast evaluation literature;
- user-focused verification across predeclared decision thresholds;
- WMO impact-based forecasting and warnings, which aim to move from "what the hazard will be" toward "what the hazard will do."

The synthetic engine should quantify decision value under frozen assumptions, not infer a real user's risk preference, decide a national warning threshold or compare synthetic utility units with lives saved or financial budgets.

## 10. Integration boundary

- Start as an independent offline evaluator.
- Preserve the eight independent v0.6 gates.
- Any future integration proposal must keep proper forecast verification, user-context utility and human/authority evidence separate.
- No warning issuance, automatic model selection, real action recommendation, funding release or responder instruction.

## 11. Sources

- Emerton et al., *How Good Is Your Forecast? The Importance of User-Centred, Decision-Relevant Evaluation of Forecasts and Climate Services*, Meteorological Applications (first published 30 August 2026): https://rmets.onlinelibrary.wiley.com/doi/full/10.1002/met.70234
- Pagano et al., *Enhancing Forecast Verification in National Meteorological and Hydrological Services*, Meteorological Applications (2025): https://rmets.onlinelibrary.wiley.com/doi/full/10.1002/met.70051
- *A User-Focused Approach to Evaluating Probabilistic and Categorical Forecasts*, Weather and Forecasting (2024): https://journals.ametsoc.org/abstract/journals/wefo/39/8/WAF-D-23-0201.1.xml
- WMO, *Hazard Monitoring and Forecasting 2025 — EW4All Pillar 2*: https://wmo.int/sites/default/files/2025-10/EW4All-Pillar-2-Report_en.pdf

## 12. Non-claims

This is synthetic offline decision-value evaluation. It is not an operational warning threshold, a model of real user preferences, a warning/action policy, an emergency-response system, an impact/lives-saved estimator or a causal inference study.
