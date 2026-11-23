# Randomized soil-trial operating-characteristics benchmark v0.1

**Status:** deterministic Monte Carlo methods benchmark over manufactured trial populations. It does not estimate any real soil treatment effect.

## Why this follows the randomized trial auditor

Mycelix #6536 computes a blocked intention-to-treat (ITT) difference in baseline-to-follow-up change and an exact two-sided paired-block sign-flip randomization p-value. That is a useful narrow estimator, but a golden fixture proves arithmetic only. It does not establish Type-I error, power, bias, robustness or transportability.

The four-block #6536 fixture has 16 possible sign assignments and a theoretical minimum two-sided p-value of 2/16 = 0.125. Consequently, it cannot reject at nominal alpha 0.05 even with the most extreme possible observed contrast. Six blocks are the first count with resolution <=0.05 (2/64 = 0.03125); eight are the first with resolution <=0.01 (2/256 = 0.0078125). This is a discreteness fact, not a power guarantee.

## v0.1 DGP

A trial contains B paired randomized blocks and two eligible plots per block. The allocation selects one of the two plots for assignment to treatment with probability 0.5; the other is control. Generate potential treatment effects, baseline outcomes, shared block changes, plot noise, measurement error and treatment receipt before computing the assignment-based observed contrast.

The synthetic generator varies:
- paired block count (2–12);
- true mean unit-level treatment effect;
- unit-level treatment-effect heterogeneity;
- block-level common effects and plot-level outcome noise;
- baseline dispersion and shared laboratory measurement error;
- treatment adherence and control crossover;
- missing outcomes under MCAR or response-dependent attrition;
- interference status.

The target is the expected finite-population ITT under assignment:
`mean(unit treatment effect) × (P(receive | assigned treatment) - P(receive | assigned control))`.

The realized randomized trial estimator uses assigned arms, not received treatment. The as-treated effect is never substituted.

## Exact estimator and independent check

For each complete, non-interfering trial:
1. compute each plot's observed baseline-to-follow-up change;
2. calculate treatment-assigned change minus control-assigned change within each block;
3. average paired block contrasts for the ITT estimate;
4. enumerate all 2^B sign flips and compute the exact two-sided randomization p-value.

The benchmark independently brute-forces the sign-flip reference distribution. It does not re-use the production CLI or trust the reported p-value.

## Abstention / denominator rules

- Any missing endpoint abstains the complete trial. No randomized unit is silently dropped.
- Possible or unassessed interference abstains causal qualification.
- Non-adherence and control crossover remain in the assigned groups and are reported separately.
- All trials, including abstentions, remain in the denominator for the qualification rate. Type-I error / power are reported among qualified trials with their denominator visible; this conditional estimand must not hide the abstention rate.
- Repeated subsamples, sensors and laboratory replicates are not randomized units.

## Metrics

For each scenario:
- requested, qualified and abstained trial counts, by reason;
- expected DGP ITT effect, mean estimate, bias and RMSE;
- empirical rejection rate at the declared alpha;
- Monte Carlo standard error of the rejection rate;
- average non-adherence and crossover rates;
- theoretical minimum two-sided randomization p-value from block count;
- explicit label `type_i_error` when true ITT is zero, otherwise `power`.

No confidence interval coverage is reported in v0.1 because the production reference auditor does not yet emit a validated interval procedure. Do not treat the standard error alone as a validated confidence interval.

## Required scenarios

The fixture includes:
- four-block sharp-null case (shows p-value resolution cannot reach 0.05);
- eight-block sharp-null case (tests null operating characteristics);
- constant-effect case;
- heterogeneous-effect case;
- non-adherence / crossover case (still ITT);
- missing-outcome case (abstention, not complete-case estimation);
- unassessed interference case (causal abstention).

The seeded simulator uses a deterministic SplitMix64 stream and Box–Muller normal draws. Replays are deterministic for identical bytes, code and seed. A changed DGP or estimator creates a new version/digest.

## Comparators and next extensions

The current harness validates the exact paired-block estimator, not every competing estimator. The next bake-off should add baseline-adjusted ANCOVA, hierarchical site/block/plot models, studentized randomization tests, spatial/network interference estimators, and a deliberately invalid unpaired/pseudo-replicated control. Compare them on identical seeds and DGPs.

Before empirical deployment, extend to:
- systematic operating-characteristic grids with Monte Carlo confidence intervals;
- skew/heavy-tail and bounded / zero-inflated outcomes;
- explicit MAR/MNAR attrition models and partial-identification / sensitivity bounds;
- multisite, soil/crop strata and cluster randomization;
- event-level co-outcomes (GHG, nutrient export, water response) and shared assay-batch covariance;
- calibrated estimator intervals and coverage.

## Proposed research targets (not achieved)

- independent brute-force p-value agreement for every tested small block count;
- null rejection rates close to nominal, with binomial Monte Carlo intervals and discrete-p-value resolution shown;
- increasing power with effect size and block count under the frozen DGP;
- no false causal qualification under missing outcomes or possible/unassessed interference;
- no systematic denominator shrinkage;
- ITT remains assignment-based under non-adherence;
- report worst-stratum operating characteristics before any pooled headline;
- any precision/power improvement retains Type-I error control and coverage.

## Literature / design comparators

- Mycelix #6536 randomized soil-trial auditor.
- Symthaea #12987 causal-value RCT allocation; use it for *which eligible blocks/units to randomize*, not as an alternative inference implementation.
- University of Florida on-farm field-trial guidance recommends replicated randomized complete blocks and warns that unreplicated side-by-side demonstrations confound field variation with treatment: https://ask.ifas.ufl.edu/publication/ag447
- Field-trial guidance for enhanced-efficiency fertilizers recommends at least four replications as a starting practice, with power analysis for smaller effects / greater variability: https://doi.org/10.1002/saj2.20787
- Agricultural experiments with complex blocking and network/spillover effects need to model interference explicitly, not just spatial covariance: https://doi.org/10.1007/s13253-023-00544-3

These sources inform the benchmark, not a universal block-count recommendation. Six/eight-block randomization resolution alone is not proof of adequate power.

## Claim ceiling

All DGPs are synthetic. The benchmark evaluates statistical behaviour under declared assumptions only. It does not establish actual randomization, an actual soil intervention effect, a safe amendment rate, field suitability or transportability to a South African farm.
