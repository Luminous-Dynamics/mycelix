# Soil-health assessment agreement and decision-robustness benchmark v0.1

**Status:** synthetic-only method audit. The six scoring archetypes below are transparent representative families, not reproductions of named commercial or published protocols.

## Why this benchmark is needed

Zwaan et al. (September 2026), *Soil health indices in disagreement: A systematic review and comparison of soil health assessment methods*, Soil Biology and Biochemistry 220, 110184, DOI https://doi.org/10.1016/j.soilbio.2026.110184, reviewed 199 studies containing 212 assessments and compared six representative methods on a dataset of 87 farms. It identified four clusters (modelling, scoring function, expert judgement and multifunctionality) and found low agreement / poor correlations between selected methods. It cautions that method scores are not interchangeable and that aggregate metrics can conceal trade-offs.

The central insight is that disagreement is not merely noise to average away. It reveals different purposes, assumptions, indicator functions, scales, normalizations, weights and aggregation rules. A soil-health number without its purpose and construction is under-specified.

## Paradigm shift

Do not attempt to win by inventing another universal soil-health scalar. Build an assessment-method audit that makes the assumptions and disagreement visible:
- define the assessment purpose and soil functions before indicator selection;
- retain absolute measurements, methods, units, depth and uncertainty;
- make indicator direction and target ranges explicit;
- version reference ranges, thresholds, normalization, weights and aggregation;
- compare multiple assessment archetypes on the same units and evidence;
- measure rank agreement, reversals, top-choice stability and sensitivity;
- keep function-specific trade-offs visible;
- preserve non-compensable ecological / regulatory constraints;
- report Pareto and uncertainty-aware partial orders rather than forcing every pair into a winner;
- distinguish local soil-function assessment from broad-scale soil-health evaluation.

## V0.1 assessment archetypes

The synthetic tool compares six deliberately transparent method families:
1. Equal linear score: mean of fixed-reference-range normalized indicators.
2. Declared expert-weighted score: weighted mean under an explicit weight profile.
3. Empirical percentile score: mean of within-cohort oriented ranks; cohort-relative by construction.
4. Threshold-pass fraction: fraction of declared indicator thresholds passed.
5. Limiting-function score: minimum normalized indicator score.
6. Geometric-mean score: geometric mean of normalized indicator scores.

These methods are not claimed to reproduce CASH, SMAF, Soil Navigator, SAS, PCA weighting or any vendor protocol. A later adapter can reproduce named methods only after their indicator definitions, scaling curves, weights, thresholds, reference populations and implementation details are pinned.

## V0.1 evidence and outputs

Each alternative carries one measured value and standard error for each indicator. Each indicator declares functional group, unit, direction, fixed reference interval for normalization, and a separate threshold criterion for the threshold-pass method.

The scenario also carries declared weight profiles and synthetic hard guardrails. The report includes all six method-specific scores and ranks; pairwise Spearman rank agreement and Kendall's W across method rankings; five weight-profile rankings and pairwise order reversals; nominal Pareto fronts from the oriented function vector; conservative interval-separated dominance relations / front; and hard-guardrail results for every alternative.

A scalar score is an output of a named method under its declared assumptions, not an absolute soil-health truth. The report never chooses one method as universally correct.

## Pareto and uncertainty rules

For each indicator, values are oriented so larger values are beneficial. Nominal Pareto dominance requires no worse value on every declared function and a strict improvement on at least one.

The conservative uncertainty-aware relation requires the candidate's lower 95% beneficial bound to be at least the comparator's upper 95% beneficial bound for every indicator, with a strict separation on at least one. This is deliberately conservative. Overlapping intervals remain incomparable; they are not broken by arbitrary weights.

Hard guardrails are evaluated separately using the measurement's 95% interval: clear means the whole interval lies inside the declared bound(s); violated means the whole interval lies beyond at least one bound; uncertain means the interval overlaps a bound. Only clear passes the synthetic feasibility gate. A high composite rank cannot override a violated or uncertain guardrail. Synthetic limits have no real-world authority.

## Negative controls

- Input row order and indicator order do not change outputs.
- Changing weights may change method-specific ranks but cannot change raw measurements, guardrail results or Pareto relations.
- A known strictly dominated alternative cannot enter the nominal Pareto front.
- An alternative failing a hard guardrail remains ineligible even when a method ranks it first.
- Unknown indicator direction, missing data or invalid weights fail closed.
- No synthetic archetype is labelled as a reproduced industry method.

## Proposed real-data qualification targets (not achieved)

- Exact deterministic replay and permutation invariance.
- 100% preservation of hard constraints in all known-truth controls.
- Zero false nondominated-front admissions for known-truth strictly dominated options.
- Rank disagreement, top-k stability, pairwise reversals, Spearman, Kendall tau-b, Kendall W and ICC (only where its assumptions fit) reported together, not hidden behind a preferred metric.
- Sensitivity to weights, reference ranges, threshold choices, missing indicators and measurement uncertainty.
- Separate evaluation for local field-function decisions and regional / broad-scale mapping.
- No recommendation emitted solely from a composite rank; decision eligibility and the function vector remain visible.

## Related work

- #5525 whole soil-function model
- #5527 persistence / long-horizon effects
- #5529 nutrient fate and mass balance
- #5632 calibration and uncertainty
- #6445 context-conditioned potential, absolute condition and trajectory
- #6446 context-conditioned soil-function benchmark
- #6427 cross-stack evidence contract
- #6438 South African soil-source admission

## Scientific boundaries

All v0.1 values and limits are manufactured. This is a method-disagreement and decision-robustness audit, not a field diagnostic, treatment recommendation, or universally valid index. Local soil-function assessment and broad-scale soil-health evaluation have different purposes, spatial support and evidence requirements.