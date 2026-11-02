# GCPN probabilistic Fractions Skill Score v0.1

## Objective

Add a transparent, scale-dependent spatial verification measure for threshold events, with explicit deterministic and ensemble estimands, grids/masks/weights, boundaries, missingness, ensemble size, and reference-score interpretation.

FSS complements—not replaces—Brier score, CRPS, energy score, variogram score, calibration/reliability and user decision-value measures.

## Paradigm shift 1: separate location tolerance from probability quality

For each threshold and neighbourhood scale, compute the event-fraction field. A small displacement should not be treated the same as a complete miss, but increasing neighbourhood size makes the problem easier. Therefore never report one aggregate FSS or average over scales without showing the scale curve.

For deterministic forecasts, use the standard fractions Brier score and worst-case fractions Brier score:
- FBS = weighted mean of (forecast fraction - observed fraction)^2;
- WFBS = weighted mean of forecast fraction^2 + observed fraction^2;
- FSS = 1 - FBS/WFBS.

For ensembles, use the Necker et al. (2024) probabilistic FSS (pFSS): form binary event indicators for each member, combine ensemble and neighbourhood probabilities, then compute the same normalized fractions score. Do not call member-averaged FSS, ensemble-mean FSS or ensemble-aggregated FSS pFSS.

## Paradigm shift 2: replace fixed usefulness heuristics with an explicit null

The traditional FSS > 0.5 and FSS > 0.5 + observed base rate / 2 rules are not universal usefulness criteria. Antonio & Aitchison (2025) derive a neighbourhood-dependent reference score for a spatially independent Bernoulli random forecast with event probability equal to the observed base rate. It can materially change the interpretation of skillful scales.

For deterministic FSS, report the analytical random-reference score when its assumptions and required summary statistics are available. For finite ensembles, report a reproducible empirical random-reference distribution matched to:
- the same event base rate;
- the same ensemble member count;
- the same grid, mask, boundary and missingness policy;
- the same neighbourhood scale and cell weights.

Where possible also compare to empirical climatology and persistence references. The independent-Bernoulli random null is not a substitute for a spatially correlated climatological forecast.

Report the scale set/range exceeding the declared reference. Do not assume the scale curve is monotonic.

## Mathematical contracts

For each grid cell:
- apply an explicit event threshold and inequality (greater-than-or-equal or greater-than);
- for an ensemble, compute each member's binary event indicator and ensemble event probability;
- compute neighbourhood event fractions with explicit spatial weights.

For centre i and neighbourhood N(i):
- forecast fraction = sum of area weights times ensemble event probability / sum of in-support area weights;
- observation fraction = sum of area weights times observed event indicator / sum of in-support area weights.

Aggregate the squared fraction difference and worst-case denominator over eligible centres using their declared centre weights. pFSS = 1 - FBS/WFBS. A zero denominator is a typed degenerate case, never silently converted to 1.

## Spatial support, missingness and boundaries

- Input shape, weights, mask and support are explicit.
- Window width is a positive odd number of grid cells.
- Boundary policy is either truncate the window at domain edge or require a full window.
- Masked cells are outside support, not missing values.
- Missing observation or any ensemble-member value is distinct from a non-event.
- Strict mode rejects missing data on the scoring mask. Pairwise-complete mode omits incomplete cells and reports exact effective centres/weight.
- Grid-cell area weights are used inside neighbourhood fractions; declared centre weights aggregate neighbourhood scores.
- No longitude wrap or spherical geometry is implied by a rectangular local grid; a future geospatial adapter must define projection and wrap semantics.

## Random-reference baseline

The v0.1 empirical null samples independent Bernoulli event indicators at each eligible grid cell with probability equal to the observed area-weighted event frequency. It uses the exact ensemble member count and scoring support. A fixed, recorded seed and deterministic PRNG yield byte-identical output. It reports:
- draws and seed;
- observed event frequency;
- null mean and 95th percentile;
- upper-tail Monte Carlo p-value with +1 correction.

This null represents spatially independent random events, not climatology or persistence. The PRNG is for reproducible simulation only, never for cryptographic use.

## Required negative controls

- exact match/perfect forecast;
- all-zero no-event case (typed degenerate denominator);
- complete miss and false-alarm-only forecast;
- one-member pFSS equals deterministic FSS;
- translated event at multiple neighbourhood widths;
- same pointwise marginals but different spatial dependence;
- rare, moderate and common event frequencies;
- ensemble-size sweep, including duplicated and independent members;
- odd/even/zero window widths;
- strict and pairwise-complete missing cells;
- irregular masks and boundaries;
- invalid/negative/non-finite weights and values;
- threshold equality convention;
- candidate/reference common-source case-frame requirement before score comparison;
- reproducible random-reference simulation.

## Proposed targets (not standards or achieved results)

- 100% analytic identity controls.
- 100% rejection of invalid thresholds, weights, shapes, masks and non-finite data.
- 100% deterministic output reproducibility.
- Zero conversion of missing values into non-events.
- Zero unlabelled mixing of pFSS, avFSS, agFSS or emFSS.
- Report every threshold × scale × lead × event-frequency stratum separately.
- Report ensemble-size sensitivity and null-reference uncertainty.
- Confidence intervals resample independent weather events/blocks, never IID grid cells.

## Relationship to GCPN

- PR #5664 provides spatial ensemble score primitives.
- Issue #5679 / PR #5688 provides strict common-source and paired-case manifests.
- This crate computes threshold-neighbourhood spatial scores only on a frozen case frame.
- A later adapter will consume common-source eligible/paired-verifiable rows and report identical denominators.
- CRPS/Brier calibration and decision-value analyses remain complementary.

## Research references

- Necker et al. (2024), The fractions skill score for ensemble forecast verification, QJRMS, DOI: https://doi.org/10.1002/qj.4824
- Antonio & Aitchison (2025), How to Derive Skill from the Fractions Skill Score, Monthly Weather Review, DOI: https://doi.org/10.1175/MWR-D-24-0120.1
- ECMWF (2023), Verifying high-resolution forecasts: https://www.ecmwf.int/en/about/media-centre/science-blog/2023/verifying-high-resolution-forecasts
- ECMWF (2021), Scale-dependent verification of precipitation and cloudiness: https://www.ecmwf.int/en/newsletter/174/earth-system-science/scale-dependent-verification-precipitation-and-cloudiness

## Non-claims

Synthetic scorer qualification does not establish real forecast skill or warning value. FSS is not a full probabilistic calibration score and does not replace CRPS/Brier/reliability. The random null tests superiority to independent Bernoulli noise, not superiority to climatology, persistence, operational models or a national warning system.
