# Soil method-bridge benchmark v0.1

**Status:** synthetic-only method comparison auditor. It separates numerical agreement, rank concordance and decision agreement. A high rank correlation does not establish analytical equivalence.

## Scientific motivation

Epelde et al., *Beyond the laboratory: rank concordance of rapid methodologies for soil health assessment*, SOIL 12, 1187–1194 (published 16 November 2026), DOI https://doi.org/10.5194/soil-12-1187-2026, compared rapid/field methods with laboratory methods in a long-term grassland experiment. Reported median Spearman rank concordance was ρ=0.65 for aggregate stability and ρ=0.68 for microbial biomass C, while portable soil respiration had very weak rank concordance. The authors note that field and laboratory methods can capture different aspects of a process.

Zwaan et al. (September 2026), *Soil health indices in disagreement*, Soil Biology & Biochemistry, DOI https://doi.org/10.1016/j.soilbio.2026.110184, reviewed 199 studies / 212 assessments and applied six representative methods to 87 farm sites. It found low agreement and poor correlations among several assessment methods, warning that aggregate scores can hide trade-offs.

## Three separate claims

1. **Numerical agreement:** methods are analytical substitutes only when they measure the same property, unit, measurement basis and support, with explicit equivalence evidence. V0.1 reports descriptive bias, MAE, RMSE and limits of agreement only in this case.
2. **Rank concordance:** for paired specimens or plots, does method A preserve the ordering of samples? Spearman rho can be useful even where units, extraction processes or observation operators differ. It does not establish numeric interchangeability.
3. **Decision agreement:** do both methods reach the same classification under one predeclared threshold whose authority/version is recorded? A synthetic threshold is a software test only.

A portable respiration measurement and a laboratory incubation are not presumed to be the same measurand. Strong Spearman rho does not authorize a unit conversion or replacement claim.

## Input contract

Each comparison declares target property, objective(s), method identity/version/class, units, measurement basis and support; analytical equivalence status; pairing basis (same specimen, matched plot or matched site); unique paired sample IDs; and independence-group IDs. A decision threshold has an explicit provenance state.

Technical replicates and repeated sensor readings must be handled upstream; they are not new independent samples here. The report counts unique independence groups but v0.1 does not perform cluster bootstrap or inferential uncertainty.

## Fail-closed rules

- Numerical agreement requested for non-equivalent/unassessed methods yields `not_eligible_non_equivalent`; no bias, MAE, RMSE or limits are emitted.
- An asserted equivalence with mismatched property, unit, basis or support is rejected.
- Rank concordance requires valid paired sample/plot identities and at least three pairs; it does not require equal units.
- Decision agreement requires analytical equivalence and a common declared threshold.
- Constant-rank inputs abstain; no rho is fabricated.
- No composite score is emitted across method families or soil indicators.

## Metrics and future baselines

- Spearman rank correlation with average ranks for ties.
- For truly comparable methods only: mean paired difference (B minus A), MAE, RMSE and descriptive Bland–Altman limits.
- For a shared decision threshold: both-positive, A-only, B-only and both-negative counts, plus agreement fraction.
- Report pair count and independent-group count.
- Empirical confirmation needs cluster-aware intervals and leave-lab / leave-soil / leave-season holdouts.

Compare standard laboratory methods, rapid field tests (e.g. SLAKES, portable CO2, microbial biomass devices, enzyme readers), proximal sensing (NIR/MIR) and in-situ sensors on matched objectives. Report turnaround, sampling burden, cost, calibration transfer, repeatability, rank concordance, numeric agreement and functional outcome prediction separately.

## Proposed targets (not achieved)

- Zero numerical-equivalence reports with missing/mismatched unit, basis, property or support.
- All methods retain source, method version, sample support and lab/instrument identity.
- Report rank concordance and uncertainty on independent site/soil groups; do not let pooled rho hide subgroup failures.
- Report numerical bias only when methods are commensurate.
- Decision agreement only against the same approved threshold and validated functional outcome.
- Cost/latency improvements count only at equivalent decision or ranking quality.

## Boundaries

Fixtures are synthetic. This tool does not certify a commercial sensor, establish lab equivalence, authorize field sampling or prove agronomic benefit. A rapid method's ranking agreement in one trial does not establish global calibration.
