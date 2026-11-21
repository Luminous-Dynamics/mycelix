# Context-conditioned soil-function and ecological-potential benchmark v0.1

**Status:** synthetic-only scoring and evidence contract. It is inspired by SHAPE-SSA but is not a reproduction, not a South African calibration and not a field diagnostic.

## 1. Research shift

Birru et al. (2026), *A region-specific soil health assessment protocol and evaluation for Sub-Saharan Africa*, Communications Earth & Environment 7:670, DOI [10.1038/s43247-026-03727-1](https://doi.org/10.1038/s43247-026-03727-1), demonstrate a peer-context approach using 28,000 Ethiopian topsoil samples. Its SOC example conditions on texture, WRB Reference Soil Group, mean annual temperature (MAT) and mean annual precipitation (MAP), with a hierarchical Bayesian model. It reports R² = 0.651 and RMSE = 0.534% SOC for its selected four-covariate model.

The key lesson is to compare observations against ecologically relevant peer distributions rather than a universal temperate threshold. But percentile rank must not be confused with absolute improvement. The article shows that the same absolute SOC can receive different percentiles under different MAT because the peer reference distribution changes. A peer-group 95th percentile is an empirical upper-tail reference, not a physical maximum, causal recovery target, or guaranteed attainable ceiling.

## 2. Four separate evidence axes

Every indicator is reported as a separate record with four non-interchangeable views:

1. **Absolute condition:** measured value, unit, reference method, sample depth and date.
2. **Context-conditioned peer position:** percentile and uncertainty against a peer reference distribution, with domain and sample-support qualification.
3. **Longitudinal within-unit change:** absolute endpoint difference across the same experimental unit, with propagated measurement uncertainty. This is descriptive unless an appropriate causal design and estimand are separately qualified.
4. **Guardrails:** independently specified absolute floors / ceilings, with authority and source. Peer rank never overrides a guardrail.

No aggregate soil-health score is emitted. No automatic BUILD / PROTECT / MAINTAIN label is emitted. No management effect is inferred from a cross-sectional percentile.

## 3. Peer-reference contract

The benchmark input carries a versioned, conditional reference model with:
- peer-group identity and context (texture, reference soil group, MAT, MAP);
- direct-measurement reference-target semantics;
- training independence groups and the evaluated unit's separate independence group;
- number of training samples and independent sites;
- model-domain status;
- predictive distribution parameters and empirical reference quantiles;
- the reference-model and source digests.

A percentile is produced only when the peer reference is in-domain and satisfies predeclared minimum sample and independent-site counts. Otherwise, the system abstains and reports why. It never silently borrows a different peer group.
The peer reference must also match the observation's analytical method, unit and depth support exactly in v0.1. The source dataset and model each carry a SHA-256 identity in the report. This intentionally rejects implicit harmonization; a future method/depth transfer requires an explicit, separately validated conversion or calibration-transfer record.

The v0.1 implementation uses a Gaussian predictive reference distribution as a **manufactured software test approximation**. It is not a substitute for the hierarchical Bayesian posterior / posterior predictive distribution used in SHAPE-SSA or a fitted local reference model.

## 4. Meaning of the reference-distance metric

The reported distance to peer q90 / q95 / q99 is:
`peer_quantile - observed_value`.

Positive values mean the observation is below the empirical peer quantile; negative values mean it exceeds that quantile. These distances are not labelled "recoverable gap", "attainable gap" or "causal response". Their uncertainty combines supplied quantile and measurement standard errors using a first-order independent-error approximation; covariance is not inferred.

## 5. Longitudinal contract

Repeated measurements:
- belong to one declared experimental unit / plot;
- share the declared indicator, method, unit and depth support;
- are strictly ordered by date;
- report first-to-last absolute change and endpoint standard error.

V0.1 does not estimate trend slopes, autocorrelation, treatment effects, difference-in-differences or counterfactual responses. Repeated measurements are not independent replicates. A future longitudinal model must represent plot/site/block hierarchy, serial correlation, missingness and treatment assignment.

## 6. Guardrail contract

A guardrail can be:
- `not_set`: no decision limit is asserted;
- `synthetic_fixture`: a manufactured limit used only to test that peer rank cannot hide an absolute violation;
- `externally_approved`: a locally authorized threshold with named authority, date and reference.

The synthetic guardrail in the fixture is not a real soil threshold. V0.1 does not infer, recommend or choose agronomic thresholds. In deployment, an `externally_approved` guardrail requires independent local technical / regulatory approval.

## 7. Baselines and benchmark comparison

The scientific comparison should include:
- fixed-threshold / conventional indicator scoring (SMAF / CASH-style) under the declared intended context;
- context-conditioned peer percentiles (SHAPE-SSA-style);
- hierarchical partial-pooling context models;
- local-only, global pooled and hierarchical transfer variants;
- longitudinal mixed-effects and paired-change baselines for repeated fields;
- randomization inference when treatment assignment makes it valid.

The Ethiopian SHAPE-SSA performance is a literature comparator, not a South African score target or ready-made reference distribution. The actual South African model must be trained and externally validated against admitted South African data.

## 8. Qualification targets (proposed, not achieved)

- Zero conflation of relative rank, absolute condition, temporal change and causal treatment effect.
- Nominal 90% peer-reference intervals target 85–95% empirical coverage on independent spatial / soil-group holdouts; report interval width and proper scores.
- Explicit abstention for out-of-domain and sparse peer groups.
- Zero leakage across sample, field, site, survey, batch and reference-model training groups.
- Report worst-group calibration, coverage and transfer loss, not only pooled R².
- At least 10% relative improvement in held-out absolute-property prediction is a research target only if calibration, coverage and domain support do not regress.
- Each indicator retains its own reference, units, uncertainty and guardrail; no compensatory composite score.

## 9. South African transfer plan

Do not port Ethiopian peer-group boundaries, thresholds or fitted coefficients to South Africa. Establish local peer groups using source-admitted profile / landtype / mineralogical, climate, depth and landscape context. AGIS, SOTER, Soil Data of Africa and global map products can assist stratification only at their native support and with measured / predicted / pedotransfer semantics preserved.

Western Highveld NIR/MIR data may support spectral calibration tests but are not a longitudinal field-treatment dataset. MIR-derived estimates are not independent wet-chemistry targets.

## 10. Scientific source and limits

Birru, M. K. et al. (2026). *A region-specific soil health assessment protocol and evaluation for Sub-Saharan Africa*. Communications Earth & Environment 7, 670. DOI [10.1038/s43247-026-03727-1](https://doi.org/10.1038/s43247-026-03727-1).

The paper applies the SOC demonstration to Ethiopia's EthioSIS-derived data. The method is a useful benchmark architecture, but new South African references and independent validation are required before local use.

This software fixture is synthetic. It does not measure or diagnose soil, establish local ecological potential, authorize field sampling, recommend amendments or fertilizers, or prove that management caused any observed change.
