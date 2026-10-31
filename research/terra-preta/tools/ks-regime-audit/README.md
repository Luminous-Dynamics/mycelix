# Saturated hydraulic-conductivity regime audit v0.1

This Rust CLI evaluates frozen, held-out Ks predictions by relative air capacity (RAC), total porosity and declared laboratory measurement methods. It scores supplied model outputs; it does not fit Ks models or calculate RAC.

## Run

    cargo test --manifest-path research/terra-preta/tools/ks-regime-audit/Cargo.toml
    cargo run --manifest-path research/terra-preta/tools/ks-regime-audit/Cargo.toml -- \
      research/terra-preta/tools/ks-regime-audit/fixtures/synthetic-regime-predictions.json \
      /tmp/terra-preta-ks-regime-report.json

The report binds to the exact input JSON bytes and records the independent source and split-manifest digests. Output paths must not already exist.

## Input contract

- Exact source SHA-256 and a reviewed held-out split-manifest SHA-256.
- Evaluation track is either `uniform_method` or `multi_method`; the uniform track requires one declared Ks measurement method and one water-retention method.
- Each observation has a unique sample ID, independently reviewed group ID, Ks measurement method/protocol, water-retention method/protocol, RAC fraction and derivation evidence, total porosity, measured Ks, and one positive prediction per declared model.
- Every model declaration carries family, version, configuration SHA-256 and implementation evidence reference.
- All measured and predicted Ks values are in m/s. Convert upstream with a separately verified unit crosswalk; no implicit conversion occurs.

## Regime and method strata

The tool creates separate descriptive results for: all samples; RAC < 5% and RAC >= 5%; total porosity <= 40% and > 40%; each Ks measurement method; each water-retention method; and RAC x Ks measurement method.

RAC is the fraction of total porosity that is air-filled at -10 kPa. It is supplied, not calculated. The 5% RAC boundary and approximately 40% porosity diagnostic reflect patterns reported by Qin et al. (2026); they are literature-informed strata, not universal phase transitions or physical laws.

## Metrics

For each model within each non-empty stratum, the tool reports MAE, RMSE and bias in log10(Ks m/s), underprediction/overprediction fractions, and paper-style residual categories. Residual is defined as observed log10(Ks) minus predicted log10(Ks). Material under/overprediction uses +/-0.5 times the pooled sample SD of observed log10(Ks) for the evaluation track, following the published article's residual categorization. The pooled SD is deliberately common across strata so the material residual threshold is comparable; it is not a prediction interval.

Each stratum reports sample count and distinct group count. Fewer than five groups is labelled exploratory only. No confidence interval or significance test is calculated. Model ordering is descriptive for the supplied holdout and must not be reused as model selection evidence.

## Industry comparison

Use identical samples, preprocessing, train/calibration/test assignments and source digests for: Brooks-Corey pore-spectrum integration, van Genuchten pore-spectrum integration, Kozeny-Carman or other semi-empirical conductance models, Rosetta3 and the Terra Preta/Symtropy candidate. Preserve each implementation/version/configuration and all parameter sources. Do not treat a model's intended domain or paper ranking as proof that it wins on another dataset.

Keep the methodologically uniform and multi-method tracks separate. Report the worst-stratum error and low-RAC underprediction explicitly; a high-RAC win cannot hide low-RAC failure. Where available, include measurement uncertainty and replicate variability in a separate uncertainty analysis.

## Interpretation limits

This is a descriptive evaluator for already frozen predictions. It does not acquire HYP-UNI or HYPRES, confirm the published 378/1,734 counts, resolve source identities, estimate missing RAC values or prove field transfer. No South African or Terra Preta result follows from the synthetic fixture.

Implementation tracker: [TERRA-KS-REGIME-001](https://github.com/Luminous-Dynamics/mycelix/issues/5567).


Reference: Qin, M., Giménez, D., Hirmas, D. R., & Nemes, A. (2026). Predicting soil saturated hydraulic conductivity from water retention data – The roles of macroporosity and database composition on model performance. Geoderma 471, 117866. https://doi.org/10.1016/j.geoderma.2026.117866.