# Preferential-flow threshold sensitivity audit v0.1

This Rust CLI compares a sensor/event's apparent velocity against multiple declared velocity thresholds and reports how stable the resulting PF proxy label is to the selected reference. It is a threshold-sensitivity audit, not a physical pathway detector.

## Run

    cargo test --manifest-path research/terra-preta/tools/pf-threshold-audit/Cargo.toml
    cargo run --manifest-path research/terra-preta/tools/pf-threshold-audit/Cargo.toml -- \
      research/terra-preta/tools/pf-threshold-audit/fixtures/synthetic-threshold-sensitive.json \
      /tmp/terra-preta-pf-threshold-report.json

## Supported threshold families

- Rosetta Ksat estimates: low, maximum, high, median-based and median-plus-standard-deviation upper estimate.
- Matrix-flow model threshold: the 99th-percentile matrix velocity estimate, reported separately from Rosetta Ksat.

All values must use the same explicitly declared scale: `linear_cm_per_day` or `log10_cm_per_day`. Log-scale values are compared in their stored scale without exponentiation or conversion. The tool rejects mixed scales, duplicate threshold IDs/kinds, inverted Ksat intervals and invalid digests. It applies the strict operator velocity > threshold; equality is negative under that threshold.

## Output interpretation

- `stable_positive_across_declared_thresholds`: every threshold in that family yields PF proxy=true.
- `stable_negative_across_declared_thresholds`: every threshold in that family yields PF proxy=false.
- `threshold_sensitive`: the result changes with the declared threshold choice.
- `single_reference_not_robustness_tested`: only one reference threshold was supplied.
- `not_evaluated`: no threshold in that family was supplied.

Rosetta Ksat and matrix-model 99th-percentile thresholds are not merged into one score. A label stable across these references is robust only to the supplied threshold set, not a tracer-confirmed preferential pathway. A threshold-sensitive result should be retained as an explicit label uncertainty rather than collapsed to a single binary class.

The synthetic fixture demonstrates a threshold-sensitive proxy label only. It is not an actual NEON/HydroShare observation or a field result.