# Spatial heterogeneity audit v0.1

This standalone Rust CLI computes event-relative spatial soil-moisture dispersion and empirical semivariograms from an explicitly aligned sensor array. It reports a **spatial heterogeneity signature only**; it does not certify macropore flow or infer water flux.

## Run

    cargo test --manifest-path research/terra-preta/tools/spatial-heterogeneity-audit/Cargo.toml
    cargo run --manifest-path research/terra-preta/tools/spatial-heterogeneity-audit/Cargo.toml -- \
      research/terra-preta/tools/spatial-heterogeneity-audit/fixtures/synthetic-array.json \
      /tmp/terra-preta-spatial-report.json

## Input requirements

- One site/plot/profile/event, explicit UTC start/end and a fixed 5–30 minute sampling grid.
- At least two sensors with x/y coordinates in metres, depth in cm and a declared condition. For NEON DP1.00094.001, derive depth from `swc_depthsV3.csv`, not `sensor_positions` zOffset; use position coordinates only after a time-valid sensor-position join.
- One directly observed pre-event baseline for each sensor, followed by complete shared-grid bins. Missing bins must be explicit rows with null theta.
- Volumetric theta in m³/m³, with `observed`, `linear_interpolated_gap_lt6h` or `missing` status.
- Non-usable sensor conditions (saturated/frozen/fault/unknown) are retained in the roster and excluded from direct response statistics.
- Distance bins are explicit, non-overlapping, lower-exclusive and upper-inclusive.

## Metrics

At every event time and depth, the report includes roster/valid/missing/interpolated/non-usable counts, distinct valid positions, fraction crossing the declared baseline-delta threshold, mean, median, population standard deviation, IQR, MAD, robust coefficient of variation (1.4826 × MAD / |median|), positive max-to-median ratio, range and distance-binned empirical semivariance. Semivariance is half the mean squared difference between paired event-relative theta changes.

Only directly observed values from usable sensors enter the metrics. No missing or interpolated sample is imputed as zero. Semivariogram bins with no supporting pairs have null semivariance and pair_count 0; no variogram model is fitted.

## Evidence boundary

Spatial variability can arise from texture, roots, lateral interflow, sensor support, calibration, drift or sampling geometry. It is a candidate event signature, not unique evidence of preferential pathways, vertical connectivity or exported water/tracer. Interpret alongside profile NSR, velocity thresholds, trajectories, water balance, ERT/GPR, tracer breakthrough and boundary flux.

## Metamorphic tests

The v0.1 Rust suite currently checks sensor-order invariance, isotropic translation/rotation invariance, homogeneous matrix-only zero-dispersion behavior, missing/interpolated handling and non-usable sensor exclusion. Spatial map recovery and sparse-layout ablations remain follow-on work in [PF-SPATIAL-001](https://github.com/Luminous-Dynamics/mycelix/issues/5557).

The unit tests require spatial statistics to be invariant to sensor input order and to isotropic translation/rotation. They also verify that missing/interpolated and unusable observations are excluded without zero-filling, and that distance-bin overlap is rejected.