# NEON soil-water normalizer v0.1

This schema-driven Rust CLI converts a one-minute soil-water CSV table into an audited regular 5–30 minute profile series. It does not guess NEON table headers or silently apply a default quality policy: the configuration explicitly maps source columns and accepted values.

## Run

    cargo test --manifest-path research/terra-preta/tools/neon-swc-normalizer/Cargo.toml
    cargo run --manifest-path research/terra-preta/tools/neon-swc-normalizer/Cargo.toml -- \
      config.json SWS_1_minute.csv normalized-profile.json aggregation-report.json

Then feed the normalized profile into `profile-response-extractor` and `profile-flow-audit`.

## Configuration contract

- Pin product/release, raw file label and input/source evidence reference.
- Declare the exact timestamp, sensor-key, theta and QA-filter column names after checking the pinned NEON variables table.
- Supply sensor roster keys plus metadata evidence, depth, x/y metres and condition. For DP1.00094.001, obtain depth from corrected `swc_depthsV3.csv`, not the normal `sensor_positions` zOffset; join sensor positions using time-valid intervals.
- Declare one or more quality filters and accepted values. The tool will not default to finalQF==0 or assume any release-specific QA semantics.
- Configure 1-minute raw resolution, target 5–30 minute resolution, minimum good fraction, and response threshold.

## Aggregation

Target bins are left-closed/right-open. Each target median is calculated from directly accepted 1-minute values in the bin. A bin is `observed` only when good rows meet `ceil(minimum_good_fraction × expected_rows)`; otherwise it is `missing` with null theta. Missing rows and QA rejects are separately counted. No zero-fill, linear interpolation, or silent removal is performed.

The report records exact config/source SHA-256, source column mapping, all QA filters, expected raw rows, present rows, accepted/rejected rows, absent raw rows, valid fraction, median and bin status for each sensor/time bin.

## Evidence boundary

This is a deterministic executor of a declared mapping/filter rule. It does not establish that the configured flags or thresholds are scientifically sufficient; the pinned NEON release, variables file, ATBD and issue log must justify the policy. It does not detect preferential flow or infer a unique pore pathway.