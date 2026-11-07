# NEON soil-water normalizer v0.1

This schema-driven Rust CLI converts a one-minute soil-water CSV table into an audited regular 5–30 minute profile series. It does not guess NEON table headers or silently apply a default quality policy: the configuration explicitly maps source columns and accepted values.

## Run

    cargo test --manifest-path research/terra-preta/tools/neon-swc-normalizer/Cargo.toml
    cargo run --manifest-path research/terra-preta/tools/neon-swc-normalizer/Cargo.toml -- \
      config.json SWS_1_minute.csv \
      normalized-raw-profile.json paper-analysis-profile.json aggregation-report.json

Then feed the normalized profile directly into the existing extractor and profile auditor:

    cargo run --manifest-path research/terra-preta/tools/profile-response-extractor/Cargo.toml -- \
      normalized-profile.json profile-response-input.json 0.01
    cargo run --manifest-path research/terra-preta/tools/profile-flow-audit/Cargo.toml -- \
      profile-response-input.json profile-flow-report.json

## Configuration contract

- Pin product/release, raw file label and input/source evidence reference.
- Declare the exact timestamp, sensor-key, theta and QA-filter column names after checking the pinned NEON variables table.
- Supply sensor roster keys plus metadata evidence, corrected depth, x/y metres, condition and the sensor position validity start/end. For DP1.00094.001, obtain depth from corrected `swc_depthsV3.csv`, not the ordinary `sensor_positions` zOffset; join sensor positions using time-valid intervals.
- Declare one or more quality filters and accepted values. The tool will not default to finalQF==0 or assume any release-specific QA semantics.
- Configure 1-minute raw resolution, target 5–30 minute resolution, minimum good fraction, response threshold, and the maximum whole-profile porosity across the site's soil pits plus an evidence reference for the soil-property source/crosswalk.

## Aggregation

Target bins are left-closed/right-open. Each target median is calculated from directly accepted 1-minute values in the bin. A bin is `observed` only when good rows meet `ceil(minimum_good_fraction × expected_rows)`; otherwise it is `missing` with null theta. Missing rows, QA rejects, and values above the configured maximum whole-profile porosity are counted separately. Porosity is applied after source QA filtering and before aggregation. `quality_pass_rows` means passed the configured QA filters; `porosity_pass_rows` are the values actually used in the median. The raw-normalized output uses no zero-fill or interpolation.

The CLI writes two profile series plus an aggregation report. `normalized-raw-profile.json` preserves QA/porosity-qualified medians with missing bins untouched. `paper-analysis-profile.json` is a separate clone in which only interior gaps with observed endpoints less than six hours apart are linearly interpolated; edge gaps and endpoint separations of six hours or more remain missing. The report records original missing, interpolated and still-missing bin counts and the exact config/source SHA-256. Both profile JSONs are shaped to the strict input schema of `profile-response-extractor`. For the 2026 paper-reproduction branch, use the paper-analysis profile as the explicit short-gap-interpolated input for event-response computation. For missingness-sensitive NSR and profile diagnostics, use the raw-normalized profile; it preserves the gaps and allows interval-censored/indeterminate response handling. Never overwrite or discard the raw-normalized series when evaluating the paper-analysis series.

## Evidence boundary

This is a deterministic executor of a declared mapping/filter rule. It does not establish that the configured flags or thresholds are scientifically sufficient; the pinned NEON release, variables file, ATBD and issue log must justify the policy. It does not detect preferential flow or infer a unique pore pathway.