# NEON precipitation normalizer v0.1

This Rust CLI converts configured NEON primary/secondary precipitation rows into auditable 10-minute rainfall bins. It emits two tracks in one report: a literal interpretation of the 2026 paper's reported five-good-measurement zero rule, and a missing-preserving coverage-aware sensitivity track.

## Run

    cargo test --manifest-path research/terra-preta/tools/neon-precip-normalizer/Cargo.toml
    cargo run --manifest-path research/terra-preta/tools/neon-precip-normalizer/Cargo.toml -- \
      research/terra-preta/tools/neon-precip-normalizer/fixtures/synthetic-config.json \
      research/terra-preta/tools/neon-precip-normalizer/fixtures/synthetic-primary.csv \
      research/terra-preta/tools/neon-precip-normalizer/fixtures/synthetic-secondary.csv \
      /tmp/neon-precip-report.json

Use `-` for a source file whose config is absent.

## Input contract

- One configured primary 5-minute weighing-gauge source and/or one configured secondary 1-minute tipping-bucket source.
- Each source maps its own gauge ID, timestamp convention, precipitation interval-depth column and QA filters. No implicit column names or QA policy are guessed.
- All timestamps are explicit UTC. The input config declares whether each timestamp represents the left edge or right edge of its source interval.
- Precipitation values are interval depths in mm. The normalizer sums interval depths into left-closed/right-open 10-minute bins and reports the derived 10-minute rate in mm/h.
- Secondary data are preferred if at least one secondary row exists within a 10-minute bin. If secondary rows exist but fail QA, the tool does not silently use the primary gauge. Primary is a fallback only when no secondary row exists in that bin.
- Throughfall DP1.00046.001 is rejected as a substitute for open precipitation. Current hourly/daily weighing-gauge DP1.00044.001 is rejected as a 10-minute intensity input.

## Two output tracks

**paper_reproduction** applies the literal five-good-observation threshold reported by Li et al. (2026): when at least one selected-source row exists but fewer than five rows pass configured QA, the 10-minute value is set to zero and status is explicitly `paper_zero_imputed_lt5_good`. A documented zero-rainfall row with adequate QA remains `observed_sum`; no-source-row bins remain missing rather than being invented as zero.

**coverage_aware_sensitivity** preserves bins with insufficient coverage as missing. The required good-row count is `ceil(coverage_aware_minimum_good_fraction × expected source rows in the selected 10-minute bin)`, with a minimum of one. This track distinguishes the 1-minute secondary source (up to ten records/bin) from the 5-minute primary source (up to two records/bin).

### Important unresolved reproduction ambiguity

The 2026 paper states that precipitation is aggregated to 10-minute intervals and assigns zero when original 1-minute or 5-minute data contain fewer than five good measurements. A 5-minute weighing gauge can provide only two observations in a 10-minute bin. The literal five-good rule therefore zero-imputes every selected primary 5-minute bin, even if both records pass QA. v0.1 preserves the paper's literal rule and reports the consequence; it does not silently reinterpret the threshold. Resolve the intended primary-gauge rule against the authors' preprocessing code or supplement before claiming bitwise reproduction. The article's Supporting Information file was not accessible in this pass.

## Provenance and limits

The report contains SHA-256 digests of the exact config and source CSV bytes, release/product/gauge metadata, source evidence references, input counts, QA pass/reject counts, ignored nonmatching gauge rows, out-of-window rows, per-bin source selection and both normalized tracks.

This tool executes a declared mapping/filter rule. It does not verify whether the selected NEON QA flags are scientifically sufficient, correct gauge calibration, or exact author-code parity. Preserve raw source files and the NEON variables table/ATBD alongside the report. The output is rainfall input, not a preferential-flow label.