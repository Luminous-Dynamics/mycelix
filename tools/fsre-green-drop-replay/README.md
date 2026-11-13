# FSRE Green Drop public-summary replay

A dependency-free Rust parser and arithmetic replay for the published South African Green Drop national-summary comparison. It is a bounded source-backed replay, not a system-level Green Drop dataset.

## Source and claim boundary

The Department of Water and Sanitation release published on 1 April 2026 reports that the 2025 Green Drop report covered 848 municipal wastewater systems for the 2023/24 municipal financial year. It compares the 2025 assessment with the 2022 report. SAnews' same-day account states that the 2022 report covered 850 systems for the 2020/21 municipal financial year, with the difference attributable to some systems being decommissioned or merged.

- Critical systems: 334/850 (39%) in 2022; 396/848 (47%) in 2025.
- Good/excellent systems: 118/850 (14%) in 2022; 66/848 (8%) in 2025.
- Green Drop certifications: 22 in 2022; 14 in 2025.

Sources:
- DWS / South African Government: https://www.gov.za/news/media-statements/water-and-sanitation-crowns-steve-tshwete-local-municipality-2025-green-drop
- SAnews / Government Communication and Information System: https://www.sanews.gov.za/south-africa/majodina-calls-accountability-and-collective-action-protect-water-resources

The replay checks both vintages' published rounded rates against their separately sourced denominators, using integer half-up rounding. It reports the change in source-reported aggregate rates as +8 percentage points for critical systems and -6 points for good/excellent systems. It retains the report-boundary change, and does not treat the two report vintages as a fixed longitudinal cohort. No system identity linkage or definition harmonisation has been performed.

The full report is referenced through the DWS IRIS portal. A WRO catalog collection records the report as a PDF and describes the collection as published PDF reports with no system-level tables extracted. Its dataset landing page displays a CC BY 4.0 licence while the individual 2025 PDF resource record says "No License Provided". This licence discrepancy is unresolved; the tool uses the government summary facts and does not redistribute or parse that PDF.

## What it validates

- Exact PSV header and strict nine-field fact rows.
- Required primary and secondary source identities, HTTPS URLs, publication dates, observation period, geography and source-claim boundary.
- Unique fact IDs and unique metric-period pairs.
- Non-negative integer values, supported metric kinds and explicit units.
- Numerator/denominator arithmetic for all four published rounded rates: 334/850, 118/850, 396/848 and 66/848.
- Deterministic report output with the two source references and visible claim ceiling.
- A missing denominator, zero denominator, missing ratio component, duplicate fact or inconsistent rounded rate is rejected.

## What it does not validate

It does not authenticate either publisher, verify the underlying Green Drop report file, extract system-level rows, reconcile system IDs between report vintages, establish that definitions remained identical, infer causes, estimate intervention effects, or qualify any treatment works. It does not turn national aggregate percentages into household service outcomes.

## Run

```sh
cargo test --locked --offline --manifest-path tools/fsre-green-drop-replay/Cargo.toml
cargo run --locked --offline --manifest-path tools/fsre-green-drop-replay/Cargo.toml -- \
  tools/fsre-green-drop-replay/fixtures/ZA_GREEN_DROP_PUBLIC_SUMMARY_2025.psv
```

The reference currently has 11 unit tests covering both report-vintage denominators, all four rounded rates, missing/zero denominator controls, arithmetic mismatch and prohibited field/causal claims. The path-specific workflow checks formatting, tests, the byte-for-byte golden report and Clippy. A skipped workflow is not a pass.
