# FSRE Green Drop public-summary replay

A dependency-free Rust parser and arithmetic replay for the published South African Green Drop national-summary comparison. It is a bounded source-backed replay, not a system-level Green Drop dataset.

## Source and claim boundary

The Department of Water and Sanitation release published on 1 April 2026 reports that the 2025 Green Drop report covered 848 municipal wastewater systems for the 2023/24 municipal financial year. SAnews' same-day account states that the 2022 report covered 850 municipal systems for 2020/21, with the difference attributable to decommissioning or merging. The WRO catalog separately describes the broader 2022 full audit as 995 works across municipal, Department of Public Works and private ownership. The 995 all-sector figure is retained as context, **not** used as the denominator for the municipal rate comparison.

- Critical systems: 334/850 (39%) in 2022; 396/848 (47%) in 2025.
- Good/excellent systems: 118/850 (14%) in 2022; 66/848 (8%) in 2025.
- Green Drop certifications: 22 in 2022; 14 in 2025.

Sources:
- DWS / South African Government: https://www.gov.za/news/media-statements/water-and-sanitation-crowns-steve-tshwete-local-municipality-2025-green-drop
- SAnews / Government Communication and Information System: https://www.sanews.gov.za/south-africa/majodina-calls-accountability-and-collective-action-protect-water-resources
- Water Research Observatory catalog: https://wrcwro01.arc.agric.za/en/dataset/green-drop-reports-2009-2025-south-african-municipal-wastewater-management-performance

The replay checks both vintages' published rounded rates against their separately sourced denominators, using integer half-up rounding. It reports the change in source-reported aggregate rates as +8 percentage points for critical systems and -6 points for good/excellent systems. It retains the report-boundary change, and does not treat the two report vintages as a fixed longitudinal cohort. No system identity linkage or definition harmonisation has been performed.

The WRO catalog records the full report collection as PDFs and states that system-level tables were not extracted into machine-readable form. As viewed on 13 November 2026, the dataset page's prominent licence field says "No License Provided", while its lower metadata fields say "Open (Creative Commons)" and provide a CC BY 4.0 URL. The individual 2025 PDF resource page says "No License Provided" (metadata updated 11 November 2026). This is an unresolved dataset/resource rights conflict; do not infer that parent metadata licenses the PDF. Do not download, extract or redistribute the PDF until the DWS custodian clarifies resource-level terms. This replay uses only DWS and SAnews summary facts plus public WRO catalog metadata.

## What it validates

- Exact PSV header and strict nine-field fact rows.
- Required primary, secondary and tertiary source identities, HTTPS URLs, publication dates, observation period, geography and source-claim boundary.
- Unique fact IDs and unique metric-period pairs.
- Non-negative integer values, supported metric kinds and explicit units.
- Numerator/denominator arithmetic for all four published rounded rates: 334/850, 118/850, 396/848 and 66/848.
- Separate municipal and all-sector totals for 2022 (850 municipal systems versus 995 all-sector works).
- Deterministic report output with three source references and visible claim ceiling.
- A missing denominator, zero denominator, missing ratio component, duplicate fact or inconsistent rounded rate is rejected.

## What it does not validate

It does not authenticate either publisher, verify the underlying Green Drop report file, extract system-level rows, reconcile system IDs between report vintages, establish that definitions remained identical, infer causes, estimate intervention effects, or qualify any treatment works. It does not turn national aggregate percentages into household service outcomes.

## Run

```sh
cargo test --locked --offline --manifest-path tools/fsre-green-drop-replay/Cargo.toml
cargo run --locked --offline --manifest-path tools/fsre-green-drop-replay/Cargo.toml -- \
  tools/fsre-green-drop-replay/fixtures/ZA_GREEN_DROP_PUBLIC_SUMMARY_2025.psv
```

The reference currently has 11 unit tests covering both municipal denominators, all four rounded rates, separate 2022 all-sector scope, missing/zero denominator controls, arithmetic mismatch and prohibited field/causal claims. The path-specific workflow checks formatting, tests, the byte-for-byte golden report and Clippy. A skipped workflow is not a pass.
