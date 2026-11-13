# FSRE Green Drop public-summary replay

A dependency-free Rust parser and arithmetic replay for a narrowly scoped, published South African government summary. It is the first source-backed public-data replay in the FSRE water programme; it is not a system-level Green Drop dataset.

## Source and claim boundary

The fixture transcribes the national comparisons stated in the Department of Water and Sanitation release published on 1 April 2026:

- 2025 Green Drop report vintage, covering the 2023/24 municipal financial year: 848 municipal wastewater systems audited.
- Critical systems: 396 (47%) in the 2025 report; the release compares this with 334 (39%) in the 2022 report.
- Good/excellent systems: 66 (8%) in 2025; compared with 118 (14%) in 2022.
- Systems certified: 14 in 2025, compared with 22 in 2022.

The published 2025 counts and percentages are arithmetically checked against 848. The source release does not provide a 2022 denominator in this summary. The fixture leaves it absent; the replay does not infer it. The output compares the source-reported rounded rates only, labels the comparison as not independently reassessed, and withholds an interpretation of the raw count differences.

Primary source: https://www.gov.za/news/media-statements/water-and-sanitation-crowns-steve-tshwete-local-municipality-2025-green-drop
Secondary source for the 2022 denominator and system-boundary change: https://www.sanews.gov.za/south-africa/majodina-calls-accountability-and-collective-action-protect-water-resources

The full report is referenced through the DWS IRIS portal. A WRO catalog collection records the report as a PDF and describes the collection as published PDF reports with no system-level tables extracted. Its dataset landing page displays a CC BY 4.0 licence while the individual 2025 PDF resource record says "No License Provided". This licence discrepancy is unresolved; this tool uses the government release facts and does not redistribute or parse that PDF.

## What it validates

- Exact PSV header and strict nine-field fact rows.
- Required source identity, source URL, publication date, period, geography and source-claim boundary.
- Unique fact IDs and unique metric-period pairs.
- Non-negative integer values, supported metric kinds and explicit units.
- Exact numerator/denominator arithmetic for the two 2025 percentages, using integer round-half-up.
- Explicit absence of the 2022 denominator; a denominator-free source percentage cannot smuggle in a reconstructed ratio.
- Deterministic report output with a visible claim ceiling.

## What it does not validate

It does not authenticate the government publisher, verify the underlying report file, extract system-level rows, reconcile report vintages, determine whether definitions remained identical between 2022 and 2025, infer causes, estimate intervention effects, or qualify any treatment works. It does not turn national aggregate percentages into household service outcomes.

## Run

```sh
cargo test --locked --offline --manifest-path tools/fsre-green-drop-replay/Cargo.toml
cargo run --locked --offline --manifest-path tools/fsre-green-drop-replay/Cargo.toml -- \
  tools/fsre-green-drop-replay/fixtures/ZA_GREEN_DROP_PUBLIC_SUMMARY_2025.psv
```

The path-specific workflow checks formatting, tests, the byte-for-byte golden report and Clippy. A skipped workflow is not a pass.
