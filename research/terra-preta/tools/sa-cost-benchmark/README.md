# South African soil decision-cost benchmark

Synthetic Rust ledger auditor plus a published-price reference catalogue for South African soil measurement workflows.

## Run

```sh
cargo fmt --manifest-path research/terra-preta/tools/sa-cost-benchmark/Cargo.toml -- --check
cargo test --manifest-path research/terra-preta/tools/sa-cost-benchmark/Cargo.toml
cargo clippy --manifest-path research/terra-preta/tools/sa-cost-benchmark/Cargo.toml --all-targets -- -D warnings
cargo run --manifest-path research/terra-preta/tools/sa-cost-benchmark/Cargo.toml -- \
  research/terra-preta/tools/sa-cost-benchmark/fixtures/synthetic-cost-comparison.json \
  /tmp/sa-cost-report.json
```

## What the input requires

Every strategy must account for ten cost categories. Each category is either priced with quantity, unit, positive unit cost and method scope, or explicitly marked not applicable with a reason. An unknown/unpriced category is rejected, not silently assigned zero.

Price provenance is typed as published tariff, written quote, measured actual cost or synthetic assumption. Published tariffs and quotes require source name/URL and validity dates. Measured actual costs require a cost-record reference and measurement period. The output preserves the scenario SHA-256.

Decision counts reconcile to a common reference cohort: true/false positives/negatives plus unresolved positives/negatives. Outputs include cost/ha, cost/sample, cost/correctly resolved decision, coverage, accuracy among resolved cases, false-negative rate among resolved positives and unresolved-positive fraction.

No aggregate strategy score is emitted.

## Tariff reference catalogue

`research/terra-preta/examples/sa-soil-tariff-references.json` records selected Western Cape, KZN and SHSC 2026/27 published prices as source references. These are not the synthetic scenario's actual paid costs. They have different methods and scopes:
- Western Cape carbon is Walkley–Black; KZN total carbon is listed as Dumas. Do not treat the numbers as method-equivalent.
- Western Cape P/K/Ca/Mg/Na is listed as a citric-acid combined panel; it is not automatically Bray-1.
- SHSC VAS / microbial respiration are physical/biological tests, not conventional fertility chemistry.
- Per-sample tariffs are not directly comparable to per-hectare proximal service pricing.

The tariff catalogue excludes unverified sampling/travel/courier/interpretation/turnaround items; obtain current written quotes and measure actual workflow costs before procurement or savings claims.

## Interpretation boundary

All strategy costs and decision outcomes in the fixture are synthetic. The tariff catalogue documents selected published reference price items only. This does not establish current commercial prices, supplier service equivalence, actual farm savings, NIR suitability or an approved agronomic decision threshold.
