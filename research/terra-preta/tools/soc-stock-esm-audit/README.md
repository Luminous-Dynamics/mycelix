# SOC stock ESM auditor

Standalone Rust auditor for repeated-profile SOC stock accounting on synthetic inputs.

## Run

```sh
cargo fmt --manifest-path research/terra-preta/tools/soc-stock-esm-audit/Cargo.toml -- --check
cargo test --manifest-path research/terra-preta/tools/soc-stock-esm-audit/Cargo.toml
cargo clippy --manifest-path research/terra-preta/tools/soc-stock-esm-audit/Cargo.toml --all-targets -- -D warnings
cargo run --manifest-path research/terra-preta/tools/soc-stock-esm-audit/Cargo.toml -- \
  research/terra-preta/tools/soc-stock-esm-audit/fixtures/synthetic-stock-esm.json \
  /tmp/soc-stock-report.json
```

## Output

- baseline/follow-up fixed-depth SOC stocks;
- baseline fine-earth reference soil mass;
- baseline/follow-up ESM stocks at that reference mass;
- fixed-depth and ESM stock changes separately;
- follow-up depth required to reach the baseline reference mass;
- applied amendment-carbon input as a separate ledger;
- source attribution and uncertainty limitations.

## Critical unit contract

Fine-earth bulk density is dry fine-earth mass per fine-earth volume. Coarse-fragment volume fraction of total core volume is applied once. SOC concentration is grams C per kilogram dry fine earth. The output is Mg C/ha.

## Boundaries

- If follow-up fine-earth mass is insufficient to reach baseline reference mass, ESM stock/change are null; there is no extrapolation.
- Amendment input is not assumed to be retained SOC.
- Bulk SOC change does not identify native vs amendment-derived carbon.
- Input standard errors are retained but not propagated in v0.1.
- No net CO2e, leakage, additionality, permanence, credit issuance or registry compliance is calculated.
- Fixtures are synthetic only.
