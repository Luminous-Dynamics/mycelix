# Maritime Commons Cross-Stack Golden-Trace Oracle

This isolated Rust workspace implements the independent reference model for Mycelix #5974 and consumes the frozen 35-case corpus.

## What it does

- Reads the declarative cross-stack golden-trace corpus and a companion typed synthetic owner-result fixture for every case.
- Checks exact case-set equality and that synthetic owner-result fixtures agree with the corresponding frozen corpus expectations.
- Applies a small independently authored cross-owner disposition rule table.
- Checks the handoff-template verifier report from Mycelix #5972 is PASS and is bound to the exact template bytes supplied to the oracle.
- Emits SHA-256 digests of all input artifacts and one result per case in deterministic order.

## What it does not do

The 35 owner-result combinations are synthetic fixtures, not outputs from Sol-Atlas, Symtropy, Symthaea or Mycelix production implementations. This oracle tests the composition rules only. It does not reimplement spatial projection, physics, cognition, provider allocation, identity, dependency analysis or booking reducers. It has no network or external-effect capability.

A PASS means the independent rule table agrees with the expected disposition for all exact frozen synthetic cases. It does not establish that a domain owner actually produced the declared result or that any underlying maritime claim is true.

## Run

The verifier workflow first runs Mycelix #5972's handoff verifier and writes its report. Then:

```sh
cargo test --locked --all-targets
cargo fmt -- --check
cargo clippy --locked --all-targets -- -D warnings
cargo run --locked -- ../../docs/maritime-stay/integration/cross-stack-golden-traces-v0.1.json ../../docs/maritime-stay/integration/cross-stack-oracle-inputs-v0.1.json ../maritime-handoff-verifier/fixtures/contract-template-v0.1.json /path/to/handoff-verifier-report.json
```
