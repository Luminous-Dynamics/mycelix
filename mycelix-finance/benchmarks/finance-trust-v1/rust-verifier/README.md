# FIN-TRUST S0 Rust verifier

Offline reference verifier for the public synthetic FIN-TRUST-BENCH v1.0.0 corpus.

## Run

From this directory:

```sh
cargo fmt --all -- --check
cargo test --locked
cargo clippy --locked --all-targets -- -D warnings
cargo run --locked -- --out /tmp/fin-trust-s0-report.json
```

The CLI accepts `--tasks`, `--task-schema`, `--oracle`, `--oracle-schema`, `--responses`, `--response-schema`, and `--out`.

## Verified invariants

- Draft 2020-12 validation of task, response, oracle and report contracts.
- Exact task/oracle/response identity and adjudicated terminal disposition.
- Minimum question-set IDs; forbidden and redundant clarification IDs are rejected.
- Question targets must reference declared unknowns.
- SHA-256 of canonical compact source JSON, source revision and exact document locator.
- A source citation is separate from a qualified evidence claim.
- Evidence cannot be qualified across entity, currency, valid-time or recorded/knowledge-time boundaries.
- Journal entry count, entity, period, currency, policy, source lineage and exact integer debit/credit totals.
- Independent statement reconstruction line parity.
- Explicit rejection of complete-books claims from this narrow task corpus.

Canonical source digest profile is compact UTF-8 JSON with recursively sorted object keys and preserved array order. The current fixtures are ASCII-only; implementations must preserve UTF-8 byte semantics for future non-ASCII source payloads.

## Public vs blind

The S0 oracle and reference responses are public. The S0 suite proves deterministic conformance only. It is not a model benchmark or blind holdout. Future private candidate/evaluator bundles, access controls, run manifests and repeated-run scoring remain with #5799; do not mount the oracle in a candidate environment.
