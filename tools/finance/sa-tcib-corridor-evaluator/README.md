# South African / SADC TCIB Corridor Evaluator

Independent, effects-disabled Rust reference evaluator for the corridor qualification corpus.

## Run

```sh
cargo fmt --manifest-path tools/finance/sa-tcib-corridor-evaluator/Cargo.toml -- --check
cargo test --manifest-path tools/finance/sa-tcib-corridor-evaluator/Cargo.toml
cargo run --manifest-path tools/finance/sa-tcib-corridor-evaluator/Cargo.toml -- --self-test
cargo clippy --manifest-path tools/finance/sa-tcib-corridor-evaluator/Cargo.toml --all-targets -- -D warnings
```

The evaluator checks Draft 2020-12 schema conformance, source/profile references, future programme status, participant-pair admission, message compatibility, operating windows, currency/FX, funding/liquidity, settlement asset/PvP, bilateral-leg evidence, finality, jurisdictional compliance, fraud provenance, audience scope and recourse.

The only positive disposition is SYNTHETIC_TEST_ONLY. It never authorises a live payment or corridor.

## Claim ceiling

All 30 scenarios are synthetic. A PASS demonstrates only that the evaluator derives the expected block/qualification disposition from the frozen fixture inputs. It does not prove TCIB connectivity, participant admission, FX availability, liquidity, PvP, settlement finality, legal discharge, regulatory approval or customer recourse.
