# South African Payment Human Decision Evaluator

Static, effects-disabled qualification of the synthetic human decision study instrument.

## Run

```sh
cargo fmt --manifest-path tools/finance/sa-payment-human-decision-evaluator/Cargo.toml -- --check
cargo test --manifest-path tools/finance/sa-payment-human-decision-evaluator/Cargo.toml
cargo run --manifest-path tools/finance/sa-payment-human-decision-evaluator/Cargo.toml -- --self-test
cargo clippy --manifest-path tools/finance/sa-payment-human-decision-evaluator/Cargo.toml --all-targets -- -D warnings
```

The evaluator checks Draft 2020-12 schemas, balanced development task classes, unique synthetic request identities, exact expected-action mapping, material-field completeness in the treatment presentation, matched presentation fields, and no-real-money/no-participant-data boundaries.

It does not run a human experiment, estimate a fraud-detection rate, establish comprehension, or claim a causal treatment effect. The checked-in 24 tasks are public development fixtures, not a blinded holdout set.

A real experiment requires an appropriate ethics review/approval determination, cognitive pilot, prospective power analysis, preregistration, independently adjudicated tasks and a separately generated locked holdout set. No real payment or financial account data is used.
