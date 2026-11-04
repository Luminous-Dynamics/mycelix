# South African Request-to-Pay Limit Evaluator

Effects-disabled Rust reference evaluator for the participant/direction/route/counterparty scoped limit corpus.

Run:
```sh
cargo fmt --manifest-path tools/finance/sa-payment-rtp-limits-evaluator/Cargo.toml -- --check
cargo test --manifest-path tools/finance/sa-payment-rtp-limits-evaluator/Cargo.toml
cargo run --manifest-path tools/finance/sa-payment-rtp-limits-evaluator/Cargo.toml -- --self-test
```

The evaluator checks profile/source resolution, selected rule identity, direction, route, counterparty count, rolling-window semantics, per-request and explicit aggregate amount limits, receiver-side lower limits, expiry and appeals. It has no provider connector or payment effect.
