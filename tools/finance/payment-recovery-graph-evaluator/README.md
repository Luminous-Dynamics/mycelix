# Payment Recovery Graph Evaluator

Effects-disabled Rust reference evaluator for multi-hop payment recovery graph accounting.

## Run

```sh
cargo fmt --manifest-path tools/finance/payment-recovery-graph-evaluator/Cargo.toml -- --check
cargo test --manifest-path tools/finance/payment-recovery-graph-evaluator/Cargo.toml
cargo run --manifest-path tools/finance/payment-recovery-graph-evaluator/Cargo.toml -- --self-test
```

The evaluator validates the Draft 2020-12 schema, source/profile references, transaction graph reachability, parent/account continuity, timestamp ordering, graph cycles, complete account balance equations, account-scoped authority-backed holds, unique return identity and per-root net-return/residual-loss arithmetic.

It reports graph counts and reconciled return totals only. It does not sum downstream transaction values into an attribution or recovery amount. A candidate path does not prove that the same fungible funds moved along the path.

The checked-in corpus is synthetic and uses Brazil Pix MED v4.2 only as an industry comparator. It does not implement DICT, contact PSPs, open notifications, place holds, trace real accounts or move funds.


## Shared-account hold conservation

The evaluator reconciles simultaneous active holds across all accepted notifications targeting the same account/currency. Individually valid requests cannot reserve more than the one shared available balance. Only `applied` and `partial` holds with valid authority count as active; `released` and `contested` records remain history, not current reservations. The check uses checked integer arithmetic and fails closed on inconsistent snapshots or aggregate overflow.
