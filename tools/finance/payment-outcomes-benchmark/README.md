# Payment Outcomes Benchmark

A Rust reference evaluator for two deliberately separate comparison classes:

1. **Public market context snapshots** — source-dated UPI, Pix and PayShap facts. The evaluator explains why non-matched national aggregates are context only and must not be ranked.
2. **Matched provider runs** — the same frozen tasks, corridor/jurisdiction, amount band, participant segment, rail profile, time window and task mix are compared across incumbents, conventional adapters and Mycelix candidates.

## Run

From the repository root:

```sh
cargo fmt --manifest-path tools/finance/payment-outcomes-benchmark/Cargo.toml -- --check
cargo test --manifest-path tools/finance/payment-outcomes-benchmark/Cargo.toml
cargo run --manifest-path tools/finance/payment-outcomes-benchmark/Cargo.toml -- --self-test
```

## Hard gates

A run is disqualified from performance comparison if any of these are non-zero:
- duplicate effects;
- false settlement-finality promotions;
- unexplained reconciliation deltas;
- unauthorised secondary use of payment data; or
- denial of essential payment service because a user refused unrelated data-sharing consent.

Hard gates are never averaged against latency, price or task completion.

## Multi-dimensional comparison

A matched candidate is evaluated on:
- recipient-available completion rate;
- all-in cost per completed delivery (exact integer numerator/denominator);
- p95 initiation-to-recipient-available time;
- operator minutes per 1,000 attempts;
- user task success; and
- active merchant coverage in the eligible study cohort.

The evaluator uses Pareto dominance rather than a weighted scalar score. If a candidate is cheaper but materially worse on task completion, latency, usability or operator effort, the result is trade-off/incomparable rather than a claimed win.

## Market context

The checked-in UPI (October 2026), Pix (September 2026) and PayShap (cumulative through 20 October 2026) values are public context snapshots only. Their time windows, population definitions, source methodology and payment scopes differ. No raw-volume leaderboard or direct superiority assertion is emitted.

## Claim ceiling

The matched-run fixture is synthetic and illustrative only. It is not a real Mycelix performance result. A future real study must use authorised providers, same frozen inputs and task mix, pre-registered outcome definitions, suitable denominators, confidence intervals, data minimisation, and a publicly reviewable source/evidence register.