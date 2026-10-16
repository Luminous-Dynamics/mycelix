# CIV-IMPACT causal investigation arena v0.1

A standalone Rust prototype for a deterministic, known-truth synthetic causal investigation benchmark.

## Run

    cargo test --manifest-path tools/civ-impact-causal-arena/Cargo.toml
    cargo run --release --manifest-path tools/civ-impact-causal-arena/Cargo.toml -- run --replications 128 --units 256 --seed 20261016 > arena-report.json

Default: 64 replications, 256 units per replication, seed 20261016. The report is emitted to stdout as JSON and includes every regime/strategy/replication row. It is validated against docs/research/schema/CIV_IMPACT_CAUSAL_ARENA_REPORT_V0_1.schema.json before emission.

## Contract

Five causal regimes × six strategies are compared on paired seeds. The report includes exact source SHA-256, treatment/control counts, cohort-average truth, estimate and standard error, nominal 95% interval coverage, directional claims, causal-identification status, cost units, and a full scheduled/completed/missing cell census. Aggregate effect-error metrics include Monte Carlo standard errors; repeated-run sign recovery, interval coverage, and null-world directional-claim rates include Wilson 95% intervals.

The strategy called cue_guided sees only a noisy risk cue; it does not receive the hidden world parameters. The evaluator scores against the known structural causal model. always_abstain is a control, not a high-performing strategy: estimability and abstention rates remain visible.

## Boundaries

All scenarios and outcomes are synthetic. Source is public so point estimates are diagnostic, not confirmatory. Simulator randomization uses a deterministic replayable PRNG, not a security-grade or authenticated allocation process. Wald intervals are simple diagnostics and may be inadequate for some designs or sample sizes. No treatment effect is established in a real population. A future hidden evaluation requires evaluator-held seeds, a predeclared challenge rotation policy, and externally anchored run receipts.
