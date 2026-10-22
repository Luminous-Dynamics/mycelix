# Life disclosure correlation benchmark

This dependency-free Rust benchmark advances [LIFE-DISCLOSURE-COMPOSITION-001 (#5404)](https://github.com/Luminous-Dynamics/mycelix/issues/5404).

## Question

How much does a balanced/independent synthetic population overestimate the candidate set left by cumulative disclosure when claim values are correlated and their prevalence is skewed?

## Two frozen populations

Both populations contain 4,096 rows and use the same six task predicates and two disclosure profiles.

- `balanced_independent`: the complete (4^6) Cartesian product, each categorical tuple appears once.
- `correlated_skewed`: 16 latent cohorts with sizes `1, 2, 3, 5, 8, 13, 21, 34, 55, 89, 144, 233, 377, 610, 987, 1514`. The sizes sum to 4,096. Several attributes share a latent cohort factor, and each row in a cohort has the same observable attributes.

The skewed fixture is an adversarial stress distribution, **not a measured model of a real population**.

## Comparison

For each population, profile and disclosure depth, the report emits the minimum, p05, p50, p90, p95 and maximum candidate-set sizes, unique-subject rate, shares with at most 10 and 100 candidates, candidate retention relative to the previous disclosure depth, and task-utility accuracy.

The task predicate for each attribute is unchanged between:
- `exact_attribute`: reveal its full categorical value;
- `predicate_only`: reveal only the Boolean answer needed for that attribute's task.

Both profiles must achieve 100% task-answer accuracy. Candidate sets are computed from observed outputs only. Hidden cohort labels are used only by tests/oracle code and are never exported.

## Independent checks

- The balanced population has exact closed-form candidate sizes: (4096/4^d) for full values and (4096/2^d) for Boolean predicates at depth (d).
- The correlated model is checked against a separate cohort-enumeration oracle that sums the sizes of every cohort sharing the same visible prefix.
- Candidate sets must monotonically stay the same size or shrink at each disclosure.
- CI computes SHA-256 over the generated feature corpus and report. Printed FNV-1a fingerprints are non-cryptographic deterministic fingerprints only.

## Why this is different from the pairwise benchmark

- #5402 measures whether features let an observer correlate two presentations across recipients.
- #5405 measures cumulative inference under a balanced Cartesian population.
- #5412 measures residual presentation-structure linkability.
- This crate isolates population prevalence and correlations given that the observation sequence is already associated with a subject.

## Run

```sh
cargo +1.96.0 fmt --manifest-path research/life-disclosure-correlated/Cargo.toml --all -- --check
cargo +1.96.0 test --locked --manifest-path research/life-disclosure-correlated/Cargo.toml
cargo +1.96.0 clippy --locked --manifest-path research/life-disclosure-correlated/Cargo.toml --all-targets -- -D warnings
cargo +1.96.0 run --release --locked --manifest-path research/life-disclosure-correlated/Cargo.toml -- --write-corpus-dir target/life-disclosure-correlated
sha256sum target/life-disclosure-correlated/*
```

## Claim ceiling

This synthetic distribution sensitivity study is not an anonymity guarantee, differential-privacy mechanism, cryptographic unlinkability proof, representative population study, real-world efficacy claim, or SOTA superiority result. Passing establishes only the frozen deterministic generator, candidate-set evaluator and known-answer checks.
