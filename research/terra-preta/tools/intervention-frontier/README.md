# Intervention-agnostic regenerative soil candidate frontier

Standalone Rust evaluator for synthetic candidate options. It applies hard evidence/guardrail gates before a conservative interval-aware Pareto comparison.

## Run

```sh
cargo fmt --manifest-path research/terra-preta/tools/intervention-frontier/Cargo.toml -- --check
cargo test --manifest-path research/terra-preta/tools/intervention-frontier/Cargo.toml
cargo clippy --manifest-path research/terra-preta/tools/intervention-frontier/Cargo.toml --all-targets -- -D warnings
cargo run --manifest-path research/terra-preta/tools/intervention-frontier/Cargo.toml -- \
  research/terra-preta/tools/intervention-frontier/fixtures/synthetic-intervention-frontier.json \
  /tmp/intervention-frontier-report.json
```

## Eligibility before optimization

- out-of-domain / adverse / prohibited / failed quality are blocked;
- unknown or incomplete critical evidence remains unresolved;
- uncertainty intervals overlapping a hard bound remain unresolved;
- every outcome needs a complete lower/upper interval;
- candidate-specific quality and sourcing requirements depend on the declared candidate class.

## Pareto comparison

No weighted score exists. A robust dominant candidate must be at least as good under worst-case interval bounds on every objective and strictly better on at least one. Target-range outcomes use worst-case distance to the declared interval. Broad uncertainty tends to preserve alternatives rather than manufacture a winner.

## Boundary

All candidates, effects, units and thresholds are synthetic. The frontier is not a treatment recommendation, safe-use determination, rate calculator or field authorization. Real thresholds require local authorities; material quality, sourcing and field response require independent evidence.