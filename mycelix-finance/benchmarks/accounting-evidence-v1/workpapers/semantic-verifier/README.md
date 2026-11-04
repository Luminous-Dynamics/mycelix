# FIN-ACC workpaper DAG semantic verifier (v0.1 draft)

This isolated Rust package semantically checks the accounting close-workpaper DAG contract introduced in the sibling `workpapers/` directory. It deliberately distinguishes three questions:

1. Is the input structurally valid under the pinned Draft 2020-12 schema?
2. Is the graph semantically coherent, replayable and authority-safe?
3. Is the close actually ready?

A structurally valid graph may be semantically valid but not ready. The checked-in smoke-suite example is expected to return `VALID_NOT_READY`: it contains only three in-progress nodes and its source-scope frontier is unknown.

## Run

From this directory:

```sh
cargo metadata --locked --format-version 1
cargo fmt --all -- --check
cargo test --locked --all-targets
cargo clippy --locked --all-targets -- -D warnings
cargo run --locked -- \
  --dag ../examples/acc-syn-0001-minimal-dag-v0.1.json \
  --schema ../WORKPAPER_DAG_V0.1.schema.json \
  --out semantic-report.json
```

## Semantic invariants

- unique node IDs, dependency closure and no self/duplicate edges;
- cycle detection and deterministic canonical topological order;
- exact agreement between graph edges and `prior_workpaper` inputs;
- completed child inputs bind the exact parent workpaper commitment and version;
- non-terminal/stale parents cannot support completed descendants;
- source/policy/authority/scope frontier changes invalidate direct consumers and transitive descendants;
- each required close node kind is represented before readiness;
- state/terminal-disposition consistency;
- candidate and simulation nodes cannot escalate authority;
- ready dispositions require qualified source scope, completed required nodes and authorized human review;
- exact ratio numerator/denominator, zero-denominator policy, join cardinality, null semantics and unit-profile checks;
- exact rational/fixed-decimal numeric representation validation;
- payload and full-workpaper commitment recomputation for completed nodes.

The report differentiates `INVALID`, `VALID_NOT_READY` and `READY_ELIGIBLE`. A not-ready sample is not a failed graph: it is a valid graph with missing work and/or evidence gates outstanding.

## Limits

This verifier does not execute workpaper calculations, decide accounting policy, qualify evidence authenticity, prove source-universe completeness, authorize journal posting or give an audit opinion. It checks declared graph and calculation contracts only. It composes the existing Mycelix Business Fabric and Finance boundaries; it is not a workflow engine or ledger.
