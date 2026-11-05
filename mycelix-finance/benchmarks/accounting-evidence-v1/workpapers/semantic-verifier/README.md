# FIN-ACC-CLOSE-VERIFY-001 — workpaper DAG semantic verifier v0.1

Standalone Rust reference verifier for the existing Draft 2020-12 workpaper DAG contract from PR #5804 / issue #5803. This package does not define a second DAG schema or execute close workflows.

## Run

From this directory:

```sh
cargo fmt --all -- --check
cargo test --locked
cargo clippy --locked --all-targets -- -D warnings
cargo run --locked -- --input ../examples/acc-syn-0001-minimal-dag-v0.1.json --out /tmp/workpaper-dag-report.json
```

The CLI accepts `--input PATH`, `--schema PATH` and `--out PATH`. Its default input and schema are the v0.1 fixture and schema in the parent `workpapers/` directory. A semantically valid but not-ready DAG returns process success with `status=PASS` and `close_disposition=not_ready`. An invalid graph returns non-zero with a machine-readable report.

## Semantic checks

- Unique node identities; no dangling, self or duplicate dependency edges; deterministic Kahn topological order; concrete cycle path when cyclic.
- `depends_on` edges exactly match `prior_workpaper` input references. Parent version must match the DAG version. Completed parents require the exact workpaper commitment; completed children cannot consume non-terminal parents.
- Completed workpaper commitments are recomputed from domain-separated canonical JSON after excluding the commitment field itself. Object keys are recursively sorted; arrays retain authored order.
- Node-scoped frontier bindings are compared with the DAG's current frontier. Directly stale nodes and their transitive descendants are reported separately; in-progress/completed stale nodes must be marked `invalidated_stale`.
- Required review obligations on completed workpapers require a completed human-review node with an exact parent dependency and exact workpaper commitment binding.
- Candidate-analysis and simulation-only nodes cannot claim journal/statement/verification/review authority. Symthaea and Symtropy cannot claim Mycelix deterministic projection or verification authority.
- Recipe checks cover named ratio numerator/denominator and zero-denominator disposition, join keys/cardinality and explicit aggregation, period/cutoff semantics, null policy, numeric canonicality, decimal scale, negative zero and unit-profile consistency.
- A ready close requires every declared required node kind, completed and verified required nodes, no stale required node, qualified source/evidence/policy/taxonomy/authority frontiers and qualified reporting framework/jurisdiction/currency profiles. Scope-limited readiness must declare its limitations.

## Interpretation and claim ceiling

The frozen three-node smoke DAG is intentionally incomplete and source/evidence/authority-unqualified. The correct result is a *valid graph that is not ready*, not a false close failure and certainly not a qualified close.

The verifier does not authenticate source bytes, reviewer identities, signatures or authority; does not recompute external payload digests without payload bytes; does not decide accounting/tax treatments; and does not authorize journals, filing, payments or period-close release. A passing result proves only the semantic consistency of the declared graph and supplied bindings under this exact schema/verifier version. It does not establish source-population completeness, audit sufficiency, IFRS/tax compliance or production readiness.
