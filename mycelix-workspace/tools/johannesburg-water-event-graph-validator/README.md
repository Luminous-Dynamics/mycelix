# Johannesburg Water Event Graph Validator

Standalone, read-only Rust validator for the metadata-only Johannesburg Water source register, event graph and negative-control corpus.

## Run

From the Mycelix repository root:

```sh
cargo test --manifest-path mycelix-workspace/tools/johannesburg-water-event-graph-validator/Cargo.toml
cargo run --manifest-path mycelix-workspace/tools/johannesburg-water-event-graph-validator/Cargo.toml -- \
  docs/research/nrvca/johannesburg-water-service-sources-v0.1.json \
  docs/research/nrvca/johannesburg-water-event-graph-v0.1.json \
  docs/research/nrvca/johannesburg-water-event-graph-negative-controls-v0.1.json \
  docs/research/nrvca/johannesburg-water-event-graph-v0.1.schema.json \
  artifacts/johannesburg-water-event-graph/receipt.json
```

The tool is an isolated Cargo workspace. It does not join the Mycelix root workspace or the Mycelix Commons workspace, and it adds no dependency to a water-quality, water-allocation or Holochain zome.

## What it checks

- Git blob SHA-1 of the supplied source-register bytes matches the exact source revision pinned by the event graph.
- Every assertion resolves to the registered source ID, publisher, media index, exact media URI, notice class and source digest.
- Assertion, event and relation IDs are unique; all memberships and relation endpoints resolve; an assertion belongs to exactly one event.
- The Lonehill update edge points from the later 20:50 notice to the earlier 12:20 notice while both remain one provisional reported incident.
- The High Level / Hospital Hill notice remains one recurring schedule, not generated actual outage occurrences.
- City capacity reports remain separate snapshots with metric-method and comparability gaps intact.
- Publisher-only events remain reported-only, event onset/restoration unknown, and unverified geography remains unlocated.
- The 13 negative-control cases preserve three source-register-backed fixtures and ten synthetic adversarial controls.

A deterministic receipt binds the source register, graph, schema, negative corpus and exact implementation revision.

## Qualification boundary

The implementation validates source binding and event-graph semantics only. It does not acquire image bytes, assess copyright or data rights, read telemetry, geocode streets, qualify hydraulic state, measure customer service or authorize utility actuation.

The receipt deliberately reports `qualification_status: NotRun`. A structurally matched metadata graph is not a complete incident register, service measurement, hydraulic boundary condition or operational digital twin.

## Exact-head workflow and receipt

The path-filtered workflow at `.github/workflows/johannesburg-water-event-graph.yml` runs formatting, Rust tests, strict Clippy and the CLI against the exact checked-out revision. Cargo generates the standalone workspace lockfile during the first build; the workflow uploads it alongside the four frozen inputs and the receipt. The receipt hashes the lockfile and package manifest as well as the source register, event graph, schema and negative-control corpus.

The current Rust validator checks the Draft 2020-12 schema declaration and required top-level contract fields, then performs its own semantic checks. It does **not** implement the complete JSON Schema 2020-12 specification. Full schema-engine validation remains a separate qualification step.
