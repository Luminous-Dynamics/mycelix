# Mycelix Global Profile Resolver (development draft)

Standalone Rust resolver for the catalogue and exact qualification-intersection contract in Mycelix [#5811](https://github.com/Luminous-Dynamics/mycelix/issues/5811).

## Scope

This crate selects an explicitly recorded profile intersection for a declared jurisdiction/entity/period/use. It does not execute tax or accounting rules, convert currencies, validate invoices, map XBRL facts, authenticate sources, create journals or issue a benchmark run receipt.

An individual profile's existence or status never implies that a combination is qualified. Qualification requires an exact intersection record matching the following fields, with every selected profile bound to its exact version and SHA-256 of canonical profile bytes:
- jurisdiction-scope set;
- entity scope;
- selected profile-ID set;
- requested intended use;
- reporting-period validity;
- knowledge frontier and approval time.

Synthetic-only intersections require an explicit `allow_synthetic=true` request and retain the `synthetic_fixture_only` claim ceiling. No real-world qualified intersection is present in v1.

## Run

From this directory, in a Rust-enabled environment:

```sh
cargo generate-lockfile
cargo test
cargo fmt -- --check
cargo clippy --all-targets -- -D warnings
cargo run -- --request fixtures/synthetic-qualified-request.json
```

The first lockfile must be generated and reviewed/committed before a later `--locked` qualification command is used. No network access is required at runtime; Cargo dependency acquisition happens before execution.

Use explicit paths to evaluate other requests:

```sh
cargo run -- \
  --registry ../profile_reference_v1.0.0.json \
  --registry-schema ../profile_reference_v1.0.0.schema.json \
  --request fixtures/reference-only-request.json \
  --request-schema resolve_request.schema.json \
  --report-schema report.schema.json
```

The program emits one deterministic JSON resolution report. A refusal/abstention is a successful evaluation result, not a process crash. Invalid schemas, malformed JSON or I/O failures exit non-zero.

## Current evidence status

- The Rust package, CLI, schemas and 30 unit tests are authored. Tests cover positive synthetic qualification, opt-in synthetic mode, exact profile/jurisdiction/entity intersection matching, unsupported references, missing dimensions, partial coverage, profile-set ordering, review/ambiguous intersections, time validity, historical knowledge frontiers, early application, profile content/version mutation, digest stability under JSON key reordering, and early-application evidence scope/uniqueness.
- The catalogue and request/report schemas are validated at runtime using the pinned JSON Schema Draft 2020-12 implementation.
- No Cargo.lock has been generated.
- No Rust compile, unit test, rustfmt, Clippy or CI result is claimed in this authoring environment.
- The public resolver fixtures are synthetic and do not constitute jurisdictional, legal, tax or accounting qualification.

## Architecture boundaries

Composes #5811/#5813. Does not duplicate #5814 currency/instrument identity, #5792 bitemporal policy evaluation, #5802 XBRL compilation, #5793 invoice validation, #5812 CAMT evidence or #5799 benchmark run manifests.
