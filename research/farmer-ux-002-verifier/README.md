# Farmer UX contract verifier

Standalone Rust CLI for validating the farmer UX research fixtures on disk. It is intentionally a **corpus verifier**, not a running UI test, Holochain integration test, agronomic model, route engine, or field-impact estimator.

## Run

From the repository root:

```sh
cargo run --manifest-path research/farmer-ux-001-verifier/Cargo.toml -- --root .
cargo test --manifest-path research/farmer-ux-001-verifier/Cargo.toml
```

Before merge, generate and review a `Cargo.lock`, commit it, and rerun both commands with `--locked` so transitive dependency resolution is reproducible.

Optional deterministic JSON report:

```sh
cargo run --manifest-path research/farmer-ux-001-verifier/Cargo.toml -- --root . --report-json /tmp/farmer-ux-report.json
```

The default repository root is derived from this crate's location. `--root` may point at a separate checkout for mutation testing. This tool reads only the three committed JSON artifacts and their paired schemas; it performs no network access, writes no repository files by default, and does not inspect live farmer data.

## Checks

1. Validate the three main instances against their declared JSON Schema Draft 2020-12 schemas using the Rust `jsonschema` crate, including schema documents against the crate's bundled meta-schema validator.
2. Verify the draft dialect identifier before validating.
3. Require the canonical v1 scenario IDs, role IDs, and state-copy keys so removing high-risk coverage fails closed.
4. Check shared invariants, uniqueness, contradictory `must_show`/`must_not_show` labels, and critical side-effect expectations.
5. Hash the exact source and schema bytes and emit a report with stable field ordering, no timestamps, and a deliberately narrow claim ceiling.

The verifier uses local schema documents and internal references only. External reference fetching is disabled in dependency features.

## Mutation tests

The frozen `mutations-v1.json` file defines 12 data-driven semantic mutations and the expected failure reason. Unit tests apply that corpus to fresh in-memory fixture copies, ensuring the verifier rejects duplicate IDs, missing high-risk scenarios, contradictory visible states, an expired-quote booking, digest syntax treated as source authentication, critical state drift, unauthorized relay effects, no-show denominator loss, copy overclaims, and stale provider capacity. Additional unit tests cover schema meta-validation and a standard SHA-256 test vector. These are verifier mutation checks; they are not end-to-end production state-machine tests.

## Interpretation

- `PASS` means the exact local artifacts passed the implemented schema and corpus checks for that invocation.
- It does not establish the authenticity of data sources, farmer comprehension, browser/mobile accessibility, UI integration, production synchronization correctness, safe agronomic advice, actual service fulfilment, causal improvements in income/yield, or readiness to deploy.
- The report embeds raw-byte digests for reproducibility. A digest proves byte equality relative to the captured bytes, not identity, authorship, truth, or external source authenticity.
- This contract is pinned to v1 fixture identities. Intentional corpus evolution requires a reviewed fixture/schema/code update together; silently deleting or renaming a required scenario is expected to fail.
