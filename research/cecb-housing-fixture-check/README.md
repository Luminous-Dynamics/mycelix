# CECB Housing Fixture Checker

Standalone Rust-first validator for the synthetic known-truth fixture used by the Capability-Economy Comparative Benchmark (CECB).

## Status and claim ceiling

This tool checks the *fixture's internal consistency* and generates a candidate-visible observation view. It does not execute economic policies, rank institutional profiles, validate the real world, or prove causal impact.

Its evaluator contains ground-truth expectations. Candidate policy code must consume the policy-view export, never the original fixture or validation report.

## Run

From this directory in a Rust toolchain environment:

```sh
cargo test
cargo run
cargo run -- --policy-view-out /tmp/cecb-policy-view-v0-1.json
cargo run -- --fixture /path/to/CECB_HOUSING_KNOWN_TRUTH_V0_1.json
```

The default fixture is embedded at compile time using `include_str!`, so it is version-bound to the source tree. `--fixture` is intended for explicit mutation testing and alternate copies. A passing report is emitted as JSON to stdout. Exit codes: `0` fixture passes, `1` consistency validation fails, `2` input or invocation error.

## Checks

- Frozen schema version, exact required case-ID set, and unique IDs.
- Required case sections and non-empty invariant lists.
- Per-case arithmetic/physical constraints for the ten canonical fixtures.
- Grantor/recipient transfer closure and refinancing decomposition.
- Housing suitability and maintenance capacity changes.
- Candidate-observation separation from evaluator facts, including a recursive intersection check for future oracle field names.
- Mutation tests that deliberately break each case's known truth or inject oracle-only data.

## Known limitations

- The checker validates fixed v0.1 fixture expectations; it is not a general-purpose economic simulator or a generic JSON Schema engine.
- It has not been run against the Rust toolchain as part of authoring because runner/toolchain access was unavailable. A skipped GitHub Actions run must not be represented as a pass.
- The fixture is synthetic. No result from this checker may be cited as evidence that a particular financing, ownership, or allocation regime works better in a real community.
- The evaluator code is not a security boundary if candidate code can read the original fixture or share process memory. Keep evaluator and candidate execution separated in the future runner, and pass only the generated policy-view artifact across that boundary.

## Exact-run evidence receipt

The path-scoped GitHub Actions job creates a `cecb.ci_run_receipt.v0.1` JSON receipt and uploads it with the test logs, validation report, candidate policy view, and resolved `Cargo.lock` when available. It records the exact source commit, run ID/attempt, Rust/Cargo versions, SHA-256 hashes of the fixture and checker inputs, output hashes, and each qualification step's outcome. The artifact is designed to remain useful when a check fails; a missing artifact or a skipped job is not a passing result.

The receipt's evidence class is explicitly E0 synthetic-fixture contract validation. It must not be cited as a policy-performance result or as real-world impact.
