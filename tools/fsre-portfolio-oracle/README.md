# FSRE independent portfolio reference v0.1

**Status:** Separately authored Rust reference evaluator for differential testing. It does not import the production portfolio crate, parser, records, helpers, or report implementation.

## Scope

The oracle independently parses the frozen 13-column portfolio receipt contract and computes option qualification, capacity constraints, per-cohort service floors, shared-resource demand, common failure-domain effects, unknown evidence, and the resulting claim ceiling.

It produces a canonical semantic report that the dedicated workflow compares byte-for-byte with the corresponding fields from the production evaluator. Its parser and evaluation implementation are separate, but both implementations intentionally consume the same written v0.1 contract and frozen input fixture. This does not eliminate common-specification errors; a disjoint externally authored challenge set remains required.

## Run

`cargo test --locked --offline --manifest-path tools/fsre-portfolio-oracle/Cargo.toml`

`cargo run --locked --offline --manifest-path tools/fsre-portfolio-oracle/Cargo.toml -- tools/fsre-portfolio-reference/fixtures/synthetic-feasible.psv`

The reference tests include common-cause failure, resource overcommit, missing capacity, non-compensable cohort floors, unknown numeric-bound rejection and row-order invariance. The oracle is a deterministic read-only reference, not an optimiser, dispatcher, source authenticator, or operational authority.
