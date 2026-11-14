# FSRE independent resource stock-flow oracle v0.1

**Status:** Separately authored deterministic Rust reference. It does not import the production stock-flow crate, parser, types or helper functions.

## Purpose

Independently reproduce stock-flow balances across a frozen multi-interval resource series. The oracle uses its own parser and arithmetic path. It consumes the production crate's synthetic fixture, but the output is also checked against a frozen expected report.

The oracle and production evaluator share the written v0.1 contract and fixture. Agreement does not establish specification independence or source authenticity. A disjoint externally authored challenge set remains required.

## Run

`cargo test --locked --offline --manifest-path tools/fsre-resource-stock-flow-oracle/Cargo.toml`

`cargo run --quiet --locked --offline --manifest-path tools/fsre-resource-stock-flow-oracle/Cargo.toml -- tools/fsre-resource-stock-flow/fixtures/synthetic-balanced.psv`

The test set checks the normal multi-interval case, consumption overdraw, unknown stock propagation, invalid closing assertions and row-order invariance.
