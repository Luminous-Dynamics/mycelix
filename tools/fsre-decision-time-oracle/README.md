# FSRE independent decision-time replay oracle v0.1

**Status:** Separately authored Rust reference for differential testing. It does not import the production decision-time crate, its parser, evidence structures or helper functions.

## Purpose

Independently reconstruct the latest evidence revision observed and recorded by a frozen decision cut-off, then classify it against valid-time and revocation boundaries. The oracle consumes the production crate's synthetic fixture and is compared byte-for-byte with the production report.

The two implementations share the written v0.1 contract and fixture. Agreement does not establish specification independence or source authenticity. A disjoint sealed challenge set and external adjudication are still required.

## Run

`cargo test --locked --offline --manifest-path tools/fsre-decision-time-oracle/Cargo.toml`

`cargo run --quiet --locked --offline --manifest-path tools/fsre-decision-time-oracle/Cargo.toml -- tools/fsre-decision-time-replay/fixtures/synthetic-decision.psv`

Tests cover the baseline replay, late ingestion, expiry, revocation, future revocation, revision identity drift, invalid transaction ordering and row-order invariance.
