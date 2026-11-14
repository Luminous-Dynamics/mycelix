# FSRE independent as-of evidence oracle v0.1

**Status:** Separately authored Rust parser and evaluator for differential qualification of historical evidence admissibility.

The oracle does not import the production as-of evidence crate, its parser, its types or its evaluation functions. It consumes the same documented 16-column input contract and frozen synthetic fixture. This protects against implementation disagreement; it does not remove shared-specification errors, source-authenticity gaps or the need for an independently authored sealed challenge set.

## What is independently reproduced

- claim requirements and per-claim freshness overrides;
- observation time versus decision-process availability;
- half-open valid-time boundaries;
- future observations and delayed availability;
- stale evidence;
- revocation effective time versus revocation knowledge time;
- minimum evidence counts, unknown evidence and source-reference deduplication;
- deterministic, row-order-invariant reports.

## Run

`cargo test --locked --offline --manifest-path tools/fsre-asof-evidence-oracle/Cargo.toml`

`cargo run --quiet --locked --offline --manifest-path tools/fsre-asof-evidence-oracle/Cargo.toml -- tools/fsre-asof-evidence/fixtures/synthetic-asof.psv`

This reference assesses admissibility as known at a frozen decision cutoff. It does not authenticate evidence, prove that a source statement is physically correct, or create operational authority.
