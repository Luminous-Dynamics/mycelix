# FSRE independent as-of portfolio oracle v0.1

**Status:** Separately authored composition reference. It does not import the production composer or its parsing/linking implementation.

## Independence boundary

The oracle uses the independent as-of evidence evaluator and independent portfolio evaluator as component references, then independently reconstructs:
- scenario, evidence-snapshot, decision-cutoff and clock-basis equality;
- as-of claim disposition plus per-record admissibility;
- owner/source-reference uniqueness and portfolio binding;
- matched, unmatched and nonvalid portfolio records;
- final assurance disposition and claim ceiling.

The production composer and this oracle share the written contract and frozen fixtures, so the oracle tests implementation agreement, not specification truth. A blind, externally adjudicated challenge set remains a separate requirement.

## Run

`cargo test --locked --offline --manifest-path tools/fsre-asof-portfolio-oracle/Cargo.toml`

`cargo run --quiet --locked --offline --manifest-path tools/fsre-asof-portfolio-oracle/Cargo.toml -- tools/fsre-asof-portfolio-composer/fixtures/synthetic-asof.psv tools/fsre-asof-portfolio-composer/fixtures/synthetic-portfolio.psv`

All inputs are synthetic. The oracle does not authenticate sources, grant authority, dispatch or validate physical outcomes.
