# FSRE world-set stability E0

This crate computes the exact finite-world expansion needed to invalidate a proposed service-safety property.

## Objective

Given a base world set W0, a safety value s(w), service floor tau, and a finite omitted-world candidate pool C, find the minimum candidate expansion that makes the minimum safety value below tau.

Three separate stability radii are reported:
- minimum number of added worlds;
- minimum declared expansion cost;
- minimum declared population mass in milli-units.

These are distinct metrics. All tied minimum expansions are retained.

## Evidence status

Only candidates labelled SUPPORTED and ADMISSIBLE enter the evidence-supported search. HYPOTHETICAL candidates enter a separate stress-only search. UNKNOWN candidates are counted as unresolved. BLOCKED candidates are excluded.

A found supported counterexample is a bounded witness within the declared pool. A negative result is scoped to the searched pool and cannot establish global safety. Caller-declared completeness is never interpreted as empirical coverage.

## Run

    cargo fmt --manifest-path tools/fsre-worldset-stability/Cargo.toml -- --check
    cargo test --locked --offline --manifest-path tools/fsre-worldset-stability/Cargo.toml
    cargo clippy --locked --offline --manifest-path tools/fsre-worldset-stability/Cargo.toml --all-targets -- -D warnings
    cargo run --quiet --locked --offline --manifest-path tools/fsre-worldset-stability/Cargo.toml -- tools/fsre-worldset-stability/fixtures/omitted-worlds.psv

## Claim ceiling

This E0 implementation searches only a finite declared pool. It does not establish physical plausibility, source authenticity, institutional authority, population completeness, causal truth or global safety. Evidence-state labels are supplied inputs and are not authenticated here.
