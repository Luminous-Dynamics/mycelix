# GCPN compositional-assurance adapter v0.1

This standalone Rust crate adds executable synthetic mutations against the actual gcpn-compositional-assurance library through a local path dependency.

## What it executes

The adapter constructs a six-stage clean scenario and verifies that all six claims support with zero violations. It then applies 12 concrete fault types and invokes Scenario::audit() for every single fault:

- forecast late and stale;
- observation missing and common-source lineage;
- CAP receipt identity mismatch;
- unknown receiver-population denominator;
- missing after-event review evidence;
- post-cutoff evidence;
- post-processing outcome leakage;
- valid contradictory evidence;
- OutOfScope and Unavailable controls.

It also executes all C(12, 2) = 66 pairwise combinations, checking that each affected claim is not incorrectly left in Supports.

## Explicit coverage gaps

Five fault dimensions are deliberately marked NotRepresented and cause the adapter to fail closed:

| Fault | Why not approximated |
|---|---|
| Authority assertion mismatch | The current six-stage API has no dedicated authority assertion contract. |
| Delivery receipt incompleteness | Population accounting totals are not receiver/channel delivery receipts. |
| Accessibility unverified | There is no receiver accessibility evidence contract. |
| Action receipt missing | A decision-stage record is not proof that an action was executed. |
| Duplicate case key | Unique evidence identifiers do not establish unique eligible case-frame keys. |

These gaps are actionable follow-on work. They must not be reported as adapter passes.

## Run

    cargo fmt --all -- --check
    cargo test --locked --offline
    cargo clippy --locked --offline --all-targets -- -D warnings
    cargo run --locked --offline

The dependency is local and there are no external crates. The adapter is based on the exact source branch of the compositional-assurance engine.

## Qualification boundary

This validates only synthetic mutations against the compositional-assurance library. It does not validate CAP transport, national authority, real recipient delivery, accessibility, executed action, or disaster impact. The 66 pairwise cases are test interactions within this library, not a complete SADC or national MHEWS validation.

A production adapter must preserve the exact commit and fixture digests of both crates. The compositional-assurance parent PR remains a draft; this adapter PR is intentionally stacked on that source branch.
