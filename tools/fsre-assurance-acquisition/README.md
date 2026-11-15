# FSRE assurance acquisition E0

A dependency-free Rust exact set-cover planner for candidate evidence acquisitions.

## Purpose

The tool plans which admissible inspections could supply evidence for the currently unresolved proof obligations in a decision episode. It complements experiment potency: potency measures how observations partition decision-relevant worlds; this crate covers assurance obligations. It does not estimate EVSI or guarantee a successful or qualifying measurement.

## Exact objective

For unresolved obligations U and acquisition coverage sets C_j, find subsets S with union C_j equal to U, minimizing cost and, separately, acquisition count. Search respects a hard budget, prerequisite closure, resource-slot exclusivity, scope-digest equality and six non-compensatory gates: authority, rights, safety, physical realizability, schedule and data access.

All minimum-cost and minimum-count ties are retained. If no complete plan exists, the report describes the best partial coverage and uncovered obligations. Search limits yield no optimality claim.

## Semantic boundary

A complete set cover means only that the declared tasks could produce relevant evidence. It does not mean that the task succeeds, the evidence qualifies, the source is authentic, the authority accepts it, or an action is safe or authorized. The planner does not execute tasks, reserve resources or authorize field work.

## Run

    cargo fmt --manifest-path tools/fsre-assurance-acquisition/Cargo.toml -- --check
    cargo test --locked --offline --manifest-path tools/fsre-assurance-acquisition/Cargo.toml
    cargo clippy --locked --offline --manifest-path tools/fsre-assurance-acquisition/Cargo.toml --all-targets -- -D warnings
    cargo run --quiet --locked --offline --manifest-path tools/fsre-assurance-acquisition/Cargo.toml -- tools/fsre-assurance-acquisition/fixtures/water-obligations.psv

## Research roadmap

E0 provides exact finite set-cover planning. E1 must add calibrated outcome models and decision-conditioned EVSI/ENBS, distribution-shift stress, robust regret and separate model-ignorance penalties. Do not report probabilistic value when the outcome model is merely a caller assertion.
