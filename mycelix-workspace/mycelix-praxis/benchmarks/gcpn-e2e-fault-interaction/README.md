# GCPN end-to-end fault-interaction oracle v0.1

This crate starts a synthetic, end-to-end evidence-chain qualification programme. It addresses a gap left by independently passing package tests: component correctness does not, by itself, establish that combined evidence remains valid when faults cross stage boundaries.

## Scope

The reference catalogue models faults across forecast freshness, reference/observation lineage, authority assertion, CAP message identity, receiver population accounting, delivery, accessibility, action, outcome ascertainment, temporal cutoffs, hindsight leakage, and case identity.

It generates a deterministic catalogue containing:
- 1 clean baseline;
- 17 single-fault controls;
- 136 pairwise fault interactions;
- 6 preregistered three-way interactions.

Total: 160 synthetic scenarios.

The expected claim dispositions are Supports, Refutes, Indeterminate, OutOfScope, and Unavailable. Missing evidence is not negative evidence; a missing outcome is not zero impact; a control signal mixed with another fault is Indeterminate rather than a way to mask the additional fault.

## Run

    cargo fmt --all -- --check
    cargo test --locked --offline
    cargo clippy --locked --offline --all-targets -- -D warnings
    cargo run --locked --offline

The CLI prints an explicit synthetic-oracle disclaimer, a disposition summary, and a CSV scenario catalogue.

## Qualification boundary

**This is an oracle and scenario catalogue, not an integration adapter.** It does not call the forecast, observation, CAP, reach, decision, outcome-ascertainment, or compositional-assurance crates. The current executable therefore demonstrates deterministic scenario generation and reference semantics only. It does not demonstrate that any GCPN component detects the injected faults.

The next adapter stage must:
1. convert each scenario into concrete, versioned package fixtures;
2. execute the actual package evaluators at the exact tested source revisions;
3. submit observed dispositions through compare_observed;
4. report overclaims, underclaims, wrong dispositions, unexecuted cases and missing outputs;
5. prove package-version and fixture digests in the run manifest.

No result may be presented as cross-track integration qualification until those adapters exist and exact-head tests pass.

## Design principles

- Fail closed on malformed scenario identifiers and duplicate fault atoms.
- Preserve OutOfScope and Unavailable as distinct dispositions.
- Permit Refutes only for a valid contradictory-evidence control in isolation.
- Permit Supports only for the clean baseline.
- Preserve common-source dependence by collapsing corroboration families when a shared lineage fault is injected.
- Keep every scenario and denominator visible; do not manufacture one composite score.
- Use synthetic identifiers and fixtures only. No live warning authority, recipient data or operational endpoint is involved.

## References

- WMO, Guidelines on Multi-hazard Impact-based Forecast and Warning Services: whole-system validation includes impact evidence, simulation exercises and stakeholder evaluation, rather than relying only on forecast scores.
- SADC, 2026 MHEWS capacity-strengthening scope: harmonized procedures, CAP interoperability, people-centred actionable alerts, and transparent human oversight.

## Status

Stage B oracle implementation authored. Package tests have not yet been executed in this environment. CI qualification, and especially actual cross-package adapter qualification, remain pending.
