# GCPN integrated end-to-end fault matrix v0.1

This crate composes the two independently versioned synthetic adapters behind one deterministic qualification report:

- `gcpn-compositional-assurance-adapter`: 12 fault families that the six-stage engine directly represents.
- `gcpn-civil-evidence-adapter`: five original missing civil-evidence dimensions plus five additional linkage faults.

## Coverage contract

The 17 original GCPN fault families are assigned exactly one adapter owner:

| Dimension | Owner | Original families |
|---|---|---:|
| Forecast, observation, CAP lifecycle, population denominator, decision timing, AER/outcome and post-processing | Six-stage compositional assurance | 12 |
| Authority, device-delivery receipt, accessibility, action execution, case-frame identity | Civil evidence contracts | 5 |

Single-fault qualification expects all 17 original family mutations to be detected with the declared typed disposition or fail-closed behavior.

The pairwise matrix records every run:
- 12 supported six-stage faults produce 66 pairwise combinations.
- 10 civil-contract faults produce 45 pairwise combinations.
- Pairwise CSV rows contain the exact fault pair, adapter owner, detection status, no-false-support status and whether the interaction failed closed.
- Duplicate case-key interactions are expected to fail closed before a report can support an outcome claim.

The integrated matrix does not collapse claim families into one composite "warning success" number. It counts and records the separate assurance outcomes.

## Run

    cargo fmt --all -- --check
    cargo test --locked --offline
    cargo clippy --locked --offline --all-targets -- -D warnings
    cargo run --locked --offline

Rust 1.98.1 is pinned. All dependencies are local Rust crates; there are no network or data downloads in the test.

## Research/assurance boundary

The matrix exercises synthetic mutations against the actual Rust audit implementations. It is not a live MHEWS, CAP sender, telecom integration, legal-authority determination, recipient tracker, operational-impact evaluation or validation of national emergency procedures.

The original 17 families are covered across two adapters rather than one monolithic engine. The exact dependency SHAs and both component-level qualification records must be retained when producing any integrated release or external assurance statement. The pairwise matrix covers two-way fault interactions, not all higher-order combinations or every possible external-system failure mode.
