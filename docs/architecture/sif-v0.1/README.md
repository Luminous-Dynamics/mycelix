# SIF v0.1 implementation notes

Canonical architecture: `../SOVEREIGN_INTELLIGENCE_FABRIC_V0_1.md`.

Tracking issues:

- #16 — cross-stack subject lookup receipts
- #17 — delayed notification expiry
- #18 — query/privacy budgets and denied attempts
- #19 — subject rights: know/inspect/contest/prove
- #20 — independent witness + aggregate transparency
- #21 — retention semantics
- #22 — schema invariants
- #23 — test gate
- #24 — stable cross-stack commitments
- #26 — shared Rust protocol types

Draft implementation PR: #25.

The first code tranche is additive and lives in `mycelix-workspace/crates/mycelix-sif-protocol` so the base contract stays independent from Holochain, Xenia, and Symthaea internals. It provides versioned/domain-separated commitments plus the common capability, receipt, delayed-notification, provenance, proof, and witness types.

Integration policy remains strict: reuse existing identity, governance, consent, audit, and ZK primitives through adapters rather than creating a parallel trust system.
